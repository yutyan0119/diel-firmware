/* PREC wireless rev1/rev2 driver. TIMER2 / SAADC are exclusively owned here. */
#define DT_DRV_COMPAT prec_ec_kscan

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/retention/bootmode.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_saadc.h>
#include "ec_logic.h"
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
#include "ec_standby.h"
#endif

#if defined(CONFIG_SHELL)
#include <zephyr/bluetooth/conn.h>
#include <zephyr/shell/shell.h>
#endif

#define ROWS 4
#define COLS 12
#define KEYS (ROWS * COLS)
#define CALIBRATION_FRAMES 32
#define STARTUP_NOISE_LIMIT 120
#define NODE DT_DRV_INST(0)
#define DISCHARGE_ACTIVE_HIGH DT_PROP(NODE, discharge_active_high)
#define DISCHARGE_DRIVE (DISCHARGE_ACTIVE_HIGH ? NRF_GPIO_PIN_S0S1 : NRF_GPIO_PIN_S0D1)

BUILD_ASSERT(DT_PROP_LEN(NODE, row_pins) == ROWS);
BUILD_ASSERT(DT_PROP_LEN(NODE, column_channels) == COLS);
BUILD_ASSERT(DT_PROP_LEN(NODE, mux_select_pins) == 3);
BUILD_ASSERT(DT_PROP_LEN(NODE, mux_enable_pins) == 2);
BUILD_ASSERT(CONFIG_PREC_EC_PRESS_DELTA > CONFIG_PREC_EC_RELEASE_DELTA);
BUILD_ASSERT(!IS_ENABLED(CONFIG_ADC_NRFX_SAADC), "SAADC must be exclusive to PREC");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(timer2), okay), "TIMER2 must be exclusive to PREC");

static const uint8_t rows[] = DT_PROP(NODE, row_pins);
static const uint8_t selects[] = DT_PROP(NODE, mux_select_pins);
static const uint8_t enables[] = DT_PROP(NODE, mux_enable_pins);
static const uint8_t channels[] = DT_PROP(NODE, column_channels);
static const uint8_t discharge = DT_PROP(NODE, discharge_pin);
static const uint8_t amplifier = DT_PROP(NODE, amplifier_pin);
static const uint8_t battery_enable = DT_PROP(NODE, battery_enable_pin);
static const uint8_t wake = DT_PROP(NODE, wake_pin);
static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

static uint8_t row_tasks[ROWS], discharge_task;
static nrf_ppi_channel_t ppi_release, ppi_row, ppi_sample;
static int16_t adc_result __aligned(4);
static int16_t calibration_scratch[2] __aligned(4);
static const struct device *scan_device;
static kscan_callback_t key_callback;
static atomic_t callbacks_enabled;
static atomic_t scanner_started;
static K_SEM_DEFINE(scanner_gate, 0, 1);
static K_MUTEX_DEFINE(scan_lock);

static uint16_t raw[KEYS], baseline[KEYS], minimum[KEYS], maximum[KEYS];
static uint16_t cal_min[KEYS], cal_max[KEYS];
static uint32_t cal_sum[KEYS];
static struct ec_key_state keys[KEYS];
static bool valid_key[KEYS];
static bool calibrated, recalibrate = true;
static bool hardware_ready;
static bool report_keys = IS_ENABLED(CONFIG_PREC_EC_REPORT_KEYS);
static unsigned calibration_count;
static uint16_t press_delta = CONFIG_PREC_EC_PRESS_DELTA;
static uint16_t release_delta = CONFIG_PREC_EC_RELEASE_DELTA;
static uint16_t sample_delay = CONFIG_PREC_EC_SAMPLE_DELAY_US;
static uint16_t active_period = CONFIG_PREC_EC_ACTIVE_PERIOD_MS;
static uint16_t idle_period = CONFIG_PREC_EC_IDLE_PERIOD_MS;
static uint32_t frame_count, adc_errors, last_frame_us, max_frame_us;
static atomic_t overruns;
static uint16_t battery_mv;
static bool battery_valid;
static uint8_t low_voltage_count;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
#define STANDBY_INDEX (CONFIG_PREC_EC_STANDBY_ROW * COLS + CONFIG_PREC_EC_STANDBY_COL)
static struct ec_standby_state standby;
static bool standby_requested, standby_exit_requested, standby_reference_valid;
static uint16_t standby_raw, standby_baseline, standby_cal_min, standby_cal_max;
static uint32_t standby_cal_sum, standby_cal_samples;
static uint32_t standby_samples, standby_entries, standby_wakes;
static uint32_t standby_last_us, standby_max_us;
#endif
#if defined(CONFIG_SHELL)
/* Diagnostic comparison only; CPU edges may be delayed by interrupts. */
static bool software_edges;
static bool static_probe;
/* Observe debounced transitions even while HID output is paused. */
static struct ec_key_state observed_keys[KEYS];
static uint32_t observed_presses[KEYS], observed_releases[KEYS];
/* A candidate crossing that disappears before the second confirming frame. */
static uint32_t short_press_pulses[KEYS];
#endif

static bool wait_event(volatile uint32_t *event, uint32_t timeout_us) {
    uint32_t start = k_cycle_get_32();
    while (*event == 0) {
        if (k_cyc_to_us_floor32(k_cycle_get_32() - start) >= timeout_us) {
            return false;
        }
    }
    return true;
}

static void discharge_gpio(bool clamped) {
    nrf_gpio_pin_write(discharge, ec_discharge_level(DISCHARGE_ACTIVE_HIGH, clamped));
}

static void discharge_gpiote(bool clamped) {
    if (ec_discharge_level(DISCHARGE_ACTIVE_HIGH, clamped)) {
        NRF_GPIOTE->TASKS_SET[discharge_task] = 1;
    } else {
        NRF_GPIOTE->TASKS_CLR[discharge_task] = 1;
    }
}

static void discharge_settle(void) {
    /* Let the local rev2 switch close before ROW falls. Rev1 drives HOLD directly. */
    if (DISCHARGE_ACTIVE_HIGH) {
        k_busy_wait(1);
    }
}

static void ppi_disable(void) {
    nrfx_ppi_channel_disable(ppi_release);
    nrfx_ppi_channel_disable(ppi_row);
    nrfx_ppi_channel_disable(ppi_sample);
}

static void analog_safe(void) {
    NRF_TIMER2->TASKS_STOP = 1;
    ppi_disable();
    /* Clamp HOLD before lowering ROW, to avoid a negative-going HOLD pulse. */
    discharge_gpiote(true);
    discharge_settle();
    for (unsigned r = 0; r < ROWS; r++) {
        NRF_GPIOTE->TASKS_CLR[row_tasks[r]] = 1;
    }
    nrf_gpio_pin_set(enables[0]);
    nrf_gpio_pin_set(enables[1]);
    nrf_gpio_pin_clear(amplifier);
    nrf_gpio_pin_clear(battery_enable);
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
}

static bool adc_stop(void) {
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP = 1;
    return wait_event(&NRF_SAADC->EVENTS_STOPPED, 250);
}

static void adc_configure(uint8_t input, nrf_saadc_acqtime_t acquisition) {
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    NRF_SAADC->SAMPLERATE = 0;
    nrf_saadc_channel_config_t config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = NRF_SAADC_GAIN1_6,
        .reference = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time = acquisition,
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED,
    };
    nrf_saadc_channel_init(NRF_SAADC, 0, &config);
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0 + input;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
}

static int adc_start(void) {
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_result;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_STARTED = 0;
    __DMB();
    NRF_SAADC->TASKS_START = 1;
    return wait_event(&NRF_SAADC->EVENTS_STARTED, 250) ? 0 : -ETIMEDOUT;
}

static int sample_key(unsigned row) {
    int ret = adc_start();
    if (ret != 0) {
        adc_stop();
        return ret;
    }
#if defined(CONFIG_SHELL)
    if (software_edges) {
        /* Bypass TIMER/PPI/GPIOTE routing without changing the electrical
         * sequence or board-specific GPIO drive. Keep HOLD clamped before
         * either ROW falling edge or task handover.
         */
        discharge_gpio(true);
        nrf_gpio_pin_clear(rows[row]);
        nrf_gpiote_task_disable(NRF_GPIOTE, discharge_task);
        nrf_gpiote_task_disable(NRF_GPIOTE, row_tasks[row]);
        k_busy_wait(10);
        discharge_gpio(false);
        k_busy_wait(1);
        nrf_gpio_pin_set(rows[row]);
        k_busy_wait(sample_delay);
        NRF_SAADC->TASKS_SAMPLE = 1;
        bool completed = wait_event(&NRF_SAADC->EVENTS_END, 250);
        discharge_gpio(true);
        discharge_settle();
        nrf_gpio_pin_clear(rows[row]);
        nrf_gpiote_task_enable(NRF_GPIOTE, row_tasks[row]);
        nrf_gpiote_task_enable(NRF_GPIOTE, discharge_task);
        bool stopped = adc_stop();
        if (!completed || !stopped) {
            return -ETIMEDOUT;
        }
        return CLAMP(adc_result, 0, 4095);
    }
#endif
    NRF_TIMER2->TASKS_CLEAR = 1;
    for (unsigned i = 0; i < 4; i++) {
        NRF_TIMER2->EVENTS_COMPARE[i] = 0;
    }
    /* All three edges use the same 1 MHz timer, with no interrupt masking. */
    NRF_TIMER2->CC[0] = 1;                  /* board-specific HOLD release */
    NRF_TIMER2->CC[1] = 2;                  /* ROW rising edge */
    NRF_TIMER2->CC[2] = 2 + sample_delay;   /* SAADC SAMPLE, then 3 us acquisition */
    NRF_TIMER2->CC[3] = 100;                /* bounded one-shot */
    nrfx_ppi_channel_assign(ppi_row, (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1],
                           (uint32_t)&NRF_GPIOTE->TASKS_SET[row_tasks[row]]);
    nrfx_ppi_channel_enable(ppi_release);
    nrfx_ppi_channel_enable(ppi_row);
    nrfx_ppi_channel_enable(ppi_sample);
    NRF_TIMER2->TASKS_START = 1;
    bool completed = wait_event(&NRF_SAADC->EVENTS_END, 250);
    NRF_TIMER2->TASKS_STOP = 1;
    ppi_disable();
    discharge_gpiote(true);
    discharge_settle();
    NRF_GPIOTE->TASKS_CLR[row_tasks[row]] = 1;
    bool stopped = adc_stop();
    if (!completed || !stopped) {
        return -ETIMEDOUT;
    }
    return CLAMP(adc_result, 0, 4095);
}

static void select_column(unsigned col) {
    nrf_gpio_pin_set(enables[0]);
    nrf_gpio_pin_set(enables[1]);
    for (unsigned bit = 0; bit < 3; bit++) {
        nrf_gpio_pin_write(selects[bit], (channels[col] >> bit) & 1);
    }
    nrf_gpio_pin_clear(enables[col / 6]);
}

#if defined(CONFIG_PREC_EC_KEY_STANDBY)
static int read_standby_key(void) {
    uint32_t start = k_cycle_get_32();
    adc_configure(DT_PROP(NODE, ec_analog_input), NRF_SAADC_ACQTIME_3US);
    nrf_gpio_pin_set(amplifier);
    k_busy_wait(100);
    select_column(CONFIG_PREC_EC_STANDBY_COL);
    k_busy_wait(10);
    int ret = sample_key(CONFIG_PREC_EC_STANDBY_ROW);
    analog_safe();
    standby_last_us = k_cyc_to_us_floor32(k_cycle_get_32() - start);
    standby_max_us = MAX(standby_max_us, standby_last_us);
    return ret;
}

static bool all_keys_released(void) {
    for (unsigned i = 0; i < KEYS; i++) {
        if (!valid_key[i] || keys[i].pressed || keys[i].consecutive ||
            (int32_t)raw[i] > (int32_t)baseline[i] + release_delta) {
            return false;
        }
#if defined(CONFIG_SHELL)
        if (observed_keys[i].pressed || observed_keys[i].consecutive) {
            return false;
        }
#endif
    }
    return true;
}
#endif

static int read_frame(void) {
    adc_configure(DT_PROP(NODE, ec_analog_input), NRF_SAADC_ACQTIME_3US);
    nrf_gpio_pin_set(amplifier);
    k_busy_wait(100);
    int ret = 0;
    for (unsigned c = 0; c < COLS; c++) {
        select_column(c);
        for (unsigned r = 0; r < ROWS; r++) {
            k_busy_wait(10);
            ret = sample_key(r);
            if (ret < 0) {
                goto done;
            }
            raw[r * COLS + c] = ret;
        }
    }
    ret = 0;
done:
    analog_safe();
    return ret;
}

static int read_battery(void) {
    adc_configure(DT_PROP(NODE, battery_analog_input), NRF_SAADC_ACQTIME_10US);
    nrf_gpio_pin_set(battery_enable);
    k_msleep(5);
    uint32_t sum = 0;
    int ret = 0;
    /* Discard the first conversion after changing SAADC inputs. */
    for (unsigned i = 0; i < 9; i++) {
        ret = adc_start();
        if (ret != 0) {
            adc_stop();
            break;
        }
        NRF_SAADC->TASKS_SAMPLE = 1;
        bool completed = wait_event(&NRF_SAADC->EVENTS_END, 250);
        bool stopped = adc_stop();
        if (!completed || !stopped) {
            ret = -ETIMEDOUT;
            break;
        }
        if (i != 0) {
            sum += CLAMP(adc_result, 0, 4095);
        }
    }
    analog_safe();
    if (ret == 0) {
        /* 0.6 V reference, gain 1/6, 12 bits, 100k/100k divider. */
        battery_mv = (sum * 7200U + 16384U) / 32768U;
        battery_valid = battery_mv >= 500 && battery_mv <= 4000;
    } else {
        battery_valid = false;
    }
    return ret;
}

static void release_keys(void) {
    for (unsigned i = 0; i < KEYS; i++) {
        if (keys[i].pressed && key_callback && atomic_get(&callbacks_enabled)) {
            key_callback(scan_device, i / COLS, i % COLS, false);
        }
    }
    memset(keys, 0, sizeof(keys));
}

static void begin_calibration(void) {
    release_keys();
    calibrated = false;
    recalibrate = false;
    calibration_count = 0;
    memset(cal_sum, 0, sizeof(cal_sum));
    memset(cal_min, 0xff, sizeof(cal_min));
    memset(cal_max, 0, sizeof(cal_max));
    memset(minimum, 0xff, sizeof(minimum));
    memset(maximum, 0, sizeof(maximum));
    memset(valid_key, 0, sizeof(valid_key));
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    standby = (struct ec_standby_state){.held_since = -1};
    standby_requested = standby_exit_requested = false;
    standby_reference_valid = false;
    standby_cal_sum = standby_cal_samples = 0;
    standby_cal_min = UINT16_MAX;
    standby_cal_max = 0;
#endif
#if defined(CONFIG_SHELL)
    memset(observed_keys, 0, sizeof(observed_keys));
    memset(observed_presses, 0, sizeof(observed_presses));
    memset(observed_releases, 0, sizeof(observed_releases));
    memset(short_press_pulses, 0, sizeof(short_press_pulses));
#endif
}

static void update_calibration(void) {
    for (unsigned i = 0; i < KEYS; i++) {
        cal_sum[i] += raw[i];
        cal_min[i] = MIN(cal_min[i], raw[i]);
        cal_max[i] = MAX(cal_max[i], raw[i]);
    }
    if (++calibration_count == CALIBRATION_FRAMES) {
        for (unsigned i = 0; i < KEYS; i++) {
            baseline[i] = (cal_sum[i] + CALIBRATION_FRAMES / 2) / CALIBRATION_FRAMES;
            valid_key[i] = cal_max[i] - cal_min[i] <= STARTUP_NOISE_LIMIT &&
                           baseline[i] + press_delta < 4095;
        }
        calibrated = true;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
        /* Cold single-key acquisition has its own reference; do not assume it
         * has exactly the same offset as a key partway through a full frame.
         */
        standby_baseline = (standby_cal_sum + CALIBRATION_FRAMES / 2) / CALIBRATION_FRAMES;
        standby_reference_valid = standby_cal_samples == CALIBRATION_FRAMES &&
            standby_cal_max - standby_cal_min <= STARTUP_NOISE_LIMIT &&
            standby_baseline + press_delta < 4095 && valid_key[STANDBY_INDEX];
#endif
    }
}

static void enter_bootloader(void) {
    if (bootmode_set(BOOT_MODE_TYPE_BOOTLOADER) == 0) {
        sys_reboot(SYS_REBOOT_WARM);
    }
}

static void start_measurements(void) {
    if (atomic_cas(&scanner_started, 0, 1)) {
        k_sem_give(&scanner_gate);
    }
}

static void scanner_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    int64_t last_active = k_uptime_get(), next_battery = 0, button_since = -1;
    bool button_handled = false;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    bool usb_was_present = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
#endif
    k_msleep(500);
    if (nrf_gpio_pin_read(wake) == 0) {
        enter_bootloader();
    }
    k_sem_take(&scanner_gate, K_FOREVER);
    for (;;) {
        int64_t frame_start_ticks = k_uptime_ticks();
        int64_t frame_start = k_ticks_to_ms_floor64(frame_start_ticks);
        k_mutex_lock(&scan_lock, K_FOREVER);
        if (recalibrate) {
            begin_calibration();
        }
        bool usb_present = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
        if (frame_start >= next_battery) {
            if (read_battery() != 0) {
                adc_errors++;
            }
            next_battery = k_uptime_get() + 60000;
            if (!usb_present && battery_valid && battery_mv < 2000) {
                low_voltage_count++;
            } else if (usb_present || !battery_valid || battery_mv >= 2200) {
                low_voltage_count = 0;
            }
            if (low_voltage_count >= 3 && nrf_gpio_pin_read(wake)) {
                release_keys();
                analog_safe();
                nrf_gpio_cfg_sense_input(wake, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
                sys_poweroff();
            }
        }
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
        if (standby.mode == EC_SCAN_WATCH &&
            (standby_exit_requested || (usb_present && !usb_was_present) ||
             nrf_gpio_pin_read(wake) == 0)) {
            ec_standby_exit(&standby);
            last_active = frame_start;
        }
        standby_exit_requested = false;
        usb_was_present = usb_present;
        bool watch_frame = standby.mode == EC_SCAN_WATCH;
#endif
        uint32_t start = k_cycle_get_32();
        int ret;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
        if (watch_frame) {
            int value = read_standby_key();
            ret = value < 0 ? value : 0;
            if (ret == 0) {
                standby_raw = value;
                standby_samples++;
                if (ec_standby_watch_update(&standby, value, standby_baseline,
                                            press_delta, release_delta, k_uptime_get(),
                                            CONFIG_PREC_EC_STANDBY_HOLD_MS)) {
                    standby_wakes++;
                    last_active = k_uptime_get();
                }
            } else {
                ec_standby_reset_hold(&standby);
            }
        } else
#endif
        {
            ret = read_frame();
            last_frame_us = k_cyc_to_us_floor32(k_cycle_get_32() - start);
            max_frame_us = MAX(max_frame_us, last_frame_us);
        }
        if (ret != 0) {
            adc_errors++;
            release_keys();
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
            standby.released_frames = 0;
#endif
            /* A partial frame must never contribute to calibration or HID events. */
            if (!calibrated) {
                recalibrate = true;
            }
        }
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
        else if (watch_frame) {
            /* Other raw[] positions retain their last full-frame values. */
        }
#endif
        else {
            frame_count++;
            if (!calibrated) {
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
                int value = read_standby_key();
                if (value >= 0) {
                    standby_raw = value;
                    standby_cal_sum += value;
                    standby_cal_min = MIN(standby_cal_min, value);
                    standby_cal_max = MAX(standby_cal_max, value);
                    standby_cal_samples++;
                } else {
                    /* Disable automatic standby if its own calibration failed. */
                    adc_errors++;
                }
#endif
                update_calibration();
                last_active = frame_start;
            } else {
                bool report_frame = true;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
                if (standby.mode == EC_SCAN_RELEASE) {
                    report_frame = false;
                    ec_standby_release_update(&standby, all_keys_released());
                    last_active = frame_start;
                }
#endif
                for (unsigned i = 0; i < KEYS; i++) {
                    minimum[i] = MIN(minimum[i], raw[i]);
                    maximum[i] = MAX(maximum[i], raw[i]);
                    if (valid_key[i] && raw[i] > baseline[i] + release_delta) {
                        last_active = frame_start;
                    }
                    if (!report_frame) {
                        continue;
                    }
#if defined(CONFIG_SHELL)
                    if (valid_key[i] && !observed_keys[i].pressed &&
                        observed_keys[i].consecutive == 1 &&
                        (int32_t)raw[i] < (int32_t)baseline[i] + press_delta) {
                        short_press_pulses[i]++;
                    }
                    if (valid_key[i] && ec_key_update(&observed_keys[i], raw[i], baseline[i],
                                                     press_delta, release_delta)) {
                        if (observed_keys[i].pressed) {
                            observed_presses[i]++;
                        } else {
                            observed_releases[i]++;
                        }
                    }
#endif
                    if (report_keys && atomic_get(&callbacks_enabled) && key_callback && valid_key[i] &&
                        ec_key_update(&keys[i], raw[i], baseline[i], press_delta, release_delta)) {
                        key_callback(scan_device, i / COLS, i % COLS, keys[i].pressed);
                    }
                }
            }
        }
        if (nrf_gpio_pin_read(wake) == 0) {
            if (button_since < 0) {
                button_since = frame_start;
            }
            if (!button_handled && frame_start - button_since >= 2000) {
                recalibrate = true;
                button_handled = true;
            }
        } else {
            button_since = -1;
            button_handled = false;
        }
        unsigned period = ret != 0 ? 20 :
            (frame_start - last_active < 1000 ? active_period : idle_period);
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
        /* Automatic entry is battery-only and never interrupts a held/pending
         * key or a paused diagnostic session. Manual entry supports USB tests.
         */
        bool automatic_entry = !usb_present && nrf_gpio_pin_read(wake) &&
            report_keys && atomic_get(&callbacks_enabled) &&
            frame_start - last_active >= CONFIG_PREC_EC_STANDBY_TIMEOUT_MS;
        if (standby.mode == EC_SCAN_FULL && (automatic_entry || standby_requested) &&
            ret == 0 && calibrated && standby_reference_valid && all_keys_released()) {
            release_keys();
#if defined(CONFIG_SHELL)
            memset(observed_keys, 0, sizeof(observed_keys));
#endif
            ec_standby_enter(&standby);
            standby_entries++;
        }
        standby_requested = false;
        if (standby.mode == EC_SCAN_WATCH) {
            period = CONFIG_PREC_EC_STANDBY_PERIOD_MS;
        } else if (standby.mode == EC_SCAN_RELEASE && ret == 0) {
            period = active_period;
        }
#endif
        k_mutex_unlock(&scan_lock);
        /* Preserve sub-ms slack: rounding both endpoints down to milliseconds
         * can falsely overrun a 5 ms frame that actually completed in <5 ms.
         */
        int64_t remaining = frame_start_ticks + k_ms_to_ticks_ceil64(period) - k_uptime_ticks();
        if (remaining <= 0) {
            atomic_inc(&overruns);
            k_msleep(1);
        } else {
            k_sleep(K_TICKS(remaining));
        }
    }
}

static int configure(const struct device *dev, kscan_callback_t callback) {
    ARG_UNUSED(dev);
    if (!callback) {
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    key_callback = callback;
    k_mutex_unlock(&scan_lock);
    return 0;
}

static int enable(const struct device *dev) {
    ARG_UNUSED(dev);
    atomic_set(&callbacks_enabled, 1);
    return 0;
}

static int disable(const struct device *dev) {
    ARG_UNUSED(dev);
    k_mutex_lock(&scan_lock, K_FOREVER);
    release_keys();
    atomic_clear(&callbacks_enabled);
    k_mutex_unlock(&scan_lock);
    return 0;
}

static K_THREAD_STACK_DEFINE(scanner_stack, 3072);
static struct k_thread scanner;

static int init(const struct device *dev) {
    scan_device = dev;
    nrf_gpio_cfg_output(amplifier);
    nrf_gpio_pin_clear(amplifier);
    nrf_gpio_cfg_output(battery_enable);
    nrf_gpio_pin_clear(battery_enable);
    nrf_gpio_cfg_input(wake, NRF_GPIO_PIN_PULLUP);
    for (unsigned i = 0; i < 2; i++) {
        nrf_gpio_pin_set(enables[i]);
        nrf_gpio_cfg_output(enables[i]);
    }
    for (unsigned i = 0; i < 3; i++) {
        nrf_gpio_pin_clear(selects[i]);
        nrf_gpio_cfg_output(selects[i]);
    }
    for (unsigned r = 0; r < ROWS; r++) {
        nrf_gpio_pin_clear(rows[r]);
        nrf_gpio_cfg_output(rows[r]);
    }
    /* Preload the safe latch before enabling output: rev1 Low/open drain;
     * rev2 High/push-pull closes TMUX1101. Rev1 never sources EC_HOLD.
     */
    discharge_gpio(true);
    nrf_gpio_cfg(discharge, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL, DISCHARGE_DRIVE, NRF_GPIO_PIN_NOSENSE);
    for (unsigned r = 0; r < ROWS; r++) {
        if (nrfx_gpiote_channel_alloc(&gpiote, &row_tasks[r]) != NRFX_SUCCESS) {
            return -ENOMEM;
        }
        nrf_gpiote_task_configure(NRF_GPIOTE, row_tasks[r], rows[r],
                                  NRF_GPIOTE_POLARITY_LOTOHI, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_enable(NRF_GPIOTE, row_tasks[r]);
    }
    if (nrfx_gpiote_channel_alloc(&gpiote, &discharge_task) != NRFX_SUCCESS ||
        nrfx_ppi_channel_alloc(&ppi_release) != NRFX_SUCCESS ||
        nrfx_ppi_channel_alloc(&ppi_row) != NRFX_SUCCESS ||
        nrfx_ppi_channel_alloc(&ppi_sample) != NRFX_SUCCESS) {
        return -ENOMEM;
    }
    nrf_gpiote_task_configure(NRF_GPIOTE, discharge_task, discharge,
                              NRF_GPIOTE_POLARITY_LOTOHI, DISCHARGE_ACTIVE_HIGH ?
                              NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(NRF_GPIOTE, discharge_task);
    NRF_TIMER2->TASKS_STOP = 1;
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER2->PRESCALER = 4; /* 16 MHz / 16 = 1 MHz */
    NRF_TIMER2->INTENCLR = UINT32_MAX;
    NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE3_STOP_Msk;
    nrfx_ppi_channel_assign(ppi_release, (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0],
                           DISCHARGE_ACTIVE_HIGH ?
                           (uint32_t)&NRF_GPIOTE->TASKS_CLR[discharge_task] :
                           (uint32_t)&NRF_GPIOTE->TASKS_SET[discharge_task]);
    nrfx_ppi_channel_assign(ppi_sample, (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[2],
                           (uint32_t)&NRF_SAADC->TASKS_SAMPLE);
    hardware_ready = true;
    NRF_SAADC->INTENCLR = UINT32_MAX;
    for (unsigned i = 0; i < 8; i++) {
        NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
        NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELN_PSELN_NC;
    }
    adc_configure(DT_PROP(NODE, ec_analog_input), NRF_SAADC_ACQTIME_3US);
    NRF_SAADC->RESULT.PTR = (uint32_t)calibration_scratch;
    NRF_SAADC->RESULT.MAXCNT = ARRAY_SIZE(calibration_scratch);
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    __DMB();
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    bool calibration_ok = wait_event(&NRF_SAADC->EVENTS_CALIBRATEDONE, 10000);
    /* Match nrfx's post-calibration flush: latch a two-sample scratch buffer
     * with START before STOP. Spurious calibration samples must not become
     * the first key/battery conversion (nor target an uninitialised DMA pointer).
     */
    bool latched = false;
    if (calibration_ok) {
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->EVENTS_STARTED = 0;
        NRF_SAADC->RESULT.PTR = (uint32_t)calibration_scratch;
        NRF_SAADC->RESULT.MAXCNT = ARRAY_SIZE(calibration_scratch);
        __DMB();
        NRF_SAADC->TASKS_START = 1;
        latched = wait_event(&NRF_SAADC->EVENTS_STARTED, 250);
    }
    bool stopped = adc_stop();
    NRF_SAADC->EVENTS_END = 0;
    analog_safe();
    if (!calibration_ok || !latched || !stopped) {
        return -EIO;
    }
    k_thread_create(&scanner, scanner_stack, K_THREAD_STACK_SIZEOF(scanner_stack),
                    scanner_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&scanner, "prec_ec");
    if (IS_ENABLED(CONFIG_PREC_EC_AUTO_SCAN)) {
        start_measurements();
    }
    return 0;
}

static const struct kscan_driver_api api = {
    .config = configure,
    .enable_callback = enable,
    .disable_callback = disable,
};
DEVICE_DT_INST_DEFINE(0, init, NULL, NULL, NULL, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY, &api);

/* No second SAADC client: ZMK consumes the cached measurement from this sensor. */
static int battery_fetch(const struct device *dev, enum sensor_channel chan) {
    ARG_UNUSED(dev);
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_GAUGE_VOLTAGE &&
        chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
        return -ENOTSUP;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    bool valid = battery_valid;
    k_mutex_unlock(&scan_lock);
    return valid ? 0 : -ENODATA;
}

static int battery_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    ARG_UNUSED(dev);
    k_mutex_lock(&scan_lock, K_FOREVER);
    int ret = battery_fetch(dev, chan);
    if (ret == 0) {
        if (chan == SENSOR_CHAN_GAUGE_VOLTAGE) {
            val->val1 = battery_mv / 1000;
            val->val2 = (battery_mv % 1000) * 1000;
        } else if (chan == SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
            val->val1 = ec_alkaline_percent(battery_mv);
            val->val2 = 0;
        } else {
            ret = -ENOTSUP;
        }
    }
    k_mutex_unlock(&scan_lock);
    return ret;
}

static const struct sensor_driver_api battery_api = {
    .sample_fetch = battery_fetch,
    .channel_get = battery_get,
};
DEVICE_DT_DEFINE(DT_NODELABEL(alkaline_battery), NULL, NULL, NULL, NULL,
                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &battery_api);

#if defined(CONFIG_SHELL)
/* Called with scan_lock held. Keep the GPIO latch clamped during task handover. */
static void stop_static_probe(void) {
    if (static_probe) {
        discharge_gpio(true);
        nrf_gpiote_task_enable(NRF_GPIOTE, discharge_task);
        static_probe = false;
        analog_safe();
    }
}

#if defined(CONFIG_PREC_EC_KEY_STANDBY)
static int cmd_standby(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    if (strcmp(argv[1], "status") != 0 && strcmp(argv[1], "enter") != 0 &&
        strcmp(argv[1], "exit") != 0) {
        shell_error(sh, "Use: ec standby <status|enter|exit>");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    bool can_enter = atomic_get(&scanner_started) && calibrated && !recalibrate &&
        standby_reference_valid && all_keys_released() && standby.mode == EC_SCAN_FULL;
    bool rejected = strcmp(argv[1], "enter") == 0 && !can_enter;
    if (!rejected && strcmp(argv[1], "enter") == 0) {
        standby_requested = true;
    } else if (strcmp(argv[1], "exit") == 0) {
        standby_exit_requested = true;
    }
    enum ec_standby_mode mode = standby.mode;
    uint16_t value = standby_raw, reference = standby_baseline;
    unsigned noise = standby_cal_samples ? standby_cal_max - standby_cal_min : 0;
    bool reference_valid = standby_reference_valid;
    uint32_t samples = standby_samples, entries = standby_entries, wakes = standby_wakes;
    uint32_t duration = standby_last_us, max_duration = standby_max_us;
    uint32_t reference_samples = standby_cal_samples;
    k_mutex_unlock(&scan_lock);
    if (rejected) {
        shell_error(sh, "Requires full-scan mode, valid calibration/reference and all 48 keys released");
        return -EBUSY;
    }
    shell_print(sh, "standby=%s row=%u col=%u timeout=%ums period=%ums hold=%ums samples=%u entries=%u wakes=%u",
                mode == EC_SCAN_WATCH ? "watch" : mode == EC_SCAN_RELEASE ? "release_gate" : "full",
                CONFIG_PREC_EC_STANDBY_ROW, CONFIG_PREC_EC_STANDBY_COL,
                CONFIG_PREC_EC_STANDBY_TIMEOUT_MS, CONFIG_PREC_EC_STANDBY_PERIOD_MS,
                CONFIG_PREC_EC_STANDBY_HOLD_MS, samples, entries, wakes);
    shell_print(sh, "standby_raw=%u reference=%u delta=%d noise=%u valid=%d ref_samples=%u sample=%uus max=%uus",
                value, reference, (int)value - reference, noise, reference_valid,
                reference_samples, duration, max_duration);
    if (strcmp(argv[1], "status") != 0) {
        shell_print(sh, "Request queued; exit uses two released full frames before HID resumes");
    }
    return 0;
}
#endif

static int cmd_status(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    k_mutex_lock(&scan_lock, K_FOREVER);
    unsigned valid = 0;
    for (unsigned i = 0; i < KEYS; i++) {
        valid += valid_key[i];
    }
    const bool ready = device_is_ready(scan_device), is_calibrated = calibrated;
    const bool reporting = report_keys, bat_valid = battery_valid, direct = software_edges;
    const bool probe = static_probe;
    const uint32_t frames = frame_count, errors = adc_errors;
    const uint32_t duration = last_frame_us, max_duration = max_frame_us;
    const uint16_t press = press_delta, release = release_delta, delay = sample_delay;
    const uint16_t active_ms = active_period, idle_ms = idle_period;
    const uint16_t mv = battery_mv;
    const uint8_t low_count = low_voltage_count;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "ready=%d scanning=%ld calibrated=%d valid=%u/48 report=%d callbacks=%ld frames=%u errors=%u overruns=%ld",
                ready, (long)atomic_get(&scanner_started), is_calibrated, valid, reporting, (long)atomic_get(&callbacks_enabled),
                frames, errors, (long)atomic_get(&overruns));
    shell_print(sh, "press=+%u release=+%u sample_delay=%uus frame=%uus max=%uus",
                press, release, delay, duration, max_duration);
    shell_print(sh, "period_active=%ums period_idle=%ums", active_ms, idle_ms);
    shell_print(sh, "battery=%umV valid=%d approx=%u%% warning=%d low_count=%u",
                mv, bat_valid, ec_alkaline_percent(mv), bat_valid && mv < 2200, low_count);
    shell_print(sh, "edges=%s analog_probe=%d",
                direct ? "software (diagnostic, interrupt-sensitive)" : "timer", probe);
    shell_print(sh, "board=%s discharge=%s",
                DISCHARGE_ACTIVE_HIGH ? "rev2" : "rev1",
                DISCHARGE_ACTIVE_HIGH ? "push-pull High=clamp Low=release" :
                                        "open-drain Low=clamp High-Z=release");
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    char *standby_args[] = {"standby", "status"};
    cmd_standby(sh, 2, standby_args);
#endif
    return 0;
}

static int cmd_raw(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    /* Copy first: terminal output must not hold up the scan thread. */
    uint16_t snap[5][KEYS];
    bool valid[KEYS];
    bool observed[KEYS];
    uint32_t presses[KEYS], releases[KEYS];
    k_mutex_lock(&scan_lock, K_FOREVER);
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    bool watching = standby.mode == EC_SCAN_WATCH;
#endif
    memcpy(snap[0], raw, sizeof(raw));
    memcpy(snap[1], baseline, sizeof(baseline));
    memcpy(snap[2], minimum, sizeof(minimum));
    memcpy(snap[3], maximum, sizeof(maximum));
    for (unsigned i = 0; i < KEYS; i++) {
        snap[4][i] = cal_max[i] - cal_min[i];
    }
    memcpy(valid, valid_key, sizeof(valid));
    for (unsigned i = 0; i < KEYS; i++) {
        observed[i] = observed_keys[i].pressed;
    }
    memcpy(presses, observed_presses, sizeof(presses));
    memcpy(releases, observed_releases, sizeof(releases));
    k_mutex_unlock(&scan_lock);
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    if (watching) {
        shell_print(sh, "raw_snapshot=last_full_frame; use ec standby status for the live wake key");
    }
#endif
    shell_print(sh, "row col raw baseline delta min max startup_noise valid pressed presses releases");
    for (unsigned i = 0; i < KEYS; i++) {
        shell_print(sh, "%u %u %u %u %d %u %u %u %d %d %u %u", i / COLS, i % COLS,
                    snap[0][i], snap[1][i], (int)snap[0][i] - snap[1][i],
                    snap[2][i], snap[3][i], snap[4][i], valid[i], observed[i], presses[i], releases[i]);
    }
    return 0;
}

struct link_snapshot {
    struct {
        uint16_t interval, latency, timeout;
        uint8_t index, role;
    } links[CONFIG_BT_MAX_CONN];
    unsigned count;
};

static void snapshot_link(struct bt_conn *conn, void *data) {
    struct link_snapshot *snap = data;
    struct bt_conn_info info;
    if (snap->count >= ARRAY_SIZE(snap->links) || bt_conn_get_info(conn, &info) != 0 ||
        info.type != BT_CONN_TYPE_LE || info.state != BT_CONN_STATE_CONNECTED) {
        return;
    }
    unsigned i = snap->count++;
    snap->links[i].interval = info.le.interval;
    snap->links[i].latency = info.le.latency;
    snap->links[i].timeout = info.le.timeout;
    snap->links[i].index = bt_conn_index(conn);
    snap->links[i].role = info.role;
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    struct link_snapshot snap = {0};
    bt_conn_foreach(BT_CONN_TYPE_LE, snapshot_link, &snap);
    shell_print(sh, "BLE preferred: interval=%u..%uus latency=%u events",
                CONFIG_BT_PERIPHERAL_PREF_MIN_INT * 1250U,
                CONFIG_BT_PERIPHERAL_PREF_MAX_INT * 1250U,
                CONFIG_BT_PERIPHERAL_PREF_LATENCY);
    shell_print(sh, "Connected LE links=%u (connection state, not selected HID output)", snap.count);
    for (unsigned i = 0; i < snap.count; i++) {
        shell_print(sh, "index=%u role=%s interval=%uus latency=%u events timeout=%ums",
                    snap.links[i].index,
                    snap.links[i].role == BT_CONN_ROLE_PERIPHERAL ? "peripheral" : "central",
                    snap.links[i].interval * 1250U, snap.links[i].latency,
                    snap.links[i].timeout * 10U);
    }
    return 0;
}

static int cmd_calibrate(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    k_mutex_lock(&scan_lock, K_FOREVER);
    recalibrate = true;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "Recalibrating 32 frames. Keep all keys released.");
    return 0;
}

static int cmd_pulses(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    uint32_t counts[KEYS];
    k_mutex_lock(&scan_lock, K_FOREVER);
    memcpy(counts, short_press_pulses, sizeof(counts));
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "row col single_sample_press_pulses");
    for (unsigned i = 0; i < KEYS; i++) {
        shell_print(sh, "%u %u %u", i / COLS, i % COLS, counts[i]);
    }
    return 0;
}

static int cmd_analog(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    if (strcmp(argv[1], "clamp") != 0 && strcmp(argv[1], "off") != 0) {
        shell_error(sh, "Use: ec analog <clamp|off>; only before ec start");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    if (!device_is_ready(scan_device) || atomic_get(&scanner_started)) {
        k_mutex_unlock(&scan_lock);
        shell_error(sh, "Requires a ready scanner before ec start. Reset to stop scanning first.");
        return -EBUSY;
    }
    stop_static_probe();
    analog_safe();
    release_keys();
    report_keys = false;
    if (strcmp(argv[1], "off") == 0) {
        k_mutex_unlock(&scan_lock);
        shell_print(sh, "Probe off: amplifier disabled, HOLD clamped, ROWs Low, MUXes inhibited.");
        return 0;
    }
    /* A steady state for multimeter measurements. Rev1 clamps HOLD directly;
     * rev2 drives the local switch. No ROW pulse is generated.
     */
    discharge_gpio(true);
    nrf_gpiote_task_disable(NRF_GPIOTE, discharge_task);
    nrf_gpio_cfg(discharge, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL, DISCHARGE_DRIVE, NRF_GPIO_PIN_NOSENSE);
    static_probe = true;
    adc_configure(DT_PROP(NODE, ec_analog_input), NRF_SAADC_ACQTIME_10US);
    nrf_gpio_pin_set(amplifier);
    k_busy_wait(100);
    unsigned sum = 0, lo = 4095, hi = 0;
    int ret = 0;
    for (unsigned i = 0; i < 17; i++) {
        ret = adc_start();
        if (ret != 0) {
            adc_stop();
            break;
        }
        NRF_SAADC->TASKS_SAMPLE = 1;
        bool completed = wait_event(&NRF_SAADC->EVENTS_END, 250);
        bool stopped = adc_stop();
        if (!completed || !stopped) {
            ret = -ETIMEDOUT;
            break;
        }
        if (i != 0) {
            unsigned value = CLAMP(adc_result, 0, 4095);
            sum += value;
            lo = MIN(lo, value);
            hi = MAX(hi, value);
        }
    }
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
    if (ret != 0) {
        stop_static_probe();
    }
    k_mutex_unlock(&scan_lock);
    if (ret != 0) {
        shell_error(sh, "Static ADC measurement failed (%d); probe off", ret);
        return ret;
    }
    unsigned average = (sum + 8) / 16;
    shell_print(sh, "Static clamp: raw=%u min=%u max=%u approx=%umV", average, lo, hi,
                (average * 3600U + 2048) / 4096);
    shell_print(sh, "%s; ROWs Low; MUXes inhibited; amplifier stays ON.",
                DISCHARGE_ACTIVE_HIGH ? "HOLD clamped by local switch (control High)" :
                                        "HOLD driven Low by open-drain GPIO");
    shell_print(sh, "Use DC voltage mode for meter checks. ec analog off disables the amplifier.");
    return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    if (!device_is_ready(scan_device)) {
        shell_error(sh, "Scanner device failed to initialise");
        return -ENODEV;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    stop_static_probe();
    k_mutex_unlock(&scan_lock);
    start_measurements();
    shell_print(sh, "Measurements started; keep all keys released during calibration.");
    return 0;
}

static int cmd_reporting(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    k_mutex_lock(&scan_lock, K_FOREVER);
    release_keys();
    report_keys = strcmp(argv[0], "resume") == 0;
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    standby_exit_requested = true;
#endif
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "Key reporting %s", report_keys ? "enabled" : "paused");
    return 0;
}

static int parse_value(const char *text, unsigned max, unsigned *out) {
    char *end;
    unsigned long value = strtoul(text, &end, 10);
    if (*text == '\0' || *end != '\0' || value == 0 || value > max) {
        return -EINVAL;
    }
    *out = value;
    return 0;
}

static int cmd_thresholds(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    unsigned press, release;
    if (parse_value(argv[1], 3000, &press) || parse_value(argv[2], 2999, &release) || release >= press) {
        shell_error(sh, "Use: ec thresholds <press_delta> <release_delta>; 0 < release < press <= 3000");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    release_keys();
    press_delta = press;
    release_delta = release;
    recalibrate = true;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "Thresholds set; recalibrating. Keep keys released. Changes last until reset.");
    return 0;
}

static int cmd_timing(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    unsigned delay;
    if (parse_value(argv[1], 60, &delay)) {
        shell_error(sh, "Use: ec timing <sample_delay_us>, 1..60");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    sample_delay = delay;
    recalibrate = true;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "SAMPLE delay=%u us; recalibrating. Changes last until reset.", delay);
    return 0;
}

static int cmd_period(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    unsigned active_ms, idle_ms;
    if (parse_value(argv[1], 100, &active_ms) || parse_value(argv[2], 100, &idle_ms) ||
        idle_ms < active_ms) {
        shell_error(sh, "Use: ec period <active_ms> <idle_ms>; 1 <= active <= idle <= 100");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    if (active_ms * 1000U <= last_frame_us) {
        k_mutex_unlock(&scan_lock);
        shell_error(sh, "Active period must exceed the measured frame duration");
        return -EINVAL;
    }
    active_period = active_ms;
    idle_period = idle_ms;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "Periods: active=%u ms idle=%u ms; calibration unchanged; until reset",
                active_ms, idle_ms);
    return 0;
}

static int cmd_edges(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    if (strcmp(argv[1], "timer") != 0 && strcmp(argv[1], "software") != 0) {
        shell_error(sh, "Use: ec edges <timer|software>");
        return -EINVAL;
    }
    k_mutex_lock(&scan_lock, K_FOREVER);
    release_keys();
    report_keys = false;
    software_edges = strcmp(argv[1], "software") == 0;
    recalibrate = true;
    k_mutex_unlock(&scan_lock);
    shell_print(sh, "Edges=%s; reporting paused; recalibrating. Keep keys released.", argv[1]);
    return 0;
}

static int cmd_bootloader(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    k_mutex_lock(&scan_lock, K_FOREVER);
    release_keys();
    if (hardware_ready) {
        stop_static_probe();
        analog_safe();
    }
    enter_bootloader();
    k_mutex_unlock(&scan_lock);
    return -EIO;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ec_commands,
    SHELL_CMD_ARG(start, NULL, "Start measurements with HID reporting still paused", cmd_start, 1, 0),
    SHELL_CMD_ARG(status, NULL, "Scanner, timing and battery status", cmd_status, 1, 0),
    SHELL_CMD_ARG(link, NULL, "Actual connected BLE intervals and peripheral latency", cmd_link, 1, 0),
#if defined(CONFIG_PREC_EC_KEY_STANDBY)
    SHELL_CMD_ARG(standby, NULL, "<status|enter|exit>; enter also permits USB diagnostic tests", cmd_standby, 2, 0),
#endif
    SHELL_CMD_ARG(raw, NULL, "48 raw/baseline/min/max values", cmd_raw, 1, 0),
    SHELL_CMD_ARG(pulses, NULL, "Unconfirmed one-frame press crossings since calibration", cmd_pulses, 1, 0),
    SHELL_CMD_ARG(analog, NULL, "<clamp|off> static voltage probe before ec start", cmd_analog, 2, 0),
    SHELL_CMD_ARG(calibrate, NULL, "Recalibrate with all keys released", cmd_calibrate, 1, 0),
    SHELL_CMD_ARG(pause, NULL, "Release keys and pause HID reporting", cmd_reporting, 1, 0),
    SHELL_CMD_ARG(resume, NULL, "Enable HID reporting", cmd_reporting, 1, 0),
    SHELL_CMD_ARG(thresholds, NULL, "<press_delta> <release_delta>", cmd_thresholds, 3, 0),
    SHELL_CMD_ARG(timing, NULL, "<sample_delay_us> (1..60)", cmd_timing, 2, 0),
    SHELL_CMD_ARG(period, NULL, "<active_ms> <idle_ms>; no recalibration", cmd_period, 3, 0),
    SHELL_CMD_ARG(edges, NULL, "<timer|software> (diagnostic comparison, pauses HID)", cmd_edges, 2, 0),
    SHELL_CMD_ARG(bootloader, NULL, "Reboot into existing UF2 bootloader", cmd_bootloader, 1, 0),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(ec, &ec_commands, "PREC wireless EC diagnostics", NULL);
#endif
