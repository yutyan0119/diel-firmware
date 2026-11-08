#ifndef PREC_EC_LOGIC_H
#define PREC_EC_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

struct ec_key_state {
    bool pressed;
    uint8_t consecutive;
};

/* Rev1: Low clamps, High releases an open drain. Rev2: High turns on TMUX1101. */
static inline bool ec_discharge_level(bool active_high, bool clamped) {
    return active_high == clamped;
}

/* Two consecutive samples outside the hysteresis band confirm a transition. */
static inline bool ec_key_update(struct ec_key_state *key, uint16_t raw, uint16_t baseline,
                                 uint16_t press, uint16_t release) {
    bool target = key->pressed ? (int32_t)raw > (int32_t)baseline + release
                               : (int32_t)raw >= (int32_t)baseline + press;
    if (target == key->pressed) {
        key->consecutive = 0;
        return false;
    }
    if (++key->consecutive < 2) {
        return false;
    }
    key->pressed = target;
    key->consecutive = 0;
    return true;
}

/* Provisional voltage indicator, not a measured capacity/discharge model. */
static inline uint8_t ec_alkaline_percent(uint16_t mv) {
    static const uint16_t voltage[] = {2000, 2200, 2400, 2600, 2800, 3000, 3200};
    static const uint8_t percent[] = {0, 5, 15, 35, 60, 85, 100};
    if (mv <= voltage[0]) {
        return 0;
    }
    for (unsigned i = 1; i < sizeof(voltage) / sizeof(voltage[0]); i++) {
        if (mv < voltage[i]) {
            return percent[i - 1] + (uint32_t)(mv - voltage[i - 1]) *
                       (percent[i] - percent[i - 1]) / (voltage[i] - voltage[i - 1]);
        }
    }
    return 100;
}

#endif
