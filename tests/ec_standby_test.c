#include <assert.h>
#include <stdio.h>
#include "../src/ec_standby.h"

static bool sample(struct ec_standby_state *state, unsigned raw, int64_t time) {
    return ec_standby_watch_update(state, raw, 1000, 500, 300, time, 300);
}

int main(void) {
    struct ec_standby_state state = {0};
    ec_standby_enter(&state);
    /* One spike and an isolated short tap cannot wake the keyboard. */
    assert(!sample(&state, 1600, 0));
    assert(!sample(&state, 1000, 50));
    assert(!sample(&state, 1000, 1000));
    assert(state.mode == EC_SCAN_WATCH && !state.wake_key.pressed);

    /* Continuous hold, with hysteresis, wakes at the configured duration. */
    assert(!sample(&state, 1500, 2000));
    assert(!sample(&state, 1500, 2050));
    assert(!sample(&state, 1400, 2299));
    assert(sample(&state, 1400, 2300));
    assert(state.mode == EC_SCAN_RELEASE);

    /* Wake holds and other held keys are consumed before returning to full HID. */
    assert(!ec_standby_release_update(&state, false));
    assert(!ec_standby_release_update(&state, true));
    assert(!ec_standby_release_update(&state, false));
    assert(!ec_standby_release_update(&state, true));
    assert(ec_standby_release_update(&state, true));
    assert(state.mode == EC_SCAN_FULL);
    assert(!sample(&state, 1600, 5000));

    /* Two short taps must not accumulate time into a long hold. */
    ec_standby_enter(&state);
    assert(!sample(&state, 1600, 0));
    assert(!sample(&state, 1600, 50));
    assert(!sample(&state, 1300, 200));
    assert(!sample(&state, 1000, 250));
    assert(!sample(&state, 1600, 350));
    assert(!sample(&state, 1600, 400));
    assert(!sample(&state, 1000, 550));
    assert(state.mode == EC_SCAN_WATCH);

    /* An ADC failure invalidates the hold candidate, including its elapsed time. */
    ec_standby_enter(&state);
    assert(!sample(&state, 1600, 0));
    assert(!sample(&state, 1600, 50));
    ec_standby_reset_hold(&state);
    assert(!sample(&state, 1600, 400));
    assert(!sample(&state, 1600, 450));
    assert(!sample(&state, 1600, 699));
    assert(sample(&state, 1600, 700));

    /* A forced/USB/button exit uses the same release gate. */
    ec_standby_enter(&state);
    ec_standby_exit(&state);
    assert(!ec_standby_release_update(&state, true));
    assert(ec_standby_release_update(&state, true));
    assert(!ec_standby_release_update(&state, true));
    puts("PASS: wake hold, noisy/short taps, ADC recovery and suppressed wake/release transitions");
    return 0;
}
