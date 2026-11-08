#include <assert.h>
#include <stdio.h>
#include "../src/ec_logic.h"

int main(void) {
    /* Revision selection must preserve reset/clamp and invert measurement. */
    assert(!ec_discharge_level(false, true));
    assert(ec_discharge_level(false, false));
    assert(ec_discharge_level(true, true));
    assert(!ec_discharge_level(true, false));
    struct ec_key_state key = {0};
    assert(!ec_key_update(&key, 1250, 1000, 250, 180));
    assert(!key.pressed);
    /* A single noisy spike must not create a key press. */
    assert(!ec_key_update(&key, 1249, 1000, 250, 180));
    assert(!ec_key_update(&key, 1250, 1000, 250, 180));
    assert(ec_key_update(&key, 1250, 1000, 250, 180));
    assert(key.pressed);
    /* Values inside the hysteresis band keep the key down. */
    assert(!ec_key_update(&key, 1200, 1000, 250, 180));
    assert(key.pressed);
    assert(!ec_key_update(&key, 1180, 1000, 250, 180));
    assert(!ec_key_update(&key, 1181, 1000, 250, 180));
    assert(!ec_key_update(&key, 1180, 1000, 250, 180));
    assert(ec_key_update(&key, 1180, 1000, 250, 180));
    assert(!key.pressed);
    /* An unreachable threshold near ADC full scale must not wrap. */
    for (int i = 0; i < 10; i++) {
        assert(!ec_key_update(&key, 4095, 4000, 250, 180));
    }
    assert(!key.pressed);
    assert(ec_alkaline_percent(0) == 0);
    assert(ec_alkaline_percent(2000) == 0);
    assert(ec_alkaline_percent(2200) == 5);
    assert(ec_alkaline_percent(2600) == 35);
    assert(ec_alkaline_percent(3200) == 100);
    assert(ec_alkaline_percent(6000) == 100);
    for (unsigned mv = 1; mv < 6000; mv++) {
        assert(ec_alkaline_percent(mv) >= ec_alkaline_percent(mv - 1));
        assert(ec_alkaline_percent(mv) <= 100);
    }
    puts("PASS: revision discharge levels, hysteresis, debounce, ADC saturation and alkaline indicator");
    return 0;
}
