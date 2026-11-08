#ifndef PREC_EC_STANDBY_H
#define PREC_EC_STANDBY_H

#include "ec_logic.h"

enum ec_standby_mode { EC_SCAN_FULL, EC_SCAN_WATCH, EC_SCAN_RELEASE };

struct ec_standby_state {
    enum ec_standby_mode mode;
    struct ec_key_state wake_key;
    int64_t held_since;
    uint8_t released_frames;
};

static inline void ec_standby_reset_hold(struct ec_standby_state *state) {
    state->wake_key = (struct ec_key_state){0};
    state->held_since = -1;
}

static inline void ec_standby_enter(struct ec_standby_state *state) {
    ec_standby_reset_hold(state);
    state->released_frames = 0;
    state->mode = EC_SCAN_WATCH;
}

/* All wake causes pass through a gate that consumes held keys and their release. */
static inline void ec_standby_exit(struct ec_standby_state *state) {
    ec_standby_reset_hold(state);
    state->released_frames = 0;
    state->mode = EC_SCAN_RELEASE;
}

static inline bool ec_standby_watch_update(struct ec_standby_state *state,
                                          uint16_t raw, uint16_t baseline,
                                          uint16_t press, uint16_t release,
                                          int64_t now_ms, unsigned hold_ms) {
    if (state->mode != EC_SCAN_WATCH) {
        return false;
    }
    ec_key_update(&state->wake_key, raw, baseline, press, release);
    if ((int32_t)raw <= (int32_t)baseline + release ||
        (!state->wake_key.pressed && state->wake_key.consecutive == 0)) {
        state->held_since = -1;
    } else if (state->held_since < 0) {
        state->held_since = now_ms;
    }
    if (state->wake_key.pressed && state->held_since >= 0 &&
        now_ms - state->held_since >= hold_ms) {
        ec_standby_exit(state);
        return true;
    }
    return false;
}

/* Require two complete, successful frames with every key released. */
static inline bool ec_standby_release_update(struct ec_standby_state *state,
                                            bool all_released) {
    if (state->mode != EC_SCAN_RELEASE) {
        return false;
    }
    if (!all_released) {
        state->released_frames = 0;
        return false;
    }
    if (++state->released_frames < 2) {
        return false;
    }
    state->mode = EC_SCAN_FULL;
    return true;
}

#endif
