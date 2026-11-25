// =====================================================================
//  Ariadna V2 — Static Fire Link: the clamp ("klapka") profile
//
//  Pure logic, no Arduino dependency; tested on a workstation
//  (`cd test && make sflink`) and compiled unchanged into the sketch.
//
//  The motor cannot be throttled, so on the stand the clamp is what is
//  being studied: it diverts part of the thrust. The profile this runs:
//
//      T0 .. thrust > 100 N      fully OPEN                (WAIT)
//      thrust crosses 100 N      50 %  open  for 300 ms    (S1)
//                                minimum     for 300 ms    (S2)
//                                75 %  open  for 300 ms    (S3)
//                                fully open until the end  (FULL)
//
//  Percentages are how far the clamp is OPEN: 100 = open (the flight
//  firmware's position at ignition), 0 = shut as far as the linkage goes.
//  "Minimum" is the flight software's own floor, DG_CLAMP_MIN = 0.10 (so the
//  clamp is 90 % closed there); change open_pct[1] to taste.
//
//  Fail-safe is always OPEN: no trigger within giveup_us, or the stand
//  link lost, and the clamp goes to fully open and stays there.
// =====================================================================
#ifndef SF_FLAPS_H
#define SF_FLAPS_H

#include <stdint.h>

typedef enum {
    SFF_WAIT   = 0,   // open, waiting for the thrust to cross the trigger
    SFF_S1     = 1,   // first step (50 %)
    SFF_S2     = 2,   // second step (minimum)
    SFF_S3     = 3,   // third step (75 %)
    SFF_FULL   = 4,   // fully open for the rest of the burn
    SFF_GAVEUP = 5    // no trigger in time (or stand lost): open, sequence over
} sff_phase_t;

typedef struct {
    float    trigger_n;         // thrust that starts the sequence, N
    uint32_t debounce_us;       // ... and has to stay above this long
    uint32_t step_us;           // length of S1, S2 and S3
    uint32_t giveup_us;         // no trigger by this long after T0 -> stay open
    float    open_pct[4];       // S1, S2, S3, FULL: percent OPEN
} sff_cfg_t;

typedef struct {
    uint8_t  phase;
    uint8_t  above;
    uint32_t enter_us;          // micros() when the current phase began
    uint32_t above_us;          // ... when the thrust first stayed above the trigger
} sff_t;

static inline void sff_cfg_default(sff_cfg_t *c)
{
    c->trigger_n   = 100.0f;
    c->debounce_us = 10000u;
    c->step_us     = 300000u;
    c->giveup_us   = 8000000u;
    c->open_pct[0] = 50.0f;
    c->open_pct[1] = 10.0f;     // DG_CLAMP_MIN
    c->open_pct[2] = 75.0f;
    c->open_pct[3] = 100.0f;
}

static inline void sff_init(sff_t *s)
{
    s->phase = SFF_WAIT; s->above = 0; s->enter_us = 0; s->above_us = 0;
}

static inline void sff_enter(sff_t *s, uint8_t phase, uint32_t now_us)
{
    s->phase = phase;
    s->enter_us = now_us;
}

/** Link lost, or anything else that makes the profile meaningless. */
static inline void sff_failsafe(sff_t *s, uint32_t now_us)
{
    if (s->phase != SFF_GAVEUP && s->phase != SFF_FULL) sff_enter(s, SFF_GAVEUP, now_us);
}

/**
 * One step. `t_us` is the time since T0 (only used for the give-up), `now_us`
 * the local clock. Returns how far the clamp should be OPEN, in percent.
 */
static inline float sff_step(sff_t *s, const sff_cfg_t *c, uint32_t now_us, int32_t t_us,
                             float thrust_n)
{
    switch (s->phase) {
    case SFF_WAIT:
        if (thrust_n >= c->trigger_n) {
            if (!s->above) { s->above = 1; s->above_us = now_us; }
            if ((uint32_t)(now_us - s->above_us) >= c->debounce_us) {
                sff_enter(s, SFF_S1, now_us);
                return c->open_pct[0];
            }
        } else {
            s->above = 0;
        }
        if (t_us > 0 && (uint32_t)t_us >= c->giveup_us) sff_enter(s, SFF_GAVEUP, now_us);
        return 100.0f;
    case SFF_S1:
        if ((uint32_t)(now_us - s->enter_us) >= c->step_us) { sff_enter(s, SFF_S2, now_us); return c->open_pct[1]; }
        return c->open_pct[0];
    case SFF_S2:
        if ((uint32_t)(now_us - s->enter_us) >= c->step_us) { sff_enter(s, SFF_S3, now_us); return c->open_pct[2]; }
        return c->open_pct[1];
    case SFF_S3:
        if ((uint32_t)(now_us - s->enter_us) >= c->step_us) { sff_enter(s, SFF_FULL, now_us); return c->open_pct[3]; }
        return c->open_pct[2];
    case SFF_FULL:
        return c->open_pct[3];
    default:
        return 100.0f;
    }
}

/** Open percent -> servo pulse in microseconds. */
static inline float sff_pct_to_us(float open_pct, float us_closed, float us_open)
{
    if (open_pct < 0.0f) open_pct = 0.0f;
    if (open_pct > 100.0f) open_pct = 100.0f;
    return us_closed + (us_open - us_closed) * (open_pct / 100.0f);
}

/** Open percent -> closure in permille (0 = open, 1000 = shut). */
static inline uint16_t sff_closed_permille(float open_pct)
{
    if (open_pct < 0.0f) open_pct = 0.0f;
    if (open_pct > 100.0f) open_pct = 100.0f;
    return (uint16_t)((100.0f - open_pct) * 10.0f + 0.5f);
}

#endif // SF_FLAPS_H
