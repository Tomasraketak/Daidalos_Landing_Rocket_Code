// =====================================================================
//  Ariadna V2 — Static Fire Link: the run supervisor
//
//  Everything that decides WHEN TO FIRE, and what the clamp does, in one
//  place with no hardware in it, so it is tested on a workstation
//  (`cd test && make sflink`) and compiled unchanged into the sketch.
//
//  Inputs are the last good STATUS from the stand, the local prerequisites
//  and what the Navigation MCU reports; outputs are requests (arm / fire /
//  disarm), the clamp position, the heartbeat to send and whether to log.
//
//  RULES THAT KEEP AN IGNITER FROM FIRING WHEN IT SHOULD NOT
//    * Ariadna fires ONLY for a countdown it watched from the inside: it saw
//      the stand in COUNTDOWN with a given run number, armed the pyro, and
//      then saw IGNITE for that same run number. An IGNITE out of nowhere,
//      a stale one, or one for another run does nothing.
//    * The stand must be heard (a good STATUS within SFR_LOST_MS) the whole
//      time; so must everything local (PCA9685, flash, Navigation MCU, RBF
//      pulled, continuity). Lose any of it during the countdown and Ariadna
//      disarms and says "not ready", which makes the stand stop the countdown.
//    * An IGNITE that arrives later than fire_max_late_us after T0 is refused.
//    * One run, one pulse: the ack for the run is sent with the command and
//      never taken back, and the stand picks a new run number every time.
//    * The clamp's default and its fail-safe are the same: fully OPEN.
// =====================================================================
#ifndef SF_RUN_H
#define SF_RUN_H

#include <stdint.h>
#include <string.h>
#include "sf_link_proto.h"
#include "sf_flaps.h"

#define SFR_LOST_MS 200u       // no good STATUS for this long = stand lost

typedef enum {
    SFR_IDLE   = 0,   // waiting; clamp open, nothing armed
    SFR_ARMING = 1,   // saw a countdown, asked the Navigation MCU to arm
    SFR_ARMED  = 2,   // pyro armed, waiting for T0
    SFR_BURN   = 3,   // fired; running the clamp profile and logging
    SFR_DONE   = 4    // burn over, waiting for the stand to go idle
} sfr_phase_t;

// event bits, one tick can raise several
#define SFR_EV_COUNTDOWN   0x0001u   // a countdown was seen, arming
#define SFR_EV_ARMED       0x0002u   // the Navigation MCU says armed
#define SFR_EV_ABORT       0x0004u   // countdown ended without ignition
#define SFR_EV_ARM_TIMEOUT 0x0008u   // the pyro did not arm
#define SFR_EV_FIRE        0x0010u   // FIRE sent
#define SFR_EV_LATE        0x0020u   // IGNITE too late (or not armed): refused
#define SFR_EV_TRIGGER     0x0040u   // thrust crossed the trigger
#define SFR_EV_STEP        0x0080u   // clamp profile moved on
#define SFR_EV_STAND_LOST  0x0100u   // lost the stand during the burn: clamp open
#define SFR_EV_FINISHED    0x0200u   // burn over

typedef struct {
    uint16_t ign_ms;             // pyro pulse; the stand's setting, clamped
    uint32_t fire_max_late_us;   // refuse an IGNITE older than this
    uint32_t arm_timeout_ms;     // the Navigation MCU must report armed in this time
    uint32_t log_pre_us;         // start logging this long before T0
    uint32_t recording_ms;       // the stand's recording window (INFO)
    sff_cfg_t flaps;
} sfr_cfg_t;

typedef struct {
    uint32_t now_ms, now_us;
    int      stand_ok;           // a good STATUS no older than SFR_LOST_MS
    sfl_status_t st;             // the last good one
    int      st_new;             // 1 on the tick a new STATUS arrived
    uint32_t st_us;              // micros() at the middle of the read that produced it
    int      prereq_ok;          // PCA9685, flash slot, Navigation MCU, RBF, continuity
    int      nav_armed;          // the Navigation MCU's report
    int      cont;               // igniter continuity on pyro 1
    int      busy;               // copying / writing flash
    int      hold;               // the operator said "not ready"
} sfr_in_t;

typedef struct {
    int      want_arm;           // level: keep asking the Navigation MCU to arm
    int      want_disarm;        // one-shot
    int      fire;               // one-shot: send FIRE, pyro 1
    uint16_t fire_ms;
    float    flap_open_pct;      // where the clamp should be
    int      log_on;
    int32_t  t_us;               // our timeline, 0 = T0 (valid if t_valid)
    int      t_valid;
    uint8_t  flap_phase;
    uint8_t  hb_flags, hb_ack;   // the heartbeat to send
    uint16_t ev;                 // SFR_EV_*
} sfr_out_t;

typedef struct {
    uint8_t  phase;
    uint8_t  run_seq;            // the run being handled
    uint8_t  ack_seq;            // level sent in the heartbeat
    uint8_t  fault;              // sticky until the stand is idle again
    uint8_t  fired;
    uint8_t  lost;               // stand lost during the burn
    uint8_t  t0_valid;
    uint8_t  last_flap_phase;
    uint32_t t0_local_us;        // micros() of T0, from the stand's own clock
    uint32_t phase_since_ms;
    uint32_t lost_since_ms;
    int32_t  t_fire_us;          // for the record, SFL_T_NONE until fired
    int32_t  t_trig_us;
    int32_t  t_phase_us[4];      // when S1, S2, S3, FULL began
    sff_t    flaps;
} sfr_t;

static inline void sfr_cfg_default(sfr_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->ign_ms           = 300;
    c->fire_max_late_us = 150000u;
    c->arm_timeout_ms   = 1500;
    c->log_pre_us       = 3500000u;
    c->recording_ms     = 10000;
    sff_cfg_default(&c->flaps);
}

static inline void sfr_init(sfr_t *r)
{
    memset(r, 0, sizeof(*r));
    r->phase = SFR_IDLE;
    r->t_fire_us = r->t_trig_us = SFL_T_NONE;
    for (int i = 0; i < 4; i++) r->t_phase_us[i] = SFL_T_NONE;
    sff_init(&r->flaps);
}

static inline void sfr_enter(sfr_t *r, uint8_t phase, uint32_t now_ms)
{
    r->phase = phase;
    r->phase_since_ms = now_ms;
}

/** Back to idle; also forgets the run so a new one starts clean. */
static inline void sfr_to_idle(sfr_t *r, uint32_t now_ms)
{
    sfr_enter(r, SFR_IDLE, now_ms);
    r->fired = 0; r->lost = 0; r->t0_valid = 0;
    r->t_fire_us = r->t_trig_us = SFL_T_NONE;
    for (int i = 0; i < 4; i++) r->t_phase_us[i] = SFL_T_NONE;
    sff_init(&r->flaps);
    r->last_flap_phase = SFF_WAIT;
}

static inline uint16_t sfr_clamp_pulse_ms(uint16_t ms)
{
    return ms < 20u ? 20u : (ms > 1000u ? 1000u : ms);   // the Navigation MCU's own window
}

/** How often the stand should be polled, ms. */
static inline uint32_t sfr_poll_ms(const sfr_t *r, const sfr_in_t *in)
{
    const int32_t t_us = (int32_t)(in->now_us - r->t0_local_us);
    switch (r->phase) {
    case SFR_BURN:   return 5;
    case SFR_ARMING:
    case SFR_ARMED:  return (r->t0_valid && t_us > -1500000) ? 5 : 20;
    case SFR_DONE:   return 20;
    default:
        return (in->stand_ok && in->st.state == SFL_ST_COUNTDOWN) ? 20 : 100;
    }
}

static inline void sfr_tick(sfr_t *r, const sfr_cfg_t *c, const sfr_in_t *in, sfr_out_t *o)
{
    memset(o, 0, sizeof(*o));
    o->flap_open_pct = 100.0f;
    o->t_us = SFL_T_NONE;
    const sfl_status_t *st = &in->st;
    const int ok = in->stand_ok;

    // ---- the stand's timeline, taken from its own clock ----------------
    // Every good STATUS says how long ago / how long until T0 it was, at the
    // moment the stand answered, so T0 on OUR clock is st_us - t_rel.
    if (ok && in->st_new && st->t_rel_us != SFL_T_NONE) {
        r->t0_local_us = in->st_us - (uint32_t)st->t_rel_us;
        r->t0_valid = 1;
    }
    const int32_t t_us = (int32_t)(in->now_us - r->t0_local_us);
    if (r->t0_valid) { o->t_us = t_us; o->t_valid = 1; }

    // ---- readiness: what we tell the stand --------------------------------
    const int stand_fine = ok && (st->flags & SFL_F_HX_OK) && (st->flags & SFL_F_LOG_OK);
    const int ready = stand_fine && in->prereq_ok && !in->hold && !r->fault;

    switch (r->phase) {
    // ---------------------------------------------------------------------
    case SFR_IDLE:
        if (r->fault && ok && st->state == SFL_ST_IDLE) r->fault = 0;
        if (ready && st->state == SFL_ST_COUNTDOWN) {
            sfr_to_idle(r, in->now_ms);                 // a clean slate for this run
            r->run_seq = st->run_seq;
            sfr_enter(r, SFR_ARMING, in->now_ms);
            o->ev |= SFR_EV_COUNTDOWN;
            o->want_arm = 1;
        }
        break;

    // ---------------------------------------------------------------------
    case SFR_ARMING:
    case SFR_ARMED:
        // 1. Ignition. Only for the run we watched, only if armed, only on time.
        if (r->phase == SFR_ARMED && ok && st->state == SFL_ST_RECORDING &&
            (st->flags & SFL_F_IGNITE) && st->run_seq == r->run_seq) {
            const uint32_t late_us = (uint32_t)st->t_rel_us + (in->now_us - in->st_us);
            if (in->nav_armed && in->prereq_ok && !in->hold && st->t_rel_us >= 0 &&
                late_us <= c->fire_max_late_us) {
                o->fire = 1;
                o->fire_ms = sfr_clamp_pulse_ms(c->ign_ms);
                r->fired = 1;
                r->ack_seq = r->run_seq;               // the ack goes out with the command
                r->t_fire_us = t_us;
                sff_init(&r->flaps);
                r->last_flap_phase = SFF_WAIT;
                sfr_enter(r, SFR_BURN, in->now_ms);
                o->ev |= SFR_EV_FIRE;
            } else {
                o->ev |= SFR_EV_LATE;
                o->want_disarm = 1;
                r->fault = 1;
                sfr_to_idle(r, in->now_ms);
            }
            break;
        }
        // 2. The countdown ended some other way, or something we need went away.
        //    One exception: the stand publishes RECORDING a hair before it raises
        //    IGNITE, so for the first few ms of T0 "recording without ignite" is
        //    waited out rather than taken for an abort. If IGNITE never comes
        //    (the stand's own RBF check failed at T0) it is an abort after all.
        {
            const int t0_grace = (r->phase == SFR_ARMED) && ok && st->state == SFL_ST_RECORDING &&
                                 st->run_seq == r->run_seq && st->t_rel_us >= 0 &&
                                 st->t_rel_us < 30000 && !(st->flags & SFL_F_IGNITE);
            if (t0_grace && in->prereq_ok && !in->hold) break;
        }
        if (!ok || st->state != SFL_ST_COUNTDOWN || st->run_seq != r->run_seq ||
            !in->prereq_ok || in->hold) {
            o->ev |= SFR_EV_ABORT;
            o->want_disarm = 1;
            sfr_to_idle(r, in->now_ms);
            break;
        }
        // 3. Arming.
        if (r->phase == SFR_ARMING) {
            if (in->nav_armed) {
                sfr_enter(r, SFR_ARMED, in->now_ms);
                o->ev |= SFR_EV_ARMED;
            } else if ((uint32_t)(in->now_ms - r->phase_since_ms) >= c->arm_timeout_ms) {
                o->ev |= SFR_EV_ARM_TIMEOUT;
                o->want_disarm = 1;
                r->fault = 1;                       // not ready -> the stand stops the countdown
                sfr_to_idle(r, in->now_ms);
            } else {
                o->want_arm = 1;
            }
        }
        break;

    // ---------------------------------------------------------------------
    case SFR_BURN: {
        float thrust_n = 0.0f;
        if (ok) {
            thrust_n = (float)st->thrust_mN * 0.001f;
            if (r->lost) r->lost = 0;
        } else if (!r->lost) {
            r->lost = 1;
            r->lost_since_ms = in->now_ms;
            sff_failsafe(&r->flaps, in->now_us);     // no thrust reading: open, and stay open
            o->ev |= SFR_EV_STAND_LOST;
        }
        o->flap_open_pct = sff_step(&r->flaps, &c->flaps, in->now_us, t_us, thrust_n);
        o->flap_phase = r->flaps.phase;

        if (r->flaps.phase != r->last_flap_phase) {
            if (r->flaps.phase >= SFF_S1 && r->flaps.phase <= SFF_FULL) {
                r->t_phase_us[r->flaps.phase - 1] = t_us;
                if (r->flaps.phase == SFF_S1) { r->t_trig_us = t_us; o->ev |= SFR_EV_TRIGGER; }
                else                           o->ev |= SFR_EV_STEP;
            }
            r->last_flap_phase = r->flaps.phase;
        }
        o->log_on = 1;

        const int over  = ok && st->state != SFL_ST_RECORDING;
        const int stale = r->lost && (uint32_t)(in->now_ms - r->lost_since_ms) > 3000u;
        const int late  = t_us > (int32_t)((c->recording_ms + 3000u) * 1000u);
        if (over || stale || late) {
            o->ev |= SFR_EV_FINISHED;
            o->want_disarm = 1;
            o->flap_open_pct = 100.0f;
            o->log_on = 0;
            sfr_enter(r, SFR_DONE, in->now_ms);
        }
        break;
    }

    // ---------------------------------------------------------------------
    case SFR_DONE:
        if ((ok && st->state == SFL_ST_IDLE) || (uint32_t)(in->now_ms - r->phase_since_ms) > 5000u) {
            // t_fire / t_trig / t_phase stay readable until the caller has
            // stored them; sfr_to_idle() is the caller's job after that.
            r->phase = SFR_IDLE;
            r->phase_since_ms = in->now_ms;
        }
        break;
    }

    // ---- logging window ---------------------------------------------------
    if (r->phase == SFR_ARMING || r->phase == SFR_ARMED)
        o->log_on = r->t0_valid && t_us >= -(int32_t)c->log_pre_us;
    o->flap_phase = r->flaps.phase;

    // ---- the heartbeat ----------------------------------------------------
    // (recomputed: a fault raised above must show in THIS heartbeat, not the next)
    const int ready_now = stand_fine && in->prereq_ok && !in->hold && !r->fault;
    uint8_t f = 0;
    if (ready_now)                     f |= SFL_A_READY;
    if (in->nav_armed)                 f |= SFL_A_ARMED;
    if (r->fired)                      f |= SFL_A_FIRED;
    if (in->cont)                      f |= SFL_A_CONT;
    if (in->busy)                      f |= SFL_A_BUSY;
    if (r->fault)                      f |= SFL_A_FAULT;
    o->hb_flags = f;
    o->hb_ack   = r->ack_seq;
}

#endif // SF_RUN_H
