// =====================================================================
//  Ariadna V2 — flight event sequencer
//
//  The one-shot events of the landing, and when each one happens:
//
//    AEROCOVER    unlatched  seq.cover_lead_s  (0.6 s) BEFORE the ignition
//                 command. A servo on the Master's PCA9685; this module
//                 only raises the request.
//    PYRO 1       main igniter, at the ignition command.
//    PYRO 2       backup igniter, fired only if the flight computer has
//                 NOT felt the motor light - no bump in axial acceleration
//                 at the command or within seq.backup_win_s (150 ms) of it.
//    PYRO 3       burns the band that holds the landing legs, fired
//                 seq.legs_lead_s (1.7 s) before the PLANNED touchdown and
//                 held for at least seq.legs_pulse_s (0.5 s).
//
//  The guidance module supplies the two countdowns (time to the ignition
//  command, time to touchdown), the ignition command itself and "thrust
//  detected". This module turns them into output levels. It owns no pins
//  and no clock, so the rules are tested on a workstation
//  (test/seq_test.cpp) rather than with an igniter on the bench.
//
//  SAFE BY CONSTRUCTION
//    * Every output is a level that is false unless a rule holds it true.
//    * `permit` false (the vehicle is not armed) drops every pyro level at
//      once and cancels the pulse; it does not resume when permit returns.
//    * Each channel fires at most ONCE per flight.
//    * The leg release is refused above seq.legs_h_max_m, so a countdown
//      that is wrong by ten seconds cannot drop the legs at apogee.
// =====================================================================
#ifndef ARIADNA_SEQ_H
#define ARIADNA_SEQ_H

#include <stdint.h>
#include <string.h>

// event bits: reported in nav_status.events and on the downlink
#define ARIADNA_EV_APOGEE     (1u << 0)   // descent phase entered
#define ARIADNA_EV_COVER      (1u << 1)   // aerocover unlatched
#define ARIADNA_EV_IGN1       (1u << 2)   // main igniter fired
#define ARIADNA_EV_THRUST     (1u << 3)   // thrust onset felt
#define ARIADNA_EV_IGN2       (1u << 4)   // backup igniter fired
#define ARIADNA_EV_LEGS       (1u << 5)   // leg release fired
#define ARIADNA_EV_LIDAR      (1u << 6)   // altitude is lidar referenced
#define ARIADNA_EV_TOUCHDOWN  (1u << 7)

// phases, numerically equal to dg_phase_t (checked in the test)
#define ARIADNA_SEQ_PH_DESCENT   2
#define ARIADNA_SEQ_PH_IGNITING  3
#define ARIADNA_SEQ_PH_BURN      4
#define ARIADNA_SEQ_PH_DONE      5

typedef struct {
    float cover_lead_s;
    float backup_win_s;
    float legs_lead_s;
    float legs_pulse_s;
    float ign_pulse_s;
    float legs_h_max_m;
    uint8_t cover_en, backup_en, legs_en;
} ariadna_seq_cfg_t;

typedef struct {
    uint8_t  phase;            // dg_phase_t
    uint8_t  ignite;           // level from the guidance
    uint8_t  thrust_on;        // level: onset DETECTED
    uint8_t  permit;           // pyros permitted (armed, not test, not abort)
    float    t_to_ignition;    // s, < 0 unknown
    float    t_to_touchdown;   // s, < 0 unknown
    float    h;                // m above the pad
} ariadna_seq_in_t;

typedef struct {
    // outputs
    uint8_t  cover_open;       // latched request to the Master
    uint8_t  pyro[3];          // output levels, channel 1..3
    uint8_t  events;           // ARIADNA_EV_* seen so far
    // internal
    uint8_t  fired[3];
    uint8_t  backup_pending;
    uint32_t t_on_ms[3];
    uint32_t t_ignite_ms;
} ariadna_seq_t;

static inline void ariadna_seq_init(ariadna_seq_t *s)
{
    memset(s, 0, sizeof(*s));
}

static inline void ariadna_seq__fire(ariadna_seq_t *s, uint8_t ch, uint32_t now_ms,
                                     uint8_t event)
{
    if (s->fired[ch]) return;
    s->fired[ch]  = 1;
    s->pyro[ch]   = 1;
    s->t_on_ms[ch] = now_ms;
    s->events    |= event;
}

/**
 * One step. Call it every control tick, after the guidance step.
 * `now_ms` is any monotonic millisecond clock.
 */
static inline void ariadna_seq_step(ariadna_seq_t *s, const ariadna_seq_cfg_t *c,
                                    const ariadna_seq_in_t *in, uint32_t now_ms)
{
    const uint8_t flying = (in->phase >= ARIADNA_SEQ_PH_DESCENT &&
                            in->phase <= ARIADNA_SEQ_PH_BURN);

    if (in->phase >= ARIADNA_SEQ_PH_DESCENT) s->events |= ARIADNA_EV_APOGEE;
    if (in->phase == ARIADNA_SEQ_PH_DONE)    s->events |= ARIADNA_EV_TOUCHDOWN;

    // ---- aerocover: a servo, so it is not behind the pyro permit ----
    if (c->cover_en && !s->cover_open) {
        const uint8_t due = (in->phase == ARIADNA_SEQ_PH_DESCENT &&
                             in->t_to_ignition >= 0.0f &&
                             in->t_to_ignition <= c->cover_lead_s);
        // The command arriving with the cover still latched means the
        // countdown was never valid. Late is better than never.
        if (due || (flying && in->ignite)) {
            s->cover_open = 1;
            s->events |= ARIADNA_EV_COVER;
        }
    }

    // ---- withdrawn arming: everything off, pulses cancelled ----
    if (!in->permit) {
        s->pyro[0] = s->pyro[1] = s->pyro[2] = 0;
        s->backup_pending = 0;
        return;
    }

    // ---- main igniter, at the command ----
    if (flying && in->ignite && !s->fired[0]) {
        ariadna_seq__fire(s, 0, now_ms, ARIADNA_EV_IGN1);
        s->t_ignite_ms    = now_ms;
        s->backup_pending = c->backup_en ? 1 : 0;
    }

    // ---- thrust felt? then the backup is not needed ----
    if (in->thrust_on) {
        s->events |= ARIADNA_EV_THRUST;
        s->backup_pending = 0;
    }

    // ---- backup igniter: no bump at the command or within the window ----
    if (s->backup_pending &&
        (uint32_t)(now_ms - s->t_ignite_ms) >= (uint32_t)(c->backup_win_s * 1000.0f)) {
        s->backup_pending = 0;
        ariadna_seq__fire(s, 1, now_ms, ARIADNA_EV_IGN2);
    }

    // ---- leg release ----
    if (c->legs_en && !s->fired[2] && flying && in->h <= c->legs_h_max_m) {
        const uint8_t due = (in->t_to_touchdown >= 0.0f &&
                             in->t_to_touchdown <= c->legs_lead_s);
        // No valid countdown during the burn and the ground is close:
        // the legs go anyway.
        const uint8_t late = (in->phase == ARIADNA_SEQ_PH_BURN &&
                              in->t_to_touchdown < 0.0f && in->h < 10.0f);
        if (due || late) ariadna_seq__fire(s, 2, now_ms, ARIADNA_EV_LEGS);
    }

    // ---- end the pulses on time ----
    const float legs_s = c->legs_pulse_s < 0.5f ? 0.5f : c->legs_pulse_s;   // never shorter
    const uint32_t dur_ms[3] = { (uint32_t)(c->ign_pulse_s * 1000.0f),
                                 (uint32_t)(c->ign_pulse_s * 1000.0f),
                                 (uint32_t)(legs_s * 1000.0f) };
    for (uint8_t i = 0; i < 3; i++)
        if (s->pyro[i] && (uint32_t)(now_ms - s->t_on_ms[i]) >= dur_ms[i]) s->pyro[i] = 0;
}

#endif // ARIADNA_SEQ_H
