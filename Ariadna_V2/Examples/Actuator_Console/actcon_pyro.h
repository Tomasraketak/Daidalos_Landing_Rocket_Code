// =====================================================================
//  Ariadna V2 — Actuator Console: pyro bridge protocol and safety logic
//
//  Used by two EXAMPLE sketches, never by the flight firmware:
//
//      Examples/Actuator_Console   Master MCU (Pico 2 W): servos + USB GUI
//      Examples/Pyro_Companion     Navigation MCU (XIAO RP2350): pyros
//
//  The pyro outputs and their continuity inputs are wired to the Navigation
//  MCU, not to the Master (PINOUT.md, section 3). The flight protocol has
//  no "fire channel N now" command on purpose, so the console adds two
//  packet types of its own on the existing Master <-> Navigation UART,
//  framed exactly like every other Ariadna packet (ariadna_link.h).
//
//  Like ariadna_link.h this header has NO Arduino dependency, so the
//  safety state machine below is tested on a workstation:
//  `cd ../../test && make actcon`.
//
//  Each sketch carries its own copy (Arduino compiles only the sketch
//  folder); `make sync` checks the copies, `make sync-fix` refreshes them.
//
//  ---------------------------------------------------------------------
//  SAFETY MODEL — the Navigation MCU decides, the Master only asks
//  ---------------------------------------------------------------------
//   * Outputs are LOW at power-up and whenever the board is not firing.
//   * FIRE is refused unless the board is ARMED.
//   * ARMED expires on its own after ACTCON_ARM_TIMEOUT_MS.
//   * ARMED is dropped (and a pulse in progress is cut) when the Master's
//     heartbeat stops for ACTCON_HB_TIMEOUT_MS: an unplugged cable or a
//     crashed laptop leaves a safe board, not a live one.
//   * One channel at a time; a pulse is clamped to
//     ACTCON_PULSE_MIN_MS..ACTCON_PULSE_MAX_MS.
//   * Every FIRE ends in DISARMED. The next one needs a new ARM.
//   * A command is sent three times with one sequence number; a repeated
//     sequence number is ignored, so one click is at most one pulse.
//
//  The Master adds its own interlocks on top (RBF pulled, the GUI's
//  heartbeat alive), see Actuator_Console.ino.
// =====================================================================
#ifndef ACTCON_PYRO_H
#define ACTCON_PYRO_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
#define ACTCON_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define ACTCON_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

// Packet types. Outside the flight range (0x10..0x40) so a flight binary
// rejects them on the type check and never mistakes one for its own.
#define ACTCON_PKT_PYRO_CMD     0x70   // Master -> Navigation
#define ACTCON_PKT_PYRO_REPORT  0x71   // Navigation -> Master, 20 Hz

#define ACTCON_N_PYRO            3

#define ACTCON_ARM_TIMEOUT_MS    60000u  // armed state expires by itself
#define ACTCON_HB_TIMEOUT_MS       500u  // Master heartbeat loss -> disarm
#define ACTCON_HB_PERIOD_MS        100u  // Master sends a heartbeat this often
#define ACTCON_REPORT_PERIOD_MS     50u  // Navigation reports this often
#define ACTCON_PULSE_MIN_MS         20u
#define ACTCON_PULSE_MAX_MS       1000u
#define ACTCON_PULSE_DEFAULT_MS    250u

typedef enum {
    ACTCON_CMD_HEARTBEAT = 1,
    ACTCON_CMD_ARM       = 2,
    ACTCON_CMD_DISARM    = 3,
    ACTCON_CMD_FIRE      = 4
} actcon_cmd_id_t;

/** Why the last command was refused. Reported back so the GUI can say it. */
typedef enum {
    ACTCON_OK              = 0,
    ACTCON_REJ_NOT_ARMED   = 1,
    ACTCON_REJ_BAD_CHANNEL = 2,
    ACTCON_REJ_BUSY        = 3,   // another channel is firing
    ACTCON_REJ_NO_LINK     = 4,   // no heartbeat from the Master
    ACTCON_REJ_BAD_CMD     = 5
} actcon_reject_t;

/** Master -> Navigation. */
typedef struct {
    uint32_t seq;        // increments per user action, repeated x3 on the wire
    uint16_t pulse_ms;   // FIRE only
    uint8_t  cmd;        // actcon_cmd_id_t
    uint8_t  ch;         // FIRE only, 1..ACTCON_N_PYRO
} actcon_pyro_cmd_t;
ACTCON_STATIC_ASSERT(sizeof(actcon_pyro_cmd_t) == 8, "actcon cmd layout changed");

/** Navigation -> Master, every ACTCON_REPORT_PERIOD_MS. */
typedef struct {
    uint32_t t_ms;                    // Navigation MCU clock
    uint16_t raw[ACTCON_N_PYRO];      // continuity ADC counts, 12 bit
    uint16_t threshold;               // CONT_THRESHOLD of that board
    uint16_t arm_left_ds;             // armed time left, 0.1 s units
    uint16_t _pad16;
    uint8_t  cont_mask;               // bit n: channel n+1 has an igniter
    uint8_t  fired_mask;              // bit n: channel n+1 fired since boot
    uint8_t  firing_mask;             // bit n: channel n+1 is HIGH right now
    uint8_t  armed;                   // 1 = armed
    uint8_t  last_reject;             // actcon_reject_t of the last command
    uint8_t  active_high;             // continuity polarity of that board
    uint8_t  _pad[2];
} actcon_pyro_report_t;
ACTCON_STATIC_ASSERT(sizeof(actcon_pyro_report_t) == 24, "actcon report layout changed");

// =====================================================================
//  Navigation-side safety state machine
// =====================================================================
typedef struct {
    uint8_t  armed;
    uint8_t  link;          // a heartbeat seen within ACTCON_HB_TIMEOUT_MS
    int8_t   firing_ch;     // -1 idle, else 0-based channel
    uint8_t  fired_mask;
    uint8_t  last_reject;
    uint8_t  have_seq;
    uint32_t last_seq;
    uint32_t arm_until;
    uint32_t last_hb;
    uint32_t fire_until;
} actcon_pyro_sm_t;

/** True when `now` is at or after `t`, across the 49-day millis() wrap. */
static inline int actcon_reached(uint32_t now, uint32_t t)
{
    return (int32_t)(now - t) >= 0;
}

static inline void actcon_pyro_init(actcon_pyro_sm_t *sm)
{
    memset(sm, 0, sizeof(*sm));
    sm->firing_ch = -1;
}

static inline uint16_t actcon_clamp_pulse(uint16_t ms)
{
    if (ms == 0) return (uint16_t)ACTCON_PULSE_DEFAULT_MS;
    if (ms < ACTCON_PULSE_MIN_MS) return (uint16_t)ACTCON_PULSE_MIN_MS;
    if (ms > ACTCON_PULSE_MAX_MS) return (uint16_t)ACTCON_PULSE_MAX_MS;
    return ms;
}

static inline void actcon_pyro_disarm(actcon_pyro_sm_t *sm)
{
    sm->armed = 0;
    sm->firing_ch = -1;
}

/**
 * Apply one received command. Returns ACTCON_OK or the reason it was
 * refused; a repeated sequence number returns ACTCON_OK and does nothing.
 */
static inline uint8_t actcon_pyro_command(actcon_pyro_sm_t *sm,
                                          const actcon_pyro_cmd_t *c,
                                          uint32_t now)
{
    // Any frame from the Master proves the link is alive.
    sm->last_hb = now;
    sm->link    = 1;

    if (c->cmd == ACTCON_CMD_HEARTBEAT) return ACTCON_OK;

    if (sm->have_seq && c->seq == sm->last_seq) return ACTCON_OK;   // repeat
    sm->have_seq = 1;
    sm->last_seq = c->seq;

    uint8_t rej = ACTCON_OK;
    switch (c->cmd) {
    case ACTCON_CMD_ARM:
        if (sm->firing_ch >= 0) { rej = ACTCON_REJ_BUSY; break; }
        sm->armed     = 1;
        sm->arm_until = now + ACTCON_ARM_TIMEOUT_MS;
        break;
    case ACTCON_CMD_DISARM:
        actcon_pyro_disarm(sm);              // also cuts a pulse in progress
        break;
    case ACTCON_CMD_FIRE:
        if (!sm->armed)                                   rej = ACTCON_REJ_NOT_ARMED;
        else if (c->ch < 1 || c->ch > ACTCON_N_PYRO)      rej = ACTCON_REJ_BAD_CHANNEL;
        else if (sm->firing_ch >= 0)                      rej = ACTCON_REJ_BUSY;
        else {
            sm->firing_ch  = (int8_t)(c->ch - 1);
            sm->fire_until = now + actcon_clamp_pulse(c->pulse_ms);
            sm->fired_mask = (uint8_t)(sm->fired_mask | (1u << (c->ch - 1)));
        }
        break;
    default:
        rej = ACTCON_REJ_BAD_CMD;
        break;
    }
    sm->last_reject = rej;
    return rej;
}

/**
 * Advance timers. Call every loop. Returns the output mask: bit n set means
 * pyro channel n+1 must be driven HIGH right now; every other one LOW.
 */
static inline uint8_t actcon_pyro_tick(actcon_pyro_sm_t *sm, uint32_t now)
{
    if (sm->link && actcon_reached(now, sm->last_hb + ACTCON_HB_TIMEOUT_MS)) {
        sm->link = 0;
        if (sm->armed || sm->firing_ch >= 0) sm->last_reject = ACTCON_REJ_NO_LINK;
        actcon_pyro_disarm(sm);
    }
    if (sm->firing_ch >= 0 && actcon_reached(now, sm->fire_until)) {
        actcon_pyro_disarm(sm);              // every pulse ends disarmed
    }
    if (sm->armed && sm->firing_ch < 0 && actcon_reached(now, sm->arm_until)) {
        sm->armed = 0;
    }
    if (sm->firing_ch < 0) return 0;
    return (uint8_t)(1u << sm->firing_ch);
}

/** Armed time left in 0.1 s units, for the report. */
static inline uint16_t actcon_arm_left_ds(const actcon_pyro_sm_t *sm, uint32_t now)
{
    if (!sm->armed || actcon_reached(now, sm->arm_until)) return 0;
    return (uint16_t)((sm->arm_until - now + 99u) / 100u);
}

#endif // ACTCON_PYRO_H
