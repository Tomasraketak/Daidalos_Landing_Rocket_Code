// =====================================================================
//  Ariadna V2 — NAVIGATION MCU
//
//  Target: Seeed XIAO RP2350, arduino-pico core (earlephilhower).
//  Needs:  Adafruit_NeoPixel
//
//  Role: flight path prediction and active control. This board decides
//  WHEN the landing motor lights, WHERE the nozzle and the fins point, HOW
//  MUCH thrust is kept, and when the three pyro channels fire. It is the
//  only board in the vehicle that can fire a pyro.
//
//  It owns no sensors. Everything it knows arrives as ariadna_nav_state_t
//  from the Position MCU, routed through the Master.
//
//  ---------------------------------------------------------------------
//  WHAT RUNS HERE
//  ---------------------------------------------------------------------
//    guidance.cpp     the LandSim_Light flight algorithm: guidance law,
//                     nozzle and fin attitude loops, fin adaptation, clamp
//                     planner, ignition altitude. Verified on a workstation
//                     against the simulation: `cd ../../test && make guid`.
//    ariadna_seq.h    the one-shot events: aerocover, main igniter, backup
//                     igniter, leg release. `make seq`.
//    this file        pins, the link, the failsafes, and nothing clever.
//
//  ---------------------------------------------------------------------
//  THE THREE PYRO CHANNELS
//  ---------------------------------------------------------------------
//    PYRO 1  D3   main igniter of the landing motor
//    PYRO 2  D8   backup igniter of the same motor. Fired only if no bump
//                 in axial acceleration is felt at the ignition command or
//                 within seq.backup_win_s (150 ms) of it.
//    PYRO 3  D9   burns the band that holds the landing legs, 1.7 s before
//                 the planned touchdown, held for at least 0.5 s.
//
//  The aerocover is NOT a pyro: it is a latch servo on the Master's
//  PCA9685. This board asks for it 0.6 s before the ignition command with
//  ARIADNA_ACT_COVER_OPEN in the actuator packet.
//
//  ---------------------------------------------------------------------
//  CORE ALLOCATION
//  ---------------------------------------------------------------------
//    core 0   200 Hz: link, guidance step, sequencer, pyros, actuator TX
//    core 1   the clamp planner (a forward simulation, 10 Hz)
//
//  Shared through a seqlock, never a mutex: the 200 Hz loop must not block
//  on the planner.
//
//  ---------------------------------------------------------------------
//  READ BEFORE CONNECTING AN IGNITER
//  ---------------------------------------------------------------------
//  This is unflown software. Bench it with PIN_PYRO_* on a LED. Every pin
//  number, threshold and polarity below describes an INTENDED vehicle and
//  must be checked against yours. In TEST mode (diagnostic GUI) the
//  channels can be fired by hand; the rules for that are in actcon_pyro.h.
// =====================================================================

#include "guidance.h"
#include "ariadna_link.h"
#include "ariadna_uart_dma.h"
#include "ariadna_params.h"
#include "ariadna_seq.h"
#include "actcon_pyro.h"
#include "ariadna_rampa_link.h"
// Named here, not only inside ariadna_rgb.h: the Arduino builder adds a
// library's include path only when it sees a real #include of it.
// ARDUINO is undefined in the host `make syntax` build, which has no
// libraries.
#ifdef ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
#include "ariadna_rgb.h"

#include <Arduino.h>
#include <Wire.h>
#include "hardware/watchdog.h"
#include <string.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE CONFIGURATION  (PINOUT.md, section 3)
// =====================================================================
static const uint8_t PIN_PYRO_1  = D3;   // landing motor igniter
static const uint8_t PIN_PYRO_2  = D8;   // landing motor igniter, backup
static const uint8_t PIN_PYRO_3  = D9;   // leg release (burn wire)

static const uint8_t PIN_CONT_1  = D0;   // A0, continuity sense, channel 1
static const uint8_t PIN_CONT_2  = D1;   // A1
static const uint8_t PIN_CONT_3  = D2;   // A2

// Dedicated Rampa umbilical I2C port (I2CN uSup: D4=GP6 SDA, D5=GP7 SCL -> Wire1).
// Completely isolated from Master MCU's PCA9685 servo bus (I2C0) so pogo
// hot-separation can never disturb servos. Detached (Wire1.end + Hi-Z) at T-100 ms.
static const uint8_t PIN_UMB_SDA = D4;
static const uint8_t PIN_UMB_SCL = D5;

// D6/D7. If this port stays silent on the bench, change this one line to
// Serial2 before touching anything else.
#define MASTER_PORT   Serial1
static const uint8_t PIN_MASTER_TX = D6;
static const uint8_t PIN_MASTER_RX = D7;

// ---- continuity detection -------------------------------------------
// Both numbers are guesses about YOUR hardware. Measure the ADC counts
// with a known-good igniter and with an open channel (the diagnostic GUI
// shows them live) and put the midpoint here.
#define CONT_ACTIVE_HIGH  1
static const uint16_t CONT_THRESHOLD = 1200;   // of 4095, at 12-bit
static const uint8_t  CONT_SAMPLES   = 4;

// ---- link supervision ------------------------------------------------
// A short gap is dead reckoned, a long one invalidates the state, which
// parks the nozzle and the fins and FREEZES the clamp.
static const uint32_t NAV_STALE_MS = 60;
static const uint32_t NAV_HOLD_MS  = 250;

// ---- options ---------------------------------------------------------
#define NAV_USE_CORE1   1        // 0 = run the planner inline on core 0
#define PAD_CONSOLE_HZ  2        // USB CDC status rate, only with a host attached

// The body axis TVC servo 1 deflects the nozzle in, and the radial
// direction of fin 1. A HARDWARE fact about the airframe; it must match
// what srv.tvc_x_sign drives at the Master.
static const float BODY_SERVO1_AXIS[3] = { 1.0f, 0.0f, 0.0f };

#define NAV_PRINT(...)                                            \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        if (Serial && Serial.availableForWrite() > 100)           \
            Serial.printf(__VA_ARGS__);                           \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

static void handle_command(const ariadna_cmd_t *c);
static void umb_i2c_detach(void);
static void umb_handle_rampa_cmd(const ariadna_rampa_cmd_t *c);

// =====================================================================
//  2. STATE
// =====================================================================
static ariadna_rx_t           s_rx;
static ariadna_nav_state_t    s_state;
static ariadna_master_state_t s_master;
static uint32_t               s_state_us    = 0;
static bool                   s_have_state  = false;
static uint32_t               s_master_us   = 0;
static bool                   s_have_master = false;

static dg_nav_t  s_dg_nav;
static dg_out_t  s_dg_out;
static bool      s_post_pass = false;

static ariadna_params_t  s_params;
static uint8_t           s_param_seen[(ARIADNA_PARAM_ID_MAX + 7) / 8];
static uint8_t           s_params_seen_n = 0;
static ariadna_seq_cfg_t s_seq_cfg;
static ariadna_seq_t     s_seq;

// apogee tracking
static float     s_h_max      = 0.0f;
static float     s_vh_apogee  = 0.0f;

// pyro
static uint8_t   s_cont_mask  = 0;
static uint16_t  s_cont_raw[3] = { 0, 0, 0 };
static uint8_t   s_fired_mask = 0;
static uint8_t   s_out_mask   = 0;          // channels HIGH right now

// TEST mode (bench firing from the diagnostic GUI)
static actcon_pyro_sm_t s_test_sm;

// housekeeping
static uint32_t  s_loop_max_us = 0;
static uint32_t  s_overruns    = 0;
static volatile uint32_t s_plan_calls = 0;

// =====================================================================
//  3. CROSS-CORE HANDOVER (seqlock)
// =====================================================================
static volatile uint32_t s_seq_lock = 0;
static dg_nav_t          s_shared;
static volatile bool     s_ready = false;

static inline void nav_publish(const dg_nav_t *n)
{
    s_seq_lock++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_shared = *n;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_seq_lock++;
}

static inline bool nav_acquire(dg_nav_t *n)
{
    uint32_t a = s_seq_lock;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *n = s_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_seq_lock == a;
}

// =====================================================================
//  4. PARAMETERS
//
//  The Master streams the table while the vehicle is on the ground. This
//  board copies what it needs into the guidance module and the sequencer.
//  Nothing is accepted once the Master reports a flight state.
// =====================================================================
static void params_apply(void)
{
    const float *v = s_params.v;

    dg_params_t g;
    dg_params_default(&g);
    g.mass_gross     = v[ARIADNA_P_G_MASS_GROSS];
    g.mass_prop      = v[ARIADNA_P_G_MASS_PROP];
    g.wn_tvc         = v[ARIADNA_P_G_WN_TVC];
    g.zeta_tvc       = v[ARIADNA_P_G_ZETA_TVC];
    g.sched_tvc      = v[ARIADNA_P_G_SCHED_TVC];
    g.att_ki         = v[ARIADNA_P_G_ATT_KI];
    g.att_i_lim      = v[ARIADNA_P_G_ATT_I_LIM];
    g.wn_fin         = v[ARIADNA_P_G_WN_FIN];
    g.zeta_fin       = v[ARIADNA_P_G_ZETA_FIN];
    g.sched_fin      = v[ARIADNA_P_G_SCHED_FIN];
    g.roll_gain      = v[ARIADNA_P_G_ROLL_GAIN];
    g.roll_max_deg   = v[ARIADNA_P_G_ROLL_MAX];
    g.tilt_min_deg   = v[ARIADNA_P_G_TILT_MIN];
    g.tilt_slope     = v[ARIADNA_P_G_TILT_SLOPE];
    g.tilt_cap_deg   = v[ARIADNA_P_G_TILT_CAP];
    g.alt_pad        = v[ARIADNA_P_G_ALT_PAD];
    g.h_ign_bias     = v[ARIADNA_P_G_H_IGN_BIAS];
    g.h_ign_override = v[ARIADNA_P_G_H_IGN_OVERRIDE];
    g.plan_target    = v[ARIADNA_P_G_PLAN_TARGET];
    g.plan_margin    = v[ARIADNA_P_G_PLAN_MARGIN];
    g.plan_hold      = v[ARIADNA_P_G_PLAN_HOLD];
    g.onset_bump     = v[ARIADNA_P_G_ONSET_BUMP];
    g.onset_timeout  = v[ARIADNA_P_G_ONSET_TIMEOUT];
    g.onset_assumed  = v[ARIADNA_P_G_ONSET_ASSUMED];
    g.fin_enable     = v[ARIADNA_P_G_FIN_ENABLE];
    g.fin_max_deg    = v[ARIADNA_P_G_FIN_MAX];
    g.fin_brake      = v[ARIADNA_P_G_FIN_BRAKE];
    g.fin_min_airspeed  = v[ARIADNA_P_G_FIN_MIN_V];
    g.fin_drift_gain    = v[ARIADNA_P_G_FIN_DRIFT_GAIN];
    g.fin_drift_max_deg = v[ARIADNA_P_G_FIN_DRIFT_MAX];
    g.fin_ctrl_lead  = v[ARIADNA_P_G_FIN_CTRL_LEAD];
    g.fin_ascent_mode = v[ARIADNA_P_G_FIN_ASCENT];
    g.fin_adapt      = v[ARIADNA_P_G_FIN_ADAPT];
    g.fin_gain0      = v[ARIADNA_P_G_FIN_GAIN0];
    g.cd_body        = v[ARIADNA_P_G_CD_BODY];
    g.thrust_scale   = v[ARIADNA_P_G_THRUST_SCALE];
    g.ign_delay_pad  = v[ARIADNA_P_G_IGN_DELAY_PAD];
    g.drag_adapt     = v[ARIADNA_P_G_DRAG_ADAPT];
    g.plan_ts_tau    = v[ARIADNA_P_G_PLAN_TS_TAU];
    g.fin_dcd_zero   = v[ARIADNA_P_G_FIN_DCD_ZERO];
    g.fin_dcd_brake  = v[ARIADNA_P_G_FIN_DCD_BRAKE];
    g.diameter       = v[ARIADNA_P_G_DIAMETER];
    // the ignition band table, as the diagnostic GUI computed it with LandSim
    g.tbl_vz0        = v[ARIADNA_P_TBL_VZ0];
    g.tbl_dvz        = v[ARIADNA_P_TBL_DVZ];
    for (int i = 0; i < DG_IGN_N; i++) {
        g.tbl_hmin[i] = v[ARIADNA_P_TBL_HMIN_00 + i];
        g.tbl_hmax[i] = v[ARIADNA_P_TBL_HMAX_00 + i];
    }
    dg_set_params(&g);

    s_seq_cfg.cover_lead_s = v[ARIADNA_P_SEQ_COVER_LEAD];
    s_seq_cfg.backup_win_s = v[ARIADNA_P_SEQ_BACKUP_WIN];
    s_seq_cfg.legs_lead_s  = v[ARIADNA_P_SEQ_LEGS_LEAD];
    s_seq_cfg.legs_pulse_s = v[ARIADNA_P_SEQ_LEGS_PULSE];
    s_seq_cfg.ign_pulse_s  = v[ARIADNA_P_SEQ_IGN_PULSE];
    s_seq_cfg.legs_h_max_m = v[ARIADNA_P_SEQ_LEGS_H_MAX];
    s_seq_cfg.cover_en     = v[ARIADNA_P_SEQ_COVER_EN]  > 0.5f;
    s_seq_cfg.backup_en    = v[ARIADNA_P_SEQ_BACKUP_EN] > 0.5f;
    s_seq_cfg.legs_en      = v[ARIADNA_P_SEQ_LEGS_EN]   > 0.5f;
}

static inline bool on_ground(void)
{
    // Unknown counts as "on the ground" only before the first master
    // state: a board that has just booted on the pad must take the table.
    if (!s_have_master) return true;
    return s_master.state == ARIADNA_STATE_BOOT  ||
           s_master.state == ARIADNA_STATE_IDLE  ||
           s_master.state == ARIADNA_STATE_ARMED ||
           s_master.state == ARIADNA_STATE_TEST  ||
           s_master.state == ARIADNA_STATE_ERROR;
}

static void param_receive(const ariadna_param_pkt_t *p)
{
    if (!on_ground()) return;                         // never in flight
    if (p->count != ARIADNA_PARAM_COUNT) return;      // a different table layout
    if (s_dg_out.phase != DG_PHASE_IDLE) return;
    if (p->id >= ARIADNA_PARAM_ID_MAX) return;
    const float before = s_params.v[p->id];
    if (!ariadna_param_set(&s_params, p->id, p->value)) return;
    uint8_t bit = (uint8_t)(1u << (p->id & 7));
    if (!(s_param_seen[p->id >> 3] & bit)) {
        s_param_seen[p->id >> 3] |= bit;
        s_params_seen_n++;
    }
    if (s_params.v[p->id] != before) params_apply();
}

// =====================================================================
//  5. LINK
// =====================================================================
static bool state_sane(const ariadna_nav_state_t *s)
{
    if (!isfinite(s->pz) || !isfinite(s->vx) || !isfinite(s->vy) ||
        !isfinite(s->vz) || !isfinite(s->ux) || !isfinite(s->uy) ||
        !isfinite(s->uz) || !isfinite(s->wx) || !isfinite(s->wy) ||
        !isfinite(s->wz))
        return false;
    if (!isfinite(s->px) || !isfinite(s->py))                return false;
    if (!isfinite(s->ax) || !isfinite(s->ay) || !isfinite(s->az))
        return false;

    if (s->pz < -20.0f || s->pz > 3000.0f)                   return false;
    if (fabsf(s->vx) > 120.0f || fabsf(s->vy) > 120.0f)      return false;
    if (fabsf(s->vz) > 200.0f)                               return false;
    if (fabsf(s->wx) > 35.0f  || fabsf(s->wy) > 35.0f || fabsf(s->wz) > 35.0f)
        return false;

    float n2 = s->ux * s->ux + s->uy * s->uy + s->uz * s->uz;
    if (n2 < 0.90f || n2 > 1.10f)                            return false;
    float q2 = s->qw * s->qw + s->qx * s->qx + s->qy * s->qy + s->qz * s->qz;
    if (q2 < 0.90f || q2 > 1.10f)                            return false;
    return true;
}

static void link_poll(void)
{
    while (MASTER_PORT.available()) {
        const uint8_t *pl = nullptr;
        uint8_t len = 0;
        uint8_t type = ariadna_rx_byte(&s_rx, (uint8_t)MASTER_PORT.read(), &pl, &len);
        if (!type) continue;

        switch (type) {
        case ARIADNA_PKT_NAV_STATE: {
            ariadna_nav_state_t s;
            if (!ariadna_unpack(pl, len, &s, sizeof(s))) break;
            if (!state_sane(&s)) break;
            s_state      = s;
            s_state_us   = micros();
            s_have_state = true;
            break;
        }
        case ARIADNA_PKT_MASTER_STATE: {
            ariadna_master_state_t m;
            if (!ariadna_unpack(pl, len, &m, sizeof(m))) break;
            s_master       = m;
            s_master_us    = micros();
            s_have_master  = true;
            if (!on_ground()) umb_i2c_detach();
            break;
        }
        case ARIADNA_PKT_CMD: {
            ariadna_cmd_t c;
            if (ariadna_unpack(pl, len, &c, sizeof(c))) handle_command(&c);
            break;
        }
        case ARIADNA_PKT_PARAM: {
            ariadna_param_pkt_t p;
            if (ariadna_unpack(pl, len, &p, sizeof(p))) param_receive(&p);
            break;
        }
        case ARIADNA_PKT_RAMPA_CMD: {
            ariadna_rampa_cmd_t rc;
            if (ariadna_unpack(pl, len, &rc, sizeof(rc))) umb_handle_rampa_cmd(&rc);
            break;
        }
        case ACTCON_PKT_PYRO_CMD: {
            // Bench firing. Listened to only while the Master says TEST;
            // in every other state the packet is simply not for us.
            actcon_pyro_cmd_t c;
            if (s_have_master && (s_master.flags & ARIADNA_MST_TEST) &&
                ariadna_unpack(pl, len, &c, sizeof(c)))
                (void)actcon_pyro_command(&s_test_sm, &c, millis());
            break;
        }
        default:
            break;
        }
    }
}

static ariadna_uart_tx_t s_tx;

static void link_send(uint8_t type, const void *payload, uint8_t len)
{
    ariadna_uart_tx_send(&s_tx, MASTER_PORT, type, payload, len);
}

// =====================================================================
//  5b. RAMPA UMBILICAL I2C BRIDGE (DEDICATED I2CN / Wire1, D4/D5)
//
//  Completely isolated from Master MCU's PCA9685 servo bus (I2C0).
//  Only active on the ground; at T-100 ms (RAMPA_CMD_COMMIT) or liftoff,
//  Wire1.end() is called and D4/D5 are floated Hi-Z before pogo separation.
// =====================================================================
static bool s_umb_attached = false;

static void umb_i2c_attach(void)
{
    if (s_umb_attached) return;
    // NSDA/NSCL have no external pull-ups on Ariadna V2: enable internal
    // RP2350 pull-ups and clock at 100 kHz for clean edges over pogo + R21.
    pinMode(PIN_UMB_SDA, INPUT_PULLUP);
    pinMode(PIN_UMB_SCL, INPUT_PULLUP);
    Wire1.setSDA(PIN_UMB_SDA);
    Wire1.setSCL(PIN_UMB_SCL);
    Wire1.begin();
    Wire1.setClock(ARIADNA_RAMPA_I2C_HZ);
    Wire1.setTimeout(5);
    s_umb_attached = true;
}

static void umb_i2c_detach(void)
{
    if (!s_umb_attached) return;
    Wire1.end();
    pinMode(PIN_UMB_SDA, INPUT);
    pinMode(PIN_UMB_SCL, INPUT);
    s_umb_attached = false;
}

static void umb_handle_rampa_cmd(const ariadna_rampa_cmd_t *c)
{
    // Never touch the umbilical I2C bus in flight.
    if (!on_ground()) {
        umb_i2c_detach();
        return;
    }

    umb_i2c_attach();

    Wire1.beginTransmission((uint8_t)ARIADNA_RAMPA_I2C_ADDR);
    Wire1.write((const uint8_t *)c, sizeof(*c));
    const uint8_t wr_rc = Wire1.endTransmission(true);

    if (wr_rc == 0) {
        uint8_t buf[ARIADNA_RAMPA_STAT_SIZE];
        const int got = Wire1.requestFrom((int)ARIADNA_RAMPA_I2C_ADDR, (int)sizeof(buf));
        size_t n = 0;
        while (Wire1.available()) {
            const int ch = Wire1.read();
            if (n < sizeof(buf)) buf[n++] = (uint8_t)ch;
        }
        ariadna_rampa_status_t st;
        if (got == (int)sizeof(buf) && n == sizeof(buf) &&
            ariadna_rampa_status_decode(buf, n, &st) == ARIADNA_RAMPA_OK) {
            link_send(ARIADNA_PKT_RAMPA_STATUS, &st, (uint8_t)sizeof(st));
        }
    }

    // At T-100 ms COMMIT, both ends immediately detach I2C before ignition
    // and magnetic pogo separation.
    if (c->cmd == RAMPA_CMD_COMMIT) {
        umb_i2c_detach();
    }
}

// =====================================================================
//  6. PYRO
//
//  The SAFE state is the one reached by doing nothing. Outputs are driven
//  LOW before anything else in setup(), and one function - pyro_write() -
//  is the only place in this file that can drive a channel HIGH.
// =====================================================================
static inline bool master_fresh(void)
{
    return s_have_master && (micros() - s_master_us) <= 500000u;
}

/** Flight logic armed: guidance, fins, TVC, aerocover and sequencer events. */
static inline bool flight_armed(void)
{
    if (!master_fresh())                                         return false;
    if (!(s_master.flags & ARIADNA_MST_ARMED))                   return false;
    if (s_master.flags & (ARIADNA_MST_TEST | ARIADNA_MST_ABORT)) return false;
    return true;
}

/** Physical pyro firing: flight armed AND not inhibited (HIL safe mode). */
static inline bool pyro_permitted(void)
{
    if (!flight_armed())                                         return false;
    if (s_master.flags & ARIADNA_MST_PYRO_INHIBIT)               return false;
    return true;
}

/** Bench firing: the Master is present, fresh and in TEST, RBF pulled. */
static inline bool test_permitted(void)
{
    if (!master_fresh())                                  return false;
    if (!(s_master.flags & ARIADNA_MST_TEST))             return false;
    if (s_master.flags & ARIADNA_MST_ABORT)               return false;
    if (!(s_master.flags & ARIADNA_MST_RBF_OUT))          return false;
    return s_dg_out.phase == DG_PHASE_IDLE;
}

/** Bench TVC+fin polarity check on real IMU attitude (ground IDLE, pyros OFF). */
static inline bool polarity_test_active(void)
{
    if (!master_fresh())                                         return false;
    if (flight_armed() || test_permitted())                      return false;
    if (s_master.state != ARIADNA_STATE_IDLE)                    return false;
    if (s_master.flags & (ARIADNA_MST_TEST | ARIADNA_MST_ABORT)) return false;
    return (s_master.flags2 & ARIADNA_MST2_POLARITY_TEST) != 0;
}

static void pyro_write(uint8_t mask)
{
    const uint8_t pins[3] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };
    for (uint8_t i = 0; i < 3; i++)
        digitalWrite(pins[i], (mask & (1u << i)) ? HIGH : LOW);
    s_out_mask    = mask;
    s_fired_mask |= mask;
}

// Continuity is sampled only when no channel is firing: the sense node is
// meaningless while current flows through the bridge.
static void pyro_scan_continuity(void)
{
    if (s_out_mask) return;
    const uint8_t pins[3] = { PIN_CONT_1, PIN_CONT_2, PIN_CONT_3 };
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t acc = 0;
        for (uint8_t k = 0; k < CONT_SAMPLES; k++) acc += (uint32_t)analogRead(pins[i]);
        uint16_t v = (uint16_t)(acc / CONT_SAMPLES);
        s_cont_raw[i] = v;
#if CONT_ACTIVE_HIGH
        bool present = (v >= CONT_THRESHOLD);
#else
        bool present = (v <  CONT_THRESHOLD);
#endif
        if (present) mask |= (uint8_t)(1u << i);
    }
    s_cont_mask = mask;
}

// =====================================================================
//  7. NAV STATE -> GUIDANCE STATE
// =====================================================================
/** Rotate a body vector into the pad frame: v' = v + 2*qv x (qv x v + qw*v). */
static void quat_rotate(const ariadna_nav_state_t *s, const float v[3], float o[3])
{
    const float qw = s->qw, qx = s->qx, qy = s->qy, qz = s->qz;
    const float tx = 2.0f * (qy * v[2] - qz * v[1]);
    const float ty = 2.0f * (qz * v[0] - qx * v[2]);
    const float tz = 2.0f * (qx * v[1] - qy * v[0]);
    o[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    o[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    o[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

static void nav_from_state(const ariadna_nav_state_t *s, dg_nav_t *out)
{
    out->h  = s->pz;
    out->vx = s->vx;  out->vy = s->vy;  out->vz = s->vz;
    out->bx = s->ux;  out->by = s->uy;  out->bz = s->uz;
    out->wx = s->wx;  out->wy = s->wy;  out->wz = s->wz;
    float g[3];
    quat_rotate(s, BODY_SERVO1_AXIS, g);
    out->gx = g[0];  out->gy = g[1];  out->gz = g[2];
    // Specific force along the thrust axis. The link carries the KINEMATIC
    // acceleration (gravity removed), so gravity goes back in: what an
    // accelerometer on that axis reads is (a - g_vec) . b.
    out->a_axial  = s->ax * s->ux + s->ay * s->uy + (s->az + DG_G) * s->uz;
    out->lidar_ok = (s->flags & ARIADNA_NAV_LIDAR_OK) != 0;
    out->valid    = (s->flags & ARIADNA_NAV_VALID) != 0;
}

// Dead reckoning across a short link gap, on the last acceleration the
// Position MCU reported. It exists to survive a gap, not to fly on.
static void nav_extrapolate(dg_nav_t *n, const ariadna_nav_state_t *s, float dt)
{
    n->h  += n->vz * dt + 0.5f * s->az * dt * dt;
    n->vx += s->ax * dt;
    n->vy += s->ay * dt;
    n->vz += s->az * dt;
    // the body axes ride the rate: db/dt = w x b, dg/dt = w x g
    float nx = n->bx + (n->wy * n->bz - n->wz * n->by) * dt;
    float ny = n->by + (n->wz * n->bx - n->wx * n->bz) * dt;
    float nz = n->bz + (n->wx * n->by - n->wy * n->bx) * dt;
    float inv = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    n->bx = nx * inv;  n->by = ny * inv;  n->bz = nz * inv;
    float mx = n->gx + (n->wy * n->gz - n->wz * n->gy) * dt;
    float my = n->gy + (n->wz * n->gx - n->wx * n->gz) * dt;
    float mz = n->gz + (n->wx * n->gy - n->wy * n->gx) * dt;
    float gd = mx * n->bx + my * n->by + mz * n->bz;
    mx -= gd * n->bx;  my -= gd * n->by;  mz -= gd * n->bz;
    float gi = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    n->gx = mx * gi;  n->gy = my * gi;  n->gz = mz * gi;
}

// =====================================================================
//  8. FLIGHT PHASE
//
//  The Master owns the mission state; this board follows it for ASCENT
//  and finds the apogee itself, because the ignition altitude is chosen
//  from the apogee altitude and the drift there, and both are measured
//  here at 240 Hz.
// =====================================================================
static void phase_logic(void)
{
    if (!s_have_state || !s_dg_nav.valid) return;
    const dg_phase_t ph = s_dg_out.phase;
    if (ph != DG_PHASE_IDLE && ph != DG_PHASE_ASCENT) return;
    if (!flight_armed()) {
        // Not armed: nothing here may leave IDLE. (TEST and ABORT included.)
        return;
    }
    const uint8_t ms = s_master.state;
    const bool flying = (ms == ARIADNA_STATE_ASCENT || ms == ARIADNA_STATE_APOGEE ||
                         ms == ARIADNA_STATE_DESCENT || ms == ARIADNA_STATE_GUIDANCE);
    if (!flying) { s_h_max = 0.0f; return; }

    if (ph == DG_PHASE_IDLE && ms == ARIADNA_STATE_ASCENT) dg_begin_ascent();

    const float h  = s_dg_nav.h;
    const float vh = sqrtf(s_dg_nav.vx * s_dg_nav.vx + s_dg_nav.vy * s_dg_nav.vy);
    if (h > s_h_max) s_h_max = h;
    if (s_dg_nav.vz > -0.5f) s_vh_apogee = vh;       // the drift at the top

    // Falling, clear of the top, and high enough to have been a flight.
    if (s_dg_nav.vz < -3.0f && h < s_h_max - 1.5f &&
        s_h_max >= s_params.v[ARIADNA_P_SEQ_APOGEE_MIN]) {
        // A board that reset in the air has no h_max of its own. The energy
        // height is a lower bound on the apogee (drag only ever took energy
        // out), and a low apogee selects a LOW ignition altitude, so take
        // the larger of the two.
        float h_e = h + s_dg_nav.vz * s_dg_nav.vz / (2.0f * DG_G);
        float h_apo = s_h_max > h_e ? s_h_max : h_e;
        dg_begin_descent(h_apo, s_vh_apogee);
        NAV_PRINT("# DESCENT apogee=%.1f m drift=%.1f m/s -> ignite at %.2f m\n",
                  h_apo, s_vh_apogee, dg_h_ignite());
    }
}

// =====================================================================
//  9. COMMANDS
// =====================================================================
static void handle_command(const ariadna_cmd_t *c)
{
    switch (c->cmd) {
    case ARIADNA_CMD_ABORT:
        // Latching. Cut any pulse, park the actuators, stop the guidance.
        pyro_write(0);
        actcon_pyro_disarm(&s_test_sm);
        dg_init();
        ariadna_seq_init(&s_seq);
        break;
    case ARIADNA_CMD_DISARM:
        pyro_write(0);
        actcon_pyro_disarm(&s_test_sm);
        // On the ground a disarm also rewinds the flight logic, so a bench
        // run of the sequence can be repeated without a power cycle.
        if (on_ground()) {
            dg_init();
            ariadna_seq_init(&s_seq);
            s_h_max = 0.0f;
            s_fired_mask = 0;
        }
        break;
    case ARIADNA_CMD_REBOOT:
        if (on_ground() && (c->arg == 0 || c->arg == 2)) {
            pyro_write(0);
            actcon_pyro_disarm(&s_test_sm);
            delay(20);
            watchdog_enable(1, 1);
            while (1) { }
        }
        break;
    default:
        break;
    }
}

// =====================================================================
//  10. ANNUNCIATOR
//
//    solid red         self test failed. Do not arm.
//    red, 2 flashes    armed, but channel 1 has NO CONTINUITY
//    red, 3 flashes    armed, parameters not received from the Master
//    slow blue         alive, waiting for a navigation state
//    slow amber        navigation state arriving, not armed
//    slow green        armed and receiving, continuity on channel 1
//    fast cyan         descending, ignition armed
//    fast green        the landing burn is running
//    fast magenta      the navigation state has gone stale, actuators parked
//    fast amber        TEST mode, bench pyro arming is LIVE
//    solid white       a pyro channel is firing
// =====================================================================
static ariadna_rgb_t s_rgb;
static const uint8_t NAV_RGB_BRIGHTNESS = 60;

static void annunciate(bool fresh, bool stale_parked)
{
    const bool armed = flight_armed();
    if (!s_post_pass)      { ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_SOLID, 0); return; }
    if (s_out_mask)        { ariadna_rgb_set(&s_rgb, ARIADNA_C_WHITE, ARIADNA_PAT_SOLID, 0); return; }
    if (s_test_sm.armed)   { ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_FAST, 0); return; }
    if (stale_parked)      { ariadna_rgb_set(&s_rgb, ARIADNA_C_MAGENTA, ARIADNA_PAT_FAST, 0); return; }
    if (s_dg_out.phase == DG_PHASE_BURN || s_dg_out.phase == DG_PHASE_IGNITING) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_FAST, 0); return;
    }
    if (s_dg_out.phase == DG_PHASE_DESCENT) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_CYAN, ARIADNA_PAT_FAST, 0); return;
    }
    if (armed && !(s_cont_mask & 0x01)) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_BLINK_N, 2); return;
    }
    if (armed && s_params_seen_n < ARIADNA_PARAM_COUNT) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_BLINK_N, 3); return;
    }
    if (!fresh)            { ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SLOW, 0); return; }
    if (armed)             { ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_SLOW, 0); return; }
    ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_SLOW, 0);
}

// =====================================================================
//  11. POWER-ON SELF TEST
//
//  Cheap, and it catches the failure that matters: a guidance module that
//  linked but does not compute, or a sign that turns a loop into positive
//  feedback. A failure here is reported and the board refuses to fly.
// =====================================================================
static bool self_test(void)
{
    bool ok = true;

    // Motor table: zero outside the burn, the peak where the table says.
    if (dg_thrust_ref(-0.1f) != 0.0f || dg_thrust_ref(DG_BURN_T + 0.1f) != 0.0f) {
        NAV_PRINT("POST thrust outside the burn is not zero\n");
        ok = false;
    }
    float f = dg_thrust_ref(1.25f);
    if (!(f > 0.9f * DG_PEAK_THRUST && f <= DG_PEAK_THRUST + 0.01f)) {
        NAV_PRINT("POST thrust(1.25)=%.1f\n", f);
        ok = false;
    }

    // Guidance law: no drift, no tilt; drifting East must tilt WEST; the
    // two horizontal channels must be the same law rotated 90 degrees.
    float ax, ay, az, bx, by, bz;
    dg_guidance_dir(30.0f, 0.0f, 0.0f, -30.0f, 2.8f, &ax, &ay, &az);
    if (fabsf(ax) > 1e-5f || fabsf(ay) > 1e-5f) { NAV_PRINT("POST tilt bias\n"); ok = false; }
    dg_guidance_dir(30.0f, 6.0f, 0.0f, -30.0f, 2.8f, &ax, &ay, &az);
    dg_guidance_dir(30.0f, 0.0f, 6.0f, -30.0f, 2.8f, &bx, &by, &bz);
    if (ax >= 0.0f || fabsf(ax - by) > 1e-5f || fabsf(ay) > 1e-6f || fabsf(bx) > 1e-6f) {
        NAV_PRINT("POST guidance direction: (%.4f %.4f) (%.4f %.4f)\n", ax, ay, bx, by);
        ok = false;
    }

    // The gimbal axes must FOLLOW the airframe as it rolls.
    float u1x, u1y, u1z, u2x, u2y, u2z;
    dg_gimbal_basis(0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
    if (fabsf(u1y - 1.0f) > 1e-4f || fabsf(u2x + 1.0f) > 1e-4f) {
        NAV_PRINT("POST gimbal axes do not follow roll\n");
        ok = false;
    }

    // Ignition search: inside the envelope, never above the apogee, and a
    // band table that is a band (h_min below h_max wherever there is one).
    for (float h0 = 100.0f; h0 <= 220.0f; h0 += 40.0f) {
        float hc = dg_ign_compute(h0, 0.0f, 0.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        if (!(hc > 15.0f && hc < h0 - 0.9f)) {
            NAV_PRINT("POST h_cmd(%.0f)=%.1f\n", h0, hc);
            ok = false;
        }
    }
    {
        const dg_params_t *gp = dg_get_params();
        for (int i = 0; i < DG_IGN_N; i++)
            if (gp->tbl_hmax[i] > 0.0f && gp->tbl_hmax[i] < gp->tbl_hmin[i]) {
                NAV_PRINT("POST ignition band %d inverted\n", i);
                ok = false;
            }
    }

    // The clamp plan from a typical ignition state must be solvable and
    // must predict a touchdown the landing gear survives.
    float resid, t_td;
    float k = dg_solve_plan(52.0f, -31.0f, 0.0f, 1.0f, &resid, &t_td);
    float v_td = dg_project_vz(52.0f, -31.0f, 0.0f, k, 1.0f, nullptr);
    if (!(k >= DG_CLAMP_MIN && k <= DG_CLAMP_MAX && v_td > -4.0f && v_td <= 0.0f &&
          t_td > 1.0f && t_td < 5.0f)) {
        NAV_PRINT("POST plan k=%.2f v_td=%.2f t_td=%.2f\n", k, v_td, t_td);
        ok = false;
    }

    // The quaternion rotation the roll reference comes from.
    ariadna_nav_state_t q0;
    memset(&q0, 0, sizeof(q0));
    q0.qw = 0.70710678f;  q0.qz = 0.70710678f;       // 90 deg yaw about pad +z
    float gv[3];
    quat_rotate(&q0, BODY_SERVO1_AXIS, gv);
    if (fabsf(gv[0]) > 1e-4f || fabsf(gv[1] - 1.0f) > 1e-4f) {
        NAV_PRINT("POST quat yaw90 g=(%.3f %.3f %.3f)\n", gv[0], gv[1], gv[2]);
        ok = false;
    }

    // The parameter table's defaults must be the guidance module's own.
    dg_params_t d;
    dg_params_default(&d);
    if (fabsf(s_params.v[ARIADNA_P_G_WN_TVC] - d.wn_tvc) > 1e-3f ||
        fabsf(s_params.v[ARIADNA_P_G_MASS_GROSS] - d.mass_gross) > 1e-4f ||
        fabsf(s_params.v[ARIADNA_P_G_ROLL_GAIN] - d.roll_gain) > 1e-3f) {
        NAV_PRINT("POST parameter defaults differ from the guidance module\n");
        ok = false;
    }

    dg_init();
    return ok;
}

// =====================================================================
//  12. CORE 0 — 200 Hz control
// =====================================================================
void setup()
{
    // Pyro outputs first, before anything can go wrong anywhere else.
    pinMode(PIN_PYRO_1, OUTPUT); digitalWrite(PIN_PYRO_1, LOW);
    pinMode(PIN_PYRO_2, OUTPUT); digitalWrite(PIN_PYRO_2, LOW);
    pinMode(PIN_PYRO_3, OUTPUT); digitalWrite(PIN_PYRO_3, LOW);

    pinMode(PIN_CONT_1, INPUT);
    pinMode(PIN_CONT_2, INPUT);
    pinMode(PIN_CONT_3, INPUT);
    analogReadResolution(12);

    Serial.begin(ARIADNA_CONSOLE_BAUD);

    ariadna_rgb_begin(&s_rgb, NAV_RGB_BRIGHTNESS, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

    // 2 kB RX buffer: the nav_state stream is ~113 bytes per 5 ms tick and
    // the arduino-pico default of 32 bytes lost almost all of it.
    ariadna_uart_begin(MASTER_PORT, PIN_MASTER_TX, PIN_MASTER_RX);
    ariadna_uart_tx_begin(&s_tx, uart0);     // uart0 is Serial1. Must match MASTER_PORT.

    ariadna_rx_init(&s_rx);
    memset(&s_state,  0, sizeof(s_state));
    memset(&s_master, 0, sizeof(s_master));
    memset(&s_dg_nav, 0, sizeof(s_dg_nav));
    memset(&s_dg_out, 0, sizeof(s_dg_out));
    memset(s_param_seen, 0, sizeof(s_param_seen));

    ariadna_params_default(&s_params);
    dg_init();
    params_apply();
    ariadna_seq_init(&s_seq);
    actcon_pyro_init(&s_test_sm);

    s_post_pass = self_test();
    params_apply();

    NAV_PRINT("\nAriadna V2 Navigation MCU fw %d, guidance %d - POST %s\n",
              ARIADNA_FW_VERSION, DG_VERSION, s_post_pass ? "PASS" : "FAIL");
    s_ready = true;
}

void loop()
{
    static uint32_t next_us        = 0;
    static uint32_t next_status_ms = 0;
    static uint32_t next_pad_ms    = 0;
    static uint32_t next_report_ms = 0;
    const  uint32_t period         = (uint32_t)(1e6f / DG_CTRL_HZ);

    // ---- link, on EVERY pass ------------------------------------------
    // Draining the port only once per 5 ms tick overflowed the receive
    // buffer (fw <= 3.20) and made the state up to 5 ms older than it had
    // to be. The DMA ring is serviced here too, so a queued actuator frame
    // starts the moment the previous one has left.
    ariadna_uart_tx_service(&s_tx);
    link_poll();

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) {
        s_overruns++;
        next_us = micros() + period;
    }
    uint32_t t0 = micros();
    const uint32_t now_ms = millis();

    // ---- link: drained once more, so the tick sees the newest state ----
    ariadna_uart_tx_service(&s_tx);
    link_poll();

    bool state_fresh = false;
    if (s_have_state) {
        uint32_t age = micros() - s_state_us;
        if (age <= NAV_STALE_MS * 1000u) {
            state_fresh = true;
            nav_from_state(&s_state, &s_dg_nav);
        } else if (age > NAV_HOLD_MS * 1000u) {
            s_dg_nav.valid = false;
        } else {
            nav_extrapolate(&s_dg_nav, &s_state, DG_CTRL_DT);
        }
    }

    // ---- flight phase, guidance, sequencer ----------------------------
    const bool pol_mode = polarity_test_active();
    if (pol_mode) {
        const float pol_thr = ariadna_pol_thrust_decode(s_master.flags2);
        const float pol_spd = ariadna_pol_speed_decode(s_master.t_launch_ms);
        if (s_have_state && state_fresh && (s_state.flags & ARIADNA_NAV_IMU_OK)) {
            dg_step_polarity(&s_dg_nav, &s_dg_out, pol_thr, pol_spd);
        } else {
            dg_step_polarity(nullptr, &s_dg_out, pol_thr, pol_spd);
        }
    } else {
        phase_logic();

        if (s_have_state) {
            nav_publish(&s_dg_nav);
            dg_step_control(&s_dg_nav, &s_dg_out);
#if !NAV_USE_CORE1
            dg_step_planner(&s_dg_nav);
            s_plan_calls++;
#endif
        }
    }

    const bool armed   = flight_armed();
    const bool pyro_ok = pyro_permitted();
    {
        ariadna_seq_in_t in;
        in.phase          = (uint8_t)s_dg_out.phase;
        in.ignite         = s_dg_out.ignite ? 1 : 0;
        in.thrust_on      = s_dg_out.thrust_on ? 1 : 0;
        in.permit         = armed ? 1 : 0;
        in.t_to_ignition  = s_dg_out.t_to_ignition;
        in.t_to_touchdown = s_dg_out.t_to_touchdown;
        in.h              = s_dg_nav.h;
        ariadna_seq_step(&s_seq, &s_seq_cfg, &in, now_ms);
        if (s_dg_nav.lidar_ok && s_dg_out.phase >= DG_PHASE_DESCENT)
            s_seq.events |= ARIADNA_EV_LIDAR;
    }

    // ---- pyro outputs --------------------------------------------------
    // Flight and bench firing are mutually exclusive by construction: one
    // needs ARMED and not TEST, the other needs TEST. The permit is checked
    // again here, at the output stage, so withdrawing it CUTS current.
    // In HIL safe mode (ARIADNA_MST_PYRO_INHIBIT) `armed` is true so the
    // sequencer runs, while `pyro_ok` is false so physical pins stay LOW.
    {
        uint8_t mask = 0;
        if (pyro_ok) {
            for (uint8_t i = 0; i < 3; i++) if (s_seq.pyro[i]) mask |= (uint8_t)(1u << i);
        } else if (test_permitted()) {
            mask = actcon_pyro_tick(&s_test_sm, now_ms);
        } else {
            actcon_pyro_disarm(&s_test_sm);
            (void)actcon_pyro_tick(&s_test_sm, now_ms);
        }
        static uint8_t last_mask = 0;
        if (mask & ~last_mask)
            NAV_PRINT("# PYRO mask 0x%X at h=%.1f vz=%.1f t_td=%.2f\n", mask, s_dg_nav.h,
                      s_dg_nav.vz, s_dg_out.t_to_touchdown);
        last_mask = mask;
        pyro_write(mask);
    }
    pyro_scan_continuity();

    // ---- actuator command ----------------------------------------------
    {
        ariadna_actuator_t act;
        memset(&act, 0, sizeof(act));
        act.t_ms  = now_ms;
        act.clamp = s_dg_out.clamp;
        act.phase = (uint8_t)s_dg_out.phase;

        const bool act_ok = pol_mode
            ? (s_have_state && state_fresh && (s_state.flags & ARIADNA_NAV_IMU_OK))
            : (s_have_state && s_dg_nav.valid);
        if (act_ok) {
            act.tvc_x_deg = s_dg_out.servo1_deg;
            act.tvc_y_deg = s_dg_out.servo2_deg;
            act.flags |= ARIADNA_ACT_VALID;
            if (s_dg_out.fins_live) {
                for (uint8_t i = 0; i < 4; i++) act.fin_deg[i] = s_dg_out.fin_deg[i];
                act.flags |= ARIADNA_ACT_FINS_LIVE;
            }
        } else {
            // No usable state: nozzle and fins to neutral, clamp held.
            act.flags |= ARIADNA_ACT_TVC_PARK;
        }
        if (s_seq.cover_open) act.flags |= ARIADNA_ACT_COVER_OPEN;
        act.events = s_seq.events;
        if ((s_fired_mask & 0x03) || (s_seq.events & (ARIADNA_EV_IGN1 | ARIADNA_EV_IGN2)))
            act.flags |= ARIADNA_ACT_IGNITE;

        link_send(ARIADNA_PKT_ACTUATOR, &act, sizeof(act));
    }

    // ---- status ---------------------------------------------------------
    if (now_ms >= next_status_ms) {
        next_status_ms = now_ms + 1000 / ARIADNA_STATUS_HZ;

        ariadna_nav_status_t st;
        memset(&st, 0, sizeof(st));
        st.t_ms        = now_ms;
        st.k_plan      = s_dg_out.k_plan;
        st.thrust_est  = s_dg_out.thrust_est;
        st.h_ignite    = dg_h_ignite();
        st.tilt_cmd    = s_dg_out.tilt_cmd;
        st.tilt_err    = s_dg_out.tilt_err;
        st.roll_rate   = s_dg_out.roll_rate;
        st.t_to_ign    = s_dg_out.t_to_ignition;
        st.t_to_td     = s_dg_out.t_to_touchdown;
        st.t_scale     = s_dg_out.t_scale;
        st.fin_gain    = s_dg_out.fin_gain;
        st.h_apogee    = s_h_max;
        st.plan_resid  = s_dg_out.plan_resid;
        st.rx_ok       = s_rx.rx_ok;
        st.rx_bad      = s_rx.rx_crc + s_rx.rx_len;
        st.plan_iters  = s_plan_calls;
        st.loop_max_us = (uint16_t)(s_loop_max_us > 65535 ? 65535 : s_loop_max_us);
        st.overruns    = (uint16_t)(s_overruns > 65535 ? 65535 : s_overruns);
        st.fw_version  = ARIADNA_FW_VERSION;
        st.param_crc   = ariadna_params_crc(&s_params);
        st.phase       = (uint8_t)s_dg_out.phase;
        st.pyro_cont   = s_cont_mask;
        st.pyro_fired  = s_fired_mask;
        st.events      = s_seq.events;
        st.pyro_active = s_out_mask;
        st.params_seen = s_params_seen_n;
        st.drag_scale  = s_dg_out.drag_scale;
        st.band_lo     = s_dg_out.h_band_lo;
        st.band_hi     = s_dg_out.h_band_hi;
        st.flags       = (uint8_t)((armed        ? ARIADNA_NST_ARMED      : 0) |
                                   (s_post_pass  ? ARIADNA_NST_POST_PASS  : 0) |
                                   (state_fresh  ? ARIADNA_NST_LINK_OK    : 0) |
                                   (s_plan_calls ? ARIADNA_NST_PLANNER_OK : 0) |
                                   ((s_cont_mask & 0x02) ? ARIADNA_NST_BACKUP_ARM : 0) |
                                   (s_params_seen_n >= ARIADNA_PARAM_COUNT ? ARIADNA_NST_PARAM_OK : 0) |
                                   (s_test_sm.armed ? ARIADNA_NST_TEST_ARMED : 0));
        link_send(ARIADNA_PKT_NAV_STATUS, &st, sizeof(st));
    }

    // ---- bench report, TEST mode only -----------------------------------
    if (s_have_master && (s_master.flags & ARIADNA_MST_TEST) && now_ms >= next_report_ms) {
        next_report_ms = now_ms + ACTCON_REPORT_PERIOD_MS;
        actcon_pyro_report_t r;
        memset(&r, 0, sizeof(r));
        r.t_ms        = now_ms;
        for (uint8_t i = 0; i < 3; i++) r.raw[i] = s_cont_raw[i];
        r.threshold   = CONT_THRESHOLD;
        r.arm_left_ds = actcon_arm_left_ds(&s_test_sm, now_ms);
        r.cont_mask   = s_cont_mask;
        r.fired_mask  = s_test_sm.fired_mask;
        r.firing_mask = s_out_mask;
        r.armed       = s_test_sm.armed;
        r.last_reject = s_test_sm.last_reject;
        r.active_high = CONT_ACTIVE_HIGH;
        link_send(ACTCON_PKT_PYRO_REPORT, &r, sizeof(r));
    }

    // ---- pad console: only with a USB host attached, never blocking ------
    if (now_ms >= next_pad_ms) {
        next_pad_ms = now_ms + 1000 / PAD_CONSOLE_HZ;
        NAV_PRINT("ph=%d h=%.1f vz=%.1f hcmd=%.1f tign=%.1f ttd=%.1f srv=%.2f/%.2f "
                  "fin=%.1f/%.1f/%.1f/%.1f clamp=%.2f ev=0x%02X cont=%X prm=%u/%u "
                  "loop=%luus ovr=%lu\n",
                  (int)s_dg_out.phase, s_dg_nav.h, s_dg_nav.vz, dg_h_ignite(),
                  s_dg_out.t_to_ignition, s_dg_out.t_to_touchdown,
                  s_dg_out.servo1_deg, s_dg_out.servo2_deg,
                  s_dg_out.fin_deg[0], s_dg_out.fin_deg[1], s_dg_out.fin_deg[2],
                  s_dg_out.fin_deg[3], s_dg_out.clamp, s_seq.events, s_cont_mask,
                  s_params_seen_n, ARIADNA_PARAM_COUNT,
                  (unsigned long)s_loop_max_us, (unsigned long)s_overruns);
    }

    annunciate(state_fresh, s_have_state && !s_dg_nav.valid);
    ariadna_rgb_service(&s_rgb);

    uint32_t dt = micros() - t0;
    if (dt > s_loop_max_us) s_loop_max_us = dt;
}

// =====================================================================
//  13. CORE 1 — the clamp planner
//
//  dg_step_planner() decides for itself whether a solve is due (10 Hz
//  during the burn, 1 Hz for the landing countdown during the descent)
//  and otherwise returns at once. A solve is a few milliseconds; this core
//  has nothing else to do.
// =====================================================================
#if NAV_USE_CORE1
void setup1()
{
    while (!s_ready) tight_loop_contents();
}

void loop1()
{
    static uint32_t next_us = 0;
    const  uint32_t period  = (uint32_t)(1e6f / DG_CTRL_HZ);

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;
    next_us = micros() + period;

    dg_nav_t n;
    if (!nav_acquire(&n)) return;      // torn read: try again next tick
    dg_step_planner(&n);
    s_plan_calls++;
}
#endif
