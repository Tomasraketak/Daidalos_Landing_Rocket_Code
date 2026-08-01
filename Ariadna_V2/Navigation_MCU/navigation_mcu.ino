// =====================================================================
//  Ariadna V2 — NAVIGATION MCU
//
//  Target: Seeed XIAO RP2350, arduino-pico core (earlephilhower).
//
//  Role: flight path prediction and active control. This board decides
//  WHEN the landing motor lights and WHERE the nozzle points. It is the
//  only board in the vehicle that can fire a pyro.
//
//  It owns no sensors. Everything it knows arrives as ariadna_nav_state_t
//  from the Position MCU, routed through the Master.
//
//  ---------------------------------------------------------------------
//  WHAT IS AND IS NOT VERIFIED
//  ---------------------------------------------------------------------
//  guidance.cpp is a verified port of the Monte-Carlo tested simulation:
//  108 numerical checks, `cd ../../test && make`. It is PLANAR — one
//  altitude, one horizontal axis, one tilt angle, one servo.
//
//  Daidalos flies in three dimensions with a two-axis gimbal. Section 6 of
//  this file bridges the two by choosing a vertical guidance plane,
//  rotating the state into it, running the verified law, and decomposing
//  the result back onto two servos.
//
//  THAT BRIDGE IS NEW CODE AND IS NOT COVERED BY THE GOLDEN VECTORS. It is
//  the first thing to review and the first thing to test on the bench. Its
//  failure mode is a TVC command pointing the wrong way, which diverges in
//  well under a second.
//
//  ---------------------------------------------------------------------
//  CORE ALLOCATION
//  ---------------------------------------------------------------------
//    core 0   200 Hz: link, attitude loop, ignition, pyro, actuator TX
//    core 1   trajectory replanner, one solver iteration per call
//
//  Shared through a seqlock, never a mutex: the 200 Hz loop must not block
//  on the planner.
//
//  ---------------------------------------------------------------------
//  READ BEFORE CONNECTING AN IGNITER
//  ---------------------------------------------------------------------
//  This is unflown software. Bench it with PIN_PYRO_* on a LED and PYRO
//  ARMING held off. Every pin number, threshold and polarity below
//  describes an INTENDED vehicle and must be checked against yours.
// =====================================================================

#include "guidance.h"
#include "../common/ariadna_link.h"
#include "../common/ariadna_uart_dma.h"

#include <Arduino.h>
#include <string.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE CONFIGURATION
//
//  Pin names are the XIAO silkscreen aliases (D0..D10) that the
//  arduino-pico board variant defines. Using them rather than raw GPIO
//  numbers means this file stays correct if Seeed renumbers the die, and
//  it matches the pinout spreadsheet directly.
// =====================================================================

// ---- pyro ------------------------------------------------------------
// Channel 1 fires the landing motor. Channel 2 is a fully independent
// backup igniter in the SAME motor, fired only if channel 1 produced no
// thrust (see section 8). Channel 3 is wired, continuity-checked and
// reserved; nothing in this file ever fires it automatically.
static const uint8_t PIN_PYRO_1  = D3;   // landing motor igniter
static const uint8_t PIN_PYRO_2  = D8;   // landing motor igniter, backup
static const uint8_t PIN_PYRO_3  = D9;   // reserve

static const uint8_t PIN_CONT_1  = D0;   // A0, continuity sense, channel 1
static const uint8_t PIN_CONT_2  = D1;   // A1
static const uint8_t PIN_CONT_3  = D2;   // A2

// ---- link to the Master ---------------------------------------------
// The pinout spreadsheet labels D6/D7 "UART1". On the arduino-pico core
// Serial1 is UART0 and Serial2 is UART1, and which instance can reach
// D6/D7 depends on the XIAO RP2350 variant's GPIO mapping. If this port
// stays silent on the bench, change this one line to Serial2 before
// touching anything else.
#define MASTER_PORT   Serial1
static const uint8_t PIN_MASTER_TX = D6;
static const uint8_t PIN_MASTER_RX = D7;

// ---- continuity detection -------------------------------------------
//
// Assumed circuit: the sense pin sits on a divider between the pyro rail
// and the firing FET's drain. With an igniter bridging the gap the pin is
// pulled toward the rail; with the igniter open or already fired it floats
// low. CONT_ACTIVE_HIGH selects that sense.
//
// Both numbers are guesses about YOUR hardware. Measure the ADC counts on
// the bench with a known-good igniter and with an open channel, and put
// the midpoint here. Getting the polarity backwards means the board
// reports a good igniter on an empty channel.
#define CONT_ACTIVE_HIGH  1
static const uint16_t CONT_THRESHOLD = 1200;   // of 4095, at 12-bit
static const uint8_t  CONT_SAMPLES   = 4;      // averaged, ADC noise is real

// ---- igniter timing --------------------------------------------------
static const uint32_t PYRO_PULSE_MS = 600;     // current through the bridge

// ---- link supervision ------------------------------------------------
// Identical policy to the V1 flight computer: a short gap is dead
// reckoned, a long one invalidates the state, which parks the TVC and
// FREEZES the clamp. Shutting the clamp mid-burn guarantees a crash and
// the grain finishes burning either way.
static const uint32_t NAV_STALE_MS = 60;
static const uint32_t NAV_HOLD_MS  = 250;

// ---- options ---------------------------------------------------------
#define NAV_USE_CORE1   1        // 0 = run the planner inline on core 0
#define PAD_CONSOLE_HZ  5        // USB CDC status rate before flight

// Diagnostics go through this. printf's varargs promote float to double by
// language rule, and on the RP2350 double is software emulated, so the
// promotion is a real cost. Confining it here keeps -Wdouble-promotion
// meaningful everywhere it matters — that is, in the control path.
#define NAV_PRINT(...)                                            \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        Serial.printf(__VA_ARGS__);                               \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

// Forward declarations. The Arduino IDE generates these automatically, but
// `make syntax` compiles this file as plain C++ where it does not — and a
// build that only works inside one IDE is a build that cannot be tested.
static void handle_command(const ariadna_cmd_t *c);

// =====================================================================
//  2. STATE
// =====================================================================

// --- link -------------------------------------------------------------
static ariadna_rx_t          s_rx;
static ariadna_nav_state_t   s_state;        // newest from the Position MCU
static ariadna_master_state_t s_master;      // newest from the Master
static uint32_t              s_state_us   = 0;   // micros() of last accepted
static bool                  s_have_state = false;
static uint32_t              s_master_us  = 0;
static bool                  s_have_master = false;

// --- guidance ---------------------------------------------------------
static dg_nav_t  s_dg_nav;      // planar state handed to the verified module
static dg_out_t  s_dg_out;
static bool      s_armed     = false;
static bool      s_post_pass = false;
static float     s_plane_az  = 0.0f;    // guidance plane azimuth, rad, pad-ENU
static bool      s_plane_locked = false;

// --- pyro -------------------------------------------------------------
static uint8_t   s_cont_mask  = 0;      // bit n set: channel n+1 has continuity
static uint8_t   s_fired_mask = 0;
static uint32_t  s_fire_start_ms[3] = { 0, 0, 0 };
static bool      s_fire_active[3]   = { false, false, false };
static uint32_t  s_ignite_ms  = 0;      // millis() when channel 1 fired
static bool      s_backup_due = false;

// --- housekeeping -----------------------------------------------------
static uint32_t  s_loop_max_us = 0;
static uint32_t  s_overruns    = 0;
static uint32_t  s_plan_calls  = 0;

// =====================================================================
//  3. CROSS-CORE HANDOVER
//
//  Core 0 writes the planar navigation state, core 1 reads it. A seqlock:
//  the writer bumps an odd/even counter around the copy and the reader
//  retries if it saw a torn window. On a two-core M33 with no data cache
//  this is a handful of instructions and it can never block core 0.
// =====================================================================
static volatile uint32_t s_seq   = 0;
static dg_nav_t          s_shared;
static volatile bool     s_ready = false;   // core 1 must not run before init

static inline void nav_publish(const dg_nav_t *n)
{
    s_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_shared = *n;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_seq++;
}

static inline bool nav_acquire(dg_nav_t *n)
{
    uint32_t a = s_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *n = s_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_seq == a;
}

// =====================================================================
//  4. LINK
// =====================================================================

// A frame that survives the CRC can still be nonsense — a sensor gone mad
// upstream, or a stale binary on the other end. Anything outside the
// flight envelope is treated as a link fault, not as a state.
static bool state_sane(const ariadna_nav_state_t *s)
{
    if (!isfinite(s->h) || !isfinite(s->vx) || !isfinite(s->vy) ||
        !isfinite(s->vz) || !isfinite(s->ux) || !isfinite(s->uy) ||
        !isfinite(s->uz) || !isfinite(s->wx) || !isfinite(s->wy))
        return false;

    if (s->h < -20.0f || s->h > 3000.0f)                     return false;
    if (fabsf(s->vx) > 120.0f || fabsf(s->vy) > 120.0f)      return false;
    if (fabsf(s->vz) > 200.0f)                               return false;
    if (fabsf(s->wx) > 35.0f  || fabsf(s->wy) > 35.0f)       return false;

    // The attitude vector must actually be a unit vector. A drifting norm
    // means the sender's quaternion is not being normalised, which would
    // silently scale every tilt angle this board computes.
    float n2 = s->ux * s->ux + s->uy * s->uy + s->uz * s->uz;
    if (n2 < 0.90f || n2 > 1.10f)                            return false;

    return true;
}

// Drain the port, keeping the newest state. Never blocks and never waits
// for a whole frame, so a truncated packet costs one control tick.
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
            if (!state_sane(&s)) break;          // counted as a bad frame below
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
            break;
        }
        case ARIADNA_PKT_CMD: {
            ariadna_cmd_t c;
            if (!ariadna_unpack(pl, len, &c, sizeof(c))) break;
            handle_command(&c);
            break;
        }
        default:
            break;
        }
    }
}

// DMA-fed transmit. A 42-byte actuator frame is 10 bytes longer than the
// UART's hardware FIFO, so a blocking write would spin for 217 us every
// tick — 4.3 % of this core, inside the 200 Hz loop. Queuing it costs a
// memcpy. See ../common/ariadna_uart_dma.h.
static ariadna_uart_tx_t s_tx;

static void link_send(uint8_t type, const void *payload, uint8_t len)
{
    ariadna_uart_tx_send(&s_tx, MASTER_PORT, type, payload, len);
}

// =====================================================================
//  5. PYRO
//
//  Every path here is written so that the SAFE state is the one reached by
//  doing nothing. Outputs are driven LOW before anything else in setup(),
//  firing requires several independent conditions to be true at once, and
//  the arming check is repeated at the output stage rather than trusted
//  from wherever the decision was made.
// =====================================================================

static inline bool pyro_permitted(void)
{
    // Four independent gates. Any one of them false inhibits every channel.
    if (!s_have_master)                                   return false;
    if (micros() - s_master_us > 500000u)                 return false;  // stale
    if (!(s_master.flags & ARIADNA_MST_ARMED))            return false;
    if (s_master.flags & (ARIADNA_MST_TEST | ARIADNA_MST_ABORT)) return false;
    return true;
}

static uint16_t cont_read(uint8_t pin)
{
    uint32_t acc = 0;
    for (uint8_t i = 0; i < CONT_SAMPLES; i++) acc += (uint32_t)analogRead(pin);
    return (uint16_t)(acc / CONT_SAMPLES);
}

// Continuity is sampled continuously, but only when no channel is firing:
// the sense node is meaningless while current flows through the bridge.
static void pyro_scan_continuity(void)
{
    if (s_fire_active[0] || s_fire_active[1] || s_fire_active[2]) return;

    const uint8_t pins[3] = { PIN_CONT_1, PIN_CONT_2, PIN_CONT_3 };
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 3; i++) {
        uint16_t v = cont_read(pins[i]);
#if CONT_ACTIVE_HIGH
        bool present = (v >= CONT_THRESHOLD);
#else
        bool present = (v <  CONT_THRESHOLD);
#endif
        if (present) mask |= (uint8_t)(1u << i);
    }
    s_cont_mask = mask;
}

// ch is 0..2. Returns true if the channel actually fired.
static bool pyro_fire(uint8_t ch)
{
    if (ch > 2)             return false;
    if (!pyro_permitted())  return false;
    if (s_fire_active[ch])  return false;

    const uint8_t pins[3] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };
    digitalWrite(pins[ch], HIGH);
    s_fire_active[ch]   = true;
    s_fire_start_ms[ch] = millis();
    s_fired_mask       |= (uint8_t)(1u << ch);
    return true;
}

// Called every tick. Ends each pulse on time, and immediately if arming is
// withdrawn — opening the arm switch must cut current, not merely prevent
// the next firing.
static void pyro_service(void)
{
    const uint8_t pins[3] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };
    bool permitted = pyro_permitted();

    for (uint8_t i = 0; i < 3; i++) {
        if (!s_fire_active[i]) continue;
        if (!permitted || millis() - s_fire_start_ms[i] >= PYRO_PULSE_MS) {
            digitalWrite(pins[i], LOW);
            s_fire_active[i] = false;
        }
    }
}

static void pyro_all_safe(void)
{
    digitalWrite(PIN_PYRO_1, LOW);
    digitalWrite(PIN_PYRO_2, LOW);
    digitalWrite(PIN_PYRO_3, LOW);
    s_fire_active[0] = s_fire_active[1] = s_fire_active[2] = false;
}

// =====================================================================
//  6. THREE DIMENSIONS TO ONE PLANE  — the new, unverified code
//
//  guidance.cpp solves a planar problem: altitude h, one horizontal axis
//  x, tilt theta from vertical toward +x, one servo. Daidalos flies in
//  three dimensions with a two-axis gimbal.
//
//  The bridge is a vertical GUIDANCE PLANE, chosen each tick:
//
//    * Its azimuth is the direction of horizontal motion. That is the
//      direction the landing burn has to kill, and by construction the
//      out-of-plane velocity is then zero — so the planar law is not an
//      approximation of the problem, it is the whole problem, expressed in
//      the frame that makes it planar.
//
//    * Below PLANE_LOCK_VH the azimuth is FROZEN. atan2 of a horizontal
//      velocity that is mostly noise spins the plane through 360 degrees
//      per tick, and the servo command spins with it. Freezing costs
//      nothing: with no horizontal velocity there is no direction to
//      correct, and the tilt command is near zero anyway.
//
//    * The azimuth is also frozen once the burn starts. Re-aiming the
//      plane mid-burn would rotate the attitude reference underneath a
//      closed loop that assumes it is fixed.
//
//  Rotating in:  everything is a dot product with the plane's unit vector.
//  Rotating out: the single planar servo angle is split across the two
//  gimbal axes by the same azimuth.
//
//  WHY THIS IS THE RISKIEST CODE IN THE FILE: a sign error here does not
//  produce a wrong number, it produces positive feedback. Bench test 3 in
//  the README exists specifically to catch it — deflect the airframe by
//  hand and confirm the nozzle swings to OPPOSE the motion, in both axes
//  independently.
// =====================================================================

static const float PLANE_LOCK_VH = 0.5f;   // m/s, below this the plane freezes

/**
 * Choose the guidance plane and rotate the 3D state into it.
 *
 * Fills `out` with the planar state the verified module expects, and
 * returns the plane azimuth in radians (pad-ENU, measured from +x toward
 * +y) so the servo command can be rotated back out.
 */
static float project_to_guidance_plane(const ariadna_nav_state_t *s,
                                       dg_nav_t *out)
{
    const float vh = sqrtf(s->vx * s->vx + s->vy * s->vy);

    // --- pick the plane ------------------------------------------------
    if (!s_plane_locked && vh >= PLANE_LOCK_VH)
        s_plane_az = atan2f(s->vy, s->vx);
    // else: keep the previous azimuth. On the very first tick that is 0,
    // which is harmless because vh < PLANE_LOCK_VH means there is nothing
    // to steer against.

    const float ca = cosf(s_plane_az);
    const float sa = sinf(s_plane_az);

    // --- rotate in -----------------------------------------------------
    // In-plane horizontal velocity: the signed projection, not |vh|. When
    // the plane is frozen the vehicle can drift backwards along it, and
    // the sign is what tells the guidance law which way to lean.
    out->h  = s->h;
    out->vx = s->vx * ca + s->vy * sa;
    out->vz = s->vz;

    // Tilt of the thrust axis within the plane. u is a unit vector, so
    // this is exact — no small-angle assumption, and no gimbal lock at
    // the vertical where an Euler yaw would be singular.
    const float u_in = s->ux * ca + s->uy * sa;
    out->theta = atan2f(u_in, s->uz);

    // Pitch rate about the in-plane horizontal axis. That axis is the
    // plane normal projected onto the horizontal, (-sin a, cos a, 0); a
    // positive rotation about it tilts the thrust axis toward +x, which is
    // the direction theta increases. Straight from the gyro, never
    // differentiated.
    out->omega = -s->wx * sa + s->wy * ca;

    out->t     = (float)s->t_ms * 1e-3f;
    out->valid = (s->flags & ARIADNA_NAV_VALID) != 0;

    return s_plane_az;
}

/**
 * Split the planar servo command back onto the two gimbal axes.
 *
 * The gimbal deflects the nozzle in a plane; commanding cos/sin of the
 * azimuth on the two axes aims that deflection along the guidance plane.
 * Total deflection is preserved, which matters because the planar law was
 * calibrated against a travel limit of DG_SERVO_MAX_DEG.
 */
static void decompose_tvc(float servo_deg, float az, float *tvc_x, float *tvc_y)
{
    *tvc_x = servo_deg * cosf(az);
    *tvc_y = servo_deg * sinf(az);
}

// Dead reckoning across a short link gap. Thrust and attitude are known
// locally — the clamp position and the grain clock are ours — so the state
// can be pushed forward well enough to keep the loop closed for a few tens
// of milliseconds. Drag is deliberately ignored: it would flatter the
// estimate, and this path exists to survive a gap, not to fly on.
static void nav_extrapolate(dg_nav_t *n, float dt, float thrust, float mass)
{
    float a  = (mass > 0.1f) ? thrust / mass : 0.0f;
    float ax = a * sinf(n->theta);
    float az = a * cosf(n->theta) - DG_G;
    n->h     += n->vz * dt + 0.5f * az * dt * dt;
    n->vx    += ax * dt;
    n->vz    += az * dt;
    n->theta += n->omega * dt;
    n->t     += dt;
}

// =====================================================================
//  7. FUTURE — controllable aerodynamic fins
//
//  Structure only. Nothing here is wired to an actuator and the mixer
//  returns zeros, so the fin fields in every actuator packet are zero and
//  the Master ignores them (ARIADNA_ACT_FINS_LIVE stays clear).
//
//  Why it exists now: the pipeline from a control decision to a servo
//  command crosses two boards and a packet format. Proving that path with
//  zeros costs nothing today and means adding fins later is a control
//  problem rather than a plumbing problem.
//
//  When you do implement it, the fins are only useful where the motor is
//  not — during the unpowered coast, where DG_PHASE_COAST currently parks
//  the TVC because a cold nozzle produces no torque. That is the gap fins
//  fill, and it is why the mixer is given the phase.
// =====================================================================
struct FinCommand {
    float deg[4];        // per-fin deflection, +ve = trailing edge to +yaw
    bool  live;          // false keeps the Master's fin outputs parked
};

class FinController {
public:
    void begin(void) { reset(); }

    void reset(void)
    {
        for (uint8_t i = 0; i < 4; i++) _cmd.deg[i] = 0.0f;
        _cmd.live = false;
    }

    /**
     * Mix a roll/pitch/yaw demand onto four fins.
     *
     * NOT IMPLEMENTED. Returns a parked command so the data path can be
     * exercised end to end without the fins ever moving.
     */
    const FinCommand &update(const dg_nav_t *nav, dg_phase_t phase)
    {
        (void)nav;
        (void)phase;
        reset();
        return _cmd;
    }

private:
    FinCommand _cmd;
};

static FinController s_fins;

// =====================================================================
//  8. COMMANDS
// =====================================================================
static void handle_command(const ariadna_cmd_t *c)
{
    switch (c->cmd) {
    case ARIADNA_CMD_ABORT:
        // Latching. Park the actuators, cut any pulse in progress, and
        // stop the guidance from asking for another. Only a power cycle
        // clears it.
        pyro_all_safe();
        dg_init();
        s_armed = false;
        NAV_PRINT("# ABORT\n");
        break;

    case ARIADNA_CMD_DISARM:
        pyro_all_safe();
        s_armed = false;
        break;

    case ARIADNA_CMD_ZERO_CLOCK:
        // Launch detected. Nothing on this board is referenced to the
        // mission clock yet; the hook exists so the Master's timestamps
        // and this board's telemetry can be aligned in the log.
        break;

    default:
        break;
    }
}

// =====================================================================
//  9. POWER-ON SELF TEST
//
//  Cheap, and it catches the failure that matters: a guidance module that
//  linked but does not compute. A failure here inhibits arming — the board
//  reports it and refuses to fly rather than flying with a broken table.
// =====================================================================
static bool self_test(void)
{
    bool ok = true;

    // Motor table sane at the plateau, and zero outside the burn.
    float f = dg_thrust_ref(0.30f);
    if (!(f > 200.0f && f < 250.0f)) {
        NAV_PRINT("POST thrust_ref(0.3)=%.1f\n", f);
        ok = false;
    }
    if (dg_thrust_ref(-0.1f) != 0.0f || dg_thrust_ref(1.5f) != 0.0f) {
        Serial.println("POST thrust_ref out of range not zero");
        ok = false;
    }

    // Guidance law: no horizontal error must command no tilt.
    float sn, cs;
    dg_guidance_tilt(30.0f, 0.0f, -40.0f, 3.0f, &sn, &cs);
    if (fabsf(sn) > 1e-4f) {
        NAV_PRINT("POST tilt bias %.5f\n", sn);
        ok = false;
    }

    // Ignition table monotone in entry speed and inside the arming altitude.
    float prev = 0.0f;
    for (float v = 20.0f; v <= 55.0f; v += 5.0f) {
        dg_init();
        dg_arm(0.0f, -v);
        float hi = dg_h_ignite();
        if (hi <= prev || hi > DG_H_START) {
            NAV_PRINT("POST h_ign(%.0f)=%.2f\n", v, hi);
            ok = false;
        }
        prev = hi;
    }

    // The 3D-to-planar bridge. These are the sign conventions that turn
    // the attitude loop into positive feedback if they are wrong, so they
    // are checked on every boot rather than trusted.
    {
        ariadna_nav_state_t s;
        memset(&s, 0, sizeof(s));
        dg_nav_t n;

        // Moving due East, thrust axis vertical: the plane must lie along
        // +x, the in-plane speed must come out positive, tilt zero.
        s_plane_locked = false;
        s_plane_az = 0.0f;
        s.vx = 10.0f; s.vy = 0.0f; s.vz = -30.0f;
        s.ux = 0.0f;  s.uy = 0.0f; s.uz = 1.0f;
        s.h = 50.0f;  s.flags = ARIADNA_NAV_VALID;
        float az = project_to_guidance_plane(&s, &n);
        if (fabsf(az) > 1e-3f || fabsf(n.vx - 10.0f) > 1e-3f ||
            fabsf(n.theta) > 1e-3f) {
            NAV_PRINT("POST plane east az=%.3f vx=%.2f th=%.4f\n", az, n.vx, n.theta);
            ok = false;
        }

        // Same speed due North: the plane must rotate 90 degrees and the
        // in-plane speed must be unchanged.
        s_plane_locked = false;
        s.vx = 0.0f; s.vy = 10.0f;
        az = project_to_guidance_plane(&s, &n);
        if (fabsf(az - (float)M_PI_2) > 1e-3f || fabsf(n.vx - 10.0f) > 1e-3f) {
            NAV_PRINT("POST plane north az=%.3f vx=%.2f\n", az, n.vx);
            ok = false;
        }

        // Leaning downrange along the plane must give POSITIVE theta.
        s_plane_locked = false;
        s.vx = 10.0f; s.vy = 0.0f;
        s.ux = 0.17365f; s.uy = 0.0f; s.uz = 0.98481f;    // 10 degrees toward +x
        az = project_to_guidance_plane(&s, &n);
        if (fabsf(n.theta - 0.17453f) > 2e-3f) {
            NAV_PRINT("POST plane tilt th=%.4f want 0.1745\n", n.theta);
            ok = false;
        }

        // Servo decomposition must preserve magnitude and point along the
        // plane. A swapped sin/cos passes a magnitude check but fails this.
        float tx, ty;
        decompose_tvc(4.0f, 0.0f, &tx, &ty);
        if (fabsf(tx - 4.0f) > 1e-3f || fabsf(ty) > 1e-3f) {
            NAV_PRINT("POST tvc east %.2f %.2f\n", tx, ty);
            ok = false;
        }
        decompose_tvc(4.0f, (float)M_PI_2, &tx, &ty);
        if (fabsf(tx) > 1e-3f || fabsf(ty - 4.0f) > 1e-3f) {
            NAV_PRINT("POST tvc north %.2f %.2f\n", tx, ty);
            ok = false;
        }
    }

    s_plane_locked = false;
    s_plane_az = 0.0f;
    dg_init();
    return ok;
}

// =====================================================================
//  10. CORE 0 — 200 Hz control
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

    Serial.begin(ARIADNA_CONSOLE_BAUD);      // pad console, internal USB-C

    MASTER_PORT.setTX(PIN_MASTER_TX);
    MASTER_PORT.setRX(PIN_MASTER_RX);
    MASTER_PORT.begin(ARIADNA_MCU_BAUD);
    // uart0 is Serial1 on the arduino-pico core. Must match MASTER_PORT.
    ariadna_uart_tx_begin(&s_tx, uart0);

    ariadna_rx_init(&s_rx);
    memset(&s_state,  0, sizeof(s_state));
    memset(&s_master, 0, sizeof(s_master));
    memset(&s_dg_nav, 0, sizeof(s_dg_nav));
    memset(&s_dg_out, 0, sizeof(s_dg_out));

    s_fins.begin();
    dg_init();
    s_post_pass = self_test();

    NAV_PRINT("\nAriadna V2 Navigation MCU — POST %s\n", s_post_pass ? "PASS" : "FAIL");
    NAV_PRINT("guidance entry %.0f m, servo limit %.1f deg\n",
              DG_H_START, DG_SERVO_MAX_DEG);

    s_ready = true;
}

void loop()
{
    static uint32_t next_us    = 0;
    static uint32_t next_status_ms = 0;
    static uint32_t next_pad_ms    = 0;
    const  uint32_t period     = (uint32_t)(1e6f / DG_CTRL_HZ);

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;          // not our slot yet
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) {           // missed a slot entirely
        s_overruns++;
        next_us = micros() + period;
    }
    uint32_t t0 = micros();

    // ---- link ---------------------------------------------------------
    ariadna_uart_tx_service(&s_tx);      // retire a finished DMA run first
    link_poll();

    bool state_fresh = false;
    if (s_have_state) {
        uint32_t age = micros() - s_state_us;
        if (age <= NAV_STALE_MS * 1000u) {
            state_fresh = true;
        } else if (age > NAV_HOLD_MS * 1000u) {
            s_dg_nav.valid = false;     // parks the TVC, freezes the clamp
        } else {
            nav_extrapolate(&s_dg_nav, 1.0f / DG_CTRL_HZ, s_dg_out.thrust_est,
                            DG_M_INITIAL - 0.5f * DG_M_PROP);
        }
    }

    float plane_az = s_plane_az;
    if (state_fresh) plane_az = project_to_guidance_plane(&s_state, &s_dg_nav);

    // ---- arming -------------------------------------------------------
    // Two conditions, both required: the Master says the vehicle is armed,
    // and the vehicle is actually falling through the guidance entry
    // altitude. The entry velocity latched here selects the ignition
    // altitude, so it must be taken on the way DOWN and only once.
    if (!s_armed && s_post_pass && s_have_state && s_dg_nav.valid &&
        pyro_permitted() &&
        s_dg_nav.h <= DG_H_START && s_dg_nav.vz < -2.0f) {
        dg_arm(s_dg_nav.vx, s_dg_nav.vz);
        s_armed = true;
        s_plane_locked = true;         // freeze the plane for the whole burn
        NAV_PRINT("# ARMED vx=%.1f vz=%.1f az=%.1f deg -> h_ign=%.2f m\n",
                  s_dg_nav.vx, s_dg_nav.vz, plane_az * (float)RAD_TO_DEG,
                  dg_h_ignite());
    }

    // ---- guidance -----------------------------------------------------
    if (s_have_state) {
        nav_publish(&s_dg_nav);
        dg_step_attitude(&s_dg_nav, &s_dg_out);
#if !NAV_USE_CORE1
        dg_step_planner(&s_dg_nav);
        s_plan_calls++;
#endif
    }

    // ---- ignition -----------------------------------------------------
    // dg_out.ignite is a level, so the rising edge is what fires. The
    // arming gate is re-checked inside pyro_fire() at the output stage.
    if (s_dg_out.ignite && !(s_fired_mask & 0x01)) {
        if (pyro_fire(0)) {
            s_ignite_ms  = millis();
            s_backup_due = true;
            NAV_PRINT("# IGNITE ch1 h=%.2f vz=%.1f\n", s_dg_nav.h, s_dg_nav.vz);
        }
    }

    // ---- backup igniter -----------------------------------------------
    //
    // Channel 1 fires, and then the vehicle either accelerates or it does
    // not. If it does not, the igniter failed and there is exactly one
    // thing left to try. This is the only automatic use of channel 2.
    //
    // The window is deliberately short. The grain burns for ~1.75 s and
    // the ignition altitude has no margin for a late light — waiting long
    // enough to be certain is the same as not firing at all.
    //
    // accel_mag includes gravity, so a vehicle in free fall reads ~0 g and
    // a burning motor reads several g. The threshold sits well above the
    // aerodynamic drag deceleration at terminal velocity.
    static const uint32_t BACKUP_WINDOW_MS = 250;
    static const float    IGNITION_DETECT_G = 2.0f;

    if (s_backup_due) {
        if (state_fresh && s_state.accel_mag >= IGNITION_DETECT_G) {
            s_backup_due = false;                       // it lit
        } else if (millis() - s_ignite_ms >= BACKUP_WINDOW_MS) {
            s_backup_due = false;
            if (pyro_fire(1))
                NAV_PRINT("# BACKUP ch2 — no thrust %lu ms after ch1\n",
                          (unsigned long)BACKUP_WINDOW_MS);
        }
    }

    pyro_service();
    pyro_scan_continuity();

    // ---- actuator command ---------------------------------------------
    {
        ariadna_actuator_t act;
        memset(&act, 0, sizeof(act));
        act.t_ms  = millis();
        act.clamp = s_dg_out.clamp;
        act.phase = (uint8_t)s_dg_out.phase;

        if (s_have_state && s_dg_nav.valid) {
            decompose_tvc(s_dg_out.servo_deg, plane_az,
                          &act.tvc_x_deg, &act.tvc_y_deg);
            act.flags |= ARIADNA_ACT_VALID;
        } else {
            // No usable state: park the gimbal at neutral. The clamp value
            // is still sent, and the Master holds it where it was.
            act.tvc_x_deg = 0.0f;
            act.tvc_y_deg = 0.0f;
            act.flags |= ARIADNA_ACT_TVC_PARK;
        }

        const FinCommand &fc = s_fins.update(&s_dg_nav, s_dg_out.phase);
        for (uint8_t i = 0; i < 4; i++) act.fin_deg[i] = fc.deg[i];
        if (fc.live) act.flags |= ARIADNA_ACT_FINS_LIVE;

        if (s_fired_mask) act.flags |= ARIADNA_ACT_IGNITE;

        link_send(ARIADNA_PKT_ACTUATOR, &act, sizeof(act));
    }

    // ---- status --------------------------------------------------------
    if (millis() >= next_status_ms) {
        next_status_ms = millis() + 1000 / ARIADNA_STATUS_HZ;

        ariadna_nav_status_t st;
        memset(&st, 0, sizeof(st));
        st.t_ms        = millis();
        st.p_plan      = s_dg_out.p_plan;
        st.thrust_est  = s_dg_out.thrust_est;
        st.h_ignite    = dg_h_ignite();
        st.theta_cmd   = s_dg_out.theta_cmd;
        st.theta_err   = s_dg_out.theta_err;
        st.plane_az    = plane_az;
        st.rx_ok       = s_rx.rx_ok;
        st.rx_bad      = s_rx.rx_crc + s_rx.rx_len;
        st.plan_iters  = s_plan_calls;
        st.loop_max_us = (uint16_t)(s_loop_max_us > 65535 ? 65535 : s_loop_max_us);
        st.overruns    = (uint16_t)(s_overruns > 65535 ? 65535 : s_overruns);
        st.phase       = (uint8_t)s_dg_out.phase;
        st.pyro_cont   = s_cont_mask;
        st.pyro_fired  = s_fired_mask;
        st.flags       = (uint8_t)((s_armed      ? ARIADNA_NST_ARMED      : 0) |
                                   (s_post_pass  ? ARIADNA_NST_POST_PASS  : 0) |
                                   (state_fresh  ? ARIADNA_NST_LINK_OK    : 0) |
                                   (s_plan_calls ? ARIADNA_NST_PLANNER_OK : 0) |
                                   ((s_cont_mask & 0x02) ? ARIADNA_NST_BACKUP_ARM : 0));

        link_send(ARIADNA_PKT_NAV_STATUS, &st, sizeof(st));
    }

    // ---- pad console ----------------------------------------------------
    // Last, and rate limited, so a slow USB host can never stretch a
    // control tick. printf is the most expensive call in this loop.
    if (millis() >= next_pad_ms) {
        next_pad_ms = millis() + 1000 / PAD_CONSOLE_HZ;
        NAV_PRINT("ph=%d h=%.1f vz=%.1f th=%.1f srv=%.2f clamp=%.2f "
                  "az=%.0f cont=%c%c%c rx=%lu/%lu loop=%luus ovr=%lu plan=%lu\n",
                  (int)s_dg_out.phase, s_dg_nav.h, s_dg_nav.vz,
                  s_dg_nav.theta * (float)RAD_TO_DEG, s_dg_out.servo_deg,
                  s_dg_out.clamp, plane_az * (float)RAD_TO_DEG,
                  (s_cont_mask & 1) ? '1' : '-',
                  (s_cont_mask & 2) ? '2' : '-',
                  (s_cont_mask & 4) ? '3' : '-',
                  (unsigned long)s_rx.rx_ok,
                  (unsigned long)(s_rx.rx_crc + s_rx.rx_len),
                  (unsigned long)s_loop_max_us,
                  (unsigned long)s_overruns,
                  (unsigned long)s_plan_calls);
    }

    uint32_t dt = micros() - t0;
    if (dt > s_loop_max_us) s_loop_max_us = dt;
}

// =====================================================================
//  11. CORE 1 — trajectory replanner
//
//  dg_step_planner() does ONE trajectory projection per call and keeps the
//  root find's bracket between calls. A full solve is about seven
//  projections, so it converges in ~35 ms — comfortably inside the 100 ms
//  replan period — without ever blocking for a whole solve at once. Pacing
//  it at 200 Hz gives that convergence with the core idle the rest of the
//  time; there is nothing to gain by spinning faster.
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
    next_us += period;

    dg_nav_t n;
    if (!nav_acquire(&n)) return;      // torn read: try again next tick
    dg_step_planner(&n);
    s_plan_calls++;
}
#endif
