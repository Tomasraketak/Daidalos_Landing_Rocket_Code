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
#include "ariadna_link.h"
#include "ariadna_uart_dma.h"
// Named here, not only inside ariadna_rgb.h: the Arduino builder adds a
// library's include path only when it sees a real #include of it, so the
// header's __has_include probe on its own always reads "absent" and the
// LED silently compiled to nothing. ARDUINO is undefined in the host
// `make syntax` build, which has no libraries.
#ifdef ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
#include "ariadna_rgb.h"

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
static dg_nav_t  s_dg_nav;      // 3-D state handed to the verified module
static dg_out_t  s_dg_out;
static bool      s_armed     = false;
static bool      s_post_pass = false;

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
    if (!isfinite(s->pz) || !isfinite(s->vx) || !isfinite(s->vy) ||
        !isfinite(s->vz) || !isfinite(s->ux) || !isfinite(s->uy) ||
        !isfinite(s->uz) || !isfinite(s->wx) || !isfinite(s->wy))
        return false;

    // px/py are checked for finiteness only. They are not in the control
    // path — the guidance law flies velocity to zero, not position to a
    // coordinate — so a wild horizontal position must not be allowed to
    // reject a frame whose altitude and attitude are perfectly good.
    if (!isfinite(s->px) || !isfinite(s->py))                return false;

    if (!isfinite(s->ax) || !isfinite(s->ay) || !isfinite(s->az))
        return false;

    if (s->pz < -20.0f || s->pz > 3000.0f)                   return false;
    if (fabsf(s->vx) > 120.0f || fabsf(s->vy) > 120.0f)      return false;
    if (fabsf(s->vz) > 200.0f)                               return false;
    if (fabsf(s->wx) > 35.0f  || fabsf(s->wy) > 35.0f)       return false;

    // The attitude vector must actually be a unit vector. A drifting norm
    // means the sender's quaternion is not being normalised, which would
    // silently scale every tilt angle this board computes.
    float n2 = s->ux * s->ux + s->uy * s->uy + s->uz * s->uz;
    if (n2 < 0.90f || n2 > 1.10f)                            return false;

    // Same test on the quaternion. It is not in the control path either,
    // but it goes into the log and the ground navball, and a quaternion
    // whose norm has drifted is the signature of a sender that has
    // stopped normalising — which would eventually reach u as well.
    float q2 = s->qw * s->qw + s->qx * s->qx +
               s->qy * s->qy + s->qz * s->qz;
    if (q2 < 0.90f || q2 > 1.10f)                            return false;

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
//  6. NAV STATE -> GUIDANCE STATE
//
//  Almost nothing to bridge. guidance.cpp solves the full three-dimensional
//  problem with a two-axis gimbal, and the link already carries most of
//  what it wants:
//
//    u  -> b   the thrust-axis unit vector, identically defined at both
//              ends (see the conventions block in ariadna_link.h)
//    w  -> w   angular rate in pad-ENU, straight from the gyro
//
//  The one thing it does not carry directly is the airframe's ROLL
//  ORIENTATION, and the guidance needs it: the two TVC servos are bolted
//  to the vehicle, so as it spins the direction each one pushes spins with
//  it. A controller that assumed those axes were anchored to the world
//  would steer into the wrong mixture of channels, smoothly, with no
//  symptom but a landing somewhere else.
//
//  It is one rotation away. The link carries the full body-to-pad
//  quaternion, so the reference axis is
//
//    g = R(q) * BODY_SERVO1_AXIS
//
//  the same construction the Position MCU already uses for u. Deriving it
//  here rather than adding it to the wire keeps ariadna_nav_state_t, the
//  link tests and the Python decoder untouched, and costs ~30 flops. An earlier version of this file
//  rotated the 3-D state into a vertical guidance plane, ran the planar
//  law inside it, and split the single servo command back across the two
//  gimbal axes by the plane azimuth. That worked, but it steered only
//  within the chosen plane and had to FREEZE the azimuth at low speed and
//  again at ignition, because re-aiming the plane mid-burn rotates the
//  attitude reference underneath a closed loop that assumes it is fixed.
//  The two-axis law has no such restriction: it controls both channels
//  directly, at any azimuth, for the whole burn.
//
//  It also removes the riskiest code that was in this file. The plane
//  rotation was four sign conventions deep, and a sign error there did not
//  produce a wrong number, it produced positive feedback.
//
//  What has NOT gone away: the two servo signs at the Master. Those are
//  still hardware conventions, and bench test 3 in the README exists to
//  catch them. Deflect the airframe by hand and confirm the nozzle swings
//  to OPPOSE the motion, in both axes independently.
// =====================================================================

// The body axis servo 1 deflects the nozzle in. This is a HARDWARE fact about the
// gimbal mount, and it must match the axis TVC_SIGN_X drives at the Master; if the
// two disagree by 90 degrees the channels are simply swapped.
static const float BODY_SERVO1_AXIS[3] = { 1.0f, 0.0f, 0.0f };

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
    out->t     = (float)s->t_ms * 1e-3f;
    out->valid = (s->flags & ARIADNA_NAV_VALID) != 0;
}

// Dead reckoning across a short link gap. Thrust and attitude are known
// locally -- the clamp position and the grain clock are ours -- so the state
// can be pushed forward well enough to keep the loop closed for a few tens
// of milliseconds. Drag is deliberately ignored: it would flatter the
// estimate, and this path exists to survive a gap, not to fly on.
static void nav_extrapolate(dg_nav_t *n, float dt, float thrust, float mass)
{
    float a  = (mass > 0.1f) ? thrust / mass : 0.0f;
    float az = a * n->bz - DG_G;
    n->h  += n->vz * dt + 0.5f * az * dt * dt;
    n->vx += a * n->bx * dt;
    n->vy += a * n->by * dt;
    n->vz += az * dt;
    // the body axes are carried by the rate: db/dt = w x b, dg/dt = w x g, then
    // renormalised so they stay unit vectors however long the gap runs
    float nx = n->bx + (n->wy * n->bz - n->wz * n->by) * dt;
    float ny = n->by + (n->wz * n->bx - n->wx * n->bz) * dt;
    float nz = n->bz + (n->wx * n->by - n->wy * n->bx) * dt;
    float inv = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    n->bx = nx * inv;  n->by = ny * inv;  n->bz = nz * inv;
    // The reference axis MUST ride the same rotation. Across a 250 ms gap at 90 deg/s
    // the airframe turns 22 degrees; a frozen g would hand the loop a stale roll
    // orientation and mix the two servo channels by that angle.
    float mx = n->gx + (n->wy * n->gz - n->wz * n->gy) * dt;
    float my = n->gy + (n->wz * n->gx - n->wx * n->gz) * dt;
    float mz = n->gz + (n->wx * n->gy - n->wy * n->gx) * dt;
    float gd = mx * n->bx + my * n->by + mz * n->bz;         // re-orthogonalise
    mx -= gd * n->bx;  my -= gd * n->by;  mz -= gd * n->bz;
    float gi = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    n->gx = mx * gi;  n->gy = my * gi;  n->gz = mz * gi;
    n->t += dt;
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
//  9. ANNUNCIATOR
//
//  The XIAO's own WS2812, reporting the one thing this board knows that
//  nothing else in the vehicle does: whether it is READY TO FIRE A PYRO.
//
//  That is worth a dedicated LED. The Master's annunciator reports the
//  mission state and the Position MCU's reports the navigation solution;
//  neither can tell you that the igniter has continuity, that the
//  guidance planner is still converging, or that this board has stopped
//  receiving a navigation state and has therefore parked the TVC.
//
//    solid red         self test failed. Do not arm.
//    red, 2 flashes    armed, but channel 1 has NO CONTINUITY. The
//                      igniter is open, or the wrong side of a connector
//                      is in your hand.
//    slow blue         alive, waiting for a navigation state
//    slow amber        navigation state arriving, not armed
//    slow green        armed and receiving. Continuity on channel 1.
//    fast green        the landing burn is running
//    magenta fast      the navigation state has gone stale and the TVC is
//                      parked. This is the failure the link supervision
//                      exists to make visible.
//
//  ---------------------------------------------------------------------
//  WHY THIS IS SAFE INSIDE A 200 Hz CONTROL LOOP
//  ---------------------------------------------------------------------
//  Because ariadna_rgb_service() only touches the LED when the computed
//  pixel value has changed. A solid colour costs one 30 microsecond
//  transfer and then nothing; the fastest pattern here costs two
//  transfers per second. It is called at the very end of the tick, after
//  the actuator packet is already on the wire, so even that cost lands in
//  the slack rather than in the control path.
// =====================================================================
static ariadna_rgb_t s_rgb;

static const uint8_t NAV_RGB_BRIGHTNESS = 60;

/**
 * Choose the annunciator's message.
 *
 * `fresh` is the link supervisor's verdict, passed in rather than
 * recomputed so the LED and the actuator packet cannot disagree about
 * whether the navigation state is usable.
 */
static void annunciate(bool fresh, bool stale_parked)
{
    if (!s_post_pass) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_SOLID, 0);
        return;
    }
    if (stale_parked) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_MAGENTA, ARIADNA_PAT_FAST, 0);
        return;
    }
    if (s_armed && !(s_cont_mask & 0x01)) {
        // Armed with an open igniter. The single most useful thing this
        // board can tell somebody standing next to the vehicle.
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_BLINK_N, 2);
        return;
    }
    if (s_dg_out.phase == DG_PHASE_BURN) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_FAST, 0);
        return;
    }
    if (!fresh) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SLOW, 0);
        return;
    }
    if (s_armed) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_SLOW, 0);
        return;
    }
    ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_SLOW, 0);
}

// =====================================================================
//  10. POWER-ON SELF TEST
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

    // The nav-state copy and the two-axis guidance. A sign or axis error
    // here turns the attitude loop into positive feedback, so it is checked
    // on every boot rather than trusted.
    {
        ariadna_nav_state_t s;
        memset(&s, 0, sizeof(s));
        dg_nav_t n;

        // The copy must be a copy: u -> b, w -> w, with nothing rotated.
        s.pz = 50.0f;  s.qw = 1.0f;
        s.vx = 10.0f;  s.vy = -4.0f;  s.vz = -30.0f;
        s.ux = 0.17365f; s.uy = 0.0f; s.uz = 0.98481f;   // 10 deg toward +x
        s.wx = 0.2f;   s.wy = -0.1f;  s.wz = 0.05f;
        s.flags = ARIADNA_NAV_VALID;
        nav_from_state(&s, &n);
        if (fabsf(n.vx - s.vx) > 1e-6f || fabsf(n.vy - s.vy) > 1e-6f ||
            fabsf(n.bx - s.ux) > 1e-6f || fabsf(n.bz - s.uz) > 1e-6f ||
            fabsf(n.wy - s.wy) > 1e-6f || n.h != s.pz || !n.valid) {
            NAV_PRINT("POST nav copy mismatch\n");
            ok = false;
        }

        // The two horizontal channels must behave identically. Guidance for
        // a due-East drift must command exactly what a due-North drift does,
        // rotated 90 degrees -- a swapped or mis-signed axis shows up here
        // and nowhere else until the vehicle is in the air.
        float ax, ay, az2, bx2, by2, bz2;
        dg_guidance_dir(30.0f, 8.0f, 0.0f, -40.0f, 3.0f, &ax, &ay, &az2);
        dg_guidance_dir(30.0f, 0.0f, 8.0f, -40.0f, 3.0f, &bx2, &by2, &bz2);
        if (fabsf(ax - by2) > 1e-5f || fabsf(ay) > 1e-6f ||
            fabsf(bx2) > 1e-6f || fabsf(az2 - bz2) > 1e-6f) {
            NAV_PRINT("POST horizontal channels not symmetric\n");
            ok = false;
        }
        // and drifting East must command a tilt toward WEST, not East
        if (ax >= 0.0f) {
            NAV_PRINT("POST tilt sign wrong: drift +x commanded ux=%.4f\n", ax);
            ok = false;
        }

        // The gimbal basis must be orthonormal, or the two servo commands are
        // projections onto the wrong axes.
        float u1x, u1y, u1z, u2x, u2y, u2z;
        dg_gimbal_basis(0.12f, 0.05f, 0.99f, 0.9f, 0.1f, -0.1f,
                        &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
        float dotp = u1x * u2x + u1y * u2y + u1z * u2z;
        float nrm1 = u1x * u1x + u1y * u1y + u1z * u1z;
        if (fabsf(dotp) > 1e-4f || fabsf(nrm1 - 1.0f) > 1e-4f) {
            NAV_PRINT("POST gimbal basis dot=%.6f norm=%.6f\n", dotp, nrm1);
            ok = false;
        }

        // ---- the roll case ----
        // The gimbal axes must FOLLOW the airframe. Roll the reference axis a
        // quarter turn about a vertical thrust axis and the servo axes must go
        // with it. A basis anchored to the world passes every other check on this
        // board and fails only this one -- and the symptom in flight would be a
        // vehicle steering steadily off-target with no error signal to show for it.
        dg_gimbal_basis(0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
                        &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
        if (fabsf(u1y - 1.0f) > 1e-4f || fabsf(u2x + 1.0f) > 1e-4f) {
            NAV_PRINT("POST gimbal axes do not follow roll: u1y=%.4f u2x=%.4f\n",
                      u1y, u2x);
            ok = false;
        }

        // The reference axis must come out of the quaternion the right way: at
        // identity attitude it is the body servo-1 axis unrotated.
        ariadna_nav_state_t q0;
        memset(&q0, 0, sizeof(q0));
        q0.qw = 1.0f;
        float gv[3];
        quat_rotate(&q0, BODY_SERVO1_AXIS, gv);
        if (fabsf(gv[0] - 1.0f) > 1e-5f || fabsf(gv[1]) > 1e-5f ||
            fabsf(gv[2]) > 1e-5f) {
            NAV_PRINT("POST quat identity g=(%.3f %.3f %.3f)\n", gv[0], gv[1], gv[2]);
            ok = false;
        }
        // ...and a 90-degree yaw about pad +z must carry it onto pad +y
        memset(&q0, 0, sizeof(q0));
        q0.qw = 0.70710678f;  q0.qz = 0.70710678f;
        quat_rotate(&q0, BODY_SERVO1_AXIS, gv);
        if (fabsf(gv[0]) > 1e-4f || fabsf(gv[1] - 1.0f) > 1e-4f) {
            NAV_PRINT("POST quat yaw90 g=(%.3f %.3f %.3f) want (0 1 0)\n",
                      gv[0], gv[1], gv[2]);
            ok = false;
        }
    }

    dg_init();
    return ok;
}

// =====================================================================
//  11. CORE 0 — 200 Hz control
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

    // Up before the self test, so a board that fails one has a colour.
    ariadna_rgb_begin(&s_rgb, NAV_RGB_BRIGHTNESS, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

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

    if (state_fresh) nav_from_state(&s_state, &s_dg_nav);

    // ---- arming -------------------------------------------------------
    // Two conditions, both required: the Master says the vehicle is armed,
    // and the vehicle is actually falling through the guidance entry
    // altitude. The entry velocity latched here selects the ignition
    // altitude, so it must be taken on the way DOWN and only once.
    if (!s_armed && s_post_pass && s_have_state && s_dg_nav.valid &&
        pyro_permitted() &&
        s_dg_nav.h <= DG_H_START && s_dg_nav.vz < -2.0f) {
        float vh = sqrtf(s_dg_nav.vx * s_dg_nav.vx + s_dg_nav.vy * s_dg_nav.vy);
        dg_arm(vh, s_dg_nav.vz);
        s_armed = true;
        NAV_PRINT("# ARMED v_h=%.1f vz=%.1f -> h_ign=%.2f m\n",
                  vh, s_dg_nav.vz, dg_h_ignite());
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
            // The guidance already resolved the deflection onto the two
            // gimbal axes; axis 1 is the world-x plane, axis 2 the world-y
            // plane. Nothing left to rotate.
            act.tvc_x_deg = s_dg_out.servo1_deg;
            act.tvc_y_deg = s_dg_out.servo2_deg;
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
        st.tilt_cmd    = s_dg_out.tilt_cmd;
        st.tilt_err    = s_dg_out.tilt_err;
        st.roll_rate   = s_dg_out.roll_rate;
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
        float bzc = s_dg_nav.bz > 1.0f ? 1.0f
                                       : (s_dg_nav.bz < -1.0f ? -1.0f : s_dg_nav.bz);
        NAV_PRINT("ph=%d h=%.1f vz=%.1f tilt=%.1f roll=%.0f srv=%.2f/%.2f clamp=%.2f "
                  "cont=%c%c%c rx=%lu/%lu loop=%luus ovr=%lu plan=%lu\n",
                  (int)s_dg_out.phase, s_dg_nav.h, s_dg_nav.vz,
                  acosf(bzc) * (float)RAD_TO_DEG,
                  s_dg_out.roll_rate * (float)RAD_TO_DEG,
                  s_dg_out.servo1_deg, s_dg_out.servo2_deg,
                  s_dg_out.clamp,
                  (s_cont_mask & 1) ? '1' : '-',
                  (s_cont_mask & 2) ? '2' : '-',
                  (s_cont_mask & 4) ? '3' : '-',
                  (unsigned long)s_rx.rx_ok,
                  (unsigned long)(s_rx.rx_crc + s_rx.rx_len),
                  (unsigned long)s_loop_max_us,
                  (unsigned long)s_overruns,
                  (unsigned long)s_plan_calls);
    }

    // ---- annunciator ----------------------------------------------------
    // Very last, after the actuator packet is already on the wire. See
    // the section 9 header for why this is affordable at 200 Hz.
    annunciate(state_fresh, s_have_state && !s_dg_nav.valid);
    ariadna_rgb_service(&s_rgb);

    uint32_t dt = micros() - t0;
    if (dt > s_loop_max_us) s_loop_max_us = dt;
}

// =====================================================================
//  12. CORE 1 — trajectory replanner
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
