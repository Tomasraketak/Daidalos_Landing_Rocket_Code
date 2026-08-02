// =====================================================================
//  Daidalos — propulsive landing flight computer
//
//  Target: RP2350 (Raspberry Pi Pico 2 / Pico 2 W) with the
//          arduino-pico core (earlephilhower), board "Raspberry Pi Pico 2".
//
//  This sketch is the I/O shell. It owns pins, timing, the link to the
//  navigation processor and the failsafes. It owns NO guidance maths —
//  that all lives in guidance.cpp, which has no Arduino dependency and is
//  verified on a workstation against the simulation it was ported from
//  (`cd test && make`). Keeping the split means a change to the algorithm
//  can be tested before it is flown.
//
//  What the guidance produces, twice per 10 ms tick:
//    * TWO TVC servo angles — where to point the nozzle, in 3-D
//    * a thrust-clamp value — how much of the grain's thrust to keep
//
//  The gimbal has two axes. Roll about the vehicle's own long axis has no
//  actuator — fin control is future work — so the airframe simply spins at
//  whatever rate it inherited, up to 90 deg/s, and the guidance is built to
//  fly through that. The servo axes spin with it, which is why the nav
//  frame carries a second body-fixed vector. See the top of guidance.h.
//
//  Core allocation:
//    core 0  200 Hz attitude loop, ignition decision, actuators, telemetry
//    core 1  trajectory replanner, one solver iteration per call so the
//            load is spread instead of arriving as a spike every 100 ms
//
//  ---------------------------------------------------------------------
//  READ BEFORE FLYING
//  ---------------------------------------------------------------------
//  This is unflown software. It has been verified in simulation and by
//  numerical equivalence against that simulation; it has not been through
//  hardware-in-the-loop or a flight. Everything below — pin map, servo
//  travel, igniter timing, packet format, sign conventions — describes an
//  INTENDED vehicle and must be checked against yours before the igniter
//  is ever connected. Bench-test with PIN_ARM open and no motor installed.
// =====================================================================

#include "guidance.h"
#include <Arduino.h>
#include <string.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE CONFIGURATION — check every line of this against the vehicle
// =====================================================================

// ---- pins -----------------------------------------------------------
static const uint8_t PIN_TVC1    = 2;   // TVC servo, x channel
static const uint8_t PIN_TVC2    = 4;   // TVC servo, y channel
static const uint8_t PIN_CLAMP   = 3;   // thrust-clamp servo signal
static const uint8_t PIN_IGNITER = 6;   // igniter MOSFET gate (LOW = safe)
static const uint8_t PIN_ARM     = 7;   // arming switch, LOW = armed (pull-up)
static const uint8_t PIN_LED     = LED_BUILTIN;

static const uint8_t PIN_NAV_TX  = 8;   // UART1 TX -> nav processor RX
static const uint8_t PIN_NAV_RX  = 9;   // UART1 RX <- nav processor TX
#define NAV_PORT     Serial2            // UART1 on the arduino-pico core
static const uint32_t NAV_BAUD = 460800;

// ---- servo pulse generation ----------------------------------------
// Pulses come from the RP2350's PWM slices rather than from the Servo
// library, because the library refreshes at a fixed 50 Hz. A 50 Hz command
// path adds up to 20 ms of dead time to a loop running at 200 Hz, which is
// enough phase lag to matter at a 9 rad/s bandwidth. Digital servos will
// take 333 Hz; drop SERVO_HZ to 50 for analogue ones and expect the loop
// to feel softer.
static const uint32_t SERVO_HZ    = 333;
static const uint32_t SERVO_RANGE = 10000;          // PWM counts per period
static const float    US_TO_COUNT = (float)SERVO_HZ * (float)SERVO_RANGE * 1e-6f;

// TVC servo calibration, PER AXIS. The trim centres the NOZZLE, not the servo
// horn — set each with the motor installed and the nozzle visibly straight.
// The sign makes a POSITIVE dg command deflect the nozzle the way guidance.cpp
// assumes: axis 1 in the vertical plane containing world +x, axis 2 in the plane
// containing world +y.
//
// Get either sign backwards and that channel is positive feedback: it diverges in
// well under a second. Verify BOTH by hand before flight — push the nose toward
// +x, axis 1 must swing to push it back; push it toward +y, axis 2 must. Testing
// only one axis is how a sign error survives to the pad.
static const float TVC1_TRIM_US   = 1500.0f;
static const float TVC2_TRIM_US   = 1500.0f;
static const float TVC_US_PER_DEG = 10.0f;          // typical for a 90 deg/1000 us servo
static const float TVC1_SIGN      = +1.0f;
static const float TVC2_SIGN      = +1.0f;

// Clamp servo calibration: pulse widths at fully diverted and fully open.
static const float CLAMP_US_CLOSED = 1150.0f;       // clamp = DG_CLAMP_MIN
static const float CLAMP_US_OPEN   = 1900.0f;       // clamp = 1.0

// ---- igniter --------------------------------------------------------
static const uint32_t IGNITER_PULSE_MS = 600;       // continuity current duration

// ---- link supervision ----------------------------------------------
// A missed packet or two must not stall a 200 Hz loop, so a short gap is
// covered by dead reckoning. Past NAV_HOLD_MS the state is declared
// invalid, which parks the servo and FREEZES the clamp where it is. That
// is the right failure: shutting the clamp during a burn guarantees a
// crash, and the grain is going to finish burning either way.
static const uint32_t NAV_STALE_MS = 60;            // start extrapolating
static const uint32_t NAV_HOLD_MS  = 250;           // give up

// ---- options --------------------------------------------------------
#define DG_USE_CORE1   1                            // 0 = run the planner inline
#define TELEM_HZ       20

// Every diagnostic print goes through this. printf's varargs promote float to
// double by language rule, and on the RP2350 double is software-emulated — so
// the promotion is a real cost, not a pedantic warning, and it is confined
// here on purpose. Silencing it locally keeps -Wdouble-promotion meaningful
// everywhere else, where a promoted expression in the control path would be a
// genuine performance bug.
#define DG_PRINT(...)                                             \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        Serial.printf(__VA_ARGS__);                               \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

// =====================================================================
//  2. NAVIGATION LINK
//
//  Binary frame from the fusion processor, little-endian, 61 bytes:
//
//    offset  size  field
//    0       2     sync, 0xDA 0x1D
//    2       1     flags: bit0 = solution valid
//    3       4     h              m, above the pad
//    7       4     vx             m/s, world +x
//    11      4     vy             m/s, world +y
//    15      4     vz             m/s, NEGATIVE while descending
//    19      12    bx, by, bz     THRUST-AXIS unit vector, world frame
//    31      12    gx, gy, gz     BODY-FIXED reference axis, world frame
//    43      12    wx, wy, wz     rad/s, world frame, straight off the GYRO
//    55      4     t_ms           u32 milliseconds, monotonic
//    59      2     CRC-16/CCITT-FALSE over bytes 2..58
//
//  ATTITUDE IS TWO UNIT VECTORS, not Euler angles. b is the direction
//  thrust pushes the vehicle — roughly up during the descent. g is a
//  second, body-fixed axis marking the airframe's ROLL orientation: it is
//  what tells the guidance where the two servo axes have got to as the
//  vehicle spins. Send g as the body axis servo 1 deflects the nozzle in.
//  A quaternion gives both in one rotation each.
//
//  w is the angular velocity in the same world frame; rotate the gyro out
//  of body coordinates with the same quaternion. Its component along b is
//  the roll rate, which this vehicle cannot control — send it anyway, the
//  guidance projects it out of the rate loop and uses it to cancel the
//  gyroscopic cross-coupling the spin produces.
//
//  w must be the MEASURED rate. Differencing the attitude estimate instead
//  is the single most damaging thing that can be done to this loop — in
//  simulation it drops the landing rate from 75.6 % to 1.3 %, because
//  differentiated estimator noise saturates the servos.
// =====================================================================

static const uint8_t  NAV_SYNC0 = 0xDA;
static const uint8_t  NAV_SYNC1 = 0x1D;
static const uint8_t  NAV_PAYLOAD = 57;    // flags..t_ms
static const uint8_t  NAV_FRAME   = NAV_PAYLOAD + 4;

static uint16_t crc16_ccitt(const uint8_t *p, uint8_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline float rd_f32(const uint8_t *p)
{
    float v; memcpy(&v, p, 4); return v;      // both ends are little-endian ARM
}

// Byte-at-a-time frame assembler. Never blocks and never waits for a whole
// frame, so a truncated packet costs one control tick and nothing else.
static uint8_t  s_rx[NAV_FRAME];
static uint8_t  s_rx_n   = 0;
static uint32_t s_rx_ok  = 0;
static uint32_t s_rx_bad = 0;

struct nav_rx_t {
    float    h, vx, vy, vz;
    float    bx, by, bz;
    float    gx, gy, gz;
    float    wx, wy, wz;
    uint32_t t_ms;
    bool     valid;
};

static bool nav_poll(nav_rx_t *out)
{
    bool got = false;
    while (NAV_PORT.available()) {
        uint8_t b = (uint8_t)NAV_PORT.read();

        if (s_rx_n == 0) { if (b != NAV_SYNC0) continue; }
        else if (s_rx_n == 1 && b != NAV_SYNC1) {
            s_rx_n = (b == NAV_SYNC0) ? 1 : 0;   // resync on a repeated lead byte
            s_rx[0] = NAV_SYNC0;
            continue;
        }
        s_rx[s_rx_n++] = b;
        if (s_rx_n < NAV_FRAME) continue;
        s_rx_n = 0;

        uint16_t want = (uint16_t)s_rx[59] | ((uint16_t)s_rx[60] << 8);
        if (crc16_ccitt(&s_rx[2], NAV_PAYLOAD) != want) { s_rx_bad++; continue; }

        out->valid = (s_rx[2] & 0x01) != 0;
        out->h     = rd_f32(&s_rx[3]);
        out->vx    = rd_f32(&s_rx[7]);
        out->vy    = rd_f32(&s_rx[11]);
        out->vz    = rd_f32(&s_rx[15]);
        out->bx    = rd_f32(&s_rx[19]);
        out->by    = rd_f32(&s_rx[23]);
        out->bz    = rd_f32(&s_rx[27]);
        out->gx    = rd_f32(&s_rx[31]);
        out->gy    = rd_f32(&s_rx[35]);
        out->gz    = rd_f32(&s_rx[39]);
        out->wx    = rd_f32(&s_rx[43]);
        out->wy    = rd_f32(&s_rx[47]);
        out->wz    = rd_f32(&s_rx[51]);
        memcpy(&out->t_ms, &s_rx[55], 4);
        s_rx_ok++;
        got = true;                              // keep draining: use the newest
    }
    return got;
}

// A frame that survives the CRC can still be nonsense. Anything outside the
// flight envelope is treated as a link fault rather than as a state.
static bool nav_sane(const nav_rx_t *n)
{
    float bn = n->bx * n->bx + n->by * n->by + n->bz * n->bz;
    float gn = n->gx * n->gx + n->gy * n->gy + n->gz * n->gz;
    float wn = n->wx * n->wx + n->wy * n->wy + n->wz * n->wz;
    // g must be a unit vector and must not have collapsed onto b, or the roll
    // orientation it is supposed to carry is not in it
    float gdb = n->gx * n->bx + n->gy * n->by + n->gz * n->bz;
    return isfinite(n->h) && isfinite(n->vx) && isfinite(n->vy) && isfinite(n->vz)
        && isfinite(bn) && isfinite(gn) && isfinite(wn)
        && n->h > -5.0f && n->h < 500.0f
        && fabsf(n->vx) < 120.0f && fabsf(n->vy) < 120.0f && fabsf(n->vz) < 160.0f
        // b must be a unit vector pointing up: a sender that forgot to normalise,
        // or that has the vehicle inverted, is a link fault and not a state
        && bn > 0.9f && bn < 1.1f && n->bz > 0.03f
        && gn > 0.9f && gn < 1.1f && fabsf(gdb) < 0.7f
        && wn < 400.0f;
}

// =====================================================================
//  3. SHARED STATE (core 0 writes, core 1 reads)
//
//  A seqlock, not a mutex: the 200 Hz loop must never block on the
//  planner. The writer bumps an odd/even counter around the copy and the
//  reader retries if it saw a torn window. On a two-core M33 with no cache
//  this is a handful of instructions.
// =====================================================================
static volatile uint32_t s_nav_seq = 0;
static dg_nav_t          s_nav_shared;
static volatile bool     s_ready = false;      // core 1 must not run before init

static inline void nav_publish(const dg_nav_t *n)
{
    s_nav_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_nav_shared = *n;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_nav_seq++;
}

static inline bool nav_acquire(dg_nav_t *n)
{
    uint32_t a = s_nav_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *n = s_nav_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_nav_seq == a;
}

// =====================================================================
//  4. ACTUATORS
// =====================================================================
static inline void pulse_us(uint8_t pin, float us)
{
    if (us < 800.0f)  us = 800.0f;             // hard travel guard: a bad
    if (us > 2200.0f) us = 2200.0f;            // command must not stall a servo
    analogWrite(pin, (int)(us * US_TO_COUNT + 0.5f));
}

static inline void tvc_write(float s1_deg, float s2_deg)
{
    pulse_us(PIN_TVC1, TVC1_TRIM_US + TVC1_SIGN * s1_deg * TVC_US_PER_DEG);
    pulse_us(PIN_TVC2, TVC2_TRIM_US + TVC2_SIGN * s2_deg * TVC_US_PER_DEG);
}

static inline void clamp_write(float k)
{
    float f = (k - DG_CLAMP_MIN) / (DG_CLAMP_MAX - DG_CLAMP_MIN);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    pulse_us(PIN_CLAMP, CLAMP_US_CLOSED + f * (CLAMP_US_OPEN - CLAMP_US_CLOSED));
}

// =====================================================================
//  5. FLIGHT STATE
// =====================================================================
static dg_nav_t  s_nav;                  // newest accepted (or extrapolated) state
static dg_out_t  s_out;
static uint32_t  s_nav_us      = 0;      // micros() of the last accepted frame
static bool      s_have_nav    = false;
static bool      s_armed       = false;
static uint32_t  s_ign_start   = 0;
static bool      s_ign_active  = false;
static bool      s_ign_fired   = false;

static uint32_t  s_loop_max_us = 0;
static uint32_t  s_overruns    = 0;
static uint32_t  s_plan_calls  = 0;

// Dead reckoning across a short link gap. Thrust and attitude are known
// locally — the clamp position and the grain clock are ours — so the state
// can be pushed forward well enough to keep the loop closed for a few tens
// of milliseconds. Drag is deliberately ignored: it would flatter the
// estimate, and this path exists to survive a gap, not to fly on.
static void nav_extrapolate(dg_nav_t *n, float dt, float thrust, float mass)
{
    float a = (mass > 0.1f) ? thrust / mass : 0.0f;
    float az = a * n->bz - DG_G;
    n->h  += n->vz * dt + 0.5f * az * dt * dt;
    n->vx += a * n->bx * dt;
    n->vy += a * n->by * dt;
    n->vz += az * dt;
    // the body axis is carried by the rate: db/dt = w x b, then renormalised so
    // it stays a unit vector however long the gap runs
    float nx = n->bx + (n->wy * n->bz - n->wz * n->by) * dt;
    float ny = n->by + (n->wz * n->bx - n->wx * n->bz) * dt;
    float nz = n->bz + (n->wx * n->by - n->wy * n->bx) * dt;
    float inv = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    n->bx = nx * inv; n->by = ny * inv; n->bz = nz * inv;
    // The reference axis rides the same rotation, and it MUST: across a 250 ms gap at
    // 90 deg/s the airframe turns 22 degrees, so a frozen g would hand the loop a
    // stale roll orientation and mix the two servo channels by that angle.
    float mx = n->gx + (n->wy * n->gz - n->wz * n->gy) * dt;
    float my = n->gy + (n->wz * n->gx - n->wx * n->gz) * dt;
    float mz = n->gz + (n->wx * n->gy - n->wy * n->gx) * dt;
    float gd = mx * n->bx + my * n->by + mz * n->bz;      // re-orthogonalise
    mx -= gd * n->bx; my -= gd * n->by; mz -= gd * n->bz;
    float gi = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    n->gx = mx * gi; n->gy = my * gi; n->gz = mz * gi;
    n->t += dt;
}

// =====================================================================
//  6. POWER-ON SELF TEST
//
//  Cheap, and it catches the failure that matters: a guidance module that
//  linked but does not compute. Anything wrong here inhibits arming.
// =====================================================================
static bool self_test(void)
{
    bool ok = true;

    // motor table sane at the plateau
    float f = dg_thrust_ref(0.30f);
    if (!(f > 200.0f && f < 250.0f)) { DG_PRINT("POST thrust_ref(0.3)=%.1f\n", f); ok = false; }
    if (dg_thrust_ref(-0.1f) != 0.0f || dg_thrust_ref(1.5f) != 0.0f) {
        Serial.println("POST thrust_ref out of range not zero"); ok = false;
    }

    // gimbal axes must FOLLOW the airframe: roll the reference axis through a
    // quarter turn and the servo axes must go with it. A basis anchored to the
    // world passes every other check here and fails only this one.
    {
        float a1x, a1y, a1z, a2x, a2y, a2z;
        dg_gimbal_basis(0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
                        &a1x, &a1y, &a1z, &a2x, &a2y, &a2z);
        if (fabsf(a1y - 1.0f) > 1e-4f || fabsf(a2x + 1.0f) > 1e-4f) {
            DG_PRINT("POST gimbal axes do not follow roll: %.4f %.4f\n", a1y, a2x);
            ok = false;
        }
    }

    // guidance law: no horizontal error must command no tilt, and the two
    // horizontal channels must behave identically -- a swapped or mis-signed
    // axis shows up here and nowhere else until the vehicle is in the air
    float ux, uy, uz;
    dg_guidance_dir(30.0f, 0.0f, 0.0f, -40.0f, 3.0f, &ux, &uy, &uz);
    if (fabsf(ux) > 1e-4f || fabsf(uy) > 1e-4f) {
        DG_PRINT("POST tilt bias %.5f %.5f\n", ux, uy); ok = false;
    }
    float ax, ay, az2, bx2, by2, bz2;
    dg_guidance_dir(30.0f, 8.0f, 0.0f, -40.0f, 3.0f, &ax, &ay, &az2);
    dg_guidance_dir(30.0f, 0.0f, 8.0f, -40.0f, 3.0f, &bx2, &by2, &bz2);
    if (fabsf(ax - by2) > 1e-5f || fabsf(ay) > 1e-6f || fabsf(bx2) > 1e-6f
        || fabsf(az2 - bz2) > 1e-6f) {
        DG_PRINT("POST horizontal channels not symmetric\n"); ok = false;
    }
    // gimbal basis must be orthonormal, or the two servo commands are wrong
    float u1x, u1y, u1z, u2x, u2y, u2z;
    dg_gimbal_basis(0.12f, 0.05f, 0.99f, 0.9f, 0.1f, -0.1f,
                    &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
    float dotp = u1x * u2x + u1y * u2y + u1z * u2z;
    float n1 = u1x * u1x + u1y * u1y + u1z * u1z;
    if (fabsf(dotp) > 1e-4f || fabsf(n1 - 1.0f) > 1e-4f) {
        DG_PRINT("POST gimbal basis dot %.6f norm %.6f\n", dotp, n1); ok = false;
    }

    // ignition table monotone and inside the arming altitude
    float prev = 0.0f;
    for (float v = 20.0f; v <= 55.0f; v += 5.0f) {
        dg_init(); dg_arm(0.0f, -v);
        float hi = dg_h_ignite();
        if (hi <= prev || hi > DG_H_START) {
            DG_PRINT("POST h_ign(%.0f)=%.2f\n", v, hi); ok = false;
        }
        prev = hi;
    }

    dg_init();
    return ok;
}

// =====================================================================
//  7. CORE 0 — 200 Hz control
// =====================================================================
void setup()
{
    pinMode(PIN_IGNITER, OUTPUT);
    digitalWrite(PIN_IGNITER, LOW);        // before anything else
    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    Serial.begin(921600);

    analogWriteFreq(SERVO_HZ);
    analogWriteRange(SERVO_RANGE);
    pinMode(PIN_TVC1, OUTPUT);
    pinMode(PIN_TVC2, OUTPUT);
    pinMode(PIN_CLAMP, OUTPUT);
    tvc_write(0.0f, 0.0f);
    clamp_write(0.6f);                     // mid travel: neither extreme on the pad

    NAV_PORT.setTX(PIN_NAV_TX);
    NAV_PORT.setRX(PIN_NAV_RX);
    NAV_PORT.begin(NAV_BAUD);

    dg_init();
    bool post = self_test();
    DG_PRINT("\nDaidalos landing guidance — POST %s\n", post ? "PASS" : "FAIL");
    Serial.println("t_ms,phase,h,vx,vy,vz,tilt,tilt_cmd,tilt_err,wmag,roll_dps,"
                   "servo1,servo2,clamp,thrust,p,rx_ok,rx_bad,loop_us,overrun,plan");

    if (!post) {                           // refuse to fly, and say so
        for (;;) { digitalWrite(PIN_LED, millis() % 200 < 100); delay(10); }
    }
    s_ready = true;
}

void loop()
{
    static uint32_t next_us  = 0;
    static uint32_t next_tlm = 0;
    const  uint32_t period   = (uint32_t)(1e6f / DG_CTRL_HZ);

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;     // not our slot yet
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) {      // missed a slot entirely
        s_overruns++;
        next_us = micros() + period;
    }
    uint32_t t0 = micros();

    // ---- link ---------------------------------------------------------
    nav_rx_t rx;
    bool fresh = nav_poll(&rx);
    if (fresh && !nav_sane(&rx)) {       // survived the CRC but is not a state
        s_rx_bad++;
        fresh = false;
    }
    if (fresh) {
        s_nav.h  = rx.h;
        s_nav.vx = rx.vx;  s_nav.vy = rx.vy;  s_nav.vz = rx.vz;
        s_nav.bx = rx.bx;  s_nav.by = rx.by;  s_nav.bz = rx.bz;
        s_nav.gx = rx.gx;  s_nav.gy = rx.gy;  s_nav.gz = rx.gz;
        s_nav.wx = rx.wx;  s_nav.wy = rx.wy;  s_nav.wz = rx.wz;
        s_nav.t     = (float)rx.t_ms * 1e-3f;
        s_nav.valid = rx.valid;
        s_nav_us    = micros();
        s_have_nav  = true;
    } else if (s_have_nav) {
        uint32_t age = micros() - s_nav_us;
        if (age > NAV_HOLD_MS * 1000u) {
            s_nav.valid = false;                  // parks the servo, freezes the clamp
        } else if (age > NAV_STALE_MS * 1000u) {
            nav_extrapolate(&s_nav, 1.0f / DG_CTRL_HZ, s_out.thrust_est,
                            DG_M_INITIAL - 0.5f * DG_M_PROP);
        }
    }

    // ---- arming -------------------------------------------------------
    // Two conditions, both required: the physical switch, and the vehicle
    // actually falling through the guidance entry altitude. The entry
    // velocity latched here selects the ignition altitude, so it must be
    // taken on the way DOWN and only once.
    bool sw_armed = (digitalRead(PIN_ARM) == LOW);
    if (!s_armed && sw_armed && s_have_nav && s_nav.valid
        && s_nav.h <= DG_H_START && s_nav.vz < -2.0f) {
        dg_arm(s_nav.vx, s_nav.vz);
        s_armed = true;
        DG_PRINT("# ARMED vx=%.1f vz=%.1f -> h_ign=%.2f m\n",
                      s_nav.vx, s_nav.vz, dg_h_ignite());
    }

    // ---- guidance -----------------------------------------------------
    if (s_have_nav) {
        nav_publish(&s_nav);
        dg_step_attitude(&s_nav, &s_out);
#if !DG_USE_CORE1
        dg_step_planner(&s_nav);
        s_plan_calls++;
#endif
    }

    // ---- igniter ------------------------------------------------------
    // The arming switch is checked again here, at the output, so opening it
    // inhibits the igniter no matter what the guidance decided.
    if (s_out.ignite && !s_ign_fired && sw_armed) {
        s_ign_fired  = true;
        s_ign_active = true;
        s_ign_start  = millis();
        digitalWrite(PIN_IGNITER, HIGH);
        DG_PRINT("# IGNITE h=%.2f vz=%.1f\n", s_nav.h, s_nav.vz);
    }
    if (s_ign_active && (!sw_armed || millis() - s_ign_start >= IGNITER_PULSE_MS)) {
        digitalWrite(PIN_IGNITER, LOW);
        s_ign_active = false;
    }

    // ---- actuators ----------------------------------------------------
    if (s_have_nav) {
        tvc_write(s_out.servo1_deg, s_out.servo2_deg);
        clamp_write(s_out.clamp);
    }
    digitalWrite(PIN_LED, s_out.phase == DG_PHASE_BURN ? HIGH
                                                       : (millis() % 1000 < 50));

    // ---- telemetry ----------------------------------------------------
    // Last, and rate limited, so a slow USB host can never stretch a
    // control tick. printf is the single most expensive call in this loop.
    if (millis() >= next_tlm) {
        next_tlm = millis() + 1000 / TELEM_HZ;
        float bzc = s_nav.bz > 1.0f ? 1.0f : (s_nav.bz < -1.0f ? -1.0f : s_nav.bz);
        float tilt = acosf(bzc) * (float)RAD_TO_DEG;
        float wmag = sqrtf(s_nav.wx * s_nav.wx + s_nav.wy * s_nav.wy
                           + s_nav.wz * s_nav.wz) * (float)RAD_TO_DEG;
        DG_PRINT("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.3f,"
                      "%.1f,%.2f,%lu,%lu,%lu,%lu,%lu\n",
                      (unsigned long)millis(), (int)s_out.phase,
                      s_nav.h, s_nav.vx, s_nav.vy, s_nav.vz,
                      tilt,
                      s_out.tilt_cmd * (float)RAD_TO_DEG,
                      s_out.tilt_err * (float)RAD_TO_DEG,
                      wmag,
                      s_out.roll_rate * (float)RAD_TO_DEG,
                      s_out.servo1_deg, s_out.servo2_deg,
                      s_out.clamp, s_out.thrust_est, s_out.p_plan,
                      (unsigned long)s_rx_ok, (unsigned long)s_rx_bad,
                      (unsigned long)s_loop_max_us,
                      (unsigned long)s_overruns, (unsigned long)s_plan_calls);
    }

    uint32_t dt = micros() - t0;
    if (dt > s_loop_max_us) s_loop_max_us = dt;
}

// =====================================================================
//  8. CORE 1 — trajectory replanner
//
//  dg_step_planner() does ONE trajectory projection per call and keeps the
//  root find's bracket between calls. A full solve is about seven
//  projections, so it converges in ~35 ms — comfortably inside the 100 ms
//  replan period — without ever blocking for a whole solve at once. Pacing
//  it at 200 Hz gives that convergence with the core idle the rest of the
//  time; there is nothing to gain by spinning faster.
// =====================================================================
#if DG_USE_CORE1
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
