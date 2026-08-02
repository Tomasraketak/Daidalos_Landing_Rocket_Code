// =====================================================================
//  Ariadna V2 — POSITION MCU
//
//  Target: Seeed XIAO RP2350, arduino-pico core (earlephilhower).
//  Needs:  Voltino TriSense library
//          https://github.com/Tomasraketak/Voltino_TriSense_Library
//
//  Role: state estimation and sensor fusion. This board turns an IMU, a
//  barometer, a magnetometer and a GPS into ONE navigation solution, and
//  ships it to the Master, which routes it to the Navigation MCU.
//
//  It makes no control decisions. Its entire product is
//  ariadna_nav_state_t, emitted the moment each estimator step finishes.
//
//  ---------------------------------------------------------------------
//  WHY THE ESTIMATOR LIVES HERE AND NOT ON THE NAVIGATION MCU
//  ---------------------------------------------------------------------
//  The guidance loop must never wait on a sensor. Barometer reads block on
//  I2C for hundreds of microseconds and GPS sentences arrive in bursts;
//  putting either in the same core as a 200 Hz control loop means the loop
//  time is set by the slowest sensor. Splitting them means the Navigation
//  MCU's worst case is bounded by its own arithmetic, which is bounded by
//  construction.
//
//  ---------------------------------------------------------------------
//  CORE ALLOCATION
//  ---------------------------------------------------------------------
//    core 0   ALL sensor I/O — TriSense fusion, barometer — plus the
//             estimator AND every outgoing packet. Every I2C and SPI
//             transaction in the vehicle's sensor stack happens on this
//             core, so there is no bus arbitration to get wrong, and the
//             DMA transmit ring has exactly one producer.
//    core 1   GPS parsing and receiving from the Master. Both are UARTs,
//             so this core touches no shared bus and can be as bursty as
//             NMEA wants to be.
//
//  The GPS fix crosses from core 1 to core 0 through a seqlock. Neither
//  core ever blocks on the other.
//
//  ---------------------------------------------------------------------
//  EVENT-DRIVEN, NOT TIMED
//  ---------------------------------------------------------------------
//  The navigation state is transmitted at the end of each estimator step,
//  not by a separate timer. Two things were wrong with a timer:
//
//    * it samples the estimator at an unrelated rate, so the state sits
//      in a buffer for anything from zero to a full transmit period
//      before it goes out — latency that varies packet to packet, which
//      is worse for a control loop than latency that is merely large;
//    * two free-running timers beat against each other, so the age of the
//      state the Navigation MCU sees wanders.
//
//  Emitting on completion removes both. The state on the wire is always
//  the state that was just computed.
//
//  The rate is therefore the estimator's rate: 240 Hz, feeding a 200 Hz
//  control loop. Deliberately faster than the consumer — at equal rates
//  two crystals drift and a tick occasionally finds no new frame.
//
//  ---------------------------------------------------------------------
//  WHAT IS NOT VERIFIED
//  ---------------------------------------------------------------------
//  Everything in this file. Unlike guidance.cpp there are no golden
//  vectors here. The two things that most need a bench before they need a
//  sky are in section 6 (the body-to-pad rotation, where a convention
//  mismatch silently mirrors every attitude) and section 7 (the filter
//  tuning). Section 10 self-tests what can be self-tested, which now
//  includes the horizontal channel, the innovation gate, and the
//  agreement between the rotation matrix and the quaternion.
//
//  ---------------------------------------------------------------------
//  THE ESTIMATOR IS A KALMAN FILTER, NOT A COMPLEMENTARY FILTER
//  ---------------------------------------------------------------------
//  Section 7 holds three 3-state Kalman filters, one per pad-ENU axis,
//  each estimating position, velocity and accelerometer bias. It replaced
//  a third-order complementary filter on the vertical channel plus dead
//  reckoning with a fixed-gain GPS velocity nudge on the horizontal.
//
//  The short version of why is that the old horizontal channel had no
//  position state at all, so the local x/y this board now ships could not
//  have been produced by it, and its fixed gains could not weight a GPS
//  fix by how good that fix actually was. The long version, including
//  what was kept, is at the top of section 7.
//
//  Section 6 changed with it: world-frame acceleration is now obtained by
//  rotating BODY-frame linear acceleration with this file's own matrix,
//  rather than by taking the fusion library's world frame and relabelling
//  its axes. That relabelling was mirrored in East/West. See the note
//  above the call.
// =====================================================================

#include "../common/ariadna_link.h"
#include "../common/ariadna_uart_dma.h"
#include "../common/ariadna_kf.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TriSense.h>

#include <string.h>
#include <stdlib.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE CONFIGURATION
//
//  Pin names are the XIAO silkscreen aliases (D0..D12) that the
//  arduino-pico board variant defines, matching the pinout spreadsheet
//  directly.
// =====================================================================

// ---- Voltino TriSense (ICM42688P over SPI, BMP580 + AK09918C over I2C) --
static const uint8_t PIN_TRISENSE_CS   = D3;
static const uint8_t PIN_SPI_SCK       = D8;
static const uint8_t PIN_SPI_MISO      = D9;
static const uint8_t PIN_SPI_MOSI      = D10;
static const uint8_t PIN_I2C_SDA       = D4;   // BMP580, AK09918C, TF Luna
static const uint8_t PIN_I2C_SCL       = D5;

// The ICM42688P will take 12 MHz. 10 MHz leaves margin for the flight
// harness, which is not a bench jumper.
static const uint32_t TRISENSE_SPI_HZ  = 10000000;

// ---- FUTURE: UWB DWM1000, shares the SPI bus ---------------------------
static const uint8_t PIN_UWB_INT       = D0;
static const uint8_t PIN_UWB_RST       = D1;
static const uint8_t PIN_UWB_CS        = D2;

// ---- link to the Master ------------------------------------------------
// The spreadsheet labels D6/D7 "UART1". On the arduino-pico core Serial1
// is UART0 and Serial2 is UART1, and which instance reaches D6/D7 depends
// on the XIAO RP2350 variant's GPIO mapping. If this port stays silent on
// the bench, change this one line to Serial2 first.
#define MASTER_PORT   Serial1
static const uint8_t PIN_MASTER_TX     = D6;
static const uint8_t PIN_MASTER_RX     = D7;

// ---- LK76 GPS ----------------------------------------------------------
//
// D11 and D12 are on the underside of the XIAO and are not guaranteed to
// land on a hardware UART. Rather than depend on that, the GPS runs on a
// PIO state machine, which the RP2350 will happily put on ANY pin. This is
// one of the places the chip's architecture actually saves a design
// compromise instead of just being fast.
//
// If the module is silent, swap these two: the labels describe the RP2350
// end, so PIN_GPS_TX goes to the module's RX.
static const uint8_t PIN_GPS_TX        = D11;
static const uint8_t PIN_GPS_RX        = D12;
static const uint32_t GPS_BAUD         = 9600;   // LK76 factory default
// The rate gps_configure() moves the module to. 5 Hz of GGA + RMC is
// roughly 750 byte/s, which does not fit inside 9600 baud with any margin
// once the receiver adds a leading talker ID and a checksum to each
// sentence — so the baud rate has to come up before the fix rate can.
static const uint32_t GPS_FAST_BAUD    = 115200;

SerialPIO GpsPort(PIN_GPS_TX, PIN_GPS_RX, 256);

// ---- rates -------------------------------------------------------------
//
// The estimator rate is the nav_state transmit rate; see "Event-driven,
// not timed" above. ARIADNA_NAV_STATE_HZ is the single definition, shared
// with the link budget check in ../../test/link_test.cpp.
static const uint32_t ESTIMATOR_HZ = ARIADNA_NAV_STATE_HZ;   // 240 Hz

// Barometer sampling. NOT the estimator rate, and deliberately far below
// it.
//
// The BMP580 will run at 240 Hz, but only at minimum oversampling. This
// build uses OSR x16 on pressure for the altitude noise floor, which caps
// the achievable rate far lower — so 50 Hz is what the sensor can actually
// deliver at this quality, and asking for more would mean asking for a
// noisier altitude.
//
// That costs nothing. The vertical filter believes the barometer below
// about 0.3 Hz and the accelerometer above it, so 50 Hz is already more
// than a hundred times oversampled for the job the barometer is doing.
// A quieter sample at 50 Hz beats a noisier one at 240 Hz here, every time.
static const uint32_t BARO_HZ = 50;

// Diagnostics only. printf promotes float to double by language rule and
// the RP2350 emulates double in software, so the promotion is confined
// here to keep -Wdouble-promotion meaningful in the estimator.
#define POS_PRINT(...)                                            \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        Serial.printf(__VA_ARGS__);                               \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

// =====================================================================
//  2. SENSORS
// =====================================================================
TriSense          sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

static uint16_t s_sensor_ok  = 0;      // ARIADNA_SENS_* bits
static uint8_t  s_cal_state  = 0;
static bool     s_post_pass  = false;

// =====================================================================
//  3. FUTURE SENSORS
//
//  Declared, wired into the pipeline, and deliberately inert.
//
//  Both of these are landing-phase sensors: below about 10 m the barometer
//  is being disturbed by the vehicle's own plume and GPS altitude is worth
//  little, which is exactly where a lidar and a UWB range are worth a lot.
//  The estimator already has the hooks to blend them (see est_fuse_range),
//  so bringing either online is a matter of filling in read() and setting
//  the health bit — not of rewiring the data path.
// =====================================================================

/** One range measurement, from whichever sensor produced it. */
struct RangeFix {
    float    range_m;      // metres
    float    sigma_m;      // 1-sigma uncertainty, drives the blend weight
    uint32_t t_ms;
    bool     valid;
};

/**
 * TF Luna lidar, I2C, shares the bus with the barometer and magnetometer.
 *
 * NOT IMPLEMENTED. Returns an invalid fix, so ARIADNA_NAV_LIDAR_OK stays
 * clear and the estimator ignores it.
 */
class LidarTFLuna {
public:
    bool begin(uint8_t i2c_addr = 0x10)
    {
        _addr = i2c_addr;
        _ok   = false;          // no probe yet: nothing to probe
        return _ok;
    }

    const RangeFix &read(void)
    {
        _fix.valid = false;     // until implemented, contributes nothing
        return _fix;
    }

    bool healthy(void) const { return _ok; }

private:
    uint8_t  _addr = 0x10;
    bool     _ok   = false;
    RangeFix _fix  = { 0.0f, 0.0f, 0, false };
};

/**
 * Decawave DWM1000 UWB, SPI, shares the bus with the IMU.
 *
 * NOT IMPLEMENTED. The intended use is ranging to anchors around the pad
 * for a horizontal position fix an order of magnitude better than GPS,
 * which is what turns "lands somewhere near the pad" into "lands on it".
 *
 * Note for whoever implements this: the DWM1000 wants a 2 MHz SPI clock
 * until it is configured, while the ICM42688P is being driven at 10 MHz on
 * the same bus. Use SPISettings per transaction, and do not assume the
 * clock the other device left behind.
 */
class UwbDWM1000 {
public:
    bool begin(uint8_t cs, uint8_t rst, uint8_t irq)
    {
        _cs = cs; _rst = rst; _irq = irq;
        pinMode(_cs,  OUTPUT); digitalWrite(_cs, HIGH);   // deselect: it
        pinMode(_rst, OUTPUT); digitalWrite(_rst, HIGH);  // shares the IMU bus
        pinMode(_irq, INPUT);
        _ok = false;
        return _ok;
    }

    /** Horizontal position fix in the pad frame. */
    bool position(float *x, float *y, float *sigma)
    {
        (void)x; (void)y; (void)sigma;
        return false;
    }

    const RangeFix &range(void)
    {
        _fix.valid = false;
        return _fix;
    }

    bool healthy(void) const { return _ok; }

private:
    uint8_t  _cs = 0, _rst = 0, _irq = 0;
    bool     _ok = false;
    RangeFix _fix = { 0.0f, 0.0f, 0, false };
};

static LidarTFLuna s_lidar;
static UwbDWM1000  s_uwb;

// =====================================================================
//  4. GPS — LK76 NMEA parser
//
//  Field-splitting in place on a fixed buffer. No String, no malloc, no
//  unbounded loop: this runs on core 1 next to the link and must not
//  produce latency spikes.
//
//  Only GGA and RMC are decoded. GGA carries fix quality, satellite count,
//  HDOP and MSL altitude; RMC carries position, ground speed and course.
//  Everything else the module emits is skipped at the sentence-type check.
// =====================================================================
struct GpsFix {
    int32_t  lat_1e7;
    int32_t  lon_1e7;
    float    alt_msl;      // m
    float    vel_e;        // m/s, East
    float    vel_n;        // m/s, North
    uint8_t  sats;
    uint8_t  fix;          // 0 none, 2 2D, 3 3D
    uint8_t  hdop_x10;
    uint32_t t_ms;         // local millis() at parse
    bool     valid;
};

class NmeaParser {
public:
    void begin(void)
    {
        _n = 0;
        memset(&_fix, 0, sizeof(_fix));
    }

    /** Feed one byte. Returns true when a sentence updated the fix. */
    bool feed(char c)
    {
        if (c == '$') { _n = 0; _buf[_n++] = c; return false; }
        if (_n == 0)  return false;

        if (c == '\r' || c == '\n') {
            _buf[_n] = '\0';
            bool updated = parse();
            _n = 0;
            return updated;
        }

        if (_n < sizeof(_buf) - 1) _buf[_n++] = c;
        else                        _n = 0;      // overlong: drop it
        return false;
    }

    const GpsFix &fix(void) const { return _fix; }

private:
    char    _buf[100];
    uint8_t _n = 0;
    GpsFix  _fix = {};

    // Split on commas in place. Returns the field count.
    uint8_t split(char *fields[], uint8_t max)
    {
        uint8_t n = 0;
        char *p = _buf;
        fields[n++] = p;
        while (*p && n < max) {
            if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
            else if (*p == '*') { *p = '\0'; break; }   // drop the checksum
            p++;
        }
        return n;
    }

    // NMEA latitude is ddmm.mmmm and longitude dddmm.mmmm. Degrees and
    // minutes are separated by position, not by a delimiter, which is why
    // this cannot just be an atof.
    static int32_t parse_coord(const char *s, char hemi, uint8_t deg_digits)
    {
        if (!s || !*s) return 0;

        char degbuf[4] = { 0, 0, 0, 0 };
        for (uint8_t i = 0; i < deg_digits && s[i]; i++) degbuf[i] = s[i];

        float deg = (float)atoi(degbuf);
        float min = (float)atof(s + deg_digits);
        float val = deg + min * (1.0f / 60.0f);

        if (hemi == 'S' || hemi == 'W') val = -val;
        return (int32_t)(val * 1e7f);
    }

    bool parse(void)
    {
        // Accept any talker ID: the LK76 emits GP, GN and GA sentences
        // depending on which constellations it is tracking.
        if (_n < 7 || _buf[0] != '$') return false;
        const char *type = _buf + 3;

        char *f[24];
        uint8_t nf;

        if (!strncmp(type, "GGA", 3)) {
            nf = split(f, 24);
            if (nf < 10) return false;
            _fix.lat_1e7  = parse_coord(f[2], f[3][0], 2);
            _fix.lon_1e7  = parse_coord(f[4], f[5][0], 3);
            uint8_t q     = (uint8_t)atoi(f[6]);
            _fix.sats     = (uint8_t)atoi(f[7]);
            _fix.hdop_x10 = (uint8_t)(atof(f[8]) * 10.0f);
            _fix.alt_msl  = (float)atof(f[9]);
            // GGA quality is 0 for no fix; it does not distinguish 2D from
            // 3D, so RMC's validity flag refines this below.
            if (q == 0) _fix.fix = 0;
            else if (_fix.fix < 2) _fix.fix = 2;
            _fix.valid = (q != 0);
            _fix.t_ms  = millis();
            return true;
        }

        if (!strncmp(type, "RMC", 3)) {
            nf = split(f, 24);
            if (nf < 9) return false;
            if (f[2][0] != 'A') { _fix.valid = false; return true; }

            _fix.lat_1e7 = parse_coord(f[3], f[4][0], 2);
            _fix.lon_1e7 = parse_coord(f[5], f[6][0], 3);

            // Ground speed is in knots, course is degrees true (clockwise
            // from North). Resolve into the pad-ENU axes the rest of the
            // system uses: East is sin(course), North is cos(course).
            float knots  = (float)atof(f[7]);
            float course = (float)atof(f[8]) * (float)DEG_TO_RAD;
            float speed  = knots * 0.514444f;
            _fix.vel_e   = speed * sinf(course);
            _fix.vel_n   = speed * cosf(course);

            _fix.valid = true;
            if (_fix.fix < 3) _fix.fix = 3;
            _fix.t_ms  = millis();
            return true;
        }

        return false;
    }
};

static NmeaParser s_nmea;

// =====================================================================
//  5. CROSS-CORE HANDOVER
//
//  One seqlock, one direction: the GPS fix, from core 1 to core 0.
//
//  There used to be a second one carrying the navigation solution the
//  other way, back when core 1 did the transmitting. Emitting the state
//  from the core that computes it removed both the seqlock and the timer
//  that sampled it — the state now goes on the wire without ever being
//  buffered. Fewer moving parts and less latency for the same work.
//
//  A seqlock rather than a mutex because neither core may ever block: core
//  0 is running the estimator on a fixed tick and core 1 is servicing two
//  UARTs. The writer bumps an odd/even counter around the copy; the reader
//  retries if it saw a torn window. A stale read is harmless — the payload
//  is a snapshot, not a delta.
// =====================================================================

// core 1 -> core 0 : the GPS fix
static volatile uint32_t     s_gps_seq = 0;
static GpsFix                s_gps_shared;

static volatile bool         s_core0_ready = false;

static inline void gps_publish(const GpsFix *g)
{
    s_gps_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_gps_shared = *g;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_gps_seq++;
}

static inline bool gps_acquire(GpsFix *g)
{
    uint32_t a = s_gps_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *g = s_gps_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_gps_seq == a;
}

// =====================================================================
//  6. BODY TO PAD ROTATION
//
//  THE HIGHEST-RISK CODE IN THIS FILE. Every attitude and rate the
//  Navigation MCU sees passes through here, and a convention mismatch does
//  not produce a plausible-but-wrong number — it mirrors an axis, which
//  turns the TVC loop into positive feedback.
//
//  Three conventions have to agree, and only one of them is ours:
//
//   1. The Euler order the fusion library uses internally. This code
//      assumes the usual aerospace Z-Y-X: yaw about Z, then pitch about
//      the new Y, then roll about the new X.
//
//   2. Yaw versus heading. The library returns a COMPASS HEADING —
//      degrees clockwise from magnetic North. Pad-ENU azimuth is measured
//      counter-clockwise from East. The conversion is az = 90 - heading,
//      and it is a reflection, not a rotation: getting it wrong swaps East
//      and West without changing anything that looks obviously wrong on a
//      bench that is not pointed North.
//
//   3. Where the sensor is mounted. Handled by the library's
//      setMountOrientation(), configured in setup().
//
//  BENCH IT. Section 11 checks the arithmetic; only a board on a table
//  checks the conventions. Point the nose North, then East, then tip it
//  10 degrees each way, and confirm u moves where this comment says.
// =====================================================================

/**
 * The landing motor's thrust direction, in BODY axes.
 *
 * The landing motor sits in the nose and fires forward-out, so it pushes
 * the vehicle along its own tail-ward axis. With the TriSense mounted
 * Z-up on a vehicle standing on the pad, that is body -Z.
 *
 * Sanity check against the link header's convention: standing on the pad,
 * body -Z maps to pad -Z, giving u = (0, 0, -1). Correct. In a nose-first
 * descent the vehicle is inverted, body -Z maps to pad +Z, u = (0, 0, +1),
 * which is thrust straight up — the attitude the landing burn wants.
 */
static const float BODY_THRUST_AXIS[3] = { 0.0f, 0.0f, -1.0f };

/**
 * Build the body-to-pad rotation matrix from the fused Euler angles.
 *
 * Rows are the pad-frame components of each body axis, so
 * v_pad = R * v_body.
 */
static void build_rotation(float roll_deg, float pitch_deg, float heading_deg,
                           float R[3][3])
{
    // Compass heading to pad-ENU azimuth. See convention 2 above.
    const float yaw = (90.0f - heading_deg) * (float)DEG_TO_RAD;
    const float pit = pitch_deg * (float)DEG_TO_RAD;
    const float rol = roll_deg  * (float)DEG_TO_RAD;

    const float cy = cosf(yaw), sy = sinf(yaw);
    const float cp = cosf(pit), sp = sinf(pit);
    const float cr = cosf(rol), sr = sinf(rol);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

static inline void rotate(const float R[3][3], const float v[3], float out[3])
{
    out[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    out[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    out[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

/**
 * The SAME rotation as build_rotation(), expressed as a quaternion.
 *
 * The Navigation MCU and the ground station both want the attitude as a
 * quaternion — no wrap-around, no discontinuity, and interpolable. It is
 * built here from the same three angles that build the matrix, rather
 * than taken from the fusion library's own quaternion, for one reason:
 *
 *   THE LIBRARY'S WORLD FRAME IS NOT PAD-ENU. AdvancedTriFusion builds
 *   its world frame as Rz(+heading) with heading a CLOCKWISE compass
 *   bearing, while pad-ENU azimuth runs COUNTER-CLOCKWISE from East. The
 *   two agree along the body's +X axis and come out MIRRORED for +Y. A
 *   quaternion taken straight from the library would therefore disagree
 *   with the u vector sitting next to it in the same packet, in a way
 *   that looks fine on a bench pointed North and mirrors East/West
 *   everywhere else.
 *
 * Deriving both from one angle triple makes that class of mismatch
 * impossible, and self_test() asserts it every boot by rotating a test
 * vector through the matrix and the quaternion and comparing.
 *
 * KNOWN LIMITATION, and the reason this comment is long. Going through
 * Euler angles inherits their singularity: a Z-Y-X triple is degenerate
 * at pitch = +/-90 deg, where heading stops being observable. On this
 * vehicle that is passed through TRANSIENTLY during the nose-over, not
 * dwelt in — standing on the pad and descending nose-first are both
 * pitch ~ 0 with roll near 0 and 180 respectively, which is well away
 * from it. The landing burn, the only phase where attitude is in a
 * control loop, is entirely in the safe region.
 *
 * The real fix is to take the library's quaternion and apply a fixed
 * frame correction instead of round-tripping through angles. That needs
 * the library's internal convention pinned down on a bench first, which
 * is why it is not being guessed at here.
 */
static void build_quaternion(float roll_deg, float pitch_deg, float heading_deg,
                             float q[4])
{
    // Half angles of the same Z-Y-X sequence build_rotation() applies.
    const float hy = 0.5f * (90.0f - heading_deg) * (float)DEG_TO_RAD;
    const float hp = 0.5f * pitch_deg * (float)DEG_TO_RAD;
    const float hr = 0.5f * roll_deg  * (float)DEG_TO_RAD;

    const float cy = cosf(hy), sy = sinf(hy);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cr = cosf(hr), sr = sinf(hr);

    q[0] = cy * cp * cr + sy * sp * sr;   // w
    q[1] = cy * cp * sr - sy * sp * cr;   // x
    q[2] = cy * sp * cr + sy * cp * sr;   // y
    q[3] = sy * cp * cr - cy * sp * sr;   // z

    // Normalise. The trig above is exact in theory and drifts in single
    // precision; the Navigation MCU range-checks the norm of what it
    // receives, so a drifting one would show up there as a rejected frame.
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 > 1e-12f) {
        const float inv = 1.0f / sqrtf(n2);
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    }
}

/** Rotate a body vector into the pad frame with a quaternion. Self-test only. */
static void rotate_q(const float q[4], const float v[3], float out[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float tx = 2.0f * (y * v[2] - z * v[1]);
    const float ty = 2.0f * (z * v[0] - x * v[2]);
    const float tz = 2.0f * (x * v[1] - y * v[0]);

    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

// =====================================================================
//  7. THE ESTIMATOR
//
//  Three independent 3-state Kalman filters, one per pad-ENU axis, each
//  holding
//
//      x = [ position, velocity, accelerometer bias ]
//
//  driven by world-frame acceleration and corrected by the barometer
//  (vertical), GPS position and velocity (horizontal), and zero-velocity
//  updates while the vehicle is still on the pad.
//
//  ---------------------------------------------------------------------
//  WHY THIS REPLACED THE COMPLEMENTARY FILTER
//  ---------------------------------------------------------------------
//  The previous estimator was a third-order complementary filter on the
//  vertical channel and, on the horizontal, dead reckoning with a fixed
//  0.15 gain toward GPS velocity and a 20 s leak toward zero. It was
//  sound for what it did. Four things it could not do:
//
//   1. IT HAD NO HORIZONTAL POSITION AT ALL. Not a tuning weakness — the
//      state simply did not exist, so the local x/y this packet now
//      carries could not have been filled in.
//
//   2. Fixed gains cannot weight a measurement by how good it is. GPS
//      horizontal accuracy swings by an order of magnitude with HDOP and
//      satellite count; one gain is right at one HDOP and wrong at every
//      other. A Kalman gain is computed from the measurement's variance
//      every time, which is the entire reason to pay for the covariance.
//
//   3. There was no outlier rejection, so one multipath fix went straight
//      into the state. There is now an innovation gate, with a
//      re-anchoring escape hatch for the case where the filter — not the
//      GPS — is the thing that is wrong.
//
//   4. The leak toward zero was a documented lie, and it biased velocity
//      low precisely during a long descent. With GPS position and
//      velocity both fused and a bias state per axis, the integrator has
//      a real anchor and the lie is not needed.
//
//  The vertical channel keeps everything that was RIGHT about the old
//  one: pad-referenced barometry (below), a bias state, and dropping
//  trust in the barometer during the burn.
//
//  ---------------------------------------------------------------------
//  WHY THREE 3-STATE FILTERS AND NOT ONE 9-STATE FILTER
//  ---------------------------------------------------------------------
//  Because attitude is supplied externally by the fusion library, the
//  9-state system matrix is exactly block diagonal per axis, the process
//  noise is diagonal, and every measurement touches exactly one axis. A
//  block-diagonal covariance therefore STAYS block diagonal forever, so
//  three 3x3 filters are mathematically identical to one 9x9 filter at
//  roughly a ninth of the arithmetic. An exact factorisation, not an
//  approximation.
//
//  The consequence worth stating: every measurement is scalar and
//  observes one state directly, so H is a unit row and the update
//  collapses to a handful of multiply-adds. THERE IS NO MATRIX INVERSION
//  ANYWHERE IN THIS FILE.
//
//  ---------------------------------------------------------------------
//  WHAT THE BIAS STATE ACTUALLY ABSORBS
//  ---------------------------------------------------------------------
//  More than its name suggests. It soaks up anything that looks like a
//  constant acceleration offset in the pad frame: true accelerometer
//  bias, an error in the local gravity constant, and — dominating both —
//  a small residual TILT error, which projects gravity onto the
//  horizontal axes. One degree of tilt error injects 0.17 m/s^2 of
//  phantom horizontal acceleration, far more than the ICM42688P's own
//  bias, which is why estimating this matters more than the datasheet
//  figure implies.
//
//  Vertically it is the same argument the old filter made: 0.05 m/s^2 of
//  bias integrates to 2.5 m/s of phantom velocity over a 10 s descent,
//  and the guidance law divides by vertical speed to get time-to-go.
//
//  ---------------------------------------------------------------------
//  THE PLUME, AND WHY IT IS AN R AND NOT A BANDWIDTH
//  ---------------------------------------------------------------------
//  The landing motor's plume raises static pressure around the airframe,
//  so the barometer reads LOW — it thinks the vehicle is higher than it
//  is, exactly when altitude matters most. The old filter handled this by
//  dropping its barometer bandwidth during the burn. This one inflates
//  the barometer's measurement variance instead.
//
//  Same intent, better mechanism. Inflating R says the thing that is
//  actually true — "this measurement is now bad" — and lets the filter
//  work out the consequences itself, including how much to trust the
//  altitude it already has and how fast to let the bias state drift.
//  Lowering a bandwidth asserts the answer instead of the evidence, and
//  it does nothing to stop a plume-corrupted sample from being gated in
//  as though it were good.
// =====================================================================

class Estimator {
public:
    void begin(void)
    {
        reset_states();
        _pad_pressure = 0.0f;
        _have_pad_ref = false;
        _have_origin  = false;
        _origin_lat   = 0;
        _origin_lon   = 0;
        _m_per_rad_lat = 6367000.0f;
        _m_per_rad_lon = 6367000.0f;
        _baro_var     = BARO_SIGMA * BARO_SIGMA;
        _gps_rejects  = 0;
        _baro_rejects = 0;
        _last_gps_ms  = 0;
        _zupt         = false;
    }

    /**
     * Capture the pad reference. Called on the ground, at arming.
     *
     * Everything downstream is measured FROM THE PAD, so this is what
     * makes the guidance module's 85 m entry altitude mean the right
     * thing. Without it the vehicle would be flying against sea level.
     *
     * `fix` sets the horizontal origin at the same instant. If there is
     * no GPS fix the vertical channel still works — that is the whole
     * point of referencing pressure rather than position — and the
     * horizontal origin is picked up later, on the first good fix.
     */
    void set_pad_reference(float pressure_pa, const GpsFix *fix)
    {
        if (!(pressure_pa > 30000.0f && pressure_pa < 110000.0f)) return;
        _pad_pressure = pressure_pa;
        _have_pad_ref = true;
        reset_states();

        if (fix && fix->valid && fix->fix >= 3 && fix->sats >= GPS_MIN_SATS)
            set_origin(fix->lat_1e7, fix->lon_1e7);
    }

    bool  have_pad_reference(void) const { return _have_pad_ref; }
    bool  have_origin(void)       const { return _have_origin; }
    float pad_pressure(void)      const { return _pad_pressure; }

    /**
     * Pressure to height above the pad.
     *
     * The barometric formula referenced to the pad rather than to sea
     * level. Ratio form, so the pad temperature and the absolute pressure
     * both cancel — only the RATIO matters, which is what makes this
     * robust to a barometer with an offset, and it is why this survived
     * the move to a Kalman filter unchanged.
     */
    float baro_altitude(float pressure_pa) const
    {
        if (!_have_pad_ref || pressure_pa < 1000.0f) return 0.0f;
        return 44330.0f * (1.0f - powf(pressure_pa / _pad_pressure, 0.1902949f));
    }

    /**
     * Barometer trust. `trusted` false means the burn is running and the
     * plume is corrupting static pressure — see the note above on why
     * this inflates a variance instead of moving a pole.
     */
    void set_baro_trust(bool trusted)
    {
        const float s = trusted ? BARO_SIGMA : BARO_SIGMA_BURN;
        _baro_var = s * s;
    }

    /**
     * One prediction step. a_* is pad-ENU acceleration in m/s^2 with
     * gravity already removed, az positive up.
     */
    void predict(float dt, float ax, float ay, float az)
    {
        if (dt <= 0.0f || dt > 0.1f) return;    // a stalled tick, not a state

        ariadna_kf_predict(&_kx, ax, dt, ACC_PSD_HORZ, ACC_BIAS_RW);
        ariadna_kf_predict(&_ky, ay, dt, ACC_PSD_HORZ, ACC_BIAS_RW);
        ariadna_kf_predict(&_kz, az, dt, ACC_PSD_VERT, ACC_BIAS_RW);

        ariadna_kf_clamp_bias(&_kx, BIAS_LIMIT);
        ariadna_kf_clamp_bias(&_ky, BIAS_LIMIT);
        ariadna_kf_clamp_bias(&_kz, BIAS_LIMIT);
    }

    /**
     * Barometric altitude correction. Vertical position only.
     *
     * The re-anchor is not optional here, and it matters more than the
     * GPS one. The barometer is the ONLY absolute altitude reference this
     * vehicle has, and the guidance law flies altitude — so a vertical
     * channel that has drifted far enough to gate the barometer out is a
     * channel that will gate it out forever, dead reckoning on an
     * accelerometer all the way to the ground while reporting a
     * confident, wrong height.
     *
     * So a run of consecutive rejections is taken to mean the filter is
     * wrong rather than the sensor, exactly as on the GPS path. At the
     * barometer's 50 Hz, BARO_MAX_REJECTS is a little under a second of
     * disagreement before the filter gives way.
     *
     * The threshold is deliberately far longer than any real transient.
     * A gust, a door, or the vehicle's own wake moves the barometer for a
     * few samples, not for fifty, and those are what the gate is there to
     * absorb.
     */
    void fuse_baro(float h_baro)
    {
        if (!_have_pad_ref) return;

        if (ariadna_kf_correct(&_kz, 0, h_baro, _baro_var, GATE_SIGMA)) {
            _baro_rejects = 0;
            return;
        }

        if (++_baro_rejects >= BARO_MAX_REJECTS) {
            // Re-anchor on the barometer, keeping velocity and re-opening
            // the covariance so the next samples can pull it the rest of
            // the way instead of being gated out in turn. The bias state
            // is deliberately reset with it: a divergence this large
            // means whatever it had learned was part of the problem.
            ariadna_kf_reset(&_kz, h_baro, _kz.x[1],
                             sqrtf(_baro_var) * 3.0f, 5.0f, BIAS_LIMIT);
            _baro_rejects = 0;
        }
    }

    /**
     * Fold in a GPS fix: horizontal position, then horizontal velocity.
     *
     * `age_s` is how old the fix is by the time it gets here — receiver
     * processing, time on the wire, and the wait for an estimator slot.
     * Both measurements are extrapolated forward over it, position by the
     * filter's velocity and velocity by the measured acceleration. An
     * uncompensated 150 ms lag at 40 m/s of descent is 6 m of position
     * error that the filter would otherwise have to interpret as real
     * motion.
     *
     * accel_g is the measured specific force, which de-weights the fix
     * when the vehicle is being thrown around — that is both when latency
     * hurts most and when the receiver's tracking loops are under the
     * most stress.
     */
    void fuse_gps(const GpsFix &fix, float age_s, float ax, float ay,
                  float accel_g, uint32_t now_ms)
    {
        if (!fix.valid || fix.fix < 3)              return;
        if (fix.sats < GPS_MIN_SATS)                return;

        const float hdop = hdop_of(fix);
        if (hdop > GPS_MAX_HDOP)                    return;

        if (!_have_origin) {
            set_origin(fix.lat_1e7, fix.lon_1e7);
            _last_gps_ms = now_ms;
            return;
        }

        if (age_s < 0.0f)  age_s = 0.0f;
        if (age_s > 1.0f)  age_s = 1.0f;

        const float mult = accel_trust(accel_g);

        // --- position ---------------------------------------------------
        // The lat/lon difference is taken in INTEGER 1e-7 degrees, which
        // is exact: an int32 subtraction cannot round. This is the one
        // place a geodetic filter normally has to reach for double —
        // at 50 deg latitude a float holds only ~4e-6 deg of absolute
        // longitude, about 0.4 m of quantisation — and carrying the fix
        // as scaled integers removes the need entirely. The RP2350
        // emulates double in software, so this is worth the arithmetic.
        const int32_t dlat = fix.lat_1e7 - _origin_lat;
        const int32_t dlon = fix.lon_1e7 - _origin_lon;

        float zy = (float)dlat * 1e-7f * (float)DEG_TO_RAD * _m_per_rad_lat;
        float zx = (float)dlon * 1e-7f * (float)DEG_TO_RAD * _m_per_rad_lon;

        zx += _kx.x[1] * age_s;
        zy += _ky.x[1] * age_s;

        const float sig_p = GPS_UERE_M * hdop * mult;
        const float R_pos = sig_p * sig_p;

        const bool ok_x = ariadna_kf_correct(&_kx, 0, zx, R_pos, GATE_SIGMA);
        const bool ok_y = ariadna_kf_correct(&_ky, 0, zy, R_pos, GATE_SIGMA);

        if (ok_x && ok_y) {
            _gps_rejects = 0;
        } else if (++_gps_rejects >= GPS_MAX_REJECTS) {
            // Repeated rejections mean the filter drifted away from
            // reality, not that the GPS started lying. Re-anchor on it
            // and re-open the covariance so the next fixes can pull it
            // back rather than being gated out in turn.
            ariadna_kf_reset(&_kx, zx, _kx.x[1], sig_p, 5.0f, BIAS_LIMIT);
            ariadna_kf_reset(&_ky, zy, _ky.x[1], sig_p, 5.0f, BIAS_LIMIT);
            _gps_rejects = 0;
        }

        // --- velocity ---------------------------------------------------
        // Doppler-derived, and independent of the inertial drift it is
        // correcting, which makes it the stronger of the two GPS
        // measurements even though its sigma looks small.
        const float sig_v = GPS_VEL_SIGMA * hdop * mult;
        const float R_vel = sig_v * sig_v;
        ariadna_kf_correct(&_kx, 1, fix.vel_e + ax * age_s, R_vel, GATE_SIGMA);
        ariadna_kf_correct(&_ky, 1, fix.vel_n + ay * age_s, R_vel, GATE_SIGMA);

        _last_gps_ms = now_ms;
    }

    /**
     * Zero-velocity update. PRE-LAUNCH ONLY — see the caller.
     *
     * Standing still is the only condition under which accelerometer bias
     * is directly observable, so a few seconds of this on the pad is worth
     * more to the bias states than the whole rest of the flight.
     *
     * Applied with no innovation gate, because a filter that has diverged
     * is exactly the case this exists to rescue.
     */
    void fuse_zupt(void)
    {
        const float R = ZUPT_VEL_SIGMA * ZUPT_VEL_SIGMA;
        ariadna_kf_correct(&_kx, 1, 0.0f, R, 0.0f);
        ariadna_kf_correct(&_ky, 1, 0.0f, R, 0.0f);
        ariadna_kf_correct(&_kz, 1, 0.0f, R, 0.0f);
        _zupt = true;
    }

    void clear_zupt(void) { _zupt = false; }
    bool zupt(void) const { return _zupt; }

    /**
     * Weak horizontal speed prior. Applied at about 1 Hz whenever GPS has
     * gone stale.
     *
     * WHY THIS EXISTS, AND WHY IT IS NOT THE OLD LEAK. The previous
     * estimator multiplied horizontal velocity by exp(-dt/20 s) on every
     * step, unconditionally. That bounded the dead-reckoning drift, and
     * it was described in its own comment as a deliberate lie — because
     * it also bled away REAL velocity, by about 40 % over a ten-second
     * descent, biasing the one horizontal quantity the guidance law
     * actually flies.
     *
     * This says something true instead: the vehicle cannot be travelling
     * horizontally at HORIZ_PRIOR_SIGMA. It is a genuine prior, not a
     * fudge, and because it goes in as a measurement the Kalman gain
     * decides how much notice to take of it:
     *
     *   * with the filter converged (P_vv ~ 1), K is about 1/2500 and a
     *     once-a-second application is worth nothing at all — real
     *     velocity is left alone, which is exactly what the old leak got
     *     wrong;
     *   * after a long outage, P_vv grows to the same order as R, K
     *     approaches a half, and the prior starts holding the estimate
     *     inside the envelope.
     *
     * So it is silent precisely when the filter knows what it is doing,
     * and firm precisely when it does not. The failure it prevents is not
     * a subtle one: an unbounded horizontal velocity eventually exceeds
     * the Navigation MCU's sanity limits, at which point that board
     * rejects the WHOLE nav_state — throwing away a perfectly good
     * altitude and attitude with it, and parking the TVC.
     */
    void fuse_horizontal_prior(void)
    {
        const float R = HORIZ_PRIOR_SIGMA * HORIZ_PRIOR_SIGMA;
        ariadna_kf_correct(&_kx, 1, 0.0f, R, 0.0f);
        ariadna_kf_correct(&_ky, 1, 0.0f, R, 0.0f);
    }

    /**
     * FUTURE: fold in a direct range-to-ground measurement.
     *
     * The hook the lidar and the UWB will use. Deliberately written now,
     * against the RangeFix the future drivers already return, so that
     * bringing either online does not require touching the filter — with
     * a Kalman filter it really is one ariadna_kf_correct() call, because the
     * sensor's own sigma is already the R the update wants.
     *
     * A range sensor is a far better altimeter than a barometer below
     * about 10 m, and it does not care about the plume — so when this
     * goes live it should also relax the burn-time inflation that
     * set_baro_trust() currently applies.
     */
    void fuse_range(const RangeFix &r)
    {
        if (!r.valid) return;
        (void)r;      // NOT IMPLEMENTED — no sensor produces a valid fix yet
    }

    // --- outputs -------------------------------------------------------
    float px(void) const { return _kx.x[0]; }
    float py(void) const { return _ky.x[0]; }
    float pz(void) const { return _kz.x[0]; }
    float vx(void) const { return _kx.x[1]; }
    float vy(void) const { return _ky.x[1]; }
    float vz(void) const { return _kz.x[1]; }
    float bias_x(void) const { return _kx.x[2]; }
    float bias_y(void) const { return _ky.x[2]; }
    float bias_z(void) const { return _kz.x[2]; }

    float sigma_pos_h(void) const
    {
        return 0.5f * (ariadna_kf_sigma(&_kx, 0) + ariadna_kf_sigma(&_ky, 0));
    }
    float sigma_vel_h(void) const
    {
        return 0.5f * (ariadna_kf_sigma(&_kx, 1) + ariadna_kf_sigma(&_ky, 1));
    }
    float sigma_pos_v(void) const { return ariadna_kf_sigma(&_kz, 0); }

    bool gps_aided(uint32_t now_ms) const
    {
        return _last_gps_ms != 0 && (uint32_t)(now_ms - _last_gps_ms) < 3000u;
    }

    /**
     * Estimator self-confidence, 0..255.
     *
     * Derived from the filter's own covariance rather than counted up
     * from boot. The old version incremented once per step until it hit
     * 255, which measured how long the board had been powered on and
     * nothing else — it read "full confidence" for a filter that had
     * never seen a measurement. This reads what the filter actually
     * believes: 255 at or below QUALITY_BEST_M of horizontal 1-sigma,
     * falling to 0 at QUALITY_WORST_M, and it drops on its own during a
     * GPS outage.
     */
    uint8_t quality(void) const
    {
        if (!_have_pad_ref) return 0;

        const float s = sigma_pos_h();
        if (s <= QUALITY_BEST_M)  return 255;
        if (s >= QUALITY_WORST_M) return 0;

        const float f = (QUALITY_WORST_M - s) / (QUALITY_WORST_M - QUALITY_BEST_M);
        return (uint8_t)(f * 255.0f);
    }

    // --- tuning, all exposed so the self test can reason about them ----
    //
    // ACC_PSD is the acceleration noise DENSITY driving velocity, in
    // m/s^2 per sqrt(Hz). The ICM42688P itself is around 0.0007. These
    // are nearly three orders larger ON PURPOSE, because what actually
    // perturbs the velocity estimate is residual attitude error and
    // airframe vibration, not sensor noise — and a solid motor on the
    // same frame produces a great deal of both. Raise them if the filter
    // lags real motion; lower them if the output is jittery between
    // fixes.
    static constexpr float ACC_PSD_HORZ   = 0.5f;    // m/s^2/sqrt(Hz)
    static constexpr float ACC_PSD_VERT   = 0.6f;    // also absorbs baro noise
    static constexpr float ACC_BIAS_RW    = 0.004f;  // m/s^2/sqrt(s)
    static constexpr float BIAS_LIMIT     = 1.5f;    // m/s^2, as before

    // BARO_SIGMA is set from the ATMOSPHERE, not from the BMP580's noise
    // figure. Per sample the sensor contributes centimetres at OSR x16.
    // What does not average down is the air — gusts, the vehicle's own
    // wake, ground effect near the pad — which is correlated over
    // seconds. Quoting the sensor's number here would make the filter
    // over-confident in altitude and slow to accept any correction.
    static constexpr float BARO_SIGMA      = 0.5f;   // m
    // During the burn, the plume. Twenty-five metres is not a
    // measurement, it is a formal way of saying "ignore this until the
    // motor is out", while still letting a truly enormous discrepancy
    // pull the filter back.
    static constexpr float BARO_SIGMA_BURN = 25.0f;  // m

    static constexpr float GPS_UERE_M     = 2.5f;    // m per unit HDOP
    static constexpr float GPS_VEL_SIGMA  = 0.3f;    // m/s at HDOP 1
    static constexpr float GPS_MAX_HDOP   = 6.0f;
    static constexpr uint8_t GPS_MIN_SATS = 4;
    // A rocket lives at high g by design, so the knee sits far above the
    // 2 g that suits a car. Below it the fix is taken at face value.
    static constexpr float GPS_ACCEL_KNEE_G = 4.0f;
    static constexpr float GPS_ACCEL_SLOPE  = 1.0f;
    static constexpr float GPS_ACCEL_MAX    = 8.0f;

    static constexpr float GATE_SIGMA      = 4.0f;
    static constexpr uint8_t GPS_MAX_REJECTS  = 12;
    static constexpr uint8_t BARO_MAX_REJECTS = 40;   // ~0.8 s at 50 Hz
    static constexpr float ZUPT_VEL_SIGMA  = 0.02f;  // m/s

    // The horizontal speed this vehicle is asserted not to exceed. Well
    // outside the 0-15 m/s design entry envelope, so it never argues with
    // a real trajectory — it only catches an estimate that has stopped
    // being one. See fuse_horizontal_prior().
    static constexpr float HORIZ_PRIOR_SIGMA = 50.0f;  // m/s

    static constexpr float QUALITY_BEST_M  = 1.0f;
    static constexpr float QUALITY_WORST_M = 25.0f;

private:
    void reset_states(void)
    {
        // Horizontal starts wide open: with no GPS fix yet the position
        // is genuinely unknown, and saying so lets the first fix snap it
        // into place instead of being gated out as an outlier.
        ariadna_kf_reset(&_kx, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);
        ariadna_kf_reset(&_ky, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);
        // Vertical starts well known: the pad reference was just taken,
        // so the vehicle is at 0 m by definition and the barometer works
        // from that instant.
        ariadna_kf_reset(&_kz, 0.0f, 0.0f, 0.5f, 0.5f, 0.5f);
        _gps_rejects  = 0;
        _baro_rejects = 0;
        _zupt = false;
    }

    static float hdop_of(const GpsFix &f)
    {
        const float h = (float)f.hdop_x10 * 0.1f;
        return h < 0.8f ? 0.8f : h;      // no receiver is better than this
    }

    static float accel_trust(float accel_g)
    {
        if (accel_g <= GPS_ACCEL_KNEE_G) return 1.0f;
        const float m = 1.0f + GPS_ACCEL_SLOPE * (accel_g - GPS_ACCEL_KNEE_G);
        return (m > GPS_ACCEL_MAX) ? GPS_ACCEL_MAX : m;
    }

    /**
     * Fix the local tangent plane at the pad.
     *
     * The radii of curvature are computed ONCE, here, rather than a full
     * geodetic projection per fix. Over the few kilometres this vehicle
     * can reach, a tangent plane built from them is accurate to well
     * under a metre, and it costs two transcendental calls at arming
     * instead of several on every fix.
     */
    void set_origin(int32_t lat_1e7, int32_t lon_1e7)
    {
        _origin_lat = lat_1e7;
        _origin_lon = lon_1e7;

        const float lat = (float)lat_1e7 * 1e-7f * (float)DEG_TO_RAD;
        const float s   = sinf(lat);
        const float e2  = 6.69437999014e-3f;
        const float w   = 1.0f - e2 * s * s;
        const float a   = 6378137.0f;
        const float rw  = sqrtf(w);

        _m_per_rad_lon = (a / rw) * cosf(lat);          // prime vertical x cos
        _m_per_rad_lat = (a * (1.0f - e2)) / (w * rw);  // meridian
        _have_origin   = true;
    }

    ariadna_kf_t   _kx, _ky, _kz;          // East, North, Up

    float    _pad_pressure = 0.0f;
    bool     _have_pad_ref = false;

    bool     _have_origin  = false;
    int32_t  _origin_lat   = 0;
    int32_t  _origin_lon   = 0;
    float    _m_per_rad_lat = 6367000.0f;
    float    _m_per_rad_lon = 6367000.0f;

    float    _baro_var    = BARO_SIGMA * BARO_SIGMA;
    uint8_t  _gps_rejects  = 0;
    uint8_t  _baro_rejects = 0;
    uint32_t _last_gps_ms = 0;
    bool     _zupt        = false;
};

static Estimator s_est;

/**
 * How old a GPS fix is before this board ever sees it.
 *
 * The receiver computes a solution, then formats and clocks out an NMEA
 * sentence; only then does core 1 parse it. This constant covers the part
 * that happens before the timestamp is taken. The rest — the wait for an
 * estimator slot, and the cross-core handover — is measured from the
 * timestamp and added on top at the call site.
 *
 * 0.10 to 0.25 s is typical of a consumer receiver. Getting it roughly
 * right matters more than getting it exactly right: at a 40 m/s descent
 * an uncompensated 150 ms is 6 m of position error, and any value in the
 * right range removes most of it.
 */
static const float GPS_LATENCY_S = 0.15f;

/** Measured specific force magnitude, in g. Includes gravity. */
static float accel_magnitude_g(void)
{
    TriSenseDataSnapshot snap;
    if (!sensor.getSnapshot(snap)) return 1.0f;    // assume 1 g rather than 0
    const float g2 = snap.accelX * snap.accelX +
                     snap.accelY * snap.accelY +
                     snap.accelZ * snap.accelZ;
    return sqrtf(g2) * (1.0f / 9.80665f);          // library reports m/s^2
}

/**
 * Standstill detector for the zero-velocity update.
 *
 * Specific force within a small tolerance of 1 g AND a quiet gyro, held
 * for long enough that a momentary coincidence does not trigger it.
 *
 * Read the ZUPT gating note at the call site before reusing this: on its
 * own, "1 g and not rotating" does NOT mean stationary.
 */
static bool motion_is_quiet(float accel_g, uint32_t now_ms)
{
    static const float    ZUPT_ACC_TOL_G    = 0.04f;
    static const float    ZUPT_GYRO_TOL_DPS = 2.0f;
    static const uint32_t ZUPT_HOLD_MS      = 300;

    static uint32_t quiet_since_ms = 0;

    const float gsum = fabsf((float)fusion.lastGx) +
                       fabsf((float)fusion.lastGy) +
                       fabsf((float)fusion.lastGz);

    if (fabsf(accel_g - 1.0f) < ZUPT_ACC_TOL_G && gsum < ZUPT_GYRO_TOL_DPS) {
        if (quiet_since_ms == 0) quiet_since_ms = now_ms ? now_ms : 1;
    } else {
        quiet_since_ms = 0;
    }

    return quiet_since_ms != 0 &&
           (uint32_t)(now_ms - quiet_since_ms) >= ZUPT_HOLD_MS;
}

// =====================================================================
//  8. LINK
// =====================================================================
static ariadna_rx_t           s_rx;
static ariadna_master_state_t s_master;
static bool                   s_have_master = false;
static uint32_t               s_master_us   = 0;

// DMA-fed transmit. A 90-byte nav_state frame is nearly three times the
// UART's hardware FIFO, so a blocking write would spin for 1.26 ms every
// 10 ms — 12.6 % of this core. See ../common/ariadna_uart_dma.h.
static ariadna_uart_tx_t s_tx;

static void link_send(uint8_t type, const void *payload, uint8_t len)
{
    ariadna_uart_tx_send(&s_tx, MASTER_PORT, type, payload, len);
}

// Commands arrive from the Master. Anything that touches a sensor is
// deferred to core 0 through a flag rather than executed here — core 1
// must never touch the I2C or SPI bus.
static volatile bool s_req_pad_ref  = false;
static volatile bool s_req_cal_gyro = false;

static void handle_command(const ariadna_cmd_t *c)
{
    switch (c->cmd) {
    case ARIADNA_CMD_SET_PAD_REF: s_req_pad_ref  = true; break;
    case ARIADNA_CMD_CAL_GYRO:    s_req_cal_gyro = true; break;
    default: break;
    }
}

static void link_poll(void)
{
    while (MASTER_PORT.available()) {
        const uint8_t *pl = nullptr;
        uint8_t len = 0;
        uint8_t type = ariadna_rx_byte(&s_rx, (uint8_t)MASTER_PORT.read(), &pl, &len);
        if (!type) continue;

        switch (type) {
        case ARIADNA_PKT_MASTER_STATE: {
            ariadna_master_state_t m;
            if (!ariadna_unpack(pl, len, &m, sizeof(m))) break;
            s_master      = m;
            s_master_us   = micros();
            s_have_master = true;
            break;
        }
        case ARIADNA_PKT_CMD: {
            ariadna_cmd_t c;
            if (!ariadna_unpack(pl, len, &c, sizeof(c))) break;
            handle_command(&c);
            break;
        }
        default: break;
        }
    }
}

// =====================================================================
//  9. SENSOR INITIALISATION
// =====================================================================
static bool sensors_begin(void)
{
    // Route both buses to the flight harness pins BEFORE begin(). The
    // arduino-pico core latches the pin mapping at begin(), and
    // TriSense::beginAll() calls Wire.begin()/SPI.begin() internally with
    // no pin arguments — so it inherits whatever is configured here.
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);

    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);

    // Hybrid: the IMU on SPI at 10 MHz where the bandwidth matters, the
    // barometer and magnetometer on I2C where it does not.
    bool ok = sensor.beginAll(MODE_HYBRID, PIN_TRISENSE_CS, TRISENSE_SPI_HZ);

    s_sensor_ok = 0;
    if (ok) s_sensor_ok |= ARIADNA_SENS_IMU | ARIADNA_SENS_BARO | ARIADNA_SENS_MAG;

    // ---- IMU ranges --------------------------------------------------
    // 16 g and 2000 dps. A landing motor pulls several g and a tumbling
    // recovery spins fast; a saturated sensor is a lost solution, and the
    // resolution given up is far cheaper than the clipping avoided.
    sensor.imu.setAccelFS(AFS_16G);
    sensor.imu.setGyroFS(GFS_2000DPS);
    sensor.imu.setFIFOMode(FIFO_16BIT);

    // ---- barometer ---------------------------------------------------
    // Heavy pressure oversampling, light temperature oversampling, and a
    // short IIR. The BMP580 is the only absolute altitude reference above
    // the height a lidar would work at, so its noise floor is worth paying
    // for in conversion time.
    //
    // The IIR is deliberately mild: the estimator in section 7 is already a
    // filter, and filtering twice adds phase lag that the vertical channel
    // then has to fight. Anything past IIR_3 here shows up as lag in vz.
    sensor.bmp.setOversampling(BMP580_OSR_x16, BMP580_OSR_x2);
    sensor.bmp.setIIRFilter(BMP580_IIR_3, BMP580_IIR_OFF);
    sensor.bmp.setODR(BMP580_ODR_240Hz);   // free-running well above BARO_HZ

    // ---- fusion ------------------------------------------------------
    // Mount orientation: TriSense Z-up on a vehicle standing on the pad.
    // Change this ONE line if the board is rotated, rather than patching
    // signs into the rotation matrix — that is what it exists for.
    fusion.setMountOrientation(ORIENTATION_Z_UP);

    // Learn gyro drift continuously. The descent is long enough for a
    // fixed boot-time offset to go stale, and a rate offset feeds straight
    // into the Navigation MCU's inner loop.
    fusion.setDynamicGyroBias(true, 0.0001f);
    fusion.setMaxGyroBias(5.0f);

    // Magnetic declination for the pad. Update for your launch site — this
    // is the value for central Bohemia in 2026.
    fusion.setDeclination(5.6f);

    // A rocket under thrust is not in a 1 g field, so accelerometer-based
    // levelling must be OFF whenever the vehicle is accelerating. The
    // Gaussian gain does exactly that: full trust at 1 g, none away from
    // it. This is what keeps a 10 g boost from tipping the attitude
    // estimate over.
    fusion.setAccelGaussian(1.0f, 0.05f);
    fusion.setMaxGains(0.1f, 0.05f);

    return ok;
}

// =====================================================================
//  10. SELF TEST
//
//  What can honestly be checked without a bench: the arithmetic, and
//  whether the sensors answered. The conventions cannot be checked here —
//  see the warning in section 6.
// =====================================================================
static bool self_test(void)
{
    bool ok = true;

    // --- rotation matrix -----------------------------------------------
    float R[3][3], u[3];

    // Level, nose up, pointing North. Body -Z must map to pad -Z.
    build_rotation(0.0f, 0.0f, 0.0f, R);
    rotate(R, BODY_THRUST_AXIS, u);
    if (fabsf(u[0]) > 1e-4f || fabsf(u[1]) > 1e-4f || fabsf(u[2] + 1.0f) > 1e-4f) {
        POS_PRINT("POST rot level u=(%.3f %.3f %.3f) want (0 0 -1)\n", u[0], u[1], u[2]);
        ok = false;
    }

    // The matrix must stay orthonormal at an arbitrary attitude, or the
    // unit vector the Navigation MCU sanity-checks will drift out of range.
    build_rotation(23.0f, -17.0f, 200.0f, R);
    for (int i = 0; i < 3; i++) {
        float n2 = R[i][0] * R[i][0] + R[i][1] * R[i][1] + R[i][2] * R[i][2];
        if (fabsf(n2 - 1.0f) > 1e-4f) {
            POS_PRINT("POST rot row %d norm %.5f\n", i, n2);
            ok = false;
        }
    }
    rotate(R, BODY_THRUST_AXIS, u);
    {
        float n2 = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
        if (fabsf(n2 - 1.0f) > 1e-4f) {
            POS_PRINT("POST thrust axis norm %.5f\n", n2);
            ok = false;
        }
    }

    // --- quaternion agrees with the matrix -------------------------------
    // These two are built from the same angles by different arithmetic and
    // are shipped side by side in every packet. If they ever disagree, one
    // of the two things the Navigation MCU reads is a lie, so it is
    // checked rather than assumed. A test vector off every axis catches a
    // sign error that a symmetric one would hide.
    {
        const float v[3] = { 0.37f, -0.61f, 0.70f };
        const float angles[3][3] = {
            {  0.0f,   0.0f,   0.0f },
            { 23.0f, -17.0f, 200.0f },
            {-11.0f,  40.0f,  95.0f }
        };
        for (int i = 0; i < 3; i++) {
            float Rq[3][3], qq[4], a[3], b[3];
            build_rotation(angles[i][0], angles[i][1], angles[i][2], Rq);
            build_quaternion(angles[i][0], angles[i][1], angles[i][2], qq);
            rotate(Rq, v, a);
            rotate_q(qq, v, b);
            for (int k = 0; k < 3; k++) {
                if (fabsf(a[k] - b[k]) > 1e-3f) {
                    POS_PRINT("POST quat/matrix case %d axis %d: %.4f vs %.4f\n",
                              i, k, a[k], b[k]);
                    ok = false;
                }
            }
            float n2 = qq[0]*qq[0] + qq[1]*qq[1] + qq[2]*qq[2] + qq[3]*qq[3];
            if (fabsf(n2 - 1.0f) > 1e-4f) {
                POS_PRINT("POST quat norm %.5f\n", n2);
                ok = false;
            }
        }
    }

    // --- barometric conversion ------------------------------------------
    {
        Estimator e;
        e.begin();
        e.set_pad_reference(100000.0f, nullptr);
        if (fabsf(e.baro_altitude(100000.0f)) > 0.05f) {
            POS_PRINT("POST baro at pad %.3f m\n", e.baro_altitude(100000.0f));
            ok = false;
        }
        // Lower pressure must read HIGHER. A sign error here inverts the
        // entire flight.
        if (!(e.baro_altitude(99000.0f) > 5.0f)) {
            POS_PRINT("POST baro sign %.2f\n", e.baro_altitude(99000.0f));
            ok = false;
        }
        // Roughly 8.4 m per hPa near sea level.
        float d = e.baro_altitude(99000.0f);
        if (d < 75.0f || d > 95.0f) {
            POS_PRINT("POST baro scale %.2f m for 10 hPa\n", d);
            ok = false;
        }
    }

    // --- vertical filter learns the bias on the pad ----------------------
    //
    // The pad configuration exactly: sitting still at zero altitude, the
    // barometer agreeing, ZUPT running, and an accelerometer reading a
    // constant 0.4 m/s^2 that is not there. Twenty seconds later the
    // filter must have attributed all of it to bias and none of it to
    // motion.
    //
    // It is worth understanding why ZUPT is in this test rather than left
    // out for simplicity. With only the barometer, position is the sole
    // measurement, and a constant bias and a slowly growing velocity
    // explain a position error almost equally well — the two states are
    // barely separable, and the same run takes over two minutes to reach
    // this accuracy. A velocity measurement breaks the degeneracy
    // directly, which is what makes standstill the one condition where
    // accelerometer bias is genuinely observable. This test would pass
    // eventually without ZUPT; the vehicle would not wait.
    {
        Estimator e;
        e.begin();
        e.set_pad_reference(100000.0f, nullptr);
        const float dt    = 1.0f / (float)ESTIMATOR_HZ;
        const int   every = (int)(ESTIMATOR_HZ / BARO_HZ);
        for (int i = 0; i < (int)ESTIMATOR_HZ * 20; i++) {
            e.predict(dt, 0.0f, 0.0f, 0.4f);
            if ((i % every) == 0) {
                e.fuse_baro(0.0f);      // on the pad, so the datum is 0
                e.fuse_zupt();
            }
        }

        if (fabsf(e.pz()) > 0.5f) {
            POS_PRINT("POST filter pz %.2f want 0\n", e.pz());
            ok = false;
        }
        if (fabsf(e.vz()) > 0.05f) {
            POS_PRINT("POST filter vz %.3f want 0\n", e.vz());
            ok = false;
        }
        if (fabsf(e.bias_z() - 0.4f) > 0.1f) {
            POS_PRINT("POST filter bias %.3f want 0.4\n", e.bias_z());
            ok = false;
        }
    }

    // --- the barometer re-anchor breaks a gate lockout -------------------
    //
    // A vertical channel that has drifted far enough to gate the
    // barometer out would otherwise stay drifted forever, dead reckoning
    // to the ground on a confident wrong altitude. Force exactly that
    // and confirm the filter gives way to the sensor.
    {
        Estimator e;
        e.begin();
        e.set_pad_reference(100000.0f, nullptr);

        // 200 m of disagreement is far outside any gate.
        const int every = (int)(ESTIMATOR_HZ / BARO_HZ);
        for (int i = 0; i < (int)ESTIMATOR_HZ * 3; i++) {
            e.predict(1.0f / (float)ESTIMATOR_HZ, 0.0f, 0.0f, 0.0f);
            if ((i % every) == 0) e.fuse_baro(200.0f);
        }

        if (fabsf(e.pz() - 200.0f) > 5.0f) {
            POS_PRINT("POST baro re-anchor failed: pz %.2f want 200\n", e.pz());
            ok = false;
        }
    }

    // --- horizontal channel holds a GPS position -------------------------
    // The capability the old estimator did not have at all, so it is worth
    // checking that it exists rather than only that it is well tuned.
    // Origin at the pad, then a fix 50 m East and 20 m North held steady:
    // the filter must converge on it and stay there.
    {
        Estimator e;
        e.begin();

        GpsFix pad;
        memset(&pad, 0, sizeof(pad));
        pad.lat_1e7 = 500000000;      // 50 deg N, 14 deg E
        pad.lon_1e7 = 140000000;
        pad.sats = 9; pad.fix = 3; pad.hdop_x10 = 10; pad.valid = true;
        e.set_pad_reference(100000.0f, &pad);

        if (!e.have_origin()) {
            Serial.println("POST horizontal origin not captured");
            ok = false;
        }

        // 50 m East, 20 m North of the origin, in 1e-7 degree steps. At
        // 50 deg latitude a degree of longitude is ~71.7 km and a degree
        // of latitude ~111.2 km.
        GpsFix f = pad;
        f.lon_1e7 = pad.lon_1e7 + (int32_t)(50.0f / 71700.0f * 1e7f);
        f.lat_1e7 = pad.lat_1e7 + (int32_t)(20.0f / 111200.0f * 1e7f);

        const float dt = 1.0f / (float)ESTIMATOR_HZ;
        for (int i = 0; i < (int)ESTIMATOR_HZ * 10; i++) {
            e.predict(dt, 0.0f, 0.0f, 0.0f);
            if ((i % (int)(ESTIMATOR_HZ / 5)) == 0)      // 5 Hz of fixes
                e.fuse_gps(f, 0.0f, 0.0f, 0.0f, 1.0f, (uint32_t)(i * 1000 / ESTIMATOR_HZ));
        }

        // Two metres of tolerance on a GPS-derived position is generous by
        // design: this checks the sign, the axis assignment and the scale,
        // not the receiver.
        if (fabsf(e.px() - 50.0f) > 2.0f || fabsf(e.py() - 20.0f) > 2.0f) {
            POS_PRINT("POST horizontal px %.2f py %.2f want 50 20\n",
                      e.px(), e.py());
            ok = false;
        }
        // Stationary target: the velocity estimate must not run away.
        if (fabsf(e.vx()) > 1.0f || fabsf(e.vy()) > 1.0f) {
            POS_PRINT("POST horizontal vx %.2f vy %.2f want 0\n", e.vx(), e.vy());
            ok = false;
        }
        // And the filter must have become confident about it.
        if (!(e.sigma_pos_h() < 5.0f) || e.quality() == 0) {
            POS_PRINT("POST horizontal sigma %.2f quality %d\n",
                      e.sigma_pos_h(), (int)e.quality());
            ok = false;
        }
    }

    // --- the innovation gate actually rejects ----------------------------
    // A converged filter must not swallow a fix hundreds of metres away.
    // This is the multipath case, and it is the one that would otherwise
    // put a step into the position the guidance law flies to.
    {
        Estimator e;
        e.begin();

        GpsFix pad;
        memset(&pad, 0, sizeof(pad));
        pad.lat_1e7 = 500000000;
        pad.lon_1e7 = 140000000;
        pad.sats = 9; pad.fix = 3; pad.hdop_x10 = 10; pad.valid = true;
        e.set_pad_reference(100000.0f, &pad);

        const float dt = 1.0f / (float)ESTIMATOR_HZ;
        for (int i = 0; i < (int)ESTIMATOR_HZ * 10; i++) {
            e.predict(dt, 0.0f, 0.0f, 0.0f);
            if ((i % (int)(ESTIMATOR_HZ / 5)) == 0)
                e.fuse_gps(pad, 0.0f, 0.0f, 0.0f, 1.0f, (uint32_t)(i * 1000 / ESTIMATOR_HZ));
        }

        GpsFix bad = pad;
        bad.lon_1e7 += (int32_t)(300.0f / 71700.0f * 1e7f);   // 300 m jump
        e.fuse_gps(bad, 0.0f, 0.0f, 0.0f, 1.0f, 100000u);

        if (fabsf(e.px()) > 10.0f) {
            POS_PRINT("POST gate let a 300 m jump through: px %.2f\n", e.px());
            ok = false;
        }
    }

    // --- NMEA coordinate parsing ----------------------------------------
    {
        NmeaParser p;
        p.begin();
        const char *s = "$GPGGA,123519,4947.9150,N,01641.4770,E,1,12,0.9,545.4,M,46.9,M,,*47\r\n";
        for (const char *c = s; *c; c++) p.feed(*c);

        // 49 deg 47.9150 min = 49.7985833 deg. Tolerance is a few 1e-7
        // degrees, i.e. centimetres — this checks the ddmm.mmmm split, not
        // the receiver.
        int32_t dlat = p.fix().lat_1e7 - 497985833;
        int32_t dlon = p.fix().lon_1e7 - 166912833;
        if (dlat < -200 || dlat > 200) {
            POS_PRINT("POST nmea lat %ld\n", (long)p.fix().lat_1e7);
            ok = false;
        }
        // 16 deg 41.4770 min = 16.6912833 deg
        if (dlon < -200 || dlon > 200) {
            POS_PRINT("POST nmea lon %ld\n", (long)p.fix().lon_1e7);
            ok = false;
        }
        if (p.fix().sats != 12) {
            POS_PRINT("POST nmea sats %d\n", (int)p.fix().sats);
            ok = false;
        }
    }

    return ok;
}

// =====================================================================
//  11. CORE 0 — sensors and the estimator
// =====================================================================
void setup()
{
    Serial.begin(ARIADNA_CONSOLE_BAUD);

    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);      // deselect before SPI comes up:
                                         // it shares the bus with the IMU

    bool sensors = sensors_begin();
    s_est.begin();

    s_uwb.begin(PIN_UWB_CS, PIN_UWB_RST, PIN_UWB_INT);
    s_lidar.begin();

    // Level the attitude estimate against gravity. The vehicle must be
    // still for this; it runs before the Master ever arms.
    if (sensors) {
        sensor.autoCalibrateGyro(1000);
        fusion.initOrientation(200);
        s_cal_state = 1;
    }

    s_post_pass = self_test() && sensors;
    POS_PRINT("\nAriadna V2 Position MCU — POST %s (sensors %s)\n",
              s_post_pass ? "PASS" : "FAIL", sensors ? "ok" : "FAIL");

    // The Master link belongs to core 0: it transmits, so it owns the DMA
    // ring. Core 1 only receives, which needs no state of its own.
    MASTER_PORT.setTX(PIN_MASTER_TX);
    MASTER_PORT.setRX(PIN_MASTER_RX);
    MASTER_PORT.begin(ARIADNA_MCU_BAUD);
    ariadna_uart_tx_begin(&s_tx, uart0);   // uart0 is Serial1 on this core
    ariadna_rx_init(&s_rx);
    memset(&s_master, 0, sizeof(s_master));

    memset(&s_gps_shared, 0, sizeof(s_gps_shared));
    s_core0_ready = true;
}

void loop()
{
    static uint32_t next_us     = 0;
    static uint32_t next_baro_us = 0;
    static float    h_baro      = 0.0f;
    static uint32_t last_step_us = 0;
    static uint32_t last_gps_ms  = 0;
    static uint32_t last_prior_ms = 0;
    static uint32_t next_status_ms = 0;

    const uint32_t period = (uint32_t)(1000000UL / ESTIMATOR_HZ);

    uint32_t now = micros();
    if (next_us == 0) { next_us = now; last_step_us = now; }
    if ((int32_t)(now - next_us) < 0) return;
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) next_us = micros() + period;

    ariadna_uart_tx_service(&s_tx);      // retire a finished DMA run

    // ---- deferred commands from core 1 ---------------------------------
    // Executed here because they touch the sensor buses, which belong to
    // this core exclusively.
    if (s_req_pad_ref) {
        s_req_pad_ref = false;
        float p = sensor.readPressure();
        // The horizontal origin is captured from the same command, so the
        // pad is the origin of BOTH channels and px/py mean "metres from
        // the pad" rather than "metres from wherever the first fix
        // happened to land". Without a fix yet, the vertical reference is
        // still taken and the origin is picked up on the first good one.
        GpsFix g;
        const bool have = gps_acquire(&g);
        s_est.set_pad_reference(p, have ? &g : nullptr);
        POS_PRINT("# pad reference %.1f Pa, origin %s\n",
                  p, s_est.have_origin() ? "set" : "pending GPS");
    }
    if (s_req_cal_gyro) {
        s_req_cal_gyro = false;
        sensor.autoCalibrateGyro(1000);
        fusion.initOrientation(200);
        s_cal_state = 2;
    }

    // ---- fusion ---------------------------------------------------------
    // update() drains the IMU FIFO, so calling it at 200 Hz keeps up with a
    // 1 kHz ODR without losing samples. It is the only place the IMU is
    // read.
    bool fused = fusion.update();
    if (fused) s_sensor_ok |= ARIADNA_SENS_IMU;

    // ---- barometer ------------------------------------------------------
    // baro_fresh is true for exactly ONE estimator step per sample.
    //
    // This still matters with a Kalman filter, for a sharper reason than
    // it did with the complementary one. The predictor runs at 240 Hz and
    // the barometer produces a sample at 50 Hz; correcting on every step
    // would feed the SAME measurement in five times, and the filter would
    // count each as independent evidence. It would not merely be
    // mistuned — it would end up roughly sqrt(5) times more confident in
    // its altitude than the sensor justifies, and quietly reject the GPS
    // corrections that disagree.
    bool baro_fresh = false;
    if ((int32_t)(micros() - next_baro_us) >= 0) {
        next_baro_us = micros() + 1000000UL / BARO_HZ;
        float p = sensor.readPressure();
        if (p > 30000.0f && p < 110000.0f) {
            h_baro      = s_est.baro_altitude(p);
            baro_fresh  = s_est.have_pad_reference();
            s_sensor_ok |= ARIADNA_SENS_BARO;
        } else {
            s_sensor_ok &= (uint16_t)~ARIADNA_SENS_BARO;
        }
    }

    // ---- attitude and rates ---------------------------------------------
    float roll, pitch, heading;
    fusion.getOrientationDegrees(roll, pitch, heading);

    float R[3][3], u[3], q[4];
    build_rotation(roll, pitch, heading, R);
    build_quaternion(roll, pitch, heading, q);
    rotate(R, BODY_THRUST_AXIS, u);

    // Gyro rates, straight from the sensor, rotated into the pad frame.
    // ROTATED, NOT DIFFERENTIATED — see the note in ariadna_link.h. The
    // library exposes them in deg/s.
    const float g_body[3] = { (float)fusion.lastGx * (float)DEG_TO_RAD,
                              (float)fusion.lastGy * (float)DEG_TO_RAD,
                              (float)fusion.lastGz * (float)DEG_TO_RAD };
    float w[3];
    rotate(R, g_body, w);

    // ---- acceleration ----------------------------------------------------
    //
    // BODY-frame linear acceleration, rotated into pad-ENU by the SAME
    // matrix that produced u and w above.
    //
    // This used to call getGlobalLinearAcceleration() and relabel its X
    // as North and Y as East. That was wrong, and wrong in the quietest
    // possible way. The library builds its world frame as Rz(+heading)
    // with heading a clockwise compass bearing, while pad-ENU azimuth
    // runs counter-clockwise from East — the two agree along the body's
    // +X axis and are MIRRORED for +Y. A bench test with the board
    // pointed North, or tilted along +X only, passes either way; every
    // other heading fed the horizontal channel an East component of the
    // wrong sign.
    //
    // getLinearAcceleration() is yaw-independent by construction: the
    // gravity vector it removes is the third row of the rotation matrix,
    // a function of roll and pitch only. So it is safe to take from the
    // library and rotate here, which also means this file now performs
    // exactly ONE body-to-pad rotation and everything on the wire is
    // consistent with it.
    float ab[3];
    fusion.getLinearAcceleration(ab[0], ab[1], ab[2], ACCEL_UNIT_MS2);
    float a_pad[3];
    rotate(R, ab, a_pad);

    // ---- filter step ------------------------------------------------------
    uint32_t step_now = micros();
    float dt = (float)(step_now - last_step_us) * 1e-6f;
    last_step_us = step_now;

    // The plume corrupts the barometer during the burn. The Master tells
    // us when that is; until then the barometer is the altitude
    // reference. Set BEFORE the correction, not after, or the first
    // sample of the burn goes in at full weight.
    s_est.set_baro_trust(!(s_have_master &&
                           s_master.state == ARIADNA_STATE_GUIDANCE));

    s_est.predict(dt, a_pad[0], a_pad[1], a_pad[2]);
    if (baro_fresh) s_est.fuse_baro(h_baro);

    // ---- GPS -------------------------------------------------------------
    // Position and velocity both, with the fix aged forward to now. The
    // old estimator fused velocity only, at a fixed gain; there was no
    // horizontal position state for the position to correct.
    GpsFix gps;
    bool have_gps = gps_acquire(&gps);
    const uint32_t now_ms = millis();
    const float amag_g = accel_magnitude_g();

    if (have_gps && gps.valid && gps.t_ms != last_gps_ms) {
        last_gps_ms = gps.t_ms;
        const float age_s = (float)(uint32_t)(now_ms - gps.t_ms) * 1e-3f
                          + GPS_LATENCY_S;
        s_est.fuse_gps(gps, age_s, a_pad[0], a_pad[1], amag_g, now_ms);
        s_sensor_ok |= ARIADNA_SENS_GPS;
    }

    // ---- zero-velocity update, ON THE PAD ONLY ----------------------------
    //
    // Standing still is the only condition under which accelerometer bias
    // is directly observable, so the minutes spent on the pad are where
    // the bias states are actually learned. It is the cheapest and most
    // effective thing in this filter.
    //
    // IT IS ALSO THE MOST DANGEROUS, WHICH IS WHY IT IS GATED ON THE
    // MISSION STATE AND NOT ON THE MOTION TEST ALONE. A vehicle in
    // steady-state descent at terminal velocity — drag balancing weight —
    // reads exactly 1 g with a quiet gyro. That is indistinguishable from
    // sitting on the pad by any test this board can make from its own
    // sensors, and applying a ZUPT there would pin the velocity estimate
    // to zero during the descent: the guidance law divides by vertical
    // speed, so it would compute an infinite time-to-go and never light
    // the motor.
    //
    // So the motion test only ever runs while the Master says the vehicle
    // has not left the pad. Before the Master is heard from at all, the
    // state is unknown and the answer is no.
    const bool on_pad = s_have_master &&
                        (s_master.state == ARIADNA_STATE_BOOT  ||
                         s_master.state == ARIADNA_STATE_IDLE  ||
                         s_master.state == ARIADNA_STATE_ARMED ||
                         s_master.state == ARIADNA_STATE_TEST);

    if (on_pad && motion_is_quiet(amag_g, now_ms)) s_est.fuse_zupt();
    else                                           s_est.clear_zupt();

    // ---- horizontal speed prior, ~1 Hz while GPS is stale ----------------
    // Keeps a dead-reckoned horizontal velocity inside the physical
    // envelope without touching it while the filter is still confident.
    // See Estimator::fuse_horizontal_prior() for why this is not the leak
    // it replaced.
    if (!s_est.gps_aided(now_ms) && (uint32_t)(now_ms - last_prior_ms) >= 1000u) {
        last_prior_ms = now_ms;
        s_est.fuse_horizontal_prior();
    }

    // ---- future sensors ---------------------------------------------------
    // Both return invalid fixes today. The calls are here so that the path
    // from sensor to filter is exercised every tick and cannot rot.
    s_est.fuse_range(s_lidar.read());
    s_est.fuse_range(s_uwb.range());

    // ---- publish -----------------------------------------------------------
    //
    // Built and transmitted right here, at the end of the step that
    // produced it. Nothing buffers it and no timer decides when it leaves;
    // see "Event-driven, not timed" at the top of this file.
    //
    // Only control-relevant fields. Attitude in degrees, position, GPS and
    // temperature go out in the status packet below at 20 Hz — see the
    // note on ariadna_nav_state_t.
    ariadna_nav_state_t st;
    memset(&st, 0, sizeof(st));
    st.t_ms  = millis();
    st.px    = s_est.px();  st.py = s_est.py();  st.pz = s_est.pz();
    st.vx    = s_est.vx();  st.vy = s_est.vy();  st.vz = s_est.vz();

    // Acceleration goes out with the filter's ESTIMATED BIAS REMOVED, not
    // as it came off the rotation. That is the number the vehicle is
    // actually being accelerated by, and it is what the Navigation MCU's
    // dead reckoner should integrate across a link gap — handing it the
    // biased version would make it reproduce the very error the bias
    // state exists to take out.
    st.ax    = a_pad[0] - s_est.bias_x();
    st.ay    = a_pad[1] - s_est.bias_y();
    st.az    = a_pad[2] - s_est.bias_z();

    st.qw    = q[0];  st.qx = q[1];  st.qy = q[2];  st.qz = q[3];
    st.ux    = u[0];  st.uy = u[1];  st.uz = u[2];
    st.wx    = w[0];  st.wy = w[1];  st.wz = w[2];
    st.accel_mag = amag_g;

    st.quality = s_est.quality();
    st.flags   = 0;
    if (s_sensor_ok & ARIADNA_SENS_BARO) st.flags |= ARIADNA_NAV_BARO_OK;
    if (s_sensor_ok & ARIADNA_SENS_IMU)  st.flags |= ARIADNA_NAV_IMU_OK;
    if (s_sensor_ok & ARIADNA_SENS_MAG)  st.flags |= ARIADNA_NAV_MAG_OK;
    if (have_gps && gps.valid && gps.fix >= 3) st.flags |= ARIADNA_NAV_GPS_FIX;
    if (s_est.have_pad_reference())      st.flags |= ARIADNA_NAV_PAD_REF;
    if (s_est.have_origin())             st.flags |= ARIADNA_NAV_POS_ORIGIN;
    if (s_est.gps_aided(now_ms))         st.flags |= ARIADNA_NAV_GPS_AIDED;
    if (s_est.zupt())                    st.flags |= ARIADNA_NAV_ZUPT;

    // VALID is what unlocks control on the Navigation MCU, so it demands
    // everything the guidance law actually depends on: an attitude, a
    // vertical channel, and an altitude that means height above the pad.
    // GPS is deliberately NOT required — the landing burn is flown on the
    // barometer and the IMU, and a lost fix must not disarm the vehicle.
    const bool core_ok = (s_sensor_ok & ARIADNA_SENS_IMU) &&
                         (s_sensor_ok & ARIADNA_SENS_BARO) &&
                         s_est.have_pad_reference() &&
                         s_post_pass;
    if (core_ok) st.flags |= ARIADNA_NAV_VALID;

    link_send(ARIADNA_PKT_NAV_STATE, &st, sizeof(st));

    // ---- status, 20 Hz -------------------------------------------------
    // Also sent from this core, so the DMA transmit ring has exactly one
    // producer and needs no lock.
    if (millis() >= next_status_ms) {
        next_status_ms = millis() + 1000 / ARIADNA_STATUS_HZ;

        ariadna_pos_status_t ps;
        memset(&ps, 0, sizeof(ps));
        ps.t_ms         = millis();
        ps.rx_ok        = s_rx.rx_ok;
        ps.rx_bad       = s_rx.rx_crc + s_rx.rx_len;
        ps.fusion_hz    = fusion.getActualFusionHz();
        ps.pad_pressure = s_est.pad_pressure();
        ps.pad_alt_msl  = (have_gps && gps.valid) ? gps.alt_msl : 0.0f;
        ps.roll         = roll;
        ps.pitch        = pitch;
        ps.yaw          = heading;
        ps.temp_c       = sensor.readTemperature();
        ps.sigma_pos_h  = s_est.sigma_pos_h();
        ps.sigma_vel_h  = s_est.sigma_vel_h();
        ps.sensor_ok    = s_sensor_ok;
        ps.imu_overflow = (uint16_t)sensor.imu.getFIFOOverflowCount();
        ps.cal_state    = s_cal_state;

        if (have_gps && gps.valid) {
            ps.lat      = gps.lat_1e7;
            ps.lon      = gps.lon_1e7;
            ps.gps_alt  = gps.alt_msl;
            ps.sats     = gps.sats;
            ps.fix      = gps.fix;
            ps.hdop_x10 = gps.hdop_x10;
        }

        link_send(ARIADNA_PKT_POS_STATUS, &ps, sizeof(ps));
    }
}

// =====================================================================
//  12. CORE 1 — GPS and receiving from the Master
//
//  No sensor bus is touched here, by design. GPS is a UART and the Master
//  link is a UART, so this core can be as bursty as NMEA wants to be
//  without ever delaying a filter step.
//
//  It does not TRANSMIT. Everything outbound goes from core 0, which is
//  what lets the DMA transmit ring be a single-producer structure with no
//  lock in it.
// =====================================================================
/**
 * Send a PCAS command as "$<body>*<checksum>\r\n".
 *
 * The checksum is computed here rather than written out in the constants,
 * so a new command can be added without hand-calculating one — which is
 * the kind of arithmetic that is easy to get wrong and impossible to
 * notice, because a bad checksum makes the receiver ignore the sentence
 * silently.
 */
static void gps_send(const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; ++p) cs ^= (uint8_t)*p;
    char tail[6];
    snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
    GpsPort.print('$');
    GpsPort.print(body);
    GpsPort.print(tail);
    GpsPort.flush();
}

/** True if plausible NMEA arrives at the currently open baud within `ms`. */
static bool gps_listening(uint32_t ms)
{
    NmeaParser probe;
    probe.begin();
    uint8_t sentences = 0;
    const uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < ms) {
        while (GpsPort.available()) {
            if (probe.feed((char)GpsPort.read()) && ++sentences >= 2) return true;
        }
    }
    return false;
}

static void gps_open(uint32_t baud)
{
    GpsPort.end();
    GpsPort.begin(baud);
    delay(50);
}

/**
 * Bring the receiver up at a useful rate.
 *
 * WHY THIS EXISTS. The module boots at 9600 baud emitting 1 Hz. One fix
 * per second is close to worthless to this filter: the vehicle descends
 * at up to 45 m/s, so consecutive fixes are 45 m apart and the whole
 * descent from the guidance entry altitude is covered by about two of
 * them. At 5 Hz the same descent gets ten, which is the difference
 * between GPS anchoring the horizontal channel and merely decorating it.
 *
 * Three steps, each with a way back:
 *
 *   1. Try 115200 first — the module keeps its baud across an MCU reset,
 *      so after the first boot it is probably already there.
 *   2. Otherwise 9600, ask for 115200, reopen and confirm. If the switch
 *      does not take, stay at 9600 and drop to 1 Hz rather than saturate
 *      a link that cannot carry 5 Hz.
 *   3. Trim the sentence set to GGA and RMC, which is everything the
 *      parser reads. If the firmware does not accept that field layout
 *      and goes quiet, put the default set back rather than leave the
 *      receiver mute.
 *
 * Every branch ends with a talking receiver. A silent GPS is survivable
 * here — the landing burn is flown on the barometer and the IMU — but it
 * costs the horizontal channel its only absolute reference, so it is
 * worth this much care to avoid.
 */
static void gps_configure(void)
{
    // PCAS command bodies. Field order for PCAS03 is
    // GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,DHV,LPS,res,res,UTC,GST,res,res,res,TIM
    static const char *PCAS_BAUD_115200  = "PCAS01,5";
    static const char *PCAS_RATE_1HZ     = "PCAS02,1000";
    static const char *PCAS_RATE_5HZ     = "PCAS02,200";
    static const char *PCAS_NMEA_MINIMAL = "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0";
    static const char *PCAS_NMEA_DEFAULT = "PCAS03,1,1,1,1,1,1,0,0,0,0,,,0,0,,,,0";

    gps_open(GPS_FAST_BAUD);
    if (!gps_listening(1500)) {
        gps_open(GPS_BAUD);
        if (!gps_listening(1500)) {
            POS_PRINT("# GPS silent at %lu and %lu - check TX/RX and antenna\n",
                      (unsigned long)GPS_BAUD, (unsigned long)GPS_FAST_BAUD);
            return;
        }
        gps_send(PCAS_BAUD_115200);
        delay(200);
        gps_open(GPS_FAST_BAUD);
        if (!gps_listening(1500)) {
            gps_open(GPS_BAUD);
            gps_send(PCAS_RATE_1HZ);
            POS_PRINT("# GPS stayed at %lu, 1 Hz\n", (unsigned long)GPS_BAUD);
            return;
        }
    }

    gps_send(PCAS_RATE_5HZ);
    delay(100);
    gps_send(PCAS_NMEA_MINIMAL);
    delay(150);
    if (!gps_listening(2000)) {
        gps_send(PCAS_NMEA_DEFAULT);
        delay(150);
        POS_PRINT("# GPS rejected the trimmed sentence set, default restored\n");
    }

    POS_PRINT("# GPS at %lu baud, 5 Hz\n", (unsigned long)GPS_FAST_BAUD);
}

void setup1()
{
    // The Master port is opened by core 0, which also owns the transmit
    // ring. Wait for it rather than racing it.
    while (!s_core0_ready) tight_loop_contents();

    gps_configure();
    s_nmea.begin();
}

void loop1()
{
    // ---- GPS -------------------------------------------------------------
    // Bounded per pass so a burst of sentences cannot monopolise the core.
    // At 9600 baud the module cannot produce more than about one byte per
    // millisecond anyway; the bound is insurance, not throughput.
    for (int i = 0; i < 64 && GpsPort.available(); i++) {
        if (s_nmea.feed((char)GpsPort.read()))
            gps_publish(&s_nmea.fix());
    }

    // ---- inbound from the Master -------------------------------------------
    link_poll();
}
