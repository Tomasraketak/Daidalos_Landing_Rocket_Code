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
//  mismatch silently mirrors every attitude) and section 7 (the vertical
//  filter gains). Section 11 self-tests what can be self-tested.
// =====================================================================

#include "../common/ariadna_link.h"
#include "../common/ariadna_uart_dma.h"

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
static const uint32_t GPS_BAUD         = 9600;   // LK76 default

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

// =====================================================================
//  7. THE ESTIMATOR
//
//  ---------------------------------------------------------------------
//  Vertical channel: a third-order complementary filter
//  ---------------------------------------------------------------------
//  Three states — altitude, vertical velocity, accelerometer bias — driven
//  by the accelerometer and corrected by the barometer:
//
//      h_dot  = vz + K1 * e
//      vz_dot = a  - b + K2 * e
//      b_dot  =         -K3 * e          e = h_baro - h
//
//  With a triple pole at -w, K1 = 3w, K2 = 3w^2, K3 = w^3. One number to
//  tune, and it means something physical: below w the barometer is
//  believed, above it the accelerometer is.
//
//  The bias state is the reason this is third order rather than second.
//  An accelerometer bias of 0.05 m/s^2 — well inside the ICM42688P's spec
//  after a warm-up — integrates to 2.5 m/s of phantom velocity over a
//  10 second descent. The guidance law divides by vertical speed to get
//  time-to-go, so that error goes straight into the ignition altitude.
//  Estimating the bias costs one state and removes the whole term.
//
//  BARO_TRUST during the burn: the landing motor's plume raises static
//  pressure around the airframe, so the barometer reads LOW — it thinks
//  the vehicle is higher than it is, exactly when altitude matters most.
//  The filter therefore drops its barometer bandwidth once the burn
//  starts and coasts on the accelerometer, whose bias it has spent the
//  whole descent learning. This is the single most important line in the
//  vertical channel and it is why the bias state earns its keep.
//
//  ---------------------------------------------------------------------
//  Horizontal channel
//  ---------------------------------------------------------------------
//  Dead reckoning on world-frame acceleration, corrected toward GPS
//  velocity whenever a fix arrives. GPS at 5-10 Hz cannot see the dynamics
//  of a landing burn, so it is used as a slow anchor, not as a
//  measurement: the correction gain is small and the accelerometer carries
//  the short term.
//
//  With no GPS at all the horizontal velocity leaks toward zero on a long
//  time constant. That is a deliberate lie — it is better for the guidance
//  to under-correct a drift it cannot see than to chase an integrator that
//  is running away.
// =====================================================================
class Estimator {
public:
    void begin(void)
    {
        _h = _vz = _bias = 0.0f;
        _vx = _vy = 0.0f;
        _since_corr = 0.0f;
        _pad_pressure = 0.0f;
        _have_pad_ref = false;
        _baro_w       = BARO_W_NOMINAL;
        _quality      = 0;
    }

    /**
     * Capture the pad reference. Called on the ground, at arming.
     *
     * Everything downstream is altitude ABOVE THE PAD, so this is what
     * makes the guidance module's 85 m entry altitude mean the right
     * thing. Without it the vehicle would be flying against sea level.
     */
    void set_pad_reference(float pressure_pa)
    {
        if (!(pressure_pa > 30000.0f && pressure_pa < 110000.0f)) return;
        _pad_pressure = pressure_pa;
        _have_pad_ref = true;
        _h = _vz = _bias = 0.0f;
        _vx = _vy = 0.0f;
        _since_corr = 0.0f;
    }

    bool have_pad_reference(void) const { return _have_pad_ref; }
    float pad_pressure(void)      const { return _pad_pressure; }

    /**
     * Pressure to height above the pad.
     *
     * The barometric formula referenced to the pad rather than to sea
     * level. Ratio form, so the pad temperature and the absolute pressure
     * both cancel — only the RATIO matters, which is what makes this
     * robust to a barometer with an offset.
     */
    float baro_altitude(float pressure_pa) const
    {
        if (!_have_pad_ref || pressure_pa < 1000.0f) return 0.0f;
        return 44330.0f * (1.0f - powf(pressure_pa / _pad_pressure, 0.1902949f));
    }

    /** Raise or lower how much the barometer is believed. */
    void set_baro_trust(bool trusted)
    {
        _baro_w = trusted ? BARO_W_NOMINAL : BARO_W_BURN;
    }

    /**
     * One filter step.
     *
     * az_world is world-frame vertical acceleration with gravity ALREADY
     * REMOVED, in m/s^2, positive up — that is what the fusion library's
     * getGlobalLinearAcceleration() produces.
     */
    void step(float dt, float ax_world, float ay_world, float az_world,
              float h_baro, bool baro_fresh)
    {
        if (dt <= 0.0f || dt > 0.1f) return;      // a stalled tick, not a state

        // --- vertical: predict ------------------------------------------
        const float a_corr = az_world - _bias;
        _h  += _vz * dt + 0.5f * a_corr * dt * dt;
        _vz += a_corr * dt;

        // --- vertical: correct ------------------------------------------
        //
        // ONLY on a genuinely new barometer sample, and with the time since
        // the previous one — not the prediction step.
        //
        // This distinction is the whole reason the parameter is called
        // `baro_fresh` rather than `baro_ok`. The predictor runs at 240 Hz;
        // the barometer produces a sample at about 50 Hz. Applying the
        // correction on every predict step would feed the same measurement
        // in roughly five times, which multiplies the effective correction
        // gain by five and moves the filter's crossover well above the
        // bandwidth it was tuned for. The filter would still converge — it
        // would just be a different filter than the one `_baro_w` claims.
        _since_corr += dt;

        if (baro_fresh) {
            // The correction integrates over the interval since the LAST
            // correction, not over one prediction step. Using the predict
            // dt here would scale every gain by the ratio of the two rates
            // and quietly detune the filter by a factor of five.
            const float cdt = (_since_corr > 0.5f) ? 0.5f : _since_corr;
            _since_corr = 0.0f;

            const float dt = cdt;      // shadows deliberately: the gains below
                                       // are all per correction interval
            const float w  = _baro_w;
            const float K1 = 3.0f * w;
            const float K2 = 3.0f * w * w;
            const float K3 = w * w * w;

            const float e = h_baro - _h;
            _h    += K1 * e * dt;
            _vz   += K2 * e * dt;
            _bias -= K3 * e * dt;

            // Anti-windup. A bias larger than this is not a bias, it is a
            // broken accelerometer, and letting the filter absorb it would
            // hide the fault instead of reporting it.
            if (_bias >  BIAS_LIMIT) _bias =  BIAS_LIMIT;
            if (_bias < -BIAS_LIMIT) _bias = -BIAS_LIMIT;
        }

        // --- horizontal: dead reckon ------------------------------------
        _vx += ax_world * dt;
        _vy += ay_world * dt;

        // Leak toward zero when GPS cannot anchor it. See the note above:
        // an unanchored integrator is worse than a slow one.
        const float leak = expf(-dt / HORIZ_LEAK_TAU);
        _vx *= leak;
        _vy *= leak;

        if (_quality < 255) _quality++;
    }

    /** Fold in a GPS velocity fix. Slow anchor, not a measurement. */
    void fuse_gps_velocity(float ve, float vn, uint8_t hdop_x10)
    {
        // Weight by HDOP: a 0.8 fix is worth believing, a 4.0 fix is not.
        float k = GPS_VEL_GAIN * (10.0f / (float)(hdop_x10 > 5 ? hdop_x10 : 5));
        if (k > GPS_VEL_GAIN) k = GPS_VEL_GAIN;

        _vx += k * (ve - _vx);
        _vy += k * (vn - _vy);
    }

    /**
     * FUTURE: fold in a direct range-to-ground measurement.
     *
     * The hook the lidar and the UWB will use. Deliberately written now,
     * against the RangeFix the future drivers already return, so that
     * bringing either online does not require touching the filter.
     *
     * A range sensor is a far better altimeter than a barometer below
     * about 10 m, and it does not care about the plume — so when this goes
     * live it should also raise the burn-time trust that set_baro_trust()
     * currently lowers.
     */
    void fuse_range(const RangeFix &r)
    {
        if (!r.valid) return;
        (void)r;      // NOT IMPLEMENTED — no sensor produces a valid fix yet
    }

    float h(void)    const { return _h; }
    float vz(void)   const { return _vz; }
    float vx(void)   const { return _vx; }
    float vy(void)   const { return _vy; }
    float bias(void) const { return _bias; }
    uint8_t quality(void) const { return _quality; }

private:
    // Barometer bandwidth, rad/s. Nominal is a compromise between baro
    // noise (which wants it low) and accelerometer drift (which wants it
    // high). BURN is deliberately near-deaf: see the plume note above.
    static constexpr float BARO_W_NOMINAL  = 2.0f;
    static constexpr float BARO_W_BURN     = 0.3f;
    static constexpr float BIAS_LIMIT      = 1.5f;    // m/s^2
    static constexpr float HORIZ_LEAK_TAU  = 20.0f;   // s
    static constexpr float GPS_VEL_GAIN    = 0.15f;

    float   _h = 0.0f, _vz = 0.0f, _bias = 0.0f;
    float   _vx = 0.0f, _vy = 0.0f;
    float   _since_corr = 0.0f;   // seconds of prediction since the last
                                  // barometer correction
    float   _pad_pressure = 0.0f;
    bool    _have_pad_ref = false;
    float   _baro_w = BARO_W_NOMINAL;
    uint8_t _quality = 0;
};

static Estimator s_est;

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

    // --- barometric conversion ------------------------------------------
    {
        Estimator e;
        e.begin();
        e.set_pad_reference(100000.0f);
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

    // --- vertical filter converges --------------------------------------
    // Hold a constant barometric altitude with a biased accelerometer and
    // check that the filter finds both the altitude and the bias. If this
    // fails the gains are wrong, not the sensors.
    {
        Estimator e;
        e.begin();
        e.set_pad_reference(100000.0f);
        const float dt = 1.0f / (float)ESTIMATOR_HZ;
        const int   every = (int)(ESTIMATOR_HZ / BARO_HZ);
        for (int i = 0; i < (int)ESTIMATOR_HZ * 20; i++)
            // Correction only on a fresh sample, exactly as in flight.
            e.step(dt, 0.0f, 0.0f, 0.4f, 100.0f, (i % every) == 0);

        if (fabsf(e.h() - 100.0f) > 1.0f) {
            POS_PRINT("POST filter h %.2f want 100\n", e.h());
            ok = false;
        }
        if (fabsf(e.vz()) > 0.5f) {
            POS_PRINT("POST filter vz %.2f want 0\n", e.vz());
            ok = false;
        }
        if (fabsf(e.bias() - 0.4f) > 0.1f) {
            POS_PRINT("POST filter bias %.3f want 0.4\n", e.bias());
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
        s_est.set_pad_reference(p);
        POS_PRINT("# pad reference %.1f Pa\n", p);
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
    // baro_fresh is true for exactly ONE estimator step per sample. The
    // correction consumes it; see Estimator::step().
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

    float R[3][3], u[3];
    build_rotation(roll, pitch, heading, R);
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
    // World frame, gravity removed, m/s^2. The library's world frame is
    // built from the same heading convention as build_rotation(), so its X
    // is North and Y is East; the estimator wants ENU, hence the swap.
    // BENCH THIS with the board tilted along one axis at a time.
    float an, ae, aup;
    fusion.getGlobalLinearAcceleration(an, ae, aup, ACCEL_UNIT_MS2);
    const float ax_world = ae;      // East
    const float ay_world = an;      // North

    // ---- filter step ------------------------------------------------------
    uint32_t step_now = micros();
    float dt = (float)(step_now - last_step_us) * 1e-6f;
    last_step_us = step_now;
    s_est.step(dt, ax_world, ay_world, aup, h_baro, baro_fresh);

    // The plume corrupts the barometer during the burn. The Master tells us
    // when that is; until then the barometer is the altitude reference.
    s_est.set_baro_trust(!(s_have_master &&
                           s_master.state == ARIADNA_STATE_GUIDANCE));

    // ---- GPS -------------------------------------------------------------
    GpsFix gps;
    bool have_gps = gps_acquire(&gps);
    if (have_gps && gps.valid && gps.t_ms != last_gps_ms) {
        last_gps_ms = gps.t_ms;
        s_est.fuse_gps_velocity(gps.vel_e, gps.vel_n, gps.hdop_x10);
        s_sensor_ok |= ARIADNA_SENS_GPS;
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
    st.h     = s_est.h();
    st.vx    = s_est.vx();
    st.vy    = s_est.vy();
    st.vz    = s_est.vz();
    st.ux    = u[0];  st.uy = u[1];  st.uz = u[2];
    st.wx    = w[0];  st.wy = w[1];  st.wz = w[2];

    TriSenseDataSnapshot snap;
    if (sensor.getSnapshot(snap)) {
        float g2 = snap.accelX * snap.accelX +
                   snap.accelY * snap.accelY +
                   snap.accelZ * snap.accelZ;
        st.accel_mag = sqrtf(g2) * (1.0f / 9.80665f);   // library gives m/s^2
    }

    st.quality = s_est.quality();
    st.flags   = 0;
    if (s_sensor_ok & ARIADNA_SENS_BARO) st.flags |= ARIADNA_NAV_BARO_OK;
    if (s_sensor_ok & ARIADNA_SENS_IMU)  st.flags |= ARIADNA_NAV_IMU_OK;
    if (s_sensor_ok & ARIADNA_SENS_MAG)  st.flags |= ARIADNA_NAV_MAG_OK;
    if (have_gps && gps.valid && gps.fix >= 3) st.flags |= ARIADNA_NAV_GPS_FIX;
    if (s_est.have_pad_reference())      st.flags |= ARIADNA_NAV_PAD_REF;

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
void setup1()
{
    // The Master port is opened by core 0, which also owns the transmit
    // ring. Wait for it rather than racing it.
    while (!s_core0_ready) tight_loop_contents();

    GpsPort.begin(GPS_BAUD);
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
