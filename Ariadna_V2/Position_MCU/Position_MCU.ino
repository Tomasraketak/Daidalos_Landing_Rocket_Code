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
//    core 0   ALL sensor I/O — TriSense fusion, barometer, UWB tag — plus the
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
//  tuning). Section 11 self-tests what can be self-tested, which now
//  includes the horizontal channel, the innovation gate, the UWB
//  multilateration, and the agreement between the rotation matrix and
//  the quaternion.
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

#include "ariadna_link.h"
#include "ariadna_uart_dma.h"
#include "ariadna_kf.h"
#include "ariadna_uwb.h"
#include "ariadna_uwb_link.h"
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

// The RP2350 pins out I2C0 SDA on GPIO 0/4/8/... and I2C1 SDA on GPIO
// 2/6/10/..., so bit 1 of the SDA pin number picks the block. D4 is GPIO6:
// Wire1. Derived rather than written down, so it stays right if the
// harness moves.
#define SENSOR_I2C  ((((PIN_I2C_SDA) >> 1) & 1) ? Wire1 : Wire)

// The ICM42688P will take 12 MHz. 10 MHz leaves margin for the flight
// harness, which is not a bench jumper.
static const uint32_t TRISENSE_SPI_HZ  = 10000000;

// ---- UWB tag: an I2C slave on the sensor bus ---------------------------
//
// The DWM1000 is no longer on this board. The TriSense module drives SCK
// and MOSI itself whenever its IMU is not selected, so nothing else can
// share the IMU's SPI bus. The tag now runs on an anchor PCB flashed with
// ../Uwb_Tag and plugged into the I2CP uSup connector (PSDA/PSCL = D4/D5),
// at ARIADNA_UWB_I2C_ADDR. D0, D1 and D2 are free.

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
//  3. RANGE AND UWB SENSORS
//
//  Two landing-phase sensors live here. Below about ten metres the
//  barometer is being disturbed by the vehicle's own plume and GPS is
//  worth very little, which is exactly where a lidar and a UWB
//  constellation are worth a great deal.
//
//    TF Luna lidar   NOT IMPLEMENTED. Declared, wired into the pipeline
//                    and deliberately inert; see the class.
//    DWM1000 UWB     IMPLEMENTED. A separate tag board ranges to four
//                    surveyed ground anchors and hands the ranges over
//                    I2C; this board multilaterates a position in the
//                    pad frame, which the estimator fuses alongside GPS.
// =====================================================================

// ---------------------------------------------------------------------
//  Is UWB flown at all?
//
//  The ranging library now lives on the tag board, so this board needs
//  nothing extra to build. ARIADNA_UWB_ENABLED 0 still rules UWB out
//  deliberately: the tag is never addressed and the board flies exactly
//  as it did before UWB existed — GPS, barometer, IMU.
// ---------------------------------------------------------------------
#ifndef ARIADNA_UWB_ENABLED
#  define ARIADNA_UWB_ENABLED 1
#endif

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

// =====================================================================
//  THE UWB CONSTELLATION
//
//  Four DWM1000 anchors stand on the ground around the pad; this board
//  carries the tag. Each anchor answers a two-way ranging exchange, the
//  tag gets four distances, and ariadna_uwb.h turns those into a
//  position with an honest per-axis sigma which the Kalman filter treats
//  exactly like a GPS fix — only about twenty times more accurate.
//
//  ---------------------------------------------------------------------
//  WHY THIS IS WORTH THE TROUBLE
//  ---------------------------------------------------------------------
//  The vehicle is trying to land ON the pad, and the landing burn is
//  flown from an estimate of where it is relative to that pad. GPS gives
//  that to about 2.5 m in good conditions. Two and a half metres is the
//  difference between landing on the pad and landing next to it, and no
//  amount of guidance authority recovers a position error the estimator
//  does not know it has. UWB two-way ranging gives 10-30 cm. That is the
//  whole argument.
//
//  ---------------------------------------------------------------------
//  SURVEYING THE ANCHORS — READ THIS BEFORE FLYING
//  ---------------------------------------------------------------------
//  The anchor coordinates below are in the PAD FRAME: metres East, metres
//  North, metres up, measured from the point the vehicle lifts off from,
//  which is the same origin the estimator's px/py/pz use.
//
//  These numbers are an INPUT, not an output. The solver has no way to
//  discover that an anchor was moved two metres from where the table says
//  it is; it will simply produce a position that is confidently wrong by
//  about that much, and the filter will believe it in preference to the
//  GPS, because the sigma it comes with is small. Measure the square with
//  a tape, write the numbers here, and re-flash. The post-fit residual
//  reported in the status packet is the check on that work: a survey
//  error shows up as a residual that will not go below a few tens of
//  centimetres no matter how good the radio conditions are.
//
//  Height matters least and is easiest to get wrong. All four anchors
//  sitting on flat ground at the pad's own level means z = 0 for all of
//  them, and that is the default below. If the ground slopes, measure it.
//
//  ---------------------------------------------------------------------
//  THE SHORT ADDRESS, AND THE ONE ARGUMENT THAT MUST NOT BE DEFAULTED
//  ---------------------------------------------------------------------
//  The tag identifies which anchor a range came from by the anchor's
//  16-bit short address. In the arduino-dw1000 library that address is
//  RANDOMISED at startup unless startAsAnchor() is passed `false` as its
//  third argument — a default that is fine for a demo and fatal here,
//  because a random address matches no row in this table and every range
//  is discarded.
//
//  With `false`, the short address is the FIRST TWO BYTES of the anchor's
//  EUI, little-endian. So an anchor whose EUI begins "01:00:" answers to
//  0x0001. That is the convention the anchor sketch in ../Uwb_Anchor uses
//  and it is why the addresses below are 0x0001 to 0x0004.
// =====================================================================

/**
 * A range older than this is dropped rather than extrapolated.
 *
 * 200 ms lets an anchor miss one exchange at 10 Hz and still count,
 * which is what keeps a marginal link contributing instead of flapping
 * in and out of the solution.
 *
 * It is not longer because the ranges inside a set are NOT simultaneous,
 * and the difference is corrected by a velocity extrapolation — see
 * ariadna_uwb_derotate(). That correction is first order, so what it
 * leaves behind grows as the SQUARE of the age: at 4 g, 200 ms leaves
 * 0.8 m and 400 ms would leave 3.2 m. The residual gate would then start
 * throwing away good fixes at exactly the accelerations the landing burn
 * produces.
 *
 * Must stay at or below ariadna_uwb_cfg_t::max_age_s, which screens on
 * the same quantity inside the solver.
 */
static const uint32_t UWB_RANGE_MAX_AGE_MS = 200;

/** Rate the multilateration itself runs at. Ranges arrive far slower. */
static const uint32_t UWB_SOLVE_HZ = 25;

/**
 * Surveyed anchor positions, pad-ENU metres. THE DEFAULT IS A 40 m
 * SQUARE CENTRED ON THE PAD — replace it with your tape measure.
 *
 * A square is close to the best geometry available on flat ground: the
 * two horizontal axes are equally well observed and the solution is
 * insensitive to any one anchor. A line is the worst, and three anchors
 * in a rough line is a common accident when the fourth is placed in a
 * hurry. Spread them.
 */
static ariadna_uwb_anchor_t s_uwb_anchors[ARIADNA_UWB_MAX_ANCHORS] = {
    // addr,  enabled, pad,   x,      y,     z
    { 0x0001, 1, 0, -20.0f, -20.0f, 0.0f },
    { 0x0002, 1, 0,  20.0f, -20.0f, 0.0f },
    { 0x0003, 1, 0,  20.0f,  20.0f, 0.0f },
    { 0x0004, 1, 0, -20.0f,  20.0f, 0.0f },
    { 0x0000, 0, 0,   0.0f,   0.0f, 0.0f },
    { 0x0000, 0, 0,   0.0f,   0.0f, 0.0f },
    { 0x0000, 0, 0,   0.0f,   0.0f, 0.0f },
    { 0x0000, 0, 0,   0.0f,   0.0f, 0.0f },
};
static const uint8_t UWB_ANCHOR_COUNT = 4;

/**
 * Client for the UWB tag board.
 *
 * The DWM1000 and its ranging stack run on a separate XIAO (../Uwb_Tag)
 * that answers on the sensor I2C bus at ARIADNA_UWB_I2C_ADDR with the
 * latest range to every anchor it hears, each with its age. This class
 * reads that frame, decides which ranges are new, and from there on is
 * the same driver it always was: per-anchor slots, the jump test, the
 * sigma model, the snapshot for the solver. Everything that judges a
 * range stays on this board, next to the anchor table and the speed
 * estimate it needs.
 *
 * ---------------------------------------------------------------------
 * ROBUSTNESS IS THE WHOLE DESIGN, NOT A FEATURE OF IT
 * ---------------------------------------------------------------------
 * This is the only sensor in the vehicle whose availability depends on
 * hardware that is not bolted to the vehicle. Anchors run off their own
 * batteries, sit in the grass, and are line-of-sight devices pointed at
 * something that is about to light a rocket motor between them. Assume
 * they will fail, individually and together, at the worst moment — and
 * now the tag board and its cable can fail too.
 *
 * So every layer here is written to degrade rather than break:
 *
 *   * Each anchor has its OWN slot with its own timestamp. One that stops
 *     answering ages out of the observation set on its own; nothing
 *     needs to detect it, and the other three carry on unchanged.
 *   * A range that is not finite, is negative, or jumps further than the
 *     vehicle could have moved is dropped at the slot, before it can
 *     reach the solver.
 *   * A frame that is short, fails its CRC or comes from a different
 *     protocol version is discarded whole. A tag that is unplugged,
 *     unpowered or rebooting is a NACK, which costs tens of
 *     microseconds; every slot then ages out and the solver is simply
 *     not run.
 *   * Nothing here allocates. The one blocking call is the frame read,
 *     about 1.7 ms at 400 kHz, bounded by the Wire timeout set in
 *     sensors_begin().
 *
 * ---------------------------------------------------------------------
 * SHARING THE SENSOR BUS
 * ---------------------------------------------------------------------
 * The tag sits on the same I2C wires as the BMP580 and the AK09918C, all
 * of them driven from CORE 0 ONLY, which is what makes the bus safe
 * without a lock. If you ever move a reader of any of them to core 1,
 * you must add one.
 */
class UwbTag {
public:
    /**
     * Remember the bus and try the tag once, so the POST can report it.
     * A tag that comes up later — it boots its radio for a few hundred
     * milliseconds — is picked up by poll(); nothing here is final.
     */
    bool begin(TwoWire &wire)
    {
        _wire = &wire;
        _fresh = 0;
        _gen = 0;
        _last_good_ms = 0;
        _flags = 0;
        for (uint8_t i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) slot_clear(i);
        for (uint8_t i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
            _seen_addr[i] = 0;
            _seen_count[i] = 0;
        }
#if ARIADNA_UWB_ENABLED
        read_frame();
#endif
        return present();
    }

    /**
     * Read the tag's frame and take in every range that is new since the
     * last read. Call it right before the solve, so the solver sees the
     * freshest ranges there are.
     */
    void poll(void)
    {
#if ARIADNA_UWB_ENABLED
        read_frame();
#endif
    }

    /**
     * Build the observation set for the solver.
     *
     * Returns how many anchors are fresh. Every slot is written, so the
     * caller never reads a stale observation for an anchor that has gone
     * quiet — the `valid` flag is cleared explicitly rather than left
     * over from last time.
     */
    uint8_t snapshot(ariadna_uwb_obs_t *obs, uint32_t now_ms) const
    {
        uint8_t n = 0;
        for (uint8_t i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) {
            obs[i].range_m = 0.0f;
            obs[i].sigma_m = 0.0f;
            obs[i].age_s   = 0.0f;
            obs[i].valid   = 0;
            obs[i]._pad[0] = obs[i]._pad[1] = obs[i]._pad[2] = 0;

            const Slot &s = _slot[i];
            if (!s.have) continue;
            const uint32_t age_ms = (uint32_t)(now_ms - s.t_ms);
            if (age_ms > UWB_RANGE_MAX_AGE_MS) continue;

            obs[i].range_m = s.range_m;
            obs[i].sigma_m = s.sigma_m;
            // THE AGE IS THE POINT, not bookkeeping. The anchors answer
            // one at a time, so these ranges were taken from several
            // different places, and at 20 m/s those places are metres
            // apart. ariadna_uwb_derotate() uses this to bring them all
            // to one epoch before the solve.
            obs[i].age_s   = (float)age_ms * 1e-3f;
            obs[i].valid   = 1;
            n++;
        }
        return n;
    }

    /**
     * Tell the driver roughly how fast the vehicle is going.
     *
     * Used only by the jump test in accept(). Written and read on core 0
     * at the solve rate, so there is nothing to synchronise.
     */
    void set_speed_hint(float mps)
    {
        _speed_hint = (mps == mps && mps > 0.0f && mps < 200.0f) ? mps : 0.0f;
    }

    /** The tag answered a good frame within the last second and its
     *  radio is up. */
    bool present(void) const
    {
        return _last_good_ms != 0 &&
               (uint32_t)(millis() - _last_good_ms) < UWB_LINK_TIMEOUT_MS &&
               (_flags & ARIADNA_UWB_LF_RADIO_UP);
    }
    bool     healthy(void)     const { return present() && _fresh > 0; }
    uint32_t generation(void)  const { return _gen; }
    uint8_t  fresh(void)       const { return _fresh; }
    uint32_t rejected(void)    const { return _rejected; }
    uint32_t link_errors(void) const { return _link_errors; }

    /** Age out slots. Called once per solve, so `fresh` is never stale. */
    void age(uint32_t now_ms)
    {
        uint8_t n = 0;
        for (uint8_t i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) {
            if (!_slot[i].have) continue;
            if ((uint32_t)(now_ms - _slot[i].t_ms) > UWB_RANGE_MAX_AGE_MS) {
                _slot[i].have = false;      // keep last_range: a returning
                continue;                   // anchor is not a new one
            }
            n++;
        }
        _fresh = n;
    }

private:
    // No good frame for this long and the tag counts as gone.
    static const uint32_t UWB_LINK_TIMEOUT_MS = 1000;

    // Ranging sigma. NOT the DWM1000 datasheet figure, which is about
    // 10 cm in a laboratory. This is what a range is worth on a launch
    // site with a metal airframe rotating between the antennas, and it is
    // deliberately pessimistic: the cost of an over-large sigma is that
    // the filter converges a little more slowly, while the cost of an
    // over-small one is that it believes a reflection.
    static constexpr float UWB_SIGMA_BASE_M = 0.25f;
    // Ranging error grows with distance as the link margin falls.
    static constexpr float UWB_SIGMA_PER_M  = 0.004f;

    // Jump test. A range that disagrees with the previous one from the
    // SAME anchor by more than the vehicle could possibly have moved in
    // between is a decode error or a reflection, not motion.
    //
    // THE THRESHOLD HAS TO SCALE WITH SPEED AND WITH THE GAP. A fixed
    // 15 m is a sound test on a bench and a broken one on a rocket: at
    // 50 m/s the range to an anchor legitimately changes by 5 m between
    // consecutive exchanges and by 25 m across a 500 ms gap. A fixed
    // threshold would start rejecting the fastest part of the descent —
    // that is, it would look most like a fault exactly when the data is
    // most real.
    //
    // So the allowance is a floor for ranging noise plus what the
    // vehicle could have covered: its own estimated speed, with wide
    // margin for that estimate being wrong.
    static constexpr float UWB_JUMP_BASE_M    = 3.0f;    // noise floor
    static constexpr float UWB_JUMP_MARGIN_MS = 25.0f;   // m/s of headroom

    struct Slot {
        float    range_m;
        float    sigma_m;
        float    last_range;     // survives a dropout, for the jump test
        uint32_t t_ms;
        uint32_t last_ms;
        bool     have;
    };

    void slot_clear(uint8_t i)
    {
        _slot[i].range_m    = 0.0f;
        _slot[i].sigma_m    = 0.0f;
        _slot[i].last_range = 0.0f;
        _slot[i].t_ms       = 0;
        _slot[i].last_ms    = 0;
        _slot[i].have       = false;
    }

    /**
     * One read of the tag's frame. A range is new when its tag slot shows
     * a different anchor or a different count than the last read did;
     * the same range read twice is fed in once.
     *
     * The range's time is THIS board's clock at the end of the read,
     * minus the age the tag reported. The two boards share no clock, and
     * this is the only place the difference matters.
     */
    void read_frame(void)
    {
        if (!_wire) return;
        uint8_t buf[ARIADNA_UWB_LINK_FRAME_SIZE];
        const size_t want = sizeof(buf);
        const size_t got  = _wire->requestFrom((uint8_t)ARIADNA_UWB_I2C_ADDR, want);
        size_t n = 0;
        while (_wire->available()) {
            const int c = _wire->read();
            if (n < want) buf[n++] = (uint8_t)c;
        }
        const uint32_t t_done = millis();

        ariadna_uwb_link_frame_t f;
        if (got != want || n != want ||
            ariadna_uwb_link_decode(buf, n, &f) != ARIADNA_UWB_LINK_OK) {
            // Nobody there is a NACK and got == 0. Counted only once the
            // tag has been seen, so an absent tag does not read as a
            // stream of errors.
            if (_last_good_ms != 0 || got != 0) _link_errors++;
            return;
        }
        _last_good_ms = t_done ? t_done : 1;
        _flags = f.flags;

        for (uint8_t i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
            const ariadna_uwb_link_slot_t &s = f.slot[i];
            if (!s.valid) { _seen_addr[i] = 0; continue; }
            if (s.addr == _seen_addr[i] && s.count == _seen_count[i]) continue;
            _seen_addr[i]  = s.addr;
            _seen_count[i] = s.count;
            accept(s.addr, s.range_m, (float)s.rxp_dbm, t_done - s.age_ms);
        }
    }

    /**
     * Accept one range into an anchor's slot.
     *
     * This is where a bad measurement is stopped. Everything downstream
     * — the solver, the innovation gate, the filter — is cheaper and more
     * honest if nonsense never gets past here.
     */
    void accept(uint16_t addr, float range_m, float rx_power_dbm, uint32_t t_ms)
    {
        const int slot = ariadna_uwb_find(s_uwb_anchors, UWB_ANCHOR_COUNT, addr);
        if (slot < 0) { _rejected++; return; }     // not one of ours

        if (!(range_m == range_m)) { _rejected++; return; }          // NaN
        if (!(range_m > 0.05f && range_m < 300.0f)) { _rejected++; return; }

        Slot &s = _slot[slot];

        // Jump test, but only against a RECENT previous range. After a
        // dropout the vehicle really may be somewhere else, and refusing
        // to re-acquire an anchor that went quiet for a second would turn
        // a momentary outage into a permanent one.
        const float gap_s = (s.last_ms != 0)
                                ? (float)(uint32_t)(t_ms - s.last_ms) * 1e-3f
                                : 0.0f;
        const float allowed = UWB_JUMP_BASE_M +
                              (_speed_hint + UWB_JUMP_MARGIN_MS) * gap_s;

        if (s.last_ms != 0 && (uint32_t)(t_ms - s.last_ms) < 500u &&
            fabsf(range_m - s.last_range) > allowed) {
            _rejected++;
            s.last_range = range_m;    // but remember it: two agreeing
            s.last_ms    = t_ms;       // outliers are a real move
            return;
        }

        float sigma = UWB_SIGMA_BASE_M + UWB_SIGMA_PER_M * range_m;
        // A weak signal is a long or an obstructed path, and both make
        // the range read long. Widen the sigma rather than guess a
        // correction: the solver weights by it and the residual test
        // catches what is left.
        if (rx_power_dbm < -85.0f) sigma *= 2.0f;
        else if (rx_power_dbm < -80.0f) sigma *= 1.4f;

        s.range_m    = range_m;
        s.sigma_m    = sigma;
        s.t_ms       = t_ms;
        s.last_range = range_m;
        s.last_ms    = t_ms;
        s.have       = true;
        _gen++;
    }

    TwoWire *_wire = nullptr;
    uint8_t  _flags = 0;
    uint8_t  _fresh = 0;
    uint32_t _gen = 0;
    uint32_t _rejected = 0;
    uint32_t _link_errors = 0;
    uint32_t _last_good_ms = 0;
    float    _speed_hint = 0.0f;
    Slot     _slot[ARIADNA_UWB_MAX_ANCHORS];
    uint16_t _seen_addr[ARIADNA_UWB_LINK_SLOTS];
    uint8_t  _seen_count[ARIADNA_UWB_LINK_SLOTS];
};

static LidarTFLuna s_lidar;
static UwbTag      s_uwb;

// The solver's limits, filled in at setup(). Held at file scope so the
// self test can check them against the anchor geometry.
static ariadna_uwb_cfg_t s_uwb_cfg;

// The most recent solve, kept for the status packet and the console.
static ariadna_uwb_fix_t s_uwb_fix;

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
//  The landing motor's plume disturbs static pressure around the
//  airframe, so the barometric altitude is wrong by metres exactly when
//  altitude matters most. The SIGN depends on where the static port sits
//  relative to the recirculation and is not worth asserting from a desk;
//  what is certain is the magnitude and that it is transient. The old
//  filter handled this by dropping its barometer bandwidth during the
//  burn. This one inflates the barometer's measurement variance instead.
//
//  Not knowing the sign is itself an argument for inflating R rather than
//  applying a correction: a variance says "this measurement is bad",
//  which is true either way, while a correction has to commit to a
//  direction and doubles the error if it commits wrongly.
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
        _uwb_rejects  = 0;
        _last_gps_ms  = 0;
        _last_uwb_ms  = 0;
        _uwb_sigma_h  = 0.0f;
        _uwb_resid    = 0.0f;
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
        _baro_var    = s * s;
        _baro_trusted = trusted;
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
    void fuse_baro(float h_baro_raw, uint32_t now_ms)
    {
        if (!_have_pad_ref) return;

        // Remember the RAW reading. fuse_uwb() compares it against the
        // UWB height to learn the trim below, and it must compare
        // against what the sensor actually said, not against a value
        // this filter has already corrected — otherwise the trim is
        // measuring its own output and converges to whatever it likes.
        //
        // The clock is a PARAMETER, not a millis() call, for the same
        // reason every other method here takes one: a filter that reads
        // the wall clock cannot be run against a synthetic descent on a
        // workstation, and this path decides the altitude the vehicle
        // lands on.
        _last_h_baro    = h_baro_raw;
        _last_h_baro_ms = now_ms ? now_ms : 1;

        const float h_baro = h_baro_raw - ariadna_baro_trim_value(&_trim);

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
     * The hook the lidar will use. Deliberately written now, against the
     * RangeFix the future driver already returns, so that bringing it
     * online does not require touching the filter — with a Kalman filter
     * it really is one ariadna_kf_correct() call, because the sensor's own
     * sigma is already the R the update wants.
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

    /**
     * Fold in a UWB position fix.
     *
     * ------------------------------------------------------------------
     * WHY THIS IS ALMOST THE SAME CODE AS fuse_gps(), AND WHY THAT IS THE
     * POINT
     * ------------------------------------------------------------------
     * A UWB fix arrives here as a position with a per-axis sigma —
     * exactly the shape of a GPS fix, and exactly the shape
     * ariadna_kf_correct() consumes. Everything that makes UWB different
     * (four anchors, a Gauss-Newton solve, a constellation that loses
     * members) has already been dealt with in ariadna_uwb.h, and none of
     * it reaches the filter. The filter sees a measurement and a
     * variance, which is all a filter should ever see.
     *
     * That is what makes a partial constellation safe. A three-anchor
     * horizontal-only fix, a two-anchor stretched one and a single-anchor
     * line-of-sight hint all come through this same call; the difference
     * between them is entirely in `use_*` and in the sigmas. There is no
     * mode logic here to get wrong.
     *
     * ------------------------------------------------------------------
     * THE THREE THINGS THIS DOES BEYOND CALLING THE FILTER
     * ------------------------------------------------------------------
     *  1. AGE COMPENSATION. Like GPS, though the numbers are far smaller:
     *     a solve is at most a few tens of milliseconds old.
     *  2. A SIGMA FLOOR. The solver's covariance is the covariance of the
     *     RANGES it was given, and it does not know about a mis-surveyed
     *     anchor, an antenna delay calibration that is 20 cm out, or the
     *     tag's own mounting offset. UWB_SIGMA_FLOOR is the honest total
     *     of what the geometry cannot see, and without it the filter
     *     would trust UWB far past the point where UWB is right.
     *  3. RE-ANCHORING, on the same argument as GPS and the barometer: a
     *     run of consecutive rejections means the filter has drifted away
     *     from a sensor that is telling the truth, and a gate that never
     *     re-opens is a filter that has quietly stopped navigating.
     *     Deliberately SHORTER than the GPS one, because UWB is the more
     *     accurate of the two: if the two disagree persistently, the one
     *     to believe is UWB.
     */
    void fuse_uwb(const ariadna_uwb_fix_t &f, float age_s, uint32_t now_ms)
    {
        if (f.mode == ARIADNA_UWB_SOL_NONE)   return;
        if (!(f.use_x || f.use_y || f.use_z)) return;
        if (age_s < 0.0f) age_s = 0.0f;
        if (age_s > 0.5f) return;             // too old to be about now

        bool any = false, rejected = false;

        if (f.use_x) {
            const float s = inflate(f.sigma_x);
            const float z = f.x + _kx.x[1] * age_s;
            any = true;
            if (!ariadna_kf_correct(&_kx, 0, z, s * s, GATE_SIGMA)) rejected = true;
        }
        if (f.use_y) {
            const float s = inflate(f.sigma_y);
            const float z = f.y + _ky.x[1] * age_s;
            any = true;
            if (!ariadna_kf_correct(&_ky, 0, z, s * s, GATE_SIGMA)) rejected = true;
        }
        // --- the vertical -------------------------------------------------
        //
        // Offered only when the anchor geometry actually observed it,
        // and then only from FOUR anchors. Three coplanar anchors can
        // produce a height whose covariance looks acceptable while
        // resting on a single weak direction cosine; the horizontal is
        // worth having from three, the vertical is not.
        //
        // WHY IT IS FUSED AT ALL, GIVEN THE BAROMETER EXISTS. Two
        // reasons, and they are strongest exactly where it matters:
        //
        //   * during the landing burn the plume corrupts static pressure
        //     and set_baro_trust() inflates the barometer's variance to
        //     BARO_SIGMA_BURN — twenty-five metres, which is a formal
        //     way of saying "ignore this". UWB does not care about the
        //     plume. In that window it is the only absolute vertical
        //     reference the vehicle has;
        //   * the barometer can be biased — weather drift over a long
        //     countdown, ground effect near the pad — and a bias is
        //     invisible to a sensor that only ever compares against
        //     itself. UWB is an independent measurement of the same
        //     state, which is what makes the bias observable at all.
        //
        // WHY IT IS FUSED *LIGHTLY* BELOW UWB_Z_LIGHT_ALT. Because the
        // geometry is degrading fast down there: with a 40 m anchor
        // square, sigma_z is 0.37 m at 10 m, 0.72 m at 5 m and 1.77 m at
        // 2 m, and the vertical is the state the guidance law flies. The
        // scale factor buys margin against a marginal-geometry height
        // dragging the altitude around in the last seconds, while still
        // letting a persistent disagreement be corrected over about a
        // second. The barometer stays the fast, low-noise altimeter; UWB
        // supplies the slow truth.
        //
        // And the vertical is NEVER re-anchored from UWB — see below.
        if (f.use_z && f.n_used >= UWB_Z_MIN_ANCHORS) {
            float s = inflate(f.sigma_z);
            if (_kz.x[0] < UWB_Z_LIGHT_ALT) s *= UWB_Z_LIGHT_SCALE;
            const float z = f.z + _kz.x[1] * age_s;
            any = true;
            if (!ariadna_kf_correct(&_kz, 0, z, s * s, GATE_SIGMA)) rejected = true;
            else                                    learn_baro_trim(f, z, now_ms);
        }
        if (!any) return;

        if (!rejected) {
            _uwb_rejects = 0;
            _last_uwb_ms = now_ms ? now_ms : 1;
            _uwb_sigma_h = 0.5f * (inflate(f.sigma_x) + inflate(f.sigma_y));
            _uwb_resid   = f.residual_rms;
            return;
        }

        if (++_uwb_rejects >= UWB_MAX_REJECTS) {
            if (f.use_x)
                ariadna_kf_reset(&_kx, f.x, _kx.x[1], inflate(f.sigma_x) * 3.0f,
                                 5.0f, BIAS_LIMIT);
            if (f.use_y)
                ariadna_kf_reset(&_ky, f.y, _ky.x[1], inflate(f.sigma_y) * 3.0f,
                                 5.0f, BIAS_LIMIT);
            // NOT the vertical. The barometer is the vertical channel's
            // absolute reference and it has its own re-anchor; letting a
            // weakly observed UWB height reset it as well would give two
            // sensors authority over one state, which is how a filter
            // ends up oscillating between two answers.
            _uwb_rejects = 0;
        }
    }

    /** True if a UWB fix was ACCEPTED recently, not merely produced. */
    bool uwb_aided(uint32_t now_ms) const
    {
        return _last_uwb_ms != 0 && (uint32_t)(now_ms - _last_uwb_ms) < 1000u;
    }

    /**
     * The learned barometer offset, metres, positive = reads high.
     *
     * Worth watching on the pad console. A value that settles at a few
     * tens of centimetres is a barometer being calibrated. A value
     * pinned at UWB_TRIM_LIMIT_M is the filter telling you the anchor
     * survey and the pad reference disagree about where the ground is.
     */
    float baro_trim(void) const { return ariadna_baro_trim_value(&_trim); }

    /**
     * True when the trim is sitting on its clamp.
     *
     * This is not a barometer being calibrated any more — it is the
     * anchor survey and the pad reference disagreeing about where the
     * ground is by more than a barometer plausibly drifts. It gets its
     * own annunciator code because it is fixable in the ten minutes
     * before launch and unfixable afterwards.
     */
    bool baro_trim_saturated(void) const
    {
        return ariadna_baro_trim_saturated(&_trim) != 0;
    }

    float uwb_sigma_h(void) const { return _uwb_sigma_h; }
    float uwb_resid(void)   const { return _uwb_resid; }
    uint8_t uwb_rejects(void) const { return _uwb_rejects; }

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

    // --- UWB ------------------------------------------------------------
    // Everything the solver's covariance cannot know about, added in
    // quadrature: anchor survey error, antenna delay calibration, and the
    // offset between the tag's antenna and the vehicle's centre of mass.
    // Twelve centimetres is optimistic for a tape-measured square and
    // pessimistic for a properly surveyed one; measure yours.
    static constexpr float UWB_SIGMA_FLOOR = 0.12f;   // m
    // Four consecutive rejected fixes is under a second at the solve
    // rate. Shorter than GPS_MAX_REJECTS on purpose — see fuse_uwb().
    static constexpr uint8_t UWB_MAX_REJECTS = 4;

    // --- the vertical ---------------------------------------------------
    // A UWB height is accepted only from a full constellation. Three
    // coplanar anchors give a horizontal fix worth having and a vertical
    // one that is not.
    static constexpr uint8_t UWB_Z_MIN_ANCHORS = 4;
    // Below this the anchor geometry is visibly degrading, so the height
    // goes in with its sigma scaled up: enough to correct a standing
    // barometer error over about a second, not enough to move the
    // altitude the guidance law is flying in one step.
    static constexpr float UWB_Z_LIGHT_ALT   = 10.0f;   // m
    static constexpr float UWB_Z_LIGHT_SCALE = 2.0f;

    // --- the barometer trim ---------------------------------------------
    // See learn_baro_trim(). The time constant is deliberately long
    // compared with anything the vehicle does: this corrects a standing
    // offset, never a transient.
    static constexpr float UWB_TRIM_TAU_S      = 2.5f;   // s
    // A barometer can plausibly be a metre out over a long countdown.
    // More than that is a survey error, and the clamp makes it visible
    // rather than absorbing it.
    static constexpr float UWB_TRIM_LIMIT_M    = 1.5f;   // m
    static constexpr float UWB_TRIM_MAX_DISCREP = 3.0f;  // m
    static constexpr float UWB_TRIM_SIGMA_MAX  = 1.0f;   // m

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
        _uwb_rejects  = 0;
        _zupt = false;

        // The barometer trim is cleared HERE and only here. It describes
        // the barometer relative to a particular pad reference, so it is
        // meaningless once a new one is taken — and reset_states() is
        // reached from exactly the two places that take one, begin() and
        // set_pad_reference(). It deliberately survives everything else,
        // including a filter re-anchor: a calibration learned during the
        // descent is most valuable in the last two metres, which is
        // exactly where UWB stops being able to see height.
        ariadna_baro_trim_reset(&_trim, UWB_TRIM_TAU_S, UWB_TRIM_LIMIT_M);
        _last_h_baro    = 0.0f;
        _last_h_baro_ms = 0;
        _baro_trusted   = true;
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

    uint8_t  _uwb_rejects = 0;
    uint32_t _last_uwb_ms = 0;
    float    _uwb_sigma_h = 0.0f;
    float    _uwb_resid   = 0.0f;

    bool     _baro_trusted   = true;
    ariadna_baro_trim_t _trim;         // how much the barometer reads HIGH
    float    _last_h_baro    = 0.0f;   // m, RAW, before the trim
    uint32_t _last_h_baro_ms = 0;

    /** Solver sigma, floored by what the solver cannot see. */
    static float inflate(float sigma)
    {
        return sqrtf(sigma * sigma + UWB_SIGMA_FLOOR * UWB_SIGMA_FLOOR);
    }

    /**
     * Learn the barometer's bias from UWB, slowly.
     *
     * ------------------------------------------------------------------
     * WHAT THIS IS FOR
     * ------------------------------------------------------------------
     * Fusing a UWB height and a barometric height together AVERAGES a
     * barometer bias; it does not remove it. With the barometer at 50 Hz
     * and sigma 0.5 m against UWB at 25 Hz and sigma 0.7 m, a one-metre
     * offset comes out as roughly 0.8 m of residual error — better, and
     * still wrong, and still wrong in the direction that matters.
     *
     * Removing it needs the bias to be estimated as its own quantity.
     * That is all this is: a slow, clamped, heavily gated first-order
     * observer of the difference between the two independent
     * measurements, subtracted from the barometer in fuse_baro().
     *
     * ------------------------------------------------------------------
     * THE THREE GATES, AND WHY EACH ONE IS LOAD-BEARING
     * ------------------------------------------------------------------
     *  1. ONLY WHILE THE BAROMETER IS TRUSTED. During the burn the
     *     difference between the two heights is mostly PLUME, not bias.
     *     Learning then would store the plume as a permanent offset and
     *     then apply it after burnout, when the plume is gone — turning
     *     a transient into a lasting altitude error. This is the gate
     *     that would hurt most if it were missing.
     *  2. ONLY ON GOOD GEOMETRY. sigma_z must be small; near the ground
     *     it is not, and a trim learned from a 2 m sigma is noise.
     *  3. ONLY SMALL DISCREPANCIES, AND CLAMPED. A metre-class offset is
     *     a barometer. A five-metre one is a mis-surveyed anchor plane,
     *     and absorbing that would silently move the vehicle's idea of
     *     the ground. The clamp means such a survey error shows up as a
     *     trim pinned at its limit — visible in the status packet and on
     *     the pad console — instead of being quietly accommodated.
     *
     * The time constant is seconds, so this cannot move the altitude
     * suddenly no matter what UWB does. If UWB drops out in the last two
     * metres, where the geometry gives up anyway, the trim FREEZES at
     * what it had learned: the vehicle lands on a barometer that has
     * been calibrated against UWB, which is the whole point.
     */
    void learn_baro_trim(const ariadna_uwb_fix_t &f, float z_uwb, uint32_t now)
    {
        if (!_baro_trusted)                     return;   // gate 1: the plume
        if (f.sigma_z > UWB_TRIM_SIGMA_MAX)     return;   // gate 2: geometry
        if (_last_h_baro_ms == 0 || now == 0)   return;

        if ((uint32_t)(now - _last_h_baro_ms) > 200u) return;  // stale baro

        // Positive means the barometer reads HIGH.
        const float d = _last_h_baro - z_uwb;
        if (!(d == d))                          return;
        if (fabsf(d) > UWB_TRIM_MAX_DISCREP)    return;   // gate 3: not a bias

        // The arithmetic itself is in ariadna_kf.h, where est_test.cpp can
        // reach it. What stays here is only the gating, which depends on
        // types that live in this sketch.
        (void)ariadna_baro_trim_update(&_trim, d, now);
    }
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
//  9. ANNUNCIATOR
//
//  One WS2812 on the XIAO, saying what this board thinks of itself.
//
//  The Master's LED reports the MISSION state. This one reports the
//  NAVIGATION state, and they are different questions: the mission can be
//  perfectly ready to arm while the estimator has no pad reference, and
//  nothing on the Master's LED would show it.
//
//  Read the colour, then count the flashes:
//
//    solid blue        booting
//    red, 1 flash      self test failed, or the IMU is not talking
//    red, 2 flashes    no barometer — the vertical channel has no
//                      absolute reference and the vehicle cannot fly
//    amber slow        alive, but no pad reference yet: the Master has
//                      not sent SET_PAD_REF, so px/py/pz mean nothing
//    yellow, 3 flashes the barometer trim has hit its clamp: UWB and the
//                      pad reference disagree about where the ground is
//                      by more than a barometer drifts. Re-measure the
//                      anchor survey BEFORE flying.
//    magenta slow      pad reference taken, still waiting for a GPS fix
//                      and with no UWB — dead reckoning only
//    cyan slow         navigating on GPS
//    green pulse       navigating on GPS, UWB constellation alive
//    green solid       UWB fixes are being ACCEPTED by the filter. This
//                      is the one to wait for before arming: it means
//                      the vehicle knows where the pad is to a decimetre.
//
//  The distinction between the last two is the one worth understanding
//  from behind the safety line. A pulsing green says the anchors are
//  powered; a solid green says they are powered, surveyed correctly, and
//  agreeing with the rest of the navigation solution.
// =====================================================================
static ariadna_rgb_t s_rgb;

// Dim on purpose. This LED is read from a few metres away in daylight,
// and full brightness on a WS2812 is both blinding at night and a
// noticeable current draw next to the sensors it sits beside.
static const uint8_t POS_RGB_BRIGHTNESS = 60;

/**
 * Choose the annunciator's message from the flags that were just built.
 *
 * Driven from the nav_state flags rather than from the estimator
 * directly, so that what the LED says and what the vehicle transmits
 * cannot disagree. If the LED is green, the packet said VALID.
 */
static void annunciate(uint16_t nav_flags, uint32_t now_ms)
{
    (void)now_ms;

    if (!s_post_pass || !(nav_flags & ARIADNA_NAV_IMU_OK)) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_BLINK_N, 1);
        return;
    }
    if (!(nav_flags & ARIADNA_NAV_BARO_OK)) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_BLINK_N, 2);
        return;
    }
    if (!(nav_flags & ARIADNA_NAV_PAD_REF)) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_SLOW, 0);
        return;
    }
    // Ranked ABOVE the healthy states on purpose. A saturated barometer
    // trim means UWB and the pad reference disagree about where the
    // ground is by more than a barometer drifts — a survey error, which
    // is fixable now and not fixable once the vehicle has left the rail.
    // It must not be hidden behind a reassuring green.
    if (s_est.baro_trim_saturated()) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_YELLOW, ARIADNA_PAT_BLINK_N, 3);
        return;
    }
    if (nav_flags & ARIADNA_NAV_UWB_AIDED) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_SOLID, 0);
        return;
    }
    if (nav_flags & ARIADNA_NAV_UWB_OK) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_PULSE, 0);
        return;
    }
    if (nav_flags & ARIADNA_NAV_GPS_AIDED) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_CYAN, ARIADNA_PAT_SLOW, 0);
        return;
    }
    ariadna_rgb_set(&s_rgb, ARIADNA_C_MAGENTA, ARIADNA_PAT_SLOW, 0);
}

// =====================================================================
//  10. SENSOR INITIALISATION
// =====================================================================
static bool sensors_begin(void)
{
    // Route both buses to the flight harness pins BEFORE begin(). The
    // arduino-pico core latches the pin mapping at begin(), and
    // TriSense::beginAll() calls wire.begin()/SPI.begin() internally with
    // no pin arguments — so it inherits whatever is configured here.
    //
    // SENSOR_I2C, not Wire: D4/D5 are GPIO6/7, which only the RP2350's
    // SECOND I2C block can reach. Wire is i2c0, refuses those pins, and
    // the barometer and magnetometer were never found.
    SENSOR_I2C.setSDA(PIN_I2C_SDA);
    SENSOR_I2C.setSCL(PIN_I2C_SCL);

    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);

    // Hybrid: the IMU on SPI at 10 MHz where the bandwidth matters, the
    // barometer and magnetometer on I2C where it does not.
    //
    // THE IMU'S SPI BUS IS ITS OWN. The module drives SCK and MOSI itself
    // whenever the IMU is not selected, so nothing else may sit on those
    // wires, and the MCU must not drive them between transfers either.
    // TriSense 1.6.0 and later releases SCK/MOSI between transfers on the
    // RP2350 by itself; an older library leaves the two drivers fighting
    // (tens of mA, and switching noise next to the barometer).
    bool ok = sensor.beginAll(MODE_HYBRID, PIN_TRISENSE_CS, TRISENSE_SPI_HZ, SENSOR_I2C);

    // 400 kHz, not the core's 100 kHz default, which beginAll() leaves in
    // place. The UWB tag's 74-byte frame shares this bus and takes 1.7 ms
    // at 400 kHz against 6.8 ms at 100 kHz — on the core that runs the
    // 240 Hz estimator. The BMP580, the AK09918C and the TF Luna are all
    // fast-mode parts.
    //
    // The timeout bounds every transaction on this bus: a tag that hangs
    // with SCL stretched, or a stuck slave, costs at most 10 ms and
    // resets the I2C block, instead of stalling core 0.
    SENSOR_I2C.setClock(400000);
    SENSOR_I2C.setTimeout(10, true);

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
//  11. SELF TEST
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
                // on the pad, so the datum is 0; a synthetic clock, as
                // everywhere else the filter is driven off the board
                e.fuse_baro(0.0f, (uint32_t)(i * 1000 / (int)ESTIMATOR_HZ) + 1u);
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
            if ((i % every) == 0)
                e.fuse_baro(200.0f, (uint32_t)(i * 1000 / (int)ESTIMATOR_HZ) + 1u);
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

    // --- UWB anchor table and multilateration ---------------------------
    //
    // Two different things are checked here and only one of them is
    // arithmetic.
    //
    // The GEOMETRY check is the important one, and it is the only place
    // in the system where a mis-entered anchor coordinate can be caught
    // before flight. Two anchors at the same point, or an anchor left at
    // the origin because a row was half-edited, produce a constellation
    // that still ranges, still solves, and is confidently wrong. It costs
    // nothing to notice on the pad and it cannot be noticed in the air.
    //
    // The SOLVER check runs the real multilateration on synthetic ranges
    // from the real anchor table, so it fails if the table is degenerate
    // even in ways the pairwise test does not name.
    {
        uint8_t n_en = 0;
        for (uint8_t i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) {
            if (!s_uwb_anchors[i].enabled) continue;
            n_en++;
            for (uint8_t j = i + 1; j < ARIADNA_UWB_MAX_ANCHORS; j++) {
                if (!s_uwb_anchors[j].enabled) continue;
                if (s_uwb_anchors[i].addr == s_uwb_anchors[j].addr) {
                    POS_PRINT("POST uwb duplicate address 0x%04X\n",
                              (unsigned)s_uwb_anchors[i].addr);
                    ok = false;
                }
                const float dx = s_uwb_anchors[i].x - s_uwb_anchors[j].x;
                const float dy = s_uwb_anchors[i].y - s_uwb_anchors[j].y;
                const float dz = s_uwb_anchors[i].z - s_uwb_anchors[j].z;
                if (dx * dx + dy * dy + dz * dz < 1.0f) {
                    POS_PRINT("POST uwb anchors %u and %u are within 1 m\n",
                              (unsigned)i, (unsigned)j);
                    ok = false;
                }
            }
        }
        if (n_en != UWB_ANCHOR_COUNT) {
            POS_PRINT("POST uwb %u enabled rows, UWB_ANCHOR_COUNT is %u\n",
                      (unsigned)n_en, (unsigned)UWB_ANCHOR_COUNT);
            ok = false;
        }

        // Synthetic ranges from a point 30 m above and 5 m east of the
        // pad. If the configured geometry cannot recover that, it cannot
        // recover the vehicle either.
        ariadna_uwb_cfg_t cfg;
        ariadna_uwb_cfg_default(&cfg);
        ariadna_uwb_obs_t obs[ARIADNA_UWB_MAX_ANCHORS];
        const float tx = 5.0f, ty = -3.0f, tz = 30.0f;
        memset(obs, 0, sizeof(obs));
        for (uint8_t i = 0; i < UWB_ANCHOR_COUNT; i++) {
            if (!s_uwb_anchors[i].enabled) continue;
            const float dx = tx - s_uwb_anchors[i].x;
            const float dy = ty - s_uwb_anchors[i].y;
            const float dz = tz - s_uwb_anchors[i].z;
            obs[i].range_m = sqrtf(dx * dx + dy * dy + dz * dz);
            obs[i].sigma_m = 0.2f;
            obs[i].valid   = 1;
        }
        const float prior[3] = { 0.0f, 0.0f, 20.0f };
        ariadna_uwb_fix_t f;
        if (!ariadna_uwb_solve(s_uwb_anchors, obs, UWB_ANCHOR_COUNT,
                               prior, &cfg, &f) ||
            fabsf(f.x - tx) > 0.5f || fabsf(f.y - ty) > 0.5f) {
            POS_PRINT("POST uwb solve (%.2f %.2f) want (%.2f %.2f) mode %u\n",
                      f.x, f.y, tx, ty, (unsigned)f.mode);
            ok = false;
        }

        // And the degraded case, because it is the one that will actually
        // happen: three anchors must still place the vehicle horizontally.
        obs[UWB_ANCHOR_COUNT - 1].valid = 0;
        if (!ariadna_uwb_solve(s_uwb_anchors, obs, UWB_ANCHOR_COUNT,
                               prior, &cfg, &f) ||
            fabsf(f.x - tx) > 1.0f || fabsf(f.y - ty) > 1.0f) {
            POS_PRINT("POST uwb 3-anchor solve (%.2f %.2f) mode %u\n",
                      f.x, f.y, (unsigned)f.mode);
            ok = false;
        }
    }

    return ok;
}

// =====================================================================
//  12. CORE 0 — sensors and the estimator
// =====================================================================
void setup()
{
    Serial.begin(ARIADNA_CONSOLE_BAUD);

    // The annunciator first: from here on every failure below has a
    // colour, and a board that dies in sensors_begin() still says so.
    ariadna_rgb_begin(&s_rgb, POS_RGB_BRIGHTNESS, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

    bool sensors = sensors_begin();
    s_est.begin();

    ariadna_uwb_cfg_default(&s_uwb_cfg);
    memset(&s_uwb_fix, 0, sizeof(s_uwb_fix));
    const bool uwb_ok = s_uwb.begin(SENSOR_I2C);
    s_lidar.begin();

    // Level the attitude estimate against gravity. The vehicle must be
    // still for this; it runs before the Master ever arms.
    if (sensors) {
        sensor.autoCalibrateGyro(1000);
        fusion.initOrientation(200);
        s_cal_state = 1;
    }

    s_post_pass = self_test() && sensors;
    // The tag is a separate board that may still be booting its radio, so
    // "not answering yet" is not a POST failure; the status packet's
    // ARIADNA_SENS_UWB bit says when it is actually ranging.
    POS_PRINT("\nAriadna V2 Position MCU — POST %s (sensors %s, UWB tag %s)\n",
              s_post_pass ? "PASS" : "FAIL", sensors ? "ok" : "FAIL",
              uwb_ok ? "up" : (ARIADNA_UWB_ENABLED ? "not answering yet" : "not built"));
    if (ARIADNA_UWB_ENABLED)
        POS_PRINT("# UWB: tag at I2C 0x%02X, %u anchors configured, read + solve at %lu Hz\n",
                  (unsigned)ARIADNA_UWB_I2C_ADDR, (unsigned)UWB_ANCHOR_COUNT,
                  (unsigned long)UWB_SOLVE_HZ);

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
    static uint32_t last_uwb_solve_ms = 0;
    static uint32_t last_uwb_gen  = 0;

    const uint32_t period = (uint32_t)(1000000UL / ESTIMATOR_HZ);

    uint32_t now = micros();
    if (next_us == 0) { next_us = now; last_step_us = now; }

    // ---- the idle path ---------------------------------------------------
    //
    // Only the annunciator lives here now. UWB ranging used to be polled
    // from this path; it runs on its own board since the tag moved off the
    // IMU's SPI bus, and is read once per solve instead. THE ORDER
    // MATTERS: the return below is what makes this the idle path, so
    // anything placed after it runs at 240 Hz instead.
    ariadna_rgb_service(&s_rgb);

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
    // The mission clock is taken ONCE per tick and passed to everything
    // that needs it, so the barometer, the GPS, UWB and the status
    // packet all agree on when "now" was.
    const uint32_t now_ms = millis();

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
    if (baro_fresh) s_est.fuse_baro(h_baro, now_ms);

    // ---- GPS -------------------------------------------------------------
    // Position and velocity both, with the fix aged forward to now. The
    // old estimator fused velocity only, at a fixed gain; there was no
    // horizontal position state for the position to correct.
    GpsFix gps;
    bool have_gps = gps_acquire(&gps);
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

    // ---- lidar ------------------------------------------------------------
    // Returns an invalid fix today. The call is here so that the path from
    // sensor to filter is exercised every tick and cannot rot.
    s_est.fuse_range(s_lidar.read());

    // ---- UWB --------------------------------------------------------------
    //
    // The tag board ranges at roughly 10 Hz per anchor; this board reads
    // its frame once per solve, at UWB_SOLVE_HZ, right before solving, and
    // the multilateration runs only when something has actually changed. Solving faster than the
    // ranges arrive would re-fuse the same measurement repeatedly, which
    // is the same mistake the barometer's baro_fresh flag exists to
    // prevent, with the same consequence: a filter that believes it has
    // seen five independent fixes when it has seen one.
    //
    // The whole path is written so that ANY subset of the anchors works.
    // Nothing here counts anchors or branches on how many answered — the
    // solver reports what the geometry supported and the estimator
    // weights it. A dead anchor costs accuracy and nothing else.
    if (ARIADNA_UWB_ENABLED &&
        (uint32_t)(now_ms - last_uwb_solve_ms) >= (1000u / UWB_SOLVE_HZ)) {
        last_uwb_solve_ms = now_ms;
        s_uwb.poll();                    // ~1.7 ms of I2C at 400 kHz
        s_uwb.age(now_ms);

        if (s_uwb.fresh() == 0) {
            // The constellation has gone quiet. Clear the published fix
            // rather than leave the last good one sitting in the status
            // packet, where it would read as a working constellation for
            // as long as nobody thought to power-cycle something.
            s_uwb_fix.mode   = ARIADNA_UWB_SOL_NONE;
            s_uwb_fix.use_x  = s_uwb_fix.use_y = s_uwb_fix.use_z = 0;
            s_uwb_fix.n_used = 0;
            s_uwb_fix.mask   = 0;
            s_uwb_fix.residual_rms = 0.0f;
        } else if (s_uwb.generation() != last_uwb_gen) {
            last_uwb_gen = s_uwb.generation();

            ariadna_uwb_obs_t obs[ARIADNA_UWB_MAX_ANCHORS];
            (void)s_uwb.snapshot(obs, now_ms);

            // The prior is the filter's own position. It is the
            // linearisation point, the held height in the degraded 2-D
            // modes, and the tie-breaker for the two-anchor ambiguity —
            // which is exactly why the vertical channel being
            // barometer-anchored matters to the HORIZONTAL accuracy of a
            // partial constellation.
            const float prior[3] = { s_est.px(), s_est.py(), s_est.pz() };
            const float vel[3]   = { s_est.vx(), s_est.vy(), s_est.vz() };

            // BRING THE RANGES TO ONE EPOCH BEFORE SOLVING THEM.
            //
            // The anchors answer one at a time, so this set was taken
            // from several different places. Standing still that is
            // free; descending it is not, and the failure is not a
            // graceful one. Measured on the host, on a 40 m square with
            // a 200 ms spread, an UNCOMPENSATED solve is out by 2.1 m at
            // 20 m/s and 11.6 m at 50 m/s — and the 20 m/s case sails
            // through the residual gate, because the error is a
            // consistent translation along the direction of travel
            // rather than a disagreement between the ranges.
            //
            // That is the worst kind of sensor error: confident, wrong,
            // and biased in the direction the vehicle is moving, at the
            // altitude where the guidance law has the least time to
            // recover. With the de-rotation the same cases come out at
            // 0.22 m and 0.13 m. See ../../test/uwb_test.cpp, which
            // walks the whole descent profile.
            ariadna_uwb_derotate(s_uwb_anchors, obs, UWB_ANCHOR_COUNT,
                                 prior, vel, &s_uwb_cfg);

            // The jump test in the driver has to know how fast the
            // vehicle is, for the same reason: at 50 m/s a range to one
            // anchor legitimately changes by metres between exchanges.
            s_uwb.set_speed_hint(sqrtf(vel[0] * vel[0] + vel[1] * vel[1] +
                                       vel[2] * vel[2]));

            if (ariadna_uwb_solve(s_uwb_anchors, obs, UWB_ANCHOR_COUNT,
                                  prior, &s_uwb_cfg, &s_uwb_fix)) {
                // The ranges have been brought to now_ms, so what is left
                // is only the gap between this solve and the estimator
                // step that consumes it — sub-millisecond, since they are
                // the same tick.
                s_est.fuse_uwb(s_uwb_fix, 0.0f, now_ms);
            }
        }
    }
    if (s_uwb.healthy()) s_sensor_ok |= ARIADNA_SENS_UWB;
    else                 s_sensor_ok &= (uint16_t)~ARIADNA_SENS_UWB;

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
    // Two separate facts, and the difference between them is diagnostic
    // gold on the pad: UWB_OK means the radio is ranging, UWB_AIDED means
    // the filter accepted what it produced.
    if (s_uwb.healthy())                 st.flags |= ARIADNA_NAV_UWB_OK;
    if (s_est.uwb_aided(now_ms))         st.flags |= ARIADNA_NAV_UWB_AIDED;

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
        ps.uwb_sigma_h  = s_est.uwb_sigma_h();
        ps.uwb_resid    = s_uwb_fix.residual_rms;
        ps.uwb_anchors  = s_uwb.fresh();
        ps.uwb_mask     = s_uwb_fix.mask;
        ps.uwb_mode     = s_uwb_fix.mode;
        ps.uwb_rejects  = s_est.uwb_rejects();
        ps.baro_trim    = s_est.baro_trim();

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

    // ---- annunciator ------------------------------------------------------
    // Last in the tick, after everything that matters. Choosing the colour
    // is arithmetic on values already computed; pushing it to the LED
    // happens in the idle path above and only when it changed.
    annunciate(st.flags, now_ms);
}

// =====================================================================
//  13. CORE 1 — GPS and receiving from the Master
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
