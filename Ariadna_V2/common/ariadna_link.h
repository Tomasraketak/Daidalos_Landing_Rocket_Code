// =====================================================================
//  Ariadna V2 — inter-processor link protocol
//
//  ONE definition of the wire format, shared by all four programs:
//
//      Master MCU      (Raspberry Pi Pico 2 W)
//      Position MCU    (XIAO RP2350)
//      Navigation MCU  (XIAO RP2350)
//      Ground Station  (Raspberry Pi Pico 2)
//
//  This header has NO Arduino dependency on purpose. It compiles for the
//  host, which is what lets ../../test/link_test.cpp verify the codec on a
//  workstation instead of on the pad — the same discipline the guidance
//  module already follows.
//
//  Include it with a relative path from each sketch:
//      #include "../common/ariadna_link.h"
//  Arduino IDE 2 resolves quoted includes relative to the including file,
//  so one copy serves every sketch. (Only headers work this way; a .cpp
//  outside the sketch folder would not be compiled.)
//
//  ---------------------------------------------------------------------
//  FRAME FORMAT
//  ---------------------------------------------------------------------
//      offset  size  field
//      0       1     SYNC0  0xA5
//      1       1     SYNC1  0x1D
//      2       1     TYPE   ariadna_pkt_t
//      3       1     LEN    payload length, 0..ARIADNA_PAYLOAD_MAX
//      4       LEN   payload
//      4+LEN   2     CRC-16/CCITT-FALSE over bytes 2..3+LEN, little-endian
//
//  The CRC covers TYPE and LEN as well as the payload, so a corrupted
//  length byte cannot make the receiver accept a mis-sized frame.
//
//  The CRC routine is bit-identical to the one already flying in
//  Daidalos_Landing_Guidance.ino. There is exactly one CRC in this system.
//
//  ---------------------------------------------------------------------
//  STRUCT LAYOUT RULES — read before adding a field
//  ---------------------------------------------------------------------
//  Every inter-MCU struct below is laid out 4-byte fields first, then
//  2-byte, then 1-byte, and explicitly padded to a multiple of 4. That
//  makes the packed layout identical to the natural layout, so:
//
//    * no #pragma pack is needed and no member is ever misaligned;
//    * both ends are little-endian ARM, so a memcpy of the struct IS the
//      wire format — no per-field serialisation to get out of sync;
//    * static_assert catches the day someone inserts a field in the middle
//      and forgets the padding.
//
//  The single exception is ariadna_tlm_t, the LoRa downlink packet, which
//  is packed to 46 bytes because radio airtime is the scarce resource
//  there. Its members are still individually aligned; only the tail
//  padding is removed.
// =====================================================================
#ifndef ARIADNA_LINK_H
#define ARIADNA_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
#define ARIADNA_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define ARIADNA_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

// ---------------------------------------------------------------------
//  Link speeds
//
//  The inter-MCU links run at 460800, not 115200. They are 20 mm PCB
//  traces, so the higher rate is free, and it keeps the 200 Hz actuator
//  stream and the 100 Hz navigation stream each under a quarter of the
//  link. At 115200 the same traffic sits near 80 % and any added field
//  starts costing control latency. Change these two lines to drop back.
//
//  The ground-station USB link stays at 115200 to match the existing
//  Python tooling.
// ---------------------------------------------------------------------
#define ARIADNA_MCU_BAUD    460800UL
#define ARIADNA_GS_USB_BAUD 115200UL

// ---------------------------------------------------------------------
//  Framing constants
// ---------------------------------------------------------------------
#define ARIADNA_SYNC0        0xA5
#define ARIADNA_SYNC1        0x1D
#define ARIADNA_HDR_LEN      4
#define ARIADNA_CRC_LEN      2
#define ARIADNA_PAYLOAD_MAX  96
#define ARIADNA_FRAME_MAX    (ARIADNA_HDR_LEN + ARIADNA_PAYLOAD_MAX + ARIADNA_CRC_LEN)

// ---------------------------------------------------------------------
//  Packet types
//
//  Numbered explicitly and never reused: a stale binary talking to a new
//  one must fail the type check, not silently misinterpret a payload.
// ---------------------------------------------------------------------
typedef enum {
    ARIADNA_PKT_NONE         = 0x00,
    ARIADNA_PKT_NAV_STATE    = 0x10,  // Position -> Master -> Navigation
    ARIADNA_PKT_POS_STATUS   = 0x11,  // Position -> Master
    ARIADNA_PKT_ACTUATOR     = 0x20,  // Navigation -> Master
    ARIADNA_PKT_NAV_STATUS   = 0x21,  // Navigation -> Master
    ARIADNA_PKT_MASTER_STATE = 0x30,  // Master -> Position, Navigation
    ARIADNA_PKT_CMD          = 0x31,  // Master -> Position, Navigation
    ARIADNA_PKT_TELEMETRY    = 0x40   // Master -> LoRa -> Ground Station
} ariadna_pkt_t;

// ---------------------------------------------------------------------
//  Coordinate and sign conventions
//
//  These are the conventions the guidance module was verified against.
//  Getting one backwards turns the attitude loop into positive feedback,
//  so they are stated once, here, and every program refers to this block.
//
//  Pad-local ENU frame, origin at the pad, fixed at arming:
//      +x  East
//      +y  North
//      +z  Up
//
//  h    metres above the pad, from the barometer referenced at arming.
//  vz   POSITIVE up. It is NEGATIVE for the whole descent. The guidance
//       module uses the same sign.
//
//  Attitude travels as the THRUST-AXIS UNIT VECTOR (ux, uy, uz) expressed
//  in pad-ENU. Not as Euler angles.
//
//  u is the direction the LANDING MOTOR accelerates the vehicle, i.e. the
//  vehicle's long axis pointing away from that motor's nozzle. On Daidalos
//  the landing motor lives under the nose cone and fires downward during
//  the nose-first descent, so:
//
//    * standing on the pad, nose up      -> u = (0, 0, -1)
//    * nose-first descent, perfect       -> u = (0, 0, +1)
//    * descending, leaning downrange     -> u tilts away from +z
//
//  Defining u this way makes the guidance tilt angle simply the angle
//  between u and vertical, which is exactly dg_nav_t.theta with no sign
//  gymnastics at the Navigation MCU. theta = 0 means "thrust straight up",
//  which is the attitude the landing burn wants.
//
//  Carrying a unit vector rather than angles is deliberate and matches the
//  guidance module's own philosophy ("the thrust direction is carried as a
//  NORMALISED VECTOR, never as an angle"). It removes a trig pair from the
//  sender, it has no wrap-around discontinuity, and it lets the Navigation
//  MCU pick its guidance plane with a single atan2f instead of unwrapping
//  Euler angles near vertical, where yaw is singular.
//
//  wx / wy / wz are angular rates in rad/s, taken STRAIGHT FROM THE GYRO
//  and then ROTATED into pad-ENU by the fused attitude. Rotating is not
//  differentiating: the numbers are still the gyro's own measurements,
//  just expressed in the frame the Navigation MCU plans in, which saves it
//  from having to carry a second attitude representation.
//
//  Never differentiate the fused attitude to obtain these. In the guidance
//  simulation that one substitution drops the landing rate from 72.6 % to
//  6.9 %, because differentiated estimator noise saturates the TVC servo.
//
//  roll / pitch / yaw are carried too, in degrees, but ONLY for telemetry
//  and the ground-station navball. No control law reads them.
// ---------------------------------------------------------------------

// --- ariadna_nav_state_t.flags ---------------------------------------
#define ARIADNA_NAV_VALID      (1u << 0)   // full solution usable for control
#define ARIADNA_NAV_BARO_OK    (1u << 1)
#define ARIADNA_NAV_IMU_OK     (1u << 2)
#define ARIADNA_NAV_MAG_OK     (1u << 3)
#define ARIADNA_NAV_GPS_FIX    (1u << 4)
#define ARIADNA_NAV_PAD_REF    (1u << 5)   // pad altitude reference captured
#define ARIADNA_NAV_UWB_OK     (1u << 6)   // FUTURE: DWM1000 ranging valid
#define ARIADNA_NAV_LIDAR_OK   (1u << 7)   // FUTURE: TF Luna range valid
#define ARIADNA_NAV_ACC_SAT    (1u << 8)   // accelerometer clipped this cycle
#define ARIADNA_NAV_DEADRECK   (1u << 9)   // extrapolated, not measured

/**
 * Navigation state estimate. The most important packet in the system:
 * every control decision on the Navigation MCU is taken from this.
 *
 * Path: Position MCU -> Master (routed, unmodified) -> Navigation MCU.
 * Default rate 100 Hz, see ARIADNA_NAV_STATE_HZ.
 */
typedef struct {
    uint32_t t_ms;        // Position MCU monotonic clock, ms
    float    h;           // m above pad
    float    vx, vy, vz;  // m/s, pad-ENU, vz positive up
    float    ux, uy, uz;  // thrust-axis unit vector, pad-ENU (see conventions)
    float    wx, wy, wz;  // rad/s, RAW GYRO rotated into pad-ENU
    float    roll, pitch, yaw;   // deg, telemetry only
    int32_t  lat, lon;    // 1e-7 deg  (V2 raises this from the V1 1e-6)
    float    gps_alt;     // m MSL
    float    accel_mag;   // |a| in g, including gravity
    float    temp_c;
    uint16_t flags;       // ARIADNA_NAV_* bits
    uint8_t  sats;
    uint8_t  fix;         // 0 none, 2 2D, 3 3D
    uint8_t  hdop_x10;
    uint8_t  quality;     // 0..255 estimator self-confidence, 255 = best
    uint8_t  _pad[2];
} ariadna_nav_state_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_nav_state_t) == 84, "nav_state layout changed");

// --- ariadna_actuator_t.flags ----------------------------------------
#define ARIADNA_ACT_VALID      (1u << 0)   // commands are live; else hold
#define ARIADNA_ACT_IGNITE     (1u << 1)   // Navigation has fired its pyro
#define ARIADNA_ACT_FINS_LIVE  (1u << 2)   // FUTURE: fin commands are meaningful
#define ARIADNA_ACT_TVC_PARK   (1u << 3)   // park TVC at neutral

/**
 * Actuator command, Navigation MCU -> Master, 200 Hz.
 *
 * The Master owns the PCA9685, so every servo angle in the vehicle is
 * commanded through this packet. Transport cost is ~0.9 ms at 460800 plus
 * the Master's servo tick; budget ~3-6 ms end to end from the guidance
 * solve to the servo pulse. That is real phase lag on a 9 rad/s loop and
 * is the reason the Master's servo path is written to do nothing else in
 * that critical section.
 */
typedef struct {
    uint32_t t_ms;
    float    tvc_x_deg;   // TVC servo command in the pad-ENU x plane
    float    tvc_y_deg;   // ... and the y plane
    float    clamp;       // thrust-clamp fraction, DG_CLAMP_MIN..DG_CLAMP_MAX
    float    fin_deg[4];  // FUTURE: controllable aero fins. Zero until flown.
    uint8_t  flags;       // ARIADNA_ACT_* bits
    uint8_t  phase;       // dg_phase_t, mirrored for the Master's LED/buzzer
    uint8_t  _pad[2];
} ariadna_actuator_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_actuator_t) == 36, "actuator layout changed");

// --- ariadna_nav_status_t.flags --------------------------------------
#define ARIADNA_NST_ARMED      (1u << 0)
#define ARIADNA_NST_POST_PASS  (1u << 1)   // power-on self test passed
#define ARIADNA_NST_LINK_OK    (1u << 2)   // nav state arriving on time
#define ARIADNA_NST_PLANNER_OK (1u << 3)   // core 1 still iterating
#define ARIADNA_NST_BACKUP_ARM (1u << 4)   // backup igniter channel armed

/** Navigation MCU housekeeping, 20 Hz. Never in the control path. */
typedef struct {
    uint32_t t_ms;
    float    p_plan;       // planner parameter, 0..4
    float    thrust_est;   // N
    float    h_ignite;     // m, commanded ignition altitude
    float    theta_cmd;    // rad, in the guidance plane
    float    theta_err;    // rad
    float    plane_az;     // rad, azimuth of the guidance plane in pad-ENU
    uint32_t rx_ok;
    uint32_t rx_bad;
    uint32_t plan_iters;   // must keep advancing or core 1 has died
    uint16_t loop_max_us;  // worst 200 Hz loop time seen
    uint16_t overruns;     // missed 200 Hz slots
    uint8_t  phase;        // dg_phase_t
    uint8_t  pyro_cont;    // continuity bitmask, bit n = channel n+1 has a match
    uint8_t  pyro_fired;   // fired bitmask, same bit order
    uint8_t  flags;        // ARIADNA_NST_* bits
} ariadna_nav_status_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_nav_status_t) == 48, "nav_status layout changed");

// --- ariadna_pos_status_t.sensor_ok ----------------------------------
#define ARIADNA_SENS_IMU       (1u << 0)
#define ARIADNA_SENS_BARO      (1u << 1)
#define ARIADNA_SENS_MAG       (1u << 2)
#define ARIADNA_SENS_GPS       (1u << 3)
#define ARIADNA_SENS_UWB       (1u << 4)   // FUTURE
#define ARIADNA_SENS_LIDAR     (1u << 5)   // FUTURE

/** Position MCU housekeeping, 20 Hz. */
typedef struct {
    uint32_t t_ms;
    uint32_t rx_ok;
    uint32_t rx_bad;
    float    fusion_hz;      // measured, from TriSense getActualFusionHz()
    float    pad_pressure;   // Pa, reference captured at arming
    float    pad_alt_msl;    // m, GPS altitude at the pad
    uint16_t sensor_ok;      // ARIADNA_SENS_* bits
    uint16_t imu_overflow;   // ICM42688P FIFO overflow count
    uint8_t  sats;
    uint8_t  fix;
    uint8_t  hdop_x10;
    uint8_t  cal_state;      // 0 uncal, 1 gyro done, 2 gyro+accel, 3 full
} ariadna_pos_status_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_pos_status_t) == 32, "pos_status layout changed");

// --- ariadna_master_state_t.flags ------------------------------------
#define ARIADNA_MST_ARMED      (1u << 0)   // RBF removed and software armed
#define ARIADNA_MST_RBF_OUT    (1u << 1)   // Remove-Before-Flight pin pulled
#define ARIADNA_MST_BREAKWIRE  (1u << 2)   // breakwire broken -> launch detected
#define ARIADNA_MST_TEST       (1u << 3)   // test mode, pyros inhibited
#define ARIADNA_MST_LOGGING    (1u << 4)   // flash log is running
#define ARIADNA_MST_ABORT      (1u << 5)

/** Master flight state machine. Broadcast to both XIAOs at 10 Hz. */
typedef struct {
    uint32_t t_ms;         // master mission clock
    uint32_t t_launch_ms;  // t_ms at launch detect, 0 before
    uint16_t vbat_mv;
    uint16_t current_ma;
    uint8_t  state;        // ariadna_state_t
    uint8_t  flags;        // ARIADNA_MST_* bits
    uint8_t  _pad[2];
} ariadna_master_state_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_master_state_t) == 16, "master_state layout changed");

/**
 * Master flight state machine.
 *
 * The Navigation MCU has its own, narrower phase (dg_phase_t) which
 * governs only the landing burn. This one governs the mission.
 */
typedef enum {
    ARIADNA_STATE_BOOT     = 0,   // powering up, self tests running
    ARIADNA_STATE_IDLE     = 1,   // safe, RBF in place, pyros inhibited
    ARIADNA_STATE_ARMED    = 2,   // on the pad, ready, waiting for breakwire
    ARIADNA_STATE_ASCENT   = 3,   // powered / coasting up
    ARIADNA_STATE_APOGEE   = 4,   // vertical speed through zero
    ARIADNA_STATE_DESCENT  = 5,   // falling, above guidance entry altitude
    ARIADNA_STATE_GUIDANCE = 6,   // below 85 m, Navigation MCU has authority
    ARIADNA_STATE_LANDED   = 7,
    ARIADNA_STATE_RECOVERY = 8,   // beacon, slow telemetry, log flushed
    ARIADNA_STATE_ABORT    = 9,
    ARIADNA_STATE_TEST     = 10,
    ARIADNA_STATE_ERROR    = 11
} ariadna_state_t;

/** Master -> XIAO commands. Event driven, sent three times for reliability. */
typedef struct {
    uint32_t arg32;
    uint8_t  cmd;          // ariadna_cmd_id_t
    uint8_t  arg;
    uint8_t  _pad[2];
} ariadna_cmd_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_cmd_t) == 8, "cmd layout changed");

typedef enum {
    ARIADNA_CMD_NOP        = 0,
    ARIADNA_CMD_SET_PAD_REF= 1,   // capture pad pressure / GPS origin
    ARIADNA_CMD_CAL_GYRO   = 2,   // vehicle must be still
    ARIADNA_CMD_ARM        = 3,
    ARIADNA_CMD_DISARM     = 4,
    ARIADNA_CMD_ABORT      = 5,   // park actuators, inhibit pyros, latch
    ARIADNA_CMD_TEST_MODE  = 6,   // arg: 1 enter, 0 leave
    ARIADNA_CMD_ZERO_CLOCK = 7    // align mission clocks at launch detect
} ariadna_cmd_id_t;

// ---------------------------------------------------------------------
//  LoRa downlink packet
//
//  Packed to 46 bytes because airtime, not RAM, is the constraint. At
//  SF7 / BW125 / CR4-5 a 46-byte payload is ~72 ms on air, so the 5 Hz
//  default downlink is about a 36 % duty cycle. Raising the rate or the
//  size eats that margin directly — check ARIADNA_TLM_HZ in master_mcu.ino
//  before adding a field.
//
//  Field order is 4-byte, then 2-byte, then 1-byte, so every member is
//  naturally aligned even though the struct is packed; only the two bytes
//  of tail padding are removed.
//
//  ground_visualizer.py must be updated in lockstep. Its struct format
//  string is asserted against this layout by ../../test/link_test.cpp.
// ---------------------------------------------------------------------
#define ARIADNA_TLM_VERSION  2

#pragma pack(push, 1)
typedef struct {
    uint32_t t_ms;         //  0  master mission clock
    int32_t  lat;          //  4  1e-7 deg
    int32_t  lon;          //  8  1e-7 deg
    int16_t  alt_dm;       // 12  m above pad * 10
    int16_t  vz_dms;       // 14  m/s * 10, NEGATIVE descending
    int16_t  vh_dms;       // 16  horizontal speed, m/s * 10
    int16_t  acc_cg;       // 18  |a| in g * 100
    int16_t  roll_ddeg;    // 20  deg * 10
    int16_t  pitch_ddeg;   // 22
    int16_t  yaw_ddeg;     // 24
    int16_t  temp_dc;      // 26  degC * 10
    int16_t  tvc_x_cdeg;   // 28  TVC servo deg * 100
    int16_t  tvc_y_cdeg;   // 30
    int16_t  thrust_n;     // 32  estimated thrust, N
    uint8_t  version;      // 34  == ARIADNA_TLM_VERSION
    uint8_t  state;        // 35  ariadna_state_t
    uint8_t  nav_phase;    // 36  dg_phase_t
    uint8_t  clamp_pct;    // 37  thrust clamp, percent
    uint8_t  vbat_cv;      // 38  (V - 2.5) * 100, same encoding as V1
    uint8_t  curr_da;      // 39  amps * 10
    uint8_t  hdop_x10;     // 40
    uint8_t  sats;         // 41
    uint8_t  pyro;         // 42  low nibble continuity, high nibble fired
    uint8_t  health;       // 43  ARIADNA_HLT_* bits
    uint8_t  flags;        // 44  ARIADNA_TF_* bits
    uint8_t  exp;          // 45  experiment / payload phase, free for you
} ariadna_tlm_t;
#pragma pack(pop)
ARIADNA_STATIC_ASSERT(sizeof(ariadna_tlm_t) == 46, "telemetry layout changed - update ground_visualizer.py");

// --- ariadna_tlm_t.health --------------------------------------------
#define ARIADNA_HLT_POS_LINK   (1u << 0)   // Position MCU talking
#define ARIADNA_HLT_NAV_LINK   (1u << 1)   // Navigation MCU talking
#define ARIADNA_HLT_NAV_VALID  (1u << 2)   // its solution is usable
#define ARIADNA_HLT_FLASH_OK   (1u << 3)
#define ARIADNA_HLT_LOGGING    (1u << 4)
#define ARIADNA_HLT_SERVO_OK   (1u << 5)   // PCA9685 responding
#define ARIADNA_HLT_GPS_FIX    (1u << 6)
#define ARIADNA_HLT_POST_PASS  (1u << 7)   // all boards passed self test

// --- ariadna_tlm_t.flags ---------------------------------------------
#define ARIADNA_TF_ARMED       (1u << 0)
#define ARIADNA_TF_RBF_OUT     (1u << 1)
#define ARIADNA_TF_BREAKWIRE   (1u << 2)
#define ARIADNA_TF_TEST        (1u << 3)
#define ARIADNA_TF_ABORT       (1u << 4)
#define ARIADNA_TF_ERR_MASK    (7u << 5)   // 3-bit error code in the top bits
#define ARIADNA_TF_ERR_SHIFT   5

/** Ground station -> PC. The downlink packet plus link quality. */
#pragma pack(push, 1)
typedef struct {
    ariadna_tlm_t tlm;
    int8_t        rssi;    // dBm
    int8_t        snr;     // dB
} ariadna_gs_serial_t;
#pragma pack(pop)
ARIADNA_STATIC_ASSERT(sizeof(ariadna_gs_serial_t) == 48, "gs serial layout changed");

// ---------------------------------------------------------------------
//  Default rates
// ---------------------------------------------------------------------
#define ARIADNA_NAV_STATE_HZ   100   // Position -> Navigation state stream
#define ARIADNA_ACTUATOR_HZ    200   // Navigation -> Master servo commands
#define ARIADNA_STATUS_HZ       20   // housekeeping both ways
#define ARIADNA_MASTER_HZ       10   // Master state broadcast

// LoRa downlink rate. A 46-byte payload at SF7 / BW125 / CR4-5 is ~92 ms
// on air, so 4 Hz is already a 37 % duty cycle and 5 Hz would be 46 %.
// Raise this only after checking the airtime figure printed by
// ../../test/link_test.cpp, and note that the 433 MHz ISM band has duty
// cycle limits in the EU that this is already close to. If you need a
// faster downlink, widen the bandwidth to 250 kHz — that halves airtime
// for about 3 dB of link budget — rather than shortening the packet.
#define ARIADNA_TLM_HZ           4   // Master -> ground station

// =====================================================================
//  CRC-16/CCITT-FALSE
//
//  poly 0x1021, init 0xFFFF, no reflection, no final xor. Bit-identical
//  to the routine in Daidalos_Landing_Guidance.ino so the whole vehicle
//  has one checksum. Bitwise rather than table-driven: a 96-byte frame is
//  ~770 shift-xor pairs, well under 10 us on a 150 MHz M33, and a 512-byte
//  table is not worth the flash for that.
// =====================================================================
static inline uint16_t ariadna_crc16(const uint8_t *p, uint16_t n)
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

// =====================================================================
//  Encoder
//
//  Builds a complete frame into `out`, which must hold ARIADNA_FRAME_MAX
//  bytes. Returns the total frame length, or 0 if the payload is too big.
//
//  Callers write the whole frame in ONE port write. Splitting it lets a
//  receiver see a torn frame during a reset and burn a resync cycle.
// =====================================================================
static inline uint16_t ariadna_encode(uint8_t *out, uint8_t type,
                                      const void *payload, uint8_t len)
{
    if (len > ARIADNA_PAYLOAD_MAX) return 0;

    out[0] = ARIADNA_SYNC0;
    out[1] = ARIADNA_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len && payload) memcpy(&out[4], payload, len);

    uint16_t crc = ariadna_crc16(&out[2], (uint16_t)(len + 2));
    out[4 + len] = (uint8_t)(crc & 0xFF);
    out[5 + len] = (uint8_t)(crc >> 8);

    return (uint16_t)(ARIADNA_HDR_LEN + len + ARIADNA_CRC_LEN);
}

// =====================================================================
//  Decoder
//
//  Byte-at-a-time state machine. It never blocks and never waits for a
//  whole frame, so a truncated packet costs one control tick and nothing
//  else — the property that matters in a 200 Hz loop.
//
//  Resynchronisation is the part worth reading carefully, because it is
//  what decides how much good data a single corrupted byte destroys.
//
//  Two hazards, both handled:
//
//   1. A rejected second byte may itself be the start of the real frame.
//      Without that case, the sequence 0xA5 0xA5 0x1D never syncs.
//
//   2. This is a LENGTH-PREFIXED protocol, so a corrupted length byte
//      makes the parser wait for a frame that will never arrive while
//      good frames stream past inside the bogus one. The naive fix — drop
//      everything and start again — throws away up to 102 bytes, which at
//      200 Hz is two whole actuator commands.
//
//      Instead, a frame rejected on CRC or on an impossible length is
//      RESCANNED: the parser looks for the next sync pair inside the bytes
//      it already holds and carries on from there. The real frame hiding
//      inside the bogus one is usually recovered in the same call.
//
//      The guarantee this buys, measured in ../../test/link_test.cpp: after
//      arbitrary garbage the parser resynchronises within one frame, and
//      over a randomly corrupted stream it recovers essentially every
//      frame that was not itself hit.
//
//  Every rejection path strictly reduces the buffered byte count, so the
//  retry loop is bounded — there are no unbounded loops in this parser.
// =====================================================================
typedef struct {
    uint8_t  buf[ARIADNA_FRAME_MAX];
    uint16_t n;            // bytes currently in buf
    uint16_t consume;      // bytes of a delivered frame, removed on next call
    uint32_t rx_ok;        // frames accepted
    uint32_t rx_crc;       // frames dropped on CRC
    uint32_t rx_len;       // frames dropped on an impossible length
} ariadna_rx_t;

static inline void ariadna_rx_init(ariadna_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

/**
 * Discard buf[0 .. from-1] and re-hunt for a frame start in what is left.
 *
 * Called only after a frame has been rejected, so `from` is always >= 1 and
 * the buffered byte count strictly decreases.
 */
static inline void ariadna_rx_resync(ariadna_rx_t *rx, uint16_t from)
{
    for (uint16_t k = from; k + 1 < rx->n; k++) {
        if (rx->buf[k] == ARIADNA_SYNC0 && rx->buf[k + 1] == ARIADNA_SYNC1) {
            memmove(rx->buf, &rx->buf[k], (size_t)(rx->n - k));
            rx->n = (uint16_t)(rx->n - k);
            return;
        }
    }

    // No complete sync pair left. A trailing lone SYNC0 may still be the
    // first byte of the next frame, so keep it rather than lose a frame to
    // an unlucky split.
    if (rx->n > from && rx->buf[rx->n - 1] == ARIADNA_SYNC0) {
        rx->buf[0] = ARIADNA_SYNC0;
        rx->n = 1;
    } else {
        rx->n = 0;
    }
}

/**
 * Feed one received byte.
 *
 * Returns the packet type (> 0) when a frame passes its CRC, and sets
 * *payload / *len to point into rx->buf. The pointer stays valid until the
 * next call, so copy anything you intend to keep.
 *
 * Returns 0 when the frame is still incomplete or was discarded.
 */
static inline uint8_t ariadna_rx_byte(ariadna_rx_t *rx, uint8_t b,
                                      const uint8_t **payload, uint8_t *len)
{
    // --- retire the frame handed out last call -----------------------
    // Delivery returns a pointer into buf, so the frame cannot be removed
    // until the caller has had it. Anything after it is the start of the
    // next frame and must survive.
    if (rx->consume) {
        if (rx->n > rx->consume) {
            memmove(rx->buf, &rx->buf[rx->consume],
                    (size_t)(rx->n - rx->consume));
            rx->n = (uint16_t)(rx->n - rx->consume);
        } else {
            rx->n = 0;
        }
        rx->consume = 0;
    }

    // --- byte 0: hunt for SYNC0 --------------------------------------
    if (rx->n == 0) {
        if (b != ARIADNA_SYNC0) return 0;
        rx->buf[0] = b;
        rx->n = 1;
        return 0;
    }

    // --- byte 1: must be SYNC1, else re-hunt without losing this byte -
    if (rx->n == 1) {
        if (b == ARIADNA_SYNC1) {
            rx->buf[1] = b;
            rx->n = 2;
        } else if (b == ARIADNA_SYNC0) {
            rx->buf[0] = ARIADNA_SYNC0;      // this may be the real start
            rx->n = 1;
        } else {
            rx->n = 0;
        }
        return 0;
    }

    if (rx->n >= ARIADNA_FRAME_MAX) rx->n = 0;   // unreachable; never overrun
    rx->buf[rx->n++] = b;

    // --- try to complete a frame, retrying after each rejection -------
    for (;;) {
        if (rx->n < ARIADNA_HDR_LEN) return 0;

        uint8_t plen = rx->buf[3];
        if (plen > ARIADNA_PAYLOAD_MAX) {
            rx->rx_len++;
            ariadna_rx_resync(rx, 1);
            continue;
        }

        uint16_t want = (uint16_t)(ARIADNA_HDR_LEN + plen + ARIADNA_CRC_LEN);
        if (rx->n < want) return 0;

        uint16_t got = (uint16_t)rx->buf[4 + plen] |
                       ((uint16_t)rx->buf[5 + plen] << 8);

        if (ariadna_crc16(&rx->buf[2], (uint16_t)(plen + 2)) == got) {
            rx->rx_ok++;
            rx->consume = want;
            if (payload) *payload = &rx->buf[4];
            if (len)     *len     = plen;
            return rx->buf[2];
        }

        rx->rx_crc++;
        ariadna_rx_resync(rx, 1);
    }
}

/**
 * Copy a decoded payload into a typed struct, refusing a length mismatch.
 *
 * A frame can pass its CRC and still be the wrong shape — an old binary on
 * the other end of the link, most likely. Checking the length here means
 * that shows up as a dropped packet rather than as a struct read off the
 * end of the buffer.
 */
static inline int ariadna_unpack(const uint8_t *payload, uint8_t len,
                                 void *dst, size_t expect)
{
    if (len != expect) return 0;
    memcpy(dst, payload, expect);
    return 1;
}

#endif // ARIADNA_LINK_H
