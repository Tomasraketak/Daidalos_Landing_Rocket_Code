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

// Ground station -> laptop. This one is a real rate on a real link and
// ground_visualizer.py must open the port at the same number.
#define ARIADNA_GS_USB_BAUD 115200UL

// The three flight boards' debug consoles, all on native USB CDC.
//
// 115200, to match every other serial tool in this project — but note that
// on the RP2350 the number is cosmetic. `Serial` is USB CDC, not a UART, so
// the argument to Serial.begin() is ignored and the link runs at USB full
// speed regardless of what either end asks for. Changing it will not make a
// console faster and will not break one that is set differently.
//
// It is kept separate from ARIADNA_GS_USB_BAUD because they mean different
// things: that one is a constraint, this one is a convention.
#define ARIADNA_CONSOLE_BAUD 115200UL

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
 * Emitted on the estimator's own schedule, not on a timer — see
 * ARIADNA_NAV_STATE_HZ.
 *
 * ---------------------------------------------------------------------
 * THIS PACKET CARRIES ONLY WHAT A CONTROL LOOP CONSUMES
 * ---------------------------------------------------------------------
 * Attitude in degrees, latitude, longitude, GPS altitude and temperature
 * used to live here. They were moved to ariadna_pos_status_t, which goes
 * to the Master at 20 Hz and which the Navigation MCU ignores.
 *
 * The reason is arithmetic. This packet is the highest-rate thing on the
 * link, and it crosses the link TWICE — Position to Master, then Master to
 * Navigation. At 240 Hz the old 84-byte version cost 47 % of a 460800 baud
 * link in each direction; the 52-byte version costs 30 %. Nothing was lost:
 * every field that moved is telemetry, GPS updates at 10 Hz at best, and a
 * navball redrawn at 20 Hz looks identical to one redrawn at 240 Hz.
 *
 * Before adding a field here, ask whether a control law reads it. If not,
 * it belongs in the status packet.
 */
typedef struct {
    uint32_t t_ms;        // Position MCU monotonic clock, ms
    float    h;           // m above pad
    float    vx, vy, vz;  // m/s, pad-ENU, vz positive up
    float    ux, uy, uz;  // thrust-axis unit vector, pad-ENU (see conventions)
    float    wx, wy, wz;  // rad/s, RAW GYRO rotated into pad-ENU
    float    accel_mag;   // |a| in g, including gravity. The Navigation MCU
                          // uses this to decide whether the motor lit.
    uint16_t flags;       // ARIADNA_NAV_* bits
    uint8_t  quality;     // 0..255 estimator self-confidence, 255 = best
    uint8_t  _pad;
} ariadna_nav_state_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_nav_state_t) == 52, "nav_state layout changed");

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

/**
 * Position MCU housekeeping and telemetry, 20 Hz.
 *
 * Everything the ground wants to see but no control law reads. Keeping it
 * off the 240 Hz path is what makes that path affordable — see the note on
 * ariadna_nav_state_t.
 *
 * 20 Hz is five times the fastest downlink rate and comfortably faster
 * than GPS produces new fixes, so nothing here is ever the stale link.
 */
typedef struct {
    uint32_t t_ms;
    uint32_t rx_ok;
    uint32_t rx_bad;
    float    fusion_hz;      // measured, from TriSense getActualFusionHz()
    float    pad_pressure;   // Pa, reference captured at arming
    float    pad_alt_msl;    // m, GPS altitude at the pad
    float    roll, pitch, yaw;   // deg, telemetry and the ground navball
    int32_t  lat, lon;       // 1e-7 deg  (V2 raises this from the V1 1e-6)
    float    gps_alt;        // m MSL
    float    temp_c;
    uint16_t sensor_ok;      // ARIADNA_SENS_* bits
    uint16_t imu_overflow;   // ICM42688P FIFO overflow count
    uint8_t  sats;
    uint8_t  fix;            // 0 none, 2 2D, 3 3D
    uint8_t  hdop_x10;
    uint8_t  cal_state;      // 0 uncal, 1 gyro done, 2 gyro+accel, 3 full
} ariadna_pos_status_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_pos_status_t) == 60, "pos_status layout changed");

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
// The navigation state is NOT sent on a timer. It is emitted by the
// Position MCU the instant its estimator finishes a step, so the number
// below is the estimator's rate rather than a transmit schedule — see
// "Event-driven, not timed" in position_mcu.ino.
//
// 240 Hz, feeding a 200 Hz control loop. Running the producer FASTER than
// the consumer is the point: at equal rates two free-running crystals beat
// against each other and a tick occasionally finds no new frame. At 1.2
// frames per tick every tick has one.
#define ARIADNA_NAV_STATE_HZ   240   // Position -> Navigation state stream
#define ARIADNA_ACTUATOR_HZ    200   // Navigation -> Master servo commands
#define ARIADNA_STATUS_HZ       20   // housekeeping both ways
#define ARIADNA_MASTER_HZ       10   // Master state broadcast

// ---------------------------------------------------------------------
//  LoRa radio
//
//  Defined here, not in the two sketches, because the vehicle and the
//  ground station must agree on all five. A mismatch in any one of them is
//  a radio that hears nothing, with no error message to show for it.
//
//  BANDWIDTH IS 250 kHz, NOT THE USUAL 125 kHz. That halves airtime, which
//  is what makes a 10 Hz downlink possible at all:
//
//      46-byte payload, SF7, CR4/5      BW 125 kHz    BW 250 kHz
//          time on air                     92.4 ms       46.2 ms
//          duty cycle at 10 Hz             92.4 %        46.2 %
//
//  At 125 kHz a 10 Hz downlink would need the transmitter on 92 % of the
//  time, which is not a margin, it is a coincidence.
//
//  The cost is 3 dB of receiver sensitivity, -123 dBm to -120 dBm. That is
//  nothing here: at 2 km and +20 dBm the received signal is about -71 dBm,
//  roughly 49 dB of margin. Airtime is the scarce resource in this link,
//  not sensitivity. If you ever do need the range back, SF8 at 250 kHz
//  buys most of it and still beats SF7/125k on airtime.
//
//  ⚠️ DUTY CYCLE REGULATION. 433.05-434.79 MHz is an ISM band, and in the
//  EU it is generally limited to a 10 % duty cycle at 10 mW ERP under
//  ETSI EN 300 220. 10 Hz at +20 dBm is well outside that. Check your
//  competition rules and your national regulator before flying it; drop
//  ARIADNA_TLM_HZ to 5 (23 %) or 2 (9 %), or reduce TX power, if you need
//  to comply. This is your call to make, not the firmware's, so nothing
//  here enforces it — but do not discover it at the pad.
// ---------------------------------------------------------------------
#define ARIADNA_LORA_FREQ_HZ   433000000UL
#define ARIADNA_LORA_SF                 7
#define ARIADNA_LORA_BW_HZ         250000UL
#define ARIADNA_LORA_CR4                5    // 4/5
#define ARIADNA_LORA_TX_DBM            20
#define ARIADNA_LORA_PREAMBLE           8

// Master -> ground station. 10 Hz is 46 % duty at the settings above;
// ../../test/link_test.cpp prints the figure and fails if it goes past a
// half. See the regulatory note before raising it.
#define ARIADNA_TLM_HZ          10

/**
 * LoRa time on air, in microseconds, for a payload of `pl` bytes.
 *
 * The SX1276/78 datasheet formula, with an explicit header and CRC on. Put
 * here rather than in a spreadsheet so the firmware, the ground station
 * and the test all get the same number from the same source — and so that
 * changing a radio setting immediately shows up as a changed duty cycle in
 * the test output instead of as a surprise on the pad.
 *
 * Integer arithmetic throughout: this is also called on the RP2350, where
 * a stray double would be software-emulated.
 */
static inline uint32_t ariadna_lora_airtime_us(uint16_t pl)
{
    const uint32_t sf = ARIADNA_LORA_SF;
    const uint32_t cr = ARIADNA_LORA_CR4 - 4;          // 1..4

    // Symbol time in nanoseconds, to keep the integer maths honest.
    const uint32_t t_sym_ns = (uint32_t)(((uint64_t)(1u << sf) * 1000000000ULL)
                                         / ARIADNA_LORA_BW_HZ);

    // Payload symbol count. DE (low data rate optimise) is off: it only
    // applies when the symbol time exceeds 16 ms, and SF7 is nowhere near.
    int32_t num = 8 * (int32_t)pl - 4 * (int32_t)sf + 28 + 16;   // IH = 0, CRC = 1
    int32_t den = 4 * (int32_t)sf;
    int32_t blocks = (num + den - 1) / den;                      // ceil
    if (blocks < 0) blocks = 0;
    uint32_t n_payload = 8 + (uint32_t)blocks * (cr + 4);

    // Preamble is n + 4.25 symbols; x4 keeps the quarter symbol exact.
    uint32_t sym_x4 = (ARIADNA_LORA_PREAMBLE * 4 + 17) + n_payload * 4;

    return (uint32_t)(((uint64_t)sym_x4 * t_sym_ns) / 4000ULL);
}

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

// =====================================================================
//  TRANSMIT RING
//
//  ---------------------------------------------------------------------
//  WHY THIS EXISTS
//  ---------------------------------------------------------------------
//  The RP2350's UART has a 32-byte hardware transmit FIFO. At 460800 baud
//  a byte takes 21.7 us to shift out, so a frame longer than 32 bytes
//  cannot be handed over in one go — something has to wait for the rest:
//
//      Master forwards a 90-byte nav_state @100 Hz
//          58 bytes past the FIFO x 21.7 us  =  1.26 ms per frame
//          at 100 Hz                         =  12.6 % of a core
//
//      Navigation sends a 42-byte actuator @200 Hz
//          10 bytes past the FIFO            =   217 us per frame
//          at 200 Hz                         =   4.3 % of a core
//
//  A blocking write spends that time spinning. Worse than the wasted
//  cycles, on the Master those 1.26 ms land INSIDE the forwarding path —
//  the one place in the vehicle where latency was deliberately minimised.
//  Forwarding the frame before parsing it saves microseconds; blocking on
//  the FIFO immediately afterwards gives back a millisecond.
//
//  So frames go into this ring instead, and DMA drains it into the UART
//  while the processor does something useful. The producer never waits.
//
//  For comparison, the arithmetic this module does — CRC plus the byte
//  parser — costs about 1 % of a core at full traffic. That is why there
//  is no DMA anywhere else in this project: everything else is already
//  cheap, and DMA that saves nothing only adds ways to be wrong.
//
//  ---------------------------------------------------------------------
//  The ring is kept HERE, with no hardware dependency, so it can be tested
//  on a workstation. The DMA binding lives in ariadna_uart_dma.h. An
//  off-by-one in this bookkeeping loses a servo command, which is not
//  something to discover on the pad.
//
//  Single producer, single consumer, both on the SAME core. The consumer
//  is a DMA channel that only ever reads, so no lock is needed — but the
//  ring must not be serviced from two cores.
// =====================================================================
#define ARIADNA_TX_RING_SZ   512               // must be a power of two
#define ARIADNA_TX_RING_MASK (ARIADNA_TX_RING_SZ - 1)

typedef struct {
    uint8_t  buf[ARIADNA_TX_RING_SZ];
    uint16_t head;       // producer writes here
    uint16_t tail;       // oldest byte not yet handed to the hardware
    uint16_t inflight;   // bytes currently being sent; tail is frozen mean-
                         // while, which is what protects them from the producer
    uint32_t sent;
    uint32_t dropped;    // frames refused because the ring was full
} ariadna_tx_ring_t;

static inline void ariadna_tx_init(ariadna_tx_ring_t *r)
{
    memset(r, 0, sizeof(*r));
}

static inline uint16_t ariadna_tx_used(const ariadna_tx_ring_t *r)
{
    return (uint16_t)((r->head - r->tail) & ARIADNA_TX_RING_MASK);
}

/**
 * Queue a complete frame. Returns 0 if it did not fit.
 *
 * All or nothing: a partially queued frame would be transmitted as a
 * truncated one, which costs the receiver a resync. Dropping it whole
 * costs one packet. Note that `used` is measured from tail, which has not
 * advanced past the in-flight bytes — so the region DMA is reading is
 * counted as occupied and can never be overwritten.
 */
static inline int ariadna_tx_push(ariadna_tx_ring_t *r,
                                  const uint8_t *frame, uint16_t n)
{
    if ((uint32_t)ariadna_tx_used(r) + n > ARIADNA_TX_RING_SZ - 1) {
        r->dropped++;
        return 0;
    }
    for (uint16_t i = 0; i < n; i++) {
        r->buf[r->head] = frame[i];
        r->head = (uint16_t)((r->head + 1) & ARIADNA_TX_RING_MASK);
    }
    return 1;
}

/**
 * Claim the next contiguous run for the hardware.
 *
 * Returns 0 when there is nothing to send or a transfer is still in
 * flight. The run stops at the end of the buffer, so a wrapped ring simply
 * takes two transfers — cheaper than the alternative of forcing frames to
 * be contiguous.
 */
static inline uint16_t ariadna_tx_next(ariadna_tx_ring_t *r, const uint8_t **out)
{
    if (r->inflight) return 0;

    uint16_t used = ariadna_tx_used(r);
    if (!used) return 0;

    uint16_t run = (uint16_t)(ARIADNA_TX_RING_SZ - r->tail);
    if (run > used) run = used;

    *out = &r->buf[r->tail];
    r->inflight = run;
    return run;
}

/** The hardware has finished the run claimed by ariadna_tx_next(). */
static inline void ariadna_tx_complete(ariadna_tx_ring_t *r)
{
    r->tail = (uint16_t)((r->tail + r->inflight) & ARIADNA_TX_RING_MASK);
    r->sent += r->inflight;
    r->inflight = 0;
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
