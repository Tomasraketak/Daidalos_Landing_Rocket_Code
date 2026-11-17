// =====================================================================
//  Ariadna V2 — the parameter table
//
//  Every number a person may want to change without recompiling: the
//  timing of the flight events, the guidance gains, the fin options, the
//  servo calibration, the sensor options. One table, one set of ids, shared
//  by the three flight boards and listed to the diagnostic GUI.
//
//  HOW IT MOVES
//    * The MASTER owns the table. It keeps it in sectors 0 and 1 of its
//      W25Q128 flash (two copies, newest valid one wins) and loads it at boot
//      - defaults if neither copy passes its CRC and layout version. The
//      RP2350 has no EEPROM: arduino-pico's EEPROM.h emulates one in the
//      program flash, and writing that stalls BOTH cores.
//    * The diagnostic GUI edits it over USB: PARAM SET / SAVE / DEFAULTS.
//      Edits are accepted only on the ground (IDLE or TEST).
//    * The Master streams the whole table, one ARIADNA_PKT_PARAM at a
//      time, round-robin, to the Position and Navigation MCUs for as long
//      as the vehicle is on the ground. No acknowledgements: both boards
//      report the CRC of the table they hold in their status packets, and
//      the Master compares it with its own. A board that reset picks the
//      table up again within a few seconds on its own.
//    * In flight (ASCENT and later) the Master stops sending and the other
//      boards stop listening. A parameter cannot change under a burn.
//
//  This header has no Arduino dependency: test/seq_test.cpp checks that the
//  defaults here are the guidance module's own, and that the ranges hold.
//
//  ADDING A PARAMETER: append a line with the next free id. Never reuse or
//  renumber an id - the flash image and the GUI's saved profiles are keyed
//  on them. Bump ARIADNA_PARAM_LAYOUT only if a default must be forced.
//
//  Ids 96..163 are the IGNITION BAND TABLE (tbl.*), generated from
//  LandSim_Light into ariadna_params_tbl.h. The diagnostic GUI recomputes it
//  for the vehicle you enter and uploads it like any other parameter.
// =====================================================================
#ifndef ARIADNA_PARAMS_H
#define ARIADNA_PARAMS_H

#include "ariadna_params_tbl.h"

#include <stdint.h>
#include <stddef.h>

#define ARIADNA_PARAM_LAYOUT   4     // stored with the flash image (4: 200 ids - calibration, pad, servo check)
// Layout 3 images (164 ids) are still read: the ids they hold keep their
// stored values, the new ones start at their defaults. See the Master.
#define ARIADNA_PARAM_ID_MAX_V3 164

// X(id, NAME, key, default, min, max, unit, owner)
//   owner: 'N' Navigation MCU, 'M' Master, 'P' Position MCU
//   key  : the short name the GUI and the USB console use
#define ARIADNA_PARAM_LIST(X) \
    /* ---- flight event timing (Navigation MCU sequencer) ---- */ \
    X( 0, SEQ_COVER_LEAD,   "seq.cover_lead_s",   0.60f, 0.0f,  5.0f, "s",    'N') \
    X( 1, SEQ_BACKUP_WIN,   "seq.backup_win_s",   0.15f, 0.05f, 1.0f, "s",    'N') \
    X( 2, SEQ_LEGS_LEAD,    "seq.legs_lead_s",    1.70f, 0.3f, 10.0f, "s",    'N') \
    X( 3, SEQ_LEGS_PULSE,   "seq.legs_pulse_s",   0.50f, 0.5f,  3.0f, "s",    'N') \
    X( 4, SEQ_IGN_PULSE,    "seq.ign_pulse_s",    0.60f, 0.1f,  2.0f, "s",    'N') \
    X( 5, SEQ_COVER_EN,     "seq.cover_enable",   1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X( 6, SEQ_BACKUP_EN,    "seq.backup_enable",  1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X( 7, SEQ_LEGS_EN,      "seq.legs_enable",    1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X( 8, SEQ_LEGS_H_MAX,   "seq.legs_h_max_m",  60.0f,  5.0f, 200.0f, "m",   'N') \
    X( 9, SEQ_APOGEE_MIN,   "seq.apogee_min_m",  40.0f, 10.0f, 500.0f, "m",   'N') \
    /* ---- guidance: vehicle ---- */ \
    X(10, G_MASS_GROSS,     "g.mass_gross_kg",    2.85f, 1.0f, 10.0f, "kg",   'N') \
    X(11, G_MASS_PROP,      "g.mass_prop_kg",     0.277f, 0.05f, 2.0f, "kg",  'N') \
    /* ---- guidance: nozzle attitude loop ---- */ \
    X(12, G_WN_TVC,         "g.wn_tvc",           8.230423f, 1.0f, 20.0f, "rad/s", 'N') \
    X(13, G_ZETA_TVC,       "g.zeta_tvc",         1.039857f, 0.3f, 3.0f, "-",  'N') \
    X(14, G_SCHED_TVC,      "g.sched_tvc",        0.6f, -1.0f, 1.0f, "-",  'N') \
    X(15, G_ATT_KI,         "g.att_ki",           2.0f,  0.0f,  4.0f, "1/s",  'N') \
    X(16, G_ATT_I_LIM,      "g.att_i_lim",        0.05f, 0.0f,  0.2f, "rad*s", 'N') \
    /* ---- guidance: fin attitude loop ---- */ \
    X(17, G_WN_FIN,         "g.wn_fin",           2.0f, 1.0f, 20.0f, "rad/s", 'N') \
    X(18, G_ZETA_FIN,       "g.zeta_fin",         0.994114f, 0.3f, 3.0f, "-",  'N') \
    X(19, G_SCHED_FIN,      "g.sched_fin",        -0.6f, -1.0f, 1.0f, "-", 'N') \
    X(20, G_ROLL_GAIN,      "g.roll_gain",        0.748533f, 0.0f, 10.0f, "rad/s", 'N') \
    X(21, G_ROLL_MAX,       "g.roll_max_deg",     2.0f,  0.0f,  6.0f, "deg",  'N') \
    /* ---- guidance: where to point, when to light ---- */ \
    X(22, G_TILT_MIN,       "g.tilt_min_deg",     1.5f,  0.0f, 10.0f, "deg",  'N') \
    X(23, G_TILT_SLOPE,     "g.tilt_slope",       1.5f,  0.0f,  5.0f, "deg/m", 'N') \
    X(24, G_TILT_CAP,       "g.tilt_cap_deg",    20.0f,  2.0f, 45.0f, "deg",  'N') \
    X(25, G_ALT_PAD,        "g.alt_pad_m",        1.0f,  0.0f,  5.0f, "m",    'N') \
    X(26, G_H_IGN_BIAS,     "g.h_ign_bias_m",     0.0f, -20.0f, 30.0f, "m",   'N') \
    X(27, G_H_IGN_OVERRIDE, "g.h_ign_override_m", 0.0f,  0.0f, 300.0f, "m",   'N') \
    /* ---- guidance: clamp planner ---- */ \
    X(28, G_PLAN_TARGET,    "g.plan_target_mps", -0.5f, -3.0f,  0.0f, "m/s",  'N') \
    X(29, G_PLAN_MARGIN,    "g.plan_margin",      0.03f, 0.0f,  0.2f, "-",    'N') \
    X(30, G_PLAN_HOLD,      "g.plan_hold_s",      0.30f, 0.1f,  1.0f, "s",    'N') \
    /* ---- guidance: thrust onset ---- */ \
    X(31, G_ONSET_BUMP,     "g.onset_bump",       4.0f,  1.0f, 30.0f, "m/s2", 'N') \
    X(32, G_ONSET_TIMEOUT,  "g.onset_timeout_s",  0.5f,  0.2f,  3.0f, "s",    'N') \
    X(33, G_ONSET_ASSUMED,  "g.onset_assumed_s",  0.15f, 0.0f,  2.0f, "s",    'N') \
    /* ---- guidance: fins ---- */ \
    X(34, G_FIN_ENABLE,     "fin.enable",         1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X(35, G_FIN_MAX,        "fin.max_deg",       15.0f,  2.0f, 30.0f, "deg",  'N') \
    X(36, G_FIN_BRAKE,      "fin.brake",          1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X(37, G_FIN_MIN_V,      "fin.min_airspeed",   8.0f,  3.0f, 30.0f, "m/s",  'N') \
    X(38, G_FIN_DRIFT_GAIN, "fin.drift_gain",     1.5f,  0.0f,  4.0f, "deg/(m/s)", 'N') \
    X(39, G_FIN_DRIFT_MAX,  "fin.drift_max_deg",  5.0f,  0.0f, 10.0f, "deg",  'N') \
    X(40, G_FIN_CTRL_LEAD,  "fin.ctrl_lead_m",   40.0f,  0.0f, 200.0f, "m",   'N') \
    X(41, G_FIN_ASCENT,     "fin.ascent_mode",    2.0f,  0.0f,  2.0f, "0/1/2", 'N') \
    X(42, G_FIN_ADAPT,      "fin.adapt",          1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X(43, G_FIN_GAIN0,      "fin.gain0",          1.0f,  0.4f,  2.0f, "-",    'N') \
    /* ---- guidance: the vehicle as the ignition search sees it ---- */ \
    X(44, G_CD_BODY,        "g.cd_body",          0.35f, 0.1f,  1.5f, "-",    'N') \
    X(45, G_THRUST_SCALE,   "g.thrust_scale",     1.0f,  0.5f,  2.0f, "-",    'N') \
    X(46, G_IGN_DELAY_PAD,  "g.ign_delay_pad_s",  0.40f, 0.0f,  1.5f, "s",    'N') \
    X(47, G_DRAG_ADAPT,     "g.drag_adapt",       1.0f,  0.0f,  1.0f, "0/1",  'N') \
    X(48, G_DIAMETER,       "g.diameter_m",       0.105f, 0.03f, 0.3f, "m",   'N') \
    X(49, G_PLAN_TS_TAU,    "g.plan_ts_tau_s",    0.70f, 0.0f,  5.0f, "s",    'N') \
    /* ---- servo calibration (Master) ---- */ \
    X(50, SRV_TVC_TRIM_X,   "srv.tvc_x_trim_us", 1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(51, SRV_TVC_TRIM_Y,   "srv.tvc_y_trim_us", 1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(52, SRV_TVC_US_DEG,   "srv.tvc_us_per_deg", 10.0f,  2.0f,  30.0f, "us/deg", 'M') \
    X(53, SRV_TVC_SIGN_X,   "srv.tvc_x_sign",     1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(54, SRV_TVC_SIGN_Y,   "srv.tvc_y_sign",     1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(55, SRV_CLAMP_CLOSED, "srv.clamp_closed_us", 1150.0f, 500.0f, 2500.0f, "us", 'M') \
    X(56, SRV_CLAMP_OPEN,   "srv.clamp_open_us",  1900.0f, 500.0f, 2500.0f, "us", 'M') \
    X(57, SRV_FIN_US_DEG,   "srv.fin_us_per_deg", 10.0f,  2.0f,  30.0f, "us/deg", 'M') \
    X(58, SRV_FIN1_TRIM,    "srv.fin1_trim_us",  1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(59, SRV_FIN2_TRIM,    "srv.fin2_trim_us",  1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(60, SRV_FIN3_TRIM,    "srv.fin3_trim_us",  1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(61, SRV_FIN4_TRIM,    "srv.fin4_trim_us",  1500.0f, 900.0f, 2100.0f, "us", 'M') \
    X(62, SRV_FIN1_SIGN,    "srv.fin1_sign",      1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(63, SRV_FIN2_SIGN,    "srv.fin2_sign",      1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(64, SRV_FIN3_SIGN,    "srv.fin3_sign",      1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(65, SRV_FIN4_SIGN,    "srv.fin4_sign",      1.0f, -1.0f,   1.0f, "+1/-1", 'M') \
    X(66, SRV_COVER_LOCK,   "srv.cover_lock_us", 1200.0f, 500.0f, 2500.0f, "us", 'M') \
    X(67, SRV_COVER_UNLOCK, "srv.cover_unlock_us", 1800.0f, 500.0f, 2500.0f, "us", 'M') \
    /* ---- mission state machine (Master) ---- */ \
    X(70, MST_LAUNCH_G,     "mst.launch_g",       3.0f,  1.5f,  20.0f, "g",   'M') \
    X(71, MST_AUTO_ARM,     "mst.auto_arm",       1.0f,  0.0f,   1.0f, "0/1", 'M') \
    /* ---- navigation (Position MCU) ---- */ \
    X(80, POS_GPS_LATENCY,  "pos.gps_latency_ms", 50.0f, 0.0f, 500.0f, "ms",  'P') \
    X(81, POS_LIDAR_EN,     "pos.lidar_enable",   1.0f,  0.0f,   1.0f, "0/1", 'P') \
    X(82, POS_LIDAR_OFFSET, "pos.lidar_offset_m", 0.0f, -2.0f,   2.0f, "m",   'P') \
    X(83, POS_LIDAR_MAX,    "pos.lidar_max_m",    7.5f,  1.0f,  12.0f, "m",   'P') \
    X(84, POS_ATT_FREEZE,   "pos.att_gyro_only",  1.0f,  0.0f,   1.0f, "0/1", 'P') \
    X(85, POS_UWB_EN,       "pos.uwb_enable",     1.0f,  0.0f,   1.0f, "0/1", 'P') \
    X(86, POS_LIDAR_DELAY,  "pos.lidar_delay_ms", 500.0f, 0.0f, 3000.0f, "ms", 'P') \
    X(87, POS_LIDAR_TILT_X, "pos.lidar_tilt_x_deg", 0.0f, -20.0f, 20.0f, "deg", 'P') \
    X(88, POS_LIDAR_TILT_Y, "pos.lidar_tilt_y_deg", 0.0f, -20.0f, 20.0f, "deg", 'P') \
    /* ---- guidance: fin drag (LandSim band.json fin_cd_* minus g.cd_body) ---- */ \
    X(89, G_FIN_DCD_ZERO,   "g.fin_dcd_zero",     0.14306195f, 0.0f, 1.5f, "-",   'N') \
    X(90, G_FIN_DCD_BRAKE,  "g.fin_dcd_brake",    0.44720534f, 0.0f, 3.0f, "-",   'N') \
    ARIADNA_PARAM_LIST_TBL(X) \
    /* ---- TriSense calibration (Position MCU), from the GUI's calibration tab ---- */ \
    /* accelerometer: a = (raw - off) * scale, in g, board axes                     */ \
    X(164, IMU_ACC_OFF_X,   "imu.acc_off_x",      0.0f, -0.5f,   0.5f, "g",   'P') \
    X(165, IMU_ACC_OFF_Y,   "imu.acc_off_y",      0.0f, -0.5f,   0.5f, "g",   'P') \
    X(166, IMU_ACC_OFF_Z,   "imu.acc_off_z",      0.0f, -0.5f,   0.5f, "g",   'P') \
    X(167, IMU_ACC_SCL_X,   "imu.acc_scale_x",    1.0f,  0.8f,   1.2f, "-",   'P') \
    X(168, IMU_ACC_SCL_Y,   "imu.acc_scale_y",    1.0f,  0.8f,   1.2f, "-",   'P') \
    X(169, IMU_ACC_SCL_Z,   "imu.acc_scale_z",    1.0f,  0.8f,   1.2f, "-",   'P') \
    /* magnetometer: m = SI * (raw - HI), uT, board axes (MotionCal convention)     */ \
    X(170, IMU_MAG_HI_X,    "imu.mag_hi_x",       0.0f, -500.0f, 500.0f, "uT", 'P') \
    X(171, IMU_MAG_HI_Y,    "imu.mag_hi_y",       0.0f, -500.0f, 500.0f, "uT", 'P') \
    X(172, IMU_MAG_HI_Z,    "imu.mag_hi_z",       0.0f, -500.0f, 500.0f, "uT", 'P') \
    X(173, IMU_MAG_SI_00,   "imu.mag_si_00",      1.0f, -3.0f,   3.0f, "-",   'P') \
    X(174, IMU_MAG_SI_01,   "imu.mag_si_01",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(175, IMU_MAG_SI_02,   "imu.mag_si_02",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(176, IMU_MAG_SI_10,   "imu.mag_si_10",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(177, IMU_MAG_SI_11,   "imu.mag_si_11",      1.0f, -3.0f,   3.0f, "-",   'P') \
    X(178, IMU_MAG_SI_12,   "imu.mag_si_12",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(179, IMU_MAG_SI_20,   "imu.mag_si_20",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(180, IMU_MAG_SI_21,   "imu.mag_si_21",      0.0f, -3.0f,   3.0f, "-",   'P') \
    X(181, IMU_MAG_SI_22,   "imu.mag_si_22",      1.0f, -3.0f,   3.0f, "-",   'P') \
    /* ---- the pad (Position MCU). 0 / 0 = not set: the origin is the GPS fix  */ \
    /*      at arming, as before. Set from the GUI ("Rampa = tady").            */ \
    X(182, POS_PAD_LAT,     "pos.pad_lat_deg",    0.0f, -90.0f,  90.0f, "deg", 'P') \
    X(183, POS_PAD_LON,     "pos.pad_lon_deg",    0.0f, -180.0f, 180.0f, "deg", 'P') \
    /* ---- servo connection check (Master) ---- */ \
    X(184, MST_SRV_CHECK_ARM, "mst.servo_check_on_rbf", 1.0f, 0.0f, 1.0f, "0/1", 'M') \
    X(185, MST_SRV_CHECK_MA,  "mst.servo_check_ma",    60.0f, 10.0f, 1000.0f, "mA", 'M')

// ---- ids --------------------------------------------------------------
enum {
#define X(id, NAME, key, def, lo, hi, unit, owner) ARIADNA_P_##NAME = id,
    ARIADNA_PARAM_LIST(X)
#undef X
    ARIADNA_PARAM_ID_MAX = 200         // ids are 0 .. ARIADNA_PARAM_ID_MAX-1
};

typedef struct {
    uint8_t     id;
    char        owner;
    const char *key;
    const char *unit;
    float       def, lo, hi;
} ariadna_param_info_t;

static const ariadna_param_info_t ARIADNA_PARAM_INFO[] = {
#define X(id, NAME, key, def, lo, hi, unit, owner) { id, owner, key, unit, def, lo, hi },
    ARIADNA_PARAM_LIST(X)
#undef X
};
#define ARIADNA_PARAM_COUNT \
    ((uint8_t)(sizeof(ARIADNA_PARAM_INFO) / sizeof(ARIADNA_PARAM_INFO[0])))

/** The table itself: indexed by id, unused ids stay 0. */
typedef struct {
    float v[ARIADNA_PARAM_ID_MAX];
} ariadna_params_t;

static inline const ariadna_param_info_t *ariadna_param_find(uint8_t id)
{
    for (uint8_t i = 0; i < ARIADNA_PARAM_COUNT; i++)
        if (ARIADNA_PARAM_INFO[i].id == id) return &ARIADNA_PARAM_INFO[i];
    return NULL;
}

static inline void ariadna_params_default(ariadna_params_t *p)
{
    for (uint8_t i = 0; i < ARIADNA_PARAM_ID_MAX; i++) p->v[i] = 0.0f;
    for (uint8_t i = 0; i < ARIADNA_PARAM_COUNT; i++)
        p->v[ARIADNA_PARAM_INFO[i].id] = ARIADNA_PARAM_INFO[i].def;
}

/**
 * Set one parameter, clamped to its range. Returns 0 for an unknown id or
 * a value that is not a number: a NaN in the table would propagate through
 * every control law that reads it.
 */
static inline int ariadna_param_set(ariadna_params_t *p, uint8_t id, float value)
{
    const ariadna_param_info_t *info = ariadna_param_find(id);
    if (!info) return 0;
    if (!(value == value)) return 0;                   // NaN
    if (value < info->lo) value = info->lo;
    if (value > info->hi) value = info->hi;
    p->v[id] = value;
    return 1;
}

/** CRC-16/CCITT-FALSE over the first n_ids values (a stored image's own size). */
static inline uint16_t ariadna_params_crc_n(const float *v, size_t n_ids)
{
    const uint8_t *b = (const uint8_t *)v;
    uint16_t crc = 0xFFFF;
    for (size_t n = 0; n < n_ids * sizeof(float); n++) {
        crc ^= (uint16_t)b[n] << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/** CRC-16/CCITT-FALSE over the table bytes: what the boards compare. */
static inline uint16_t ariadna_params_crc(const ariadna_params_t *p)
{
    const uint8_t *b = (const uint8_t *)p->v;
    uint16_t crc = 0xFFFF;
    for (size_t n = 0; n < sizeof(p->v); n++) {
        crc ^= (uint16_t)b[n] << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

#endif // ARIADNA_PARAMS_H
