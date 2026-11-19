// =====================================================================
//  Ariadna V2 — Hardware-in-the-Loop (HIL) USB sensor injection
//
//  Header-only, zero Arduino dependency so ../../test/hil_test.cpp tests
//  the exact same struct layout, dual ASCII/binary USB parser and IMU
//  frame rotation that run on the Position MCU.
//
//  How HIL works across the three boards:
//
//      PC (daidalos_diag.py)
//        |-- USB CDC (binary ARIADNA_PKT_HIL_IN @ 100 Hz) --> Position MCU
//        |-- USB CDC (ASCII HIL 1 [0|1], H @ 50 Hz)       <-> Master MCU
//
//      Position MCU replaces its physical TriSense / BMP580 / GPS /
//      TF-Luna / UWB reads with the synthetic measurements carried in
//      ariadna_hil_in_t, runs its REAL Kalman filter, UWB solver, lidar
//      leg gate and ZUPT, and ships the resulting ariadna_nav_state_t
//      over the real inter-MCU UART to the Master MCU -> Navigation MCU.
//
//      Master MCU and Navigation MCU run the real flight state machine,
//      guidance, actuator commands and sequencer. Physical pyro pins on
//      Navigation MCU remain strictly inhibited whenever Master sets
//      ARIADNA_MST_PYRO_INHIBIT (HIL default: safe).
//
//  This file is the original; `cd ../../test && make sync-fix` copies it
//  into Position_MCU/.
// =====================================================================
#ifndef ARIADNA_HIL_H
#define ARIADNA_HIL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "ariadna_link.h"

// Watchdog timeout: if the PC stops streaming valid HIL frames for this
// long, Position MCU exits HIL mode and returns to its physical sensors.
#define ARIADNA_HIL_TIMEOUT_MS    500u

// --- ariadna_hil_in_t.flags ------------------------------------------
#define ARIADNA_HIL_BARO_FRESH    (1u << 0)  // feed pressure_pa into baro filter
#define ARIADNA_HIL_GPS_FRESH     (1u << 1)  // feed GPS fix into horizontal filter
#define ARIADNA_HIL_LIDAR_VALID   (1u << 2)  // lidar_mm is a valid slant reading
#define ARIADNA_HIL_UWB_FRESH     (1u << 3)  // feed uwb_range_cm[] into UWB solver
#define ARIADNA_HIL_SET_PAD_REF   (1u << 4)  // take pad pressure & GPS origin now
#define ARIADNA_HIL_RESET_EST     (1u << 5)  // reset KF state before fusing packet

// ---------------------------------------------------------------------
//  PC -> Position MCU synthetic sensor frame (84 bytes, 4-byte aligned)
//
//  Laid out 4-byte fields first, then 2-byte, then 1-byte so packed and
//  natural layouts are identical (same rule as ariadna_link.h).
//  Python struct format pinned by ../../test/check_hil_format.py:
//      "<I11fiifhhHH4HhBBBBBB"
// ---------------------------------------------------------------------
typedef struct {
    // 4-byte fields (15 * 4 = 60 B)
    uint32_t seq;                //  0  monotonic packet sequence number
    float    qw, qx, qy, qz;     //  4  body (TriSense) -> pad-ENU quaternion
    float    acc_body_ms2[3];    // 20  specific force in body axes, m/s^2
    float    gyr_body_rads[3];   // 32  angular velocity in body axes, rad/s
    float    pressure_pa;        // 44  static barometric pressure, Pa
    int32_t  gps_lat_1e7;        // 48  latitude, 1e-7 deg
    int32_t  gps_lon_1e7;        // 52  longitude, 1e-7 deg
    float    gps_alt_msl;        // 56  GPS altitude MSL, m
    // 2-byte fields (9 * 2 = 18 B)
    int16_t  gps_ve_cms;         // 60  GPS East velocity, cm/s
    int16_t  gps_vn_cms;         // 62  GPS North velocity, cm/s
    uint16_t lidar_mm;           // 64  TF-Luna slant range, mm (0 = none)
    uint16_t lidar_strength;     // 66  TF-Luna return strength (0..65535)
    uint16_t uwb_range_cm[4];    // 68  slant range to UWB anchors 0..3, cm
    int16_t  temp_c_x10;         // 76  barometer temperature, 0.1 deg C
    // 1-byte fields (6 * 1 = 6 B)
    uint8_t  flags;              // 78  ARIADNA_HIL_* bits
    uint8_t  gps_sats;           // 79  tracked satellites
    uint8_t  gps_fix;            // 80  0 none, 2 2D, 3 3D
    uint8_t  gps_hdop_x10;       // 81  HDOP * 10
    uint8_t  uwb_mask;           // 82  bit i: uwb_range_cm[i] valid
    uint8_t  _pad;               // 83  reserved, 0
} ariadna_hil_in_t;
ARIADNA_STATIC_ASSERT(sizeof(ariadna_hil_in_t) == 84, "hil_in layout changed");
ARIADNA_STATIC_ASSERT(sizeof(ariadna_hil_in_t) <= ARIADNA_PAYLOAD_MAX, "hil_in exceeds frame max");

// ---------------------------------------------------------------------
//  Dual USB CDC parser: ASCII control commands + binary HIL frames
//
//  ASCII commands (newline terminated, uppercase):
//      HELLO       -> GUI probes the port to confirm it is Position MCU
//      HIL 0       -> immediate clean exit from HIL mode
//      HIL STATUS  -> query HIL state / packet counters
//
//  Binary frames: standard ariadna_link.h frames with type
//  ARIADNA_PKT_HIL_IN and payload sizeof(ariadna_hil_in_t).
// ---------------------------------------------------------------------
typedef enum {
    ARIADNA_HIL_RX_NONE   = 0,
    ARIADNA_HIL_RX_PKT    = 1,
    ARIADNA_HIL_RX_HELLO  = 2,
    ARIADNA_HIL_RX_STOP   = 3,
    ARIADNA_HIL_RX_STATUS = 4
} ariadna_hil_rx_event_t;

typedef struct {
    ariadna_rx_t bin;
    char         line[48];
    uint8_t      line_len;
} ariadna_hil_rx_t;

static inline void ariadna_hil_rx_init(ariadna_hil_rx_t *rx)
{
    ariadna_rx_init(&rx->bin);
    rx->line_len = 0;
    rx->line[0] = '\0';
}

static inline ariadna_hil_rx_event_t ariadna_hil_match_ascii(const char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    if (strcmp(s, "HELLO") == 0)      return ARIADNA_HIL_RX_HELLO;
    if (strcmp(s, "HIL 0") == 0)      return ARIADNA_HIL_RX_STOP;
    if (strcmp(s, "HIL STATUS") == 0) return ARIADNA_HIL_RX_STATUS;
    return ARIADNA_HIL_RX_NONE;
}

/**
 * Feed one byte from Position MCU's USB CDC port.
 * On ARIADNA_HIL_RX_PKT, `out_pkt` is populated with the validated packet.
 */
static inline ariadna_hil_rx_event_t ariadna_hil_rx_byte(
    ariadna_hil_rx_t *rx, uint8_t b, ariadna_hil_in_t *out_pkt)
{
    const uint16_t pending = (rx->bin.n > rx->bin.consume)
                                 ? (uint16_t)(rx->bin.n - rx->bin.consume)
                                 : 0u;
    if (pending != 0u || b == ARIADNA_SYNC0) {
        const uint8_t *pl = NULL;
        uint8_t len = 0;
        const uint8_t t = ariadna_rx_byte(&rx->bin, b, &pl, &len);
        rx->line_len = 0;
        if (t == ARIADNA_PKT_HIL_IN &&
            ariadna_unpack(pl, len, out_pkt, sizeof(*out_pkt))) {
            return ARIADNA_HIL_RX_PKT;
        }
        const uint16_t after = (rx->bin.n > rx->bin.consume)
                                   ? (uint16_t)(rx->bin.n - rx->bin.consume)
                                   : 0u;
        // If we had only seen a stray 0xA5 and the next byte wasn't SYNC1
        // or SYNC0, let printable ASCII still start a command.
        if (!(pending == 1u && after == 0u && b >= 0x20 && b <= 0x7E)) {
            return ARIADNA_HIL_RX_NONE;
        }
    }

    if (b == '\r' || b == '\n') {
        if (rx->line_len > 0) {
            rx->line[rx->line_len] = '\0';
            while (rx->line_len > 0 &&
                   (rx->line[rx->line_len - 1] == ' ' ||
                    rx->line[rx->line_len - 1] == '\t')) {
                rx->line[--rx->line_len] = '\0';
            }
            const ariadna_hil_rx_event_t ev = ariadna_hil_match_ascii(rx->line);
            rx->line_len = 0;
            return ev;
        }
        return ARIADNA_HIL_RX_NONE;
    }

    if (b >= 0x20 && b <= 0x7E) {
        if (rx->line_len < (uint8_t)(sizeof(rx->line) - 1u)) {
            rx->line[rx->line_len++] = (char)b;
        } else {
            rx->line_len = 0;
        }
    } else {
        rx->line_len = 0;
    }
    return ARIADNA_HIL_RX_NONE;
}

// ---------------------------------------------------------------------
//  IMU / attitude conversion (single precision only — RP2350 FPU)
//
//  Turns the body-frame specific force, body-frame gyro rate and
//  body->pad-ENU quaternion from ariadna_hil_in_t into the exact fields
//  Position_MCU's Attitude struct expects.
// ---------------------------------------------------------------------
typedef struct {
    float R[3][3];
    float q[4];
    float w_pad[3];
    float w_abs;
    float a_pad[3];
    float roll, pitch, heading;
} ariadna_hil_imu_out_t;

static inline bool ariadna_hil_finite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static inline bool ariadna_hil_apply_imu(const ariadna_hil_in_t *in,
                                         ariadna_hil_imu_out_t *out)
{
    if (!isfinite(in->qw) || !isfinite(in->qx) ||
        !isfinite(in->qy) || !isfinite(in->qz)) {
        return false;
    }
    if (!ariadna_hil_finite3(in->acc_body_ms2) ||
        !ariadna_hil_finite3(in->gyr_body_rads)) {
        return false;
    }

    const float n2 = in->qw * in->qw + in->qx * in->qx +
                     in->qy * in->qy + in->qz * in->qz;
    if (!(n2 > 1e-6f)) return false;

    const float inv_n = 1.0f / sqrtf(n2);
    const float w = in->qw * inv_n;
    const float x = in->qx * inv_n;
    const float y = in->qy * inv_n;
    const float z = in->qz * inv_n;

    out->q[0] = w;
    out->q[1] = x;
    out->q[2] = y;
    out->q[3] = z;

    out->R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    out->R[0][1] = 2.0f * (x * y - w * z);
    out->R[0][2] = 2.0f * (x * z + w * y);
    out->R[1][0] = 2.0f * (x * y + w * z);
    out->R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    out->R[1][2] = 2.0f * (y * z - w * x);
    out->R[2][0] = 2.0f * (x * z - w * y);
    out->R[2][1] = 2.0f * (y * z + w * x);
    out->R[2][2] = 1.0f - 2.0f * (x * x + y * y);

    const float *g = in->gyr_body_rads;
    const float *a = in->acc_body_ms2;
    for (uint8_t i = 0; i < 3; i++) {
        out->w_pad[i] = out->R[i][0] * g[0] + out->R[i][1] * g[1] + out->R[i][2] * g[2];
        out->a_pad[i] = out->R[i][0] * a[0] + out->R[i][1] * a[1] + out->R[i][2] * a[2];
    }
    // Remove gravity (+z is Up in pad-ENU, specific force on stationary pad is +g*z)
    out->a_pad[2] -= 9.80665f;

    const float rad2deg = 57.2957795f;
    out->w_abs = (fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2])) * rad2deg;

    // Telemetry-only Euler triple (never read by any control law)
    out->roll = atan2f(out->R[2][1], out->R[2][2]) * rad2deg;
    float sp = -out->R[2][0];
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    out->pitch = asinf(sp) * rad2deg;
    float hdg = atan2f(out->R[0][0], out->R[1][0]) * rad2deg;
    if (hdg < 0.0f) hdg += 360.0f;
    out->heading = hdg;
    return true;
}

#endif // ARIADNA_HIL_H
