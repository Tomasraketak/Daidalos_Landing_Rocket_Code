// =====================================================================
//  Host verification of Ariadna V2 HIL USB injection (ariadna_hil.h)
//
//  Tests:
//    1. Struct size, alignment, and exact field offsets
//    2. Dual binary + ASCII USB CDC demux (back-to-back, interleaved,
//       stray 0xA5 before ASCII, ASCII text embedded inside binary
//       payload, CRC error rejection)
//    3. IMU / attitude conversion (pad upright, nose-first descent,
//       arbitrary quaternion, gravity subtraction, NaN/zero rejection)
//
//  Build:  cd test && make hil
// =====================================================================
#include "../Ariadna_V2/common/ariadna_hil.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>

static int g_checks = 0;
static int g_fails  = 0;

static void check(bool ok, const char *what)
{
    g_checks++;
    if (!ok) {
        g_fails++;
        printf("  FAIL  %s\n", what);
    }
}

static bool nearf(float a, float b, float tol = 1e-4f)
{
    return fabsf(a - b) <= tol;
}

static void test_layout(void)
{
    printf("hil layout\n");
    check(sizeof(ariadna_hil_in_t) == 84, "hil_in is 84 bytes");
    check(sizeof(ariadna_hil_in_t) % 4 == 0, "hil_in 4-byte aligned");
    check(sizeof(ariadna_hil_in_t) <= ARIADNA_PAYLOAD_MAX, "hil_in fits in frame");

    check(offsetof(ariadna_hil_in_t, seq)            ==  0, "seq at 0");
    check(offsetof(ariadna_hil_in_t, qw)             ==  4, "qw at 4");
    check(offsetof(ariadna_hil_in_t, acc_body_ms2)   == 20, "acc at 20");
    check(offsetof(ariadna_hil_in_t, gyr_body_rads)  == 32, "gyr at 32");
    check(offsetof(ariadna_hil_in_t, pressure_pa)    == 44, "pressure at 44");
    check(offsetof(ariadna_hil_in_t, gps_lat_1e7)    == 48, "lat at 48");
    check(offsetof(ariadna_hil_in_t, gps_lon_1e7)    == 52, "lon at 52");
    check(offsetof(ariadna_hil_in_t, gps_alt_msl)    == 56, "alt at 56");
    check(offsetof(ariadna_hil_in_t, gps_ve_cms)     == 60, "ve at 60");
    check(offsetof(ariadna_hil_in_t, gps_vn_cms)     == 62, "vn at 62");
    check(offsetof(ariadna_hil_in_t, lidar_mm)       == 64, "lidar_mm at 64");
    check(offsetof(ariadna_hil_in_t, lidar_strength) == 66, "lidar_str at 66");
    check(offsetof(ariadna_hil_in_t, uwb_range_cm)   == 68, "uwb at 68");
    check(offsetof(ariadna_hil_in_t, temp_c_x10)     == 76, "temp at 76");
    check(offsetof(ariadna_hil_in_t, flags)          == 78, "flags at 78");
    check(offsetof(ariadna_hil_in_t, gps_sats)       == 79, "sats at 79");
    check(offsetof(ariadna_hil_in_t, gps_fix)        == 80, "fix at 80");
    check(offsetof(ariadna_hil_in_t, gps_hdop_x10)   == 81, "hdop at 81");
    check(offsetof(ariadna_hil_in_t, uwb_mask)       == 82, "uwb_mask at 82");
    check(offsetof(ariadna_hil_in_t, _pad)           == 83, "_pad at 83");
}

static void test_parser(void)
{
    printf("hil dual parser\n");
    ariadna_hil_rx_t rx;
    ariadna_hil_rx_init(&rx);
    ariadna_hil_in_t out;

    // 1. ASCII HELLO
    const char *hello = "HELLO\r\n";
    int got_hello = 0;
    for (size_t i = 0; hello[i]; i++) {
        if (ariadna_hil_rx_byte(&rx, (uint8_t)hello[i], &out) == ARIADNA_HIL_RX_HELLO)
            got_hello++;
    }
    check(got_hello == 1, "HELLO parsed");

    // 2. Binary packet containing ASCII "HIL 0\n" bytes inside its payload must NOT trigger STOP
    ariadna_hil_in_t in;
    memset(&in, 0, sizeof(in));
    in.seq = 42u;
    in.qw = 1.0f;
    in.pressure_pa = 101325.0f;
    in.flags = ARIADNA_HIL_BARO_FRESH | ARIADNA_HIL_GPS_FRESH;
    // Plant "HIL 0\n" bytes inside acc_body_ms2 raw bytes
    memcpy(in.acc_body_ms2, "HIL 0\n", 6);

    uint8_t frame[ARIADNA_FRAME_MAX];
    size_t flen = ariadna_encode(frame, ARIADNA_PKT_HIL_IN, &in, sizeof(in));
    check(flen == 90, "packed frame is 90 bytes");

    int got_pkt = 0, got_stop = 0;
    for (size_t i = 0; i < flen; i++) {
        ariadna_hil_rx_event_t ev = ariadna_hil_rx_byte(&rx, frame[i], &out);
        if (ev == ARIADNA_HIL_RX_PKT)  got_pkt++;
        if (ev == ARIADNA_HIL_RX_STOP) got_stop++;
    }
    check(got_pkt == 1, "binary HIL packet decoded");
    check(got_stop == 0, "embedded ASCII inside binary did not trigger STOP");
    check(out.seq == 42u && nearf(out.pressure_pa, 101325.0f), "payload intact");

    // 3. Immediately followed by ASCII "HIL 0\n"
    const char *stop = "HIL 0\n";
    for (size_t i = 0; stop[i]; i++) {
        if (ariadna_hil_rx_byte(&rx, (uint8_t)stop[i], &out) == ARIADNA_HIL_RX_STOP)
            got_stop++;
    }
    check(got_stop == 1, "HIL 0 parsed immediately after binary frame");

    // 4. Stray 0xA5 before HELLO
    ariadna_hil_rx_byte(&rx, ARIADNA_SYNC0, &out);
    int got_hello2 = 0;
    for (size_t i = 0; hello[i]; i++) {
        if (ariadna_hil_rx_byte(&rx, (uint8_t)hello[i], &out) == ARIADNA_HIL_RX_HELLO)
            got_hello2++;
    }
    check(got_hello2 == 1, "HELLO parsed after stray SYNC0");

    // 5. Corrupted CRC must be rejected, followed by valid packet
    uint8_t bad[ARIADNA_FRAME_MAX];
    memcpy(bad, frame, flen);
    bad[10] ^= 0xFF;
    int bad_pkts = 0;
    for (size_t i = 0; i < flen; i++) {
        if (ariadna_hil_rx_byte(&rx, bad[i], &out) == ARIADNA_HIL_RX_PKT) bad_pkts++;
    }
    check(bad_pkts == 0, "corrupted CRC rejected");
    check(rx.bin.rx_crc == 1, "rx_crc incremented");

    in.seq = 99u;
    flen = ariadna_encode(frame, ARIADNA_PKT_HIL_IN, &in, sizeof(in));
    int rec_pkts = 0;
    for (size_t i = 0; i < flen; i++) {
        if (ariadna_hil_rx_byte(&rx, frame[i], &out) == ARIADNA_HIL_RX_PKT) rec_pkts++;
    }
    check(rec_pkts == 1 && out.seq == 99u, "recovered immediately on next valid frame");
}

static void test_imu(void)
{
    printf("hil imu math\n");
    ariadna_hil_in_t in;
    ariadna_hil_imu_out_t out;

    // Case 1: Standing upright on pad (board -Y = nose = +z_pad, +Y = thrust = -z_pad).
    // Rotation by -90 deg about X: q = (cos(-45), sin(-45), 0, 0)
    memset(&in, 0, sizeof(in));
    const float s45 = 0.70710678f;
    in.qw = s45; in.qx = -s45; in.qy = 0.0f; in.qz = 0.0f;
    in.acc_body_ms2[0] = 0.0f;
    in.acc_body_ms2[1] = -9.80665f;   // gravity felt along -Y (up)
    in.acc_body_ms2[2] = 0.0f;
    check(ariadna_hil_apply_imu(&in, &out), "pad upright valid");

    // Thrust axis body +Y -> R * (0,1,0) = col 1 = (0, 0, -1)
    check(nearf(out.R[0][1], 0.0f) && nearf(out.R[1][1], 0.0f) && nearf(out.R[2][1], -1.0f),
          "pad upright u = (0,0,-1)");
    check(nearf(out.a_pad[0], 0.0f) && nearf(out.a_pad[1], 0.0f) && nearf(out.a_pad[2], 0.0f),
          "pad upright a_pad = (0,0,0)");
    check(nearf(out.w_abs, 0.0f), "pad upright w_abs = 0");

    // Case 2: Nose-first descent (+Y = thrust = +z_pad).
    // Rotation by +90 deg about X: q = (cos(+45), sin(+45), 0, 0)
    // Powered landing burn with +20 m/s^2 upward acceleration (specific force = +29.80665 along +Y).
    in.qw = s45; in.qx = s45; in.qy = 0.0f; in.qz = 0.0f;
    in.acc_body_ms2[0] = 0.0f;
    in.acc_body_ms2[1] = 29.80665f;
    in.acc_body_ms2[2] = 0.0f;
    in.gyr_body_rads[0] = 0.5f;
    in.gyr_body_rads[1] = 0.0f;
    in.gyr_body_rads[2] = 0.0f;
    check(ariadna_hil_apply_imu(&in, &out), "descent burn valid");
    check(nearf(out.R[0][1], 0.0f) && nearf(out.R[1][1], 0.0f) && nearf(out.R[2][1], 1.0f),
          "descent u = (0,0,+1)");
    check(nearf(out.a_pad[0], 0.0f) && nearf(out.a_pad[1], 0.0f) && nearf(out.a_pad[2], 20.0f),
          "descent burn a_pad_z = +20 m/s^2");
    check(nearf(out.w_pad[0], 0.5f) && nearf(out.w_pad[1], 0.0f) && nearf(out.w_pad[2], 0.0f),
          "gyro rotated correctly");
    check(nearf(out.w_abs, 0.5f * 57.2957795f, 1e-2f), "w_abs in deg/s");

    // Case 3: Reject NaN and zero quaternion
    in.qw = 0.0f; in.qx = 0.0f; in.qy = 0.0f; in.qz = 0.0f;
    check(!ariadna_hil_apply_imu(&in, &out), "zero quaternion rejected");
    in.qw = 1.0f;
    in.acc_body_ms2[0] = std::numeric_limits<float>::quiet_NaN();
    check(!ariadna_hil_apply_imu(&in, &out), "NaN accel rejected");
}

int main(void)
{
    test_layout();
    test_parser();
    test_imu();
    printf("\nhil_test: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
