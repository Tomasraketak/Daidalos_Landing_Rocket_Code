// =====================================================================
//  Ariadna V2 — POSITION MCU bench test
//
//  Target: Seeed XIAO RP2350, arduino-pico core.
//  Needs:  Adafruit_NeoPixel. Nothing for UWB: the tag is a separate
//          board (../../Uwb_Tag) read over I2C.
//
//  Every sensor is talked to by its raw registers, not through TriSense,
//  so each one passes or fails ON ITS OWN and a dead chip cannot take the
//  others down with it:
//
//    I2C bus     idle levels, scan                         D4/D5 (i2c1)
//    ICM42688P   WHO_AM_I, accel/gyro/temp, |a| ~ 1 g      SPI, CS D3
//    BMP580      CHIP_ID, pressure + temperature, noise    I2C
//    AK09918C    WIA, field strength                       I2C 0x0C
//    TF Luna     distance / signal (optional sensor)       I2C 0x10
//    LK76 GPS    NMEA at 9600 or 115200 (auto), fix, sats  PIO UART D11/D12
//    UWB tag     frame from the tag board, ranges to the   I2C 0x2A
//                4 anchors (flash ../../Uwb_Tag on it)
//    RGB LED, core 1, link to the Master, link to Navigation (via Master)
//
//  NOTHING BLOCKS. I2C has a 10 ms timeout, every sensor that fails is
//  re-probed every few seconds (so it can be plugged in live), and the GPS
//  is serviced a few bytes at a time.
//
//  The IMU's SPI pins are released between transfers: the module drives
//  SCK and MOSI itself whenever the IMU is not selected, so the MCU must
//  not. In flight that is the TriSense library's job.
//
//  USB serial, 115200 baud. Commands: h help, p summary, i I2C scan,
//  g raw GPS echo on/off.
// =====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "bench_common.h"

#include "ariadna_uwb_link.h"

// =====================================================================
//  1. PINS — identical to Position_MCU/Position_MCU.ino
// =====================================================================
// D0, D1, D2 are free: the DWM1000 moved to its own board on I2C.
static const uint8_t PIN_IMU_CS      = D3;   // TriSense ICM42688P
static const uint8_t PIN_I2C_SDA     = D4;
static const uint8_t PIN_I2C_SCL     = D5;
static const uint8_t PIN_MASTER_TX   = D6;
static const uint8_t PIN_MASTER_RX   = D7;
static const uint8_t PIN_SPI_SCK     = D8;
static const uint8_t PIN_SPI_MISO    = D9;
static const uint8_t PIN_SPI_MOSI    = D10;
static const uint8_t PIN_GPS_TX      = D11;  // to the module's RX
static const uint8_t PIN_GPS_RX      = D12;  // from the module's TX
static const uint8_t PIN_RGB         = 22;   // XIAO RP2350 on-board WS2812
static const uint8_t PIN_RGB_POWER   = 23;

// D4/D5 are GPIO6/7, which only the SECOND I2C block can reach — so this
// is Wire1, not Wire. Picked from the pin number, so it stays right if the
// pins are ever moved.
static TwoWire &I2C = (((PIN_I2C_SDA >> 1) & 1) ? Wire1 : Wire);

// SPI0 belongs to the IMU alone.
static SPISettings s_imu_spi(10000000, MSBFIRST, SPI_MODE0);   // flight speed
static SPISettings s_imu_spi_slow(1000000, MSBFIRST, SPI_MODE0);

static const uint8_t ADDR_BMP_A  = 0x47;
static const uint8_t ADDR_BMP_B  = 0x46;
static const uint8_t ADDR_MAG    = 0x0C;
static const uint8_t ADDR_LIDAR  = 0x10;

static const uint32_t GPS_BAUDS[] = { 9600, 115200, 38400, 57600 };

// The anchors' short addresses are 0x0001..0x0004 (Uwb_Anchor sketch,
// ANCHOR_ID 1..4); the tag board reports ranges by those addresses.

// =====================================================================
//  2. STATE
// =====================================================================
static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static SerialPIO         GpsPort(PIN_GPS_TX, PIN_GPS_RX, 256);
static bench_port_t      s_master;
static bench_looptime_t  s_loop;

// =====================================================================
//  3. I2C helpers
// =====================================================================
static bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val)
{
    I2C.beginTransmission(addr);
    I2C.write(reg);
    I2C.write(val);
    return I2C.endTransmission() == 0;
}

static bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n)
{
    I2C.beginTransmission(addr);
    I2C.write(reg);
    if (I2C.endTransmission(false) != 0) return false;
    if (I2C.requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)I2C.read();
    return true;
}

static bool i2c_ping(uint8_t addr)
{
    I2C.beginTransmission(addr);
    return I2C.endTransmission() == 0;
}

static const char *i2c_name(uint8_t a)
{
    switch (a) {
    case ADDR_BMP_A: case ADDR_BMP_B: return "BMP580";
    case ADDR_MAG:   return "AK09918C";
    case ADDR_LIDAR: return "TF-Luna";
    case 0x68: case 0x69: return "ICM42688(CS high)";
    case ARIADNA_UWB_I2C_ADDR: return "UWB tag";
    default: return "?";
    }
}

static void i2c_scan(bool verdict)
{
    uint8_t found = 0;
    bench_printf("# I2C scan (%s on D4/D5):", &I2C == &Wire1 ? "Wire1/i2c1" : "Wire/i2c0");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_ping(a)) { bench_printf(" 0x%02X(%s)", a, i2c_name(a)); found++; }
    }
    bench_printf(found ? "\n" : " nothing\n");
    if (verdict) {
        if (found) bench_set(PT_I2C_BUS, ST_PASS, "%u device(s) answer", found);
        else       bench_set(PT_I2C_BUS, ST_FAIL, "no device answers - SDA/SCL, pull-ups, 3V3?");
    }
}

// =====================================================================
//  4. ICM42688P (TriSense IMU) — SPI
// =====================================================================
static bool     s_imu_ok = false;
static bool     s_imu_slow = false;
static float    s_ax, s_ay, s_az, s_gx, s_gy, s_gz, s_imu_t;
static uint32_t s_imu_reads = 0, s_imu_fail = 0, s_imu_stuck = 0;

// The TriSense level shifter on SCK/MOSI points AT the MCU while CS is
// HIGH, so the MCU drives those pins only inside a transfer: CS low first
// (the shifter turns round), then the pins; pins released first, then CS
// high. Otherwise two push-pull drivers fight on each line, ~15-25 mA.
static void spi_pins_release(void)
{
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_NULL);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_NULL);
}

static void imu_rw(uint8_t reg, uint8_t *buf, uint8_t n, bool read)
{
    SPI.beginTransaction(s_imu_slow ? s_imu_spi_slow : s_imu_spi);
    digitalWrite(PIN_IMU_CS, LOW);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    SPI.transfer(read ? (uint8_t)(reg | 0x80) : reg);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t b = SPI.transfer(read ? 0x00 : buf[i]);
        if (read) buf[i] = b;
    }
    spi_pins_release();
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}
static uint8_t imu_r8(uint8_t reg) { uint8_t v = 0; imu_rw(reg, &v, 1, true); return v; }
static void    imu_w8(uint8_t reg, uint8_t v) { imu_rw(reg, &v, 1, false); }

static bool imu_begin(void)
{
    s_imu_slow = false;
    imu_w8(0x76, 0x00);                 // bank 0
    uint8_t who = imu_r8(0x75);
    if (who != 0x47) {
        // Try again at 1 MHz: works slow but not fast = signal integrity.
        s_imu_slow = true;
        uint8_t who_slow = imu_r8(0x75);
        if (who_slow == 0x47) {
            bench_set(PT_IMU_ID, ST_WARN, "WHO_AM_I OK only at 1 MHz (10 MHz gave 0x%02X) - wiring/length", who);
        } else {
            bench_set(PT_IMU_ID, ST_FAIL, "WHO_AM_I 0x%02X, want 0x47 - CS D3, SPI D8/D9/D10, power", who_slow);
            return false;
        }
    } else {
        bench_set(PT_IMU_ID, ST_PASS, "ICM42688P WHO_AM_I 0x47 at 10 MHz");
    }
    imu_w8(0x11, 0x01);                 // DEVICE_CONFIG: soft reset
    delay(2);
    imu_w8(0x76, 0x00);
    imu_w8(0x4F, 0x06);                 // GYRO_CONFIG0: 2000 dps, 1 kHz
    imu_w8(0x50, 0x06);                 // ACCEL_CONFIG0: 16 g, 1 kHz
    imu_w8(0x4E, 0x0F);                 // PWR_MGMT0: gyro + accel low-noise
    delay(1);
    if (imu_r8(0x4E) != 0x0F) {
        bench_set(PT_IMU_DATA, ST_FAIL, "PWR_MGMT0 did not stick - writes fail (MOSI?)");
        return false;
    }
    return true;
}

static void imu_service(void)
{
    static uint32_t next_ms = 0, retry_ms = 0;
    static int16_t  last_raw[6];
    static uint8_t  same = 0;
    static uint32_t ok_since = 0;

    if (!s_imu_ok) {
        if (millis() >= retry_ms) {
            retry_ms = millis() + 3000;
            s_imu_ok = imu_begin();
            ok_since = millis();
            if (!s_imu_ok) bench_set(PT_IMU_DATA, ST_FAIL, "no IMU");
        }
        return;
    }
    if (millis() < next_ms) return;
    next_ms = millis() + 20;                               // 50 Hz

    uint8_t b[14];
    imu_rw(0x1D, b, 14, true);                             // TEMP, ACCEL, GYRO
    int16_t raw[7];
    for (uint8_t i = 0; i < 7; i++) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);

    // All-0x00 or all-0xFF is a bus that is not there any more.
    bool all00 = true, allFF = true;
    for (uint8_t i = 0; i < 14; i++) { if (b[i]) all00 = false; if (b[i] != 0xFF) allFF = false; }
    if (all00 || allFF) {
        s_imu_fail++;
        if (s_imu_fail % 25 == 0 && imu_r8(0x75) != 0x47) {
            s_imu_ok = false;
            bench_set(PT_IMU_ID, ST_FAIL, "IMU stopped answering");
        }
        return;
    }
    s_imu_reads++;
    s_imu_t = (float)raw[0] / 132.48f + 25.0f;
    s_ax = (float)raw[1] / 2048.0f; s_ay = (float)raw[2] / 2048.0f; s_az = (float)raw[3] / 2048.0f;
    s_gx = (float)raw[4] / 16.4f;   s_gy = (float)raw[5] / 16.4f;   s_gz = (float)raw[6] / 16.4f;

    if (memcmp(last_raw, &raw[1], sizeof(last_raw)) == 0) { if (same < 255) same++; }
    else same = 0;
    memcpy(last_raw, &raw[1], sizeof(last_raw));
    if (same > 20) s_imu_stuck++;

    if (millis() - ok_since < 300) return;                 // gyro start-up
    const float a = sqrtf(s_ax * s_ax + s_ay * s_ay + s_az * s_az);
    const float w = sqrtf(s_gx * s_gx + s_gy * s_gy + s_gz * s_gz);
    if (same > 20)
        bench_set(PT_IMU_DATA, ST_FAIL, "data frozen - same sample %u times", same);
    else if (a < 0.85f || a > 1.15f)
        bench_set(PT_IMU_DATA, ST_WARN, "|a| = %.2f g (want ~1 g at rest) - moving?", (double)a);
    else if (w > 10.0f)
        bench_set(PT_IMU_DATA, ST_WARN, "|w| = %.1f dps at rest? (board moving or bias)", (double)w);
    else
        bench_set(PT_IMU_DATA, ST_PASS, "|a| %.3f g  |w| %.2f dps  %.1f C", (double)a, (double)w, (double)s_imu_t);
}

// =====================================================================
//  5. BMP580 — I2C
// =====================================================================
static uint8_t  s_bmp_addr = 0;
static float    s_press = 0, s_btemp = 0;
static double   s_p_sum = 0, s_p_sum2 = 0;
static uint32_t s_p_n = 0, s_bmp_fail = 0;

static bool bmp_begin(void)
{
    uint8_t id = 0;
    s_bmp_addr = 0;
    if (i2c_read(ADDR_BMP_A, 0x01, &id, 1)) s_bmp_addr = ADDR_BMP_A;
    else if (i2c_read(ADDR_BMP_B, 0x01, &id, 1)) s_bmp_addr = ADDR_BMP_B;
    if (!s_bmp_addr) {
        bench_set(PT_BARO_ID, ST_FAIL, "no ACK at 0x47 or 0x46");
        return false;
    }
    if (id != 0x50 && id != 0x51) {
        bench_set(PT_BARO_ID, ST_FAIL, "0x%02X answers but CHIP_ID 0x%02X (want 0x50)", s_bmp_addr, id);
        s_bmp_addr = 0;
        return false;
    }
    i2c_write8(s_bmp_addr, 0x7E, 0xB6);                  // soft reset
    delay(3);
    bool ok = i2c_write8(s_bmp_addr, 0x36, 0x58)          // press on, OSR_P x8, OSR_T x1
           && i2c_write8(s_bmp_addr, 0x37, 0xBD);         // 50 Hz, normal mode
    uint8_t odr = 0;
    ok = ok && i2c_read(s_bmp_addr, 0x37, &odr, 1) && (odr & 0x7F) == 0x3D;
    if (!ok) {
        bench_set(PT_BARO_ID, ST_FAIL, "BMP58%c at 0x%02X, config write failed", id == 0x50 ? '0' : '1', s_bmp_addr);
        s_bmp_addr = 0;
        return false;
    }
    bench_set(PT_BARO_ID, ST_PASS, "BMP58%c at 0x%02X", id == 0x50 ? '0' : '1', s_bmp_addr);
    return true;
}

static void bmp_service(void)
{
    static uint32_t next_ms = 0, retry_ms = 0;
    if (!s_bmp_addr) {
        if (millis() >= retry_ms) {
            retry_ms = millis() + 3000;
            if (!bmp_begin()) bench_set(PT_BARO_DATA, ST_FAIL, "no barometer");
        }
        return;
    }
    if (millis() < next_ms) return;
    next_ms = millis() + 40;                               // 25 Hz

    uint8_t b[6];
    if (!i2c_read(s_bmp_addr, 0x1D, b, 6)) {
        if (++s_bmp_fail > 10) { s_bmp_addr = 0; s_bmp_fail = 0; bench_set(PT_BARO_ID, ST_FAIL, "stopped answering"); }
        return;
    }
    s_bmp_fail = 0;
    int32_t t = (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16));
    if (t & 0x800000) t |= (int32_t)0xFF000000;
    uint32_t p = (uint32_t)b[3] | ((uint32_t)b[4] << 8) | ((uint32_t)b[5] << 16);
    s_btemp = (float)t / 65536.0f;
    s_press = (float)p / 64.0f;
    s_p_sum += s_press; s_p_sum2 += (double)s_press * s_press; s_p_n++;

    if (p == 0 || s_press < 30000.0f || s_press > 110000.0f)
        bench_set(PT_BARO_DATA, ST_FAIL, "pressure %.0f Pa implausible", (double)s_press);
    else if (s_btemp < -20.0f || s_btemp > 70.0f)
        bench_set(PT_BARO_DATA, ST_WARN, "temperature %.1f C implausible", (double)s_btemp);
    else
        bench_set(PT_BARO_DATA, ST_PASS, "%.1f Pa  %.2f C", (double)s_press, (double)s_btemp);
}

// =====================================================================
//  6. AK09918C magnetometer — I2C 0x0C
// =====================================================================
static bool     s_mag_ok = false;
static float    s_mx, s_my, s_mz;
static uint32_t s_mag_fail = 0;

static bool mag_begin(void)
{
    uint8_t wia[2];
    if (!i2c_read(ADDR_MAG, 0x00, wia, 2)) {
        bench_set(PT_MAG_ID, ST_FAIL, "no ACK at 0x0C");
        return false;
    }
    if (wia[0] != 0x48 || wia[1] != 0x0C) {
        bench_set(PT_MAG_ID, ST_FAIL, "WIA %02X %02X, want 48 0C", wia[0], wia[1]);
        return false;
    }
    i2c_write8(ADDR_MAG, 0x32, 0x01);                     // soft reset
    delay(1);
    if (!i2c_write8(ADDR_MAG, 0x31, 0x08)) {              // continuous 100 Hz
        bench_set(PT_MAG_ID, ST_FAIL, "mode write failed");
        return false;
    }
    bench_set(PT_MAG_ID, ST_PASS, "AK09918C WIA 48 0C");
    return true;
}

static void mag_service(void)
{
    static uint32_t next_ms = 0, retry_ms = 0, last_drdy = 0;
    if (!s_mag_ok) {
        if (millis() >= retry_ms) {
            retry_ms = millis() + 3000;
            s_mag_ok = mag_begin();
            last_drdy = millis();
            if (!s_mag_ok) bench_set(PT_MAG_DATA, ST_FAIL, "no magnetometer");
        }
        return;
    }
    if (millis() < next_ms) return;
    next_ms = millis() + 50;                               // 20 Hz

    uint8_t st1 = 0;
    if (!i2c_read(ADDR_MAG, 0x10, &st1, 1)) {
        if (++s_mag_fail > 10) { s_mag_ok = false; s_mag_fail = 0; bench_set(PT_MAG_ID, ST_FAIL, "stopped answering"); }
        return;
    }
    s_mag_fail = 0;
    if (!(st1 & 0x01)) {
        if (millis() - last_drdy > 1000) bench_set(PT_MAG_DATA, ST_FAIL, "no new data for 1 s (DRDY never set)");
        return;
    }
    uint8_t b[8];                                          // HXL..ST2: reading ST2 ends the cycle
    if (!i2c_read(ADDR_MAG, 0x11, b, 8)) return;
    last_drdy = millis();
    s_mx = (float)(int16_t)(b[0] | (b[1] << 8)) * 0.15f;
    s_my = (float)(int16_t)(b[2] | (b[3] << 8)) * 0.15f;
    s_mz = (float)(int16_t)(b[4] | (b[5] << 8)) * 0.15f;
    const float B = sqrtf(s_mx * s_mx + s_my * s_my + s_mz * s_mz);
    if (b[7] & 0x08)
        bench_set(PT_MAG_DATA, ST_WARN, "magnetic overflow - magnet / steel nearby?");
    else if (B < 15.0f || B > 100.0f)
        bench_set(PT_MAG_DATA, ST_WARN, "|B| = %.1f uT (Earth ~50) - disturbance nearby?", (double)B);
    else
        bench_set(PT_MAG_DATA, ST_PASS, "|B| %.1f uT", (double)B);
}

// =====================================================================
//  7. TF Luna lidar — I2C 0x10 (fitted for the future; flight code ignores it)
// =====================================================================
static uint16_t s_lidar_cm = 0, s_lidar_amp = 0;

static void lidar_service(void)
{
    static uint32_t next_ms = 0;
    if (millis() < next_ms) return;
    next_ms = millis() + 100;                              // 10 Hz
    uint8_t b[6];
    if (!i2c_read(ADDR_LIDAR, 0x00, b, 6)) {
        bench_set(PT_LIDAR, ST_INFO, "not found at 0x10 (optional; pin 5 to GND for I2C mode)");
        next_ms = millis() + 3000;
        return;
    }
    s_lidar_cm  = (uint16_t)(b[0] | (b[1] << 8));
    s_lidar_amp = (uint16_t)(b[2] | (b[3] << 8));
    const float t = (float)(int16_t)(b[4] | (b[5] << 8)) * 0.01f;
    if (s_lidar_amp < 100 || s_lidar_amp == 0xFFFF)
        bench_set(PT_LIDAR, ST_WARN, "%u cm but signal %u too weak/saturated - aim at a surface",
                  s_lidar_cm, s_lidar_amp);
    else
        bench_set(PT_LIDAR, ST_PASS, "%u cm, signal %u, %.1f C", s_lidar_cm, s_lidar_amp, (double)t);
}

// =====================================================================
//  8. GPS — NMEA, auto-baud, a few bytes per loop
// =====================================================================
static uint8_t  s_gps_baud_i = 0;
static uint32_t s_gps_baud_t0 = 0;
static bool     s_gps_locked = false;
static uint32_t s_gps_bytes = 0, s_gps_good = 0, s_gps_bad = 0, s_gps_last_ms = 0;
static char     s_nmea[100];
static uint8_t  s_nmea_n = 0;
static int      s_gps_fixq = 0, s_gps_sats = 0;
static float    s_gps_hdop = 99.9f, s_gps_lat = 0, s_gps_lon = 0, s_gps_alt = 0;
static bool     s_gps_echo = false;

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// ddmm.mmmm + hemisphere -> signed degrees
static float nmea_deg(const char *f, char hemi)
{
    if (!*f) return 0.0f;
    float v = (float)atof(f);
    int deg = (int)(v / 100.0f);
    float d = (float)deg + (v - (float)deg * 100.0f) / 60.0f;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
}

static void nmea_sentence(char *s)
{
    // $....*CS
    char *star = strchr(s, '*');
    if (s[0] != '$' || !star || hexval(star[1]) < 0 || hexval(star[2]) < 0) { s_gps_bad++; return; }
    uint8_t cs = 0;
    for (char *p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
    if (cs != (uint8_t)(hexval(star[1]) * 16 + hexval(star[2]))) { s_gps_bad++; return; }
    s_gps_good++;
    s_gps_last_ms = millis();
    *star = 0;
    if (s_gps_echo) bench_printf("# %s\n", s);

    // Split in place. Empty fields are kept, so indices are stable.
    char *f[20];
    uint8_t n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < 20; p++) if (*p == ',') { *p = 0; f[n++] = p + 1; }

    if (strlen(f[0]) == 6 && strcmp(f[0] + 3, "GGA") == 0 && n >= 10) {
        s_gps_fixq = atoi(f[6]);
        s_gps_sats = atoi(f[7]);
        s_gps_hdop = f[8][0] ? (float)atof(f[8]) : 99.9f;
        s_gps_alt  = (float)atof(f[9]);
        if (s_gps_fixq > 0) {
            s_gps_lat = nmea_deg(f[2], f[3][0]);
            s_gps_lon = nmea_deg(f[4], f[5][0]);
        }
    }
}

static void gps_service(void)
{
    // Auto-baud: 3 s per rate until a valid sentence arrives, then stay.
    if (!s_gps_locked && millis() - s_gps_baud_t0 > 3000) {
        s_gps_baud_i = (uint8_t)((s_gps_baud_i + 1) % (sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0])));
        GpsPort.end();
        GpsPort.begin(GPS_BAUDS[s_gps_baud_i]);
        s_gps_baud_t0 = millis();
        s_nmea_n = 0;
    }

    uint16_t budget = 128;
    while (budget-- && GpsPort.available()) {
        char c = (char)GpsPort.read();
        s_gps_bytes++;
        if (c == '$') s_nmea_n = 0;
        if (c == '\r' || c == '\n') {
            if (s_nmea_n > 6) {
                s_nmea[s_nmea_n] = 0;
                uint32_t good = s_gps_good;
                nmea_sentence(s_nmea);
                if (s_gps_good != good && !s_gps_locked) {
                    s_gps_locked = true;
                    bench_printf("[%8.2f s] EVENT GPS talks NMEA at %lu baud\n",
                                 (double)bench_t_s(), (unsigned long)GPS_BAUDS[s_gps_baud_i]);
                }
            }
            s_nmea_n = 0;
            continue;
        }
        if (s_nmea_n < sizeof(s_nmea) - 1) s_nmea[s_nmea_n++] = c;
        else s_nmea_n = 0;
    }

    static uint32_t next_ms = 0;
    if (millis() < next_ms) return;
    next_ms = millis() + 500;
    if (s_gps_locked && millis() - s_gps_last_ms > 3000)
        bench_set(PT_GPS_NMEA, ST_FAIL, "NMEA stopped %lu ms ago - module lost power/wire?",
                  (unsigned long)(millis() - s_gps_last_ms));
    else if (s_gps_locked)
        bench_set(PT_GPS_NMEA, s_gps_bad > s_gps_good / 10 + 2 ? ST_WARN : ST_PASS,
                  "%lu baud, %lu good / %lu bad sentences", (unsigned long)GPS_BAUDS[s_gps_baud_i],
                  (unsigned long)s_gps_good, (unsigned long)s_gps_bad);
    else if (millis() < 13000)
        bench_set(PT_GPS_NMEA, ST_PEND, "searching baud rate (%lu B so far)", (unsigned long)s_gps_bytes);
    else if (s_gps_bytes == 0)
        bench_set(PT_GPS_NMEA, ST_FAIL, "no bytes at all - power, or swap D11/D12");
    else
        bench_set(PT_GPS_NMEA, ST_FAIL, "%lu bytes but no valid NMEA at any baud", (unsigned long)s_gps_bytes);

    if (!s_gps_locked)
        bench_set(PT_GPS_FIX, ST_PEND, "no NMEA yet");
    else if (s_gps_fixq > 0)
        bench_set(PT_GPS_FIX, ST_PASS, "fix q%d, %d sats, hdop %.1f, %.6f %.6f, %.0f m",
                  s_gps_fixq, s_gps_sats, (double)s_gps_hdop, (double)s_gps_lat,
                  (double)s_gps_lon, (double)s_gps_alt);
    else
        bench_set(PT_GPS_FIX, ST_INFO, "no fix yet, %d sats - needs open sky", s_gps_sats);
}

// =====================================================================
//  9. UWB TAG — the separate tag board, read over I2C
//
//  The tag (an anchor PCB flashed with Uwb_Tag) sits on this bus at
//  ARIADNA_UWB_I2C_ADDR and answers every read with a 74-byte frame: the
//  latest range to each anchor it hears. UWB_TAG checks the link and the
//  tag's radio, UWB_RANGE the ranges themselves.
// =====================================================================
static ariadna_uwb_link_frame_t s_tag;
static bool     s_tag_seen = false;
static uint32_t s_tag_ok_ms = 0, s_tag_reads = 0, s_tag_bad = 0, s_tag_nack = 0;
static uint16_t s_tag_seq_prev = 0;
static uint32_t s_tag_seq_stuck = 0;
static float    s_uwb_range[5];      // index = anchor short address 1..4
static float    s_uwb_rxp[5];
static uint32_t s_uwb_ms[5], s_uwb_count[5];
static uint8_t  s_uwb_last_count[5];

/** One frame read. Returns the decoder's verdict, or 1 for no answer. */
static int tag_read(void)
{
    uint8_t buf[ARIADNA_UWB_LINK_FRAME_SIZE];
    const size_t got = I2C.requestFrom((uint8_t)ARIADNA_UWB_I2C_ADDR, sizeof(buf));
    size_t n = 0;
    while (I2C.available()) {
        const int c = I2C.read();
        if (n < sizeof(buf)) buf[n++] = (uint8_t)c;
    }
    const uint32_t t_done = millis();
    if (got == 0) { s_tag_nack++; return 1; }
    s_tag_reads++;

    ariadna_uwb_link_frame_t f;
    const int r = ariadna_uwb_link_decode(buf, n, &f);
    if (r != ARIADNA_UWB_LINK_OK) { s_tag_bad++; return r; }

    // A frame counter that never moves means the tag answers from a
    // buffer it stopped refreshing - a hung core 0 on the tag.
    if (s_tag_seen && f.seq == s_tag_seq_prev) s_tag_seq_stuck++;
    s_tag_seq_prev = f.seq;
    s_tag = f;
    s_tag_seen = true;
    s_tag_ok_ms = t_done;

    for (uint8_t i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
        const ariadna_uwb_link_slot_t &s = f.slot[i];
        if (!s.valid) continue;
        if (s.addr >= 1 && s.addr <= 4) {
            const uint16_t a = s.addr;
            if (s_uwb_count[a] == 0 || s.count != s_uwb_last_count[a]) {
                s_uwb_count[a]++;
                s_uwb_last_count[a] = s.count;
            }
            s_uwb_range[a] = s.range_m;
            s_uwb_rxp[a]   = (float)s.rxp_dbm;
            s_uwb_ms[a]    = t_done - s.age_ms;
        } else {
            static uint16_t reported = 0;
            if (reported != s.addr) {
                reported = s.addr;
                bench_printf("[%8.2f s] EVENT UWB range from unknown anchor 0x%04X - "
                             "ANCHOR_ID outside 1..4?\n", (double)bench_t_s(), s.addr);
            }
        }
    }
    return r;
}

static void uwb_service(void)
{
    static uint32_t next_read = 0, next_verdict = 0;
    static int last = 1;
    // 10 Hz: enough to see every range at the tag's ~10 Hz per anchor.
    if (millis() >= next_read) {
        next_read = millis() + 100;
        last = tag_read();
    }
    if (millis() < next_verdict) return;
    next_verdict = millis() + 500;

    // --- UWB_TAG: link and radio ------------------------------------------
    const bool fresh = s_tag_seen && millis() - s_tag_ok_ms < 1500;
    if (!s_tag_seen) {
        if (last == 1)
            bench_set(PT_UWB_ID, millis() < 5000 ? ST_PEND : ST_FAIL,
                      "no answer at 0x%02X - uSup cable to I2CP, tag powered, flashed with Uwb_Tag?",
                      ARIADNA_UWB_I2C_ADDR);
        else
            bench_set(PT_UWB_ID, ST_FAIL, "answers at 0x%02X but frame bad (%s) - Uwb_Tag version?",
                      ARIADNA_UWB_I2C_ADDR,
                      last == ARIADNA_UWB_LINK_E_VERSION ? "protocol version" :
                      last == ARIADNA_UWB_LINK_E_CRC ? "CRC" : "short/magic");
    } else if (!fresh) {
        bench_set(PT_UWB_ID, ST_FAIL, "tag stopped answering %lu ms ago - cable or tag reset?",
                  (unsigned long)(millis() - s_tag_ok_ms));
    } else if (!(s_tag.flags & ARIADNA_UWB_LF_RADIO_UP)) {
        bench_set(PT_UWB_ID, ST_FAIL, "tag answers, but its DWM1000 is not up - tag's SPI/3V3/RST");
    } else if (s_tag_bad > s_tag_reads / 20 + 1 || s_tag_seq_stuck > 2) {
        bench_set(PT_UWB_ID, ST_WARN, "radio up, %lu frames, %lu bad, %lu stale - pull-ups/cable length?",
                  (unsigned long)s_tag_reads, (unsigned long)s_tag_bad,
                  (unsigned long)s_tag_seq_stuck);
    } else {
        bench_set(PT_UWB_ID, ST_PASS, "tag 0x%02X radio up, %lu frames, %lu bad",
                  ARIADNA_UWB_I2C_ADDR, (unsigned long)s_tag_reads, (unsigned long)s_tag_bad);
    }

    // --- UWB_RANGE: what the anchors say ----------------------------------
    if (!fresh || !(s_tag.flags & ARIADNA_UWB_LF_RADIO_UP)) {
        bench_set(PT_UWB_RANGE, ST_FAIL, "no tag");
        return;
    }
    char buf[72];
    int n = 0;
    uint8_t live = 0;
    for (uint8_t a = 1; a <= 4; a++) {
        if (s_uwb_count[a] && millis() - s_uwb_ms[a] < 2000) {
            live++;
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "A%u %.2fm ", a, (double)s_uwb_range[a]);
        }
    }
    if (live)
        bench_set(PT_UWB_RANGE, live >= 3 ? ST_PASS : ST_WARN, "%u/4 anchors: %s", live, buf);
    else if (millis() < 10000)
        bench_set(PT_UWB_RANGE, ST_PEND, "listening for anchors...");
    else
        bench_set(PT_UWB_RANGE, ST_FAIL, "no anchor ranges - anchors on? flashed with ANCHOR_ID 1..4?");
}

// =====================================================================
//  10. LINK — ping Master and Navigation, report to the Master
// =====================================================================
static void on_rx(bench_port_t *p, uint8_t type, const uint8_t *pl, uint8_t len)
{
    bench_ping_t reply;
    if (bench_handle(type, pl, len, &reply)) bench_send(p, PKT_PONG, &reply, sizeof(reply));
}

static void link_service(void)
{
    static uint32_t next_ping = 0, next_report = 0;
    if (millis() >= next_ping) {
        next_ping = millis() + 100;
        bench_ping_t pk;
        bench_ping_build(&pk, NODE_MASTER);     bench_send(&s_master, PKT_PING, &pk, sizeof(pk));
        bench_ping_build(&pk, NODE_NAVIGATION); bench_send(&s_master, PKT_PING, &pk, sizeof(pk));
    }
    if (millis() >= next_report) {
        next_report = millis() + 1000;
        bench_report_t r;
        bench_report_build(&r, NODE_MASTER, s_loop.max_ever_us);
        bench_send(&s_master, PKT_REPORT, &r, sizeof(r));
    }
}

// =====================================================================
//  11. RGB, SUMMARY, CONSOLE
// =====================================================================
static void rgb_service(void)
{
    static uint32_t last = 0xFFFFFFFF;
    const uint32_t t = millis();
    uint8_t r = 0, g = 0, b = 0;
    if (t < 2400) {
        static const uint8_t seq[4][3] = { {60,0,0}, {0,60,0}, {0,0,60}, {40,40,40} };
        r = seq[(t / 600) % 4][0]; g = seq[(t / 600) % 4][1]; b = seq[(t / 600) % 4][2];
    } else {
        const uint8_t w = bench_worst();
        const bool on = (t % (w == ST_FAIL ? 200 : 1000)) < (w == ST_FAIL ? 100 : 500);
        if (on) { if (w == ST_FAIL) r = 60; else if (w == ST_PASS) g = 60; else { r = 60; g = 25; } }
    }
    uint32_t c = s_px.Color(r, g, b);
    if (c != last) { last = c; s_px.setPixelColor(0, c); s_px.show(); }
    digitalWrite(LED_BUILTIN, (t / 250) & 1);
}

static void print_summary(void)
{
    bench_print_table();
    bench_printf("  IMU  a %+.3f %+.3f %+.3f g  w %+.2f %+.2f %+.2f dps  %.1f C  (%lu reads, %lu bad%s)\n",
                 (double)s_ax, (double)s_ay, (double)s_az, (double)s_gx, (double)s_gy, (double)s_gz,
                 (double)s_imu_t, (unsigned long)s_imu_reads, (unsigned long)s_imu_fail,
                 s_imu_slow ? ", SLOW SPI" : "");
    if (s_p_n > 1) {
        double mean = s_p_sum / s_p_n;
        double var  = s_p_sum2 / s_p_n - mean * mean;
        double sd   = var > 0 ? sqrt(var) : 0;
        bench_printf("  BARO %.1f Pa  %.2f C  noise %.2f Pa (~%.0f cm) over %lu samples\n",
                     mean, (double)s_btemp, sd, sd * 8.3, (unsigned long)s_p_n);
    }
    s_p_sum = s_p_sum2 = 0; s_p_n = 0;
    bench_printf("  MAG  %+.1f %+.1f %+.1f uT | LIDAR %u cm sig %u | GPS %lu B q%d sats %d\n",
                 (double)s_mx, (double)s_my, (double)s_mz, s_lidar_cm, s_lidar_amp,
                 (unsigned long)s_gps_bytes, s_gps_fixq, s_gps_sats);
    bench_printf("  UWB  %s", !s_tag_seen ? "no tag" :
                 (s_tag.flags & ARIADNA_UWB_LF_RADIO_UP) ? "tag up " : "tag radio DOWN ");
    for (uint8_t a = 1; a <= 4; a++)
        if (s_uwb_count[a]) bench_printf("A%u %.2f m %.0f dBm (%lu)  ", a, (double)s_uwb_range[a],
                                         (double)s_uwb_rxp[a], (unsigned long)s_uwb_count[a]);
    bench_printf("\n  loop max %lu us (worst %lu), die %.1f C\n", (unsigned long)s_loop.max_us,
                 (unsigned long)s_loop.max_ever_us, (double)analogReadTemp());
    bench_print_port(&s_master);
    bench_print_peer(NODE_MASTER);
    bench_print_peer(NODE_NAVIGATION);
    bench_printf("\n");
    bench_window_reset();
    s_loop.max_us = 0;
}

static void console_service(void)
{
    while (Serial.available()) {
        switch (Serial.read()) {
        case 'h': case '?': bench_printf("# h help | p summary | i I2C scan | g raw GPS echo\n"); break;
        case 'p': print_summary(); break;
        case 'i': i2c_scan(false); break;
        case 'g': s_gps_echo = !s_gps_echo; bench_printf("# GPS echo %s\n", s_gps_echo ? "ON" : "OFF"); break;
        default: break;
        }
    }
}

// =====================================================================
//  12. SETUP / LOOP
// =====================================================================
void setup()
{
    // IMU deselected before anything touches SPI.
    pinMode(PIN_IMU_CS, OUTPUT); digitalWrite(PIN_IMU_CS, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_RGB_POWER, OUTPUT); digitalWrite(PIN_RGB_POWER, HIGH);

    Serial.begin(BENCH_CONSOLE_BAUD);
    bench_tests_init(NODE_POSITION);
    s_px.begin(); s_px.clear(); s_px.show();

    // Idle I2C lines must read HIGH. LOW = shorted line or a hung slave.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(50);
    const int sda = digitalRead(PIN_I2C_SDA), scl = digitalRead(PIN_I2C_SCL);

    I2C.setSDA(PIN_I2C_SDA);
    I2C.setSCL(PIN_I2C_SCL);
    I2C.setTimeout(10, true);
    I2C.begin();
    I2C.setClock(400000);
    if (!sda || !scl) bench_set(PT_I2C_BUS, ST_FAIL, "idle %s%s LOW - short or stuck device",
                                !sda ? "SDA " : "", !scl ? "SCL" : "");
    else i2c_scan(true);

    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.begin();
    spi_pins_release();                  // held only inside IMU transfers

    s_imu_ok = imu_begin();
    bmp_begin();
    s_mag_ok = mag_begin();
    tag_read();

    GpsPort.begin(GPS_BAUDS[0]);
    s_gps_baud_t0 = millis();

    bench_port_begin(&s_master, Serial1, uart0, PIN_MASTER_TX, PIN_MASTER_RX, "UART0>MASTER");

    bench_printf("\n=== Ariadna V2 POSITION bench test, proto v%d, UWB tag at I2C 0x%02X ===\n",
                 BENCH_PROTO_VERSION, ARIADNA_UWB_I2C_ADDR);
}

void loop()
{
    static uint32_t next_summary = 3000, next_verdict = 0;
    bench_loop_begin(&s_loop);

    bench_port_poll(&s_master, on_rx);
    bench_port_service_tx(&s_master);
    link_service();

    imu_service();
    bmp_service();
    mag_service();
    lidar_service();
    gps_service();
    uwb_service();
    rgb_service();
    console_service();

    if (millis() >= next_verdict) {
        next_verdict = millis() + 250;
        bench_link_verdict(PT_LINK_MASTER, NODE_MASTER, &s_master);
        bench_link_verdict(PT_LINK_NAV, NODE_NAVIGATION, &s_master);
        bench_core1_verdict(PT_CORE1);
    }
    if (millis() >= next_summary) {
        next_summary = millis() + 5000;
        print_summary();
    }
    bench_loop_end(&s_loop);
}

void setup1() {}
void loop1()  { bench_core1_tick(); }
