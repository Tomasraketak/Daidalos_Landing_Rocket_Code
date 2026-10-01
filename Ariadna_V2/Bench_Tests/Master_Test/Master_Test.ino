// =====================================================================
//  Ariadna V2 — MASTER MCU bench test
//
//  Target: Raspberry Pi Pico 2 W (RP2350), arduino-pico core.
//  Needs:  LoRa (sandeepmistry), Adafruit_NeoPixel
//
//  Tests everything hanging off the Master and prints it on USB serial
//  (115200 baud):
//
//    PCA9685        found on I2C, prescale written and read back
//    SERVOS         TVC X, TVC Y and the thrust clamp stepped one at a
//                   time: trim -> +5 deg -> trim -> -5 deg -> trim
//    W25Q128        JEDEC ID, then erase / write / verify / erase of the
//                   LAST sector (the flight log never reaches it)
//    LoRa SX1278    init, then a telemetry-shaped packet every 2 s that
//                   the ground station decodes and shows (state = TEST)
//    ADC            battery voltage, ACS712 current, chip temperature
//    RBF, BREAKWIRE pin states; PASS once each has been seen in both states
//    RGB, BUZZER    colour cycle and a jingle at startup (look / listen)
//    AUX S16        toggles at 1 Hz (scope or LED on GP22)
//    CORE 1         heartbeat
//    LINKS          ping / pong with Position and Navigation, and routing
//                   of their pings to each other (Position <-> Navigation
//                   only exists THROUGH this board)
//    RESET LINES    after 15 s each subordinate is pulsed through its
//                   RESET line; PASS if it is seen to reboot
//
//  NOTHING BLOCKS. A missing chip is a FAIL line and a retry every few
//  seconds; the rest of the test keeps running. Flash Position_Test and
//  Navigation_Test to the other two boards for the link tests.
//
//  Console commands (type a letter + Enter):
//    h help   p summary now   s servo test on/off   b beep
//    l LoRa packet now        f rerun flash test    i I2C scan
//    r reset Position         n reset Navigation
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "bench_common.h"

// =====================================================================
//  1. CONFIGURATION — pins identical to Master_MCU/master_mcu.ino
// =====================================================================
#define SERVO_TEST        1       // step the servos (0 = only init the PCA9685)
#define FLASH_WRITE_TEST  1       // erase/write/verify the LAST flash sector
#define LORA_TX_TEST      1       // transmit a test packet every 2 s
#define RESET_LINE_TEST   1       // pulse the subordinates' RESET lines at 15 s
#define AUX_S16_TOGGLE    1       // square wave on GP22
#define BUZZER_TEST       1

static const uint8_t PIN_POS_TX     = 0;   // UART0 -> Position MCU
static const uint8_t PIN_POS_RX     = 1;
static const uint8_t PIN_RGB        = 2;   // SK6812
static const uint8_t PIN_BUZZER     = 3;
static const uint8_t PIN_NAV_TX     = 4;   // UART1 -> Navigation MCU
static const uint8_t PIN_NAV_RX     = 5;
static const uint8_t PIN_RST_POS    = 6;   // active LOW into Position RUN
static const uint8_t PIN_RST_NAV    = 7;   // active LOW into Navigation RUN
static const uint8_t PIN_I2C_SDA    = 8;   // I2C0 -> PCA9685
static const uint8_t PIN_I2C_SCL    = 9;
static const uint8_t PIN_BREAKWIRE  = 10;
static const uint8_t PIN_FLASH_MISO = 12;  // SPI1 -> W25Q128
static const uint8_t PIN_FLASH_CS   = 13;
static const uint8_t PIN_FLASH_SCK  = 14;
static const uint8_t PIN_FLASH_MOSI = 15;
static const uint8_t PIN_LORA_MISO  = 16;  // SPI0 -> SX1278
static const uint8_t PIN_LORA_CS    = 17;
static const uint8_t PIN_LORA_SCK   = 18;
static const uint8_t PIN_LORA_MOSI  = 19;
static const uint8_t PIN_LORA_DIO0  = 20;
static const uint8_t PIN_LORA_RST   = 21;
static const uint8_t PIN_AUX_S16    = 22;
static const uint8_t PIN_I_SENSE    = 26;  // ADC0, ACS712
static const uint8_t PIN_V_BAT      = 27;  // ADC1, 3:1 divider
static const uint8_t PIN_RBF        = 28;  // pull-up, LOW = tag fitted

static const float ADC_VREF       = 3.30f;
static const float VBAT_DIVIDER   = 3.00f;
static const float ACS712_ZERO_V  = 1.65f;
static const float ACS712_V_PER_A = 0.100f;

// ---- PCA9685 ----------------------------------------------------------
static const uint8_t PCA_ADDR   = 0x40;
static const float   PCA_OSC_HZ = 25000000.0f;
static const float   PCA_PWM_HZ = 339.0f;      // flight rate: DIGITAL servos only
enum { CH_TVC_X = 0, CH_TVC_Y = 1, CH_CLAMP = 2, PCA_N_USED = 8 };

static const float TVC_TRIM_X_US   = 1500.0f;
static const float TVC_TRIM_Y_US   = 1500.0f;
static const float TVC_US_PER_DEG  = 10.0f;
static const float SERVO_TEST_DEG  = 5.0f;     // TVC travel used by the test
static const float CLAMP_US_CLOSED = 1150.0f;
static const float CLAMP_US_OPEN   = 1900.0f;
static const float SERVO_US_MIN    = 900.0f;
static const float SERVO_US_MAX    = 2100.0f;
static const uint32_t SERVO_STEP_MS = 700;

// ---- LoRa: must match ariadna_link.h, or the ground station hears nothing
static const long     LORA_FREQ_HZ  = 433000000L;
static const int      LORA_SF       = 7;
static const long     LORA_BW_HZ    = 250000L;
static const int      LORA_CR4      = 5;
static const int      LORA_PREAMBLE = 8;
// 10 dBm, not the flight 20 dBm: enough across a workshop, and kinder to
// a module someone forgot to put an antenna on. NEVER transmit without one.
static const int      LORA_TEST_DBM = 10;
static const uint32_t LORA_PERIOD_MS = 2000;

// =====================================================================
//  2. STATE
// =====================================================================
static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

static bench_port_t     s_pos;          // UART0 <-> Position
static bench_port_t     s_nav;          // UART1 <-> Navigation
static bench_looptime_t s_loop;
static uint32_t         s_routed = 0, s_misrouted = 0, s_crosswired = 0;

// =====================================================================
//  3. RGB + BUZZER (non-blocking: a tiny step table advanced by millis)
// =====================================================================
static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    static uint32_t last = 0xFFFFFFFF;
    uint32_t c = s_px.Color(r, g, b);
    if (c == last) return;
    last = c;
    s_px.setPixelColor(0, c);
    s_px.show();
}

static void rgb_service(void)
{
    const uint32_t t = millis();
    // First 2.4 s: red, green, blue, white — check the colours are right.
    if (t < 2400) {
        static const uint8_t seq[4][3] = { {60,0,0}, {0,60,0}, {0,0,60}, {40,40,40} };
        const uint8_t *c = seq[(t / 600) % 4];
        rgb(c[0], c[1], c[2]);
        return;
    }
    // Then: slow green = all PASS, amber = warnings / pending, fast red = a FAIL.
    const uint8_t w = bench_worst();
    if (w == ST_FAIL)      rgb((t % 200) < 100 ? 60 : 0, 0, 0);
    else if (w == ST_PASS) rgb(0, (t % 1000) < 500 ? 60 : 0, 0);
    else                   rgb((t % 1000) < 500 ? 60 : 0, (t % 1000) < 500 ? 25 : 0, 0);
}

typedef struct { uint16_t hz, ms; } beep_t;
static const beep_t JINGLE[] = { {1500,120}, {0,60}, {2000,120}, {0,60}, {2700,180}, {0,0} };
static const beep_t *s_beep = nullptr;
static uint32_t      s_beep_t0 = 0;

static void beep_start(void)
{
#if BUZZER_TEST
    s_beep = JINGLE;
    s_beep_t0 = millis();
    if (s_beep->hz) tone(PIN_BUZZER, s_beep->hz);
    bench_printf("# buzzer: you should hear three rising beeps\n");
#endif
}

static void beep_service(void)
{
    if (!s_beep) return;
    if (millis() - s_beep_t0 < s_beep->ms) return;
    s_beep++;
    s_beep_t0 = millis();
    if (s_beep->ms == 0) { noTone(PIN_BUZZER); s_beep = nullptr; return; }
    if (s_beep->hz) tone(PIN_BUZZER, s_beep->hz); else noTone(PIN_BUZZER);
}

// =====================================================================
//  4. PCA9685 + SERVOS
// =====================================================================
static bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

static void i2c_scan(void)
{
    bench_printf("# I2C0 scan (GP8/GP9):");
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            bench_printf(" 0x%02X%s", a, a == PCA_ADDR ? "(PCA9685)" : a == 0x70 ? "(PCA all-call)" : "");
            found++;
        }
    }
    bench_printf(found ? "\n" : " nothing - check SDA/SCL, pull-ups, servo board power\n");
}

static bool     s_pca_ok = false;
static uint16_t s_counts[PCA_N_USED];
static uint32_t s_servo_writes = 0, s_servo_fail = 0;
static bool     s_servo_test = SERVO_TEST;

static uint16_t us_to_counts(float us)
{
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    return (uint16_t)(us * (4096.0f * PCA_PWM_HZ * 1e-6f) + 0.5f);
}

static bool pca_begin(void)
{
    uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * PCA_PWM_HZ) + 0.5f) - 1;
    uint8_t mode1 = 0, readback = 0;
    if (!i2c_write8(PCA_ADDR, 0x00, 0x00)) {
        bench_set(MT_PCA9685, ST_FAIL, "no ACK at 0x%02X (retry every 3 s)", PCA_ADDR);
        return false;
    }
    delay(1);
    bool ok = i2c_read(PCA_ADDR, 0x00, &mode1, 1)
           && i2c_write8(PCA_ADDR, 0x00, (uint8_t)((mode1 & 0x7F) | 0x10))   // sleep
           && i2c_write8(PCA_ADDR, 0xFE, prescale)
           && i2c_write8(PCA_ADDR, 0x00, (uint8_t)(mode1 & 0x7F));           // wake
    delay(1);
    ok = ok && i2c_write8(PCA_ADDR, 0x00, (uint8_t)((mode1 & 0x7F) | 0xA0))  // AI + RESTART
            && i2c_write8(PCA_ADDR, 0x01, 0x04)                              // totem pole
            && i2c_read(PCA_ADDR, 0xFE, &readback, 1);
    if (!ok) {
        bench_set(MT_PCA9685, ST_FAIL, "ACKs but register access failed");
        return false;
    }
    if (readback != prescale) {
        bench_set(MT_PCA9685, ST_FAIL, "prescale readback %u, wrote %u", readback, prescale);
        return false;
    }
    bench_set(MT_PCA9685, ST_PASS, "0x%02X, %.0f Hz (prescale %u read back OK)",
              PCA_ADDR, (double)PCA_PWM_HZ, prescale);
    return true;
}

static bool pca_write_all(void)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(0x06);                               // LED0_ON_L, auto-increment
    for (uint8_t i = 0; i < PCA_N_USED; i++) {
        Wire.write(0); Wire.write(0);
        Wire.write((uint8_t)(s_counts[i] & 0xFF));
        Wire.write((uint8_t)(s_counts[i] >> 8));
    }
    bool ok = Wire.endTransmission() == 0;
    s_servo_writes++;
    if (!ok) s_servo_fail++;
    return ok;
}

static void servos_neutral(void)
{
    s_counts[CH_TVC_X] = us_to_counts(TVC_TRIM_X_US);
    s_counts[CH_TVC_Y] = us_to_counts(TVC_TRIM_Y_US);
    s_counts[CH_CLAMP] = us_to_counts(0.5f * (CLAMP_US_CLOSED + CLAMP_US_OPEN));
    for (uint8_t i = 3; i < PCA_N_USED; i++) s_counts[i] = us_to_counts(1500.0f);
}

/** One channel at a time, five steps each, so a wrong channel or a wrong
 *  direction is obvious by eye. +deg must tilt the vehicle toward +x/+y. */
static void servo_service(void)
{
    static uint32_t next_ms = 0, next_retry = 0;
    static uint8_t  ch = 0, step = 0;

    if (!s_pca_ok) {
        if (millis() >= next_retry) {
            next_retry = millis() + 3000;
            s_pca_ok = pca_begin();
            if (s_pca_ok) { servos_neutral(); pca_write_all(); }
        }
        bench_set(MT_SERVO_WRITE, ST_FAIL, "no PCA9685");
        return;
    }
    if (millis() < next_ms) return;
    next_ms = millis() + SERVO_STEP_MS;

    if (s_servo_test) {
        static const char *const NAME[3] = { "TVC_X", "TVC_Y", "CLAMP" };
        static const float TVC_STEP[5]   = { 0.0f, +1.0f, 0.0f, -1.0f, 0.0f };
        float us;
        if (ch < 2) {
            const float trim = ch == 0 ? TVC_TRIM_X_US : TVC_TRIM_Y_US;
            us = trim + TVC_STEP[step] * SERVO_TEST_DEG * TVC_US_PER_DEG;
            bench_printf("# servo ch%u %-5s %+5.1f deg  %4.0f us\n", ch, NAME[ch],
                         (double)(TVC_STEP[step] * SERVO_TEST_DEG), (double)us);
        } else {
            static const float CL[5] = { 0.5f, 0.0f, 0.5f, 1.0f, 0.5f };
            us = CLAMP_US_CLOSED + CL[step] * (CLAMP_US_OPEN - CLAMP_US_CLOSED);
            bench_printf("# servo ch%u %-5s %3.0f %% open %4.0f us\n", ch, NAME[ch],
                         (double)(CL[step] * 100.0f), (double)us);
        }
        servos_neutral();
        s_counts[ch] = us_to_counts(us);
        if (++step >= 5) { step = 0; ch = (uint8_t)((ch + 1) % 3); }
    }

    if (!pca_write_all()) {
        // A PCA that stops answering mid-test: drop back to re-init.
        if (s_servo_fail > 5 && s_servo_fail * 2 > s_servo_writes) s_pca_ok = false;
    }
    bench_set(MT_SERVO_WRITE, s_servo_fail ? ST_WARN : ST_PASS,
              "%lu writes, %lu failed%s", (unsigned long)s_servo_writes,
              (unsigned long)s_servo_fail, s_servo_test ? "" : " (sweep off)");
}

// =====================================================================
//  5. W25Q128 FLASH — a state machine, so a 400 ms erase never stalls
// =====================================================================
static SPISettings s_flash_spi(20000000, MSBFIRST, SPI_MODE0);
static uint32_t    s_flash_size = 0;

static void flash_cmd(uint8_t cmd, int32_t addr, const uint8_t *tx, uint8_t *rx, uint32_t n)
{
    SPI1.beginTransaction(s_flash_spi);
    digitalWrite(PIN_FLASH_CS, LOW);
    SPI1.transfer(cmd);
    if (addr >= 0) {
        SPI1.transfer((uint8_t)(addr >> 16));
        SPI1.transfer((uint8_t)(addr >> 8));
        SPI1.transfer((uint8_t)addr);
    }
    for (uint32_t i = 0; i < n; i++) {
        uint8_t b = SPI1.transfer(tx ? tx[i] : 0x00);
        if (rx) rx[i] = b;
    }
    digitalWrite(PIN_FLASH_CS, HIGH);
    SPI1.endTransaction();
}

static uint8_t flash_sr1(void) { uint8_t s; flash_cmd(0x05, -1, nullptr, &s, 1); return s; }
static void    flash_wren(void) { flash_cmd(0x06, -1, nullptr, nullptr, 0); }

static bool flash_identify(void)
{
    uint8_t id[3];
    flash_cmd(0x9F, -1, nullptr, id, 3);
    if (id[0] == 0xEF && id[2] >= 0x14 && id[2] <= 0x19) {
        s_flash_size = 1UL << id[2];
        // Log usage: same binary search the flight firmware uses.
        uint8_t b[16];
        uint32_t lo = 0, hi = s_flash_size / 4096, used = 0;
        flash_cmd(0x03, 0, nullptr, b, 16);
        bool empty = true;
        for (uint8_t i = 0; i < 16; i++) if (b[i] != 0xFF) empty = false;
        if (!empty) {
            while (hi - lo > 1) {
                uint32_t mid = (lo + hi) / 2;
                flash_cmd(0x03, (int32_t)(mid * 4096), nullptr, b, 16);
                bool er = true;
                for (uint8_t i = 0; i < 16; i++) if (b[i] != 0xFF) er = false;
                if (er) hi = mid; else lo = mid;
            }
            used = hi * 4096;
        }
        bench_set(MT_FLASH_ID, ST_PASS, "JEDEC %02X %02X %02X = %lu MB, log uses %lu KB",
                  id[0], id[1], id[2], (unsigned long)(s_flash_size >> 20),
                  (unsigned long)(used / 1024));
        return true;
    }
    bench_set(MT_FLASH_ID, ST_FAIL, "JEDEC %02X %02X %02X (want EF 40 18) - CS/MISO/power?",
              id[0], id[1], id[2]);
    return false;
}

enum { FL_IDLE, FL_ERASE, FL_WAIT_ERASE, FL_PROGRAM, FL_WAIT_PROG, FL_VERIFY,
       FL_RESTORE, FL_WAIT_RESTORE, FL_DONE };
static uint8_t  s_fl = FL_IDLE;
static uint32_t s_fl_t0 = 0;
static bool     s_fl_failed = false;

static void flash_test_start(void)
{
    s_fl = s_flash_size ? FL_ERASE : FL_IDLE;
    s_fl_failed = false;
}

static void flash_service(void)
{
    static uint32_t next_id_retry = 0;
    if (!s_flash_size) {
        if (millis() >= next_id_retry) {
            next_id_retry = millis() + 3000;
            if (flash_identify()) flash_test_start();
            else bench_set(MT_FLASH_RW, ST_FAIL, "no flash chip");
        }
        return;
    }
#if !FLASH_WRITE_TEST
    bench_set(MT_FLASH_RW, ST_SKIP, "FLASH_WRITE_TEST 0");
    return;
#endif
    const uint32_t addr = s_flash_size - 4096;   // last sector, the log never gets there
    switch (s_fl) {
    case FL_ERASE:
        flash_wren();
        flash_cmd(0x20, (int32_t)addr, nullptr, nullptr, 0);
        s_fl_t0 = millis();
        s_fl = FL_WAIT_ERASE;
        break;
    case FL_WAIT_ERASE:
    case FL_WAIT_RESTORE:
        if (flash_sr1() & 0x01) {
            if (millis() - s_fl_t0 > 2000) {
                bench_set(MT_FLASH_RW, ST_FAIL, "sector erase never finished (>2 s)");
                s_fl = FL_DONE;
            }
            break;
        }
        if (s_fl == FL_WAIT_RESTORE) {
            if (!s_fl_failed)
            bench_set(MT_FLASH_RW, ST_PASS, "erase %lu ms, write+verify 256 B OK at 0x%06lX",
                      (unsigned long)(millis() - s_fl_t0), (unsigned long)addr);
            s_fl = FL_DONE;
            break;
        }
        {   // the erase must actually have erased
            uint8_t b[64];
            flash_cmd(0x03, (int32_t)addr, nullptr, b, sizeof(b));
            for (uint8_t i = 0; i < sizeof(b); i++) if (b[i] != 0xFF) {
                bench_set(MT_FLASH_RW, ST_FAIL, "erased sector reads 0x%02X at +%u", b[i], i);
                s_fl = FL_DONE;
                return;
            }
        }
        s_fl = FL_PROGRAM;
        break;
    case FL_PROGRAM: {
        uint8_t page[256];
        for (uint16_t i = 0; i < 256; i++) page[i] = (uint8_t)(i * 7 + 0x5A);
        flash_wren();
        flash_cmd(0x02, (int32_t)addr, page, nullptr, 256);
        s_fl_t0 = millis();
        s_fl = FL_WAIT_PROG;
        break;
    }
    case FL_WAIT_PROG:
        if (flash_sr1() & 0x01) {
            if (millis() - s_fl_t0 > 50) {
                bench_set(MT_FLASH_RW, ST_FAIL, "page program never finished");
                s_fl = FL_DONE;
            }
            break;
        }
        s_fl = FL_VERIFY;
        break;
    case FL_VERIFY: {
        uint8_t page[256];
        flash_cmd(0x03, (int32_t)addr, nullptr, page, 256);
        for (uint16_t i = 0; i < 256; i++) {
            if (page[i] != (uint8_t)(i * 7 + 0x5A)) {
                bench_set(MT_FLASH_RW, ST_FAIL, "verify mismatch at +%u: %02X want %02X",
                          i, page[i], (uint8_t)(i * 7 + 0x5A));
                s_fl_failed = true;
                s_fl = FL_RESTORE;
                return;
            }
        }
        s_fl = FL_RESTORE;
        break;
    }
    case FL_RESTORE:                               // leave the sector erased again
        flash_wren();
        flash_cmd(0x20, (int32_t)addr, nullptr, nullptr, 0);
        s_fl_t0 = millis();
        s_fl = FL_WAIT_RESTORE;
        break;
    default:
        break;
    }
    if (s_fl != FL_DONE && g_test[MT_FLASH_RW].st == ST_PEND)
        bench_set(MT_FLASH_RW, ST_PEND, "running...");
}

// =====================================================================
//  6. LoRa — asynchronous transmit of a ground-station-readable packet
// =====================================================================
// Mirror of ariadna_tlm_t version 3 (common/ariadna_link.h), 47 bytes.
// The ground station only shows packets of exactly this size and version.
#pragma pack(push, 1)
typedef struct {
    uint32_t t_ms;
    int32_t  lat, lon;
    int16_t  alt_dm, vz_dms, vh_dms, acc_cg, roll_ddeg, pitch_ddeg, yaw_ddeg,
             temp_dc, tvc_x_cdeg, tvc_y_cdeg, thrust_n;
    uint8_t  version, state, nav_phase, clamp_pct, vbat_cv, curr_da, hdop_x10,
             sats, pyro, health, flags, exp, uwb;
} bench_tlm_t;
#pragma pack(pop)
static_assert(sizeof(bench_tlm_t) == 47, "must match ariadna_tlm_t");

static bool     s_lora_ok = false;
static bool     s_lora_busy = false;
static uint32_t s_lora_t0 = 0, s_lora_sent = 0, s_lora_done = 0, s_lora_air_ms = 0;
static uint32_t s_lora_noirq = 0;
static uint32_t s_lora_next = 3000;
static volatile bool     s_lora_irq = false;
static volatile uint32_t s_lora_irq_ms = 0;
static float    s_vbat = 0.0f, s_amps = 0.0f;

// DIO0 rising edge = TX done. Using the interrupt, rather than polling the
// radio, is what tests the DIO0 wire (GP20) as well.
static void lora_on_tx_done(void)
{
    s_lora_irq_ms = millis();
    s_lora_irq    = true;
}

static void lora_begin(void)
{
    SPI.setRX(PIN_LORA_MISO);
    SPI.setTX(PIN_LORA_MOSI);
    SPI.setSCK(PIN_LORA_SCK);
    SPI.begin();
    LoRa.setSPI(SPI);
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
    // begin() only reads the version register: it returns at once if the
    // module is missing, it does not wait for it.
    if (LoRa.begin(LORA_FREQ_HZ)) {
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setSignalBandwidth(LORA_BW_HZ);
        LoRa.setCodingRate4(LORA_CR4);
        LoRa.setPreambleLength(LORA_PREAMBLE);
        LoRa.setTxPower(LORA_TEST_DBM);
        LoRa.enableCrc();
        LoRa.onTxDone(lora_on_tx_done);
        s_lora_ok = true;
        bench_set(MT_LORA_INIT, ST_PASS, "SX1278 433 MHz SF%d BW%ldk, %d dBm",
                  LORA_SF, LORA_BW_HZ / 1000, LORA_TEST_DBM);
    } else {
        bench_set(MT_LORA_INIT, ST_FAIL, "SX1278 not answering (NSS GP17, RST GP21, SPI0)");
#if LORA_TX_TEST
        bench_set(MT_LORA_TX, ST_FAIL, "no radio");
#endif
    }
}

static void lora_send_now(void) { s_lora_next = millis(); }

static void lora_service(void)
{
#if !LORA_TX_TEST
    bench_set(MT_LORA_TX, ST_SKIP, "LORA_TX_TEST 0");
    return;
#endif
    if (!s_lora_ok) return;

    if (s_lora_busy) {
        if (s_lora_irq) {
            s_lora_irq  = false;
            s_lora_busy = false;
            s_lora_done++;
            s_lora_air_ms = s_lora_irq_ms - s_lora_t0;
            bench_set(MT_LORA_TX, ST_PASS, "%lu sent, %lu TX-done IRQs, on air %lu ms",
                      (unsigned long)s_lora_sent, (unsigned long)s_lora_done,
                      (unsigned long)s_lora_air_ms);
        } else if (millis() - s_lora_t0 > 1000) {
            // No IRQ. Whether the radio finished anyway is answered by the
            // next beginPacket(), which refuses while a TX is still running.
            s_lora_busy = false;
            s_lora_noirq++;
        }
        return;
    }
    if ((int32_t)(millis() - s_lora_next) < 0) return;
    s_lora_next = millis() + LORA_PERIOD_MS;

    bench_tlm_t t;
    memset(&t, 0, sizeof(t));
    t.t_ms    = millis();
    t.version = 3;                 // ARIADNA_TLM_VERSION
    t.state   = 10;                // ARIADNA_STATE_TEST
    t.flags   = (1u << 3);         // ARIADNA_TF_TEST
    t.temp_dc = (int16_t)(analogReadTemp() * 10.0f);
    float vb = (s_vbat - 2.5f) * 100.0f;
    t.vbat_cv = (uint8_t)(vb < 0.0f ? 0.0f : vb > 255.0f ? 255.0f : vb);
    float ca = s_amps * 10.0f;
    t.curr_da = (uint8_t)(ca < 0.0f ? 0.0f : ca > 255.0f ? 255.0f : ca);
    t.exp     = (uint8_t)s_lora_sent;   // counter, so missed packets are visible

    if (!LoRa.beginPacket()) {
        bench_set(MT_LORA_TX, ST_FAIL, "radio still transmitting after %lu ms - stuck",
                  (unsigned long)(millis() - s_lora_t0));
        return;
    }
    if (s_lora_noirq && s_lora_done == 0)
        bench_set(MT_LORA_TX, ST_WARN, "packets leave, but no TX-done IRQ - DIO0 (GP20) wire?");
    LoRa.write((const uint8_t *)&t, sizeof(t));
    LoRa.endPacket(true);          // async: returns immediately
    s_lora_busy = true;
    s_lora_t0   = millis();
    s_lora_sent++;
}

// =====================================================================
//  7. ANALOGUE + DIGITAL INPUTS
// =====================================================================
static void inputs_service(void)
{
    static uint32_t next_ms = 0;
    static bool rbf_seen[2] = { false, false }, bw_seen[2] = { false, false };
    static int  rbf_last = -1, bw_last = -1;
    if (millis() < next_ms) return;
    next_ms = millis() + 50;

    const int raw_v = analogRead(PIN_V_BAT);
    const int raw_i = analogRead(PIN_I_SENSE);
    const float v_adc = (float)raw_v * (ADC_VREF / 4095.0f);
    const float i_adc = (float)raw_i * (ADC_VREF / 4095.0f);
    s_vbat += 0.2f * (v_adc * VBAT_DIVIDER - s_vbat);
    s_amps += 0.2f * ((i_adc - ACS712_ZERO_V) / ACS712_V_PER_A - s_amps);

    if (s_vbat < 0.5f)
        bench_set(MT_VBAT, ST_WARN, "%.2f V (raw %d) - no battery? USB-only power", (double)s_vbat, raw_v);
    else if (s_vbat >= 6.0f && s_vbat <= 8.7f)
        bench_set(MT_VBAT, ST_PASS, "%.2f V (2S, %.2f V/cell)", (double)s_vbat, (double)(s_vbat * 0.5f));
    else
        bench_set(MT_VBAT, ST_WARN, "%.2f V outside 6.0..8.7 V - divider or flat pack?", (double)s_vbat);

    if (raw_i < 50 || raw_i > 4045)
        bench_set(MT_CURRENT, ST_FAIL, "raw %d at the rail - ACS712 missing/unpowered", raw_i);
    else if (s_amps > -1.0f && s_amps < 15.0f)
        bench_set(MT_CURRENT, ST_PASS, "%.2f A (sensor %.3f V)", (double)s_amps, (double)i_adc);
    else
        bench_set(MT_CURRENT, ST_WARN, "%.2f A implausible - zero offset %.3f V?", (double)s_amps, (double)i_adc);

    const int rbf = digitalRead(PIN_RBF);
    const int bw  = digitalRead(PIN_BREAKWIRE);
    if (rbf != rbf_last) {
        if (rbf_last >= 0) bench_printf("[%8.2f s] EVENT RBF %s\n", (double)bench_t_s(),
                                        rbf ? "REMOVED (HIGH)" : "FITTED (LOW)");
        rbf_last = rbf;
    }
    if (bw != bw_last) {
        if (bw_last >= 0) bench_printf("[%8.2f s] EVENT breakwire %s\n", (double)bench_t_s(),
                                       bw ? "BROKEN (HIGH)" : "INTACT (LOW)");
        bw_last = bw;
    }
    rbf_seen[rbf ? 1 : 0] = true;
    bw_seen[bw ? 1 : 0]   = true;
    if (rbf_seen[0] && rbf_seen[1]) bench_set(MT_RBF, ST_PASS, "both states seen, now %s", rbf ? "REMOVED" : "FITTED");
    else bench_set(MT_RBF, ST_INFO, "%s - insert/remove the tag to test", rbf ? "REMOVED (HIGH)" : "FITTED (LOW)");
    if (bw_seen[0] && bw_seen[1]) bench_set(MT_BREAKWIRE, ST_PASS, "both states seen, now %s", bw ? "BROKEN" : "INTACT");
    else bench_set(MT_BREAKWIRE, ST_INFO, "%s - connect/disconnect it to test", bw ? "BROKEN (HIGH)" : "INTACT (LOW)");

#if AUX_S16_TOGGLE
    digitalWrite(PIN_AUX_S16, (millis() / 500) & 1);
#endif
}

// =====================================================================
//  8. LINKS — answer, route, ping
// =====================================================================
static void on_rx(bench_port_t *p, uint8_t type, const uint8_t *pl, uint8_t len)
{
    if (len < 4) return;
    const uint8_t src = pl[1], dst = pl[2];
    const uint8_t expect = (p == &s_pos) ? NODE_POSITION : NODE_NAVIGATION;

    // A packet from the WRONG board on this port means the UARTs are crossed.
    if (src != expect && src != NODE_MASTER) {
        if (s_crosswired++ == 0)
            bench_printf("[%8.2f s] EVENT %s port receives packets from %s - links crossed?\n",
                         (double)bench_t_s(), p->name, BENCH_NODE_NAME[src < NODE_N ? src : 0]);
    }

    if ((type == PKT_PING || type == PKT_PONG) &&
        (dst == NODE_POSITION || dst == NODE_NAVIGATION)) {
        bench_port_t *out = (dst == NODE_POSITION) ? &s_pos : &s_nav;
        if (out == p) { s_misrouted++; return; }
        uint8_t buf[BENCH_PAYLOAD_MAX];
        memcpy(buf, pl, len);
        buf[3]++;                                  // hops
        bench_note_peer(src, ((const bench_ping_t *)buf)->uptime_ms);
        bench_send(out, type, buf, len);
        s_routed++;
        return;
    }

    bench_ping_t reply;
    if (bench_handle(type, pl, len, &reply)) bench_send(p, PKT_PONG, &reply, sizeof(reply));
}

static void ping_service(void)
{
    static uint32_t next_ms = 0;
    if (millis() < next_ms) return;
    next_ms = millis() + 100;                      // 10 Hz per peer
    bench_ping_t pk;
    bench_ping_build(&pk, NODE_POSITION);   bench_send(&s_pos, PKT_PING, &pk, sizeof(pk));
    bench_ping_build(&pk, NODE_NAVIGATION); bench_send(&s_nav, PKT_PING, &pk, sizeof(pk));
}

// =====================================================================
//  9. RESET LINES — pulse, then watch the peer's uptime go backwards
// =====================================================================
typedef struct {
    uint8_t  pin, node, test, state;
    uint32_t t0, reboots0;
} reset_test_t;
enum { RT_WAIT, RT_PULSE, RT_WATCH, RT_DONE };
static reset_test_t s_rt[2] = {
    { PIN_RST_POS, NODE_POSITION,   MT_RESET_POS, RT_WAIT, 15000, 0 },
    { PIN_RST_NAV, NODE_NAVIGATION, MT_RESET_NAV, RT_WAIT, 21000, 0 },
};

static void reset_request(uint8_t i)
{
    s_rt[i].state = RT_WAIT;
    s_rt[i].t0    = millis();
}

static void reset_service(void)
{
    for (uint8_t i = 0; i < 2; i++) {
        reset_test_t *r = &s_rt[i];
        bench_peer_t *pe = &g_peer[r->node];
        const bool alive = pe->seen && millis() - pe->last_rx_ms < 1500;
        switch (r->state) {
        case RT_WAIT:
            if ((int32_t)(millis() - r->t0) < 0) {
#if RESET_LINE_TEST
                bench_set(r->test, ST_PEND, "pulse at %lu s", (unsigned long)(r->t0 / 1000));
#else
                bench_set(r->test, ST_SKIP, "RESET_LINE_TEST 0 (press %c)", i ? 'n' : 'r');
#endif
                break;
            }
            if (!alive) {
                bench_set(r->test, ST_SKIP, "%s not talking - cannot see a reboot",
                          BENCH_NODE_NAME[r->node]);
                r->state = RT_DONE;
                break;
            }
            bench_printf("[%8.2f s] EVENT pulsing RESET of %s (GP%u)\n",
                         (double)bench_t_s(), BENCH_NODE_NAME[r->node], r->pin);
            r->reboots0 = pe->reboots;
            digitalWrite(r->pin, LOW);
            r->t0 = millis();
            r->state = RT_PULSE;
            break;
        case RT_PULSE:
            if (millis() - r->t0 < 20) break;
            digitalWrite(r->pin, HIGH);
            r->t0 = millis();
            r->state = RT_WATCH;
            break;
        case RT_WATCH:
            if (pe->reboots != r->reboots0) {
                bench_set(r->test, ST_PASS, "%s rebooted %lu ms after the pulse",
                          BENCH_NODE_NAME[r->node], (unsigned long)(millis() - r->t0));
                r->state = RT_DONE;
            } else if (millis() - r->t0 > 6000) {
                bench_set(r->test, ST_FAIL, "%s did NOT reboot - RESET wire to its RUN pin?",
                          BENCH_NODE_NAME[r->node]);
                r->state = RT_DONE;
            }
            break;
        default:
            break;
        }
    }
}

// =====================================================================
//  10. SUMMARY
// =====================================================================
static void print_summary(void)
{
    bench_print_table();
    bench_printf("  servo %s | vbat %.2f V  I %.2f A  die %.1f C | loop max %lu us (worst %lu)\n",
                 s_servo_test ? "sweep" : "hold", (double)s_vbat, (double)s_amps,
                 (double)analogReadTemp(), (unsigned long)s_loop.max_us,
                 (unsigned long)s_loop.max_ever_us);
    bench_print_port(&s_pos);
    bench_print_port(&s_nav);
    bench_printf("  routed %lu  misrouted %lu  crossed %lu\n", (unsigned long)s_routed,
                 (unsigned long)s_misrouted, (unsigned long)s_crosswired);
    bench_print_peer(NODE_POSITION);
    bench_print_peer(NODE_NAVIGATION);

    // The other two boards' results, as they reported them.
    for (uint8_t node = NODE_POSITION; node <= NODE_NAVIGATION; node++) {
        const bench_peer_t *pe = &g_peer[node];
        if (!pe->have_report || millis() - pe->last_rx_ms > 3000) {
            bench_printf("---- %s: no report ----\n", BENCH_NODE_NAME[node]);
            continue;
        }
        const bench_report_t *r = &pe->report;
        bench_printf("---- %s (remote, up %lu s, die %.1f C, loop max %lu us) ----\n",
                     BENCH_NODE_NAME[node], (unsigned long)(r->uptime_ms / 1000),
                     (double)(r->temp_dc * 0.1f), (unsigned long)r->loop_max_us);
        for (uint8_t i = 0; i < r->n_tests && i < BENCH_TEST_COUNT[node]; i++) {
            uint8_t st = bench_report_st(r, i);
            bench_printf("%s%-4s %-12s", (i % 4) ? "  " : "  ", BENCH_ST_NAME[st < 6 ? st : 0],
                         BENCH_TEST_NAMES[node][i]);
            if (i % 4 == 3 || i + 1 == r->n_tests) bench_printf("\n");
        }
    }
    bench_printf("\n");
    bench_window_reset();
    s_loop.max_us = 0;
}

static void console_service(void)
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
        case 'h': case '?':
            bench_printf("# h help | p summary | s servo sweep on/off | b beep | l LoRa now\n"
                         "# f rerun flash test | i I2C scan | r reset Position | n reset Navigation\n");
            break;
        case 'p': print_summary(); break;
        case 's': s_servo_test = !s_servo_test;
                  bench_printf("# servo sweep %s\n", s_servo_test ? "ON" : "OFF (neutral)");
                  servos_neutral(); break;
        case 'b': beep_start(); break;
        case 'l': lora_send_now(); break;
        case 'f': g_test[MT_FLASH_RW].st = ST_PEND; flash_test_start(); break;
        case 'i': i2c_scan(); break;
        case 'r': reset_request(0); break;
        case 'n': reset_request(1); break;
        default: break;
        }
    }
}

// =====================================================================
//  11. SETUP / LOOP
// =====================================================================
void setup()
{
    // Subordinates held in reset while the UARTs come up, as in flight.
    pinMode(PIN_RST_POS, OUTPUT); digitalWrite(PIN_RST_POS, LOW);
    pinMode(PIN_RST_NAV, OUTPUT); digitalWrite(PIN_RST_NAV, LOW);
    pinMode(PIN_BREAKWIRE, INPUT_PULLUP);
    pinMode(PIN_RBF, INPUT_PULLUP);
    pinMode(PIN_AUX_S16, OUTPUT); digitalWrite(PIN_AUX_S16, LOW);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_FLASH_CS, OUTPUT); digitalWrite(PIN_FLASH_CS, HIGH);
    pinMode(PIN_LORA_CS, OUTPUT);  digitalWrite(PIN_LORA_CS, HIGH);
    analogReadResolution(12);

    Serial.begin(BENCH_CONSOLE_BAUD);          // no while(!Serial): never wait for a PC
    bench_tests_init(NODE_MASTER);

    s_px.begin();
    s_px.clear();
    s_px.show();

    // I2C with a timeout: a shorted or unpowered bus returns an error
    // instead of hanging the loop.
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setTimeout(10, true);
    Wire.begin();
    Wire.setClock(400000);
    s_pca_ok = pca_begin();
    servos_neutral();
    if (s_pca_ok) pca_write_all();

    SPI1.setRX(PIN_FLASH_MISO);
    SPI1.setTX(PIN_FLASH_MOSI);
    SPI1.setSCK(PIN_FLASH_SCK);
    SPI1.begin();
    if (flash_identify()) flash_test_start();

    lora_begin();

    bench_port_begin(&s_pos, Serial1, uart0, PIN_POS_TX, PIN_POS_RX, "UART0>POS");
    bench_port_begin(&s_nav, Serial2, uart1, PIN_NAV_TX, PIN_NAV_RX, "UART1>NAV");

    digitalWrite(PIN_RST_POS, HIGH);           // release the subordinates
    digitalWrite(PIN_RST_NAV, HIGH);

    beep_start();
    bench_printf("\n=== Ariadna V2 MASTER bench test, proto v%d, CPU %lu MHz ===\n",
                 BENCH_PROTO_VERSION, (unsigned long)(rp2040.f_cpu() / 1000000));
}

void loop()
{
    static uint32_t next_summary = 3000, next_verdict = 0;
    bench_loop_begin(&s_loop);

    bench_port_poll(&s_pos, on_rx);
    bench_port_poll(&s_nav, on_rx);
    bench_port_service_tx(&s_pos);
    bench_port_service_tx(&s_nav);

    ping_service();
    servo_service();
    flash_service();
    lora_service();
    inputs_service();
    reset_service();
    rgb_service();
    beep_service();
    console_service();

    if (millis() >= next_verdict) {
        next_verdict = millis() + 250;
        bench_link_verdict(MT_LINK_POS, NODE_POSITION, &s_pos);
        bench_link_verdict(MT_LINK_NAV, NODE_NAVIGATION, &s_nav);
        bench_core1_verdict(MT_CORE1);
    }
    if (millis() >= next_summary) {
        next_summary = millis() + 5000;
        print_summary();
    }
    bench_loop_end(&s_loop);
}

// Core 1: heartbeat only. The flight firmware puts flash and LoRa here;
// this proves the core starts and keeps running.
void setup1() {}
void loop1()  { bench_core1_tick(); }
