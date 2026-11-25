// =====================================================================
//  Ariadna V2 — Static Fire Link for the MASTER MCU
//
//  Target: Raspberry Pi Pico 2 W (RP2350), arduino-pico core.
//  Needs:  Adafruit_NeoPixel. The PCA9685 and the W25Q128 are driven by
//          their registers / commands, nothing else.
//  Pair:   Examples/Pyro_Companion on the Navigation MCU (pyro 1),
//          the Static Fire Stand (firmware 2.1+, `link ariadna`).
//
//  WHAT IT DOES
//    The stand (load cell, I2C slave 0x2C) and Ariadna tell each other that
//    they are READY. The operator then starts the countdown on the stand
//    (button or WiFi page), as always. Ariadna watches it, arms pyro 1 on the
//    Navigation MCU, and when the stand reaches T0 and says IGNITE, fires it.
//    The clamp ("klapka"), fully open until then, runs this profile as soon
//    as the thrust passes 100 N:
//
//        50 % open, 300 ms -> minimum, 300 ms -> 75 %, 300 ms -> fully open
//
//    All the while the stand records the thrust (its own power-fail safe log)
//    and Ariadna records battery voltage, current, the clamp pulse and the
//    clamp closure. When the burn is over Ariadna copies the stand's whole
//    record over I2C into its own W25Q128 flash and merges the two: one row
//    per thrust sample, with Ariadna's numbers beside it.
//
//    The decisions (when to arm, when to fire, what the clamp does, what is
//    stored) are in sf_run.h, sf_flaps.h and sf_record.h, which have no
//    hardware in them and are tested on a workstation: `cd test && make sflink`.
//    This file is the wire around them.
//
//  WIRING
//    Stand GP4 (SDA), GP5 (SCL), GND  ->  Ariadna Master I2C1 or I2C2 uSup
//    (pins SDA, SCL, GND; leave 3V3 UNCONNECTED, two supplies must not be tied).
//    That is the PCA9685 bus (GP8/GP9, 10k pull-ups, 400 kHz): the stand is one
//    more slave on it, 0x2C next to the PCA9685 at 0x40. The pull-ups are on
//    Ariadna's side; the stand has none.
//    Igniter on Ariadna pyro 1 (Navigation MCU D3). Clamp servo on PCA9685
//    channel 7, as in flight.
//
//  SAFETY (details at the top of sf_run.h)
//    * Fires only for a countdown it watched, armed for, and saw IGNITE for.
//    * Needs RBF pulled, continuity on pyro 1, stand and Navigation MCU
//      answering, a blank flash slot ready. Anything missing = NOT READY,
//      which stops the stand's countdown.
//    * The clamp is open at power-up, in every idle state, and the fail-safe.
//    * The pyro rules themselves (armed only 60 s, heartbeat, one pulse) are
//      the Navigation MCU's, in actcon_pyro.h, exactly as in Actuator_Console.
//
//  USB console, 115200, one command per line:
//    ?            help
//    s            one status line
//    i            status as JSON
//    l            list the stored runs
//    p [slot]     dump stored runs (stand format + merged CSV), see README
//    pull         copy the stand's LAST burn now (after a reset, or a missed copy)
//    hold / go    say "not ready" / "ready" (stops a countdown)
//    wipe         erase all stored runs (type: wipe yes)
//
//  !! Bench first with LEDs on the pyro output and the clamp linkage unhooked.
//  !! With a real igniter: people clear, motor restrained, line of sight to
//  !! the stand, RBF keys out on BOTH boards last.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "ariadna_link.h"
#include "actcon_pyro.h"
#include "sf_link_proto.h"
#include "sf_flaps.h"
#include "sf_run.h"
#include "sf_record.h"

// =====================================================================
//  CONFIGURATION — the things to check against YOUR vehicle
// =====================================================================

// ---- the clamp ("klapka"), PCA9685 ---------------------------------------
static const uint8_t CH_CLAMP        = 7;       // flight default: srv channel 7
// The pulses for fully open / fully shut. The same numbers as the parameters
// srv.clamp_open_us / srv.clamp_closed_us of the flight firmware: use YOUR
// calibrated values, and MEASURE the linkage before the first move.
static const float   CLAMP_US_OPEN   = 1900.0f;
static const float   CLAMP_US_CLOSED = 1150.0f;
static const float   PWM_HZ          = 50.0f;   // any servo; 339 for a digital servo as in flight

// ---- the profile ------------------------------------------------------------
static const float   TRIGGER_N       = 100.0f;  // thrust that starts it
static const uint32_t STEP_MS        = 300;     // each of the three steps
static const float   OPEN_PCT_1      = 50.0f;   // first step, % open
static const float   OPEN_PCT_MIN    = 10.0f;   // "minimum": the flight software's DG_CLAMP_MIN. 0 = shut
static const float   OPEN_PCT_3      = 75.0f;   // third step, % open
// after that: 100 % (fully open) until the end of the burn

// ---- interlocks ---------------------------------------------------------------
#define REQUIRE_RBF   1     // pyro 1 needs the RBF pin pulled on THIS board
#define REQUIRE_CONT  1     // ... and an igniter on it (0 on a bench without one)
#define RBF_REMOVED_IS_HIGH  1

// ---- pins: identical to Master_MCU/Master_MCU.ino ---------------------------
static const uint8_t PIN_RGB        = 2;    // SK6812
static const uint8_t PIN_BUZZER     = 3;
static const uint8_t PIN_NAV_TX     = 4;    // UART1 -> Navigation MCU
static const uint8_t PIN_NAV_RX     = 5;
static const uint8_t PIN_PCA_OE     = 6;    // PCA9685 /OE, LOW = outputs on
static const uint8_t PIN_I2C_SDA    = 8;    // PCA9685 + the stand
static const uint8_t PIN_I2C_SCL    = 9;
static const uint8_t PIN_FLASH_MISO = 12;
static const uint8_t PIN_FLASH_CS   = 13;
static const uint8_t PIN_FLASH_SCK  = 14;
static const uint8_t PIN_FLASH_MOSI = 15;
static const uint8_t PIN_I_SENSE    = 26;   // ADC0, ACS712
static const uint8_t PIN_V_BAT      = 27;   // ADC1, 2S through 3:1
static const uint8_t PIN_RBF        = 28;   // HIGH = tag removed

#define NAV_PORT  Serial2                   // UART1 on arduino-pico

// ---- measurement: same constants as Master_MCU.ino / Actuator_Console ------
static const float ADC_VREF       = 3.30f;
static const float ADC_FULL_SCALE = 4095.0f;
static const float VBAT_DIVIDER   = 3.00f;
static const float ACS712_ZERO_V  = 2.504f;    // sensor output at 0 A: CHECK AGAINST YOUR BOARD
static const float ACS712_V_PER_A = 0.100f;    // 20 A part
static const float ACS712_DIVIDER = 0.50f;

// ---- PCA9685 ---------------------------------------------------------------------
static const uint8_t PCA_ADDR    = 0x40;
static const float   PCA_OSC_HZ  = 25000000.0f;
static const uint8_t PCA_N_CH    = 16;
static const float   ABS_US_MIN  = 500.0f;
static const float   ABS_US_MAX  = 2500.0f;

// ---- timing ------------------------------------------------------------------------
static const uint32_t NAV_TIMEOUT_MS = 300;
static const uint32_t LOG_PERIOD_US  = 5000;     // 200 Hz own samples
static const uint32_t PCA_CHECK_MS   = 1000;
static const uint32_t OWN_MAX        = 8192;     // 41 s at 200 Hz

// =====================================================================
//  state
// =====================================================================
static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

// clamp
static bool     s_pca_ok   = false;
static float    s_open_pct = 100.0f;
static uint16_t s_clamp_us = 0;

// Navigation MCU / pyro
static ariadna_rx_t          s_nav_rx;
static actcon_pyro_report_t  s_nav;
static uint32_t s_nav_last = 0;
static bool     s_nav_ok   = false;
static uint32_t s_pyro_seq = 1;

// sensors
static bool  s_rbf_out  = false;
static float s_vbat     = 0.0f;
static float s_i_acc    = 0.0f;
static uint16_t s_i_n   = 0;
static float s_i_last   = 0.0f;

// flash
static bool     s_flash_ok     = false;
static uint32_t s_run_id_next  = 1;        // the run the NEXT stored record gets
static uint32_t s_slot         = 0;        // slot it goes to
static bool     s_slot_ready   = false;    // blank and waiting
static uint32_t s_erase_pos    = 0;        // sectors, absolute; pos < end = erasing
static uint32_t s_erase_end    = 0;
static uint32_t s_erase_next_ms = 0;

// the stand
static sfl_status_t s_st;
static bool     s_st_valid  = false;
static bool     s_st_new    = false;
static uint32_t s_st_ms     = 0;           // millis() of the last good STATUS
static uint32_t s_st_us     = 0;           // micros() of the moment it was taken
static sfl_info_t s_info;
static bool     s_info_valid = false;
static uint32_t s_info_ms   = 0;
static uint8_t  s_data_seq  = 0;           // run whose data is safely on Ariadna's flash
static bool     s_busy      = false;
static bool     s_hold      = false;
static bool     s_proto_bad = false;
static bool     s_stand_was_ok = false;

// the run
static sfr_cfg_t s_cfg;
static sfr_t     s_r;
static sfr_out_t s_out;
static uint8_t   s_own_run_seq = 0;        // the stand's run the samples belong to
static uint16_t  s_own_flags   = 0;        // AFR_H_* collected during the run
static afr_sample_t s_own[OWN_MAX];
static uint32_t  s_own_n = 0;
static uint8_t   s_raw[AFR_RAW_MAX * AFR_PAGE];

// heartbeat bookkeeping
static uint8_t  s_hb_flags = 0, s_hb_ack = 0, s_hb_data = 0;
static uint32_t s_hb_ms    = 0;

// =====================================================================
//  PCA9685 — only the clamp channel is ever driven
// =====================================================================
static bool i2c_write8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static int i2c_read8(uint8_t reg)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return -1;
    return Wire.read();
}

static uint16_t us_to_counts(float us)
{
    const float c = us * (4096.0f * PWM_HZ * 1e-6f) + 0.5f;
    if (c < 1.0f) return 1;
    if (c > 4095.0f) return 4095;
    return (uint16_t)c;
}

static bool pca_write_ch(uint8_t ch, uint16_t us)      // 0 = full off
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(0x06 + 4 * ch));              // LEDn_ON_L, auto-increment
    if (us == 0) {
        Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
    } else {
        const uint16_t c = us_to_counts((float)us);
        Wire.write(0); Wire.write(0);
        Wire.write((uint8_t)(c & 0xFF));
        Wire.write((uint8_t)(c >> 8));
    }
    return Wire.endTransmission() == 0;
}

static bool pca_begin(void)
{
    const uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * PWM_HZ) + 0.5f) - 1;
    bool ok = i2c_write8(0x00, 0x10) && i2c_write8(0xFE, prescale) && i2c_write8(0x00, 0x00);
    delay(1);
    ok = ok && i2c_write8(0x00, 0xA0) && i2c_write8(0x01, 0x04) && i2c_read8(0xFE) == prescale;
    if (!ok) return false;
    for (uint8_t ch = 0; ch < PCA_N_CH; ch++)          // nothing else moves, ever
        if (ch != CH_CLAMP && !pca_write_ch(ch, 0)) return false;
    return pca_write_ch(CH_CLAMP, s_clamp_us);         // and the clamp goes where it should be
}

static void pca_lost(const char *why)
{
    if (s_pca_ok) Serial.printf("# PCA9685 lost: %s\n", why);
    s_pca_ok = false;
    digitalWrite(PIN_PCA_OE, HIGH);
}

static void pca_try_start(void)
{
    s_pca_ok = pca_begin();
    if (s_pca_ok) {
        digitalWrite(PIN_PCA_OE, LOW);
        Serial.printf("# PCA9685 ready, %.0f Hz, clamp on channel %u\n", (double)PWM_HZ, CH_CLAMP);
    }
}

/** Where the clamp should be, in percent OPEN. */
static void clamp_set(float open_pct)
{
    float us = sff_pct_to_us(open_pct, CLAMP_US_CLOSED, CLAMP_US_OPEN);
    if (us < ABS_US_MIN) us = ABS_US_MIN;
    if (us > ABS_US_MAX) us = ABS_US_MAX;
    s_open_pct = open_pct;
    const uint16_t u = (uint16_t)(us + 0.5f);
    if (u == s_clamp_us) return;
    s_clamp_us = u;
    if (s_pca_ok && !pca_write_ch(CH_CLAMP, s_clamp_us)) pca_lost("write failed");
}

// =====================================================================
//  Navigation MCU: pyro 1 through the Pyro_Companion
// =====================================================================
static void nav_send(uint8_t cmd, uint8_t ch, uint16_t pulse_ms, uint8_t repeats)
{
    actcon_pyro_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd      = cmd;
    c.ch       = ch;
    c.pulse_ms = pulse_ms;
    c.seq      = (cmd == ACTCON_CMD_HEARTBEAT) ? 0 : s_pyro_seq++;
    uint8_t frame[ARIADNA_FRAME_MAX];
    const uint16_t n = ariadna_encode(frame, ACTCON_PKT_PYRO_CMD, &c, sizeof(c));
    for (uint8_t i = 0; i < repeats; i++) NAV_PORT.write(frame, n);
}

static void nav_service(void)
{
    while (NAV_PORT.available()) {
        const uint8_t *pl; uint8_t len;
        const uint8_t type = ariadna_rx_byte(&s_nav_rx, (uint8_t)NAV_PORT.read(), &pl, &len);
        if (type == ACTCON_PKT_PYRO_REPORT && ariadna_unpack(pl, len, &s_nav, sizeof(s_nav))) {
            if (!s_nav_ok) Serial.println("# Navigation MCU (Pyro_Companion) answering");
            s_nav_last = millis();
            s_nav_ok   = true;
        } else if (type == ARIADNA_PKT_NAV_STATUS || type == ARIADNA_PKT_ACTUATOR) {
            static uint32_t warned = 0;
            if (millis() - warned > 5000) {
                warned = millis();
                Serial.println("# Navigation MCU runs FLIGHT firmware - flash Examples/Pyro_Companion");
            }
        }
    }
    if (s_nav_ok && millis() - s_nav_last > NAV_TIMEOUT_MS) {
        s_nav_ok = false;
        memset(&s_nav, 0, sizeof(s_nav));
        Serial.println("# Navigation MCU silent");
    }
}

// =====================================================================
//  sensors
// =====================================================================
/** ACS712, every millisecond, summed; the log takes the mean per sample. */
static void current_service(void)
{
    static uint32_t next_us = 0;
    const uint32_t t = micros();
    if ((int32_t)(t - next_us) < 0) return;
    next_us = t + 1000;
    const float pin = (float)analogRead(PIN_I_SENSE) * (ADC_VREF / ADC_FULL_SCALE);
    s_i_acc += (pin / ACS712_DIVIDER - ACS712_ZERO_V) / ACS712_V_PER_A;
    if (++s_i_n >= 1000) {                 // not logging: keep the sum bounded
        s_i_last = s_i_acc / (float)s_i_n;
        s_i_acc = 0.0f; s_i_n = 0;
    }
}

static int16_t current_take_ma(void)
{
    if (s_i_n) { s_i_last = s_i_acc / (float)s_i_n; }
    s_i_acc = 0.0f; s_i_n = 0;
    float ma = s_i_last * 1000.0f;
    if (ma > 32000.0f) ma = 32000.0f;
    if (ma < -32000.0f) ma = -32000.0f;
    return (int16_t)ma;
}

static void sense_service(void)
{
    static float vf = -1.0f;
    const float v = (float)analogRead(PIN_V_BAT) * (ADC_VREF / ADC_FULL_SCALE) * VBAT_DIVIDER;
    vf = vf < 0.0f ? v : vf + 0.05f * (v - vf);
    s_vbat = vf;
#if RBF_REMOVED_IS_HIGH
    s_rbf_out = digitalRead(PIN_RBF) == HIGH;
#else
    s_rbf_out = digitalRead(PIN_RBF) == LOW;
#endif
}

// =====================================================================
//  W25Q128 flash — the top 1 MB, see sf_record.h
// =====================================================================
static const uint8_t W25_READ_ID    = 0x9F;
static const uint8_t W25_READ_DATA  = 0x03;
static const uint8_t W25_PAGE_PROG  = 0x02;
static const uint8_t W25_SECT_ERASE = 0x20;
static const uint8_t W25_WRITE_EN   = 0x06;
static const uint8_t W25_READ_SR1   = 0x05;
static const uint32_t FLASH_SIZE    = 16UL * 1024UL * 1024UL;
static const uint32_t FLASH_SECTOR  = 4096;

static SPISettings s_flash_spi(20000000, MSBFIRST, SPI_MODE0);

static inline void flash_select(void)   { digitalWrite(PIN_FLASH_CS, LOW); }
static inline void flash_deselect(void) { digitalWrite(PIN_FLASH_CS, HIGH); }

static void flash_cmd_addr(uint8_t cmd, uint32_t addr)
{
    SPI1.transfer(cmd);
    SPI1.transfer((uint8_t)(addr >> 16));
    SPI1.transfer((uint8_t)(addr >> 8));
    SPI1.transfer((uint8_t)addr);
}

static bool flash_busy(void)
{
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    SPI1.transfer(W25_READ_SR1);
    const uint8_t sr = SPI1.transfer(0x00);
    flash_deselect();
    SPI1.endTransaction();
    return (sr & 0x01) != 0;
}

static bool flash_wait(uint32_t timeout_ms)
{
    const uint32_t t0 = millis();
    while (flash_busy()) if (millis() - t0 > timeout_ms) return false;
    return true;
}

static void flash_write_enable(void)
{
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    SPI1.transfer(W25_WRITE_EN);
    flash_deselect();
    SPI1.endTransaction();
}

static void flash_read(uint32_t addr, uint8_t *buf, uint32_t n)
{
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    flash_cmd_addr(W25_READ_DATA, addr);
    for (uint32_t i = 0; i < n; i++) buf[i] = SPI1.transfer(0x00);
    flash_deselect();
    SPI1.endTransaction();
}

static void flash_page_program(uint32_t addr, const uint8_t *buf)
{
    flash_write_enable();
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    flash_cmd_addr(W25_PAGE_PROG, addr);
    for (uint32_t i = 0; i < AFR_PAGE; i++) SPI1.transfer(buf[i]);
    flash_deselect();
    SPI1.endTransaction();
}

static void flash_sector_erase(uint32_t addr)
{
    flash_write_enable();
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    flash_cmd_addr(W25_SECT_ERASE, addr);
    flash_deselect();
    SPI1.endTransaction();
}

static bool flash_begin(void)
{
    pinMode(PIN_FLASH_CS, OUTPUT);
    flash_deselect();
    SPI1.setRX(PIN_FLASH_MISO);
    SPI1.setTX(PIN_FLASH_MOSI);
    SPI1.setSCK(PIN_FLASH_SCK);
    SPI1.begin();
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    SPI1.transfer(W25_READ_ID);
    const uint8_t mfr = SPI1.transfer(0x00);
    SPI1.transfer(0x00);
    const uint8_t cap = SPI1.transfer(0x00);
    flash_deselect();
    SPI1.endTransaction();
    Serial.printf("# flash JEDEC %02X ..%02X\n", mfr, cap);
    return mfr == 0xEF && cap >= 0x14 && cap <= 0x19;       // any Winbond of a plausible size
}

/** Program one page and read it back. A page that did not land is worth
 *  knowing about now, not when somebody looks at the data. */
static bool flash_write_page(uint32_t addr, const uint8_t *buf)
{
    flash_page_program(addr, buf);
    if (!flash_wait(20)) return false;
    uint8_t back[AFR_PAGE];
    flash_read(addr, back, AFR_PAGE);
    return memcmp(back, buf, AFR_PAGE) == 0;
}

static bool flash_blank(uint32_t addr, uint32_t bytes)
{
    uint8_t buf[AFR_PAGE];
    for (uint32_t a = 0; a < bytes; a += AFR_PAGE) {
        flash_read(addr + a, buf, AFR_PAGE);
        for (uint32_t i = 0; i < AFR_PAGE; i++) if (buf[i] != 0xFF) return false;
    }
    return true;
}

static void erase_queue(uint32_t first_sector, uint32_t n_sectors)
{
    s_erase_pos  = first_sector;
    s_erase_end  = first_sector + n_sectors;
    s_slot_ready = false;
}

/** One sector at a time while idle; the chip is busy ~45 ms per sector. */
static void erase_service(void)
{
    if (!s_flash_ok || s_slot_ready) return;
    const uint32_t now = millis();
    if ((int32_t)(now - s_erase_next_ms) < 0) return;
    s_erase_next_ms = now + 5;
    if (flash_busy()) return;
    if (s_erase_pos < s_erase_end) {
        flash_sector_erase(s_erase_pos * FLASH_SECTOR);
        s_erase_pos++;
        return;
    }
    s_slot_ready = true;
    Serial.printf("# slot %lu erased, ready for run %lu\n", (unsigned long)s_slot,
                  (unsigned long)s_run_id_next);
}

/** Read every slot's header: which run is next, which slot, is it blank. */
static void slots_scan(void)
{
    uint32_t best = 0;
    afr_header_t h;
    for (uint32_t s = 0; s < AFR_SLOTS; s++) {
        flash_read(afr_slot_addr(s), h.raw, AFR_PAGE);
        if (afr_header_ok(&h) && h.f.run_id > best) best = h.f.run_id;
    }
    s_run_id_next = best + 1;
    s_slot = (s_run_id_next - 1) % AFR_SLOTS;
    if (flash_blank(afr_slot_addr(s_slot), AFR_SLOT_BYTES)) {
        s_slot_ready = true;
    } else {
        erase_queue(afr_slot_addr(s_slot) / FLASH_SECTOR, AFR_SECTORS_PER_SLOT);
    }
    Serial.printf("# stored runs up to #%lu, next run #%lu goes to slot %lu (%s)\n",
                  (unsigned long)best, (unsigned long)s_run_id_next, (unsigned long)s_slot,
                  s_slot_ready ? "blank" : "erasing");
}

// =====================================================================
//  The stand, over I2C. The PCA9685 shares this bus: single threaded.
// =====================================================================
static void i2c_drain(void) { while (Wire.available()) (void)Wire.read(); }

static bool stand_read(uint8_t reg, uint8_t *buf, size_t n, uint32_t *t_read_us)
{
    Wire.beginTransmission(SFL_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    const uint32_t t = micros();
    if (t_read_us) *t_read_us = t;
    if ((size_t)Wire.requestFrom((int)SFL_I2C_ADDR, (int)n) != n) { i2c_drain(); return false; }
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

static bool stand_poll(void)
{
    uint8_t b[SFL_STATUS_SIZE];
    uint32_t t;
    if (!stand_read(SFL_REG_STATUS, b, sizeof(b), &t)) return false;
    sfl_status_t st;
    if (!sfl_status_decode(b, &st)) return false;
    s_st       = st;
    s_st_us    = t + 40;                    // the stand stamps t_rel as the read begins: ~40 us after this
    s_st_ms    = millis();
    s_st_valid = true;
    s_st_new   = true;
    return true;
}

static bool stand_poll_info(void)
{
    uint8_t b[SFL_INFO_SIZE];
    if (!stand_read(SFL_REG_INFO, b, sizeof(b), nullptr)) return false;
    sfl_info_t i;
    if (!sfl_info_decode(b, &i)) return false;
    s_info = i;
    s_info_valid = true;
    s_info_ms = millis();
    s_proto_bad = (i.proto != SFL_PROTO_VERSION);
    return true;
}

static void stand_send_hb(uint8_t flags, uint8_t ack, uint8_t data)
{
    sfl_hb_t h;
    h.flags = flags; h.ack_seq = ack; h.data_seq = data;
    uint8_t b[SFL_HB_SIZE];
    sfl_hb_encode(&h, b);
    Wire.beginTransmission(SFL_I2C_ADDR);
    Wire.write((uint8_t)SFL_REG_HEARTBEAT);
    Wire.write(b, SFL_HB_SIZE);
    (void)Wire.endTransmission();
    s_hb_flags = flags; s_hb_ack = ack; s_hb_data = data;
    s_hb_ms = millis();
}

/** One 64 byte chunk of the stand's last burn. 0 = ok, >0 = the stand's code, <0 = bus / CRC. */
static int stand_read_chunk(uint16_t page, uint8_t chunk, uint8_t *dst)
{
    uint8_t req[SFL_PAGE_REQ_SIZE];
    sfl_page_req_encode(page, chunk, req);
    Wire.beginTransmission(SFL_I2C_ADDR);
    Wire.write(req, SFL_PAGE_REQ_SIZE);
    if (Wire.endTransmission() != 0) return -1;
    if ((size_t)Wire.requestFrom((int)SFL_I2C_ADDR, (int)SFL_CHUNK_REPLY) != SFL_CHUNK_REPLY) {
        i2c_drain();
        return -1;
    }
    uint8_t rep[SFL_CHUNK_REPLY];
    for (size_t i = 0; i < sizeof(rep); i++) rep[i] = (uint8_t)Wire.read();
    uint8_t code;
    if (!sfl_chunk_reply_decode(rep, &code, dst)) return -2;
    return code;
}

static bool stand_ok_now(uint32_t now_ms)
{
    return s_st_valid && (uint32_t)(now_ms - s_st_ms) < SFR_LOST_MS;
}

// =====================================================================
//  readiness
// =====================================================================
#define PR_PCA    0x01
#define PR_FLASH  0x02
#define PR_SLOT   0x04
#define PR_NAV    0x08
#define PR_RBF    0x10
#define PR_CONT   0x20
#define PR_PROTO  0x40
#define PR_BUSY   0x80

static uint8_t prereq_missing(void)
{
    uint8_t m = 0;
    if (!s_pca_ok)                         m |= PR_PCA;
    if (!s_flash_ok)                       m |= PR_FLASH;
    else if (!s_slot_ready)                m |= PR_SLOT;
    if (!s_nav_ok)                         m |= PR_NAV;
#if REQUIRE_RBF
    if (!s_rbf_out)                        m |= PR_RBF;
#endif
#if REQUIRE_CONT
    if (!(s_nav_ok && (s_nav.cont_mask & 1u))) m |= PR_CONT;
#endif
    if (s_proto_bad)                       m |= PR_PROTO;
    if (s_busy)                            m |= PR_BUSY;
    return m;
}

static void print_missing(uint8_t m)
{
    if (!m) { Serial.println("# local checks all fine"); return; }
    Serial.printf("# not ready:%s%s%s%s%s%s%s%s\n",
        (m & PR_PCA)   ? " PCA9685(clamp)" : "",
        (m & PR_FLASH) ? " flash-chip" : "",
        (m & PR_SLOT)  ? " flash-slot-erasing" : "",
        (m & PR_NAV)   ? " Navigation-MCU(Pyro_Companion)" : "",
        (m & PR_RBF)   ? " RBF-pin-in" : "",
        (m & PR_CONT)  ? " no-igniter-on-pyro1" : "",
        (m & PR_PROTO) ? " stand-protocol-version" : "",
        (m & PR_BUSY)  ? " busy" : "");
}

// =====================================================================
//  copying the stand's record and writing Ariadna's
// =====================================================================
struct RowSink {
    afr_row_t rows[AFR_ROWS_PER_PAGE];
    uint16_t  n;
    uint16_t  page_idx;
    uint32_t  total;
    uint32_t  addr;
    uint32_t  run_id;
    bool      ok, overflow;
};

static void sink_flush(RowSink *k)
{
    if (!k->n) return;
    afr_rowpage_t pg;
    afr_rowpage_build(&pg, k->run_id, k->page_idx, k->rows, k->n);
    if (!flash_write_page(k->addr + (AFR_ROWS_FIRST + k->page_idx) * AFR_PAGE, pg.raw)) k->ok = false;
    k->page_idx++;
    k->n = 0;
}

static void sink_emit(void *ctx, const afr_row_t *row)
{
    RowSink *k = (RowSink *)ctx;
    if (!k->ok) return;
    if (k->page_idx >= AFR_ROWS_MAX_PAGES) { k->ok = false; k->overflow = true; return; }
    k->rows[k->n++] = *row;
    k->total++;
    if (k->n == AFR_ROWS_PER_PAGE) sink_flush(k);
}

/** Keep the stand's heartbeat going during a long job on this thread. */
static void keep_link_alive(void)
{
    const uint32_t now = millis();
    if (now - s_hb_ms >= 40) stand_send_hb(s_hb_flags | SFL_A_BUSY, s_hb_ack, s_hb_data);
    nav_service();
}

/**
 * Copy the stand's last burn and store it, merged with our own samples.
 * `own_valid`: our samples belong to this burn. Returns true when the slot
 * has been written and its header reads back.
 */
static bool copy_and_store(bool own_valid)
{
    s_busy = true;
    stand_send_hb(SFL_A_BUSY | (s_hb_flags & SFL_A_READY), s_hb_ack, s_hb_data);
    const uint32_t t_start = millis();

    stand_poll_info();                                        // burn id
    const uint16_t total = s_st.pages;
    if (total == 0) { Serial.println("# copy: the stand has no finished burn"); s_busy = false; return false; }
    const uint16_t kept = total > AFR_RAW_MAX ? AFR_RAW_MAX : total;
    uint32_t flags = 0;
    if (total > kept) flags |= AFR_H_STAND_PARTIAL;

    // ---- 1. the stand's pages, verified, with retries ------------------------
    uint16_t bad = 0;
    for (uint16_t p = 0; p < kept; p++) {
        bool got = false;
        for (uint8_t attempt = 0; attempt < 6 && !got; attempt++) {
            bool chunks_ok = true;
            for (uint8_t c = 0; c < SFL_CHUNKS_PER_PAGE && chunks_ok; c++) {
                const int code = stand_read_chunk(p, c, s_raw + (size_t)p * AFR_PAGE + (size_t)c * SFL_CHUNK);
                if (code == SFL_PG_BUSY) { delay(20); chunks_ok = false; }      // stand not idle yet
                else if (code != 0)      { chunks_ok = false; }
            }
            if (chunks_ok && afr_stand_page_type(s_raw + (size_t)p * AFR_PAGE) != AFR_PG_BAD) got = true;
            keep_link_alive();
        }
        if (!got) {
            bad++;
            memset(s_raw + (size_t)p * AFR_PAGE, 0xFF, AFR_PAGE);
        }
    }
    if (bad) {
        flags |= AFR_H_STAND_PARTIAL;
        Serial.printf("# copy: %u of %u pages could not be read cleanly\n", bad, kept);
    }
    if (bad == kept) { Serial.println("# copy failed: nothing readable"); s_busy = false; return false; }

    afr_stand_t sc;
    afr_stand_scan(s_raw, kept, &sc);
    if (!sc.has_header || !sc.has_footer || !sc.clean) flags |= AFR_H_STAND_PARTIAL;
    if (!(flags & AFR_H_STAND_PARTIAL)) flags |= AFR_H_STAND_OK;

    // ---- 2. write: raw pages, merged rows, header LAST ----------------------
    const uint32_t base = afr_slot_addr(s_slot);
    bool ok = true;
    for (uint16_t p = 0; p < kept && ok; p++) {
        ok = flash_write_page(base + (AFR_RAW_FIRST + p) * AFR_PAGE, s_raw + (size_t)p * AFR_PAGE);
        if (!(p & 7)) keep_link_alive();
    }

    RowSink sink;
    memset(&sink, 0, sizeof(sink));
    sink.addr = base; sink.run_id = s_run_id_next; sink.ok = ok;
    const uint32_t n_own = own_valid ? s_own_n : 0;
    const uint32_t rows = afr_merge(s_raw, kept, s_own, n_own, sink_emit, &sink);
    sink_flush(&sink);
    (void)rows;
    ok = sink.ok;
    if (sink.overflow) { flags |= AFR_H_STAND_PARTIAL; Serial.println("# copy: more rows than a slot holds, the tail was dropped"); }
    if (!n_own) flags |= AFR_H_NO_OWN;
    if (own_valid && !s_r.fired) flags |= AFR_H_NOT_FIRED;
    if (s_r.flaps.phase == SFF_FULL) flags |= AFR_H_PROFILE_DONE;
    if (s_own_flags & AFR_H_STAND_LOST) flags |= AFR_H_STAND_LOST;

    if (ok) {
        afr_header_t h;
        memset(h.raw, 0, sizeof(h.raw));
        h.f.run_id            = s_run_id_next;
        h.f.stand_burn_id     = sc.has_header ? sc.burn_id : s_info.burn_id;
        h.f.stand_pages       = kept;
        h.f.stand_pages_total = total;
        h.f.rows_first        = AFR_ROWS_FIRST;
        h.f.rows_pages        = sink.page_idx;
        h.f.n_rows            = sink.total;
        h.f.flags             = flags;
        h.f.cal_counts_per_n  = sc.cal_counts_per_n;
        h.f.tare_offset       = sc.tare_offset;
        h.f.t_fire_us         = own_valid ? s_r.t_fire_us : SFL_T_NONE;
        h.f.t_trigger_us      = own_valid ? s_r.t_trig_us : SFL_T_NONE;
        for (int i = 0; i < 4; i++) h.f.t_phase_us[i] = own_valid ? s_r.t_phase_us[i] : SFL_T_NONE;
        h.f.ign_ms            = s_cfg.ign_ms;
        h.f.step_ms           = (uint16_t)(s_cfg.flaps.step_us / 1000u);
        h.f.trigger_n_x10     = (uint16_t)(s_cfg.flaps.trigger_n * 10.0f);
        h.f.us_closed         = (uint16_t)CLAMP_US_CLOSED;
        h.f.us_open           = (uint16_t)CLAMP_US_OPEN;
        for (int i = 0; i < 4; i++) h.f.open_pct_x10[i] = (uint16_t)(s_cfg.flaps.open_pct[i] * 10.0f + 0.5f);
        h.f.n_own             = (uint16_t)(n_own > 65535u ? 65535u : n_own);
        afr_header_seal(&h);
        ok = flash_write_page(base, h.raw);
    }

    if (!ok) {
        Serial.println("# copy: WRITING THE FLASH FAILED - the stand still has the data");
        s_busy = false;
        return false;
    }

    Serial.printf("# run #%lu stored in slot %lu: %lu rows, %u stand pages, %lu own samples, "
                  "%lu ms (%s)\n",
                  (unsigned long)s_run_id_next, (unsigned long)s_slot, (unsigned long)sink.total,
                  kept, (unsigned long)n_own, (unsigned long)(millis() - t_start),
                  (flags & AFR_H_STAND_OK) ? "complete" : "PARTIAL");

    s_data_seq = s_st.run_seq;                                // tell the stand: you may go on
    s_run_id_next++;
    s_slot = (s_run_id_next - 1) % AFR_SLOTS;
    erase_queue(afr_slot_addr(s_slot) / FLASH_SECTOR, AFR_SECTORS_PER_SLOT);
    s_busy = false;
    s_own_n = 0;
    s_own_flags = 0;
    sfr_to_idle(&s_r, millis());
    stand_send_hb(s_hb_flags & ~SFL_A_BUSY, s_hb_ack, s_data_seq);
    return true;
}

// =====================================================================
//  dumping what is stored
// =====================================================================
static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void print_base64_page(const uint8_t *data)
{
    static char out[AFR_PAGE / 3 * 4 + 8];
    size_t o = 0;
    for (size_t i = 0; i < AFR_PAGE; i += 3) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i + 1 < AFR_PAGE) v |= (uint32_t)data[i + 1] << 8;
        if (i + 2 < AFR_PAGE) v |= data[i + 2];
        out[o++] = B64[(v >> 18) & 63];
        out[o++] = B64[(v >> 12) & 63];
        out[o++] = (i + 1 < AFR_PAGE) ? B64[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < AFR_PAGE) ? B64[v & 63] : '=';
    }
    out[o++] = '\n';
    Serial.write((const uint8_t *)out, o);
}

static bool read_header(uint32_t slot, afr_header_t *h)
{
    flash_read(afr_slot_addr(slot), h->raw, AFR_PAGE);
    return afr_header_ok(h);
}

static void list_runs(void)
{
    for (uint32_t s = 0; s < AFR_SLOTS; s++) {
        afr_header_t h;
        if (!read_header(s, &h)) { Serial.printf("slot %lu: empty\n", (unsigned long)s); continue; }
        Serial.printf("slot %lu: run #%lu  stand burn #%lu  rows=%lu  stand pages=%u/%u  flags=0x%02lX%s%s\n",
            (unsigned long)s, (unsigned long)h.f.run_id, (unsigned long)h.f.stand_burn_id,
            (unsigned long)h.f.n_rows, h.f.stand_pages, h.f.stand_pages_total,
            (unsigned long)h.f.flags,
            (h.f.flags & AFR_H_STAND_OK) ? "  complete" : "  PARTIAL",
            (h.f.flags & AFR_H_NOT_FIRED) ? "  NOT-FIRED-BY-ARIADNA" : "");
    }
}

/** One slot: a CSV of the merged table, then the stand's pages in the stand's own dump format. */
static void dump_slot(uint32_t slot)
{
    afr_header_t h;
    if (!read_header(slot, &h)) return;
    const uint32_t base = afr_slot_addr(slot);
    const float cal = h.f.cal_counts_per_n != 0.0f ? h.f.cal_counts_per_n : 1.0f;

    Serial.printf("#BEGIN ARIADNA_CSV run=%lu stand_burn=%lu\n", (unsigned long)h.f.run_id,
                  (unsigned long)h.f.stand_burn_id);
    Serial.println("t_s,thrust_N,vbat_V,current_A,servo_us,closure_pct,phase,pyro,cont,own_data");
    uint32_t left = h.f.n_rows;
    for (uint16_t p = 0; p < h.f.rows_pages && left; p++) {
        afr_rowpage_t pg;
        flash_read(base + (h.f.rows_first + p) * AFR_PAGE, pg.raw, AFR_PAGE);
        if (!afr_rowpage_ok(&pg)) { Serial.printf("# rows page %u failed its CRC\n", p); continue; }
        for (uint16_t k = 0; k < pg.f.n_rows && left; k++, left--) {
            const afr_row_t &r = pg.f.rows[k];
            int32_t raw = (int32_t)(r.packed & 0x00FFFFFFu);
            if (raw & 0x800000) raw -= 0x1000000;
            const uint8_t fl = (uint8_t)(r.packed >> 24);
            Serial.printf("%.4f,%.3f,%.2f,%.3f,%u,%.1f,%u,%u,%u,%u\n",
                (double)r.t_us * 1e-6, (double)(raw - h.f.tare_offset) / (double)cal,
                (double)r.vbat_mv * 1e-3, (double)r.current_ma * 1e-3, r.servo_us,
                (double)r.closed_pm * 0.1, (fl >> AFR_ROW_PHASE_SHIFT) & 7u,
                fl & 1u, (fl >> 1) & 1u, (fl & AFR_ROW_NODATA) ? 0u : 1u);
        }
    }
    Serial.println("#END ARIADNA_CSV");

    Serial.printf("#SLOT %lu pages=%u\n", (unsigned long)slot, h.f.stand_pages);
    for (uint16_t p = 0; p < h.f.stand_pages; p++) {
        uint8_t page[AFR_PAGE];
        flash_read(base + (AFR_RAW_FIRST + p) * AFR_PAGE, page, AFR_PAGE);
        print_base64_page(page);
    }
    Serial.printf("#ENDSLOT %lu\n", (unsigned long)slot);
}

static void dump_runs(int only)
{
    Serial.println("#BEGIN SFDUMP v2");
    Serial.printf("#INFO {\"source\":\"ariadna\",\"slots\":%lu,\"next_run\":%lu}\n",
                  (unsigned long)AFR_SLOTS, (unsigned long)s_run_id_next);
    for (uint32_t s = 0; s < AFR_SLOTS; s++) {
        if (only >= 0 && (uint32_t)only != s) continue;
        dump_slot(s);
    }
    Serial.println("#END SFDUMP");
}

// =====================================================================
//  logging our own samples during the run
// =====================================================================
static void log_service(uint32_t now_us)
{
    static uint32_t next_us = 0;
    if (!s_out.log_on || !s_out.t_valid) { next_us = now_us; return; }
    if ((int32_t)(now_us - next_us) < 0) return;
    next_us += LOG_PERIOD_US;
    if ((int32_t)(now_us - next_us) > 0) next_us = now_us + LOG_PERIOD_US;    // fell behind: do not burst
    if (s_own_n >= OWN_MAX) return;

    afr_sample_t &o = s_own[s_own_n++];
    memset(&o, 0, sizeof(o));
    o.t_us       = s_out.t_us;
    o.vbat_mv    = (uint16_t)(s_vbat * 1000.0f);
    o.current_ma = current_take_ma();
    o.servo_us   = s_clamp_us;
    o.closed_pm  = sff_closed_permille(s_open_pct);
    o.phase      = s_out.flap_phase;
    const int pyro = s_r.fired && s_out.t_us >= s_r.t_fire_us &&
                     (s_out.t_us - s_r.t_fire_us) < (int32_t)s_cfg.ign_ms * 1000;
    o.flags = (uint8_t)((pyro ? 1u : 0u) | ((s_nav.cont_mask & 1u) ? 2u : 0u) | (s_r.lost ? 4u : 0u));
    if (s_r.lost) s_own_flags |= AFR_H_STAND_LOST;
}

// =====================================================================
//  annunciators
// =====================================================================
static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    static uint32_t last = 0xFFFFFFFF;
    const uint32_t c = s_px.Color(r, g, b);
    if (c == last) return;
    last = c;
    s_px.setPixelColor(0, c);
    s_px.show();
}

static void annunciate(uint32_t now, bool stand_ok, bool ready)
{
    static bool chirping = false;
    const bool armed = s_nav_ok && s_nav.armed;
    const bool chirp = armed && (now % 1000) < 80;
    if (chirp != chirping) {
        chirping = chirp;
        if (chirp) tone(PIN_BUZZER, 2700); else noTone(PIN_BUZZER);
    }
    const uint8_t blink = (uint8_t)((now / 150) % 2);
    if (s_r.phase == SFR_BURN || (s_nav_ok && s_nav.firing_mask)) rgb(255, 0, 0);
    else if (armed)                                               rgb(blink ? 255 : 0, blink ? 110 : 0, 0);
    else if (!s_pca_ok)                                           rgb((now / 300) % 2 ? 80 : 0, 0, (now / 300) % 2 ? 0 : 80);
    else if (s_busy)                                              rgb(0, 70, 90);
    else if (s_r.fault)                                           rgb(blink ? 180 : 0, 0, blink ? 60 : 0);
    else if (!stand_ok)                                           rgb(0, 0, (now / 500) % 2 ? 40 : 0);
    else if (ready)                                               rgb(0, 45, 0);
    else                                                          rgb(60, 40, 0);
}

// =====================================================================
//  console
// =====================================================================
static void print_status_line(uint32_t now_ms)
{
    static const char *PH[] = { "idle", "arming", "ARMED", "BURN", "done" };
    static const char *ST[] = { "boot", "idle", "COUNTDOWN", "RECORDING", "fault", "dumping" };
    Serial.printf("stand: %s%s  thrust %.1f N  run %u | ariadna: %s  clamp %.0f%% open (%u us)  "
                  "nav %s%s  RBF %s  cont %s  flash %s | V %.2f  I %.2f A | rows stored: next run #%lu\n",
        stand_ok_now(now_ms) ? (s_st.state < 6 ? ST[s_st.state] : "?") : "NO LINK",
        (stand_ok_now(now_ms) && (s_st.flags & SFL_F_READY)) ? " (ready)" : "",
        stand_ok_now(now_ms) ? (double)s_st.thrust_mN * 0.001 : 0.0, s_st.run_seq,
        PH[s_r.phase > 4 ? 4 : s_r.phase], (double)s_open_pct, s_clamp_us,
        s_nav_ok ? "ok" : "SILENT", (s_nav_ok && s_nav.armed) ? " ARMED" : "",
        s_rbf_out ? "out" : "IN", (s_nav_ok && (s_nav.cont_mask & 1u)) ? "yes" : "NO",
        !s_flash_ok ? "NONE" : (s_slot_ready ? "ready" : "erasing"),
        (double)s_vbat, (double)s_i_last, (unsigned long)s_run_id_next);
}

static void print_info_json(uint32_t now_ms)
{
    Serial.printf("{\"fw\":\"static-fire-link\",\"proto\":%u,\"stand\":%d,\"stand_state\":%u,"
                  "\"stand_ready\":%d,\"stand_ignition_ms\":%u,\"phase\":%u,\"hold\":%d,"
                  "\"nav\":%d,\"armed\":%d,\"rbf_out\":%d,\"cont\":%d,\"pca\":%d,\"flash\":%d,"
                  "\"slot_ready\":%d,\"next_run\":%lu,\"clamp_open_pct\":%.1f,\"clamp_us\":%u,"
                  "\"vbat\":%.2f,\"i\":%.3f}\n",
        (unsigned)SFL_PROTO_VERSION, stand_ok_now(now_ms) ? 1 : 0, s_st.state,
        (s_st.flags & SFL_F_READY) ? 1 : 0, s_info.ignition_ms, s_r.phase, s_hold ? 1 : 0,
        s_nav_ok ? 1 : 0, (s_nav_ok && s_nav.armed) ? 1 : 0, s_rbf_out ? 1 : 0,
        (s_nav_ok && (s_nav.cont_mask & 1u)) ? 1 : 0, s_pca_ok ? 1 : 0, s_flash_ok ? 1 : 0,
        s_slot_ready ? 1 : 0, (unsigned long)s_run_id_next, (double)s_open_pct, s_clamp_us,
        (double)s_vbat, (double)s_i_last);
}

static void print_help(void)
{
    Serial.println("commands: s status | i json | l list runs | p [slot] dump | pull copy the stand's last burn");
    Serial.println("          hold / go  not ready / ready | wipe yes  erase all stored runs");
}

static void handle_line(char *line, uint32_t now_ms)
{
    char *save = nullptr;
    char *cmd = strtok_r(line, " \t", &save);
    if (!cmd) return;
    for (char *p = cmd; *p; p++) *p = (char)tolower((unsigned char)*p);
    char *a1 = strtok_r(nullptr, " \t", &save);

    if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) { print_help(); return; }
    if (!strcmp(cmd, "s"))    { print_status_line(now_ms); print_missing(prereq_missing()); return; }
    if (!strcmp(cmd, "i"))    { print_info_json(now_ms); return; }
    if (!strcmp(cmd, "l"))    { list_runs(); return; }
    if (!strcmp(cmd, "p"))    { dump_runs(a1 ? atoi(a1) : -1); return; }
    if (!strcmp(cmd, "hold")) { s_hold = true;  Serial.println("OK hold: NOT READY (a running countdown will stop)"); return; }
    if (!strcmp(cmd, "go"))   { s_hold = false; Serial.println("OK go"); return; }
    if (!strcmp(cmd, "pull")) {
        if (s_r.phase != SFR_IDLE)        { Serial.println("ERR a run is in progress"); return; }
        if (!stand_ok_now(now_ms))        { Serial.println("ERR no link to the stand"); return; }
        if (!s_slot_ready)                { Serial.println("ERR flash slot not ready yet"); return; }
        if (!stand_poll())                { Serial.println("ERR the stand did not answer"); return; }
        if (s_st.state != SFL_ST_IDLE)    { Serial.println("ERR the stand is not idle"); return; }
        copy_and_store(false);
        return;
    }
    if (!strcmp(cmd, "wipe")) {
        if (!a1 || strcmp(a1, "yes")) { Serial.println("ERR this erases every stored run: type  wipe yes"); return; }
        if (s_r.phase != SFR_IDLE || s_busy) { Serial.println("ERR a run is in progress"); return; }
        s_run_id_next = 1;
        s_slot = 0;
        erase_queue(AFR_REGION_BASE / FLASH_SECTOR, (AFR_SLOTS * AFR_SLOT_BYTES) / FLASH_SECTOR);
        Serial.println("OK erasing the whole region (about 15 s), NOT READY until it is done");
        return;
    }
    Serial.printf("ERR unknown '%s' (? for help)\n", cmd);
}

static void console_service(uint32_t now_ms)
{
    static char buf[48];
    static uint8_t len = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            buf[len] = 0;
            if (len) handle_line(buf, now_ms);
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
}

// =====================================================================
//  events from the supervisor
// =====================================================================
static void announce(uint16_t ev, uint32_t now_ms)
{
    (void)now_ms;
    if (ev & SFR_EV_COUNTDOWN) {
        s_own_n = 0; s_own_flags = 0;
        s_own_run_seq = s_r.run_seq;
        Serial.printf("# countdown seen (run %u): arming pyro 1\n", s_r.run_seq);
    }
    if (ev & SFR_EV_ARMED)       Serial.println("# pyro 1 ARMED, waiting for T0");
    if (ev & SFR_EV_ABORT)       Serial.println("# countdown ended without ignition: disarmed");
    if (ev & SFR_EV_ARM_TIMEOUT) Serial.println("# the pyro did not arm: NOT READY (the stand stops its countdown)");
    if (ev & SFR_EV_FIRE)        Serial.printf("# T0: FIRE pyro 1 (%u ms), %ld us after T0\n", s_out.fire_ms, (long)s_r.t_fire_us);
    if (ev & SFR_EV_LATE)        Serial.println("# IGNITE came too late (or the pyro was not armed): NOT fired");
    if (ev & SFR_EV_TRIGGER)     Serial.printf("# thrust over %.0f N: clamp %.0f %% open\n", (double)s_cfg.flaps.trigger_n, (double)s_cfg.flaps.open_pct[0]);
    if (ev & SFR_EV_STEP)        Serial.printf("# clamp step %u: %.0f %% open\n", s_out.flap_phase - 1, (double)s_out.flap_open_pct);
    if (ev & SFR_EV_STAND_LOST)  Serial.println("# LOST THE STAND during the burn: clamp fully open");
    if (ev & SFR_EV_FINISHED)    Serial.printf("# burn over: %lu own samples, copying the stand's record next\n", (unsigned long)s_own_n);
}

// =====================================================================
void setup()
{
    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH);                      // servo outputs off until configured
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RBF, INPUT_PULLUP);
    analogReadResolution(12);

    Serial.begin(115200);
    s_px.begin();
    s_px.clear();
    s_px.show();

    NAV_PORT.setTX(PIN_NAV_TX);
    NAV_PORT.setRX(PIN_NAV_RX);
    NAV_PORT.setFIFOSize(256);
    NAV_PORT.begin(ARIADNA_MCU_BAUD);
    ariadna_rx_init(&s_nav_rx);
    memset(&s_nav, 0, sizeof(s_nav));

    memset(&s_st, 0, sizeof(s_st));
    memset(&s_info, 0, sizeof(s_info));
    sfr_cfg_default(&s_cfg);
    s_cfg.flaps.trigger_n   = TRIGGER_N;
    s_cfg.flaps.step_us     = STEP_MS * 1000u;
    s_cfg.flaps.open_pct[0] = OPEN_PCT_1;
    s_cfg.flaps.open_pct[1] = OPEN_PCT_MIN;
    s_cfg.flaps.open_pct[2] = OPEN_PCT_3;
    s_cfg.flaps.open_pct[3] = 100.0f;
    sfr_init(&s_r);
    memset(&s_out, 0, sizeof(s_out));

    // the clamp starts OPEN
    s_open_pct = 100.0f;
    s_clamp_us = (uint16_t)(sff_pct_to_us(100.0f, CLAMP_US_CLOSED, CLAMP_US_OPEN) + 0.5f);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setTimeout(10, true);
    Wire.begin();
    Wire.setClock(400000);
    pca_try_start();

    s_flash_ok = flash_begin();
    if (s_flash_ok) slots_scan();
    else Serial.println("# NO W25Q128 flash: runs cannot be stored, NOT READY");

    Serial.printf("# Static Fire Link ready. Clamp %.0f us (open) .. %.0f us (shut); trigger %.0f N; "
                  "steps %.0f/%.0f/%.0f %% open, %lu ms each. Type ? for help.\n",
                  (double)CLAMP_US_OPEN, (double)CLAMP_US_CLOSED, (double)TRIGGER_N,
                  (double)OPEN_PCT_1, (double)OPEN_PCT_MIN, (double)OPEN_PCT_3, (unsigned long)STEP_MS);
}

void loop()
{
    static uint32_t next_poll_us = 0, next_pca = 0, next_nav_hb = 0, next_disarm = 0, next_arm = 0;
    static uint32_t next_copy_try = 0;
    static uint8_t  last_missing = 0xFF;
    const uint32_t now_ms = millis();
    const uint32_t now_us = micros();

    console_service(now_ms);
    nav_service();
    sense_service();
    current_service();

    // ---- keep the Navigation MCU's heartbeat up, always ------------------
    if ((int32_t)(now_ms - next_nav_hb) >= 0) {
        next_nav_hb = now_ms + ACTCON_HB_PERIOD_MS;
        nav_send(ACTCON_CMD_HEARTBEAT, 0, 0, 1);
    }

    // ---- the stand: poll at the rate the phase needs ------------------------
    sfr_in_t in;
    memset(&in, 0, sizeof(in));
    in.stand_ok = stand_ok_now(now_ms);
    in.st = s_st;
    in.now_ms = now_ms;
    in.now_us = now_us;
    in.prereq_ok = 1;
    if ((int32_t)(now_us - next_poll_us) >= 0) {
        const bool ok = stand_poll();
        const uint32_t period_ms = ok ? sfr_poll_ms(&s_r, &in) : 50u;     // an absent stand is not worth hammering
        next_poll_us = now_us + period_ms * 1000u;
        in.stand_ok = stand_ok_now(millis());
        in.st = s_st;
    }
    const bool stand_ok = in.stand_ok;
    if (stand_ok != s_stand_was_ok) {
        s_stand_was_ok = stand_ok;
        Serial.println(stand_ok ? "# stand link up" : "# stand link LOST");
        if (stand_ok) s_info_ms = 0;                       // read INFO again
    }
    if (stand_ok && s_r.phase == SFR_IDLE && (now_ms - s_info_ms) > 2000) {
        stand_poll_info();
        s_info_ms = millis();
        if (s_info_valid) {
            // The stand's setting is the one that counts. The Navigation MCU's own window still applies.
            s_cfg.ign_ms       = sfr_clamp_pulse_ms(s_info.ignition_ms ? s_info.ignition_ms : 300);
            s_cfg.recording_ms = s_info.recording_ms ? s_info.recording_ms : 10000;
            s_cfg.log_pre_us   = (uint32_t)s_info.preroll_ms * 1000u + 500000u;
        }
    }

    // ---- local prerequisites -------------------------------------------------
    const uint8_t missing = prereq_missing();
    if (missing != last_missing && stand_ok) { last_missing = missing; print_missing(missing); }
    if (!stand_ok) last_missing = 0xFF;

    in.prereq_ok  = (missing == 0);
    in.nav_armed  = s_nav_ok && s_nav.armed;
    in.cont       = s_nav_ok && (s_nav.cont_mask & 1u);
    in.busy       = s_busy;
    in.hold       = s_hold;
    in.st_new     = s_st_new;
    in.st_us      = s_st_us;
    s_st_new      = false;

    // ---- the supervisor ----------------------------------------------------------
    sfr_tick(&s_r, &s_cfg, &in, &s_out);

    if (s_out.fire) {                                      // the one thing that matters: first
        nav_send(ACTCON_CMD_FIRE, 1, s_out.fire_ms, 3);
        stand_send_hb(s_out.hb_flags, s_out.hb_ack, s_data_seq);   // the ack, at once
    }
    if (s_out.want_disarm) nav_send(ACTCON_CMD_DISARM, 0, 0, 3);
    if (s_out.want_arm && !in.nav_armed && (int32_t)(now_ms - next_arm) >= 0) {
        next_arm = now_ms + 150;
        nav_send(ACTCON_CMD_ARM, 0, 0, 3);
    }
    // Nothing should stay armed once the run is over; ARM also expires by itself.
    if (s_r.phase == SFR_IDLE && in.nav_armed && (int32_t)(now_ms - next_disarm) >= 0) {
        next_disarm = now_ms + 500;
        nav_send(ACTCON_CMD_DISARM, 0, 0, 1);
    }
    if (s_out.ev) announce(s_out.ev, now_ms);

    // ---- the clamp ---------------------------------------------------------------
    if (s_pca_ok && fabsf(s_out.flap_open_pct - s_open_pct) > 0.01f) clamp_set(s_out.flap_open_pct);

    log_service(now_us);

    // ---- the heartbeat to the stand: on change, and every SFL_HB_PERIOD_MS ----------
    {
        uint8_t flags = s_out.hb_flags | (s_busy ? SFL_A_BUSY : 0);
        if (stand_ok && (flags != s_hb_flags || s_out.hb_ack != s_hb_ack || s_data_seq != s_hb_data ||
                         (now_ms - s_hb_ms) >= SFL_HB_PERIOD_MS))
            stand_send_hb(flags, s_out.hb_ack, s_data_seq);
    }

    // ---- the record: copy it when the stand says it has one ---------------------------
    if (stand_ok && s_r.phase == SFR_IDLE && !s_busy && s_slot_ready &&
        (s_st.flags & SFL_F_DATA_READY) && s_st.state == SFL_ST_IDLE && s_data_seq != s_st.run_seq &&
        (int32_t)(now_ms - next_copy_try) >= 0) {
        next_copy_try = now_ms + 5000;                     // a failed copy is retried, not hammered
        const bool own = (s_own_n > 0) && (s_own_run_seq == s_st.run_seq);
        Serial.printf("# the stand has a finished burn: copying (%u pages)\n", s_st.pages);
        copy_and_store(own);
    }

    // ---- housekeeping ---------------------------------------------------------------------
    if (s_r.phase == SFR_IDLE && !s_busy) erase_service();

    if ((int32_t)(now_ms - next_pca) >= 0) {
        next_pca = now_ms + PCA_CHECK_MS;
        if (!s_pca_ok) pca_try_start();
        else {
            // A PCA9685 that lost power comes back asleep (MODE1 bit 4): re-initialise it,
            // which also puts the clamp back where it belongs.
            const int m = i2c_read8(0x00);
            if (m < 0 || (m & 0x10)) { pca_lost(m < 0 ? "no ACK" : "it reset (brown-out?)"); pca_try_start(); }
        }
    }

    annunciate(now_ms, stand_ok, in.prereq_ok && stand_ok && !s_hold);
}
