// =====================================================================
//  Ariadna V2 — MASTER MCU
//
//  Target: Raspberry Pi Pico 2 W (RP2350), arduino-pico core.
//  Needs:  LoRa (sandeepmistry), Adafruit_NeoPixel
//
//  Role: central hub and mission state machine.
//
//    * routes the navigation solution from the Position MCU to the
//      Navigation MCU, and the actuator commands back
//    * drives every servo in the vehicle through a PCA9685
//    * logs telemetry to a W25Q128 flash
//    * runs the LoRa downlink to the ground station
//    * owns arming, the RBF interlock, and the buzzer and RGB annunciators
//
//  It contains NO guidance mathematics. It does not decide when to ignite,
//  where to point the nozzle, or how much thrust to keep — that is the
//  Navigation MCU's job, and this board's only opinion about it is whether
//  the vehicle is armed at all.
//
//  ---------------------------------------------------------------------
//  CORE ALLOCATION
//  ---------------------------------------------------------------------
//    core 0   200 Hz: UART routing, PCA9685, state machine, ADC
//    core 1   flash logging and the LoRa downlink — everything that blocks
//
//  This split is the whole reason the design works. A 4 KB sector erase
//  takes up to 400 ms and a LoRa packet is ~92 ms on air. Either one on
//  core 0 would stall the actuator path for longer than the landing burn
//  lasts. Core 1 does the blocking; a RAM ring buffer absorbs it.
//
//  ---------------------------------------------------------------------
//  THE LATENCY THIS BOARD ADDS
//  ---------------------------------------------------------------------
//  The TVC servos hang off this board's PCA9685, but the 200 Hz loop that
//  commands them runs on the Navigation MCU. So every nozzle command
//  crosses a UART and an I2C bus before it reaches a servo:
//
//    guidance solve -> UART 460800, 42 B  ~0.9 ms
//                   -> core 0 tick             <= 5 ms
//                   -> PCA9685 I2C write   ~0.3 ms
//
//  Budget 3-6 ms end to end. At the guidance loop's 9 rad/s bandwidth that
//  is a few degrees of phase, which is acceptable — but it is only
//  acceptable because the servo path below does nothing else. Do not add
//  work between reading an actuator packet and writing the PCA9685.
//
//  The PCA9685 is driven at 339 Hz rather than the usual 50 Hz for the
//  same reason. A 50 Hz refresh adds up to 20 ms of dead time on its own,
//  which is more than the rest of the chain put together.
//
//  *** DIGITAL SERVOS ONLY. *** The PCA9685's PWM frequency is global
//  across all 16 channels. Every servo on this chip sees 339 Hz, and most
//  analogue servos will overheat or buzz at that rate.
// =====================================================================

#include "../common/ariadna_link.h"
#include "../common/ariadna_uart_dma.h"

// Vehicle constants come from the guidance module rather than being
// repeated here. The state machine's handover altitude MUST equal the
// altitude the Navigation MCU arms at, and the only way to guarantee that
// is to read it from the same file. `cd ../../test && make sync` keeps
// this copy identical to the verified original.
#include "../Navigation_MCU/guidance.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

#include <string.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE CONFIGURATION
// =====================================================================

// ---- inter-processor links ------------------------------------------
// On the arduino-pico core Serial1 is UART0 (GP0/GP1) and Serial2 is
// UART1 (GP4/GP5), which matches the pinout spreadsheet exactly.
#define POS_PORT   Serial1              // Position MCU
#define NAV_PORT   Serial2              // Navigation MCU
static const uint8_t PIN_POS_TX  = 0;
static const uint8_t PIN_POS_RX  = 1;
static const uint8_t PIN_NAV_TX  = 4;
static const uint8_t PIN_NAV_RX  = 5;

// ---- annunciators ----------------------------------------------------
static const uint8_t PIN_RGB     = 2;   // SK6812, PIO driven
static const uint8_t PIN_BUZZER  = 3;   // PWM

// ---- subordinate resets ---------------------------------------------
// Active LOW into each XIAO's RUN pin. Held high in normal operation.
static const uint8_t PIN_RST_POS = 6;
static const uint8_t PIN_RST_NAV = 7;

// ---- servo bus -------------------------------------------------------
static const uint8_t PIN_I2C_SDA = 8;   // I2C0 -> PCA9685
static const uint8_t PIN_I2C_SCL = 9;

// ---- launch detect ---------------------------------------------------
// Breakwire to the pad. Intact (on the pad) reads LOW through the internal
// pull-up being shorted to ground; it goes HIGH when the wire parts.
static const uint8_t PIN_BREAKWIRE = 10;

// ---- W25Q128 flash log, SPI1 ----------------------------------------
static const uint8_t PIN_FLASH_MISO = 12;
static const uint8_t PIN_FLASH_CS   = 13;
static const uint8_t PIN_FLASH_SCK  = 14;
static const uint8_t PIN_FLASH_MOSI = 15;

// ---- LoRa Ra-02 (SX1278, 433 MHz), SPI0 -----------------------------
static const uint8_t PIN_LORA_MISO = 16;
static const uint8_t PIN_LORA_CS   = 17;
static const uint8_t PIN_LORA_SCK  = 18;
static const uint8_t PIN_LORA_MOSI = 19;
static const uint8_t PIN_LORA_DIO0 = 20;
static const uint8_t PIN_LORA_RST  = 21;

// ---- auxiliary --------------------------------------------------------
// The pinout spreadsheet labels GP22 "S16". It is a plain GPIO, not a
// PCA9685 channel, so it is exposed here as a spare digital output — a
// 17th servo, a camera trigger, a recovery release. Nothing in the state
// machine drives it; wire it up and add the one line you need.
static const uint8_t PIN_AUX_S16 = 22;

// ---- analogue ---------------------------------------------------------
static const uint8_t PIN_I_SENSE = 26;  // ADC0, ACS712
static const uint8_t PIN_V_BAT   = 27;  // ADC1, 2S through a divider
static const uint8_t PIN_RBF     = 28;  // ADC2, Remove-Before-Flight

// CHECK THESE AGAINST YOUR BOARD. Every one is a property of the divider
// you fitted, not of the software.
static const float ADC_VREF        = 3.30f;
static const float ADC_FULL_SCALE  = 4095.0f;
static const float VBAT_DIVIDER    = 3.00f;    // e.g. 20k over 10k
static const float ACS712_ZERO_V   = 1.65f;    // output at 0 A, after divider
static const float ACS712_V_PER_A  = 0.100f;   // 20 A part: 100 mV/A
static const float ACS712_DIVIDER  = 1.00f;    // 1.0 if the part runs at 3V3

// RBF: pin pulled to ground while the tag is fitted. Removing it lets the
// pull-up take the pin high, which is the ARMED-permissive state. Fail
// safe: a broken wire reads LOW, which inhibits arming.
#define RBF_REMOVED_IS_HIGH  1

// ---- PCA9685 ----------------------------------------------------------
static const uint8_t PCA_ADDR = 0x40;

// The internal oscillator is nominally 25 MHz and varies part to part.
// If your measured pulse widths are off by a few percent, trim this rather
// than the pulse tables — it is the actual cause.
static const float   PCA_OSC_HZ   = 25000000.0f;
static const float   PCA_PWM_HZ   = 339.0f;    // see the header note

// Channel map.
enum {
    CH_TVC_X   = 0,
    CH_TVC_Y   = 1,
    CH_CLAMP   = 2,
    CH_FIN_0   = 3,     // FUTURE
    CH_FIN_1   = 4,
    CH_FIN_2   = 5,
    CH_FIN_3   = 6,
    CH_SPARE   = 7,
    PCA_N_USED = 8
};

// ---- servo calibration ------------------------------------------------
//
// TVC_TRIM_US centres the NOZZLE, not the servo horn. Set it with the
// motor installed and the nozzle visibly straight.
//
// TVC_SIGN_* makes a POSITIVE command from the Navigation MCU tilt the
// vehicle toward pad +x / +y. Get either backwards and the attitude loop
// becomes positive feedback and diverges in well under a second. Bench
// test 3 in the README exists to catch exactly this.
static const float TVC_TRIM_X_US   = 1500.0f;
static const float TVC_TRIM_Y_US   = 1500.0f;
static const float TVC_US_PER_DEG  = 10.0f;
static const float TVC_SIGN_X      = +1.0f;
static const float TVC_SIGN_Y      = +1.0f;

// Thrust clamp: pulse widths at fully diverted and fully open. Calibrate
// these against MEASURED thrust in a static fire, not by eye.
static const float CLAMP_US_CLOSED = 1150.0f;
static const float CLAMP_US_OPEN   = 1900.0f;

static const float FIN_TRIM_US     = 1500.0f;
static const float FIN_US_PER_DEG  = 10.0f;

// Absolute travel guard. A bad command must not stall a servo against its
// end stop and burn out on the pad.
static const float SERVO_US_MIN    = 900.0f;
static const float SERVO_US_MAX    = 2100.0f;

// ---- flash log --------------------------------------------------------
static const uint32_t FLASH_SIZE     = 16UL * 1024UL * 1024UL;  // W25Q128
static const uint32_t FLASH_SECTOR   = 4096;
static const uint32_t FLASH_PAGE     = 256;
static const uint32_t LOG_HZ         = 50;

// ---- link supervision -------------------------------------------------
static const uint32_t LINK_TIMEOUT_MS = 300;

// ---- state machine ----------------------------------------------------
// Handover to the Navigation MCU happens at the altitude it arms at. Read
// from guidance.h so the two can never disagree.
static const float GUIDANCE_ENTRY_M = DG_H_START;

#define MASTER_PRINT(...)                                         \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        Serial.printf(__VA_ARGS__);                               \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

// =====================================================================
//  2. STATE
// =====================================================================
static ariadna_rx_t          s_rx_pos;
static ariadna_rx_t          s_rx_nav;

static ariadna_nav_state_t   s_nav;          // newest from Position
static ariadna_actuator_t    s_act;          // newest from Navigation
static ariadna_nav_status_t  s_nav_status;
static ariadna_pos_status_t  s_pos_status;

static uint32_t s_nav_us = 0, s_act_us = 0;
static bool     s_have_nav = false, s_have_act = false;
static bool     s_have_nav_status = false, s_have_pos_status = false;

static volatile uint8_t s_state = ARIADNA_STATE_BOOT;   // core 1 reads this
static uint8_t  s_flags       = 0;
static uint32_t s_t_launch_ms = 0;
static uint32_t s_state_since = 0;

static float    s_vbat        = 0.0f;
static float    s_current     = 0.0f;
static bool     s_rbf_out     = false;

static volatile bool s_flash_ok = false;                // core 1 reads this
static volatile bool s_lora_ok  = false;                // core 1 owns the radio

// Downlink accounting. A slot skipped because the previous packet was
// still on air is not an error — it is the guard that keeps the link from
// falling behind — but if `skipped` climbs faster than `sent`, the
// configured rate does not fit the airtime and ARIADNA_TLM_HZ is too high.
static volatile uint32_t s_tlm_sent    = 0;
static volatile uint32_t s_tlm_skipped = 0;
static bool     s_servo_ok    = false;
static bool     s_post_pass   = false;

Adafruit_NeoPixel s_rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

// =====================================================================
//  3. PCA9685 — servo bus
//
//  Written out rather than pulled from a library. It is four registers,
//  and the servo path is the most latency-sensitive thing this board does:
//  worth knowing exactly what goes on the wire.
// =====================================================================
static const uint8_t PCA_MODE1      = 0x00;
static const uint8_t PCA_MODE2      = 0x01;
static const uint8_t PCA_LED0_ON_L  = 0x06;
static const uint8_t PCA_PRESCALE   = 0xFE;

static bool pca_write8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool pca_read8(uint8_t reg, uint8_t *val)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return false;
    *val = (uint8_t)Wire.read();
    return true;
}

static bool pca_begin(void)
{
    // Reset to a known state, then sleep: PRE_SCALE is only writable while
    // the oscillator is stopped.
    if (!pca_write8(PCA_MODE1, 0x00)) return false;
    delay(1);

    uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * PCA_PWM_HZ) + 0.5f) - 1;

    uint8_t mode1;
    if (!pca_read8(PCA_MODE1, &mode1)) return false;
    if (!pca_write8(PCA_MODE1, (uint8_t)((mode1 & 0x7F) | 0x10))) return false;  // SLEEP
    if (!pca_write8(PCA_PRESCALE, prescale)) return false;
    if (!pca_write8(PCA_MODE1, (uint8_t)(mode1 & 0x7F))) return false;           // wake
    delay(1);
    // AI: register auto-increment, which is what lets several channels go
    // out in one transaction below. RESTART clears the sleep latch.
    if (!pca_write8(PCA_MODE1, (uint8_t)((mode1 & 0x7F) | 0xA0))) return false;
    // Totem-pole outputs. OUTDRV = 1 is what a servo signal line wants.
    if (!pca_write8(PCA_MODE2, 0x04)) return false;

    return true;
}

static inline uint16_t us_to_counts(float us)
{
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    // counts = us * 4096 * f / 1e6
    float c = us * (4096.0f * PCA_PWM_HZ * 1e-6f);
    if (c < 0.0f)    c = 0.0f;
    if (c > 4095.0f) c = 4095.0f;
    return (uint16_t)(c + 0.5f);
}

/**
 * Write a contiguous run of channels in ONE I2C transaction.
 *
 * The PCA9685 auto-increments its register pointer, so eight channels cost
 * one transaction instead of eight. At 400 kHz that is the difference
 * between ~330 us and ~1.1 ms inside a 5 ms control tick.
 */
static bool pca_write_run(uint8_t first_ch, const uint16_t *counts, uint8_t n)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(PCA_LED0_ON_L + 4 * first_ch));
    for (uint8_t i = 0; i < n; i++) {
        Wire.write(0x00);                            // ON_L : rising edge at 0
        Wire.write(0x00);                            // ON_H
        Wire.write((uint8_t)(counts[i] & 0xFF));     // OFF_L
        Wire.write((uint8_t)(counts[i] >> 8));       // OFF_H
    }
    return Wire.endTransmission() == 0;
}

// Servo shadow. Written by the actuator path, and kept so that a lost link
// leaves the surfaces where they were rather than snapping to a default.
static uint16_t s_servo_counts[PCA_N_USED];

static void servos_park(void)
{
    s_servo_counts[CH_TVC_X] = us_to_counts(TVC_TRIM_X_US);
    s_servo_counts[CH_TVC_Y] = us_to_counts(TVC_TRIM_Y_US);
    // Mid travel on the pad: neither extreme, so a stuck clamp is not
    // automatically the worst case.
    s_servo_counts[CH_CLAMP] = us_to_counts(
        CLAMP_US_CLOSED + 0.6f * (CLAMP_US_OPEN - CLAMP_US_CLOSED));
    for (uint8_t i = CH_FIN_0; i <= CH_FIN_3; i++)
        s_servo_counts[i] = us_to_counts(FIN_TRIM_US);
    s_servo_counts[CH_SPARE] = us_to_counts(1500.0f);
}

/**
 * Turn one actuator packet into servo pulses.
 *
 * The only work between a packet arriving and the servos moving. Keep it
 * that way — see the latency budget in the file header.
 */
static void servos_apply(const ariadna_actuator_t *a)
{
    if (a->flags & ARIADNA_ACT_VALID) {
        s_servo_counts[CH_TVC_X] =
            us_to_counts(TVC_TRIM_X_US + TVC_SIGN_X * a->tvc_x_deg * TVC_US_PER_DEG);
        s_servo_counts[CH_TVC_Y] =
            us_to_counts(TVC_TRIM_Y_US + TVC_SIGN_Y * a->tvc_y_deg * TVC_US_PER_DEG);
    } else if (a->flags & ARIADNA_ACT_TVC_PARK) {
        s_servo_counts[CH_TVC_X] = us_to_counts(TVC_TRIM_X_US);
        s_servo_counts[CH_TVC_Y] = us_to_counts(TVC_TRIM_Y_US);
    }

    // The clamp is applied whether or not the solution is valid, because
    // the Navigation MCU FREEZES its clamp output on a bad solution rather
    // than zeroing it. Shutting the clamp during a burn guarantees a crash
    // and the grain finishes burning either way.
    float f = (a->clamp - DG_CLAMP_MIN) / (DG_CLAMP_MAX - DG_CLAMP_MIN);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    s_servo_counts[CH_CLAMP] =
        us_to_counts(CLAMP_US_CLOSED + f * (CLAMP_US_OPEN - CLAMP_US_CLOSED));

    // FUTURE: fins. The Navigation MCU sends zeros and leaves FINS_LIVE
    // clear, so this branch never runs today — but the path is here and is
    // exercised by the packet arriving every tick.
    if (a->flags & ARIADNA_ACT_FINS_LIVE) {
        for (uint8_t i = 0; i < 4; i++)
            s_servo_counts[CH_FIN_0 + i] =
                us_to_counts(FIN_TRIM_US + a->fin_deg[i] * FIN_US_PER_DEG);
    }

    s_servo_ok = pca_write_run(0, s_servo_counts, PCA_N_USED);
}

// =====================================================================
//  4. W25Q128 FLASH LOG
//
//  A raw driver, again on purpose: the erase-ahead scheme below is the
//  only thing that makes 50 Hz logging possible on a chip whose erase
//  takes up to 400 ms, and it needs control of the timing.
// =====================================================================
static const uint8_t W25_READ_ID   = 0x9F;
static const uint8_t W25_READ_DATA = 0x03;
static const uint8_t W25_PAGE_PROG = 0x02;
static const uint8_t W25_SECT_ERASE= 0x20;
static const uint8_t W25_WRITE_EN  = 0x06;
static const uint8_t W25_READ_SR1  = 0x05;

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

static uint8_t flash_status(void)
{
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    SPI1.transfer(W25_READ_SR1);
    uint8_t sr = SPI1.transfer(0x00);
    flash_deselect();
    SPI1.endTransaction();
    return sr;
}

static inline bool flash_busy(void) { return (flash_status() & 0x01) != 0; }

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

/** Program up to one page. Must not cross a page boundary. */
static void flash_page_program(uint32_t addr, const uint8_t *buf, uint32_t n)
{
    flash_write_enable();
    SPI1.beginTransaction(s_flash_spi);
    flash_select();
    flash_cmd_addr(W25_PAGE_PROG, addr);
    for (uint32_t i = 0; i < n; i++) SPI1.transfer(buf[i]);
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
    uint8_t mfr = SPI1.transfer(0x00);
    uint8_t typ = SPI1.transfer(0x00);
    uint8_t cap = SPI1.transfer(0x00);
    flash_deselect();
    SPI1.endTransaction();

    MASTER_PRINT("# flash JEDEC %02X %02X %02X\n", mfr, typ, cap);
    // Winbond is 0xEF; capacity 0x18 is 128 Mbit. Accept any Winbond of a
    // plausible size rather than pinning one part number.
    return (mfr == 0xEF && cap >= 0x14 && cap <= 0x19);
}

/**
 * Find the append point: the first sector that is still erased.
 *
 * The log is append-only, so "erased" is monotone in sector index and a
 * binary search finds the boundary in ~12 reads instead of 4096. Checking
 * the first 16 bytes is enough — a partially written sector always has its
 * start written, because that is the order it is filled in.
 */
static uint32_t flash_find_append(void)
{
    uint8_t buf[16];
    uint32_t lo = 0;                       // known written (or the start)
    uint32_t hi = FLASH_SIZE / FLASH_SECTOR;

    flash_read(0, buf, sizeof(buf));
    bool first_erased = true;
    for (uint8_t i = 0; i < sizeof(buf); i++) if (buf[i] != 0xFF) first_erased = false;
    if (first_erased) return 0;            // empty chip

    while (hi - lo > 1) {
        uint32_t mid = (lo + hi) / 2;
        flash_read(mid * FLASH_SECTOR, buf, sizeof(buf));
        bool erased = true;
        for (uint8_t i = 0; i < sizeof(buf); i++) if (buf[i] != 0xFF) erased = false;
        if (erased) hi = mid;
        else        lo = mid;
    }
    return hi * FLASH_SECTOR;
}

/**
 * One log record.
 *
 * Deliberately larger than the downlink packet: this is the only complete
 * record of the flight, and flash is the one resource that is not scarce.
 * 88 bytes at 50 Hz is 4.4 kB/s, so a 16 MB chip holds about an hour.
 */
#pragma pack(push, 1)
struct LogRecord {
    uint32_t t_ms;
    float    h, vx, vy, vz;
    float    ux, uy, uz;
    float    roll, pitch, yaw;
    int32_t  lat, lon;
    float    tvc_x, tvc_y, clamp;
    float    thrust_est, p_plan;
    uint16_t vbat_mv, current_ma;
    uint16_t nav_flags;
    uint8_t  state, nav_phase, pyro, health;
    uint8_t  reserved[6];      // room for fields you will want later
};
#pragma pack(pop)
ARIADNA_STATIC_ASSERT(sizeof(LogRecord) == 88, "LogRecord layout changed");

// RAM ring, drained to flash by core 1.
//
// Sized for the worst case that actually happens: a 400 ms sector erase
// during which nothing can be programmed. At 50 Hz x 88 B that is 1.8 kB
// of arrivals, so 8 kB is roughly a 4x margin — and it also covers a LoRa
// transmission landing in the same window.
static const uint32_t LOG_RING_SZ = 8192;
static uint8_t  s_log_ring[LOG_RING_SZ];
static volatile uint32_t s_log_head = 0;    // written by core 0
static volatile uint32_t s_log_tail = 0;    // written by core 1
static volatile uint32_t s_log_dropped = 0;

static void log_push(const LogRecord *r)
{
    const uint8_t *p = (const uint8_t *)r;
    uint32_t head = s_log_head;
    uint32_t tail = s_log_tail;
    uint32_t used = (head - tail) & (LOG_RING_SZ - 1);

    if (used + sizeof(LogRecord) >= LOG_RING_SZ - 1) {
        s_log_dropped++;                    // never block the control core
        return;
    }
    for (uint32_t i = 0; i < sizeof(LogRecord); i++) {
        s_log_ring[head] = p[i];
        head = (head + 1) & (LOG_RING_SZ - 1);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_log_head = head;
}

// =====================================================================
//  5. ANNUNCIATORS
// =====================================================================
static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    s_rgb.setPixelColor(0, s_rgb.Color(r, g, b));
    s_rgb.show();
}

// One colour per mission state, so the pad crew can read the vehicle from
// outside the safety line without a laptop.
static void annunciate(void)
{
    static uint32_t next_ms = 0;
    static bool     blink   = false;
    if (millis() < next_ms) return;
    next_ms = millis() + 250;
    blink = !blink;

    switch (s_state) {
    case ARIADNA_STATE_BOOT:     rgb_set(0, 0, 40); break;                 // blue
    case ARIADNA_STATE_IDLE:     rgb_set(0, 30, 0); break;                 // green
    case ARIADNA_STATE_ARMED:    rgb_set(blink ? 60 : 0, 0, 0); break;     // red blink
    case ARIADNA_STATE_ASCENT:   rgb_set(60, 30, 0); break;                // amber
    case ARIADNA_STATE_APOGEE:   rgb_set(40, 40, 0); break;
    case ARIADNA_STATE_DESCENT:  rgb_set(40, 0, 40); break;                // magenta
    case ARIADNA_STATE_GUIDANCE: rgb_set(60, 60, 60); break;               // white
    case ARIADNA_STATE_LANDED:   rgb_set(0, 60, 0); break;
    case ARIADNA_STATE_RECOVERY: rgb_set(0, blink ? 60 : 0, 0); break;
    case ARIADNA_STATE_ABORT:    rgb_set(blink ? 60 : 0, blink ? 20 : 0, 0); break;
    case ARIADNA_STATE_TEST:     rgb_set(0, 0, blink ? 60 : 0); break;
    default:                     rgb_set(60, 0, 0); break;                 // ERROR
    }

    // The recovery beacon. Loud, periodic, and only after landing — a
    // buzzer during the flight is just battery drain.
    if (s_state == ARIADNA_STATE_RECOVERY) {
        if (blink) tone(PIN_BUZZER, 2700, 120);
    } else if (s_state == ARIADNA_STATE_ARMED) {
        if (blink) tone(PIN_BUZZER, 1800, 40);
    }
}

// =====================================================================
//  6. LINKS
// =====================================================================
// DMA-fed transmit, one channel per port.
//
// This is the single biggest processor saving in the vehicle. Forwarding a
// 90-byte nav_state means 58 bytes past the UART's 32-byte FIFO, which a
// blocking write spins on for 1.26 ms — 12.6 % of this core at 100 Hz, and
// worse, it lands inside the forwarding path below. Forwarding the frame
// before parsing it saves microseconds; blocking on the FIFO immediately
// afterwards would give back a millisecond.
static ariadna_uart_tx_t s_tx_pos;
static ariadna_uart_tx_t s_tx_nav;

static inline void tx_service_all(void)
{
    ariadna_uart_tx_service(&s_tx_pos);
    ariadna_uart_tx_service(&s_tx_nav);
}

static void send_to(ariadna_uart_tx_t *tx, HardwareSerial &port, uint8_t type,
                    const void *payload, uint8_t len)
{
    ariadna_uart_tx_send(tx, port, type, payload, len);
}

static void broadcast_cmd(uint8_t cmd, uint8_t arg, uint32_t arg32)
{
    ariadna_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd = cmd; c.arg = arg; c.arg32 = arg32;
    // Sent three times. Commands are events, not a stream — there is no
    // next packet to carry a missed one, and all of them are idempotent.
    for (uint8_t i = 0; i < 3; i++) {
        send_to(&s_tx_pos, POS_PORT, ARIADNA_PKT_CMD, &c, sizeof(c));
        send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_CMD, &c, sizeof(c));
    }
}

/**
 * Drain the Position MCU port.
 *
 * The navigation state is FORWARDED FIRST and parsed second. That ordering
 * is the point: the Navigation MCU's control loop is waiting on it, and
 * this board's own state machine can afford the extra few microseconds.
 */
static void poll_position(void)
{
    while (POS_PORT.available()) {
        const uint8_t *pl = nullptr;
        uint8_t len = 0;
        uint8_t type = ariadna_rx_byte(&s_rx_pos, (uint8_t)POS_PORT.read(), &pl, &len);
        if (!type) continue;

        if (type == ARIADNA_PKT_NAV_STATE && len == sizeof(ariadna_nav_state_t)) {
            send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_NAV_STATE, pl, len);   // forward first
            memcpy(&s_nav, pl, sizeof(s_nav));
            s_nav_us   = micros();
            s_have_nav = true;
        } else if (type == ARIADNA_PKT_POS_STATUS) {
            if (ariadna_unpack(pl, len, &s_pos_status, sizeof(s_pos_status)))
                s_have_pos_status = true;
        }
    }
}

/**
 * Drain the Navigation MCU port.
 *
 * Actuator packets go to the servos immediately, inside this loop, rather
 * than being stashed for the end of the tick. That removes up to a full
 * 5 ms tick from the command path for the cost of nothing.
 */
static void poll_navigation(void)
{
    while (NAV_PORT.available()) {
        const uint8_t *pl = nullptr;
        uint8_t len = 0;
        uint8_t type = ariadna_rx_byte(&s_rx_nav, (uint8_t)NAV_PORT.read(), &pl, &len);
        if (!type) continue;

        if (type == ARIADNA_PKT_ACTUATOR) {
            if (ariadna_unpack(pl, len, &s_act, sizeof(s_act))) {
                s_act_us   = micros();
                s_have_act = true;
                servos_apply(&s_act);
            }
        } else if (type == ARIADNA_PKT_NAV_STATUS) {
            if (ariadna_unpack(pl, len, &s_nav_status, sizeof(s_nav_status)))
                s_have_nav_status = true;
        }
    }
}

static inline bool link_alive(uint32_t last_us)
{
    return (micros() - last_us) < LINK_TIMEOUT_MS * 1000u;
}

// =====================================================================
//  7. ANALOGUE MONITORING
// =====================================================================
static void read_analogue(void)
{
    // Slow-averaged: these feed telemetry and the arming interlock, not a
    // control loop, and the ADC on this chip is noisy enough to matter.
    static float vbat_f = 0.0f, curr_f = 0.0f;

    float v_adc = (float)analogRead(PIN_V_BAT) * (ADC_VREF / ADC_FULL_SCALE);
    float vbat  = v_adc * VBAT_DIVIDER;

    float i_adc = (float)analogRead(PIN_I_SENSE) * (ADC_VREF / ADC_FULL_SCALE);
    float amps  = ((i_adc / ACS712_DIVIDER) - ACS712_ZERO_V) / ACS712_V_PER_A;

    vbat_f += 0.02f * (vbat - vbat_f);
    curr_f += 0.02f * (amps - curr_f);
    s_vbat    = vbat_f;
    s_current = curr_f;

#if RBF_REMOVED_IS_HIGH
    s_rbf_out = digitalRead(PIN_RBF) == HIGH;
#else
    s_rbf_out = digitalRead(PIN_RBF) == LOW;
#endif
}

// =====================================================================
//  8. MISSION STATE MACHINE
//
//  One transition per tick, each guarded by a condition that can only be
//  met by physics rather than by a timer wherever that is possible. The
//  vehicle is trusted to tell you it has launched; it is not trusted to
//  have launched because five seconds elapsed.
// =====================================================================
static void set_state(uint8_t s)
{
    if (s == s_state) return;
    MASTER_PRINT("# state %u -> %u at t=%lu h=%.1f vz=%.1f\n",
                 s_state, s, (unsigned long)millis(), s_nav.h, s_nav.vz);
    s_state = s;
    s_state_since = millis();
}

static void state_machine(void)
{
    const bool nav_ok  = s_have_nav && link_alive(s_nav_us) &&
                         (s_nav.flags & ARIADNA_NAV_VALID);
    const bool on_pad  = s_have_nav && fabsf(s_nav.vz) < 2.0f && s_nav.h < 5.0f;
    const bool bw_open = digitalRead(PIN_BREAKWIRE) == HIGH;

    if (bw_open) s_flags |= ARIADNA_MST_BREAKWIRE;
    if (s_rbf_out) s_flags |= ARIADNA_MST_RBF_OUT;
    else           s_flags &= (uint8_t)~ARIADNA_MST_RBF_OUT;

    switch (s_state) {

    case ARIADNA_STATE_BOOT:
        if (millis() - s_state_since > 2000)
            set_state(s_post_pass ? ARIADNA_STATE_IDLE : ARIADNA_STATE_ERROR);
        break;

    case ARIADNA_STATE_IDLE:
        s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
        // Arming needs the RBF tag out, a healthy navigation solution, and
        // the vehicle actually sitting still on the pad. Any one missing
        // and it stays safe.
        if (s_rbf_out && nav_ok && on_pad && !bw_open) {
            // Capture the pad reference at the moment of arming, not at
            // boot: the vehicle may have sat on the pad for an hour while
            // the weather moved.
            broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);
            broadcast_cmd(ARIADNA_CMD_ARM, 0, 0);
            s_flags |= ARIADNA_MST_ARMED;
            set_state(ARIADNA_STATE_ARMED);
        }
        break;

    case ARIADNA_STATE_ARMED:
        s_flags |= ARIADNA_MST_ARMED;
        if (!s_rbf_out) {                       // tag went back in
            s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
            broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
            set_state(ARIADNA_STATE_IDLE);
            break;
        }
        // Launch: the breakwire parts, or the vehicle is simply gone. The
        // second test exists because a breakwire that fails to part is a
        // known failure and it must not strand the state machine on the
        // pad while the rocket is in the air.
        if (bw_open || (nav_ok && s_nav.accel_mag > 3.0f) ||
            (nav_ok && s_nav.h > 15.0f)) {
            s_t_launch_ms = millis();
            broadcast_cmd(ARIADNA_CMD_ZERO_CLOCK, 0, s_t_launch_ms);
            set_state(ARIADNA_STATE_ASCENT);
        }
        break;

    case ARIADNA_STATE_ASCENT:
        if (nav_ok && s_nav.vz < 0.0f && s_nav.h > 20.0f)
            set_state(ARIADNA_STATE_APOGEE);
        break;

    case ARIADNA_STATE_APOGEE:
        if (nav_ok && s_nav.vz < -5.0f)
            set_state(ARIADNA_STATE_DESCENT);
        break;

    case ARIADNA_STATE_DESCENT:
        // Handover. From here the Navigation MCU has authority: it arms
        // itself on the same altitude and picks its own ignition point.
        // This board only reports it — announcing the handover is not the
        // same as commanding it, and nothing here can hurry the burn.
        if (nav_ok && s_nav.h <= GUIDANCE_ENTRY_M && s_nav.vz < -2.0f)
            set_state(ARIADNA_STATE_GUIDANCE);
        break;

    case ARIADNA_STATE_GUIDANCE:
        // Down and stopped, or the guidance module says it is finished.
        if ((nav_ok && s_nav.h < 3.0f && fabsf(s_nav.vz) < 1.5f &&
             millis() - s_state_since > 1000) ||
            (s_have_nav_status && s_nav_status.phase == DG_PHASE_DONE))
            set_state(ARIADNA_STATE_LANDED);
        break;

    case ARIADNA_STATE_LANDED:
        servos_park();
        pca_write_run(0, s_servo_counts, PCA_N_USED);
        if (millis() - s_state_since > 5000)
            set_state(ARIADNA_STATE_RECOVERY);
        break;

    case ARIADNA_STATE_RECOVERY:
    case ARIADNA_STATE_ABORT:
    case ARIADNA_STATE_ERROR:
    case ARIADNA_STATE_TEST:
        break;                                  // terminal until power cycle

    default:
        set_state(ARIADNA_STATE_ERROR);
        break;
    }
}

// =====================================================================
//  9. TELEMETRY ASSEMBLY
// =====================================================================
static uint8_t health_bits(void)
{
    uint8_t h = 0;
    if (s_have_nav && link_alive(s_nav_us))      h |= ARIADNA_HLT_POS_LINK;
    if (s_have_act && link_alive(s_act_us))      h |= ARIADNA_HLT_NAV_LINK;
    if (s_have_nav && (s_nav.flags & ARIADNA_NAV_VALID)) h |= ARIADNA_HLT_NAV_VALID;
    if (s_flash_ok)                              h |= ARIADNA_HLT_FLASH_OK;
    if (s_flags & ARIADNA_MST_LOGGING)           h |= ARIADNA_HLT_LOGGING;
    if (s_servo_ok)                              h |= ARIADNA_HLT_SERVO_OK;
    if (s_have_pos_status && s_pos_status.fix >= 3)  h |= ARIADNA_HLT_GPS_FIX;
    if (s_post_pass)                             h |= ARIADNA_HLT_POST_PASS;
    return h;
}

// Saturating conversion to the downlink's fixed-point fields. A telemetry
// value that wraps looks like a real reading of the opposite sign, which
// is worse than a value pinned at its limit.
static inline int16_t sat16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static void build_telemetry(ariadna_tlm_t *t)
{
    memset(t, 0, sizeof(*t));

    t->version = ARIADNA_TLM_VERSION;
    t->t_ms    = millis();
    t->state   = s_state;

    // Attitude in degrees, position and temperature live in the status
    // packet now, not the 240 Hz state packet — see ariadna_nav_state_t.
    // 20 Hz is five times the downlink rate, so nothing here is ever stale
    // relative to what the ground actually sees.
    t->lat = s_pos_status.lat;
    t->lon = s_pos_status.lon;

    t->alt_dm  = sat16(s_nav.h * 10.0f);
    t->vz_dms  = sat16(s_nav.vz * 10.0f);
    t->vh_dms  = sat16(sqrtf(s_nav.vx * s_nav.vx + s_nav.vy * s_nav.vy) * 10.0f);
    t->acc_cg  = sat16(s_nav.accel_mag * 100.0f);

    t->roll_ddeg  = sat16(s_pos_status.roll  * 10.0f);
    t->pitch_ddeg = sat16(s_pos_status.pitch * 10.0f);
    t->yaw_ddeg   = sat16(s_pos_status.yaw   * 10.0f);
    t->temp_dc    = sat16(s_pos_status.temp_c * 10.0f);

    t->tvc_x_cdeg = sat16(s_act.tvc_x_deg * 100.0f);
    t->tvc_y_cdeg = sat16(s_act.tvc_y_deg * 100.0f);
    t->clamp_pct  = (uint8_t)(s_act.clamp * 100.0f);
    t->nav_phase  = s_act.phase;

    t->thrust_n = sat16(s_have_nav_status ? s_nav_status.thrust_est : 0.0f);

    // (V - 2.5) * 100, the encoding the V1 ground station already uses. A
    // 2S pack never sits below 2.5 V and never above 5.05 V per the
    // encoding's range, so one byte is plenty.
    float vb = (s_vbat - 2.5f) * 100.0f;
    if (vb < 0.0f)   vb = 0.0f;
    if (vb > 255.0f) vb = 255.0f;
    t->vbat_cv = (uint8_t)vb;

    float ca = s_current * 10.0f;
    if (ca < 0.0f)   ca = 0.0f;
    if (ca > 255.0f) ca = 255.0f;
    t->curr_da = (uint8_t)ca;

    t->hdop_x10 = s_pos_status.hdop_x10;
    t->sats     = s_pos_status.sats;

    // Low nibble continuity, high nibble fired.
    if (s_have_nav_status)
        t->pyro = (uint8_t)((s_nav_status.pyro_cont & 0x0F) |
                            ((s_nav_status.pyro_fired & 0x0F) << 4));

    t->health = health_bits();

    uint8_t f = 0;
    if (s_flags & ARIADNA_MST_ARMED)     f |= ARIADNA_TF_ARMED;
    if (s_flags & ARIADNA_MST_RBF_OUT)   f |= ARIADNA_TF_RBF_OUT;
    if (s_flags & ARIADNA_MST_BREAKWIRE) f |= ARIADNA_TF_BREAKWIRE;
    if (s_flags & ARIADNA_MST_TEST)      f |= ARIADNA_TF_TEST;
    if (s_flags & ARIADNA_MST_ABORT)     f |= ARIADNA_TF_ABORT;

    // Three-bit error code in the top bits. Most severe wins.
    uint8_t err = 0;
    if (!s_post_pass)                                    err = 1;
    else if (!(s_have_nav && link_alive(s_nav_us)))      err = 2;
    else if (!(s_have_act && link_alive(s_act_us)))      err = 3;
    else if (!s_flash_ok)                                err = 4;
    else if (!s_servo_ok)                                err = 5;
    else if (s_vbat > 0.5f && s_vbat < 7.0f)             err = 6;   // low pack
    f |= (uint8_t)((err << ARIADNA_TF_ERR_SHIFT) & ARIADNA_TF_ERR_MASK);
    t->flags = f;

    t->exp = 0;      // free for your payload experiment
}

// Snapshot handed to core 1 for the downlink. A seqlock again: the LoRa
// path must never make core 0 wait.
static volatile uint32_t s_tlm_seq = 0;
static ariadna_tlm_t     s_tlm_shared;

static void tlm_publish(const ariadna_tlm_t *t)
{
    s_tlm_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_tlm_shared = *t;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_tlm_seq++;
}

static bool tlm_acquire(ariadna_tlm_t *t)
{
    uint32_t a = s_tlm_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *t = s_tlm_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_tlm_seq == a;
}

// =====================================================================
//  10. SELF TEST
// =====================================================================
static bool self_test(void)
{
    bool ok = true;

    // Servo bus. A PCA9685 that does not acknowledge means no TVC at all.
    uint8_t m1;
    if (!pca_read8(PCA_MODE1, &m1)) {
        Serial.println("POST PCA9685 not responding");
        ok = false;
    }

    // Pulse arithmetic. A wrong prescale or a wrong counts conversion
    // silently halves every servo's travel.
    uint16_t c1500 = us_to_counts(1500.0f);
    uint16_t c1000 = us_to_counts(1000.0f);
    float expect1500 = 1500.0f * (4096.0f * PCA_PWM_HZ * 1e-6f);
    if (fabsf((float)c1500 - expect1500) > 2.0f) {
        MASTER_PRINT("POST pulse 1500us -> %u want %.0f\n", c1500, expect1500);
        ok = false;
    }
    if (!(c1000 < c1500)) {
        Serial.println("POST pulse conversion not monotone");
        ok = false;
    }
    // The travel guard must actually clamp.
    if (us_to_counts(500.0f) != us_to_counts(SERVO_US_MIN) ||
        us_to_counts(3000.0f) != us_to_counts(SERVO_US_MAX)) {
        Serial.println("POST servo travel guard not clamping");
        ok = false;
    }

    if (!s_flash_ok) {
        Serial.println("POST flash log unavailable");
        // Not fatal: the vehicle can fly without a log, and refusing to
        // fly over a logging fault would be the wrong trade. It is
        // reported in the telemetry error code instead.
    }

    return ok;
}

// =====================================================================
//  11. CORE 0 — routing, servos, state machine
// =====================================================================
void setup()
{
    // Subordinates held in reset until this board is ready, so they cannot
    // start streaming into a UART that is not listening yet.
    pinMode(PIN_RST_POS, OUTPUT); digitalWrite(PIN_RST_POS, LOW);
    pinMode(PIN_RST_NAV, OUTPUT); digitalWrite(PIN_RST_NAV, LOW);

    pinMode(PIN_BREAKWIRE, INPUT_PULLUP);
    pinMode(PIN_RBF,       INPUT_PULLUP);
    pinMode(PIN_AUX_S16,   OUTPUT); digitalWrite(PIN_AUX_S16, LOW);
    pinMode(PIN_BUZZER,    OUTPUT);
    analogReadResolution(12);

    Serial.begin(ARIADNA_CONSOLE_BAUD);

    s_rgb.begin();
    rgb_set(0, 0, 40);

    // ---- servo bus ----
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    s_servo_ok = pca_begin();
    servos_park();
    pca_write_run(0, s_servo_counts, PCA_N_USED);

    // ---- flash ----
    s_flash_ok = flash_begin();

    // ---- inter-processor links ----
    POS_PORT.setTX(PIN_POS_TX); POS_PORT.setRX(PIN_POS_RX);
    POS_PORT.begin(ARIADNA_MCU_BAUD);
    NAV_PORT.setTX(PIN_NAV_TX); NAV_PORT.setRX(PIN_NAV_RX);
    NAV_PORT.begin(ARIADNA_MCU_BAUD);

    // Serial1 is uart0 and Serial2 is uart1 on the arduino-pico core.
    // Swapping these two sends the Navigation MCU's traffic out of the
    // Position MCU's pins, which looks exactly like both links being dead.
    ariadna_uart_tx_begin(&s_tx_pos, uart0);
    ariadna_uart_tx_begin(&s_tx_nav, uart1);

    ariadna_rx_init(&s_rx_pos);
    ariadna_rx_init(&s_rx_nav);
    memset(&s_nav,        0, sizeof(s_nav));
    memset(&s_act,        0, sizeof(s_act));
    memset(&s_nav_status, 0, sizeof(s_nav_status));
    memset(&s_pos_status, 0, sizeof(s_pos_status));
    s_act.clamp = DG_CLAMP_MAX;

    s_post_pass = self_test();
    MASTER_PRINT("\nAriadna V2 Master MCU — POST %s\n", s_post_pass ? "PASS" : "FAIL");
    MASTER_PRINT("# servo bus %s, flash %s, guidance entry %.0f m\n",
                 s_servo_ok ? "ok" : "FAIL", s_flash_ok ? "ok" : "FAIL",
                 GUIDANCE_ENTRY_M);

    // Release the subordinates now that both UARTs are up.
    digitalWrite(PIN_RST_POS, HIGH);
    digitalWrite(PIN_RST_NAV, HIGH);

    s_state_since = millis();
}

void loop()
{
    static uint32_t next_us   = 0;
    static uint32_t next_log_ms = 0;
    static uint32_t next_bcast_ms = 0;
    static uint32_t next_print_ms = 0;
    const  uint32_t period    = 5000;      // 200 Hz

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) next_us = micros() + period;

    // ---- routing, highest priority ------------------------------------
    tx_service_all();     // retire finished DMA runs before queuing more
    poll_position();      // forwards nav state to Navigation first
    poll_navigation();    // applies actuator commands to the servos

    // ---- housekeeping ---------------------------------------------------
    read_analogue();
    state_machine();
    annunciate();

    // ---- state broadcast, 10 Hz -----------------------------------------
    if (millis() >= next_bcast_ms) {
        next_bcast_ms = millis() + 1000 / ARIADNA_MASTER_HZ;

        ariadna_master_state_t ms;
        memset(&ms, 0, sizeof(ms));
        ms.t_ms        = millis();
        ms.t_launch_ms = s_t_launch_ms;
        ms.vbat_mv     = (uint16_t)(s_vbat * 1000.0f);
        ms.current_ma  = (uint16_t)(s_current * 1000.0f);
        ms.state       = s_state;
        ms.flags       = s_flags;
        if (s_flash_ok) ms.flags |= ARIADNA_MST_LOGGING;

        send_to(&s_tx_pos, POS_PORT, ARIADNA_PKT_MASTER_STATE, &ms, sizeof(ms));
        send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_MASTER_STATE, &ms, sizeof(ms));
    }

    // ---- telemetry snapshot and log -------------------------------------
    {
        ariadna_tlm_t t;
        build_telemetry(&t);
        tlm_publish(&t);
    }

    // Logging runs from ARMED onward. Before that there is nothing worth
    // keeping, and a chip that fills up on the pad has no room for the
    // flight.
    if (millis() >= next_log_ms &&
        s_state >= ARIADNA_STATE_ARMED && s_state <= ARIADNA_STATE_LANDED) {
        next_log_ms = millis() + 1000 / LOG_HZ;
        s_flags |= ARIADNA_MST_LOGGING;

        LogRecord r;
        memset(&r, 0, sizeof(r));
        r.t_ms  = millis();
        r.h     = s_nav.h;
        r.vx    = s_nav.vx;  r.vy = s_nav.vy;  r.vz = s_nav.vz;
        r.ux    = s_nav.ux;  r.uy = s_nav.uy;  r.uz = s_nav.uz;
        // Attitude in degrees and position come from the status packet at
        // 20 Hz. The 240 Hz thrust-axis unit vector above already captures
        // tilt at full rate; roll about the long axis is the only thing
        // logged at the slower rate, and it is not a control quantity.
        r.roll  = s_pos_status.roll;
        r.pitch = s_pos_status.pitch;
        r.yaw   = s_pos_status.yaw;
        r.lat   = s_pos_status.lat;
        r.lon   = s_pos_status.lon;
        r.tvc_x = s_act.tvc_x_deg;
        r.tvc_y = s_act.tvc_y_deg;
        r.clamp = s_act.clamp;
        r.thrust_est = s_have_nav_status ? s_nav_status.thrust_est : 0.0f;
        r.p_plan     = s_have_nav_status ? s_nav_status.p_plan : 0.0f;
        r.vbat_mv    = (uint16_t)(s_vbat * 1000.0f);
        r.current_ma = (uint16_t)(s_current * 1000.0f);
        r.nav_flags  = s_nav.flags;
        r.state      = s_state;
        r.nav_phase  = s_act.phase;
        r.pyro       = s_have_nav_status
                     ? (uint8_t)((s_nav_status.pyro_cont & 0x0F) |
                                 ((s_nav_status.pyro_fired & 0x0F) << 4)) : 0;
        r.health     = health_bits();

        log_push(&r);
    }

    // ---- USB console, last and rate limited ------------------------------
    if (millis() >= next_print_ms) {
        next_print_ms = millis() + 500;
        MASTER_PRINT("st=%u h=%.1f vz=%.1f tvc=%.2f/%.2f clamp=%.2f "
                     "vb=%.2f i=%.2f pos=%lu/%lu nav=%lu/%lu drop=%lu tlm=%lu/%lu\n",
                     s_state, s_nav.h, s_nav.vz,
                     s_act.tvc_x_deg, s_act.tvc_y_deg, s_act.clamp,
                     s_vbat, s_current,
                     (unsigned long)s_rx_pos.rx_ok,
                     (unsigned long)(s_rx_pos.rx_crc + s_rx_pos.rx_len),
                     (unsigned long)s_rx_nav.rx_ok,
                     (unsigned long)(s_rx_nav.rx_crc + s_rx_nav.rx_len),
                     (unsigned long)s_log_dropped,
                     (unsigned long)s_tlm_sent,
                     (unsigned long)s_tlm_skipped);
    }
}

// =====================================================================
//  12. CORE 1 — flash log and LoRa downlink
//
//  Everything here can block for tens or hundreds of milliseconds. That is
//  precisely why it is here and not on core 0.
// =====================================================================
static uint32_t s_log_addr       = 0;
static uint32_t s_log_erased_to  = 0;   // everything below this is erased

/**
 * Keep erased space ahead of the write pointer.
 *
 * Erasing on demand would stall the log for up to 400 ms exactly when it
 * is busiest. Erasing ahead means the erase happens during the quiet parts
 * of the flight, and the RAM ring covers whatever is left.
 */
static void flash_erase_ahead(void)
{
    const uint32_t LOOKAHEAD = 4 * FLASH_SECTOR;

    if (s_log_erased_to >= s_log_addr + LOOKAHEAD) return;
    if (s_log_erased_to >= FLASH_SIZE)             return;
    if (flash_busy())                              return;

    flash_sector_erase(s_log_erased_to);
    s_log_erased_to += FLASH_SECTOR;
}

/** Drain the RAM ring into flash, at most one page per call. */
static void flash_drain(void)
{
    uint32_t head = s_log_head;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint32_t tail = s_log_tail;
    uint32_t used = (head - tail) & (LOG_RING_SZ - 1);

    if (used < FLASH_PAGE)          return;    // wait for a full page
    if (s_log_addr >= FLASH_SIZE)   return;    // chip full: stop, do not wrap
    if (s_log_addr + FLASH_PAGE > s_log_erased_to) return;   // not erased yet
    if (flash_busy())               return;

    uint8_t page[FLASH_PAGE];
    for (uint32_t i = 0; i < FLASH_PAGE; i++) {
        page[i] = s_log_ring[tail];
        tail = (tail + 1) & (LOG_RING_SZ - 1);
    }

    flash_page_program(s_log_addr, page, FLASH_PAGE);
    s_log_addr += FLASH_PAGE;

    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_log_tail = tail;
}

void setup1()
{
    // Wait for core 0 to finish bringing up SPI1 and the flash.
    while (s_state == ARIADNA_STATE_BOOT && millis() < 3000)
        tight_loop_contents();

    if (s_flash_ok) {
        s_log_addr      = flash_find_append();
        s_log_erased_to = s_log_addr;
        MASTER_PRINT("# log append at 0x%06lX (%lu KB used)\n",
                     (unsigned long)s_log_addr,
                     (unsigned long)(s_log_addr / 1024));
    }

    // ---- LoRa ----
    SPI.setRX(PIN_LORA_MISO);
    SPI.setTX(PIN_LORA_MOSI);
    SPI.setSCK(PIN_LORA_SCK);
    SPI.begin();

    LoRa.setSPI(SPI);
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    // Every parameter comes from ariadna_link.h, which the ground station
    // reads too. A mismatch in any one of them is a radio that hears
    // nothing, with no error to show for it — so there is one definition.
    if (LoRa.begin(ARIADNA_LORA_FREQ_HZ)) {
        LoRa.setSpreadingFactor(ARIADNA_LORA_SF);
        LoRa.setSignalBandwidth(ARIADNA_LORA_BW_HZ);
        LoRa.setCodingRate4(ARIADNA_LORA_CR4);
        LoRa.setPreambleLength(ARIADNA_LORA_PREAMBLE);
        LoRa.setTxPower(ARIADNA_LORA_TX_DBM);
        LoRa.enableCrc();
        s_lora_ok = true;
        MASTER_PRINT("# LoRa %lu Hz SF%d BW%lu, %lu us on air, %d Hz downlink\n",
                     (unsigned long)ARIADNA_LORA_FREQ_HZ, ARIADNA_LORA_SF,
                     (unsigned long)ARIADNA_LORA_BW_HZ,
                     (unsigned long)ariadna_lora_airtime_us(sizeof(ariadna_tlm_t)),
                     ARIADNA_TLM_HZ);
    }
}

void loop1()
{
    static uint32_t next_tlm_ms = 0;

    // Flash first: the log is the only record that survives a lost link.
    if (s_flash_ok) {
        flash_erase_ahead();
        flash_drain();
    }

    // ---- LoRa downlink ---------------------------------------------------
    //
    // ASYNCHRONOUS. endPacket(true) returns as soon as the packet is handed
    // to the radio, and isTransmitting() reports when the air is clear.
    //
    // At 10 Hz a 46-byte packet is 46 ms on air inside a 100 ms slot, so a
    // blocking endPacket() would put this core in the radio driver for
    // nearly half its life — and the flash log is the other thing this core
    // does. Blocking would not lose telemetry, but it would make the log
    // bursty for no reason.
    //
    // A slot whose predecessor is still on air is SKIPPED, never queued.
    // Queuing is how a downlink degrades from "occasionally late" to
    // "permanently a second behind": each late packet delays the next, and
    // the backlog only grows. Skipping keeps every packet that does arrive
    // fresh, which is what a ground station needs.
    if (s_lora_ok && millis() >= next_tlm_ms) {
        if (LoRa.isTransmitting()) {
            s_tlm_skipped++;
        } else {
            next_tlm_ms = millis() + 1000 / ARIADNA_TLM_HZ;
            ariadna_tlm_t t;
            if (tlm_acquire(&t)) {
                LoRa.beginPacket();
                LoRa.write((const uint8_t *)&t, sizeof(t));
                LoRa.endPacket(true);          // async
                s_tlm_sent++;
            }
        }
    }
}
