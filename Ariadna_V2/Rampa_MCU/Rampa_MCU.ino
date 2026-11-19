// =====================================================================
//  Ariadna V2 — RAMPA (LAUNCH PAD CONTROLLER), PCB1
//
//  Target: Raspberry Pi Pico 2 (RP2350, U7), arduino-pico core.
//  Needs:  no external I2C libraries (direct Wire1 drivers for TCA9534,
//          INA226, ADS1115); Adafruit_NeoPixel optional (ariadna_rgb.h).
//
//  Pinout verified directly from RampaNetlist_PCB1_2026-11-19.enet.
//  See ../PINOUT.md for the complete board reference.
//
//  ---------------------------------------------------------------------
//  ARCHITECTURE & SAFETY
//  ---------------------------------------------------------------------
//  * Umbilical link (J3 -> GP0/GP1 = hardware I2C0 Slave @ 0x2B):
//      Rocket (Master MCU) is I2C Master, Rampa is I2C Slave.
//      Completely separate hardware I2C block from Rampa's internal
//      sensors (Wire1 / I2C1 on GP6/GP7).
//  * LoRa radio (M1 Ra-02, power-gated by U12 TPS22918 via TCA9534 P4):
//      Kept hardware-powered OFF (LORA_EN = LOW) whenever the umbilical
//      is active, preventing RF collision with the Rocket's LoRa radio.
//  * Dual-stage pyro safety:
//      1. Physical key switch J4 + 5 A fuse F2 -> VPYRO_KEY (RBF_SENSE).
//      2. High-side arming MOSFET Q20 (driven by PYRO_ARM = GP19).
//      3. Low-side channel MOSFETs Q22/Q23/Q24 (PYRO1..3_FIRE = GP16..18).
//      4. Continuity measured safely through R26 (10k) + D5 bleed (< 1 mA)
//         into ADS1115 (U11 @ 0x48, AIN0..2) with PYRO_ARM OFF.
//  * Safe hot-disconnect before liftoff (T - 100 ms):
//      When Rocket sends RAMPA_CMD_COMMIT (at T - 100 ms, after the
//      T - 1 s LoRa final-abort window), Rampa latches COMMITTED, replies
//      with RAMPA_STATE_COMMITTED on the immediately following I2C read,
//      and then calls Wire.end() + puts GP0/GP1 to Hi-Z INPUT BEFORE
//      energizing the igniter at T - 0. Magnetic pogo rip-away therefore
//      cannot glitch or hang either MCU's I2C peripheral.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include "hardware/watchdog.h"
#include "ariadna_rampa_link.h"

#ifdef ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
// Rampa PCB1 SK6812 (LED1) is on GP20 (U7.26) via R56 (100R).
#define ARIADNA_RGB_PIN       20
#define ARIADNA_RGB_POWER_PIN 0xFF
#include "ariadna_rgb.h"

#include <string.h>
#include <stdio.h>

// =====================================================================
//  1. HARDWARE PINOUT (from RampaNetlist_PCB1_2026-11-19.enet)
// =====================================================================

// Umbilical port J3 — hardware I2C0 Slave (0x2B)
static const uint8_t PIN_UMB_SDA      = 0;   // U7.1  UMB_TX -> R21 (1k)  -> J3.4
static const uint8_t PIN_UMB_SCL      = 1;   // U7.2  UMB_RX -> R22 (100R)-> J3.3

// Internal sensor/expander bus — hardware I2C1 Master (400 kHz)
static const uint8_t PIN_INT_SDA      = 6;   // U7.9  I2C_SDA (4.7k R18)
static const uint8_t PIN_INT_SCL      = 7;   // U7.10 I2C_SCL (4.7k R19)

// Direct pyro firing GPIOs (ALL forced LOW at boot!)
static const uint8_t PIN_PYRO1_FIRE   = 16;  // U7.21 -> R31 -> Q22 (J5)
static const uint8_t PIN_PYRO2_FIRE   = 17;  // U7.22 -> R36 -> Q23 (J6)
static const uint8_t PIN_PYRO3_FIRE   = 18;  // U7.24 -> R41 -> Q24 (J7)
static const uint8_t PIN_PYRO_ARM     = 19;  // U7.25 -> R29 -> Q21 -> Q20 (VPYRO_ARMED)

// Spare / expansion hardware pins (defined for future sensor logging)
static const uint8_t PIN_SPI0_SCK     = 2;   // U9 W25Q128 + U14 ADS1220
static const uint8_t PIN_SPI0_MOSI    = 3;
static const uint8_t PIN_SPI0_MISO    = 4;
static const uint8_t PIN_FLASH_CS     = 5;
static const uint8_t PIN_ADS1220_CS   = 15;
static const uint8_t PIN_ADS1220_DRDY = 21;
static const uint8_t PIN_LORA_SCK     = 10;  // SPI1 -> M1 Ra-02 (powered OFF on umbilical)
static const uint8_t PIN_LORA_MOSI    = 11;
static const uint8_t PIN_LORA_MISO    = 12;
static const uint8_t PIN_LORA_CS      = 13;
static const uint8_t PIN_SERVO1       = 8;   // J12.1
static const uint8_t PIN_SERVO2       = 9;   // J12.4
static const uint8_t PIN_SERVO3       = 14;  // J12.7
static const uint8_t PIN_SERVO4       = 22;  // J12.10
static const uint8_t PIN_PRESSURE_ADC = 26;  // J9.2 via 4.7k/10k

// Internal I2C1 addresses (verified from address pins in netlist)
static const uint8_t ADDR_TCA9534     = 0x20; // U10: A2=A1=A0=GND
static const uint8_t ADDR_INA226_BAT  = 0x40; // U1:  A1=GND, A0=GND (VBAT, 5 mOhm R4)
static const uint8_t ADDR_INA226_UMB  = 0x41; // U8:  A1=3V3, A0=GND (UMB_5V, 5 mOhm R20)
static const uint8_t ADDR_ADS1115     = 0x48; // U11: ADDR=GND (AIN0..2 CONT1..3, AIN3 5V/2)

// TCA9534 (U10) bit assignments
static const uint8_t EXP_BUZZER_EN    = (1u << 0); // P0 OUT: HIGH = buzzer ON
static const uint8_t EXP_ROCKET_EN    = (1u << 1); // P1 OUT: HIGH = U2 5V umbilical buck ON
static const uint8_t EXP_RBF_SENSE    = (1u << 2); // P2 IN:  HIGH = J4 key switch closed
static const uint8_t EXP_LORA_RST     = (1u << 3); // P3 OUT: LOW = LoRa reset
static const uint8_t EXP_LORA_EN      = (1u << 4); // P4 OUT: HIGH = U12 LoRa 3.3V power ON
static const uint8_t EXP_LORA_DIO0    = (1u << 5); // P5 IN:  LoRa DIO0
static const uint8_t EXP_FIRE_LED     = (1u << 6); // P6 OUT: HIGH = red fire LED ON
static const uint8_t EXP_BTN          = (1u << 7); // P7 IN:  LOW = SW1 local abort pressed

// =====================================================================
//  2. SAFETY CONSTANTS
// =====================================================================
static const uint16_t RAMPA_VBAT_MIN_MV     = 6600u; // 2S LiPo minimum safe firing voltage
static const uint16_t RAMPA_CONT_MIN_MV     = 350u;  // ADS1115 CONTx threshold (~1.0-1.5 V good)
static const uint32_t RAMPA_FIRE_PULSE_MS   = 1000u; // igniter firing pulse duration
static const uint32_t RAMPA_POST_FIRE_MS    = 5000u; // safe cooldown before re-enabling I2C0
static const uint32_t RAMPA_COMMIT_GUARD_MS = 250u;  // max allowed remaining time at COMMIT

// =====================================================================
//  3. INTERNAL I2C1 DRIVERS (TCA9534, INA226, ADS1115 — no extra libs)
// =====================================================================
static uint8_t s_exp_out = 0; // cached TCA9534 output byte (LORA_EN=0 by default!)
static bool    s_exp_ok  = false;
static bool    s_ina_ok  = false;
static bool    s_ads_ok  = false;

static bool i2c1_write8(uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(val);
    return Wire1.endTransmission(true) == 0;
}

static bool i2c1_read8(uint8_t addr, uint8_t reg, uint8_t *val)
{
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)addr, 1) != 1) return false;
    *val = (uint8_t)Wire1.read();
    return true;
}

static bool i2c1_write16be(uint8_t addr, uint8_t reg, uint16_t val)
{
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write((uint8_t)(val >> 8));
    Wire1.write((uint8_t)(val & 0xFFu));
    return Wire1.endTransmission(true) == 0;
}

static bool i2c1_read16be(uint8_t addr, uint8_t reg, uint16_t *val)
{
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)addr, 2) != 2) return false;
    uint8_t hi = (uint8_t)Wire1.read();
    uint8_t lo = (uint8_t)Wire1.read();
    *val = (uint16_t)(((uint16_t)hi << 8) | lo);
    return true;
}

static bool exp_begin(void)
{
    // Outputs: P0 (BUZZER_EN), P1 (ROCKET_EN), P3 (LORA_RST), P4 (LORA_EN), P6 (FIRE_LED)
    // Inputs:  P2 (RBF_SENSE), P5 (DIO0), P7 (BTN)
    const uint8_t cfg = (uint8_t)(EXP_RBF_SENSE | EXP_LORA_DIO0 | EXP_BTN);
    s_exp_out = 0; // buzzer OFF, rocket 5V OFF, LoRa OFF, fire LED OFF
    bool ok = i2c1_write8(ADDR_TCA9534, 0x01, s_exp_out);
    ok = ok && i2c1_write8(ADDR_TCA9534, 0x03, cfg);
    s_exp_ok = ok;
    return ok;
}

static bool exp_write_outputs(uint8_t out_byte)
{
    if (out_byte == s_exp_out && s_exp_ok) return true;
    s_exp_out = out_byte;
    s_exp_ok = i2c1_write8(ADDR_TCA9534, 0x01, s_exp_out);
    return s_exp_ok;
}

static bool exp_read_inputs(uint8_t *in_byte)
{
    s_exp_ok = i2c1_read8(ADDR_TCA9534, 0x00, in_byte);
    return s_exp_ok;
}

/** Read INA226 bus voltage in millivolts (reg 0x02, 1.25 mV/LSB). */
static bool ina226_read_vbus_mv(uint8_t addr, uint16_t *mv)
{
    uint16_t raw = 0;
    if (!i2c1_read16be(addr, 0x02, &raw)) return false;
    *mv = (uint16_t)(((uint32_t)raw * 125u) / 100u);
    return true;
}

/** Start one single-shot ADS1115 conversion on AIN`ch` (0..3), +/-4.096 V FSR. */
static bool ads1115_start(uint8_t ch)
{
    // OS=1 (start), MUX=100+ch (single-ended), PGA=001 (+/-4.096 V -> 0.125 mV/LSB),
    // MODE=1 (single-shot), DR=110 (475 SPS -> ~2.1 ms), COMP_QUE=11 (disable)
    const uint16_t cfg = (uint16_t)(0x8000u |
                                    (((uint16_t)(4u + (ch & 3u))) << 12) |
                                    (1u << 9) |
                                    (1u << 8) |
                                    (6u << 5) |
                                    0x0003u);
    return i2c1_write16be(ADDR_ADS1115, 0x01, cfg);
}

/** Read ADS1115 conversion result in millivolts if ready. */
static bool ads1115_try_read_mv(uint16_t *mv)
{
    uint16_t cfg = 0;
    if (!i2c1_read16be(ADDR_ADS1115, 0x01, &cfg)) return false;
    if ((cfg & 0x8000u) == 0u) return false; // still converting
    uint16_t raw = 0;
    if (!i2c1_read16be(ADDR_ADS1115, 0x00, &raw)) return false;
    int16_t sraw = (int16_t)raw;
    if (sraw < 0) sraw = 0;
    // 4.096 V / 32768 = 0.125 mV/LSB -> divide by 8
    *mv = (uint16_t)((uint32_t)sraw >> 3);
    return true;
}

// =====================================================================
//  4. STATE & I2C SLAVE HANDLERS (Wire / I2C0 @ 0x2B)
// =====================================================================
static ariadna_rgb_t          s_rgb;
static ariadna_rampa_state_t  s_state = RAMPA_STATE_IDLE;
static uint8_t                s_fault_mask = 0;
static uint8_t                s_flags = ARIADNA_RMP_LORA_OFF;
static uint8_t                s_last_seq = 0;
static uint8_t                s_pyro_ch = 1;

static uint16_t               s_vbat_mv = 0;
static uint16_t               s_cont_mv[3] = { 0, 0, 0 };
static bool                   s_key_armed = false;
static bool                   s_btn_pressed = false;

static bool                   s_umb_enabled = false;
static volatile bool          s_umb_detach_req = false;
static volatile bool          s_cmd_pending = false;
static ariadna_rampa_cmd_t    s_cmd_rx;
static uint32_t               s_cmd_rx_ms = 0;
static uint32_t               s_last_hb_ms = 0;

static bool                   s_cd_running = false;
static uint32_t               s_t_fire_ms  = 0;
static uint32_t               s_fire_start_ms = 0;
static uint32_t               s_fire_end_ms   = 0;

// Cached response frame served directly from on_umb_request() ISR.
static uint8_t                s_tx_buf[ARIADNA_RAMPA_STAT_SIZE];

static void pyro_all_off(void)
{
    digitalWrite(PIN_PYRO1_FIRE, LOW);
    digitalWrite(PIN_PYRO2_FIRE, LOW);
    digitalWrite(PIN_PYRO3_FIRE, LOW);
    digitalWrite(PIN_PYRO_ARM,   LOW);
}

static uint8_t pyro_pin_for(uint8_t ch)
{
    if (ch == 2u) return PIN_PYRO2_FIRE;
    if (ch == 3u) return PIN_PYRO3_FIRE;
    return PIN_PYRO1_FIRE;
}

static void refresh_tx_frame(uint32_t now_ms)
{
    ariadna_rampa_status_t st;
    memset(&st, 0, sizeof(st));
    st.state      = (uint8_t)s_state;
    st.last_seq   = s_last_seq;
    st.flags      = s_flags;
    st.fault_mask = s_fault_mask;
    st.vbat_mv    = s_vbat_mv;
    st.t_rem_ms   = s_cd_running ? (int32_t)(s_t_fire_ms - now_ms) : -1;
    st.cont_mv[0] = s_cont_mv[0];
    st.cont_mv[1] = s_cont_mv[1];
    st.cont_mv[2] = s_cont_mv[2];
    ariadna_rampa_status_seal(&st);
    memcpy(s_tx_buf, &st, sizeof(st));
}

static void on_umb_receive(int n)
{
    uint8_t buf[ARIADNA_RAMPA_CMD_SIZE];
    size_t got = 0;
    while (n-- > 0 && Wire.available()) {
        const int c = Wire.read();
        if (got < sizeof(buf)) buf[got++] = (uint8_t)c;
    }
    ariadna_rampa_cmd_t c;
    if (got == sizeof(buf) && ariadna_rampa_cmd_decode(buf, got, &c) == ARIADNA_RAMPA_OK) {
        s_cmd_rx      = c;
        s_cmd_rx_ms   = millis();
        s_cmd_pending = true;
    }
}

static void on_umb_request(void)
{
    Wire.write(s_tx_buf, sizeof(s_tx_buf));
    // If we just sent RAMPA_STATE_COMMITTED (with DETACHING flag), request
    // immediate I2C0 shutdown on the main loop before liftoff!
    if (s_state == RAMPA_STATE_COMMITTED) {
        s_umb_detach_req = true;
    }
}

static void umb_i2c_attach(void)
{
    if (s_umb_enabled) return;
    Wire.setSDA(PIN_UMB_SDA);
    Wire.setSCL(PIN_UMB_SCL);
    Wire.begin((uint8_t)ARIADNA_RAMPA_I2C_ADDR);
    Wire.onReceive(on_umb_receive);
    Wire.onRequest(on_umb_request);
    s_umb_enabled    = true;
    s_umb_detach_req = false;
}

/** Cleanly shut down I2C0 Slave and float GP0/GP1 (Hi-Z) before liftoff so
 *  magnetic pogo separation cannot glitch or lock up either MCU. */
static void umb_i2c_detach(void)
{
    if (!s_umb_enabled) return;
    Wire.end();
    pinMode(PIN_UMB_SDA, INPUT);
    pinMode(PIN_UMB_SCL, INPUT);
    s_umb_enabled    = false;
    s_umb_detach_req = false;
}

// =====================================================================
//  5. SAFETY INTERLOCKS & STATE MACHINE
// =====================================================================
static void enter_abort(uint8_t fault_bit)
{
    pyro_all_off();
    s_cd_running = false;
    s_fault_mask |= fault_bit;
    s_state = RAMPA_STATE_ABORT;
    s_flags &= (uint8_t)~(ARIADNA_RMP_PYRO_ARMED | ARIADNA_RMP_DETACHING);
}

static void update_sensors(uint32_t now_ms)
{
    static uint32_t next_fast_ms = 0;
    static uint8_t  ads_ch = 0;
    static bool     ads_started = false;

    // 1. Poll ADS1115 continuity channels round-robin (~4 ms/ch -> ~80 Hz/ch)
    if (!ads_started) {
        s_ads_ok = ads1115_start(ads_ch);
        ads_started = s_ads_ok;
    } else {
        uint16_t mv = 0;
        if (ads1115_try_read_mv(&mv)) {
            s_ads_ok = true;
            if (ads_ch < 3u) s_cont_mv[ads_ch] = mv;
            ads_ch = (uint8_t)((ads_ch + 1u) & 3u);
            ads_started = ads1115_start(ads_ch);
        }
    }

    if ((int32_t)(now_ms - next_fast_ms) < 0) return;
    next_fast_ms = now_ms + 20u; // 50 Hz

    // 2. TCA9534 inputs: physical key (RBF_SENSE) & local abort button (SW1)
    uint8_t inp = 0;
    if (exp_read_inputs(&inp)) {
        s_key_armed   = (inp & EXP_RBF_SENSE) != 0u;
        s_btn_pressed = (inp & EXP_BTN) == 0u; // active LOW
    } else {
        s_key_armed = false;
    }

    // 3. INA226 VBAT
    uint16_t vb = 0;
    if (ina226_read_vbus_mv(ADDR_INA226_BAT, &vb)) {
        s_vbat_mv = vb;
        s_ina_ok  = true;
    } else {
        s_ina_ok = false;
    }

    // 4. Local physical abort button ALWAYS wins immediately!
    if (s_btn_pressed && (s_state == RAMPA_STATE_COUNTDOWN ||
                          s_state == RAMPA_STATE_COMMITTED ||
                          s_state == RAMPA_STATE_READY)) {
        enter_abort(ARIADNA_RMPF_BTN_ABORT);
    }

    // 5. Recompute live interlock flags & fault mask
    uint8_t fl = ARIADNA_RMP_LORA_OFF; // LoRa kept OFF while umbilical mode active
    if (s_key_armed)                        fl |= ARIADNA_RMP_KEY_ARMED;
    if (s_cont_mv[0] >= RAMPA_CONT_MIN_MV)  fl |= ARIADNA_RMP_CONT1;
    if (s_cont_mv[1] >= RAMPA_CONT_MIN_MV)  fl |= ARIADNA_RMP_CONT2;
    if (s_cont_mv[2] >= RAMPA_CONT_MIN_MV)  fl |= ARIADNA_RMP_CONT3;
    if (s_state == RAMPA_STATE_FIRED)       fl |= ARIADNA_RMP_PYRO_ARMED;
    if (s_state == RAMPA_STATE_COMMITTED)   fl |= ARIADNA_RMP_DETACHING;

    uint8_t fm = 0;
    if (!s_exp_ok || !s_ina_ok || !s_ads_ok) fm |= ARIADNA_RMPF_HW;
    if (s_vbat_mv < RAMPA_VBAT_MIN_MV)       fm |= ARIADNA_RMPF_LOW_VBAT;
    if (!s_key_armed)                        fm |= ARIADNA_RMPF_NO_KEY;
    const uint8_t cbit = (uint8_t)(ARIADNA_RMP_CONT1 << ((s_pyro_ch >= 1u && s_pyro_ch <= 3u) ? (s_pyro_ch - 1u) : 0u));
    if ((fl & cbit) == 0u)                   fm |= ARIADNA_RMPF_NO_CONT;
    if (s_btn_pressed)                       fm |= ARIADNA_RMPF_BTN_ABORT;

    if (fm == 0u) fl |= ARIADNA_RMP_READY;
    s_flags = fl;
    if (s_state != RAMPA_STATE_ABORT) {
        s_fault_mask = fm;
    } else {
        s_fault_mask |= fm;
    }
}

static void process_rocket_cmd(const ariadna_rampa_cmd_t &c, uint32_t rx_ms)
{
    s_last_seq   = c.seq;
    s_last_hb_ms = rx_ms ? rx_ms : 1u;
    if (c.pyro_ch >= 1u && c.pyro_ch <= 3u) s_pyro_ch = c.pyro_ch;

    const bool rocket_ok = (c.rocket_flags & (ARIADNA_RK_POST_OK | ARIADNA_RK_NAV_VALID |
                                              ARIADNA_RK_POS_OK | ARIADNA_RK_NAV_OK |
                                              ARIADNA_RK_NO_ABORT)) ==
                           (ARIADNA_RK_POST_OK | ARIADNA_RK_NAV_VALID |
                            ARIADNA_RK_POS_OK | ARIADNA_RK_NAV_OK |
                            ARIADNA_RK_NO_ABORT);

    switch (c.cmd) {
    case RAMPA_CMD_ABORT:
        enter_abort(0);
        break;

    case RAMPA_CMD_DETACH:
        pyro_all_off();
        s_cd_running = false;
        s_state = RAMPA_STATE_IDLE;
        s_umb_detach_req = true;
        break;

    case RAMPA_CMD_PING:
        pyro_all_off();
        s_cd_running = false;
        if (s_state != RAMPA_STATE_FIRED) {
            s_fault_mask &= (uint8_t)~(ARIADNA_RMPF_TIMEOUT | ARIADNA_RMPF_ROCKET | ARIADNA_RMPF_CLOCK_SKEW);
            s_state = (s_fault_mask == 0u) ? RAMPA_STATE_IDLE : RAMPA_STATE_FAULT;
        }
        break;

    case RAMPA_CMD_PREPARE:
        pyro_all_off();
        s_cd_running = false;
        if (s_fault_mask == 0u && rocket_ok) {
            s_state = RAMPA_STATE_READY;
        } else {
            if (!rocket_ok) s_fault_mask |= ARIADNA_RMPF_ROCKET;
            s_state = RAMPA_STATE_FAULT;
        }
        break;

    case RAMPA_CMD_COUNTDOWN:
        if (s_state == RAMPA_STATE_ABORT || s_state == RAMPA_STATE_FIRED) break;
        if (c.t_rem_ms <= 0 || c.t_rem_ms > (int32_t)ARIADNA_COUNTDOWN_MAX_MS) {
            enter_abort(ARIADNA_RMPF_CLOCK_SKEW);
        } else if (!rocket_ok) {
            enter_abort(ARIADNA_RMPF_ROCKET);
        } else if (s_fault_mask != 0u) {
            enter_abort(0);
        } else {
            // Lock Rampa's T-0 firing clock directly to Rocket's 20 Hz updates (< 1 ms error).
            s_t_fire_ms  = rx_ms + (uint32_t)c.t_rem_ms;
            s_cd_running = true;
            s_state      = RAMPA_STATE_COUNTDOWN;
        }
        break;

    case RAMPA_CMD_COMMIT:
        if (s_state != RAMPA_STATE_COUNTDOWN) {
            enter_abort(ARIADNA_RMPF_CLOCK_SKEW);
        } else if (!rocket_ok) {
            enter_abort(ARIADNA_RMPF_ROCKET);
        } else if (s_fault_mask != 0u) {
            enter_abort(0);
        } else if (c.t_rem_ms < 5 || c.t_rem_ms > (int32_t)RAMPA_COMMIT_GUARD_MS) {
            enter_abort(ARIADNA_RMPF_CLOCK_SKEW);
        } else {
            s_t_fire_ms  = rx_ms + (uint32_t)c.t_rem_ms;
            s_cd_running = true;
            s_state      = RAMPA_STATE_COMMITTED;
            s_flags     |= ARIADNA_RMP_DETACHING;
        }
        break;

    default:
        break;
    }
}

static void update_outputs(uint32_t now_ms)
{
    uint8_t out = 0;
    // Keep LORA_EN = 0 (LoRa hardware powered OFF) while umbilical is active/committed!
    out &= (uint8_t)~EXP_LORA_EN;

    // Red FIRE_LED + buzzer warnings
    if (s_state == RAMPA_STATE_FIRED) {
        out |= (EXP_FIRE_LED | EXP_BUZZER_EN);
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_SOLID, 0);
    } else if (s_state == RAMPA_STATE_COMMITTED) {
        out |= (EXP_FIRE_LED | EXP_BUZZER_EN);
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_FAST, 0);
    } else if (s_state == RAMPA_STATE_COUNTDOWN) {
        const int32_t rem = (int32_t)(s_t_fire_ms - now_ms);
        const uint32_t period = (rem < 10000) ? 250u : 1000u;
        if ((now_ms % period) < 80u) out |= (EXP_FIRE_LED | EXP_BUZZER_EN);
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_FAST, 0);
    } else if (s_state == RAMPA_STATE_ABORT) {
        if ((now_ms % 500u) < 250u) out |= EXP_FIRE_LED;
        ariadna_rgb_set(&s_rgb, ARIADNA_C_MAGENTA, ARIADNA_PAT_FAST, 0);
    } else if (s_state == RAMPA_STATE_READY) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_SOLID, 0);
    } else if (s_fault_mask != 0u) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_YELLOW, ARIADNA_PAT_SLOW, 0);
    } else {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_PULSE, 0);
    }

    (void)exp_write_outputs(out);
    ariadna_rgb_service(&s_rgb);
}

// =====================================================================
//  6. SETUP & LOOP
// =====================================================================
void setup()
{
    // FIRST: force every pyro pin LOW before touching any bus or delay.
    pinMode(PIN_PYRO_ARM,   OUTPUT); digitalWrite(PIN_PYRO_ARM,   LOW);
    pinMode(PIN_PYRO1_FIRE, OUTPUT); digitalWrite(PIN_PYRO1_FIRE, LOW);
    pinMode(PIN_PYRO2_FIRE, OUTPUT); digitalWrite(PIN_PYRO2_FIRE, LOW);
    pinMode(PIN_PYRO3_FIRE, OUTPUT); digitalWrite(PIN_PYRO3_FIRE, LOW);

    Serial.begin(115200);

    ariadna_rgb_begin(&s_rgb, 60, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

    // Internal I2C1 bus (TCA9534, INA226 x2, ADS1115)
    Wire1.setSDA(PIN_INT_SDA);
    Wire1.setSCL(PIN_INT_SCL);
    Wire1.begin();
    Wire1.setClock(400000);
    Wire1.setTimeout(5);
    (void)exp_begin();

    // Populate initial sensor readings and TX buffer before opening I2C0 slave.
    update_sensors(millis());
    refresh_tx_frame(millis());
    umb_i2c_attach();
}

void loop()
{
    const uint32_t now = millis();

    // 1. Hardware sensors, physical key, continuity, local abort button
    update_sensors(now);

    // 2. Process incoming command from Rocket (I2C0 Slave)
    if (s_cmd_pending) {
        s_cmd_pending = false;
        process_rocket_cmd(s_cmd_rx, s_cmd_rx_ms);
        refresh_tx_frame(now);
    }

    // 3. Clean I2C0 hot-detach right after Rocket reads COMMITTED ACK
    if (s_umb_detach_req) {
        umb_i2c_detach();
    }

    // 4. Heartbeat timeout during un-committed COUNTDOWN -> safe ABORT
    if (s_state == RAMPA_STATE_COUNTDOWN &&
        (uint32_t)(now - s_last_hb_ms) > ARIADNA_RAMPA_TIMEOUT_MS) {
        enter_abort(ARIADNA_RMPF_TIMEOUT);
    }

    // 5. Committed final countdown -> T-0 IGNITION
    if (s_state == RAMPA_STATE_COMMITTED) {
        // Ensure I2C0 is definitely detached at least 20 ms before T-0 even if
        // Rocket's read-back was missed.
        const int32_t rem = (int32_t)(s_t_fire_ms - now);
        if (rem <= 20 && s_umb_enabled) {
            umb_i2c_detach();
        }
        if (rem <= 0) {
            // Final last-microsecond hardware interlock verification!
            if (s_key_armed && !s_btn_pressed && s_vbat_mv >= RAMPA_VBAT_MIN_MV) {
                digitalWrite(PIN_PYRO_ARM, HIGH);
                digitalWrite(pyro_pin_for(s_pyro_ch), HIGH);
                s_fire_start_ms = now;
                s_fire_end_ms   = now + RAMPA_FIRE_PULSE_MS;
                s_cd_running    = false;
                s_state         = RAMPA_STATE_FIRED;
            } else {
                enter_abort(0);
            }
        }
    }

    // 6. Firing pulse end & safe cooldown
    if (s_state == RAMPA_STATE_FIRED) {
        if ((int32_t)(now - s_fire_end_ms) >= 0) {
            pyro_all_off();
            if ((uint32_t)(now - s_fire_end_ms) >= RAMPA_POST_FIRE_MS) {
                s_state = RAMPA_STATE_IDLE;
                umb_i2c_attach(); // safe to accept next rocket connection
            }
        }
    }

    // 7. Refresh cached I2C status frame & visual/audible annunciators
    refresh_tx_frame(now);
    update_outputs(now);
}
