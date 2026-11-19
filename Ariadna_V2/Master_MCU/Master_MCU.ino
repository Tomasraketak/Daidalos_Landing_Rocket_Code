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
//    * holds the PARAMETER TABLE (W25Q128, sectors 0-1) and streams it to the other two
//      boards while the vehicle is on the ground
//    * talks to the diagnostic GUI over USB — only while a host has the
//      port open, and never by waiting for it (section 13)
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

#include "ariadna_link.h"
#include "ariadna_uart_dma.h"

// Vehicle constants come from the guidance module rather than being
// repeated here. The state machine's handover altitude MUST equal the
// altitude the Navigation MCU arms at, and the only way to guarantee that
// is to read it from the same file. `cd ../../test && make sync` keeps
// this copy identical to the verified original.
#include "guidance.h"
#include "ariadna_params.h"
#include "ariadna_seq.h"        // ARIADNA_EV_* event bits, for the downlink
#include "ariadna_batt.h"       // battery state of charge and time to 20 % (fw 3.29)
#include "actcon_pyro.h"        // bench pyro bridge, TEST mode only
#include "ariadna_rampa_link.h" // Rocket <-> Rampa umbilical I2C protocol (fw 3.34)

#include <Arduino.h>
#include "hardware/watchdog.h"
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

// The shared annunciator engine. ARIADNA_RGB_PIN must be set BEFORE the
// include: the header instantiates the pixel object at that pin, and the
// Master's SK6812 is on GP2 rather than on the XIAO's on-board GPIO.
#define ARIADNA_RGB_PIN  2
#include "ariadna_rgb.h" 

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
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

// ariadna_rgb.h instantiates its pixel object at ARIADNA_RGB_PIN, which
// is set immediately above the include at the top of this file. The two
// numbers describe the same wire, so they are checked against each other
// here rather than left to agree by luck.
static_assert(ARIADNA_RGB_PIN == PIN_RGB,
              "ARIADNA_RGB_PIN disagrees with PIN_RGB - the SK6812 will not light");

// ---- subordinate resets ---------------------------------------------
// Active LOW into the Navigation XIAO's RUN pin. Held high in normal
// operation. The Position MCU has no reset line: it boots on its own.
static const uint8_t PIN_RST_NAV = 7;

// PCA9685 /OE, active LOW: HIGH switches every servo output off, LOW
// lets them drive. Held HIGH from the first instruction until the PCA9685
// is configured and every channel holds its parked pulse, so the servos
// never see the chip's power-on garbage, then LOW for good.
static const uint8_t PIN_PCA_OE  = 6;

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
// The ACS712 runs from 5 V behind a 2:1 divider: the bench read 1.252 V
// at GP26 with no current flowing, i.e. 2.504 V at the sensor's output.
// Its zero is ratiometric (Vcc/2), so re-measure if the 5 V rail changes.
static const float ACS712_ZERO_V   = 2.504f;   // output at 0 A, at the sensor
static const float ACS712_V_PER_A  = 0.100f;   // 20 A part: 100 mV/A
static const float ACS712_DIVIDER  = 0.50f;    // pin volts / sensor volts

// RBF polarity, as WIRED on Daidalos (checked on the bench, fw 3.22): the
// pin reads HIGH (internal pull-up) while the tag is FITTED and is pulled
// LOW once the tag is REMOVED. fw <= 3.21 assumed the opposite, so the
// diagnostic GUI showed the tag inverted - and, worse, the arming
// interlock and TEST mode treated a fitted tag as "removed".
//
// Still fail safe: a broken or unplugged RBF wire leaves the pull-up
// reading HIGH, i.e. "fitted", which inhibits arming.
//
// 1 = removed reads HIGH (tag shorts the pin to ground while fitted)
// 0 = removed reads LOW  (Daidalos)
#define RBF_REMOVED_IS_HIGH  0

// ---- PCA9685 ----------------------------------------------------------
static const uint8_t PCA_ADDR = 0x40;

// The internal oscillator is nominally 25 MHz and varies part to part.
// If your measured pulse widths are off by a few percent, trim this rather
// than the pulse tables — it is the actual cause.
static const float   PCA_OSC_HZ   = 25000000.0f;
static const float   PCA_PWM_HZ   = 339.0f;    // see the header note

// The flight servos, by INDEX. Which PCA9685 channel each one is on is a
// PARAMETER since fw 3.28 (srv.ch_*, ids 200..207, in this order), as are
// its hard pulse limits (srv.*_min_us / *_max_us, ids 208..223, a min/max
// pair per servo in this order). See servo_map_update().
enum {
    SV_TVC_X = 0,
    SV_TVC_Y,
    SV_CLAMP,
    SV_COVER,           // aerocover latch
    SV_FIN_0,           // fin 1: on body axis 1 (the TVC X side); fins 2..4 follow,
                        // 90 deg apart toward body axis 2, then opposite 1, opposite 2
    SV_N = SV_FIN_0 + 4,
    PCA_N_CH = 16
};
// Daidalos as wired: TVC X 5, TVC Y 6, clamp 7, cover 8, fins 9..12. Used
// when the parameters do not describe eight different channels 0..15.
static const uint8_t SV_CH_DEFAULT[SV_N] = { 5, 6, 7, 8, 9, 10, 11, 12 };

// ---- servo calibration ------------------------------------------------
//
// TVC_TRIM_US centres the NOZZLE, not the servo horn. Set it with the
// motor installed and the nozzle visibly straight.
//
// TVC_SIGN_* makes a POSITIVE command from the Navigation MCU tilt the
// vehicle toward pad +x / +y. Get either backwards and the attitude loop
// becomes positive feedback and diverges in well under a second. Bench
// test 3 in the README exists to catch exactly this.
//
// ALL OF THESE ARE PARAMETERS NOW (ariadna_params.h, group "srv."), set
// from the diagnostic GUI and kept in the W25Q128 flash, because every one is a
// property of how a horn was pressed onto a spline. The names below are
// kept so the servo code reads as it always did.
static ariadna_params_t s_params;
#define PRM(NAME)        (s_params.v[ARIADNA_P_##NAME])
#define TVC_TRIM_X_US    PRM(SRV_TVC_TRIM_X)
#define TVC_TRIM_Y_US    PRM(SRV_TVC_TRIM_Y)
#define TVC_US_PER_DEG   PRM(SRV_TVC_US_DEG)
#define TVC_SIGN_X       PRM(SRV_TVC_SIGN_X)
#define TVC_SIGN_Y       PRM(SRV_TVC_SIGN_Y)
#define CLAMP_US_CLOSED  PRM(SRV_CLAMP_CLOSED)
#define CLAMP_US_OPEN    PRM(SRV_CLAMP_OPEN)
#define FIN_US_PER_DEG   PRM(SRV_FIN_US_DEG)
#define FIN_TRIM_US(i)   (s_params.v[ARIADNA_P_SRV_FIN1_TRIM + (i)])
#define FIN_SIGN(i)      (s_params.v[ARIADNA_P_SRV_FIN1_SIGN + (i)] < 0.0f ? -1.0f : 1.0f)

// Absolute travel guard. A bad command must not stall a servo against its
// end stop and burn out on the pad. The per-servo limits (srv.*_min_us /
// *_max_us, default 900..2100) are applied first; this only catches what
// they cannot: limits switched off (min >= max) or a corrupt value.
static const float SERVO_US_MIN    = 500.0f;
static const float SERVO_US_MAX    = 2500.0f;

// ---- flash log --------------------------------------------------------
static const uint32_t FLASH_SIZE     = 16UL * 1024UL * 1024UL;  // W25Q128
static const uint32_t FLASH_SECTOR   = 4096;
static const uint32_t FLASH_PAGE     = 256;
// 200 Hz in flight (ASCENT..LANDED: every control tick), 10 Hz on the
// ground (BOOT, IDLE, ARMED, TEST, RECOVERY ...). 96 B x 200 Hz = 19.2 kB/s:
// the 16 MB chip holds ~14 min of flight. Ground logging stops when less
// than LOG_FLIGHT_RESERVE is left, so a long day on the pad can never eat
// the room the flight needs.
static const uint32_t LOG_HZ_FLIGHT  = 200;
static const uint32_t LOG_HZ_GROUND  = 10;
static const uint32_t LOG_FLIGHT_RESERVE = 2UL * 1024UL * 1024UL;   // ~100 s at 200 Hz
// Sectors 0 and 1 hold the parameter table (two copies, see section 13);
// the flight log starts after them. The log never wraps, so it can never
// overwrite them.
static const uint32_t PARAM_SECTOR_A = 0;
static const uint32_t PARAM_SECTOR_B = 1;
static const uint32_t LOG_BASE       = 2 * 4096;

// ---- link supervision -------------------------------------------------
static const uint32_t LINK_TIMEOUT_MS = 300;

// ---- state machine ----------------------------------------------------
// GUIDANCE is entered when the Navigation MCU reports the ignition command.
// If that board cannot be heard, the fallback altitude from guidance.h is
// used instead, so the Position MCU still de-weights the barometer.
static const float GUIDANCE_FALLBACK_M = DG_H_HANDOVER_FALLBACK;

// Everything this board says on USB goes through usb_printf() (section 13):
// a RAM ring drained only as fast as the host takes it, and only while a
// host has the port open. Nothing here can make a control tick wait for a
// laptop.
static void usb_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define MASTER_PRINT(...)                                         \
    do {                                                          \
        _Pragma("GCC diagnostic push")                            \
        _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")  \
        usb_printf(__VA_ARGS__);                                  \
        _Pragma("GCC diagnostic pop")                             \
    } while (0)

// =====================================================================
//  2. STATE
// =====================================================================

// The Arduino builder generates a prototype for every function and puts
// them all ABOVE the first one in the file — before LogRecord is defined
// in section 4. Declaring the type here is what lets log_push(const
// LogRecord *) have a prototype at all.
struct LogRecord;

// Parameter image stored in flash. Defined here (not next to its users) so the
// Arduino-generated function prototypes can see the type.
struct ParamImage {
    uint32_t magic;
    uint32_t seq;
    uint16_t layout;
    uint16_t crc;
    float    v[ARIADNA_PARAM_ID_MAX];
};
static const uint32_t PARAM_MAGIC = 0x44414932;      // "DAI2"

// Forward declarations. The Arduino IDE generates these by itself, but
// `make syntax` compiles this file as plain C++ where it does not - and a
// build that only works inside one IDE cannot be checked on a workstation.
static void params_load(void);
static void params_request_poll(void);
static inline uint16_t counts_to_us(uint16_t c);
static void flash_param_service(void);
static void log_dump_service(void);
static bool log_service_busy(void);
static void send_master_state(void);
static void params_stream(void);
static bool params_synced(void);
static inline bool pos_status_fresh(void);
static void usb_service(void);
static void test_service(void);
static void hil_enter(bool pyro_en);
static void hil_leave(void);
static void diag_emit(void);
static void pyro_bridge_send(uint8_t cmd, uint8_t ch, uint16_t pulse_ms, uint8_t repeats);
static inline bool pyro_rep_fresh(void);
static inline bool nav_status_fresh(void);

static ariadna_rx_t          s_rx_pos;
static ariadna_rx_t          s_rx_nav;

static ariadna_nav_state_t   s_nav;          // newest from Position
static ariadna_actuator_t    s_act;          // newest from Navigation
static ariadna_nav_status_t  s_nav_status;
static ariadna_pos_status_t  s_pos_status;
// Raw sensor data for the GUI (ariadna_raw_imu_t). s_raw_on is the GUI's
// "RAW 1" switch; it goes out in the state broadcast as ARIADNA_MST_RAW_IMU
// and drops by itself when the USB host goes away.
static ariadna_raw_imu_t     s_raw;
static bool                  s_raw_on  = false;
static volatile bool         s_raw_new = false;
static ariadna_uwb_diag_t    s_uwb_diag;
static volatile bool         s_uwb_diag_new = false;
static uint32_t              s_utc_base_ms  = 0;
static uint32_t              s_utc_ref_ms   = 0;
static bool                  s_utc_valid    = false;
static bool                  s_utc_from_gps = false;

static uint32_t s_nav_us = 0, s_act_us = 0;
static bool     s_have_nav = false, s_have_act = false;
static bool     s_have_nav_status = false, s_have_pos_status = false;

static volatile uint8_t s_state = ARIADNA_STATE_BOOT;   // core 1 reads this
static uint8_t  s_flags       = 0;
static uint32_t s_t_launch_ms = 0;
static uint32_t s_state_since = 0;

static float    s_vbat        = 0.0f;
static float    s_current     = 0.0f;
static batt_t   s_batt;                          // 5 min window, estimate refreshed once a minute
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

// The pixel object itself lives in ariadna_rgb.h, so the pattern engine
// and the hardware it drives cannot get out of step between boards.
static ariadna_rgb_t  s_rgb;
static ariadna_buzzer_t s_buzz;

// flash log write pointer (core 1 writes, the USB status only reads)
static volatile uint32_t s_log_addr = 0;
static uint32_t s_log_erased_to     = 0;   // everything below this is erased

// aerocover latch: opened on the Navigation MCU's request, latched here
static bool     s_cover_open  = false;

// parameter sync
static uint16_t s_param_crc   = 0;
static bool     s_params_from_flash = false;

// TEST mode: the diagnostic GUI drives the actuators directly
static bool     s_test        = false;
// HIL (Hardware-in-the-Loop) simulation mode: synthetic sensor flight from PC
static bool     s_hil         = false;
static bool     s_hil_pyro    = false;
// Bench TVC+fin polarity check on real IMU attitude (ground IDLE, pyros OFF)
static bool     s_polarity    = false;
static float    s_pol_thrust_n = 0.0f;  // 0 = default avg(mean, median); 10..250 N override
static float    s_pol_speed_ms = 0.0f;  // 0 = default 40 m/s; 5..70 m/s override
static float    s_pwm_hz      = 339.0f;          // PCA_PWM_HZ outside TEST
static uint16_t s_test_us[16];                   // commanded pulse, 0 = off
static bool     s_test_awake[16];
static uint32_t s_test_moved_ms[16];
static uint16_t s_test_lim_lo[16], s_test_lim_hi[16];
static uint32_t s_test_idle_ms = 500;
static uint32_t s_gui_last    = 0;
static bool     s_gui_alive   = false;
static actcon_pyro_report_t s_pyro_rep;          // from the Navigation MCU, TEST only
static uint32_t s_pyro_rep_ms = 0;
static uint32_t s_pyro_seq    = 1;
static float    s_amps_now    = 0.0f;            // 100 ms mean
static float    s_amps_tick   = 0.0f;            // this tick's sample, unfiltered (servo check)
static float    s_amps_max    = 0.0f;            // peak over 10 s
static uint32_t s_nav_status_ms = 0, s_pos_status_ms = 0;

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

    uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * s_pwm_hz) + 0.5f) - 1;

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
    float c = us * (4096.0f * s_pwm_hz * 1e-6f);
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

// Servo shadow, per CHANNEL. Written by the actuator path, and kept so that
// a lost link leaves the surfaces where they were rather than snapping to a
// default. Channels without a flight servo hold PCA_FULL_OFF (no pulse).
static const uint16_t PCA_FULL_OFF = 0x1000;          // OFF_H bit 4
static uint16_t s_pca[PCA_N_CH];
static uint8_t  s_ch[SV_N]   = { 5, 6, 7, 8, 9, 10, 11, 12 };   // servo -> channel in force
static bool     s_map_ok     = true;                  // the srv.ch_* parameters were usable
// The span of channels that holds every flight servo, written in ONE
// auto-increment I2C transaction (~0.9 ms for 8 channels at 400 kHz, ~1.7 ms
// for all 16). Keep the servos on neighbouring channels and it stays short.
static uint8_t  s_run_lo = 5, s_run_n = 8;

static bool ch_is_flight(uint8_t ch)
{
    for (uint8_t i = 0; i < SV_N; i++) if (s_ch[i] == ch) return true;
    return false;
}

/** The servo's own hard limits; min >= max = none (the absolute guard only). */
static inline float srv_limit(uint8_t sv, float us)
{
    const float lo = s_params.v[ARIADNA_P_SRV_TVC_X_MIN + 2 * sv];
    const float hi = s_params.v[ARIADNA_P_SRV_TVC_X_MIN + 2 * sv + 1];
    if (lo < hi) {
        if (us < lo) us = lo;
        if (us > hi) us = hi;
    }
    return us;
}
static inline void     srv_set(uint8_t sv, float us) { s_pca[s_ch[sv]] = us_to_counts(srv_limit(sv, us)); }
static inline uint16_t srv_get(uint8_t sv)           { return s_pca[s_ch[sv]]; }
static inline bool     servos_write(void)            { return pca_write_run(s_run_lo, &s_pca[s_run_lo], s_run_n); }

static bool pca_full_off(uint8_t ch)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(PCA_LED0_ON_L + 4 * ch));
    Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
    return Wire.endTransmission() == 0;
}

static void servos_park(void)
{
    srv_set(SV_TVC_X, TVC_TRIM_X_US);
    srv_set(SV_TVC_Y, TVC_TRIM_Y_US);
    // Wide open on the pad: the clamp starts the burn open (the startup
    // gate in the guidance), and open is the position that chokes nothing.
    srv_set(SV_CLAMP, CLAMP_US_OPEN);
    for (uint8_t i = 0; i < 4; i++)
        srv_set((uint8_t)(SV_FIN_0 + i), FIN_TRIM_US(i));
    srv_set(SV_COVER, s_cover_open ? PRM(SRV_COVER_UNLOCK) : PRM(SRV_COVER_LOCK));
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
        srv_set(SV_TVC_X, TVC_TRIM_X_US + TVC_SIGN_X * a->tvc_x_deg * TVC_US_PER_DEG);
        srv_set(SV_TVC_Y, TVC_TRIM_Y_US + TVC_SIGN_Y * a->tvc_y_deg * TVC_US_PER_DEG);
    } else if (a->flags & ARIADNA_ACT_TVC_PARK) {
        srv_set(SV_TVC_X, TVC_TRIM_X_US);
        srv_set(SV_TVC_Y, TVC_TRIM_Y_US);
    }

    // The clamp is applied whether or not the solution is valid, because
    // the Navigation MCU FREEZES its clamp output on a bad solution rather
    // than zeroing it. Shutting the clamp during a burn guarantees a crash
    // and the grain finishes burning either way.
    float f = (a->clamp - DG_CLAMP_MIN) / (DG_CLAMP_MAX - DG_CLAMP_MIN);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    srv_set(SV_CLAMP, CLAMP_US_CLOSED + f * (CLAMP_US_OPEN - CLAMP_US_CLOSED));

    // Fins. Not live (ascent with fins parked, a lost navigation solution)
    // means neutral, not "wherever they were": a fin left deflected is a
    // steady torque nobody is correcting.
    const bool fins = (a->flags & ARIADNA_ACT_FINS_LIVE) && (a->flags & ARIADNA_ACT_VALID);
    for (uint8_t i = 0; i < 4; i++) {
        float d = fins ? a->fin_deg[i] : 0.0f;
        if (d >  30.0f) d =  30.0f;
        if (d < -30.0f) d = -30.0f;
        srv_set((uint8_t)(SV_FIN_0 + i), FIN_TRIM_US(i) + FIN_SIGN(i) * d * FIN_US_PER_DEG);
    }

    // Aerocover: a request latches. It is honoured only once the vehicle
    // has actually flown, so a bench packet cannot drop the cover on the pad.
    if ((a->flags & ARIADNA_ACT_COVER_OPEN) && s_state >= ARIADNA_STATE_ASCENT &&
        s_state <= ARIADNA_STATE_GUIDANCE)
        s_cover_open = true;
    srv_set(SV_COVER, s_cover_open ? PRM(SRV_COVER_UNLOCK) : PRM(SRV_COVER_LOCK));

    s_servo_ok = servos_write();
}

/**
 * Put the srv.ch_* parameters into force. Ground only: called at boot and
 * whenever the table changes (PARAM SET / LOAD / DEFAULTS), never in flight.
 * Eight different channels 0..15, or the default map is kept and the
 * servo bus is reported as faulty, which blocks arming.
 */
static void servo_map_update(void)
{
    uint8_t  m[SV_N];
    uint16_t used = 0;
    bool     ok = true;
    for (uint8_t sv = 0; sv < SV_N; sv++) {
        const float v = s_params.v[ARIADNA_P_SRV_CH_TVC_X + sv];
        const int   c = (int)(v + 0.5f);
        if (!(v >= -0.5f) || c < 0 || c >= PCA_N_CH || (used & (1u << c))) { ok = false; break; }
        used = (uint16_t)(used | (1u << c));
        m[sv] = (uint8_t)c;
    }
    if (!ok) memcpy(m, SV_CH_DEFAULT, sizeof(m));
    if (ok != s_map_ok)
        MASTER_PRINT("# servo map: %s\n", ok ? "parameters in force"
                                              : "srv.ch_* not 8 different channels 0..15 - DEFAULT map, servo bus FAULT");
    s_map_ok = ok;
    if (!memcmp(m, s_ch, sizeof(m))) return;

    uint16_t was = 0;
    for (uint8_t sv = 0; sv < SV_N; sv++) was = (uint16_t)(was | (1u << s_ch[sv]));
    memcpy(s_ch, m, sizeof(m));
    uint8_t lo = PCA_N_CH, hi = 0;
    uint16_t now = 0;
    for (uint8_t sv = 0; sv < SV_N; sv++) {
        now = (uint16_t)(now | (1u << m[sv]));
        if (m[sv] < lo) lo = m[sv];
        if (m[sv] > hi) hi = m[sv];
    }
    s_run_lo = lo;
    s_run_n  = (uint8_t)(hi - lo + 1);
    for (uint8_t ch = 0; ch < PCA_N_CH; ch++)
        if (!(now & (1u << ch))) s_pca[ch] = PCA_FULL_OFF;
    servos_park();
    if (!s_test) {                    // in TEST the GUI owns the outputs
        for (uint8_t ch = 0; ch < PCA_N_CH; ch++)
            if ((was & ~now) & (1u << ch)) (void)pca_full_off(ch);
        s_servo_ok = servos_write();
    }
    MASTER_PRINT("# servo map: TVC X %u, TVC Y %u, clamp %u, cover %u, fins %u %u %u %u\n",
                 m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7]);
}

// =====================================================================
//  3b. SERVO CONNECTION CHECK
//
//  Moves ONE servo at a time 5 deg away from its parked position and back,
//  and watches the battery current (ACS712). A connected servo draws a
//  burst of current to move; an unplugged one draws nothing. Started by the
//  RBF tag coming out (mst.servo_check_on_rbf = 1) or by "SERVOCHECK" from
//  the GUI, only in IDLE. Arming waits until it has finished. Nothing moves
//  that could release anything: the cover latch is pushed further INTO its
//  lock, the clamp further toward closed, TVC and fins about their trims.
//
//  Per channel: 300 ms baseline, 300 ms at +5 deg, 300 ms back, 400 ms gap.
//  The current is read every 5 ms tick and averaged over 4 ticks (20 ms):
//  one ADC count is ~16 mA, so a single sample is too noisy to test against
//  mst.servo_check_ma, and a servo moving 5 deg draws for 50-150 ms.
// =====================================================================
enum { SC_IDLE = 0, SC_BASE, SC_MOVE, SC_BACK, SC_GAP };
static const uint16_t SC_TICKS_BASE = 60, SC_TICKS_MOVE = 60, SC_TICKS_BACK = 60, SC_TICKS_GAP = 80;
static uint8_t  s_sc_phase = SC_IDLE;
static uint8_t  s_sc_ch    = 0;              // servo INDEX (SV_*): channel = s_ch[s_sc_ch]
// SRVCHK res[]/ma[] are by servo: TVC X, TVC Y, clamp, cover, fin 1..4
static uint16_t s_sc_tick  = 0;
static float    s_sc_base  = 0.0f, s_sc_peak = 0.0f, s_sc_win[4] = { 0, 0, 0, 0 };
static uint8_t  s_sc_res[SV_N];      // 0 untested, 1 connected, 2 NOT detected
static float    s_sc_ma[SV_N];       // measured rise, mA
static bool     s_sc_report = false;       // a result line is due on USB
static bool     s_sc_ran    = false;       // at least one complete run since boot

static inline bool srvchk_active(void) { return s_sc_phase != SC_IDLE; }

/** Pulse offset for "5 deg" on this servo, signed so nothing is released. */
static float srvchk_delta_us(uint8_t sv)
{
    switch (sv) {
    case SV_TVC_X: case SV_TVC_Y: return 5.0f * TVC_US_PER_DEG;
    case SV_CLAMP: {
        const float d = 5.0f * 10.0f;                       // ~10 us/deg servo
        return CLAMP_US_CLOSED < CLAMP_US_OPEN ? -d : d;   // toward closed
    }
    case SV_COVER: {
        const float d = 5.0f * 10.0f;
        return PRM(SRV_COVER_LOCK) < PRM(SRV_COVER_UNLOCK) ? -d : d;   // deeper into the lock
    }
    default:       return 5.0f * FIN_US_PER_DEG;           // fins
    }
}

static void srvchk_start(void)
{
    if (srvchk_active()) return;
    memset(s_sc_res, 0, sizeof(s_sc_res));
    memset(s_sc_ma, 0, sizeof(s_sc_ma));
    servos_park();
    servos_write();
    s_sc_ch = 0; s_sc_tick = 0; s_sc_phase = SC_BASE;
    s_sc_base = 0.0f; s_sc_peak = -1e9f;
    s_sc_report = true;
    MASTER_PRINT("# servo check: start\n");
}

/** One 5 ms tick. */
static void srvchk_service(void)
{
    if (!srvchk_active()) return;
    if (s_state != ARIADNA_STATE_IDLE) {             // TEST, ARMED, ERROR...: give up
        s_sc_phase = SC_IDLE;
        servos_park();
        servos_write();
        s_sc_report = true;
        return;
    }
    // 20 ms running mean of the current
    s_sc_win[s_sc_tick & 3] = s_amps_tick;
    const float a20 = 0.25f * (s_sc_win[0] + s_sc_win[1] + s_sc_win[2] + s_sc_win[3]);
    const uint16_t t = ++s_sc_tick;
    const uint16_t park = srv_get(s_sc_ch);

    switch (s_sc_phase) {
    case SC_BASE:
        if (t > 4) s_sc_base += s_amps_tick;              // skip the window warm-up
        if (t >= SC_TICKS_BASE) {
            s_sc_base /= (float)(SC_TICKS_BASE - 4);
            uint16_t c = us_to_counts(srv_limit(s_sc_ch, counts_to_us(park) + srvchk_delta_us(s_sc_ch)));
            pca_write_run(s_ch[s_sc_ch], &c, 1);
            s_sc_phase = SC_MOVE; s_sc_tick = 0;
        }
        break;
    case SC_MOVE:
    case SC_BACK:
        if (t > 3 && a20 > s_sc_peak) s_sc_peak = a20;
        if (t >= (s_sc_phase == SC_MOVE ? SC_TICKS_MOVE : SC_TICKS_BACK)) {
            if (s_sc_phase == SC_MOVE) {
                pca_write_run(s_ch[s_sc_ch], &park, 1);         // and back
                s_sc_phase = SC_BACK;
            } else {
                const float rise_ma = (s_sc_peak - s_sc_base) * 1000.0f;
                s_sc_ma[s_sc_ch]  = rise_ma;
                s_sc_res[s_sc_ch] = rise_ma >= PRM(MST_SRV_CHECK_MA) ? 1 : 2;
                s_sc_report = true;
                s_sc_phase = SC_GAP;
            }
            s_sc_tick = 0;
        }
        break;
    case SC_GAP:
        if (t >= SC_TICKS_GAP) {
            if (++s_sc_ch >= SV_N) {
                s_sc_phase = SC_IDLE;
                s_sc_ran = true;
                s_sc_report = true;
                MASTER_PRINT("# servo check: done\n");
            } else {
                s_sc_phase = SC_BASE;
                s_sc_base = 0.0f; s_sc_peak = -1e9f;
            }
            s_sc_tick = 0;
        }
        break;
    default: break;
    }
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
    uint32_t lo = LOG_BASE / FLASH_SECTOR; // known written (or the start)
    uint32_t hi = FLASH_SIZE / FLASH_SECTOR;

    flash_read(LOG_BASE, buf, sizeof(buf));
    bool first_erased = true;
    for (uint8_t i = 0; i < sizeof(buf); i++) if (buf[i] != 0xFF) first_erased = false;
    if (first_erased) return LOG_BASE;     // empty log

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
 * 96 bytes at 50 Hz is 4.8 kB/s, so a 16 MB chip holds about an hour.
 *
 * px/py are logged even though no control law reads them, because the
 * question this log exists to answer after a flight — "where did it
 * actually go?" — is a horizontal one, and the downlink's 1e-7 degree
 * lat/lon is a clumsy way to ask it. Metres from the pad plot directly.
 */
#pragma pack(push, 1)
struct LogRecord {
    uint32_t t_ms;
    float    px, py;
    float    h, vx, vy, vz;
    float    ux, uy, uz;
    float    roll, pitch, yaw;
    int32_t  lat, lon;
    float    tvc_x, tvc_y, clamp;
    float    thrust_est, k_plan;
    uint16_t vbat_mv, current_ma;
    uint16_t nav_flags;
    uint8_t  state, nav_phase, pyro, health;
    uint8_t  events;           // ARIADNA_EV_* from the sequencer
    int8_t   t_event_ds;       // countdown, 0.1 s, saturating
    uint8_t  fin_gain_pct;     // estimated fin effectiveness * 100
    uint8_t  drag_pct;         // measured / modelled free-fall drag * 100
    uint8_t  tscale_pct;       // measured / table thrust * 100
    uint8_t  flags;            // bit 0: flight-rate record (ASCENT..LANDED)
};
#pragma pack(pop)
ARIADNA_STATIC_ASSERT(sizeof(LogRecord) == 96, "LogRecord layout changed");

// RAM ring, drained to flash by core 1.
//
// Sized for the worst case that actually happens: a 400 ms sector erase
// during which nothing can be programmed. At 50 Hz x 88 B that is 1.8 kB
// of arrivals, so 8 kB is roughly a 4x margin — and it also covers a LoRa
// transmission landing in the same window.
// 32 kB: 1.7 s of 200 Hz records, against a 400 ms worst-case sector erase.
static const uint32_t LOG_RING_SZ = 32768;
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
// ---------------------------------------------------------------------
//  The pad crew's instrument panel
//
//  One colour per MISSION state, so the vehicle can be read from outside
//  the safety line without a laptop. The Position MCU's own LED reports
//  the NAVIGATION state, which is a different question — see the
//  annunciator section in Position_MCU.ino.
//
//    solid blue        BOOT
//    slow green        IDLE, safe, RBF fitted
//    fast red          ARMED. Stay behind the line.
//    solid amber       ASCENT
//    solid yellow      APOGEE
//    solid magenta     DESCENT
//    solid white       GUIDANCE — the Navigation MCU has authority
//    solid green       LANDED
//    slow green        RECOVERY, with the beacon
//    fast amber        ABORT
//    slow blue         TEST mode, pyros inhibited
//    solid red         ERROR
//
//  The state colours are the same ones the V1 flight computer used. They
//  are not changed here because the people who read them have not.
// ---------------------------------------------------------------------

// Beacon and warning sequences. Expressed as data so that nothing in the
// vehicle ever calls delay() to make a noise — ariadna_buzzer_service()
// advances them from the main loop.
static const ariadna_beep_step_t BEEP_RECOVERY[] = {
    { 2700, 150 }, { 0, 850 }
};
static const ariadna_beep_step_t BEEP_ARMED[] = {
    { 1800,  40 }, { 0, 960 }
};

// ---------------------------------------------------------------------
//  Start-up tune and start-up verdict (fw 3.31)
//
//  1. Right after power-up a ~1.6 s rising tune says "the Master is alive".
//  2. Then the Master waits until the whole vehicle has finished its own
//     start-up (Position MCU: sensors up and gyro calibrated at boot;
//     Navigation MCU talking; the parameter table in sync on all boards; its
//     own POST, servo bus and flash) and plays a 1.0 s verdict:
//        OK   - two rising tones and a long high one
//        FAIL - three falling low tones
//     No verdict is played before the first one is decided, and none is played
//     twice per power-up. A board that is still not ready 30 s after power-up
//     gets the FAIL tune; the console's "Kontrola po startu" row says what.
//  Skipped entirely if the Master resets while a flight is in progress.
// ---------------------------------------------------------------------
static const ariadna_beep_step_t BEEP_STARTUP[] = {
    { 1047, 300 }, { 1319, 300 }, { 1568, 300 }, { 2093, 700 }          // 1.6 s
};
static const ariadna_beep_step_t BEEP_START_OK[] = {
    { 1568, 200 }, { 2093, 200 }, { 2637, 600 }                          // 1.0 s
};
static const ariadna_beep_step_t BEEP_START_FAIL[] = {
    { 440, 300 }, { 0, 100 }, { 330, 300 }, { 0, 100 }, { 220, 200 }     // 1.0 s
};

#define MST_SU_TIMEOUT_MS  30000u
// bits of s_su_mask: what was NOT ready
#define MST_SU_POST    (1u << 0)     // Master self test
#define MST_SU_SERVO   (1u << 1)     // PCA9685 / servo map
#define MST_SU_FLASH   (1u << 2)     // flash chip (log, parameters)
#define MST_SU_POS     (1u << 3)     // Position MCU not reporting
#define MST_SU_POSCAL  (1u << 4)     // Position: IMU / barometer / gyro calibration not done
#define MST_SU_NAV     (1u << 5)     // Navigation MCU not reporting
#define MST_SU_PARAMS  (1u << 6)     // parameter table not in sync on all boards
static uint8_t  s_su_res  = 0;       // 0 deciding, 1 OK, 2 FAIL, 3 skipped (reset in flight)
static uint8_t  s_su_mask = 0xFF;
static uint32_t s_su_t0 = 0, s_su_hold_until = 0;

static bool startup_tune_busy(void)
{
    return s_su_t0 != 0 && (int32_t)(millis() - s_su_hold_until) < 0;
}

static uint8_t startup_missing(void)
{
    uint8_t m = 0;
    if (!s_post_pass)                    m |= MST_SU_POST;
    if (!s_servo_ok || !s_map_ok)        m |= MST_SU_SERVO;
    if (!s_flash_ok)                     m |= MST_SU_FLASH;
    if (!pos_status_fresh())             m |= MST_SU_POS;
    else if (!((s_pos_status.sensor_ok & ARIADNA_SENS_IMU) &&
               (s_pos_status.sensor_ok & ARIADNA_SENS_BARO) &&
               s_pos_status.cal_state >= 1)) m |= MST_SU_POSCAL;
    if (!nav_status_fresh())             m |= MST_SU_NAV;
    if (!params_synced())                m |= MST_SU_PARAMS;
    return m;
}

static void startup_tune_service(void)
{
    if (s_su_res == 3) return;
    const uint32_t now = millis();
    if (s_su_t0 == 0) {
        s_su_t0 = now ? now : 1;
        if (s_state >= ARIADNA_STATE_ASCENT && s_state <= ARIADNA_STATE_RECOVERY) { s_su_res = 3; return; }
        ariadna_buzzer_play(&s_buzz, BEEP_STARTUP, 4, 0, 0);
        s_su_hold_until = now + 1700;            // 1.6 s tune + a short gap
        return;
    }
    if (s_su_res != 0 || (int32_t)(now - s_su_hold_until) < 0) return;

    s_su_mask = startup_missing();
    if (s_su_mask == 0) {
        s_su_res = 1;
    } else if (!s_post_pass || !s_servo_ok || !s_flash_ok || (now - s_su_t0) > MST_SU_TIMEOUT_MS) {
        s_su_res = 2;                            // hard faults at once, the rest after the timeout
    } else {
        return;                                  // others still starting: keep waiting
    }
    if (s_su_res == 1) ariadna_buzzer_play(&s_buzz, BEEP_START_OK, 3, 0, 0);
    else               ariadna_buzzer_play(&s_buzz, BEEP_START_FAIL, 5, 0, 0);
    s_su_hold_until = now + 1100;
    usb_printf("# startup check %s (not ready mask 0x%02X: 1 POST, 2 servo, 4 flash, 8 Position, 16 Position cal, 32 Navigation, 64 params)\n",
               s_su_res == 1 ? "OK" : "FAIL", (unsigned)s_su_mask);
}

static void annunciate(void)
{
    // Rate limited because choosing the message is the cheap part; the
    // engine below only touches the LED when the pixel actually changes.
    static uint32_t next_ms = 0;
    if (millis() < next_ms) return;
    next_ms = millis() + 50;

    switch (s_state) {
    case ARIADNA_STATE_BOOT:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE,    ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_IDLE:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN,   ARIADNA_PAT_SLOW,  0); break;
    case ARIADNA_STATE_ARMED:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED,     ARIADNA_PAT_FAST,  0); break;
    case ARIADNA_STATE_ASCENT:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER,   ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_APOGEE:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_YELLOW,  ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_DESCENT:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_MAGENTA, ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_GUIDANCE:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_WHITE,   ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_LANDED:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN,   ARIADNA_PAT_SOLID, 0); break;
    case ARIADNA_STATE_RECOVERY:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN,   ARIADNA_PAT_SLOW,  0); break;
    case ARIADNA_STATE_ABORT:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER,   ARIADNA_PAT_FAST,  0); break;
    case ARIADNA_STATE_TEST:
        if (pyro_rep_fresh() && s_pyro_rep.firing_mask)
            ariadna_rgb_set(&s_rgb, ARIADNA_C_RED,   ARIADNA_PAT_SOLID, 0);
        else if (pyro_rep_fresh() && s_pyro_rep.armed)
            ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_FAST,  0);
        else
            ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE,  ARIADNA_PAT_SLOW,  0);
        break;
    default:
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED,     ARIADNA_PAT_SOLID, 0); break;
    }

    // The recovery beacon. Loud, periodic, and only after landing — a
    // buzzer during the flight is just battery drain, and it is the one
    // thing that finds a vehicle in long grass.
    //
    // min_gap_ms is what lets these be called unconditionally from a
    // state that persists: a sequence already running is not restarted.
    if (startup_tune_busy()) return;         // the start-up tune / verdict owns the buzzer
    if (s_state == ARIADNA_STATE_RECOVERY) {
        ariadna_buzzer_play(&s_buzz, BEEP_RECOVERY, 2, 1, 2000);
    } else if (s_state == ARIADNA_STATE_ARMED ||
               (s_state == ARIADNA_STATE_TEST && pyro_rep_fresh() && s_pyro_rep.armed)) {
        ariadna_buzzer_play(&s_buzz, BEEP_ARMED, 2, 1, 2000);
    } else {
        ariadna_buzzer_stop(&s_buzz);
    }
}

// =====================================================================
//  6. LINKS
// =====================================================================
// DMA-fed transmit, one channel per port.
//
// This is the single biggest processor saving in the vehicle. Forwarding a
// 94-byte nav_state means 62 bytes past the UART's 32-byte FIFO, which a
// blocking write spins on for 673 us — 16.2 % of this core at 240 Hz, and
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
            if (ariadna_unpack(pl, len, &s_pos_status, sizeof(s_pos_status))) {
                s_have_pos_status = true;
                s_pos_status_ms   = millis();
            }
        } else if (type == ARIADNA_PKT_RAW_IMU) {
            // Stored only; printed by diag_emit(), outside the forwarding path.
            if (ariadna_unpack(pl, len, &s_raw, sizeof(s_raw))) s_raw_new = true;
        } else if (type == ARIADNA_PKT_UWB_DIAG) {
            // Stored only; printed by diag_emit() as a "U" JSON line at 2 Hz.
            if (ariadna_unpack(pl, len, &s_uwb_diag, sizeof(s_uwb_diag))) s_uwb_diag_new = true;
        } else if (type == ARIADNA_PKT_GPS_TIME) {
            ariadna_gps_time_t gt;
            if (ariadna_unpack(pl, len, &gt, sizeof(gt)) && gt.valid) {
                s_utc_base_ms  = gt.utc_ms % 86400000u;
                s_utc_ref_ms   = millis();
                s_utc_valid    = true;
                s_utc_from_gps = true;
            }
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
static void rampa_on_status(const ariadna_rampa_status_t &st, uint32_t now_ms);

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
                // In TEST the diagnostic GUI owns the servo outputs, and
                // during the servo connection check that check does.
                if (!s_test && !srvchk_active()) servos_apply(&s_act);
            }
        } else if (type == ARIADNA_PKT_NAV_STATUS) {
            if (ariadna_unpack(pl, len, &s_nav_status, sizeof(s_nav_status))) {
                s_have_nav_status = true;
                s_nav_status_ms   = millis();
            }
        } else if (type == ARIADNA_PKT_RAMPA_STATUS) {
            ariadna_rampa_status_t rst;
            if (ariadna_rampa_status_decode(pl, len, &rst) == ARIADNA_RAMPA_OK)
                rampa_on_status(rst, millis());
        } else if (type == ACTCON_PKT_PYRO_REPORT) {
            if (ariadna_unpack(pl, len, &s_pyro_rep, sizeof(s_pyro_rep)))
                s_pyro_rep_ms = millis();
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

static float    s_bacc_i = 0.0f, s_bacc_v = 0.0f;
static uint16_t s_bacc_n = 0;
static float    s_learn_stored = 1.0f;         // batt.learn as last seen / written

static inline void batt_accumulate(float volts, float amps)
{
    s_bacc_i += amps;
    s_bacc_v += volts;
    if (s_bacc_n < 0xFFFF) s_bacc_n++;
}

static void read_analogue(void)
{
    // Slow-averaged: these feed telemetry and the arming interlock, not a
    // control loop, and the ADC on this chip is noisy enough to matter.
    static float vbat_f = 0.0f, curr_f = 0.0f;

    float v_adc = (float)analogRead(PIN_V_BAT) * (ADC_VREF / ADC_FULL_SCALE);
    float vbat  = v_adc * VBAT_DIVIDER;

    float i_adc = (float)analogRead(PIN_I_SENSE) * (ADC_VREF / ADC_FULL_SCALE);
    float amps  = ((i_adc / ACS712_DIVIDER) - ACS712_ZERO_V) / ACS712_V_PER_A;

    s_amps_tick = amps;
    batt_accumulate(vbat, amps);                     // battery estimate: mean of 200 Hz samples
    vbat_f += 0.02f * (vbat - vbat_f);
    curr_f += 0.02f * (amps - curr_f);
    s_vbat    = vbat_f;
    s_current = curr_f;

    // 100 ms mean and the peak over ten 1 s buckets, for the bench: a
    // stalled servo shows as a peak long before it shows as smoke.
    {
        static float acc = 0.0f, fast = 0.0f, bucket[10];
        static uint8_t n = 0, bi = 0;
        static uint32_t bucket_t0 = 0;
        fast += 0.5f * (amps - fast);
        acc += amps;
        if (++n >= 20) { s_amps_now = acc / 20.0f; acc = 0.0f; n = 0; }
        if (fast > bucket[bi]) bucket[bi] = fast;
        if (millis() - bucket_t0 >= 1000) {
            bucket_t0 = millis();
            bi = (uint8_t)((bi + 1) % 10);
            bucket[bi] = fast;
        }
        float m = bucket[0];
        for (uint8_t i = 1; i < 10; i++) if (bucket[i] > m) m = bucket[i];
        s_amps_max = m;
    }

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
                 s_state, s, (unsigned long)millis(), s_nav.pz, s_nav.vz);
    if (s != ARIADNA_STATE_IDLE) s_polarity = false;
    s_state = s;
    s_state_since = millis();
}

static bool s_arm_request = false;      // FLIGHTARM from the GUI, when mst.auto_arm = 0

static inline bool nav_status_fresh(void)
{
    return s_have_nav_status && (millis() - s_nav_status_ms) < 500;
}
static inline bool pos_status_fresh(void)
{
    return s_have_pos_status && (millis() - s_pos_status_ms) < 500;
}

/**
 * Battery run-time estimate (common/ariadna_batt.h).
 *   - current and voltage: every 200 Hz tick of read_analogue() adds its raw
 *     sample to a sum (batt_accumulate), once a second the mean goes to the
 *     estimator - 200 samples a second, not one;
 *   - air temperature: the Position MCU's barometer, read every 30 s and held
 *     (it changes slowly); without a fresh status packet the model uses 20 C;
 *   - the estimator turns a 5-minute mean into a new estimate once a minute and,
 *     from the charge it counts, corrects the battery health (batt.learn).
 *     A learned value is written to the table - and, on the ground in IDLE, to
 *     flash - so it survives a power cycle.
 */
static bool params_save_request(void);       // defined with the flash parameter code (section 9)

static void batt_service(void)
{
    static uint32_t next_ms = 0, next_t_ms = 0, last_save_ms = 0;
    static float    t_hold = BATT_T_DEFAULT_C;
    static uint8_t  seen_events = 0;
    const uint32_t now = millis();
    if (now < 3000) { s_bacc_i = s_bacc_v = 0.0f; s_bacc_n = 0; return; }   // ADC filter settling
    if ((int32_t)(now - next_ms) < 0) return;
    next_ms = now + 1000;
    if (s_bacc_n == 0) return;
    const float amps  = s_bacc_i / (float)s_bacc_n;
    const float volts = s_bacc_v / (float)s_bacc_n;
    s_bacc_i = s_bacc_v = 0.0f; s_bacc_n = 0;

    if ((int32_t)(now - next_t_ms) >= 0) {                 // temperature: every 30 s
        next_t_ms = now + 30000;
        if (pos_status_fresh()) t_hold = s_pos_status.temp_c;
    }

    // batt.learn changed from outside (the GUI, a restored image): take it over
    if (fabsf(PRM(BATT_LEARN) - s_learn_stored) > 1e-4f) {
        s_learn_stored = PRM(BATT_LEARN);
        batt_set_learn(&s_batt, s_learn_stored);
        batt_learn_drop(&s_batt);
    }

    batt_cfg_t c;
    c.cap_mah    = PRM(BATT_CAP_MAH);
    c.health     = PRM(BATT_HEALTH);
    c.r25_mohm   = PRM(BATT_R25_MOHM);
    c.cutoff_pct = PRM(BATT_CUTOFF_PCT);
    (void)batt_feed_1hz(&s_batt, &c, volts, amps, t_hold);

    // a stretch was judged: keep the result, on the ground, at most every 10 min
    if (s_batt.learn_events != seen_events) {
        seen_events = s_batt.learn_events;
        if (fabsf(s_batt.learn - s_learn_stored) >= 0.01f &&
            (s_state == ARIADNA_STATE_IDLE) && !s_test && !s_hil &&
            (last_save_ms == 0 || now - last_save_ms > 600000u)) {
            if (ariadna_param_set(&s_params, ARIADNA_P_BATT_LEARN, s_batt.learn)) {
                s_learn_stored = PRM(BATT_LEARN);
                s_param_crc = ariadna_params_crc(&s_params);     // streamed to the other boards
                if (params_save_request()) last_save_ms = now;
            }
        } else if (fabsf(s_batt.learn - s_learn_stored) >= 0.01f && s_state != ARIADNA_STATE_IDLE) {
            seen_events = (uint8_t)(seen_events - 1);          // not on the ground: try again at the next minute
        }
    }
}

/** Both other boards report the CRC of the parameter table held here. */
static bool params_synced(void)
{
    return nav_status_fresh() && pos_status_fresh() &&
           s_nav_status.param_crc == s_param_crc &&
           s_pos_status.param_crc == s_param_crc;
}

static void state_machine(void)
{
    const bool nav_ok  = s_have_nav && link_alive(s_nav_us) &&
                         (s_nav.flags & ARIADNA_NAV_VALID);
    const bool on_pad  = s_have_nav && fabsf(s_nav.vz) < 2.0f && s_nav.pz < 5.0f;
    const bool bw_open = digitalRead(PIN_BREAKWIRE) == HIGH;

    if (bw_open) s_flags |= ARIADNA_MST_BREAKWIRE;
    if (s_rbf_out) s_flags |= ARIADNA_MST_RBF_OUT;
    else           s_flags &= (uint8_t)~ARIADNA_MST_RBF_OUT;

    // Hardware RBF kill-switch for HIL with live pyrotechnics: reinserting
    // the RBF pin immediately inhibits pyros even mid-simulation.
    if (s_hil && s_hil_pyro && !s_rbf_out) {
        s_hil_pyro = false;
        s_flags |= ARIADNA_MST_PYRO_INHIBIT;
        send_master_state();
        usb_printf("# HIL RBF reinserted - pyros inhibited\n");
    }

    switch (s_state) {

    case ARIADNA_STATE_BOOT:
        if (millis() - s_state_since > 2000)
            set_state(s_post_pass ? ARIADNA_STATE_IDLE : ARIADNA_STATE_ERROR);
        break;

    case ARIADNA_STATE_IDLE: {
        s_flags &= (uint8_t)~(ARIADNA_MST_ARMED | ARIADNA_MST_TEST | ARIADNA_MST_PYRO_INHIBIT);
        s_cover_open = false;                   // the latch re-engages on the ground
        // Servo connection check when the RBF tag comes out (section 3b).
        static bool rbf_was_out = true;          // no check for a tag already out at boot
        if (s_rbf_out && !rbf_was_out && PRM(MST_SRV_CHECK_ARM) > 0.5f) srvchk_start();
        rbf_was_out = s_rbf_out;
        if (srvchk_active()) break;             // arming waits for the check
        // Arming needs the RBF tag out, a healthy navigation solution, and
        // the vehicle actually sitting still on the pad. Any one missing
        // and it stays safe.
        // ... and the other two boards must hold THIS board's parameter
        // table. A Navigation MCU flying yesterday's timings is not armed.
        if (s_rbf_out && nav_ok && on_pad && !bw_open && params_synced() &&
            (PRM(MST_AUTO_ARM) > 0.5f || s_arm_request)) {
            s_arm_request = false;
            // Capture the pad reference at the moment of arming, not at
            // boot: the vehicle may have sat on the pad for an hour while
            // the weather moved.
            broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);
            broadcast_cmd(ARIADNA_CMD_ARM, 0, 0);
            s_flags |= ARIADNA_MST_ARMED;
            set_state(ARIADNA_STATE_ARMED);
        }
        break;
    }

    case ARIADNA_STATE_ARMED:
        s_flags |= ARIADNA_MST_ARMED;
        if (!s_hil && !s_rbf_out) {             // tag went back in (real flight)
            s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
            broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
            set_state(ARIADNA_STATE_IDLE);
            break;
        }
        // Launch: the breakwire parts, or the vehicle is simply gone. The
        // second test exists because a breakwire that fails to part is a
        // known failure and it must not strand the state machine on the
        // pad while the rocket is in the air. In HIL on a bench the physical
        // breakwire is usually unplugged (HIGH), so launch relies solely on
        // the synthetic IMU acceleration or altitude.
        if ((!s_hil && bw_open) || (nav_ok && s_nav.accel_mag > PRM(MST_LAUNCH_G)) ||
            (nav_ok && s_nav.pz > 15.0f)) {
            s_t_launch_ms = millis();
            broadcast_cmd(ARIADNA_CMD_ZERO_CLOCK, 0, s_t_launch_ms);
            set_state(ARIADNA_STATE_ASCENT);
        }
        break;

    case ARIADNA_STATE_ASCENT:
        if (nav_ok && s_nav.vz < 0.0f && s_nav.pz > 20.0f)
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
        if ((nav_status_fresh() && s_nav_status.phase >= DG_PHASE_IGNITING) ||
            (!nav_status_fresh() && nav_ok && s_nav.pz <= GUIDANCE_FALLBACK_M &&
             s_nav.vz < -2.0f))
            set_state(ARIADNA_STATE_GUIDANCE);
        break;

    case ARIADNA_STATE_GUIDANCE:
        // Down and stopped, or the guidance module says it is finished.
        if ((nav_ok && s_nav.pz < 3.0f && fabsf(s_nav.vz) < 1.5f &&
             millis() - s_state_since > 1000) ||
            (nav_status_fresh() && s_nav_status.phase == DG_PHASE_DONE))
            set_state(ARIADNA_STATE_LANDED);
        break;

    case ARIADNA_STATE_LANDED:
        s_flags &= (uint8_t)~ARIADNA_MST_ARMED;     // down: no pyro may fire any more
        servos_park();
        servos_write();
        if (millis() - s_state_since > 5000)
            set_state(ARIADNA_STATE_RECOVERY);
        break;

    case ARIADNA_STATE_TEST:
        // Entered and left only from the diagnostic GUI (section 13).
        s_flags |= ARIADNA_MST_TEST;
        s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
        break;

    case ARIADNA_STATE_RECOVERY:
    case ARIADNA_STATE_ABORT:
    case ARIADNA_STATE_ERROR:
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
    if (s_servo_ok && s_map_ok)                  h |= ARIADNA_HLT_SERVO_OK;
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

    t->alt_dm  = sat16(s_nav.pz * 10.0f);
    t->vz_dms  = sat16(s_nav.vz * 10.0f);
    t->vh_dms  = sat16(sqrtf(s_nav.vx * s_nav.vx + s_nav.vy * s_nav.vy) * 10.0f);
    t->acc_cg  = sat16(s_nav.accel_mag * 100.0f);

    // Attitude: the quaternion from the 240 Hz state, sign fixed so w >= 0.
    {
        float sgn = (s_nav.qw < 0.0f) ? -32767.0f : 32767.0f;
        t->qx = sat16(s_nav.qx * sgn);
        t->qy = sat16(s_nav.qy * sgn);
        t->qz = sat16(s_nav.qz * sgn);
    }

    t->tvc_x_cdeg = sat16(s_act.tvc_x_deg * 100.0f);
    t->tvc_y_cdeg = sat16(s_act.tvc_y_deg * 100.0f);
    t->clamp_pct  = (uint8_t)(s_act.clamp * 100.0f);
    t->nav_phase  = s_act.phase;

    t->thrust_n = sat16(s_have_nav_status ? s_nav_status.thrust_est : 0.0f);

    float vb = (s_vbat - 5.0f) * 50.0f;        // 5.0..10.1 V, 20 mV steps
    if (vb < 0.0f)   vb = 0.0f;
    if (vb > 255.0f) vb = 255.0f;
    t->vbat_cv = (uint8_t)vb;

    float ca = s_current * 10.0f;
    if (ca < 0.0f)   ca = 0.0f;
    if (ca > 255.0f) ca = 255.0f;
    t->curr_da = (uint8_t)ca;

    t->hdop_x10 = s_pos_status.hdop_x10;
    t->sats     = s_pos_status.sats;

    // UWB constellation health, one byte. Built from BOTH status sources
    // on purpose: the anchor count and the solution mode come from the
    // Position MCU's 20 Hz housekeeping, while "the filter accepted a
    // fix" is a flag on the 240 Hz navigation state. Reading the second
    // without the first is how you end up believing a constellation is
    // working because it was working half a second ago.
    {
        uint8_t u = 0;
        if (s_have_pos_status) {
            uint8_t n = s_pos_status.uwb_anchors;
            if (n > 15) n = 15;
            u |= n;
            u |= (uint8_t)((s_pos_status.uwb_mode & 0x03) << ARIADNA_TUWB_MODE_SHIFT);
            if (s_pos_status.sensor_ok & ARIADNA_SENS_UWB) u |= ARIADNA_TUWB_ALIVE;
        }
        if (s_have_nav && (s_nav.flags & ARIADNA_NAV_UWB_AIDED))
            u |= ARIADNA_TUWB_AIDED;
        t->uwb = u;
    }

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
    else if (!s_servo_ok || !s_map_ok)                   err = 5;
    else if (s_state <= ARIADNA_STATE_ARMED && !params_synced()) err = 7;
    else if (s_vbat > 0.5f && s_vbat < 7.0f)             err = 6;   // low pack
    f |= (uint8_t)((err << ARIADNA_TF_ERR_SHIFT) & ARIADNA_TF_ERR_MASK);
    t->flags = f;

    // What the sequencer has done, and the countdown to what it does next.
    t->t_event_ds = ARIADNA_TLM_T_UNKNOWN;
    if (nav_status_fresh()) {
        t->events = s_nav_status.events;
        float te = (s_nav_status.phase >= DG_PHASE_IGNITING) ? s_nav_status.t_to_td
                                                             : s_nav_status.t_to_ign;
        if (te >= 0.0f) t->t_event_ds = sat16(te * 10.0f);
    }
    if (s_cover_open) t->events |= ARIADNA_EV_COVER;
    if (s_t_launch_ms) {
        uint32_t tf = (millis() - s_t_launch_ms) / 1000u;
        t->t_flight_s = (uint8_t)(tf > 255 ? 255 : tf);
    }
    {
        float px = s_nav.px, py = s_nav.py;
        if (px >  127.0f) px =  127.0f;
        if (px < -127.0f) px = -127.0f;
        if (py >  127.0f) py =  127.0f;
        if (py < -127.0f) py = -127.0f;
        t->px_m = (int8_t)px;
        t->py_m = (int8_t)py;
    }
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
        usb_printf("POST PCA9685 not responding\n");
        ok = false;
    }

    // Pulse arithmetic. A wrong prescale or a wrong counts conversion
    // silently halves every servo's travel.
    uint16_t c1500 = us_to_counts(1500.0f);
    uint16_t c1000 = us_to_counts(1000.0f);
    float expect1500 = 1500.0f * (4096.0f * s_pwm_hz * 1e-6f);
    if (fabsf((float)c1500 - expect1500) > 2.0f) {
        MASTER_PRINT("POST pulse 1500us -> %u want %.0f\n", c1500, expect1500);
        ok = false;
    }
    if (!(c1000 < c1500)) {
        usb_printf("POST pulse conversion not monotone\n");
        ok = false;
    }
    // The travel guard must actually clamp.
    if (us_to_counts(400.0f) != us_to_counts(SERVO_US_MIN) ||
        us_to_counts(3000.0f) != us_to_counts(SERVO_US_MAX)) {
        usb_printf("POST servo travel guard not clamping\n");
        ok = false;
    }

    if (!s_flash_ok) {
        usb_printf("POST flash log unavailable\n");
        // Not fatal: the vehicle can fly without a log, and refusing to
        // fly over a logging fault would be the wrong trade. It is
        // reported in the telemetry error code instead.
    }

    return ok;
}

// =====================================================================
//  13. PARAMETERS, USB DIAGNOSTICS AND TEST MODE
//
//  ---------------------------------------------------------------------
//  THE RULE: USB MAY NEVER COST THE FLIGHT ANYTHING
//  ---------------------------------------------------------------------
//  * Nothing is sent unless a host has the port open (`Serial` is true
//    only with DTR asserted). In flight there is no host, so this whole
//    section is two comparisons per tick.
//  * Text is queued into a RAM ring and handed to the USB stack only in
//    the amounts it says it can take right now. There is no call in this
//    file that waits for the host. A full ring drops the LINE, whole.
//  * Commands are parsed a character at a time, never waited for.
//  * Everything that moves an actuator or touches a pyro from here needs
//    TEST mode, and TEST mode can only be entered on the ground.
//
//  ---------------------------------------------------------------------
//  THE PROTOCOL  (115200, one command per line; see the User Manual)
//  ---------------------------------------------------------------------
//    HELLO                       -> I ACTCON 1 16 / I DAIDALOS <fw> ...
//    STATUS                      telemetry lines now
//    TEST 1 | TEST 0             enter / leave TEST mode
//    PARAM LIST                  -> PRM <id> <key> <val> <def> <min> <max> <unit> <owner>
//    PARAM SET <id|key> <value>  (IDLE / TEST only)
//    PARAM SAVE | LOAD | DEFAULTS
//    PADREF                      capture the pad reference now
//    CAL GYRO                    gyro calibration (vehicle still)
//    LOG INFO | FLIGHTS | DUMP [from [count]] | STOP | ERASE
//                                flight log: LOGR <n> <96 bytes hex> lines,
//                                then OK LOG DUMP <n>; not while logging
//    FLIGHTARM 1 | 0             arm request when mst.auto_arm = 0 / disarm
//    HIL 1 [0|1] | HIL 0 | HIL STATUS
//                                enter / leave Hardware-in-the-Loop flight
//                                simulation (2nd arg: 1 enables physical pyros,
//                                requires RBF removed; default 0 = inhibited)
//    HB                          heartbeat (any line counts)
//   TEST mode only - the Actuator Console's own commands, unchanged:
//    PWM <hz>, LIM <ch> <min> <max>, SET <ch> <us>, OFF <ch>|ALL,
//    IDLE <ms>, ARM, DISARM, FIRE <ch> <ms>
//
//  Five telemetry lines, 10 Hz each, staggered one per 20 ms:
//    T {..}  actuators, in exactly the Actuator Console's format
//    M {..}  this board        N {..}  navigation state (Position MCU)
//    P {..}  Position status   G {..}  guidance / Navigation status
//  During HIL mode an extra 50 Hz line is emitted every 20 ms:
//    H {..}  high-rate HIL snapshot (state, nav, actuator commands + servo us)
// =====================================================================

// ---- parameter store: the W25Q128, not EEPROM -----------------------------
//
// The RP2350 has NO EEPROM. arduino-pico's EEPROM.h emulates one in the last
// sector of the PROGRAM flash, and every commit() erases and reprograms that
// sector with execute-in-place switched off - BOTH cores stop for tens of
// milliseconds. Core 1 runs the LoRa link and the flight log, so that is not
// acceptable here even on the ground.
//
// The table lives on the external W25Q128 instead, in sectors 0 and 1, two
// copies with a sequence number: a save always writes the OLDER copy, so a
// power cut in the middle of a save leaves the newer one intact, and a boot
// takes the newest copy that passes its CRC. The flight log starts at
// LOG_BASE, after them.
//
// The flash bus belongs to core 1 (the log). Core 0 never touches it after
// setup(): a save or load is a REQUEST that core 1 executes between two log
// pages, and the USB console reports the result when it is done.
// (struct ParamImage is defined near the top of the file, before the
//  auto-generated prototypes that mention it.)
static ParamImage        s_pimg;                    // staging, handed between cores
static volatile uint8_t  s_flash_req = 0;            // 1 save, 2 load (core 0 -> core 1)
static volatile uint8_t  s_flash_res = 0;            // 0 pending, 1 ok, 2 failed
static uint32_t          s_param_seq = 0;            // seq of the copy in use

static void flash_wait_idle(uint32_t timeout_ms)
{
    const uint32_t t0 = millis();
    while (flash_busy() && millis() - t0 < timeout_ms) tight_loop_contents();
}

/** How many ids an image of this layout holds (0 = unknown layout). */
static uint16_t pimg_ids(const ParamImage *im)
{
    if (im->magic != PARAM_MAGIC) return 0;
    if (im->layout == ARIADNA_PARAM_LAYOUT) return ARIADNA_PARAM_ID_MAX;
    // Layout 5 (fw 3.28): 232 ids, and its CRC covers all 232. The battery model
    // (224..227) did not exist yet and the image holds zeros there - see
    // pimg_trusted_ids(): those start at their defaults.
    if (im->layout == 5 || im->layout == 6) return ARIADNA_PARAM_ID_MAX;
    // Layout 4 (fw 3.25..3.27): 200 ids. The servo map and limits (ids
    // 200..223) start at their defaults, everything else is kept.
    if (im->layout == 4) return ARIADNA_PARAM_ID_MAX_V4;
    // Layout 3 (fw <= 3.24): 164 ids. Kept readable so that going to 200
    // ids does not throw away a calibrated vehicle's servo trims.
    if (im->layout == 3) return ARIADNA_PARAM_ID_MAX_V3;
    return 0;
}

/** How many of those ids hold values somebody actually set (the CRC covers more for layout 5). */
static uint16_t pimg_trusted_ids(const ParamImage *im)
{
    const uint16_t n = pimg_ids(im);
    if (im->layout == 5 && n > ARIADNA_PARAM_ID_MAX_V5) return ARIADNA_PARAM_ID_MAX_V5;
    if (im->layout == 6 && n > ARIADNA_PARAM_ID_MAX_V6) return ARIADNA_PARAM_ID_MAX_V6;
    return n;
}

static bool pimg_valid(const ParamImage *im)
{
    const uint16_t n = pimg_ids(im);
    return n && ariadna_params_crc_n(im->v, n) == im->crc;
}

/** Read both copies into `out`, newest valid wins. Flash-bus owner only. */
static bool flash_params_read(ParamImage *out)
{
    ParamImage a, b;
    flash_read(PARAM_SECTOR_A * FLASH_SECTOR, (uint8_t *)&a, sizeof(a));
    flash_read(PARAM_SECTOR_B * FLASH_SECTOR, (uint8_t *)&b, sizeof(b));
    const bool va = pimg_valid(&a), vb = pimg_valid(&b);
    if (!va && !vb) return false;
    *out = (va && (!vb || a.seq >= b.seq)) ? a : b;
    return true;
}

/** Write `im` over the OLDER copy. Flash-bus owner only; blocks ~50 ms. */
static bool flash_params_write(ParamImage *im)
{
    ParamImage a, b;
    flash_read(PARAM_SECTOR_A * FLASH_SECTOR, (uint8_t *)&a, sizeof(a));
    flash_read(PARAM_SECTOR_B * FLASH_SECTOR, (uint8_t *)&b, sizeof(b));
    const bool va = pimg_valid(&a), vb = pimg_valid(&b);
    uint32_t newest = 0, sector = PARAM_SECTOR_A;
    if (va) newest = a.seq;
    if (vb && b.seq > newest) newest = b.seq;
    // overwrite the copy that is NOT the newest valid one
    if (va && (!vb || a.seq >= b.seq)) sector = PARAM_SECTOR_B;
    im->seq = newest + 1;
    const uint32_t base = sector * FLASH_SECTOR;

    flash_wait_idle(500);
    flash_sector_erase(base);
    flash_wait_idle(500);
    const uint8_t *p = (const uint8_t *)im;
    for (uint32_t off = 0; off < sizeof(*im); off += FLASH_PAGE) {
        uint32_t n = sizeof(*im) - off;
        if (n > FLASH_PAGE) n = FLASH_PAGE;
        flash_page_program(base + off, p + off, n);
        flash_wait_idle(10);
    }
    ParamImage chk;                                   // read back: it really took
    flash_read(base, (uint8_t *)&chk, sizeof(chk));
    return memcmp(&chk, im, sizeof(chk)) == 0;
}

static void params_apply_image(const ParamImage *im)
{
    ariadna_params_default(&s_params);
    // Through ariadna_param_set(), so a stored value outside today's range
    // is clamped rather than trusted, and an id the image does not know
    // keeps its default.
    const uint16_t n = pimg_trusted_ids(im);
    for (uint8_t i = 0; i < ARIADNA_PARAM_COUNT; i++) {
        uint8_t id = ARIADNA_PARAM_INFO[i].id;
        if (id < n) (void)ariadna_param_set(&s_params, id, im->v[id]);
    }
    s_param_seq = im->seq;
}

/** setup() only, before core 1 owns the flash bus. */
static void params_load(void)
{
    ariadna_params_default(&s_params);
    s_params_from_flash = false;
    if (s_flash_ok && flash_params_read(&s_pimg)) {
        params_apply_image(&s_pimg);
        s_params_from_flash = true;
    }
    s_param_crc = ariadna_params_crc(&s_params);
}

/** Core 0: ask core 1 to save the current table. */
static bool params_save_request(void)
{
    if (!s_flash_ok || s_flash_req) return false;
    s_pimg.magic  = PARAM_MAGIC;
    s_pimg.layout = ARIADNA_PARAM_LAYOUT;
    memcpy(s_pimg.v, s_params.v, sizeof(s_pimg.v));
    s_pimg.crc    = ariadna_params_crc(&s_params);
    s_flash_res   = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_flash_req   = 1;
    return true;
}

static bool params_load_request(void)
{
    if (!s_flash_ok || s_flash_req) return false;
    s_flash_res = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_flash_req = 2;
    return true;
}

// ---- flight log readout and erase (LOG DUMP / LOG ERASE) ----------------
// Same pattern as the parameter store: core 0 asks, core 1 (which owns the
// flash) does it between log pages, core 0 prints what came back as fast as
// USB takes it. Only while the vehicle is NOT logging (not ARMED..LANDED).
static const uint32_t LOG_REC_SZ   = 96;                  // sizeof(LogRecord)
static const uint32_t DUMP_CHUNK   = 8 * LOG_REC_SZ;
static uint8_t           s_dump_buf[DUMP_CHUNK];
static volatile uint32_t s_dump_addr = 0;     // next flash address to read
static volatile uint32_t s_dump_end  = 0;     // stop here
static volatile uint32_t s_dump_len  = 0;     // bytes in s_dump_buf
static volatile uint8_t  s_dump_req  = 0;     // 1 = core 1: fill the buffer
static volatile uint8_t  s_erase_req = 0;     // 1 = core 1: erase the log
static volatile uint32_t s_erase_at  = 0;
// LOG FLIGHTS: core 1 scans the whole log for runs of flight-rate records
struct LogSeg { uint32_t start, count, t0, t1; float h_max; };
static const uint8_t     LOG_SEG_MAX = 32;
static LogSeg            s_seg[LOG_SEG_MAX];
static volatile uint8_t  s_seg_n    = 0;
static volatile uint8_t  s_scan_req = 0;      // 1 = core 1: scan, 2 = done
static volatile uint32_t s_scan_rec = 0;      // progress

static bool log_service_busy(void) { return s_dump_req || s_erase_req || s_scan_req; }

static void flash_log_scan(void)
{
    // A flight is a run of records with flags bit 0, broken by a clock that
    // jumps back (reboot) or a gap > 2 s. Read 8 records at a time; a full
    // 16 MB chip is ~5 s of SPI on a core that is idle on the ground.
    const uint32_t end = s_log_addr;
    uint8_t buf[8 * LOG_REC_SZ];
    uint8_t n = 0;
    bool in = false;
    uint32_t last_t = 0;
    for (uint32_t a = LOG_BASE; a + LOG_REC_SZ <= end; a += sizeof(buf)) {
        uint32_t len = end - a < sizeof(buf) ? end - a : sizeof(buf);
        while (flash_busy()) tight_loop_contents();
        flash_read(a, buf, len);
        for (uint32_t o = 0; o + LOG_REC_SZ <= len; o += LOG_REC_SZ) {
            const LogRecord *r = (const LogRecord *)(buf + o);
            const uint32_t idx = (a + o - LOG_BASE) / LOG_REC_SZ;
            const bool fl = (r->flags & 1) && r->t_ms != 0xFFFFFFFFu;
            const bool brk = in && (r->t_ms < last_t || r->t_ms - last_t > 2000);
            if (in && (!fl || brk)) in = false;
            if (fl && !in && n < LOG_SEG_MAX) {
                s_seg[n] = { idx, 0, r->t_ms, r->t_ms, -1e9f };
                n++;
                in = true;
            }
            if (fl && in) {
                LogSeg &g = s_seg[n - 1];
                g.count++;
                g.t1 = r->t_ms;
                if (r->h > g.h_max) g.h_max = r->h;
            }
            last_t = r->t_ms;
            s_scan_rec = idx;
        }
    }
    s_seg_n = n;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_scan_req = 2;
}

static void flash_log_service(void)
{
    if (s_scan_req == 1) flash_log_scan();
    if (s_dump_req == 1) {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        uint32_t n = s_dump_end - s_dump_addr;
        if (n > DUMP_CHUNK) n = DUMP_CHUNK;
        if (!flash_busy()) {
            flash_read(s_dump_addr, s_dump_buf, n);
            s_dump_len = n;
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_dump_req = 2;                        // core 0: print it
        }
    }
    if (s_erase_req == 1 && !flash_busy()) {
        // sector by sector, one per call: never blocks this core for long
        if (s_erase_at < s_log_addr && s_erase_at < FLASH_SIZE) {
            flash_sector_erase(s_erase_at);
            s_erase_at += FLASH_SECTOR;
        } else {
            s_log_addr      = LOG_BASE;
            s_log_erased_to = LOG_BASE;
            s_log_head = s_log_tail = 0;
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_erase_req = 2;
        }
    }
}

/** Core 1, between log pages. */
static void flash_param_service(void)
{
    flash_log_service();
    const uint8_t req = s_flash_req;
    if (!req) return;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    bool ok = (req == 1) ? flash_params_write(&s_pimg) : flash_params_read(&s_pimg);
    s_flash_res = ok ? 1 : 2;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_flash_req = 0;
}

/** Core 0, every tick: finish a request core 1 has completed. */
static void params_request_poll(void)
{
    static uint8_t pending = 0;               // what core 0 is waiting for
    if (s_flash_req) { if (!pending) pending = s_flash_req; return; }
    if (!pending) return;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    if (pending == 1) {
        if (s_flash_res == 1) {
            s_param_seq = s_pimg.seq;
            s_params_from_flash = true;
            usb_printf("OK PARAM SAVE %u (flash copy %lu)\n", s_param_crc, (unsigned long)s_param_seq);
        } else {
            usb_printf("ERR PARAM SAVE flash write failed\n");
        }
    } else {
        if (s_flash_res == 1) {
            params_apply_image(&s_pimg);
            s_param_crc = ariadna_params_crc(&s_params);
            s_params_from_flash = true;
            servo_map_update();
            usb_printf("OK PARAM LOAD flash %u\n", s_param_crc);
        } else {
            usb_printf("ERR PARAM LOAD no valid copy in flash\n");
        }
    }
    pending = 0;
}

static inline bool on_ground_state(void)
{
    return s_state == ARIADNA_STATE_BOOT || s_state == ARIADNA_STATE_IDLE ||
           s_state == ARIADNA_STATE_ARMED || s_state == ARIADNA_STATE_TEST ||
           s_state == ARIADNA_STATE_ERROR;
}

/** One parameter every 25 ms to both boards, round-robin, on the ground. */
static void params_stream(void)
{
    static uint32_t next_ms = 0;
    static uint8_t  idx = 0;
    if (!on_ground_state()) return;
    if (millis() < next_ms) return;
    next_ms = millis() + 25;

    ariadna_param_pkt_t p;
    memset(&p, 0, sizeof(p));
    p.id    = ARIADNA_PARAM_INFO[idx].id;
    p.value = s_params.v[p.id];
    p.count = ARIADNA_PARAM_COUNT;
    send_to(&s_tx_pos, POS_PORT, ARIADNA_PKT_PARAM, &p, sizeof(p));
    send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_PARAM, &p, sizeof(p));
    idx = (uint8_t)((idx + 1) % ARIADNA_PARAM_COUNT);
}

// ---- the USB transmit ring -----------------------------------------------
static const uint16_t USB_RING_SZ = 4096;            // power of two
static char              s_usb_ring[USB_RING_SZ];
static volatile uint16_t s_usb_head = 0, s_usb_tail = 0;
static volatile uint32_t s_usb_dropped = 0;
static mutex_t           s_usb_mutex;                // core 1 prints too, at boot
static bool              s_usb_mutex_ok = false;

static void usb_puts_n(const char *txt, uint16_t n)
{
    if (!s_usb_mutex_ok) { mutex_init(&s_usb_mutex); s_usb_mutex_ok = true; }
    if (!mutex_try_enter(&s_usb_mutex, nullptr)) { s_usb_dropped++; return; }
    uint16_t used = (uint16_t)((s_usb_head - s_usb_tail) & (USB_RING_SZ - 1));
    if ((uint32_t)used + n >= USB_RING_SZ - 1) {
        s_usb_dropped++;                              // all or nothing
    } else {
        uint16_t h = s_usb_head;
        for (uint16_t i = 0; i < n; i++) {
            s_usb_ring[h] = txt[i];
            h = (uint16_t)((h + 1) & (USB_RING_SZ - 1));
        }
        s_usb_head = h;
    }
    mutex_exit(&s_usb_mutex);
}

static void usb_printf(const char *fmt, ...)
{
    char buf[400];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    usb_puts_n(buf, (uint16_t)n);
}

/** Core 0: print the chunk core 1 read, then ask for the next. */
static void log_dump_service(void)
{
    static uint32_t rec = 0, off = 0;
    if (s_erase_req == 2) { s_erase_req = 0; usb_printf("OK LOG ERASE\n"); }
    static uint8_t seg_print = 255;
    if (s_scan_req == 2) { s_scan_req = 0; seg_print = 0; }
    if (seg_print != 255) {
        if (seg_print < s_seg_n) {
            const LogSeg &g = s_seg[seg_print];
            usb_printf("LOGF %u %lu %lu %lu %lu %.1f\n", seg_print, (unsigned long)g.start,
                       (unsigned long)g.count, (unsigned long)g.t0, (unsigned long)g.t1, (double)g.h_max);
            seg_print++;
        } else {
            usb_printf("OK LOG FLIGHTS %u\n", s_seg_n);
            seg_print = 255;
        }
        return;
    }
    if (s_dump_req == 3) {                                              // start
        rec = (s_dump_addr - LOG_BASE) / LOG_REC_SZ;
        off = 0;
        s_dump_req = 1;
        return;
    }
    if (s_dump_req != 2) return;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    // one record per call while the ring has room: a 96-byte record is a
    // 205-character line, and the telemetry lines must still get through
    while (off + LOG_REC_SZ <= s_dump_len &&
           (uint16_t)((s_usb_head - s_usb_tail) & (USB_RING_SZ - 1)) < USB_RING_SZ - 512) {
        char line[2 * LOG_REC_SZ + 24];
        int k = snprintf(line, sizeof(line), "LOGR %lu ", (unsigned long)rec);
        static const char HEXCH[] = "0123456789abcdef";
        for (uint32_t i = 0; i < LOG_REC_SZ; i++) {
            line[k++] = HEXCH[s_dump_buf[off + i] >> 4];
            line[k++] = HEXCH[s_dump_buf[off + i] & 15];
        }
        line[k++] = '\n';
        usb_puts_n(line, (uint16_t)k);
        off += LOG_REC_SZ;
        rec++;
        if (!Serial || Serial.availableForWrite() < 64) return;
    }
    if (off + LOG_REC_SZ <= s_dump_len) return;          // ring full: next tick
    if (off + LOG_REC_SZ > s_dump_len) {
        s_dump_addr = s_dump_addr + s_dump_len;
        off = 0;
        if (s_dump_addr + LOG_REC_SZ > s_dump_end) {
            usb_printf("OK LOG DUMP %lu\n", (unsigned long)rec);
            s_dump_req = 0;
        } else {
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_dump_req = 1;
        }
    }
}

static void console_line(char *line);
static uint8_t s_list_idx = 255;        // PARAM LIST in progress: next entry to print

/** Drain the ring as far as the host allows, and read what it sent. */
static void usb_service(void)
{
    static char    line[80];
    static uint8_t len = 0;

    if (!Serial) {
        // No host: forget the backlog, so the first thing a host sees when
        // it connects is the present, not the last four kilobytes.
        s_usb_tail = s_usb_head;
        s_gui_alive = false;
        s_raw_on = false;
        if (s_hil) hil_leave();
        len = 0;
        return;
    }

    int room = Serial.availableForWrite();
    uint16_t t = s_usb_tail;
    while (room > 0 && t != s_usb_head) {
        uint16_t run = (uint16_t)((s_usb_head - t) & (USB_RING_SZ - 1));
        if (run > USB_RING_SZ - t) run = (uint16_t)(USB_RING_SZ - t);
        if (run > (uint16_t)room)  run = (uint16_t)room;
        Serial.write((const uint8_t *)&s_usb_ring[t], run);
        t = (uint16_t)((t + run) & (USB_RING_SZ - 1));
        room -= run;
    }
    s_usb_tail = t;

    // PARAM LIST, one line per tick: seventy lines at once would not fit
    // the ring, and a list with holes in it is worse than a slow one.
    if (s_list_idx < ARIADNA_PARAM_COUNT &&
        (uint16_t)((s_usb_head - s_usb_tail) & (USB_RING_SZ - 1)) < USB_RING_SZ / 2) {
        const ariadna_param_info_t *q = &ARIADNA_PARAM_INFO[s_list_idx++];
        MASTER_PRINT("PRM %u %s %.9g %.6g %.6g %.6g %s %c\n", q->id, q->key,
                     s_params.v[q->id], q->def, q->lo, q->hi, q->unit, q->owner);
        if (s_list_idx >= ARIADNA_PARAM_COUNT) {
            usb_printf("OK PARAM LIST %u %u\n", ARIADNA_PARAM_COUNT, s_param_crc);
            s_list_idx = 255;
        }
    }
    log_dump_service();

    uint8_t budget = 64;                              // bytes per tick, bounded
    while (budget-- && Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            line[len] = 0;
            if (len) {
                s_gui_last = millis();
                if (!s_gui_alive) { s_gui_alive = true; usb_printf("# GUI connected\n"); }
                console_line(line);
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
    if (s_gui_alive && millis() - s_gui_last > 1500) {
        s_gui_alive = false;
        if (s_test) pyro_bridge_send(ACTCON_CMD_DISARM, 0, 0, 3);
        if (s_hil)  hil_leave();
        if (s_polarity) {
            s_polarity = false;
            send_master_state();
            servos_park();
            servos_write();
        }
        usb_printf("# GUI silent - bench pyros disarmed, HIL/polarity stopped, servos hold\n");
    }
}

// ---- TEST mode: the Actuator Console inside the flight firmware -----------
static float test_max_pulse_us(void)
{
    const float lim = 1e6f / s_pwm_hz - 200.0f;       // never closer to the period
    return lim < 3500.0f ? lim : 3500.0f;
}

static uint16_t test_clamp_us(uint8_t ch, long us)
{
    if (us < s_test_lim_lo[ch]) us = s_test_lim_lo[ch];
    if (us > s_test_lim_hi[ch]) us = s_test_lim_hi[ch];
    if (us > (long)test_max_pulse_us()) us = (long)test_max_pulse_us();
    return (uint16_t)us;
}

/** One channel: a pulse of s_test_us[ch], or FULL OFF when 0 or idled. */
static bool test_write_ch(uint8_t ch)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(PCA_LED0_ON_L + 4 * ch));
    if (s_test_us[ch] == 0 || !s_test_awake[ch]) {
        Wire.write(0); Wire.write(0);
        Wire.write(0); Wire.write(0x10);              // OFF_H bit 4: full off
    } else {
        float c = (float)s_test_us[ch] * (4096.0f * s_pwm_hz * 1e-6f) + 0.5f;
        if (c < 1.0f) c = 1.0f;
        if (c > 4095.0f) c = 4095.0f;
        uint16_t n = (uint16_t)c;
        Wire.write(0); Wire.write(0);
        Wire.write((uint8_t)(n & 0xFF));
        Wire.write((uint8_t)(n >> 8));
    }
    return Wire.endTransmission() == 0;
}

static void test_write_all(void)
{
    bool ok = true;
    for (uint8_t ch = 0; ch < PCA_N_CH; ch++) ok = test_write_ch(ch) && ok;
    s_servo_ok = ok;
}

static void pyro_bridge_send(uint8_t cmd, uint8_t ch, uint16_t pulse_ms, uint8_t repeats)
{
    actcon_pyro_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd      = cmd;
    c.ch       = ch;
    c.pulse_ms = pulse_ms;
    c.seq      = (cmd == ACTCON_CMD_HEARTBEAT) ? 0 : s_pyro_seq++;
    for (uint8_t i = 0; i < repeats; i++)
        send_to(&s_tx_nav, NAV_PORT, ACTCON_PKT_PYRO_CMD, &c, sizeof(c));
}

static void test_enter(void)
{
    if (s_test) return;
    if (s_state == ARIADNA_STATE_ARMED) broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
    s_test = true;
    s_flags |= ARIADNA_MST_TEST;
    s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
    // Like the Actuator Console: every channel starts with NO pulse, so
    // nothing moves until the GUI moves it.
    for (uint8_t i = 0; i < PCA_N_CH; i++) { s_test_us[i] = 0; s_test_awake[i] = false; }
    test_write_all();
    set_state(ARIADNA_STATE_TEST);
    broadcast_cmd(ARIADNA_CMD_TEST_MODE, 1, 0);
}

static void test_leave(void)
{
    if (!s_test) return;
    pyro_bridge_send(ACTCON_CMD_DISARM, 0, 0, 3);
    s_test = false;
    s_flags &= (uint8_t)~ARIADNA_MST_TEST;
    s_pwm_hz = PCA_PWM_HZ;                            // back to the flight refresh rate
    s_servo_ok = pca_begin();
    servos_park();
    servos_write();
    for (uint8_t ch = 0; ch < PCA_N_CH; ch++)
        if (!ch_is_flight(ch)) { s_test_us[ch] = 0; (void)test_write_ch(ch); }
    broadcast_cmd(ARIADNA_CMD_TEST_MODE, 0, 0);
    broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
    set_state(s_post_pass ? ARIADNA_STATE_IDLE : ARIADNA_STATE_ERROR);
}

/**
 * Enter Hardware-in-the-Loop flight simulation.
 *
 * All three MCUs execute the real flight pipeline (state machine, guidance,
 * sequencer, PCA9685 servos), fed by synthetic sensor packets injected into
 * the Position MCU over USB. By default physical pyro channels are inhibited
 * via ARIADNA_MST_PYRO_INHIBIT unless pyro_en is true (which requires the
 * hardware RBF pin removed).
 */
static void hil_enter(bool pyro_en)
{
    if (s_test) test_leave();
    s_hil      = true;
    s_hil_pyro = pyro_en;
    s_cover_open  = false;
    s_t_launch_ms = 0;
    s_flags &= (uint8_t)~(ARIADNA_MST_ARMED | ARIADNA_MST_TEST | ARIADNA_MST_LOGGING);
    if (s_hil_pyro) s_flags &= (uint8_t)~ARIADNA_MST_PYRO_INHIBIT;
    else            s_flags |= ARIADNA_MST_PYRO_INHIBIT;

    // Put both slave MCUs cleanly back on the ground BEFORE broadcasting
    // DISARM so Navigation_MCU sees on_ground() == true and rewinds guidance,
    // sequencer events, and s_fired_mask cleanly.
    set_state(ARIADNA_STATE_IDLE);
    send_master_state();
    broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
    broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);

    servos_park();
    servos_write();

    s_flags |= ARIADNA_MST_ARMED;
    set_state(ARIADNA_STATE_ARMED);
    broadcast_cmd(ARIADNA_CMD_ARM, 0, 0);
    send_master_state();
}

static void hil_leave(void)
{
    if (!s_hil) return;
    s_hil      = false;
    s_hil_pyro = false;
    s_cover_open = false;
    s_flags &= (uint8_t)~(ARIADNA_MST_ARMED | ARIADNA_MST_PYRO_INHIBIT);

    // Broadcast IDLE/ERROR before DISARM so Navigation_MCU rewinds cleanly.
    set_state(s_post_pass ? ARIADNA_STATE_IDLE : ARIADNA_STATE_ERROR);
    send_master_state();
    broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
    servos_park();
    servos_write();
}

/** Idle-off and the heartbeat to the Navigation MCU's bench pyro logic. */
static void test_service(void)
{
    static uint32_t next_hb = 0;
    const uint32_t now = millis();
    if (s_test_idle_ms) {
        for (uint8_t ch = 0; ch < PCA_N_CH; ch++) {
            if (s_test_awake[ch] && s_test_us[ch] && now - s_test_moved_ms[ch] >= s_test_idle_ms) {
                s_test_awake[ch] = false;
                (void)test_write_ch(ch);
            }
        }
    }
    // The heartbeat follows the GUI's: no GUI, no heartbeat, and the
    // Navigation MCU disarms itself half a second later.
    if (s_gui_alive && now >= next_hb) {
        next_hb = now + ACTCON_HB_PERIOD_MS;
        pyro_bridge_send(ACTCON_CMD_HEARTBEAT, 0, 0, 1);
    }
}

// ---- commands ----------------------------------------------------------------
static bool parse_ch(const char *tok, uint8_t *ch)
{
    if (!tok) return false;
    char *end;
    const long v = strtol(tok, &end, 10);
    if (end == tok || v < 0 || v >= PCA_N_CH) return false;
    *ch = (uint8_t)v;
    return true;
}

static inline bool pyro_rep_fresh(void) { return s_pyro_rep_ms && millis() - s_pyro_rep_ms < 300; }

static void console_param(char *a1, char *a2, char *a3)
{
    if (!a1) { usb_printf("ERR PARAM LIST|SET|SAVE|LOAD|DEFAULTS\n"); return; }
    for (char *q = a1; *q; q++) *q = (char)toupper((unsigned char)*q);

    if (!strcmp(a1, "LIST")) {
        s_list_idx = 0;                 // sent a few lines per tick, see usb_service()
        return;
    }
    // Everything below changes the table: on the ground and not armed.
    if (!(s_state == ARIADNA_STATE_IDLE || s_state == ARIADNA_STATE_TEST ||
          s_state == ARIADNA_STATE_BOOT || s_state == ARIADNA_STATE_ERROR)) {
        usb_printf("ERR PARAM only in IDLE or TEST (state %u)\n", s_state);
        return;
    }
    if (!strcmp(a1, "SET")) {
        if (!a2 || !a3) { usb_printf("ERR PARAM SET <id|key> <value>\n"); return; }
        const ariadna_param_info_t *info = nullptr;
        char *end;
        long id = strtol(a2, &end, 10);
        if (end != a2 && *end == 0) {
            info = ariadna_param_find((uint8_t)id);
        } else {
            for (uint8_t i = 0; i < ARIADNA_PARAM_COUNT; i++)
                if (!strcmp(ARIADNA_PARAM_INFO[i].key, a2)) info = &ARIADNA_PARAM_INFO[i];
        }
        if (!info) { usb_printf("ERR PARAM unknown '%s'\n", a2); return; }
        float v = strtof(a3, &end);
        if (end == a3 || !ariadna_param_set(&s_params, info->id, v)) {
            usb_printf("ERR PARAM bad value '%s'\n", a3);
            return;
        }
        s_param_crc = ariadna_params_crc(&s_params);
        servo_map_update();                           // srv.ch_* take effect at once
        if (!s_test) {                                // servo trims show at once
            servos_park();
            servos_write();
        }
        MASTER_PRINT("OK PARAM %u %s %.9g\n", info->id, info->key, s_params.v[info->id]);
        return;
    }
    if (!strcmp(a1, "SAVE")) {
        // Done by core 1, which owns the flash; reported when it is.
        if (!params_save_request()) usb_printf("ERR PARAM SAVE flash missing or busy\n");
        return;
    }
    if (!strcmp(a1, "LOAD")) {
        if (!params_load_request()) usb_printf("ERR PARAM LOAD flash missing or busy\n");
        return;
    }
    if (!strcmp(a1, "DEFAULTS")) {
        ariadna_params_default(&s_params);
        s_param_crc = ariadna_params_crc(&s_params);
        servo_map_update();
        usb_printf("OK PARAM DEFAULTS %u (not saved)\n", s_param_crc);
        return;
    }
    usb_printf("ERR PARAM LIST|SET|SAVE|LOAD|DEFAULTS\n");
}

static void console_line(char *line)
{
    char *save = nullptr;
    char *cmd = strtok_r(line, " \t", &save);
    if (!cmd) return;
    for (char *q = cmd; *q; q++) *q = (char)toupper((unsigned char)*q);
    char *a1 = strtok_r(nullptr, " \t", &save);
    char *a2 = strtok_r(nullptr, " \t", &save);
    char *a3 = strtok_r(nullptr, " \t", &save);
    uint8_t ch;

    if (!strcmp(cmd, "HB")) return;
    if (!strcmp(cmd, "HELLO")) {
        usb_printf("I ACTCON 1 %u\n", PCA_N_CH);
        usb_printf("I DAIDALOS %d FLIGHT guidance %d params %u\n", ARIADNA_FW_VERSION,
                   DG_VERSION, ARIADNA_PARAM_COUNT);
        return;
    }
    if (!strcmp(cmd, "STATUS")) return;               // the stream is always on
    if (!strcmp(cmd, "PARAM")) { console_param(a1, a2, a3); return; }

    if (!strcmp(cmd, "TEST")) {
        const bool on = a1 && a1[0] == '1';
        if (on) {
            if (s_hil) {
                usb_printf("ERR TEST leave HIL first (send HIL 0)\n");
                return;
            }
            if (s_state != ARIADNA_STATE_IDLE && s_state != ARIADNA_STATE_ARMED &&
                s_state != ARIADNA_STATE_BOOT && s_state != ARIADNA_STATE_ERROR &&
                s_state != ARIADNA_STATE_TEST) {
                usb_printf("ERR TEST not on the ground (state %u)\n", s_state);
                return;
            }
            test_enter();
            usb_printf("OK TEST 1\n");
        } else {
            test_leave();
            usb_printf("OK TEST 0\n");
        }
        return;
    }
    if (!strcmp(cmd, "HIL")) {
        if (a1) {
            for (char *q = a1; *q; q++) *q = (char)toupper((unsigned char)*q);
        }
        if (!a1 || !strcmp(a1, "STATUS")) {
            usb_printf("OK HIL STATUS %d %d %u\n", s_hil ? 1 : 0, s_hil_pyro ? 1 : 0, s_state);
            return;
        }
        if (a1[0] == '0') {
            hil_leave();
            usb_printf("OK HIL 0\n");
            return;
        }
        if (a1[0] == '1') {
            if (s_hil) { usb_printf("ERR HIL already active (send HIL 0 first)\n"); return; }
            if (!on_ground_state()) {
                usb_printf("ERR HIL not on the ground (state %u)\n", s_state);
                return;
            }
            if (srvchk_active()) { usb_printf("ERR HIL servo check running\n"); return; }
            const bool pyro_en = a2 && a2[0] == '1';
            if (pyro_en && !s_rbf_out) {
                usb_printf("ERR HIL pyro requires RBF pin removed\n");
                return;
            }
            if (!(s_have_nav && link_alive(s_nav_us))) {
                usb_printf("ERR HIL Position MCU link down\n");
                return;
            }
            if (!(s_have_act && link_alive(s_act_us))) {
                usb_printf("ERR HIL Navigation MCU link down\n");
                return;
            }
            if (!(s_nav.flags & ARIADNA_NAV_VALID)) {
                usb_printf("ERR HIL navigation not valid (stream HIL frames first)\n");
                return;
            }
            if (!params_synced()) {
                usb_printf("ERR HIL parameters not synced\n");
                return;
            }
            hil_enter(pyro_en);
            usb_printf("OK HIL 1 %d\n", s_hil_pyro ? 1 : 0);
            return;
        }
        usb_printf("ERR HIL 1 [0|1] | 0 | STATUS\n");
        return;
    }
    if (!strcmp(cmd, "POLARITY")) {
        if (a1) {
            for (char *q = a1; *q; q++) *q = (char)toupper((unsigned char)*q);
        }
        if (!a1 || !strcmp(a1, "STATUS")) {
            usb_printf("OK POLARITY STATUS %d %.0f %.0f\n",
                       s_polarity ? 1 : 0, (double)s_pol_thrust_n, (double)s_pol_speed_ms);
            return;
        }
        if (!strcmp(a1, "THRUST")) {
            if (!a2) {
                usb_printf("ERR POLARITY THRUST <0|10..250>\n");
                return;
            }
            float v = (float)atof(a2);
            if (v <= 0.0f) {
                s_pol_thrust_n = 0.0f;
            } else {
                if (v < 10.0f) v = 10.0f;
                if (v > 250.0f) v = 250.0f;
                s_pol_thrust_n = v;
            }
            if (s_polarity) send_master_state();
            usb_printf("OK POLARITY THRUST %.0f\n", (double)s_pol_thrust_n);
            return;
        }
        if (!strcmp(a1, "SPEED")) {
            if (!a2) {
                usb_printf("ERR POLARITY SPEED <0|5..70>\n");
                return;
            }
            float v = (float)atof(a2);
            if (v <= 0.0f) {
                s_pol_speed_ms = 0.0f;
            } else {
                if (v < 5.0f) v = 5.0f;
                if (v > 70.0f) v = 70.0f;
                s_pol_speed_ms = v;
            }
            if (s_polarity) send_master_state();
            usb_printf("OK POLARITY SPEED %.0f\n", (double)s_pol_speed_ms);
            return;
        }
        if (a1[0] == '0') {
            s_polarity = false;
            send_master_state();
            if (!s_test && !srvchk_active()) {
                servos_park();
                servos_write();
            }
            usb_printf("OK POLARITY 0\n");
            return;
        }
        if (a1[0] == '1') {
            if (s_test) { usb_printf("ERR POLARITY leave TEST first\n"); return; }
            if (s_hil)  { usb_printf("ERR POLARITY leave HIL first\n"); return; }
            if (s_state != ARIADNA_STATE_IDLE) {
                usb_printf("ERR POLARITY only in IDLE (state %u)\n", s_state);
                return;
            }
            if (srvchk_active()) { usb_printf("ERR POLARITY servo check running\n"); return; }
            if (!(s_have_nav && link_alive(s_nav_us))) {
                usb_printf("ERR POLARITY Position MCU link down\n");
                return;
            }
            if (!(s_nav.flags & ARIADNA_NAV_IMU_OK)) {
                usb_printf("ERR POLARITY IMU not ready\n");
                return;
            }
            if (!(s_have_act && link_alive(s_act_us))) {
                usb_printf("ERR POLARITY Navigation MCU link down\n");
                return;
            }
            if (a2) {
                float v = (float)atof(a2);
                if (v <= 0.0f) s_pol_thrust_n = 0.0f;
                else           s_pol_thrust_n = (v < 10.0f) ? 10.0f : (v > 250.0f ? 250.0f : v);
            }
            if (a3) {
                float v = (float)atof(a3);
                if (v <= 0.0f) s_pol_speed_ms = 0.0f;
                else           s_pol_speed_ms = (v < 5.0f) ? 5.0f : (v > 70.0f ? 70.0f : v);
            }
            s_polarity = true;
            send_master_state();
            usb_printf("OK POLARITY 1\n");
            return;
        }
        usb_printf("ERR POLARITY 1 [N] [m/s] | 0 | THRUST <0|10..250> | SPEED <0|5..70> | STATUS\n");
        return;
    }
    if (!strcmp(cmd, "SERVOCHECK")) {
        if (s_state != ARIADNA_STATE_IDLE || s_polarity) {
            usb_printf("ERR SERVOCHECK only in IDLE (leave TEST/POLARITY first)\n");
            return;
        }
        srvchk_start();
        usb_printf("OK SERVOCHECK\n");
        return;
    }
    if (!strcmp(cmd, "RAW")) {
        // Raw TriSense data as "R" lines (Position MCU, ~25 Hz). Ground only.
        const bool on = a1[0] == '1';
        if (on && !on_ground_state()) { usb_printf("ERR RAW not on the ground\n"); return; }
        s_raw_on = on;
        s_raw_new = false;
        send_master_state();                 // tell the Position MCU now, not in 100 ms
        usb_printf("OK RAW %d\n", on ? 1 : 0);
        return;
    }
    if (!strcmp(cmd, "PADREF")) {
        if (!on_ground_state()) { usb_printf("ERR PADREF not on the ground\n"); return; }
        broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);
        usb_printf("OK PADREF\n");
        return;
    }
    if (!strcmp(cmd, "LOG")) {
        // on the ground the 10 Hz log pauses itself while the log is read
        const bool logging = s_state >= ARIADNA_STATE_ARMED && s_state <= ARIADNA_STATE_LANDED;
        const uint32_t used = s_log_addr > LOG_BASE ? s_log_addr - LOG_BASE : 0;
        if (!s_flash_ok) { usb_printf("ERR LOG no flash\n"); return; }
        if (!a1 || !strcmp(a1, "INFO")) {
            usb_printf("OK LOG INFO %lu %lu %u\n", (unsigned long)used,
                       (unsigned long)(used / LOG_REC_SZ), (unsigned)LOG_REC_SZ);
            return;
        }
        if (logging) { usb_printf("ERR LOG busy - the vehicle is logging\n"); return; }
        if (!strcmp(a1, "DUMP")) {
            if (s_dump_req || s_erase_req) { usb_printf("ERR LOG busy\n"); return; }
            uint32_t from = a2 ? (uint32_t)strtoul(a2, nullptr, 10) : 0;
            uint32_t cnt  = a3 ? (uint32_t)strtoul(a3, nullptr, 10) : 0xFFFFFFFFu;
            s_dump_end  = LOG_BASE + (used / LOG_REC_SZ) * LOG_REC_SZ;
            s_dump_addr = LOG_BASE + from * LOG_REC_SZ;
            if (cnt != 0xFFFFFFFFu && s_dump_addr + cnt * LOG_REC_SZ < s_dump_end)
                s_dump_end = s_dump_addr + cnt * LOG_REC_SZ;
            if (s_dump_addr >= s_dump_end) { usb_printf("OK LOG DUMP 0\n"); return; }
            usb_printf("OK LOG BEGIN %lu %u\n", (unsigned long)((s_dump_end - s_dump_addr) / LOG_REC_SZ),
                       (unsigned)LOG_REC_SZ);
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_dump_req = 3;
            return;
        }
        if (!strcmp(a1, "FLIGHTS")) {
            if (log_service_busy()) { usb_printf("ERR LOG busy\n"); return; }
            usb_printf("OK LOG SCAN %lu\n", (unsigned long)(used / LOG_REC_SZ));
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_scan_req = 1;
            return;
        }
        if (!strcmp(a1, "STOP")) { s_dump_req = 0; usb_printf("OK LOG STOP\n"); return; }
        if (!strcmp(a1, "ERASE")) {
            if (!(s_state == ARIADNA_STATE_IDLE || s_state == ARIADNA_STATE_TEST)) {
                usb_printf("ERR LOG ERASE only in IDLE or TEST\n"); return;
            }
            if (s_dump_req || s_erase_req) { usb_printf("ERR LOG busy\n"); return; }
            s_erase_at = LOG_BASE;            // the parameter sectors 0-1 are kept
            __atomic_thread_fence(__ATOMIC_RELEASE);
            s_erase_req = 1;
            usb_printf("OK LOG ERASING %lu KB\n", (unsigned long)(used / 1024));
            return;
        }
        usb_printf("ERR LOG INFO|FLIGHTS|DUMP [from [count]]|STOP|ERASE\n");
        return;
    }
    if (!strcmp(cmd, "CAL")) {
        if (!on_ground_state() || s_state == ARIADNA_STATE_ARMED) { usb_printf("ERR CAL only in IDLE or TEST\n"); return; }
        broadcast_cmd(ARIADNA_CMD_CAL_GYRO, 0, 0);
        usb_printf("OK CAL GYRO - keep the vehicle still for 2 s\n");
        return;
    }
    if (!strcmp(cmd, "FLIGHTARM")) {
        if (a1 && a1[0] == '1') {
            s_arm_request = true;
            usb_printf("OK FLIGHTARM requested - arms when RBF is out and navigation is ready\n");
        } else {
            s_arm_request = false;
            if (s_hil) {
                hil_leave();
            } else if (s_state == ARIADNA_STATE_ARMED) {
                s_flags &= (uint8_t)~ARIADNA_MST_ARMED;
                broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
                set_state(ARIADNA_STATE_IDLE);
            }
            usb_printf("OK FLIGHTARM 0\n");
        }
        return;
    }

    // ---- the Actuator Console's commands: TEST mode only -----------------
    const bool act = !strcmp(cmd, "SET") || !strcmp(cmd, "OFF") || !strcmp(cmd, "IDLE") ||
                     !strcmp(cmd, "LIM") || !strcmp(cmd, "PWM") || !strcmp(cmd, "ARM") ||
                     !strcmp(cmd, "FIRE");
    if (act && !s_test) {
        // SET is streamed by the GUI: answer it once a second, not 50 times.
        static uint32_t told = 0;
        if (millis() - told > 1000) { told = millis(); usb_printf("ERR %s needs TEST mode (send TEST 1)\n", cmd); }
        return;
    }
    if (!strcmp(cmd, "SET")) {
        if (!parse_ch(a1, &ch) || !a2) { usb_printf("ERR SET <ch> <us>\n"); return; }
        s_test_us[ch]       = test_clamp_us(ch, strtol(a2, nullptr, 10));
        s_test_awake[ch]    = true;
        s_test_moved_ms[ch] = millis();
        s_servo_ok = test_write_ch(ch);
        return;
    }
    if (!strcmp(cmd, "OFF")) {
        if (a1 && toupper((unsigned char)a1[0]) == 'A') {
            for (uint8_t i = 0; i < PCA_N_CH; i++) s_test_us[i] = 0;
            test_write_all();
            usb_printf("OK OFF ALL\n");
            return;
        }
        if (!parse_ch(a1, &ch)) { usb_printf("ERR OFF <ch>|ALL\n"); return; }
        s_test_us[ch] = 0;
        s_servo_ok = test_write_ch(ch);
        usb_printf("OK OFF %u\n", ch);
        return;
    }
    if (!strcmp(cmd, "IDLE")) {
        const long ms = a1 ? strtol(a1, nullptr, 10) : -1;
        if (ms < 0 || ms > 60000) { usb_printf("ERR IDLE 0..60000 ms\n"); return; }
        s_test_idle_ms = (uint32_t)ms;
        usb_printf("OK IDLE %ld\n", ms);
        return;
    }
    if (!strcmp(cmd, "LIM")) {
        if (!parse_ch(a1, &ch) || !a2 || !a3) { usb_printf("ERR LIM <ch> <min> <max>\n"); return; }
        long lo = strtol(a2, nullptr, 10), hi = strtol(a3, nullptr, 10);
        if (lo < 500)  lo = 500;
        if (hi > 3500) hi = 3500;
        if (lo >= hi) { usb_printf("ERR LIM %u min >= max\n", ch); return; }
        s_test_lim_lo[ch] = (uint16_t)lo;
        s_test_lim_hi[ch] = (uint16_t)hi;
        if (s_test_us[ch]) {
            s_test_us[ch] = test_clamp_us(ch, s_test_us[ch]);
            s_servo_ok = test_write_ch(ch);
        }
        usb_printf("OK LIM %u %ld %ld\n", ch, lo, hi);
        return;
    }
    if (!strcmp(cmd, "PWM")) {
        const long hz = a1 ? strtol(a1, nullptr, 10) : 0;
        if (hz < 40 || hz > 400) { usb_printf("ERR PWM 40..400\n"); return; }
        s_pwm_hz = (float)hz;
        for (uint8_t i = 0; i < PCA_N_CH; i++)
            if (s_test_us[i]) s_test_us[i] = test_clamp_us(i, s_test_us[i]);
        s_servo_ok = pca_begin();
        test_write_all();
        usb_printf("OK PWM %ld\n", hz);
        return;
    }
    if (!strcmp(cmd, "DISARM")) {                     // always allowed
        pyro_bridge_send(ACTCON_CMD_DISARM, 0, 0, 3);
        usb_printf("OK DISARM sent\n");
        return;
    }
    if (!strcmp(cmd, "ARM") || !strcmp(cmd, "FIRE")) {
        if (!s_rbf_out)        { usb_printf("ERR %s RBF pin is in\n", cmd); return; }
        if (!pyro_rep_fresh()) { usb_printf("ERR %s Navigation MCU not answering\n", cmd); return; }
        if (!strcmp(cmd, "ARM")) {
            pyro_bridge_send(ACTCON_CMD_ARM, 0, 0, 3);
            usb_printf("OK ARM sent\n");
            return;
        }
        const long pch = a1 ? strtol(a1, nullptr, 10) : 0;
        const long ms  = a2 ? strtol(a2, nullptr, 10) : (long)ACTCON_PULSE_DEFAULT_MS;
        if (pch < 1 || pch > ACTCON_N_PYRO) { usb_printf("ERR FIRE channel 1..3\n"); return; }
        if (!s_pyro_rep.armed) { usb_printf("ERR FIRE not armed\n"); return; }
        const uint16_t pulse = actcon_clamp_pulse((uint16_t)(ms < 0 ? 0 : ms > 65535 ? 65535 : ms));
        pyro_bridge_send(ACTCON_CMD_FIRE, (uint8_t)pch, pulse, 3);
        usb_printf("OK FIRE %ld %u ms\n", pch, pulse);
        return;
    }
    usb_printf("ERR unknown '%s'\n", cmd);
}

// ---- telemetry lines ---------------------------------------------------------
static inline uint16_t counts_to_us(uint16_t c)
{
    return (uint16_t)((float)c / (4096.0f * s_pwm_hz * 1e-6f) + 0.5f);
}

static void diag_emit(void)
{
    static uint32_t next_ms = 0;
    static uint8_t  which = 0;
    if (!Serial) return;
    if (millis() < next_ms) return;
    next_ms = millis() + 20;

    char b[512];            // the M line alone is ~330 characters (fw 3.29: with the battery estimate)
    int n = 0;
    _Pragma("GCC diagnostic push")
    _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"")
    if (s_sc_report) {
        // SRVCHK: the servo connection check. Sent on every change.
        s_sc_report = false;
        n = snprintf(b, sizeof(b),
            "SRVCHK {\"run\":%d,\"done\":%d,\"ch\":%u,\"res\":[%u,%u,%u,%u,%u,%u,%u,%u],"
            "\"ma\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],\"thr\":%.0f}\n",
            srvchk_active() ? 1 : 0, s_sc_ran ? 1 : 0, s_sc_ch,
            s_sc_res[0], s_sc_res[1], s_sc_res[2], s_sc_res[3], s_sc_res[4], s_sc_res[5],
            s_sc_res[6], s_sc_res[7], s_sc_ma[0], s_sc_ma[1], s_sc_ma[2], s_sc_ma[3],
            s_sc_ma[4], s_sc_ma[5], s_sc_ma[6], s_sc_ma[7], PRM(MST_SRV_CHECK_MA));
        if (n > 0 && n < (int)sizeof(b)) usb_puts_n(b, (uint16_t)n);
        n = 0;
    }
    if (s_raw_new && s_raw_on) {
        // R: raw TriSense data, board axes. Extra to the five rotating lines.
        s_raw_new = false;
        const ariadna_raw_imu_t &r = s_raw;
        n = snprintf(b, sizeof(b),
            "R {\"t\":%lu,\"acc\":[%.4f,%.4f,%.4f],\"accr\":[%.5f,%.5f,%.5f],\"gyr\":[%.3f,%.3f,%.3f],"
            "\"mag\":[%.2f,%.2f,%.2f],\"magc\":[%.2f,%.2f,%.2f],\"p\":%.1f,\"temp\":%.2f,"
            "\"src\":%u,\"mf\":%u,\"hz\":%u,\"goff\":[%.3f,%.3f,%.3f]}\n",
            (unsigned long)r.t_ms, r.acc_g[0], r.acc_g[1], r.acc_g[2],
            r.acc_raw_g[0], r.acc_raw_g[1], r.acc_raw_g[2],
            r.gyr_dps[0], r.gyr_dps[1], r.gyr_dps[2], r.mag_ut[0], r.mag_ut[1], r.mag_ut[2],
            r.mag_cal_ut[0], r.mag_cal_ut[1], r.mag_cal_ut[2], r.pressure_pa, r.temp_c,
            r.src, r.mag_fresh, r.imu_hz, r.gyr_off_dps[0], r.gyr_off_dps[1], r.gyr_off_dps[2]);
        if (n > 0 && n < (int)sizeof(b)) usb_puts_n(b, (uint16_t)n);
        n = 0;
    }
    if (s_uwb_diag_new) {
        s_uwb_diag_new = false;
        const ariadna_uwb_diag_t &u = s_uwb_diag;
        n = snprintf(b, sizeof(b),
            "U {\"t\":%lu,\"ok\":%u,\"fl\":%u,\"err\":%u,\"rej\":%u,\"na\":%u,\"a\":["
            "[%u,%u,%u,%d,%u],[%u,%u,%u,%d,%u],[%u,%u,%u,%d,%u],[%u,%u,%u,%d,%u]]}\n",
            (unsigned long)u.t_ms, (unsigned)u.tag_present, (unsigned)u.tag_flags,
            (unsigned)u.link_errors, (unsigned)u.rejected, (unsigned)u.n_anchors,
            (unsigned)u.anchor[0].addr, (unsigned)u.anchor[0].range_cm, (unsigned)u.anchor[0].age_ms,
            (int)u.anchor[0].rxp_dbm, (unsigned)u.anchor[0].flags,
            (unsigned)u.anchor[1].addr, (unsigned)u.anchor[1].range_cm, (unsigned)u.anchor[1].age_ms,
            (int)u.anchor[1].rxp_dbm, (unsigned)u.anchor[1].flags,
            (unsigned)u.anchor[2].addr, (unsigned)u.anchor[2].range_cm, (unsigned)u.anchor[2].age_ms,
            (int)u.anchor[2].rxp_dbm, (unsigned)u.anchor[2].flags,
            (unsigned)u.anchor[3].addr, (unsigned)u.anchor[3].range_cm, (unsigned)u.anchor[3].age_ms,
            (int)u.anchor[3].rxp_dbm, (unsigned)u.anchor[3].flags);
        if (n > 0 && n < (int)sizeof(b)) usb_puts_n(b, (uint16_t)n);
        n = 0;
    }
    if (s_hil) {
        // H: 50 Hz HIL telemetry snapshot (every 20 ms tick) with nav state,
        // guidance commands, pyro events/state, and exact PCA9685 servo pulses
        // for all 8 flight servos [TVC_X, TVC_Y, CLAMP, COVER, FIN_0..3].
        n = snprintf(b, sizeof(b),
            "H {\"t\":%lu,\"st\":%u,\"ph\":%u,\"fl\":%u,\"ev\":%u,\"pa\":%u,\"pf\":%u,\"cov\":%d,"
            "\"p\":[%.2f,%.2f,%.2f],\"v\":[%.2f,%.2f,%.2f],\"q\":[%.4f,%.4f,%.4f,%.4f],"
            "\"tvc\":[%.2f,%.2f],\"cl\":%.2f,\"fin\":[%.1f,%.1f,%.1f,%.1f],\"afl\":%u,"
            "\"us\":[%u,%u,%u,%u,%u,%u,%u,%u],"
            "\"hign\":%.1f,\"thr\":%.1f,\"k\":%.2f,\"tign\":%.2f,\"ttd\":%.2f,"
            "\"tc\":%.3f,\"te\":%.3f}\n",
            (unsigned long)millis(), s_state,
            s_have_nav_status ? s_nav_status.phase : 0u,
            s_flags,
            s_have_nav_status ? s_nav_status.events : s_act.events,
            s_have_nav_status ? s_nav_status.pyro_active : 0u,
            s_have_nav_status ? s_nav_status.pyro_fired  : 0u,
            s_cover_open ? 1 : 0,
            s_nav.px, s_nav.py, s_nav.pz,
            s_nav.vx, s_nav.vy, s_nav.vz,
            s_nav.qw, s_nav.qx, s_nav.qy, s_nav.qz,
            s_act.tvc_x_deg, s_act.tvc_y_deg, s_act.clamp,
            s_act.fin_deg[0], s_act.fin_deg[1], s_act.fin_deg[2], s_act.fin_deg[3],
            s_act.flags,
            counts_to_us(srv_get(SV_TVC_X)), counts_to_us(srv_get(SV_TVC_Y)),
            counts_to_us(srv_get(SV_CLAMP)), counts_to_us(srv_get(SV_COVER)),
            counts_to_us(srv_get(SV_FIN_0)),
            counts_to_us(srv_get((uint8_t)(SV_FIN_0 + 1))),
            counts_to_us(srv_get((uint8_t)(SV_FIN_0 + 2))),
            counts_to_us(srv_get((uint8_t)(SV_FIN_0 + 3))),
            s_have_nav_status ? s_nav_status.h_ignite   : 0.0f,
            s_have_nav_status ? s_nav_status.thrust_est : 0.0f,
            s_have_nav_status ? s_nav_status.k_plan     : 0.0f,
            s_have_nav_status ? s_nav_status.t_to_ign   : -1.0f,
            s_have_nav_status ? s_nav_status.t_to_td    : -1.0f,
            s_have_nav_status ? s_nav_status.tilt_cmd   : 0.0f,
            s_have_nav_status ? s_nav_status.tilt_err   : 0.0f);
        if (n > 0 && n < (int)sizeof(b)) usb_puts_n(b, (uint16_t)n);
        n = 0;
    }
    switch (which) {
    case 0: {       // T: the Actuator Console's line, key for key
        uint16_t on = 0, us[PCA_N_CH];
        for (uint8_t ch = 0; ch < PCA_N_CH; ch++) {
            if (s_test) {
                us[ch] = s_test_us[ch];
                if (s_test_us[ch] && s_test_awake[ch]) on = (uint16_t)(on | (1u << ch));
            } else {
                us[ch] = ch_is_flight(ch) ? counts_to_us(s_pca[ch]) : 0;
                if (ch_is_flight(ch)) on = (uint16_t)(on | (1u << ch));
            }
        }
        const bool rep = s_test && pyro_rep_fresh();
        n = snprintf(b, sizeof(b),
            "T {\"t\":%lu,\"pca\":%d,\"hz\":%.0f,\"rbf\":%d,\"vbat\":%.2f,\"i\":%.2f,\"imax\":%.2f,"
            "\"idle\":%lu,\"maxus\":%.0f,\"on\":%u,\"us\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
            "\"nav\":%d,\"arm\":%d,\"arm_left\":%.1f,\"cont\":[%u,%u,%u],\"thr\":%u,\"cmask\":%u,"
            "\"fired\":%u,\"firing\":%u,\"rej\":%u,\"test\":%d}\n",
            (unsigned long)millis(), s_servo_ok ? 1 : 0, s_pwm_hz, s_rbf_out ? 1 : 0, s_vbat,
            s_amps_now, s_amps_max, (unsigned long)s_test_idle_ms, test_max_pulse_us(), on,
            us[0], us[1], us[2], us[3], us[4], us[5], us[6], us[7], us[8], us[9], us[10],
            us[11], us[12], us[13], us[14], us[15],
            rep ? 1 : 0, rep ? s_pyro_rep.armed : 0, rep ? s_pyro_rep.arm_left_ds / 10.0f : 0.0f,
            s_pyro_rep.raw[0], s_pyro_rep.raw[1], s_pyro_rep.raw[2], s_pyro_rep.threshold,
            rep ? s_pyro_rep.cont_mask : (s_have_nav_status ? s_nav_status.pyro_cont : 0),
            rep ? s_pyro_rep.fired_mask : (s_have_nav_status ? s_nav_status.pyro_fired : 0),
            rep ? s_pyro_rep.firing_mask : (s_have_nav_status ? s_nav_status.pyro_active : 0),
            rep ? s_pyro_rep.last_reject : 0, s_test ? 1 : 0);
        break;
    }
    case 1:         // M: this board
        n = snprintf(b, sizeof(b),
            "M {\"t\":%lu,\"fw\":%d,\"gv\":%d,\"st\":%u,\"fl\":%u,\"tl\":%lu,\"vbat\":%.2f,\"i\":%.2f,"
            "\"rbf\":%d,\"bw\":%d,\"post\":%d,\"servo\":%d,\"flash\":%d,\"lora\":%d,\"log\":%lu,"
            "\"logdrop\":%lu,\"tlm\":[%lu,%lu],\"pos\":[%lu,%lu,%d],\"nav\":[%lu,%lu,%d],"
            "\"pcrc\":%u,\"psync\":%d,\"peep\":%d,\"cover\":%d,\"hl\":%u,\"usbdrop\":%lu,"
            "\"bat\":[%u,%.1f,%.1f,%.1f,%u,%.1f,%.0f,%.2f,%.3f,%.2f,%u],\"su\":[%u,%u],\"hil\":%d,\"pol\":%d}\n",
            (unsigned long)millis(), ARIADNA_FW_VERSION, DG_VERSION, s_state, s_flags,
            (unsigned long)s_t_launch_ms, s_vbat, s_current, s_rbf_out ? 1 : 0,
            digitalRead(PIN_BREAKWIRE) == HIGH ? 1 : 0, s_post_pass ? 1 : 0, s_servo_ok ? 1 : 0,
            s_flash_ok ? 1 : 0, s_lora_ok ? 1 : 0, (unsigned long)s_log_addr,
            (unsigned long)s_log_dropped, (unsigned long)s_tlm_sent, (unsigned long)s_tlm_skipped,
            (unsigned long)s_rx_pos.rx_ok, (unsigned long)(s_rx_pos.rx_crc + s_rx_pos.rx_len),
            (s_have_nav && link_alive(s_nav_us)) ? 1 : 0,
            (unsigned long)s_rx_nav.rx_ok, (unsigned long)(s_rx_nav.rx_crc + s_rx_nav.rx_len),
            (s_have_act && link_alive(s_act_us)) ? 1 : 0,
            s_param_crc, params_synced() ? 1 : 0, s_params_from_flash ? 1 : 0,
            s_cover_open ? 1 : 0, health_bits(), (unsigned long)s_usb_dropped,
            (unsigned)s_batt.status, s_batt.soc_pct, s_batt.minutes, s_batt.v_oc,
            (unsigned)s_batt.win_s, s_batt.t_avg, s_batt.cap_eff_mah, s_batt.i_avg,
            s_batt.learn, s_batt.last_k, (unsigned)s_batt.learn_events,
            (unsigned)s_su_res, (unsigned)s_su_mask, s_hil ? (s_hil_pyro ? 2 : 1) : 0,
            s_polarity ? 1 : 0);
        break;
    case 2:         // N: the navigation state
        n = snprintf(b, sizeof(b),
            "N {\"p\":[%.2f,%.2f,%.2f],\"v\":[%.2f,%.2f,%.2f],\"a\":[%.2f,%.2f,%.2f],"
            "\"q\":[%.4f,%.4f,%.4f,%.4f],\"u\":[%.3f,%.3f,%.3f],\"w\":[%.3f,%.3f,%.3f],"
            "\"g\":%.2f,\"fl\":%u,\"ql\":%u,\"ok\":%d}\n",
            s_nav.px, s_nav.py, s_nav.pz, s_nav.vx, s_nav.vy, s_nav.vz,
            s_nav.ax, s_nav.ay, s_nav.az, s_nav.qw, s_nav.qx, s_nav.qy, s_nav.qz,
            s_nav.ux, s_nav.uy, s_nav.uz, s_nav.wx, s_nav.wy, s_nav.wz,
            s_nav.accel_mag, s_nav.flags, s_nav.quality,
            (s_have_nav && link_alive(s_nav_us)) ? 1 : 0);
        break;
    case 3:         // P: Position MCU status
        n = snprintf(b, sizeof(b),
            "P {\"ok\":%d,\"fw\":%u,\"pcrc\":%u,\"pseen\":%u,\"rpy\":[%.1f,%.1f,%.1f],\"lat\":%ld,"
            "\"lon\":%ld,\"galt\":%.1f,\"sats\":%u,\"fix\":%u,\"hdop\":%u,\"temp\":%.1f,"
            "\"sig\":[%.2f,%.2f],\"uwb\":[%u,%u,%u,%u],\"uwbs\":[%.2f,%.2f],\"trim\":%.2f,"
            "\"lidar\":[%.2f,%u],\"ins\":%u,\"att\":%u,\"hz\":%.0f,\"sens\":%u,\"ovf\":%u,"
            "\"cal\":%u,\"rx\":[%lu,%lu],\"pp\":%.0f}\n",
            pos_status_fresh() ? 1 : 0, s_pos_status.fw_version, s_pos_status.param_crc,
            s_pos_status.params_seen, s_pos_status.roll, s_pos_status.pitch, s_pos_status.yaw,
            (long)s_pos_status.lat, (long)s_pos_status.lon, s_pos_status.gps_alt,
            s_pos_status.sats, s_pos_status.fix, s_pos_status.hdop_x10, s_pos_status.temp_c,
            s_pos_status.sigma_pos_h, s_pos_status.sigma_vel_h, s_pos_status.uwb_anchors,
            s_pos_status.uwb_mask, s_pos_status.uwb_mode, s_pos_status.uwb_rejects,
            s_pos_status.uwb_sigma_h, s_pos_status.uwb_resid, s_pos_status.baro_trim,
            s_pos_status.lidar_m, s_pos_status.lidar_strength, s_pos_status.ins_mode,
            s_pos_status.att_src, s_pos_status.fusion_hz, s_pos_status.sensor_ok,
            s_pos_status.imu_overflow, s_pos_status.cal_state,
            (unsigned long)s_pos_status.rx_ok, (unsigned long)s_pos_status.rx_bad,
            s_pos_status.pad_pressure);
        break;
    default:        // G: guidance / Navigation MCU status and what it commands
        n = snprintf(b, sizeof(b),
            "G {\"ok\":%d,\"fw\":%u,\"pcrc\":%u,\"pseen\":%u,\"ph\":%u,\"k\":%.2f,\"thr\":%.1f,"
            "\"hign\":%.1f,\"tc\":%.3f,\"te\":%.3f,\"roll\":%.2f,\"tign\":%.2f,\"ttd\":%.2f,"
            "\"ts\":%.3f,\"fg\":%.2f,\"apo\":%.1f,\"res\":%.2f,\"rx\":[%lu,%lu],\"plan\":%lu,"
            "\"loop\":%u,\"ovr\":%u,\"cont\":%u,\"fired\":%u,\"act\":%u,\"nfl\":%u,\"ev\":%u,"
            "\"drag\":%.3f,\"blo\":%.2f,\"bhi\":%.2f,"
            "\"tvc\":[%.2f,%.2f],\"cl\":%.2f,\"fin\":[%.1f,%.1f,%.1f,%.1f],\"afl\":%u}\n",
            nav_status_fresh() ? 1 : 0, s_nav_status.fw_version, s_nav_status.param_crc,
            s_nav_status.params_seen, s_nav_status.phase, s_nav_status.k_plan,
            s_nav_status.thrust_est, s_nav_status.h_ignite, s_nav_status.tilt_cmd,
            s_nav_status.tilt_err, s_nav_status.roll_rate, s_nav_status.t_to_ign,
            s_nav_status.t_to_td, s_nav_status.t_scale, s_nav_status.fin_gain,
            s_nav_status.h_apogee, s_nav_status.plan_resid,
            (unsigned long)s_nav_status.rx_ok, (unsigned long)s_nav_status.rx_bad,
            (unsigned long)s_nav_status.plan_iters, s_nav_status.loop_max_us,
            s_nav_status.overruns, s_nav_status.pyro_cont, s_nav_status.pyro_fired,
            s_nav_status.pyro_active, s_nav_status.flags, s_nav_status.events,
            s_nav_status.drag_scale, s_nav_status.band_lo, s_nav_status.band_hi,
            s_act.tvc_x_deg, s_act.tvc_y_deg, s_act.clamp, s_act.fin_deg[0], s_act.fin_deg[1],
            s_act.fin_deg[2], s_act.fin_deg[3], s_act.flags);
        break;
    }
    _Pragma("GCC diagnostic pop")
    which = (uint8_t)((which + 1) % 5);
    if (n > 0 && n < (int)sizeof(b)) usb_puts_n(b, (uint16_t)n);
}

// =====================================================================
//  10b. RAMPA UMBILICAL I2C (0x2B) & BIDIRECTIONAL LORA UPLINK (fw 3.34)
//
//  Rampa sits on Master's I2C0 bus (I2C1/I2C2 uSup -> magnetic pogo ->
//  Rampa J3 GP0/GP1). Master polls Rampa at 20 Hz while on the ground,
//  synchronizing the exact T-0 countdown (< 1 ms) and cross-checking
//  interlocks. At T - ARIADNA_UMB_DETACH_MS (100 ms), after the mandatory
//  T - 1 s LoRa final-abort window has closed, Master sends
//  RAMPA_CMD_COMMIT, confirms RAMPA_STATE_COMMITTED, and stops addressing
//  0x2B while Rampa disables its I2C0 slave (Hi-Z) before liftoff.
// =====================================================================
struct LoRaSyncSnap {
    uint32_t utc_base_ms;
    uint32_t utc_ref_ms;
    uint32_t cd_t0_ms;
    uint16_t rampa_vbat_mv;
    uint8_t  utc_valid;
    uint8_t  utc_from_gps;
    uint8_t  cd_active;
    uint8_t  in_flight;
    uint8_t  last_cmd_seq;
    uint8_t  last_cmd_rc;
    uint8_t  rampa_state;
    uint8_t  rampa_flags;
    uint8_t  rampa_fault;
    uint8_t  rampa_detached;
    uint8_t  launch_soon;
    uint8_t  rocket_ready;
};

static volatile uint32_t s_sync_seq = 0;
static LoRaSyncSnap      s_sync_shared;

// Core 1 -> Core 0 uplink mailbox (single-slot lock-free handoff).
static ariadna_lora_uplink_t s_ul_box;
static uint32_t              s_ul_rx_ms = 0;
static volatile uint32_t     s_ul_rx_seq = 0;   // bumped by Core 1 on new packet
static uint32_t              s_ul_seen_seq = 0; // consumed by Core 0
static volatile uint32_t     s_ul_ack_req = 0;  // bumped by Core 0 to trigger immediate ACK beacon

// Core 0 Rampa & countdown state.
static ariadna_rampa_status_t s_rampa_stat;
static bool     s_rampa_connected = false;
static bool     s_rampa_detached  = false;
static uint32_t s_rampa_last_ok_ms = 0;
static uint32_t s_rampa_detach_ms  = 0;
static uint8_t  s_rampa_seq        = 0;

static bool     s_launch_soon       = false;
static bool     s_cd_active         = false;
static bool     s_cd_commit_sent    = false;
static uint32_t s_cd_commit_sent_ms = 0;
static bool     s_cd_committed      = false;
static uint32_t s_cd_t0_ms          = 0;
static uint8_t  s_cd_pyro_ch       = 1;
static uint8_t  s_ul_last_seq      = 0;
static uint8_t  s_ul_last_rc       = ARIADNA_ULRC_NONE;
static bool     s_reboot_pending   = false;
static uint32_t s_reboot_at_ms     = 0;

static uint32_t rocket_utc_now_ms(uint32_t now_ms)
{
    if (!s_utc_valid) return ARIADNA_UTC_UNKNOWN;
    return (s_utc_base_ms + (uint32_t)(now_ms - s_utc_ref_ms)) % 86400000u;
}

static bool rocket_ready_for_launch(void)
{
    return s_post_pass &&
           s_have_nav && link_alive(s_nav_us) &&
           s_have_act && link_alive(s_act_us) &&
           (s_nav.flags & ARIADNA_NAV_VALID) != 0u &&
           s_servo_ok &&
           !(s_flags & ARIADNA_MST_ABORT) &&
           !s_test && !s_hil && !s_polarity;
}

static uint8_t rocket_flags_for_rampa(void)
{
    uint8_t f = 0;
    if (s_post_pass)                           f |= ARIADNA_RK_POST_OK;
    if (s_nav.flags & ARIADNA_NAV_VALID)       f |= ARIADNA_RK_NAV_VALID;
    if (s_flags & ARIADNA_MST_ARMED)           f |= ARIADNA_RK_ARMED;
    if (s_rbf_out)                             f |= ARIADNA_RK_RBF_OUT;
    if (s_have_nav && link_alive(s_nav_us))    f |= ARIADNA_RK_POS_OK;
    if (s_have_act && link_alive(s_act_us))    f |= ARIADNA_RK_NAV_OK;
    if (!(s_flags & ARIADNA_MST_ABORT))        f |= ARIADNA_RK_NO_ABORT;
    if (s_utc_valid && s_utc_from_gps)         f |= ARIADNA_RK_GPS_UTC;
    return f;
}

static void sync_publish(uint32_t now_ms)
{
    LoRaSyncSnap sn;
    memset(&sn, 0, sizeof(sn));
    sn.utc_base_ms    = s_utc_base_ms;
    sn.utc_ref_ms     = s_utc_ref_ms;
    sn.cd_t0_ms       = s_cd_t0_ms;
    sn.rampa_vbat_mv  = s_rampa_connected ? s_rampa_stat.vbat_mv : 0u;
    sn.utc_valid      = s_utc_valid ? 1u : 0u;
    sn.utc_from_gps   = s_utc_from_gps ? 1u : 0u;
    sn.cd_active      = s_cd_active ? 1u : 0u;
    sn.in_flight      = (s_state >= ARIADNA_STATE_ASCENT && s_state <= ARIADNA_STATE_LANDED) ? 1u : 0u;
    sn.last_cmd_seq   = s_ul_last_seq;
    sn.last_cmd_rc    = s_ul_last_rc;
    sn.rampa_state    = s_rampa_connected ? s_rampa_stat.state
                        : (s_cd_committed ? (uint8_t)RAMPA_STATE_COMMITTED : (uint8_t)RAMPA_STATE_OFFLINE);
    sn.rampa_flags    = s_rampa_connected ? s_rampa_stat.flags : 0u;
    sn.rampa_fault    = s_rampa_connected ? s_rampa_stat.fault_mask : 0u;
    sn.rampa_detached = s_rampa_detached ? 1u : 0u;
    sn.launch_soon    = s_launch_soon ? 1u : 0u;
    sn.rocket_ready   = rocket_ready_for_launch() ? 1u : 0u;
    (void)now_ms;

    s_sync_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_sync_shared = sn;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_sync_seq++;
}

static bool sync_acquire(LoRaSyncSnap *sn)
{
    uint32_t a = s_sync_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *sn = s_sync_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_sync_seq == a;
}

/** Send one Rocket -> Rampa command to Navigation MCU (which owns I2CN / Wire1).
 *  Master MCU's I2C0 (GP8/GP9) stays 100% dedicated to PCA9685 servos. */
static void rampa_send_cmd(uint8_t cmd_id, int32_t t_rem_ms, uint32_t now_ms)
{
    if (s_rampa_detached) return;

    ariadna_rampa_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd          = cmd_id;
    c.seq          = ++s_rampa_seq;
    c.t_rem_ms     = t_rem_ms;
    c.utc_ms       = rocket_utc_now_ms(now_ms);
    c.rocket_flags = rocket_flags_for_rampa();
    c.pyro_ch      = s_cd_pyro_ch;
    ariadna_rampa_cmd_seal(&c);

    send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_RAMPA_CMD, &c, (uint8_t)sizeof(c));
}

static void launch_abort_now(uint32_t now_ms, const char *reason)
{
    s_cd_active      = false;
    s_cd_commit_sent = false;
    s_cd_committed   = false;
    s_launch_soon    = false;
    if (!s_rampa_detached && s_rampa_connected) {
        rampa_send_cmd(RAMPA_CMD_ABORT, -1, now_ms);
    }
    MASTER_PRINT("# LAUNCH ABORT (%s)\n", reason);
    s_ul_ack_req = s_ul_ack_req + 1u;
}

/** Handle a verified Rampa status frame forwarded from Navigation MCU over UART1. */
static void rampa_on_status(const ariadna_rampa_status_t &st, uint32_t now_ms)
{
    s_rampa_stat       = st;
    s_rampa_connected  = true;
    s_rampa_last_ok_ms = now_ms ? now_ms : 1u;

    if (s_cd_active && !s_cd_committed) {
        if (s_cd_commit_sent) {
            if (st.state == RAMPA_STATE_COMMITTED) {
                s_cd_committed    = true;
                s_rampa_detached  = true;
                s_rampa_connected = false;
                s_rampa_detach_ms = now_ms;
                MASTER_PRINT("# RAMPA COMMITTED (T-%ld ms), Nav I2CN umbilical detached\n",
                             (long)st.t_rem_ms);
            } else {
                launch_abort_now(now_ms, "Rampa COMMIT refused");
            }
            return;
        }
        const uint8_t cont_bit = (uint8_t)(ARIADNA_RMP_CONT1 << (s_cd_pyro_ch - 1u));
        if (st.state == RAMPA_STATE_ABORT ||
            st.state == RAMPA_STATE_FAULT ||
            st.fault_mask != 0u ||
            !(st.flags & ARIADNA_RMP_KEY_ARMED) ||
            !(st.flags & cont_bit)) {
            launch_abort_now(now_ms, "Rampa interlock/abort");
        }
    }
}

/** Process one verified LoRa uplink command handed off from Core 1. */
static void handle_lora_uplink(const ariadna_lora_uplink_t &u, uint32_t rx_ms)
{
    const uint32_t now = millis();
    const bool in_flight = (s_state >= ARIADNA_STATE_ASCENT && s_state <= ARIADNA_STATE_LANDED);

    // Deduplicate retries of the same command (ABORT is always idempotent).
    if (u.seq != 0 && u.seq == s_ul_last_seq && u.cmd != ARIADNA_UL_ABORT) {
        s_ul_ack_req = s_ul_ack_req + 1u;
        return;
    }

    // If GPS has not yet locked UTC, accept GS PC UTC so windows align cleanly.
    if (!s_utc_from_gps && u.utc_ms != ARIADNA_UTC_UNKNOWN && u.utc_ms < 86400000u) {
        const uint32_t air_ms = ariadna_lora_airtime_us(sizeof(ariadna_lora_uplink_t)) / 1000u;
        s_utc_base_ms = (u.utc_ms + air_ms) % 86400000u;
        s_utc_ref_ms  = rx_ms;
        s_utc_valid   = true;
    }

    s_ul_last_seq = u.seq;
    s_ul_last_rc  = ARIADNA_ULRC_OK;

    switch (u.cmd) {
    case ARIADNA_UL_NOP:
        s_ul_last_rc = ARIADNA_ULRC_OK;
        break;

    case ARIADNA_UL_PADREF:
        if (in_flight) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_FLIGHT;
        } else {
            broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);
            s_ul_last_rc = ARIADNA_ULRC_OK;
        }
        break;

    case ARIADNA_UL_LAUNCH_SOON:
        if (in_flight) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_FLIGHT;
        } else if (u.token != ARIADNA_UL_TOK_PREP) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_TOKEN;
        } else {
            s_launch_soon    = true;
            s_rampa_detached = false;
            broadcast_cmd(ARIADNA_CMD_SET_PAD_REF, 0, 0);
            rampa_send_cmd(RAMPA_CMD_PREPARE, -1, now);
            s_ul_last_rc = ARIADNA_ULRC_OK;
        }
        break;

    case ARIADNA_UL_COUNTDOWN: {
        const uint8_t ch = (u.target >= 1u && u.target <= 3u) ? u.target : 1u;
        const uint8_t cont_bit = (uint8_t)(ARIADNA_RMP_CONT1 << (ch - 1u));
        if (in_flight) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_FLIGHT;
        } else if (u.token != ARIADNA_UL_TOK_COUNTDOWN ||
                   u.arg_ms < ARIADNA_COUNTDOWN_MIN_MS ||
                   u.arg_ms > ARIADNA_COUNTDOWN_MAX_MS) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_TOKEN;
        } else if (!rocket_ready_for_launch() ||
                   (s_state != ARIADNA_STATE_IDLE && s_state != ARIADNA_STATE_ARMED)) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_STATE;
        } else if (!s_rampa_connected || s_rampa_detached ||
                   !(s_rampa_stat.flags & ARIADNA_RMP_KEY_ARMED) ||
                   !(s_rampa_stat.flags & cont_bit) ||
                   s_rampa_stat.fault_mask != 0u) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_RAMPA;
        } else {
            // Compensate for uplink LoRa airtime + cross-core latency (~28 ms)
            // so Rocket, Rampa, and Ground Station hit T-0 within <= 5 ms.
            const uint32_t air_ms = ariadna_lora_airtime_us(sizeof(ariadna_lora_uplink_t)) / 1000u;
            const uint32_t elapsed = (uint32_t)(now - rx_ms) + air_ms;
            const uint32_t rem = (u.arg_ms > elapsed) ? (u.arg_ms - elapsed) : 0u;
            s_cd_pyro_ch     = ch;
            s_cd_t0_ms       = now + rem;
            s_cd_active      = true;
            s_cd_commit_sent = false;
            s_cd_committed   = false;
            s_launch_soon    = true;
            rampa_send_cmd(RAMPA_CMD_COUNTDOWN, (int32_t)rem, now);
            MASTER_PRINT("# LORA COUNTDOWN T-%lu ms (ch %u)\n", (unsigned long)rem, (unsigned)ch);
            s_ul_last_rc = ARIADNA_ULRC_OK;
        }
        break;
    }

    case ARIADNA_UL_ABORT:
        if (u.token != ARIADNA_UL_TOK_ABORT) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_TOKEN;
        } else {
            launch_abort_now(now, "LoRa ABORT");
            if (s_state == ARIADNA_STATE_ARMED) {
                s_flags |= ARIADNA_MST_ABORT;
                broadcast_cmd(ARIADNA_CMD_ABORT, 0, 0);
            }
            s_ul_last_rc = ARIADNA_ULRC_ABORTED;
        }
        break;

    case ARIADNA_UL_DISARM:
        if (in_flight) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_FLIGHT;
        } else {
            s_cd_active      = false;
            s_cd_commit_sent = false;
            s_cd_committed   = false;
            s_launch_soon    = false;
            s_flags &= (uint8_t)~(ARIADNA_MST_ARMED | ARIADNA_MST_ABORT);
            broadcast_cmd(ARIADNA_CMD_DISARM, 0, 0);
            if (!s_rampa_detached && s_rampa_connected)
                rampa_send_cmd(RAMPA_CMD_PING, -1, now);
            s_ul_last_rc = ARIADNA_ULRC_OK;
        }
        break;

    case ARIADNA_UL_REBOOT:
        if (in_flight) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_FLIGHT;
        } else if (u.token != ARIADNA_UL_TOK_REBOOT) {
            s_ul_last_rc = ARIADNA_ULRC_REJECT_TOKEN;
        } else {
            s_cd_active      = false;
            s_cd_commit_sent = false;
            s_launch_soon    = false;
            if (!s_rampa_detached && s_rampa_connected)
                rampa_send_cmd(RAMPA_CMD_ABORT, -1, now);
            broadcast_cmd(ARIADNA_CMD_REBOOT, u.target, 0);
            if (u.target == 0u) {
                s_reboot_pending = true;
                s_reboot_at_ms   = now + 200u; // let ACK beacon & UART commands flush
            }
            s_ul_last_rc = ARIADNA_ULRC_OK;
        }
        break;

    default:
        s_ul_last_rc = ARIADNA_ULRC_REJECT_TOKEN;
        break;
    }
    s_ul_ack_req = s_ul_ack_req + 1u;
}

/** Called at 200 Hz from Core 0 loop(). Handles uplink inbox, Rampa UART->I2CN
 *  polling @ 20 Hz, pre-liftoff commit/detach at T - 100 ms, and publishes
 *  sync snapshot to Core 1. */
static void rampa_and_uplink_service(void)
{
    static uint32_t next_rampa_ms = 0;
    const uint32_t now = millis();
    const bool in_flight = (s_state >= ARIADNA_STATE_ASCENT && s_state <= ARIADNA_STATE_LANDED);

    // 1. Deferred ground-only reboot after ACK flush.
    if (s_reboot_pending && !in_flight && (int32_t)(now - s_reboot_at_ms) >= 0) {
        digitalWrite(PIN_PCA_OE, HIGH);
        watchdog_enable(1, 1);
        while (1) { }
    }

    // 2. Consume any uplink packet received by Core 1.
    const uint32_t rx_seq = s_ul_rx_seq;
    if (rx_seq != s_ul_seen_seq) {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const ariadna_lora_uplink_t u = s_ul_box;
        const uint32_t rx_ms = s_ul_rx_ms;
        s_ul_seen_seq = rx_seq;
        handle_lora_uplink(u, rx_ms);
    }

    // 3. Liftoff detected -> immediately freeze umbilical & clear countdown.
    if (in_flight) {
        s_cd_active       = false;
        s_cd_commit_sent  = false;
        s_cd_committed    = false;
        s_launch_soon     = false;
        s_rampa_detached  = true;
        s_rampa_connected = false;
        sync_publish(now);
        return;
    }

    // 4. If we detached for liftoff, check for no-liftoff timeout (T + 5 s).
    if (s_rampa_detached && s_cd_committed && (uint32_t)(now - s_rampa_detach_ms) > 5000u) {
        s_cd_active      = false;
        s_cd_commit_sent = false;
        s_cd_committed   = false;
        s_launch_soon    = false;
        s_rampa_detached = false;
        MASTER_PRINT("# RAMPA: no liftoff within 5 s after commit -> re-enabled umbilical\n");
    }

    // 5. Active countdown checks & T - 100 ms commit / hot-detach via Navigation I2CN.
    int32_t t_rem_ms = -1;
    if (s_cd_active) {
        t_rem_ms = (int32_t)(s_cd_t0_ms - now);
        if (!s_cd_committed) {
            if (!rocket_ready_for_launch()) {
                launch_abort_now(now, "Rocket not ready");
                t_rem_ms = -1;
            } else if (!s_rampa_connected) {
                launch_abort_now(now, "Rampa link lost");
                t_rem_ms = -1;
            } else if (s_cd_commit_sent) {
                // Waiting for Navigation MCU to return RAMPA_STATE_COMMITTED (~4 ms).
                if ((uint32_t)(now - s_cd_commit_sent_ms) > 25u) {
                    launch_abort_now(now, "Rampa COMMIT timeout");
                    t_rem_ms = -1;
                }
            } else if (t_rem_ms <= (int32_t)ARIADNA_UMB_DETACH_MS) {
                // Final T - 100 ms handoff: Navigation MCU writes COMMIT over
                // dedicated I2CN (Wire1), reads back COMMITTED, and calls Wire1.end().
                const int32_t send_rem = t_rem_ms > 10 ? t_rem_ms : 10;
                s_cd_commit_sent    = true;
                s_cd_commit_sent_ms = now;
                rampa_send_cmd(RAMPA_CMD_COMMIT, send_rem, now);
            }
        }
    }

    // 6. Periodic 20 Hz Rampa poll forwarded to Navigation MCU (I2CN) while on pad.
    if (!s_rampa_detached && !s_cd_commit_sent && !s_test && !s_hil &&
        (int32_t)(now - next_rampa_ms) >= 0) {
        next_rampa_ms = now + 1000u / ARIADNA_RAMPA_POLL_HZ;
        const uint8_t cmd = s_cd_active   ? (uint8_t)RAMPA_CMD_COUNTDOWN
                          : s_launch_soon ? (uint8_t)RAMPA_CMD_PREPARE
                                          : (uint8_t)RAMPA_CMD_PING;
        rampa_send_cmd(cmd, t_rem_ms, now);
    }

    // 7. Supervise Rampa heartbeat timeout.
    if (s_rampa_connected && !s_rampa_detached &&
        (uint32_t)(now - s_rampa_last_ok_ms) > ARIADNA_RAMPA_TIMEOUT_MS) {
        s_rampa_connected = false;
        if (s_cd_active && !s_cd_committed)
            launch_abort_now(now, "Rampa I2CN timeout");
    }

    sync_publish(now);
}

// =====================================================================
//  11. CORE 0 — routing, servos, state machine
// =====================================================================
static void send_master_state(void)
{
    ariadna_master_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.t_ms        = millis();
    ms.t_launch_ms = s_t_launch_ms;
    ms.vbat_mv     = (uint16_t)(s_vbat * 1000.0f);
    ms.current_ma  = (uint16_t)(s_current * 1000.0f);
    ms.state       = s_state;
    ms.flags       = s_flags;
    ms.nav_events  = s_act.events;
    if (s_flash_ok && !s_hil) ms.flags |= ARIADNA_MST_LOGGING;
    if (s_raw_on && on_ground_state()) ms.flags |= ARIADNA_MST_RAW_IMU;
    if (s_polarity && s_state == ARIADNA_STATE_IDLE) {
        ms.flags2 |= ARIADNA_MST2_POLARITY_TEST;
        ms.flags2 |= ariadna_pol_thrust_encode(s_pol_thrust_n);
        ms.t_launch_ms = ariadna_pol_speed_encode(s_pol_speed_ms);
    }
    send_to(&s_tx_pos, POS_PORT, ARIADNA_PKT_MASTER_STATE, &ms, sizeof(ms));
    send_to(&s_tx_nav, NAV_PORT, ARIADNA_PKT_MASTER_STATE, &ms, sizeof(ms));
}

void setup()
{
    // Navigation held in reset until this board is ready, so it cannot
    // start streaming into a UART that is not listening yet. Position has
    // no reset line and boots on its own; whatever it sends before
    // POS_PORT is up is lost, and the framed link resynchronises on the
    // first whole packet after it.
    pinMode(PIN_RST_NAV, OUTPUT); digitalWrite(PIN_RST_NAV, LOW);
    pinMode(PIN_PCA_OE,  OUTPUT); digitalWrite(PIN_PCA_OE,  HIGH);   // servos off

    pinMode(PIN_BREAKWIRE, INPUT_PULLUP);
    pinMode(PIN_RBF,       INPUT_PULLUP);
    pinMode(PIN_AUX_S16,   OUTPUT); digitalWrite(PIN_AUX_S16, LOW);
    pinMode(PIN_BUZZER,    OUTPUT);
    analogReadResolution(12);

    Serial.begin(ARIADNA_CONSOLE_BAUD);

    // The flash first, then the parameters (they live on it), then
    // everything that reads one (servo trims, below). Core 1 is still
    // waiting in setup1(), so the bus is ours.
    s_flash_ok = flash_begin();
    params_load();
    batt_init(&s_batt);
    batt_set_learn(&s_batt, PRM(BATT_LEARN));
    s_learn_stored = PRM(BATT_LEARN);

    // 0xFF: the SK6812 is powered from the board's own rail, so there is
    // no LED power-enable pin to drive as there is on the XIAOs.
    ariadna_rgb_begin(&s_rgb, 60, 0xFF);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);
    ariadna_buzzer_begin(&s_buzz, PIN_BUZZER);

    // ---- servo bus ----
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(3);
    s_servo_ok = pca_begin();
    for (uint8_t ch = 0; ch < PCA_N_CH; ch++) s_pca[ch] = PCA_FULL_OFF;
    s_ch[0] = 0xFF;                      // force servo_map_update() to build the map
    servo_map_update();                  // srv.ch_* from the parameters (loaded above)
    servos_park();
    servos_write();
    digitalWrite(PIN_PCA_OE, LOW);       // outputs on, at the parked pulse


    // ---- inter-processor links ----
    // ariadna_uart_begin() sets a 2 kB receive buffer before begin(); the
    // arduino-pico default of 32 bytes overflowed every 5 ms tick and cost
    // ~99 % of the frames (see ariadna_uart_dma.h).
    ariadna_uart_begin(POS_PORT, PIN_POS_TX, PIN_POS_RX);
    ariadna_uart_begin(NAV_PORT, PIN_NAV_TX, PIN_NAV_RX);

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
    memset(&s_pyro_rep,   0, sizeof(s_pyro_rep));
    s_nav.qw = 1.0f;
    s_act.clamp = DG_CLAMP_MAX;
    for (uint8_t i = 0; i < PCA_N_CH; i++) {
        s_test_lim_lo[i] = (uint16_t)SERVO_US_MIN;
        s_test_lim_hi[i] = (uint16_t)SERVO_US_MAX;
    }

    s_post_pass = self_test();
    MASTER_PRINT("\nAriadna V2 Master MCU — POST %s\n", s_post_pass ? "PASS" : "FAIL");
    MASTER_PRINT("# fw %d, servo bus %s, flash %s, parameters from %s\n",
                 ARIADNA_FW_VERSION, s_servo_ok ? "ok" : "FAIL", s_flash_ok ? "ok" : "FAIL",
                 s_params_from_flash ? "FLASH" : "DEFAULTS");

    // Release the Navigation MCU now that both UARTs are up.
    digitalWrite(PIN_RST_NAV, HIGH);

    s_state_since = millis();
}

void loop()
{
    static uint32_t next_us   = 0;
    static uint32_t next_log_ms = 0;
    static uint32_t next_bcast_ms = 0;
    const  uint32_t period    = 5000;      // 200 Hz

    // ---- routing, highest priority, on EVERY pass ------------------------
    // Not gated on the 5 ms tick. Reading the ports only once per tick
    // left the receive buffers to absorb 5 ms of 921600 baud traffic, and
    // forwarding the nav state once per tick added up to 5 ms of jitter to
    // the Navigation MCU's input. Here a frame is forwarded the moment its
    // last byte arrives and the actuator command reaches the PCA9685 as
    // soon as it is complete.
    tx_service_all();     // retire finished DMA runs before queuing more
    poll_position();      // forwards nav state to Navigation first
    poll_navigation();    // applies actuator commands to the servos

    uint32_t now = micros();
    if (next_us == 0) next_us = now;
    if ((int32_t)(now - next_us) < 0) return;
    next_us += period;
    if ((int32_t)(micros() - next_us) > 0) next_us = micros() + period;

    // ---- housekeeping ---------------------------------------------------
    read_analogue();
    batt_service();       // battery run-time estimate, 1 Hz feed (section 7)
    srvchk_service();     // servo connection check, when one is running (section 3b)
    state_machine();
    rampa_and_uplink_service();
    params_stream();
    usb_service();        // diagnostic GUI: a no-op unless a host has the port open
    params_request_poll();
    if (s_test) test_service();
    startup_tune_service();
    annunciate();
    ariadna_rgb_service(&s_rgb);
    ariadna_buzzer_service(&s_buzz);

    // ---- state broadcast, 10 Hz -----------------------------------------
    // 10 Hz, and AT ONCE when the Navigation MCU reports a new flight event:
    // the Position MCU times its lidar from the moment the legs open, and a
    // 100 ms wait for the next slot would be 100 ms of a usable lidar lost.
    {
        static uint8_t last_events = 0;
        const bool event_new = (s_act.events != last_events);
        if (millis() >= next_bcast_ms || event_new) {
            next_bcast_ms = millis() + 1000 / ARIADNA_MASTER_HZ;
            last_events = s_act.events;
            send_master_state();
        }
    }

    // ---- telemetry snapshot and log -------------------------------------
    {
        ariadna_tlm_t t;
        build_telemetry(&t);
        tlm_publish(&t);
    }

    // Flight: every tick. Ground: 10 Hz, while there is more than the
    // flight reserve left. Not while the log is being read or erased, and
    // not during HIL simulation (the PC records the full HIL trace).
    const bool in_flight = s_state >= ARIADNA_STATE_ASCENT && s_state <= ARIADNA_STATE_LANDED;
    const bool ground_ok = s_log_addr + LOG_FLIGHT_RESERVE < FLASH_SIZE && !log_service_busy();
    if (s_flash_ok && !s_hil && millis() >= next_log_ms && (in_flight || ground_ok)) {
        next_log_ms = millis() + 1000 / (in_flight ? LOG_HZ_FLIGHT : LOG_HZ_GROUND);
        if (in_flight) next_log_ms = millis();          // every 200 Hz tick
        s_flags |= ARIADNA_MST_LOGGING;

        LogRecord r;
        memset(&r, 0, sizeof(r));
        r.t_ms  = millis();
        r.px    = s_nav.px;  r.py = s_nav.py;
        r.h     = s_nav.pz;
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
        r.k_plan     = s_have_nav_status ? s_nav_status.k_plan : 0.0f;
        r.events     = s_have_nav_status ? s_nav_status.events : 0;
        {
            float te = !s_have_nav_status ? -1.0f
                     : (s_nav_status.phase >= DG_PHASE_IGNITING ? s_nav_status.t_to_td
                                                                : s_nav_status.t_to_ign);
            r.t_event_ds = (int8_t)(te < 0.0f ? -1 : (te > 12.7f ? 127 : (int)(te * 10.0f)));
            float fg = s_have_nav_status ? s_nav_status.fin_gain * 100.0f : 0.0f;
            r.fin_gain_pct = (uint8_t)(fg > 255.0f ? 255.0f : (fg < 0.0f ? 0.0f : fg));
            float dg = s_have_nav_status ? s_nav_status.drag_scale * 100.0f : 0.0f;
            r.drag_pct = (uint8_t)(dg > 255.0f ? 255.0f : (dg < 0.0f ? 0.0f : dg));
            float ts = s_have_nav_status ? s_nav_status.t_scale * 100.0f : 0.0f;
            r.tscale_pct = (uint8_t)(ts > 255.0f ? 255.0f : (ts < 0.0f ? 0.0f : ts));
        }
        r.flags      = in_flight ? 1 : 0;
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

    // ---- USB diagnostics ----------------------------------------------------
    // Last, and only queued here: usb_service() above does the sending.
    diag_emit();
}

// =====================================================================
//  12. CORE 1 — flash log and LoRa downlink
//
//  Everything here can block for tens or hundreds of milliseconds. That is
//  precisely why it is here and not on core 0.
// =====================================================================

/**
 * Keep erased space ahead of the write pointer.
 *
 * Erasing on demand would stall the log for up to 400 ms exactly when it
 * is busiest. Erasing ahead means the erase happens during the quiet parts
 * of the flight, and the RAM ring covers whatever is left.
 */
static void flash_erase_ahead(void)
{
    const uint32_t LOOKAHEAD = 16 * FLASH_SECTOR;    // 64 kB: 3 s at 200 Hz

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
    static uint32_t next_tlm_ms   = 0;
    static uint32_t next_sync_ms  = 0;
    static uint32_t win_open_ms   = 0;
    static uint32_t rx_end_ms     = 0;
    static bool     win_armed     = false;
    static bool     win_is_final  = false;
    static bool     rx_active     = false;
    static uint32_t last_ack_seen = 0;
    static LoRaSyncSnap sn        = {};

    // Flash first: the log is the only record that survives a lost link.
    if (s_flash_ok) {
        flash_param_service();
        flash_erase_ahead();
        flash_drain();
    }

    if (!s_lora_ok) return;

    (void)sync_acquire(&sn);
    const uint32_t now = millis();
    const uint32_t utc_ms = sn.utc_valid
        ? (sn.utc_base_ms + (uint32_t)(now - sn.utc_ref_ms)) % 86400000u
        : ARIADNA_UTC_UNKNOWN;
    const int32_t t_rem_ms = sn.cd_active ? (int32_t)(sn.cd_t0_ms - now) : -1;

    // ---- 1. Active LoRa RX window (300 ms every 10 s, plus T - 1 s) -------
    if (rx_active) {
        if (sn.in_flight) {
            rx_active = false;
            LoRa.idle();
        } else {
            const int sz = LoRa.parsePacket();
            if (sz == (int)sizeof(ariadna_lora_uplink_t)) {
                uint8_t buf[sizeof(ariadna_lora_uplink_t)];
                LoRa.readBytes(buf, sizeof(buf));
                ariadna_lora_uplink_t ul;
                if (ariadna_lora_uplink_decode(buf, sizeof(buf), &ul)) {
                    s_ul_box   = ul;
                    s_ul_rx_ms = now;
                    __atomic_thread_fence(__ATOMIC_RELEASE);
                    s_ul_rx_seq = s_ul_rx_seq + 1u;
                }
            } else if (sz > 0) {
                while (LoRa.available()) LoRa.read();
            }
            if ((int32_t)(now - rx_end_ms) >= 0) {
                rx_active    = false;
                LoRa.idle();
                // Schedule an ACK/sync beacon 25 ms later once Core 0 has
                // processed any received command and updated Rampa state.
                next_sync_ms = now + 25u;
            }
            return;
        }
    }

    // ---- 2. Schedule next RX window (ground / countdown only) --------------
    if (!sn.in_flight) {
        int is_fin = 0;
        const uint32_t to_win = ariadna_lora_next_window_ms(
            now, utc_ms, sn.utc_valid ? 1 : 0, t_rem_ms, &is_fin);
        if (!win_armed) {
            win_open_ms  = now + to_win;
            win_is_final = (is_fin != 0);
            win_armed    = true;
        } else {
            // Pull the scheduled window forward if a T-1 s final abort window
            // or a fresh UTC lock requires an earlier opening.
            const uint32_t cand = now + to_win;
            if ((int32_t)(cand - win_open_ms) < -150) {
                win_open_ms  = cand;
                win_is_final = (is_fin != 0);
            }
        }

        const int32_t until_win = (int32_t)(win_open_ms - now);
        if (until_win <= 0) {
            rx_active = true;
            rx_end_ms = now + ARIADNA_LORA_WIN_DUR_MS;
            win_armed = false;
            LoRa.receive();
            return;
        }
        // Guard band: do not start a new TX that would collide with RX open.
        if (until_win <= (int32_t)ARIADNA_LORA_TX_GUARD_MS) {
            return;
        }
    } else {
        win_armed = false;
    }

    // ---- 3. 1 Hz sync/ACK/Rampa beacon on ground & countdown ---------------
    const bool ack_due = (s_ul_ack_req != last_ack_seen);
    const bool sync_due = !sn.in_flight && (ack_due || (int32_t)(now - next_sync_ms) >= 0);
    if (sync_due) {
        if (!LoRa.beginPacket()) {
            // Previous packet still on air; retry next pass.
            return;
        }
        const uint32_t tx_now = millis();
        const uint32_t tx_utc = sn.utc_valid
            ? (sn.utc_base_ms + (uint32_t)(tx_now - sn.utc_ref_ms)) % 86400000u
            : ARIADNA_UTC_UNKNOWN;
        const int32_t tx_rem = sn.cd_active ? (int32_t)(sn.cd_t0_ms - tx_now) : -1;
        uint32_t to_next = (win_armed && (int32_t)(win_open_ms - tx_now) > 0)
                               ? (uint32_t)(win_open_ms - tx_now)
                               : ARIADNA_LORA_WIN_PERIOD_MS;
        if (to_next > 65535u) to_next = 65535u;

        ariadna_lora_sync_t sb;
        memset(&sb, 0, sizeof(sb));
        sb.rocket_t_ms     = tx_now;
        sb.utc_ms          = tx_utc;
        sb.next_rx_in_ms   = (uint16_t)to_next;
        sb.rx_dur_ms       = ARIADNA_LORA_WIN_DUR_MS;
        sb.t_launch_rem_ms = tx_rem;
        sb.last_cmd_seq    = sn.last_cmd_seq;
        sb.last_cmd_rc     = sn.last_cmd_rc;
        sb.rampa_state     = sn.rampa_state;
        sb.rampa_flags     = sn.rampa_flags;
        sb.rampa_fault     = sn.rampa_fault;
        sb.rampa_vbat_mv   = sn.rampa_vbat_mv;
        sb.sync_flags      = 0;
        if (sn.utc_valid && sn.utc_from_gps) sb.sync_flags |= ARIADNA_SYNCF_GPS_UTC;
        if (win_is_final)                    sb.sync_flags |= ARIADNA_SYNCF_FINAL_WIN;
        if (sn.rampa_detached)               sb.sync_flags |= ARIADNA_SYNCF_RAMPA_DETACH;
        if (sn.launch_soon)                  sb.sync_flags |= ARIADNA_SYNCF_LAUNCH_SOON;
        if (sn.cd_active)                    sb.sync_flags |= ARIADNA_SYNCF_CD_ACTIVE;
        if (sn.rocket_ready)                 sb.sync_flags |= ARIADNA_SYNCF_ROCKET_READY;
        ariadna_lora_sync_seal(&sb);

        LoRa.write((const uint8_t *)&sb, sizeof(sb));
        LoRa.endPacket(true);
        last_ack_seen = s_ul_ack_req;
        next_sync_ms  = tx_now + (sn.cd_active ? 500u : 1000u);
        next_tlm_ms   = tx_now + 1000u / ARIADNA_TLM_HZ;
        return;
    }

    // ---- 4. Standard 10 Hz flight telemetry downlink -----------------------
    if (now >= next_tlm_ms) {
        ariadna_tlm_t t;
        if (!tlm_acquire(&t)) {
            // Torn snapshot: core 0 was mid-publish. Try again next pass.
        } else if (!LoRa.beginPacket()) {
            s_tlm_skipped++;                   // previous packet still on air
        } else {
            next_tlm_ms = now + 1000 / ARIADNA_TLM_HZ;
            LoRa.write((const uint8_t *)&t, sizeof(t));
            LoRa.endPacket(true);              // async
            s_tlm_sent++;
        }
    }
}
