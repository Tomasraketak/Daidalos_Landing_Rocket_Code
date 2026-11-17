// =====================================================================
//  Ariadna V2 — Actuator Console for the MASTER MCU
//
//  Target: Raspberry Pi Pico 2 W (RP2350), arduino-pico core.
//  Needs:  Adafruit_NeoPixel. The PCA9685 is driven by its registers.
//  Pair:   Examples/Pyro_Companion on the Navigation MCU (for the pyros)
//  GUI:    gui/actuator_gui.py on the laptop, over USB
//
//  The board is deliberately dumb: the GUI owns the servo map (which
//  servo is on which channel, its centre, its two aerocover positions),
//  the board owns the LIMITS. Every pulse is clamped on the board to the
//  per-channel window the GUI last sent (LIM), and that window is itself
//  clamped to ABS_US_MIN..ABS_US_MAX. A GUI bug cannot drive a linkage
//  into its end stop.
//
//  Every channel starts with NO PULSE (PCA9685 full-off) and stays that
//  way until the GUI commands it, so nothing moves at power-up.
//
//  Idle off: a channel whose command has not changed for IDLE (default
//  500 ms) stops its pulse, so a servo resting at a position does not sit
//  there buzzing and drawing holding current. The next SET wakes it.
//  `IDLE 0` keeps every servo powered.
//
//  Pulses go up to 3500 us for servos that use the wide range, but never
//  closer than PERIOD_GAP_US to the PWM period (at 339 Hz the period is
//  2950 us, so 2750 us is the most a pulse can be there).
//
//  Current: the ACS712 on GP26, sampled every millisecond. Telemetry
//  carries the mean over the last 100 ms and the peak over the last 10 s.
//
//  The pyros are on the Navigation MCU. The Master forwards ARM / DISARM /
//  FIRE to it over UART1 (actcon_pyro.h) and relays its continuity report
//  back to the GUI. Interlocks here, on top of the Navigation MCU's own:
//    * ARM and FIRE need the RBF pin pulled (REQUIRE_RBF)
//    * ARM and FIRE need the Navigation MCU to be answering
//    * the GUI must keep talking: 1.5 s of silence -> DISARM, heartbeats
//      to the Navigation MCU stop, and it disarms itself 0.5 s later
//
//  USB serial, 115200, lines ending in "\n". One command per line:
//
//    HELLO                 -> "I ACTCON 1 16"
//    PWM <hz>              PCA9685 frequency, 40..400 (50 = any servo,
//                          339 = flight rate, DIGITAL servos only)
//    LIM <ch> <min> <max>  per-channel pulse window, us
//    SET <ch> <us>         pulse on channel 0..15, clamped to its window
//    OFF <ch> | OFF ALL    stop the pulse (servo goes limp / holds)
//    IDLE <ms>             idle-off time, 0 = never, max 60000
//    ARM | DISARM          pyro arming on the Navigation MCU
//    FIRE <ch> <ms>        pyro channel 1..3, pulse 20..1000 ms
//    HB                    heartbeat (any line counts as one)
//    STATUS                one telemetry line now
//
//  Replies: "OK ...", "ERR ...", "# text" (log), and every 100 ms a
//  telemetry line "T {json}".
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "ariadna_link.h"
#include "actcon_pyro.h"

// ---- pins: identical to Master_MCU/Master_MCU.ino ----------------------
static const uint8_t PIN_RGB     = 2;    // SK6812
static const uint8_t PIN_BUZZER  = 3;
static const uint8_t PIN_NAV_TX  = 4;    // UART1 -> Navigation MCU
static const uint8_t PIN_NAV_RX  = 5;
static const uint8_t PIN_PCA_OE  = 6;    // PCA9685 /OE: LOW = outputs on
static const uint8_t PIN_I2C_SDA = 8;
static const uint8_t PIN_I2C_SCL = 9;
static const uint8_t PIN_I_SENSE = 26;   // ADC0, ACS712
static const uint8_t PIN_V_BAT   = 27;   // ADC1, 2S through a 3:1 divider
static const uint8_t PIN_RBF     = 28;   // HIGH = tag removed

#define NAV_PORT  Serial2                // UART1 on arduino-pico

// Set to 0 only on a bench with LEDs on the pyro outputs and no RBF tag.
#define REQUIRE_RBF  1
#define RBF_REMOVED_IS_HIGH  1

static const float ADC_VREF       = 3.30f;
static const float ADC_FULL_SCALE = 4095.0f;
static const float VBAT_DIVIDER   = 3.00f;
// ACS712: same constants as Master_MCU.ino. CHECK THEM AGAINST YOUR BOARD:
// the zero is ratiometric to the sensor's 5 V, re-measure if that changes.
static const float ACS712_ZERO_V  = 2.504f;    // sensor output at 0 A
static const float ACS712_V_PER_A = 0.100f;    // 20 A part: 100 mV/A
static const float ACS712_DIVIDER = 0.50f;     // pin volts / sensor volts
static const float ACS712_MIN_PIN_V = 0.10f;   // below this the sensor is unpowered

// ---- PCA9685 -------------------------------------------------------------
static const uint8_t  PCA_ADDR    = 0x40;
static const float    PCA_OSC_HZ  = 25000000.0f;
static const uint8_t  N_CH        = 16;
static const float    ABS_US_MIN  = 500.0f;    // nothing outside this, ever
static const float    ABS_US_MAX  = 3500.0f;   // wide-range servos
static const float    PERIOD_GAP_US = 200.0f;  // pulse never closer to the period
static const float    DEF_US_MIN  = 900.0f;    // window until the GUI sends LIM
static const float    DEF_US_MAX  = 2100.0f;   // = the flight travel guard

// ---- timing --------------------------------------------------------------
static const uint32_t GUI_TIMEOUT_MS = 1500;
static const uint32_t NAV_TIMEOUT_MS = 300;
static const uint32_t TLM_PERIOD_MS  = 100;
static const uint32_t PCA_CHECK_MS   = 1000;
static const uint32_t IDLE_DEFAULT_MS = 500;

// =====================================================================
//  state
// =====================================================================
static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

static bool     s_pca_ok  = false;
static float    s_pwm_hz  = 50.0f;
static uint16_t s_us[N_CH];            // commanded pulse, 0 = off
static bool     s_awake[N_CH];         // pulse actually output (not idled off)
static uint32_t s_moved_ms[N_CH];      // last time the command changed
static uint32_t s_idle_ms = IDLE_DEFAULT_MS;
static uint16_t s_lim_lo[N_CH];
static uint16_t s_lim_hi[N_CH];

static uint32_t s_gui_last = 0;        // last line from the GUI
static bool     s_gui_alive = false;

static ariadna_rx_t          s_nav_rx;
static actcon_pyro_report_t  s_nav;    // last report from the Navigation MCU
static uint32_t s_nav_last = 0;
static bool     s_nav_ok   = false;
static uint32_t s_pyro_seq = 1;

static bool  s_rbf_out = false;
static float s_vbat    = 0.0f;
static float s_amps_now = 0.0f;        // mean of the last telemetry period
static float s_amps_max = 0.0f;        // peak over the last 10 s

// =====================================================================
//  PCA9685
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
    const float c = us * (4096.0f * s_pwm_hz * 1e-6f) + 0.5f;
    if (c < 1.0f) return 1;
    if (c > 4095.0f) return 4095;
    return (uint16_t)c;
}

/** Longest pulse the current PWM frequency allows. */
static float max_pulse_us(void)
{
    const float lim = 1e6f / s_pwm_hz - PERIOD_GAP_US;
    return lim < ABS_US_MAX ? lim : ABS_US_MAX;
}

/** One channel: a pulse of s_us[ch], or full-off when it is 0 or idled. */
static bool pca_write_ch(uint8_t ch)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(0x06 + 4 * ch));          // LEDn_ON_L, auto-increment
    if (s_us[ch] == 0 || !s_awake[ch]) {
        Wire.write(0); Wire.write(0);
        Wire.write(0); Wire.write(0x10);           // OFF_H bit 4: full off
    } else {
        const uint16_t c = us_to_counts((float)s_us[ch]);
        Wire.write(0); Wire.write(0);
        Wire.write((uint8_t)(c & 0xFF));
        Wire.write((uint8_t)(c >> 8));
    }
    return Wire.endTransmission() == 0;
}

static bool pca_write_all(void)
{
    bool ok = true;
    for (uint8_t ch = 0; ch < N_CH; ch++) ok = pca_write_ch(ch) && ok;
    return ok;
}

static bool pca_begin(void)
{
    const uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * s_pwm_hz) + 0.5f) - 1;
    bool ok = i2c_write8(0x00, 0x10)               // MODE1: sleep, to set prescale
           && i2c_write8(0xFE, prescale)
           && i2c_write8(0x00, 0x00);              // wake
    delay(1);
    ok = ok && i2c_write8(0x00, 0xA0)              // auto-increment + restart
            && i2c_write8(0x01, 0x04)              // MODE2: totem-pole
            && i2c_read8(0xFE) == prescale;        // read back: it really took
    return ok && pca_write_all();
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
        Serial.printf("# PCA9685 ready, %.0f Hz\n", (double)s_pwm_hz);
    }
}

// =====================================================================
//  Navigation MCU link
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
        if (type == ACTCON_PKT_PYRO_REPORT &&
            ariadna_unpack(pl, len, &s_nav, sizeof(s_nav))) {
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
//  sensors, annunciators
// =====================================================================
/** ACS712, every millisecond: 100 ms mean, and the peak in 1 s buckets x10. */
static void current_service(void)
{
    static uint32_t next_us = 0, bucket_t0 = 0;
    static float    acc = 0.0f, filt = 0.0f;
    static uint16_t n = 0;
    static float    bucket[10];
    static uint8_t  bi = 0;
    const uint32_t  t = micros();
    if ((int32_t)(t - next_us) < 0) return;
    next_us = t + 1000;

    const float pin  = (float)analogRead(PIN_I_SENSE) * (ADC_VREF / ADC_FULL_SCALE);
    // Pin near 0 V = sensor has no 5 V (USB-only bench) or a broken wire:
    // a powered ACS712 never goes below ~0.1 V. Report 0 A, not -25 A.
    const float amps = (pin < ACS712_MIN_PIN_V) ? 0.0f
                     : (pin / ACS712_DIVIDER - ACS712_ZERO_V) / ACS712_V_PER_A;
    filt += 0.25f * (amps - filt);                 // ~4 ms: keeps a stall, drops ADC noise
    acc  += amps;
    n++;

    const uint32_t ms = millis();
    if (filt > bucket[bi]) bucket[bi] = filt;
    if (ms - bucket_t0 >= 1000) {                  // next 1 s bucket
        bucket_t0 = ms;
        bi = (uint8_t)((bi + 1) % 10);
        bucket[bi] = filt;
    }
    if (n >= TLM_PERIOD_MS) {
        s_amps_now = acc / (float)n;
        acc = 0.0f;
        n = 0;
    }
    float m = bucket[0];
    for (uint8_t i = 1; i < 10; i++) if (bucket[i] > m) m = bucket[i];
    s_amps_max = m;
}

/** Stop the pulse of every servo whose command has not changed for s_idle_ms. */
static void idle_service(void)
{
    if (!s_idle_ms || !s_pca_ok) return;
    const uint32_t now = millis();
    for (uint8_t ch = 0; ch < N_CH; ch++) {
        if (s_awake[ch] && s_us[ch] && now - s_moved_ms[ch] >= s_idle_ms) {
            s_awake[ch] = false;
            if (!pca_write_ch(ch)) { pca_lost("write failed"); return; }
        }
    }
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

static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    static uint32_t last = 0xFFFFFFFF;
    const uint32_t c = s_px.Color(r, g, b);
    if (c == last) return;
    last = c;
    s_px.setPixelColor(0, c);
    s_px.show();
}

static void annunciate(void)
{
    const uint32_t now = millis();
    const bool armed = s_nav_ok && s_nav.armed;

    // Buzzer: a chirp every second while the pyros are armed.
    static bool chirping = false;
    const bool chirp = armed && (now % 1000) < 80;
    if (chirp != chirping) {
        chirping = chirp;
        if (chirp) tone(PIN_BUZZER, 2700); else noTone(PIN_BUZZER);
    }

    if (s_nav_ok && s_nav.firing_mask)  rgb(255, 0, 0);                         // firing
    else if (armed)                      rgb((now / 150) % 2 ? 255 : 0, (now / 150) % 2 ? 110 : 0, 0);
    else if (!s_pca_ok)                  rgb((now / 300) % 2 ? 80 : 0, 0, (now / 300) % 2 ? 0 : 80);
    else if (!s_gui_alive)               rgb(0, 0, (now / 500) % 2 ? 40 : 0);   // waiting for GUI
    else                                 rgb(0, 35, 0);                         // GUI connected
}

// =====================================================================
//  telemetry
// =====================================================================
static void telemetry(void)
{
    char buf[480];
    uint16_t on = 0;
    for (uint8_t ch = 0; ch < N_CH; ch++)
        if (s_us[ch] && s_awake[ch]) on = (uint16_t)(on | (1u << ch));
    int n = snprintf(buf, sizeof(buf),
        "T {\"t\":%lu,\"pca\":%d,\"hz\":%.0f,\"rbf\":%d,\"vbat\":%.2f,"
        "\"i\":%.2f,\"imax\":%.2f,\"idle\":%lu,\"maxus\":%.0f,\"on\":%u,\"us\":[",
        (unsigned long)millis(), s_pca_ok ? 1 : 0, (double)s_pwm_hz,
        s_rbf_out ? 1 : 0, (double)s_vbat, (double)s_amps_now, (double)s_amps_max,
        (unsigned long)s_idle_ms, (double)max_pulse_us(), on);
    for (uint8_t ch = 0; ch < N_CH && n < (int)sizeof(buf); ch++)
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%s%u", ch ? "," : "", s_us[ch]);
    if (n < (int)sizeof(buf))
        n += snprintf(buf + n, sizeof(buf) - (size_t)n,
            "],\"nav\":%d,\"arm\":%d,\"arm_left\":%.1f,\"cont\":[%u,%u,%u],"
            "\"thr\":%u,\"cmask\":%u,\"fired\":%u,\"firing\":%u,\"rej\":%u}",
            s_nav_ok ? 1 : 0, s_nav.armed, (double)s_nav.arm_left_ds / 10.0,
            s_nav.raw[0], s_nav.raw[1], s_nav.raw[2], s_nav.threshold,
            s_nav.cont_mask, s_nav.fired_mask, s_nav.firing_mask, s_nav.last_reject);
    Serial.println(buf);
}

// =====================================================================
//  commands
// =====================================================================
static uint16_t clamp_us(uint8_t ch, long us)
{
    if (us < s_lim_lo[ch]) us = s_lim_lo[ch];
    if (us > s_lim_hi[ch]) us = s_lim_hi[ch];
    if (us > (long)max_pulse_us()) us = (long)max_pulse_us();
    return (uint16_t)us;
}

static bool parse_ch(const char *tok, uint8_t *ch)
{
    if (!tok) return false;
    char *end;
    const long v = strtol(tok, &end, 10);
    if (end == tok || v < 0 || v >= N_CH) return false;
    *ch = (uint8_t)v;
    return true;
}

static bool pyro_permitted(const char *what)
{
#if REQUIRE_RBF
    if (!s_rbf_out) { Serial.printf("ERR %s RBF pin is in\n", what); return false; }
#endif
    if (!s_nav_ok)  { Serial.printf("ERR %s Navigation MCU not answering\n", what); return false; }
    return true;
}

static void handle_line(char *line)
{
    char *save = nullptr;
    char *cmd = strtok_r(line, " \t", &save);
    if (!cmd) return;
    for (char *p = cmd; *p; p++) *p = (char)toupper((unsigned char)*p);
    char *a1 = strtok_r(nullptr, " \t", &save);
    char *a2 = strtok_r(nullptr, " \t", &save);
    char *a3 = strtok_r(nullptr, " \t", &save);
    uint8_t ch;

    if (!strcmp(cmd, "HB")) return;
    if (!strcmp(cmd, "HELLO")) { Serial.printf("I ACTCON 1 %u\n", N_CH); return; }
    if (!strcmp(cmd, "STATUS")) { telemetry(); return; }

    if (!strcmp(cmd, "SET")) {
        if (!parse_ch(a1, &ch) || !a2) { Serial.println("ERR SET <ch> <us>"); return; }
        s_us[ch]       = clamp_us(ch, strtol(a2, nullptr, 10));
        s_awake[ch]    = true;                     // any SET wakes an idled servo
        s_moved_ms[ch] = millis();
        if (s_pca_ok && !pca_write_ch(ch)) pca_lost("write failed");
        return;                                    // silent: the GUI streams these
    }
    if (!strcmp(cmd, "OFF")) {
        if (a1 && toupper((unsigned char)a1[0]) == 'A') {
            for (uint8_t i = 0; i < N_CH; i++) s_us[i] = 0;
            if (s_pca_ok && !pca_write_all()) pca_lost("write failed");
            Serial.println("OK OFF ALL");
            return;
        }
        if (!parse_ch(a1, &ch)) { Serial.println("ERR OFF <ch>|ALL"); return; }
        s_us[ch] = 0;
        if (s_pca_ok && !pca_write_ch(ch)) pca_lost("write failed");
        Serial.printf("OK OFF %u\n", ch);
        return;
    }
    if (!strcmp(cmd, "IDLE")) {
        const long ms = a1 ? strtol(a1, nullptr, 10) : -1;
        if (ms < 0 || ms > 60000) { Serial.println("ERR IDLE 0..60000 ms"); return; }
        s_idle_ms = (uint32_t)ms;
        Serial.printf("OK IDLE %ld\n", ms);
        return;
    }
    if (!strcmp(cmd, "LIM")) {
        if (!parse_ch(a1, &ch) || !a2 || !a3) { Serial.println("ERR LIM <ch> <min> <max>"); return; }
        long lo = strtol(a2, nullptr, 10), hi = strtol(a3, nullptr, 10);
        if (lo < (long)ABS_US_MIN) lo = (long)ABS_US_MIN;
        if (hi > (long)ABS_US_MAX) hi = (long)ABS_US_MAX;
        if (lo >= hi) { Serial.printf("ERR LIM %u min >= max\n", ch); return; }
        s_lim_lo[ch] = (uint16_t)lo;
        s_lim_hi[ch] = (uint16_t)hi;
        if (s_us[ch]) {                            // re-clamp a live pulse
            s_us[ch] = clamp_us(ch, s_us[ch]);
            if (s_pca_ok && !pca_write_ch(ch)) pca_lost("write failed");
        }
        Serial.printf("OK LIM %u %ld %ld\n", ch, lo, hi);
        return;
    }
    if (!strcmp(cmd, "PWM")) {
        const long hz = a1 ? strtol(a1, nullptr, 10) : 0;
        if (hz < 40 || hz > 400) { Serial.println("ERR PWM 40..400"); return; }
        s_pwm_hz = (float)hz;
        for (uint8_t i = 0; i < N_CH; i++)         // a shorter period may cut the window
            if (s_us[i]) s_us[i] = clamp_us(i, s_us[i]);
        if (s_pca_ok && !pca_begin()) pca_lost("prescale write failed");
        Serial.printf("OK PWM %ld\n", hz);
        return;
    }
    if (!strcmp(cmd, "ARM")) {
        if (!pyro_permitted("ARM")) return;
        nav_send(ACTCON_CMD_ARM, 0, 0, 3);
        Serial.println("OK ARM sent");
        return;
    }
    if (!strcmp(cmd, "DISARM")) {
        nav_send(ACTCON_CMD_DISARM, 0, 0, 3);      // always allowed
        Serial.println("OK DISARM sent");
        return;
    }
    if (!strcmp(cmd, "FIRE")) {
        const long pch = a1 ? strtol(a1, nullptr, 10) : 0;
        const long ms  = a2 ? strtol(a2, nullptr, 10) : (long)ACTCON_PULSE_DEFAULT_MS;
        if (pch < 1 || pch > ACTCON_N_PYRO) { Serial.println("ERR FIRE channel 1..3"); return; }
        if (!pyro_permitted("FIRE")) return;
        if (!s_nav.armed) { Serial.println("ERR FIRE not armed"); return; }
        const uint16_t pulse = actcon_clamp_pulse((uint16_t)(ms < 0 ? 0 : ms > 65535 ? 65535 : ms));
        nav_send(ACTCON_CMD_FIRE, (uint8_t)pch, pulse, 3);
        Serial.printf("OK FIRE %ld %u ms\n", pch, pulse);
        return;
    }
    Serial.printf("ERR unknown '%s'\n", cmd);
}

static void console_service(void)
{
    static char buf[64];
    static uint8_t len = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            buf[len] = 0;
            if (len) {
                s_gui_last = millis();
                if (!s_gui_alive) { s_gui_alive = true; Serial.println("# GUI connected"); }
                handle_line(buf);
            }
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
    if (s_gui_alive && millis() - s_gui_last > GUI_TIMEOUT_MS) {
        s_gui_alive = false;
        nav_send(ACTCON_CMD_DISARM, 0, 0, 3);
        Serial.println("# GUI silent - pyros disarmed, servos hold");
    }
}

// =====================================================================
void setup()
{
    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH);                // outputs off until configured
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RBF, INPUT_PULLUP);
    analogReadResolution(12);

    for (uint8_t ch = 0; ch < N_CH; ch++) {
        s_us[ch] = 0;
        s_awake[ch] = false;
        s_lim_lo[ch] = (uint16_t)DEF_US_MIN;
        s_lim_hi[ch] = (uint16_t)DEF_US_MAX;
    }

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

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setTimeout(10, true);
    Wire.begin();
    Wire.setClock(400000);
    pca_try_start();
}

void loop()
{
    static uint32_t next_tlm = 0, next_hb = 0, next_pca = 0;
    const uint32_t now = millis();

    console_service();
    nav_service();
    sense_service();
    current_service();
    idle_service();

    if (s_gui_alive && (int32_t)(now - next_hb) >= 0) {
        next_hb = now + ACTCON_HB_PERIOD_MS;
        nav_send(ACTCON_CMD_HEARTBEAT, 0, 0, 1);
    }

    if ((int32_t)(now - next_pca) >= 0) {
        next_pca = now + PCA_CHECK_MS;
        if (!s_pca_ok) pca_try_start();
        else {
            // A PCA9685 that lost power comes back asleep (MODE1 bit 4) with
            // every channel off: re-initialise it and rewrite the pulses.
            const int m = i2c_read8(0x00);
            if (m < 0 || (m & 0x10)) {
                pca_lost(m < 0 ? "no ACK" : "it reset (brown-out?)");
                pca_try_start();
            }
        }
    }

    if ((int32_t)(now - next_tlm) >= 0) {
        next_tlm = now + TLM_PERIOD_MS;
        telemetry();
    }
    annunciate();
}
