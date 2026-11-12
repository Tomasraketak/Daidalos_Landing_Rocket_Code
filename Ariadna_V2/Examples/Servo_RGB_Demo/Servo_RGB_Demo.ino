// =====================================================================
//  Ariadna V2 — Servo + RGB demo for the MASTER MCU
//
//  Target: Raspberry Pi Pico 2 W (RP2350), arduino-pico core.
//  Needs:  Adafruit_NeoPixel. The PCA9685 is driven by its registers,
//          no servo library.
//
//  Servos 1-8 = PCA9685 channels 0-7 (TVC X, TVC Y, clamp, fins 0-3,
//  spare). All eight always move together.
//
//  USB serial, 115200 baud, line ending "Newline" (or "Both NL & CR"):
//
//    90          all servos to 90 deg and hold there
//    30 120      oscillate between 30 and 120 deg (also "30-120", "30,120"):
//                  fast to 120, wait 5 s, fast back to 30, wait 3 s,
//                  slowly (5 deg/s) to 120, wait 3 s, slowly back to 30,
//                  wait 5 s, and again from the top until stopped
//    s           stop: servos hold where they are right now
//    c           centre (90 deg)
//    h           help
//
//  The angle is the usual hobby-servo scale: 90 deg = 1500 us = neutral,
//  10 us per degree. Every command is limited to ANGLE_MIN..ANGLE_MAX
//  (30..150 deg = 900..2100 us, the same travel guard as the flight code).
//
//  !! TVC and clamp linkages have far less travel than a bare servo.
//  !! Unhook the linkages, or narrow ANGLE_MIN / ANGLE_MAX, before you
//  !! send anything far from 90.
//
//  RGB LED (SK6812 on GP2) follows what the servos are doing:
//
//    hold                 colour = angle: blue (min) -> green (90) -> red (max)
//    fast move            red, fast flashing
//    slow move            angle colour (blue -> green -> red) that drifts
//                         with the servo as it travels, at full brightness
//    waiting              green "breathing"; in the last second before the
//                         next move it blinks yellow
//    stopped by 's'       dim white
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

// ---- pins: identical to Master_MCU/Master_MCU.ino ----------------------
static const uint8_t PIN_RGB     = 2;    // SK6812
static const uint8_t PIN_PCA_OE  = 6;    // PCA9685 /OE: LOW = outputs on
static const uint8_t PIN_I2C_SDA = 8;
static const uint8_t PIN_I2C_SCL = 9;

// ---- PCA9685 -------------------------------------------------------------
static const uint8_t PCA_ADDR   = 0x40;
static const float   PCA_OSC_HZ = 25000000.0f;
// 50 Hz works with every servo. The flight code runs 339 Hz, which only
// DIGITAL servos accept; set 339 here if that is all you have plugged in.
static const float   PCA_PWM_HZ = 50.0f;
static const uint8_t N_SERVOS   = 8;

// ---- motion --------------------------------------------------------------
static const float ANGLE_MIN       = 30.0f;    // deg, = 900 us
static const float ANGLE_MAX       = 150.0f;   // deg, = 2100 us
static const float ANGLE_CENTER    = 90.0f;    // deg, = 1500 us
static const float US_AT_CENTER    = 1500.0f;
static const float US_PER_DEG      = 10.0f;

static const float SLOW_DEG_PER_S  = 5.0f;     // the slow leg
// A "fast" move is a single jump of the command; the servo then travels at
// its own top speed. This is only the estimate of that speed, used to
// decide when the servo has arrived and the wait starts (~0.15 s / 60 deg).
static const float FAST_DEG_PER_S  = 400.0f;

static const uint32_t WAIT_FAST_MS = 5000;     // after each fast leg
static const uint32_t WAIT_SLOW_MS = 3000;     // after each slow leg
static const uint32_t SERVO_UPDATE_MS = 20;    // 50 Hz command updates

// =====================================================================
//  state
// =====================================================================
enum Mode  { MODE_IDLE, MODE_HOLD, MODE_OSC, MODE_STOPPED };
enum Phase { PH_MOVE_FAST, PH_MOVE_SLOW, PH_WAIT };

static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static bool  s_pca_ok = false;

static Mode  s_mode  = MODE_IDLE;
static float s_cmd   = ANGLE_CENTER;   // what the servos are commanded to
static float s_est   = ANGLE_CENTER;   // where we think they are

// oscillation
static float    s_osc_a = 0, s_osc_b = 0;
static uint8_t  s_step  = 0;           // index into STEPS[]
static Phase    s_phase = PH_WAIT;
static float    s_leg_from = 0, s_leg_to = 0;
static uint32_t s_leg_t0 = 0, s_leg_ms = 0;
static uint32_t s_wait_until = 0, s_wait_ms = 0;
static bool     s_first_leg = false;   // the move to A before the first cycle

// One cycle: to B fast, wait 5 s, to A fast, wait 3 s,
//            to B slow, wait 3 s, to A slow, wait 5 s.
struct Step { bool to_b; bool slow; uint32_t wait_ms; };
static const Step STEPS[] = {
    { true,  false, WAIT_FAST_MS },
    { false, false, WAIT_SLOW_MS },
    { true,  true,  WAIT_SLOW_MS },
    { false, true,  WAIT_FAST_MS },
};
static const uint8_t N_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);

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

static bool pca_begin(void)
{
    const uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * PCA_PWM_HZ) + 0.5f) - 1;
    bool ok = i2c_write8(0x00, 0x10)               // MODE1: sleep (needed to set prescale)
           && i2c_write8(0xFE, prescale)
           && i2c_write8(0x00, 0x00);              // wake
    delay(1);
    ok = ok && i2c_write8(0x00, 0xA0)              // auto-increment + restart
            && i2c_write8(0x01, 0x04);             // MODE2: totem-pole outputs
    return ok;
}

static uint16_t angle_to_counts(float deg)
{
    const float us = US_AT_CENTER + (deg - ANGLE_CENTER) * US_PER_DEG;
    return (uint16_t)(us * (4096.0f * PCA_PWM_HZ * 1e-6f) + 0.5f);
}

/** All eight channels in one I2C transaction. */
static bool servos_write(float deg)
{
    const uint16_t c = angle_to_counts(deg);
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(0x06);                              // LED0_ON_L
    for (uint8_t i = 0; i < N_SERVOS; i++) {
        Wire.write(0); Wire.write(0);              // ON at 0
        Wire.write((uint8_t)(c & 0xFF));           // OFF
        Wire.write((uint8_t)(c >> 8));
    }
    return Wire.endTransmission() == 0;
}

// =====================================================================
//  RGB
// =====================================================================
static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    static uint32_t last = 0xFFFFFFFF;
    const uint32_t c = s_px.Color(r, g, b);
    if (c == last) return;                         // don't re-send the same colour
    last = c;
    s_px.setPixelColor(0, c);
    s_px.show();
}

/** Angle -> colour: blue at ANGLE_MIN, green at 90, red at ANGLE_MAX. */
static void angle_colour(float deg, float bright, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float t = (deg - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float fr, fg, fb;
    if (t < 0.5f) { fr = 0;              fg = t * 2.0f;          fb = 1.0f - t * 2.0f; }
    else          { fr = (t - 0.5f) * 2; fg = 1.0f - (t - 0.5f) * 2; fb = 0; }
    *r = (uint8_t)(fr * 255.0f * bright);
    *g = (uint8_t)(fg * 255.0f * bright);
    *b = (uint8_t)(fb * 255.0f * bright);
}

static void rgb_service(void)
{
    const uint32_t now = millis();
    uint8_t r, g, b;

    switch (s_mode) {
    case MODE_IDLE:
        rgb(0, 0, (now / 500) % 2 ? 40 : 0);       // slow blue blink: waiting for a command
        return;
    case MODE_STOPPED:
        rgb(25, 25, 25);
        return;
    case MODE_HOLD:
        angle_colour(s_est, 0.35f, &r, &g, &b);
        rgb(r, g, b);
        return;
    case MODE_OSC:
        break;
    }

    if (s_phase == PH_MOVE_FAST) {
        rgb((now / 60) % 2 ? 255 : 30, 0, 0);
    } else if (s_phase == PH_MOVE_SLOW) {
        angle_colour(s_est, 1.0f, &r, &g, &b);
        rgb(r, g, b);
    } else {                                       // waiting
        const uint32_t left = s_wait_until > now ? s_wait_until - now : 0;
        if (left < 1000) {
            if ((now / 125) % 2) rgb(180, 120, 0); else rgb(0, 0, 0);
        } else {
            const float k = 0.5f - 0.5f * cosf((float)now * (2.0f * (float)M_PI / 2000.0f));
            rgb(0, (uint8_t)(10 + 110 * k), 0);
        }
    }
}

// =====================================================================
//  motion
// =====================================================================
static float clamp_angle(float deg)
{
    if (deg < ANGLE_MIN) return ANGLE_MIN;
    if (deg > ANGLE_MAX) return ANGLE_MAX;
    return deg;
}

static void leg_start(float to, bool slow)
{
    s_leg_from = s_est;
    s_leg_to   = to;
    s_leg_t0   = millis();
    const float dist = fabsf(to - s_leg_from);
    s_leg_ms   = (uint32_t)(1000.0f * dist / (slow ? SLOW_DEG_PER_S : FAST_DEG_PER_S));
    s_phase    = slow ? PH_MOVE_SLOW : PH_MOVE_FAST;
    if (!slow) s_cmd = to;                         // fast: jump the command, the servo runs flat out
    Serial.printf("[%7.1f s] %s %5.1f -> %5.1f deg (%.1f s)\n", millis() / 1000.0,
                  slow ? "slow" : "FAST", (double)s_leg_from, (double)to, s_leg_ms / 1000.0);
}

static void step_begin(void)
{
    const Step &st = STEPS[s_step];
    leg_start(st.to_b ? s_osc_b : s_osc_a, st.slow);
}

static void osc_start(float a, float b)
{
    s_osc_a = clamp_angle(a);
    s_osc_b = clamp_angle(b);
    s_mode  = MODE_OSC;
    // Get to A first (fast), pause 1 s, then the cycle starts from STEPS[0].
    s_step  = N_STEPS - 1;
    s_first_leg = true;
    leg_start(s_osc_a, false);
    Serial.printf("oscillating %.1f <-> %.1f deg, 's' stops\n", (double)s_osc_a, (double)s_osc_b);
}

static void osc_service(void)
{
    const uint32_t now = millis();
    if (s_phase == PH_MOVE_FAST || s_phase == PH_MOVE_SLOW) {
        const uint32_t dt = now - s_leg_t0;
        if (dt >= s_leg_ms) {
            s_est = s_cmd = s_leg_to;
            s_wait_ms    = s_first_leg ? 1000 : STEPS[s_step].wait_ms;
            s_first_leg  = false;
            s_wait_until = now + s_wait_ms;
            s_phase      = PH_WAIT;
        } else {
            s_est = s_leg_from + (s_leg_to - s_leg_from) * ((float)dt / (float)s_leg_ms);
            if (s_phase == PH_MOVE_SLOW) s_cmd = s_est;   // ramp the command
        }
    } else if ((int32_t)(now - s_wait_until) >= 0) {
        s_step = (uint8_t)((s_step + 1) % N_STEPS);
        step_begin();
    }
}

// =====================================================================
//  console
// =====================================================================
static void help(void)
{
    Serial.println(F("\nServo + RGB demo - servos 1-8 on PCA9685 channels 0-7"));
    Serial.printf("  <deg>          all servos to <deg> and hold (%.0f..%.0f)\n",
                  (double)ANGLE_MIN, (double)ANGLE_MAX);
    Serial.println(F("  <a> <b>        oscillate a <-> b: fast, 5 s, fast back, 3 s,"));
    Serial.println(F("                 slow 5 deg/s, 3 s, slow back, 5 s, repeat"));
    Serial.println(F("  s  stop       c  centre       h  help\n"));
}

static void handle_line(char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (!*line) return;

    const char c0 = (char)tolower((unsigned char)line[0]);
    if (c0 == 'h' || c0 == '?') { help(); return; }
    if (c0 == 's') {
        s_cmd = s_est;                             // freeze where it is now
        s_mode = MODE_STOPPED;
        Serial.printf("stopped at %.1f deg\n", (double)s_est);
        return;
    }
    if (c0 == 'c') { line = (char *)"90"; }

    // up to two numbers, separated by space, comma, semicolon or '-'
    float v[2];
    int n = 0;
    char *p = line;
    while (*p && n < 2) {
        while (*p && !isdigit((unsigned char)*p) && *p != '.') p++;
        if (!*p) break;
        char *end;
        v[n++] = strtof(p, &end);
        if (end == p) break;
        p = end;
    }
    if (n == 0) { Serial.printf("? '%s' - type h for help\n", line); return; }

    for (int i = 0; i < n; i++) {
        if (v[i] < ANGLE_MIN || v[i] > ANGLE_MAX)
            Serial.printf("  %.1f deg is outside %.0f..%.0f, limited\n",
                          (double)v[i], (double)ANGLE_MIN, (double)ANGLE_MAX);
    }

    if (n == 1) {
        s_cmd = s_est = clamp_angle(v[0]);
        s_mode = MODE_HOLD;
        Serial.printf("servos 1-8 -> %.1f deg (%.0f us)\n", (double)s_cmd,
                      (double)(US_AT_CENTER + (s_cmd - ANGLE_CENTER) * US_PER_DEG));
    } else {
        osc_start(v[0], v[1]);
    }
}

static void console_service(void)
{
    static char buf[48];
    static uint8_t len = 0;
    while (Serial.available()) {
        const char ch = (char)Serial.read();
        if (ch == '\r' || ch == '\n') {
            buf[len] = 0;
            if (len) handle_line(buf);
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch;
        }
    }
}

// =====================================================================
void setup()
{
    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH);                // outputs off until a pulse is set

    Serial.begin(115200);                          // no while(!Serial): runs without a PC

    s_px.begin();
    s_px.clear();
    s_px.show();

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setTimeout(10, true);
    Wire.begin();
    Wire.setClock(400000);

    s_pca_ok = pca_begin() && servos_write(s_cmd);
    if (s_pca_ok) digitalWrite(PIN_PCA_OE, LOW);   // servos on, at 90 deg

    delay(1500);                                   // time to open the serial monitor
    help();
    if (!s_pca_ok) Serial.println(F("!! PCA9685 not found at 0x40 - check servo board power / I2C"));
}

void loop()
{
    static uint32_t next_servo = 0, next_retry = 0;
    const uint32_t now = millis();

    console_service();
    if (s_mode == MODE_OSC) osc_service();

    if (!s_pca_ok) {
        if ((int32_t)(now - next_retry) >= 0) {
            next_retry = now + 3000;
            s_pca_ok = pca_begin() && servos_write(s_cmd);
            if (s_pca_ok) {
                digitalWrite(PIN_PCA_OE, LOW);
                Serial.println(F("PCA9685 found"));
            }
        }
        // red/blue alternating: no servo driver
        rgb((now / 300) % 2 ? 80 : 0, 0, (now / 300) % 2 ? 0 : 80);
        return;
    }

    if ((int32_t)(now - next_servo) >= 0) {
        next_servo = now + SERVO_UPDATE_MS;
        if (!servos_write(s_cmd)) {
            s_pca_ok = false;
            digitalWrite(PIN_PCA_OE, HIGH);
            Serial.println(F("!! PCA9685 stopped answering"));
        }
    }
    rgb_service();
}
