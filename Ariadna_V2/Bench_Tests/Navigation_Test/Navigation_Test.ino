// =====================================================================
//  Ariadna V2 — NAVIGATION MCU bench test
//
//  Target: Seeed XIAO RP2350, arduino-pico core.
//  Needs:  Adafruit_NeoPixel
//
//    CONT_1..3   pyro continuity ADCs: raw counts, igniter present / open,
//                and a floating input (noisy reading) is flagged
//    PYRO_OUT    outputs are held LOW. The FIRING test is OFF unless
//                PYRO_OUTPUT_TEST is set to 1 below AND 'y' is typed on
//                the console — then each channel is pulsed 200 ms in turn
//                and the pin is read back. LEDs ONLY, NEVER AN IGNITER.
//    I2C_D4D5    scan of the broken-out (unused) I2C pins, for information
//    RGB LED, core 1, link to the Master, link to Position (via Master)
//
//  NOTHING BLOCKS. USB serial, 115200 baud.
//  Commands: h help, p summary, i I2C scan, y run the pyro LED test.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "bench_common.h"

// =====================================================================
//  1. CONFIGURATION — pins identical to Navigation_MCU/Navigation_MCU.ino
// =====================================================================

// !!! 1 = the 'y' command drives the pyro outputs HIGH. Bench it with LEDs
// !!! on PIN_PYRO_*, never with an igniter or a motor connected.
#define PYRO_OUTPUT_TEST  0
static const uint32_t PYRO_TEST_PULSE_MS = 200;

static const uint8_t PIN_CONT_1    = D0;   // A0
static const uint8_t PIN_CONT_2    = D1;   // A1
static const uint8_t PIN_CONT_3    = D2;   // A2
static const uint8_t PIN_PYRO_1    = D3;   // landing motor igniter
static const uint8_t PIN_I2C_SDA   = D4;   // broken out, unused in flight
static const uint8_t PIN_I2C_SCL   = D5;
static const uint8_t PIN_MASTER_TX = D6;
static const uint8_t PIN_MASTER_RX = D7;
static const uint8_t PIN_PYRO_2    = D8;   // backup igniter
static const uint8_t PIN_PYRO_3    = D9;   // reserve
static const uint8_t PIN_RGB       = 22;   // XIAO RP2350 on-board WS2812
static const uint8_t PIN_RGB_POWER = 23;

// Same assumptions as the flight sketch: present = reading >= threshold.
#define CONT_ACTIVE_HIGH  1
static const uint16_t CONT_THRESHOLD = 1200;
static const uint16_t CONT_NOISE_MAX = 200;   // peak-to-peak beyond this = floating

static const uint8_t PYRO_PIN[3] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };
static const uint8_t CONT_PIN[3] = { PIN_CONT_1, PIN_CONT_2, PIN_CONT_3 };

static TwoWire &I2C = (((PIN_I2C_SDA >> 1) & 1) ? Wire1 : Wire);

// =====================================================================
//  2. STATE
// =====================================================================
static Adafruit_NeoPixel s_px(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static bench_port_t      s_master;
static bench_looptime_t  s_loop;

static uint16_t s_cont_raw[3], s_cont_min[3], s_cont_max[3];

// =====================================================================
//  3. PYRO CONTINUITY
// =====================================================================
static void cont_service(void)
{
    static uint32_t next_ms = 0, window_t0 = 0;
    static int8_t last_present[3] = { -1, -1, -1 };
    if (millis() < next_ms) return;
    next_ms = millis() + 100;

    if (millis() - window_t0 > 2000) {                    // 2 s noise window
        window_t0 = millis();
        for (uint8_t i = 0; i < 3; i++) { s_cont_min[i] = 0xFFFF; s_cont_max[i] = 0; }
    }
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t acc = 0;
        for (uint8_t k = 0; k < 8; k++) acc += (uint32_t)analogRead(CONT_PIN[i]);
        const uint16_t v = (uint16_t)(acc / 8);
        s_cont_raw[i] = v;
        if (v < s_cont_min[i]) s_cont_min[i] = v;
        if (v > s_cont_max[i]) s_cont_max[i] = v;
#if CONT_ACTIVE_HIGH
        const bool present = v >= CONT_THRESHOLD;
#else
        const bool present = v < CONT_THRESHOLD;
#endif
        if (last_present[i] >= 0 && last_present[i] != (int8_t)present)
            bench_printf("[%8.2f s] EVENT CONT_%u %s (raw %u)\n", (double)bench_t_s(), i + 1,
                         present ? "igniter CONNECTED" : "OPEN", v);
        last_present[i] = (int8_t)present;

        const uint16_t pp = s_cont_max[i] >= s_cont_min[i] ? (uint16_t)(s_cont_max[i] - s_cont_min[i]) : 0;
        if (pp > CONT_NOISE_MAX)
            bench_set(NT_CONT_1 + i, ST_WARN, "raw %u, jumps %u counts - input floating? divider?", v, pp);
        else if (present)
            bench_set(NT_CONT_1 + i, ST_PASS, "igniter present, raw %u (threshold %u)", v, CONT_THRESHOLD);
        else
            bench_set(NT_CONT_1 + i, ST_INFO, "open, raw %u (threshold %u) - fit a test igniter/resistor",
                      v, CONT_THRESHOLD);
    }
}

// =====================================================================
//  4. PYRO OUTPUTS
// =====================================================================
static int8_t   s_fire_ch = -1;      // channel currently in the test, -1 = idle
static bool     s_fire_on = false;
static uint32_t s_fire_t0 = 0;
static uint8_t  s_fire_ok = 0, s_fire_bad = 0;

static void pyro_all_low(void)
{
    for (uint8_t i = 0; i < 3; i++) digitalWrite(PYRO_PIN[i], LOW);
}

static void pyro_test_start(void)
{
#if PYRO_OUTPUT_TEST
    bench_printf("# !!! PYRO LED TEST: CH1, CH2, CH3 go HIGH for %lu ms each !!!\n",
                 (unsigned long)PYRO_TEST_PULSE_MS);
    s_fire_ch = 0; s_fire_on = false; s_fire_ok = s_fire_bad = 0;
    s_fire_t0 = millis();
#else
    bench_printf("# pyro output test disabled: set PYRO_OUTPUT_TEST 1 (LEDs only!)\n");
#endif
}

static void pyro_service(void)
{
    if (s_fire_ch < 0) {
        // Idle: every output must read back LOW.
        uint8_t high = 0;
        for (uint8_t i = 0; i < 3; i++) if (digitalRead(PYRO_PIN[i])) high |= (uint8_t)(1u << i);
        if (high) {
            pyro_all_low();
            bench_set(NT_PYRO_OUT, ST_FAIL, "output(s) 0x%X read HIGH at idle - forced LOW", high);
        } else if (g_test[NT_PYRO_OUT].st != ST_PASS && g_test[NT_PYRO_OUT].st != ST_WARN) {
#if PYRO_OUTPUT_TEST
            bench_set(NT_PYRO_OUT, ST_INFO, "all LOW (safe); type 'y' to pulse them");
#else
            bench_set(NT_PYRO_OUT, ST_SKIP, "all LOW (safe); firing test disabled");
#endif
        }
        return;
    }
    const uint8_t ch = (uint8_t)s_fire_ch;
    if (!s_fire_on) {
        if (millis() - s_fire_t0 < 800) return;           // gap between channels
        digitalWrite(PYRO_PIN[ch], HIGH);
        s_fire_on = true;
        s_fire_t0 = millis();
        bench_printf("# PYRO CH%u HIGH\n", ch + 1);
        return;
    }
    if (millis() - s_fire_t0 < PYRO_TEST_PULSE_MS) return;
    const bool readback = digitalRead(PYRO_PIN[ch]) == HIGH;
    digitalWrite(PYRO_PIN[ch], LOW);
    bench_printf("# PYRO CH%u LOW, pin read back %s\n", ch + 1, readback ? "HIGH (ok)" : "LOW (!)");
    if (readback) s_fire_ok++; else s_fire_bad++;
    s_fire_on = false;
    s_fire_t0 = millis();
    if (++s_fire_ch >= 3) {
        s_fire_ch = -1;
        pyro_all_low();
        bench_set(NT_PYRO_OUT, s_fire_bad ? ST_FAIL : ST_PASS,
                  "%u/3 channels drove HIGH and returned LOW", s_fire_ok);
    }
}

// =====================================================================
//  5. LINK
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
        bench_ping_build(&pk, NODE_MASTER);   bench_send(&s_master, PKT_PING, &pk, sizeof(pk));
        bench_ping_build(&pk, NODE_POSITION); bench_send(&s_master, PKT_PING, &pk, sizeof(pk));
    }
    if (millis() >= next_report) {
        next_report = millis() + 1000;
        bench_report_t r;
        bench_report_build(&r, NODE_MASTER, s_loop.max_ever_us);
        bench_send(&s_master, PKT_REPORT, &r, sizeof(r));
    }
}

// =====================================================================
//  6. I2C (informational), RGB, SUMMARY, CONSOLE
// =====================================================================
static void i2c_scan(void)
{
    uint8_t found = 0;
    char list[48];
    int n = 0;
    list[0] = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        I2C.beginTransmission(a);
        if (I2C.endTransmission() == 0) {
            found++;
            if (n < (int)sizeof(list) - 6) n += snprintf(list + n, sizeof(list) - (size_t)n, "0x%02X ", a);
        }
    }
    if (found) bench_set(NT_I2C_D4D5, ST_INFO, "%u device(s): %s", found, list);
    else       bench_set(NT_I2C_D4D5, ST_INFO, "nothing connected (unused in flight)");
    bench_printf("# I2C D4/D5: %s\n", g_test[NT_I2C_D4D5].detail);
}

static void rgb_service(void)
{
    static uint32_t last = 0xFFFFFFFF;
    const uint32_t t = millis();
    uint8_t r = 0, g = 0, b = 0;
    if (s_fire_ch >= 0) {
        r = (t % 100) < 50 ? 80 : 0;                         // pyro test in progress
    } else if (t < 2400) {
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
    bench_printf("  continuity raw: %u / %u / %u (threshold %u)\n",
                 s_cont_raw[0], s_cont_raw[1], s_cont_raw[2], CONT_THRESHOLD);
    bench_printf("  loop max %lu us (worst %lu), die %.1f C, CPU %lu MHz\n",
                 (unsigned long)s_loop.max_us, (unsigned long)s_loop.max_ever_us,
                 (double)analogReadTemp(), (unsigned long)(rp2040.f_cpu() / 1000000));
    bench_print_port(&s_master);
    bench_print_peer(NODE_MASTER);
    bench_print_peer(NODE_POSITION);
    bench_printf("\n");
    bench_window_reset();
    s_loop.max_us = 0;
}

static void console_service(void)
{
    while (Serial.available()) {
        switch (Serial.read()) {
        case 'h': case '?': bench_printf("# h help | p summary | i I2C scan | y pyro LED test\n"); break;
        case 'p': print_summary(); break;
        case 'i': i2c_scan(); break;
        case 'y': if (s_fire_ch < 0) pyro_test_start(); break;
        default: break;
        }
    }
}

// =====================================================================
//  7. SETUP / LOOP
// =====================================================================
void setup()
{
    // Pyro outputs LOW before anything else, exactly as in flight.
    for (uint8_t i = 0; i < 3; i++) { pinMode(PYRO_PIN[i], OUTPUT); digitalWrite(PYRO_PIN[i], LOW); }
    for (uint8_t i = 0; i < 3; i++) pinMode(CONT_PIN[i], INPUT);
    analogReadResolution(12);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_RGB_POWER, OUTPUT); digitalWrite(PIN_RGB_POWER, HIGH);

    Serial.begin(BENCH_CONSOLE_BAUD);
    bench_tests_init(NODE_NAVIGATION);
    s_px.begin(); s_px.clear(); s_px.show();

    I2C.setSDA(PIN_I2C_SDA);
    I2C.setSCL(PIN_I2C_SCL);
    I2C.setTimeout(10, true);
    I2C.begin();
    i2c_scan();

    bench_port_begin(&s_master, Serial1, uart0, PIN_MASTER_TX, PIN_MASTER_RX, "UART0>MASTER");

    bench_printf("\n=== Ariadna V2 NAVIGATION bench test, proto v%d, pyro firing test %s ===\n",
                 BENCH_PROTO_VERSION, PYRO_OUTPUT_TEST ? "ENABLED ('y')" : "disabled");
}

void loop()
{
    static uint32_t next_summary = 3000, next_verdict = 0;
    bench_loop_begin(&s_loop);

    bench_port_poll(&s_master, on_rx);
    bench_port_service_tx(&s_master);
    link_service();

    pyro_service();
    cont_service();
    rgb_service();
    console_service();

    if (millis() >= next_verdict) {
        next_verdict = millis() + 250;
        bench_link_verdict(NT_LINK_MASTER, NODE_MASTER, &s_master);
        bench_link_verdict(NT_LINK_POS, NODE_POSITION, &s_master);
        bench_core1_verdict(NT_CORE1);
    }
    if (millis() >= next_summary) {
        next_summary = millis() + 5000;
        print_summary();
    }
    bench_loop_end(&s_loop);
}

void setup1() {}
void loop1()  { bench_core1_tick(); }
