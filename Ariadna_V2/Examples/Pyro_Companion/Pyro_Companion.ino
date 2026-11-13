// =====================================================================
//  Ariadna V2 — Pyro Companion for the NAVIGATION MCU
//
//  Target: Seeed XIAO RP2350, arduino-pico core.
//  Needs:  Adafruit_NeoPixel.
//  Pair:   Examples/Actuator_Console on the Master MCU.
//
//  Reads the three continuity inputs and reports them to the Master 20x a
//  second; fires a pyro channel when the Master asks AND this board is
//  armed. Every safety rule lives in actcon_pyro.h, which is tested on a
//  workstation (`cd ../../test && make actcon`):
//
//    * outputs LOW at power-up and whenever not firing
//    * FIRE only when ARMED; ARMED expires after 60 s
//    * no heartbeat from the Master for 0.5 s -> disarmed, pulse cut
//    * one channel at a time, pulse 20..1000 ms, every pulse ends disarmed
//
//  !! Bench first with LEDs or a resistor on the pyro outputs. With a real
//  !! igniter or motor connected, follow the static-fire procedure: people
//  !! clear, motor restrained, the person at the laptop has line of sight.
//
//  RGB (on-board WS2812):
//    blue blink        no Master heartbeat
//    green, dim        linked, disarmed
//    amber flashing    ARMED
//    red               firing
//
//  USB console (115200): events only, plus 'p' for a status line.
// =====================================================================

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

#include "ariadna_link.h"
#include "actcon_pyro.h"

// ---- pins: identical to Navigation_MCU/Navigation_MCU.ino ---------------
static const uint8_t PIN_CONT_1    = D0;   // A0
static const uint8_t PIN_CONT_2    = D1;   // A1
static const uint8_t PIN_CONT_3    = D2;   // A2
static const uint8_t PIN_PYRO_1    = D3;   // landing motor igniter
static const uint8_t PIN_PYRO_2    = D8;   // backup igniter
static const uint8_t PIN_PYRO_3    = D9;   // reserve
static const uint8_t PIN_MASTER_TX = D6;
static const uint8_t PIN_MASTER_RX = D7;

#define MASTER_PORT  Serial1               // UART0 on arduino-pico

// Continuity: polarity and threshold are properties of YOUR harness.
// Measure the counts with a known-good igniter and with an open channel
// (the Navigation bench test prints both) and put the midpoint here.
#define CONT_ACTIVE_HIGH  1
static const uint16_t CONT_THRESHOLD = 1200;

#ifndef PIN_NEOPIXEL
#  define PIN_NEOPIXEL 22
#endif

static const uint8_t PYRO_PIN[ACTCON_N_PYRO] = { PIN_PYRO_1, PIN_PYRO_2, PIN_PYRO_3 };
static const uint8_t CONT_PIN[ACTCON_N_PYRO] = { PIN_CONT_1, PIN_CONT_2, PIN_CONT_3 };

static Adafruit_NeoPixel s_px(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static actcon_pyro_sm_t  s_sm;
static ariadna_rx_t      s_rx;
static uint16_t s_raw[ACTCON_N_PYRO];
static uint8_t  s_cont_mask = 0;
static uint8_t  s_out_mask  = 0;

// =====================================================================
static void outputs_write(uint8_t mask)
{
    for (uint8_t i = 0; i < ACTCON_N_PYRO; i++)
        digitalWrite(PYRO_PIN[i], (mask & (1u << i)) ? HIGH : LOW);
}

static void continuity_read(void)
{
    uint8_t m = 0;
    for (uint8_t i = 0; i < ACTCON_N_PYRO; i++) {
        uint32_t acc = 0;
        for (uint8_t k = 0; k < 8; k++) acc += (uint32_t)analogRead(CONT_PIN[i]);
        s_raw[i] = (uint16_t)(acc / 8);
#if CONT_ACTIVE_HIGH
        if (s_raw[i] >= CONT_THRESHOLD) m |= (uint8_t)(1u << i);
#else
        if (s_raw[i] <  CONT_THRESHOLD) m |= (uint8_t)(1u << i);
#endif
    }
    if (m != s_cont_mask) {
        for (uint8_t i = 0; i < ACTCON_N_PYRO; i++)
            if ((m ^ s_cont_mask) & (1u << i))
                Serial.printf("[%8.2f s] CH%u continuity %s (raw %u)\n", millis() / 1000.0,
                              i + 1, (m & (1u << i)) ? "PRESENT" : "open", s_raw[i]);
        s_cont_mask = m;
    }
}

static void report_send(uint32_t now)
{
    actcon_pyro_report_t r;
    memset(&r, 0, sizeof(r));
    r.t_ms        = now;
    for (uint8_t i = 0; i < ACTCON_N_PYRO; i++) r.raw[i] = s_raw[i];
    r.threshold   = CONT_THRESHOLD;
    r.arm_left_ds = actcon_arm_left_ds(&s_sm, now);
    r.cont_mask   = s_cont_mask;
    r.fired_mask  = s_sm.fired_mask;
    r.firing_mask = s_out_mask;
    r.armed       = s_sm.armed;
    r.last_reject = s_sm.last_reject;
    r.active_high = CONT_ACTIVE_HIGH;
    uint8_t frame[ARIADNA_FRAME_MAX];
    const uint16_t n = ariadna_encode(frame, ACTCON_PKT_PYRO_REPORT, &r, sizeof(r));
    if (MASTER_PORT.availableForWrite() >= (int)n) MASTER_PORT.write(frame, n);
}

static void link_service(uint32_t now)
{
    while (MASTER_PORT.available()) {
        const uint8_t *pl; uint8_t len;
        const uint8_t type = ariadna_rx_byte(&s_rx, (uint8_t)MASTER_PORT.read(), &pl, &len);
        actcon_pyro_cmd_t c;
        if (type != ACTCON_PKT_PYRO_CMD || !ariadna_unpack(pl, len, &c, sizeof(c))) continue;
        const uint8_t was_armed = s_sm.armed;
        const bool repeat = s_sm.have_seq && c.seq == s_sm.last_seq;
        const uint8_t rej = actcon_pyro_command(&s_sm, &c, now);
        if (c.cmd == ACTCON_CMD_HEARTBEAT || repeat) continue;    // nothing to log
        if (rej != ACTCON_OK) {
            Serial.printf("[%8.2f s] command %u refused, reason %u\n", now / 1000.0, c.cmd, rej);
        } else if (c.cmd == ACTCON_CMD_ARM && !was_armed) {
            Serial.printf("[%8.2f s] ARMED for %lu s\n", now / 1000.0,
                          (unsigned long)(ACTCON_ARM_TIMEOUT_MS / 1000u));
        } else if (c.cmd == ACTCON_CMD_DISARM && was_armed) {
            Serial.printf("[%8.2f s] disarmed\n", now / 1000.0);
        } else if (c.cmd == ACTCON_CMD_FIRE) {
            Serial.printf("[%8.2f s] FIRE CH%u for %u ms\n", now / 1000.0, c.ch,
                          actcon_clamp_pulse(c.pulse_ms));
        }
    }
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

// =====================================================================
void setup()
{
    // Outputs LOW before anything else can run.
    for (uint8_t i = 0; i < ACTCON_N_PYRO; i++) {
        digitalWrite(PYRO_PIN[i], LOW);
        pinMode(PYRO_PIN[i], OUTPUT);
        digitalWrite(PYRO_PIN[i], LOW);
        pinMode(CONT_PIN[i], INPUT);
    }
    analogReadResolution(12);
    actcon_pyro_init(&s_sm);

#ifdef NEOPIXEL_POWER
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
    s_px.begin();
    s_px.clear();
    s_px.show();

    Serial.begin(115200);
    MASTER_PORT.setTX(PIN_MASTER_TX);
    MASTER_PORT.setRX(PIN_MASTER_RX);
    MASTER_PORT.setFIFOSize(256);
    MASTER_PORT.begin(ARIADNA_MCU_BAUD);
    ariadna_rx_init(&s_rx);
}

void loop()
{
    static uint32_t next_cont = 0, next_rep = 0;
    static uint8_t last_link = 0;
    const uint32_t now = millis();

    link_service(now);

    const uint8_t was_armed = s_sm.armed;
    s_out_mask = actcon_pyro_tick(&s_sm, now);
    outputs_write(s_out_mask);
    if (was_armed && !s_sm.armed)
        Serial.printf("[%8.2f s] disarmed (%s)\n", now / 1000.0,
                      s_sm.last_reject == ACTCON_REJ_NO_LINK ? "Master heartbeat lost"
                                                             : "pulse done / timeout");
    if (s_sm.link != last_link) {
        last_link = s_sm.link;
        Serial.printf("[%8.2f s] Master link %s\n", now / 1000.0, s_sm.link ? "up" : "LOST");
    }

    if ((int32_t)(now - next_cont) >= 0) {
        next_cont = now + 20;
        continuity_read();
    }
    if ((int32_t)(now - next_rep) >= 0) {
        next_rep = now + ACTCON_REPORT_PERIOD_MS;
        report_send(now);
    }

    if (Serial.available() && Serial.read() == 'p')
        Serial.printf("link %u armed %u (%u s left) cont 0x%X raw %u %u %u fired 0x%X\n",
                      s_sm.link, s_sm.armed, actcon_arm_left_ds(&s_sm, now) / 10u, s_cont_mask,
                      s_raw[0], s_raw[1], s_raw[2], s_sm.fired_mask);

    if (s_out_mask)           rgb(255, 0, 0);
    else if (s_sm.armed)      rgb((now / 150) % 2 ? 255 : 0, (now / 150) % 2 ? 110 : 0, 0);
    else if (!s_sm.link)      rgb(0, 0, (now / 500) % 2 ? 40 : 0);
    else                      rgb(0, 30, 0);
}
