// =====================================================================
//  Ariadna V2 — UWB GROUND ANCHOR
//
//  Target: Seeed XIAO RP2350, arduino-pico core (earlephilhower).
//  Needs:  arduino-dw1000 (thotro)  https://github.com/thotro/arduino-dw1000
//
//  Four of these stand on the ground around the pad. Each answers
//  two-way ranging requests from the tag on the vehicle; the tag turns
//  the four distances into a position. This sketch is the ANCHOR half —
//  the tag half lives in ../Position_MCU/position_mcu.ino, and the
//  geometry that consumes both is ../common/ariadna_uwb.h.
//
//  ---------------------------------------------------------------------
//  FLASHING FOUR BOARDS FROM ONE SKETCH
//  ---------------------------------------------------------------------
//  Change ANCHOR_ID below to 1, 2, 3 or 4 and flash each board in turn.
//  Nothing else differs between them. WRITE THE NUMBER ON THE BOX — an
//  anchor whose ID does not match where it physically stands is the one
//  failure mode of this whole system that produces a confident, wrong
//  position rather than no position at all.
//
//  ---------------------------------------------------------------------
//  THE SHORT ADDRESS, AND THE ARGUMENT THAT MUST NOT BE DEFAULTED
//  ---------------------------------------------------------------------
//  The tag matches a range to an anchor by the anchor's 16-bit short
//  address, looked up in the table in position_mcu.ino. In the
//  arduino-dw1000 library that address is RANDOMISED at startup unless
//  startAsAnchor() is passed `false` as its third argument.
//
//  Every published example omits that argument. With it omitted, this
//  anchor gets a fresh random address every time it is powered on, the
//  tag finds no matching row, and every range is silently discarded: the
//  radios work, the LEDs blink, and the navigation filter never sees a
//  single UWB fix. It is passed explicitly below, and it is the single
//  most important line in this file.
//
//  With `false`, the short address is the FIRST TWO BYTES of the EUI,
//  little-endian — so the EUI "01:00:..." gives short address 0x0001.
//  That is why the EUIs below differ only in their first byte.
//
//  ---------------------------------------------------------------------
//  PLACING THEM
//  ---------------------------------------------------------------------
//  Spread out, in a square if the site allows it, and clear of metal. The
//  geometry sets the accuracy far more than the radio does: four anchors
//  in a 40 m square give sub-decimetre horizontal accuracy near the
//  centre, and the same four in a rough line give metres across the line.
//
//  Then MEASURE where they are and put those numbers into
//  s_uwb_anchors[] in position_mcu.ino. Nothing discovers this for you.
//
//  ---------------------------------------------------------------------
//  THE VOLTINOLABS HARDWARE WORKAROUNDS
//  ---------------------------------------------------------------------
//  Identical to the tag's, and necessary for the same reasons:
//    1. RST is driven LOW to reset and then RELEASED to a floating input.
//       It is not a push-pull input and driving it high can damage the
//       module. The library is told the reset pin is 255 so that it never
//       drives it either.
//    2. The SPI pins are selected explicitly; the core's defaults are not
//       this board's wiring.
//    3. The library's attached interrupt is detached and the IRQ line is
//       polled from loop() instead.
// =====================================================================

#include <SPI.h>
#include <DW1000Ranging.h>
#include <DW1000.h>

#include "../common/ariadna_rgb.h"

// =====================================================================
//  1. WHICH ANCHOR IS THIS
// =====================================================================
#ifndef ANCHOR_ID
#define ANCHOR_ID  1        // 1..4 — CHANGE THIS PER BOARD
#endif

#if ANCHOR_ID < 1 || ANCHOR_ID > 4
#  error "ANCHOR_ID must be 1..4 and must match a row of s_uwb_anchors[]"
#endif

// EUIs differ only in the first byte, which becomes the short address.
static const char *ANCHOR_EUI[4] = {
    "01:00:5B:D5:A9:9A:E2:9C",   // short address 0x0001
    "02:00:5B:D5:A9:9A:E2:9C",   // 0x0002
    "03:00:5B:D5:A9:9A:E2:9C",   // 0x0003
    "04:00:5B:D5:A9:9A:E2:9C",   // 0x0004
};

// =====================================================================
//  2. PINS
//
//  Same board and the same wiring as the Position MCU, so the same pins.
//  The anchor has no other peripherals: the SPI bus is its own.
// =====================================================================
static const uint8_t PIN_SPI_SCK  = D8;
static const uint8_t PIN_SPI_MISO = D9;
static const uint8_t PIN_SPI_MOSI = D10;

static const uint8_t PIN_UWB_INT  = D0;
static const uint8_t PIN_UWB_RST  = D1;
static const uint8_t PIN_UWB_CS   = D2;

// =====================================================================
//  3. ANNUNCIATOR
//
//  The anchor's on-board RGB, because an anchor standing in the grass is
//  the one part of this system nobody can query. What the crew needs to
//  know while walking the site is answered by the colour:
//
//    solid red      the DWM1000 did not come up. The anchor is dead.
//    slow blue      up and listening, but the tag has never called.
//                   Normal before the vehicle is powered.
//    green pulse    ranging. This is what all four should look like
//                   during a countdown.
//    amber fast     it WAS ranging and the tag has gone quiet for more
//                   than five seconds. On the pad that means the vehicle
//                   is off or its radio has failed — worth knowing before
//                   somebody walks up to it.
// =====================================================================
static ariadna_rgb_t s_rgb;
static uint32_t      s_last_range_ms = 0;
static uint32_t      s_range_count   = 0;
static bool          s_up            = false;

// =====================================================================
//  4. RANGING CALLBACKS
//
//  Every one of them tolerates a null device pointer: the library calls
//  back from inside its own state machine and has been observed to do so
//  during teardown.
// =====================================================================
static void on_range(void)
{
    DW1000Device *d = DW1000Ranging.getDistantDevice();
    if (!d) return;
    s_last_range_ms = millis();
    s_range_count++;

    // Rate limited, and only over USB. A print inside the ranging path is
    // the classic way to make an anchor that works on the bench fail in
    // the field: at 115200 baud a line is a millisecond, which is long
    // enough to miss the next exchange.
    static uint32_t next_ms = 0;
    if (millis() >= next_ms) {
        next_ms = millis() + 500;
        Serial.print("tag ");
        Serial.print(d->getShortAddress(), HEX);
        Serial.print("  range ");
        Serial.print(d->getRange());
        Serial.print(" m  rx ");
        Serial.print(d->getRXPower());
        Serial.println(" dBm");
    }
}

static void on_blink(DW1000Device *d)
{
    if (!d) return;
    Serial.print("tag appeared: ");
    Serial.println(d->getShortAddress(), HEX);
}

static void on_inactive(DW1000Device *d)
{
    if (!d) return;
    Serial.print("tag gone: ");
    Serial.println(d->getShortAddress(), HEX);
}

// =====================================================================
//  5. SETUP
// =====================================================================
void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (uint32_t)(millis() - t0) < 3000) { }

    ariadna_rgb_begin(&s_rgb, 60, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

    // --- workaround 1: open-drain reset --------------------------------
    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(20);
    pinMode(PIN_UWB_RST, INPUT);      // release, do NOT drive high
    delay(150);

    // --- workaround 2: explicit SPI pins -------------------------------
    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.begin();
    pinMode(PIN_UWB_INT, INPUT);

    // 255 as the reset pin keeps the library away from RST entirely.
    DW1000Ranging.initCommunication(255, PIN_UWB_CS, PIN_UWB_INT);

    // --- workaround 3: no background interrupt -------------------------
    detachInterrupt(digitalPinToInterrupt(PIN_UWB_INT));

    DW1000Ranging.attachNewRange(on_range);
    DW1000Ranging.attachBlinkDevice(on_blink);
    DW1000Ranging.attachInactiveDevice(on_inactive);

    // MODE_LONGDATA_RANGE_ACCURACY: the slowest and longest-ranging of
    // the library's presets. Range and reliability matter here; data rate
    // does not, because the payload is a handful of timestamps.
    //
    // The third argument is the one described at the top of this file.
    // Do not remove it.
    DW1000Ranging.startAsAnchor((char *)ANCHOR_EUI[ANCHOR_ID - 1],
                                DW1000.MODE_LONGDATA_RANGE_ACCURACY,
                                false);
    s_up = true;

    Serial.print("\nAriadna V2 UWB anchor ");
    Serial.print(ANCHOR_ID);
    Serial.print("  EUI ");
    Serial.print(ANCHOR_EUI[ANCHOR_ID - 1]);
    Serial.print("  short address 0x000");
    Serial.println(ANCHOR_ID);
    Serial.println("Put this anchor's surveyed position into s_uwb_anchors[] "
                   "in position_mcu.ino.");
}

// =====================================================================
//  6. LOOP
//
//  Poll the IRQ, service the ranging stack, throttle. The throttle is not
//  cosmetic: without it the RP2350 services the DWM1000 far faster than
//  it can answer, and the module responds by dropping exchanges.
// =====================================================================
void loop()
{
    if (digitalRead(PIN_UWB_INT) == HIGH) DW1000.handleInterrupt();
    DW1000Ranging.loop();

    // --- annunciator ---------------------------------------------------
    const uint32_t now = millis();
    if (!s_up) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_SOLID, 0);
    } else if (s_last_range_ms == 0) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SLOW, 0);
    } else if ((uint32_t)(now - s_last_range_ms) > 5000u) {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_FAST, 0);
    } else {
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_PULSE, 0);
    }
    ariadna_rgb_service(&s_rgb);

    delayMicroseconds(300);
}
