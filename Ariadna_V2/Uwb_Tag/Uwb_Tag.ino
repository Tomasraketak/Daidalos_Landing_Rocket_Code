// =====================================================================
//  Ariadna V2 — UWB TAG on the anchor PCB, I2C slave to the Position MCU
//
//  Target: Seeed XIAO RP2350 on the anchor PCB, arduino-pico core.
//  Needs:  arduino-dw1000 (thotro)  https://github.com/thotro/arduino-dw1000
//          Adafruit_NeoPixel
//
//  ---------------------------------------------------------------------
//  WHY THE TAG IS A SEPARATE BOARD
//  ---------------------------------------------------------------------
//  The DWM1000 used to share the Position MCU's SPI bus with the TriSense
//  IMU. It cannot: the TriSense module's level shifter drives SCK and
//  MOSI itself whenever the IMU is not selected, so every DWM1000
//  transaction was corrupted. The tag therefore runs here, on an anchor
//  PCB carried in the vehicle — same XIAO, same DWM1000 wiring as a
//  ground anchor — and hands its ranges to the Position MCU over I2C.
//
//  Moving it also takes the ranging stack off the Position MCU's core 0,
//  which runs the 240 Hz estimator and used to poll the DWM1000 between
//  ticks.
//
//  ---------------------------------------------------------------------
//  WIRING
//  ---------------------------------------------------------------------
//  One uSup cable from this board's I2CP connector to I2CP on the flight
//  computer: GND, 3V3, PSDA, PSCL. That puts this XIAO's D4/D5 on the
//  Position MCU's sensor bus, next to the BMP580 and the AK09918C. The
//  board is POWERED THROUGH THAT CABLE (about 150-250 mA, mostly the
//  DWM1000 receiver), so:
//
//    * do not fit a 1S battery to this board while it is a tag, or open
//      solder jumper R behind its AP7361C — two regulators on one 3V3
//      rail is not a configuration either was designed for;
//    * do not plug USB into this XIAO while the cable is connected —
//      unplug the cable to flash it.
//
//  The I2C pull-ups on this board (R2/R3, 10k) sit in parallel with the
//  TriSense module's own; keep the cable short (under ~30 cm).
//
//  ---------------------------------------------------------------------
//  WHAT THE POSITION MCU SEES
//  ---------------------------------------------------------------------
//  A 74-byte frame at I2C address 0x2A, defined in ariadna_uwb_link.h:
//  the latest range to each anchor, with its age at the moment of the
//  read. The Position MCU does all the judging — the anchor table, the
//  jump test, the sigma, the solve. This board only measures.
//
//  ---------------------------------------------------------------------
//  TWO CORES, ON PURPOSE
//  ---------------------------------------------------------------------
//    core 0   DW1000 ranging, polled exactly as on the ground anchors.
//    core 1   the I2C slave. Its interrupt is attached from setup1(), so
//             it runs on core 1 and the ~1.7 ms it takes to clock a
//             frame out never delays a ranging exchange on core 0 — the
//             DW1000 schedules its replies a few milliseconds ahead, and
//             a stall there costs exchanges.
//
//  The range table is shared through a critical section (a hardware
//  spinlock plus interrupts off on the core that holds it), held for a
//  few hundred nanoseconds on either side.
//
//  ---------------------------------------------------------------------
//  ANNUNCIATOR (on-board RGB)
//  ---------------------------------------------------------------------
//    solid red      the DWM1000 did not come up
//    slow blue      radio up, no anchor has answered yet
//    green pulse    ranging, and the Position MCU is reading
//    yellow pulse   ranging, but nobody has read for 2 s — cable, or
//                   the Position MCU is not running
//    amber fast     it WAS ranging and every anchor has been quiet for 5 s
// =====================================================================

#include <SPI.h>
#include <Wire.h>
#include <DW1000Ranging.h>
#include <DW1000.h>
#include <pico/critical_section.h>

// Named here, not only inside ariadna_rgb.h: the Arduino builder adds a
// library's include path only when it sees a real #include of it.
#ifdef ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
#include "ariadna_rgb.h"
#include "ariadna_uwb_link.h"

// =====================================================================
//  1. PINS — the anchor PCB
// =====================================================================
static const uint8_t PIN_SPI_SCK  = D8;
static const uint8_t PIN_SPI_MISO = D9;
static const uint8_t PIN_SPI_MOSI = D10;

static const uint8_t PIN_UWB_INT  = D0;
static const uint8_t PIN_UWB_RST  = D1;    // OPEN DRAIN: never drive it high
static const uint8_t PIN_UWB_CS   = D2;

static const uint8_t PIN_I2C_SDA  = D4;    // PSDA on the I2CP connector
static const uint8_t PIN_I2C_SCL  = D5;    // PSCL

// D4/D5 are GPIO6/7, which only the RP2350's second I2C block reaches.
#define TAG_I2C  ((((PIN_I2C_SDA) >> 1) & 1) ? Wire1 : Wire)

// =====================================================================
//  2. IDENTITY
//
//  The tag's EUI, unchanged from when it lived on the Position MCU. Only
//  its first two bytes matter to the anchors, and only because the tag is
//  started with a deterministic short address (third argument `false`).
// =====================================================================
static const char *TAG_EUI = "AA:00:5B:D5:A9:9A:E2:9C";

// Minimum microseconds between two services of the ranging stack. Same
// throttle as the anchors: without it the RP2350 services the DWM1000
// faster than it can answer and the module drops exchanges.
static const uint32_t POLL_MIN_US = 250;

static const uint8_t TAG_RGB_BRIGHTNESS = 25;   // powered through a thin cable

// =====================================================================
//  3. THE RANGE TABLE — written by core 0, read by core 1
// =====================================================================
struct Slot {
    uint16_t addr;          // 0 = free
    uint8_t  count;
    int8_t   rxp_dbm;
    float    range_m;
    uint32_t t_ms;          // when the range arrived, this board's clock
};

static Slot               s_slot[ARIADNA_UWB_LINK_SLOTS];
static uint16_t           s_total = 0;
static uint8_t            s_flags = 0;
static critical_section_t s_cs;

static volatile bool      s_core0_ready  = false;
static volatile uint32_t  s_last_read_ms = 0;     // written by core 1
static volatile uint32_t  s_reads        = 0;
static volatile uint32_t  s_core1_beat   = 0;
static uint32_t           s_last_range_ms = 0;    // core 0 only

static uint8_t clamp_dbm(float dbm)
{
    if (!(dbm == dbm)) return (uint8_t)(int8_t)-128;
    if (dbm < -128.0f) dbm = -128.0f;
    if (dbm >  127.0f) dbm =  127.0f;
    return (uint8_t)(int8_t)lrintf(dbm);
}

/** Core 0: store one range. The slot for an anchor is the one already
 *  holding its address, else a free one, else the stalest — so a table
 *  full of anchors that went away cannot lock a new one out. */
static void table_put(uint16_t addr, float range_m, float rxp_dbm)
{
    if (addr == 0) return;
    const uint32_t now = millis();
    critical_section_enter_blocking(&s_cs);
    int idx = -1, free_idx = -1, oldest = 0;
    for (int i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
        if (s_slot[i].addr == addr) { idx = i; break; }
        if (s_slot[i].addr == 0 && free_idx < 0) free_idx = i;
        if ((uint32_t)(now - s_slot[i].t_ms) > (uint32_t)(now - s_slot[oldest].t_ms)) oldest = i;
    }
    if (idx < 0) {
        idx = free_idx >= 0 ? free_idx : oldest;
        s_slot[idx].addr  = addr;
        s_slot[idx].count = 0;
    }
    s_slot[idx].range_m = range_m;
    s_slot[idx].rxp_dbm = (int8_t)clamp_dbm(rxp_dbm);
    s_slot[idx].t_ms    = now;
    s_slot[idx].count++;
    s_total++;
    critical_section_exit(&s_cs);
    s_last_range_ms = now;
}

// =====================================================================
//  4. I2C SLAVE — core 1
// =====================================================================

/**
 * The master is reading. Build the frame from a snapshot of the table,
 * ages computed NOW, and queue all of it.
 *
 * Wire.write() in slave mode feeds the TX FIFO directly and waits, per
 * byte, up to the Wire timeout for room. A master that stops early would
 * otherwise leave this handler waiting the timeout once per remaining
 * byte, so the first refused byte ends the frame.
 */
static void on_request(void)
{
    static uint16_t seq = 0;
    Slot     snap[ARIADNA_UWB_LINK_SLOTS];
    uint16_t total;
    uint8_t  flags;

    critical_section_enter_blocking(&s_cs);
    memcpy(snap, s_slot, sizeof(snap));
    total = s_total;
    flags = s_flags;
    critical_section_exit(&s_cs);

    const uint32_t now = millis();
    ariadna_uwb_link_frame_t f;
    memset(&f, 0, sizeof(f));
    f.flags = flags;
    f.seq   = seq++;
    f.total = total;
    for (int i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
        if (snap[i].addr == 0) continue;
        const uint32_t age = (uint32_t)(now - snap[i].t_ms);
        f.slot[i].addr    = snap[i].addr;
        f.slot[i].range_m = snap[i].range_m;
        f.slot[i].age_ms  = (uint16_t)(age > 65535u ? 65535u : age);
        f.slot[i].rxp_dbm = snap[i].rxp_dbm;
        f.slot[i].count   = snap[i].count;
        f.slot[i].valid   = 1;
    }

    uint8_t buf[ARIADNA_UWB_LINK_FRAME_SIZE];
    ariadna_uwb_link_encode(&f, buf);
    for (size_t i = 0; i < sizeof(buf); i++)
        if (TAG_I2C.write(buf[i]) != 1) break;

    s_last_read_ms = now;
    s_reads = s_reads + 1;
}

/** Writes from the master carry nothing; drain them so a bus scan's
 *  zero-length write or a stray byte cannot leave anything behind. */
static void on_receive(int n)
{
    while (n-- > 0 && TAG_I2C.available()) (void)TAG_I2C.read();
}

// =====================================================================
//  5. RANGING CALLBACKS — core 0
//
//  Every one tolerates a null device pointer: the library calls back from
//  inside its own state machine and has been seen to do so in teardown.
// =====================================================================
static void on_range(void)
{
    DW1000Device *d = DW1000Ranging.getDistantDevice();
    if (!d) return;
    table_put(d->getShortAddress(), d->getRange(), d->getRXPower());
}

static void on_device(DW1000Device *d)   { (void)d; }
// Attached so the library does not fall back to its default handler,
// which prints from inside the ranging path.
static void on_inactive(DW1000Device *d) { (void)d; }

static bool radio_identified(void)
{
    char id[128];
    memset(id, 0, sizeof(id));
    DW1000.getPrintableDeviceIdentifier(id);
    return strncmp(id, "DECA", 4) == 0;
}

// =====================================================================
//  6. CORE 0 — ranging
// =====================================================================
static ariadna_rgb_t s_rgb;
static bool          s_radio_up = false;

void setup()
{
    Serial.begin(115200);              // no wait: this board flies

    critical_section_init(&s_cs);
    memset(s_slot, 0, sizeof(s_slot));

    ariadna_rgb_begin(&s_rgb, TAG_RGB_BRIGHTNESS, ARIADNA_RGB_POWER_PIN);
    ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SOLID, 0);
    ariadna_rgb_service(&s_rgb);

    // --- open-drain reset: pull LOW, then RELEASE. Never drive it high.
    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(20);
    pinMode(PIN_UWB_RST, INPUT);
    delay(150);

    // --- explicit SPI pins: the core's defaults are not this wiring ----
    SPI.setSCK(PIN_SPI_SCK);
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.begin();
    pinMode(PIN_UWB_INT, INPUT);

    // 255 as the reset pin keeps the library away from RST entirely.
    DW1000Ranging.initCommunication(255, PIN_UWB_CS, PIN_UWB_INT);
    // The library's ISR path is not reliable on this core; the IRQ line
    // is polled from loop() instead.
    detachInterrupt(digitalPinToInterrupt(PIN_UWB_INT));

    DW1000Ranging.attachNewRange(on_range);
    DW1000Ranging.attachNewDevice(on_device);
    DW1000Ranging.attachInactiveDevice(on_inactive);

    s_radio_up = radio_identified();
    if (s_radio_up) {
        // Third argument false: a deterministic short address, so an
        // anchor that logs who it ranged with logs something meaningful.
        DW1000Ranging.startAsTag((char *)TAG_EUI,
                                 DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
    }
    critical_section_enter_blocking(&s_cs);
    s_flags = s_radio_up ? ARIADNA_UWB_LF_RADIO_UP : 0;
    critical_section_exit(&s_cs);

    Serial.printf("\nAriadna V2 UWB tag  EUI %s  radio %s  I2C slave 0x%02X on D4/D5\n",
                  TAG_EUI, s_radio_up ? "up" : "NOT FOUND", ARIADNA_UWB_I2C_ADDR);

    // A hung core is a silent tag; a reset is a tag that is back in a
    // few hundred milliseconds. Fed from loop() only while core 1 is
    // alive too.
    rp2040.wdt_begin(1000);
    s_core0_ready = true;
}

void loop()
{
    static uint32_t last_poll_us = 0, last_beat = 0, beat_ms = 0, next_print = 0;
    const uint32_t now_us = micros();
    if (s_radio_up && (uint32_t)(now_us - last_poll_us) >= POLL_MIN_US) {
        last_poll_us = now_us;
        if (digitalRead(PIN_UWB_INT) == HIGH) DW1000.handleInterrupt();
        DW1000Ranging.loop();
    }

    const uint32_t now = millis();

    // --- flags the master sees ------------------------------------------
    const bool ranging = s_last_range_ms != 0 && (uint32_t)(now - s_last_range_ms) < 1000u;
    const uint8_t flags = (uint8_t)((s_radio_up ? ARIADNA_UWB_LF_RADIO_UP : 0) |
                                    (ranging ? ARIADNA_UWB_LF_RANGING : 0));
    if (flags != s_flags) {
        critical_section_enter_blocking(&s_cs);
        s_flags = flags;
        critical_section_exit(&s_cs);
    }

    // --- watchdog: both cores must be alive ------------------------------
    if (s_core1_beat != last_beat) { last_beat = s_core1_beat; beat_ms = now; }
    if ((uint32_t)(now - beat_ms) < 200u) rp2040.wdt_reset();

    // --- annunciator -----------------------------------------------------
    const bool read_recently = s_last_read_ms != 0 &&
                               (uint32_t)(now - s_last_read_ms) < 2000u;
    if (!s_radio_up)
        ariadna_rgb_set(&s_rgb, ARIADNA_C_RED, ARIADNA_PAT_SOLID, 0);
    else if (s_last_range_ms == 0)
        ariadna_rgb_set(&s_rgb, ARIADNA_C_BLUE, ARIADNA_PAT_SLOW, 0);
    else if ((uint32_t)(now - s_last_range_ms) > 5000u)
        ariadna_rgb_set(&s_rgb, ARIADNA_C_AMBER, ARIADNA_PAT_FAST, 0);
    else if (!read_recently)
        ariadna_rgb_set(&s_rgb, ARIADNA_C_YELLOW, ARIADNA_PAT_PULSE, 0);
    else
        ariadna_rgb_set(&s_rgb, ARIADNA_C_GREEN, ARIADNA_PAT_PULSE, 0);
    ariadna_rgb_service(&s_rgb);

    // --- USB status, rate limited and never inside the ranging path -----
    if ((int32_t)(now - next_print) >= 0) {
        next_print = now + 1000;
        Slot snap[ARIADNA_UWB_LINK_SLOTS];
        critical_section_enter_blocking(&s_cs);
        memcpy(snap, s_slot, sizeof(snap));
        critical_section_exit(&s_cs);
        Serial.printf("tag %s, %lu reads by Position |", s_radio_up ? "up" : "DOWN",
                      (unsigned long)s_reads);
        for (int i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++) {
            if (!snap[i].addr) continue;
            Serial.printf(" A%04X %.2f m %d dBm %lu ms", snap[i].addr, (double)snap[i].range_m,
                          snap[i].rxp_dbm, (unsigned long)(now - snap[i].t_ms));
        }
        Serial.println();
    }
}

// =====================================================================
//  7. CORE 1 — the I2C slave
// =====================================================================
void setup1()
{
    while (!s_core0_ready) { }         // the critical section must exist

    TAG_I2C.setSDA(PIN_I2C_SDA);
    TAG_I2C.setSCL(PIN_I2C_SCL);
    // Per-byte wait for TX FIFO room. The master clocks a byte in ~23 us
    // at 400 kHz, so 3 ms only ever expires when the master has gone.
    TAG_I2C.setTimeout(3, false);
    TAG_I2C.onRequest(on_request);
    TAG_I2C.onReceive(on_receive);
    TAG_I2C.begin(ARIADNA_UWB_I2C_ADDR);   // IRQ attached on THIS core
}

void loop1()
{
    s_core1_beat = s_core1_beat + 1;
    delay(10);
}
