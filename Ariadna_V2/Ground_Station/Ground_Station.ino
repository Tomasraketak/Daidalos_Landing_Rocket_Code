// =====================================================================
//  Ariadna V2 — GROUND STATION RECEIVER
//
//  Space Carrots / Czech Rocket Challenge 2026
//
//  Target: Raspberry Pi Pico 2 (RP2350), arduino-pico core.
//  Needs:  LoRa (sandeepmistry), Adafruit_GFX, Adafruit_SSD1306
//
//  Listens for the vehicle's 46-byte downlink, stamps it with the measured
//  link quality, and forwards it over USB to ground_visualizer.py. A local
//  OLED shows enough to run the flight if the laptop dies.
//
//  ---------------------------------------------------------------------
//  WHAT CHANGED FROM THE V1 GROUND STATION
//  ---------------------------------------------------------------------
//   * The downlink packet is ariadna_tlm_t (46 B), carrying the V2 state:
//     mission phase, TVC angles, thrust clamp, pyro continuity, per-board
//     health. The V1 24-byte packet is NOT compatible and this build will
//     not decode it.
//
//   * The USB stream to the laptop is FRAMED — sync word, length, CRC —
//     instead of a bare struct write. The V1 scheme had no way to
//     resynchronise: one dropped byte on the USB link and every packet
//     after it was misaligned garbage until the app was restarted. This
//     one recovers on the next packet, and the CRC means the visualiser
//     can tell a corrupted frame from a real reading of the same value.
//
//   * Same pinout, same OLED, same buttons. Nothing in the ground hardware
//     needs rewiring.
//
//  ---------------------------------------------------------------------
//  WHY THIS RUNS ON BOTH CORES
//  ---------------------------------------------------------------------
//  A full SSD1306 refresh pushes 1024 bytes over I2C. At the Arduino
//  default 100 kHz that is 92 ms of blocking, every time the screen is
//  redrawn:
//
//      1024 bytes x 9 bits / 100 kHz  =  92.2 ms
//      1024 bytes x 9 bits / 400 kHz  =  23.0 ms
//
//  With a 10 Hz downlink the packets are 100 ms apart, so a display update
//  at 100 kHz would leave the receiver blind for most of the interval. The
//  SX1278 holds one packet in its FIFO, so a single overlap is survivable
//  — two in a row is not, and that is a packet loss mechanism with no
//  error message attached to it.
//
//  Two fixes, both here: the I2C bus runs at 400 kHz, and the display
//  lives on core 1 while the radio has core 0 to itself. The OLED and the
//  radio are on different buses (I2C and SPI1), so the split is clean.
//
//    core 0   LoRa receive and the USB uplink — never blocked by anything
//    core 1   OLED and the buttons — may block as much as it likes
// =====================================================================

#include "ariadna_link.h"

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hardware/watchdog.h"

#include <string.h>
#include <math.h>

// =====================================================================
//  1. HARDWARE — unchanged from the V1 ground station
// =====================================================================
static const int LORA_MISO  = 12;       // SPI1
static const int LORA_CS    = 11;
static const int LORA_SCK   = 14;
static const int LORA_MOSI  = 15;
static const int LORA_RESET = 20;
static const int LORA_DIO0  = 21;

static const int I2C_SDA    = 4;        // OLED
static const int I2C_SCL    = 5;

static const int TEST_ON_PIN   = 3;     // active LOW
static const int TEST_OFF_PIN  = 6;     // active LOW
static const int RESET_BTN_PIN = 10;    // active HIGH

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Mission state names, short enough for a 128 px line. Must match
// ariadna_state_t.
static const char *STATE_NAME[12] = {
    "BOOT", "IDLE", "ARM", "ASCENT", "APOGEE", "DESC",
    "GUIDE", "LANDED", "RECOV", "ABORT", "TEST", "ERROR"
};

// =====================================================================
//  2. STATE
// =====================================================================
static ariadna_tlm_t      s_pkt;
static ariadna_gs_serial_t s_out;

static bool     s_test_mode      = false;
static uint32_t s_last_test_ms   = 0;

static uint32_t s_last_rx_ms     = 0;
static uint32_t s_last_disp_ms   = 0;
static uint32_t s_packet_delta   = 0;
static uint32_t s_rx_count       = 0;
static uint32_t s_rx_bad_size    = 0;
static bool     s_have_packet    = false;

static int      s_rssi           = 0;
static int      s_snr            = 0;

static volatile bool s_setup_done = false;

// Core 0 receives, core 1 draws. The display gets its own snapshot through
// a seqlock so a redraw can never show half of one packet and half of the
// next. A torn frame on a screen is only cosmetic, but the lock is ten
// lines and it makes the data flow obvious.
struct DisplaySnapshot {
    ariadna_tlm_t pkt;
    uint32_t last_rx_ms;
    uint32_t packet_delta;
    uint32_t rx_count;
    int      rssi, snr;
    bool     have_packet;
    bool     test_mode;
};

static volatile uint32_t s_disp_seq = 0;
static DisplaySnapshot   s_disp_shared;

// Buttons live on core 1, the test-mode generator on core 0.
static volatile bool s_test_mode_req = false;

static void disp_publish(const DisplaySnapshot *d)
{
    s_disp_seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_disp_shared = *d;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_disp_seq++;
}

static bool disp_acquire(DisplaySnapshot *d)
{
    uint32_t a = s_disp_seq;
    if (a & 1u) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *d = s_disp_shared;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return s_disp_seq == a;
}

// =====================================================================
//  3. USB FRAMING
//
//  The whole reason this exists: USB CDC can lose or reorder nothing, but
//  the app on the other end can be started at any moment, mid-packet. A
//  fixed-size raw struct read has no way to find the boundary again. The
//  sync word gives it one, and the CRC stops it from believing a frame it
//  found by luck.
// =====================================================================
static void usb_send(const ariadna_gs_serial_t *g)
{
    uint8_t frame[ARIADNA_FRAME_MAX];
    uint16_t n = ariadna_encode(frame, ARIADNA_PKT_TELEMETRY, g, sizeof(*g));
    if (n) Serial.write(frame, n);
}

// =====================================================================
//  4. OLED
// =====================================================================
static void display_banner(const char *line1, const char *line2)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(6, 20);
    display.println(line1);
    display.setCursor(6, 40);
    display.println(line2);
    display.display();
}

static void display_test_status(bool enabled)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(5, 10);
    display.println("TEST MODE");
    display.setCursor(35, 35);
    display.println(enabled ? "ON" : "OFF");
    display.display();
    delay(800);
}

/**
 * The flight display.
 *
 * Laid out so the two things that decide whether to press abort — altitude
 * and mission state — are readable at arm's length, and everything else is
 * secondary. Health and pyro continuity get a line of their own because on
 * this vehicle they are the pre-launch go/no-go.
 */
static void display_flight(const DisplaySnapshot *d)
{
    const ariadna_tlm_t &pkt = d->pkt;
    display.clearDisplay();

    // No signal, and we have had one before: say so rather than showing
    // stale numbers that look live.
    if (!d->test_mode && d->have_packet && millis() - d->last_rx_ms > 2500) {
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 12);
        display.print("NO SIGNAL");
        display.setTextSize(1);
        display.setCursor(10, 40);
        display.print("last ");
        display.print((millis() - d->last_rx_ms) / 1000);
        display.print("s ago");
        display.setCursor(10, 52);
        display.print("rx:");
        display.print(d->rx_count);
        display.display();
        return;
    }

    display.setTextColor(SSD1306_WHITE);

    // --- line 1: link ---------------------------------------------------
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("R:");   display.print(d->rssi);
    display.print(" S:");  display.print(d->snr);
    display.print(" ");    display.print(d->packet_delta);
    display.print("ms");

    // --- line 2: altitude, large ----------------------------------------
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.print(pkt.alt_dm / 10);
    display.print("m");

    // State, right-aligned on the same band.
    display.setTextSize(1);
    const char *sn = (pkt.state < 12) ? STATE_NAME[pkt.state] : "?";
    display.setCursor(128 - (int)strlen(sn) * 6, 14);
    display.print(sn);

    // Vertical speed under it: sign matters, so print it explicitly.
    display.setCursor(0, 30);
    display.print("vz:");
    display.print(pkt.vz_dms / 10.0f, 1);
    display.print(" vh:");
    display.print(pkt.vh_dms / 10.0f, 1);

    // --- line 4: TVC and clamp, live only during the burn ----------------
    display.setCursor(0, 40);
    if (pkt.nav_phase == 3 || pkt.nav_phase == 4) {   // IGNITING / BURN
        display.print("TVC");
        display.print(pkt.tvc_x_cdeg / 100.0f, 1);
        display.print("/");
        display.print(pkt.tvc_y_cdeg / 100.0f, 1);
        display.print(" K");
        display.print(pkt.clamp_pct);
    } else {
        display.print("B:");
        display.print((pkt.vbat_cv / 50.0f) + 5.0f, 2);
        display.print("V ");
        if (pkt.nav_phase == 2 && pkt.t_event_ds != ARIADNA_TLM_T_UNKNOWN) {
            // descending: the countdown to the ignition command
            display.print("IGN-");
            display.print(pkt.t_event_ds / 10.0f, 1);
        } else {
            display.print(pkt.curr_da / 10.0f, 1);
            display.print("A S:");
            display.print(pkt.sats);
        }
    }

    // --- line 5: go/no-go ------------------------------------------------
    display.setCursor(0, 52);
    display.print(pkt.flags & ARIADNA_TF_ARMED   ? "A" : "-");
    display.print(pkt.flags & ARIADNA_TF_RBF_OUT ? "R" : "-");
    display.print(pkt.health & ARIADNA_HLT_POS_LINK  ? "P" : "-");
    display.print(pkt.health & ARIADNA_HLT_NAV_LINK  ? "N" : "-");
    display.print(pkt.health & ARIADNA_HLT_NAV_VALID ? "V" : "-");
    display.print(pkt.health & ARIADNA_HLT_FLASH_OK  ? "F" : "-");
    display.print(pkt.health & ARIADNA_HLT_SERVO_OK  ? "S" : "-");

    // Pyro continuity, low nibble of .pyro. This is the number the pad
    // crew is actually waiting for.
    display.print(" py:");
    for (int i = 0; i < 3; i++)
        display.print((pkt.pyro & (1 << i)) ? (char)('1' + i) : '-');

    uint8_t err = (uint8_t)((pkt.flags & ARIADNA_TF_ERR_MASK) >> ARIADNA_TF_ERR_SHIFT);
    if (err) { display.print(" E"); display.print(err); }
    if (d->test_mode) { display.setCursor(112, 52); display.print("[T]"); }

    display.display();
}

// =====================================================================
//  5. TEST MODE
//
//  Generates a plausible V2 packet so the whole ground chain — this board,
//  the USB framing, the visualiser — can be exercised with no vehicle
//  present. It is the only way to rehearse a launch day indoors.
//
//  Deliberately drives the fields the V1 simulator could not: it walks the
//  full mission state machine, lights a burn with a moving TVC command and
//  a closing clamp, and toggles pyro continuity.
// =====================================================================
static void generate_test_packet(void)
{
    const float m = (float)millis() / 1000.0f;
    const float cyc = fmodf(m, 40.0f);           // one 40 s mission per loop

    memset(&s_pkt, 0, sizeof(s_pkt));
    s_pkt.version = ARIADNA_TLM_VERSION;
    s_pkt.t_ms    = millis();

    // A crude but continuous flight profile: up, over, down, burn, land.
    float h, vz;
    uint8_t state, phase;
    // dg_phase_t: 0 idle, 1 ascent, 2 descent, 3 igniting, 4 burn, 5 done
    float t_event = -1.0f;
    uint8_t events = 0;
    if (cyc < 4.0f)        { state = ARIADNA_STATE_ARMED;    h = 0.0f;   vz = 0.0f;   phase = 0; }
    else if (cyc < 9.0f)   { state = ARIADNA_STATE_ASCENT;   h = (cyc - 4.0f) * 32.0f;  vz = 32.0f;  phase = 1; }
    else if (cyc < 10.0f)  { state = ARIADNA_STATE_APOGEE;   h = 160.0f; vz = 0.0f;   phase = 1; }
    else if (cyc < 14.0f)  { state = ARIADNA_STATE_DESCENT;  h = 160.0f - (cyc - 10.0f) * 26.0f; vz = -26.0f; phase = 2;
                             t_event = 14.0f - cyc; events = 0x01 | (t_event < 0.6f ? 0x02 : 0); }
    else if (cyc < 16.6f)  { state = ARIADNA_STATE_GUIDANCE; h = 56.0f * (1.0f - (cyc - 14.0f) / 2.6f); vz = -30.0f * (1.0f - (cyc - 14.0f) / 2.7f); phase = 4;
                             t_event = 16.6f - cyc; events = 0x0F | (t_event < 1.7f ? 0x20 : 0) | (h < 7.0f ? 0x40 : 0); }
    else if (cyc < 22.0f)  { state = ARIADNA_STATE_LANDED;   h = 0.0f;   vz = 0.0f;   phase = 5; events = 0xEF; }
    else                   { state = ARIADNA_STATE_RECOVERY; h = 0.0f;   vz = 0.0f;   phase = 5; events = 0xEF; }

    s_pkt.state     = state;
    s_pkt.nav_phase = phase;

    s_pkt.lat = (int32_t)((49.7985411 + sin(m * 0.1) * 0.0004) * 1e7);
    s_pkt.lon = (int32_t)((16.6912772 + cos(m * 0.1) * 0.0004) * 1e7);

    s_pkt.alt_dm = (int16_t)(h * 10.0f);
    s_pkt.vz_dms = (int16_t)(vz * 10.0f);
    s_pkt.vh_dms = (int16_t)((4.0f + sinf(m) * 3.0f) * 10.0f);
    s_pkt.acc_cg = (int16_t)((phase == 4 ? 4.0f : 1.0f) * 100.0f);

    // Attitude: nose up until the apogee, nose down after, with a few
    // degrees of wobble. A rotation of `ang` about the horizontal axis
    // (cos a, sin a, 0); nose-down is that rotation plus half a turn.
    {
        float tilt = (5.0f + 4.0f * sinf(m * 0.9f)) * 0.0174533f;
        float az   = m * 0.6f;
        float ang  = (phase >= 2) ? 3.14159265f - tilt : tilt;
        float qw = cosf(0.5f * ang), sh = sinf(0.5f * ang);
        float qx = sh * cosf(az), qy = sh * sinf(az), qz = 0.0f;
        float sg = qw < 0.0f ? -32767.0f : 32767.0f;
        s_pkt.qx = (int16_t)(qx * sg);
        s_pkt.qy = (int16_t)(qy * sg);
        s_pkt.qz = (int16_t)(qz * sg);
    }
    s_pkt.t_event_ds = t_event < 0.0f ? (int16_t)ARIADNA_TLM_T_UNKNOWN : (int16_t)(t_event * 10.0f);
    s_pkt.events     = events;
    s_pkt.t_flight_s = (uint8_t)(cyc > 4.0f ? cyc - 4.0f : 0.0f);
    s_pkt.px_m       = (int8_t)(sinf(m * 0.3f) * 6.0f);
    s_pkt.py_m       = (int8_t)(cosf(m * 0.2f) * 4.0f);

    // TVC only moves during the burn, which is what the real vehicle does.
    if (phase == 4) {
        s_pkt.tvc_x_cdeg = (int16_t)(sinf(m * 6.0f) * 3.5f * 100.0f);
        s_pkt.tvc_y_cdeg = (int16_t)(cosf(m * 5.0f) * 2.5f * 100.0f);
        s_pkt.clamp_pct  = (uint8_t)(55 + (int)(sinf(m * 3.0f) * 40.0f));
        s_pkt.thrust_n   = (int16_t)(95 + (int)(sinf(m * 4.0f) * 20.0f));
    } else {
        s_pkt.clamp_pct  = 100;
    }

    s_pkt.vbat_cv  = (uint8_t)((8.10f - fmodf(m, 120.0f) * 0.004f - 5.0f) * 50.0f);
    s_pkt.curr_da  = (uint8_t)(phase == 4 ? 55 : 12);
    s_pkt.hdop_x10 = (uint8_t)(9 + (int)(fabsf(sinf(m * 0.5f)) * 8.0f));
    s_pkt.sats     = (uint8_t)(11 + (int)(sinf(m * 0.2f) * 3.0f));

    // Continuity on channels 1 and 2 until they fire, then the fired bits
    // come up in the high nibble.
    uint8_t cont  = (phase >= 4) ? 0x00 : 0x07;
    uint8_t fired = (phase >= 4) ? ((events & 0x20) ? 0x05 : 0x01) : 0x00;
    s_pkt.pyro = (uint8_t)((cont & 0x0F) | ((fired & 0x0F) << 4));

    s_pkt.health = ARIADNA_HLT_POS_LINK | ARIADNA_HLT_NAV_LINK |
                   ARIADNA_HLT_NAV_VALID | ARIADNA_HLT_FLASH_OK |
                   ARIADNA_HLT_SERVO_OK | ARIADNA_HLT_GPS_FIX |
                   ARIADNA_HLT_POST_PASS |
                   (state >= ARIADNA_STATE_ARMED ? ARIADNA_HLT_LOGGING : 0);

    s_pkt.flags = ARIADNA_TF_TEST;
    if (state >= ARIADNA_STATE_ARMED) s_pkt.flags |= ARIADNA_TF_ARMED | ARIADNA_TF_RBF_OUT;
    if (state >= ARIADNA_STATE_ASCENT) s_pkt.flags |= ARIADNA_TF_BREAKWIRE;


    s_rssi = -65 + (int)(sinf(m * 5.0f) * 10.0f);
    s_snr  = 10 + (int)(cosf(m * 3.0f) * 3.0f);
}

// =====================================================================
//  6. SETUP
// =====================================================================
void setup()
{
    Serial.begin(ARIADNA_GS_USB_BAUD);

    // Give a host three seconds to attach, then carry on regardless — the
    // ground station must come up whether or not a laptop is plugged in.
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) { }

    pinMode(TEST_ON_PIN,   INPUT);
    pinMode(TEST_OFF_PIN,  INPUT);
    pinMode(RESET_BTN_PIN, INPUT);

    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    // 400 kHz, not the default 100 kHz. A full framebuffer is 1024 bytes;
    // this is the difference between 23 ms and 92 ms per redraw. See the
    // note at the top of the file.
    Wire.setClock(400000);

    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 12);
        display.println("SPACE CARROTS");
        display.setCursor(10, 28);
        display.println("ARIADNA V2 GS");
        display.setCursor(10, 44);
        display.println("waiting for link");
        display.display();
    }

    SPI1.setRX(LORA_MISO);
    SPI1.setTX(LORA_MOSI);
    SPI1.setSCK(LORA_SCK);
    SPI1.begin();

    LoRa.setSPI(SPI1);
    LoRa.setPins(LORA_CS, LORA_RESET, LORA_DIO0);

    if (!LoRa.begin(ARIADNA_LORA_FREQ_HZ)) {
        display_banner("LORA FAIL", "check wiring");
        while (1) { }
    }

    // Every parameter comes from ariadna_link.h, the same header the
    // vehicle reads. That is the point: these used to be duplicated in two
    // files under a "must match" comment, which is a comment, not a
    // guarantee.
    LoRa.setSpreadingFactor(ARIADNA_LORA_SF);
    LoRa.setSignalBandwidth(ARIADNA_LORA_BW_HZ);
    LoRa.setCodingRate4(ARIADNA_LORA_CR4);
    LoRa.setPreambleLength(ARIADNA_LORA_PREAMBLE);
    LoRa.setTxPower(ARIADNA_LORA_TX_DBM);
    LoRa.enableCrc();

    s_setup_done = true;
}

// =====================================================================
//  7. LOOP
// =====================================================================
// =====================================================================
//  7. CORE 0 — receive and forward
//
//  Nothing in this loop blocks. The OLED, which does, is on core 1.
// =====================================================================
void loop()
{
    const uint32_t now = millis();

    // ---- commands from the laptop ---------------------------------------
    // Plain text: it is typed by a human as often as it is sent by the
    // visualiser, and a human cannot type a binary frame.
    if (Serial.available()) {
        static char cmd[16];
        static uint8_t n = 0;
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[n] = '\0';
            if      (!strcmp(cmd, "T:1")) s_test_mode_req = true;
            else if (!strcmp(cmd, "T:0")) s_test_mode_req = false;
            n = 0;
        } else if (n < sizeof(cmd) - 1) {
            cmd[n++] = c;
        }
    }
    s_test_mode = s_test_mode_req;

    // ---- LoRa reception ---------------------------------------------------
    int size = LoRa.parsePacket();
    if (size) {
        if (size == (int)sizeof(ariadna_tlm_t)) {
            LoRa.readBytes((uint8_t *)&s_pkt, sizeof(s_pkt));

            s_rssi = LoRa.packetRssi();
            s_snr  = (int)LoRa.packetSnr();

            // A packet whose version we do not understand is dropped rather
            // than decoded as if it were ours. This is what stops another
            // team's flight computer on the pad next to yours from painting
            // nonsense across your display.
            if (s_pkt.version != ARIADNA_TLM_VERSION) {
                s_rx_bad_size++;
            } else {
                if (s_last_rx_ms) s_packet_delta = now - s_last_rx_ms;
                s_last_rx_ms  = now;
                s_have_packet = true;
                s_rx_count++;
                s_test_mode_req = false;   // a real vehicle beats a simulation
                s_test_mode     = false;

                memcpy(&s_out.tlm, &s_pkt, sizeof(s_pkt));
                s_out.rssi = (int8_t)s_rssi;
                s_out.snr  = (int8_t)s_snr;
                usb_send(&s_out);
            }
        } else {
            // Wrong length: drain it so the radio is ready for the next.
            while (LoRa.available()) LoRa.read();
            s_rx_bad_size++;
        }
    }

    // ---- test mode ---------------------------------------------------------
    else if (s_test_mode && now - s_last_test_ms >= 1000 / ARIADNA_TLM_HZ) {
        s_last_test_ms = now;
        if (s_last_rx_ms) s_packet_delta = now - s_last_rx_ms;
        s_last_rx_ms  = now;
        s_have_packet = true;

        generate_test_packet();

        memcpy(&s_out.tlm, &s_pkt, sizeof(s_pkt));
        s_out.rssi = (int8_t)s_rssi;
        s_out.snr  = (int8_t)s_snr;
        usb_send(&s_out);
    }

    // ---- hand the display core a consistent snapshot ------------------------
    {
        DisplaySnapshot d;
        d.pkt          = s_pkt;
        d.last_rx_ms   = s_last_rx_ms;
        d.packet_delta = s_packet_delta;
        d.rx_count     = s_rx_count;
        d.rssi         = s_rssi;
        d.snr          = s_snr;
        d.have_packet  = s_have_packet;
        d.test_mode    = s_test_mode;
        disp_publish(&d);
    }
}

// =====================================================================
//  8. CORE 1 — display and buttons
//
//  Everything here may block, and the OLED does: 23 ms per full redraw at
//  400 kHz. That is why it is not on the core holding the radio.
// =====================================================================
void setup1()
{
    while (!s_setup_done) tight_loop_contents();
}

void loop1()
{
    static uint32_t next_disp_ms = 0;
    const  uint32_t now = millis();

    // ---- buttons, ignored for the first 5 s while the lines settle --------
    if (now > 5000) {
        if (digitalRead(RESET_BTN_PIN) == HIGH) {
            display_banner("RESETTING", "");
            delay(100);
            watchdog_enable(1, 1);
            while (1) { }
        }
        if (digitalRead(TEST_ON_PIN) == LOW && !s_test_mode_req) {
            s_test_mode_req = true;
            display_test_status(true);
        }
        if (digitalRead(TEST_OFF_PIN) == LOW && s_test_mode_req) {
            s_test_mode_req = false;
            display_test_status(false);
        }
    }

    // ---- display ------------------------------------------------------------
    // 5 Hz, not 10. A redraw is 23 ms of I2C and nothing on this screen
    // changes usefully faster than five times a second; the spare time is
    // better spent not being in the middle of a transaction.
    if ((int32_t)(now - next_disp_ms) >= 0) {
        next_disp_ms = now + 200;

        DisplaySnapshot d;
        if (disp_acquire(&d)) display_flight(&d);
    }
}
