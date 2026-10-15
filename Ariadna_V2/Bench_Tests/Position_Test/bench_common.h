// =====================================================================
//  Ariadna V2 — BENCH TEST common code
//
//  Shared by Master_Test, Position_Test and Navigation_Test.
//
//  THIS FILE IS COPIED into each test sketch folder, because the Arduino
//  build only compiles what is inside the sketch folder. The copy in
//  Bench_Tests/common/ is the original; `cd ../../test && make bench-sync`
//  checks the three copies have not drifted, `make bench-sync-fix`
//  refreshes them.
//
//  What lives here:
//    1. test registry   — named PASS / WARN / FAIL results, printed on change
//    2. bench link      — a tiny framed protocol on the inter-MCU UARTs
//    3. ping / pong     — round-trip, loss and payload check between boards
//    4. report          — each board's results, shipped to the Master
//
//  NOTHING HERE BLOCKS. Every transmit fits inside the UART's 32-byte
//  hardware FIFO and is only written when that FIFO is empty, so a dead
//  or unplugged peer costs nothing but a FAIL line on the console.
//
//  The bench link deliberately uses DIFFERENT sync bytes from the flight
//  protocol (ariadna_link.h). A board still running flight firmware is
//  therefore detected and reported ("peer runs FLIGHT firmware") instead
//  of being misread as a broken link.
// =====================================================================
#ifndef BENCH_COMMON_H
#define BENCH_COMMON_H

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"

// Each board uses a different subset of the helpers below.
#pragma GCC diagnostic ignored "-Wunused-function"

#define BENCH_PROTO_VERSION  1
#define BENCH_MCU_BAUD       921600UL   // = ARIADNA_MCU_BAUD, the flight rate
#define BENCH_CONSOLE_BAUD   115200UL

// =====================================================================
//  1. NODES AND TEST TABLES
//
//  The names live here, in the shared file, so the Master can print the
//  results of the other two boards by name.
// =====================================================================
enum { NODE_NONE = 0, NODE_MASTER = 1, NODE_POSITION = 2, NODE_NAVIGATION = 3,
       NODE_N = 4 };

static const char *const BENCH_NODE_NAME[NODE_N] = {
    "?", "MASTER", "POSITION", "NAVIGATION"
};

enum {
    ST_PEND = 0,     // not decided yet
    ST_PASS = 1,
    ST_WARN = 2,     // works, but something is off — read the detail
    ST_FAIL = 3,
    ST_SKIP = 4,     // disabled at compile time, or cannot be tested
    ST_INFO = 5      // a reading, not a verdict (e.g. continuity open)
};

static const char *const BENCH_ST_NAME[6] = {
    "....", "PASS", "WARN", "FAIL", "SKIP", "INFO"
};

#define BENCH_MAX_TESTS 16

// ---- Master --------------------------------------------------------
enum {
    MT_PCA9685 = 0, MT_SERVO_WRITE, MT_FLASH_ID, MT_FLASH_RW, MT_LORA_INIT,
    MT_LORA_TX, MT_VBAT, MT_CURRENT, MT_RBF, MT_BREAKWIRE, MT_CORE1,
    MT_LINK_POS, MT_LINK_NAV, MT_RESET_POS, MT_RESET_NAV, MT_N
};
static const char *const MASTER_TEST_NAME[MT_N] = {
    "PCA9685", "SERVO_WRITE", "FLASH_ID", "FLASH_RW", "LORA_INIT",
    "LORA_TX", "VBAT_ADC", "CURRENT_ADC", "RBF_PIN", "BREAKWIRE", "CORE1",
    "LINK_POS", "LINK_NAV", "RESET_POS", "RESET_NAV"
};

// ---- Position --------------------------------------------------------
enum {
    PT_I2C_BUS = 0, PT_IMU_ID, PT_IMU_DATA, PT_BARO_ID, PT_BARO_DATA,
    PT_MAG_ID, PT_MAG_DATA, PT_LIDAR, PT_GPS_NMEA, PT_GPS_FIX, PT_UWB_ID,
    PT_UWB_RANGE, PT_CORE1, PT_LINK_MASTER, PT_LINK_NAV, PT_N
};
static const char *const POSITION_TEST_NAME[PT_N] = {
    "I2C_BUS", "IMU_ID", "IMU_DATA", "BARO_ID", "BARO_DATA",
    "MAG_ID", "MAG_DATA", "LIDAR", "GPS_NMEA", "GPS_FIX", "UWB_ID",
    "UWB_RANGE", "CORE1", "LINK_MASTER", "LINK_NAV"
};

// ---- Navigation ------------------------------------------------------
enum {
    NT_CONT_1 = 0, NT_CONT_2, NT_CONT_3, NT_PYRO_OUT, NT_I2C_D4D5,
    NT_CORE1, NT_LINK_MASTER, NT_LINK_POS, NT_N
};
static const char *const NAVIGATION_TEST_NAME[NT_N] = {
    "CONT_1", "CONT_2", "CONT_3", "PYRO_OUT", "I2C_D4D5",
    "CORE1", "LINK_MASTER", "LINK_POS"
};

static const char *const *const BENCH_TEST_NAMES[NODE_N] = {
    nullptr, MASTER_TEST_NAME, POSITION_TEST_NAME, NAVIGATION_TEST_NAME
};
static const uint8_t BENCH_TEST_COUNT[NODE_N] = { 0, MT_N, PT_N, NT_N };

// =====================================================================
//  2. CONSOLE
// =====================================================================
static void bench_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void bench_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    // USB CDC drops the bytes when no terminal is open, so this never
    // waits for a laptop that is not there.
    Serial.write((const uint8_t *)buf, (size_t)n);
}

static inline float bench_t_s(void) { return (float)millis() * 0.001f; }

// =====================================================================
//  3. TEST REGISTRY
// =====================================================================
typedef struct {
    uint8_t st;
    char    detail[72];
} bench_test_t;

static bench_test_t g_test[BENCH_MAX_TESTS];
static uint8_t      g_me = NODE_NONE;

static void bench_tests_init(uint8_t me)
{
    g_me = me;
    memset(g_test, 0, sizeof(g_test));
}

/**
 * Record a result. The detail is always updated; a line is printed only
 * when the VERDICT changes, so a steady board is quiet and a flapping
 * sensor is loud.
 */
static void bench_set(uint8_t id, uint8_t st, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void bench_set(uint8_t id, uint8_t st, const char *fmt, ...)
{
    if (id >= BENCH_TEST_COUNT[g_me]) return;
    bench_test_t *t = &g_test[id];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(t->detail, sizeof(t->detail), fmt, ap);
    va_end(ap);
    if (t->st != st) {
        t->st = st;
        bench_printf("[%8.2f s] %-4s %-12s %s\n", (double)bench_t_s(),
                     BENCH_ST_NAME[st], BENCH_TEST_NAMES[g_me][id], t->detail);
    }
}

static void bench_print_table(void)
{
    const uint8_t n = BENCH_TEST_COUNT[g_me];
    uint8_t cnt[6] = { 0 };
    bench_printf("---- %s self test @ %.1f s ----\n", BENCH_NODE_NAME[g_me],
                 (double)bench_t_s());
    for (uint8_t i = 0; i < n; i++) {
        cnt[g_test[i].st < 6 ? g_test[i].st : 0]++;
        bench_printf("  %-4s %-12s %s\n", BENCH_ST_NAME[g_test[i].st],
                     BENCH_TEST_NAMES[g_me][i], g_test[i].detail);
    }
    bench_printf("  => pass %u  warn %u  fail %u  pending %u  skip %u  info %u\n",
                 cnt[ST_PASS], cnt[ST_WARN], cnt[ST_FAIL], cnt[ST_PEND],
                 cnt[ST_SKIP], cnt[ST_INFO]);
}

/** Worst verdict on this board: drives the RGB LED. */
static uint8_t bench_worst(void)
{
    bool warn = false, pend = false;
    for (uint8_t i = 0; i < BENCH_TEST_COUNT[g_me]; i++) {
        if (g_test[i].st == ST_FAIL) return ST_FAIL;
        if (g_test[i].st == ST_WARN) warn = true;
        if (g_test[i].st == ST_PEND) pend = true;
    }
    if (warn) return ST_WARN;
    if (pend) return ST_PEND;
    return ST_PASS;
}

// =====================================================================
//  4. BENCH LINK — framing
//
//    B7 5E | type | len | payload (len <= 26) | crc16 lo | crc16 hi
//
//  A whole frame is at most 32 bytes: exactly one UART hardware FIFO.
//  It is only written when the FIFO is EMPTY, so uart_putc never waits.
// =====================================================================
#define BENCH_SYNC0        0xB7
#define BENCH_SYNC1        0x5E
#define BENCH_PAYLOAD_MAX  26
#define BENCH_FRAME_MAX    (4 + BENCH_PAYLOAD_MAX + 2)
#define BENCH_TXQ          8

// Sync bytes of the FLIGHT protocol, only to recognise it.
#define FLIGHT_SYNC0       0xA5
#define FLIGHT_SYNC1       0x1D

enum { PKT_PING = 0x61, PKT_PONG = 0x62, PKT_REPORT = 0x63 };

static uint16_t bench_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFF;                    // CRC-16/CCITT-FALSE
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

typedef struct {
    SerialUART  *port;
    uart_inst_t *hw;
    const char  *name;
    // receive
    uint8_t  rb[BENCH_FRAME_MAX];
    uint8_t  rn;
    uint8_t  prev;
    // transmit queue, frames waiting for an empty FIFO
    uint8_t  q[BENCH_TXQ][BENCH_FRAME_MAX];
    uint8_t  qlen[BENCH_TXQ];
    uint8_t  qh, qt;
    // counters
    uint32_t rx_bytes, rx_ok, rx_crc, rx_flight, tx_ok, tx_drop;
} bench_port_t;

static void bench_port_begin(bench_port_t *p, SerialUART &port, uart_inst_t *hw,
                             uint8_t tx_pin, uint8_t rx_pin, const char *name)
{
    memset(p, 0, sizeof(*p));
    p->port = &port;
    p->hw   = hw;
    p->name = name;
    port.setTX(tx_pin);
    port.setRX(rx_pin);
    port.setFIFOSize(512);          // software RX buffer: a busy loop loses nothing
    port.begin(BENCH_MCU_BAUD);
}

/** Queue one frame. Never waits: a full queue is counted and dropped. */
static bool bench_send(bench_port_t *p, uint8_t type, const void *payload, uint8_t len)
{
    if (len > BENCH_PAYLOAD_MAX) return false;
    uint8_t next = (uint8_t)((p->qh + 1) % BENCH_TXQ);
    if (next == p->qt) { p->tx_drop++; return false; }

    uint8_t *f = p->q[p->qh];
    f[0] = BENCH_SYNC0;
    f[1] = BENCH_SYNC1;
    f[2] = type;
    f[3] = len;
    if (len) memcpy(&f[4], payload, len);
    uint16_t crc = bench_crc16(&f[2], (uint16_t)(len + 2));
    f[4 + len] = (uint8_t)(crc & 0xFF);
    f[5 + len] = (uint8_t)(crc >> 8);
    p->qlen[p->qh] = (uint8_t)(len + 6);
    p->qh = next;
    return true;
}

/** Move at most one frame into the FIFO, and only into an EMPTY one. */
static void bench_port_service_tx(bench_port_t *p)
{
    if (p->qt == p->qh) return;
    if (!(uart_get_hw(p->hw)->fr & UART_UARTFR_TXFE_BITS)) return;   // still sending
    p->port->write(p->q[p->qt], p->qlen[p->qt]);                      // <= 32 B
    p->tx_ok++;
    p->qt = (uint8_t)((p->qt + 1) % BENCH_TXQ);
}

typedef void (*bench_rx_cb)(bench_port_t *p, uint8_t type, const uint8_t *pl, uint8_t len);

/** Drain the receive buffer, at most `budget` bytes per call. */
static void bench_port_poll(bench_port_t *p, bench_rx_cb cb, uint16_t budget = 256)
{
    while (budget-- && p->port->available()) {
        uint8_t b = (uint8_t)p->port->read();
        p->rx_bytes++;
        if (p->prev == FLIGHT_SYNC0 && b == FLIGHT_SYNC1) p->rx_flight++;
        p->prev = b;

        if (p->rn == 0) {
            if (b == BENCH_SYNC0) p->rb[p->rn++] = b;
            continue;
        }
        if (p->rn == 1) {
            if (b == BENCH_SYNC1)      p->rb[p->rn++] = b;
            else if (b != BENCH_SYNC0) p->rn = 0;
            continue;
        }
        p->rb[p->rn++] = b;
        if (p->rn == 4 && p->rb[3] > BENCH_PAYLOAD_MAX) {   // impossible length
            p->rx_crc++;
            p->rn = 0;
            continue;
        }
        if (p->rn >= 4 && p->rn == p->rb[3] + 6) {
            uint8_t len = p->rb[3];
            uint16_t got = (uint16_t)(p->rb[4 + len] | (p->rb[5 + len] << 8));
            if (bench_crc16(&p->rb[2], (uint16_t)(len + 2)) == got) {
                p->rx_ok++;
                cb(p, p->rb[2], &p->rb[4], len);
            } else {
                p->rx_crc++;
            }
            p->rn = 0;
        }
    }
}

// =====================================================================
//  5. PING / PONG
//
//  Every board pings every other board at 10 Hz. Position and Navigation
//  are not wired to each other, so their pings go THROUGH the Master,
//  which routes on `dst` — that is the same path the navigation state
//  and the actuator commands take in flight.
//
//  The payload carries a pattern derived from the sequence number, so a
//  frame that passes its CRC but was corrupted in a buffer is caught too.
// =====================================================================
typedef struct __attribute__((packed)) {
    uint8_t  ver;
    uint8_t  src;           // who sent THIS packet
    uint8_t  dst;           // final destination
    uint8_t  hops;          // incremented by the Master when it routes
    uint32_t seq;
    uint32_t t_origin_us;   // the pinger's micros(), echoed back untouched
    uint32_t uptime_ms;     // millis() of the sender of THIS packet
    uint8_t  pattern[8];
} bench_ping_t;             // 24 bytes

typedef struct __attribute__((packed)) {
    uint8_t  ver;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  n_tests;
    uint32_t uptime_ms;
    uint32_t loop_max_us;
    int16_t  temp_dc;       // die temperature, 0.1 degC
    uint8_t  st[8];         // 16 verdicts, one nibble each
    uint8_t  _pad[2];
} bench_report_t;           // 24 bytes

static_assert(sizeof(bench_ping_t)   <= BENCH_PAYLOAD_MAX, "ping too big for one FIFO");
static_assert(sizeof(bench_report_t) <= BENCH_PAYLOAD_MAX, "report too big for one FIFO");

typedef struct {
    bool     seen;
    uint32_t last_rx_ms;     // any bench packet from this node
    uint32_t uptime_ms;      // its own clock, from its last packet
    uint32_t reboots;        // uptime went backwards: it was reset
    uint32_t ping_tx, pong_rx, ping_answered, bad_payload, bad_version;
    uint32_t win_tx, win_rx; // since the last summary
    uint32_t rtt_last_us, rtt_min_us, rtt_max_us;
    uint64_t rtt_sum_us;
    uint32_t last_pong_ms;
    bool     have_report;
    bench_report_t report;
} bench_peer_t;

static bench_peer_t g_peer[NODE_N];
static uint32_t     g_ping_seq = 0;

static inline void bench_pattern(uint8_t *pat, uint32_t seq, uint8_t origin)
{
    for (uint8_t i = 0; i < 8; i++) pat[i] = (uint8_t)(seq * 13u + i * 29u + origin * 101u);
}

static bool bench_pattern_ok(const uint8_t *pat, uint32_t seq, uint8_t origin)
{
    uint8_t want[8];
    bench_pattern(want, seq, origin);
    return memcmp(want, pat, 8) == 0;
}

/** Called for every packet from a node: tracks liveness and reboots. */
static void bench_note_peer(uint8_t node, uint32_t uptime_ms)
{
    if (node == NODE_NONE || node >= NODE_N || node == g_me) return;
    bench_peer_t *pe = &g_peer[node];
    if (pe->seen && uptime_ms + 500u < pe->uptime_ms) {
        pe->reboots++;
        bench_printf("[%8.2f s] EVENT %s rebooted (uptime %lu ms -> %lu ms)\n",
                     (double)bench_t_s(), BENCH_NODE_NAME[node],
                     (unsigned long)pe->uptime_ms, (unsigned long)uptime_ms);
    }
    pe->seen       = true;
    pe->uptime_ms  = uptime_ms;
    pe->last_rx_ms = millis();
}

static void bench_ping_build(bench_ping_t *pk, uint8_t dst)
{
    memset(pk, 0, sizeof(*pk));
    pk->ver         = BENCH_PROTO_VERSION;
    pk->src         = g_me;
    pk->dst         = dst;
    pk->seq         = ++g_ping_seq;
    pk->t_origin_us = micros();
    pk->uptime_ms   = millis();
    bench_pattern(pk->pattern, pk->seq, g_me);
    g_peer[dst].ping_tx++;
    g_peer[dst].win_tx++;
}

/**
 * Handle a ping / pong / report addressed to THIS board. Returns a pong
 * to send in *reply (and true) when the packet was a ping.
 */
static bool bench_handle(uint8_t type, const uint8_t *pl, uint8_t len, bench_ping_t *reply)
{
    if (type == PKT_PING || type == PKT_PONG) {
        if (len != sizeof(bench_ping_t)) return false;
        bench_ping_t pk;
        memcpy(&pk, pl, sizeof(pk));
        if (pk.src >= NODE_N || pk.dst != g_me) return false;
        bench_peer_t *pe = &g_peer[pk.src];
        if (pk.ver != BENCH_PROTO_VERSION) { pe->bad_version++; return false; }
        bench_note_peer(pk.src, pk.uptime_ms);

        if (type == PKT_PING) {
            if (!bench_pattern_ok(pk.pattern, pk.seq, pk.src)) pe->bad_payload++;
            *reply = pk;                     // seq, t_origin, pattern echoed
            reply->src       = g_me;
            reply->dst       = pk.src;
            reply->hops      = 0;
            reply->uptime_ms = millis();
            pe->ping_answered++;
            return true;
        }

        // PONG: the pattern was made by US, with our node id
        if (!bench_pattern_ok(pk.pattern, pk.seq, g_me)) { pe->bad_payload++; return false; }
        uint32_t rtt = micros() - pk.t_origin_us;
        pe->pong_rx++;
        pe->win_rx++;
        pe->last_pong_ms = millis();
        pe->rtt_last_us  = rtt;
        pe->rtt_sum_us  += rtt;
        if (pe->rtt_min_us == 0 || rtt < pe->rtt_min_us) pe->rtt_min_us = rtt;
        if (rtt > pe->rtt_max_us) pe->rtt_max_us = rtt;
        return false;
    }

    if (type == PKT_REPORT && len == sizeof(bench_report_t)) {
        bench_report_t r;
        memcpy(&r, pl, sizeof(r));
        if (r.src >= NODE_N || r.src == NODE_NONE) return false;
        if (r.ver != BENCH_PROTO_VERSION) { g_peer[r.src].bad_version++; return false; }
        bench_note_peer(r.src, r.uptime_ms);
        g_peer[r.src].report      = r;
        g_peer[r.src].have_report = true;
    }
    return false;
}

static void bench_report_build(bench_report_t *r, uint8_t dst, uint32_t loop_max_us)
{
    memset(r, 0, sizeof(*r));
    r->ver         = BENCH_PROTO_VERSION;
    r->src         = g_me;
    r->dst         = dst;
    r->n_tests     = BENCH_TEST_COUNT[g_me];
    r->uptime_ms   = millis();
    r->loop_max_us = loop_max_us;
    r->temp_dc     = (int16_t)(analogReadTemp() * 10.0f);
    for (uint8_t i = 0; i < r->n_tests && i < 16; i++)
        r->st[i / 2] |= (uint8_t)((g_test[i].st & 0x0F) << ((i & 1) * 4));
}

static inline uint8_t bench_report_st(const bench_report_t *r, uint8_t i)
{
    return (uint8_t)((r->st[i / 2] >> ((i & 1) * 4)) & 0x0F);
}

/**
 * Turn a peer's ping statistics into a verdict for a LINK test.
 * Grace period first, so a board that boots a second later is not a FAIL.
 */
static void bench_link_verdict(uint8_t test_id, uint8_t node, const bench_port_t *via)
{
    bench_peer_t *pe = &g_peer[node];
    const uint32_t now = millis();
    const uint32_t avg = pe->pong_rx ? (uint32_t)(pe->rtt_sum_us / pe->pong_rx) : 0;
    const uint32_t loss_pct = pe->win_tx ? 100u - (100u * (pe->win_rx > pe->win_tx ? pe->win_tx : pe->win_rx)) / pe->win_tx : 0;

    if (pe->pong_rx && now - pe->last_pong_ms < 1500) {
        bench_set(test_id, loss_pct > 10 || pe->bad_payload ? ST_WARN : ST_PASS,
                  "rtt %lu/%lu/%lu us (min/avg/max) loss %lu%% bad %lu",
                  (unsigned long)pe->rtt_min_us, (unsigned long)avg,
                  (unsigned long)pe->rtt_max_us, (unsigned long)loss_pct,
                  (unsigned long)pe->bad_payload);
    } else if (now < 4000) {
        bench_set(test_id, ST_PEND, "waiting for %s...", BENCH_NODE_NAME[node]);
    } else if (via && via->rx_flight > 20 && via->rx_ok == 0) {
        bench_set(test_id, ST_FAIL, "%s runs FLIGHT firmware - flash its bench test",
                  via->name);
    } else if (pe->bad_version) {
        bench_set(test_id, ST_FAIL, "%s: bench protocol version mismatch",
                  BENCH_NODE_NAME[node]);
    } else if (via && via->rx_bytes == 0) {
        bench_set(test_id, ST_FAIL, "no bytes at all on %s: wiring TX/RX, GND, baud?",
                  via->name);
    } else {
        bench_set(test_id, ST_FAIL, "no pong from %s for %lu ms (rx %lu B, crc err %lu)",
                  BENCH_NODE_NAME[node],
                  (unsigned long)(pe->pong_rx ? now - pe->last_pong_ms : now),
                  via ? (unsigned long)via->rx_bytes : 0ul,
                  via ? (unsigned long)via->rx_crc : 0ul);
    }
}

static void bench_print_port(const bench_port_t *p)
{
    bench_printf("  port %-10s rx %lu B, ok %lu, crc %lu, flight-sync %lu | tx %lu, drop %lu\n",
                 p->name, (unsigned long)p->rx_bytes, (unsigned long)p->rx_ok,
                 (unsigned long)p->rx_crc, (unsigned long)p->rx_flight,
                 (unsigned long)p->tx_ok, (unsigned long)p->tx_drop);
}

static void bench_print_peer(uint8_t node)
{
    const bench_peer_t *pe = &g_peer[node];
    bench_printf("  peer %-10s %s  ping %lu pong %lu answered %lu bad %lu  uptime %lu ms reboots %lu\n",
                 BENCH_NODE_NAME[node],
                 pe->seen && millis() - pe->last_rx_ms < 1500 ? "ALIVE" : "DEAD ",
                 (unsigned long)pe->ping_tx, (unsigned long)pe->pong_rx,
                 (unsigned long)pe->ping_answered, (unsigned long)pe->bad_payload,
                 (unsigned long)pe->uptime_ms, (unsigned long)pe->reboots);
}

static void bench_window_reset(void)
{
    for (uint8_t i = 0; i < NODE_N; i++) g_peer[i].win_tx = g_peer[i].win_rx = 0;
}

// =====================================================================
//  6. CORE 1 HEARTBEAT and LOOP TIMING
//
//  Core 1 does nothing but count. If the counter stops, the second core
//  is not running — every flight sketch puts its blocking work there.
// =====================================================================
static volatile uint32_t g_core1_count = 0;

static void bench_core1_tick(void) { g_core1_count++; delayMicroseconds(100); }

static void bench_core1_verdict(uint8_t test_id)
{
    static uint32_t last = 0;
    uint32_t c = g_core1_count;
    if (c != last) bench_set(test_id, ST_PASS, "core 1 running (%lu ticks)", (unsigned long)c);
    else           bench_set(test_id, millis() < 2000 ? ST_PEND : ST_FAIL, "core 1 counter stopped");
    last = c;
}

/** Longest single loop() pass, in us. Proves that nothing blocks. */
typedef struct {
    uint32_t start_us, max_us, max_ever_us, count;
} bench_looptime_t;

static inline void bench_loop_begin(bench_looptime_t *l) { l->start_us = micros(); }
static inline void bench_loop_end(bench_looptime_t *l)
{
    uint32_t d = micros() - l->start_us;
    if (d > l->max_us) l->max_us = d;
    if (d > l->max_ever_us && millis() > 3000) l->max_ever_us = d;
    l->count++;
}

#endif // BENCH_COMMON_H
