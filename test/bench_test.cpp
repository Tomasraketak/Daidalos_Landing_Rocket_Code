// =====================================================================
//  Host test of the bench-test link, Ariadna_V2/Bench_Tests/common/
//  bench_common.h. `make bench`.
//
//  The bench sketches exist to tell a broken wire from a broken board, so
//  their own protocol has to be beyond suspicion: a bug here would show up
//  on the bench as a "bad link" that is not there. Checked:
//
//    framing      round trip, corrupted byte, impossible length, garbage
//    flight sync  a board still on flight firmware is recognised
//    transmit     never writes into a non-empty FIFO, drops when full
//    ping/pong    as two boards, including the Master's hop rewrite
//    payload      a corrupted pattern is caught even with a good CRC
//    reboot       uptime going backwards is counted once
//    report       16 verdicts survive the nibble packing
// =====================================================================
#include "Arduino.h"
#include "../Ariadna_V2/Bench_Tests/common/bench_common.h"

#include <stdio.h>

uint32_t  g_fake_us = 0;
uart_hw_t g_fake_uart_hw = { UART_UARTFR_TXFE_BITS };
USBStub   Serial;

static int s_fail = 0, s_checks = 0;
#define CHECK(cond, ...) do { s_checks++; if (!(cond)) { s_fail++; \
    printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---- capture whatever the parser delivers ---------------------------
struct Rx { uint8_t type; uint8_t len; uint8_t pl[BENCH_PAYLOAD_MAX]; };
static std::vector<Rx> s_got;
static void capture(bench_port_t *, uint8_t type, const uint8_t *pl, uint8_t len)
{
    Rx r; r.type = type; r.len = len; memcpy(r.pl, pl, len);
    s_got.push_back(r);
}

static SerialUART s_uart;
static bench_port_t s_port;

/** Send through the TX queue and FIFO model, collect the wire bytes. */
static std::vector<uint8_t> wire_frame(uint8_t type, const void *pl, uint8_t len)
{
    s_uart.out.clear();
    bench_send(&s_port, type, pl, len);
    bench_port_service_tx(&s_port);
    return s_uart.out;
}

static void feed(const std::vector<uint8_t> &b)
{
    for (uint8_t x : b) s_uart.in.push_back(x);
    bench_port_poll(&s_port, capture, 4096);
}

int main()
{
    bench_tests_init(NODE_POSITION);
    bench_port_begin(&s_port, s_uart, nullptr, 0, 1, "test");

    // ---- framing -----------------------------------------------------
    {
        const uint8_t pl[5] = { 1, 2, 3, 0xB7, 0x5E };    // sync bytes inside the payload
        auto f = wire_frame(0x42, pl, 5);
        CHECK(f.size() == 11, "frame size %zu", f.size());
        s_got.clear();
        feed(f);
        CHECK(s_got.size() == 1 && s_got[0].type == 0x42 && s_got[0].len == 5 &&
              memcmp(s_got[0].pl, pl, 5) == 0, "round trip");

        // a corrupted byte loses that frame and only that frame
        auto bad = f; bad[6] ^= 0x10;
        s_got.clear();
        uint32_t crc0 = s_port.rx_crc;
        feed(bad); feed(f);
        CHECK(s_got.size() == 1 && s_port.rx_crc == crc0 + 1, "crc reject then recover (%zu)", s_got.size());

        // impossible length, then garbage, then a good frame
        s_got.clear();
        feed({ BENCH_SYNC0, BENCH_SYNC1, 0x42, 200, 0, 0, 0, 0x13, 0xB7, 0x00, 0xFF });
        feed(f);
        CHECK(s_got.size() == 1, "resync after bad length (%zu)", s_got.size());

        // a full-size payload is exactly one FIFO
        uint8_t big[BENCH_PAYLOAD_MAX];
        for (uint8_t i = 0; i < sizeof(big); i++) big[i] = (uint8_t)(i * 3);
        auto fb = wire_frame(0x43, big, sizeof(big));
        CHECK(fb.size() == 32, "max frame %zu bytes, want 32", fb.size());
        CHECK(!bench_send(&s_port, 0x44, big, BENCH_PAYLOAD_MAX + 1), "oversize payload refused");
    }

    // ---- flight-firmware recognition ---------------------------------
    {
        uint32_t fl0 = s_port.rx_flight;
        feed({ 0xA5, 0x1D, 0x10, 0x02, 0x00, 0x00, 0x12, 0x34 });
        CHECK(s_port.rx_flight == fl0 + 1, "flight sync counted");
    }

    // ---- transmit discipline -----------------------------------------
    {
        uint8_t pl[4] = { 9, 9, 9, 9 };
        s_uart.out.clear();
        g_fake_uart_hw.fr = 0;                             // FIFO busy
        bench_send(&s_port, 0x50, pl, 4);
        bench_port_service_tx(&s_port);
        CHECK(s_uart.out.empty(), "wrote into a busy FIFO");
        uint32_t drop0 = s_port.tx_drop;
        for (int i = 0; i < BENCH_TXQ + 3; i++) bench_send(&s_port, 0x50, pl, 4);
        CHECK(s_port.tx_drop > drop0, "full queue drops instead of waiting");
        g_fake_uart_hw.fr = UART_UARTFR_TXFE_BITS;         // FIFO empty again
        bench_port_service_tx(&s_port);
        CHECK(s_uart.out.size() == 10, "one frame per empty FIFO (%zu)", s_uart.out.size());
        for (int i = 0; i < 20; i++) bench_port_service_tx(&s_port);   // drain
        s_uart.out.clear();
    }

    // ---- ping / pong between two boards, through the Master's hop ------
    {
        g_fake_us = 5000000;
        bench_ping_t ping;
        bench_ping_build(&ping, NODE_NAVIGATION);          // as POSITION
        CHECK(g_peer[NODE_NAVIGATION].ping_tx == 1, "ping counted");

        ping.hops++;                                       // the Master routes it

        g_me = NODE_NAVIGATION;                            // now be the other board
        g_fake_us += 700;
        bench_ping_t pong;
        bool reply = bench_handle(PKT_PING, (const uint8_t *)&ping, sizeof(ping), &pong);
        CHECK(reply && pong.src == NODE_NAVIGATION && pong.dst == NODE_POSITION &&
              pong.seq == ping.seq, "pong built");
        CHECK(g_peer[NODE_POSITION].bad_payload == 0, "pattern survives the hop rewrite");

        g_me = NODE_POSITION;
        g_fake_us += 700;
        pong.hops++;
        bench_ping_t none;
        CHECK(!bench_handle(PKT_PONG, (const uint8_t *)&pong, sizeof(pong), &none), "pong needs no reply");
        const bench_peer_t *pe = &g_peer[NODE_NAVIGATION];
        CHECK(pe->pong_rx == 1 && pe->rtt_last_us == 1400, "rtt %u us, pongs %u",
              (unsigned)pe->rtt_last_us, (unsigned)pe->pong_rx);

        // the same pong, with a corrupted pattern, is rejected
        pong.pattern[3] ^= 1;
        bench_handle(PKT_PONG, (const uint8_t *)&pong, sizeof(pong), &none);
        CHECK(pe->pong_rx == 1 && pe->bad_payload == 1, "bad pattern caught");

        // a pong for someone else is ignored
        pong.pattern[3] ^= 1;
        pong.dst = NODE_MASTER;
        bench_handle(PKT_PONG, (const uint8_t *)&pong, sizeof(pong), &none);
        CHECK(pe->pong_rx == 1, "misaddressed pong ignored");

        // wrong protocol version is reported, not decoded
        bench_ping_t old = ping; old.ver = 99; old.dst = NODE_POSITION; old.src = NODE_MASTER;
        bench_handle(PKT_PING, (const uint8_t *)&old, sizeof(old), &none);
        CHECK(g_peer[NODE_MASTER].bad_version == 1, "version mismatch counted");
    }

    // ---- reboot detection --------------------------------------------
    {
        uint32_t r0 = g_peer[NODE_MASTER].reboots;
        bench_note_peer(NODE_MASTER, 20000);
        bench_note_peer(NODE_MASTER, 20100);
        bench_note_peer(NODE_MASTER, 20050);               // reordering jitter: not a reboot
        CHECK(g_peer[NODE_MASTER].reboots == r0, "jitter is not a reboot");
        bench_note_peer(NODE_MASTER, 150);
        CHECK(g_peer[NODE_MASTER].reboots == r0 + 1, "reboot counted");
    }

    // ---- report packing ----------------------------------------------
    {
        for (uint8_t i = 0; i < PT_N; i++) g_test[i].st = (uint8_t)(i % 6);
        bench_report_t r;
        bench_report_build(&r, NODE_MASTER, 1234);
        CHECK(r.n_tests == PT_N && r.loop_max_us == 1234 && r.temp_dc == 250, "report header");
        bool all = true;
        for (uint8_t i = 0; i < PT_N; i++) if (bench_report_st(&r, i) != i % 6) all = false;
        CHECK(all, "verdicts survive nibble packing");

        bench_ping_t none;
        g_me = NODE_MASTER;
        bench_handle(PKT_REPORT, (const uint8_t *)&r, sizeof(r), &none);
        CHECK(g_peer[NODE_POSITION].have_report &&
              bench_report_st(&g_peer[NODE_POSITION].report, 5) == 5, "Master stores the report");
    }

    // ---- name tables agree with the enums ----------------------------
    CHECK(MT_N <= BENCH_MAX_TESTS && PT_N <= BENCH_MAX_TESTS && NT_N <= BENCH_MAX_TESTS,
          "too many tests for the report");

    printf("bench link: %d checks, %d failed\n", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
