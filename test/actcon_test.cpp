// =====================================================================
//  Host test: Actuator Console pyro bridge (Ariadna_V2/common/actcon_pyro.h)
//
//  The rules that keep an igniter from firing when it should not are
//  checked here, on a workstation, rather than discovered on the pad:
//
//    - nothing fires unless armed
//    - arming expires, heartbeat loss disarms and cuts a pulse
//    - one pulse per command even though the wire carries it three times
//    - pulses are clamped, one channel at a time, every pulse ends disarmed
//    - millis() wrap-around does not stretch or shorten anything
//    - the frames survive the shared Ariadna codec both ways
//
//  Run:  cd test && make actcon
// =====================================================================
#include <stdio.h>
#include <stdlib.h>
#include "../Ariadna_V2/common/ariadna_link.h"
#include "../Ariadna_V2/common/actcon_pyro.h"

static int s_fail = 0, s_pass = 0;
#define CHECK(cond) do { if (cond) s_pass++; else { s_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static actcon_pyro_cmd_t mk(uint8_t cmd, uint32_t seq, uint8_t ch = 0, uint16_t ms = 0)
{
    actcon_pyro_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.cmd = cmd; c.seq = seq; c.ch = ch; c.pulse_ms = ms;
    return c;
}

static void hb(actcon_pyro_sm_t *sm, uint32_t now)
{
    actcon_pyro_cmd_t c = mk(ACTCON_CMD_HEARTBEAT, 0);
    actcon_pyro_command(sm, &c, now);
}

/** Run the machine from t0 to t1 with a heartbeat every 100 ms; returns
 *  the number of ms any output was HIGH and the last mask seen. */
static uint32_t run(actcon_pyro_sm_t *sm, uint32_t t0, uint32_t t1, bool beat, uint8_t *mask_seen)
{
    uint32_t high = 0;
    uint8_t seen = 0;
    for (uint32_t t = t0; t != t1; t++) {
        if (beat && (t - t0) % ACTCON_HB_PERIOD_MS == 0) hb(sm, t);
        const uint8_t m = actcon_pyro_tick(sm, t);
        if (m) { high++; seen |= m; }
    }
    if (mask_seen) *mask_seen = seen;
    return high;
}

static void test_fire_needs_arm(uint32_t base)
{
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    actcon_pyro_cmd_t f = mk(ACTCON_CMD_FIRE, 1, 1, 200);
    CHECK(actcon_pyro_command(&sm, &f, base) == ACTCON_REJ_NOT_ARMED);
    CHECK(actcon_pyro_tick(&sm, base) == 0);
    CHECK(run(&sm, base, base + 1000, true, nullptr) == 0);
}

static void test_normal_fire(uint32_t base)
{
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    actcon_pyro_cmd_t a = mk(ACTCON_CMD_ARM, 1);
    CHECK(actcon_pyro_command(&sm, &a, base) == ACTCON_OK);
    CHECK(sm.armed == 1);
    CHECK(actcon_arm_left_ds(&sm, base) == ACTCON_ARM_TIMEOUT_MS / 100);

    actcon_pyro_cmd_t f = mk(ACTCON_CMD_FIRE, 2, 2, 300);
    // The same frame three times, as the Master sends it.
    CHECK(actcon_pyro_command(&sm, &f, base + 10) == ACTCON_OK);
    CHECK(actcon_pyro_command(&sm, &f, base + 10) == ACTCON_OK);
    CHECK(actcon_pyro_command(&sm, &f, base + 11) == ACTCON_OK);

    uint8_t seen = 0;
    const uint32_t high = run(&sm, base + 10, base + 2000, true, &seen);
    CHECK(high == 300);               // exactly one pulse of the asked length
    CHECK(seen == 0x02);              // only channel 2
    CHECK(sm.armed == 0);             // every pulse ends disarmed
    CHECK(sm.fired_mask == 0x02);

    // A late repeat of the same FIRE (seq 2) does nothing.
    CHECK(actcon_pyro_command(&sm, &f, base + 2000) == ACTCON_OK);
    CHECK(actcon_pyro_tick(&sm, base + 2000) == 0);

    // A new FIRE without a new ARM is refused.
    actcon_pyro_cmd_t f2 = mk(ACTCON_CMD_FIRE, 3, 2, 300);
    CHECK(actcon_pyro_command(&sm, &f2, base + 2001) == ACTCON_REJ_NOT_ARMED);
}

static void test_clamp_and_channels(uint32_t base)
{
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    uint32_t seq = 10;

    actcon_pyro_cmd_t a = mk(ACTCON_CMD_ARM, seq++);
    actcon_pyro_command(&sm, &a, base);
    actcon_pyro_cmd_t bad0 = mk(ACTCON_CMD_FIRE, seq++, 0, 100);
    actcon_pyro_cmd_t bad4 = mk(ACTCON_CMD_FIRE, seq++, 4, 100);
    CHECK(actcon_pyro_command(&sm, &bad0, base) == ACTCON_REJ_BAD_CHANNEL);
    CHECK(actcon_pyro_command(&sm, &bad4, base) == ACTCON_REJ_BAD_CHANNEL);
    CHECK(sm.armed == 1);

    // Too long -> clamped to the maximum.
    actcon_pyro_cmd_t lng = mk(ACTCON_CMD_FIRE, seq++, 1, 60000);
    CHECK(actcon_pyro_command(&sm, &lng, base) == ACTCON_OK);
    // A second channel while the first is HIGH is refused.
    actcon_pyro_cmd_t other = mk(ACTCON_CMD_FIRE, seq++, 3, 100);
    CHECK(actcon_pyro_command(&sm, &other, base + 1) == ACTCON_REJ_BUSY);
    uint8_t seen = 0;
    CHECK(run(&sm, base, base + 3000, true, &seen) == ACTCON_PULSE_MAX_MS);
    CHECK(seen == 0x01);

    // Too short -> clamped to the minimum; 0 -> the default.
    a = mk(ACTCON_CMD_ARM, seq++);
    actcon_pyro_command(&sm, &a, base + 3000);
    actcon_pyro_cmd_t sh = mk(ACTCON_CMD_FIRE, seq++, 3, 1);
    actcon_pyro_command(&sm, &sh, base + 3000);
    CHECK(run(&sm, base + 3000, base + 4000, true, nullptr) == ACTCON_PULSE_MIN_MS);

    a = mk(ACTCON_CMD_ARM, seq++);
    actcon_pyro_command(&sm, &a, base + 4000);
    actcon_pyro_cmd_t df = mk(ACTCON_CMD_FIRE, seq++, 3, 0);
    actcon_pyro_command(&sm, &df, base + 4000);
    CHECK(run(&sm, base + 4000, base + 5000, true, nullptr) == ACTCON_PULSE_DEFAULT_MS);
}

static void test_arm_timeout(uint32_t base)
{
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    actcon_pyro_cmd_t a = mk(ACTCON_CMD_ARM, 1);
    actcon_pyro_command(&sm, &a, base);
    run(&sm, base, base + ACTCON_ARM_TIMEOUT_MS - 1, true, nullptr);
    CHECK(sm.armed == 1);
    run(&sm, base + ACTCON_ARM_TIMEOUT_MS - 1, base + ACTCON_ARM_TIMEOUT_MS + 1, true, nullptr);
    CHECK(sm.armed == 0);
    actcon_pyro_cmd_t f = mk(ACTCON_CMD_FIRE, 2, 1, 100);
    CHECK(actcon_pyro_command(&sm, &f, base + ACTCON_ARM_TIMEOUT_MS + 2) == ACTCON_REJ_NOT_ARMED);
}

static void test_heartbeat_loss(uint32_t base)
{
    // Armed, then the Master goes quiet: disarmed after the timeout.
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    actcon_pyro_cmd_t a = mk(ACTCON_CMD_ARM, 1);
    actcon_pyro_command(&sm, &a, base);
    run(&sm, base, base + ACTCON_HB_TIMEOUT_MS - 1, false, nullptr);
    CHECK(sm.armed == 1);
    run(&sm, base + ACTCON_HB_TIMEOUT_MS - 1, base + ACTCON_HB_TIMEOUT_MS + 1, false, nullptr);
    CHECK(sm.armed == 0);
    CHECK(sm.link == 0);
    CHECK(sm.last_reject == ACTCON_REJ_NO_LINK);

    // Firing, then the Master goes quiet: the pulse is cut.
    actcon_pyro_init(&sm);
    hb(&sm, base);
    a = mk(ACTCON_CMD_ARM, 5);
    actcon_pyro_command(&sm, &a, base);
    actcon_pyro_cmd_t f = mk(ACTCON_CMD_FIRE, 6, 1, 1000);
    actcon_pyro_command(&sm, &f, base);       // last frame at `base`
    const uint32_t high = run(&sm, base, base + 2000, false, nullptr);
    CHECK(high == ACTCON_HB_TIMEOUT_MS);       // cut at the timeout, not at 1000 ms
}

static void test_disarm_cuts_pulse(uint32_t base)
{
    actcon_pyro_sm_t sm; actcon_pyro_init(&sm);
    hb(&sm, base);
    actcon_pyro_cmd_t a = mk(ACTCON_CMD_ARM, 1);
    actcon_pyro_command(&sm, &a, base);
    actcon_pyro_cmd_t f = mk(ACTCON_CMD_FIRE, 2, 1, 800);
    actcon_pyro_command(&sm, &f, base);
    uint32_t high = run(&sm, base, base + 100, true, nullptr);
    actcon_pyro_cmd_t d = mk(ACTCON_CMD_DISARM, 3);
    actcon_pyro_command(&sm, &d, base + 100);
    high += run(&sm, base + 100, base + 1000, true, nullptr);
    CHECK(high == 100);
    CHECK(sm.armed == 0);
}

static void test_codec(void)
{
    uint8_t frame[ARIADNA_FRAME_MAX];
    ariadna_rx_t rx; ariadna_rx_init(&rx);

    actcon_pyro_cmd_t c = mk(ACTCON_CMD_FIRE, 0xDEADBEEF, 3, 250);
    uint16_t n = ariadna_encode(frame, ACTCON_PKT_PYRO_CMD, &c, sizeof(c));
    CHECK(n == ARIADNA_HDR_LEN + sizeof(c) + ARIADNA_CRC_LEN);
    const uint8_t *pl = nullptr; uint8_t len = 0, type = 0;
    for (uint16_t i = 0; i < n; i++) type = ariadna_rx_byte(&rx, frame[i], &pl, &len);
    actcon_pyro_cmd_t back;
    CHECK(type == ACTCON_PKT_PYRO_CMD);
    CHECK(ariadna_unpack(pl, len, &back, sizeof(back)) == 1);
    CHECK(memcmp(&back, &c, sizeof(c)) == 0);

    // A flipped bit must not decode.
    frame[6] ^= 0x01;
    type = 0;
    for (uint16_t i = 0; i < n; i++) type |= ariadna_rx_byte(&rx, frame[i], &pl, &len);
    CHECK(type == 0);

    actcon_pyro_report_t r;
    memset(&r, 0, sizeof(r));
    r.raw[0] = 3000; r.raw[2] = 15; r.cont_mask = 1; r.armed = 1; r.arm_left_ds = 599;
    n = ariadna_encode(frame, ACTCON_PKT_PYRO_REPORT, &r, sizeof(r));
    type = 0;
    for (uint16_t i = 0; i < n; i++) {
        const uint8_t t = ariadna_rx_byte(&rx, frame[i], &pl, &len);
        if (t) type = t;
    }
    actcon_pyro_report_t rb;
    CHECK(type == ACTCON_PKT_PYRO_REPORT);
    CHECK(ariadna_unpack(pl, len, &rb, sizeof(rb)) == 1);
    CHECK(memcmp(&rb, &r, sizeof(r)) == 0);

    // The console's packet types never collide with the flight ones.
    CHECK(ACTCON_PKT_PYRO_CMD != ARIADNA_PKT_CMD && ACTCON_PKT_PYRO_CMD != ARIADNA_PKT_TELEMETRY);
    CHECK(ACTCON_PKT_PYRO_REPORT > ARIADNA_PKT_TELEMETRY);
}

int main(void)
{
    // Every timing test runs twice: from boot, and straddling the millis() wrap.
    const uint32_t bases[2] = { 1000u, 0xFFFFFFFFu - 1500u };
    for (uint32_t b : bases) {
        test_fire_needs_arm(b);
        test_normal_fire(b);
        test_clamp_and_channels(b);
        test_arm_timeout(b);
        test_heartbeat_loss(b);
        test_disarm_cuts_pulse(b);
    }
    test_codec();
    printf("actcon: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
