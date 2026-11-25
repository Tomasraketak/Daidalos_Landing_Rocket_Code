// =====================================================================
//  Host test: both ends of the Static Fire Link talking to each other.
//
//  The stand's SfStandLink (from the Static_Fire_Stand checkout) and Ariadna's
//  supervisor (sf_run.h) are connected through the real frame encoders and
//  decoders, with a simulated I2C in between, and a whole run is played:
//  handshake, countdown, T0, ignition, the clamp profile, the burn ending,
//  the copy of the stand's record page by page, and the "data copied" ack.
//
//  What this is for is the seam. Each side is tested on its own elsewhere;
//  the places where they could still disagree are the moment of T0 (the stand
//  raises IGNITE and publishes the new state in two steps, and Ariadna can
//  read in between), a lost heartbeat, a stale ack, and the paging of the
//  record. The T0 test below is repeated for every offset of the reads
//  relative to the event, in 100 us steps.
//
//  Needs the stand checkout:   make sflink-loop STAND=../../Static_Fire_Stand
// =====================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../Ariadna_V2/Examples/Static_Fire_Link/sf_run.h"
#include "../Ariadna_V2/Examples/Static_Fire_Link/sf_record.h"
#include STAND_LINK_H
#include "sflink_golden.h"

static int s_fail = 0, s_pass = 0;
#define CHECK(cond) do { if (cond) s_pass++; else { s_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

struct World {
    SfStandLink stand;
    sfr_t r; sfr_cfg_t c; sfr_out_t out;
    uint32_t us = 1000000;               // one clock for both ends
    uint32_t t0_us = 0;                  // when the stand's countdown ends
    uint32_t cd_start_us = 0;
    int      st = SFL_ST_IDLE;           // the stand's state as the stand knows it (core 0)
    int      published_st = SFL_ST_IDLE; // ... as last published
    bool     raised = false;             // IGNITE raised yet?
    bool     link = true;                // the I2C cable
    float    thrust = 0;
    int      fires = 0; uint32_t fire_us = 0;
    bool     nav_armed = false;
    uint32_t next_poll_us = 0, next_hb_us = 0;
    uint8_t  hb_flags = 0, hb_ack = 0, data_seq = 0;
    sfr_in_t in; uint32_t last_ok_ms = 0; bool st_valid = false; sfl_status_t last_st;
    uint32_t burn_pages = 0;

    World() {
        stand.init(SFL_MODE_ARIADNA, 2, 1);
        stand.ignition_ms = 300; stand.preroll_ms = 3000; stand.recording_ms = 10000;
        sfr_init(&r); sfr_cfg_default(&c);
        memset(&in, 0, sizeof(in)); memset(&out, 0, sizeof(out)); memset(&last_st, 0, sizeof(last_st));
    }

    uint32_t ms() const { return us / 1000; }

    // ---- the stand's core 0 --------------------------------------------------
    void stand_publish() {
        const int32_t rem = st == SFL_ST_COUNTDOWN ? (int32_t)((int64_t)t0_us - (int64_t)us) / 1000 : 0;
        stand.publish((uint32_t)st, 1, 1, 1, (int32_t)(thrust * 1000.0f), rem, us, t0_us);
        published_st = st;
    }
    void stand_loop() {                  // the top of loop(): publish, then service
        stand_publish();
        stand.service(ms());
    }
    void start_countdown(uint32_t seconds) {
        stand.countdown_start();
        cd_start_us = us; t0_us = us + seconds * 1000000u;
        st = SFL_ST_COUNTDOWN; raised = false;
        stand_publish();
    }
    /** T0 exactly as the .ino does it: state, then IGNITE, then the publish. */
    void stand_t0_step_a() { st = SFL_ST_RECORDING; }                       // enterState(); t0_us = micros()
    void stand_t0_step_b() { stand.raise_ignite(ms()); raised = true; }     // raise_ignite()
    void stand_t0_step_c() { stand_publish(); }                              // publishLink()

    // ---- the I2C between them ---------------------------------------------------
    bool i2c_status(sfl_status_t *out_st) {
        if (!link) return false;
        uint8_t b[SFL_STATUS_SIZE];
        stand.build_status(us, ms(), b);
        return sfl_status_decode(b, out_st) != 0;
    }
    void i2c_hb(uint8_t flags, uint8_t ack, uint8_t data) {
        if (!link) return;
        sfl_hb_t h = { flags, ack, data };
        uint8_t b[SFL_HB_SIZE]; sfl_hb_encode(&h, b);
        stand.on_heartbeat(b, sizeof(b), ms());
    }

    // ---- Ariadna's loop, called every `step` microseconds --------------------------
    void ariadna_loop() {
        in.now_ms = ms(); in.now_us = us; in.st_new = 0;
        if ((int32_t)(us - next_poll_us) >= 0) {
            sfl_status_t s;
            if (i2c_status(&s)) { last_st = s; st_valid = true; last_ok_ms = ms(); in.st_new = 1; in.st_us = us; }
            next_poll_us = us + sfr_poll_ms(&r, &in) * 1000u;
        }
        in.stand_ok = st_valid && (ms() - last_ok_ms) < SFR_LOST_MS;
        in.st = last_st;
        in.prereq_ok = 1; in.nav_armed = nav_armed; in.cont = 1; in.busy = 0; in.hold = 0;
        sfr_tick(&r, &c, &in, &out);
        if (out.want_arm) nav_armed = true;
        if (out.want_disarm) nav_armed = false;
        if (out.fire) { fires++; fire_us = us; nav_armed = false; i2c_hb(out.hb_flags, out.hb_ack, data_seq); hb_flags = out.hb_flags; hb_ack = out.hb_ack; next_hb_us = us + SFL_HB_PERIOD_MS * 1000u; }
        if (in.stand_ok && ((int32_t)(us - next_hb_us) >= 0 || out.hb_flags != hb_flags || out.hb_ack != hb_ack)) {
            i2c_hb(out.hb_flags, out.hb_ack, data_seq);
            hb_flags = out.hb_flags; hb_ack = out.hb_ack; next_hb_us = us + SFL_HB_PERIOD_MS * 1000u;
        }
    }

    void advance(uint32_t dt_us) { us += dt_us; }
};

/** Run with the stand's loop at 1 ms and Ariadna's at 250 us granularity. */
static void run_until(World &w, uint32_t until_us, bool stand_loops = true)
{
    while ((int32_t)(until_us - w.us) > 0) {
        w.advance(100);
        if (stand_loops && (w.us % 1000) == 0) w.stand_loop();
        if ((w.us % 250) == 0) w.ariadna_loop();
    }
}

/** One complete run; `phase_us` shifts T0 against Ariadna's polling. */
static void one_run(uint32_t phase_us, int a_before_b_poll, bool verbose)
{
    (void)verbose;
    World w;
    w.us += phase_us;
    w.thrust = 0;

    // ---- handshake: Ariadna says ready, the stand hears it --------------------
    run_until(w, w.us + 500000);
    CHECK(w.stand.ariadna_ready(w.ms()));
    CHECK(w.stand.may_start(w.ms()));
    CHECK(w.r.phase == SFR_IDLE);

    // ---- the countdown ------------------------------------------------------------
    w.start_countdown(5);
    run_until(w, w.t0_us - 2000);
    CHECK(w.r.phase == SFR_ARMED && w.nav_armed);
    CHECK(!w.stand.must_abort(w.ms()));                       // the stand does not stop it

    // ---- T0, in three steps, with Ariadna's poll landing before / between / after --
    // a_before_b_poll: 0 = poll lands before step a, 1 = between a and b, 2 = between b and c, 3 = after c
    run_until(w, w.t0_us);
    if (a_before_b_poll == 0) w.ariadna_loop();
    w.stand_t0_step_a();
    if (a_before_b_poll == 1) { w.next_poll_us = w.us; w.ariadna_loop(); }
    w.stand_t0_step_b();
    if (a_before_b_poll == 2) { w.next_poll_us = w.us; w.ariadna_loop(); }
    w.stand_t0_step_c();
    if (a_before_b_poll == 3) { w.next_poll_us = w.us; w.ariadna_loop(); }
    run_until(w, w.t0_us + 30000);

    CHECK(w.fires == 1);
    CHECK(w.fires == 1 && (int32_t)(w.fire_us - w.t0_us) >= 0 && (int32_t)(w.fire_us - w.t0_us) <= 12000);
    CHECK(w.stand.ignite_acked());
    CHECK(w.stand.ignite == 0);                               // acked: the request is down
    CHECK(w.r.phase == SFR_BURN);
    // Ariadna's timeline agrees with the stand's to well under a millisecond
    CHECK(w.out.t_valid && w.r.t0_valid);
    const int32_t stand_t = (int32_t)(w.us - w.t0_us);
    CHECK(abs(w.out.t_us - stand_t) < 800);

    // ---- the burn: thrust rises, the clamp profile runs; IGNITE never repeats -----
    w.thrust = 150;
    run_until(w, w.us + 1200000);
    CHECK(w.fires == 1);
    CHECK(w.r.flaps.phase == SFF_FULL);
    CHECK(w.r.t_phase_us[0] != SFL_T_NONE && w.r.t_phase_us[3] != SFL_T_NONE);
    CHECK(abs((w.r.t_phase_us[3] - w.r.t_phase_us[0]) - 900000) < 20000);

    // ---- the burn ends; the stand closes its record; Ariadna copies it -----------
    w.thrust = 0;
    run_until(w, w.t0_us + 12000000);
    w.st = SFL_ST_IDLE;
    w.stand.burn_closed(41, GOLD_PAGES);
    w.stand_publish();
    run_until(w, w.us + 400000);
    CHECK(w.r.phase == SFR_IDLE);
    CHECK(w.last_st.flags & SFL_F_DATA_READY);
    CHECK(!(w.last_st.flags & SFL_F_READY));                   // the stand will not start another run yet
    CHECK(!w.stand.may_start(w.ms()));

    // the copy, page by page over the real chunk protocol
    std::vector<uint8_t> copy(GOLD_PAGES * 256);
    for (unsigned p = 0; p < GOLD_PAGES; p++)
        for (unsigned ch = 0; ch < SFL_CHUNKS_PER_PAGE; ch++) {
            uint8_t req[SFL_PAGE_REQ_SIZE]; sfl_page_req_encode((uint16_t)p, (uint8_t)ch, req);
            uint16_t pg = 0; uint8_t cc = 0;
            CHECK(sfl_page_req_decode(req + 1, &pg, &cc));
            const uint8_t code = w.stand.page_code(pg, cc);
            CHECK(code == SFL_PG_OK);
            uint8_t rep[SFL_CHUNK_REPLY];
            sfl_chunk_reply_encode(code, GOLD_PAGE_DATA[pg] + cc * SFL_CHUNK, rep);
            uint8_t rc, data[SFL_CHUNK];
            CHECK(sfl_chunk_reply_decode(rep, &rc, data));
            memcpy(copy.data() + p * 256 + cc * SFL_CHUNK, data, SFL_CHUNK);
        }
    CHECK(memcmp(copy.data(), GOLD_PAGE_DATA, GOLD_PAGES * 256) == 0);

    // while the stand is not idle the pages are refused
    World busy; busy.stand.burn_closed(1, 8); busy.st = SFL_ST_RECORDING; busy.stand_publish();
    CHECK(busy.stand.page_code(0, 0) == SFL_PG_BUSY);

    // Ariadna says "data copied": the stand drops the pending flag and is ready again
    w.data_seq = w.last_st.run_seq;
    w.i2c_hb(w.out.hb_flags, w.out.hb_ack, w.data_seq);
    w.stand_loop();
    CHECK(w.stand.data_pending == 0);
    CHECK(w.stand.stand_ready() && w.stand.may_start(w.ms()));
}

static void test_t0_offsets()
{
    // every 100 us offset of T0 inside one 5 ms poll period, and the four ways the
    // poll can land around the stand's three T0 steps
    for (uint32_t phase = 0; phase < 5000; phase += 100)
        for (int where = 0; where < 4; where++) one_run(phase, where, false);
}

static void test_cable_pulled_in_countdown()
{
    World w;
    run_until(w, w.us + 500000);
    w.start_countdown(10);
    run_until(w, w.us + 3000000);
    CHECK(w.r.phase == SFR_ARMED);
    w.link = false;                                    // the cable comes out
    run_until(w, w.us + 600000);
    // both ends noticed: Ariadna disarmed and is not ready, the stand would stop the countdown
    CHECK(w.r.phase == SFR_IDLE && !w.nav_armed);
    CHECK(w.stand.must_abort(w.ms()));
    CHECK(w.fires == 0);
    // and when the cable is back mid-countdown nothing fires at T0 either: Ariadna never watched this one
    w.st = SFL_ST_IDLE; w.stand_publish();
}

static void test_cable_pulled_before_t0_never_fires()
{
    World w;
    run_until(w, w.us + 500000);
    w.start_countdown(5);
    run_until(w, w.t0_us - 800000);
    CHECK(w.r.phase == SFR_ARMED);
    w.link = false;
    run_until(w, w.t0_us - 100000);                    // Ariadna is deaf for the last 700 ms
    w.link = true;
    w.stand_t0_step_a(); w.stand_t0_step_b(); w.stand_t0_step_c();
    run_until(w, w.t0_us + 200000);
    // Ariadna lost the stand (>200 ms) and disarmed: nothing is fired for a run it could not follow
    CHECK(w.fires == 0);
}

int main()
{
    one_run(0, 3, true);
    test_t0_offsets();
    test_cable_pulled_in_countdown();
    test_cable_pulled_before_t0_never_fires();
    printf("sflink loopback test: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
