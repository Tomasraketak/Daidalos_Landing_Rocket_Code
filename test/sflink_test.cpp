// =====================================================================
//  Host test: Static Fire Link (Ariadna_V2/Examples/Static_Fire_Link)
//
//  The sketch is a thin wire around three headers, all checked here:
//    sf_flaps.h   the clamp profile      50 % -> minimum -> 75 % -> open
//    sf_run.h     when to arm, when to fire, what to tell the stand
//    sf_record.h  the merged record on Ariadna's flash
//
//  An example, but one that fires an igniter on a signal from another board,
//  so the rules that stop it firing at the wrong time are tested like flight
//  code: a stale IGNITE, an IGNITE for a run it never watched, a link that
//  dies, a late request, a flipped bit.
//
//  The stand's pages in sflink_golden.h were built and decoded by the stand's
//  own Python tools (gen_sflink_golden.py), so the merge is checked against
//  the other implementation of the format, not against itself.
//
//  Run:  cd test && make sflink
// =====================================================================
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "../Ariadna_V2/Examples/Static_Fire_Link/sf_flaps.h"
#include "../Ariadna_V2/Examples/Static_Fire_Link/sf_run.h"
#include "../Ariadna_V2/Examples/Static_Fire_Link/sf_record.h"
#include "sflink_golden.h"

static int s_fail = 0, s_pass = 0;
#define CHECK(cond) do { if (cond) s_pass++; else { s_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

// =====================================================================
//  the clamp profile
// =====================================================================
static void test_flaps()
{
    sff_cfg_t c; sff_cfg_default(&c);
    sff_t s; sff_init(&s);
    uint32_t t = 1000000;                         // some local clock

    // fully open while the thrust is below 100 N, whatever it does
    for (int i = 0; i < 100; i++, t += 5000) NEAR(sff_step(&s, &c, t, (int32_t)(i * 5000), 99.9f), 100.0f, 0.001f);
    CHECK(s.phase == SFF_WAIT);

    // one noisy sample over the trigger does not start it ...
    sff_step(&s, &c, t, 0, 120.0f); t += 5000;
    sff_step(&s, &c, t, 0, 80.0f);  t += 5000;
    CHECK(s.phase == SFF_WAIT);

    // ... staying over it for the debounce time does, and the first step is 50 %
    const uint32_t t_trig = t;
    float pct = 100;
    for (int i = 0; i < 4; i++, t += 5000) pct = sff_step(&s, &c, t, 0, 130.0f);
    CHECK(s.phase == SFF_S1);
    NEAR(pct, 50.0f, 0.001f);

    // 300 ms at 50, 300 ms at minimum, 300 ms at 75, then fully open for good
    const uint32_t t1 = s.enter_us;
    CHECK(t1 >= t_trig + c.debounce_us);
    NEAR(sff_step(&s, &c, t1 + 299999, 0, 150.0f), 50.0f, 0.001f);
    NEAR(sff_step(&s, &c, t1 + 300000, 0, 150.0f), 10.0f, 0.001f);   CHECK(s.phase == SFF_S2);
    NEAR(sff_step(&s, &c, t1 + 599999, 0, 20.0f), 10.0f, 0.001f);     // thrust no longer matters
    NEAR(sff_step(&s, &c, t1 + 600000, 0, 20.0f), 75.0f, 0.001f);     CHECK(s.phase == SFF_S3);
    NEAR(sff_step(&s, &c, t1 + 899999, 0, 20.0f), 75.0f, 0.001f);
    NEAR(sff_step(&s, &c, t1 + 900000, 0, 20.0f), 100.0f, 0.001f);    CHECK(s.phase == SFF_FULL);
    NEAR(sff_step(&s, &c, t1 + 5000000, 0, 500.0f), 100.0f, 0.001f);  // and it does not start over
    CHECK(s.phase == SFF_FULL);

    // no trigger by the give-up time: open, and the sequence is over
    sff_init(&s);
    NEAR(sff_step(&s, &c, 5, (int32_t)c.giveup_us - 1, 0.0f), 100.0f, 0.001f);  CHECK(s.phase == SFF_WAIT);
    sff_step(&s, &c, 10, (int32_t)c.giveup_us, 0.0f);                            CHECK(s.phase == SFF_GAVEUP);
    NEAR(sff_step(&s, &c, 20, (int32_t)c.giveup_us + 1000, 400.0f), 100.0f, 0.001f);  // late thrust: still open
    CHECK(s.phase == SFF_GAVEUP);

    // fail-safe in the middle of a step: open at once, for good
    sff_init(&s);
    for (int i = 0; i < 4; i++) sff_step(&s, &c, 1000 + i * 5000u, 0, 200.0f);
    CHECK(s.phase == SFF_S1);
    sff_failsafe(&s, 50000);
    NEAR(sff_step(&s, &c, 50001, 0, 200.0f), 100.0f, 0.001f);
    CHECK(s.phase == SFF_GAVEUP);

    // wrap of the local clock does not shorten a step
    sff_init(&s);
    uint32_t w = 0xFFFFFFF0u;
    for (int i = 0; i < 5; i++, w += 5000) sff_step(&s, &c, w, 0, 200.0f);
    CHECK(s.phase == SFF_S1);
    NEAR(sff_step(&s, &c, s.enter_us + 100000, 0, 200.0f), 50.0f, 0.001f);

    // positions
    NEAR(sff_pct_to_us(100, 1150, 1900), 1900, 0.01f);
    NEAR(sff_pct_to_us(0, 1150, 1900), 1150, 0.01f);
    NEAR(sff_pct_to_us(50, 1150, 1900), 1525, 0.01f);
    NEAR(sff_pct_to_us(150, 1150, 1900), 1900, 0.01f);                // clamped
    CHECK(sff_closed_permille(100) == 0 && sff_closed_permille(10) == 900 && sff_closed_permille(0) == 1000);
    CHECK(sff_closed_permille(75) == 250 && sff_closed_permille(50) == 500);
}

// =====================================================================
//  the supervisor: a stand simulator and the happy path
// =====================================================================
struct Sim {
    sfr_t r; sfr_cfg_t c; sfr_in_t in; sfr_out_t out;
    uint32_t ms, us;                  // our clock
    uint32_t t0_ms;                   // stand: T0 on the same clock, ms (0 = none)
    uint8_t  run;
    int      state;
    int      ignite;
    float    thrust;
    int      nav_armed, prereq, cont, hold;
    int      ack_wait;                // ticks since a STATUS was delivered
    int      fires;

    Sim() {
        sfr_init(&r); sfr_cfg_default(&c);
        ms = 5000; us = ms * 1000; t0_ms = 0; run = 7; state = SFL_ST_IDLE; ignite = 0;
        thrust = 0; nav_armed = 0; prereq = 1; cont = 1; hold = 0; fires = 0;
        memset(&in, 0, sizeof(in));
        memset(&out, 0, sizeof(out));
        in.stand_ok = 1;
    }

    /** STATUS as the stand would answer at our time `us`. */
    void deliver_status() {
        sfl_status_t &st = in.st;
        memset(&st, 0, sizeof(st));
        st.state = (uint8_t)state; st.run_seq = run;
        st.flags = SFL_F_HX_OK | SFL_F_LOG_OK | SFL_F_LINK_OK | (ignite ? SFL_F_IGNITE : 0);
        st.thrust_mN = (int32_t)(thrust * 1000.0f);
        if (state == SFL_ST_COUNTDOWN || state == SFL_ST_RECORDING)
            st.t_rel_us = (int32_t)((int64_t)us - (int64_t)t0_ms * 1000);
        else st.t_rel_us = SFL_T_NONE;
        in.st_us = us; in.st_new = 1; in.stand_ok = 1;
    }

    /** One loop tick of 5 ms; a STATUS is delivered every tick unless `link` is 0. */
    void tick(int link = 1) {
        if (link) deliver_status(); else { in.st_new = 0; if (ms - last_ok > SFR_LOST_MS) in.stand_ok = 0; }
        if (link) last_ok = ms;
        in.now_ms = ms; in.now_us = us;
        in.prereq_ok = prereq; in.nav_armed = nav_armed; in.cont = cont; in.hold = hold;
        sfr_tick(&r, &c, &in, &out);
        in.st_new = 0;
        if (out.want_arm) nav_armed = 1;                    // a well behaved Navigation MCU
        if (out.want_disarm) nav_armed = 0;
        if (out.fire) { fires++; nav_armed = 0; }
        ms += 5; us += 5000;
    }
    uint32_t last_ok = 0;
};

static void test_happy_path()
{
    Sim s;
    // idle: ready, nothing armed, clamp open
    s.tick();
    CHECK(s.out.hb_flags & SFL_A_READY);
    CHECK(s.r.phase == SFR_IDLE && !s.out.want_arm && s.out.flap_open_pct == 100.0f);

    // countdown starts: Ariadna sees it and arms
    s.state = SFL_ST_COUNTDOWN; s.t0_ms = s.ms + 15000;
    s.tick();
    CHECK(s.r.phase == SFR_ARMING && (s.out.ev & SFR_EV_COUNTDOWN) && s.out.want_arm);
    s.tick();
    CHECK(s.r.phase == SFR_ARMED);                         // the simulated Navigation MCU armed
    CHECK(s.out.hb_flags & SFL_A_ARMED);
    CHECK(s.out.hb_flags & SFL_A_READY);                   // still ready: the stand must not abort

    // 14.9 s of countdown: nothing fires, the timeline is T-minus
    int logged = 0;
    while (s.ms + 100 < s.t0_ms) { s.tick(); CHECK(!s.out.fire); if (s.out.log_on) logged++; CHECK(s.r.phase == SFR_ARMED); }
    CHECK(logged > 0 && logged < 14000 / 5);               // only the last 3.5 s are logged
    CHECK(s.out.t_valid && s.out.t_us < 0 && s.out.t_us > -200000);

    // T0: the stand publishes COUNTDOWN + IGNITE for one poll, then RECORDING + IGNITE
    while (s.ms < s.t0_ms) s.tick();
    s.ignite = 1;
    s.tick();                                              // countdown + ignite: wait
    CHECK(!s.out.fire && s.r.phase == SFR_ARMED);
    s.state = SFL_ST_RECORDING;
    s.tick();
    CHECK(s.out.fire && s.out.fire_ms == 300 && s.r.phase == SFR_BURN);
    CHECK(s.out.hb_ack == s.run);                          // acked with the command
    CHECK(s.out.hb_flags & SFL_A_FIRED);
    CHECK(s.r.t_fire_us >= 0 && s.r.t_fire_us < 20000);    // the pulse went out within 20 ms of T0
    const uint32_t ms_fire = s.ms;

    // IGNITE stays up for a few polls: exactly one pulse
    for (int i = 0; i < 20; i++) { s.tick(); CHECK(!s.out.fire); }
    CHECK(s.fires == 1);
    s.ignite = 0;

    // thrust builds: below 100 N the clamp stays open
    s.thrust = 40;
    for (int i = 0; i < 20; i++) { s.tick(); NEAR(s.out.flap_open_pct, 100.0f, 0.001f); }
    // crosses 100 N -> 50 %, then 300 ms later minimum, 300 ms later 75 %, then open
    s.thrust = 150;
    uint32_t t_cross = s.ms;
    float seen[5] = {100, 0, 0, 0, 0}; int n = 0; uint32_t at[5] = {0};
    float last = 100;
    for (int i = 0; i < 400; i++) {
        s.tick();
        if (s.out.flap_open_pct != last) { last = s.out.flap_open_pct; if (n < 5) { seen[n] = last; at[n] = s.ms; n++; } }
    }
    CHECK(n == 4);
    NEAR(seen[0], 50, 0.001f); NEAR(seen[1], 10, 0.001f); NEAR(seen[2], 75, 0.001f); NEAR(seen[3], 100, 0.001f);
    CHECK(at[0] - t_cross <= 30);
    NEAR(at[1] - at[0], 300, 10); NEAR(at[2] - at[1], 300, 10); NEAR(at[3] - at[2], 300, 10);
    CHECK(s.r.t_trig_us != SFL_T_NONE && s.r.t_phase_us[0] != SFL_T_NONE && s.r.t_phase_us[3] != SFL_T_NONE);
    NEAR(s.r.t_phase_us[1] - s.r.t_phase_us[0], 300000, 10000);
    (void)ms_fire;

    // the burn ends: stand back to idle -> DONE (clamp open, disarm), then IDLE
    s.state = SFL_ST_IDLE; s.thrust = 0; s.t0_ms = 0;
    s.tick();
    CHECK((s.out.ev & SFR_EV_FINISHED) && s.r.phase == SFR_DONE && s.out.flap_open_pct == 100.0f && !s.out.log_on);
    s.tick();
    CHECK(s.r.phase == SFR_IDLE);
    CHECK(s.r.t_fire_us != SFL_T_NONE);                   // still there for the record
}

// =====================================================================
//  what must NOT fire
// =====================================================================
static void to_armed(Sim &s)
{
    s.state = SFL_ST_COUNTDOWN; s.t0_ms = s.ms + 15000;
    s.tick(); s.tick();
    CHECK(s.r.phase == SFR_ARMED);
}

static void test_never_fires()
{
    // IGNITE out of nowhere, in idle
    { Sim s; s.state = SFL_ST_RECORDING; s.t0_ms = s.ms; s.ignite = 1;
      for (int i = 0; i < 10; i++) { s.tick(); CHECK(!s.out.fire); }
      CHECK(s.r.phase == SFR_IDLE && s.fires == 0); }

    // IGNITE during arming (not armed yet)
    { Sim s; s.nav_armed = 0; s.state = SFL_ST_COUNTDOWN; s.t0_ms = s.ms + 100;
      s.tick(); CHECK(s.r.phase == SFR_ARMING);
      // the Navigation MCU never answers
      s.nav_armed = 0;
      for (int i = 0; i < 5; i++) { s.tick(); s.nav_armed = 0; }
      s.state = SFL_ST_RECORDING; s.ignite = 1;
      for (int i = 0; i < 10; i++) { s.tick(); s.nav_armed = 0; CHECK(!s.out.fire); }
      CHECK(s.fires == 0); }

    // IGNITE for a run Ariadna did not watch
    { Sim s; to_armed(s);
      while (s.ms < s.t0_ms) s.tick();
      s.state = SFL_ST_RECORDING; s.ignite = 1; s.run = (uint8_t)(s.run + 1);
      for (int i = 0; i < 10; i++) { s.tick(); CHECK(!s.out.fire); }
      CHECK(s.fires == 0 && s.r.phase == SFR_IDLE); }

    // IGNITE too late (the poll was stuck for 400 ms)
    { Sim s; to_armed(s);
      while (s.ms < s.t0_ms + 5) s.tick(1);              // T0 passes with Ariadna deaf to it
      s.state = SFL_ST_RECORDING; s.ignite = 1;
      s.ms += 395; s.us += 395000;                       // ... and the first status shows t_rel = 400 ms
      s.tick();
      CHECK(!s.out.fire && (s.out.ev & SFR_EV_LATE) && s.fires == 0);
      CHECK(s.out.hb_flags & SFL_A_FAULT); }

    // countdown aborted on the stand: Ariadna disarms and drops back
    { Sim s; to_armed(s);
      for (int i = 0; i < 200; i++) s.tick();
      s.state = SFL_ST_IDLE; s.t0_ms = 0;
      s.tick();
      CHECK((s.out.ev & SFR_EV_ABORT) && s.out.want_disarm && s.r.phase == SFR_IDLE && s.fires == 0);
      // and the next, unrelated IGNITE-looking status does nothing
      s.state = SFL_ST_RECORDING; s.ignite = 1;
      for (int i = 0; i < 10; i++) { s.tick(); CHECK(!s.out.fire); } }

    // the stand goes silent during the countdown
    { Sim s; to_armed(s);
      for (int i = 0; i < 100; i++) s.tick();
      for (int i = 0; i < 200; i++) { s.tick(0); }       // 1 s of silence
      CHECK(s.r.phase == SFR_IDLE && s.out.want_disarm == 0 /* already disarmed */);
      CHECK(!(s.out.hb_flags & SFL_A_READY));            // so the stand cannot think we are fine
      CHECK(s.fires == 0); }

    // a local prerequisite goes away (RBF key put back, continuity lost, PCA9685 dead ...)
    { Sim s; to_armed(s);
      for (int i = 0; i < 50; i++) s.tick();
      s.prereq = 0; s.tick();
      CHECK((s.out.ev & SFR_EV_ABORT) && s.out.want_disarm && !(s.out.hb_flags & SFL_A_READY)); }

    // the operator says no
    { Sim s; to_armed(s);
      s.hold = 1; s.tick();
      CHECK((s.out.ev & SFR_EV_ABORT) && !(s.out.hb_flags & SFL_A_READY)); }

    // the pyro never arms: not ready, so the stand stops the countdown
    { Sim s; s.state = SFL_ST_COUNTDOWN; s.t0_ms = s.ms + 15000;
      for (int i = 0; i < 400; i++) { s.tick(); s.nav_armed = 0; if (s.out.ev & SFR_EV_ARM_TIMEOUT) break; }
      CHECK(s.r.phase == SFR_IDLE && s.r.fault);
      CHECK(!(s.out.hb_flags & SFL_A_READY) && (s.out.hb_flags & SFL_A_FAULT));
      // fault clears only once the stand is idle again
      s.tick(); CHECK(s.r.fault);
      s.state = SFL_ST_IDLE; s.t0_ms = 0; s.tick(); s.tick();
      CHECK(!s.r.fault && (s.out.hb_flags & SFL_A_READY)); }

    // RECORDING without IGNITE for our run (the stand's own RBF check refused at T0):
    // waited out for a moment, then an abort - never a fire
    { Sim s; to_armed(s);
      while (s.ms < s.t0_ms) s.tick();
      s.state = SFL_ST_RECORDING; s.ignite = 0;
      s.tick(); CHECK(s.r.phase == SFR_ARMED && !s.out.fire);        // within the grace
      for (int i = 0; i < 20; i++) s.tick();
      CHECK(s.r.phase == SFR_IDLE && s.fires == 0); }

    // a flipped hold bit in the middle: not ready never reads as ready
    { Sim s; s.prereq = 0; s.tick();
      CHECK(!(s.out.hb_flags & SFL_A_READY));
      s.prereq = 1; s.in.st.flags &= (uint8_t)~SFL_F_LOG_OK; s.in.stand_ok = 1;
      // a stand with a faulty flash log is not ready either
      s.deliver_status(); s.in.st.flags &= (uint8_t)~SFL_F_LOG_OK;
      s.in.now_ms = s.ms; s.in.now_us = s.us; s.in.prereq_ok = 1; s.in.st_new = 0;
      sfr_tick(&s.r, &s.c, &s.in, &s.out);
      CHECK(!(s.out.hb_flags & SFL_A_READY)); }
}

static void test_link_lost_in_burn()
{
    Sim s; to_armed(s);
    while (s.ms < s.t0_ms) s.tick();
    s.ignite = 1; s.state = SFL_ST_RECORDING;
    s.tick(); s.tick();
    CHECK(s.r.phase == SFR_BURN);
    s.ignite = 0;
    s.thrust = 200;
    for (int i = 0; i < 20; i++) s.tick();
    CHECK(s.r.flaps.phase == SFF_S1 || s.r.flaps.phase == SFF_S2);
    // the link dies in the middle of the profile: open at once, and it stays open
    int saw = 0;
    for (int i = 0; i < 60; i++) { s.tick(0); if (s.out.ev & SFR_EV_STAND_LOST) saw = 1; if (s.in.stand_ok == 0) NEAR(s.out.flap_open_pct, 100.0f, 0.001f); }
    CHECK(saw);
    NEAR(s.out.flap_open_pct, 100.0f, 0.001f);
    // it comes back with a big thrust: the profile does not restart
    for (int i = 0; i < 100; i++) { s.tick(1); NEAR(s.out.flap_open_pct, 100.0f, 0.001f); }
    CHECK(s.r.flaps.phase == SFF_GAVEUP);
}

static void test_poll_rates()
{
    Sim s; s.tick();
    CHECK(sfr_poll_ms(&s.r, &s.in) == 100);
    s.state = SFL_ST_COUNTDOWN; s.t0_ms = s.ms + 15000; s.tick();
    CHECK(sfr_poll_ms(&s.r, &s.in) == 20);
    s.tick(); CHECK(s.r.phase == SFR_ARMED);
    CHECK(sfr_poll_ms(&s.r, &s.in) == 20);                 // far from T0
    while (s.ms + 1400 < s.t0_ms) s.tick();
    CHECK(sfr_poll_ms(&s.r, &s.in) == 5);                  // in the last 1.5 s
}

// =====================================================================
//  the record
// =====================================================================
struct Rows { std::vector<afr_row_t> v; };
static void collect(void *ctx, const afr_row_t *r) { ((Rows *)ctx)->v.push_back(*r); }

static uint8_t *gold_pages()
{
    static uint8_t buf[GOLD_PAGES * 256];
    for (unsigned i = 0; i < GOLD_PAGES; i++) memcpy(buf + i * 256, GOLD_PAGE_DATA[i], 256);
    return buf;
}

static void test_crc_matches_python()
{
    // zlib CRC of every golden page, as the stand's tools compute it
    uint8_t *pg = gold_pages();
    for (unsigned i = 0; i < GOLD_PAGES; i++) CHECK(afr_stand_page_type(pg + i * 256) != AFR_PG_BAD);
    CHECK(afr_stand_page_type(pg) == AFR_PG_HEADER);
    CHECK(afr_stand_page_type(pg + 256) == AFR_PG_DATA);
    CHECK(afr_stand_page_type(pg + (GOLD_PAGES - 1) * 256) == AFR_PG_FOOTER);
    CHECK(afr_crc32("123456789", 9) == 0xCBF43926u);          // the zlib check value

    uint8_t bad[256]; memcpy(bad, pg + 256, 256); bad[100] ^= 0x01;
    CHECK(afr_stand_page_type(bad) == AFR_PG_BAD);
}

static void test_scan_and_merge()
{
    uint8_t *pg = gold_pages();
    afr_stand_t info; afr_stand_scan(pg, GOLD_PAGES, &info);
    CHECK(info.has_header && info.has_footer && info.clean && info.bad_pages == 0);
    CHECK(info.burn_id == GOLD_BURN_ID && info.tare_offset == GOLD_TARE);
    NEAR(info.cal_counts_per_n, GOLD_CAL, 0.001f);
    CHECK(info.n_samples == GOLD_SAMPLES);

    // Ariadna's samples every 5 ms from t = -0.25 s, clamp stepping at 0.5 s
    std::vector<afr_sample_t> own;
    for (int32_t t = -250000; t < 1500000; t += 5000) {
        afr_sample_t o; memset(&o, 0, sizeof(o));
        o.t_us = t; o.vbat_mv = (uint16_t)(7400 + t / 1000); o.current_ma = (int16_t)(t > 500000 ? 900 : 100);
        o.servo_us = (uint16_t)(t < 500000 ? 1900 : 1525); o.closed_pm = (uint16_t)(t < 500000 ? 0 : 500);
        o.phase = (uint8_t)(t < 500000 ? SFF_WAIT : SFF_S1);
        own.push_back(o);
    }
    Rows rows;
    const uint32_t n = afr_merge(pg, GOLD_PAGES, own.data(), (uint32_t)own.size(), collect, &rows);
    CHECK(n == GOLD_SAMPLES && rows.v.size() == GOLD_SAMPLES);

    int bad_t = 0, bad_raw = 0, bad_nodata = 0, bad_hold = 0, nodata_rows = 0;
    for (unsigned i = 0; i < GOLD_SAMPLES && i < rows.v.size(); i++) {
        const afr_row_t &r = rows.v[i];
        if (r.t_us != GOLD_T_US[i]) bad_t++;
        int32_t raw = (int32_t)(r.packed & 0x00FFFFFFu);
        if (raw & 0x800000) raw -= 0x1000000;
        if (raw != GOLD_RAW[i]) bad_raw++;
        const uint8_t flags = (uint8_t)(r.packed >> 24);
        const int nodata = (flags & AFR_ROW_NODATA) != 0;
        if (nodata) nodata_rows++;
        if (nodata != (r.t_us < -250000)) bad_nodata++;
        if (!nodata) {
            // the held sample is the latest one at or before this row
            const int32_t tq = (r.t_us / 5000) * 5000 - (r.t_us < 0 && (r.t_us % 5000) ? 5000 : 0);
            if (r.vbat_mv != (uint16_t)(7400 + tq / 1000)) bad_hold++;
            const int past = r.t_us >= 500000;
            if (r.servo_us != (past ? 1525 : 1900) || r.closed_pm != (past ? 500 : 0)) bad_hold++;
            if (((flags >> AFR_ROW_PHASE_SHIFT) & 7) != (past ? SFF_S1 : SFF_WAIT)) bad_hold++;
        }
    }
    CHECK(bad_t == 0 && bad_raw == 0 && bad_nodata == 0 && bad_hold == 0);
    CHECK(nodata_rows > 0 && nodata_rows < 60);               // only the first few rows precede Ariadna's samples

    // the stand's own flag bits survive in the low nibble
    int pyro_rows = 0;
    for (auto &r : rows.v) if (((r.packed >> 24) & 0x01u)) pyro_rows++;
    CHECK(pyro_rows > 0);

    // with no samples of its own, every row says so
    Rows empty;
    afr_merge(pg, GOLD_PAGES, nullptr, 0, collect, &empty);
    CHECK(empty.v.size() == GOLD_SAMPLES);
    int allnd = 1; for (auto &r : empty.v) if (!((r.packed >> 24) & AFR_ROW_NODATA) || r.vbat_mv) allnd = 0;
    CHECK(allnd);

    // a corrupt data page is dropped, as the Python decoder drops it; the rest still merge
    uint8_t copy[GOLD_PAGES * 256]; memcpy(copy, pg, sizeof(copy));
    copy[3 * 256 + 50] ^= 0x10;
    Rows part;
    const uint32_t m = afr_merge(copy, GOLD_PAGES, own.data(), (uint32_t)own.size(), collect, &part);
    CHECK(m == GOLD_SAMPLES - 29 && part.v.size() == m);
    afr_stand_t pinfo; afr_stand_scan(copy, GOLD_PAGES, &pinfo);
    CHECK(pinfo.bad_pages == 1 && pinfo.has_header && pinfo.has_footer);
}

static void test_pages()
{
    // rows -> pages -> back, with CRCs
    std::vector<afr_row_t> rows(40);
    for (size_t i = 0; i < rows.size(); i++) {
        memset(&rows[i], 0, sizeof(afr_row_t));
        rows[i].t_us = (int32_t)(i * 12500) - 3000000; rows[i].packed = (uint32_t)i * 7u; rows[i].vbat_mv = (uint16_t)(7000 + i);
    }
    afr_rowpage_t pg;
    afr_rowpage_build(&pg, 9, 1, rows.data() + 15, 15);
    CHECK(afr_rowpage_ok(&pg) && pg.f.index == 1 && pg.f.n_rows == 15 && pg.f.run_id == 9);
    CHECK(pg.f.rows[0].vbat_mv == 7015 && pg.f.rows[14].vbat_mv == 7029);
    pg.raw[100] ^= 1; CHECK(!afr_rowpage_ok(&pg));
    afr_rowpage_build(&pg, 9, 2, rows.data() + 30, 10);
    CHECK(afr_rowpage_ok(&pg) && pg.f.n_rows == 10);
    uint8_t blank[256]; memset(blank, 0xFF, sizeof(blank));           // erased flash is not a page
    CHECK(!afr_rowpage_ok((const afr_rowpage_t *)blank));

    afr_header_t h; memset(&h, 0xFF, sizeof(h));
    memset(h.raw, 0, sizeof(h.raw));
    h.f.run_id = 3; h.f.stand_burn_id = 41; h.f.n_rows = 160; h.f.flags = AFR_H_STAND_OK | AFR_H_PROFILE_DONE;
    afr_header_seal(&h);
    CHECK(afr_header_ok(&h) && h.f.magic == AFR_MAGIC_HDR);
    h.f.n_rows ^= 1; CHECK(!afr_header_ok(&h));
    CHECK(!afr_header_ok((const afr_header_t *)blank));

    // every slot lives inside the top megabyte and slots do not overlap
    CHECK(afr_slot_addr(0) == 0x00F00000u && afr_slot_addr(7) + AFR_SLOT_BYTES == 0x01000000u);
    CHECK(afr_slot_addr(8) == afr_slot_addr(0));
    CHECK(AFR_ROWS_FIRST + AFR_ROWS_MAX_PAGES == AFR_SLOT_PAGES);
    CHECK(AFR_ROWS_MAX_PAGES * AFR_ROWS_PER_PAGE > 5000);             // > 60 s at 80 SPS
}

int main()
{
    test_flaps();
    test_happy_path();
    test_never_fires();
    test_link_lost_in_burn();
    test_poll_rates();
    test_crc_matches_python();
    test_scan_and_merge();
    test_pages();
    printf("sflink test: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
