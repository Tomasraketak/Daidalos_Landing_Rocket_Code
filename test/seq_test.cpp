// =====================================================================
//  Host test of the flight event sequencer (ariadna_seq.h) and of the
//  parameter table (ariadna_params.h).
//
//  The sequencer decides when the aerocover unlatches and when each of the
//  three pyro channels gets current. Its rules are few and each one is a
//  line in the user manual, so each one is a check here.
// =====================================================================
#include "../Ariadna_V2/common/ariadna_seq.h"
#include "../Ariadna_V2/common/ariadna_params.h"
#include "../Ariadna_V2/common/guidance.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int g_checks = 0, g_fail = 0;
#define CHECK(cond, msg)                                              \
    do {                                                              \
        g_checks++;                                                   \
        if (!(cond)) { g_fail++; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, msg); } \
    } while (0)

static ariadna_seq_cfg_t cfg_default()
{
    ariadna_params_t p;
    ariadna_params_default(&p);
    ariadna_seq_cfg_t c;
    c.cover_lead_s = p.v[ARIADNA_P_SEQ_COVER_LEAD];
    c.backup_win_s = p.v[ARIADNA_P_SEQ_BACKUP_WIN];
    c.legs_lead_s  = p.v[ARIADNA_P_SEQ_LEGS_LEAD];
    c.legs_pulse_s = p.v[ARIADNA_P_SEQ_LEGS_PULSE];
    c.ign_pulse_s  = p.v[ARIADNA_P_SEQ_IGN_PULSE];
    c.legs_h_max_m = p.v[ARIADNA_P_SEQ_LEGS_H_MAX];
    c.cover_en = c.backup_en = c.legs_en = 1;
    return c;
}

struct Timeline {
    // result of one simulated flight, times in ms from the ignition command
    long cover = -99999, ign1 = -99999, ign2 = -99999, legs_on = -99999, legs_off = -99999;
    long ign1_off = -99999;
    uint8_t events = 0;
};

// A scripted descent: the command at t = 0, thrust felt `onset_ms` later
// (< 0: never), touchdown at `td_ms`. Sampled at 200 Hz like the firmware.
static Timeline run(const ariadna_seq_cfg_t &c, long onset_ms, long td_ms,
                    bool permit = true, long permit_lost_ms = -1, bool countdown_valid = true)
{
    ariadna_seq_t s;
    ariadna_seq_init(&s);
    Timeline r;
    uint8_t prev[3] = { 0, 0, 0 }, prev_cover = 0;
    for (long t = -4000; t <= td_ms + 3000; t += 5) {
        ariadna_seq_in_t in;
        std::memset(&in, 0, sizeof(in));
        in.permit = permit && !(permit_lost_ms >= 0 && t >= permit_lost_ms);
        in.ignite = t >= 0;
        in.thrust_on = onset_ms >= 0 && t >= onset_ms;
        if (t < 0)            in.phase = DG_PHASE_DESCENT;
        else if (!in.thrust_on && t < 600) in.phase = DG_PHASE_IGNITING;
        else if (t < td_ms)   in.phase = DG_PHASE_BURN;
        else                  in.phase = DG_PHASE_DONE;
        in.t_to_ignition  = (t < 0 && countdown_valid) ? -t / 1000.0f : -1.0f;
        in.t_to_touchdown = (t < td_ms && countdown_valid) ? (td_ms - t) / 1000.0f : -1.0f;
        in.h = t < 0 ? 55.0f - 0.04f * t : std::fmax(0.0f, 55.0f * (1.0f - (float)t / td_ms));
        ariadna_seq_step(&s, &c, &in, (uint32_t)(t + 100000));
        if (s.cover_open && !prev_cover) r.cover = t;
        if (s.pyro[0] && !prev[0]) r.ign1 = t;
        if (!s.pyro[0] && prev[0]) r.ign1_off = t;
        if (s.pyro[1] && !prev[1]) r.ign2 = t;
        if (s.pyro[2] && !prev[2]) r.legs_on = t;
        if (!s.pyro[2] && prev[2]) r.legs_off = t;
        prev_cover = s.cover_open;
        std::memcpy(prev, s.pyro, 3);
    }
    r.events = s.events;
    return r;
}

static void test_sequence()
{
    ariadna_seq_cfg_t c = cfg_default();

    // ---- nominal: thrust felt 80 ms after the command, 2.7 s to the ground
    Timeline r = run(c, 80, 2700);
    CHECK(r.cover >= -605 && r.cover <= -595, "aerocover 0.6 s before the ignition command");
    CHECK(r.ign1 == 0, "PYRO1 at the ignition command");
    CHECK(r.ign1_off >= 595 && r.ign1_off <= 605, "PYRO1 pulse length");
    CHECK(r.ign2 == -99999, "no backup when thrust is felt inside the window");
    CHECK(r.legs_on >= 2700 - 1705 && r.legs_on <= 2700 - 1695, "PYRO3 1.7 s before touchdown");
    CHECK(r.legs_off - r.legs_on >= 500, "PYRO3 held at least 500 ms");
    CHECK((r.events & (ARIADNA_EV_COVER | ARIADNA_EV_IGN1 | ARIADNA_EV_THRUST | ARIADNA_EV_LEGS |
                       ARIADNA_EV_TOUCHDOWN | ARIADNA_EV_APOGEE)) ==
          (ARIADNA_EV_COVER | ARIADNA_EV_IGN1 | ARIADNA_EV_THRUST | ARIADNA_EV_LEGS |
           ARIADNA_EV_TOUCHDOWN | ARIADNA_EV_APOGEE), "event bits, nominal");
    CHECK(!(r.events & ARIADNA_EV_IGN2), "no IGN2 event, nominal");

    // ---- thrust at the very moment of the command: still no backup
    r = run(c, 0, 2700);
    CHECK(r.ign2 == -99999, "no backup when thrust is felt at the command");

    // ---- thrust late (200 ms): the backup fires at 150 ms
    r = run(c, 200, 2700);
    CHECK(r.ign2 >= 150 && r.ign2 <= 155, "backup 150 ms after the command without thrust");
    CHECK(r.events & ARIADNA_EV_IGN2, "IGN2 event");

    // ---- thrust never: the backup fires once, and only once
    r = run(c, -1, 2700);
    CHECK(r.ign2 >= 150 && r.ign2 <= 155, "backup fires when the motor never lights");

    // ---- backup disabled
    c.backup_en = 0;
    r = run(c, -1, 2700);
    CHECK(r.ign2 == -99999, "backup disabled");
    c = cfg_default();

    // ---- not armed: the cover still opens (a servo), no channel fires
    r = run(c, 80, 2700, false);
    CHECK(r.cover >= -605 && r.cover <= -595, "cover is not behind the pyro permit");
    CHECK(r.ign1 == -99999 && r.ign2 == -99999 && r.legs_on == -99999, "nothing fires unarmed");

    // ---- arming withdrawn mid-pulse cuts the current at once
    r = run(c, 80, 2700, true, 300);
    CHECK(r.ign1 == 0 && r.ign1_off >= 300 && r.ign1_off <= 305, "permit loss cuts PYRO1");
    CHECK(r.legs_on == -99999, "nothing fires after the permit is lost");

    // ---- no valid countdown: the cover opens at the command, legs go late
    r = run(c, 80, 2700, true, -1, false);
    CHECK(r.cover == 0, "cover opens with the command when the countdown was never valid");
    CHECK(r.legs_on > 0 && r.legs_on < 2700, "legs fire in the burn without a countdown");

    // ---- the leg release is refused high up, whatever the countdown says
    {
        ariadna_seq_t s;
        ariadna_seq_init(&s);
        ariadna_seq_in_t in;
        std::memset(&in, 0, sizeof(in));
        in.permit = 1; in.phase = DG_PHASE_DESCENT;
        in.t_to_ignition = 5.0f; in.t_to_touchdown = 1.0f; in.h = 150.0f;
        for (int i = 0; i < 100; i++) ariadna_seq_step(&s, &c, &in, 1000 + 5 * i);
        CHECK(!s.pyro[2], "legs refused above legs_h_max");
        // ... and nothing at all happens on the ground or going up
        ariadna_seq_init(&s);
        in.h = 2.0f; in.t_to_touchdown = 0.5f; in.t_to_ignition = 0.1f; in.ignite = 1;
        in.phase = DG_PHASE_IDLE;
        for (int i = 0; i < 100; i++) ariadna_seq_step(&s, &c, &in, 1000 + 5 * i);
        CHECK(!s.pyro[0] && !s.pyro[1] && !s.pyro[2] && !s.cover_open, "inert in IDLE");
        in.phase = DG_PHASE_ASCENT;
        for (int i = 0; i < 100; i++) ariadna_seq_step(&s, &c, &in, 1000 + 5 * i);
        CHECK(!s.pyro[0] && !s.pyro[1] && !s.pyro[2] && !s.cover_open, "inert in ASCENT");
    }

    // ---- a leg pulse shorter than 0.5 s cannot be configured
    c.legs_pulse_s = 0.1f;
    r = run(c, 80, 2700);
    CHECK(r.legs_off - r.legs_on >= 500, "leg pulse floor of 500 ms");
    c = cfg_default();

    // ---- other timings are honoured
    c.cover_lead_s = 1.0f; c.legs_lead_s = 1.0f; c.legs_pulse_s = 0.8f; c.backup_win_s = 0.3f;
    r = run(c, -1, 2700);
    CHECK(r.cover >= -1005 && r.cover <= -995, "configurable cover lead");
    CHECK(r.legs_on >= 1695 && r.legs_on <= 1705, "configurable legs lead");
    CHECK(r.legs_off - r.legs_on >= 800 && r.legs_off - r.legs_on <= 810, "configurable legs pulse");
    CHECK(r.ign2 >= 300 && r.ign2 <= 305, "configurable backup window");

    // phases are numerically the guidance module's
    CHECK(ARIADNA_SEQ_PH_DESCENT == DG_PHASE_DESCENT && ARIADNA_SEQ_PH_IGNITING == DG_PHASE_IGNITING &&
          ARIADNA_SEQ_PH_BURN == DG_PHASE_BURN && ARIADNA_SEQ_PH_DONE == DG_PHASE_DONE,
          "sequencer phase numbers match dg_phase_t");
}

static void test_params()
{
    ariadna_params_t p;
    ariadna_params_default(&p);

    // ids unique, inside the table, defaults inside their ranges
    bool seen[ARIADNA_PARAM_ID_MAX] = { false };
    for (uint8_t i = 0; i < ARIADNA_PARAM_COUNT; i++) {
        const ariadna_param_info_t &q = ARIADNA_PARAM_INFO[i];
        CHECK(q.id < ARIADNA_PARAM_ID_MAX, "param id inside the table");
        CHECK(!seen[q.id], "param id unique");
        seen[q.id] = true;
        CHECK(q.lo <= q.def && q.def <= q.hi, q.key);
        CHECK(q.owner == 'N' || q.owner == 'M' || q.owner == 'P', "param owner");
    }

    // The table's defaults ARE the guidance module's. A drift here means the
    // vehicle flies different gains from the ones the tests checked.
    dg_params_t d;
    dg_params_default(&d);
    const struct { uint8_t id; float v; const char *n; } map[] = {
        { ARIADNA_P_G_MASS_GROSS, d.mass_gross, "mass_gross" },
        { ARIADNA_P_G_MASS_PROP, d.mass_prop, "mass_prop" },
        { ARIADNA_P_G_WN_TVC, d.wn_tvc, "wn_tvc" }, { ARIADNA_P_G_ZETA_TVC, d.zeta_tvc, "zeta_tvc" },
        { ARIADNA_P_G_SCHED_TVC, d.sched_tvc, "sched_tvc" }, { ARIADNA_P_G_ATT_KI, d.att_ki, "att_ki" },
        { ARIADNA_P_G_ATT_I_LIM, d.att_i_lim, "att_i_lim" }, { ARIADNA_P_G_WN_FIN, d.wn_fin, "wn_fin" },
        { ARIADNA_P_G_ZETA_FIN, d.zeta_fin, "zeta_fin" }, { ARIADNA_P_G_SCHED_FIN, d.sched_fin, "sched_fin" },
        { ARIADNA_P_G_ROLL_GAIN, d.roll_gain, "roll_gain" }, { ARIADNA_P_G_ROLL_MAX, d.roll_max_deg, "roll_max" },
        { ARIADNA_P_G_TILT_MIN, d.tilt_min_deg, "tilt_min" }, { ARIADNA_P_G_TILT_SLOPE, d.tilt_slope, "tilt_slope" },
        { ARIADNA_P_G_TILT_CAP, d.tilt_cap_deg, "tilt_cap" }, { ARIADNA_P_G_ALT_PAD, d.alt_pad, "alt_pad" },
        { ARIADNA_P_G_H_IGN_BIAS, d.h_ign_bias, "h_ign_bias" },
        { ARIADNA_P_G_H_IGN_OVERRIDE, d.h_ign_override, "h_ign_override" },
        { ARIADNA_P_G_PLAN_TARGET, d.plan_target, "plan_target" },
        { ARIADNA_P_G_PLAN_MARGIN, d.plan_margin, "plan_margin" }, { ARIADNA_P_G_PLAN_HOLD, d.plan_hold, "plan_hold" },
        { ARIADNA_P_G_ONSET_BUMP, d.onset_bump, "onset_bump" },
        { ARIADNA_P_G_ONSET_TIMEOUT, d.onset_timeout, "onset_timeout" },
        { ARIADNA_P_G_ONSET_ASSUMED, d.onset_assumed, "onset_assumed" },
        { ARIADNA_P_G_FIN_ENABLE, d.fin_enable, "fin_enable" }, { ARIADNA_P_G_FIN_MAX, d.fin_max_deg, "fin_max" },
        { ARIADNA_P_G_FIN_BRAKE, d.fin_brake, "fin_brake" }, { ARIADNA_P_G_FIN_MIN_V, d.fin_min_airspeed, "fin_min_v" },
        { ARIADNA_P_G_FIN_DRIFT_GAIN, d.fin_drift_gain, "fin_drift_gain" },
        { ARIADNA_P_G_FIN_DRIFT_MAX, d.fin_drift_max_deg, "fin_drift_max" },
        { ARIADNA_P_G_FIN_CTRL_LEAD, d.fin_ctrl_lead, "fin_ctrl_lead" },
        { ARIADNA_P_G_FIN_ASCENT, d.fin_ascent_mode, "fin_ascent" },
        { ARIADNA_P_G_FIN_ADAPT, d.fin_adapt, "fin_adapt" }, { ARIADNA_P_G_FIN_GAIN0, d.fin_gain0, "fin_gain0" },
        { ARIADNA_P_G_CD_BODY, d.cd_body, "cd_body" }, { ARIADNA_P_G_THRUST_SCALE, d.thrust_scale, "thrust_scale" },
        { ARIADNA_P_G_IGN_DELAY_PAD, d.ign_delay_pad, "ign_delay_pad" },
        { ARIADNA_P_G_DRAG_ADAPT, d.drag_adapt, "drag_adapt" }, { ARIADNA_P_G_DIAMETER, d.diameter, "diameter" },
        { ARIADNA_P_TBL_VZ0, d.tbl_vz0, "tbl_vz0" }, { ARIADNA_P_TBL_DVZ, d.tbl_dvz, "tbl_dvz" },
    };
    for (int i = 0; i < DG_IGN_N; i++) {
        CHECK(std::fabs(p.v[ARIADNA_P_TBL_HMIN_00 + i] - d.tbl_hmin[i]) < 1e-4f, "tbl.hmin default = guidance table");
        CHECK(std::fabs(p.v[ARIADNA_P_TBL_HMAX_00 + i] - d.tbl_hmax[i]) < 1e-4f, "tbl.hmax default = guidance table");
    }
    CHECK(ARIADNA_PARAM_COUNT < 255, "parameter count fits the packet");
    CHECK(p.v[ARIADNA_P_POS_LIDAR_DELAY] == 500.0f, "lidar waits 500 ms after the legs");
        for (const auto &m : map) {
        bool ok = std::fabs(p.v[m.id] - m.v) <= 1e-4f * (1.0f + std::fabs(m.v));
        if (!ok) std::printf("  %s: table %g, guidance %g\n", m.n, (double)p.v[m.id], (double)m.v);
        CHECK(ok, "parameter default equals the guidance module's");
    }

    // The timings the vehicle was specified with.
    CHECK(p.v[ARIADNA_P_SEQ_COVER_LEAD] == 0.60f, "aerocover lead default 0.6 s");
    CHECK(p.v[ARIADNA_P_SEQ_BACKUP_WIN] == 0.15f, "backup window default 150 ms");
    CHECK(p.v[ARIADNA_P_SEQ_LEGS_LEAD] == 1.70f, "legs lead default 1.7 s");
    CHECK(p.v[ARIADNA_P_SEQ_LEGS_PULSE] == 0.50f, "legs pulse default 0.5 s");
    CHECK(ARIADNA_PARAM_INFO[ARIADNA_P_SEQ_LEGS_PULSE].lo >= 0.5f, "legs pulse cannot go below 0.5 s");
    CHECK(p.v[ARIADNA_P_POS_GPS_LATENCY] == 50.0f, "GPS latency default 50 ms");

    // set: clamped, NaN refused, unknown id refused
    CHECK(ariadna_param_set(&p, ARIADNA_P_SEQ_COVER_LEAD, 99.0f) && p.v[ARIADNA_P_SEQ_COVER_LEAD] == 5.0f,
          "set clamps to the maximum");
    CHECK(ariadna_param_set(&p, ARIADNA_P_SEQ_LEGS_PULSE, 0.1f) && p.v[ARIADNA_P_SEQ_LEGS_PULSE] == 0.5f,
          "set clamps to the minimum");
    CHECK(!ariadna_param_set(&p, ARIADNA_P_G_WN_TVC, NAN), "NaN refused");
    CHECK(!ariadna_param_set(&p, 92, 1.0f), "unknown id refused");
    CHECK(!ariadna_param_set(&p, 250, 1.0f), "id past the table refused");

    // the CRC sees a single changed value
    ariadna_params_t a, b;
    ariadna_params_default(&a);
    ariadna_params_default(&b);
    CHECK(ariadna_params_crc(&a) == ariadna_params_crc(&b), "crc deterministic");
    ariadna_param_set(&b, ARIADNA_P_G_ALT_PAD, 1.01f);
    CHECK(ariadna_params_crc(&a) != ariadna_params_crc(&b), "crc sees one changed parameter");
}

int main()
{
    std::printf("flight event sequencer and parameter table\n");
    test_sequence();
    test_params();
    std::printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
