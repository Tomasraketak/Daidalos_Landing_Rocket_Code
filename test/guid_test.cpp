// =====================================================================
//  Host test of the version-2 guidance (Ariadna_V2/common/guidance.cpp).
//
//  Two halves:
//
//   A. NUMERICAL EQUIVALENCE with LandSim_Light. guid_golden.h is written
//      by tools/gen_guidance_tables.py, which imports tvc_sim.py and calls
//      its own functions - the motor curve, the guidance direction, the
//      clamp projection, the plan solver, the terminal law, the ignition
//      altitude. guidance.cpp must return the same numbers.
//
//   B. CLOSED LOOP. A 6-DOF plant (the simulation's own equations: body
//      aero ahead of the CG, four all-moving fins, a gimballed motor with
//      a clamp, an igniter delay, a rolling airframe) flown by guidance.cpp
//      exactly as the Navigation MCU calls it. This is what catches a sign
//      error: a mirrored axis is positive feedback and the vehicle tumbles.
//      It also checks the fin adaptation against fins that are weaker than
//      the model and mis-trimmed.
//
//  The plant is double precision on purpose and shares no code with the
//  module under test.
// =====================================================================
#include "../Ariadna_V2/common/guidance.h"
#include "guid_golden.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

static int g_checks = 0, g_fail = 0;

#define CHECK(cond, ...)                                             \
    do {                                                             \
        g_checks++;                                                  \
        if (!(cond)) {                                               \
            g_fail++;                                                \
            std::printf("FAIL %s:%d  ", __FILE__, __LINE__);         \
            std::printf(__VA_ARGS__);                                \
            std::printf("\n");                                       \
        }                                                            \
    } while (0)

static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ---------------------------------------------------------------------
//  A. golden vectors
// ---------------------------------------------------------------------
static void test_golden()
{
    dg_init();
    double worst = 0;
    for (const auto &m : GOLD_MOTOR) {
        CHECK(near(dg_thrust_ref(m.t), m.thrust, 1e-3), "thrust(%g)=%g want %g",
              (double)m.t, (double)dg_thrust_ref(m.t), (double)m.thrust);
        CHECK(near(dg_burned(m.t), m.burned, 1e-6), "burned(%g)=%g want %g",
              (double)m.t, (double)dg_burned(m.t), (double)m.burned);
    }

    worst = 0;
    for (const auto &c : GOLD_DIR) {
        float ux, uy, uz;
        float tgo = dg_guidance_dir(c.h, c.vx, c.vy, c.vz, c.mass, &ux, &uy, &uz);
        double e = std::fabs(ux - c.ux) + std::fabs(uy - c.uy) + std::fabs(uz - c.uz);
        if (e > worst) worst = e;
        CHECK(e < 2e-5, "guidance_dir h=%g v=(%g %g %g): err %g", (double)c.h, (double)c.vx,
              (double)c.vy, (double)c.vz, e);
        CHECK(near(tgo, c.tgo, 1e-4 * (1 + c.tgo)), "t_go %g want %g", (double)tgo, (double)c.tgo);
    }
    std::printf("  guidance direction: %zu cases, worst error %.2e\n",
                sizeof(GOLD_DIR) / sizeof(GOLD_DIR[0]), worst);

    worst = 0;
    for (const auto &c : GOLD_PROJ) {
        float v = dg_project_vz(c.h, c.vz, c.tb, c.k, c.ts, nullptr, c.ts_inf, c.ts_tau);
        double e = std::fabs(v - c.vz_td);
        if (e > worst) worst = e;
        // Past the plan hold the simulation decides "t - t0 > 0.30" on an
        // accumulated double; whether step 15 still holds the level depends on
        // its rounding. One step of clamp difference is worth up to
        // 120 N * (1 - k) / m * 20 ms, hence the k-dependent allowance.
        CHECK(e < 0.02 + 1.2 * (1.0 - c.k), "project h=%g vz=%g tb=%g k=%g ts=%g: %g want %g", (double)c.h,
              (double)c.vz, (double)c.tb, (double)c.k, (double)c.ts, (double)v, (double)c.vz_td);
    }
    std::printf("  projection: %zu cases, worst error %.4f m/s\n",
                sizeof(GOLD_PROJ) / sizeof(GOLD_PROJ[0]), worst);

    worst = 0;
    for (const auto &c : GOLD_PLAN) {
        float resid, ttd;
        float k = dg_solve_plan(c.h, c.vz, c.tb, c.ts, &resid, &ttd, c.ts_inf, c.ts_tau);
        // Where the target is reachable the level must match; where it is
        // not, the peak is flat and what must match is the achieved speed.
        if (c.resid >= 0.0f) {
            double e = std::fabs(k - c.k);
            if (e > worst) worst = e;
            CHECK(e < 0.01, "plan h=%g vz=%g tb=%g ts=%g: k=%g want %g", (double)c.h,
                  (double)c.vz, (double)c.tb, (double)c.ts, (double)k, (double)c.k);
        }
        CHECK(near(resid, c.resid, c.resid >= 0.0f ? 0.05 : 1.0), "plan resid %g want %g (h=%g vz=%g tb=%g)",
              (double)resid, (double)c.resid, (double)c.h, (double)c.vz, (double)c.tb);
        CHECK(ttd > 0.0f && ttd < 30.0f, "plan t_td %g", (double)ttd);
    }
    std::printf("  plan solver: %zu cases, worst level error %.4f\n",
                sizeof(GOLD_PLAN) / sizeof(GOLD_PLAN[0]), worst);

    for (const auto &c : GOLD_TERM) {
        CHECK(dg_terminal_active(c.h, c.vz, c.thrust, c.mass) == (c.active > 0.5f),
              "terminal gate h=%g vz=%g T=%g", (double)c.h, (double)c.vz, (double)c.thrust);
        CHECK(near(dg_terminal_throttle(c.h, c.vz, c.mass, c.thrust), c.k, 1e-4),
              "terminal throttle h=%g vz=%g", (double)c.h, (double)c.vz);
    }
    // The ignition altitude: the band from the table, the fall propagated
    // here - against the simulation's own find_ignition() from the apogee.
    double worst_ign = 0;
    for (const auto &c : GOLD_IGN) {
        float h = dg_ign_compute(c.h0, 0.0f, c.vx, 1.0f, nullptr, nullptr, nullptr, nullptr);
        double e = std::fabs(h - c.h_cmd);
        if (e > worst_ign) worst_ign = e;
        CHECK(e < 1.0, "h_cmd(%g,%g)=%g want %g", (double)c.h0, (double)c.vx, (double)h, (double)c.h_cmd);
    }
    std::printf("  ignition altitude vs find_ignition(): %zu cases, worst %.2f m\n",
                sizeof(GOLD_IGN) / sizeof(GOLD_IGN[0]), worst_ign);
    // a fall started lower down along the same trajectory gives the same answer:
    // the search depends on the fall, not on where it was started from
    {
        float t_to, vz_at;
        float h1 = dg_ign_compute(150.0f, 0.0f, 0.0f, 1.0f, &t_to, &vz_at, nullptr, nullptr);
        // propagate the same fall 3 s by hand, then search again
        float h = 150.0f, vz = 0.0f;
        const float kd = 0.5f * 1.225f * DG_AREA * (DG_CD_BODY + DG_FIN_CD_BRAKE) / DG_M_GROSS;
        for (int i = 0; i < 600; i++) { float a = -9.80665f + kd * vz * vz; vz += a * 0.005f; h += vz * 0.005f; }
        float h2 = dg_ign_compute(h, vz, 0.0f, 1.0f, nullptr, nullptr, nullptr, nullptr);
        CHECK(std::fabs(h1 - h2) < 0.3, "ignition search not path-consistent: %g vs %g", (double)h1, (double)h2);
        CHECK(t_to > 2.0f && vz_at < -20.0f, "t_to %g vz_at %g", (double)t_to, (double)vz_at);
        // more drag -> slower at the band -> a LOWER ignition altitude
        float h3 = dg_ign_compute(150.0f, 0.0f, 0.0f, 1.4f, nullptr, nullptr, nullptr, nullptr);
        // (often capped by the band's upper edge, so it may barely move - but never up)
        CHECK(h3 <= h1 + 0.05f, "more drag must not raise the ignition altitude: %g vs %g", (double)h3, (double)h1);
    }

    // landsim_export.py writes h_min = 0 where there is no band at all. The
    // vehicle must still light (best effort), never fall to the ground first.
    {
        dg_params_t prm;
        dg_params_default(&prm);
        int n0 = 0;
        for (int i = 0; i < DG_IGN_N; i++)
            if (prm.tbl_hmax[i] <= 0.0f) { prm.tbl_hmin[i] = 0.0f; n0++; }
        dg_set_params(&prm);
        float worst = 0.0f;
        for (int i = 0; i < DG_IGN_N; i++) {
            if (DG_IGN_HMAX[i] > 0.0f) continue;
            float hm;
            float lo = dg_ign_band(DG_IGN_VZ0 + DG_IGN_DVZ * i, &hm);
            CHECK(lo > 20.0f && hm <= 0.0f, "no-band entry %d: h_min %g (must extrapolate)", i, (double)lo);
            float e = std::fabs(lo - DG_IGN_HMIN[i]);
            if (e > worst) worst = e;
        }
        CHECK(n0 > 0 && worst < 6.0f, "extrapolated best effort %g m from the generator's", (double)worst);
        std::printf("  no-band entries as exported (h_min = 0): %d, extrapolated within %.1f m\n", n0, (double)worst);
        dg_params_default(&prm);
        dg_set_params(&prm);
    }
}

// ---------------------------------------------------------------------
//  structural properties
// ---------------------------------------------------------------------
static void test_properties()
{
    dg_init();
    float ux, uy, uz, vx2, vy2, vz2;
    // drifting +x must command a tilt toward -x
    dg_guidance_dir(30, 6, 0, -30, 2.8f, &ux, &uy, &uz);
    CHECK(ux < -0.01f && std::fabs(uy) < 1e-6f && uz > 0.9f, "tilt sign: u=(%g %g %g)",
          (double)ux, (double)uy, (double)uz);
    // isotropy: rotating the drift rotates the command
    dg_guidance_dir(30, 0, 6, -30, 2.8f, &vx2, &vy2, &vz2);
    CHECK(near(ux, vy2, 1e-6) && near(uy, vx2, 1e-6) && near(uz, vz2, 1e-6), "isotropy");
    // the cone at the pad is tilt_min
    dg_guidance_dir(0.0f, 5, 0, -1, 2.8f, &ux, &uy, &uz);
    CHECK(near(std::acos(uz) * 57.29578, 1.5, 0.02), "cone floor %g deg",
          std::acos(uz) * 57.29578);

    // gimbal basis: orthonormal, perpendicular to b, follows the roll
    float a[3], c[3];
    dg_gimbal_basis(0.12f, 0.05f, 0.99f, 0.9f, 0.1f, -0.1f, &a[0], &a[1], &a[2], &c[0], &c[1], &c[2]);
    CHECK(std::fabs(a[0] * c[0] + a[1] * c[1] + a[2] * c[2]) < 1e-5, "basis not orthogonal");
    CHECK(near(a[0] * a[0] + a[1] * a[1] + a[2] * a[2], 1, 1e-5), "basis not unit");
    dg_gimbal_basis(0, 0, 1, 0, 1, 0, &a[0], &a[1], &a[2], &c[0], &c[1], &c[2]);
    CHECK(near(a[1], 1, 1e-5) && near(c[0], -1, 1e-5), "basis does not follow roll");

    // fin mixer: body axis vertical, falling at 30 m/s
    const float b[3] = { 0, 0, 1 }, u1[3] = { 1, 0, 0 }, u2[3] = { 0, 1, 0 };
    const float v[3] = { 0, 0, -30 };
    const float z3[3] = { 0, 0, 0 };
    float d[4];
    // pure brake: alternating, equal magnitude, sums to zero
    dg_fin_mix(b, u1, u2, v, z3, 0.0f, 15.0f, 1.0f, z3, true, d);
    CHECK(near(d[0], 15, 1e-3) && near(d[1], -15, 1e-3) && near(d[2], 15, 1e-3) &&
          near(d[3], -15, 1e-3), "brake pattern %g %g %g %g", (double)d[0], (double)d[1],
          (double)d[2], (double)d[3]);
    // roll demand: all four the same way, capped at roll_max
    dg_fin_mix(b, u1, u2, v, z3, 1.0f, 0.0f, 1.0f, z3, true, d);
    CHECK(near(d[0], 2, 1e-3) && near(d[1], 2, 1e-3) && near(d[2], 2, 1e-3) && near(d[3], 2, 1e-3),
          "roll pattern %g %g %g %g", (double)d[0], (double)d[1], (double)d[2], (double)d[3]);
    // a torque about +u1 comes out as B only (fins 1 and 3, opposite signs),
    // and NEGATIVE: the fins are aft of the CG
    const float tau[3] = { 0.05f, 0, 0 };
    dg_fin_mix(b, u1, u2, v, tau, 0.0f, 0.0f, 1.0f, z3, true, d);
    CHECK(d[0] < -0.1f && near(d[2], -d[0], 1e-4) && std::fabs(d[1]) < 1e-4 && std::fabs(d[3]) < 1e-4,
          "steer pattern %g %g %g %g", (double)d[0], (double)d[1], (double)d[2], (double)d[3]);
    // weaker fins (kappa 0.5) must be commanded twice as far
    float d2[4];
    dg_fin_mix(b, u1, u2, v, tau, 0.0f, 0.0f, 0.5f, z3, true, d2);
    CHECK(near(d2[0], 2 * d[0], 1e-3), "kappa scaling %g vs %g", (double)d2[0], (double)d[0]);
    // below the authority gate nothing but the brake moves
    const float vslow[3] = { 0, 0, -4 };
    dg_fin_mix(b, u1, u2, vslow, tau, 1.0f, 0.0f, 1.0f, z3, true, d);
    CHECK(std::fabs(d[0]) < 1e-5 && std::fabs(d[1]) < 1e-5, "authority gate");
    // stall limiter: a huge demand with a sideslip never exceeds stall - margin of AoA
    const float vslip[3] = { 8, 0, -30 };
    const float big[3] = { 0, 50, 0 };
    dg_fin_mix(b, u1, u2, vslip, big, 0.0f, 0.0f, 1.0f, z3, true, d);
    for (int i = 0; i < 4; i++) CHECK(std::fabs(d[i]) <= 15.0001f, "travel limit fin %d = %g", i, (double)d[i]);
}

// ---------------------------------------------------------------------
//  B. the plant
// ---------------------------------------------------------------------
struct Plant {
    // configuration of this flight
    double h0 = 150, vx0 = 0, vy0 = 0;
    double ign_delay = 0.15;
    double thrust_scale = 1.0;
    double roll0 = 0.0;              // rad/s
    double fin_kappa = 1.0;          // real fin lift / modelled
    double fin_offset[4] = { 0, 0, 0, 0 };   // deg, built-in misalignment
    double alt_bias = 0.0;           // m, removed at lidar_h
    double lidar_h = 7.0;
    double wind[3] = { 0, 0, 0 };
    double misalign = 0.0;           // rad, thrust line offset toward u1
    bool   igniter_fails = false;
    bool   accel_dead = false;       // the axial accelerometer reads its baseline forever
    double drag_mult = 1.0;          // real drag / modelled (body and fins)
    double h0_drop = 0.0;            // >0: released at this altitude instead of apogee knowledge
    // results
    double td_vz = 0, td_vh = 0, td_tilt = 0, td_rate = 0, td_t = 0;
    double max_roll = 0, max_tilt_ff = 0, h_ign = 0, t_onset = -1, vh_at_ign = 0;
    double kappa_end = 0, trim_end[3] = { 0, 0, 0 }, drag_scale_end = 0;
    bool   landed = false;
    double t_cover = -1, t_legs = -1, t_cmd = -1, t_td_pred_at_legs = 0;
};

static void cross(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
static double dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static void fly(Plant &p, const dg_params_t *params)
{
    const double G = 9.80665, RHO = 1.225, DT = 0.001;
    const double area = DG_AREA, cd = DG_CD_BODY, cna = DG_CN_ALPHA, lcp = DG_L_CP;
    const double fin_area = DG_FIN_AREA, fin_arm = DG_FIN_ARM, roll_arm = DG_FIN_ROLL_ARM;
    const double fin_cla = DG_FIN_CL_ALPHA, fin_kint = DG_FIN_K_INT, stall = 22.0 * M_PI / 180;

    dg_init();
    if (params) dg_set_params(params);
    else { dg_params_t d; dg_params_default(&d); dg_set_params(&d); }
    const dg_params_t *P = dg_get_params();

    double pos[3] = { 0, 0, p.h0 }, vel[3] = { p.vx0, p.vy0, 0 };
    double b[3] = { 0.02, 0.01, 1 }, g[3] = { 1, 0, 0 };
    { double n = std::sqrt(dot(b, b)); for (double &x : b) x /= n; }
    double w[3] = { p.roll0 * b[0], p.roll0 * b[1], p.roll0 * b[2] };
    double srv1 = 0, srv2 = 0, defl[4] = { 0, 0, 0, 0 }, k_act = 1.0;
    double a_axial = 0;
    double t = 0, t_cmd = -1, t_burn0 = -1;
    bool acquired = false;
    dg_out_t out;
    std::memset(&out, 0, sizeof(out));
    dg_nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    double cmd_hist[4][7];          // 15 ms of transport delay: servos + fins + clamp
    std::memset(cmd_hist, 0, sizeof(cmd_hist));
    for (auto &c : cmd_hist) c[6] = 1.0;
    int hist_pos = 0;

    dg_begin_descent((float)p.h0, (float)std::hypot(p.vx0, p.vy0));

    for (long step = 0; step < 60000; step++) {
        if (!acquired && pos[2] <= p.lidar_h) acquired = true;
        double tb = t_burn0 >= 0 ? t - t_burn0 : -1;

        if (step % 5 == 0) {
            nav.h  = (float)(pos[2] + (acquired ? 0.0 : p.alt_bias));
            nav.vx = (float)vel[0];  nav.vy = (float)vel[1];  nav.vz = (float)vel[2];
            nav.bx = (float)b[0];  nav.by = (float)b[1];  nav.bz = (float)b[2];
            nav.gx = (float)g[0];  nav.gy = (float)g[1];  nav.gz = (float)g[2];
            nav.wx = (float)w[0];  nav.wy = (float)w[1];  nav.wz = (float)w[2];
            nav.a_axial = (float)(p.accel_dead ? 7.0 : a_axial);
            nav.lidar_ok = acquired;
            nav.valid = true;
            dg_step_control(&nav, &out);
            dg_step_planner(&nav);

            if (out.ignite && t_cmd < 0) { t_cmd = t; p.t_cmd = t; p.h_ign = pos[2]; }
            // the sequencer's two countdown events, as the Navigation MCU derives them
            if (p.t_cover < 0 && out.phase == DG_PHASE_DESCENT &&
                out.t_to_ignition >= 0 && out.t_to_ignition <= 0.6f) p.t_cover = t;
            if (p.t_legs < 0 && out.t_to_touchdown >= 0 && out.t_to_touchdown <= 1.7f &&
                out.phase >= DG_PHASE_DESCENT) { p.t_legs = t; p.t_td_pred_at_legs = out.t_to_touchdown; }

            cmd_hist[hist_pos][0] = out.servo1_deg;
            cmd_hist[hist_pos][1] = out.servo2_deg;
            for (int i = 0; i < 4; i++) cmd_hist[hist_pos][2 + i] = out.fins_live ? out.fin_deg[i] : 0.0;
            cmd_hist[hist_pos][6] = out.clamp;
            hist_pos = (hist_pos + 1) & 3;
        }
        const double *act = cmd_hist[(hist_pos + 1) & 3];     // 3 ticks = 15 ms old

        if (t_cmd >= 0 && t_burn0 < 0 && !p.igniter_fails && t - t_cmd >= p.ign_delay) {
            t_burn0 = t;
            p.t_onset = t;
            p.vh_at_ign = std::hypot(vel[0], vel[1]);
            tb = 0;
        }
        bool engine_on = tb >= 0 && tb <= DG_BURN_T;

        // actuators: rate limited
        auto slew = [&](double &x, double target, double rate) {
            double d = target - x, s = rate * DT;
            x += d > s ? s : (d < -s ? -s : d);
        };
        slew(srv1, act[0], 500.0);
        slew(srv2, act[1], 500.0);
        for (int i = 0; i < 4; i++) slew(defl[i], act[2 + i], 600.0);
        slew(k_act, act[6], 12.84);

        double mass = DG_M_GROSS - (tb > 0 ? dg_burned((float)tb) : 0.0);
        double thrust = engine_on ? dg_thrust_ref((float)tb) * p.thrust_scale * k_act : 0.0;

        // body axes
        double u1[3], u2[3];
        { double d = dot(g, b); for (int i = 0; i < 3; i++) u1[i] = g[i] - d * b[i];
          double n = std::sqrt(dot(u1, u1)); for (double &x : u1) x /= n; cross(b, u1, u2); }

        // thrust direction: body axis tilted by the nozzle deflection
        double s1 = std::sin(srv1 / DG_TVC_RATIO * M_PI / 180) + std::sin(p.misalign);
        double s2 = std::sin(srv2 / DG_TVC_RATIO * M_PI / 180);
        double sv[3], td[3];
        for (int i = 0; i < 3; i++) sv[i] = s1 * u1[i] + s2 * u2[i];
        double ca_n = std::sqrt(1 - dot(sv, sv));
        for (int i = 0; i < 3; i++) td[i] = ca_n * b[i] + sv[i];

        // aerodynamics, on the velocity through the air
        double rv[3] = { vel[0] - p.wind[0], vel[1] - p.wind[1], vel[2] - p.wind[2] };
        double v2 = dot(rv, rv), vr = std::sqrt(v2), q = 0.5 * RHO * v2;
        double F[3] = { 0, 0, 0 }, T[3] = { 0, 0, 0 };
        if (vr > 0.05) {
            double ah[3] = { -rv[0] / vr, -rv[1] / vr, -rv[2] / vr };
            double ca = dot(ah, b);
            double c[3];
            cross(b, ah, c);
            double fnk = q * area * cna * std::fabs(ca);
            double cdamp = 0.5 * RHO * vr * area * cna * lcp * lcp;
            double wr = dot(w, b);
            for (int i = 0; i < 3; i++) {
                F[i] += q * area * cd * p.drag_mult * ca * b[i] + fnk * (ah[i] - ca * b[i]);
                T[i] += lcp * fnk * c[i] - cdamp * (w[i] - wr * b[i]);
            }
            // viscous crossflow on the body, at the CG
            double sa = std::sqrt(std::fmax(0.0, 1 - ca * ca));
            double fvk = q * 0.8 * (1.4 * 0.105) * sa;
            for (int i = 0; i < 3; i++) F[i] += fvk * (ah[i] - ca * b[i]);
        }
        // fins
        for (int i = 0; i < 4; i++) {
            double ang = M_PI / 2 * i, r[3], n[3], pp[3], l[3], wxp[3];
            for (int k = 0; k < 3; k++) r[k] = std::cos(ang) * u1[k] + std::sin(ang) * u2[k];
            cross(b, r, n);
            for (int k = 0; k < 3; k++) pp[k] = fin_arm * b[k] + roll_arm * r[k];
            cross(w, pp, wxp);
            for (int k = 0; k < 3; k++) l[k] = rv[k] + wxp[k];
            double l2 = dot(l, l);
            if (l2 < 1.0) continue;
            double lv = std::sqrt(l2);
            double va = -dot(l, b);
            if (va < 2.0) va = 2.0;
            double alpha = (defl[i] + p.fin_offset[i]) * M_PI / 180 - dot(l, n) / va;
            double cl;
            if (std::fabs(alpha) <= stall) cl = fin_cla * alpha;
            else cl = (alpha > 0 ? 1 : -1) * fin_cla * stall * std::exp(-(std::fabs(alpha) - stall) * 1.5);
            double cdf = 0.048 + 0.45 * cl * cl;
            double lift = 0.5 * RHO * l2 * fin_area * cl * fin_kint * p.fin_kappa;
            double drag = 0.5 * RHO * l2 * fin_area * cdf * p.drag_mult;
            double ln = dot(l, n) / l2, pr[3];
            for (int k = 0; k < 3; k++) pr[k] = n[k] - ln * l[k];
            double pn = std::sqrt(dot(pr, pr));
            if (pn < 1e-9) pn = 1e-9;
            double ff[3], tq[3];
            for (int k = 0; k < 3; k++) ff[k] = lift * pr[k] / pn - drag * l[k] / lv;
            cross(pp, ff, tq);
            for (int k = 0; k < 3; k++) { F[k] += ff[k]; T[k] += tq[k]; }
        }
        // thrust force and gimbal moment (-L b) x F
        double Ft[3], tg[3], bs[3];
        for (int i = 0; i < 3; i++) Ft[i] = thrust * td[i];
        cross(b, sv, bs);
        for (int i = 0; i < 3; i++) tg[i] = -DG_L_GIMBAL * thrust * bs[i];

        double fsum[3];
        for (int i = 0; i < 3; i++) fsum[i] = Ft[i] + F[i];
        a_axial = dot(fsum, b) / mass;

        double It = mass * DG_IT_PER_KG, Ia = mass * DG_IA_PER_KG;
        double wr = dot(w, b), wt[3], bw[3], tot[3], al[3];
        for (int i = 0; i < 3; i++) wt[i] = w[i] - wr * b[i];
        cross(b, wt, bw);
        for (int i = 0; i < 3; i++) tot[i] = tg[i] + T[i] - (It - Ia) * wr * bw[i];
        double trr = dot(tot, b);
        for (int i = 0; i < 3; i++) al[i] = (tot[i] - trr * b[i]) / It + trr * b[i] / Ia;

        // statistics
        double tilt = std::acos(std::fmax(-1.0, std::fmin(1.0, b[2]))) * 180 / M_PI;
        if (t > 2.0 && std::fabs(wr) > p.max_roll) p.max_roll = std::fabs(wr);
        if (t > 3.0 && !engine_on && tilt > p.max_tilt_ff) p.max_tilt_ff = tilt;

        // integrate
        double dts = DT;
        bool touchdown = pos[2] + vel[2] * DT <= 0.0;
        if (touchdown && vel[2] < -1e-9) dts = std::fmin(DT, pos[2] / -vel[2]);
        for (int i = 0; i < 3; i++) vel[i] += (fsum[i] / mass - (i == 2 ? G : 0)) * dts;
        for (int i = 0; i < 3; i++) pos[i] += vel[i] * dts;
        for (int i = 0; i < 3; i++) w[i] += al[i] * dts;
        double db[3], dg_[3];
        cross(w, b, db);
        cross(w, g, dg_);
        for (int i = 0; i < 3; i++) { b[i] += db[i] * dts; g[i] += dg_[i] * dts; }
        { double n = std::sqrt(dot(b, b)); for (double &x : b) x /= n;
          double d = dot(g, b); for (int i = 0; i < 3; i++) g[i] -= d * b[i];
          n = std::sqrt(dot(g, g)); for (double &x : g) x /= n; }
        t += dts;

        if (touchdown || pos[2] <= 0.0) {
            p.landed = true;
            p.td_vz = vel[2];
            p.td_vh = std::hypot(vel[0], vel[1]);
            p.td_tilt = std::acos(std::fmax(-1.0, std::fmin(1.0, b[2]))) * 180 / M_PI;
            double wr2 = dot(w, b), wt2[3];
            for (int i = 0; i < 3; i++) wt2[i] = w[i] - wr2 * b[i];
            p.td_rate = std::sqrt(dot(wt2, wt2)) * 180 / M_PI;
            p.td_t = t;
            break;
        }
    }
    p.kappa_end = out.fin_gain;
    p.drag_scale_end = out.drag_scale;
    for (int i = 0; i < 3; i++) p.trim_end[i] = out.fin_trim[i];
    (void)P;
}

static void report(const char *name, const Plant &p)
{
    std::printf("  %-34s ign %5.1f m  td vz %6.2f vh %5.2f tilt %4.1f rate %5.1f  roll<=%4.0f dps"
                "  kappa %.2f  drag %.2f\n", name, p.h_ign, p.td_vz, p.td_vh, p.td_tilt, p.td_rate,
                p.max_roll * 180 / M_PI, p.kappa_end, p.drag_scale_end);
    std::printf("      legs %.2f s before touchdown, cover %.2f s before the command\n", p.t_legs > 0 ? p.td_t - p.t_legs : -1.0, p.t_cover > 0 ? p.t_cmd - p.t_cover : -1.0);
}

static void test_closed_loop()
{
    // ---- nominal ----
    {
        Plant p;
        fly(p, nullptr);
        report("nominal 150 m", p);
        CHECK(p.landed, "nominal: did not land");
        CHECK(p.td_vz > -4.0 && p.td_vz <= 0.0, "nominal: touchdown vz %g", p.td_vz);
        CHECK(p.td_vh < 0.5, "nominal: touchdown vh %g", p.td_vh);
        CHECK(p.td_tilt < 4.0, "nominal: touchdown tilt %g", p.td_tilt);
        CHECK(p.td_rate < 30.0, "nominal: touchdown rate %g", p.td_rate);
        CHECK(p.max_tilt_ff < 15.0, "nominal: free-fall tilt %g (fin loop unstable?)", p.max_tilt_ff);
        CHECK(near(p.h_ign, dg_h_ignite() + 1.0, 0.3), "nominal: ignited at %g, table %g + 1 m pad",
              p.h_ign, (double)dg_h_ignite());
        // the sequencer's countdowns: cover 0.6 s before the command, legs 1.7 s before touchdown
        CHECK(p.t_cover > 0 && near(p.t_cmd - p.t_cover, 0.6, 0.08), "cover lead %g s",
              p.t_cmd - p.t_cover);
        // The touchdown is a PREDICTION of a 2.5 s burn on a motor known to
        // +-15 %: over the whole envelope it lands within 1.5..2.3 s of the
        // 1.7 s asked for, and almost always early - legs out before the
        // ground, never after it.
        CHECK(p.t_legs > 0 && p.td_t - p.t_legs > 1.45 && p.td_t - p.t_legs < 2.3,
              "legs lead %g s (predicted %g)", p.td_t - p.t_legs, p.t_td_pred_at_legs);
    }
    // ---- the entry envelope, igniter spread, thrust scatter, roll ----
    struct Case { const char *name; double h0, vx, vy, delay, ts, roll; };
    const Case cases[] = {
        { "140 m, slow igniter",           140, 0, 0, 0.30, 1.00, 0.0 },
        { "180 m, fast igniter",           180, 0, 0, 0.02, 1.00, 0.0 },
        { "160 m, drift +5 m/s x",         160, 5, 0, 0.15, 1.00, 0.0 },
        { "160 m, drift -4 m/s y",         160, 0, -4, 0.15, 1.00, 0.0 },
        { "150 m, motor +8 %",             150, 0, 0, 0.15, 1.08, 0.0 },
        { "150 m, motor -6 %",             150, 2, 1, 0.10, 0.94, 0.0 },
        { "150 m, roll +90 deg/s",         150, 2, 0, 0.15, 1.00, 1.5708 },
        { "170 m, roll -90 deg/s, drift",  170, -3, 3, 0.20, 1.00, -1.5708 },
    };
    for (const Case &c : cases) {
        Plant p;
        p.h0 = c.h0; p.vx0 = c.vx; p.vy0 = c.vy; p.ign_delay = c.delay;
        p.thrust_scale = c.ts; p.roll0 = c.roll;
        fly(p, nullptr);
        report(c.name, p);
        CHECK(p.landed && p.td_vz > -4.5, "%s: vz %g", c.name, p.td_vz);
        CHECK(p.td_vh < 0.8, "%s: vh %g", c.name, p.td_vh);
        CHECK(p.td_tilt < 5.0, "%s: tilt %g", c.name, p.td_tilt);
        CHECK(p.td_rate < 30.0, "%s: rate %g", c.name, p.td_rate);
        CHECK(p.max_roll < 2.5, "%s: roll rate peaked at %g deg/s", c.name, p.max_roll * 180 / M_PI);
        CHECK(p.t_legs > 0 && p.td_t - p.t_legs > 1.45 && p.td_t - p.t_legs < 2.3,
              "%s: legs %g s before touchdown", c.name, p.td_t - p.t_legs);
        CHECK(p.t_cover > 0 && near(p.t_cmd - p.t_cover, 0.6, 0.08), "%s: cover lead %g s",
              c.name, p.t_cmd - p.t_cover);
    }
    // ---- altitude bias, corrected by the lidar at 7 m ----
    for (double bias : { +1.0, -1.0 }) {
        Plant p;
        p.alt_bias = bias;
        fly(p, nullptr);
        report(bias > 0 ? "altimeter reads 1 m high" : "altimeter reads 1 m low", p);
        CHECK(p.landed && p.td_vz > -4.5, "alt bias %+g: vz %g", bias, p.td_vz);
    }
    // ---- a misaligned thrust line: what the attitude integral is for ----
    {
        Plant p;
        p.misalign = 0.25 * M_PI / 180;
        fly(p, nullptr);
        report("thrust misaligned 0.25 deg", p);
        CHECK(p.landed && p.td_vz > -4.5 && p.td_vh < 0.8 && p.td_tilt < 5.0,
              "misalign: vz %g vh %g tilt %g", p.td_vz, p.td_vh, p.td_tilt);
    }
    // ---- wind ----
    {
        Plant p;
        p.wind[0] = 1.5; p.wind[1] = -1.0;
        fly(p, nullptr);
        report("wind 1.8 m/s", p);
        CHECK(p.landed && p.td_vz > -4.5 && p.td_tilt < 6.0, "wind: vz %g tilt %g", p.td_vz, p.td_tilt);
    }
    // ---- FIN ADAPTATION: fins at 60 % of the model, and mis-trimmed ----
    {
        Plant p;
        p.h0 = 170; p.vx0 = 3;
        p.fin_kappa = 0.6;
        p.fin_offset[0] = 1.5; p.fin_offset[2] = -1.5;     // a steering trim on axis 1
        for (double &o : p.fin_offset) o += 0.8;            // and a roll trim
        fly(p, nullptr);
        report("fins 60 %, trim 1.5 deg + roll 0.8", p);
        CHECK(p.landed && p.td_vz > -4.5 && p.td_vh < 0.8 && p.td_tilt < 5.0,
              "weak fins: vz %g vh %g tilt %g", p.td_vz, p.td_vh, p.td_tilt);
        CHECK(p.kappa_end < 0.9, "weak fins: kappa estimate %g did not come down", p.kappa_end);
        CHECK(p.trim_end[2] > 0.2, "roll trim estimate %g deg, plant has +0.8", p.trim_end[2]);
        CHECK(p.max_roll < 2.5, "weak fins: roll peaked at %g deg/s", p.max_roll * 180 / M_PI);
        std::printf("    estimates: kappa %.2f (plant 0.60), trims %.2f %.2f %.2f deg (plant 1.5, 0, 0.8)\n",
                    p.kappa_end, p.trim_end[0], p.trim_end[1], p.trim_end[2]);

        // the same flight with the adaptation OFF must still fly, only worse
        dg_params_t prm;
        dg_params_default(&prm);
        prm.fin_adapt = 0.0f;
        Plant p2 = Plant();
        p2.h0 = 170; p2.vx0 = 3; p2.fin_kappa = 0.6;
        p2.fin_offset[0] = 1.5 + 0.8; p2.fin_offset[1] = 0.8; p2.fin_offset[2] = -1.5 + 0.8; p2.fin_offset[3] = 0.8;
        fly(p2, &prm);
        report("  ... same, adaptation off", p2);
        CHECK(p2.landed, "weak fins, no adaptation: did not land");
    }
    // ---- fins stronger than the model ----
    {
        Plant p;
        p.h0 = 170; p.vx0 = 3;
        p.fin_kappa = 1.5;
        fly(p, nullptr);
        report("fins 150 %", p);
        CHECK(p.landed && p.td_vz > -4.5 && p.td_tilt < 5.0, "strong fins: vz %g tilt %g", p.td_vz, p.td_tilt);
        CHECK(p.kappa_end > 1.1, "strong fins: kappa estimate %g did not go up", p.kappa_end);
    }
    // ---- the drag is not what the model says: measured and used ----
    for (double dm : { 0.7, 1.4 }) {
        Plant p;
        p.drag_mult = dm;
        fly(p, nullptr);
        char nm[64];
        std::snprintf(nm, sizeof nm, "drag x%.1f (measured in flight)", dm);
        report(nm, p);
        CHECK(p.landed && p.td_vz > -4.5 && p.td_tilt < 5.0, "%s: vz %g tilt %g", nm, p.td_vz, p.td_tilt);
        CHECK((dm > 1 && p.drag_scale_end > 1.1) || (dm < 1 && p.drag_scale_end < 0.9),
              "%s: drag estimate %g did not move", nm, p.drag_scale_end);
    }
    // ---- higher apogees than the old table covered ----
    for (double h0 : { 200.0, 220.0 }) {
        Plant p;
        p.h0 = h0;
        fly(p, nullptr);
        char nm[64];
        std::snprintf(nm, sizeof nm, "apogee %.0f m", h0);
        report(nm, p);
        CHECK(p.landed, "%s: did not land", nm);
    }

    // ---- no fins at all: the nozzle alone must still fly the burn ----
    {
        dg_params_t prm;
        dg_params_default(&prm);
        prm.fin_enable = 0.0f;
        Plant p;
        fly(p, &prm);
        report("fins disabled", p);
        CHECK(p.landed, "no fins: did not land");
    }
    // ---- the accelerometer path dead: onset is assumed, the burn is still flown ----
    {
        Plant p;
        p.accel_dead = true;
        fly(p, nullptr);
        report("axial accelerometer dead", p);
        CHECK(p.landed && p.td_vz > -4.5, "accel dead: vz %g", p.td_vz);
    }
}

// ---------------------------------------------------------------------
//  safety properties of the state machine
// ---------------------------------------------------------------------
static void test_state_machine()
{
    dg_init();
    dg_params_t prm;
    dg_params_default(&prm);
    dg_set_params(&prm);
    dg_nav_t n;
    std::memset(&n, 0, sizeof(n));
    dg_out_t o;
    n.h = 100; n.vz = -30; n.bz = 1; n.gx = 1; n.valid = true; n.a_axial = 8;

    // IDLE: nothing moves, nothing fires, whatever the state says
    n.h = 10;
    for (int i = 0; i < 50; i++) dg_step_control(&n, &o);
    CHECK(!o.ignite && o.phase == DG_PHASE_IDLE && !o.fins_live && o.servo1_deg == 0, "IDLE is not inert");

    // ASCENT: no ignition even below the ignition altitude
    dg_begin_ascent();
    n.vz = +40; n.bz = -1; n.h = 30;
    for (int i = 0; i < 50; i++) dg_step_control(&n, &o);
    CHECK(!o.ignite && o.phase == DG_PHASE_ASCENT, "ignition during ascent");

    // DESCENT above h_cmd: no ignition; countdown is sane and decreasing
    dg_begin_descent(150, 0);
    CHECK(near(dg_h_ignite(), dg_ign_compute(150, 0, 0, 1, nullptr, nullptr, nullptr, nullptr), 1e-4),
          "h_cmd not from the ignition search");
    n.vz = -25; n.bz = 1; n.h = 90;
    dg_step_control(&n, &o);
    float t1 = o.t_to_ignition;
    n.h = 80;
    dg_step_control(&n, &o);
    CHECK(!o.ignite && t1 > 0.5f && o.t_to_ignition < t1, "countdown %g -> %g", (double)t1,
          (double)o.t_to_ignition);

    // invalid navigation: no ignition even below h_cmd
    n.h = 40; n.valid = false;
    for (int i = 0; i < 20; i++) dg_step_control(&n, &o);
    CHECK(!o.ignite, "ignited on invalid navigation");
    // thrust axis pointing down: no ignition
    n.valid = true; n.bz = -0.5f; n.bx = 0.866f;
    for (int i = 0; i < 20; i++) dg_step_control(&n, &o);
    CHECK(!o.ignite, "ignited with the thrust axis below the horizon");
    // and now everything is right
    n.bz = 1; n.bx = 0;
    dg_step_control(&n, &o);
    CHECK(o.ignite && o.phase == DG_PHASE_IGNITING && !o.thrust_on, "no ignition at h_cmd");
    CHECK(o.clamp == DG_CLAMP_MAX, "clamp not open through the startup");
    // nozzle parked until thrust is detected
    CHECK(o.servo1_deg == 0 && o.servo2_deg == 0, "nozzle moving with the motor unlit");
    // the bump
    n.a_axial = 8 + 6;
    dg_step_control(&n, &o);
    dg_step_control(&n, &o);
    CHECK(o.phase == DG_PHASE_BURN && o.thrust_on, "thrust onset not detected");

    // the altitude pad: with no lidar the guidance works 1 m low
    dg_init();
    dg_begin_descent(150, 0);
    n.a_axial = 8; n.vz = -25; n.lidar_ok = false;
    n.h = dg_h_ignite() + 0.9f;                // inside the pad -> ignites
    dg_step_control(&n, &o);
    CHECK(o.ignite, "altitude pad not applied");
    dg_init();
    dg_begin_descent(150, 0);
    n.lidar_ok = true;                          // lidar referenced -> no pad
    dg_step_control(&n, &o);
    CHECK(!o.ignite, "altitude pad not dropped with the lidar");

    // override and bias
    dg_init();
    prm.h_ign_bias = 3.0f;
    dg_set_params(&prm);
    dg_begin_descent(150, 0);
    CHECK(near(dg_h_ignite(), dg_ign_compute(150, 0, 0, 1, nullptr, nullptr, nullptr, nullptr) + 3.0, 1e-3),
          "h_ign_bias");
    dg_init();
    prm.h_ign_override = 42.0f;
    dg_set_params(&prm);
    dg_begin_descent(150, 0);
    CHECK(near(dg_h_ignite(), 42.0, 1e-4), "h_ign_override");
    dg_params_default(&prm);
    dg_set_params(&prm);
}

// ---------------------------------------------------------------------
//  bench polarity test (TVC landing 0 deg + fins ascent 0 deg)
// ---------------------------------------------------------------------
static void test_polarity()
{
    dg_init();
    dg_params_t prm;
    dg_params_default(&prm);
    dg_set_params(&prm);

    dg_nav_t n;
    dg_out_t o;

    // 1) Vertical nose-up (b = 0,0,-1): everything neutral, pyros OFF
    std::memset(&n, 0, sizeof(n));
    n.bz = -1.0f; n.gx = 1.0f; n.valid = true;
    dg_step_polarity(&n, &o);
    CHECK(!o.ignite && !o.thrust_on && o.phase == DG_PHASE_IDLE, "polarity: must stay IDLE/no ignite");
    CHECK(near(o.clamp, DG_CLAMP_MAX, 1e-4), "polarity: clamp open");
    CHECK(near(o.servo1_deg, 0.0, 1e-4) && near(o.servo2_deg, 0.0, 1e-4), "polarity: vertical TVC zero");
    for (int i = 0; i < 4; i++)
        CHECK(near(o.fin_deg[i], 0.0, 1e-4), "polarity: vertical fin %d zero", i);

    // 2) Vertical nose-down (b = 0,0,+1): everything neutral
    n.bz = 1.0f;
    dg_step_polarity(&n, &o);
    CHECK(near(o.servo1_deg, 0.0, 1e-4) && near(o.servo2_deg, 0.0, 1e-4), "polarity: nose-down TVC zero");
    for (int i = 0; i < 4; i++)
        CHECK(near(o.fin_deg[i], 0.0, 1e-4), "polarity: nose-down fin %d zero", i);

    // 3) Landing orientation (nose-down, bz > 0), tilted +10 deg toward +X:
    //    b = (+sin 10, 0, +cos 10), u1 = (+cos 10, 0, -sin 10), u2 = (0, +1, 0)
    //    e = b x (0,0,+1) = (0, -sin 10, 0) -> tr along -Y -> sq = b x tr along +u1 -> servo1 > 0
    const float th = 10.0f * (float)M_PI / 180.0f;
    std::memset(&n, 0, sizeof(n));
    n.bx = std::sin(th); n.bz = std::cos(th);
    n.gx = std::cos(th); n.gz = -std::sin(th);
    n.valid = true;
    dg_step_polarity(&n, &o);
    CHECK(o.servo1_deg > 4.0f && o.servo1_deg <= DG_SERVO_MAX_DEG + 1e-3f,
          "polarity landing +X tilt: servo1=%g", o.servo1_deg);
    CHECK(near(o.servo2_deg, 0.0, 0.2), "polarity landing +X tilt: servo2=%g", o.servo2_deg);
    // Opposite tilt must invert TVC sign symmetrically
    dg_out_t o_opp;
    n.bx = -std::sin(th); n.gz = std::sin(th);
    dg_step_polarity(&n, &o_opp);
    CHECK(near(o_opp.servo1_deg, -o.servo1_deg, 0.2),
          "polarity landing opposite tilt: %g vs %g", o_opp.servo1_deg, -o.servo1_deg);

    // 4) Ascent orientation (nose-up, bz < 0), tilted 10 deg:
    //    b = (+sin 10, 0, -cos 10), u1 = (+cos 10, 0, +sin 10), u2 = b x u1 = (0, -1, 0)
    //    e = b x (0,0,-1) = (0, +sin 10, 0) -> tr along +Y -> dot(tr, u2) < 0 -> b_cmd > 0 -> fin[1] > 0, fin[3] < 0
    std::memset(&n, 0, sizeof(n));
    n.bx = std::sin(th); n.bz = -std::cos(th);
    n.gx = std::cos(th); n.gz = std::sin(th);
    n.valid = true;
    dg_step_polarity(&n, &o);
    CHECK(o.fins_live, "polarity ascent: fins must be live");
    CHECK(o.fin_deg[1] > 0.3f && near(o.fin_deg[3], -o.fin_deg[1], 1e-3),
          "polarity ascent fin pair 1/3: %g, %g", o.fin_deg[1], o.fin_deg[3]);
    CHECK(near(o.fin_deg[0], 0.0, 1e-3) && near(o.fin_deg[2], 0.0, 1e-3),
          "polarity ascent fin pair 0/2: %g, %g", o.fin_deg[0], o.fin_deg[2]);
    // Opposite tilt must invert fin sign symmetrically
    n.bx = -std::sin(th); n.gz = -std::sin(th);
    dg_step_polarity(&n, &o_opp);
    CHECK(near(o_opp.fin_deg[1], -o.fin_deg[1], 1e-3),
          "polarity ascent opposite tilt: %g vs %g", o_opp.fin_deg[1], -o.fin_deg[1]);

    // 5) Pure roll rate damping (w_roll = dot(w, b) = +1 rad/s -> all 4 fins negative equally)
    std::memset(&n, 0, sizeof(n));
    n.bz = -1.0f; n.gx = 1.0f; n.wz = -1.0f; n.valid = true;
    dg_step_polarity(&n, &o);
    for (int i = 0; i < 4; i++)
        CHECK(o.fin_deg[i] < -0.01f && near(o.fin_deg[i], o.fin_deg[0], 1e-4),
              "polarity roll damper fin %d: %g", i, o.fin_deg[i]);

    // 6) Custom reference thrust override (10..250 N): lower thrust -> larger TVC deflection
    std::memset(&n, 0, sizeof(n));
    n.by = -std::sin(0.08f); n.bz = -std::cos(0.08f); n.gx = 1.0f; n.valid = true;
    dg_out_t o_def, o_lo, o_hi;
    dg_step_polarity(&n, &o_def, 0.0f);
    dg_step_polarity(&n, &o_lo,  50.0f);
    dg_step_polarity(&n, &o_hi,  200.0f);
    CHECK(o_def.thrust_est > 85.0f && o_def.thrust_est < 100.0f,
          "polarity default thrust_est=%g", o_def.thrust_est);
    CHECK(near(o_lo.thrust_est, 50.0, 1e-3) && near(o_hi.thrust_est, 200.0, 1e-3),
          "polarity custom thrust_est lo=%g hi=%g", o_lo.thrust_est, o_hi.thrust_est);
    CHECK(o_lo.servo2_deg > 0.5f && o_hi.servo2_deg > 0.5f
          && !near(o_lo.servo2_deg, o_hi.servo2_deg, 0.2),
          "polarity custom thrust changes TVC gain: lo=%g hi=%g", o_lo.servo2_deg, o_hi.servo2_deg);

    // 7) Custom airspeed override (5..70 m/s, default 40 m/s): lower speed -> larger fin deflection
    dg_out_t o_vdef, o_v40, o_vlo, o_vhi;
    dg_step_polarity(&n, &o_vdef, 0.0f, 0.0f);
    dg_step_polarity(&n, &o_v40,  0.0f, 40.0f);
    dg_step_polarity(&n, &o_vlo,  0.0f, 15.0f);
    dg_step_polarity(&n, &o_vhi,  0.0f, 60.0f);
    CHECK(near(o_vdef.fin_deg[0], o_v40.fin_deg[0], 1e-4),
          "polarity default speed is 40 m/s: def=%g v40=%g", o_vdef.fin_deg[0], o_v40.fin_deg[0]);
    CHECK(std::fabs(o_vlo.fin_deg[0]) > std::fabs(o_vhi.fin_deg[0]) && std::fabs(o_vhi.fin_deg[0]) > 0.01f,
          "polarity lower airspeed demands larger fin deflection: v15=%g v60=%g", o_vlo.fin_deg[0], o_vhi.fin_deg[0]);
}

int main()
{
    std::printf("guidance v2 (LandSim_Light port)\n");
    test_golden();
    test_properties();
    test_state_machine();
    test_polarity();
    test_closed_loop();
    std::printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
