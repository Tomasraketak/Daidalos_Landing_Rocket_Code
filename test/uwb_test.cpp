// =====================================================================
//  Ariadna V2 — UWB multilateration checks
//
//  ../Ariadna_V2/common/ariadna_uwb.h compiles UNCHANGED into this test
//  and unchanged into Position_MCU.ino, so what is checked here is what
//  flies. Same discipline as est_test.cpp and host_test.cpp.
//
//  What this establishes:
//
//    CAN   the solver recovers a known position from clean ranges, keeps
//          working as anchors are removed one at a time, reports honest
//          sigmas, refuses geometry it cannot solve, survives NaNs,
//          negative ranges, duplicated anchors and a fully dead
//          constellation, and never returns a non-finite number.
//    CANNOT  that the anchors are surveyed where the table says they
//          are. Nothing in software can establish that; measure the
//          square with a tape before you fly it.
//
//  Build:  make uwb
// =====================================================================
#include "../Ariadna_V2/common/ariadna_uwb.h"

#include <cstdio>
#include <cstring>
#include <cmath>

static int g_checks = 0;
static int g_fails  = 0;

static void check(bool cond, const char *what)
{
    g_checks++;
    if (!cond) {
        g_fails++;
        printf("  FAIL  %s\n", what);
    }
}

// ---------------------------------------------------------------------
//  The flight layout: a 40 m square of ground anchors centred on the pad.
// ---------------------------------------------------------------------
static const float SQ = 20.0f;

static void layout(ariadna_uwb_anchor_t *a)
{
    memset(a, 0, sizeof(ariadna_uwb_anchor_t) * ARIADNA_UWB_MAX_ANCHORS);
    const float xs[4] = { -SQ,  SQ,  SQ, -SQ };
    const float ys[4] = { -SQ, -SQ,  SQ,  SQ };
    for (int i = 0; i < 4; i++) {
        a[i].addr    = (uint16_t)(0x1001 + i);
        a[i].enabled = 1;
        a[i].x = xs[i]; a[i].y = ys[i]; a[i].z = 0.0f;
    }
}

/** Synthesise ranges from a true position, with an optional per-anchor bias. */
static void ranges(const ariadna_uwb_anchor_t *a, ariadna_uwb_obs_t *o,
                   float tx, float ty, float tz, float sigma,
                   const float *bias = nullptr)
{
    memset(o, 0, sizeof(ariadna_uwb_obs_t) * ARIADNA_UWB_MAX_ANCHORS);
    for (int i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) {
        if (!a[i].enabled) continue;
        const float dx = tx - a[i].x, dy = ty - a[i].y, dz = tz - a[i].z;
        o[i].range_m = sqrtf(dx * dx + dy * dy + dz * dz) + (bias ? bias[i] : 0.0f);
        o[i].sigma_m = sigma;
        o[i].age_s   = 0.0f;
        o[i].valid   = 1;
    }
}

/**
 * Synthesise a REALISTIC set: the anchors answer round-robin, so each
 * range was taken from where the vehicle was `age` seconds ago, on a
 * straight-line descent at `vz` (negative = falling).
 *
 * This is what a moving vehicle actually produces, and it is the thing
 * the naive version of this solver got wrong.
 */
static void ranges_moving(const ariadna_uwb_anchor_t *a, ariadna_uwb_obs_t *o,
                          float tx, float ty, float tz,
                          float vx, float vy, float vz,
                          const float *age, float sigma)
{
    memset(o, 0, sizeof(ariadna_uwb_obs_t) * ARIADNA_UWB_MAX_ANCHORS);
    for (int i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) {
        if (!a[i].enabled) continue;
        // Where the vehicle was when THIS anchor answered.
        const float px = tx - vx * age[i];
        const float py = ty - vy * age[i];
        const float pz = tz - vz * age[i];
        const float dx = px - a[i].x, dy = py - a[i].y, dz = pz - a[i].z;
        o[i].range_m = sqrtf(dx * dx + dy * dy + dz * dz);
        o[i].sigma_m = sigma;
        o[i].age_s   = age[i];
        o[i].valid   = 1;
    }
}

static bool finite_fix(const ariadna_uwb_fix_t &f)
{
    return std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.z) &&
           std::isfinite(f.sigma_x) && std::isfinite(f.sigma_y) &&
           std::isfinite(f.sigma_z) && std::isfinite(f.residual_rms);
}

// ---------------------------------------------------------------------
//  1. Four clean anchors reproduce the truth
// ---------------------------------------------------------------------
static void test_four_anchors(void)
{
    printf("four anchors\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    const float tx = 3.0f, ty = -4.0f, tz = 30.0f;
    ranges(a, o, tx, ty, tz, 0.1f);

    // Deliberately started 15 m away: a solver that only works from a
    // good prior is a solver that fails exactly when it is needed.
    const float prior[3] = { 12.0f, 8.0f, 20.0f };
    const int ok = ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);

    check(ok == 1, "clean four-anchor solve succeeds");
    check(finite_fix(f), "fix is finite");
    check(f.mode == ARIADNA_UWB_SOL_3D, "mode is 3D at 30 m altitude");
    check(f.n_used == 4, "all four used");
    check(f.mask == 0x0F, "mask names all four slots");
    check(fabsf(f.x - tx) < 0.05f, "x recovered");
    check(fabsf(f.y - ty) < 0.05f, "y recovered");
    check(fabsf(f.z - tz) < 0.10f, "z recovered");
    check(f.residual_rms < 1e-3f, "post-fit residual is ~zero on clean data");
    check(f.use_x && f.use_y, "horizontal axes published");

    // Horizontal sigma with four anchors at 0.1 m ranging noise should be
    // a small fraction of a metre — an order better than the GPS this is
    // there to beat.
    check(f.sigma_x < 0.2f && f.sigma_y < 0.2f, "horizontal sigma is sub-decimetre-ish");
    check(f.sigma_x > 0.0f && f.sigma_y > 0.0f, "sigmas are positive");
}

// ---------------------------------------------------------------------
//  2. The vertical is only claimed when the geometry supports it
//
//  This is the check that matters most for a ground-anchor layout. Near
//  the anchor plane the vertical direction cosine collapses and height
//  stops being observable. The solver must NOTICE, not merely be wrong.
// ---------------------------------------------------------------------
static void test_vertical_observability(void)
{
    printf("vertical observability\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    hi, lo;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    const float prior_hi[3] = { 0.0f, 0.0f, 25.0f };
    ranges(a, o, 1.0f, 1.0f, 25.0f, 0.1f);
    ariadna_uwb_solve(a, o, 4, prior_hi, &cfg, &hi);

    const float prior_lo[3] = { 0.0f, 0.0f, 0.5f };
    ranges(a, o, 1.0f, 1.0f, 0.5f, 0.1f);
    const int ok_lo = ariadna_uwb_solve(a, o, 4, prior_lo, &cfg, &lo);

    check(hi.use_z == 1, "height is published from 25 m up");
    check(lo.use_z == 0, "height is NOT published half a metre off the plane");
    check(lo.mode == ARIADNA_UWB_SOL_2D, "the low solve degrades to 2D");
    check(ok_lo == 1, "and still returns a usable horizontal fix");
    check(fabsf(lo.x - 1.0f) < 0.1f && fabsf(lo.y - 1.0f) < 0.1f,
          "the degraded solve is still horizontally right");
    check(lo.sigma_x < 0.5f, "horizontal accuracy is BETTER near the ground");
}

// ---------------------------------------------------------------------
//  3. Losing anchors, one at a time
//
//  The whole point of the exercise: an anchor dying must cost accuracy,
//  never correctness and never a crash.
// ---------------------------------------------------------------------
static void test_anchor_dropout(void)
{
    printf("anchor dropout\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    const float tx = 2.0f, ty = 5.0f, tz = 12.0f;
    const float prior[3] = { 0.0f, 0.0f, 12.0f };

    // --- three anchors --------------------------------------------------
    ranges(a, o, tx, ty, tz, 0.1f);
    o[3].valid = 0;
    check(ariadna_uwb_solve(a, o, 4, prior, &cfg, &f) == 1, "three anchors solve");
    check(finite_fix(f), "three-anchor fix is finite");
    check(f.n_used == 3, "three used");
    check(fabsf(f.x - tx) < 0.2f && fabsf(f.y - ty) < 0.2f,
          "three anchors still place the vehicle");

    // --- two anchors ----------------------------------------------------
    // Ambiguous by construction. Started from a prior on the true side,
    // it must converge to the true point and say honestly that one
    // direction is poorly known.
    ranges(a, o, tx, ty, tz, 0.1f);
    o[2].valid = 0; o[3].valid = 0;
    const int ok2 = ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(finite_fix(f), "two-anchor fix is finite");
    check(f.mode == ARIADNA_UWB_SOL_2D, "two anchors give a 2D mode");
    check(f.use_z == 0, "two anchors never publish height");
    if (ok2) {
        check(fabsf(f.x - tx) < 1.5f && fabsf(f.y - ty) < 1.5f,
              "two anchors converge to the nearer of the two solutions");
    } else {
        check(f.use_x == 0 && f.use_y == 0, "or refuse cleanly");
    }

    // --- one anchor -----------------------------------------------------
    // A single range is a line-of-sight constraint. It must publish at
    // most the axes it actually observes, with a sigma inflated by the
    // direction cosine.
    ranges(a, o, tx, ty, tz, 0.1f);
    o[1].valid = 0; o[2].valid = 0; o[3].valid = 0;
    ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(finite_fix(f), "one-anchor fix is finite");
    check(f.mode == ARIADNA_UWB_SOL_LOS, "one anchor gives the LOS mode");
    check(f.use_z == 0, "one anchor never publishes height");
    check(f.sigma_x >= 0.1f, "LOS sigma is never smaller than the ranging sigma");

    // Started from the truth, a clean single range must not move it.
    const float exact[3] = { tx, ty, tz };
    ranges(a, o, tx, ty, tz, 0.1f);
    o[1].valid = 0; o[2].valid = 0; o[3].valid = 0;
    ariadna_uwb_solve(a, o, 4, exact, &cfg, &f);
    check(fabsf(f.x - tx) < 1e-3f && fabsf(f.y - ty) < 1e-3f,
          "a consistent single range leaves a correct position alone");

    // --- no anchors -----------------------------------------------------
    memset(o, 0, sizeof(o));
    check(ariadna_uwb_solve(a, o, 4, prior, &cfg, &f) == 0, "no anchors: no fix");
    check(f.mode == ARIADNA_UWB_SOL_NONE, "and the mode says so");
    check(f.use_x == 0 && f.use_y == 0 && f.use_z == 0, "and nothing is publishable");
    check(finite_fix(f), "and the fix is still fully initialised");
}

// ---------------------------------------------------------------------
//  4. Garbage in, refusal out
// ---------------------------------------------------------------------
static void test_garbage(void)
{
    printf("hostile input\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);
    const float prior[3] = { 0.0f, 0.0f, 10.0f };

    // A NaN range from one anchor must not poison the other three.
    ranges(a, o, 1.0f, 2.0f, 15.0f, 0.1f);
    o[2].range_m = NAN;
    check(ariadna_uwb_solve(a, o, 4, prior, &cfg, &f) == 1, "a NaN range is dropped");
    check(finite_fix(f) && f.n_used == 3, "and the rest still solve");

    // Negative and absurd ranges: screened out by range_min/range_max.
    ranges(a, o, 1.0f, 2.0f, 15.0f, 0.1f);
    o[0].range_m = -5.0f;
    o[1].range_m = 100000.0f;
    ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(f.n_used == 2, "a negative and an absurd range are both screened");

    // A zero or negative sigma is a driver bug; it must not become an
    // infinite weight.
    ranges(a, o, 1.0f, 2.0f, 15.0f, 0.1f);
    o[0].sigma_m = 0.0f;
    o[1].sigma_m = -1.0f;
    ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(f.n_used == 2, "non-positive sigmas are screened");

    // A NaN prior must not propagate.
    ranges(a, o, 1.0f, 2.0f, 15.0f, 0.1f);
    const float bad_prior[3] = { NAN, NAN, NAN };
    ariadna_uwb_solve(a, o, 4, bad_prior, &cfg, &f);
    check(finite_fix(f), "a NaN prior does not produce a NaN fix");

    // All anchors at the same point: the normal equations are singular in
    // every direction but one.
    ariadna_uwb_anchor_t deg[ARIADNA_UWB_MAX_ANCHORS];
    memset(deg, 0, sizeof(deg));
    for (int i = 0; i < 4; i++) {
        deg[i].addr = (uint16_t)(0x2001 + i);
        deg[i].enabled = 1;
        deg[i].x = deg[i].y = deg[i].z = 0.0f;
    }
    ranges(deg, o, 1.0f, 2.0f, 15.0f, 0.1f);
    ariadna_uwb_solve(deg, o, 4, prior, &cfg, &f);
    check(finite_fix(f), "coincident anchors do not produce a NaN");
    check(!(f.use_x && f.use_y && f.use_z), "and are not reported as a full fix");

    // Collinear anchors: solvable along the line, not across it.
    ariadna_uwb_anchor_t lin[ARIADNA_UWB_MAX_ANCHORS];
    memset(lin, 0, sizeof(lin));
    for (int i = 0; i < 4; i++) {
        lin[i].addr = (uint16_t)(0x3001 + i);
        lin[i].enabled = 1;
        lin[i].x = -30.0f + 20.0f * (float)i;
        lin[i].y = 0.0f;
        lin[i].z = 0.0f;
    }
    ranges(lin, o, 1.0f, 2.0f, 15.0f, 0.1f);
    ariadna_uwb_solve(lin, o, 4, prior, &cfg, &f);
    check(finite_fix(f), "collinear anchors do not produce a NaN");
}

// ---------------------------------------------------------------------
//  5. A reflected range is caught by the post-fit residual
// ---------------------------------------------------------------------
static void test_nlos(void)
{
    printf("reflection rejection\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);
    const float prior[3] = { 0.0f, 0.0f, 20.0f };

    // A non-line-of-sight path is always LONGER. Ten metres of extra path
    // on one anchor cannot be absorbed by a three-unknown fit of four
    // equations, so the residual gives it away.
    float bias[ARIADNA_UWB_MAX_ANCHORS];
    memset(bias, 0, sizeof(bias));
    bias[1] = 10.0f;
    ranges(a, o, 1.0f, 2.0f, 20.0f, 0.1f, bias);

    const int ok = ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(ok == 0, "a 10 m reflection is rejected");
    check(f.residual_rms > cfg.resid_max_m, "because the residual says so");
    check(f.use_x == 0 && f.use_y == 0, "and no axis is offered to the filter");

    // A modest one — 30 cm, the sort of thing an antenna delay produces —
    // must NOT be rejected. A gate that fires on real data is worse than
    // no gate, because it fires when the vehicle needs the fix most.
    memset(bias, 0, sizeof(bias));
    bias[1] = 0.3f;
    ranges(a, o, 1.0f, 2.0f, 20.0f, 0.1f, bias);
    check(ariadna_uwb_solve(a, o, 4, prior, &cfg, &f) == 1,
          "a 30 cm antenna-delay error is accepted");
}

// ---------------------------------------------------------------------
//  6. Sigmas mean what they say
//
//  Monte Carlo: perturb the ranges by a known noise and check that the
//  spread of the answers matches the sigma the solver quoted. A filter
//  weights by these numbers, so a sigma that is optimistic by 3x is a
//  filter that trusts UWB nine times more than it should.
// ---------------------------------------------------------------------
static uint32_t rnd_state = 12345u;
static float randn(void)
{
    // Box-Muller on a plain LCG. Reproducible, which matters more here
    // than statistical purity.
    rnd_state = rnd_state * 1664525u + 1013904223u;
    const float u1 = ((float)((rnd_state >> 8) & 0xFFFFFFu) + 1.0f) / 16777217.0f;
    rnd_state = rnd_state * 1664525u + 1013904223u;
    const float u2 = ((float)((rnd_state >> 8) & 0xFFFFFFu)) / 16777216.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static void test_sigma_calibration(void)
{
    printf("sigma calibration\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);
    cfg.resid_max_m = 100.0f;      // not what is under test here

    const float tx = 4.0f, ty = -2.0f, tz = 25.0f;
    const float sig = 0.15f;
    const float prior[3] = { 0.0f, 0.0f, 25.0f };

    double sx = 0.0, sy = 0.0, quoted_x = 0.0;
    int n = 0;
    for (int trial = 0; trial < 600; trial++) {
        ranges(a, o, tx, ty, tz, sig);
        for (int i = 0; i < 4; i++) o[i].range_m += sig * randn();
        if (!ariadna_uwb_solve(a, o, 4, prior, &cfg, &f)) continue;
        sx += (double)((f.x - tx) * (f.x - tx));
        sy += (double)((f.y - ty) * (f.y - ty));
        quoted_x += (double)f.sigma_x;
        n++;
    }
    check(n > 550, "almost every noisy trial produces a fix");
    if (n > 0) {
        const double rms_x   = sqrt(sx / n);
        const double rms_y   = sqrt(sy / n);
        const double quoted  = quoted_x / n;
        printf("       measured rms x %.3f m, y %.3f m, quoted sigma %.3f m\n",
               rms_x, rms_y, quoted);
        check(rms_x < quoted * 1.6 && rms_x > quoted * 0.6,
              "the quoted x sigma matches the observed spread");
        check(rms_y < quoted * 2.0, "and y is the same order");
    }
}

// ---------------------------------------------------------------------
//  7. Endurance: ten thousand solves, nothing non-finite, no drift
// ---------------------------------------------------------------------
static void test_endurance(void)
{
    printf("endurance\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    int bad = 0, solved = 0;
    float prior[3] = { 0.0f, 0.0f, 60.0f };

    // A descent from 60 m to the pad with anchors dropping in and out at
    // a rate no real constellation would manage.
    for (int k = 0; k < 10000; k++) {
        const float h  = 60.0f - 0.006f * (float)k;
        const float tx = 6.0f * sinf(0.001f * (float)k);
        const float ty = 6.0f * cosf(0.001f * (float)k);
        ranges(a, o, tx, ty, h > 0.2f ? h : 0.2f, 0.12f);
        for (int i = 0; i < 4; i++) {
            o[i].range_m += 0.12f * randn();
            if (((k / 37) + i) % 5 == 0) o[i].valid = 0;   // an anchor drops out
        }
        if (ariadna_uwb_solve(a, o, 4, prior, &cfg, &f)) {
            solved++;
            if (f.use_x) prior[0] = f.x;
            if (f.use_y) prior[1] = f.y;
            if (f.use_z) prior[2] = f.z;
        }
        prior[2] = h > 0.2f ? h : 0.2f;      // the barometer keeps height honest
        if (!finite_fix(f)) bad++;
    }
    check(bad == 0, "no non-finite fix in ten thousand solves");
    check(solved > 6000, "most steps still produce a usable fix while anchors flap");
    printf("       %d/10000 solved\n", solved);
}

// ---------------------------------------------------------------------
//  8. Address lookup
// ---------------------------------------------------------------------
static void test_lookup(void)
{
    printf("anchor lookup\n");
    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    layout(a);
    check(ariadna_uwb_find(a, 4, 0x1002) == 1, "known address found");
    check(ariadna_uwb_find(a, 4, 0xBEEF) == -1, "unknown address refused");
    a[1].enabled = 0;
    check(ariadna_uwb_find(a, 4, 0x1002) == -1, "a disabled slot is not found");
}

// ---------------------------------------------------------------------
//  9. A MOVING VEHICLE
//
//  The anchors answer one at a time, so four ranges are smeared over a
//  round-robin period and the solver fits one position to four
//  measurements taken from four different places.
//
//  This is the test that matters most for this vehicle, and the reason
//  is not the top speed. At 50 m/s the smear is large enough that the
//  post-fit residual catches it and the fix is thrown away — a safe
//  failure. The dangerous band is 10-20 m/s, the speed in the last ten
//  and five metres of the descent, where the error is metres, biased
//  along the direction of travel, and small enough in residual to pass
//  every gate. Uncorrected, that is a confident wrong altitude in the
//  final seconds.
// ---------------------------------------------------------------------
static void test_motion_compensation(void)
{
    printf("motion compensation\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    // Round-robin at 10 Hz per anchor: the newest range is fresh, the
    // oldest is 200 ms old — one whole cycle, and one missed exchange.
    const float age[4] = { 0.00f, 0.07f, 0.14f, 0.20f };

    // The descent profile the vehicle actually flies: fast and high,
    // slow and low. Accuracy matters most in the last rows.
    struct { float h, vz, tol; } leg[] = {
        {  60.0f, -50.0f, 0.35f },
        {  30.0f, -35.0f, 0.30f },
        {  10.0f, -20.0f, 0.25f },
        {   5.0f, -10.0f, 0.25f },
        {   2.0f,  -5.0f, 0.25f },
    };

    for (unsigned k = 0; k < sizeof(leg) / sizeof(leg[0]); k++) {
        const float tx = 1.5f, ty = -2.0f;
        const float vel[3] = { 1.0f, -0.5f, leg[k].vz };

        ranges_moving(a, o, tx, ty, leg[k].h, vel[0], vel[1], vel[2], age, 0.20f);
        const float prior[3] = { tx, ty, leg[k].h };

        // --- without compensation, for the record ------------------------
        ariadna_uwb_obs_t raw[ARIADNA_UWB_MAX_ANCHORS];
        memcpy(raw, o, sizeof(raw));
        for (int i = 0; i < ARIADNA_UWB_MAX_ANCHORS; i++) raw[i].age_s = 0.0f;
        ariadna_uwb_fix_t nf;
        ariadna_uwb_solve(a, raw, 4, prior, &cfg, &nf);
        const float err_raw = sqrtf((nf.x - tx) * (nf.x - tx) +
                                    (nf.y - ty) * (nf.y - ty) +
                                    (nf.z - leg[k].h) * (nf.z - leg[k].h));

        // --- with it ------------------------------------------------------
        ariadna_uwb_derotate(a, o, 4, prior, vel, &cfg);
        const int ok = ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
        const float err = sqrtf((f.x - tx) * (f.x - tx) +
                                (f.y - ty) * (f.y - ty) +
                                (f.z - leg[k].h) * (f.z - leg[k].h));

        printf("       h %5.1f m  vz %6.1f m/s : uncorrected %.2f m -> corrected %.2f m"
               "  (resid %.2f m)\n",
               (double)leg[k].h, (double)leg[k].vz,
               (double)err_raw, (double)err, (double)f.residual_rms);

        char msg[96];
        snprintf(msg, sizeof msg, "de-rotated fix at %.0f m / %.0f m/s is accepted",
                 (double)leg[k].h, (double)leg[k].vz);
        check(ok == 1, msg);
        snprintf(msg, sizeof msg, "de-rotated fix at %.0f m / %.0f m/s is within %.2f m",
                 (double)leg[k].h, (double)leg[k].vz, (double)leg[k].tol);
        check(err < leg[k].tol, msg);
        snprintf(msg, sizeof msg, "compensation actually helped at %.0f m/s",
                 (double)leg[k].vz);
        check(err < err_raw, msg);
        check(finite_fix(f), "moving fix is finite");
    }

    // An old range must be trusted LESS, not merely shifted. Two
    // identical sets differing only in age must produce different sigmas.
    ranges_moving(a, o, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, -20.0f, age, 0.20f);
    const float prior[3] = { 0.0f, 0.0f, 20.0f };
    const float vel[3]   = { 0.0f, 0.0f, -20.0f };
    ariadna_uwb_derotate(a, o, 4, prior, vel, &cfg);
    check(o[3].sigma_m > o[0].sigma_m * 1.1f,
          "the oldest range comes out with a larger sigma than the freshest");
    check(o[0].sigma_m > 0.0f && o[3].sigma_m < 5.0f, "and both stay sensible");

    // A range older than the limit is dropped outright rather than
    // extrapolated across half a second of a rocket's descent.
    ranges(a, o, 0.0f, 0.0f, 20.0f, 0.2f);
    o[2].age_s = 1.0f;
    ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(f.n_used == 3, "an over-age range is dropped, the others still solve");

    // A negative age is a clock that ran backwards. Not extrapolated.
    ranges(a, o, 0.0f, 0.0f, 20.0f, 0.2f);
    o[1].age_s = -0.5f;
    ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
    check(f.n_used == 3, "a negative age is dropped");

    // No velocity available yet: ages must still inflate the sigmas, and
    // nothing may break.
    ranges_moving(a, o, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f, -20.0f, age, 0.20f);
    ariadna_uwb_derotate(a, o, 4, prior, nullptr, &cfg);
    check(o[3].sigma_m > 0.20f, "with no velocity, an old range is still de-weighted");
    check(ariadna_uwb_solve(a, o, 4, prior, &cfg, &f) == 1 || f.use_x == 0,
          "and the solve either works or refuses cleanly");
    check(finite_fix(f), "and stays finite");
}

// ---------------------------------------------------------------------
//  10. The vertical, all the way down
//
//  Four anchors on flat ground see height through the direction cosine
//  h/r, which collapses as the vehicle lands. The solver must publish a
//  useful height at 10 m, and must stop publishing one before the
//  geometry stops supporting it — silently, without a step.
// ---------------------------------------------------------------------
static void test_vertical_envelope(void)
{
    printf("vertical envelope\n");

    ariadna_uwb_anchor_t a[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_obs_t    o[ARIADNA_UWB_MAX_ANCHORS];
    ariadna_uwb_cfg_t    cfg;
    ariadna_uwb_fix_t    f;
    layout(a);
    ariadna_uwb_cfg_default(&cfg);

    printf("       height   sigma_z   z published   horizontal sigma\n");
    const float hs[] = { 30.0f, 20.0f, 10.0f, 5.0f, 3.0f, 2.0f, 1.0f, 0.3f };
    int published_at_10 = 0, published_at_0_3 = 0;
    for (unsigned k = 0; k < sizeof(hs) / sizeof(hs[0]); k++) {
        ranges(a, o, 0.5f, 0.5f, hs[k], 0.25f);
        const float prior[3] = { 0.0f, 0.0f, hs[k] };
        ariadna_uwb_solve(a, o, 4, prior, &cfg, &f);
        printf("       %5.1f m   %6.2f m      %s          %.3f m\n",
               (double)hs[k], (double)(f.use_z ? f.sigma_z : 999.0f),
               f.use_z ? "yes" : " no", (double)f.sigma_x);
        if (hs[k] == 10.0f) published_at_10 = f.use_z;
        if (hs[k] == 0.3f)  published_at_0_3 = f.use_z;
        check(finite_fix(f), "fix finite through the whole descent");
        check(f.use_x && f.use_y, "the horizontal never drops out on the way down");
        check(fabsf(f.x - 0.5f) < 0.5f, "and stays accurate");
    }
    check(published_at_10 == 1, "height IS published at 10 m — the light-anchor band");
    check(published_at_0_3 == 0, "height is NOT published at 0.3 m — geometry is gone");
}

int main(void)
{
    printf("\nAriadna V2 UWB multilateration checks\n");
    printf("-------------------------------------\n");
    test_four_anchors();
    test_vertical_observability();
    test_anchor_dropout();
    test_garbage();
    test_nlos();
    test_sigma_calibration();
    test_endurance();
    test_lookup();
    test_motion_compensation();
    test_vertical_envelope();
    printf("-------------------------------------\n");
    printf("%d checks, %d failures\n\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
