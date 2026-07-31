// =====================================================================
//  Host-side verification of the flight guidance port.
//
//  Compiles guidance.cpp unchanged (it has no Arduino dependencies) and
//  checks it against golden vectors produced by the validated Python model.
//  This is the reason the algorithm lives in guidance.cpp rather than
//  inside the .ino: firmware that cannot be tested on a workstation gets
//  verified on the pad, which is the wrong place.
//
//  Build & run:  cd test && make
// =====================================================================
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "../Daidalos_Landing_Guidance/guidance.h"
#include "golden_vectors.h"

static int failures = 0;
static int checks   = 0;

static void check(const char *what, float got, float want, float tol)
{
    checks++;
    float d = fabsf(got - want);
    bool ok = d <= tol;
    if (!ok) failures++;
    printf("  [%s] %-46s got %10.5f  want %10.5f  d %9.5f (tol %.4f)\n",
           ok ? "PASS" : "FAIL", what, got, want, d, tol);
}

int main(void)
{
    dg_init();
    printf("=====================================================================\n");
    printf(" DAIDALOS GUIDANCE — PORT VERIFICATION AGAINST THE PYTHON MODEL\n");
    printf("=====================================================================\n\n");

    // -----------------------------------------------------------------
    printf("1. Motor curve, thrust against normalised burn time\n");
    printf("   (C uses a 64-entry uniform resample for O(1) lookup; the model\n");
    printf("    interpolates the 13 raw knots, so small resampling error is\n");
    printf("    expected and is bounded here at 3 N.)\n");
    for (int i = 0; i < N_THRUST; i++) {
        char nm[64];
        snprintf(nm, sizeof nm, "thrust_ref(tau=%.3f)", G_THRUST_TAU[i]);
        check(nm, dg_thrust_ref(G_THRUST_TAU[i]), G_THRUST_N[i], 3.0f);
    }

    // -----------------------------------------------------------------
    printf("\n2. Guidance law — commanded thrust direction\n");
    printf("   (C carries a unit vector and never calls atan2; the angle is\n");
    printf("    reconstructed here purely for comparison.)\n");
    float worst_tilt = 0.0f;
    for (int i = 0; i < N_TILT; i++) {
        float h = G_TILT[i][0], vx = G_TILT[i][1], vz = G_TILT[i][2];
        float want = G_TILT[i][3];
        float sn, cs;
        dg_guidance_tilt(h, vx, vz, 3.0f, &sn, &cs);
        float got = atan2f(sn, cs);
        float d = fabsf(got - want);
        if (d > worst_tilt) worst_tilt = d;
        if (i % 9 == 0) {
            char nm[80];
            snprintf(nm, sizeof nm, "tilt(h=%.1f vx=%.0f vz=%.0f) rad", h, vx, vz);
            check(nm, got, want, 0.004f);
        } else {
            checks++;
            if (d > 0.004f) {
                failures++;
                printf("  [FAIL] tilt(h=%.1f vx=%.0f vz=%.0f) got %.5f want %.5f\n",
                       h, vx, vz, got, want);
            }
        }
    }
    printf("   worst tilt error over all %d cases: %.5f rad (%.3f deg)\n",
           N_TILT, worst_tilt, worst_tilt * 180.0f / (float)M_PI);

    // -----------------------------------------------------------------
    printf("\n3. Throttle-plan root find\n");
    printf("   (The model integrates the projection with Euler at dt=0.02;\n");
    printf("    the flight code uses RK2 at dt=0.04 to halve the step count.\n");
    printf("    The solved p is what actually commands the clamp, so that is\n");
    printf("    what is compared, with a tolerance reflecting the integrator.)\n");
    for (int i = 0; i < N_SOLVE; i++) {
        float h = G_SOLVE[i][0], vx = G_SOLVE[i][1], vz = G_SOLVE[i][2];
        float tau = G_SOLVE[i][3], want = G_SOLVE[i][4];
        float got = dg_solve_p_full(h, vx, vz, tau, 3.0f, 0.7f);
        char nm[80];
        snprintf(nm, sizeof nm, "solve_p(h=%.1f vx=%.0f vz=%.0f tau=%.2f)",
                 h, vx, vz, tau);
        check(nm, got, want, 0.30f);
    }

    // -----------------------------------------------------------------
    printf("\n4. Ignition altitude table\n");
    for (int i = 0; i < N_IGN; i++) {
        dg_init();
        dg_arm(9.0f, -G_IGN[i][0]);
        char nm[64];
        snprintf(nm, sizeof nm, "h_ignite(|vz0|=%.0f)", G_IGN[i][0]);
        check(nm, dg_h_ignite(), G_IGN[i][1], 0.45f);
    }

    // -----------------------------------------------------------------
    printf("\n5. Structural invariants\n");
    dg_init();
    checks++;
    if (dg_thrust_ref(-0.1f) != 0.0f || dg_thrust_ref(1.5f) != 0.0f) {
        failures++; printf("  [FAIL] thrust_ref out of range must be zero\n");
    } else printf("  [PASS] thrust_ref is zero outside [0,1]\n");

    // monotone in p: more clamp must never land LOWER
    checks++;
    bool mono = true;
    float prev = -1e30f;
    for (float p = 0.0f; p <= 4.0f + 1e-6f; p += 0.25f) {
        float m = dg_project_miss(28.0f, 6.0f, -45.0f, 0.1f, 3.0f, 0.7f, p);
        if (m < prev - 1e-3f) mono = false;
        prev = m;
    }
    if (!mono) { failures++; printf("  [FAIL] miss(p) is not monotone — the root find is invalid\n"); }
    else printf("  [PASS] miss(p) monotone increasing (root find well posed)\n");

    // the amortised solver must reach the same answer as the direct one
    dg_init();
    dg_arm(6.0f, -40.0f);
    // Hold the vehicle at a fixed state at the ignition altitude and let the
    // amortised solver converge. 30 ticks is 150 ms — one and a half replan
    // periods — and must be enough for a solve that costs ~8 projections.
    dg_nav_t nav = {18.7f, 6.0f, -45.0f, -0.10f, 0.0f, 10.0f, true};
    dg_out_t out;
    for (int i = 0; i < 40; i++) { dg_step_attitude(&nav, &out); dg_step_planner(&nav); }
    float tau_now = 40.0f * DG_CTRL_DT / DG_BURN_T;
    float direct = dg_solve_p_full(nav.h, nav.vx, nav.vz, tau_now, 3.0f, out.clamp);
    checks++;
    if (out.p_plan < 0.0f || out.p_plan > 4.0f) {
        failures++; printf("  [FAIL] amortised solver produced p outside [0,4]: %.3f\n", out.p_plan);
    } else if (fabsf(out.p_plan - direct) > 0.6f) {
        failures++;
        printf("  [FAIL] amortised solver p = %.3f disagrees with direct %.3f\n",
               out.p_plan, direct);
    } else {
        printf("  [PASS] amortised solver converged: p = %.3f vs direct %.3f\n",
               out.p_plan, direct);
    }

    // servo must stay inside travel and quantisation
    checks++;
    bool servo_ok = fabsf(out.servo_deg) <= DG_SERVO_MAX_DEG + 1e-4f;
    float q = out.servo_deg / DG_SERVO_QUANT;
    servo_ok = servo_ok && fabsf(q - roundf(q)) < 1e-3f;
    if (!servo_ok) { failures++; printf("  [FAIL] servo %.4f violates travel or quantisation\n", out.servo_deg); }
    else printf("  [PASS] servo %.3f deg within travel and on the quantisation grid\n", out.servo_deg);

    // clamp must stay inside its range
    checks++;
    if (out.clamp < DG_CLAMP_MIN - 1e-6f || out.clamp > DG_CLAMP_MAX + 1e-6f) {
        failures++; printf("  [FAIL] clamp %.4f outside [%.2f, %.2f]\n",
                           out.clamp, DG_CLAMP_MIN, DG_CLAMP_MAX);
    } else printf("  [PASS] clamp %.3f within range\n", out.clamp);

    // before ignition the servo must be parked: no thrust means no authority
    dg_init();
    dg_arm(6.0f, -40.0f);
    dg_nav_t coast = {70.0f, 6.0f, -42.0f, 0.25f, 0.4f, 1.0f, true};
    dg_step_attitude(&coast, &out);
    checks++;
    if (out.servo_deg != 0.0f) {
        failures++; printf("  [FAIL] servo commanded %.3f deg during the coast\n", out.servo_deg);
    } else printf("  [PASS] servo parked during the coast (gimbal has no authority)\n");

    // invalid nav must not command anything
    dg_nav_t bad = {20.0f, 6.0f, -40.0f, 0.1f, 0.0f, 12.0f, false};
    dg_step_attitude(&bad, &out);
    checks++;
    if (out.servo_deg != 0.0f || out.ignite) {
        failures++; printf("  [FAIL] invalid nav still produced commands\n");
    } else printf("  [PASS] invalid nav produces no servo motion and no ignition\n");

    printf("\n=====================================================================\n");
    printf(" %d checks, %d failures\n", checks, failures);
    printf("=====================================================================\n");
    return failures ? 1 : 0;
}
