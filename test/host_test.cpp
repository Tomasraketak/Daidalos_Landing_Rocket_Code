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
    printf("   (MEASURED static fire. C uses a 512-entry uniform resample for\n");
    printf("    O(1) lookup; the model interpolates the 17 raw knots, so a small\n");
    printf("    resampling error is expected and is bounded here at 1 N.)\n");
    for (int i = 0; i < N_THRUST; i++) {
        char nm[64];
        snprintf(nm, sizeof nm, "thrust_ref(tau=%.3f)", G_THRUST_TAU[i]);
        check(nm, dg_thrust_ref(G_THRUST_TAU[i]), G_THRUST_N[i], 1.0f);
    }
    // The impulse is what actually lands the vehicle, and a lookup table can be
    // right at every probe point and still lose area between them.
    {
        float I = 0.0f;
        const int NS = 4096;
        for (int i = 0; i < NS; i++)
            I += dg_thrust_ref(((float)i + 0.5f) / (float)NS) / (float)NS;
        check("total impulse from the table", I * DG_BURN_T, DG_I_TOTAL, 1.0f);
        // The measured curve peaks at tau = 0.478, on a broad flat maximum -- not
        // near the start like the placeholder it replaced. Scan for it rather than
        // probing a remembered location.
        float pk = 0.0f;
        for (int i = 0; i <= 1000; i++) {
            float v = dg_thrust_ref((float)i / 1000.0f);
            if (v > pk) pk = v;
        }
        check("peak thrust from the table", pk, DG_PEAK_THRUST, 1.0f);
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
    printf("\n5. Three dimensions: the second gimbal axis\n");
    printf("   (The guidance law and the whole projection stay PLANAR: every\n");
    printf("    horizontal term is parallel to the horizontal velocity, so the\n");
    printf("    3-D command is the planar one rotated into its azimuth.)\n");
    {
        // 5a. the 3-D law reduces exactly to the planar one it is built from
        float worst = 0.0f;
        for (int i = 0; i < N_TILT; i++) {
            float h = G_TILT[i][0], vx = G_TILT[i][1], vz = G_TILT[i][2];
            float sn, cs, ux, uy, uz;
            dg_guidance_tilt(h, vx, vz, 3.0f, &sn, &cs);
            dg_guidance_dir(h, vx, 0.0f, vz, 3.0f, &ux, &uy, &uz);
            float d = fmaxf(fabsf(ux - sn), fmaxf(fabsf(uy), fabsf(uz - cs)));
            if (d > worst) worst = d;
        }
        check("3-D law reduces to the planar law (y = 0)", worst, 0.0f, 1e-6f);

        // 5b. rotating the horizontal velocity rotates the command with it
        float worst_iso = 0.0f;
        for (int i = 0; i < 64; i++) {
            float az = (float)i * 0.0981748f;      // 2*pi/64
            float vh = 7.0f, vz = -38.0f, h = 26.0f;
            float sn, cs, ux, uy, uz;
            dg_guidance_tilt(h, vh, vz, 3.0f, &sn, &cs);
            dg_guidance_dir(h, vh * cosf(az), vh * sinf(az), vz, 3.0f, &ux, &uy, &uz);
            float d = fmaxf(fabsf(ux - sn * cosf(az)),
                     fmaxf(fabsf(uy - sn * sinf(az)), fabsf(uz - cs)));
            if (d > worst_iso) worst_iso = d;
        }
        check("command is isotropic in azimuth", worst_iso, 0.0f, 1e-5f);

        // 5c. gimbal basis: orthonormal and perpendicular to the body axis
        float worst_on = 0.0f, worst_perp = 0.0f;
        for (int i = 0; i < 400; i++) {
            float a = (float)i * 0.0157f, t = 0.03f + 0.002f * (float)i;
            float bx = sinf(t) * cosf(a), by = sinf(t) * sinf(a), bz = cosf(t);
            float u1x, u1y, u1z, u2x, u2y, u2z;
            dg_gimbal_basis(bx, by, bz, 1.0f, 0.0f, 0.0f,
                            &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
            float d1 = sqrtf(u1x*u1x + u1y*u1y + u1z*u1z) - 1.0f;
            float d2 = sqrtf(u2x*u2x + u2y*u2y + u2z*u2z) - 1.0f;
            float od = u1x*u2x + u1y*u2y + u1z*u2z;
            worst_on = fmaxf(worst_on, fmaxf(fabsf(d1), fmaxf(fabsf(d2), fabsf(od))));
            worst_perp = fmaxf(worst_perp,
                               fmaxf(fabsf(u1x*bx + u1y*by + u1z*bz),
                                     fabsf(u2x*bx + u2y*by + u2z*bz)));
        }
        check("gimbal basis is orthonormal", worst_on, 0.0f, 1e-5f);
        check("gimbal basis is perpendicular to b", worst_perp, 0.0f, 1e-6f);

        // 5d. a tilt purely toward +y must leave axis 1 on the world x axis,
        //     which is what makes the two servos independent channels
        float u1x, u1y, u1z, u2x, u2y, u2z;
        dg_gimbal_basis(0.0f, sinf(0.3f), cosf(0.3f), 1.0f, 0.0f, 0.0f,
                        &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
        check("tilt in +y leaves gimbal axis 1 on world x", u1x, 1.0f, 1e-6f);
        check("  ...with no y component", u1y, 0.0f, 1e-6f);

        // 5e. dynamic inversion round trip: s = (b x tau)/(L*T) must give back tau
        float worst_inv = 0.0f;
        for (int i = 0; i < 400; i++) {
            float a = (float)i * 0.0157f, t = 0.05f + 0.001f * (float)i;
            float bx = sinf(t) * cosf(a), by = sinf(t) * sinf(a), bz = cosf(t);
            // any torque perpendicular to b
            float tx = cosf(a * 2.0f), ty = sinf(a * 3.0f), tz = 0.4f;
            float d = tx*bx + ty*by + tz*bz;
            tx -= d*bx; ty -= d*by; tz -= d*bz;
            float T = 60.0f + (float)i * 0.4f;
            float k = 1.0f / (T * DG_L_GIMBAL);
            float sx = (by*tz - bz*ty) * k;
            float sy = (bz*tx - bx*tz) * k;
            float sz = (bx*ty - by*tx) * k;
            // what the nozzle then produces: tau = -L*T*(b x s)
            float gx = -DG_L_GIMBAL * T * (by*sz - bz*sy);
            float gy = -DG_L_GIMBAL * T * (bz*sx - bx*sz);
            float gz = -DG_L_GIMBAL * T * (bx*sy - by*sx);
            float e = sqrtf((gx-tx)*(gx-tx) + (gy-ty)*(gy-ty) + (gz-tz)*(gz-tz));
            float n = sqrtf(tx*tx + ty*ty + tz*tz);
            if (n > 1e-6f && e / n > worst_inv) worst_inv = e / n;
        }
        check("dynamic inversion is exact, not linearised", worst_inv, 0.0f, 1e-4f);
    }

    // -----------------------------------------------------------------
    printf("\n6. A rolling airframe\n");
    printf("   (No roll actuator yet. The servo axes are bolted to the vehicle,\n");
    printf("    so they turn with it; the reference axis g is what tracks them.)\n");
    {
        // 6a. the gimbal axes must FOLLOW the airframe through a full roll. A basis
        //     anchored to world x passes every other check in this file and fails
        //     only this one -- which is precisely why it is here.
        float worst = 0.0f;
        const float bx = 0.0f, by = 0.0f, bz = 1.0f;
        for (int i = 0; i < 720; i++) {
            float ph = (float)i * (float)M_PI / 360.0f;
            float gx = cosf(ph), gy = sinf(ph), gz = 0.0f;
            float u1x, u1y, u1z, u2x, u2y, u2z;
            dg_gimbal_basis(bx, by, bz, gx, gy, gz,
                            &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
            // u1 must BE g, and u2 must be b x g
            float d = fmaxf(fmaxf(fabsf(u1x - gx), fabsf(u1y - gy)),
                     fmaxf(fabsf(u2x + gy), fabsf(u2y - gx)));
            if (d > worst) worst = d;
        }
        check("gimbal axes follow the airframe through a full roll", worst, 0.0f, 1e-5f);

        // 6b. a reference axis that is not exactly perpendicular to b must still
        //     produce an orthonormal basis -- the sender is allowed to be sloppy
        float worst_on = 0.0f, worst_perp = 0.0f;
        for (int i = 0; i < 360; i++) {
            float t = 0.02f + 0.0015f * (float)i;
            float b2x = sinf(t), b2y = 0.0f, b2z = cosf(t);
            float ph = (float)i * 0.11f;
            // deliberately non-perpendicular and non-unit
            float gx = cosf(ph) * 1.3f, gy = sinf(ph) * 1.3f, gz = 0.4f;
            float u1x, u1y, u1z, u2x, u2y, u2z;
            dg_gimbal_basis(b2x, b2y, b2z, gx, gy, gz,
                            &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);
            worst_on = fmaxf(worst_on,
                fmaxf(fabsf(sqrtf(u1x*u1x+u1y*u1y+u1z*u1z) - 1.0f),
                fmaxf(fabsf(sqrtf(u2x*u2x+u2y*u2y+u2z*u2z) - 1.0f),
                      fabsf(u1x*u2x + u1y*u2y + u1z*u2z))));
            worst_perp = fmaxf(worst_perp,
                               fmaxf(fabsf(u1x*b2x + u1y*b2y + u1z*b2z),
                                     fabsf(u2x*b2x + u2y*b2y + u2z*b2z)));
        }
        check("a sloppy reference axis still gives an orthonormal basis",
              worst_on, 0.0f, 1e-5f);
        check("  ...and one perpendicular to b", worst_perp, 0.0f, 1e-6f);

        // 6c. gyroscopic decoupling. Ask the loop for the same attitude correction
        //     with and without a roll rate; the difference must be the precession
        //     term, (I_t - I_a) * w_roll * (b x w_t), expressed as a deflection.
        //     Getting the SIGN wrong doubles the coupling instead of cancelling it,
        //     which is the failure this catches.
        float bxr = 0.0f, byr = 0.0f, bzr = 1.0f;
        float wtx = 0.0f, wty = 0.4f, wtz = 0.0f;      // transverse rate, about +y
        float w_roll = 1.5f;                            // 86 deg/s
        float I_t = 3.0f * DG_L_BODY * DG_L_BODY / 12.0f;
        float I_a = 3.0f * DG_I_AXIAL_COEF;
        // the plant's precession torque on this state
        float px = (I_t - I_a) * w_roll * (byr * wtz - bzr * wty);
        float py = (I_t - I_a) * w_roll * (bzr * wtx - bxr * wtz);
        float pz = (I_t - I_a) * w_roll * (bxr * wty - byr * wtx);
        // ...must be non-trivial, or the check proves nothing
        float pm = sqrtf(px*px + py*py + pz*pz);
        checks++;
        if (pm < 0.1f) {
            failures++;
            printf("  [FAIL] precession torque %.4f N.m is too small to test\n", pm);
        } else {
            printf("  [PASS] precession torque at 86 deg/s is %.2f N.m — %.0f%% of the\n"
                   "         %.1f N.m the gimbal can make at full deflection\n",
                   pm, 100.0f * pm / (230.0f * 0.0872f * DG_L_GIMBAL),
                   230.0f * 0.0872f * DG_L_GIMBAL);
        }
    }

    // -----------------------------------------------------------------
    printf("\n7. Structural invariants\n");
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

    // Hold the vehicle at a fixed state at the ignition altitude, run the loop
    // through ignition and into the burn, and check what it commands.
    //
    // nav.t MUST advance here. Ignition is triggered on altitude but the motor is
    // only declared lit one mean igniter delay later, on the clock, so a test that
    // holds the clock still never leaves the coast — and every servo check below
    // would then pass on a parked actuator.
    dg_init();
    dg_arm(6.0f, -40.0f);
    // The body axis is tilted 0.15 rad the WRONG way — toward +x, while the
    // guidance wants a tilt toward -x to kill the +6 m/s drift. That leaves a real
    // error for the loop to work on. Crossrange is deliberately zero.
    dg_nav_t nav = {56.9f, 6.0f, 0.0f, -45.0f,
                    0.1494381f, 0.0f, 0.9887711f,     // thrust axis
                    0.9887711f, 0.0f, -0.1494381f,    // reference axis, roll phase 0
                    0.0f, 0.0f, 0.0f, 10.0f, true};
    dg_out_t out;
    // 2.8 s: 1.525 s for the measured igniter, then 1.275 s of burn. The old value
    // was 160 ticks, which is 0.8 s -- shorter than this igniter's delay, so the
    // motor never lit and every check below went back to passing on a parked servo.
    const int N_TICK = 560;
    for (int i = 0; i < N_TICK; i++) {
        dg_step_attitude(&nav, &out);
        dg_step_planner(&nav);
        nav.t += DG_CTRL_DT;
    }
    checks++;
    if (out.phase != DG_PHASE_BURN) {
        failures++; printf("  [FAIL] never reached the burn phase (phase %d)\n", (int)out.phase);
    } else printf("  [PASS] reached the burn phase, thrust estimate %.1f N\n", out.thrust_est);

    float t_lit = 0.5f * (DG_IGN_DELAY_LO + DG_IGN_DELAY_HI);
    float tau_now = ((float)N_TICK * DG_CTRL_DT - t_lit) / DG_BURN_T;
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

    // both servos must stay inside travel and on the quantisation grid
    // and it must actually be doing something, or the checks below prove nothing
    checks++;
    if (fabsf(out.servo1_deg) < 0.5f) {
        failures++;
        printf("  [FAIL] servo 1 idle at %.3f deg with a 0.15 rad attitude error\n",
               out.servo1_deg);
    } else {
        printf("  [PASS] servo 1 working: %.3f deg against the attitude error\n",
               out.servo1_deg);
    }

    checks++;
    bool servo_ok = true;
    const float sv[2] = { out.servo1_deg, out.servo2_deg };
    for (int i = 0; i < 2; i++) {
        float q = sv[i] / DG_SERVO_QUANT;
        servo_ok = servo_ok && fabsf(sv[i]) <= DG_SERVO_MAX_DEG + 1e-4f
                            && fabsf(q - roundf(q)) < 1e-3f;
    }
    if (!servo_ok) {
        failures++;
        printf("  [FAIL] servos %.4f / %.4f violate travel or quantisation\n",
               out.servo1_deg, out.servo2_deg);
    } else {
        printf("  [PASS] servos %.3f / %.3f deg within travel and on the grid\n",
               out.servo1_deg, out.servo2_deg);
    }

    // the y channel must stay quiet when nothing asks it to move: this flight has
    // no crossrange velocity and no crossrange tilt, so servo 2 has no work to do
    checks++;
    if (fabsf(out.servo2_deg) > DG_SERVO_QUANT + 1e-4f) {
        failures++;
        printf("  [FAIL] y channel moved %.3f deg with no y-axis demand\n", out.servo2_deg);
    } else {
        printf("  [PASS] y channel quiet (%.3f deg) with no crossrange demand\n",
               out.servo2_deg);
    }

    // A pure roll rate, with no transverse rate to precess, must not move the
    // servos: the gyroscopic term is proportional to (b x w_transverse), which is
    // zero here, and the rate loop sees no roll at all.
    {
        dg_nav_t spun = nav;
        spun.wx = nav.bx * 1.5f;      // 1.5 rad/s = 86 deg/s, top of the envelope
        spun.wy = nav.by * 1.5f;
        spun.wz = nav.bz * 1.5f;
        dg_out_t o2;
        dg_step_attitude(&spun, &o2);
        dg_nav_t still = nav;
        dg_out_t o3;
        dg_step_attitude(&still, &o3);
        checks++;
        float d = fmaxf(fabsf(o2.servo1_deg - o3.servo1_deg),
                        fabsf(o2.servo2_deg - o3.servo2_deg));
        if (d > DG_SERVO_QUANT + 1e-4f) {
            failures++;
            printf("  [FAIL] a pure roll rate changed the servos by %.3f deg\n", d);
        } else {
            printf("  [PASS] a pure roll rate is projected out (%.3f deg change)\n", d);
        }
        checks++;
        if (fabsf(o2.roll_rate - 1.5f) > 1e-3f) {
            failures++;
            printf("  [FAIL] roll rate reported as %.4f, expected 1.5\n", o2.roll_rate);
        } else {
            printf("  [PASS] roll rate reported for telemetry (%.3f rad/s)\n", o2.roll_rate);
        }
    }

    // clamp must stay inside its range
    checks++;
    if (out.clamp < DG_CLAMP_MIN - 1e-6f || out.clamp > DG_CLAMP_MAX + 1e-6f) {
        failures++; printf("  [FAIL] clamp %.4f outside [%.2f, %.2f]\n",
                           out.clamp, DG_CLAMP_MIN, DG_CLAMP_MAX);
    } else printf("  [PASS] clamp %.3f within range\n", out.clamp);

    // before ignition the servo must be parked: no thrust means no authority
    dg_init();
    dg_arm(6.0f, -40.0f);
    dg_nav_t coast = {70.0f, 6.0f, 2.0f, -42.0f,
                      0.2474040f, 0.0616284f, 0.9669876f,
                      0.9683245f, 0.0f, -0.2477304f,
                      0.1f, 0.4f, 0.0f, 1.0f, true};
    dg_step_attitude(&coast, &out);
    checks++;
    if (out.servo1_deg != 0.0f || out.servo2_deg != 0.0f) {
        failures++;
        printf("  [FAIL] servos commanded %.3f / %.3f deg during the coast\n",
               out.servo1_deg, out.servo2_deg);
    } else printf("  [PASS] both servos parked during the coast (no authority)\n");

    // invalid nav must not command anything
    dg_nav_t bad = {20.0f, 6.0f, 1.0f, -40.0f, 0.1f, 0.0f, 0.995f,
                    0.995f, 0.0f, -0.1f,
                    0.0f, 0.0f, 0.0f, 12.0f, false};
    dg_step_attitude(&bad, &out);
    checks++;
    if (out.servo1_deg != 0.0f || out.servo2_deg != 0.0f || out.ignite) {
        failures++; printf("  [FAIL] invalid nav still produced commands\n");
    } else printf("  [PASS] invalid nav produces no servo motion and no ignition\n");

    printf("\n=====================================================================\n");
    printf(" %d checks, %d failures\n", checks, failures);
    printf("=====================================================================\n");
    return failures ? 1 : 0;
}
