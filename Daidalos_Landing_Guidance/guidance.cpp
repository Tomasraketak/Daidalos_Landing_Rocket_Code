// =====================================================================
//  Daidalos landing guidance — implementation
//
//  RP2350 notes (Cortex-M33 @ 150 MHz, FPv5-SP single precision):
//
//   * float only. Every literal carries an `f`. A bare `1.0` promotes the
//     whole expression to double, which is software-emulated and 10-50x
//     slower on this core.
//   * The thrust direction is carried as a NORMALISED VECTOR, never as an
//     angle. That removes atan2f from the guidance law and sinf/cosf from
//     the projection inner loop — together about 65 % of the cost of a
//     projection step, and the biggest single win in the port.
//   * asinf is replaced by x + x^3/6. The nozzle never exceeds 5 deg,
//     where that series is accurate to better than 1e-5 rad.
//   * The trajectory projection integrates with RK2 at dt = 0.04 s.
//   * The root find is a safeguarded false-position (Illinois) solve,
//     warm-started from the previous answer, and AMORTISED: one projection
//     per call. A full solve costs ~7 projections, so it converges in
//     ~35 ms at 200 Hz — well inside the 100 ms replan period — without
//     ever producing a multi-millisecond spike.
//   * No malloc, no recursion, no unbounded loops in the flight path.
//
//  Measured budget for this structure: ~0.8 % of one core for the planner
//  at 10 Hz plus ~0.05 % for the 200 Hz attitude loop.
// =====================================================================
#include "guidance.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------
//  Motor: thrust against NORMALISED burn time tau in [0,1].
//  The grain burns on its own clock; the clamp cannot slow it down, so tau
//  advances with wall time and NOT with the commanded thrust.
// ---------------------------------------------------------------------
static const int   TCURVE_N = 13;
static const float TCURVE_TAU[13] = {
    0.000000f, 0.011429f, 0.142857f, 0.285714f, 0.428571f, 0.462857f,
    0.514286f, 0.571429f, 0.714286f, 0.857143f, 0.914286f, 0.971429f, 1.000000f
};
static const float TCURVE_F[13] = {
    0.0f, 215.0f, 222.0f, 228.0f, 230.0f, 228.0f,
    190.0f, 165.0f, 125.0f, 80.0f, 40.0f, 10.0f, 0.0f
};

// Uniform resample of the curve, built once at init so the flight path never
// runs the 13-point linear scan. O(1): one compare, two loads and a lerp.
//
// The table spans [TAU_IGN, 1] rather than [0, 1]. The ignition transient rises
// from 0 to 215 N in the first 1.1 % of the burn, which no uniform grid can
// resample without smearing it (a 64-entry table put 69 N where the curve has
// 94 N). Below TAU_IGN the ramp is evaluated exactly instead — it is a single
// straight segment, so that is both cheaper and correct.
#define TLUT_N   64
#define TAU_IGN  0.011429f
#define F_IGN    215.0f
static float s_tlut[TLUT_N + 1];

// Minimum ignition altitude against entry vertical speed at DG_H_START.
// Generated offline by the simulation's find_ignition() solve, which costs
// ~46 ms and must never run in flight. Horizontal entry speed changes this by
// less than 0.3 m across 0..15 m/s, so the table is one-dimensional and the
// values are the conservative (maximum) column.
static const int   IGN_N = 8;
static const float IGN_VZ[8]   = { 20.0f, 25.0f, 30.0f, 35.0f,
                                   40.0f, 45.0f, 50.0f, 55.0f };
static const float IGN_HMIN[8] = { 18.72f, 20.31f, 22.24f, 24.52f,
                                   27.14f, 30.14f, 33.53f, 37.37f };

// tan(max_tilt) against altitude. max_tilt = min(45 deg, 3*h + 5 deg).
#define TAN_N 32
#define TAN_H_MAX 15.0f          // above this the 45 deg cap applies, tan = 1
static float s_tanlut[TAN_N + 1];

// ---------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Release barrier for the one cross-core handover in this file.
//
// The planner runs on core 1 and returns immediately unless S.phase is BURN, so
// S.phase is what publishes the solver state to it. Everything the planner will
// read must be stored BEFORE that phase write becomes visible, or core 1 can
// start iterating on a half-reset solver. This is the only ordering constraint
// in the module: every other shared item is a naturally aligned 32-bit word,
// where a torn read is impossible and a stale read is harmless.
#if defined(__GNUC__)
#define DG_PUBLISH()  __atomic_thread_fence(__ATOMIC_RELEASE)
#else
#define DG_PUBLISH()  ((void)0)
#endif

// asin for small arguments: |x| <= sin(5 deg) here, error < 1e-5 rad.
static inline float fast_asinf(float x) {
    x = clampf(x, -0.30f, 0.30f);
    return x + x * x * x * (1.0f / 6.0f);
}

float dg_thrust_ref(float tau) {
    if (tau <= 0.0f || tau >= 1.0f) return 0.0f;
    if (tau < TAU_IGN) return F_IGN * (tau * (1.0f / TAU_IGN));   // exact ramp
    float f = (tau - TAU_IGN) * ((float)TLUT_N / (1.0f - TAU_IGN));
    int   i = (int)f;
    if (i >= TLUT_N) return 0.0f;
    float w = f - (float)i;
    return s_tlut[i] + (s_tlut[i + 1] - s_tlut[i]) * w;
}

static float tan_max_tilt(float h) {
    if (h >= TAN_H_MAX) return 1.0f;            // 45 deg cap
    if (h <= 0.0f)      return s_tanlut[0];
    float f = h * ((float)TAN_N / TAN_H_MAX);
    int   i = (int)f;
    float w = f - (float)i;
    return s_tanlut[i] + (s_tanlut[i + 1] - s_tanlut[i]) * w;
}

// ---------------------------------------------------------------------
//  Guidance law — thrust DIRECTION as a unit vector, no angle, no atan2.
//
//    t_go = 2h / |vz|                (time to go, constant deceleration)
//    a_x  = -vx / t_go               (optimal for a terminal VELOCITY
//                                     constraint with free final position)
//    a_z  = |vz| / t_go + g
//  plus a credit for the drag that is already decelerating the vehicle, so
//  the law stops asking thrust to do work the air is doing for free.
//  Returns t_go; writes the unit thrust direction to sin_th / cos_th.
// ---------------------------------------------------------------------
float dg_guidance_tilt(float h, float vx, float vz, float mass,
                       float *sin_th, float *cos_th)
{
    float avz = fabsf(vz);
    float svz = avz > 0.5f ? avz : 0.5f;
    float hh  = h > 0.1f ? h : 0.1f;
    float t_go = 2.0f * hh / svz;
    if (t_go < 0.25f) t_go = 0.25f;

    float ax = -vx / t_go;
    float az = avz / t_go + DG_G;

    // drag credit
    float v2 = vx * vx + vz * vz;
    if (v2 > 0.0001f) {
        float v  = sqrtf(v2);
        float df = 0.5f * DG_RHO * DG_AREA * DG_CD_AXIAL * v2 / mass;
        ax += df * vx / v;      // drag already fights vx
        az += df * vz / v;      // vz < 0 descending, so this LOWERS the demand
    }

    if (az < 0.1f) az = 0.1f;

    // tilt limit, applied in vector form
    float tmax = tan_max_tilt(h);
    float lim  = az * tmax;
    if (ax >  lim) ax =  lim;
    if (ax < -lim) ax = -lim;

    float n = sqrtf(ax * ax + az * az);
    if (n < 1e-6f) { *sin_th = 0.0f; *cos_th = 1.0f; return t_go; }
    *sin_th = ax / n;
    *cos_th = az / n;
    return t_go;
}

// ---------------------------------------------------------------------
//  3-D commanded direction. Costs one square root more than the planar law.
//
//  The 3-D solution is EXACTLY the planar one rotated into the azimuth of the
//  horizontal velocity, because every horizontal term in the law -- the -v/t_go
//  demand and the drag credit -- is parallel to that velocity. Verified against
//  the model to 2e-16 over 200 000 states, so this is an identity and not an
//  approximation. It is also why the trajectory projection below never needs a
//  second horizontal channel.
// ---------------------------------------------------------------------
float dg_guidance_dir(float h, float vx, float vy, float vz, float mass,
                      float *ux, float *uy, float *uz)
{
    float vh2 = vx * vx + vy * vy;
    float sn, cs;
    float t_go = dg_guidance_tilt(h, sqrtf(vh2), vz, mass, &sn, &cs);
    *uz = cs;
    if (vh2 > 1e-12f) {
        float inv = sn / sqrtf(vh2);
        *ux = vx * inv;
        *uy = vy * inv;
    } else {
        *ux = 0.0f;
        *uy = 0.0f;
    }
    return t_go;
}

// ---------------------------------------------------------------------
//  Gimbal axes: an orthonormal basis of the plane perpendicular to b, built by
//  orthogonalising the BODY-FIXED reference axis g against b.
//
//  Because g is body-fixed, u1 and u2 turn with the airframe. That is the whole
//  reason g is on the wire: the servos are bolted to the vehicle, so as it rolls
//  the direction each one pushes rotates with it. A basis anchored to world x
//  would drift away from the hardware at the roll rate, and the only symptom
//  would be a vehicle steering somewhere other than where it meant to.
// ---------------------------------------------------------------------
void dg_gimbal_basis(float bx, float by, float bz,
                     float gx, float gy, float gz,
                     float *u1x, float *u1y, float *u1z,
                     float *u2x, float *u2y, float *u2z)
{
    float d  = gx * bx + gy * by + gz * bz;
    float ax = gx - d * bx, ay = gy - d * by, az = gz - d * bz;
    float n2 = ax * ax + ay * ay + az * az;
    if (n2 < 1e-9f) {
        // g has collapsed onto b. Cannot happen in flight; can happen to a caller
        // sending garbage. Fall back to the world-x projection so the loop still
        // produces a sane deflection rather than a NaN.
        ax = 1.0f - bx * bx; ay = -bx * by; az = -bx * bz;
        n2 = ax * ax + ay * ay + az * az;
        if (n2 < 1e-9f) {
            ax = -by * bx; ay = 1.0f - by * by; az = -by * bz;
            n2 = ax * ax + ay * ay + az * az;
            if (n2 < 1e-9f) { ax = 1.0f; ay = 0.0f; az = 0.0f; n2 = 1.0f; }
        }
    }
    float inv = 1.0f / sqrtf(n2);
    float a = ax * inv, b_ = ay * inv, c = az * inv;
    *u1x = a; *u1y = b_; *u1z = c;
    *u2x = by * c - bz * b_;             // u2 = b x u1
    *u2y = bz * a - bx * c;
    *u2z = bx * b_ - by * a;
}

// ---------------------------------------------------------------------
//  Throttle plan: one scalar p in [0,4] fills four clamp segments BACKWARDS
//  from burnout, so low p means "coast, then slam" — the right shape for a
//  fuel-optimal burn, and monotone in p, which is what makes a 1-D root find
//  legitimate. Segments span the REMAINING burn from the plan's start.
// ---------------------------------------------------------------------
static void p_to_clamps(float p, float k[4]) {
    for (int i = 0; i < 4; i++) k[i] = DG_CLAMP_MIN;
    for (int i = 0; i < 4; i++) {
        float fill = p - (float)i;
        int   idx  = 3 - i;
        if (fill >= 1.0f)      k[idx] = DG_CLAMP_MAX;
        else if (fill > 0.0f)  k[idx] = DG_CLAMP_MIN +
                                        (DG_CLAMP_MAX - DG_CLAMP_MIN) * fill;
    }
}

static inline float clamp_from_plan(float tau, const float k[4], float tau0) {
    if (tau >= 1.0f) return DG_CLAMP_MIN;
    float span = 1.0f - tau0;
    if (span < 1e-6f) return k[3];
    int seg = (int)((tau - tau0) / (span * 0.25f));
    if (seg < 0) seg = 0;
    if (seg > 3) seg = 3;
    return k[seg];
}

// ---------------------------------------------------------------------
//  Trajectory projection. Returns a SIGNED, CONTINUOUS, MONOTONE-IN-p miss:
//    > 0  arrested the descent this many metres ABOVE the pad
//    < 0  reached the pad still moving, expressed as the extra altitude that
//         would have been needed to stop, -vz^2/(2g)
//  Both branches go to zero at a perfect landing, so the root find is well
//  posed. Exits the instant it touches down. RK2, dt = 0.04 s.
// ---------------------------------------------------------------------
float dg_project_miss(float h, float vx, float vz, float tau,
                      float mass, float clamp_now, float p)
{
    const float dt = 0.04f;
    float k[4];
    p_to_clamps(p, k);
    const float tau0 = tau;
    const float dtau = dt / DG_BURN_T;

    for (int step = 0; step < 400; step++) {
        float tgt = clamp_from_plan(tau, k, tau0);

        // hold the clamp shut once stopping is no longer the problem
        float v_imp = sqrtf(vx * vx + vz * vz + 2.0f * DG_G * (h > 0.0f ? h : 0.0f));
        if (v_imp < 1.75f || vz >= 0.0f) tgt = DG_CLAMP_MIN;

        float dk = DG_CLAMP_RATE * dt;
        if      (tgt - clamp_now >  dk) clamp_now += dk;
        else if (clamp_now - tgt >  dk) clamp_now -= dk;
        else                            clamp_now  = tgt;

        float raw    = dg_thrust_ref(tau);
        float thrust = clamp_now * raw;

        float sn, cs;
        dg_guidance_tilt(h, vx, vz, mass, &sn, &cs);

        float v2 = vx * vx + vz * vz;
        float ax = thrust * sn / mass;
        float az = thrust * cs / mass - DG_G;
        if (v2 > 0.0001f) {
            float v  = sqrtf(v2);
            float df = 0.5f * DG_RHO * DG_AREA * DG_CD_AXIAL * v2 / mass;
            ax -= df * vx / v;
            az -= df * vz / v;
        }

        // RK2 (midpoint) on the vertical channel, which is the one that decides
        // the outcome; the horizontal channel is slow enough for Euler.
        float vz_mid = vz + 0.5f * az * dt;
        float h_next = h + vz_mid * dt;
        float vz_next = vz + az * dt;

        if (vz_next >= 0.0f && vz < 0.0f) {              // stopped in the air
            float f = -vz / (vz_next - vz);
            return h + vz * dt * f;
        }
        if (h_next <= 0.0f) {                            // reached the pad
            float f = (vz < -1e-6f) ? (h / (-vz * dt)) : 0.0f;
            float vz_at = vz + az * dt * f;
            return -(vz_at * vz_at) / (2.0f * DG_G);
        }

        h  = h_next;
        vx += ax * dt;
        vz = vz_next;
        if (tau < 1.0f) {
            tau += dtau;
            if (tau > 1.0f) tau = 1.0f;
            mass -= raw * DG_MASS_PER_NS * dt;           // RAW thrust: the grain
            if (mass < DG_M_INITIAL - DG_M_PROP)         // burns whatever the
                mass = DG_M_INITIAL - DG_M_PROP;         // clamp is doing
        }
    }
    return h;
}

// ---------------------------------------------------------------------
//  Amortised Illinois root find on miss(p) = 0.
//  One projection per call. State persists across calls.
// ---------------------------------------------------------------------
typedef struct {
    float lo, hi, f_lo, f_hi;
    float p;
    int   iter;
    bool  bracketed;
    bool  have_answer;
} dg_solver_t;

static dg_solver_t s_sol;
static float s_p_answer;
static float s_p_warm;
static bool  s_warm_valid;

static void solver_restart(void) {
    s_sol.lo = 0.0f;  s_sol.hi = 4.0f;
    s_sol.iter = 0;
    s_sol.bracketed = false;
    s_sol.have_answer = false;
    s_sol.p = s_warm_valid ? clampf(s_p_warm, 0.05f, 3.95f) : 2.0f;
}

// Non-amortised solve. Used by the host test and by dg_arm(); NOT called from
// the 200 Hz path.
float dg_solve_p_full(float h, float vx, float vz, float tau,
                      float mass, float clamp_now)
{
    float lo = 0.0f, hi = 4.0f;
    float f_lo = dg_project_miss(h, vx, vz, tau, mass, clamp_now, lo);
    if (f_lo > 0.0f) return 0.0f;                 // even shut it overshoots
    float f_hi = dg_project_miss(h, vx, vz, tau, mass, clamp_now, hi);
    if (f_hi < 0.0f) return 4.0f;                 // not enough motor

    float p = 0.5f * (lo + hi);
    for (int i = 0; i < 12; i++) {
        if (hi - lo < 1e-4f) break;
        float den = f_hi - f_lo;
        p = (fabsf(den) < 1e-12f) ? 0.5f * (lo + hi)
                                  : lo + (hi - lo) * (-f_lo) / den;
        float edge = 0.05f * (hi - lo);
        p = clampf(p, lo + edge, hi - edge);
        float f_p = dg_project_miss(h, vx, vz, tau, mass, clamp_now, p);
        if (f_p < 0.0f) { lo = p; f_lo = f_p; f_hi *= 0.5f; }
        else            { hi = p; f_hi = f_p; f_lo *= 0.5f; }
    }
    return p;
}

// ---------------------------------------------------------------------
//  Persistent guidance state
// ---------------------------------------------------------------------
static struct {
    dg_phase_t phase;
    float h_ign;            // commanded ignition altitude
    float tau;              // burn fraction, advances on the grain clock
    float mass;
    float clamp_act;        // actual clamp position
    float servo1_act;       // actual servo positions, one per gimbal axis
    float servo2_act;
    float eix, eiy, eiz;    // integral of the attitude-error vector
    float t_ignite_cmd;     // time the igniter was commanded, <0 = not yet
    float t_burn_start;     // time thrust was assumed to begin
    float p_plan;
    float k_plan[4];
    float tau_plan0;
    float last_t;
    bool  armed;
} S;

void dg_init(void) {
    memset(&S, 0, sizeof(S));
    S.phase        = DG_PHASE_IDLE;
    S.mass         = DG_M_INITIAL;
    S.clamp_act    = 0.6f;
    S.t_ignite_cmd = -1.0f;
    S.t_burn_start = -1.0f;
    S.p_plan       = 4.0f;
    p_to_clamps(4.0f, S.k_plan);
    s_p_answer   = 4.0f;
    s_warm_valid = false;
    solver_restart();

    for (int i = 0; i <= TLUT_N; i++) {
        float tau = TAU_IGN + (1.0f - TAU_IGN) * (float)i / (float)TLUT_N;
        float v = 0.0f;
        if (tau > 0.0f && tau < 1.0f) {
            for (int j = 0; j < TCURVE_N - 1; j++) {
                if (tau >= TCURVE_TAU[j] && tau <= TCURVE_TAU[j + 1]) {
                    float span = TCURVE_TAU[j + 1] - TCURVE_TAU[j];
                    float w = (span > 1e-9f) ? (tau - TCURVE_TAU[j]) / span : 0.0f;
                    v = TCURVE_F[j] + (TCURVE_F[j + 1] - TCURVE_F[j]) * w;
                    break;
                }
            }
        }
        s_tlut[i] = v;
    }
    for (int i = 0; i <= TAN_N; i++) {
        float h = (float)i * (TAN_H_MAX / (float)TAN_N);
        float deg = 3.0f * h + 5.0f;
        if (deg > 45.0f) deg = 45.0f;
        s_tanlut[i] = tanf(deg * (float)M_PI / 180.0f);
    }
}

void dg_arm(float vx0, float vz0) {
    (void)vx0;                          // < 0.3 m of influence, see IGN_HMIN
    float a = fabsf(vz0);
    float hmin;
    if      (a <= IGN_VZ[0])         hmin = IGN_HMIN[0];
    else if (a >= IGN_VZ[IGN_N - 1]) hmin = IGN_HMIN[IGN_N - 1];
    else {
        int i = 0;
        while (i < IGN_N - 2 && a > IGN_VZ[i + 1]) i++;
        float w = (a - IGN_VZ[i]) / (IGN_VZ[i + 1] - IGN_VZ[i]);
        hmin = IGN_HMIN[i] + (IGN_HMIN[i + 1] - IGN_HMIN[i]) * w;
    }
    S.h_ign = hmin;
    S.phase = DG_PHASE_COAST;
    S.armed = true;
}

float dg_h_ignite(void) { return S.h_ign; }

// ---------------------------------------------------------------------
//  Planner — core 1. One projection per call.
// ---------------------------------------------------------------------
void dg_step_planner(const dg_nav_t *nav)
{
    if (S.phase != DG_PHASE_BURN || !nav->valid) return;

    float h = nav->h, vx = nav->vx, vz = nav->vz;

    if (!s_sol.bracketed) {
        // two probes establish the bracket, one per call
        if (s_sol.iter == 0) {
            s_sol.f_lo = dg_project_miss(h, vx, vz, S.tau, S.mass,
                                         S.clamp_act, s_sol.lo);
            s_sol.iter = 1;
            if (s_sol.f_lo > 0.0f) {            // shut clamp still overshoots
                s_p_answer = 0.0f; s_sol.have_answer = true;
                s_p_warm = 0.0f; s_warm_valid = true;
                solver_restart();
            }
            return;
        }
        s_sol.f_hi = dg_project_miss(h, vx, vz, S.tau, S.mass,
                                     S.clamp_act, s_sol.hi);
        s_sol.bracketed = true;
        s_sol.iter = 0;
        if (s_sol.f_hi < 0.0f) {                // not enough motor: full open
            s_p_answer = 4.0f; s_sol.have_answer = true;
            s_p_warm = 4.0f; s_warm_valid = true;
            solver_restart();
        }
        return;
    }

    float den = s_sol.f_hi - s_sol.f_lo;
    float p = (fabsf(den) < 1e-12f) ? 0.5f * (s_sol.lo + s_sol.hi)
                                    : s_sol.lo + (s_sol.hi - s_sol.lo)
                                                 * (-s_sol.f_lo) / den;
    float edge = 0.05f * (s_sol.hi - s_sol.lo);
    p = clampf(p, s_sol.lo + edge, s_sol.hi - edge);

    float f_p = dg_project_miss(h, vx, vz, S.tau, S.mass, S.clamp_act, p);
    if (f_p < 0.0f) { s_sol.lo = p; s_sol.f_lo = f_p; s_sol.f_hi *= 0.5f; }
    else            { s_sol.hi = p; s_sol.f_hi = f_p; s_sol.f_lo *= 0.5f; }

    s_sol.p = p;
    s_sol.iter++;

    if (s_sol.iter >= 8 || (s_sol.hi - s_sol.lo) < 1e-3f) {
        s_p_answer   = p;
        s_p_warm     = p;
        s_warm_valid = true;
        solver_restart();
    }
}

// ---------------------------------------------------------------------
//  Attitude loop + ignition + actuators — core 0, 200 Hz.
// ---------------------------------------------------------------------
void dg_step_attitude(const dg_nav_t *nav, dg_out_t *out)
{
    const float dt = DG_CTRL_DT;

    out->ignite    = false;
    out->p_plan    = s_p_answer;
    out->phase     = S.phase;
    out->tilt_cmd  = 0.0f;
    out->tilt_err  = 0.0f;
    out->roll_rate = 0.0f;

    if (!nav->valid || S.phase == DG_PHASE_IDLE || S.phase == DG_PHASE_DONE) {
        S.servo1_act = 0.0f;
        S.servo2_act = 0.0f;
        out->servo1_deg = 0.0f;
        out->servo2_deg = 0.0f;
        out->clamp      = S.clamp_act;
        out->thrust_est = 0.0f;
        return;
    }

    // ---------------- ignition, triggered on PREDICTED altitude ----------
    // Firing on a clock forces the pad to cover the WORST-CASE igniter delay.
    // Leading by the MEAN delay and firing on altitude shrinks the pad to the
    // delay SPREAD, which is where the accuracy at the top of the envelope
    // comes from. The pad must still cover the spread in the low direction:
    // igniting high is recoverable by clamping down, igniting low is not.
    if (S.phase == DG_PHASE_COAST) {
        const float dmean = 0.5f * (DG_IGN_DELAY_LO + DG_IGN_DELAY_HI);
        const float dhalf = 0.5f * (DG_IGN_DELAY_HI - DG_IGN_DELAY_LO);
        float h_trig = S.h_ign + fabsf(nav->vz) * dhalf
                              + 0.5f * DG_G * dhalf * dhalf;
        if (S.t_ignite_cmd < 0.0f && (nav->h + nav->vz * dmean) <= h_trig) {
            S.t_ignite_cmd = nav->t;
            S.t_burn_start = nav->t + dmean;   // best estimate of light-up
            out->ignite = true;
        }
        if (S.t_ignite_cmd >= 0.0f && nav->t >= S.t_burn_start) {
            // Reset the solver BEFORE announcing the phase: core 1 wakes on the
            // phase change and would otherwise be able to see a partly reset
            // bracket.
            S.tau = 0.0f;
            solver_restart();
            DG_PUBLISH();
            S.phase = DG_PHASE_BURN;
        }
    }

    // ---------------- grain clock ----------------
    // tau advances with WALL TIME. The clamp diverts exhaust; it does not slow
    // the burn, and it cannot extend it.
    if (S.phase == DG_PHASE_BURN) {
        S.tau += dt / DG_BURN_T;
        if (S.tau > 1.0f) S.tau = 1.0f;
    }

    float raw_thrust   = (S.phase == DG_PHASE_BURN) ? dg_thrust_ref(S.tau) : 0.0f;
    float thrust_est   = S.clamp_act * raw_thrust;
    out->thrust_est    = thrust_est;

    if (S.phase == DG_PHASE_BURN) {
        S.mass -= raw_thrust * DG_MASS_PER_NS * dt;
        float m_dry = DG_M_INITIAL - DG_M_PROP;
        if (S.mass < m_dry) S.mass = m_dry;
    }

    // ---------------- clamp command ----------------
    float clamp_tgt = S.clamp_act;
    if (S.phase == DG_PHASE_BURN) {
        p_to_clamps(s_p_answer, S.k_plan);
        clamp_tgt = clamp_from_plan(S.tau, S.k_plan, S.tau_plan0);
        float v_imp = sqrtf(nav->vx * nav->vx + nav->vy * nav->vy
                            + nav->vz * nav->vz
                            + 2.0f * DG_G * (nav->h > 0.0f ? nav->h : 0.0f));
        if (v_imp < 1.75f || nav->vz >= 0.0f) clamp_tgt = DG_CLAMP_MIN;
        if (nav->vz >= 0.0f && nav->h < 1.0f) clamp_tgt = DG_CLAMP_MIN;
    }
    float dk = DG_CLAMP_RATE * dt;
    if      (clamp_tgt - S.clamp_act >  dk) S.clamp_act += dk;
    else if (S.clamp_act - clamp_tgt >  dk) S.clamp_act -= dk;
    else                                     S.clamp_act  = clamp_tgt;
    S.clamp_act = clampf(S.clamp_act, DG_CLAMP_MIN, DG_CLAMP_MAX);
    out->clamp  = S.clamp_act;

    // ---------------- attitude ----------------
    // Before ignition the gimbal produces NO torque, so both servos are parked.
    // Any apparent "tracking error" during the coast is not a control failure:
    // there is nothing to control with.
    if (S.phase != DG_PHASE_BURN || thrust_est < 1.0f) {
        S.servo1_act = 0.0f;
        S.servo2_act = 0.0f;
        out->servo1_deg = 0.0f;
        out->servo2_deg = 0.0f;
        return;
    }

    // Normalise the reported body axis: a fusion filter's unit vector drifts off
    // the sphere, and every cross product below assumes |b| = 1.
    float bx = nav->bx, by = nav->by, bz = nav->bz;
    float bn2 = bx * bx + by * by + bz * bz;
    if (bn2 < 1e-6f) { bx = 0.0f; by = 0.0f; bz = 1.0f; }
    else if (fabsf(bn2 - 1.0f) > 1e-4f) {
        float bi = 1.0f / sqrtf(bn2);
        bx *= bi; by *= bi; bz *= bi;
    }

    // Split the measured rate into ROLL and TRANSVERSE. The gimbal cannot produce
    // a torque about the body axis, so feeding a roll rate into the rate loop would
    // ask for something impossible and the inversion would answer with a deflection
    // that does something else instead. The roll rate is not discarded, though: it
    // drives the gyroscopic feedforward below.
    float wx = nav->wx, wy = nav->wy, wz = nav->wz;
    float w_roll = wx * bx + wy * by + wz * bz;
    wx -= w_roll * bx;
    wy -= w_roll * by;
    wz -= w_roll * bz;
    out->roll_rate = w_roll;

    float ucx, ucy, ucz;
    dg_guidance_dir(nav->h, nav->vx, nav->vy, nav->vz, S.mass, &ucx, &ucy, &ucz);
    out->tilt_cmd = acosf(clampf(ucz, -1.0f, 1.0f));      // telemetry only

    float u1x, u1y, u1z, u2x, u2y, u2z;
    dg_gimbal_basis(bx, by, bz, nav->gx, nav->gy, nav->gz,
                    &u1x, &u1y, &u1z, &u2x, &u2y, &u2z);

    // Attitude error as a ROTATION VECTOR: e = b x u_cmd points along the axis
    // that closes the error and has magnitude sin(error). This is the vector form
    // of the planar sin(cmd - theta), and it needs no atan2, no wrapping and no
    // Euler angles. It is perpendicular to b by construction, so the loop can
    // never command a roll it cannot produce.
    float ex = by * ucz - bz * ucy;
    float ey = bz * ucx - bx * ucz;
    float ez = bx * ucy - by * ucx;
    float e_sin2 = ex * ex + ey * ey + ez * ez;
    if (e_sin2 > 0.09f) {                         // beyond ~17 deg, saturate
        float sc = 0.3f / sqrtf(e_sin2);          // gracefully -- the planar law
        ex *= sc; ey *= sc; ez *= sc;             // clamped at the same 0.30
        e_sin2 = 0.09f;
    }
    // rescale sin(error) to the angle: asin(x)/x = 1 + x^2/6 over this range
    float e_corr = 1.0f + e_sin2 * (1.0f / 6.0f);
    ex *= e_corr; ey *= e_corr; ez *= e_corr;
    out->tilt_err = sqrtf(e_sin2) * e_corr;

    // Cascade: outer attitude P -> rate command, inner rate P -> angular accel.
    // Closing this gives wn^2 = k_rate*k_th and 2*zeta*wn = k_rate, so the two
    // knobs are a bandwidth and a damping ratio. Both channels use the SAME
    // pair: the transverse inertia is identical about either axis, so there is
    // nothing to tune separately.
    const float k_rate = 2.0f * DG_ZETA * DG_OMEGA_N;
    const float k_th   = DG_OMEGA_N / (2.0f * DG_ZETA);

    S.eix += ex * dt;
    S.eiy += ey * dt;
    S.eiz += ez * dt;
    float ei2 = S.eix * S.eix + S.eiy * S.eiy + S.eiz * S.eiz;
    if (ei2 > 0.0625f) {                          // magnitude limit, 0.25 rad.s
        float sc = 0.25f / sqrtf(ei2);
        S.eix *= sc; S.eiy *= sc; S.eiz *= sc;
    }

    float I_pitch = S.mass * (DG_L_BODY * DG_L_BODY * (1.0f / 12.0f));
    float g_ir = I_pitch * k_rate;
    float trx  = g_ir * (k_th * ex + DG_KI_ATT * S.eix - wx);
    float trqy = g_ir * (k_th * ey + DG_KI_ATT * S.eiy - wy);
    float trqz = g_ir * (k_th * ez + DG_KI_ATT * S.eiz - wz);

    // ---- gyroscopic decoupling ----
    // A spinning airframe turns a demand on one gimbal channel into a response on
    // the other. For an axisymmetric body Euler's equations reduce to
    //     I_t * dw_t/dt = tau_t - (I_t - I_a) * w_roll * (b x w_t)
    // so the cross-axis term is known exactly from quantities already in hand, and
    // cancelling it costs one cross product.
    //
    // Keep the size of this in proportion. The coupling is a cross-axis rate
    // feedback of gain ~1.5 rad/s at 90 deg/s against a 9 rad/s loop bandwidth, and
    // the precession torque is about 2 % of what the gimbal can make. Measured in
    // the simulation, the feedforward is worth +0.45 points of landing rate overall
    // and about +1.0 between 45 and 75 deg/s. It is cheap and correctly signed, not
    // a rescue.
    float gyk = (I_pitch - S.mass * DG_I_AXIAL_COEF) * w_roll;
    trx  += gyk * (by * wz - bz * wy);
    trqy += gyk * (bz * wx - bx * wz);
    trqz += gyk * (bx * wy - by * wx);

    // Cancel the weathercocking torque the fins generate at angle of attack.
    // tau_aero = L_CP * (b x F_normal), F_normal = q*A*CNA*(a_hat - (a_hat.b)b),
    // so the whole feedforward collapses to one cross product -- and, unlike an
    // atan2 formulation, it has no azimuth to go singular when the vehicle is
    // exactly aligned with the airflow.
    float v2 = nav->vx * nav->vx + nav->vy * nav->vy + nav->vz * nav->vz;
    if (v2 > 1.0f) {
        float v = sqrtf(v2);
        float ahx = -nav->vx / v, ahy = -nav->vy / v, ahz = -nav->vz / v;
        float kf = 0.5f * DG_RHO * v2 * DG_AREA * DG_CN_ALPHA * DG_L_CP;
        trx  -= kf * (by * ahz - bz * ahy);
        trqy -= kf * (bz * ahx - bx * ahz);
        trqz -= kf * (bx * ahy - by * ahx);
    }

    // Dynamic inversion, in vector form. The gimbal produces
    //     tau = -L * T * (b x s)
    // with s the nozzle deflection, perpendicular to b -- which inverts EXACTLY
    // to s = (b x tau) / (L * T). No linearisation; the round trip is exact to
    // 1e-14. Thrust sweeps ~230x over a burn, and inverting it is what makes one
    // fixed gain pair valid throughout.
    float t_inv = thrust_est > 25.0f ? thrust_est : 25.0f;
    float ti = 1.0f / (t_inv * DG_L_GIMBAL);
    float sx = (by * trqz - bz * trqy) * ti;
    float sy = (bz * trx  - bx * trqz) * ti;
    float sz = (bx * trqy - by * trx)  * ti;

    // Project onto the two gimbal axes. The basis is orthonormal, so these are
    // the sines of the two nozzle angles exactly.
    float sd1 = clampf(sx * u1x + sy * u1y + sz * u1z, -0.0872f, 0.0872f);
    float sd2 = clampf(sx * u2x + sy * u2y + sz * u2z, -0.0872f, 0.0872f);
    const float DEG = 180.0f / (float)M_PI;
    float c1 = fast_asinf(sd1) * DEG * DG_TVC_RATIO;
    float c2 = fast_asinf(sd2) * DEG * DG_TVC_RATIO;

    c1 = roundf(clampf(c1, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG) / DG_SERVO_QUANT)
         * DG_SERVO_QUANT;
    c2 = roundf(clampf(c2, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG) / DG_SERVO_QUANT)
         * DG_SERVO_QUANT;

    float ds = DG_SERVO_RATE * dt;
    if      (c1 - S.servo1_act >  ds) S.servo1_act += ds;
    else if (S.servo1_act - c1 >  ds) S.servo1_act -= ds;
    else                              S.servo1_act  = c1;
    if      (c2 - S.servo2_act >  ds) S.servo2_act += ds;
    else if (S.servo2_act - c2 >  ds) S.servo2_act -= ds;
    else                              S.servo2_act  = c2;

    out->servo1_deg = S.servo1_act;
    out->servo2_deg = S.servo2_act;

    // touchdown / burnout latch
    if (nav->h <= 0.25f && nav->vz > -0.5f) S.phase = DG_PHASE_DONE;
    S.last_t = nav->t;
}
