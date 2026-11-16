// =====================================================================
//  Daidalos landing guidance, version 2 — implementation
//
//  See guidance.h for the interface and the frames, and the LandSim_Light
//  README, "The flight algorithm, step by step", for the reasoning behind
//  every number. Section numbers below (S3, S4 ...) refer to that text.
//
//  RULES OF THIS FILE
//    * float only. The M33 has no double-precision hardware.
//    * no malloc, no recursion, every loop has a compile-time bound.
//    * everything that divides is clamped first (t_go, 1/T, 1/q, the axial
//      airspeed in the crossflow term). Each of them blows up somewhere in
//      a real flight.
// =====================================================================
#include "guidance.h"

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------
//  small helpers
// ---------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline float dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static inline void cross3(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
static const float DEG = 0.01745329252f;      // rad per degree
static const float RAD = 57.29577951f;        // degrees per rad

// ---------------------------------------------------------------------
//  parameters
// ---------------------------------------------------------------------
static dg_params_t P;

void dg_params_default(dg_params_t *p)
{
    p->mass_gross    = DG_M_GROSS;
    p->mass_prop     = DG_M_PROP;
    p->wn_tvc        = DG_WN_TVC;
    p->zeta_tvc      = DG_ZETA_TVC;
    p->sched_tvc     = DG_SCHED_TVC;
    p->att_ki        = 2.0f;
    p->att_i_lim     = 0.05f;
    p->wn_fin        = DG_WN_FIN;
    p->zeta_fin      = DG_ZETA_FIN;
    p->sched_fin     = DG_SCHED_FIN;
    p->roll_gain     = DG_ROLL_GAIN;
    p->roll_max_deg  = 2.0f;
    p->tilt_min_deg  = 1.5f;
    p->tilt_slope    = 1.5f;
    p->tilt_cap_deg  = 20.0f;
    p->alt_pad       = 1.0f;
    p->h_ign_bias    = 0.0f;
    p->h_ign_override = 0.0f;
    p->plan_target   = -0.5f;
    p->plan_margin   = 0.03f;
    p->plan_hold     = 0.30f;
    p->onset_bump    = 4.0f;
    p->onset_timeout = 0.5f;
    p->onset_assumed = 0.15f;
    p->fin_enable    = 1.0f;
    p->fin_max_deg   = DG_FIN_MAX_DEG;
    p->fin_brake     = 1.0f;
    p->fin_min_airspeed = 8.0f;
    p->fin_drift_gain   = 1.5f;
    p->fin_drift_max_deg = 5.0f;
    p->fin_ctrl_lead = 40.0f;
    p->fin_ascent_mode = 2.0f;
    p->fin_adapt     = 1.0f;
    p->fin_gain0     = 1.0f;
}

// ---------------------------------------------------------------------
//  state
//
//  Core 0 owns everything here except the three planner outputs, which
//  core 1 writes. Every cross-core field is one naturally aligned 32-bit
//  word: a torn read is impossible and a stale one is one planner period
//  old, which is what a 10 Hz plan is anyway.
// ---------------------------------------------------------------------
typedef struct {
    volatile dg_phase_t phase;
    volatile float t;            // s, this module's own clock (ticks * dt)
    float h_apogee, vh_apogee;
    float h_cmd;                 // commanded ignition altitude
    float t_cmd;                 // time of the ignition command
    volatile float t_burn0;      // time of thrust onset (valid in BURN)
    bool  onset_detected;        // false while the onset is only assumed
    bool  lidar_acquired;
    float a_base;                // axial specific force just before the command
    uint8_t bump_ticks;
    volatile float t_scale;      // estimated thrust / table
    float k_act;                 // modelled clamp position
    float k_cmd;
    bool  gate_open;             // clamp held open through the startup
    float ie[3];                 // attitude error integral
    float t_done;
    // planner outputs (core 1)
    volatile float k_level;
    volatile float plan_resid;
    volatile float t_td_abs;     // predicted touchdown, on this module's clock
    volatile float t_plan;       // when the plan was made
    volatile float t_nom_burn;   // DESCENT: predicted command-to-touchdown time
    // fin adaptation
    float kappa;
    float trim[3];               // rad: axis 1, axis 2, roll
    float w_prev[3];
    bool  w_prev_ok;
    float yf[3], rf[3];          // filtered measurements and regressors
    float ym[3], rm[3];          // their slow means
    float r_hist[4][3];          // regressor delay line
    uint8_t r_pos;
    float fin_last[4];
} dg_state_t;

static dg_state_t S;

void dg_set_params(const dg_params_t *p) { P = *p; }
const dg_params_t *dg_get_params(void)   { return &P; }

void dg_init(void)
{
    static bool have_params = false;
    if (!have_params) { dg_params_default(&P); have_params = true; }
    memset(&S, 0, sizeof(S));
    S.phase     = DG_PHASE_IDLE;
    S.t_scale   = 1.0f;
    S.k_act     = DG_CLAMP_MAX;
    S.k_cmd     = DG_CLAMP_MAX;
    S.k_level   = DG_CLAMP_MAX;
    S.gate_open = true;
    S.t_burn0   = -1.0f;
    S.t_td_abs  = -1.0f;
    S.t_nom_burn = -1.0f;
    S.kappa     = clampf(P.fin_gain0, 0.4f, 2.0f);
}

// =====================================================================
//  MOTOR TABLE (S11: thrust and cumulative impulse against time)
// =====================================================================
static inline uint8_t motor_seg(float t)
{
    // last index i with T[i] <= t; the table has 17 points, a scan is fine
    uint8_t i = 1;
    while (i + 1 < DG_MOTOR_N - 1 && DG_MOTOR_T[i + 1] <= t) i++;
    return i;
}

float dg_thrust_ref(float t)
{
    if (t < 0.0f || t > DG_BURN_T) return 0.0f;
    uint8_t i = motor_seg(t);
    float span = DG_MOTOR_T[i + 1] - DG_MOTOR_T[i];
    if (span < 1e-9f) return DG_MOTOR_F[i + 1];
    float f = (t - DG_MOTOR_T[i]) / span;
    return DG_MOTOR_F[i] + f * (DG_MOTOR_F[i + 1] - DG_MOTOR_F[i]);
}

float dg_burned(float t)
{
    if (t <= 0.0f)       return 0.0f;
    if (t >= DG_BURN_T)  return P.mass_prop;
    uint8_t i = motor_seg(t);
    float span = DG_MOTOR_T[i + 1] - DG_MOTOR_T[i];
    float f = span < 1e-9f ? 1.0f : (t - DG_MOTOR_T[i]) / span;
    float cum = DG_MOTOR_CUM[i] + f * (DG_MOTOR_CUM[i + 1] - DG_MOTOR_CUM[i]);
    return P.mass_prop * cum / DG_I_TOTAL;
}

static inline float mass_at(float t_burn)
{
    return P.mass_gross - dg_burned(t_burn);
}

// =====================================================================
//  IGNITION ALTITUDE (S3) — computed on the ground, looked up in flight
// =====================================================================
float dg_h_cmd_lookup(float h_apogee, float vh)
{
    float fh = (h_apogee - DG_IGN_H0) / DG_IGN_DH;
    fh = clampf(fh, 0.0f, (float)(DG_IGN_NH - 1));
    int ih = (int)fh;
    if (ih > DG_IGN_NH - 2) ih = DG_IGN_NH - 2;
    float th = fh - (float)ih;

    vh = clampf(fabsf(vh), DG_IGN_VX[0], DG_IGN_VX[DG_IGN_NV - 1]);
    int iv = 0;
    while (iv < DG_IGN_NV - 2 && DG_IGN_VX[iv + 1] <= vh) iv++;
    float tv = (vh - DG_IGN_VX[iv]) / (DG_IGN_VX[iv + 1] - DG_IGN_VX[iv]);

    float a = DG_IGN_TABLE[ih][iv]     + tv * (DG_IGN_TABLE[ih][iv + 1]     - DG_IGN_TABLE[ih][iv]);
    float c = DG_IGN_TABLE[ih + 1][iv] + tv * (DG_IGN_TABLE[ih + 1][iv + 1] - DG_IGN_TABLE[ih + 1][iv]);
    return a + th * (c - a);
}

float dg_h_ignite(void) { return S.h_cmd; }

void dg_begin_ascent(void)
{
    if (S.phase == DG_PHASE_IDLE) S.phase = DG_PHASE_ASCENT;
}

void dg_begin_descent(float h_apogee, float vh_apogee)
{
    if (S.phase != DG_PHASE_IDLE && S.phase != DG_PHASE_ASCENT) return;
    S.h_apogee  = h_apogee;
    S.vh_apogee = vh_apogee;
    float h = (P.h_ign_override > 0.0f)
            ? P.h_ign_override
            : dg_h_cmd_lookup(h_apogee, vh_apogee) + P.h_ign_bias;
    // The command may never sit above where the vehicle already is.
    if (h > h_apogee - 2.0f) h = h_apogee - 2.0f;
    if (h < 5.0f)            h = 5.0f;
    S.h_cmd = h;
    S.phase = DG_PHASE_DESCENT;
}

// =====================================================================
//  GUIDANCE LAW (S4): commanded thrust direction
// =====================================================================
float dg_guidance_dir(float h, float vx, float vy, float vz, float mass,
                      float *ux, float *uy, float *uz)
{
    float svz = fabsf(vz) > 0.5f ? fabsf(vz) : 0.5f;
    float hh  = h > 0.1f ? h : 0.1f;
    float t_go = 2.0f * hh / svz;
    if (t_go < 0.25f) t_go = 0.25f;

    float k_h = 1.0f / t_go;
    float ax = -k_h * vx;
    float ay = -k_h * vy;
    float az = fabsf(vz) / t_go + DG_G;

    // credit the drag: the air is already decelerating the vehicle
    float v = sqrtf(vx * vx + vy * vy + vz * vz);
    if (v > 0.01f) {
        float df = 0.5f * DG_RHO * DG_AREA * DG_CD_BODY * v * v;
        float s  = df / (v * mass);
        ax += s * vx;
        ay += s * vy;
        az += s * vz;
    }
    if (az < 0.1f) az = 0.1f;

    // tilt cone: wide up high, tight near the pad, the same in every azimuth
    float max_tilt = clampf(h * P.tilt_slope + P.tilt_min_deg, 0.0f, P.tilt_cap_deg) * DEG;
    float tmax = tanf(max_tilt);
    float rh = sqrtf(ax * ax + ay * ay);
    if (rh > tmax * az && rh > 1e-12f) {
        float s = tmax * az / rh;
        ax *= s;
        ay *= s;
    }
    float n = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    *ux = ax * n;  *uy = ay * n;  *uz = az * n;
    return t_go;
}

void dg_gimbal_basis(float bx, float by, float bz,
                     float gx, float gy, float gz,
                     float *u1x, float *u1y, float *u1z,
                     float *u2x, float *u2y, float *u2z)
{
    float d  = gx * bx + gy * by + gz * bz;
    float ax = gx - d * bx, ay = gy - d * by, az = gz - d * bz;
    float n  = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) {
        ax = 1.0f - bx * bx;  ay = -bx * by;  az = -bx * bz;
        n = sqrtf(ax * ax + ay * ay + az * az);
        if (n < 1e-6f) { ax = 0.0f; ay = 1.0f; az = 0.0f; n = 1.0f; }
    }
    ax /= n;  ay /= n;  az /= n;
    *u1x = ax;  *u1y = ay;  *u1z = az;
    *u2x = by * az - bz * ay;
    *u2y = bz * ax - bx * az;
    *u2z = bx * ay - by * ax;
}

// =====================================================================
//  CLAMP LAW (S8)
// =====================================================================
bool dg_terminal_active(float h, float vz, float thrust_avail, float mass)
{
    return h < DG_H_TERM && vz > -3.0f && thrust_avail > 1.6f * mass * DG_G;
}

float dg_terminal_throttle(float h, float vz, float mass, float thrust_avail)
{
    float hh = h > 0.05f ? h : 0.05f;
    float vz_ref = -clampf(sqrtf(2.0f * DG_A_TERM * hh), 0.8f, 2.5f);
    float a_cmd = DG_G + DG_KP_VZ * (vz_ref - vz);
    if (a_cmd > DG_A_TERM_MAX) a_cmd = DG_A_TERM_MAX;
    if (thrust_avail < 1.0f) return DG_CLAMP_MIN;
    return clampf(mass * a_cmd / thrust_avail, DG_CLAMP_MIN, DG_CLAMP_MAX);
}

static inline float k_from_plan(float k_level, bool hold_over, float h, float vz,
                                float mass, float thrust_avail)
{
    if (dg_terminal_active(h, vz, thrust_avail, mass))
        return dg_terminal_throttle(h, vz, mass, thrust_avail);
    if (hold_over) return DG_CLAMP_MAX;
    return k_level;
}

// Forward-integrate the vertical channel to the ground, ballistic fall
// after burnout included: a plan that over-brakes, stops in mid-air with a
// spent grain and falls back must be seen as the failure it is.
float dg_project_vz(float h0, float vz0, float t0, float k_level, float t_scale,
                    float *t_td)
{
    float h = h0, vz = vz0, t = t0;
    const float kd = 0.5f * DG_RHO * DG_AREA * DG_CD_BURN;
    const float dt = DG_PLAN_DT;
    // The planned level is held for plan_hold and the projection then
    // assumes FULL clamp. Counted in steps, not by comparing an accumulated
    // float time against the limit: that comparison is decided by rounding
    // (15 * 0.02 is or is not > 0.3) and one step of difference is 0.6 m/s.
    const int n_hold = (int)(P.plan_hold / dt + 0.5f);
    int n = 0;
    while (h > 0.0f && n < DG_PLAN_NMAX) {
        float tn   = dg_thrust_ref(t) * t_scale;
        float mass = mass_at(t);
        float k    = k_from_plan(k_level, n >= n_hold, h, vz, mass, tn);
        float a    = (tn * k - kd * vz * fabsf(vz)) / mass - DG_G;
        float vn   = vz + a * dt;
        h  += 0.5f * (vz + vn) * dt;
        vz  = vn;
        t  += dt;
        n++;
    }
    if (t_td) *t_td = t - t0;
    return vz;
}

// The touchdown speed is NOT monotone in the clamp level: too little
// crashes, enough lands, too much stops the vehicle in mid-air and drops
// it. So: scan, bracket the FIRST crossing from below, bisect only that;
// if nothing reaches the target, ternary-search the peak.
float dg_solve_plan(float h, float vz, float t, float t_scale,
                    float *resid, float *t_td)
{
    const int   n_scan = 10;
    const float kmin = DG_CLAMP_MIN, kmax = DG_CLAMP_MAX;
    const float target = P.plan_target;
    float best_k = kmin, best_f = -1.0e9f, best_t = 0.0f;
    float prev_k = kmin;
    float lo = -1.0f, hi = -1.0f, tt = 0.0f;

    for (int i = 0; i < n_scan; i++) {
        float k = kmin + (kmax - kmin) * (float)i / (float)(n_scan - 1);
        float f = dg_project_vz(h, vz, t, k, t_scale, &tt) - target;
        if (f > best_f) { best_f = f; best_k = k; best_t = tt; }
        if (f >= 0.0f) {
            if (i == 0) {
                if (resid) *resid = f;
                if (t_td)  *t_td  = tt;
                return kmin;
            }
            lo = prev_k;  hi = k;  best_t = tt;
            break;
        }
        prev_k = k;
    }

    if (lo < 0.0f) {
        float a = best_k - (kmax - kmin) / (float)(n_scan - 1);
        float c = best_k + (kmax - kmin) / (float)(n_scan - 1);
        if (a < kmin) a = kmin;
        if (c > kmax) c = kmax;
        for (int i = 0; i < 12; i++) {
            float m1 = a + (c - a) / 3.0f;
            float m2 = c - (c - a) / 3.0f;
            float t1, t2;
            float f1 = dg_project_vz(h, vz, t, m1, t_scale, &t1) - target;
            float f2 = dg_project_vz(h, vz, t, m2, t_scale, &t2) - target;
            if (f1 > best_f) { best_f = f1; best_k = m1; best_t = t1; }
            if (f2 > best_f) { best_f = f2; best_k = m2; best_t = t2; }
            if (f1 < f2) a = m1; else c = m2;
        }
        if (resid) *resid = best_f;
        if (t_td)  *t_td  = best_t;
        return best_k;
    }

    for (int i = 0; i < 10; i++) {
        float mid = 0.5f * (lo + hi);
        float f = dg_project_vz(h, vz, t, mid, t_scale, &tt) - target;
        if (f >= 0.0f) { hi = mid; best_t = tt; }
        else             lo = mid;
    }
    if (resid) *resid = 0.0f;
    if (t_td)  *t_td  = best_t;
    return hi;
}

// =====================================================================
//  FIN MIXER (S6)
//
//  delta_i = roll + A cos(phi_i) + B sin(phi_i) + crossflow_i + brake*(+,-,+,-)
//
//  `kappa` and `trim` are the on-line estimates: the fins are assumed to
//  deliver kappa times the modelled torque per radian, plus a constant
//  offset of `trim` radians of equivalent deflection per channel. With
//  kappa = 1 and trim = 0 this is the simulation's mixer exactly.
// =====================================================================
void dg_fin_mix(const float b[3], const float u1[3], const float u2[3],
                const float v[3], const float tau[3], float tau_roll,
                float brake, float kappa, const float trim[3],
                bool control, float out_deg[4])
{
    const float fin_max = P.fin_max_deg;
    const float amax    = DG_FIN_STALL_DEG - DG_FIN_STALL_MARGIN;

    float v2 = dot3(v, v);
    float q  = 0.5f * DG_RHO * v2;
    float k_t = 2.0f * DG_FIN_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT;
    float k_r = 4.0f * DG_FIN_ROLL_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT;
    // Below this airspeed the 1/q in the inversion runs away and the four
    // fins saturate asymmetrically, which is a roll torque.
    if (sqrtf(v2) < P.fin_min_airspeed) { k_t = 0.0f; k_r = 0.0f; }
    if (!control) k_t = 0.0f;

    float a_cmd = 0.0f, b_cmd = 0.0f, d_roll = 0.0f;
    if (k_t > 1e-6f) {
        // NOTE THE MINUS: the fins are aft of the CG, so for the same force
        // they make the opposite torque to the nozzle.
        a_cmd = (-dot3(tau, u1) / k_t - trim[0]) / kappa;
        b_cmd = (-dot3(tau, u2) / k_t - trim[1]) / kappa;
    }
    if (k_r > 1e-6f) {
        d_roll = (tau_roll / k_r - trim[2]) / kappa;
        d_roll = clampf(d_roll, -P.roll_max_deg * DEG, P.roll_max_deg * DEG);
    }
    a_cmd  = clampf(a_cmd * RAD, -fin_max, fin_max);
    b_cmd  = clampf(b_cmd * RAD, -fin_max, fin_max);
    d_roll = d_roll * RAD;

    float va = -dot3(v, b);                 // axial airspeed, nose to tail
    if (va < 3.0f) va = 3.0f;

    float ctrl[4];
    float room = fin_max;
    for (int i = 0; i < 4; i++) {
        // phi_i = i * 90 deg: cos/sin are 1,0 / 0,1 / -1,0 / 0,-1
        const float ca = (i == 0) ? 1.0f : (i == 2 ? -1.0f : 0.0f);
        const float sa = (i == 1) ? 1.0f : (i == 3 ? -1.0f : 0.0f);
        float r[3] = { ca * u1[0] + sa * u2[0], ca * u1[1] + sa * u2[1],
                       ca * u1[2] + sa * u2[2] };
        float n[3];
        cross3(b, r, n);
        float xflow = (dot3(v, n) / va) * RAD;
        // With the attitude loop not flying the fins, leave them to see the
        // crossflow: that is what makes a fin a fin. Cancelling it turns
        // them into weathervanes and leaves only the body's own margin.
        float xf_c = (k_t > 1e-6f) ? xflow : 0.0f;
        float c = d_roll + a_cmd * ca + b_cmd * sa + xf_c;
        // never command a fin past the stall: clip the ANGLE OF ATTACK
        float a_i = c - xflow;
        if (a_i > amax)       c = xflow + amax;
        else if (a_i < -amax) c = xflow - amax;
        c = clampf(c, -fin_max, fin_max);
        ctrl[i] = c;
        float r1 = fin_max - fabsf(c);
        if (r1 < room) room = r1;
        float r2 = amax - fabsf(c - xflow);
        if (r2 < room) room = r2;
    }
    if (room < 0.0f) room = 0.0f;
    // control first, brake with what is left, and THE SAME on all four
    if (brake > room) brake = room;
    for (int i = 0; i < 4; i++) {
        float alt = (i % 2 == 0) ? 1.0f : -1.0f;
        out_deg[i] = clampf(ctrl[i] + brake * alt, -fin_max, fin_max);
    }
}

// =====================================================================
//  FIN ADAPTATION
//
//  The simulation's fins are a CFD table. The real ones will not match it:
//  the lift slope, the hinge linkage ratio, a servo horn one spline off.
//  So the controller measures what the fins actually do.
//
//  Model, per transverse axis and for roll, in "equivalent deflection":
//
//      y = kappa * (delivered command) + trim
//
//  where y is the deflection the MODEL says would have produced the
//  angular acceleration the gyro just measured (after removing the torques
//  the controller models: body aero and the gyroscopic term). kappa = 1
//  and trim = 0 mean the model is right.
//
//  Estimation is a normalised gradient step, deliberately not a recursive
//  least squares: with no excitation an RLS covariance winds up, and the
//  free fall is mostly un-excited. The step for kappa is scaled by the
//  command energy, so it only moves when the fins are actually working;
//  the trims move slowly all the time. Measurement and regressor go
//  through the same first-order filter, and the regressor is delayed by
//  the servo and link latency, so the comparison is like for like.
//
//  It runs only with the motor OFF. During the burn the nozzle is the
//  dominant torque and the regression would learn the nozzle's errors as
//  fin errors; the estimates are frozen at ignition.
// =====================================================================
static const float ADAPT_TAU     = 0.06f;    // s, filter on both sides
static const float ADAPT_TAU_MEAN = 1.0f;    // s, the slow means
static const float ADAPT_MU_K    = 0.03f;    // per tick, kappa
static const float ADAPT_MU_C    = 0.004f;   // per tick, trims
static const float ADAPT_KMIN    = 0.4f;
static const float ADAPT_KMAX    = 2.0f;
static const float ADAPT_TRIM_MAX = 4.0f * 0.01745329252f;

static void fin_adapt_step(const dg_nav_t *nav, const float b[3], const float u1[3],
                           const float u2[3], const float tau_model[3],
                           float I_t, float I_a, float w_roll)
{
    const float w[3] = { nav->wx, nav->wy, nav->wz };
    if (!S.w_prev_ok) {
        memcpy(S.w_prev, w, sizeof(w));
        S.w_prev_ok = true;
        return;
    }
    float dw[3] = { (w[0] - S.w_prev[0]) * DG_CTRL_HZ, (w[1] - S.w_prev[1]) * DG_CTRL_HZ,
                    (w[2] - S.w_prev[2]) * DG_CTRL_HZ };
    memcpy(S.w_prev, w, sizeof(w));

    const float v[3] = { nav->vx, nav->vy, nav->vz };
    float v2 = dot3(v, v);
    float q  = 0.5f * DG_RHO * v2;
    float k_t = 2.0f * DG_FIN_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT;
    float k_r = 4.0f * DG_FIN_ROLL_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT;
    if (sqrtf(v2) < P.fin_min_airspeed + 4.0f || k_t < 1e-3f) return;

    // What the fins were last told, as angles of attack per channel.
    float va = -dot3(v, b);
    if (va < 3.0f) va = 3.0f;
    float a[4];
    for (int i = 0; i < 4; i++) {
        const float ca = (i == 0) ? 1.0f : (i == 2 ? -1.0f : 0.0f);
        const float sa = (i == 1) ? 1.0f : (i == 3 ? -1.0f : 0.0f);
        float r[3] = { ca * u1[0] + sa * u2[0], ca * u1[1] + sa * u2[1],
                       ca * u1[2] + sa * u2[2] };
        float n[3];
        cross3(b, r, n);
        // angle of attack actually flown: deflection, minus the crossflow,
        // minus what the fin's own motion (w x p) adds. The last term is the
        // fins' aerodynamic damping; putting it in the regressor rather than
        // in the model means it scales with kappa, as it physically does.
        float p[3] = { DG_FIN_ARM * b[0] + DG_FIN_ROLL_ARM * r[0],
                       DG_FIN_ARM * b[1] + DG_FIN_ROLL_ARM * r[1],
                       DG_FIN_ARM * b[2] + DG_FIN_ROLL_ARM * r[2] };
        float wxp[3];
        cross3(w, p, wxp);
        a[i] = S.fin_last[i] * DEG - (dot3(v, n) + dot3(wxp, n)) / va;
    }
    float reg_now[3] = { 0.5f * (a[0] - a[2]), 0.5f * (a[1] - a[3]),
                         0.25f * (a[0] + a[1] + a[2] + a[3]) };

    // delay line: 3 ticks of servo + link latency
    float reg[3];
    memcpy(reg, S.r_hist[S.r_pos], sizeof(reg));
    memcpy(S.r_hist[S.r_pos], reg_now, sizeof(reg_now));
    S.r_pos = (uint8_t)((S.r_pos + 1) & 3);

    // The deflection the model says explains the measured acceleration.
    // tau_fin = I*dw - tau_model, and tau_fin.u1 = -k_t * (deflection).
    float tm[3] = { I_t * dw[0] - tau_model[0], I_t * dw[1] - tau_model[1],
                    I_t * dw[2] - tau_model[2] };
    float y[3] = { -dot3(tm, u1) / k_t, -dot3(tm, u2) / k_t,
                   I_a * dot3(dw, b) / k_r };
    (void)w_roll;

    float alpha = DG_CTRL_DT / (ADAPT_TAU + DG_CTRL_DT);
    for (int i = 0; i < 3; i++) {
        S.yf[i] += alpha * (y[i]   - S.yf[i]);
        S.rf[i] += alpha * (reg[i] - S.rf[i]);
    }

    // kappa is learned from the CHANGES only (signals with their slow means
    // removed), the trims from the means. A constant the model does not
    // know about - wind on the fins, a bent fin - is then a trim, and cannot
    // masquerade as a gain just because the command happens to be non-zero.
    float beta = DG_CTRL_DT / (ADAPT_TAU_MEAN + DG_CTRL_DT);
    float num = 0.0f, den = 2.5e-5f;          // (0.29 deg)^2 dead zone
    for (int i = 0; i < 3; i++) {
        S.ym[i] += beta * (S.yf[i] - S.ym[i]);
        S.rm[i] += beta * (S.rf[i] - S.rm[i]);
        float dy = S.yf[i] - S.ym[i];
        float dr = S.rf[i] - S.rm[i];
        num += dr * (dy - S.kappa * dr);
        den += dr * dr;
    }
    S.kappa = clampf(S.kappa + ADAPT_MU_K * num / den, ADAPT_KMIN, ADAPT_KMAX);
    for (int i = 0; i < 3; i++) {
        float e = S.ym[i] - (S.kappa * S.rm[i] + S.trim[i]);
        S.trim[i] = clampf(S.trim[i] + ADAPT_MU_C * e, -ADAPT_TRIM_MAX, ADAPT_TRIM_MAX);
    }
}

// =====================================================================
//  200 Hz CONTROL STEP
// =====================================================================
void dg_step_control(const dg_nav_t *nav, dg_out_t *out)
{
    S.t += DG_CTRL_DT;
    const float now = S.t;

    // ---- defaults: everything parked, clamp where it was ----
    out->servo1_deg = 0.0f;
    out->servo2_deg = 0.0f;
    out->clamp      = S.k_cmd;
    out->fins_live  = false;
    for (int i = 0; i < 4; i++) out->fin_deg[i] = 0.0f;
    out->ignite     = (S.phase == DG_PHASE_IGNITING || S.phase == DG_PHASE_BURN);
    out->thrust_on  = (S.phase == DG_PHASE_BURN) && S.onset_detected;
    out->phase      = S.phase;
    out->t_to_ignition  = -1.0f;
    out->t_to_touchdown = -1.0f;
    out->h_cmd      = S.h_cmd;
    out->tilt_cmd   = 0.0f;
    out->tilt_err   = 0.0f;
    out->roll_rate  = 0.0f;
    out->thrust_est = 0.0f;
    out->t_scale    = S.t_scale;
    out->k_plan     = S.k_level;
    out->plan_resid = S.plan_resid;
    out->fin_gain   = S.kappa;
    out->fin_trim[0] = S.trim[0] * RAD;
    out->fin_trim[1] = S.trim[1] * RAD;
    out->fin_trim[2] = S.trim[2] * RAD;
    out->t_burn     = (S.phase == DG_PHASE_BURN) ? now - S.t_burn0 : -1.0f;

    if (S.phase == DG_PHASE_IDLE || S.phase == DG_PHASE_DONE) {
        S.k_cmd = DG_CLAMP_MAX;
        out->clamp = S.k_cmd;
        return;
    }

    // A lost navigation solution parks the nozzle and the fins and FREEZES
    // the clamp. Shutting it mid-burn guarantees a crash and the grain
    // finishes burning either way. The burn clock keeps running.
    if (!nav->valid) {
        S.w_prev_ok = false;
        return;
    }

    float b[3] = { nav->bx, nav->by, nav->bz };
    {
        float n = sqrtf(dot3(b, b));
        if (n < 0.5f) return;
        b[0] /= n;  b[1] /= n;  b[2] /= n;
    }
    const float v[3] = { nav->vx, nav->vy, nav->vz };
    const float w[3] = { nav->wx, nav->wy, nav->wz };

    // ---- the altitude the guidance works from ----
    // Until the lidar has the ground, assume the vehicle may be alt_pad
    // LOWER than the altimeter says; drop the margin the moment it acquires.
    if (nav->lidar_ok) S.lidar_acquired = true;
    const float h_est = nav->h - (S.lidar_acquired ? 0.0f : P.alt_pad);

    // ---- phase logic ----------------------------------------------------
    if (S.phase == DG_PHASE_DESCENT) {
        // time until the ignition command, from a constant-deceleration fall
        float dh = h_est - S.h_cmd;
        float vd = -nav->vz;                              // positive downward
        float kd = 0.5f * DG_RHO * DG_AREA * DG_CD_FREE / P.mass_gross;
        float a  = DG_G - kd * vd * fabsf(vd);
        if (dh <= 0.0f) {
            out->t_to_ignition = 0.0f;
        } else if (fabsf(a) > 0.05f) {
            float disc = vd * vd + 2.0f * a * dh;
            out->t_to_ignition = disc > 0.0f ? (-vd + sqrtf(disc)) / a
                                             : (vd > 0.5f ? dh / vd : 99.0f);
        } else {
            out->t_to_ignition = vd > 0.5f ? dh / vd : 99.0f;
        }
        if (out->t_to_ignition < 0.0f)  out->t_to_ignition = 0.0f;
        if (out->t_to_ignition > 99.0f) out->t_to_ignition = 99.0f;
        if (S.t_nom_burn > 0.0f)
            out->t_to_touchdown = out->t_to_ignition + S.t_nom_burn;

        // axial specific force just before the command: the baseline the
        // onset detector measures the bump against
        S.a_base += 0.05f * (nav->a_axial - S.a_base);

        // The command. Thrust axis must be in the upper hemisphere: a motor
        // lit while pointing down accelerates the vehicle into the ground.
        if (h_est <= S.h_cmd && nav->vz < -3.0f && b[2] > 0.0f) {
            S.phase      = DG_PHASE_IGNITING;
            S.t_cmd      = now;
            S.bump_ticks = 0;
            S.t_plan     = -1.0f;
        }
    } else if (S.phase == DG_PHASE_IGNITING) {
        // Thrust onset comes from the accelerometer, never from the command:
        // the igniter delay is the dominant uncertainty and everything
        // downstream is timed from the real onset.
        if (nav->a_axial - S.a_base > P.onset_bump) {
            if (++S.bump_ticks >= 2) {
                S.t_burn0 = now - 0.04f;      // ~11 N is 40 ms up the curve
                S.onset_detected = true;
                S.phase = DG_PHASE_BURN;
            }
        } else {
            S.bump_ticks = 0;
        }
        if (S.phase == DG_PHASE_IGNITING && now - S.t_cmd > P.onset_timeout) {
            // No bump seen. If the motor did light and the accelerometer
            // path is what failed, flying blind on an assumed onset beats
            // not flying. A later bump still re-synchronises the clock.
            S.t_burn0 = S.t_cmd + P.onset_assumed;
            S.onset_detected = false;
            S.phase = DG_PHASE_BURN;
        }
        if (S.phase == DG_PHASE_BURN) {
            S.ie[0] = S.ie[1] = S.ie[2] = 0.0f;       // the integral starts at ignition
            S.gate_open = true;
        }
    } else if (S.phase == DG_PHASE_BURN) {
        if (!S.onset_detected && now - S.t_burn0 < DG_BURN_T) {
            // Assumed onset: keep looking for the real one. The detector ran
            // every tick since the command, so a bump seen now IS the rising
            // edge (a late igniter, or the backup one) and the burn clock is
            // re-synchronised to it.
            if (nav->a_axial - S.a_base > P.onset_bump) {
                if (++S.bump_ticks >= 2) {
                    S.t_burn0 = now - 0.04f;
                    S.onset_detected = true;
                    S.gate_open = true;
                    S.t_scale = 1.0f;
                }
            } else {
                S.bump_ticks = 0;
            }
        }
        float tb = now - S.t_burn0;
        // down and stopped, or long after burnout
        if ((h_est < 0.5f && fabsf(nav->vz) < 1.0f && tb > 0.5f) || tb > DG_BURN_T + 4.0f) {
            if (S.t_done <= 0.0f) S.t_done = now;
            if (now - S.t_done > 0.3f || tb > DG_BURN_T + 4.0f) S.phase = DG_PHASE_DONE;
        } else {
            S.t_done = 0.0f;
        }
    }

    const dg_phase_t phase = S.phase;
    const float tb = (phase == DG_PHASE_BURN) ? now - S.t_burn0 : -1.0f;
    const bool  engine_on = (tb >= 0.0f && tb <= DG_BURN_T);
    const float mass = (tb > 0.0f) ? mass_at(tb) : P.mass_gross;
    out->phase     = phase;
    out->ignite    = (phase == DG_PHASE_IGNITING || phase == DG_PHASE_BURN);
    out->thrust_on = (phase == DG_PHASE_BURN) && S.onset_detected;
    out->t_burn    = tb;
    if (phase == DG_PHASE_DONE) {
        S.k_cmd = DG_CLAMP_MAX;
        out->clamp = S.k_cmd;
        return;
    }

    // ---- body-fixed gimbal / fin axes ----
    float u1[3], u2[3];
    dg_gimbal_basis(b[0], b[1], b[2], nav->gx, nav->gy, nav->gz,
                    &u1[0], &u1[1], &u1[2], &u2[0], &u2[1], &u2[2]);

    // ---- where to point (S4) ----
    float u[3];
    float vh = sqrtf(v[0] * v[0] + v[1] * v[1]);
    if (phase == DG_PHASE_ASCENT) {
        u[0] = 0.0f;  u[1] = 0.0f;  u[2] = -1.0f;         // nose up
    } else if (phase == DG_PHASE_DESCENT && h_est > S.h_cmd + P.fin_ctrl_lead) {
        // Long free fall: hold the body axis vertical, optionally leaned
        // INTO the drift so the airframe's own normal force pushes back.
        u[0] = 0.0f;  u[1] = 0.0f;  u[2] = 1.0f;
        if (P.fin_drift_gain > 0.0f && vh > 0.2f && nav->vz < -P.fin_min_airspeed) {
            float tl = tanf(clampf(vh * P.fin_drift_gain, 0.0f, P.fin_drift_max_deg) * DEG);
            u[0] = tl * v[0] / vh;
            u[1] = tl * v[1] / vh;
            float n = 1.0f / sqrtf(u[0] * u[0] + u[1] * u[1] + 1.0f);
            u[0] *= n;  u[1] *= n;  u[2] = n;
        }
    } else {
        (void)dg_guidance_dir(h_est, v[0], v[1], v[2], mass, &u[0], &u[1], &u[2]);
    }
    out->tilt_cmd = acosf(clampf(fabsf(u[2]), -1.0f, 1.0f));

    // ---- attitude error as a rotation vector (S5 step 1) ----
    float e[3];
    cross3(b, u, e);
    float cn = sqrtf(dot3(e, e));
    float bu = dot3(b, u);
    float ang = 0.0f;
    if (cn > 1e-9f) {
        ang = asinf(clampf(cn, -1.0f, 1.0f));
        if (bu < 0.0f) ang = 3.14159265f - ang;
        float sc = ang / cn;
        e[0] *= sc;  e[1] *= sc;  e[2] *= sc;
    } else {
        e[0] = e[1] = e[2] = 0.0f;
        if (bu < 0.0f) ang = 3.14159265f;
    }
    out->tilt_err = ang;

    // ---- split off the roll (S5 step 2) ----
    float w_roll = dot3(w, b);
    float wt[3] = { w[0] - w_roll * b[0], w[1] - w_roll * b[1], w[2] - w_roll * b[2] };
    out->roll_rate = w_roll;

    const float I_t = mass * DG_IT_PER_KG;
    const float I_a = mass * DG_IA_PER_KG;

    // ---- aero feedforward (S5 step 4): tau = q A CNa Lcp (b x a_hat) ----
    float v2 = dot3(v, v);
    float q  = 0.5f * DG_RHO * v2;
    float taf[3] = { 0.0f, 0.0f, 0.0f };
    if (v2 > 0.0025f) {
        float vr = sqrtf(v2);
        float ah[3] = { -v[0] / vr, -v[1] / vr, -v[2] / vr };
        float c[3];
        cross3(b, ah, c);
        float ffk = q * DG_AREA * DG_CN_ALPHA * DG_L_CP;
        taf[0] = ffk * c[0];  taf[1] = ffk * c[1];  taf[2] = ffk * c[2];
    }

    // ---- clamp actuator model: where the clamp really is ----
    {
        float d = S.k_cmd - S.k_act;
        float step = DG_CLAMP_RATE * DG_CTRL_DT;
        S.k_act += clampf(d, -step, step);
        S.k_act = clampf(S.k_act, DG_CLAMP_MIN, DG_CLAMP_MAX);
    }

    // ---- thrust-scale estimator: the most valuable measurement on board ----
    // Only with a DETECTED onset. If the bump was never seen the axial
    // accelerometer path is not to be believed, and an estimator fed a dead
    // signal would talk the planner into a motor running 30 % weak.
    if (engine_on && tb > 0.05f && S.onset_detected) {
        float t_table = dg_thrust_ref(tb);
        float f_aero = 0.5f * DG_RHO * DG_AREA * DG_CD_BURN * v2 *
                       clampf(-dot3(v, b) / sqrtf(v2 + 1e-9f), -1.0f, 1.0f);
        float t_meas = nav->a_axial * mass - f_aero;
        float denom  = t_table * S.k_act;
        if (denom > 15.0f && t_meas > 0.0f) {
            float raw = clampf(t_meas / denom, 0.6f, 1.4f);
            float ts  = S.t_scale + (raw - S.t_scale) * clampf(DG_CTRL_DT / 0.25f, 0.0f, 1.0f);
            S.t_scale = clampf(ts, 0.7f, 1.3f);
        }
    }
    float t_est = engine_on ? dg_thrust_ref(tb) * S.t_scale * S.k_act : 0.0f;
    out->thrust_est = t_est;
    out->t_scale    = S.t_scale;
    if (t_est < 8.0f) t_est = 8.0f;            // floor for the inversion

    // ---- which loop is flying, and how hard (S7) ----
    float wn_a, zeta_a, fac;
    if (engine_on) {
        wn_a = P.wn_tvc;  zeta_a = P.zeta_tvc;
        fac = (P.sched_tvc != 0.0f) ? powf(t_est / DG_T_SCHED_REF, P.sched_tvc) : 1.0f;
    } else {
        wn_a = P.wn_fin;  zeta_a = P.zeta_fin;
        fac = (P.sched_fin != 0.0f && q > 1.0f) ? powf(q / DG_Q_SCHED_REF, P.sched_fin) : 1.0f;
    }
    wn_a *= clampf(fac, DG_SCHED_LO, DG_SCHED_HI);
    const float k_rate = 2.0f * zeta_a * wn_a;
    const float k_th   = wn_a / (2.0f * zeta_a);

    float oc[3] = { k_th * e[0], k_th * e[1], k_th * e[2] };
    // Integral, nozzle only: a misaligned nozzle or a CG off the axis is a
    // CONSTANT torque, and a proportional loop answers it with a constant
    // attitude error. Kept perpendicular to b and clamped.
    if (P.att_ki > 0.0f && engine_on) {
        for (int i = 0; i < 3; i++) S.ie[i] += e[i] * DG_CTRL_DT;
        float dpi = dot3(S.ie, b);
        for (int i = 0; i < 3; i++) S.ie[i] -= dpi * b[i];
        float inn = sqrtf(dot3(S.ie, S.ie));
        if (inn > P.att_i_lim)
            for (int i = 0; i < 3; i++) S.ie[i] *= P.att_i_lim / inn;
        for (int i = 0; i < 3; i++) oc[i] += k_th * P.att_ki * S.ie[i];
    }
    float tr[3];
    for (int i = 0; i < 3; i++) tr[i] = I_t * k_rate * (oc[i] - wt[i]) - taf[i];
    // gyroscopic decoupling (S5 step 5)
    float bxw[3];
    cross3(b, wt, bxw);
    const float gyk = (I_t - I_a) * w_roll;
    for (int i = 0; i < 3; i++) tr[i] += gyk * bxw[i];

    // ---- dynamic inversion: torque -> nozzle angle (S5 step 6) ----
    float cmd1 = 0.0f, cmd2 = 0.0f;
    if (engine_on) {
        float sq[3];
        cross3(b, tr, sq);
        float inv = 1.0f / (t_est * DG_L_GIMBAL);
        float sd1 = clampf(dot3(sq, u1) * inv, -0.999f, 0.999f);
        float sd2 = clampf(dot3(sq, u2) * inv, -0.999f, 0.999f);
        cmd1 = clampf(asinf(sd1) * RAD * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG);
        cmd2 = clampf(asinf(sd2) * RAD * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG);
        cmd1 = floorf(cmd1 / DG_SERVO_QUANT + 0.5f) * DG_SERVO_QUANT;
        cmd2 = floorf(cmd2 / DG_SERVO_QUANT + 0.5f) * DG_SERVO_QUANT;
    }
    out->servo1_deg = cmd1;
    out->servo2_deg = cmd2;

    // ---- fins (S6) ----
    if (P.fin_enable > 0.5f) {
        // torque the nozzle will actually deliver; the fins get the rest
        float rf[3] = { tr[0], tr[1], tr[2] };
        if (engine_on) {
            float s1 = sinf(cmd1 / DG_TVC_RATIO * DEG);
            float s2 = sinf(cmd2 / DG_TVC_RATIO * DEG);
            float gs[3] = { s1 * u1[0] + s2 * u2[0], s1 * u1[1] + s2 * u2[1],
                            s1 * u1[2] + s2 * u2[2] };
            float tg[3];
            cross3(b, gs, tg);
            for (int i = 0; i < 3; i++) rf[i] -= -DG_L_GIMBAL * t_est * tg[i];
        }
        float tau_roll = I_a * P.roll_gain * (0.0f - w_roll);

        // The attitude part only flies once the airframe is within reach of
        // its target: after apogee the vehicle turns over by itself, and a
        // loop demanding 180 degrees of correction from stalled fins at
        // walking pace only spins it up.
        bool control = (bu > 0.5f);
        float brake = 0.0f;
        if (phase == DG_PHASE_ASCENT) {
            if (P.fin_ascent_mode < 0.5f)      { control = false; tau_roll = 0.0f; }
            else if (P.fin_ascent_mode < 1.5f) { control = false; }
        } else if (!engine_on && P.fin_brake > 0.5f) {
            brake = P.fin_max_deg;             // deployed while the motor is unlit
        }
        const float trim0[3] = { 0.0f, 0.0f, 0.0f };
        const bool  adapt = P.fin_adapt > 0.5f;
        dg_fin_mix(b, u1, u2, v, rf, tau_roll, brake,
                   adapt ? S.kappa : clampf(P.fin_gain0, 0.4f, 2.0f),
                   adapt ? S.trim : trim0, control, out->fin_deg);
        out->fins_live = !(phase == DG_PHASE_ASCENT && P.fin_ascent_mode < 0.5f);

        // learn from what the previous commands did
        if (adapt && !engine_on && control &&
            (phase == DG_PHASE_DESCENT || phase == DG_PHASE_IGNITING)) {
            // the torques the controller models besides the fins: body
            // weathercock, body damping, gyroscopic coupling
            float c_damp = 0.5f * DG_RHO * sqrtf(v2) * DG_AREA * DG_CN_ALPHA * DG_L_CP * DG_L_CP;
            float tm[3];
            for (int i = 0; i < 3; i++) tm[i] = taf[i] - c_damp * wt[i] - gyk * bxw[i];
            fin_adapt_step(nav, b, u1, u2, tm, I_t, I_a, w_roll);
        } else {
            S.w_prev_ok = false;
        }
        memcpy(S.fin_last, out->fin_deg, sizeof(S.fin_last));
        out->fin_gain = S.kappa;
        out->fin_trim[0] = S.trim[0] * RAD;
        out->fin_trim[1] = S.trim[1] * RAD;
        out->fin_trim[2] = S.trim[2] * RAD;
    }

    // ---- clamp command (S8) ----
    if (phase == DG_PHASE_IGNITING || engine_on) {
        float t_nom = engine_on ? dg_thrust_ref(tb) * S.t_scale : 0.0f;
        float k = k_from_plan(S.k_level, false, h_est, nav->vz, mass, t_nom);
        // Startup gate: held wide open until the TABLE says the motor makes
        // 30 % of peak, latched, so tail-off is left to the planner.
        if (S.gate_open && engine_on && dg_thrust_ref(tb) >= DG_THRUST_GATE * DG_PEAK_THRUST)
            S.gate_open = false;
        if (S.gate_open) k = DG_CLAMP_MAX;
        S.k_cmd = k;
    } else {
        S.k_cmd = DG_CLAMP_MAX;
    }
    out->clamp  = S.k_cmd;
    out->k_plan = S.k_level;
    out->plan_resid = S.plan_resid;

    if (phase == DG_PHASE_IGNITING || phase == DG_PHASE_BURN) {
        float t_td = S.t_td_abs - now;
        if (S.t_td_abs > 0.0f) out->t_to_touchdown = t_td > 0.0f ? t_td : 0.0f;
    }
}

// =====================================================================
//  PLANNER — core 1
//
//  A full solve is at most 34 projections of ~200 steps each. On the M33
//  that is a few milliseconds, on a core that has nothing else to do, so
//  it simply runs to completion ten times a second.
// =====================================================================
void dg_step_planner(const dg_nav_t *nav)
{
    const dg_phase_t phase = S.phase;
    const float now = S.t;
    if (!nav->valid) return;
    const float h_est = nav->h - (S.lidar_acquired ? 0.0f : P.alt_pad);

    if (phase == DG_PHASE_IGNITING || phase == DG_PHASE_BURN) {
        if (S.t_plan >= 0.0f && now - S.t_plan < 1.0f / DG_PLAN_HZ) return;
        if (h_est <= 0.05f) return;
        float tb = (phase == DG_PHASE_BURN) ? now - S.t_burn0 : 0.0f;
        if (tb < 0.0f) tb = 0.0f;
        float resid = 0.0f, t_td = 0.0f;
        float k = dg_solve_plan(h_est, nav->vz, tb, S.t_scale * (1.0f - P.plan_margin),
                                &resid, &t_td);
        S.k_level    = k;
        S.plan_resid = resid;
        S.t_td_abs   = now + t_td;
        S.t_plan     = now;
        return;
    }

    if (phase == DG_PHASE_DESCENT) {
        // Once a second: how long from the ignition command to touchdown, so
        // the vehicle has a landing countdown before the motor is lit.
        if (S.t_plan >= 0.0f && now - S.t_plan < 1.0f) return;
        S.t_plan = now;
        // state at the command altitude, from an energy balance without drag
        // loss beyond what has already happened (conservative: a bit fast)
        float dh = h_est - S.h_cmd;
        float v_cmd = -sqrtf(nav->vz * nav->vz + 2.0f * DG_G * (dh > 0.0f ? dh : 0.0f));
        float kd = 0.5f * DG_RHO * DG_AREA * DG_CD_FREE / P.mass_gross;
        float vt = -sqrtf(DG_G / kd);                 // terminal velocity
        if (v_cmd < vt) v_cmd = vt;
        float t_delay = 0.5f * DG_IGN_DELAY_PAD;      // the mean igniter delay
        float h_on = S.h_cmd - (-v_cmd) * t_delay;
        if (h_on < 1.0f) h_on = 1.0f;
        float resid = 0.0f, t_td = 0.0f;
        (void)dg_solve_plan(h_on, v_cmd, 0.0f, 1.0f - P.plan_margin, &resid, &t_td);
        S.t_nom_burn = t_delay + t_td;
    }
}
