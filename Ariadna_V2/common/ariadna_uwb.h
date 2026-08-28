// =====================================================================
//  Ariadna V2 — UWB multilateration
//
//  Turns a set of DWM1000 ranges to fixed ground anchors into a POSITION
//  in the pad frame, together with an honest per-axis 1-sigma, so the
//  navigation Kalman filter can weight it like any other measurement.
//
//  Kept OUT of the sketch for the same reason ariadna_kf.h is: anything
//  inside a .ino can only be tested by flashing a board, and this file
//  contains the one piece of genuinely non-trivial geometry in the
//  vehicle. ../../test/uwb_test.cpp exercises every function here,
//  including every degraded case.
//
//  No Arduino dependency, no link-header dependency, single precision
//  throughout. Include it with a relative path:
//      #include "../common/ariadna_uwb.h"
//
//  ---------------------------------------------------------------------
//  THE PROBLEM
//  ---------------------------------------------------------------------
//  Four anchors sit on the ground around the pad at surveyed positions
//  a_i. The tag on the vehicle measures the range r_i to each of them:
//
//      r_i = | p - a_i | + noise
//
//  Three or more of those pin p down. The interesting question is not how
//  to solve it when all four report — it is what to do when they do not,
//  which on a launch site is most of the time: an anchor loses power, the
//  airframe shadows one during a roll, the plume ionises the path, a
//  packet collides. THE FILTER MUST NEVER SEE A WORSE MEASUREMENT THAN
//  IT IS TOLD IT IS SEEING, and it must never see a NaN.
//
//  So this solver degrades in defined steps rather than failing:
//
//    4+ anchors   full 3-D least squares. Position and altitude.
//    3  anchors   still 3-D in principle. In practice the anchors are
//                 COPLANAR (all on the ground), which makes altitude
//                 weakly observed near the ground and mirror-ambiguous
//                 about the anchor plane — so the covariance is computed
//                 and, if the vertical is not actually observable, the
//                 solve is repeated in 2-D with the height held at the
//                 filter's own estimate. Nothing is asserted that the
//                 geometry does not support.
//    2  anchors   two spheres intersect in a circle; with the height held
//                 that is two points. Gauss-Newton started from the
//                 filter's prior converges to the nearer one, and the
//                 covariance comes out correspondingly stretched along
//                 the ambiguous direction. Useful, and honestly weighted.
//    1  anchor    a single range is not a position, but it is still
//                 information: it constrains the distance to that anchor.
//                 Handled as a line-of-sight scalar (see below).
//    0  anchors   ARIADNA_UWB_SOL_NONE. The caller fuses nothing and the
//                 rest of the navigation solution carries on exactly as
//                 it did before UWB existed.
//
//  ---------------------------------------------------------------------
//  WHY THE OUTPUT IS ALWAYS "POSITION + PER-AXIS SIGMA"
//  ---------------------------------------------------------------------
//  Because the navigation filter is three independent per-axis Kalman
//  filters (see ariadna_kf.h), and a scalar update of one state is all it
//  can accept. Expressing every degraded mode in that same shape — even
//  the single-anchor one — means the fusion code in the sketch has ONE
//  path, not five, and the difference between a four-anchor fix and a
//  one-anchor hint is carried entirely in the sigmas.
//
//  For the well-determined modes the sigmas are the square roots of the
//  diagonal of the inverse information matrix: standard, exact, and
//  automatically enormous in a direction the geometry does not observe.
//
//  For the single-anchor mode it is a projection, and it is worth being
//  explicit that it is an approximation. The measurement is
//
//      z = e . p,   e = unit vector from the anchor toward the prior
//
//  which couples the axes, and per-axis filters cannot represent the
//  cross-covariance that a joint update would create. What is done
//  instead is to hand each axis the same innovation projected onto it,
//  with R divided by that axis's share of the direction:
//
//      z_x = p_x + e_x * nu,      sigma_x = sigma_r / |e_x|
//
//  As |e_x| falls the sigma grows without bound, so an axis the range
//  says nothing about is told nothing — which is the property that makes
//  the approximation safe rather than merely convenient. It is
//  conservative, never over-confident, and it is what lets a single
//  surviving anchor still stop horizontal drift.
//
//  ---------------------------------------------------------------------
//  WHAT THIS FILE DELIBERATELY DOES NOT DO
//  ---------------------------------------------------------------------
//  It does not talk to a DWM1000, it does not know what an anchor is
//  beyond a coordinate and an address, and it does not decide whether a
//  fix should be believed. Those belong to the driver and to the
//  estimator respectively. This is geometry.
// =====================================================================
#ifndef ARIADNA_UWB_H
#define ARIADNA_UWB_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------
//  Sizing
// ---------------------------------------------------------------------
//  Four anchors is the ground layout. The table is eight so that a spare
//  can be laid out and enabled without a firmware change, and so that a
//  future roof-mounted or mast-mounted anchor — which is what would make
//  the vertical channel properly observable — needs no restructuring.
#define ARIADNA_UWB_MAX_ANCHORS  8

/** One surveyed anchor. `addr` is the DWM1000 short address it answers to. */
typedef struct {
    uint16_t addr;        // DW1000 short address, matched against the driver
    uint8_t  enabled;     // 0 = slot ignored entirely
    uint8_t  _pad;
    float    x, y, z;     // pad-ENU metres, same frame as the estimator
} ariadna_uwb_anchor_t;

/**
 * One range observation, indexed in parallel with the anchor table.
 *
 * `age_s` is not decoration. Two-way ranging is a REQUEST/RESPONSE
 * protocol and the anchors answer one at a time, so a set of four ranges
 * is spread over up to a whole round-robin period — and the solver
 * treats them as simultaneous. On a stationary tag that is free. On a
 * vehicle descending at 20 m/s a 300 ms spread is six metres of motion
 * between the first range and the last, and the least-squares fit
 * absorbs it as a POSITION ERROR of about 2.8 m rather than rejecting
 * it. See ariadna_uwb_derotate(), which exists entirely to remove this.
 */
typedef struct {
    float   range_m;      // metres, as reported by the ranging stack
    float   sigma_m;      // 1-sigma of THIS range, per anchor and per sample
    float   age_s;        // seconds between this measurement and the solve epoch
    uint8_t valid;        // 0 = this anchor did not report in time
    uint8_t _pad[3];
} ariadna_uwb_obs_t;

/** How much of a position the geometry actually supported. */
typedef enum {
    ARIADNA_UWB_SOL_NONE = 0,   // nothing usable
    ARIADNA_UWB_SOL_LOS  = 1,   // one range, projected onto the axes
    ARIADNA_UWB_SOL_2D   = 2,   // horizontal only, height held from the prior
    ARIADNA_UWB_SOL_3D   = 3    // full position
} ariadna_uwb_mode_t;

/**
 * The solver's answer.
 *
 * `use_z` is separate from `mode` on purpose: a 3-D solve whose vertical
 * sigma came out too large to be worth anything is downgraded to a 2-D
 * answer, and the caller should not have to re-derive that.
 */
typedef struct {
    float   x, y, z;              // pad-ENU metres
    float   sigma_x, sigma_y, sigma_z;   // 1-sigma, metres
    float   residual_rms;         // metres, post-fit. Large = NLOS or a bad survey
    float   gdop;                 // sqrt(trace of the position covariance) / sigma_r
    uint8_t mode;                 // ariadna_uwb_mode_t
    uint8_t n_used;               // observations that survived screening
    uint8_t mask;                 // bit i set: anchor slot i contributed
    uint8_t iters;                // Gauss-Newton iterations actually run
    uint8_t use_x, use_y, use_z;  // which axes the caller may fuse
    uint8_t _pad;
} ariadna_uwb_fix_t;

/**
 * Solver limits. All of them are here rather than as literals in the
 * body so the sketch, the self test and the host test argue about the
 * same numbers.
 */
typedef struct {
    float   range_min_m;      // below this a range is a hardware artefact
    float   range_max_m;      // above this it is a reflection or a decode error
    float   sigma_floor_m;    // no ranging stack is better than this
    float   sigma_max_m;      // per-axis: above this the axis is not published
    float   resid_max_m;      // post-fit RMS above this rejects the whole fix
    float   z_min_m;          // solutions are clamped above the anchor plane
    float   z_max_m;
    float   los_cos_floor;    // |e_axis| floor for the single-anchor projection
    // --- motion compensation, see ariadna_uwb_derotate() --------------
    float   max_age_s;        // an observation older than this is dropped
    float   vel_sigma_m_s;    // 1-sigma of the velocity used to de-rotate
    float   acc_max_m_s2;     // acceleration the vehicle may have had meanwhile
    uint8_t max_iters;
    uint8_t allow_two;        // permit the ambiguous two-anchor solve
    uint8_t allow_los;        // permit the single-anchor projection
    uint8_t _pad;
} ariadna_uwb_cfg_t;

/**
 * Defaults sized for a 30-50 m anchor square around the pad.
 *
 * sigma_max_m is the one worth understanding. It is not a statement
 * about the radio: it is the point past which a UWB axis is no better
 * than the GPS the filter already has, so publishing it would cost
 * arithmetic and gain nothing. Two metres is roughly a good GPS fix.
 */
static inline void ariadna_uwb_cfg_default(ariadna_uwb_cfg_t *c)
{
    c->range_min_m   = 0.15f;
    c->range_max_m   = 300.0f;
    c->sigma_floor_m = 0.05f;
    c->sigma_max_m   = 2.0f;
    c->resid_max_m   = 1.5f;
    c->z_min_m       = -2.0f;    // the pad frame's zero is the pad, not the anchors
    c->z_max_m       = 400.0f;
    c->los_cos_floor = 0.15f;
    // 250 ms lets an anchor miss one exchange at 10 Hz and still count.
    // Longer would be free if the vehicle were stationary; it is not.
    c->max_age_s     = 0.25f;
    c->vel_sigma_m_s = 0.5f;
    // 4 g. The landing burn is about 3 g, so this bounds what the
    // velocity-only de-rotation cannot account for.
    c->acc_max_m_s2  = 40.0f;
    c->max_iters     = 8;
    c->allow_two     = 1;
    c->allow_los     = 1;
    c->_pad          = 0;
}

/** Anchor table slot answering to `addr`, or -1. Linear: the table is 8 long. */
static inline int ariadna_uwb_find(const ariadna_uwb_anchor_t *tab, uint8_t n,
                                   uint16_t addr)
{
    for (uint8_t i = 0; i < n && i < ARIADNA_UWB_MAX_ANCHORS; i++)
        if (tab[i].enabled && tab[i].addr == addr) return (int)i;
    return -1;
}

// ---------------------------------------------------------------------
//  3x3 symmetric solve and inverse, by Cholesky
//
//  The information matrix N = sum_i w_i h_i h_i^T is symmetric positive
//  SEMI-definite by construction, and semi-definite is exactly the
//  degenerate case this solver is built to survive: three collinear
//  anchors, or a vertical direction that four coplanar anchors cannot
//  see. So the factorisation is the DEGENERACY TEST, not just a linear
//  solve — if a pivot is not comfortably positive the geometry does not
//  support that many unknowns, and the caller drops an axis instead of
//  dividing by something near zero and reporting a confident answer
//  built from noise.
//
//  A small ridge is added to the diagonal before factorising. It costs a
//  fraction of a percent of accuracy on a healthy geometry and it means
//  a marginal one produces a large sigma rather than an infinity.
// ---------------------------------------------------------------------
#define ARIADNA_UWB_RIDGE  1e-6f

/**
 * Cholesky of an n x n symmetric matrix (n <= 3), in place into L.
 * Returns 0 if the matrix is not positive definite to within `floor`.
 */
static inline int ariadna_uwb_chol(const float A[3][3], float L[3][3], int n,
                                   float floor_)
{
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) L[i][j] = 0.0f;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            float s = A[i][j];
            for (int k = 0; k < j; k++) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (!(s > floor_)) return 0;      // catches NaN as well as <= 0
                L[i][i] = sqrtf(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
    }
    return 1;
}

/** Solve L L^T x = b for an already-factorised L. */
static inline void ariadna_uwb_chol_solve(const float L[3][3], const float *b,
                                          float *x, int n)
{
    float y[3];
    for (int i = 0; i < n; i++) {
        float s = b[i];
        for (int k = 0; k < i; k++) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    for (int i = n - 1; i >= 0; i--) {
        float s = y[i];
        for (int k = i + 1; k < n; k++) s -= L[k][i] * x[k];
        x[i] = s / L[i][i];
    }
}

/**
 * Diagonal of (L L^T)^-1, which is the variance of each solved unknown.
 *
 * Computed by solving against each unit vector rather than by forming
 * the whole inverse: it is the same arithmetic, it cannot produce an
 * asymmetric result, and only the diagonal is ever wanted.
 */
static inline void ariadna_uwb_chol_diag_inv(const float L[3][3], float *out, int n)
{
    for (int c = 0; c < n; c++) {
        float e[3] = { 0.0f, 0.0f, 0.0f };
        float col[3] = { 0.0f, 0.0f, 0.0f };
        e[c] = 1.0f;
        ariadna_uwb_chol_solve(L, e, col, n);
        out[c] = col[c];
    }
}

// ---------------------------------------------------------------------
//  The solver
// ---------------------------------------------------------------------

/** Screening result, shared by the solve paths. */
typedef struct {
    uint8_t idx[ARIADNA_UWB_MAX_ANCHORS];
    uint8_t n;
    uint8_t mask;
} ariadna_uwb_sel_t;

/**
 * Keep only observations that could plausibly have come from a radio.
 *
 * Every rejection here is a range that would otherwise enter a least
 * squares as though it were evidence. A DWM1000 that loses a
 * timestamp reports metres of nonsense — sometimes negative, sometimes
 * kilometres — and one such value in an unweighted solve moves the answer
 * by more than the other three can pull it back.
 */
static inline void ariadna_uwb_screen(const ariadna_uwb_anchor_t *tab,
                                      const ariadna_uwb_obs_t *obs, uint8_t n,
                                      const ariadna_uwb_cfg_t *cfg,
                                      ariadna_uwb_sel_t *sel)
{
    sel->n = 0;
    sel->mask = 0;
    if (n > ARIADNA_UWB_MAX_ANCHORS) n = ARIADNA_UWB_MAX_ANCHORS;

    for (uint8_t i = 0; i < n; i++) {
        if (!tab[i].enabled)     continue;
        if (!obs[i].valid)       continue;
        const float r = obs[i].range_m;
        const float s = obs[i].sigma_m;
        if (!(r == r) || !(s == s))                      continue;  // NaN
        if (!(r > cfg->range_min_m && r < cfg->range_max_m)) continue;
        if (!(s > 0.0f) || s > 100.0f)                   continue;
        // Age is screened here rather than at the driver so that the
        // limit is one number in one place, and so the host test can
        // exercise it. A negative age is a clock that ran backwards.
        const float age = obs[i].age_s;
        if (!(age == age) || age < 0.0f || age > cfg->max_age_s) continue;
        if (!(tab[i].x == tab[i].x) || !(tab[i].y == tab[i].y) ||
            !(tab[i].z == tab[i].z))                     continue;  // bad survey
        sel->idx[sel->n++] = i;
        sel->mask |= (uint8_t)(1u << i);
    }
}

/**
 * Bring every range forward to a common epoch, using the vehicle's own
 * velocity.
 *
 * ---------------------------------------------------------------------
 * WHY THIS IS NOT OPTIONAL ON A ROCKET
 * ---------------------------------------------------------------------
 * The four anchors do not answer at the same instant — they answer one
 * at a time, so a full set of ranges is smeared over up to a round-robin
 * period. Least squares then fits a single position to four measurements
 * that were taken from four different places.
 *
 * On a stationary tag that costs nothing, which is why almost every UWB
 * example ignores it. Measured against this solver, on a 40 m anchor
 * square with a 300 ms spread:
 *
 *     descent speed   position error   post-fit residual
 *          0 m/s          0.00 m            0.00 m
 *         10 m/s          1.46 m            0.41 m     <- accepted
 *         20 m/s          2.83 m            0.83 m     <- accepted
 *         50 m/s          6.28 m            2.14 m     <- gated out
 *
 * The 50 m/s row is the harmless one: the residual test catches it and
 * the fix is thrown away. The DANGEROUS rows are 10 and 20 m/s — the
 * speeds this vehicle has in its last ten and five metres — where the
 * error is large, systematically biased along the direction of travel,
 * and small enough in residual to sail through every gate. A confident,
 * wrong altitude in the final seconds is precisely the failure UWB was
 * added to prevent.
 *
 * ---------------------------------------------------------------------
 * THE CORRECTION
 * ---------------------------------------------------------------------
 * A range measured at t = now - age is a distance from where the vehicle
 * WAS. To first order the distance from where it is now is
 *
 *     r(now) = r(t) + e . v * age
 *
 * with e the unit line of sight from the anchor toward the vehicle. That
 * is exact for constant velocity and leaves an error of ~(1/2) a age^2,
 * which is not corrected but IS paid for: the sigma is inflated by that
 * term plus the uncertainty in v itself. So an old range is not merely
 * shifted, it is also trusted less, and the least squares weights it
 * accordingly instead of being told a confident lie.
 *
 * Velocity comes from the navigation filter, which is legitimate rather
 * than circular: it is the DERIVATIVE that is being borrowed, over a
 * fraction of a second, to correct a position measurement. An error in v
 * of 0.5 m/s over 200 ms is 0.1 m — an order below the ranging noise.
 *
 * `vel` may be NULL, in which case ages are only used to inflate the
 * sigmas. That is the honest fallback before the filter has a velocity.
 */
static inline void ariadna_uwb_derotate(const ariadna_uwb_anchor_t *tab,
                                        ariadna_uwb_obs_t *obs, uint8_t n,
                                        const float prior[3],
                                        const float vel[3],
                                        const ariadna_uwb_cfg_t *cfg)
{
    if (!tab || !obs || !cfg || !prior) return;
    if (n > ARIADNA_UWB_MAX_ANCHORS) n = ARIADNA_UWB_MAX_ANCHORS;

    for (uint8_t i = 0; i < n; i++) {
        if (!tab[i].enabled || !obs[i].valid) continue;
        float age = obs[i].age_s;
        if (!(age == age) || age <= 0.0f) continue;      // nothing to do
        if (age > cfg->max_age_s) continue;              // screen() drops it

        if (vel) {
            const float dx = prior[0] - tab[i].x;
            const float dy = prior[1] - tab[i].y;
            const float dz = prior[2] - tab[i].z;
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            if (d < 1e-3f) d = 1e-3f;
            const float proj = (dx * vel[0] + dy * vel[1] + dz * vel[2]) / d;
            const float corrected = obs[i].range_m + proj * age;
            // Refuse a correction that is not finite or that would move
            // the range outside anything a radio could have produced.
            if (corrected == corrected &&
                corrected > cfg->range_min_m && corrected < cfg->range_max_m)
                obs[i].range_m = corrected;
        }

        // What the correction could not remove, added in quadrature:
        // the residual acceleration term and the velocity uncertainty.
        const float acc_term = 0.5f * cfg->acc_max_m_s2 * age * age;
        const float vel_term = cfg->vel_sigma_m_s * age;
        const float s0 = obs[i].sigma_m;
        obs[i].sigma_m = sqrtf(s0 * s0 + acc_term * acc_term + vel_term * vel_term);
    }
}

/**
 * Weighted Gauss-Newton over `ndim` unknowns (2 = x,y with z held).
 *
 * The measurement model is the full 3-D range even in 2-D mode, so an
 * anchor that is not exactly at the height it was surveyed at still
 * contributes correctly — the height is HELD, not ignored.
 *
 * Returns 0 if the normal equations were singular at any iteration,
 * which is the caller's signal to try fewer unknowns.
 */
static inline int ariadna_uwb_gauss_newton(const ariadna_uwb_anchor_t *tab,
                                           const ariadna_uwb_obs_t *obs,
                                           const ariadna_uwb_sel_t *sel,
                                           const ariadna_uwb_cfg_t *cfg,
                                           int ndim, float p[3],
                                           float var_out[3], float *resid_rms,
                                           uint8_t *iters_out)
{
    float L[3][3];
    float var[3] = { 0.0f, 0.0f, 0.0f };
    uint8_t it = 0;
    float rms = 0.0f;
    int ok = 0;

    for (it = 0; it < cfg->max_iters; it++) {
        float N[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
        float b[3]    = { 0.0f, 0.0f, 0.0f };
        float ss      = 0.0f;

        for (uint8_t k = 0; k < sel->n; k++) {
            const uint8_t i = sel->idx[k];
            const float dx = p[0] - tab[i].x;
            const float dy = p[1] - tab[i].y;
            const float dz = p[2] - tab[i].z;
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            // Sitting exactly on an anchor makes the Jacobian undefined.
            // It cannot happen in flight and it must not divide by zero.
            if (d < 1e-3f) d = 1e-3f;

            float sig = obs[i].sigma_m;
            if (sig < cfg->sigma_floor_m) sig = cfg->sigma_floor_m;
            const float w = 1.0f / (sig * sig);

            const float h[3] = { dx / d, dy / d, dz / d };
            const float res  = obs[i].range_m - d;
            ss += res * res;

            for (int a = 0; a < ndim; a++) {
                b[a] += w * h[a] * res;
                for (int c = 0; c <= a; c++) N[a][c] += w * h[a] * h[c];
            }
        }
        for (int a = 0; a < ndim; a++)
            for (int c = a + 1; c < ndim; c++) N[a][c] = N[c][a];
        for (int a = 0; a < ndim; a++) N[a][a] += ARIADNA_UWB_RIDGE;

        rms = sel->n ? sqrtf(ss / (float)sel->n) : 0.0f;

        if (!ariadna_uwb_chol(N, L, ndim, 1e-9f)) { ok = 0; break; }

        float d[3] = { 0.0f, 0.0f, 0.0f };
        ariadna_uwb_chol_solve(L, b, d, ndim);

        // A Gauss-Newton step that is not finite means the geometry
        // collapsed between the factorisation and here. Bail rather than
        // write a NaN into the state.
        for (int a = 0; a < ndim; a++)
            if (!(d[a] == d[a])) { return 0; }

        // Step limiting. Far from the solution the linearisation is poor
        // and an unlimited step can throw the iterate kilometres away,
        // from where it may never come back. Ten metres per iteration
        // converges from any plausible prior within max_iters.
        const float step2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        if (step2 > 100.0f) {
            const float sc = 10.0f / sqrtf(step2);
            for (int a = 0; a < ndim; a++) d[a] *= sc;
        }

        for (int a = 0; a < ndim; a++) p[a] += d[a];
        if (p[2] < cfg->z_min_m) p[2] = cfg->z_min_m;
        if (p[2] > cfg->z_max_m) p[2] = cfg->z_max_m;

        ariadna_uwb_chol_diag_inv(L, var, ndim);
        ok = 1;

        if (step2 < 1e-6f) { it++; break; }      // converged
    }

    if (!ok) return 0;
    for (int a = 0; a < 3; a++) var_out[a] = (a < ndim) ? var[a] : 0.0f;
    *resid_rms = rms;
    *iters_out = it;
    return 1;
}

/**
 * Single-anchor line-of-sight projection. See the header comment for why
 * this is expressed as a position with per-axis sigmas.
 */
static inline void ariadna_uwb_solve_los(const ariadna_uwb_anchor_t *tab,
                                         const ariadna_uwb_obs_t *obs,
                                         const ariadna_uwb_sel_t *sel,
                                         const ariadna_uwb_cfg_t *cfg,
                                         const float prior[3],
                                         ariadna_uwb_fix_t *out)
{
    const uint8_t i = sel->idx[0];
    const float dx = prior[0] - tab[i].x;
    const float dy = prior[1] - tab[i].y;
    const float dz = prior[2] - tab[i].z;
    float d = sqrtf(dx * dx + dy * dy + dz * dz);
    if (d < 1e-3f) { out->mode = ARIADNA_UWB_SOL_NONE; return; }

    const float e[3]  = { dx / d, dy / d, dz / d };
    const float nu    = obs[i].range_m - d;          // innovation, metres
    float sig = obs[i].sigma_m;
    if (sig < cfg->sigma_floor_m) sig = cfg->sigma_floor_m;

    out->mode   = ARIADNA_UWB_SOL_LOS;
    out->n_used = 1;
    out->mask   = (uint8_t)(1u << i);
    out->iters  = 1;
    out->residual_rms = fabsf(nu);

    float axis[3];
    for (int a = 0; a < 3; a++) {
        const float c = fabsf(e[a]);
        axis[a] = prior[a] + e[a] * nu;
        // sigma = sigma_r / |e_a|, floored so an axis the LOS barely
        // touches gets a sigma the estimator will simply ignore rather
        // than a division by zero.
        const float cc = (c < cfg->los_cos_floor) ? cfg->los_cos_floor : c;
        const float s  = sig / cc;
        if (a == 0) { out->x = axis[0]; out->sigma_x = s; }
        if (a == 1) { out->y = axis[1]; out->sigma_y = s; }
        if (a == 2) { out->z = axis[2]; out->sigma_z = s; }
    }
    // An axis whose direction cosine hit the floor was not measured; say
    // so explicitly instead of relying on the sigma gate downstream.
    out->use_x = (fabsf(e[0]) >= cfg->los_cos_floor &&
                  out->sigma_x <= cfg->sigma_max_m) ? 1 : 0;
    out->use_y = (fabsf(e[1]) >= cfg->los_cos_floor &&
                  out->sigma_y <= cfg->sigma_max_m) ? 1 : 0;
    out->use_z = 0;   // never: one range cannot separate height from range
    out->gdop  = 1.0f / ((fabsf(e[0]) < cfg->los_cos_floor)
                             ? cfg->los_cos_floor : fabsf(e[0]));
}

/**
 * Solve for the tag position.
 *
 * `prior` is the estimator's current position, used as the linearisation
 * point, as the held height in 2-D, and as the tie-breaker for the
 * two-anchor ambiguity. It does NOT bias a well-determined solve — the
 * Gauss-Newton converges to the same answer from anywhere.
 *
 * Every exit writes a fully initialised `out`, including the failure
 * exits. A caller that ignores the return value and reads the fields
 * still reads zeros with mode NONE and use_* clear, never stale memory.
 */
static inline int ariadna_uwb_solve(const ariadna_uwb_anchor_t *tab,
                                    const ariadna_uwb_obs_t *obs, uint8_t n,
                                    const float prior[3],
                                    const ariadna_uwb_cfg_t *cfg,
                                    ariadna_uwb_fix_t *out)
{
    // Fully clear FIRST. Everything below may return early.
    out->x = out->y = out->z = 0.0f;
    out->sigma_x = out->sigma_y = out->sigma_z = 0.0f;
    out->residual_rms = 0.0f;
    out->gdop   = 0.0f;
    out->mode   = ARIADNA_UWB_SOL_NONE;
    out->n_used = 0;
    out->mask   = 0;
    out->iters  = 0;
    out->use_x = out->use_y = out->use_z = 0;
    out->_pad  = 0;

    if (!tab || !obs || !cfg || !prior) return 0;

    float p0[3] = { prior[0], prior[1], prior[2] };
    for (int a = 0; a < 3; a++) if (!(p0[a] == p0[a])) p0[a] = 0.0f;
    if (p0[2] < cfg->z_min_m) p0[2] = cfg->z_min_m;
    if (p0[2] > cfg->z_max_m) p0[2] = cfg->z_max_m;

    ariadna_uwb_sel_t sel;
    ariadna_uwb_screen(tab, obs, n, cfg, &sel);
    if (sel.n == 0) return 0;

    if (sel.n == 1) {
        if (!cfg->allow_los) return 0;
        ariadna_uwb_solve_los(tab, obs, &sel, cfg, p0, out);
        return out->mode != ARIADNA_UWB_SOL_NONE;
    }
    if (sel.n == 2 && !cfg->allow_two) return 0;

    // --- try the widest solve the anchor count could support ------------
    //
    // Three unknowns need three ranges, but "needs" is not "supports":
    // four coplanar anchors and a tag a metre off the ground give a
    // vertical direction cosine of a few percent, so the vertical
    // variance that comes back is hundreds of square metres. That is not
    // a failure of the solve — it is the solve correctly reporting that
    // the ground cannot see height. The 2-D retry below is what turns
    // that report into a usable horizontal fix.
    float var[3] = { 0.0f, 0.0f, 0.0f };
    float rms    = 0.0f;
    uint8_t iters = 0;
    int ndim = (sel.n >= 3) ? 3 : 2;
    float p[3] = { p0[0], p0[1], p0[2] };

    if (!ariadna_uwb_gauss_newton(tab, obs, &sel, cfg, ndim, p, var, &rms, &iters)) {
        if (ndim == 3) {
            ndim = 2;
            p[0] = p0[0]; p[1] = p0[1]; p[2] = p0[2];
            if (!ariadna_uwb_gauss_newton(tab, obs, &sel, cfg, 2, p, var,
                                          &rms, &iters))
                return 0;
        } else {
            return 0;
        }
    }

    // The vertical came back, but is it worth anything? If not, redo the
    // horizontal with the height held rather than let a meaningless z
    // drag x and y around with it.
    if (ndim == 3) {
        const float sz = (var[2] > 0.0f) ? sqrtf(var[2]) : 1e9f;
        if (!(sz <= cfg->sigma_max_m)) {
            ndim = 2;
            p[0] = p0[0]; p[1] = p0[1]; p[2] = p0[2];
            if (!ariadna_uwb_gauss_newton(tab, obs, &sel, cfg, 2, p, var,
                                          &rms, &iters))
                return 0;
        }
    }

    for (int a = 0; a < 3; a++)
        if (!(p[a] == p[a])) return 0;             // NaN escaped: publish nothing

    out->x = p[0]; out->y = p[1]; out->z = p[2];
    out->sigma_x = (var[0] > 0.0f) ? sqrtf(var[0]) : 1e9f;
    out->sigma_y = (var[1] > 0.0f) ? sqrtf(var[1]) : 1e9f;
    out->sigma_z = (ndim == 3 && var[2] > 0.0f) ? sqrtf(var[2]) : 1e9f;
    out->residual_rms = rms;
    out->mode   = (ndim == 3) ? ARIADNA_UWB_SOL_3D : ARIADNA_UWB_SOL_2D;
    out->n_used = sel.n;
    out->mask   = sel.mask;
    out->iters  = iters;
    out->gdop   = sqrtf(out->sigma_x * out->sigma_x +
                        out->sigma_y * out->sigma_y);

    // A post-fit residual much larger than the quoted sigmas means the
    // ranges do not agree with EACH OTHER: a reflection, a wrong survey,
    // or an anchor answering to a duplicated address. None of those are
    // improved by fusing them, and all of them look like a confident
    // position to a filter.
    if (rms > cfg->resid_max_m) {
        out->use_x = out->use_y = out->use_z = 0;
        return 0;
    }

    out->use_x = (out->sigma_x <= cfg->sigma_max_m) ? 1 : 0;
    out->use_y = (out->sigma_y <= cfg->sigma_max_m) ? 1 : 0;
    out->use_z = (ndim == 3 && out->sigma_z <= cfg->sigma_max_m) ? 1 : 0;

    return (out->use_x || out->use_y || out->use_z) ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif // ARIADNA_UWB_H
