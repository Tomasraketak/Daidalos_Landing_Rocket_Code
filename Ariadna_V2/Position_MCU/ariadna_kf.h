// =====================================================================
//  Ariadna V2 — one-axis navigation Kalman filter
//
//  The arithmetic behind the Position MCU's estimator, kept OUT of the
//  sketch so it can be compiled and checked on a workstation. Same
//  discipline guidance.cpp already follows, and for the same reason:
//  anything inside a .ino can only be tested by flashing a board, and
//  firmware that cannot be tested on a desk ends up being tested on the
//  pad.
//
//  ../../test/est_test.cpp exercises every function here.
//
//  No Arduino dependency, and no dependency on the link header either —
//  this is pure arithmetic.
//  Each sketch carries its own copy of this header in its folder (the
//  Arduino builder compiles a sketch from a copy of its folder, so a
//  "../common/" include does not resolve). This file is the original;
//  `cd ../../test && make sync` checks the copies, `make sync-fix`
//  refreshes them.
//
//  ---------------------------------------------------------------------
//  THE FILTER
//  ---------------------------------------------------------------------
//  One axis of a navigation solution:
//
//      x = [ position, velocity, accelerometer bias ]
//
//  Continuous model, with u the measured world-frame acceleration:
//
//      p' = v
//      v' = (u - b) + w_a       w_a ~ acceleration noise, density acc_psd
//      b' = w_b                 w_b ~ bias random walk,   density bias_rw
//
//  Three of these run per solution — one per axis. That is not an
//  approximation. Because attitude is supplied externally by the fusion
//  library, the 9-state system matrix is exactly block diagonal per axis,
//  the process noise is diagonal, and every measurement touches exactly
//  one axis. A block-diagonal covariance therefore STAYS block diagonal
//  forever, so three 3x3 filters are mathematically identical to one 9x9
//  filter at roughly a ninth of the arithmetic. An exact factorisation.
//
//  The consequence worth stating: every measurement is scalar and
//  observes one state directly, so H is a unit row and the update
//  collapses to a handful of multiply-adds. THERE IS NO MATRIX INVERSION
//  ANYWHERE IN THIS HEADER.
//
//  ---------------------------------------------------------------------
//  WHAT THE BIAS STATE ABSORBS
//  ---------------------------------------------------------------------
//  More than its name suggests, and this is why the third state earns its
//  place. It soaks up anything that looks like a constant acceleration
//  offset in the navigation frame: true accelerometer bias, an error in
//  the local gravity constant, and — dominating both — a small residual
//  TILT error, which projects gravity onto the horizontal axes. One
//  degree of tilt error injects 0.17 m/s^2 of phantom horizontal
//  acceleration, far more than the ICM42688P's own bias.
//
//  Vertically the argument is about time-to-go: 0.05 m/s^2 of bias
//  integrates to 2.5 m/s of phantom velocity over a 10 s descent, and the
//  guidance law divides by vertical speed.
//
//  Single precision throughout. The RP2350's Cortex-M33 has a
//  single-precision FPU and no double-precision unit, so a promoted
//  expression is emulated in software. The host build compiles this with
//  -Wdouble-promotion for exactly that reason.
// =====================================================================
#ifndef ARIADNA_KF_H
#define ARIADNA_KF_H

#include <stdint.h>
#include <math.h>

/** One navigation axis: x = [ position, velocity, accelerometer bias ]. */
typedef struct {
    float x[3];
    float P[3][3];
} ariadna_kf_t;

/**
 * Initialise a state and its covariance.
 *
 * The sigmas are the honest initial uncertainty, and they matter more
 * than they look. Starting a position state at a small sigma when the
 * position is not actually known makes the first real measurement look
 * like an outlier, and the innovation gate then rejects it — a filter
 * that never converges because it was told it had already converged.
 */
static inline void ariadna_kf_reset(ariadna_kf_t *k, float p0, float v0,
                                    float sig_p, float sig_v, float sig_b)
{
    k->x[0] = p0; k->x[1] = v0; k->x[2] = 0.0f;
    for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 3; j++) k->P[i][j] = 0.0f;
    k->P[0][0] = sig_p * sig_p;
    k->P[1][1] = sig_v * sig_v;
    k->P[2][2] = sig_b * sig_b;
}

/**
 * Predict one step. F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]].
 *
 * P <- F P F^T + Q is written out longhand rather than looped: the zeros
 * in F make a generic 3x3 product mostly multiplications by zero, and
 * this runs three times per estimator step at 240 Hz.
 *
 * Process noise uses the continuous white-noise-acceleration
 * discretisation, so the position/velocity block is correlated the way
 * integrating the same noise twice actually correlates it. Adding
 * independent noise to the diagonal instead — the common shortcut — makes
 * the filter believe position and velocity errors are unrelated, which
 * they emphatically are not.
 */
static inline void ariadna_kf_predict(ariadna_kf_t *k, float u, float dt,
                                      float acc_psd, float bias_rw)
{
    const float h = 0.5f * dt * dt;
    const float a = u - k->x[2];

    k->x[0] += k->x[1] * dt + a * h;
    k->x[1] += a * dt;
    // k->x[2] is constant across a step; the random walk enters through Q.

    // A = F P.  Row i of F dotted into each column of P.
    float A[3][3];
    for (uint8_t j = 0; j < 3; j++) {
        A[0][j] = k->P[0][j] + dt * k->P[1][j] - h * k->P[2][j];
        A[1][j] = k->P[1][j] - dt * k->P[2][j];
        A[2][j] = k->P[2][j];
    }

    // P = A F^T.  (A F^T)[i][j] = sum_k A[i][k] F[j][k], so column j of
    // the result is row j of F dotted into row i of A.
    //
    // GET THIS BACKWARDS AND THE FILTER STILL RUNS. The reference sketch
    // this filter was adapted from applies the F pattern down the columns
    // instead of across them, which computes F P F rather than F P F^T.
    // F is not symmetric, so the result is not either: after one step
    // from P = diag(1, 1, 0.01) at dt = 1/240 it produces P[0][1] = 2 dt
    // against P[1][0] = 0, where both should be dt.
    //
    // The reason that survives casual testing is that the correction step
    // re-symmetrises P, so the asymmetry is repeatedly averaged away
    // instead of accumulating into an obvious divergence. What is left is
    // a covariance that is quietly the wrong size — the position/velocity
    // correlation is roughly doubled and the bias correlations are off by
    // orders of magnitude — which does not crash anything. It just means
    // every Kalman gain, and every innovation gate that is scaled by S,
    // is computed from a number that is not the filter's actual
    // uncertainty. est_test.cpp catches it on the first prediction.
    for (uint8_t i = 0; i < 3; i++) {
        const float a0 = A[i][0], a1 = A[i][1], a2 = A[i][2];
        k->P[i][0] = a0 + dt * a1 - h * a2;
        k->P[i][1] = a1 - dt * a2;
        k->P[i][2] = a2;
    }

    const float qa  = acc_psd * acc_psd;
    const float qb  = bias_rw * bias_rw;
    const float dt2 = dt * dt;
    k->P[0][0] += qa * dt2 * dt * (1.0f / 3.0f);
    k->P[0][1] += qa * dt2 * 0.5f;
    k->P[1][0] += qa * dt2 * 0.5f;
    k->P[1][1] += qa * dt;
    k->P[2][2] += qb * dt;
}

/**
 * Scalar update of one state. Returns 0 if the innovation gate rejected
 * the measurement.
 *
 * gate_sigma <= 0 disables the gate. That is what a zero-velocity update
 * wants: a ZUPT exists precisely to rescue a filter that has diverged,
 * and a diverged filter rejects everything it is shown.
 *
 * P is re-symmetrised afterwards. In single precision the short-form
 * covariance update accumulates asymmetry over millions of steps, and an
 * asymmetric P eventually goes indefinite and takes the filter with it —
 * a failure whose only symptom is a solution that stops making sense.
 * Three averages prevent it permanently.
 */
static inline int ariadna_kf_correct(ariadna_kf_t *k, uint8_t idx, float z,
                                     float R, float gate_sigma)
{
    const float S = k->P[idx][idx] + R;
    if (!(S > 0.0f)) return 0;
    const float y = z - k->x[idx];

    if (gate_sigma > 0.0f && (y * y) > (gate_sigma * gate_sigma * S)) return 0;

    const float invS = 1.0f / S;
    const float K0 = k->P[0][idx] * invS;
    const float K1 = k->P[1][idx] * invS;
    const float K2 = k->P[2][idx] * invS;

    k->x[0] += K0 * y;
    k->x[1] += K1 * y;
    k->x[2] += K2 * y;

    const float r0 = k->P[idx][0], r1 = k->P[idx][1], r2 = k->P[idx][2];
    k->P[0][0] -= K0 * r0; k->P[0][1] -= K0 * r1; k->P[0][2] -= K0 * r2;
    k->P[1][0] -= K1 * r0; k->P[1][1] -= K1 * r1; k->P[1][2] -= K1 * r2;
    k->P[2][0] -= K2 * r0; k->P[2][1] -= K2 * r1; k->P[2][2] -= K2 * r2;

    k->P[0][1] = k->P[1][0] = 0.5f * (k->P[0][1] + k->P[1][0]);
    k->P[0][2] = k->P[2][0] = 0.5f * (k->P[0][2] + k->P[2][0]);
    k->P[1][2] = k->P[2][1] = 0.5f * (k->P[1][2] + k->P[2][1]);
    return 1;
}

/** 1-sigma of one state, straight off the covariance diagonal. */
static inline float ariadna_kf_sigma(const ariadna_kf_t *k, uint8_t idx)
{
    return k->P[idx][idx] > 0.0f ? sqrtf(k->P[idx][idx]) : 0.0f;
}

/**
 * Clamp the bias state.
 *
 * A bias larger than the limit is not a bias, it is a broken
 * accelerometer, and letting the filter absorb it would hide the fault
 * instead of reporting it.
 */
static inline void ariadna_kf_clamp_bias(ariadna_kf_t *k, float limit)
{
    if (k->x[2] >  limit) k->x[2] =  limit;
    if (k->x[2] < -limit) k->x[2] = -limit;
}

// =====================================================================
//  BAROMETER TRIM OBSERVER
//
//  A slow, clamped, first-order estimate of a standing offset between
//  the barometer and an independent height measurement — in this vehicle,
//  UWB. It lives here rather than in the sketch for the reason everything
//  else here does: it decides the altitude the vehicle lands on, and code
//  that decides that must be runnable on a workstation.
//
//  ---------------------------------------------------------------------
//  WHY AN OBSERVER RATHER THAN JUST FUSING BOTH HEIGHTS
//  ---------------------------------------------------------------------
//  Because fusing a biased measurement with an unbiased one AVERAGES the
//  bias; it does not remove it. Weighted by their variances and their
//  rates, a barometer at 50 Hz and sigma 0.5 m against a UWB height at
//  25 Hz and sigma 0.7 m leaves most of a one-metre offset in place.
//
//  Removing it requires the offset to be estimated as its own quantity
//  and subtracted. That is all this is. Measured end to end on a
//  synthetic 150 m descent, a +1.0 m barometer bias comes out as +1.00 m
//  of altitude error without it and +0.07 m with it.
//
//  ---------------------------------------------------------------------
//  WHAT MAKES IT SAFE
//  ---------------------------------------------------------------------
//  Three properties, and the caller supplies the gating that decides
//  WHEN a discrepancy is meaningful at all:
//
//    * the time constant is seconds, so no single measurement can move
//      the altitude reference. The per-step gain is additionally clamped,
//      so even a long stall followed by a sudden resume cannot produce a
//      jump;
//    * the estimate is CLAMPED. A metre-class offset is a barometer. A
//      five-metre one is a mis-surveyed reference, and absorbing that
//      would silently move the vehicle's idea of the ground. Clamping
//      turns such an error into a value pinned at its limit — visible on
//      the console and in telemetry — instead of a quiet accommodation;
//    * it holds its value when updates stop. That is the point rather
//      than a side effect: the independent sensor is typically LEAST
//      available in the last metres, and a calibration learned on the way
//      down is exactly what should still be in force at touchdown.
// =====================================================================

typedef struct {
    float    trim;        // m, positive = the barometer reads HIGH
    float    tau_s;
    float    limit_m;
    uint32_t last_ms;     // 0 = no update yet
} ariadna_baro_trim_t;

static inline void ariadna_baro_trim_reset(ariadna_baro_trim_t *t,
                                           float tau_s, float limit_m)
{
    t->trim    = 0.0f;
    t->tau_s   = (tau_s > 0.01f) ? tau_s : 0.01f;
    t->limit_m = (limit_m > 0.0f) ? limit_m : 0.0f;
    t->last_ms = 0;
}

/**
 * Fold in one observation of the offset.
 *
 * `discrepancy_m` is (barometric height - independent height): positive
 * when the barometer reads high. Returns 1 if the estimate moved.
 *
 * The first call after a reset only starts the clock — there is no dt to
 * form a gain from, and inventing one is how a first sample ends up
 * weighted differently from every sample after it.
 */
static inline int ariadna_baro_trim_update(ariadna_baro_trim_t *t,
                                           float discrepancy_m, uint32_t now_ms)
{
    if (!(discrepancy_m == discrepancy_m)) return 0;      // NaN
    if (now_ms == 0) return 0;

    const uint32_t last = t->last_ms;
    t->last_ms = now_ms;
    if (last == 0) return 0;

    const float dt = (float)(uint32_t)(now_ms - last) * 1e-3f;
    // A gap longer than a second means the observer was not running.
    // Restart the clock rather than apply a gain computed from a dt the
    // measurement was never continuous over.
    if (!(dt > 0.0f) || dt > 1.0f) return 0;

    float alpha = dt / (t->tau_s + dt);
    if (alpha > 0.2f) alpha = 0.2f;      // never a step, whatever dt says

    t->trim += alpha * (discrepancy_m - t->trim);
    if (t->trim >  t->limit_m) t->trim =  t->limit_m;
    if (t->trim < -t->limit_m) t->trim = -t->limit_m;
    return 1;
}

/** The current estimate, metres, positive = the barometer reads high. */
static inline float ariadna_baro_trim_value(const ariadna_baro_trim_t *t)
{
    return t->trim;
}

/** True if the estimate is sitting on its clamp — i.e. not a barometer. */
static inline int ariadna_baro_trim_saturated(const ariadna_baro_trim_t *t)
{
    const float m = t->limit_m * 0.999f;
    return (t->trim >= m || t->trim <= -m) ? 1 : 0;
}

#endif // ARIADNA_KF_H
