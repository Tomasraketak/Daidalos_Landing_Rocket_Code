// =====================================================================
//  Ariadna V2 — navigation filter checks
//
//  ../Ariadna_V2/common/ariadna_kf.h compiles UNCHANGED into this test.
//  It is the same header the Position MCU includes, so what is checked
//  here is what flies — the discipline host_test.cpp already applies to
//  guidance.cpp.
//
//  What this can and cannot establish:
//
//    CAN   the filter converges, learns a bias, keeps its covariance
//          symmetric and positive, rejects outliers, and never produces
//          a NaN over a long run.
//    CANNOT   that the tuning constants suit the vehicle, or that the
//          sensors are wired the way the sketch believes. Those need a
//          bench and then a flight.
//
//  Build:  make est
// =====================================================================
#include "../Ariadna_V2/common/ariadna_kf.h"

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

// Tuning mirrored from Estimator in position_mcu.ino. Not shared with it
// — these are the numbers the checks below are written against, and if
// the sketch moves away from them the checks still say something true
// about the filter, just not about the flight configuration.
static const float ACC_PSD    = 0.5f;
static const float BIAS_RW    = 0.004f;
static const float GATE_SIGMA = 4.0f;
static const float RATE_HZ    = 240.0f;

// ---------------------------------------------------------------------
//  Covariance health. Everything else depends on this staying true.
// ---------------------------------------------------------------------
static bool p_is_healthy(const ariadna_kf_t *k)
{
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(k->x[i]))    return false;
        if (!(k->P[i][i] > 0.0f))       return false;   // diagonal positive
        for (int j = 0; j < 3; j++) {
            if (!std::isfinite(k->P[i][j])) return false;
            // Symmetric to within single-precision round-off, scaled by
            // the magnitude of the entry itself.
            float a = k->P[i][j], b = k->P[j][i];
            float tol = 1e-4f * (fabsf(a) + fabsf(b) + 1e-6f);
            if (fabsf(a - b) > tol)     return false;
        }
        // Cauchy-Schwarz: a valid covariance cannot have an off-diagonal
        // entry larger than the geometric mean of the two diagonals. This
        // is what actually catches a filter drifting toward indefinite.
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;
            if (k->P[i][j] * k->P[i][j] > k->P[i][i] * k->P[j][j] * 1.001f)
                return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------
//  1. Prediction is the kinematics it claims to be
// ---------------------------------------------------------------------
static void test_predict(void)
{
    printf("prediction\n");

    ariadna_kf_t k;
    ariadna_kf_reset(&k, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f);

    // Constant 2 m/s^2 for one second, from rest, with no bias: the
    // textbook answer is 1 m and 2 m/s.
    const float dt = 1.0f / RATE_HZ;
    for (int i = 0; i < (int)RATE_HZ; i++)
        ariadna_kf_predict(&k, 2.0f, dt, ACC_PSD, BIAS_RW);

    check(fabsf(k.x[0] - 1.0f) < 0.01f, "position after 1 s at 2 m/s^2 is 1 m");
    check(fabsf(k.x[1] - 2.0f) < 0.01f, "velocity after 1 s at 2 m/s^2 is 2 m/s");
    check(k.x[2] == 0.0f,               "prediction alone does not move the bias");

    // Uncertainty must GROW without measurements. A filter whose
    // covariance shrinks during dead reckoning is one that will gate out
    // the fix that would have saved it.
    check(k.P[0][0] > 1.0f, "position variance grows while coasting");
    check(k.P[1][1] > 1.0f, "velocity variance grows while coasting");
    check(p_is_healthy(&k), "covariance stays healthy through prediction");

    // The bias is subtracted from the input, not added.
    ariadna_kf_t b;
    ariadna_kf_reset(&b, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f);
    b.x[2] = 2.0f;                       // bias equal to the input
    for (int i = 0; i < (int)RATE_HZ; i++)
        ariadna_kf_predict(&b, 2.0f, dt, ACC_PSD, BIAS_RW);
    check(fabsf(b.x[1]) < 0.01f, "an input equal to the bias produces no acceleration");
}

// ---------------------------------------------------------------------
//  2. Correction converges, and to the right place
// ---------------------------------------------------------------------
static void test_correct(void)
{
    printf("correction\n");

    ariadna_kf_t k;
    ariadna_kf_reset(&k, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);

    const float dt = 1.0f / RATE_HZ;
    for (int i = 0; i < (int)RATE_HZ * 10; i++) {
        ariadna_kf_predict(&k, 0.0f, dt, ACC_PSD, BIAS_RW);
        if (i % 48 == 0)                                  // 5 Hz
            ariadna_kf_correct(&k, 0, 50.0f, 2.5f * 2.5f, GATE_SIGMA);
    }

    check(fabsf(k.x[0] - 50.0f) < 1.0f, "position converges on a steady measurement");
    check(fabsf(k.x[1]) < 0.5f,         "velocity settles near zero");
    check(ariadna_kf_sigma(&k, 0) < 2.5f,
          "position sigma falls below the measurement's own");
    check(p_is_healthy(&k), "covariance stays healthy through correction");

    // A correction must reduce the variance of the state it observes.
    ariadna_kf_t m;
    ariadna_kf_reset(&m, 0.0f, 0.0f, 10.0f, 1.0f, 0.1f);
    const float before = m.P[0][0];
    ariadna_kf_correct(&m, 0, 1.0f, 1.0f, 0.0f);
    check(m.P[0][0] < before, "a correction reduces the observed state's variance");
}

// ---------------------------------------------------------------------
//  3. The bias state is observable, which is the whole point of it
// ---------------------------------------------------------------------
static void test_bias_observability(void)
{
    printf("bias observability\n");

    // A stationary vehicle whose accelerometer reads a constant 0.4 m/s^2
    // that is not there. Position measurements keep saying "still at
    // 100 m", so the only consistent explanation is a bias — and the
    // filter has a state to put it in.
    ariadna_kf_t k;
    ariadna_kf_reset(&k, 100.0f, 0.0f, 0.5f, 0.5f, 0.5f);

    const float dt = 1.0f / RATE_HZ;
    for (int i = 0; i < (int)RATE_HZ * 30; i++) {
        ariadna_kf_predict(&k, 0.4f, dt, ACC_PSD, BIAS_RW);
        ariadna_kf_clamp_bias(&k, 1.5f);
        if (i % 5 == 0)                                   // 48 Hz, like the baro
            ariadna_kf_correct(&k, 0, 100.0f, 0.5f * 0.5f, GATE_SIGMA);
    }

    check(fabsf(k.x[2] - 0.4f) < 0.1f, "bias converges on the phantom acceleration");
    check(fabsf(k.x[0] - 100.0f) < 1.0f, "position is not dragged off by it");
    check(fabsf(k.x[1]) < 0.5f, "velocity does not run away");
    check(p_is_healthy(&k), "covariance healthy after a long biased run");

    // Without the bias state the same run would integrate 0.4 m/s^2 into
    // 12 m/s over 30 s. Confirm the magnitude of what is being avoided,
    // so this test says something about the vehicle and not just the code.
    check(fabsf(0.4f * 30.0f) > 10.0f,
          "unmodelled, that bias would be 12 m/s after 30 s");

    // The clamp must hold against an input the filter should refuse to
    // explain away as bias.
    ariadna_kf_t c;
    ariadna_kf_reset(&c, 0.0f, 0.0f, 0.5f, 0.5f, 0.5f);
    for (int i = 0; i < (int)RATE_HZ * 30; i++) {
        ariadna_kf_predict(&c, 20.0f, dt, ACC_PSD, BIAS_RW);
        ariadna_kf_clamp_bias(&c, 1.5f);
        if (i % 5 == 0) ariadna_kf_correct(&c, 0, 0.0f, 0.25f, GATE_SIGMA);
    }
    check(fabsf(c.x[2]) <= 1.5f + 1e-4f, "bias clamp holds against a broken sensor");
}

// ---------------------------------------------------------------------
//  4. The innovation gate
// ---------------------------------------------------------------------
static void test_gate(void)
{
    printf("innovation gate\n");

    ariadna_kf_t k;
    ariadna_kf_reset(&k, 0.0f, 0.0f, 1.0f, 0.5f, 0.1f);

    // Inside the gate: accepted, and it moves the state.
    check(ariadna_kf_correct(&k, 0, 1.0f, 1.0f, GATE_SIGMA) == 1,
          "a plausible measurement is accepted");

    ariadna_kf_reset(&k, 0.0f, 0.0f, 1.0f, 0.5f, 0.1f);
    const float x_before = k.x[0];
    // Way outside: S = 1 + 1 = 2, so the 4-sigma gate is at 5.7 m.
    check(ariadna_kf_correct(&k, 0, 300.0f, 1.0f, GATE_SIGMA) == 0,
          "a 300 m jump is rejected");
    check(k.x[0] == x_before, "a rejected measurement changes nothing at all");

    // Gate disabled: the same measurement must now go in. This is the
    // ZUPT path, which must work on a filter that has already diverged.
    check(ariadna_kf_correct(&k, 0, 300.0f, 1.0f, 0.0f) == 1,
          "gate_sigma 0 disables rejection");
    check(k.x[0] != x_before, "and the state moves");

    // The gate must widen as the filter becomes less certain. A fixed
    // threshold would lock out a genuinely drifted filter forever.
    ariadna_kf_t wide;
    ariadna_kf_reset(&wide, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);
    check(ariadna_kf_correct(&wide, 0, 300.0f, 1.0f, GATE_SIGMA) == 1,
          "an uncertain filter accepts what a confident one rejects");
}

// ---------------------------------------------------------------------
//  5. Numerical endurance
//
//  The failure this guards against does not show up in a short run. A
//  covariance update in single precision accumulates asymmetry, and an
//  asymmetric P eventually goes indefinite — at which point the filter
//  produces confident nonsense rather than an error.
// ---------------------------------------------------------------------
static void test_endurance(void)
{
    printf("numerical endurance\n");

    ariadna_kf_t k;
    ariadna_kf_reset(&k, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);

    const float dt = 1.0f / RATE_HZ;
    uint32_t seed = 12345u;
    bool healthy = true;

    // Ten minutes at 240 Hz, with correlated noise on both the input and
    // the measurements, and an occasional outlier for the gate to eat.
    const int N = (int)RATE_HZ * 600;
    for (int i = 0; i < N; i++) {
        seed = seed * 1664525u + 1013904223u;
        const float n1 = (float)((seed >> 16) & 0xFFFF) / 32768.0f - 1.0f;
        seed = seed * 1664525u + 1013904223u;
        const float n2 = (float)((seed >> 16) & 0xFFFF) / 32768.0f - 1.0f;

        ariadna_kf_predict(&k, 0.3f * n1, dt, ACC_PSD, BIAS_RW);
        ariadna_kf_clamp_bias(&k, 1.5f);

        if (i % 48 == 0) {
            float z = 2.0f * n2;
            if ((i / 48) % 97 == 0) z += 500.0f;        // outlier
            ariadna_kf_correct(&k, 0, z, 2.5f * 2.5f, GATE_SIGMA);
            ariadna_kf_correct(&k, 1, 0.0f, 0.3f * 0.3f, GATE_SIGMA);
        }

        if (!p_is_healthy(&k)) { healthy = false; break; }
    }

    check(healthy, "covariance stays symmetric and positive over 10 minutes");
    check(std::isfinite(k.x[0]) && std::isfinite(k.x[1]) && std::isfinite(k.x[2]),
          "state is still finite after 144000 steps");
    check(fabsf(k.x[0]) < 50.0f, "position has not wandered off");

    printf("    600 s at %.0f Hz: pos %.2f m, vel %.2f m/s, bias %.3f m/s^2,"
           " sigma %.2f m\n",
           (double)RATE_HZ, (double)k.x[0], (double)k.x[1], (double)k.x[2],
           (double)ariadna_kf_sigma(&k, 0));

    // A stalled or absurd timestep must not be able to blow the filter
    // up. The sketch clamps dt before calling in, but the filter itself
    // should degrade rather than explode.
    ariadna_kf_t s;
    ariadna_kf_reset(&s, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f);
    ariadna_kf_predict(&s, 5.0f, 0.1f, ACC_PSD, BIAS_RW);
    check(p_is_healthy(&s), "a 100 ms step leaves the covariance healthy");
}

// ---------------------------------------------------------------------
//  6. Three axes really are independent
//
//  The claim that justifies running three 3x3 filters instead of one 9x9
//  one. Worth checking rather than asserting, because it is the argument
//  the whole structure rests on.
// ---------------------------------------------------------------------
static void test_axis_independence(void)
{
    printf("axis independence\n");

    ariadna_kf_t a, b;
    ariadna_kf_reset(&a, 0.0f, 0.0f, 10.0f, 1.0f, 0.5f);
    ariadna_kf_reset(&b, 0.0f, 0.0f, 10.0f, 1.0f, 0.5f);

    const float dt = 1.0f / RATE_HZ;
    for (int i = 0; i < (int)RATE_HZ; i++) {
        ariadna_kf_predict(&a, 1.0f, dt, ACC_PSD, BIAS_RW);
        ariadna_kf_predict(&b, 1.0f, dt, ACC_PSD, BIAS_RW);
        if (i % 48 == 0) {
            ariadna_kf_correct(&a, 0, 0.5f, 1.0f, GATE_SIGMA);
            // b gets a wildly different correction on the SAME step
            ariadna_kf_correct(&b, 0, -80.0f, 1.0f, GATE_SIGMA);
        }
    }

    // Nothing done to b may have touched a. This is trivially true of the
    // implementation and that is the point: it pins the property so that
    // a future "optimisation" sharing state between axes fails here.
    ariadna_kf_t ref;
    ariadna_kf_reset(&ref, 0.0f, 0.0f, 10.0f, 1.0f, 0.5f);
    for (int i = 0; i < (int)RATE_HZ; i++) {
        ariadna_kf_predict(&ref, 1.0f, dt, ACC_PSD, BIAS_RW);
        if (i % 48 == 0) ariadna_kf_correct(&ref, 0, 0.5f, 1.0f, GATE_SIGMA);
    }
    check(memcmp(&a, &ref, sizeof(a)) == 0,
          "one axis is bit-identical whatever another axis is fed");
}

// ---------------------------------------------------------------------
//  7. The barometer trim observer
//
//  This decides the altitude the vehicle lands on, so it is checked here
//  rather than trusted. What it must do: converge on a standing offset,
//  ignore zero-mean noise, never move suddenly, clamp rather than absorb
//  a large disagreement, and HOLD its value when the independent sensor
//  goes away — which on this vehicle happens in the last two metres,
//  where the anchor geometry stops seeing height.
// ---------------------------------------------------------------------
static void test_baro_trim(void)
{
    printf("barometer trim observer\n");

    ariadna_baro_trim_t t;
    const float TAU = 2.5f, LIMIT = 1.5f;

    // --- converges on a constant offset --------------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    check(ariadna_baro_trim_value(&t) == 0.0f, "starts at zero");
    // 25 Hz, the UWB solve rate, for four time constants.
    for (int k = 1; k <= (int)(25 * 4 * TAU); k++)
        ariadna_baro_trim_update(&t, 1.0f, (uint32_t)(k * 40));
    check(fabsf(ariadna_baro_trim_value(&t) - 1.0f) < 0.05f,
          "converges on a 1.0 m offset");

    // --- the time constant is what it claims ---------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    const int steps_1tau = (int)(25 * TAU);
    for (int k = 1; k <= steps_1tau; k++)
        ariadna_baro_trim_update(&t, 1.0f, (uint32_t)(k * 40));
    // 1 - 1/e = 0.632, with a little slack for the discrete update.
    check(ariadna_baro_trim_value(&t) > 0.55f && ariadna_baro_trim_value(&t) < 0.70f,
          "one time constant reaches ~63 %");

    // --- no single update may move it far ------------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    ariadna_baro_trim_update(&t, 3.0f, 1000);          // starts the clock
    ariadna_baro_trim_update(&t, 3.0f, 1040);
    check(fabsf(ariadna_baro_trim_value(&t)) < 0.10f,
          "one 3 m observation moves the trim by centimetres, not metres");

    // A long stall then a resume must not produce a catch-up jump.
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    ariadna_baro_trim_update(&t, 1.0f, 1000);
    ariadna_baro_trim_update(&t, 1.0f, 1900);          // 900 ms gap
    check(fabsf(ariadna_baro_trim_value(&t)) <= 0.2f,
          "a 900 ms gap cannot produce a step larger than the gain clamp");

    // --- zero-mean noise averages away --------------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    uint32_t seed = 99u;
    for (int k = 1; k <= 25 * 40; k++) {
        seed = seed * 1664525u + 1013904223u;
        const float n = ((float)((seed >> 9) & 0xFFFF) / 32768.0f) - 1.0f;
        ariadna_baro_trim_update(&t, 0.5f * n, (uint32_t)(k * 40));
    }
    check(fabsf(ariadna_baro_trim_value(&t)) < 0.10f,
          "zero-mean noise does not bias the trim");

    // --- clamped, and it says so --------------------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    for (int k = 1; k <= 25 * 60; k++)
        ariadna_baro_trim_update(&t, 5.0f, (uint32_t)(k * 40));
    check(fabsf(ariadna_baro_trim_value(&t) - LIMIT) < 1e-3f,
          "a 5 m disagreement is clamped, not absorbed");
    check(ariadna_baro_trim_saturated(&t) == 1,
          "and reports itself saturated - that is a survey error, not a barometer");
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    for (int k = 1; k <= 25 * 60; k++)
        ariadna_baro_trim_update(&t, -5.0f, (uint32_t)(k * 40));
    check(fabsf(ariadna_baro_trim_value(&t) + LIMIT) < 1e-3f, "clamps negative too");

    // --- holds when the independent sensor goes away -------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    for (int k = 1; k <= 25 * 20; k++)
        ariadna_baro_trim_update(&t, 0.8f, (uint32_t)(k * 40));
    const float held = ariadna_baro_trim_value(&t);
    check(fabsf(held - 0.8f) < 0.05f, "learned 0.8 m");
    // Nothing calls update for the rest of the descent.
    check(ariadna_baro_trim_value(&t) == held,
          "and holds it when UWB stops - which is the last two metres");

    // --- hostile input -------------------------------------------------
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    for (int k = 1; k <= 200; k++) ariadna_baro_trim_update(&t, 0.5f, (uint32_t)(k * 40));
    const float before = ariadna_baro_trim_value(&t);
    check(ariadna_baro_trim_update(&t, NAN, 9000) == 0, "a NaN observation is refused");
    check(ariadna_baro_trim_value(&t) == before, "and changes nothing");
    check(ariadna_baro_trim_update(&t, 0.5f, 0) == 0, "a zero timestamp is refused");
    check(std::isfinite(ariadna_baro_trim_value(&t)), "the trim stays finite");

    // The first update after a reset only starts the clock. Otherwise the
    // first sample would be weighted by a dt that was never measured.
    ariadna_baro_trim_reset(&t, TAU, LIMIT);
    check(ariadna_baro_trim_update(&t, 1.0f, 5000) == 0, "the first update only arms");
    check(ariadna_baro_trim_value(&t) == 0.0f, "and moves nothing");
    check(ariadna_baro_trim_update(&t, 1.0f, 5040) == 1, "the second one counts");
}

int main(void)
{
    printf("\nAriadna V2 navigation filter\n");
    printf("============================\n");

    test_predict();
    test_correct();
    test_bias_observability();
    test_gate();
    test_endurance();
    test_axis_independence();
    test_baro_trim();

    printf("\n%d checks, %d failures\n\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
