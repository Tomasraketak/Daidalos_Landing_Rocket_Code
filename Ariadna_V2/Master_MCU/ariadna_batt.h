// =====================================================================
//  Ariadna V2 — battery state of charge and run-time estimate (fw 3.29)
//
//  Question answered: "how long until the 2S LiPo is down to 20 % ?"
//
//  Inputs (the Master has them already): pack voltage, battery current
//  (ACS712) and air temperature (the Position MCU's barometer). The Master
//  feeds one sample per second; the estimate is recomputed once a minute from
//  the mean of the last five minutes (a 300-sample ring), so a servo burst or
//  a pyro pulse does not make the number jump.
//
//  HOW THE NUMBER IS MADE
//    1. Mean voltage / current / temperature over the window.
//    2. Voltage under load is NOT the charge state: the pack sags by I * R.
//       The open-circuit voltage is  V_oc = V + I * R(T)  and R grows when it
//       is cold (R25 * exp(k * (25 - T)), ~2x at 0 C).
//    3. V_oc / 2 (per cell) -> state of charge through a LiPo resting-voltage
//       table (linear interpolation).
//    4. The capacity actually available is smaller than the label says:
//          C_eff = C_nom * health * f_temp(T) * f_rate(I)
//       health  - parameter batt.health: an old or cheap pack that delivers
//                 less than the label (0.85 = 15 % less).
//       f_temp  - cold cells deliver less charge: 1.00 at >= 25 C, falling
//                 ~1 % per degree below 25 C (0.75 at 0 C), never below 0.5.
//       f_rate  - Peukert-like loss at high current: (I / 1C)^-(k-1), k = 1.05,
//                 only ever < 1 (a light load is not credited with a bonus).
//    5. Charge left above the cut-off:  (SoC - cutoff) * C_eff.
//       Time = that / mean current, capped at BATT_MAX_MIN. Under
//       BATT_I_MIN_A the load is too small to measure (the ACS712's own
//       noise is ~0.1 A) and the time is reported as "more than the cap".
//
//    6. SELF-CORRECTION of the health. The charge that really leaves the pack
//       is counted (current x time, exact: the Master gives the 1 s mean of a
//       200 Hz measurement). Over a stretch of >= 20 min in which the state of
//       charge fell by >= 10 points, the capacity the pack showed,
//       Q / dSoC, is compared with the capacity the model assumed. If the
//       remaining time keeps falling faster than one minute per minute, the
//       pack is worse than assumed and `learn` (a factor 0.6..1.0 on top of
//       batt.health) is lowered a little; if it falls slower it is raised back
//       toward 1.0 - never above the health the owner entered. Weak or
//       disturbed stretches (light load, charging, temperature swing) are
//       thrown away, not learned from.
//
//  This is an estimate: the voltage table is that of a typical LiPo and the
//  internal resistance is a typical value. The three parameters (capacity,
//  health, R25) are there to be tuned against the real pack: discharge it at
//  a known current and compare, then change batt.health. See USER_MANUAL.
//
//  No Arduino dependency: test/batt_test.cpp compiles this file unchanged.
//  Floats only (RP2350 has a single-precision FPU).
// =====================================================================
#ifndef ARIADNA_BATT_H
#define ARIADNA_BATT_H

#include <stdint.h>
#include <math.h>

#define BATT_CELLS          2
#define BATT_WINDOW_S       300        // moving-average window: 5 minutes, 1 sample per second
#define BATT_UPDATE_S       60         // the estimate is refreshed once a minute
#define BATT_MIN_SAMPLES    10         // first estimate after this many seconds of data
#define BATT_V_MIN_VALID    5.0f       // pack volts below this = no battery connected (USB only)
#define BATT_I_MIN_A        0.10f      // below this the load is not measurable
#define BATT_MAX_MIN        999.0f     // run-time cap, minutes
#define BATT_T_DEFAULT_C    20.0f      // used while the air temperature is not known
#define BATT_PEUKERT_K      1.05f
#define BATT_R_COLD_K       0.030f     // resistance grows by exp(k * (25 - T))
#define BATT_COLD_LOSS_PER_C 0.010f    // capacity lost per degree below 25 C
#define BATT_COLD_FLOOR     0.50f
// self-correction of the health
#define BATT_LEARN_MIN_S    1200       // shortest stretch that is judged: 20 min
#define BATT_LEARN_MAX_S    14400      // a stretch that never reaches the SoC drop is dropped after 4 h
#define BATT_LEARN_MIN_DSOC 10.0f      // points of state of charge the stretch must cover
#define BATT_LEARN_MIN_A    0.30f      // mean load below this is too weak to learn from
#define BATT_LEARN_MAX_DT_C 10.0f      // air temperature swing that voids a stretch
#define BATT_LEARN_GAIN     0.5f       // share of the error corrected per judged stretch
#define BATT_LEARN_MIN      0.60f
#define BATT_LEARN_K_LO     0.60f      // ratio measured/assumed is clamped to this range first
#define BATT_LEARN_K_HI     1.40f

// state of the estimate, as reported to the GUI
enum {
    BATT_NONE    = 0,                  // no battery measured (USB only) or no data yet
    BATT_OK      = 1,                  // run time is valid
    BATT_IDLE    = 2,                  // load too small to extrapolate: "more than BATT_MAX_MIN"
    BATT_LOW     = 3,                  // already at or below the cut-off
};

typedef struct {
    float cap_mah;       // label capacity of the pack                    (batt.cap_mah)
    float health;        // 0.5 .. 1.0 share of the label capacity it really delivers (batt.health)
    float r25_mohm;      // internal resistance of the whole pack at 25 C, wiring included (batt.r25_mohm)
    float cutoff_pct;    // run-time target: "time until the charge falls to this" (batt.cutoff_pct)
} batt_cfg_t;

typedef struct {
    // window
    float    v[BATT_WINDOW_S], i[BATT_WINDOW_S], t[BATT_WINDOW_S];
    uint16_t head;       // next slot to write
    uint16_t n;          // samples held (<= BATT_WINDOW_S)
    uint16_t since;      // seconds since the last estimate
    bool     have;       // an estimate exists
    // results (refreshed once a minute)
    uint8_t  status;     // BATT_*
    float    soc_pct;    // state of charge, % of the pack's own full charge
    float    minutes;    // time to the cut-off at the present load (BATT_MAX_MIN when idle)
    float    v_oc;       // open-circuit pack volts the SoC came from
    float    v_avg, i_avg, t_avg;
    float    cap_eff_mah;  // capacity the estimate worked with
    float    left_mah;     // charge left above the cut-off
    uint16_t win_s;      // seconds of data the estimate averaged (<= 300)
    // charge counting and the health self-correction
    float    q_mah;      // charge drawn since boot, mAh
    float    learn;      // 0.6..1.0 factor on batt.health, persisted by the Master (batt.learn)
    bool     a_on;       // a stretch is being measured
    float    a_q, a_soc, a_cap_sum, a_t_min, a_t_max, a_i_sum;
    uint16_t a_n;        // minute estimates in the stretch
    uint32_t a_s;        // seconds the stretch has lasted
    uint8_t  learn_events;   // how many stretches have been judged since boot
    float    last_k;     // the last judged ratio real/assumed capacity (0 = none yet)
} batt_t;

// ---- resting-voltage table of one LiPo cell: volts -> % --------------
static const float BATT_OCV_V[]   = { 3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f,
                                      3.82f, 3.84f, 3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f,
                                      4.11f, 4.15f, 4.20f };
static const float BATT_OCV_PCT[] = {  0.0f,  5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 35.0f, 40.0f,
                                      45.0f, 50.0f, 55.0f, 60.0f, 65.0f, 70.0f, 75.0f, 80.0f, 85.0f,
                                      90.0f, 95.0f, 100.0f };
#define BATT_OCV_N ((int)(sizeof(BATT_OCV_V) / sizeof(BATT_OCV_V[0])))

/** Resting cell voltage -> state of charge, 0..100 %. */
static inline float batt_soc_from_cell_v(float vc)
{
    if (!(vc > BATT_OCV_V[0]))              return 0.0f;     // also NaN
    if (vc >= BATT_OCV_V[BATT_OCV_N - 1])   return 100.0f;
    for (int k = 1; k < BATT_OCV_N; k++) {
        if (vc <= BATT_OCV_V[k]) {
            const float f = (vc - BATT_OCV_V[k - 1]) / (BATT_OCV_V[k] - BATT_OCV_V[k - 1]);
            return BATT_OCV_PCT[k - 1] + f * (BATT_OCV_PCT[k] - BATT_OCV_PCT[k - 1]);
        }
    }
    return 100.0f;
}

/** Pack resistance at air temperature t_c, ohms. */
static inline float batt_r_pack(const batt_cfg_t *c, float t_c)
{
    float dt = 25.0f - t_c;
    if (dt > 50.0f)  dt = 50.0f;               // never extrapolate the exponential far
    if (dt < -20.0f) dt = -20.0f;              // a hot pack is not much better than 45 C
    return 0.001f * c->r25_mohm * expf(BATT_R_COLD_K * dt);
}

/** Share of the label capacity that cold cells deliver, BATT_COLD_FLOOR..1. */
static inline float batt_f_temp(float t_c)
{
    if (t_c >= 25.0f) return 1.0f;
    const float f = 1.0f - BATT_COLD_LOSS_PER_C * (25.0f - t_c);
    return f < BATT_COLD_FLOOR ? BATT_COLD_FLOOR : f;
}

/** Share of the capacity that survives a load of i_a amps (never above 1). */
static inline float batt_f_rate(const batt_cfg_t *c, float i_a)
{
    const float one_c = 0.001f * c->cap_mah;           // 1C in amps
    if (!(i_a > one_c) || !(one_c > 0.0f)) return 1.0f;
    const float f = powf(i_a / one_c, -(BATT_PEUKERT_K - 1.0f));
    return f < 0.8f ? 0.8f : f;
}

static inline void batt_init(batt_t *b)
{
    for (uint16_t k = 0; k < BATT_WINDOW_S; k++) b->v[k] = b->i[k] = b->t[k] = 0.0f;
    b->head = b->n = b->since = 0;
    b->have = false;
    b->status = BATT_NONE;
    b->soc_pct = b->minutes = b->v_oc = 0.0f;
    b->v_avg = b->i_avg = b->t_avg = 0.0f;
    b->cap_eff_mah = b->left_mah = 0.0f;
    b->win_s = 0;
    b->q_mah = 0.0f;
    b->learn = 1.0f;
    b->a_on = false;
    b->a_q = b->a_soc = b->a_cap_sum = b->a_t_min = b->a_t_max = b->a_i_sum = 0.0f;
    b->a_n = 0;
    b->a_s = 0;
    b->learn_events = 0;
    b->last_k = 0.0f;
}

/** Start from the stored factor (the batt.learn parameter). */
static inline void batt_set_learn(batt_t *b, float learn)
{
    if (!(learn >= BATT_LEARN_MIN)) learn = BATT_LEARN_MIN;     // also NaN
    if (learn > 1.0f) learn = 1.0f;
    b->learn = learn;
}

static inline void batt_learn_drop(batt_t *b) { b->a_on = false; b->a_n = 0; b->a_s = 0; }

/** One minute estimate has just been made: extend, judge or drop the measuring stretch. */
static inline void batt_learn_step(batt_t *b)
{
    const bool usable = b->status == BATT_OK && b->i_avg >= BATT_LEARN_MIN_A;
    if (!usable) { batt_learn_drop(b); return; }
    if (!b->a_on) {
        b->a_on = true; b->a_q = b->q_mah; b->a_soc = b->soc_pct;
        b->a_cap_sum = 0.0f; b->a_i_sum = 0.0f; b->a_n = 0; b->a_s = 0;
        b->a_t_min = b->a_t_max = b->t_avg;
        return;
    }
    if (b->soc_pct > b->a_soc + 2.0f) { batt_learn_drop(b); return; }       // charged: start over
    b->a_cap_sum += b->cap_eff_mah;
    b->a_i_sum   += b->i_avg;
    b->a_n++;
    b->a_s += BATT_UPDATE_S;
    if (b->t_avg < b->a_t_min) b->a_t_min = b->t_avg;
    if (b->t_avg > b->a_t_max) b->a_t_max = b->t_avg;
    if (b->a_t_max - b->a_t_min > BATT_LEARN_MAX_DT_C) { batt_learn_drop(b); return; }
    if (b->a_s > BATT_LEARN_MAX_S) { batt_learn_drop(b); return; }
    const float dsoc = b->a_soc - b->soc_pct;
    if (b->a_s < BATT_LEARN_MIN_S || dsoc < BATT_LEARN_MIN_DSOC) return;

    // The stretch is long and deep enough: judge it.
    if (b->a_i_sum / (float)b->a_n >= BATT_LEARN_MIN_A) {
        const float cap_assumed = b->a_cap_sum / (float)b->a_n;             // mean over the stretch
        const float cap_real    = (b->q_mah - b->a_q) / (dsoc * 0.01f);
        float k = cap_real / cap_assumed;
        if (k < BATT_LEARN_K_LO) k = BATT_LEARN_K_LO;
        if (k > BATT_LEARN_K_HI) k = BATT_LEARN_K_HI;
        b->last_k = k;
        float l = b->learn * powf(k, BATT_LEARN_GAIN);
        batt_set_learn(b, l);
        b->learn_events++;
    }
    batt_learn_drop(b);                                                      // next stretch starts at the next minute
}

/** The estimate itself, from the window as it stands. */
static inline void batt_compute_inner(batt_t *b, const batt_cfg_t *c)
{
    float sv = 0.0f, si = 0.0f, st = 0.0f;
    const uint16_t n = b->n;
    for (uint16_t k = 0; k < n; k++) { sv += b->v[k]; si += b->i[k]; st += b->t[k]; }
    b->win_s  = n;
    b->v_avg  = sv / (float)n;
    b->i_avg  = si / (float)n;
    b->t_avg  = st / (float)n;
    b->since  = 0;
    b->have   = true;

    if (!(b->v_avg >= BATT_V_MIN_VALID)) {               // USB only: no battery on the divider
        b->status = BATT_NONE;
        b->soc_pct = b->minutes = b->v_oc = b->cap_eff_mah = b->left_mah = 0.0f;
        return;
    }

    const float i  = b->i_avg > 0.0f ? b->i_avg : 0.0f;  // charging / zero offset: no negative load
    b->v_oc        = b->v_avg + i * batt_r_pack(c, b->t_avg);
    b->soc_pct     = batt_soc_from_cell_v(b->v_oc / (float)BATT_CELLS);

    b->cap_eff_mah = c->cap_mah * c->health * b->learn * batt_f_temp(b->t_avg) * batt_f_rate(c, i);
    const float above = b->soc_pct - c->cutoff_pct;
    if (above <= 0.0f) {
        b->status = BATT_LOW;
        b->left_mah = 0.0f;
        b->minutes = 0.0f;
        return;
    }
    b->left_mah = above * 0.01f * b->cap_eff_mah;
    if (i < BATT_I_MIN_A) {
        b->status  = BATT_IDLE;
        b->minutes = BATT_MAX_MIN;
        return;
    }
    float m = 60.0f * (b->left_mah * 0.001f) / i;        // mAh -> Ah, h -> min
    if (m > BATT_MAX_MIN) m = BATT_MAX_MIN;
    b->minutes = m;
    b->status  = BATT_OK;
}

static inline void batt_compute(batt_t *b, const batt_cfg_t *c)
{
    batt_compute_inner(b, c);
    batt_learn_step(b);
}

/**
 * Call once a second with the MEAN of the last second's readings (the Master
 * averages the 200 Hz ADC samples: current at 200 Hz, not one sample a second). Returns true when a
 * new estimate was produced (the first one after BATT_MIN_SAMPLES seconds,
 * then every BATT_UPDATE_S).
 */
static inline bool batt_feed_1hz(batt_t *b, const batt_cfg_t *c, float vbat, float amps, float temp_c)
{
    if (!(temp_c > -60.0f && temp_c < 120.0f)) temp_c = BATT_T_DEFAULT_C;   // also NaN
    if (!(vbat >= 0.0f && vbat < 20.0f))       vbat = 0.0f;
    if (!(amps > -50.0f && amps < 50.0f))      amps = 0.0f;
    if (amps > 0.0f && vbat >= BATT_V_MIN_VALID) b->q_mah += amps * (1000.0f / 3600.0f);
    b->v[b->head] = vbat;
    b->i[b->head] = amps;
    b->t[b->head] = temp_c;
    b->head = (uint16_t)((b->head + 1) % BATT_WINDOW_S);
    if (b->n < BATT_WINDOW_S) b->n++;
    if (b->since < 0xFFFF) b->since++;

    const bool first = !b->have && b->n >= BATT_MIN_SAMPLES;
    if (first || (b->have && b->since >= BATT_UPDATE_S)) {
        batt_compute(b, c);
        return true;
    }
    return false;
}

#endif // ARIADNA_BATT_H
