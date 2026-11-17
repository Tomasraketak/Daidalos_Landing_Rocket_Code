// =====================================================================
//  Ariadna V2 — battery state of charge and run-time estimate
//
//  ../Ariadna_V2/common/ariadna_batt.h compiles UNCHANGED into this test and
//  into Master_MCU.ino.
//
//  What this establishes: the resting-voltage table is monotonic and covers
//  0..100 %; the pack's own sag (I * R) is added back before the voltage is
//  turned into a charge state, so the same charge reads the same at 0.5 A and
//  at 6 A; the time is the charge above the cut-off divided by the mean load;
//  cold, a worn pack and a heavy load each SHORTEN it and nothing lengthens
//  it; the five-minute window really is a moving average that refreshes once
//  a minute; and garbage (no battery, NaN, charging) never produces a number
//  that looks like a promise.
//
//  Build:  make batt
// =====================================================================
#include "../Ariadna_V2/common/ariadna_batt.h"
#include "../Ariadna_V2/common/ariadna_params.h"

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

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static batt_cfg_t cfg(float health = 0.90f)
{
    batt_cfg_t c;
    c.cap_mah = 2200.0f; c.health = health; c.r25_mohm = 60.0f; c.cutoff_pct = 20.0f;
    return c;
}

/** Steady readings for `secs` seconds, returns the last estimate. */
static batt_t steady(const batt_cfg_t &c, float v, float i, float t, int secs = 300)
{
    batt_t b;
    batt_init(&b);
    for (int k = 0; k < secs; k++) batt_feed_1hz(&b, &c, v, i, t);
    if (!b.have) batt_compute(&b, &c);
    return b;
}

int main()
{
    printf("battery estimate\n");
    const batt_cfg_t c = cfg();

    // ---- the table ------------------------------------------------------
    {
        bool mono = true;
        for (int k = 1; k < BATT_OCV_N; k++)
            if (!(BATT_OCV_V[k] > BATT_OCV_V[k - 1] && BATT_OCV_PCT[k] > BATT_OCV_PCT[k - 1])) mono = false;
        check(mono, "resting-voltage table strictly increasing");
        check(near(batt_soc_from_cell_v(4.20f), 100.0f, 0.01f), "4.20 V = 100 %");
        check(near(batt_soc_from_cell_v(4.35f), 100.0f, 0.01f), "above 4.20 V stays 100 %");
        check(near(batt_soc_from_cell_v(3.27f), 0.0f, 0.01f), "3.27 V = 0 %");
        check(near(batt_soc_from_cell_v(2.50f), 0.0f, 0.01f), "below 3.27 V stays 0 %");
        check(near(batt_soc_from_cell_v(3.73f), 20.0f, 0.01f), "3.73 V = 20 %");
        check(near(batt_soc_from_cell_v(3.80f), 40.0f, 0.01f), "3.80 V = 40 %");
        const float mid = batt_soc_from_cell_v(3.835f);
        check(mid > 44.0f && mid < 50.0f, "interpolates between table points");
        check(near(batt_soc_from_cell_v(NAN), 0.0f, 0.01f), "NaN volts -> 0 %, not garbage");
        float prev = -1.0f; bool up = true;
        for (float v = 3.0f; v <= 4.3f; v += 0.005f) {
            float s = batt_soc_from_cell_v(v);
            if (s < prev) up = false;
            prev = s;
        }
        check(up, "state of charge never falls as the voltage rises");
    }

    // ---- parameters in the table ----------------------------------------
    {
        ariadna_params_t p;
        ariadna_params_default(&p);
        check(near(p.v[ARIADNA_P_BATT_CAP_MAH], 2200.0f, 0.1f), "default capacity is the 2S 2200 mAh pack");
        check(near(p.v[ARIADNA_P_BATT_CUTOFF_PCT], 20.0f, 0.1f), "default cut-off is 20 %");
        check(p.v[ARIADNA_P_BATT_HEALTH] > 0.5f && p.v[ARIADNA_P_BATT_HEALTH] <= 1.0f, "default health within range");
        check(ariadna_param_set(&p, ARIADNA_P_BATT_HEALTH, 0.0f) == 1 && p.v[ARIADNA_P_BATT_HEALTH] >= 0.5f,
              "health 0 is clamped, never reaches the estimator");
        check(ariadna_param_find(224) && ariadna_param_find(227), "ids 224..227 exist");
        check(ARIADNA_PARAM_ID_MAX_V5 == 224, "layout 5 images are trusted up to id 223 only");
        bool owner_m = true;
        for (uint8_t id = 224; id <= 227; id++) if (ariadna_param_find(id)->owner != 'M') owner_m = false;
        check(owner_m, "battery parameters belong to the Master");
    }

    // ---- hand-checked numbers -------------------------------------------
    {
        // Full pack, 2 A, 25 C: V_oc = 8.40, SoC 100 %, left = 80 % of 2200*0.9 = 1584 mAh,
        // 1584 / 2000 mA = 0.792 h = 47.52 min.
        const float v = 8.40f - 2.0f * 0.060f;
        batt_t b = steady(c, v, 2.0f, 25.0f);
        check(b.status == BATT_OK, "loaded pack: valid");
        check(near(b.soc_pct, 100.0f, 0.5f), "full pack under 2 A reads 100 % (sag added back)");
        check(near(b.cap_eff_mah, 1980.0f, 1.0f), "capacity = 2200 * 0.90 at 25 C, below 1C");
        check(near(b.minutes, 47.52f, 0.2f), "47.5 min from full at 2 A");
        check(near(b.left_mah, 1584.0f, 1.0f), "1584 mAh above the cut-off");
    }

    // ---- sag compensation: same charge, different load -------------------
    {
        // The same open-circuit 7.60 V (3.80 V per cell = 40 %) at three loads.
        const float loads[3] = { 0.5f, 2.0f, 6.0f };
        float soc[3];
        for (int k = 0; k < 3; k++)
            soc[k] = steady(c, 7.60f - loads[k] * 0.060f, loads[k], 25.0f).soc_pct;
        check(near(soc[0], 40.0f, 1.0f) && near(soc[1], 40.0f, 1.0f) && near(soc[2], 40.0f, 1.0f),
              "40 % open-circuit reads 40 % at 0.5, 2 and 6 A");
        // ... and without the compensation the 6 A reading would have been far lower
        check(batt_soc_from_cell_v((7.60f - 0.36f) / 2.0f) < 30.0f, "uncompensated 6 A voltage would read < 30 %");
    }

    // ---- cold, worn pack, heavy load all shorten the time ------------------
    {
        const batt_cfg_t cw = cfg(0.80f);
        const float v = 7.80f, i = 2.0f;
        batt_t warm = steady(c, v, i, 25.0f);
        batt_t worn = steady(cw, v, i, 25.0f);
        check(worn.minutes < warm.minutes, "a worn pack (health 0.80) lasts less");
        check(near(worn.minutes / warm.minutes, 0.80f / 0.90f, 0.02f), "... in proportion to the health factor");

        // same terminal voltage, cold: more sag is assumed (higher OCV, more charge) but
        // the capacity is smaller; the capacity factor itself must be below 1
        batt_t cold = steady(c, v, i, 0.0f);
        check(cold.cap_eff_mah < warm.cap_eff_mah, "cold: smaller effective capacity");
        check(near(cold.cap_eff_mah / warm.cap_eff_mah, 0.75f, 0.01f), "0 C delivers 75 % of the capacity");
        check(batt_r_pack(&c, 0.0f) > 1.9f * batt_r_pack(&c, 25.0f), "resistance about doubles at 0 C");
        check(near(batt_f_temp(30.0f), 1.0f, 1e-6f), "warm air does not add capacity");
        check(near(batt_f_temp(-40.0f), BATT_COLD_FLOOR, 1e-6f), "deep cold: capacity factor floors at 0.5");

        check(near(batt_f_rate(&c, 1.0f), 1.0f, 1e-6f), "light load: no rate bonus");
        check(batt_f_rate(&c, 10.0f) < 1.0f && batt_f_rate(&c, 10.0f) >= 0.8f, "10 A (4.5C): rate loss, bounded");
        check(batt_f_rate(&c, 100.0f) >= 0.8f - 1e-6f, "rate loss never exceeds 20 %");

        // never optimistic: with the SAME charge state, heavier load = less time
        batt_t light = steady(c, 7.60f - 1.0f * 0.06f, 1.0f, 25.0f);
        batt_t heavy = steady(c, 7.60f - 4.0f * 0.06f, 4.0f, 25.0f);
        check(heavy.minutes < light.minutes / 3.0f, "4 A lasts less than a quarter of 1 A (rate loss on top)");
    }

    // ---- lower voltage = less time (monotonic) ---------------------------
    {
        float prev = 1e9f; bool ok = true;
        for (float v = 8.30f; v >= 7.30f; v -= 0.05f) {
            batt_t b = steady(c, v, 2.0f, 20.0f);
            if (b.minutes > prev + 1e-3f) ok = false;
            prev = b.minutes;
        }
        check(ok, "the discharge curve never gives MORE time at a lower voltage");
    }

    // ---- states -----------------------------------------------------------
    {
        batt_t usb = steady(c, 0.0f, 0.0f, 22.0f);
        check(usb.status == BATT_NONE, "USB only (no battery volts): none");
        check(usb.minutes == 0.0f && usb.soc_pct == 0.0f, "... and no numbers");

        batt_t idle = steady(c, 8.10f, 0.02f, 22.0f);
        check(idle.status == BATT_IDLE, "load below 0.1 A: idle, cannot extrapolate");
        check(idle.minutes == BATT_MAX_MIN && idle.soc_pct > 80.0f, "... reports the cap and still gives the %");

        batt_t chg = steady(c, 8.10f, -1.5f, 22.0f);
        check(chg.status == BATT_IDLE, "negative current (charging / offset) is not a load");

        batt_t low = steady(c, 7.30f - 0.12f, 2.0f, 22.0f);   // 3.65 V/cell open circuit ~ 7 %
        check(low.status == BATT_LOW && low.minutes == 0.0f, "at or below 20 %: LOW, 0 min");

        batt_t big = steady(c, 8.30f, 0.12f, 25.0f);
        check(big.status == BATT_OK && big.minutes <= BATT_MAX_MIN, "tiny load: capped, never inf");
        check(isfinite(big.minutes), "minutes always finite");
    }

    // ---- bad inputs --------------------------------------------------------
    {
        batt_t b; batt_init(&b);
        for (int k = 0; k < 30; k++) batt_feed_1hz(&b, &c, NAN, NAN, NAN);
        check(b.have && isfinite(b.minutes) && isfinite(b.soc_pct), "NaN inputs: finite output");
        check(b.status == BATT_NONE, "NaN volts = no battery");
        batt_t b2; batt_init(&b2);
        for (int k = 0; k < 30; k++) batt_feed_1hz(&b2, &c, 7.9f, 2.0f, NAN);
        check(near(b2.t_avg, BATT_T_DEFAULT_C, 0.01f), "unknown temperature -> 20 C, not 0");
        batt_t b3 = steady(c, 7.9f, 2.0f, 200.0f);
        check(isfinite(b3.minutes), "absurd temperature does not blow up");
    }

    // ---- the window ---------------------------------------------------------
    {
        batt_t b; batt_init(&b);
        int updates = 0, first_at = -1;
        int upd_at[16];
        for (int k = 1; k <= 400; k++) {
            if (batt_feed_1hz(&b, &c, 7.9f, 2.0f, 20.0f)) {
                if (updates < 16) upd_at[updates] = k;
                if (first_at < 0) first_at = k;
                updates++;
            }
        }
        check(first_at == BATT_MIN_SAMPLES, "first estimate after 10 s of data");
        check(updates == 1 + (400 - BATT_MIN_SAMPLES) / BATT_UPDATE_S, "then once a minute");
        check(upd_at[1] - upd_at[0] == 60 && upd_at[2] - upd_at[1] == 60, "updates exactly 60 s apart");
        check(b.win_s == BATT_WINDOW_S, "window holds 300 s");

        // moving average: 300 s at 1 A, then 300 s at 3 A -> the mean passes through 2 A at 150 s
        batt_init(&b);
        for (int k = 0; k < 300; k++) batt_feed_1hz(&b, &c, 7.9f, 1.0f, 20.0f);
        batt_compute(&b, &c);
        check(near(b.i_avg, 1.0f, 0.001f), "window full of 1 A: mean 1 A");
        for (int k = 0; k < 150; k++) batt_feed_1hz(&b, &c, 7.9f, 3.0f, 20.0f);
        batt_compute(&b, &c);
        check(near(b.i_avg, 2.0f, 0.001f), "150 s later the 5-minute mean is 2 A");
        for (int k = 0; k < 150; k++) batt_feed_1hz(&b, &c, 7.9f, 3.0f, 20.0f);
        batt_compute(&b, &c);
        check(near(b.i_avg, 3.0f, 0.001f), "300 s later it is 3 A: old samples are gone");

        // a 10 s, 8 A servo burst inside quiet 1 A does not move the estimate much
        batt_init(&b);
        for (int k = 0; k < 290; k++) batt_feed_1hz(&b, &c, 7.9f, 1.0f, 20.0f);
        for (int k = 0; k < 10; k++)  batt_feed_1hz(&b, &c, 7.9f, 8.0f, 20.0f);
        batt_compute(&b, &c);
        check(near(b.i_avg, 1.0f + 7.0f * 10.0f / 300.0f, 0.001f), "mean includes the burst in proportion (10 s of 300 s)");
    }

    // ---- a whole discharge, simulated ------------------------------------------
    {
        // A pack that REALLY holds 1800 mAh (the label says 2200), 2.0 A load, 25 C:
        // drain it with a cell model, check the estimate converges to the truth and
        // never promises more than ~15 % too much at the half-way point.
        const float real_mah = 1800.0f, I = 2.0f, R = 0.060f;
        batt_t b; batt_init(&b);
        const batt_cfg_t cw = cfg(0.82f);                     // owner measured and set 0.82
        float used = 0.0f; float truth_at_half = 0.0f, est_at_half = 0.0f; bool got_half = false;
        float t_end_min = 0.0f;
        for (int s = 0; s < 4 * 3600; s++) {
            const float soc = 100.0f * (1.0f - used / real_mah);
            // invert the table for the pack's open-circuit voltage
            float vc = 3.27f;
            for (int k = 1; k < BATT_OCV_N; k++)
                if (soc <= BATT_OCV_PCT[k]) {
                    const float f = (soc - BATT_OCV_PCT[k - 1]) / (BATT_OCV_PCT[k] - BATT_OCV_PCT[k - 1]);
                    vc = BATT_OCV_V[k - 1] + f * (BATT_OCV_V[k] - BATT_OCV_V[k - 1]);
                    break;
                }
            if (soc >= 100.0f) vc = 4.20f;
            batt_feed_1hz(&b, &cw, 2.0f * vc - I * R, I, 25.0f);
            used += I * 1000.0f / 3600.0f;
            if (soc <= 20.0f) { t_end_min = s / 60.0f; break; }
            if (!got_half && soc <= 60.0f && b.have) {
                got_half = true;
                est_at_half = b.minutes;
                truth_at_half = (soc - 20.0f) * 0.01f * real_mah / I * 60.0f / 1000.0f;
            }
        }
        check(t_end_min > 40.0f && t_end_min < 50.0f, "simulated discharge reaches 20 % in ~43 min");
        check(got_half, "estimate existed half-way");
        // with a 1-minute refresh and a 5-minute mean the estimate lags a little
        check(fabsf(est_at_half - truth_at_half) < 0.15f * truth_at_half,
              "tuned health 0.82: estimate within 15 % of the truth half-way");
    }

    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
