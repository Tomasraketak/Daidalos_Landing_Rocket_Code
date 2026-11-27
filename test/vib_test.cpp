// Vibration spectrum analyser (Ariadna_V2/common/ariadna_vib.h), host test.
//
// Synthetic IMU streams with known tones and known broadband noise go in
// through the same tap the Position MCU uses; the frames that come out must
// put the tones at the right frequency and RMS, and the bands must add up
// to the signal's variance (Parseval).
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include "../Ariadna_V2/common/ariadna_vib.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static vib_t V;   // ~30 kB: static, as on the MCU

static float db(float x) { return 20.0f * log10f(x); }

struct Tone { int raw; float f, a; };           // raw axis 0..5, Hz, amplitude (g or dps)

// Feeds `seconds` of signal, calling the service the way loop1() does
// (every 1 ms of signal), and returns the last complete frame.
static uint8_t g_stat_or;                 // stat bits of every frame of the last run
static double  g_band_pow[ARIADNA_VIB_BANDS];   // sum of axial band power over frames
static uint32_t g_band_n;

static int run(float fs, float seconds, const Tone *tones, int nt, const float noise[6],
               ariadna_vib_t last[2], uint32_t *frames_out, bool service = true)
{
    g_stat_or = 0; g_band_n = 0;
    memset(g_band_pow, 0, sizeof(g_band_pow));
    std::mt19937 rng(1234);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    const int n = (int)(fs * seconds);
    uint32_t frames = 0;
    int got = 0;
    for (int i = 0; i < n; i++) {
        const double t = i / (double)fs;
        float s[6] = { 0, 1.0f, 0, 0, 0, 0 };       // 1 g on the thrust axis
        s[1] += 0.3f * (float)(t / seconds);          // a thrust ramp: must not show up
        for (int k = 0; k < nt; k++) s[tones[k].raw] += tones[k].a * (float)sin(2.0 * M_PI * tones[k].f * t);
        for (int r = 0; r < 6; r++) s[r] += noise[r] * nd(rng);
        vib_push(&V, s, s + 3);
        if (service && (i % (int)(fs / 1000.0f)) == 0) {
            ariadna_vib_t out[2];
            for (int rep = 0; rep < 4; rep++)
                if (vib_service(&V, (uint32_t)(t * 1e6), out)) {
                    frames++; memcpy(last, out, sizeof(out)); got = 1;
                    g_stat_or |= out[0].stat;
                    g_band_n++;
                    for (int b = 0; b < ARIADNA_VIB_BANDS; b++) {
                        const double l = vib_level(out[0].band[0][b], 0);
                        g_band_pow[b] += l * l;
                    }
                }
        }
    }
    if (frames_out) *frames_out = frames;
    return got;
}

static int band_of(float f)
{
    for (int b = 0; b < ARIADNA_VIB_BANDS; b++)
        if (f >= V.band[b].fc_lo && f < V.band[b].fc_hi) return b;
    return -1;
}

// Is there a peak within tol (relative) of f, and what is its RMS?
static bool find_peak(const ariadna_vib_t &o, int j, int ch, float f, float tol, float *rms)
{
    for (int i = 0; i < ARIADNA_VIB_PEAKS; i++) {
        const float pf = o.peak_f[j][i] * 0.01f;
        if (pf > 0 && fabsf(pf - f) <= tol * f) { *rms = vib_level(o.peak_amp[j][i], (uint8_t)ch); return true; }
    }
    return false;
}

// --dump <file>: a 30 s "flight" (pad, ascent with a motor tone, descent,
// landing burn) as "VIB <t_ms> <state> <hex88>" lines, for check_vib_log.py.
static int dump(const char *path)
{
    FILE *fp = fopen(path, "w");
    if (!fp) return 1;
    vib_init(&V, 2000);
    std::mt19937 rng(99);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    for (int i = 0; i < 2000 * 30; i++) {
        const double t = i / 2000.0;
        // state: 2 ARMED 0-5 s, 3 ASCENT 5-12 s, 5 DESCENT 12-24 s, 6 GUIDANCE 24-30 s
        const int st = t < 5 ? 2 : (t < 12 ? 3 : (t < 24 ? 5 : 6));
        float s[6] = { 0, 1, 0, 0, 0, 0 };
        for (int r = 0; r < 6; r++) s[r] += (r < 3 ? 0.003f : 0.1f) * nd(rng);
        if (st == 3) { s[1] += 2.0f * (float)sin(2 * M_PI * 87.0 * t); s[0] += 0.4f * (float)sin(2 * M_PI * 23.0 * t); }
        if (st == 5) s[3] += 15.0f * (float)sin(2 * M_PI * 1.6 * t);
        if (st == 6) { s[4] += 6.0f * (float)sin(2 * M_PI * 4.3 * t); s[2] += 1.0f * (float)sin(2 * M_PI * 310.0 * t); }
        vib_push(&V, s, s + 3);
        ariadna_vib_t out[2];
        while (vib_service(&V, (uint32_t)(t * 1e6), out)) {
            for (int p = 0; p < 2; p++) {
                fprintf(fp, "VIB %u %d ", (unsigned)(t * 1000.0), st);
                const uint8_t *b = (const uint8_t *)&out[p];
                for (size_t k = 0; k < sizeof(ariadna_vib_t); k++) fprintf(fp, "%02x", b[k]);
                fprintf(fp, "\n");
            }
        }
    }
    fclose(fp);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "--dump") == 0) return dump(argv[2]);

    printf("sizeof(vib_t) = %zu bytes\n", sizeof(vib_t));
    CHECK(sizeof(vib_t) < 40000, "vib_t too large: %zu", sizeof(vib_t));

    // ---- codes --------------------------------------------------------------
    for (int ch = 0; ch < 4; ch++) {
        const float x = ch < 2 ? 0.0123f : 3.7f;
        const float y = vib_level(vib_code(x, (uint8_t)ch), (uint8_t)ch);
        CHECK(fabsf(db(y / x)) <= 0.26f, "code round trip ch%d: %g -> %g", ch, x, y);
    }
    CHECK(vib_code(0.0f, 0) == 0 && vib_code(1e-9f, 0) == 1 && vib_code(1e3f, 2) == 255, "code limits");

    // ---- band map: contiguous, centres where ISO 266 puts them -------------
    vib_init(&V, 2000);
    for (int b = 1; b < ARIADNA_VIB_BANDS; b++)
        CHECK(fabsf(V.band[b].fc_lo - V.band[b - 1].fc_hi) < 1e-3f * V.band[b].fc_lo, "band %d not contiguous", b);
    CHECK(V.band[0].stage == 2 && V.band[9].stage == 2 && V.band[10].stage == 1 &&
          V.band[20].stage == 1 && V.band[21].stage == 0 && V.band[30].stage == 0, "stage split");

    // ---- tones on every channel and in every stage --------------------------
    {
        vib_init(&V, 2000);
        const Tone tones[] = {
            { 1, 37.3f, 0.50f },    // axial accel, stage 1
            { 1, 212.0f, 0.10f },   // axial accel, stage 0
            { 0, 1.30f, 0.20f },    // lateral accel (X), stage 2
            { 2, 143.0f, 0.30f },   // lateral accel (Z), stage 0
            { 3, 2.20f, 20.0f },    // pitch rate, stage 2
            { 5, 11.7f, 8.0f },     // yaw rate, stage 1
            { 4, 480.0f, 5.0f },    // roll rate, stage 0, near the top
            { 4, 0.80f, 3.0f },     // roll rate, stage 2, near the bottom
        };
        const float noise[6] = { 0.002f, 0.002f, 0.002f, 0.05f, 0.05f, 0.05f };
        ariadna_vib_t f[2];
        uint32_t frames = 0;
        CHECK(run(2000.0f, 20.0f, tones, 8, noise, f, &frames), "no frame");
        CHECK(frames >= 38 && frames <= 40, "frame rate: %u frames in 20 s", frames);
        CHECK((f[0].stat & 0x07) != 0 && !(f[0].stat & ARIADNA_VIB_ST_OVERRUN) && !(f[0].stat & ARIADNA_VIB_ST_LOWRATE),
              "stat 0x%02x", f[0].stat);
        CHECK(f[0].part == 0 && f[1].part == 1 && f[0].seq == f[1].seq, "parts");
        CHECK(f[0].n_avg >= 7 && f[0].n_avg <= 9, "n_avg %u", f[0].n_avg);
        CHECK(abs((int)f[0].fs_dhz - 20000) <= 20, "fs %u", f[0].fs_dhz);

        for (const Tone &t : tones) {
            const int ch = vib_out_ch((uint8_t)t.raw);
            const ariadna_vib_t &o = f[ch >> 1];
            const int j = ch & 1;
            float rms = 0;
            // tolerance: a quarter of the bin width of the stage
            const int st = t.f < VIB_EDGE_21 ? 2 : (t.f < VIB_EDGE_10 ? 1 : 0);
            const float tol = 0.25f * vib_stage_df(&V, (uint8_t)st) / t.f + 0.002f;
            const bool ok = find_peak(o, j, ch, t.f, tol, &rms);
            CHECK(ok, "tone %.2f Hz on ch%d not among peaks (%.2f %.2f %.2f)", t.f, ch,
                  o.peak_f[j][0] * 0.01f, o.peak_f[j][1] * 0.01f, o.peak_f[j][2] * 0.01f);
            if (ok) {
                const float want = t.a / sqrtf(2.0f);
                CHECK(fabsf(db(rms / want)) < 1.0f, "tone %.2f Hz RMS %.4f, want %.4f (%.2f dB)",
                      t.f, rms, want, db(rms / want));
            }
            // and the bands around it carry the tone. Around it, not in it:
            // a Hann main lobe is 4 bins wide, wider than a 1/3 octave in the
            // low stage, and a tone near a band edge splits between two.
            const int b = band_of(t.f);
            float p = 0;
            for (int bb = b - 1; bb <= b + 1; bb++)
                if (bb >= 0 && bb < ARIADNA_VIB_BANDS) { const float l = vib_level(o.band[j][bb], (uint8_t)ch); p += l * l; }
            CHECK(b >= 0 && fabsf(db(sqrtf(p) / (t.a / sqrtf(2.0f)))) < 1.2f, "bands around %.2f Hz: %.4f", t.f, sqrtf(p));
        }
        // the thrust ramp and the 1 g are not vibration: the lowest axial band is quiet
        const float lo = vib_level(f[0].band[0][0], 0);
        CHECK(lo < 0.003f, "axial 0.5 Hz band %.5f g (ramp leaked)", lo);
        printf("tones: axial peaks %.2f / %.2f Hz, lateral %.2f / %.2f Hz, gyro lat %.2f / %.2f, roll %.2f / %.2f\n",
               f[0].peak_f[0][0] * 0.01, f[0].peak_f[0][1] * 0.01, f[0].peak_f[1][0] * 0.01, f[0].peak_f[1][1] * 0.01,
               f[1].peak_f[0][0] * 0.01, f[1].peak_f[0][1] * 0.01, f[1].peak_f[1][0] * 0.01, f[1].peak_f[1][1] * 0.01);
    }

    // ---- white noise: Parseval, averaged over a minute of frames --------------
    {
        vib_init(&V, 2000);
        const float sd = 0.05f;
        const float noise[6] = { sd, sd, sd, 2.0f, 2.0f, 2.0f };
        ariadna_vib_t f[2];
        CHECK(run(2000.0f, 60.0f, nullptr, 0, noise, f, nullptr), "no frame");
        // band RMS of white noise: sd * sqrt(bandwidth / (fs/2))
        float worst = 0;
        for (int b = 0; b < ARIADNA_VIB_BANDS; b++) {
            const float bw = V.band[b].fc_hi - V.band[b].fc_lo;
            const double want = (double)sd * sd * bw / 1000.0;
            const float e = (float)(10.0 * log10(g_band_pow[b] / g_band_n / want));
            if (fabsf(e) > fabsf(worst)) worst = e;
            // Statistical spread, not bias (the bias is < 0.5 dB with an hour
            // of data): the 0.5-4 Hz stage has ~30 independent windows a
            // minute, and the 5-8 Hz bands are only 1-2 bins wide.
            const float tol = b < 10 ? 1.5f : (b < 14 ? 1.2f : 0.8f);
            CHECK(fabsf(e) < tol, "white noise band %d (%.2f Hz): %.2f dB", b, sqrtf(V.band[b].fc_lo * V.band[b].fc_hi), e);
        }
        // overall 0.45-561 Hz, one frame
        const float want = sd * sqrtf((V.band[30].fc_hi - V.band[0].fc_lo) / 1000.0f);
        const float e_ax = db(vib_level(f[0].rms[0], 0) / want);
        const float e_lat = db(vib_level(f[0].rms[1], 1) / (want * sqrtf(2.0f)));
        CHECK(fabsf(e_ax) < 0.6f && fabsf(e_lat) < 0.6f, "white noise overall: axial %.2f dB, lateral %.2f dB", e_ax, e_lat);
        printf("white noise: overall axial %+.2f dB, lateral %+.2f dB, worst band (1 min mean) %+.2f dB\n", e_ax, e_lat, worst);
    }

    // ---- a full ring is counted, not blocked on --------------------------------
    {
        vib_init(&V, 2000);
        const float noise[6] = { 0.01f, 0.01f, 0.01f, 0.1f, 0.1f, 0.1f };
        ariadna_vib_t f[2];
        run(2000.0f, 1.0f, nullptr, 0, noise, f, nullptr, false);   // nobody drains
        CHECK(V.lost == 2000u - VIB_RING, "lost %u", V.lost);
        run(2000.0f, 2.0f, nullptr, 0, noise, f, nullptr);
        CHECK(g_stat_or & ARIADNA_VIB_ST_OVERRUN, "overrun not flagged (0x%02x)", g_stat_or);
        CHECK(!(f[0].stat & ARIADNA_VIB_ST_OVERRUN), "overrun flag must clear on the next frame");
    }

    // ---- a 1 kHz IMU (library without the 2 kHz setting) ------------------------
    {
        vib_init(&V, 1000);
        const Tone tones[] = { { 1, 37.3f, 0.5f }, { 1, 212.0f, 0.1f } };
        const float noise[6] = { 0.002f, 0.002f, 0.002f, 0.05f, 0.05f, 0.05f };
        ariadna_vib_t f[2];
        uint32_t frames = 0;
        CHECK(run(1000.0f, 12.0f, tones, 2, noise, f, &frames), "no frame at 1 kHz");
        CHECK(f[0].stat & ARIADNA_VIB_ST_LOWRATE, "LOWRATE not set at 1 kHz");
        CHECK(f[0].band[0][30] == 0 && f[0].band[0][29] != 0, "the 500 Hz band (to 561 Hz) must be void at 1 kHz");
        float rms;
        CHECK(find_peak(f[0], 0, 0, 37.3f, 0.01f, &rms) && find_peak(f[0], 0, 0, 212.0f, 0.01f, &rms), "tones at 1 kHz");
        CHECK(frames >= 22 && frames <= 24, "frames at 1 kHz: %u", frames);
    }

    // ---- sample clock 1.5 % slow: frequencies corrected ----------------------------
    {
        vib_init(&V, 2000);
        const float noise[6] = { 0.002f, 0.002f, 0.002f, 0.05f, 0.05f, 0.05f };
        // the sensor really samples at 1970 Hz while claiming 2000
        std::mt19937 rng(7);
        std::normal_distribution<float> nd(0.0f, 1.0f);
        ariadna_vib_t last[2]; int got = 0;
        for (int i = 0; i < (int)(1970 * 12); i++) {
            const double t = i / 1970.0;
            float s[6] = { 0, 1, 0, 0, 0, 0 };
            s[1] += 0.3f * (float)sin(2 * M_PI * 150.0 * t);
            for (int r = 0; r < 6; r++) s[r] += noise[r] * nd(rng);
            vib_push(&V, s, s + 3);
            ariadna_vib_t out[2];
            if (i % 2 == 0 && vib_service(&V, (uint32_t)(t * 1e6), out)) { memcpy(last, out, sizeof(out)); got = 1; }
            while (vib_service(&V, (uint32_t)(t * 1e6), out)) { memcpy(last, out, sizeof(out)); got = 1; }
        }
        float rms;
        CHECK(got && fabsf(last[0].fs_dhz * 0.1f - 1970.0f) < 3.0f, "measured fs %.1f", last[0].fs_dhz * 0.1);
        CHECK(got && find_peak(last[0], 0, 0, 150.0f, 0.004f, &rms), "150 Hz tone with a slow clock: %.2f",
              last[0].peak_f[0][0] * 0.01);
    }

    // ---- the tap installs only where the driver has one -----------------------
    {
        struct PatchedImu {
            void (*fn)(const float *, const float *, void *) = nullptr;
            void setSampleTap(void (*f)(const float *, const float *, void *), void *) { fn = f; }
        } patched;
        struct PlainImu { int x = 0; } plain;
        auto tap = +[](const float *a, const float *g, void *) { vib_push(&V, a, g); };
        CHECK(vib_install_tap(patched, tap, 0) && patched.fn == tap, "patched driver: tap installed");
        CHECK(!vib_install_tap(plain, tap, 0), "driver without setSampleTap: reported, still compiles");
        (void)plain.x;
    }

    printf("vib: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
