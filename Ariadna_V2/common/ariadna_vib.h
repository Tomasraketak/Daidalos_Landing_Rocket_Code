// =====================================================================
//  ariadna_vib.h — in-flight vibration spectrum, 0.5 Hz .. 500 Hz
//
//  Position MCU, fw 3.35. Header-only, no Arduino dependency, verified on a
//  workstation by `make -C test vib`.
//
//  WHAT IT ANSWERS
//    "At which frequencies does the vehicle shake, and how hard?" - per
//    flight phase, from the same ICM-42688 the attitude filter uses. Four
//    channels: axial and lateral acceleration (structure, motor, landing
//    legs), lateral and roll angular rate (attitude oscillation: TVC limit
//    cycles, fin flutter, aerodynamic coupling).
//
//  WHERE THE SAMPLES COME FROM
//    The IMU runs at 2 kHz and TriSenseINSLIB owns its FIFO; reading the
//    FIFO a second time would steal samples from the attitude filter (the
//    fw 3.21 bug). The Voltino driver instead calls a per-sample tap
//    (patches/voltino_trisense_sample_tap.patch) on core 0, which only
//    copies the sample into a lock-free ring - ~0.1 us per sample, ~0.03 %
//    of core 0. Everything else happens here, on core 1, which on the
//    Position MCU otherwise only parses GPS NMEA (< 2 % busy).
//
//  HOW (multi-rate Welch, the only way 0.5 Hz and 500 Hz fit one budget)
//
//    2000 Hz --+-- stage 0: 256-point FFT, Hann, 50 % overlap
//              |            df 7.8 Hz, window 0.128 s   -> bands 63..500 Hz
//              +-- 6th-order Butterworth 80 Hz, decimate /8
//    250 Hz  --+-- stage 1: 256-point FFT, Hann, 50 % overlap
//              |            df 0.98 Hz, window 1.02 s   -> bands 5..50 Hz
//              +-- 6th-order Butterworth 10 Hz, decimate /8
//    31.25 Hz ---- stage 2: 128-point FFT, Hann, 50 % overlap
//                           df 0.24 Hz, window 4.1 s    -> bands 0.5..4 Hz
//
//    Every window is detrended (mean and slope), so the thrust ramp and
//    gravity are not reported as low-frequency vibration. Bin powers are
//    normalised so that they sum to the signal variance (Parseval): a band
//    value is the true RMS in that band, whatever its width, and a tone's
//    RMS is the sum of its three Hann main-lobe bins.
//
//    One frame per stage-1 spectrum = every 128 samples at 250 Hz = 0.512 s:
//      stage 0  mean of the ~8 spectra since the previous frame
//      stage 1  the new spectrum
//      stage 2  the newest spectrum, refreshed every 4th frame (flagged)
//    Out: 31 1/3-octave band levels, the 3 strongest tones (frequency
//    interpolated between bins) and the overall RMS, per channel.
//
//  RESOLUTION IS PHYSICS, NOT A SETTING
//    Telling 0.5 Hz from 0.63 Hz needs ~8 s of signal. The low stage uses
//    4.1 s windows (0.24 Hz bins): below 1 Hz the bands are narrower than a
//    bin and the levels are an interpolation, the tone frequencies are
//    still good to ~0.05 Hz. The low bands also lag the flight by ~2-4 s.
//
//  COST (RP2350 @ 150 MHz, single-precision FPU)
//    core 0: the tap, 2000 x ~20 cycles/s                   ~0.03 %
//    core 1: 6 x 2 biquads/sample + ~106 real FFTs/s         ~1-1.5 %
//    RAM:    ~30 kB (ring 6 kB, three stage buffers 15 kB)
//    link:   2 x 94 B every 0.512 s                          +0.4 % of UART
//    flash:  2 x 96 B every 0.512 s = 375 B/s                +2 % of the log
// =====================================================================
#ifndef ARIADNA_VIB_H
#define ARIADNA_VIB_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ariadna_link.h"

// Ring between the core-0 tap and core 1. 512 samples = 256 ms at 2 kHz of
// core-1 latency before anything is lost (and then it is only counted).
#ifndef VIB_RING
#define VIB_RING 512u
#endif
#define VIB_N0      256u        // stage 0 FFT length
#define VIB_N1      256u        // stage 1 FFT length
#define VIB_N2      128u        // stage 2 FFT length
#define VIB_FS1     250.0f      // stage 1 rate [Hz]
#define VIB_D2      8u          // stage 1 -> stage 2 decimation
#define VIB_RAW     6u          // ax ay az gx gy gz

// Raw sample scaling in the ring: the sensor's own LSB at +-16 g / 2000 dps,
// so storing int16 loses nothing the ICM-42688 had.
#define VIB_ACC_LSB (1.0f / 2048.0f)    // g
#define VIB_GYR_LSB (1.0f / 16.4f)      // deg/s

// Thrust axis of the TriSense in Daidalos (board -Y = nose, thrust +Y).
#ifndef VIB_AXIAL_AXIS
#define VIB_AXIAL_AXIS 1u
#endif

typedef struct { float b0, b1, b2, a1, a2; } vib_bq_coef_t;
typedef struct { float z1, z2; } vib_bq_state_t;

typedef struct {
    float   fc_lo, fc_hi;       // band edges [Hz]
    uint8_t stage;              // which spectrum it is read from
    uint8_t k_lo, k_hi;         // first and last bin touched
    float   w_lo, w_hi;         // fractional weight of those two bins
} vib_band_map_t;

typedef struct {
    // ---- core 0 -> core 1 ring (SPSC) -------------------------------------
    int16_t           ring[VIB_RING][VIB_RAW];
    volatile uint32_t head;          // written by core 0 only
    volatile uint32_t tail;          // written by core 1 only
    volatile uint32_t lost;          // samples dropped on a full ring (core 0)

    // ---- configuration -----------------------------------------------------
    float    fs0;                    // nominal IMU rate [Hz]
    uint16_t d1;                     // stage 0 -> stage 1 decimation
    vib_bq_coef_t bq1[3], bq2[3];    // the two 6th-order anti-alias filters
    vib_band_map_t band[ARIADNA_VIB_BANDS];
    float    win0[VIB_N0], win2[VIB_N2];   // Hann (stage 1 uses win0: same N)
    float    norm0, norm2;           // 1 / (N * sum w^2)
    float    cs[VIB_N0 / 2], sn[VIB_N0 / 2]; // cos/sin(2 pi k / 256)

    // ---- stage buffers -------------------------------------------------------
    float    x0[VIB_RAW][VIB_N0]; uint16_t n0;
    float    x1[VIB_RAW][VIB_N1]; uint16_t n1;
    float    x2[VIB_RAW][VIB_N2]; uint16_t n2;
    vib_bq_state_t s1[VIB_RAW][3], s2[VIB_RAW][3];
    uint16_t dec1, dec2;
    bool     pend0, pend1, pend2;    // a full buffer waits for its FFT

    // ---- spectra (4 output channels) ----------------------------------------
    float    acc0[ARIADNA_VIB_CHANNELS][VIB_N0 / 2 + 1]; uint16_t cnt0;
    float    sp1[ARIADNA_VIB_CHANNELS][VIB_N1 / 2 + 1];
    float    sp2[ARIADNA_VIB_CHANNELS][VIB_N2 / 2 + 1]; bool have2, fresh2;
    float    fft[VIB_N0];            // scratch (kept off core 1's stack)
    float    tmpP[VIB_N0 / 2 + 1];
    float    acc2[ARIADNA_VIB_CHANNELS][VIB_N2 / 2 + 1];

    // ---- bookkeeping -------------------------------------------------------
    uint8_t  seq;
    uint32_t lost_seen;
    uint32_t consumed;               // samples taken off the ring
    uint32_t fs_mark_n, fs_mark_us;
    bool     fs_marked;
    float    fs_meas;                // measured IMU rate [Hz]
} vib_t;

// Output channel of each raw axis: lateral channels are power sums.
static inline uint8_t vib_out_ch(uint8_t raw)
{
    if (raw < 3) return raw == VIB_AXIAL_AXIS ? ARIADNA_VIB_CH_ACC_AX : ARIADNA_VIB_CH_ACC_LAT;
    return (uint8_t)(raw - 3) == VIB_AXIAL_AXIS ? ARIADNA_VIB_CH_GYR_ROLL : ARIADNA_VIB_CH_GYR_LAT;
}

// ---------------------------------------------------------------------
//  dB codes (format in ariadna_link.h)
// ---------------------------------------------------------------------
static inline uint8_t vib_code(float rms, uint8_t ch)
{
    if (!(rms > 0.0f)) return 0;
    const float off = (ch <= ARIADNA_VIB_CH_ACC_LAT) ? 100.0f : 70.0f;
    const float c = 2.0f * (20.0f * log10f(rms) + off);
    if (c < 1.0f)   return 1;            // measured, at or below the range
    if (c > 255.0f) return 255;
    return (uint8_t)(c + 0.5f);
}

static inline float vib_level(uint8_t code, uint8_t ch)
{
    if (code == 0) return 0.0f;
    const float off = (ch <= ARIADNA_VIB_CH_ACC_LAT) ? 100.0f : 70.0f;
    return powf(10.0f, ((float)code * 0.5f - off) / 20.0f);
}

// ---------------------------------------------------------------------
//  Core 0: the tap. Copy and return - nothing else.
// ---------------------------------------------------------------------
static inline int16_t vib_q(float v, float inv_lsb)
{
    float q = v * inv_lsb;
    if (q >  32767.0f) q =  32767.0f;
    if (q < -32767.0f) q = -32767.0f;
    return (int16_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
}

static inline void vib_push(vib_t *v, const float acc_g[3], const float gyr_dps[3])
{
    const uint32_t h = v->head;
    const uint32_t t = __atomic_load_n(&v->tail, __ATOMIC_ACQUIRE);
    if ((uint32_t)(h - t) >= VIB_RING) { v->lost = v->lost + 1u; return; }
    int16_t *s = v->ring[h & (VIB_RING - 1u)];
    s[0] = vib_q(acc_g[0], 2048.0f); s[1] = vib_q(acc_g[1], 2048.0f); s[2] = vib_q(acc_g[2], 2048.0f);
    s[3] = vib_q(gyr_dps[0], 16.4f); s[4] = vib_q(gyr_dps[1], 16.4f); s[5] = vib_q(gyr_dps[2], 16.4f);
    __atomic_store_n(&v->head, h + 1u, __ATOMIC_RELEASE);
}

// ---------------------------------------------------------------------
//  Design helpers (vib_init only)
// ---------------------------------------------------------------------
// RBJ low-pass, bilinear with pre-warping; a1/a2 in the y-recursion sign
// convention y = b0 x + b1 x1 + b2 x2 - a1 y1 - a2 y2.
static inline vib_bq_coef_t vib_lowpass(float fc, float fs, float q)
{
    const float w0 = 6.28318531f * fc / fs;
    const float c = cosf(w0), al = sinf(w0) / (2.0f * q);
    const float a0 = 1.0f + al;
    vib_bq_coef_t k;
    k.b0 = (1.0f - c) * 0.5f / a0;
    k.b1 = (1.0f - c) / a0;
    k.b2 = k.b0;
    k.a1 = -2.0f * c / a0;
    k.a2 = (1.0f - al) / a0;
    return k;
}

static inline void vib_butter6(vib_bq_coef_t bq[3], float fc, float fs)
{
    // Pole-pair Qs of a 6th-order Butterworth: 1 / (2 sin((2k+1) pi / 12))
    bq[0] = vib_lowpass(fc, fs, 0.51763809f);
    bq[1] = vib_lowpass(fc, fs, 0.70710678f);
    bq[2] = vib_lowpass(fc, fs, 1.93185165f);
}

static inline float vib_bq_run(const vib_bq_coef_t bq[3], vib_bq_state_t st[3], float x)
{
    for (uint8_t i = 0; i < 3; i++) {          // transposed direct form II
        const vib_bq_coef_t &k = bq[i];
        const float y = k.b0 * x + st[i].z1;
        st[i].z1 = k.b1 * x - k.a1 * y + st[i].z2;
        st[i].z2 = k.b2 * x - k.a2 * y;
        x = y;
    }
    return x;
}

static inline float vib_stage_df(const vib_t *v, uint8_t stage)
{
    if (stage == 0) return v->fs0 / (float)VIB_N0;
    if (stage == 1) return VIB_FS1 / (float)VIB_N1;
    return (VIB_FS1 / (float)VIB_D2) / (float)VIB_N2;
}

static inline uint16_t vib_stage_bins(uint8_t stage)
{
    return (uint16_t)((stage == 2 ? VIB_N2 : VIB_N0) / 2u + 1u);
}

// Stage boundaries [Hz]: the edges between the 4 Hz / 5 Hz and the
// 50 Hz / 63 Hz bands.
#define VIB_EDGE_21  4.4898f    // 4 Hz band upper edge  = 3.98 * 2^(1/6)
#define VIB_EDGE_10  56.234f    // 50 Hz band upper edge = 50.1 * 2^(1/6)

static inline void vib_init(vib_t *v, uint16_t fs_nominal_hz)
{
    memset(v, 0, sizeof(*v));
    v->fs0 = (float)fs_nominal_hz;
    v->d1  = (uint16_t)(fs_nominal_hz / 250u);
    if (v->d1 < 1) v->d1 = 1;
    v->fs_meas = v->fs0;

    vib_butter6(v->bq1, 80.0f, v->fs0);
    vib_butter6(v->bq2, 10.0f, VIB_FS1);

    float s2 = 0.0f;
    for (uint16_t n = 0; n < VIB_N0; n++) {
        v->win0[n] = 0.5f - 0.5f * cosf(6.28318531f * (float)n / (float)VIB_N0);
        s2 += v->win0[n] * v->win0[n];
    }
    v->norm0 = 1.0f / ((float)VIB_N0 * s2);
    s2 = 0.0f;
    for (uint16_t n = 0; n < VIB_N2; n++) {
        v->win2[n] = 0.5f - 0.5f * cosf(6.28318531f * (float)n / (float)VIB_N2);
        s2 += v->win2[n] * v->win2[n];
    }
    v->norm2 = 1.0f / ((float)VIB_N2 * s2);
    for (uint16_t k = 0; k < VIB_N0 / 2; k++) {
        v->cs[k] = cosf(6.28318531f * (float)k / (float)VIB_N0);
        v->sn[k] = sinf(6.28318531f * (float)k / (float)VIB_N0);
    }

    // Band -> bins. Bin k covers [(k - 0.5) df, (k + 0.5) df]; a bin is
    // counted with the fraction of that interval inside the band, so the
    // contiguous bands together sum every bin exactly once.
    for (uint8_t b = 0; b < ARIADNA_VIB_BANDS; b++) {
        const float fc = 1000.0f * powf(10.0f, ((float)b - 33.0f) / 10.0f);   // exact base-10 centre
        const float fl = fc * 0.89089872f, fh = fc * 1.12246205f;                   // 2^(-+1/6)
        vib_band_map_t &m = v->band[b];
        m.fc_lo = fl; m.fc_hi = fh;
        m.stage = (fc < VIB_EDGE_21) ? 2 : (fc < VIB_EDGE_10 ? 1 : 0);
        const float df = vib_stage_df(v, m.stage);
        const float kl = fl / df + 0.5f, kh = fh / df + 0.5f;      // in "bin edge" units
        const uint16_t nb = vib_stage_bins(m.stage);
        uint16_t klo = (uint16_t)kl, khi = (uint16_t)kh;
        if (khi >= nb) khi = (uint16_t)(nb - 1u);
        if (klo > khi) klo = khi;
        m.k_lo = (uint8_t)klo; m.k_hi = (uint8_t)khi;
        if (klo == khi) { m.w_lo = kh - kl; m.w_hi = 0.0f; }
        else            { m.w_lo = (float)(klo + 1u) - kl; m.w_hi = kh - (float)khi; }
    }
}

// ---------------------------------------------------------------------
//  Real FFT -> one-sided power, normalised so that sum(P) = variance.
//  N = 256 or 128. Packs the N reals as N/2 complex, radix-2, then splits.
//  x is destroyed (used as the complex work buffer).
// ---------------------------------------------------------------------
static inline void vib_rfft_power(const vib_t *v, float *x, uint16_t N, float norm, float *P)
{
    const uint16_t M = (uint16_t)(N / 2u);
    const uint16_t tw = (uint16_t)(VIB_N0 / N);      // twiddle stride in the 256 table
    // bit reversal of the M complex points (x[2n], x[2n+1])
    for (uint16_t i = 1, j = 0; i < M; i++) {
        uint16_t bit = (uint16_t)(M >> 1);
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = x[2 * i]; x[2 * i] = x[2 * j]; x[2 * j] = t;
            t = x[2 * i + 1]; x[2 * i + 1] = x[2 * j + 1]; x[2 * j + 1] = t;
        }
    }
    // butterflies, W_M^k = exp(-2 pi i k / M) = table[k * 2 * tw]
    for (uint16_t len = 2; len <= M; len = (uint16_t)(len << 1)) {
        const uint16_t half = (uint16_t)(len >> 1);
        const uint16_t step = (uint16_t)((M / len) * 2u * tw);
        for (uint16_t i = 0; i < M; i = (uint16_t)(i + len)) {
            for (uint16_t k = 0; k < half; k++) {
                const float wr = v->cs[k * step], wi = -v->sn[k * step];
                float *a = &x[2 * (i + k)], *b = &x[2 * (i + k + half)];
                const float tr = b[0] * wr - b[1] * wi;
                const float ti = b[0] * wi + b[1] * wr;
                b[0] = a[0] - tr; b[1] = a[1] - ti;
                a[0] += tr;       a[1] += ti;
            }
        }
    }
    // split: X_k = (Z_k + Z*_{M-k})/2 - i/2 W_N^k (Z_k - Z*_{M-k})
    const float z0r = x[0], z0i = x[1];
    P[0] = (z0r + z0i) * (z0r + z0i) * norm;           // DC
    P[M] = (z0r - z0i) * (z0r - z0i) * norm;           // Nyquist
    for (uint16_t k = 1; k < M; k++) {
        const float ar = x[2 * k], ai = x[2 * k + 1];
        const float br = x[2 * (M - k)], bi = -x[2 * (M - k) + 1];   // conj Z_{M-k}
        const float er = 0.5f * (ar + br), ei = 0.5f * (ai + bi);
        const float dr = 0.5f * (ar - br), di = 0.5f * (ai - bi);
        const float wr = v->cs[k * tw], wi = -v->sn[k * tw];        // W_N^k
        // -i * W * d
        const float pr = wr * dr - wi * di, pi = wr * di + wi * dr;
        const float Xr = er + pi, Xi = ei - pr;
        P[k] = 2.0f * (Xr * Xr + Xi * Xi) * norm;
    }
}

// Detrend (mean and slope), window, FFT, add the power to the output channel.
static inline void vib_spectrum(vib_t *v, const float *src, uint16_t N, const float *win,
                                float norm, float *P_acc, float *tmpP)
{
    float sx = 0.0f, snx = 0.0f;
    const float c = 0.5f * (float)(N - 1u);
    for (uint16_t n = 0; n < N; n++) { sx += src[n]; snx += ((float)n - c) * src[n]; }
    const float mean = sx / (float)N;
    const float sxx = (float)N * ((float)N * (float)N - 1.0f) / 12.0f;   // sum (n - c)^2
    const float slope = snx / sxx;
    for (uint16_t n = 0; n < N; n++)
        v->fft[n] = (src[n] - mean - slope * ((float)n - c)) * win[n];
    vib_rfft_power(v, v->fft, N, norm, tmpP);
    for (uint16_t k = 0; k <= N / 2u; k++) P_acc[k] += tmpP[k];
}

// ---------------------------------------------------------------------
//  Frame assembly
// ---------------------------------------------------------------------
typedef struct { float f, p; } vib_peak_t;

static inline void vib_peak_offer(vib_peak_t pk[ARIADNA_VIB_PEAKS], float f, float p)
{
    // The same tone seen twice (both sides of a stage boundary): keep the
    // stronger reading only.
    for (uint8_t i = 0; i < ARIADNA_VIB_PEAKS; i++) {
        if (pk[i].p > 0.0f && fabsf(pk[i].f - f) < 0.05f * f) {
            if (p <= pk[i].p) return;
            pk[i].p = 0.0f;
        }
    }
    // Replace the weakest, then keep the list strongest first.
    uint8_t w = 0;
    for (uint8_t i = 1; i < ARIADNA_VIB_PEAKS; i++) if (pk[i].p < pk[w].p) w = i;
    if (p <= pk[w].p) return;
    pk[w].f = f; pk[w].p = p;
    for (uint8_t i = 1; i < ARIADNA_VIB_PEAKS; i++)
        for (uint8_t j = i; j > 0 && pk[j].p > pk[j - 1].p; j--) {
            const vib_peak_t t = pk[j]; pk[j] = pk[j - 1]; pk[j - 1] = t;
        }
}

static inline void vib_find_peaks(const float *S, uint16_t nb, float df, float f_lo, float f_hi,
                                  vib_peak_t pk[ARIADNA_VIB_PEAKS])
{
    for (uint16_t k = 2; k + 2 < nb; k++) {
        const float fk = (float)k * df;
        if (fk < f_lo || fk >= f_hi) continue;
        const float a = S[k - 1], b = S[k], c = S[k + 1];
        if (!(b > a && b >= c)) continue;
        // prominence: at least 2x the mean of the bins two away (3 dB)
        if (b < (S[k - 2] + S[k + 2])) continue;
        float d = 0.0f;                                  // Gaussian interpolation
        if (a > 0.0f && c > 0.0f) {
            const float la = logf(a), lb = logf(b), lc = logf(c);
            const float den = la - 2.0f * lb + lc;
            if (den < 0.0f) d = 0.5f * (la - lc) / den;
            if (d > 0.5f) d = 0.5f;
            if (d < -0.5f) d = -0.5f;
        }
        vib_peak_offer(pk, ((float)k + d) * df, a + b + c);
    }
}

static inline void vib_make_frame(vib_t *v, ariadna_vib_t out[2])
{
    const float inv0 = v->cnt0 ? 1.0f / (float)v->cnt0 : 0.0f;
    const float fscale = v->fs_meas / v->fs0;            // sample clock error
    const float f_void = 0.45f * v->fs_meas;             // above: aliased / filtered
    const uint32_t lost = v->lost;
    uint8_t stat = (uint8_t)((v->cnt0 ? ARIADNA_VIB_ST_FRESH0 : 0u) | ARIADNA_VIB_ST_FRESH1 |
                             (v->fresh2 ? ARIADNA_VIB_ST_FRESH2 : 0u));
    if (lost != v->lost_seen) stat |= ARIADNA_VIB_ST_OVERRUN;
    if (v->fs_meas < 1800.0f) stat |= ARIADNA_VIB_ST_LOWRATE;
    v->lost_seen = lost;

    memset(out, 0, 2 * sizeof(ariadna_vib_t));
    for (uint8_t ch = 0; ch < ARIADNA_VIB_CHANNELS; ch++) {
        ariadna_vib_t &o = out[ch >> 1];
        const uint8_t j = ch & 1u;
        float *S0 = v->acc0[ch];
        for (uint16_t k = 0; k <= VIB_N0 / 2u; k++) S0[k] *= inv0;
        const float *S[3] = { S0, v->sp1[ch], v->sp2[ch] };
        const bool  ok[3] = { v->cnt0 > 0, true, v->have2 };

        float total = 0.0f;
        for (uint8_t b = 0; b < ARIADNA_VIB_BANDS; b++) {
            const vib_band_map_t &m = v->band[b];
            if (!ok[m.stage] || m.fc_hi > f_void) { o.band[j][b] = 0; continue; }
            const float *s = S[m.stage];
            float p = m.w_lo * s[m.k_lo];
            if (m.k_hi > m.k_lo) {
                p += m.w_hi * s[m.k_hi];
                for (uint16_t k = (uint16_t)(m.k_lo + 1u); k < m.k_hi; k++) p += s[k];
            }
            total += p;
            o.band[j][b] = vib_code(sqrtf(p), ch);
        }
        o.rms[j] = vib_code(sqrtf(total), ch);

        vib_peak_t pk[ARIADNA_VIB_PEAKS];
        memset(pk, 0, sizeof(pk));
        const float flo = v->band[0].fc_lo;
        if (ok[2]) vib_find_peaks(S[2], vib_stage_bins(2), vib_stage_df(v, 2), flo, VIB_EDGE_21, pk);
        vib_find_peaks(S[1], vib_stage_bins(1), vib_stage_df(v, 1), VIB_EDGE_21, VIB_EDGE_10, pk);
        if (ok[0]) {
            const float fhi = v->band[ARIADNA_VIB_BANDS - 1].fc_hi;
            vib_find_peaks(S[0], vib_stage_bins(0), vib_stage_df(v, 0), VIB_EDGE_10,
                           fhi < f_void ? fhi : f_void, pk);
        }
        for (uint8_t i = 0; i < ARIADNA_VIB_PEAKS; i++) {
            if (!(pk[i].p > 0.0f)) continue;
            const float f = pk[i].f * fscale * 100.0f;
            o.peak_f[j][i]   = (uint16_t)(f > 65535.0f ? 65535.0f : f + 0.5f);
            o.peak_amp[j][i] = vib_code(sqrtf(pk[i].p), ch);
        }
    }
    for (uint8_t p = 0; p < 2; p++) {
        out[p].part   = p;
        out[p].seq    = v->seq;
        out[p].stat   = stat;
        out[p].n_avg  = (uint8_t)(v->cnt0 > 255u ? 255u : v->cnt0);
        out[p].fs_dhz = (uint16_t)(v->fs_meas * 10.0f + 0.5f);
    }
    v->seq++;
    memset(v->acc0, 0, sizeof(v->acc0));
    v->cnt0   = 0;
    v->fresh2 = false;
}

// ---------------------------------------------------------------------
//  Core 1: call as often as convenient. Takes up to 256 samples off the
//  ring and runs at most one stage's set of six FFTs per call (~0.6 ms),
//  so the GPS parser beside it never waits long. Returns true when `out`
//  holds a new frame (both parts).
// ---------------------------------------------------------------------
static inline bool vib_service(vib_t *v, uint32_t now_us, ariadna_vib_t out[2])
{
    float *tmpP = v->tmpP;

    // ---- 1. one pending FFT set, highest stage first ----------------------
    if (v->pend2) {
        // into a scratch first: sp2 must stay whole until the new one is
        memset(v->acc2, 0, sizeof(v->acc2));
        for (uint8_t r = 0; r < VIB_RAW; r++)
            vib_spectrum(v, v->x2[r], VIB_N2, v->win2, v->norm2, v->acc2[vib_out_ch(r)], tmpP);
        memcpy(v->sp2, v->acc2, sizeof(v->acc2));
        v->have2 = v->fresh2 = true;
        for (uint8_t r = 0; r < VIB_RAW; r++)
            memmove(v->x2[r], v->x2[r] + VIB_N2 / 2u, (VIB_N2 / 2u) * sizeof(float));
        v->n2 = VIB_N2 / 2u; v->pend2 = false;
        return false;
    }
    if (v->pend0) {
        for (uint8_t r = 0; r < VIB_RAW; r++)
            vib_spectrum(v, v->x0[r], VIB_N0, v->win0, v->norm0, v->acc0[vib_out_ch(r)], tmpP);
        v->cnt0++;
        for (uint8_t r = 0; r < VIB_RAW; r++)
            memmove(v->x0[r], v->x0[r] + VIB_N0 / 2u, (VIB_N0 / 2u) * sizeof(float));
        v->n0 = VIB_N0 / 2u; v->pend0 = false;
        return false;
    }
    if (v->pend1) {
        memset(v->sp1, 0, sizeof(v->sp1));
        for (uint8_t r = 0; r < VIB_RAW; r++)
            vib_spectrum(v, v->x1[r], VIB_N1, v->win0, v->norm0, v->sp1[vib_out_ch(r)], tmpP);
        for (uint8_t r = 0; r < VIB_RAW; r++)
            memmove(v->x1[r], v->x1[r] + VIB_N1 / 2u, (VIB_N1 / 2u) * sizeof(float));
        v->n1 = VIB_N1 / 2u; v->pend1 = false;
        vib_make_frame(v, out);                 // the frame clock
        return true;
    }

    // ---- 2. samples -> stage buffers and decimators -------------------------
    uint32_t t = v->tail;
    const uint32_t h = __atomic_load_n(&v->head, __ATOMIC_ACQUIRE);
    for (uint16_t i = 0; i < 256u && t != h && !v->pend0 && !v->pend1 && !v->pend2; i++, t++) {
        const int16_t *s = v->ring[t & (VIB_RING - 1u)];
        float x[VIB_RAW];
        for (uint8_t r = 0; r < VIB_RAW; r++) x[r] = (float)s[r] * (r < 3 ? VIB_ACC_LSB : VIB_GYR_LSB);
        for (uint8_t r = 0; r < VIB_RAW; r++) v->x0[r][v->n0] = x[r];
        if (++v->n0 >= VIB_N0) v->pend0 = true;
        float y[VIB_RAW];
        for (uint8_t r = 0; r < VIB_RAW; r++) y[r] = vib_bq_run(v->bq1, v->s1[r], x[r]);
        if (++v->dec1 < v->d1) continue;
        v->dec1 = 0;
        for (uint8_t r = 0; r < VIB_RAW; r++) v->x1[r][v->n1] = y[r];
        if (++v->n1 >= VIB_N1) v->pend1 = true;
        for (uint8_t r = 0; r < VIB_RAW; r++) y[r] = vib_bq_run(v->bq2, v->s2[r], y[r]);
        if (++v->dec2 < VIB_D2) continue;
        v->dec2 = 0;
        for (uint8_t r = 0; r < VIB_RAW; r++) v->x2[r][v->n2] = y[r];
        if (++v->n2 >= VIB_N2) v->pend2 = true;
    }
    v->consumed += (uint32_t)(t - v->tail);
    __atomic_store_n(&v->tail, t, __ATOMIC_RELEASE);

    // ---- 3. the real sample rate, against the MCU crystal, every 4 s --------
    if (!v->fs_marked) {
        if (v->consumed > 0) { v->fs_mark_n = v->consumed; v->fs_mark_us = now_us; v->fs_marked = true; }
    } else if ((uint32_t)(now_us - v->fs_mark_us) >= 4000000u) {
        const float dt = (float)(uint32_t)(now_us - v->fs_mark_us) * 1e-6f;
        const float fs = (float)(uint32_t)(v->consumed - v->fs_mark_n) / dt;
        // only a plausible reading: a stalled stream (HIL, sensor fault) is
        // not a slow clock
        if (fs > 0.5f * v->fs0 && fs < 1.5f * v->fs0) v->fs_meas = fs;
        v->fs_mark_n = v->consumed; v->fs_mark_us = now_us;
    }
    return false;
}

// ---------------------------------------------------------------------
//  Installing the tap - only if the driver has one.
//
//  vib_install_tap(sensor.imu, &fn, 0) calls imu.setSampleTap(fn, nullptr)
//  and returns true when the Voltino_TriSense driver offers it (patched,
//  see Ariadna_V2/patches/), and compiles to `return false` when it does
//  not, so the firmware builds against either library.
// ---------------------------------------------------------------------
#ifdef __cplusplus
template <typename IMU, typename Fn>
static inline auto vib_install_tap(IMU &imu, Fn fn, int)
    -> decltype(imu.setSampleTap(fn, nullptr), bool())
{
    imu.setSampleTap(fn, nullptr);
    return true;
}
template <typename IMU, typename Fn>
static inline bool vib_install_tap(IMU &, Fn, long) { return false; }
#endif

#endif // ARIADNA_VIB_H
