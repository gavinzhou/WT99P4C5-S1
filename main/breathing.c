/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * HyperFi Breathing Rate Estimation — v1 (amplitude-only, Welch PSD).
 * Firmware mirror of hyperfi/tools/breathing_analyzer.py. See breathing.h and
 * docs/engineering/breathing-firmware-port-plan.md (M4.3).
 *
 * v1 is band-pass-free: the Welch in-band mask (0.1–0.5 Hz) already rejects
 * DC/drift/out-of-band, and BPM (argmax) + confidence (peak/median) are scale-
 * invariant, so no time-domain Butterworth biquad is needed and no scipy PSD
 * scaling is required — unscaled |FFT|² averaged over Hann segments suffices.
 */

#include "breathing.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_dsp.h"

static const char *TAG = "breathing";

#define PREF_BOOST 1.5f          /* PREFERRED_PEAK_BOOST */
#define OUTLIER_THR 0.4f         /* OUTLIER_THRESHOLD (voting) */

struct breathing_s {
    breathing_config_t cfg;
    int   n_sc;
    int   ring_len;              /* window_sec * resample_fs (e.g. 300) */
    int   step_samples;          /* step_sec * resample_fs (e.g. 50)    */
    int   nperseg;               /* min(ring_len, fs*20)                */

    /* 10 Hz amplitude ring per SC (PSRAM): row-major [sc][ring_len]. */
    float   *ring;               /* n_sc * ring_len */
    int      ring_head;          /* next write index (circular)         */
    int      ring_count;         /* valid samples (<= ring_len)         */

    /* time-based decimation 300 fps -> 10 Hz */
    double  *accum;              /* n_sc running sum */
    int      accum_n;
    uint64_t next_emit_us;       /* push a 10 Hz sample when ts crosses */
    bool     have_clock;
    int      samples_since_analyze;

    /* SC ranking */
    int     *top_idx;            /* n_vote selected SC indices */
    int      n_selected;
    int      windows_since_rerank;

    /* scratch (internal SRAM) */
    float   *fftbuf;             /* 2 * fft_size (interleaved complex)  */
    float   *hann;               /* nperseg */
    float   *psd;                /* fft_size/2 + 1 */
    float   *sigtmp;             /* ring_len (one SC time series)       */

    /* result snapshot */
    portMUX_TYPE   lock;
    breathing_result_t res;

    bool inited;
};

/* ----------------------------- small helpers ----------------------------- */

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median_f(const float *a, int n)
{
    if (n <= 0) return 0.0f;
    float *t = malloc(n * sizeof(float));
    if (!t) return 0.0f;
    memcpy(t, a, n * sizeof(float));
    qsort(t, n, sizeof(float), cmp_float);
    float m = (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
    free(t);
    return m;
}

static void make_hann(float *w, int n)
{
    for (int i = 0; i < n; i++)
        w[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
}

/* ------------------------- Welch PSD core estimator -----------------------
 * Computes one-sided |FFT|² averaged over Hann segments (nperseg/noverlap),
 * zero-padded to fft_size. Then band-mask -> find_peaks -> harmonic ->
 * preferred-band boost -> confidence. Mirrors gen_breathing_fixtures.py.
 * Scratch buffers must be: fftbuf[2*fft_size], hann[nperseg], psd[fft_size/2+1].
 * Returns BPM, writes confidence (selected_psd/median) to *out_conf.            */
static float welch_core(const breathing_config_t *cfg, const float *sig, int n,
                        int nperseg, float *fftbuf, float *hann, float *psd,
                        float *out_conf)
{
    const int   F   = cfg->fft_size;
    const int   half = F / 2;
    const float fs  = cfg->resample_fs;
    const float df  = fs / F;

    if (out_conf) *out_conf = 0.0f;
    if (n < 16) return 0.0f;
    int nseg = (nperseg > n) ? n : nperseg;
    int noverlap = nseg / 2;
    int step = nseg - noverlap;
    if (step < 1) step = 1;

    for (int k = 0; k <= half; k++) psd[k] = 0.0f;
    int num = 0;
    for (int s = 0; s + nseg <= n; s += step) {
        /* load windowed segment into complex buffer, zero-pad to F */
        for (int i = 0; i < F; i++) { fftbuf[2 * i] = 0.0f; fftbuf[2 * i + 1] = 0.0f; }
        for (int i = 0; i < nseg; i++) fftbuf[2 * i] = sig[s + i] * hann[i];
        dsps_fft2r_fc32(fftbuf, F);
        dsps_bit_rev_fc32(fftbuf, F);
        for (int k = 0; k <= half; k++) {
            float re = fftbuf[2 * k], im = fftbuf[2 * k + 1];
            psd[k] += re * re + im * im;
        }
        num++;
    }
    if (num == 0) {  /* signal shorter than one segment: single padded FFT */
        for (int i = 0; i < F; i++) { fftbuf[2 * i] = 0.0f; fftbuf[2 * i + 1] = 0.0f; }
        for (int i = 0; i < n && i < nseg; i++) fftbuf[2 * i] = sig[i] * hann[i];
        dsps_fft2r_fc32(fftbuf, F);
        dsps_bit_rev_fc32(fftbuf, F);
        for (int k = 0; k <= half; k++) {
            float re = fftbuf[2 * k], im = fftbuf[2 * k + 1];
            psd[k] = re * re + im * im;
        }
        num = 1;
    } else {
        for (int k = 0; k <= half; k++) psd[k] /= num;
    }

    /* band mask */
    int klo = (int)ceilf(cfg->band_lo_hz / df);
    int khi = (int)floorf(cfg->band_hi_hz / df);
    if (klo < 1) klo = 1;
    if (khi > half) khi = half;
    if (khi <= klo) return 0.0f;
    int bn = khi - klo + 1;
    float *bp = psd + klo;               /* band psd (bp[j] = psd[klo+j]) */
    /* frequency of band index j = (klo+j)*df */

    float med = median_f(bp, bn);
    if (med <= 0.0f) med = 1e-10f;

    /* find_peaks: local maxima with prominence >= med*0.8, distance >= 2 */
    float promthr = med * 0.8f;
    int   peaks[64]; int npk = 0;
    for (int j = 1; j < bn - 1 && npk < 64; j++) {
        if (!(bp[j] > bp[j - 1] && bp[j] > bp[j + 1])) continue;
        /* prominence (scipy def): peak minus higher of the two bases */
        float lmin = bp[j];
        for (int t = j - 1; t >= 0 && bp[t] < bp[j]; t--) if (bp[t] < lmin) lmin = bp[t];
        float rmin = bp[j];
        for (int t = j + 1; t < bn && bp[t] < bp[j]; t++) if (bp[t] < rmin) rmin = bp[t];
        float prom = bp[j] - (lmin > rmin ? lmin : rmin);
        if (prom >= promthr) peaks[npk++] = j;
    }
    /* enforce distance>=2: drop lower peak when two within 2 bins */
    for (int a = 0; a < npk; a++)
        for (int b2 = a + 1; b2 < npk; b2++)
            if (peaks[a] >= 0 && peaks[b2] >= 0 &&
                abs(peaks[a] - peaks[b2]) < 2) {
                if (bp[peaks[a]] >= bp[peaks[b2]]) peaks[b2] = -1; else peaks[a] = -1;
            }
    int comp = 0;
    for (int a = 0; a < npk; a++) if (peaks[a] >= 0) peaks[comp++] = peaks[a];
    npk = comp;

    int sel;
    if (npk == 0) {
        sel = 0;                          /* argmax of band */
        for (int j = 1; j < bn; j++) if (bp[j] > bp[sel]) sel = j;
        float f = (klo + sel) * df;
        if (out_conf) *out_conf = bp[sel] / med;
        return f * 60.0f;
    }

    /* global best peak */
    int gbest = peaks[0];
    for (int a = 1; a < npk; a++) if (bp[peaks[a]] > bp[gbest]) gbest = peaks[a];
    float gbest_f = (klo + gbest) * df;
    float gbest_p = bp[gbest];
    sel = gbest;

    /* harmonic: if a peak near half the global-best freq is strong, pick it */
    float half_f = gbest_f / 2.0f;
    if (half_f >= cfg->band_lo_hz) {
        /* iterate peaks in descending psd order */
        int order[64]; for (int a = 0; a < npk; a++) order[a] = peaks[a];
        /* simple selection sort by bp desc */
        for (int a = 0; a < npk; a++)
            for (int b2 = a + 1; b2 < npk; b2++)
                if (bp[order[b2]] > bp[order[a]]) { int tmp = order[a]; order[a] = order[b2]; order[b2] = tmp; }
        for (int a = 0; a < npk; a++) {
            if (order[a] == gbest) continue;
            float pf = (klo + order[a]) * df;
            if (fabsf(pf - half_f) / fmaxf(half_f, 0.01f) < 0.15f &&
                bp[order[a]] > gbest_p * 0.3f) { sel = order[a]; break; }
        }
    }

    float sel_p = bp[sel];

    /* preferred-band boost */
    int   pbest = -1;
    for (int a = 0; a < npk; a++) {
        float pf = (klo + peaks[a]) * df;
        if (pf >= cfg->pref_lo_hz && pf <= cfg->pref_hi_hz)
            if (pbest < 0 || bp[peaks[a]] > bp[pbest]) pbest = peaks[a];
    }
    if (pbest >= 0 && sel_p <= bp[pbest] * PREF_BOOST) { sel = pbest; sel_p = bp[pbest]; }

    if (out_conf) *out_conf = sel_p / med;
    return (klo + sel) * df * 60.0f;
}

/* public test wrapper: allocate scratch, run estimator on a 10 Hz signal */
float breathing_estimate_welch(const breathing_config_t *cfg,
                              const float *sig, int n, float *out_conf)
{
    int F = cfg->fft_size;
    int nperseg = (int)(cfg->resample_fs * 20.0f);
    if (nperseg > n) nperseg = n;
    if (nperseg < 16) nperseg = (n < 16) ? n : 16;
    float *fftbuf = malloc(sizeof(float) * 2 * F);
    float *hann   = malloc(sizeof(float) * nperseg);
    float *psd    = malloc(sizeof(float) * (F / 2 + 1));
    if (!fftbuf || !hann || !psd) { free(fftbuf); free(hann); free(psd); if (out_conf) *out_conf = 0; return 0; }
    /* Force a clean twiddle table for F: the global esp-dsp table may have been
     * sized for a different N by shutter (128); init is a no-op if a table
     * already exists, leaving the wrong size -> OOB twiddles -> NaN bins. */
    dsps_fft2r_deinit_fc32();
    dsps_fft2r_init_fc32(NULL, F);
    make_hann(hann, nperseg);
    float bpm = welch_core(cfg, sig, n, nperseg, fftbuf, hann, psd, out_conf);
    free(fftbuf); free(hann); free(psd);
    return bpm;
}

/* ---------------------------- voting (aggregate) -------------------------- */

static float aggregate(const float *bpm, const float *conf, int m, float *out_conf)
{
    if (m == 0) { if (out_conf) *out_conf = 0; return 0; }
    float med = median_f(bpm, m);
    if (med <= 0) { if (out_conf) *out_conf = 0; return 0; }
    float wsum = 0, vsum = 0, cmax = 0; int inl = 0;
    for (int i = 0; i < m; i++) {
        if (fabsf(bpm[i] - med) / med <= OUTLIER_THR) {
            wsum += conf[i]; vsum += conf[i] * bpm[i];
            if (conf[i] > cmax) cmax = conf[i];
            inl++;
        }
    }
    if (inl == 0 || wsum <= 0) {           /* all outliers: weighted over all */
        wsum = 0; vsum = 0; cmax = 0;
        for (int i = 0; i < m; i++) { wsum += conf[i]; vsum += conf[i] * bpm[i]; if (conf[i] > cmax) cmax = conf[i]; }
    }
    if (out_conf) *out_conf = cmax;
    return (wsum > 0) ? vsum / wsum : med;
}

/* ----------------------------- streaming path ---------------------------- */

static void extract_sc(breathing_t *b, int sc, float *out)
{
    /* copy this SC's ring into time order (oldest -> newest), length ring_count */
    int start = (b->ring_count < b->ring_len)
                ? 0
                : b->ring_head;                /* if full, oldest is at head */
    float *row = b->ring + (size_t)sc * b->ring_len;
    for (int i = 0; i < b->ring_count; i++)
        out[i] = row[(start + i) % b->ring_len];
}

static float band_energy(breathing_t *b, const float *sig, int n)
{
    /* in-band PSD energy for SC ranking (reuse welch_core PSD via conf-less call) */
    float conf; (void)conf;
    /* cheap proxy: variance of the welch peak is not what we want; use time-domain
     * variance after removing mean — out-of-band drift dominates? No: use band PSD
     * sum. We approximate by running welch_core and reading the returned confidence
     * (peak/median) as a sensitivity score — higher = stronger in-band tone. */
    float c = 0;
    welch_core(&b->cfg, sig, n, b->nperseg, b->fftbuf, b->hann, b->psd, &c);
    return c;
}

static void rerank_sc(breathing_t *b)
{
    /* rank all SC by in-band sensitivity (welch peak/median), keep top n_vote */
    int nv = b->cfg.n_vote;
    float best_score[16]; int best_idx[16];
    for (int i = 0; i < nv && i < 16; i++) { best_score[i] = -1; best_idx[i] = -1; }
    for (int sc = 0; sc < b->n_sc; sc++) {
        extract_sc(b, sc, b->sigtmp);
        float score = band_energy(b, b->sigtmp, b->ring_count);
        /* insert into top-nv */
        int minp = 0;
        for (int i = 1; i < nv; i++) if (best_score[i] < best_score[minp]) minp = i;
        if (score > best_score[minp]) { best_score[minp] = score; best_idx[minp] = sc; }
    }
    b->n_selected = 0;
    for (int i = 0; i < nv; i++) if (best_idx[i] >= 0) b->top_idx[b->n_selected++] = best_idx[i];
}

static void analyze(breathing_t *b)
{
    if (b->ring_count < b->nperseg) return;          /* not enough yet */
    if (b->n_selected == 0 || b->windows_since_rerank >= 6) {  /* re-rank ~every 30s */
        rerank_sc(b);
        b->windows_since_rerank = 0;
    }
    b->windows_since_rerank++;

    float bpm[16], conf[16]; int m = 0;
    for (int i = 0; i < b->n_selected && m < 16; i++) {
        extract_sc(b, b->top_idx[i], b->sigtmp);
        float c;
        float v = welch_core(&b->cfg, b->sigtmp, b->ring_count, b->nperseg,
                             b->fftbuf, b->hann, b->psd, &c);
        if (v > 0) { bpm[m] = v; conf[m] = c; m++; }
    }
    float agg_conf;
    float agg_bpm = aggregate(bpm, conf, m, &agg_conf);

    breathing_state_e st;
    if (b->ring_count < b->ring_len)        st = BREATHING_WARMUP;
    else if (agg_conf < b->cfg.min_confidence || agg_bpm <= 0) st = BREATHING_LOST;
    else                                    st = BREATHING_TRACKING;

    portENTER_CRITICAL(&b->lock);
    b->res.state = st;
    if (st == BREATHING_TRACKING) { b->res.bpm = agg_bpm; b->res.confidence = agg_conf; }
    b->res.window_count++;
    b->res.last_update_us = esp_timer_get_time();
    portEXIT_CRITICAL(&b->lock);
}

void breathing_on_frame(breathing_t *b, const float *amp, const float *phase,
                       uint64_t ts_us)
{
    (void)phase;                                     /* v1: amplitude only */
    if (!b || !b->inited || !amp) return;

    if (!b->have_clock) {
        b->have_clock = true;
        b->next_emit_us = ts_us + (uint64_t)(1e6f / b->cfg.resample_fs);
    }
    for (int sc = 0; sc < b->n_sc; sc++) b->accum[sc] += amp[sc];
    b->accum_n++;

    if (ts_us < b->next_emit_us) return;             /* still accumulating */

    /* push one 10 Hz sample (mean of accumulated frames) */
    float inv = (b->accum_n > 0) ? 1.0f / b->accum_n : 0.0f;
    for (int sc = 0; sc < b->n_sc; sc++) {
        b->ring[(size_t)sc * b->ring_len + b->ring_head] = (float)(b->accum[sc] * inv);
        b->accum[sc] = 0.0;
    }
    b->accum_n = 0;
    b->ring_head = (b->ring_head + 1) % b->ring_len;
    if (b->ring_count < b->ring_len) b->ring_count++;
    b->next_emit_us += (uint64_t)(1e6f / b->cfg.resample_fs);
    b->samples_since_analyze++;

    if (b->samples_since_analyze >= b->step_samples) {
        b->samples_since_analyze = 0;
        analyze(b);
    }
}

void breathing_get(const breathing_t *b, breathing_result_t *out)
{
    if (!b || !out) return;
    portENTER_CRITICAL((portMUX_TYPE *)&b->lock);
    *out = b->res;
    portEXIT_CRITICAL((portMUX_TYPE *)&b->lock);
}

esp_err_t breathing_init(breathing_t **out_state, const breathing_config_t *cfg)
{
    if (!out_state || !cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->n_subcarriers <= 0 || cfg->fft_size < 16 ||
        (cfg->fft_size & (cfg->fft_size - 1)) != 0) return ESP_ERR_INVALID_ARG;

    breathing_t *b = heap_caps_calloc(1, sizeof(*b), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!b) return ESP_ERR_NO_MEM;
    b->cfg = *cfg;
    b->n_sc = cfg->n_subcarriers;
    b->ring_len = (int)(cfg->window_sec * cfg->resample_fs + 0.5f);
    b->step_samples = (int)(cfg->step_sec * cfg->resample_fs + 0.5f);
    if (b->step_samples < 1) b->step_samples = 1;
    b->nperseg = (int)(cfg->resample_fs * 20.0f);
    if (b->nperseg > b->ring_len) b->nperseg = b->ring_len;
    if (b->nperseg < 16) b->nperseg = 16;
    b->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    b->ring   = heap_caps_calloc((size_t)b->n_sc * b->ring_len, sizeof(float), MALLOC_CAP_SPIRAM);
    b->accum  = heap_caps_calloc(b->n_sc, sizeof(double), MALLOC_CAP_SPIRAM);
    b->sigtmp = heap_caps_calloc(b->ring_len, sizeof(float), MALLOC_CAP_SPIRAM);
    b->top_idx = heap_caps_calloc(cfg->n_vote, sizeof(int), MALLOC_CAP_SPIRAM);
    b->fftbuf = heap_caps_aligned_alloc(16, sizeof(float) * 2 * cfg->fft_size, MALLOC_CAP_8BIT);
    b->hann   = heap_caps_calloc(b->nperseg, sizeof(float), MALLOC_CAP_8BIT);
    b->psd    = heap_caps_calloc(cfg->fft_size / 2 + 1, sizeof(float), MALLOC_CAP_8BIT);
    if (!b->ring || !b->accum || !b->sigtmp || !b->top_idx || !b->fftbuf || !b->hann || !b->psd) {
        breathing_deinit(b);
        return ESP_ERR_NO_MEM;
    }
    make_hann(b->hann, b->nperseg);
    dsps_fft2r_deinit_fc32();              /* clean table for our fft_size */
    dsps_fft2r_init_fc32(NULL, cfg->fft_size);

    b->res.state = BREATHING_COLD;
    b->inited = true;
    *out_state = b;
    ESP_LOGI(TAG, "init: n_sc=%d ring=%d nperseg=%d fft=%d step=%d",
             b->n_sc, b->ring_len, b->nperseg, cfg->fft_size, b->step_samples);
    return ESP_OK;
}

void breathing_deinit(breathing_t *b)
{
    if (!b) return;
    heap_caps_free(b->ring);
    heap_caps_free(b->accum);
    heap_caps_free(b->sigtmp);
    heap_caps_free(b->top_idx);
    heap_caps_free(b->fftbuf);
    heap_caps_free(b->hann);
    heap_caps_free(b->psd);
    heap_caps_free(b);
}
