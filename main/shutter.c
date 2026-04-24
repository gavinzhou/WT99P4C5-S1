/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * HyperFi Spatial Shutter implementation — SKELETON
 *
 * ADR-023 M3.1 — see shutter.h for public API.
 *
 * Status: 2026-04-24 — stub implementations; real algorithm filled in
 * milestone M3.1 Task 3 after ESP-DSP dependency is verified to resolve
 * and link clean in CI build.
 */

#include "shutter.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_dsp.h"   /* dsps_fft2r_*, dsps_bit_rev_fc32, etc. */

static const char *TAG = "shutter";

/* Physical constants */
#define SPEED_OF_LIGHT_MPS  (299792458.0f)

/* -------------------------------------------------------------------------- */
/* Subcarrier spacing table (Hz)                                               */
/* -------------------------------------------------------------------------- */

static const float s_subcarrier_spacing_hz[] = {
    [SHUTTER_FRAME_HT20] = 312500.0f,     /* 312.5 kHz */
    [SHUTTER_FRAME_HT40] = 312500.0f,     /* 312.5 kHz */
    [SHUTTER_FRAME_VHT]  = 312500.0f,     /* 312.5 kHz */
    [SHUTTER_FRAME_HE]   =  78125.0f,     /* 78.125 kHz */
};

/* -------------------------------------------------------------------------- */
/* Internal state                                                              */
/* -------------------------------------------------------------------------- */

struct shutter_state_s {
    shutter_config_t cfg;

    /* Work buffers, allocated in PSRAM at init, sized to cfg.fft_size.
     * All are float[2 * fft_size] interleaved re/im. */
    float *work_buf;        /* scratch for FFT input/output */
    float *h_cir;            /* CIR after IFFT (gated input) */
    float *h_gated;          /* CIR after gating */
    float *h_dynamic;        /* CIR after baseline subtraction */
    float *baseline_cir;     /* adaptive baseline CIR (EMA-updated) */

    /* Calibration buffer: accumulator for baseline averaging.
     * Size fft_size complex (2 * fft_size floats). */
    float *calib_accum;
    int    calib_target;
    int    calib_count;
    bool   calibrating;
    bool   has_baseline;

    /* Drift / recalibration tracking */
    bool   needs_recalibration;

    /* Frame counter */
    uint32_t frame_count;
};

/* -------------------------------------------------------------------------- */
/* Alloc helper                                                                */
/* -------------------------------------------------------------------------- */

static float *alloc_cplx_buf(int fft_size)
{
    /* Complex = 2 floats per element. Alloc in PSRAM-preferred, 16-byte
     * aligned for DSP. */
    size_t bytes = (size_t)fft_size * 2 * sizeof(float);
    return (float *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static void free_cplx_buf(float *buf)
{
    if (buf) {
        heap_caps_free(buf);
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t shutter_init(shutter_state_t **out_state, const shutter_config_t *cfg)
{
    if (!out_state || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->n_subcarriers <= 0 || cfg->fft_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->fft_size < cfg->n_subcarriers) {
        ESP_LOGE(TAG, "fft_size (%d) < n_subcarriers (%d)",
                 cfg->fft_size, cfg->n_subcarriers);
        return ESP_ERR_INVALID_ARG;
    }
    /* fft_size must be power of two for radix-2 ESP-DSP FFT */
    if ((cfg->fft_size & (cfg->fft_size - 1)) != 0) {
        ESP_LOGE(TAG, "fft_size %d is not power-of-two", cfg->fft_size);
        return ESP_ERR_INVALID_ARG;
    }

    shutter_state_t *s = heap_caps_calloc(1, sizeof(*s),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s) return ESP_ERR_NO_MEM;

    s->cfg = *cfg;

    s->work_buf     = alloc_cplx_buf(cfg->fft_size);
    s->h_cir        = alloc_cplx_buf(cfg->fft_size);
    s->h_gated      = alloc_cplx_buf(cfg->fft_size);
    s->h_dynamic    = alloc_cplx_buf(cfg->fft_size);
    s->baseline_cir = alloc_cplx_buf(cfg->fft_size);
    s->calib_accum  = alloc_cplx_buf(cfg->fft_size);

    if (!s->work_buf || !s->h_cir || !s->h_gated || !s->h_dynamic ||
        !s->baseline_cir || !s->calib_accum) {
        ESP_LOGE(TAG, "PSRAM alloc failed (fft_size=%d)", cfg->fft_size);
        shutter_deinit(s);
        return ESP_ERR_NO_MEM;
    }

    /* Initialize ESP-DSP FFT twiddle table. Per ESP-DSP docs, call once
     * per fft_size. Safe to call multiple times (idempotent). */
    esp_err_t err = dsps_fft2r_init_fc32(NULL, cfg->fft_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32(%d) failed: %d", cfg->fft_size, err);
        shutter_deinit(s);
        return err;
    }

    s->has_baseline = false;
    s->calibrating  = false;
    s->frame_count  = 0;

    *out_state = s;
    ESP_LOGI(TAG, "initialized: N_sc=%d, fft_size=%d, room=%.1fm, margin=%.1fm",
             cfg->n_subcarriers, cfg->fft_size, cfg->room_size_m, cfg->margin_m);
    return ESP_OK;
}

void shutter_deinit(shutter_state_t *state)
{
    if (!state) return;
    free_cplx_buf(state->work_buf);
    free_cplx_buf(state->h_cir);
    free_cplx_buf(state->h_gated);
    free_cplx_buf(state->h_dynamic);
    free_cplx_buf(state->baseline_cir);
    free_cplx_buf(state->calib_accum);
    heap_caps_free(state);
}

esp_err_t shutter_start_calibration(shutter_state_t *state, int num_frames)
{
    if (!state) return ESP_ERR_INVALID_ARG;
    if (num_frames <= 0) return ESP_ERR_INVALID_ARG;

    memset(state->calib_accum, 0,
           (size_t)state->cfg.fft_size * 2 * sizeof(float));
    state->calib_target       = num_frames;
    state->calib_count        = 0;
    state->calibrating        = true;
    state->needs_recalibration = false;
    ESP_LOGI(TAG, "calibration started: target=%d frames", num_frames);
    return ESP_OK;
}

bool shutter_has_baseline(const shutter_state_t *state)
{
    return state && state->has_baseline;
}

void shutter_set_baseline(shutter_state_t *state, const float *baseline_cir_interleaved)
{
    if (!state || !baseline_cir_interleaved) return;
    memcpy(state->baseline_cir, baseline_cir_interleaved,
           (size_t)state->cfg.fft_size * 2 * sizeof(float));
    state->has_baseline       = true;
    state->needs_recalibration = false;
    ESP_LOGI(TAG, "baseline set externally");
}

void shutter_clear_baseline(shutter_state_t *state)
{
    if (!state) return;
    memset(state->baseline_cir, 0,
           (size_t)state->cfg.fft_size * 2 * sizeof(float));
    state->has_baseline = false;
    state->calibrating  = false;
    ESP_LOGI(TAG, "baseline cleared");
}

/* -------------------------------------------------------------------------- */
/* IFFT helper — uses dsps_fft2r_fc32 via conjugate trick                       */
/* -------------------------------------------------------------------------- */
/*
 * ESP-DSP only ships forward FFT (dsps_fft2r_fc32 + dsps_bit_rev_fc32).
 * IFFT can be computed as:
 *     ifft(X) = (1/N) · conj(fft(conj(X)))
 * which means:
 *   1. Negate imag part of input
 *   2. Run forward FFT (+ bit reversal)
 *   3. Negate imag part of output, divide by N
 *
 * Input/output buffer is interleaved complex float[2*N]. Operates in-place.
 */
static void ifft_inplace(float *cplx, int N)
{
    const float inv_N = 1.0f / (float)N;
    /* conj input */
    for (int i = 0; i < N; i++) {
        cplx[2 * i + 1] = -cplx[2 * i + 1];
    }
    /* forward FFT + bit-reverse */
    dsps_fft2r_fc32(cplx, N);
    dsps_bit_rev_fc32(cplx, N);
    /* conj output + scale by 1/N */
    for (int i = 0; i < N; i++) {
        cplx[2 * i]     *=  inv_N;
        cplx[2 * i + 1] *= -inv_N;
    }
}

static void fft_inplace(float *cplx, int N)
{
    dsps_fft2r_fc32(cplx, N);
    dsps_bit_rev_fc32(cplx, N);
}

/* -------------------------------------------------------------------------- */
/* Per-frame processing — real algorithm per hyperfi/csi/shutter.py            */
/* -------------------------------------------------------------------------- */

esp_err_t shutter_process(
    shutter_state_t *state,
    const float *H_freq,
    shutter_frame_type_t frame_type,
    float *H_filtered,
    shutter_metadata_t *meta)
{
    if (!state || !H_freq || !H_filtered) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((int)frame_type < 0 ||
        (int)frame_type >= (int)(sizeof(s_subcarrier_spacing_hz) / sizeof(s_subcarrier_spacing_hz[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    state->frame_count++;

    const int N_fft = state->cfg.fft_size;
    const int N_sc  = state->cfg.n_subcarriers;
    const float spacing = s_subcarrier_spacing_hz[frame_type];

    /* ---- 1. Zero-pad H_freq into work_buf, IFFT → h_cir ---- */
    memset(state->work_buf, 0, (size_t)N_fft * 2 * sizeof(float));
    memcpy(state->work_buf, H_freq, (size_t)N_sc * 2 * sizeof(float));
    ifft_inplace(state->work_buf, N_fft);
    memcpy(state->h_cir, state->work_buf, (size_t)N_fft * 2 * sizeof(float));

    /* ---- 2. Compute gate_idx from room size + margin ---- */
    const float max_distance = state->cfg.room_size_m + state->cfg.margin_m;
    /* Round-trip delay: signal travels to reflector and back */
    const float tau_threshold = 2.0f * max_distance / SPEED_OF_LIGHT_MPS;
    const float bin_time = 1.0f / ((float)N_fft * spacing);
    int gate_idx = (int)(tau_threshold / bin_time);
    if (gate_idx > N_fft / 2) gate_idx = N_fft / 2;

    /* ---- 3. Gate: copy h_cir → h_gated, zero out middle bins ---- */
    memcpy(state->h_gated, state->h_cir, (size_t)N_fft * 2 * sizeof(float));
    if (gate_idx < N_fft / 2) {
        const int zero_start = gate_idx + 1;          /* first bin to zero */
        const int zero_end   = N_fft - gate_idx;      /* one past last bin to zero */
        memset(&state->h_gated[2 * zero_start], 0,
               (size_t)(zero_end - zero_start) * 2 * sizeof(float));
    }

    /* ---- 4. Calibration collection ---- */
    if (state->calibrating) {
        for (int i = 0; i < 2 * N_fft; i++) {
            state->calib_accum[i] += state->h_gated[i];
        }
        state->calib_count++;
        if (state->calib_count >= state->calib_target) {
            const float inv = 1.0f / (float)state->calib_target;
            for (int i = 0; i < 2 * N_fft; i++) {
                state->baseline_cir[i] = state->calib_accum[i] * inv;
            }
            state->calibrating  = false;
            state->has_baseline = true;
            ESP_LOGI(TAG, "calibration complete (%d frames averaged)", state->calib_target);
        }
    }

    /* ---- 5. Baseline subtraction (gated → dynamic) ---- */
    float dynamic_energy_ratio = 0.0f;
    float drift_score          = 0.0f;
    bool  is_quiet             = false;
    bool  baseline_active      = false;
    float *out_cir             = state->h_gated;   /* default output if no baseline */

    if (state->has_baseline && !state->calibrating) {
        baseline_active = true;

        /* h_dynamic = h_gated - baseline_cir */
        for (int i = 0; i < 2 * N_fft; i++) {
            state->h_dynamic[i] = state->h_gated[i] - state->baseline_cir[i];
        }

        /* Power ratios */
        float baseline_power = 0.0f;
        float dynamic_power  = 0.0f;
        for (int i = 0; i < N_fft; i++) {
            const float br = state->baseline_cir[2 * i];
            const float bi = state->baseline_cir[2 * i + 1];
            const float dr = state->h_dynamic[2 * i];
            const float di = state->h_dynamic[2 * i + 1];
            baseline_power += br * br + bi * bi;
            dynamic_power  += dr * dr + di * di;
        }
        dynamic_energy_ratio = dynamic_power / (baseline_power + 1e-10f);
        is_quiet = (dynamic_energy_ratio < state->cfg.quiet_energy_threshold);

        if (is_quiet) {
            /* EMA update of baseline */
            const float a = state->cfg.baseline_ema_alpha;
            const float one_minus_a = 1.0f - a;
            for (int i = 0; i < 2 * N_fft; i++) {
                state->baseline_cir[i] =
                    a * state->h_gated[i] + one_minus_a * state->baseline_cir[i];
            }
            drift_score = dynamic_energy_ratio;
            if (drift_score > state->cfg.drift_threshold) {
                state->needs_recalibration = true;
            }
        }

        out_cir = state->h_dynamic;
    }

    /* ---- 6. Forward FFT of out_cir → H_filtered ---- */
    memcpy(state->work_buf, out_cir, (size_t)N_fft * 2 * sizeof(float));
    fft_inplace(state->work_buf, N_fft);
    memcpy(H_filtered, state->work_buf, (size_t)N_sc * 2 * sizeof(float));

    /* ---- 7. Metadata: indoor/outdoor power ratio from h_cir ---- */
    float indoor_power  = 0.0f;
    float outdoor_power = 0.0f;
    for (int i = 0; i <= gate_idx; i++) {
        const float r = state->h_cir[2 * i];
        const float im = state->h_cir[2 * i + 1];
        indoor_power += r * r + im * im;
    }
    for (int i = gate_idx + 1; i < N_fft / 2; i++) {
        const float r = state->h_cir[2 * i];
        const float im = state->h_cir[2 * i + 1];
        outdoor_power += r * r + im * im;
    }

    if (meta) {
        meta->frame_count          = state->frame_count;
        meta->gate_idx             = gate_idx;
        meta->indoor_power_ratio   = indoor_power / (indoor_power + outdoor_power + 1e-10f);
        meta->snr_improvement_db   = 10.0f *
            log10f((indoor_power + 1e-10f) / (outdoor_power + 1e-10f));
        meta->baseline_active      = baseline_active;
        meta->calibrating          = state->calibrating;
        meta->calibration_progress = state->calibrating && state->calib_target > 0
            ? (float)state->calib_count / (float)state->calib_target
            : 0.0f;
        meta->dynamic_energy_ratio = dynamic_energy_ratio;
        meta->drift_score          = drift_score;
        meta->needs_recalibration  = state->needs_recalibration;
        meta->is_quiet             = is_quiet;
    }

    return ESP_OK;
}
