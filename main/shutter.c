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
/* Per-frame processing (STUB — real implementation in next commit)            */
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

    /* TODO(M3.1 Task 3): full IFFT / gate / FFT / baseline-subtract pipeline.
     *
     * Implementation outline (matches hyperfi/csi/shutter.py):
     *
     * 1. Zero-pad H_freq into work_buf[0..2*n_subcarriers-1], rest = 0.
     * 2. IFFT:  dsps_fft2r_fc32(work_buf, fft_size); dsps_bit_rev_fc32(...);
     *           scale by 1/fft_size. Result → h_cir.
     * 3. Gate:  compute gate_idx from room_size_m + margin_m + spacing.
     *           Copy h_cir → h_gated; zero out h_gated[gate_idx+1 .. fft_size-gate_idx-1].
     * 4. If calibrating: h_gated → accumulate into calib_accum; if count reached
     *    target, average → baseline_cir; set has_baseline; bypass subtraction.
     * 5. If has_baseline: h_dynamic = h_gated - baseline_cir.
     *    Compute dynamic_energy_ratio; if quiet, EMA-update baseline_cir.
     *    If drift > threshold, set needs_recalibration flag.
     *    Use h_dynamic for reconstruction.
     * 6. FFT back: run fft2r on output CIR; result bins [0..n_subcarriers-1]
     *    copied to H_filtered.
     * 7. Fill metadata (gate_idx, indoor/outdoor power ratio, etc.)
     *
     * For now: passthrough (copy H_freq to H_filtered, zero metadata).
     * Callers will see no filtering, but the build links and the pipeline
     * plumbing can be integrated into main.cpp while the real algorithm
     * lands in a separate commit.
     */

    memcpy(H_filtered, H_freq,
           (size_t)state->cfg.n_subcarriers * 2 * sizeof(float));

    if (meta) {
        memset(meta, 0, sizeof(*meta));
        meta->frame_count          = state->frame_count;
        meta->baseline_active      = state->has_baseline;
        meta->calibrating          = state->calibrating;
        meta->needs_recalibration  = state->needs_recalibration;
    }

    return ESP_OK;
}
