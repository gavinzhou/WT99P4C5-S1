/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Collapse Index implementation — see collapse.h.
 * Port of hyperfi/poincare/collapse.py per ADR-023 M3.2.
 */

#include "collapse.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "poincare.h"

static const char *TAG = "collapse";

/* -------------------------------------------------------------------------- */
/* Internal state                                                              */
/* -------------------------------------------------------------------------- */

#define COLLAPSE_MAX_EMBED_DIM 16  /* upper bound on n_embed_dim */

struct collapse_handle_s {
    collapse_config_t cfg;

    bool     has_ema;
    float    ema_embedding[COLLAPSE_MAX_EMBED_DIM];

    bool     has_prev;
    float    prev_embedding[COLLAPSE_MAX_EMBED_DIM];
    uint64_t prev_time_us;

    collapse_fsm_state_t fsm_state;
    int       silent_count;
    uint64_t  spike_time_us;

    uint32_t update_count;
};

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t collapse_init(collapse_handle_t **out_handle, const collapse_config_t *cfg)
{
    if (!out_handle || !cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->n_input_dim <= 0 || cfg->n_embed_dim <= 0) return ESP_ERR_INVALID_ARG;
    if (cfg->n_embed_dim > COLLAPSE_MAX_EMBED_DIM) {
        ESP_LOGE(TAG, "n_embed_dim %d > max %d", cfg->n_embed_dim, COLLAPSE_MAX_EMBED_DIM);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->ema_alpha < 0.0f || cfg->ema_alpha > 1.0f) return ESP_ERR_INVALID_ARG;

    collapse_handle_t *h = heap_caps_calloc(1, sizeof(*h),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!h) return ESP_ERR_NO_MEM;

    h->cfg = *cfg;
    collapse_reset(h);

    *out_handle = h;
    ESP_LOGI(TAG, "initialized: input_dim=%d embed_dim=%d scale=%.2f alpha=%.2f thr=%.3f/%.3f",
             cfg->n_input_dim, cfg->n_embed_dim, cfg->embed_scale,
             cfg->ema_alpha, cfg->collapse_threshold, cfg->silence_threshold);
    return ESP_OK;
}

void collapse_deinit(collapse_handle_t *handle)
{
    if (handle) heap_caps_free(handle);
}

void collapse_reset(collapse_handle_t *handle)
{
    if (!handle) return;
    handle->has_ema       = false;
    handle->has_prev      = false;
    handle->prev_time_us  = 0;
    handle->fsm_state     = COLLAPSE_FSM_MONITORING;
    handle->silent_count  = 0;
    handle->spike_time_us = 0;
    handle->update_count  = 0;
    /* Buffers don't need explicit zeroing; they'll be overwritten on first use.
     * has_ema / has_prev gate access. */
}

/* -------------------------------------------------------------------------- */
/* Update — one window worth of amplitude data                                  */
/* -------------------------------------------------------------------------- */

esp_err_t collapse_update(
    collapse_handle_t *h,
    const float *amp_input,
    uint64_t timestamp_us,
    collapse_result_t *result)
{
    if (!h || !amp_input || !result) return ESP_ERR_INVALID_ARG;

    h->update_count++;
    const int n_in    = h->cfg.n_input_dim;
    const int n_embed = h->cfg.n_embed_dim;

    /* ---- 1. Normalize by mean (collapse.py:95) ---- */
    float mean = 0.0f;
    for (int i = 0; i < n_in; i++) mean += amp_input[i];
    mean /= (float)n_in;
    if (mean < 1e-10f) mean = 1e-10f;
    const float inv_mean = 1.0f / mean;

    /* Scratch buffer for normalized + reduced amplitude.
     * Only the first n_embed entries are populated by select_8d (which assumes
     * a 53→8 selection; we generalize via stride if n_in matches POINCARE_INPUT_DIM,
     * else fall back to first n_embed positions). */
    float norm_amp_full[POINCARE_INPUT_DIM];
    if (n_in > POINCARE_INPUT_DIM) {
        ESP_LOGE(TAG, "n_input_dim %d > buffer cap %d", n_in, POINCARE_INPUT_DIM);
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < n_in; i++) {
        norm_amp_full[i] = amp_input[i] * inv_mean;
    }

    /* ---- 2. Dimension reduction 53 → 8 (or pass-through if matched) ---- */
    float feat[COLLAPSE_MAX_EMBED_DIM];
    if (n_in == POINCARE_INPUT_DIM && n_embed == POINCARE_EMBED_DIM) {
        poincare_select_8d(norm_amp_full, feat);
    } else {
        /* Fallback: take first n_embed entries (used for tests with smaller dims) */
        const int copy = (n_in < n_embed) ? n_in : n_embed;
        for (int i = 0; i < copy; i++) feat[i] = norm_amp_full[i];
        for (int i = copy; i < n_embed; i++) feat[i] = 0.0f;
    }

    /* ---- 3. Poincaré embed ---- */
    float raw_z[COLLAPSE_MAX_EMBED_DIM];
    poincare_embed(feat, n_embed, h->cfg.embed_scale, raw_z);

    /* ---- 4. EMA smoothing ---- */
    if (!h->has_ema) {
        memcpy(h->ema_embedding, raw_z, n_embed * sizeof(float));
        h->has_ema = true;
    } else {
        const float a = h->cfg.ema_alpha;
        const float one_minus_a = 1.0f - a;
        for (int i = 0; i < n_embed; i++) {
            h->ema_embedding[i] = a * raw_z[i] + one_minus_a * h->ema_embedding[i];
        }
    }

    /* ---- 5. Clamp EMA to stay inside Poincaré ball (collapse.py:108-110) ---- */
    float ema_norm_sq = 0.0f;
    for (int i = 0; i < n_embed; i++) {
        ema_norm_sq += h->ema_embedding[i] * h->ema_embedding[i];
    }
    float ema_norm = sqrtf(ema_norm_sq);
    if (ema_norm >= 1.0f) {
        const float k = 0.999f / ema_norm;  /* matches collapse.py:110 (×0.999) */
        for (int i = 0; i < n_embed; i++) h->ema_embedding[i] *= k;
        ema_norm_sq = 0.0f;
        for (int i = 0; i < n_embed; i++) {
            ema_norm_sq += h->ema_embedding[i] * h->ema_embedding[i];
        }
        ema_norm = sqrtf(ema_norm_sq);
    }

    /* ---- 6. Collapse Index = geodesic_distance(ema_z, prev_ema_z) / Δt ---- */
    float collapse_index = 0.0f;
    if (h->has_prev) {
        const uint64_t dt_us = (timestamp_us > h->prev_time_us)
                               ? timestamp_us - h->prev_time_us : 0;
        if (dt_us > 0) {
            const float d = poincare_geodesic_distance(
                h->ema_embedding, h->prev_embedding, n_embed);
            const float dt_sec = (float)dt_us / 1.0e6f;
            collapse_index = d / dt_sec;
        }
    }

    /* Save current EMA as previous for next call */
    memcpy(h->prev_embedding, h->ema_embedding, n_embed * sizeof(float));
    h->prev_time_us = timestamp_us;
    h->has_prev = true;

    /* ---- 7. State machine (collapse.py:123-145) ---- */
    bool fall_detected = false;
    switch (h->fsm_state) {
    case COLLAPSE_FSM_MONITORING:
        if (collapse_index > h->cfg.collapse_threshold) {
            h->fsm_state     = COLLAPSE_FSM_SPIKE_DETECTED;
            h->spike_time_us = timestamp_us;
            h->silent_count  = 0;
        }
        break;

    case COLLAPSE_FSM_SPIKE_DETECTED:
        if (collapse_index < h->cfg.silence_threshold) {
            h->silent_count++;
            if (h->silent_count >= h->cfg.silence_count_target) {
                fall_detected   = true;
                h->fsm_state    = COLLAPSE_FSM_MONITORING;
                h->silent_count = 0;
            }
        } else {
            h->silent_count = 0;
            const uint64_t since_spike_us = timestamp_us - h->spike_time_us;
            const float since_sec = (float)since_spike_us / 1.0e6f;
            if (since_sec > h->cfg.spike_timeout_sec) {
                /* Timeout — was not a fall */
                h->fsm_state = COLLAPSE_FSM_MONITORING;
            }
        }
        break;
    }

    /* ---- 8. Fill result ---- */
    result->collapse_index = collapse_index;
    result->state          = h->fsm_state;
    result->fall_detected  = fall_detected;
    result->embedding_norm = ema_norm;

    return ESP_OK;
}
