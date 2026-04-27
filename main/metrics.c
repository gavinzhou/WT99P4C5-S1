/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Metrics + quiet detector implementation — see metrics.h.
 * Direct port of hyperfi/csi/metrics.py per ADR-023 M3.3.
 */

#include "metrics.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "metrics";

/* -------------------------------------------------------------------------- */
/* compute_metrics — direct port of metrics.py:19-62                            */
/* -------------------------------------------------------------------------- */

esp_err_t metrics_compute(
    const float *amp_matrix,
    int n_frames,
    int n_sc,
    metrics_result_t *result)
{
    if (!amp_matrix || !result || n_frames < 2 || n_sc < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Working buffers in PSRAM. Each ≤ ~16 KB at 53 SC × 800 frame; fine. */
    /* mean_per_sc[n_sc], std_per_sc[n_sc], norm_matrix[n_frames * n_sc],
     * frame_means[n_frames] */
    float *mean_per_sc = heap_caps_calloc(n_sc, sizeof(float),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *var_per_sc  = heap_caps_calloc(n_sc, sizeof(float),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *frame_means = heap_caps_calloc(n_frames, sizeof(float),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *norm_matrix = heap_caps_calloc((size_t)n_frames * n_sc, sizeof(float),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mean_per_sc || !var_per_sc || !frame_means || !norm_matrix) {
        ESP_LOGE(TAG, "alloc failed");
        heap_caps_free(mean_per_sc);
        heap_caps_free(var_per_sc);
        heap_caps_free(frame_means);
        heap_caps_free(norm_matrix);
        return ESP_ERR_NO_MEM;
    }

    /* ---- 1. mean_per_sc[k] = (1/N) Σ_i amp_matrix[i,k] ---- */
    for (int k = 0; k < n_sc; k++) mean_per_sc[k] = 0.0f;
    for (int i = 0; i < n_frames; i++) {
        const float *row = &amp_matrix[(size_t)i * n_sc];
        for (int k = 0; k < n_sc; k++) {
            mean_per_sc[k] += row[k];
        }
    }
    const float inv_n = 1.0f / (float)n_frames;
    for (int k = 0; k < n_sc; k++) mean_per_sc[k] *= inv_n;

    /* ---- 2. std_per_sc[k] = sqrt((1/N) Σ_i (amp[i,k] - mean[k])²) ---- */
    /* numpy default ddof=0 → use population variance (1/N), not 1/(N-1) */
    for (int k = 0; k < n_sc; k++) var_per_sc[k] = 0.0f;
    for (int i = 0; i < n_frames; i++) {
        const float *row = &amp_matrix[(size_t)i * n_sc];
        for (int k = 0; k < n_sc; k++) {
            const float d = row[k] - mean_per_sc[k];
            var_per_sc[k] += d * d;
        }
    }
    /* var_per_sc now holds Σ (x-mean)²; convert to population variance */
    for (int k = 0; k < n_sc; k++) var_per_sc[k] *= inv_n;

    /* ---- 3. mask: mean > 3.0 (metrics.py:35), then raw_cv ---- */
    int n_valid = 0;
    float sum_std_valid  = 0.0f;
    float sum_mean_valid = 0.0f;
    for (int k = 0; k < n_sc; k++) {
        if (mean_per_sc[k] > METRICS_NEAR_ZERO_THRESHOLD) {
            sum_std_valid  += sqrtf(var_per_sc[k]);
            sum_mean_valid += mean_per_sc[k];
            n_valid++;
        }
    }

    if (n_valid == 0) {
        /* Match metrics.py:35-36 — return zeros */
        result->raw_cv = 0.0f;
        result->norm_cv = 0.0f;
        result->shape_corr = 0.0f;
        result->n_frames = n_frames;
        result->n_valid_subcarriers = 0;
        heap_caps_free(mean_per_sc);
        heap_caps_free(var_per_sc);
        heap_caps_free(frame_means);
        heap_caps_free(norm_matrix);
        return ESP_OK;
    }

    const float mean_std_valid  = sum_std_valid  / (float)n_valid;
    const float mean_mean_valid = sum_mean_valid / (float)n_valid;
    const float raw_cv = mean_std_valid / (mean_mean_valid + METRICS_EPSILON);

    /* ---- 4. AGC-removed: norm_matrix[i,k] = amp[i,k] / frame_means[i] ---- */
    /* frame_means[i] = mean over k of amp_matrix[i,:] */
    for (int i = 0; i < n_frames; i++) {
        const float *row = &amp_matrix[(size_t)i * n_sc];
        float sum = 0.0f;
        for (int k = 0; k < n_sc; k++) sum += row[k];
        float fm = sum / (float)n_sc;
        if (fm < METRICS_EPSILON) fm = METRICS_EPSILON;  /* metrics.py:43 */
        frame_means[i] = fm;
    }
    for (int i = 0; i < n_frames; i++) {
        const float *src = &amp_matrix[(size_t)i * n_sc];
        float *dst = &norm_matrix[(size_t)i * n_sc];
        const float inv_fm = 1.0f / frame_means[i];
        for (int k = 0; k < n_sc; k++) dst[k] = src[k] * inv_fm;
    }

    /* ---- 5. norm_mean[k_valid], norm_std[k_valid] ---- */
    /* Compute over MASKED subcarriers only. */
    float sum_norm_mean = 0.0f;
    float sum_norm_std  = 0.0f;
    for (int k = 0; k < n_sc; k++) {
        if (mean_per_sc[k] <= METRICS_NEAR_ZERO_THRESHOLD) continue;
        float nm_sum = 0.0f;
        for (int i = 0; i < n_frames; i++) {
            nm_sum += norm_matrix[(size_t)i * n_sc + k];
        }
        const float nm_mean = nm_sum * inv_n;
        float nm_var = 0.0f;
        for (int i = 0; i < n_frames; i++) {
            const float d = norm_matrix[(size_t)i * n_sc + k] - nm_mean;
            nm_var += d * d;
        }
        nm_var *= inv_n;
        sum_norm_mean += nm_mean;
        sum_norm_std  += sqrtf(nm_var);
    }
    const float mean_norm_std  = sum_norm_std  / (float)n_valid;
    const float mean_norm_mean = sum_norm_mean / (float)n_valid;
    const float norm_cv = mean_norm_std / (mean_norm_mean + METRICS_EPSILON);

    /* ---- 6. Shape correlation: mean of corrcoef(norm_matrix[i], norm_matrix[i+1]) ---- */
    /* np.corrcoef([a,b])[0,1] = cov(a,b) / (std(a) * std(b))
     * Computed across ALL subcarriers (not masked, to match metrics.py:51-55). */
    int n_valid_corrs = 0;
    float corr_sum = 0.0f;
    for (int i = 0; i < n_frames - 1; i++) {
        const float *a = &norm_matrix[(size_t)i * n_sc];
        const float *b = &norm_matrix[(size_t)(i + 1) * n_sc];
        float a_sum = 0.0f, b_sum = 0.0f;
        for (int k = 0; k < n_sc; k++) { a_sum += a[k]; b_sum += b[k]; }
        const float a_mean = a_sum / (float)n_sc;
        const float b_mean = b_sum / (float)n_sc;
        float cov = 0.0f, var_a = 0.0f, var_b = 0.0f;
        for (int k = 0; k < n_sc; k++) {
            const float da = a[k] - a_mean;
            const float db = b[k] - b_mean;
            cov   += da * db;
            var_a += da * da;
            var_b += db * db;
        }
        const float denom = sqrtf(var_a * var_b);
        if (denom > METRICS_EPSILON) {
            const float c = cov / denom;
            corr_sum += c;
            n_valid_corrs++;
        }
        /* else: NaN-equivalent, skip (metrics.py:53-54) */
    }
    const float shape_corr = (n_valid_corrs > 0)
        ? corr_sum / (float)n_valid_corrs : 0.0f;

    /* ---- 7. Fill result ---- */
    result->raw_cv              = raw_cv;
    result->norm_cv             = norm_cv;
    result->shape_corr          = shape_corr;
    result->n_frames            = n_frames;
    result->n_valid_subcarriers = n_valid;

    heap_caps_free(mean_per_sc);
    heap_caps_free(var_per_sc);
    heap_caps_free(frame_means);
    heap_caps_free(norm_matrix);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Motion classifier (metrics.py:70-77)                                        */
/* -------------------------------------------------------------------------- */

metrics_motion_state_t metrics_classify_motion(float norm_cv)
{
    if (norm_cv >= METRICS_MOTION_THRESHOLD_NORM_CV) {
        return METRICS_MOTION_STATE_MOTION;
    } else if (norm_cv <= METRICS_STATIC_THRESHOLD_NORM_CV) {
        return METRICS_MOTION_STATE_STATIC;
    } else {
        return METRICS_MOTION_STATE_LIGHT_ACTIVITY;
    }
}

/* -------------------------------------------------------------------------- */
/* Dynamic gain G = sum_k Var[amp_matrix[:, k]] (P1 patent 式 8, sum aggregation) */
/* -------------------------------------------------------------------------- */

float metrics_dynamic_gain_G(const float *amp_matrix, int n_frames, int n_sc)
{
    if (!amp_matrix || n_frames < 2 || n_sc < 1) return 0.0f;

    float G = 0.0f;
    const float inv_n = 1.0f / (float)n_frames;
    for (int k = 0; k < n_sc; k++) {
        float sum = 0.0f;
        for (int i = 0; i < n_frames; i++) {
            sum += amp_matrix[(size_t)i * n_sc + k];
        }
        const float mean = sum * inv_n;
        float var = 0.0f;
        for (int i = 0; i < n_frames; i++) {
            const float d = amp_matrix[(size_t)i * n_sc + k] - mean;
            var += d * d;
        }
        var *= inv_n;
        G += var;
    }
    return G;
}

/* -------------------------------------------------------------------------- */
/* Quiet detector                                                              */
/* -------------------------------------------------------------------------- */

void quiet_detector_init(quiet_detector_t *q, const quiet_detector_config_t *cfg)
{
    if (!q) return;
    if (cfg) {
        q->cfg = *cfg;
    } else {
        const quiet_detector_config_t def = QUIET_DETECTOR_CONFIG_DEFAULT();
        q->cfg = def;
    }
    quiet_detector_reset(q);
}

void quiet_detector_reset(quiet_detector_t *q)
{
    if (!q) return;
    q->is_quiet = false;
    q->below_threshold_start_us = 0;
    q->last_update_us = 0;
}

void quiet_detector_update(
    quiet_detector_t *q,
    float norm_cv,
    uint64_t timestamp_us,
    bool *out_is_quiet,
    float *out_duration_sec)
{
    if (!q) return;

    if (norm_cv < q->cfg.static_threshold) {
        if (q->below_threshold_start_us == 0) {
            q->below_threshold_start_us = timestamp_us;
        }
        const uint64_t persist_us = (uint64_t)(q->cfg.persist_sec * 1.0e6f);
        if (timestamp_us - q->below_threshold_start_us >= persist_us) {
            q->is_quiet = true;
        }
    } else {
        q->below_threshold_start_us = 0;
        q->is_quiet = false;
    }
    q->last_update_us = timestamp_us;

    if (out_is_quiet) *out_is_quiet = q->is_quiet;
    if (out_duration_sec) {
        *out_duration_sec = (q->below_threshold_start_us == 0)
            ? 0.0f
            : (float)(timestamp_us - q->below_threshold_start_us) / 1.0e6f;
    }
}
