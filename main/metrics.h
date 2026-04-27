/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Per-window CSI metrics + 静穏期間 (quiet period) detector + 動的ゲイン G
 *
 * Port of hyperfi/csi/metrics.py, with quiet-period state tracking and
 * G estimation for the Poincaré 原点調整 path. ADR-023 M3.3.
 *
 * Validated thresholds (metrics.py:11-13, ESP32-C5 + S3 SoftAP @ -56 dBm):
 *   Static:  normCV ~ 0.02-0.06,  shape_corr ~ 0.99
 *   Walking: normCV ~ 0.20-0.32,  shape_corr ~ 0.93
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Compute per-window CSI metrics                                              */
/* -------------------------------------------------------------------------- */

#define METRICS_NEAR_ZERO_THRESHOLD  3.0f   /* metrics.py:35 — near-zero SC mask */
#define METRICS_EPSILON              1e-10f

#define METRICS_MOTION_THRESHOLD_NORM_CV  0.10f   /* metrics.py:66 */
#define METRICS_STATIC_THRESHOLD_NORM_CV  0.05f   /* metrics.py:67 */

typedef enum {
    METRICS_MOTION_STATE_STATIC         = 0,
    METRICS_MOTION_STATE_LIGHT_ACTIVITY = 1,
    METRICS_MOTION_STATE_MOTION         = 2,
} metrics_motion_state_t;

typedef struct {
    float raw_cv;            /* AGC-affected CV */
    float norm_cv;           /* AGC-removed CV (primary metric) */
    float shape_corr;        /* mean corrcoef of consecutive frames */
    int   n_frames;
    int   n_valid_subcarriers;
} metrics_result_t;

/**
 * Compute per-window metrics from an amplitude matrix.
 *
 * @param[in]  amp_matrix  Row-major float[n_frames * n_sc]
 * @param[in]  n_frames    Number of CSI frames in this window
 * @param[in]  n_sc        Number of subcarriers per frame (e.g., 53)
 * @param[out] result      Computed metrics
 * @return  ESP_OK on success, ESP_ERR_INVALID_ARG on bad inputs
 */
esp_err_t metrics_compute(
    const float *amp_matrix,
    int n_frames,
    int n_sc,
    metrics_result_t *result);

/**
 * Classify motion state from norm_cv (metrics.py:70-77).
 */
metrics_motion_state_t metrics_classify_motion(float norm_cv);

/**
 * Compute dynamic gain G = sum of per-subcarrier amplitude variances over a
 * window. Used by Spatial Shutter origin-adjustment / 静穏期間 baseline update
 * (P1 Claim 2 式8 — sum aggregation over the 2n components).
 *
 * @param[in]  amp_matrix  Row-major float[n_frames * n_sc]
 * @return  G value (>= 0)
 */
float metrics_dynamic_gain_G(const float *amp_matrix, int n_frames, int n_sc);

/* -------------------------------------------------------------------------- */
/* Quiet period detector — simple debouncer                                    */
/* -------------------------------------------------------------------------- */

typedef struct {
    float    static_threshold;     /* normCV below this counts as "still", typ 0.05 */
    float    persist_sec;          /* required consecutive duration, typ 10.0 */
} quiet_detector_config_t;

#define QUIET_DETECTOR_CONFIG_DEFAULT() { \
    .static_threshold = METRICS_STATIC_THRESHOLD_NORM_CV, \
    .persist_sec      = 10.0f, \
}

typedef struct {
    quiet_detector_config_t cfg;
    bool     is_quiet;                     /* current state */
    uint64_t below_threshold_start_us;     /* time when normCV first crossed below; 0 if not below */
    uint64_t last_update_us;               /* most recent update time */
} quiet_detector_t;

/**
 * Initialize quiet detector to "not quiet" with no history.
 */
void quiet_detector_init(quiet_detector_t *q, const quiet_detector_config_t *cfg);

/**
 * Reset state (clears history, sets is_quiet = false).
 */
void quiet_detector_reset(quiet_detector_t *q);

/**
 * Update detector with a new norm_cv reading.
 *
 * @param[in,out] q              Detector state
 * @param[in]     norm_cv        Current per-window normalized CV
 * @param[in]     timestamp_us   Monotonic timestamp
 * @param[out]    is_quiet       Optional, current is_quiet flag
 * @param[out]    duration_sec   Optional, time since norm_cv first crossed
 *                               below threshold (0 if currently above)
 */
void quiet_detector_update(
    quiet_detector_t *q,
    float norm_cv,
    uint64_t timestamp_us,
    bool *is_quiet,
    float *duration_sec);

#ifdef __cplusplus
}
#endif
