/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.3 self-test — see m33_test.h.
 */

#include "m33_test.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "metrics.h"

#if __has_include("m33_fixtures.h")
#  include "m33_fixtures.h"
#  define M33_HAS_FIXTURES 1
#endif

static const char *TAG = "m33_test";

#ifdef M33_HAS_FIXTURES

/* Use abs error fallback when |expected| is tiny (norm_cv on flat data, etc). */
static float rel_or_abs(float got, float expected)
{
    const float diff = fabsf(got - expected);
    const float denom = fabsf(expected);
    if (denom > 1e-4f) return diff / denom;
    return diff;
}

/* -------- 1. Per-window metrics -------- */
static int test_metrics(void)
{
    ESP_LOGI(TAG, "--- Per-window metrics (%d fixtures) ---",
             M33_METRICS_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;

    for (int i = 0; i < M33_METRICS_FIXTURE_COUNT; i++) {
        const m33_metrics_fixture_t *fx = &m33_metrics_fixtures[i];

        metrics_result_t result;
        int64_t t0 = esp_timer_get_time();
        esp_err_t err = metrics_compute(fx->amp_matrix, fx->n_frames, fx->n_sc,
                                        &result);
        int64_t dt_metrics = esp_timer_get_time() - t0;

        t0 = esp_timer_get_time();
        const float G = metrics_dynamic_gain_G(fx->amp_matrix, fx->n_frames,
                                               fx->n_sc);
        int64_t dt_G = esp_timer_get_time() - t0;
        total_us += (dt_metrics + dt_G);

        if (err != ESP_OK) {
            ESP_LOGI(TAG, "  [FAIL] %-14s  metrics_compute err=%d",
                     fx->name, err);
            fail++;
            continue;
        }

        const float raw_err   = rel_or_abs(result.raw_cv,    fx->raw_cv_expected);
        const float norm_err  = rel_or_abs(result.norm_cv,   fx->norm_cv_expected);
        const float shape_err = fabsf(result.shape_corr - fx->shape_corr_expected);
        const float G_err     = rel_or_abs(G, fx->G_expected);
        const bool nvalid_ok  = (result.n_valid_subcarriers == fx->n_valid_expected);

        const bool ok = (raw_err   < 1e-3f) &&
                        (norm_err  < 1e-3f) &&
                        (shape_err < 1e-3f) &&
                        (G_err     < 1e-3f) &&
                        nvalid_ok;
        if (ok) pass++; else fail++;

        ESP_LOGI(TAG,
                 "  [%-4s] %-14s  metrics=%lldus G=%lldus  "
                 "raw=%.4f/%.4f norm=%.4f/%.4f shape=%.4f/%.4f G=%.2f/%.2f valid=%d/%d",
                 ok ? "PASS" : "FAIL", fx->name, dt_metrics, dt_G,
                 result.raw_cv,    fx->raw_cv_expected,
                 result.norm_cv,   fx->norm_cv_expected,
                 result.shape_corr,fx->shape_corr_expected,
                 G,                fx->G_expected,
                 result.n_valid_subcarriers, fx->n_valid_expected);
        if (!ok) {
            ESP_LOGW(TAG,
                     "         err: raw=%.2e norm=%.2e shape=%.2e G=%.2e",
                     raw_err, norm_err, shape_err, G_err);
        }
    }
    ESP_LOGI(TAG, "  metrics: %d/%d PASS  avg=%.1fus", pass,
             M33_METRICS_FIXTURE_COUNT,
             (float)total_us / M33_METRICS_FIXTURE_COUNT);
    return fail;
}

/* -------- 2. Quiet detector debouncer -------- */
static int test_quiet_detector(void)
{
    ESP_LOGI(TAG, "--- Quiet detector (%d steps) ---", M33_QUIET_STEP_COUNT);

    quiet_detector_t q;
    quiet_detector_init(&q, NULL);  /* default config: 0.05 / 10s */

    int pass = 0, fail = 0;
    int64_t total_us = 0;

    for (int i = 0; i < M33_QUIET_STEP_COUNT; i++) {
        const m33_quiet_step_t *step = &m33_quiet_steps[i];

        bool got_quiet = false;
        float got_duration = 0.0f;
        int64_t t0 = esp_timer_get_time();
        quiet_detector_update(&q, step->norm_cv, step->ts_us,
                              &got_quiet, &got_duration);
        int64_t dt = esp_timer_get_time() - t0;
        total_us += dt;

        const bool quiet_ok = (got_quiet == step->is_quiet_expected);
        const float dur_err = fabsf(got_duration - step->duration_sec_expected);
        const bool dur_ok = dur_err < 1e-3f;
        const bool ok = quiet_ok && dur_ok;
        if (ok) pass++; else fail++;

        if (!ok || (i < 4) || (i >= M33_QUIET_STEP_COUNT - 4)) {
            ESP_LOGI(TAG,
                     "  [%-4s] step %2d  t=%.1fs  ncv=%.3f  "
                     "quiet=%d/%d  dur=%.2f/%.2f",
                     ok ? "PASS" : "FAIL", i,
                     (float)step->ts_us / 1.0e6f, step->norm_cv,
                     got_quiet, step->is_quiet_expected,
                     got_duration, step->duration_sec_expected);
        }
    }
    ESP_LOGI(TAG, "  quiet: %d/%d PASS  avg=%.2fus", pass,
             M33_QUIET_STEP_COUNT,
             (float)total_us / M33_QUIET_STEP_COUNT);
    return fail;
}

/* -------- 3. Motion classifier sanity (no fixtures, threshold edges) -------- */
static int test_motion_classifier(void)
{
    ESP_LOGI(TAG, "--- Motion classifier thresholds (collapse_index) ---");
    int fail = 0;
    struct {
        float        c_idx;
        metrics_motion_state_t expected;
        const char  *name;
    } cases[] = {
        { 0.000f, METRICS_MOTION_STATE_STATIC,         "C=0.000 → STATIC"        },
        { 0.010f, METRICS_MOTION_STATE_STATIC,         "C=0.010 → STATIC"        },
        { 0.015f, METRICS_MOTION_STATE_STATIC,         "C=0.015 → STATIC (≤thr)" },
        { 0.020f, METRICS_MOTION_STATE_LIGHT_ACTIVITY, "C=0.020 → LIGHT"         },
        { 0.025f, METRICS_MOTION_STATE_MOTION,         "C=0.025 → MOTION (≥thr)" },
        { 0.080f, METRICS_MOTION_STATE_MOTION,         "C=0.080 → MOTION"        },
    };
    const int n = sizeof(cases) / sizeof(cases[0]);
    for (int i = 0; i < n; i++) {
        metrics_motion_state_t got = metrics_classify_motion(cases[i].c_idx);
        bool ok = (got == cases[i].expected);
        if (!ok) fail++;
        ESP_LOGI(TAG, "  [%-4s] %s  got=%d", ok ? "PASS" : "FAIL",
                 cases[i].name, (int)got);
    }
    return fail;
}

esp_err_t m33_run_self_test(void)
{
    ESP_LOGI(TAG, "=== M3.3 Self-Test (ADR-023) ===");
    int total_fail = 0;
    total_fail += test_metrics();
    total_fail += test_quiet_detector();
    total_fail += test_motion_classifier();
    ESP_LOGI(TAG, "=== M3.3 Self-Test: %s (%d failures) ===",
             total_fail == 0 ? "ALL PASS" : "FAIL", total_fail);
    return total_fail == 0 ? ESP_OK : ESP_FAIL;
}

#else  /* !M33_HAS_FIXTURES */

esp_err_t m33_run_self_test(void)
{
    ESP_LOGW(TAG, "Self-test skipped: m33_fixtures.h not generated.");
    ESP_LOGW(TAG, "Run: cd hyperfi && python3 tests/gen_m33_fixtures.py "
                 "--out ../WT99P4C5-S1/main/m33_fixtures.h");
    return ESP_ERR_NOT_FOUND;
}

#endif
