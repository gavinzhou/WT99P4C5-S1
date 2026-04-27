/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.4 self-test — see m34_test.h.
 */

#include "m34_test.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "fall_detector.h"

#if __has_include("m34_fixtures.h")
#  include "m34_fixtures.h"
#  define M34_HAS_FIXTURES 1
#endif

static const char *TAG = "m34_test";

#ifdef M34_HAS_FIXTURES

static float abs_max_err(const float *got, const float *exp_, int n)
{
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        const float d = fabsf(got[i] - exp_[i]);
        if (d > m) m = d;
    }
    return m;
}

/* -------- 1. Feature extraction -------- */
/* We exercise compute_features() indirectly: replay all entries through
 * fall_detector_update() with fall_flag=true on the last step → trigger
 * → out->feature is the value to check against feature_expected. */
static int test_features(void)
{
    ESP_LOGI(TAG, "--- Feature extraction (%d fixtures) ---",
             M34_FEATURE_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;

    for (int i = 0; i < M34_FEATURE_FIXTURE_COUNT; i++) {
        const m34_feature_fixture_t *fx = &m34_feature_fixtures[i];

        fall_detector_t d;
        fall_detector_init(&d, NULL);

        fall_detector_event_t ev;
        for (int s = 0; s < fx->n_entries; s++) {
            const bool last = (s == fx->n_entries - 1);
            const int64_t t0 = esp_timer_get_time();
            fall_detector_update(
                &d, fx->entries[s].ts_us, fx->entries[s].collapse_index,
                /*fall_flag=*/last, &ev);
            total_us += (esp_timer_get_time() - t0);
        }

        const float err = abs_max_err(ev.feature, fx->feature_expected, 4);
        const bool ok = (err < 1e-3f) && ev.triggered;
        if (ok) pass++; else fail++;

        ESP_LOGI(TAG,
                 "  [%-4s] %-18s  max=%.4f/%.4f mean=%.4f/%.4f "
                 "std=%.4f/%.4f dur=%.3f/%.3f  abs_err=%.2e",
                 ok ? "PASS" : "FAIL", fx->name,
                 ev.feature[0], fx->feature_expected[0],
                 ev.feature[1], fx->feature_expected[1],
                 ev.feature[2], fx->feature_expected[2],
                 ev.feature[3], fx->feature_expected[3], err);
    }
    ESP_LOGI(TAG, "  features: %d/%d PASS  total=%lldus", pass,
             M34_FEATURE_FIXTURE_COUNT, total_us);
    return fail;
}

/* -------- 2. Cosine matcher (in isolation) --------
 * We exercise the matcher by feeding a single update with
 * fall_flag=true, but bypassing the 30-s buffer aggregation. The
 * cleanest way: push 1 entry whose C value would yield the desired
 * feature... but compute_features re-derives features from buffer, not
 * direct input. So we test cosine_match by injecting a known feature
 * vector via a custom config with patterns = {test_vector}; this is
 * not exposed publicly. Instead, we pre-compute features from a known
 * sequence and verify the cosine match indirectly through the
 * end-to-end event tests below.
 *
 * For direct cosine coverage we use a simple harness: build a fake
 * single-entry buffer with C = magnitude factor, fall_flag=true, and
 * read back ev.best_pattern_idx + ev.best_similarity. This won't
 * exactly hit each fixture's feature vector, so we verify a different
 * property: for each fixture, manually evaluate the cosine formula
 * and compare against the pre-computed expected value, using the
 * default reference pattern bank.
 */
static int test_cosine_matcher(void)
{
    ESP_LOGI(TAG, "--- Cosine matcher (%d fixtures, in-place math) ---",
             M34_COSINE_FIXTURE_COUNT);
    int pass = 0, fail = 0;

    for (int i = 0; i < M34_COSINE_FIXTURE_COUNT; i++) {
        const m34_cosine_fixture_t *fx = &m34_cosine_fixtures[i];

        /* Re-implement cosine match in-place to verify the formula
         * matches the fixture's expectation, using fall_default_patterns. */
        float f_sq = 0.0f;
        for (int k = 0; k < 4; k++) f_sq += fx->feature[k] * fx->feature[k];
        const float f_norm = sqrtf(f_sq);

        int   best_idx = -1;
        float best_sim = -1.0f;
        if (f_norm >= 1e-10f) {
            for (int p = 0; p < FALL_DETECTOR_DEFAULT_PATTERN_N; p++) {
                const fall_pattern_t *ref = &fall_default_patterns[p];
                float r_sq = 0.0f, dot = 0.0f;
                for (int k = 0; k < 4; k++) {
                    r_sq += ref->feature[k] * ref->feature[k];
                    dot  += fx->feature[k] * ref->feature[k];
                }
                const float r_norm = sqrtf(r_sq);
                if (r_norm < 1e-10f) continue;
                const float s = dot / (f_norm * r_norm);
                if (s > best_sim) { best_sim = s; best_idx = p; }
            }
            if (best_sim < 0.0f) best_sim = 0.0f;
        } else {
            best_sim = 0.0f;
            best_idx = -1;
        }

        const bool idx_ok = (best_idx == fx->best_pattern_idx_expected);
        const float sim_err = fabsf(best_sim - fx->best_similarity_expected);
        const bool sim_ok = (sim_err < 1e-4f);
        const bool ok = idx_ok && sim_ok;
        if (ok) pass++; else fail++;

        ESP_LOGI(TAG,
                 "  [%-4s] %-22s  idx=%d/%d  sim=%.4f/%.4f  abs_err=%.2e",
                 ok ? "PASS" : "FAIL", fx->name,
                 best_idx, fx->best_pattern_idx_expected,
                 best_sim, fx->best_similarity_expected, sim_err);
    }
    ESP_LOGI(TAG, "  cosine: %d/%d PASS", pass, M34_COSINE_FIXTURE_COUNT);
    return fail;
}

/* -------- 3. End-to-end event sequences -------- */
static int test_event_sequences(void)
{
    ESP_LOGI(TAG, "--- End-to-end event sequences (%d fixtures) ---",
             M34_EVENT_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;

    for (int i = 0; i < M34_EVENT_FIXTURE_COUNT; i++) {
        const m34_event_fixture_t *fx = &m34_event_fixtures[i];

        fall_detector_t d;
        fall_detector_init(&d, NULL);

        /* Init to "no-trigger sentinel" so unchanged fields match
         * fixture expectations when triggered_expected=false. */
        fall_detector_event_t triggered_ev = {
            .triggered         = false,
            .event_ts_us       = 0,
            .confidence        = 0.0f,
            .best_pattern_idx  = -1,
            .best_pattern_name = "",
            .best_similarity   = 0.0f,
            .feature           = { 0.0f, 0.0f, 0.0f, 0.0f },
        };
        bool seen_trigger = false;

        for (int s = 0; s < fx->n_steps; s++) {
            fall_detector_event_t ev;
            const int64_t t0 = esp_timer_get_time();
            fall_detector_update(
                &d, fx->steps[s].ts_us, fx->steps[s].collapse_index,
                fx->steps[s].fall_flag, &ev);
            total_us += (esp_timer_get_time() - t0);

            if (ev.triggered && !seen_trigger) {
                triggered_ev = ev;
                seen_trigger = true;
            }
        }

        const bool trig_ok = (seen_trigger == fx->triggered_expected);
        const float feat_err = abs_max_err(triggered_ev.feature,
                                           fx->feature_expected, 4);
        const float sim_err  = fabsf(triggered_ev.best_similarity
                                     - fx->best_similarity_expected);
        const float conf_err = fabsf(triggered_ev.confidence
                                     - fx->confidence_expected);
        const bool idx_ok = (triggered_ev.best_pattern_idx
                             == fx->best_pattern_idx_expected);

        const bool ok = trig_ok && (feat_err < 1e-3f)
                     && (sim_err < 1e-4f) && (conf_err < 1e-4f)
                     && idx_ok;
        if (ok) pass++; else fail++;

        ESP_LOGI(TAG,
                 "  [%-4s] %-22s  trig=%d/%d idx=%d/%d sim=%.4f/%.4f "
                 "conf=%.4f/%.4f  feat_err=%.2e",
                 ok ? "PASS" : "FAIL", fx->name,
                 seen_trigger, fx->triggered_expected,
                 triggered_ev.best_pattern_idx, fx->best_pattern_idx_expected,
                 triggered_ev.best_similarity, fx->best_similarity_expected,
                 triggered_ev.confidence, fx->confidence_expected,
                 feat_err);
    }
    ESP_LOGI(TAG, "  events: %d/%d PASS  total=%lldus", pass,
             M34_EVENT_FIXTURE_COUNT, total_us);
    return fail;
}

esp_err_t m34_run_self_test(void)
{
    ESP_LOGI(TAG, "=== M3.4 Self-Test (ADR-023) ===");
    int total_fail = 0;
    total_fail += test_features();
    total_fail += test_cosine_matcher();
    total_fail += test_event_sequences();
    ESP_LOGI(TAG, "=== M3.4 Self-Test: %s (%d failures) ===",
             total_fail == 0 ? "ALL PASS" : "FAIL", total_fail);
    return total_fail == 0 ? ESP_OK : ESP_FAIL;
}

#else  /* !M34_HAS_FIXTURES */

esp_err_t m34_run_self_test(void)
{
    ESP_LOGW(TAG, "Self-test skipped: m34_fixtures.h not generated.");
    ESP_LOGW(TAG, "Run: cd hyperfi && python3 tests/gen_m34_fixtures.py "
                 "--out ../WT99P4C5-S1/main/m34_fixtures.h");
    return ESP_ERR_NOT_FOUND;
}

#endif
