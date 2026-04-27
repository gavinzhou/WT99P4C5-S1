/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.2 self-test — see m32_test.h.
 */

#include "m32_test.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "poincare.h"
#include "collapse.h"

#if __has_include("m32_fixtures.h")
#  include "m32_fixtures.h"
#  define M32_HAS_FIXTURES 1
#endif

static const char *TAG = "m32_test";

#ifdef M32_HAS_FIXTURES

static float max_rel_err(const float *got, const float *expected, int n)
{
    float max_rel = 0.0f;
    for (int i = 0; i < n; i++) {
        const float diff = fabsf(got[i] - expected[i]);
        const float denom = fabsf(expected[i]);
        if (denom > 1e-4f) {
            const float rel = diff / denom;
            if (rel > max_rel) max_rel = rel;
        } else if (diff > max_rel) {
            max_rel = diff;  /* fall back to absolute when expected ~0 */
        }
    }
    return max_rel;
}

/* -------- 1. Poincaré embed -------- */
static int test_embed(void)
{
    ESP_LOGI(TAG, "--- Poincaré embed (%d fixtures) ---", M32_EMBED_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;
    float out[M32_EMBED_DIM];
    for (int i = 0; i < M32_EMBED_FIXTURE_COUNT; i++) {
        const m32_embed_fixture_t *fx = &m32_embed_fixtures[i];
        int64_t t0 = esp_timer_get_time();
        poincare_embed(fx->features, M32_EMBED_DIM, 0.5f, out);
        int64_t dt = esp_timer_get_time() - t0;
        total_us += dt;

        float err = max_rel_err(out, fx->expected, M32_EMBED_DIM);
        bool ok = err < 1e-4f;
        if (ok) pass++; else fail++;
        ESP_LOGI(TAG, "  [%-4s] %-14s  %lldus  rel=%.2e",
                 ok ? "PASS" : "FAIL", fx->name, dt, err);
    }
    ESP_LOGI(TAG, "  embed: %d/%d PASS  avg=%.1fus", pass,
             M32_EMBED_FIXTURE_COUNT, (float)total_us / M32_EMBED_FIXTURE_COUNT);
    return fail;
}

/* -------- 2. Geodesic distance -------- */
static int test_geodesic(void)
{
    ESP_LOGI(TAG, "--- Geodesic distance (%d fixtures) ---", M32_GEODESIC_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;
    for (int i = 0; i < M32_GEODESIC_FIXTURE_COUNT; i++) {
        const m32_geodesic_fixture_t *fx = &m32_geodesic_fixtures[i];
        int64_t t0 = esp_timer_get_time();
        const float d = poincare_geodesic_distance(fx->u, fx->v, M32_EMBED_DIM);
        int64_t dt = esp_timer_get_time() - t0;
        total_us += dt;

        const float err = fabsf(d - fx->expected);
        bool ok = err < 1e-3f;
        if (ok) pass++; else fail++;
        ESP_LOGI(TAG, "  [%-4s] %-32s  %lldus  got=%.4f exp=%.4f  abs_err=%.2e",
                 ok ? "PASS" : "FAIL", fx->name, dt, d, fx->expected, err);
    }
    ESP_LOGI(TAG, "  geodesic: %d/%d PASS  avg=%.1fus", pass,
             M32_GEODESIC_FIXTURE_COUNT, (float)total_us / M32_GEODESIC_FIXTURE_COUNT);
    return fail;
}

/* -------- 3. Möbius addition -------- */
static int test_mobius(void)
{
    ESP_LOGI(TAG, "--- Möbius addition (%d fixtures) ---", M32_MOBIUS_FIXTURE_COUNT);
    int pass = 0, fail = 0;
    int64_t total_us = 0;
    float out[M32_EMBED_DIM];
    for (int i = 0; i < M32_MOBIUS_FIXTURE_COUNT; i++) {
        const m32_mobius_fixture_t *fx = &m32_mobius_fixtures[i];
        int64_t t0 = esp_timer_get_time();
        poincare_mobius_addition(fx->u, fx->v, M32_EMBED_DIM, out);
        int64_t dt = esp_timer_get_time() - t0;
        total_us += dt;

        float err = max_rel_err(out, fx->expected, M32_EMBED_DIM);
        bool ok = err < 1e-4f;
        if (ok) pass++; else fail++;
        ESP_LOGI(TAG, "  [%-4s] %-14s  %lldus  rel=%.2e",
                 ok ? "PASS" : "FAIL", fx->name, dt, err);
    }
    ESP_LOGI(TAG, "  mobius: %d/%d PASS  avg=%.1fus", pass,
             M32_MOBIUS_FIXTURE_COUNT, (float)total_us / M32_MOBIUS_FIXTURE_COUNT);
    return fail;
}

/* -------- 4. Collapse Index sequences -------- */
static int test_collapse_sequence(int fixture_idx)
{
    const m32_collapse_fixture_t *fx = &m32_collapse_fixtures[fixture_idx];
    ESP_LOGI(TAG, "  sequence: %s (%d steps)", fx->name, fx->n_steps);

    collapse_config_t cfg = COLLAPSE_CONFIG_DEFAULT();
    collapse_handle_t *h = NULL;
    if (collapse_init(&h, &cfg) != ESP_OK) return 1;

    int fail = 0;
    int64_t total_us = 0;
    for (int s = 0; s < fx->n_steps; s++) {
        const m32_collapse_step_t *step = &fx->steps[s];
        collapse_result_t result;
        int64_t t0 = esp_timer_get_time();
        collapse_update(h, step->amp, step->ts_us, &result);
        int64_t dt = esp_timer_get_time() - t0;
        total_us += dt;

        const float ci_err = fabsf(result.collapse_index - step->ci_expected);
        const float ci_denom = fabsf(step->ci_expected);
        const float ci_rel = (ci_denom > 1e-6f) ? ci_err / ci_denom : ci_err;
        const float norm_err = fabsf(result.embedding_norm - step->ema_norm_expected);
        const bool state_ok = ((int)result.state == step->fsm_state_expected);
        const bool fall_ok = (result.fall_detected == step->fall_detected_expected);
        const bool num_ok = (ci_rel < 1e-3f) && (norm_err < 1e-3f);
        const bool ok = state_ok && fall_ok && num_ok;
        if (!ok) fail++;

        ESP_LOGI(TAG, "    step %2d  %lldus  C=%.4f/%.4f (rel=%.1e)  ||z||=%.4f/%.4f  state=%d/%d  fall=%d/%d  %s",
                 step->step, dt,
                 result.collapse_index, step->ci_expected, ci_rel,
                 result.embedding_norm, step->ema_norm_expected,
                 (int)result.state, step->fsm_state_expected,
                 result.fall_detected, step->fall_detected_expected,
                 ok ? "OK" : "FAIL");
    }
    ESP_LOGI(TAG, "    avg per step=%.1fus", (float)total_us / fx->n_steps);
    collapse_deinit(h);
    return fail;
}

static int test_collapse(void)
{
    ESP_LOGI(TAG, "--- Collapse Index (%d sequences) ---", M32_COLLAPSE_FIXTURE_COUNT);
    int total_fail = 0;
    for (int i = 0; i < M32_COLLAPSE_FIXTURE_COUNT; i++) {
        total_fail += test_collapse_sequence(i);
    }
    return total_fail;
}

esp_err_t m32_run_self_test(void)
{
    ESP_LOGI(TAG, "=== M3.2 Self-Test (ADR-023) ===");
    int total_fail = 0;
    total_fail += test_embed();
    total_fail += test_geodesic();
    total_fail += test_mobius();
    total_fail += test_collapse();
    ESP_LOGI(TAG, "=== M3.2 Self-Test: %s (%d failures) ===",
             total_fail == 0 ? "ALL PASS" : "FAIL", total_fail);
    return total_fail == 0 ? ESP_OK : ESP_FAIL;
}

#else  /* !M32_HAS_FIXTURES */

esp_err_t m32_run_self_test(void)
{
    ESP_LOGW(TAG, "Self-test skipped: m32_fixtures.h not generated.");
    ESP_LOGW(TAG, "Run: cd hyperfi && python3 tests/gen_m32_fixtures.py "
                 "--out ../WT99P4C5-S1/main/m32_fixtures.h");
    return ESP_ERR_NOT_FOUND;
}

#endif
