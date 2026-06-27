/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Breathing rate self-test (M4.3). Feeds gen_breathing_fixtures.py signals to
 * breathing_estimate_welch() and checks BPM matches the Python gold-standard
 * (Welch nfft=256) within tolerance. Run at boot like the other mXX self-tests.
 */
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "breathing.h"
#include "m_breathing_test.h"

#if __has_include("m_breathing_fixtures.h")
#  include "m_breathing_fixtures.h"
#  define HAS_BREATHING_FIXTURES 1
#endif

static const char *TAG = "m_breathing";

#ifdef HAS_BREATHING_FIXTURES

esp_err_t breathing_run_self_test(void)
{
    ESP_LOGI(TAG, "=== Breathing Self-Test (M4.3) — %d fixtures, fs=%.1f fft=%d ===",
             BREATHING_FIXTURE_COUNT, BREATHING_FIXTURE_FS, BREATHING_FIXTURE_FFT_SIZE);

    breathing_config_t cfg = BREATHING_CONFIG_DEFAULT();
    cfg.fft_size = BREATHING_FIXTURE_FFT_SIZE;

    int pass = 0, fail = 0;
    for (int i = 0; i < BREATHING_FIXTURE_COUNT; i++) {
        const breathing_fixture_t *fx = &breathing_fixtures[i];
        float conf = 0;
        float bpm = breathing_estimate_welch(&cfg, fx->signal, fx->n, &conf);

        float tol = fx->expected_bpm * 0.06f;     /* 6% */
        if (tol < 2.5f) tol = 2.5f;               /* >= 1 FFT bin (~2.3 bpm) */
        bool bpm_ok = fabsf(bpm - fx->expected_bpm) <= tol;
        /* confidence is scale-invariant (peak/median): expect same order */
        bool conf_ok = (fx->expected_conf <= 0.0f) ||
                       (conf >= fx->expected_conf * 0.5f &&
                        conf <= fx->expected_conf * 2.0f);
        bool ok = bpm_ok && conf_ok;
        if (ok) pass++; else fail++;

        ESP_LOGI(TAG, "[%-4s] %-16s bpm=%6.2f/%6.2f (tol %.1f)  conf=%8.2f/%8.2f  known=%.0f",
                 ok ? "PASS" : "FAIL", fx->name, bpm, fx->expected_bpm, tol,
                 conf, fx->expected_conf, fx->known_bpm);
        if (!ok) {
            if (!bpm_ok) ESP_LOGW(TAG, "    bpm off by %.2f", fabsf(bpm - fx->expected_bpm));
            if (!conf_ok) ESP_LOGW(TAG, "    conf out of [0.5x,2x] band");
        }
    }
    ESP_LOGI(TAG, "=== Breathing Self-Test: %d/%d PASS ===", pass, BREATHING_FIXTURE_COUNT);
    return (fail == 0) ? ESP_OK : ESP_FAIL;
}

#else  /* no fixtures */

esp_err_t breathing_run_self_test(void)
{
    ESP_LOGW(TAG, "Breathing self-test skipped: m_breathing_fixtures.h not generated.");
    return ESP_ERR_NOT_FOUND;
}

#endif
