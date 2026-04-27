/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.5.1 self-test — see m35_proto_test.h.
 */

#include "m35_proto_test.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pipeline.h"
#include "proto_codec.h"

static const char *TAG = "m35_proto";

static bool float_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

esp_err_t m35_run_proto_self_test(void)
{
    ESP_LOGI(TAG, "=== M3.5.1 Self-Test (proto roundtrip) ===");

    /* Build a synthetic telemetry snapshot with distinctive values that
     * exercise every wire field. */
    pipeline_telemetry_t in = {0};
    in.timestamp_us             = 1234567890123ULL;
    in.n_frames_in_window       = 909;
    in.csi_fps                  = 303.5f;
    in.norm_cv                  = 0.517f;
    in.raw_cv                   = 0.523f;
    in.shape_corr               = 0.169f;
    in.dynamic_gain_G           = 63018.9f;
    in.quiet_period             = true;
    in.n_valid_subcarriers      = 53;
    for (int i = 0; i < PIPELINE_EMBED_DIM; i++) {
        in.poincare_embed[i] = -0.5f + 0.13f * (float)i;  /* -0.50, -0.37, ..., +0.41 */
    }
    in.collapse_index           = 0.0552f;
    in.fsm_state                = PIPELINE_FSM_SPIKE_DETECTED;
    in.embedding_norm           = 0.4978f;
    in.fall_detected            = true;
    in.fall_event_rising_edge   = true;
    in.fall_confidence          = 0.9914f;
    in.fall_best_pattern_idx    = 0;
    in.fall_best_pattern_name   = "stand_collapse_fast";
    in.rssi_avg                 = -64;
    in.noise_floor_avg          = -90;
    in.csi_drop_count           = 7;

    /* ---- Encode TelemetryReport ---- */
    uint8_t  buf[PROTO_TELEMETRY_BUF_SIZE];
    int64_t  t0 = esp_timer_get_time();
    int      n_enc = proto_codec_encode_telemetry(&in, buf, sizeof(buf));
    int64_t  enc_us = esp_timer_get_time() - t0;
    if (n_enc <= 0) {
        ESP_LOGE(TAG, "[FAIL] telemetry encode returned %d", n_enc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "  encode  : %d bytes  (%lld us)", n_enc, enc_us);

    /* ---- Decode ---- */
    void *dec_struct = malloc(proto_codec_telemetry_struct_size());
    if (!dec_struct) return ESP_ERR_NO_MEM;

    t0 = esp_timer_get_time();
    bool dec_ok = proto_codec_decode_telemetry_struct(buf, n_enc, dec_struct);
    int64_t dec_us = esp_timer_get_time() - t0;
    if (!dec_ok) {
        ESP_LOGE(TAG, "[FAIL] telemetry decode failed");
        free(dec_struct);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "  decode  : ok           (%lld us)", dec_us);

    /* ---- Field-by-field compare ---- */
    int fail = 0;
    const float E = 1e-5f;

#define CHECK_F(name, getter, expected, eps) do {                              \
        float got = getter(dec_struct);                                        \
        if (!float_eq(got, (expected), (eps))) {                               \
            ESP_LOGE(TAG, "  [FAIL] %-12s got=%.6f exp=%.6f", name, got, (double)(expected)); \
            fail++;                                                            \
        }                                                                      \
    } while (0)

#define CHECK_I(name, getter, expected) do {                                   \
        long got = (long)getter(dec_struct);                                   \
        if (got != (long)(expected)) {                                         \
            ESP_LOGE(TAG, "  [FAIL] %-12s got=%ld exp=%ld", name, got, (long)(expected)); \
            fail++;                                                            \
        }                                                                      \
    } while (0)

    CHECK_I("ts",            proto_codec_get_timestamp_us,        in.timestamp_us);
    CHECK_F("collapse_max",  proto_codec_get_collapse_index_max,  in.collapse_index, E);
    CHECK_F("norm_cv",       proto_codec_get_norm_cv,             in.norm_cv,        E);
    CHECK_F("shape_corr",    proto_codec_get_shape_corr,          in.shape_corr,     E);
    CHECK_F("dynamic_gain",  proto_codec_get_dynamic_gain,        in.dynamic_gain_G, 1e-2f);
    CHECK_I("quiet",         proto_codec_get_quiet_period,        (long)in.quiet_period);
    CHECK_I("rssi",          proto_codec_get_rssi,                in.rssi_avg);
    CHECK_I("n_frames",      proto_codec_get_n_frames_in_window,  (long)in.n_frames_in_window);

    /* fsm_state string */
    const char *fsm = proto_codec_get_fsm_state(dec_struct);
    if (strcmp(fsm, "SPIKE_DETECTED") != 0) {
        ESP_LOGE(TAG, "  [FAIL] fsm_state    got='%s' exp='SPIKE_DETECTED'", fsm);
        fail++;
    }

    /* poincare_embed[8] */
    float embed_dec[8];
    proto_codec_get_poincare_embed(dec_struct, embed_dec);
    for (int i = 0; i < 8; i++) {
        if (!float_eq(embed_dec[i], in.poincare_embed[i], E)) {
            ESP_LOGE(TAG, "  [FAIL] embed[%d]    got=%.6f exp=%.6f",
                     i, (double)embed_dec[i], (double)in.poincare_embed[i]);
            fail++;
        }
    }

    free(dec_struct);

#undef CHECK_F
#undef CHECK_I

    /* ---- Encode AlertReport (size sanity only) ---- */
    uint8_t alert_buf[PROTO_ALERT_BUF_SIZE];
    t0 = esp_timer_get_time();
    int n_alert = proto_codec_encode_alert(&in, alert_buf, sizeof(alert_buf));
    int64_t alert_us = esp_timer_get_time() - t0;
    if (n_alert <= 0) {
        ESP_LOGE(TAG, "[FAIL] alert encode returned %d", n_alert);
        fail++;
    } else {
        ESP_LOGI(TAG, "  alert   : %d bytes  (%lld us)", n_alert, alert_us);
    }

    if (fail == 0) {
        ESP_LOGI(TAG, "=== M3.5.1 Self-Test: ALL PASS ===");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "=== M3.5.1 Self-Test: FAIL (%d mismatches) ===", fail);
        return ESP_FAIL;
    }
}
