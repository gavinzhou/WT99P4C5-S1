/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.6.0 self-test — see m36_test.h.
 */

#include "m36_test.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "event_buffer.h"
#include "proto_codec.h"

static const char *TAG = "m36_test";

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void synth_frame(ev_buf_frame_t *f, uint64_t ts_us, uint16_t seq)
{
    memset(f, 0, sizeof(*f));
    f->ts_us       = ts_us;
    f->seq         = seq;
    f->rssi        = -60;
    f->noise_floor = -90;
    /* Mark IQ with seq so we can verify identity after extract. */
    for (int k = 0; k < EV_BUF_IQ_BYTES; k++) {
        f->iq[k] = (int8_t)((seq + k) & 0x7F);
    }
}

static void synth_window(ev_buf_window_t *w, uint64_t ts_us, float collapse)
{
    memset(w, 0, sizeof(*w));
    w->ts_us          = ts_us;
    w->collapse_index = collapse;
    w->norm_cv        = 0.5f;
    w->shape_corr     = 0.15f;
    w->fsm_state      = 0;
    w->flags          = 0;
    for (int i = 0; i < EV_BUF_EMBED_DIM; i++) {
        w->poincare_embed[i] = 0.1f * (float)i;
    }
}

/* -------------------------------------------------------------------------- */
/* Subtests                                                                    */
/* -------------------------------------------------------------------------- */

/* Subtest 1: extract a window covers exactly the expected count. */
static int test_basic_extract(void)
{
    event_buffer_reset();

    /* 200 frames @ 1 ms apart starting at t=10_000_000 us → covers 200 ms. */
    const uint64_t base_us = 10ULL * 1000000ULL;
    const int      n_total = 200;
    for (int i = 0; i < n_total; i++) {
        ev_buf_frame_t f;
        synth_frame(&f, base_us + (uint64_t)i * 1000ULL, (uint16_t)i);
        event_buffer_push_frame(&f);
    }
    /* 5 windows @ 50ms apart */
    for (int i = 0; i < 5; i++) {
        ev_buf_window_t w;
        synth_window(&w, base_us + (uint64_t)i * 50000ULL, 0.01f * (float)i);
        event_buffer_push_window(&w);
    }

    /* Extract the middle 100 ms (i=50..149 inclusive) */
    const uint64_t event_ts = base_us + 100000ULL;     /* center at 100 ms */
    event_buffer_snapshot_t *snap = NULL;
    esp_err_t err = event_buffer_extract_sync(event_ts, 0.050f, 0.050f, &snap);
    if (err != ESP_OK || !snap) {
        ESP_LOGE(TAG, "[FAIL] basic extract returned %d", err);
        return 1;
    }

    int fails = 0;
    /* Expect 101 frames (i=50..150 inclusive, since i=50 has ts=base+50ms = t_lo) */
    if (snap->n_frames != 101) {
        ESP_LOGE(TAG, "[FAIL] basic n_frames got=%d exp=101", snap->n_frames);
        fails++;
    }
    /* Frames must be ts-ascending */
    for (int i = 1; i < snap->n_frames; i++) {
        if (snap->frames[i].ts_us < snap->frames[i - 1].ts_us) {
            ESP_LOGE(TAG, "[FAIL] basic ts not monotonic at idx %d", i);
            fails++;
            break;
        }
    }
    /* All frames must satisfy [t_lo, t_hi] */
    const uint64_t t_lo = event_ts - 50000ULL;
    const uint64_t t_hi = event_ts + 50000ULL;
    for (int i = 0; i < snap->n_frames; i++) {
        if (snap->frames[i].ts_us < t_lo || snap->frames[i].ts_us > t_hi) {
            ESP_LOGE(TAG, "[FAIL] basic ts out of range at idx %d ts=%llu", i,
                     (unsigned long long)snap->frames[i].ts_us);
            fails++;
            break;
        }
    }
    /* Content identity: first frame should have seq=50 (i=50 → seq=50) */
    if (snap->n_frames > 0 && snap->frames[0].seq != 50) {
        ESP_LOGE(TAG, "[FAIL] basic first seq got=%u exp=50", snap->frames[0].seq);
        fails++;
    }
    /* IQ pattern check */
    if (snap->n_frames > 0) {
        const int8_t exp = (int8_t)((50 + 0) & 0x7F);
        if (snap->frames[0].iq[0] != exp) {
            ESP_LOGE(TAG, "[FAIL] basic iq[0] got=%d exp=%d", snap->frames[0].iq[0], exp);
            fails++;
        }
    }
    /* Windows: 3 windows fit in [50ms, 150ms] (i=1@50ms, i=2@100ms, i=3@150ms) */
    if (snap->n_windows != 3) {
        ESP_LOGE(TAG, "[FAIL] basic n_windows got=%d exp=3", snap->n_windows);
        fails++;
    }

    ESP_LOGI(TAG, "  [%-4s] basic_extract: n_frames=%d/%d, n_windows=%d/%d",
             fails == 0 ? "PASS" : "FAIL",
             snap->n_frames, 101, snap->n_windows, 3);
    event_buffer_snapshot_free(snap);
    return fails;
}

/* Subtest 2: ring overflow — push more than capacity, oldest entries are
 *            evicted, extract returns only the freshest range. */
static int test_ring_overflow(void)
{
    event_buffer_reset();

    /* Capacity is 14400 by default → push 15000 frames to force wrap. */
    const int      n_push   = 15000;
    const uint64_t base_us  = 1000ULL;
    for (int i = 0; i < n_push; i++) {
        ev_buf_frame_t f;
        synth_frame(&f, base_us + (uint64_t)i * 1000ULL, (uint16_t)(i & 0xFFFF));
        event_buffer_push_frame(&f);
    }
    event_buffer_stats_t st;
    event_buffer_get_stats(&st);
    if (st.frame_count != 14400) {
        ESP_LOGE(TAG, "[FAIL] overflow frame_count=%d exp=14400", st.frame_count);
        return 1;
    }
    if (st.total_frames_pushed != (uint32_t)n_push) {
        ESP_LOGE(TAG, "[FAIL] overflow total_pushed=%lu exp=%d",
                 (unsigned long)st.total_frames_pushed, n_push);
        return 1;
    }

    /* Extract the most recent 100 ms — should give 101 frames whose seqs
     * correspond to i = (n_push-100)..(n_push-1)  modulo 0xFFFF wraparound. */
    const uint64_t newest_ts = base_us + (uint64_t)(n_push - 1) * 1000ULL;
    event_buffer_snapshot_t *snap = NULL;
    esp_err_t err = event_buffer_extract_sync(newest_ts, 0.100f, 0.0f, &snap);
    if (err != ESP_OK || !snap) {
        ESP_LOGE(TAG, "[FAIL] overflow extract %d", err);
        return 1;
    }
    int fails = 0;
    if (snap->n_frames != 101) {
        ESP_LOGE(TAG, "[FAIL] overflow n_frames got=%d exp=101", snap->n_frames);
        fails++;
    }
    /* First frame should be at ts = newest_ts - 100ms */
    if (snap->n_frames > 0) {
        if (snap->frames[0].ts_us != newest_ts - 100000ULL) {
            ESP_LOGE(TAG, "[FAIL] overflow first ts got=%llu exp=%llu",
                     (unsigned long long)snap->frames[0].ts_us,
                     (unsigned long long)(newest_ts - 100000ULL));
            fails++;
        }
    }
    /* Verify oldest frames (i=0..n_push-14401) are gone — try extracting
     * around the very first push timestamp; should return 0 entries. */
    event_buffer_snapshot_t *old = NULL;
    if (event_buffer_extract_sync(base_us, 0.0f, 0.0f, &old) == ESP_OK && old) {
        if (old->n_frames != 0) {
            ESP_LOGE(TAG, "[FAIL] overflow expected 0 evicted-range frames, got %d",
                     old->n_frames);
            fails++;
        }
        event_buffer_snapshot_free(old);
    }

    ESP_LOGI(TAG, "  [%-4s] ring_overflow: ring_count=%d, push_total=%lu",
             fails == 0 ? "PASS" : "FAIL",
             st.frame_count, (unsigned long)st.total_frames_pushed);
    event_buffer_snapshot_free(snap);
    return fails;
}

/* Subtest 3: empty buffer / out-of-range extract returns 0 frames cleanly. */
static int test_empty_extract(void)
{
    event_buffer_reset();
    event_buffer_snapshot_t *snap = NULL;
    esp_err_t err = event_buffer_extract_sync(1000000, 0.1f, 0.1f, &snap);
    int fails = 0;
    if (err != ESP_OK || !snap) {
        ESP_LOGE(TAG, "[FAIL] empty extract %d", err);
        fails++;
    } else {
        if (snap->n_frames != 0 || snap->n_windows != 0) {
            ESP_LOGE(TAG, "[FAIL] empty got n_frames=%d n_windows=%d",
                     snap->n_frames, snap->n_windows);
            fails++;
        }
        event_buffer_snapshot_free(snap);
    }
    ESP_LOGI(TAG, "  [%-4s] empty_extract", fails == 0 ? "PASS" : "FAIL");
    return fails;
}

/* Subtest 4: 60-s production-shape test. Push 60s @ 200 fps → ~12000
 *            frames + 60 windows. Extract event_ts ± 30s → expect ~12000
 *            frames (everything we have, since pre+post=60s window). */
static int test_production_shape(void)
{
    event_buffer_reset();

    const int      fps    = 200;
    const int      n_secs = 60;
    const uint64_t base   = 100ULL * 1000000ULL;
    int seq = 0;
    /* 60s at 200fps = 12000 frames, 5ms apart */
    for (int s = 0; s < n_secs; s++) {
        for (int i = 0; i < fps; i++) {
            ev_buf_frame_t f;
            const uint64_t ts = base + (uint64_t)s * 1000000ULL +
                                (uint64_t)i * (1000000ULL / fps);
            synth_frame(&f, ts, (uint16_t)(seq++ & 0xFFFF));
            event_buffer_push_frame(&f);
        }
        ev_buf_window_t w;
        synth_window(&w, base + (uint64_t)s * 1000000ULL, 0.01f);
        event_buffer_push_window(&w);
    }

    /* Event at center of buffer, ±30s extract */
    const uint64_t event_ts = base + 30ULL * 1000000ULL;
    event_buffer_snapshot_t *snap = NULL;
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = event_buffer_extract_sync(event_ts, 30.0f, 30.0f, &snap);
    int64_t dt = esp_timer_get_time() - t0;
    int fails = 0;
    if (err != ESP_OK || !snap) {
        ESP_LOGE(TAG, "[FAIL] production extract %d", err);
        return 1;
    }
    /* Expect approximately n_secs * fps = 12000 frames (off by ±1 at the edges) */
    if (snap->n_frames < n_secs * fps - 5 || snap->n_frames > n_secs * fps + 5) {
        ESP_LOGE(TAG, "[FAIL] production n_frames got=%d exp~%d",
                 snap->n_frames, n_secs * fps);
        fails++;
    }
    if (snap->n_windows < 60 || snap->n_windows > 61) {
        ESP_LOGE(TAG, "[FAIL] production n_windows got=%d exp~60", snap->n_windows);
        fails++;
    }

    /* Approximate payload size for sanity (matches ADR-023 §471 estimate) */
    const size_t bytes_frames = (size_t)snap->n_frames * sizeof(ev_buf_frame_t);
    ESP_LOGI(TAG, "  [%-4s] production_shape: n_frames=%d n_windows=%d  "
                  "extract=%lldus  payload=%zuKB",
             fails == 0 ? "PASS" : "FAIL",
             snap->n_frames, snap->n_windows, dt, bytes_frames / 1024);
    event_buffer_snapshot_free(snap);
    return fails;
}

/* Subtest 5 (M3.6.1): EventRawContext encode — fill buffer with prod-shape
 *                     data, encode via streaming callback, verify byte count
 *                     is in the expected range, no encoder error. */
static int test_proto_encode(void)
{
    event_buffer_reset();

    /* Re-create the production-shape data set (60s × 200fps + 60 windows). */
    const int      fps    = 200;
    const int      n_secs = 60;
    const uint64_t base   = 100ULL * 1000000ULL;
    int seq = 0;
    for (int s = 0; s < n_secs; s++) {
        for (int i = 0; i < fps; i++) {
            ev_buf_frame_t f;
            const uint64_t ts = base + (uint64_t)s * 1000000ULL +
                                (uint64_t)i * (1000000ULL / fps);
            synth_frame(&f, ts, (uint16_t)(seq++ & 0xFFFF));
            event_buffer_push_frame(&f);
        }
        ev_buf_window_t w;
        synth_window(&w, base + (uint64_t)s * 1000000ULL, 0.01f);
        event_buffer_push_window(&w);
    }

    const uint64_t event_ts = base + 30ULL * 1000000ULL;
    event_buffer_snapshot_t *snap = NULL;
    if (event_buffer_extract_sync(event_ts, 30.0f, 30.0f, &snap) != ESP_OK || !snap) {
        ESP_LOGE(TAG, "[FAIL] proto_encode: extract failed");
        return 1;
    }

    /* Allocate output buffer in PSRAM (2 MB). */
    uint8_t *out = (uint8_t *)heap_caps_calloc(1, PROTO_EVENT_BUF_SIZE,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out) {
        ESP_LOGE(TAG, "[FAIL] proto_encode: PSRAM alloc failed");
        event_buffer_snapshot_free(snap);
        return 1;
    }

    proto_codec_event_meta_t meta = {
        .device_id           = "p4-test01",
        .event_id            = 0xCAFEBABEDEADBEEFULL,
        .collapse_index_peak = 0.45f,
        .confidence          = 0.991f,
        .matched_pattern     = "stand_collapse_fast",
        .best_pattern_idx    = 0,
    };

    int64_t t0 = esp_timer_get_time();
    int n = proto_codec_encode_event_raw_context(snap, &meta, out, PROTO_EVENT_BUF_SIZE);
    int64_t dt = esp_timer_get_time() - t0;

    int fails = 0;
    /* Expected size: 12000 frames × ~135B + 60 windows × ~80B + meta ~120B
     *   ≈ 1.62 MB. Accept range 1.4-1.9 MB. */
    const int min_bytes = 1400 * 1024;
    const int max_bytes = 1900 * 1024;
    if (n < 0) {
        ESP_LOGE(TAG, "[FAIL] proto_encode: encode returned %d", n);
        fails++;
    } else if (n < min_bytes || n > max_bytes) {
        ESP_LOGE(TAG, "[FAIL] proto_encode: size %d out of range [%d, %d]",
                 n, min_bytes, max_bytes);
        fails++;
    }

    /* First 2 bytes should be a valid protobuf tag for field 1 (device_id, string)
     * = wire type 2 (length-delimited), tag = (1<<3) | 2 = 0x0a */
    if (n >= 1 && out[0] != 0x0a) {
        ESP_LOGE(TAG, "[FAIL] proto_encode: first byte=0x%02x exp=0x0a", out[0]);
        fails++;
    }

    ESP_LOGI(TAG,
             "  [%-4s] proto_encode: %d bytes (%.2f MB), encode=%lldus  throughput=%.1f MB/s",
             fails == 0 ? "PASS" : "FAIL",
             n, (double)n / (1024.0 * 1024.0), dt,
             (double)n / 1024.0 / 1024.0 / ((double)dt / 1.0e6));

    heap_caps_free(out);
    event_buffer_snapshot_free(snap);
    return fails;
}

/* -------------------------------------------------------------------------- */
/* Top-level driver                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t m36_run_self_test(void)
{
    ESP_LOGI(TAG, "=== M3.6.0+M3.6.1 Self-Test (event_buffer + EventRawContext encode) ===");

    const event_buffer_config_t cfg = EVENT_BUFFER_CONFIG_DEFAULT();
    if (event_buffer_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "init failed (PSRAM exhausted?)");
        return ESP_FAIL;
    }

    int fails = 0;
    fails += test_basic_extract();
    fails += test_ring_overflow();
    fails += test_empty_extract();
    fails += test_production_shape();
    fails += test_proto_encode();

    /* Production tests will reuse the buffer, so reset rather than deinit. */
    event_buffer_reset();

    ESP_LOGI(TAG, "=== M3.6 Self-Test: %s (%d failures) ===",
             fails == 0 ? "ALL PASS" : "FAIL", fails);
    return fails == 0 ? ESP_OK : ESP_FAIL;
}
