/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_orchestrator — see event_orchestrator.h.
 */

#include "event_orchestrator.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "event_buffer.h"
#include "event_uploader.h"
#include "mqtt_publisher.h"
#include "proto_codec.h"

static const char *TAG = "event_orch";

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool                         inited;
    event_orchestrator_config_t  cfg;
    atomic_int                   in_flight;     /* 0 = idle, 1 = busy */
    uint64_t                     next_event_id;

    event_orchestrator_stats_t   stats;
} state_t;

static state_t s;

/* Per-job context — lives on the heap from snapshot-ready until upload-done. */
typedef struct {
    uint64_t   event_id;
    uint64_t   event_ts_us;
    uint8_t   *blob;             /* PSRAM, owned by this struct */
    size_t     blob_size;
    /* Snapshot of telemetry at the moment of the event, used as alert metadata. */
    float      collapse_index_peak;
    float      confidence;
    char       matched_pattern[32];
    int32_t    best_pattern_idx;
} job_t;

/* -------------------------------------------------------------------------- */
/* Callbacks                                                                   */
/* -------------------------------------------------------------------------- */

static void on_upload_done(const event_upload_result_t *res, void *ctx)
{
    job_t *job = (job_t *)ctx;
    if (res->success) s.stats.uploads_succeeded++;
    else              s.stats.uploads_failed++;

    ESP_LOGI(TAG, "%s upload event_id=%llu  status=%d  bytes=%lu  elapsed=%lums",
             res->success ? "✓" : "✗",
             (unsigned long long)res->event_id,
             res->http_status,
             (unsigned long)res->bytes_uploaded,
             (unsigned long)res->elapsed_ms);

    /* Free the PSRAM blob now that the uploader is done with it. */
    heap_caps_free(job->blob);
    free(job);

    atomic_store(&s.in_flight, 0);
}

static void on_snapshot_ready(event_buffer_snapshot_t *snap, void *ctx)
{
    job_t *job = (job_t *)ctx;
    if (!snap || (snap->n_frames == 0 && snap->n_windows == 0)) {
        ESP_LOGW(TAG, "snapshot for event %llu is empty — abandoning",
                 (unsigned long long)job->event_id);
        if (snap) event_buffer_snapshot_free(snap);
        free(job);
        atomic_store(&s.in_flight, 0);
        return;
    }
    ESP_LOGI(TAG, "snapshot ready: event_id=%llu n_frames=%d n_windows=%d  encoding...",
             (unsigned long long)job->event_id, snap->n_frames, snap->n_windows);

    /* Allocate the encode buffer in PSRAM. */
    job->blob = (uint8_t *)heap_caps_calloc(1, PROTO_EVENT_BUF_SIZE,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!job->blob) {
        ESP_LOGE(TAG, "PSRAM alloc %u failed for event %llu",
                 (unsigned)PROTO_EVENT_BUF_SIZE,
                 (unsigned long long)job->event_id);
        event_buffer_snapshot_free(snap);
        free(job);
        atomic_store(&s.in_flight, 0);
        return;
    }

    proto_codec_event_meta_t meta = {
        .device_id           = mqtt_publisher_get_device_id(),
        .event_id            = job->event_id,
        .collapse_index_peak = job->collapse_index_peak,
        .confidence          = job->confidence,
        .matched_pattern     = job->matched_pattern[0] ? job->matched_pattern : NULL,
        .best_pattern_idx    = job->best_pattern_idx,
    };

    int64_t t0 = esp_timer_get_time();
    int n = proto_codec_encode_event_raw_context(snap, &meta,
                                                   job->blob, PROTO_EVENT_BUF_SIZE);
    int64_t enc_us = esp_timer_get_time() - t0;

    /* Snapshot is done — its data is fully in the encoded blob now. */
    event_buffer_snapshot_free(snap);

    if (n <= 0) {
        ESP_LOGE(TAG, "encode failed (n=%d) for event %llu",
                 n, (unsigned long long)job->event_id);
        heap_caps_free(job->blob);
        free(job);
        atomic_store(&s.in_flight, 0);
        s.stats.uploads_failed++;
        return;
    }
    job->blob_size = (size_t)n;
    ESP_LOGI(TAG, "encoded %d bytes (%.2f MB) in %lldus → submit to uploader",
             n, (double)n / (1024.0 * 1024.0), enc_us);

    /* Hand off to event_uploader; on_upload_done will free the blob. */
    s.stats.uploads_started++;
    esp_err_t err = event_uploader_submit(
        job->event_id, job->event_ts_us,
        job->blob, job->blob_size,
        on_upload_done, job);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uploader_submit failed (%d) — uploader busy?", err);
        s.stats.uploads_failed++;
        heap_caps_free(job->blob);
        free(job);
        atomic_store(&s.in_flight, 0);
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t event_orchestrator_init(const event_orchestrator_config_t *cfg)
{
    if (s.inited) return ESP_ERR_INVALID_STATE;
    if (cfg) {
        s.cfg = *cfg;
    } else {
        const event_orchestrator_config_t def = EVENT_ORCHESTRATOR_CONFIG_DEFAULT();
        s.cfg = def;
    }
    if (s.cfg.pre_sec  < 0) s.cfg.pre_sec  = 0;
    if (s.cfg.post_sec < 0) s.cfg.post_sec = 0;

    atomic_store(&s.in_flight, 0);
    s.next_event_id = 1;
    memset(&s.stats, 0, sizeof(s.stats));
    s.inited = true;

    ESP_LOGI(TAG, "ready: pre=%.0fs post=%.0fs (alert→cloud target ≤ %.0fs)",
             (double)s.cfg.pre_sec, (double)s.cfg.post_sec,
             (double)(s.cfg.post_sec + 15.0f /* encode + upload budget */));
    return ESP_OK;
}

void event_orchestrator_handle(const pipeline_telemetry_t *t)
{
    if (!s.inited || !t) return;
    if (!t->fall_event_rising_edge) return;

    s.stats.events_seen++;

    /* Drop second concurrent event (M3.6.2 design — ADR-023 §455). */
    int expected = 0;
    if (!atomic_compare_exchange_strong(&s.in_flight, &expected, 1)) {
        ESP_LOGW(TAG, "fall event during in-flight upload — dropping");
        s.stats.events_dropped_busy++;
        return;
    }

    job_t *job = (job_t *)calloc(1, sizeof(*job));
    if (!job) {
        ESP_LOGE(TAG, "job alloc failed");
        atomic_store(&s.in_flight, 0);
        return;
    }
    job->event_id            = s.next_event_id++;
    job->event_ts_us         = t->timestamp_us;
    job->collapse_index_peak = t->collapse_index;
    job->confidence          = t->fall_confidence;
    job->best_pattern_idx    = t->fall_best_pattern_idx;
    if (t->fall_best_pattern_name) {
        strncpy(job->matched_pattern, t->fall_best_pattern_name,
                sizeof(job->matched_pattern) - 1);
    }

    ESP_LOGI(TAG, "★ FALL EVENT — event_id=%llu ts=%llu peak=%.4f conf=%.4f pattern=%s",
             (unsigned long long)job->event_id,
             (unsigned long long)job->event_ts_us,
             (double)job->collapse_index_peak,
             (double)job->confidence,
             job->matched_pattern[0] ? job->matched_pattern : "(none)");

    s.stats.snapshots_requested++;
    esp_err_t err = event_buffer_request_snapshot(
        job->event_ts_us, s.cfg.pre_sec, s.cfg.post_sec,
        on_snapshot_ready, job);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "request_snapshot failed (%d)", err);
        free(job);
        atomic_store(&s.in_flight, 0);
        s.stats.uploads_failed++;
    }
}

void event_orchestrator_get_stats(event_orchestrator_stats_t *out)
{
    if (!out) return;
    if (!s.inited) { memset(out, 0, sizeof(*out)); return; }
    *out = s.stats;
}
