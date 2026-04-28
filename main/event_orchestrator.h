/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_orchestrator — ADR-023 M3.6.4 glue layer.
 *
 * Wires fall_event_rising_edge from the live pipeline to the rolling
 * event_buffer + EventRawContext encoder + event_uploader. End-to-end
 * sequence:
 *
 *     fall_event_rising_edge (pipeline 1Hz emit)
 *           │
 *           ▼
 *   event_orchestrator_handle(telemetry)
 *           │
 *           ▼
 *   event_buffer_request_snapshot(ts, ±30s, on_snap_ready)
 *           │   ... wait ~30s for post-window frames to accumulate ...
 *           ▼
 *   on_snap_ready: alloc 2MB PSRAM, encode EventRawContext
 *                  via proto_codec, call event_uploader_submit(...)
 *           │
 *           ▼
 *   on_upload_done: free PSRAM blob, log + stats
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pre_sec;       /* default 30.0 */
    float post_sec;      /* default 30.0 */
} event_orchestrator_config_t;

#define EVENT_ORCHESTRATOR_CONFIG_DEFAULT() {  \
    .pre_sec  = 30.0f,                         \
    .post_sec = 30.0f,                         \
}

/**
 * Initialize. Requires event_buffer + event_uploader to be already
 * inited (we do not own their lifecycle).
 */
esp_err_t event_orchestrator_init(const event_orchestrator_config_t *cfg);

/**
 * Hook called from the pipeline telemetry callback on every 1 Hz emit.
 * If t->fall_event_rising_edge is true and we're not already handling
 * an event, kicks off the snapshot → encode → upload chain.
 *
 * Cheap when no event (just an if-check + return).
 */
void event_orchestrator_handle(const pipeline_telemetry_t *t);

/** Stats for monitor / debug. */
typedef struct {
    uint32_t events_seen;          /* fall_event_rising_edge total */
    uint32_t snapshots_requested;  /* requests sent to event_buffer */
    uint32_t uploads_started;      /* submitted to event_uploader */
    uint32_t uploads_succeeded;
    uint32_t uploads_failed;
    uint32_t events_dropped_busy;  /* second event during in-flight upload */
} event_orchestrator_stats_t;

void event_orchestrator_get_stats(event_orchestrator_stats_t *out);

#ifdef __cplusplus
}
#endif
