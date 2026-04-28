/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_uploader — ADR-023 M3.6.2 alert raw context cloud upload.
 *
 * State machine:
 *   IDLE → REQUEST_PENDING → URL_RECEIVED → UPLOADING → COMPLETE
 *                ↓ (timeout)     ↓ (timeout)   ↓ (HTTP fail)
 *              FAILED          FAILED         FAILED
 *
 * Wire protocol (MQTT topics, all QoS 1):
 *   P4  → broker  hyperfi/{device}/raw/request    (UploadRequest proto)
 *   P4  ← broker  hyperfi/{device}/raw/upload_url (UploadResponse proto)
 *   P4  → server  HTTPS PUT body=EventRawContext
 *   P4  → broker  hyperfi/{device}/raw/done       (UploadDone proto)
 *
 * M3 PoC: server is a Python http.server inside m3_dashboard.py that
 * generates a faux presigned URL pointing to its own endpoint and
 * receives the PUT.
 *
 * Phase 2: same firmware, server is AWS Lambda + S3 presigned URL.
 *
 * Concurrency: only one upload in flight at a time (single-slot queue).
 * A second submit() while busy returns ESP_ERR_INVALID_STATE; pipeline
 * logs a warning and continues. A second fall during upload is rare for
 * an actual care-facility deployment.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int request_timeout_sec;       /* default 10 — wait for upload_url */
    int upload_timeout_sec;        /* default 60 — entire HTTPS PUT */
} event_uploader_config_t;

#define EVENT_UPLOADER_CONFIG_DEFAULT() {  \
    .request_timeout_sec = 10,             \
    .upload_timeout_sec  = 60,             \
}

typedef struct {
    bool      success;
    uint64_t  event_id;
    int       http_status;         /* HTTP status code, or -1 if no response */
    uint32_t  bytes_uploaded;
    uint32_t  elapsed_ms;          /* request → done */
} event_upload_result_t;

typedef void (*event_upload_done_cb_t)(const event_upload_result_t *res, void *ctx);

/**
 * Initialize the uploader. Requires mqtt_publisher to be already inited
 * (we share its client + device_id).
 */
esp_err_t event_uploader_init(const event_uploader_config_t *cfg);

/**
 * Submit a blob for upload. The blob pointer must remain valid until cb
 * fires (uploader does NOT copy the buffer — it streams via PSRAM PUT).
 *
 * Returns:
 *   ESP_OK if the upload was successfully queued (state went IDLE → REQUEST)
 *   ESP_ERR_INVALID_STATE if an upload is already in flight
 *   ESP_ERR_INVALID_ARG on bad params
 */
esp_err_t event_uploader_submit(
    uint64_t                  event_id,
    uint64_t                  event_ts_us,
    const uint8_t            *blob,
    size_t                    blob_size,
    event_upload_done_cb_t    cb,
    void                     *ctx);

/** True iff an upload is currently in flight. */
bool event_uploader_is_busy(void);

#ifdef __cplusplus
}
#endif
