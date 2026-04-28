/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_uploader — see event_uploader.h.
 */

#include "event_uploader.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "telemetry.pb.h"

#include "mqtt_publisher.h"

static const char *TAG = "event_upl";

#define TOPIC_MAX            96
#define UPLOAD_URL_MAX       512
#define REQUEST_BUF_SIZE     128
#define DONE_BUF_SIZE        128

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef enum {
    ST_IDLE = 0,
    ST_BUSY,                                /* anything from REQUEST → DONE */
} uploader_state_t;

typedef struct {
    bool                     inited;
    event_uploader_config_t  cfg;
    esp_mqtt_client_handle_t mqtt;
    const char              *device_id;

    char                     topic_request[TOPIC_MAX];
    char                     topic_url[TOPIC_MAX];
    char                     topic_done[TOPIC_MAX];

    atomic_int               state;        /* uploader_state_t */
    SemaphoreHandle_t        url_received; /* signaled on UploadResponse */
    char                     pending_url[UPLOAD_URL_MAX];
    uint64_t                 pending_event_id;

    /* In-flight job (only valid while state == BUSY) */
    uint64_t                 cur_event_id;
    uint64_t                 cur_event_ts_us;
    const uint8_t           *cur_blob;
    size_t                   cur_blob_size;
    event_upload_done_cb_t   cur_cb;
    void                    *cur_ctx;

    /* Stats */
    uint32_t                 total_uploads;
    uint32_t                 total_failures;
} state_t;

static state_t s;

/* -------------------------------------------------------------------------- */
/* MQTT receive — filter for our upload_url topic                              */
/* -------------------------------------------------------------------------- */

static void on_mqtt_event(void *handler_args, esp_event_base_t base,
                           int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base;

    /* Re-issue our subscription on every (re)connect. Doing it inside
     * MQTT_EVENT_CONNECTED instead of init() handles the bootstrapping
     * race (init runs before the broker has accepted the connection)
     * AND gives free re-subscribe on broker drop/reconnect. */
    if (event_id == MQTT_EVENT_CONNECTED) {
        int msg_id = esp_mqtt_client_subscribe(s.mqtt, s.topic_url, 1);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "subscribe %s failed (msg_id=%d) — will retry on next connect",
                     s.topic_url, msg_id);
        } else {
            ESP_LOGI(TAG, "subscribed %s (msg_id=%d)", s.topic_url, msg_id);
        }
        return;
    }

    if (event_id != MQTT_EVENT_DATA) return;

    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;
    if (!e || !e->topic || e->topic_len <= 0) return;

    /* Filter: only fire on our upload_url topic. */
    if ((size_t)e->topic_len != strlen(s.topic_url) ||
        memcmp(e->topic, s.topic_url, e->topic_len) != 0) {
        return;
    }

    /* Decode UploadResponse. */
    hyperfi_csi_UploadResponse resp = hyperfi_csi_UploadResponse_init_zero;
    pb_istream_t stream = pb_istream_from_buffer((const uint8_t *)e->data,
                                                   (size_t)e->data_len);
    if (!pb_decode(&stream, hyperfi_csi_UploadResponse_fields, &resp)) {
        ESP_LOGW(TAG, "decode UploadResponse: %s", PB_GET_ERROR(&stream));
        return;
    }
    /* Correlate via event_id (drop responses for stale jobs). */
    if (resp.event_id != s.pending_event_id) {
        ESP_LOGW(TAG, "UploadResponse event_id mismatch: got %llu exp %llu",
                 (unsigned long long)resp.event_id,
                 (unsigned long long)s.pending_event_id);
        return;
    }
    strncpy(s.pending_url, resp.upload_url, sizeof(s.pending_url) - 1);
    s.pending_url[sizeof(s.pending_url) - 1] = 0;
    ESP_LOGI(TAG, "← UploadResponse: event_id=%llu url len=%d",
             (unsigned long long)resp.event_id, (int)strlen(s.pending_url));
    xSemaphoreGive(s.url_received);
}

/* -------------------------------------------------------------------------- */
/* Encoding helpers                                                            */
/* -------------------------------------------------------------------------- */

static int encode_request(uint8_t *buf, size_t buf_size,
                           uint64_t event_id, uint32_t size_bytes,
                           uint64_t event_ts_us)
{
    hyperfi_csi_UploadRequest msg = hyperfi_csi_UploadRequest_init_zero;
    if (s.device_id) strncpy(msg.device_id, s.device_id, sizeof(msg.device_id) - 1);
    msg.event_id    = event_id;
    msg.size_bytes  = size_bytes;
    msg.event_ts_us = event_ts_us;
    pb_ostream_t st = pb_ostream_from_buffer(buf, buf_size);
    if (!pb_encode(&st, hyperfi_csi_UploadRequest_fields, &msg)) {
        ESP_LOGE(TAG, "encode request: %s", PB_GET_ERROR(&st));
        return -1;
    }
    return (int)st.bytes_written;
}

static int encode_done(uint8_t *buf, size_t buf_size,
                        uint64_t event_id, bool success,
                        int http_status, uint32_t bytes_uploaded,
                        uint32_t elapsed_ms)
{
    hyperfi_csi_UploadDone msg = hyperfi_csi_UploadDone_init_zero;
    if (s.device_id) strncpy(msg.device_id, s.device_id, sizeof(msg.device_id) - 1);
    msg.event_id       = event_id;
    msg.success        = success;
    msg.http_status    = http_status;
    msg.bytes_uploaded = bytes_uploaded;
    msg.elapsed_ms     = elapsed_ms;
    pb_ostream_t st = pb_ostream_from_buffer(buf, buf_size);
    if (!pb_encode(&st, hyperfi_csi_UploadDone_fields, &msg)) {
        ESP_LOGE(TAG, "encode done: %s", PB_GET_ERROR(&st));
        return -1;
    }
    return (int)st.bytes_written;
}

/* -------------------------------------------------------------------------- */
/* HTTP PUT                                                                    */
/* -------------------------------------------------------------------------- */

static int http_put(const char *url, const uint8_t *body, size_t body_size,
                     int timeout_ms)
{
    esp_http_client_config_t cfg = {0};
    cfg.url            = url;
    cfg.method         = HTTP_METHOD_PUT;
    cfg.timeout_ms     = timeout_ms;
    cfg.disable_auto_redirect = false;
    /* M3 PoC: skip cert verification for the local mock S3.
     * Phase 2: real S3 needs cert bundle (esp_crt_bundle). */
    cfg.crt_bundle_attach = NULL;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "http_client_init failed for url=%s", url);
        return -1;
    }
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    /* Open connection with explicit content-length so the server gets
     * Content-Length up front and can size buffers / redirect to disk. */
    esp_err_t err = esp_http_client_open(client, (int)body_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "http_client_open: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return -1;
    }
    int n_written = esp_http_client_write(client, (const char *)body, body_size);
    if (n_written < 0 || (size_t)n_written != body_size) {
        ESP_LOGE(TAG, "http_client_write returned %d, expected %zu",
                 n_written, body_size);
        esp_http_client_cleanup(client);
        return -1;
    }
    int hdr_len = esp_http_client_fetch_headers(client);
    int status  = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "PUT %s → HTTP %d (resp_hdr_len=%d, body=%zu B)",
             url, status, hdr_len, body_size);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

/* -------------------------------------------------------------------------- */
/* Worker task — runs the full state machine for one upload                    */
/* -------------------------------------------------------------------------- */

static void worker_task(void *arg)
{
    (void)arg;
    const int64_t t_start = esp_timer_get_time();

    event_upload_result_t res = {
        .success        = false,
        .event_id       = s.cur_event_id,
        .http_status    = -1,
        .bytes_uploaded = 0,
        .elapsed_ms     = 0,
    };

    /* ---- 1. Publish UploadRequest ---- */
    s.pending_event_id = s.cur_event_id;
    s.pending_url[0]   = 0;
    /* Drain any stale signal from prior runs. */
    xSemaphoreTake(s.url_received, 0);

    uint8_t req_buf[REQUEST_BUF_SIZE];
    int req_len = encode_request(req_buf, sizeof(req_buf),
                                  s.cur_event_id,
                                  (uint32_t)s.cur_blob_size,
                                  s.cur_event_ts_us);
    if (req_len < 0) goto done;
    int msg_id = esp_mqtt_client_publish(s.mqtt, s.topic_request,
                                          (const char *)req_buf, req_len, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "publish UploadRequest failed (msg_id=%d)", msg_id);
        goto done;
    }
    ESP_LOGI(TAG, "→ UploadRequest: event_id=%llu size=%zu",
             (unsigned long long)s.cur_event_id, s.cur_blob_size);

    /* ---- 2. Wait for UploadResponse ---- */
    if (xSemaphoreTake(s.url_received,
                        pdMS_TO_TICKS(s.cfg.request_timeout_sec * 1000)) != pdTRUE) {
        ESP_LOGW(TAG, "timeout waiting for UploadResponse (%ds)",
                 s.cfg.request_timeout_sec);
        goto done;
    }
    if (s.pending_url[0] == 0) {
        ESP_LOGW(TAG, "UploadResponse arrived but URL is empty");
        goto done;
    }

    /* ---- 3. HTTPS PUT ---- */
    int status = http_put(s.pending_url, s.cur_blob, s.cur_blob_size,
                           s.cfg.upload_timeout_sec * 1000);
    res.http_status    = status;
    res.bytes_uploaded = (uint32_t)s.cur_blob_size;
    res.success        = (status >= 200 && status < 300);

done:
    res.elapsed_ms = (uint32_t)((esp_timer_get_time() - t_start) / 1000);

    /* ---- 4. Publish UploadDone (best-effort even on failure) ---- */
    uint8_t done_buf[DONE_BUF_SIZE];
    int done_len = encode_done(done_buf, sizeof(done_buf),
                                s.cur_event_id, res.success,
                                res.http_status, res.bytes_uploaded,
                                res.elapsed_ms);
    if (done_len > 0) {
        esp_mqtt_client_publish(s.mqtt, s.topic_done,
                                 (const char *)done_buf, done_len, 1, 0);
    }
    ESP_LOGI(TAG, "%s upload event_id=%llu  status=%d  bytes=%lu  elapsed=%lums",
             res.success ? "✓" : "✗",
             (unsigned long long)s.cur_event_id,
             res.http_status,
             (unsigned long)res.bytes_uploaded,
             (unsigned long)res.elapsed_ms);

    if (res.success) s.total_uploads++;
    else             s.total_failures++;

    /* ---- 5. Invoke caller cb + release state ---- */
    event_upload_done_cb_t cb = s.cur_cb;
    void                  *cx = s.cur_ctx;
    s.cur_blob = NULL;
    s.cur_cb   = NULL;
    s.cur_ctx  = NULL;
    atomic_store(&s.state, ST_IDLE);
    if (cb) cb(&res, cx);

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t event_uploader_init(const event_uploader_config_t *cfg)
{
    if (s.inited) return ESP_ERR_INVALID_STATE;

    if (cfg) {
        s.cfg = *cfg;
    } else {
        const event_uploader_config_t def = EVENT_UPLOADER_CONFIG_DEFAULT();
        s.cfg = def;
    }
    if (s.cfg.request_timeout_sec <= 0) s.cfg.request_timeout_sec = 10;
    if (s.cfg.upload_timeout_sec  <= 0) s.cfg.upload_timeout_sec  = 60;

    s.mqtt      = (esp_mqtt_client_handle_t)mqtt_publisher_get_client();
    s.device_id = mqtt_publisher_get_device_id();
    if (!s.mqtt || !s.device_id) {
        ESP_LOGE(TAG, "mqtt_publisher must be inited first");
        return ESP_ERR_INVALID_STATE;
    }

    snprintf(s.topic_request, sizeof(s.topic_request),
             "hyperfi/%s/raw/request", s.device_id);
    snprintf(s.topic_url, sizeof(s.topic_url),
             "hyperfi/%s/raw/upload_url", s.device_id);
    snprintf(s.topic_done, sizeof(s.topic_done),
             "hyperfi/%s/raw/done", s.device_id);

    s.url_received = xSemaphoreCreateBinary();
    if (!s.url_received) return ESP_ERR_NO_MEM;
    atomic_store(&s.state, ST_IDLE);

    /* Register for ANY event — our handler dispatches:
     *   - MQTT_EVENT_CONNECTED : (re-)subscribe to the upload_url topic
     *   - MQTT_EVENT_DATA      : decode UploadResponse on our topic */
    esp_err_t err = esp_mqtt_client_register_event(s.mqtt, ESP_EVENT_ANY_ID,
                                                     on_mqtt_event, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register MQTT event handler: %d", err);
        return err;
    }

    /* Belt-and-suspenders subscribe race fix:
     *  - If MQTT is *already* connected (e.g. mqtt_publisher_init kicked
     *    off the connection earlier and it landed before we got here),
     *    MQTT_EVENT_CONNECTED has already fired and won't fire again
     *    until a reconnect. The handler-only path would silently never
     *    subscribe.
     *  - If MQTT is *not yet* connected, this call returns msg_id=-1
     *    ("Client has not connected"), which is fine — the CONNECTED
     *    handler above will re-issue the subscribe when connection lands.
     * Either way we get exactly one valid subscription. */
    int msg_id = esp_mqtt_client_subscribe(s.mqtt, s.topic_url, 1);
    if (msg_id < 0) {
        ESP_LOGI(TAG, "init-time subscribe deferred (mqtt not connected yet) — "
                      "will subscribe on MQTT_EVENT_CONNECTED");
    } else {
        ESP_LOGI(TAG, "subscribed %s at init (msg_id=%d)", s.topic_url, msg_id);
    }

    s.inited = true;
    ESP_LOGI(TAG, "ready — topics: req=%s url=%s done=%s",
             s.topic_request, s.topic_url, s.topic_done);
    return ESP_OK;
}

esp_err_t event_uploader_submit(
    uint64_t                  event_id,
    uint64_t                  event_ts_us,
    const uint8_t            *blob,
    size_t                    blob_size,
    event_upload_done_cb_t    cb,
    void                     *ctx)
{
    if (!s.inited)            return ESP_ERR_INVALID_STATE;
    if (!blob || blob_size == 0) return ESP_ERR_INVALID_ARG;

    int expected = ST_IDLE;
    if (!atomic_compare_exchange_strong(&s.state, &expected, ST_BUSY)) {
        ESP_LOGW(TAG, "submit: busy with another upload, dropping event_id=%llu",
                 (unsigned long long)event_id);
        return ESP_ERR_INVALID_STATE;
    }

    s.cur_event_id    = event_id;
    s.cur_event_ts_us = event_ts_us;
    s.cur_blob        = blob;
    s.cur_blob_size   = blob_size;
    s.cur_cb          = cb;
    s.cur_ctx         = ctx;

    BaseType_t r = xTaskCreate(worker_task, "evup_wkr", 6144, NULL, 5, NULL);
    if (r != pdPASS) {
        atomic_store(&s.state, ST_IDLE);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool event_uploader_is_busy(void)
{
    return atomic_load(&s.state) == ST_BUSY;
}
