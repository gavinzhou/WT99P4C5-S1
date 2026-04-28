/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_buffer — see event_buffer.h.
 */

#include "event_buffer.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "event_buf";

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool                  inited;
    event_buffer_config_t cfg;

    SemaphoreHandle_t     mu;          /* protects ring head/count */

    /* Frame ring (high-rate) */
    ev_buf_frame_t       *frames;
    int                   f_head;      /* next write idx */
    int                   f_count;
    uint32_t              f_total;
    uint32_t              f_drop;

    /* Window ring (1 Hz) */
    ev_buf_window_t      *windows;
    int                   w_head;
    int                   w_count;
    uint32_t              w_total;
} state_t;

static state_t g_eb;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void *psram_calloc(size_t n, size_t sz)
{
    return heap_caps_calloc(n, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static inline void lock(void)
{
    if (g_eb.mu) xSemaphoreTake(g_eb.mu, portMAX_DELAY);
}
static inline void unlock(void)
{
    if (g_eb.mu) xSemaphoreGive(g_eb.mu);
}

/* -------------------------------------------------------------------------- */
/* Init / deinit                                                               */
/* -------------------------------------------------------------------------- */

esp_err_t event_buffer_init(const event_buffer_config_t *cfg)
{
    if (g_eb.inited) return ESP_ERR_INVALID_STATE;

    if (cfg) {
        g_eb.cfg = *cfg;
    } else {
        const event_buffer_config_t def = EVENT_BUFFER_CONFIG_DEFAULT();
        g_eb.cfg = def;
    }
    if (g_eb.cfg.frame_capacity  < 100) g_eb.cfg.frame_capacity  = 100;
    if (g_eb.cfg.window_capacity < 4)   g_eb.cfg.window_capacity = 4;

    g_eb.mu = xSemaphoreCreateMutex();
    g_eb.frames  = psram_calloc((size_t)g_eb.cfg.frame_capacity,  sizeof(ev_buf_frame_t));
    g_eb.windows = psram_calloc((size_t)g_eb.cfg.window_capacity, sizeof(ev_buf_window_t));
    if (!g_eb.mu || !g_eb.frames || !g_eb.windows) {
        ESP_LOGE(TAG, "alloc failed");
        event_buffer_deinit();
        return ESP_ERR_NO_MEM;
    }
    g_eb.inited = true;

    const size_t frame_kb  = (size_t)g_eb.cfg.frame_capacity  * sizeof(ev_buf_frame_t)  / 1024;
    const size_t window_kb = (size_t)g_eb.cfg.window_capacity * sizeof(ev_buf_window_t) / 1024;
    ESP_LOGI(TAG, "ready: frames=%d (%zu KB), windows=%d (%zu KB), window_sec=%.1f",
             g_eb.cfg.frame_capacity, frame_kb,
             g_eb.cfg.window_capacity, window_kb,
             (double)g_eb.cfg.window_sec);
    return ESP_OK;
}

void event_buffer_deinit(void)
{
    if (g_eb.mu) { vSemaphoreDelete(g_eb.mu); g_eb.mu = NULL; }
    heap_caps_free(g_eb.frames);  g_eb.frames  = NULL;
    heap_caps_free(g_eb.windows); g_eb.windows = NULL;
    memset(&g_eb, 0, sizeof(g_eb));
}

void event_buffer_reset(void)
{
    if (!g_eb.inited) return;
    lock();
    g_eb.f_head = g_eb.f_count = 0;
    g_eb.w_head = g_eb.w_count = 0;
    g_eb.f_total = g_eb.w_total = g_eb.f_drop = 0;
    unlock();
}

/* -------------------------------------------------------------------------- */
/* Push                                                                        */
/* -------------------------------------------------------------------------- */

void event_buffer_push_frame(const ev_buf_frame_t *frame)
{
    if (!g_eb.inited || !frame) {
        if (g_eb.inited) g_eb.f_drop++;
        return;
    }
    lock();
    const int cap = g_eb.cfg.frame_capacity;
    g_eb.frames[g_eb.f_head] = *frame;
    g_eb.f_head = (g_eb.f_head + 1) % cap;
    if (g_eb.f_count < cap) g_eb.f_count++;
    g_eb.f_total++;
    unlock();
}

void event_buffer_push_window(const ev_buf_window_t *window)
{
    if (!g_eb.inited || !window) return;
    lock();
    const int cap = g_eb.cfg.window_capacity;
    g_eb.windows[g_eb.w_head] = *window;
    g_eb.w_head = (g_eb.w_head + 1) % cap;
    if (g_eb.w_count < cap) g_eb.w_count++;
    g_eb.w_total++;
    unlock();
}

/* -------------------------------------------------------------------------- */
/* Extract — synchronous                                                       */
/* -------------------------------------------------------------------------- */

/* Walk a ring chronologically (oldest → newest) and copy entries whose
 * ts ∈ [t_lo, t_hi] into out_array (allocated by caller). Returns count. */
#define DEFINE_RING_COLLECT(NAME, TYPE, RING, RING_HEAD, RING_COUNT, CAP)       \
static int collect_##NAME(uint64_t t_lo, uint64_t t_hi, TYPE *out, int out_cap) \
{                                                                                \
    const int cap   = (CAP);                                                     \
    const int count = (RING_COUNT);                                              \
    if (count == 0 || out_cap == 0) return 0;                                    \
    const int tail  = ((RING_HEAD) - count + cap) % cap;                         \
    int n = 0;                                                                    \
    for (int i = 0; i < count && n < out_cap; i++) {                             \
        const int idx = (tail + i) % cap;                                        \
        const uint64_t ts = (RING)[idx].ts_us;                                   \
        if (ts >= t_lo && ts <= t_hi) {                                          \
            out[n++] = (RING)[idx];                                              \
        }                                                                         \
    }                                                                             \
    return n;                                                                     \
}

DEFINE_RING_COLLECT(frame,  ev_buf_frame_t,  g_eb.frames,  g_eb.f_head, g_eb.f_count, g_eb.cfg.frame_capacity)
DEFINE_RING_COLLECT(window, ev_buf_window_t, g_eb.windows, g_eb.w_head, g_eb.w_count, g_eb.cfg.window_capacity)

esp_err_t event_buffer_extract_sync(
    uint64_t                   event_ts_us,
    float                      pre_sec,
    float                      post_sec,
    event_buffer_snapshot_t  **out_snap)
{
    if (!g_eb.inited || !out_snap) return ESP_ERR_INVALID_ARG;
    if (pre_sec < 0)  pre_sec  = 0;
    if (post_sec < 0) post_sec = 0;

    const uint64_t pre_us  = (uint64_t)(pre_sec  * 1.0e6f);
    const uint64_t post_us = (uint64_t)(post_sec * 1.0e6f);
    const uint64_t t_lo    = (event_ts_us > pre_us) ? (event_ts_us - pre_us) : 0;
    const uint64_t t_hi    = event_ts_us + post_us;

    /* Allocate worst-case output sized to current ring counts. */
    lock();
    const int frame_max  = g_eb.f_count;
    const int window_max = g_eb.w_count;
    unlock();

    event_buffer_snapshot_t *snap = (event_buffer_snapshot_t *)
        heap_caps_calloc(1, sizeof(*snap), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snap) return ESP_ERR_NO_MEM;

    if (frame_max > 0) {
        snap->frames = (ev_buf_frame_t *)psram_calloc(frame_max, sizeof(ev_buf_frame_t));
        if (!snap->frames) { event_buffer_snapshot_free(snap); return ESP_ERR_NO_MEM; }
    }
    if (window_max > 0) {
        snap->windows = (ev_buf_window_t *)psram_calloc(window_max, sizeof(ev_buf_window_t));
        if (!snap->windows) { event_buffer_snapshot_free(snap); return ESP_ERR_NO_MEM; }
    }

    lock();
    snap->n_frames  = collect_frame(t_lo,  t_hi, snap->frames,  frame_max);
    snap->n_windows = collect_window(t_lo, t_hi, snap->windows, window_max);
    unlock();

    snap->event_ts_us = event_ts_us;
    snap->pre_sec     = pre_sec;
    snap->post_sec    = post_sec;
    *out_snap = snap;
    return ESP_OK;
}

void event_buffer_snapshot_free(event_buffer_snapshot_t *snap)
{
    if (!snap) return;
    heap_caps_free(snap->frames);
    heap_caps_free(snap->windows);
    heap_caps_free(snap);
}

/* -------------------------------------------------------------------------- */
/* Async request — schedules extract_sync to fire `post_sec` later.            */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint64_t                       event_ts_us;
    float                          pre_sec;
    float                          post_sec;
    event_buffer_snapshot_cb_t     cb;
    void                          *ctx;
} async_req_t;

static void async_worker_task(void *arg)
{
    async_req_t *req = (async_req_t *)arg;

    /* Wait until now ≥ event_ts + post_sec (so post-window data is in ring). */
    const uint64_t deadline_us = req->event_ts_us +
                                  (uint64_t)(req->post_sec * 1.0e6f);
    int64_t wait_us = (int64_t)deadline_us - esp_timer_get_time();
    if (wait_us > 0) {
        vTaskDelay(pdMS_TO_TICKS((wait_us / 1000) + 50));   /* +50ms slack */
    }

    event_buffer_snapshot_t *snap = NULL;
    esp_err_t err = event_buffer_extract_sync(
        req->event_ts_us, req->pre_sec, req->post_sec, &snap);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "async extract failed: %d", err);
        snap = NULL;
    }

    if (req->cb) req->cb(snap, req->ctx);
    /* Caller owns snap now. If no cb, free here so we don't leak. */
    if (!req->cb && snap) event_buffer_snapshot_free(snap);

    free(req);
    vTaskDelete(NULL);
}

esp_err_t event_buffer_request_snapshot(
    uint64_t                       event_ts_us,
    float                          pre_sec,
    float                          post_sec,
    event_buffer_snapshot_cb_t     cb,
    void                          *ctx)
{
    if (!g_eb.inited) return ESP_ERR_INVALID_STATE;
    async_req_t *req = (async_req_t *)calloc(1, sizeof(*req));
    if (!req) return ESP_ERR_NO_MEM;
    req->event_ts_us = event_ts_us;
    req->pre_sec     = pre_sec;
    req->post_sec    = post_sec;
    req->cb          = cb;
    req->ctx         = ctx;

    BaseType_t r = xTaskCreate(async_worker_task, "evbuf_async", 4096, req, 5, NULL);
    if (r != pdPASS) { free(req); return ESP_FAIL; }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Stats                                                                       */
/* -------------------------------------------------------------------------- */
void event_buffer_get_stats(event_buffer_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!g_eb.inited) return;
    lock();
    out->frame_count          = g_eb.f_count;
    out->window_count         = g_eb.w_count;
    out->total_frames_pushed  = g_eb.f_total;
    out->total_windows_pushed = g_eb.w_total;
    out->push_drop_count      = g_eb.f_drop;
    unlock();
}
