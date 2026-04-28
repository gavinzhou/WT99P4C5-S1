/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * event_buffer — ADR-023 M3.6.0 Tier B rolling buffer.
 *
 * Holds the most recent ~60 seconds of CSI raw frames + per-window
 * derived metrics in PSRAM, so that when M3.4 fall_detector raises
 * a fall_event_rising_edge, M3.6 can extract a ±30 s context window
 * and ship it to the cloud (Mac mock S3 in M3 PoC, AWS S3 in Phase 2).
 *
 * Two independent rings:
 *   - frame ring  (raw CSI, ~14 400 entries @ 200 fps × 60 s × 1.2 slack)
 *                 sized to keep a generous window even at peak fps.
 *   - window ring (derived per-1Hz-window snapshots, ~72 entries)
 *                 keeps collapse_index / embed / norm_cv etc.
 *
 * Memory footprint @ HT20 53 SC:
 *   frames:  14 400 × sizeof(ev_buf_frame_t)  ≈ 1.7 MB PSRAM
 *   windows:    72 × sizeof(ev_buf_window_t)  ≈ 6 KB PSRAM
 *
 * Threading: push_frame is called from the CSI consumer (RPC dispatch
 * thread). push_window is called from pipeline.c emit (same thread).
 * extract_sync may be called from any thread; it walks the rings under
 * the buffer's internal mutex. Snapshots are allocated in PSRAM and
 * owned by the caller — call event_buffer_snapshot_free.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EV_BUF_N_SC                 53
#define EV_BUF_IQ_BYTES             (2 * EV_BUF_N_SC)   /* 106 B for HT20 8-bit */
#define EV_BUF_EMBED_DIM            8

/* -------------------------------------------------------------------------- */
/* Ring entry layouts                                                          */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint64_t  ts_us;                       /* monotonic timestamp */
    uint16_t  seq;                         /* slave-side seq number */
    int8_t    rssi;
    int8_t    noise_floor;
    uint8_t   bw;
    uint8_t   reserved[3];                 /* pad to 16 B */
    int8_t    iq[EV_BUF_IQ_BYTES];         /* 106 B raw int8 [Im, Re] interleaved */
} ev_buf_frame_t;                          /* total 16 + 106 = 122 B */

typedef struct {
    uint64_t  ts_us;
    float     collapse_index;
    float     norm_cv;
    float     raw_cv;
    float     shape_corr;
    float     dynamic_gain_G;
    float     embedding_norm;
    float     poincare_embed[EV_BUF_EMBED_DIM];
    int32_t   rssi_avg;
    uint16_t  fsm_state;                   /* 0=MONITORING, 1=SPIKE_DETECTED */
    uint16_t  flags;                       /* bit0 = quiet_period, bit1 = fall_detected */
} ev_buf_window_t;                         /* ≈ 80 B */

/* -------------------------------------------------------------------------- */
/* Config                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct {
    int   frame_capacity;                  /* default 14400 */
    int   window_capacity;                 /* default 72 */
    float window_sec;                      /* total retained, default 60.0 */
} event_buffer_config_t;

#define EVENT_BUFFER_CONFIG_DEFAULT() {    \
    .frame_capacity  = 14400,              \
    .window_capacity = 72,                 \
    .window_sec      = 60.0f,              \
}

/* -------------------------------------------------------------------------- */
/* Snapshot output                                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint64_t          event_ts_us;
    float             pre_sec;             /* requested look-back */
    float             post_sec;            /* requested look-ahead */
    int               n_frames;            /* count actually extracted */
    int               n_windows;
    ev_buf_frame_t   *frames;              /* owned, PSRAM, contiguous, ts-ascending */
    ev_buf_window_t  *windows;             /* owned, PSRAM, contiguous, ts-ascending */
} event_buffer_snapshot_t;

typedef void (*event_buffer_snapshot_cb_t)(event_buffer_snapshot_t *snap, void *ctx);

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t event_buffer_init(const event_buffer_config_t *cfg);
void      event_buffer_deinit(void);
void      event_buffer_reset(void);

/* -------------------------------------------------------------------------- */
/* Push (on the CSI consumer / pipeline emit threads)                          */
/* -------------------------------------------------------------------------- */

void event_buffer_push_frame(const ev_buf_frame_t *frame);
void event_buffer_push_window(const ev_buf_window_t *window);

/* -------------------------------------------------------------------------- */
/* Extract                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * Synchronous extract: copy out everything currently in the rings whose
 * timestamps fall in [event_ts_us - pre_sec, event_ts_us + post_sec].
 *
 * Caller takes ownership of *out_snap and must call event_buffer_snapshot_free.
 *
 * Returns:
 *   ESP_OK on success (out_snap populated, may have n=0 if no entries match)
 *   ESP_ERR_NO_MEM if snapshot PSRAM alloc fails
 *   ESP_ERR_INVALID_ARG if cfg or out_snap is NULL
 */
esp_err_t event_buffer_extract_sync(
    uint64_t                   event_ts_us,
    float                      pre_sec,
    float                      post_sec,
    event_buffer_snapshot_t  **out_snap);

/**
 * Asynchronous request: schedule extraction to fire `post_sec` after
 * `event_ts_us` (so the post-window data has accumulated). When the
 * snapshot is ready the callback fires on a worker task with an owned
 * snapshot — caller must free it.
 *
 * Useful production wiring from pipeline.c on fall_event_rising_edge.
 */
esp_err_t event_buffer_request_snapshot(
    uint64_t                       event_ts_us,
    float                          pre_sec,
    float                          post_sec,
    event_buffer_snapshot_cb_t     cb,
    void                          *ctx);

void event_buffer_snapshot_free(event_buffer_snapshot_t *snap);

/* -------------------------------------------------------------------------- */
/* Stats (for monitor / telemetry)                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    int       frame_count;          /* current entries in frame ring */
    int       window_count;
    uint32_t  total_frames_pushed;  /* lifetime, may wrap */
    uint32_t  total_windows_pushed;
    uint32_t  push_drop_count;      /* should be 0 in normal operation */
} event_buffer_stats_t;

void event_buffer_get_stats(event_buffer_stats_t *out);

#ifdef __cplusplus
}
#endif
