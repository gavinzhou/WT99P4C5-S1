/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * pipeline — Live CSI integration (ADR-023 M3.5.0).
 *
 * Sits between the CSI consumer (on_csi_from_slave) and the future MQTT
 * publisher. Owns:
 *   - shutter (M3.1)        — per-frame IFFT → gate → FFT
 *   - amplitude ring buffer — 3 s sliding window, time-fixed
 *   - metrics (M3.3)        — per-window norm_cv / shape_corr / G / quiet_det
 *   - poincare (M3.2)       — 53→8 select + embed for telemetry
 *   - collapse (M3.2)       — Δd / Δt + FSM + fall_detected flag
 *   - fall_detector (M3.4)  — Stage I post-filter, confidence
 *
 * On every CSI frame: convert int8 [Im, Re] wire layout → complex float
 * [Re, Im] interleaved, push through shutter, push amplitude row to ring.
 *
 * On window stride boundary (default 1 Hz): aggregate metrics, compute
 * embedding, run collapse + fall_detector, then invoke the registered
 * telemetry callback. Callback runs on the same thread as the frame
 * delivery (RPC dispatch). Single-instance / non-reentrant.
 *
 * No MQTT here. The callback hands a snapshot to the publisher; that
 * separation lets pipeline be unit-testable and lets the publisher
 * choose its own threading / queueing policy.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Config                                                                      */
/* -------------------------------------------------------------------------- */

#define PIPELINE_N_SC_HT20            53
#define PIPELINE_EMBED_DIM            8
#define PIPELINE_DEFAULT_WINDOW_SEC   3.0f
#define PIPELINE_DEFAULT_STRIDE_MS    1000

typedef struct {
    int   n_subcarriers;     /* 53 for HT20 */
    float window_sec;        /* 3.0 (ADR-023 §256) */
    int   window_stride_ms;  /* 1000 = 1 Hz */
    int   ring_capacity;     /* max frames in buffer; sized for window_sec × max_fps */
    bool  bypass_shutter;    /* skip M3.1 shutter, push raw |H| straight to metrics
                              * — diagnostic: real WiFi CSI has frame-level CFO that
                              * the current shutter (no CFO correction) doesn't handle.
                              * Use bypass=true until CFO preprocessing is added in
                              * M3.5.x. M3.1 algorithm is still valid, just needs the
                              * CFO-corrected input the synthetic fixtures already had. */
} pipeline_config_t;

#define PIPELINE_CONFIG_DEFAULT() {                                  \
    .n_subcarriers    = PIPELINE_N_SC_HT20,                          \
    .window_sec       = PIPELINE_DEFAULT_WINDOW_SEC,                 \
    .window_stride_ms = PIPELINE_DEFAULT_STRIDE_MS,                  \
    .ring_capacity    = 2200,  /* 600 fps × 3 s × ~1.2 slack */      \
    .bypass_shutter   = true,  /* default ON for M3.5.0 hardware bringup;          \
                                * flip to false once CFO correction lands.         */ \
}

/* -------------------------------------------------------------------------- */
/* Telemetry snapshot — emitted once per window stride                          */
/* -------------------------------------------------------------------------- */

typedef enum {
    PIPELINE_FSM_MONITORING     = 0,
    PIPELINE_FSM_SPIKE_DETECTED = 1,
} pipeline_fsm_state_t;

typedef struct {
    /* Window context */
    uint64_t timestamp_us;                  /* end-of-window timestamp */
    int      n_frames_in_window;
    float    csi_fps;                       /* rolling fps in last window stride */

    /* Per-window metrics (M3.3) */
    float    norm_cv;
    float    raw_cv;
    float    shape_corr;
    float    dynamic_gain_G;
    bool     quiet_period;
    int      n_valid_subcarriers;

    /* Poincaré (M3.2) — top-8 SC selected, hyperbolic embed */
    float    poincare_embed[PIPELINE_EMBED_DIM];

    /* Collapse FSM (M3.2) */
    float                collapse_index;
    pipeline_fsm_state_t fsm_state;
    float                embedding_norm;    /* ||ema_z|| */

    /* Fall detector (M3.4) */
    bool        fall_detected;              /* FSM raised this window */
    bool        fall_event_rising_edge;     /* true ONLY on rising edge */
    float       fall_confidence;            /* 0 if best_sim < 0.7 */
    int         fall_best_pattern_idx;      /* -1 if no match */
    const char *fall_best_pattern_name;     /* "" if no match */

    /* CSI link health */
    int      rssi_avg;
    int      noise_floor_avg;
    uint32_t csi_drop_count;                /* cumulative since boot */

    /* Breathing (M4.3) — amplitude-only Welch estimate. bpm valid only when
     * breathing_state == 2 (TRACKING). */
    float    breathing_bpm;
    float    breathing_confidence;
    uint32_t breathing_state;               /* 0=cold 1=warmup 2=tracking 3=lost */
} pipeline_telemetry_t;

/** Telemetry callback. Invoked on the frame-delivery thread; do NOT block. */
typedef void (*pipeline_telemetry_cb_t)(const pipeline_telemetry_t *t,
                                         void *ctx);

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

/**
 * Initialize pipeline (single instance). Allocates ring buffer + scratch
 * in PSRAM, brings up shutter / collapse / quiet_det / fall_det.
 *
 * Pass cfg=NULL to use PIPELINE_CONFIG_DEFAULT().
 */
esp_err_t pipeline_init(const pipeline_config_t *cfg);

/** Free all allocations + tear down sub-modules. */
void pipeline_deinit(void);

/** Reset rolling state (ring, FSM, fall_det history) without reallocating. */
void pipeline_reset(void);

/**
 * Register a telemetry callback. Pass cb=NULL to clear.
 * The callback receives a stack-allocated snapshot whose lifetime ends
 * when the callback returns — copy out anything you need.
 */
void pipeline_register_telemetry_cb(pipeline_telemetry_cb_t cb, void *ctx);

/**
 * Push one CSI frame.
 *
 * @param ts_us         Monotonic timestamp (esp_timer_get_time()).
 * @param rssi          RSSI in dBm (typ. -90..-20).
 * @param noise_floor   Noise floor in dBm (typ. -95).
 * @param iq_data       Wire-format int8 [Im0, Re0, Im1, Re1, ...] of length 2*n_sc.
 * @param n_sc          Number of subcarriers (must equal cfg.n_subcarriers).
 */
void pipeline_on_csi_frame(
    uint64_t       ts_us,
    int8_t         rssi,
    int8_t         noise_floor,
    const int8_t  *iq_data,
    int            n_sc);

/** Drop counter — increment when a frame can't be processed (alloc fail etc). */
void pipeline_record_drop(void);

#ifdef __cplusplus
}
#endif
