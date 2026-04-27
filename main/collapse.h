/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Collapse Index detector — fall detection state machine
 *
 * Port of hyperfi/poincare/collapse.py to P4 C, per ADR-023 M3.2.
 * Inputs window-mean amplitudes (per ~1 Hz), outputs Collapse Index +
 * fall_detected flag via a SPIKE → SILENCE state machine.
 *
 * Pipeline per update():
 *   amp[53] → mean-normalize → select 8 SCs → poincare_embed (scale=0.5)
 *           → EMA(α=0.3) → ||ema_z|| < 1 clamp
 *           → C = geodesic_distance(ema_z, prev_ema_z) / Δt
 *           → state machine (MONITORING / SPIKE_DETECTED / fall_detected)
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                                */
/* -------------------------------------------------------------------------- */

typedef struct {
    int   n_input_dim;            /* Input amplitude vector length, typ. 53 */
    int   n_embed_dim;            /* Poincaré embedding dim, typ. 8 */
    float embed_scale;            /* Poincaré ball radius limit, typ. 0.5 */
    float ema_alpha;              /* EMA coefficient (0=slow ... 1=no smoothing), typ. 0.3 */
    float collapse_threshold;     /* C above this → enter SPIKE_DETECTED, typ. 0.12 */
    float silence_threshold;      /* C below this counts toward "silence", typ. 0.04 */
    int   silence_count_target;   /* # consecutive silent updates → fall_detected, typ. 3 */
    float spike_timeout_sec;      /* SPIKE without silence resolves → MONITORING, typ. 5.0 */
} collapse_config_t;

#define COLLAPSE_CONFIG_DEFAULT() { \
    .n_input_dim          = 53,   \
    .n_embed_dim          = 8,    \
    .embed_scale          = 0.5f, \
    .ema_alpha            = 0.3f, \
    .collapse_threshold   = 0.12f,\
    .silence_threshold    = 0.04f,\
    .silence_count_target = 3,    \
    .spike_timeout_sec    = 5.0f, \
}

/* -------------------------------------------------------------------------- */
/* Output                                                                       */
/* -------------------------------------------------------------------------- */

typedef enum {
    COLLAPSE_FSM_MONITORING     = 0,
    COLLAPSE_FSM_SPIKE_DETECTED = 1,
} collapse_fsm_state_t;

typedef struct {
    float                collapse_index;   /* Δd / Δt geodesic */
    collapse_fsm_state_t state;            /* Current FSM state after update */
    bool                 fall_detected;    /* TRUE on the update where SPIKE → silence-confirmed */
    float                embedding_norm;   /* ||ema_z|| for telemetry */
} collapse_result_t;

/* -------------------------------------------------------------------------- */
/* Opaque state                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct collapse_handle_s collapse_handle_t;

/* -------------------------------------------------------------------------- */
/* API                                                                          */
/* -------------------------------------------------------------------------- */

/**
 * Allocate detector state. Embedding + EMA buffers allocated in PSRAM.
 *
 * @param[out] out_handle  Set on success
 * @param[in]  cfg         Configuration; copied into state
 * @return  ESP_OK or ESP_ERR_NO_MEM / ESP_ERR_INVALID_ARG
 */
esp_err_t collapse_init(collapse_handle_t **out_handle, const collapse_config_t *cfg);

/**
 * Free state.
 */
void collapse_deinit(collapse_handle_t *handle);

/**
 * Reset state machine + EMA + history. Configuration is preserved.
 */
void collapse_reset(collapse_handle_t *handle);

/**
 * Process one window of amplitude data (typically called at 1 Hz).
 *
 * @param[in]  handle         Initialized detector
 * @param[in]  amp_input      Length cfg.n_input_dim float vector (mean amplitude per SC)
 * @param[in]  timestamp_us   Monotonic timestamp in microseconds
 * @param[out] result         Output (collapse index, state, fall_detected, ema norm)
 * @return  ESP_OK
 */
esp_err_t collapse_update(
    collapse_handle_t *handle,
    const float *amp_input,
    uint64_t timestamp_us,
    collapse_result_t *result);

#ifdef __cplusplus
}
#endif
