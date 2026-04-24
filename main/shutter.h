/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * HyperFi Spatial Shutter — IFFT-based CIR gating + baseline subtraction
 *
 * Port of hyperfi/csi/shutter.py to P4 C, per ADR-023 M3.1.
 * Pipeline:  H(k) -> zero-pad -> IFFT -> gate |τ| > τ_threshold
 *                 -> (optional) subtract h_baseline -> FFT -> H_clean(k)
 *
 * Implementation uses ESP-DSP for IFFT/FFT (V9 benchmark confirmed 128-pt
 * round-trip completes in ~138µs on P4 360MHz).
 *
 * Numerics: all float32 (P4 has hardware single-precision FPU only).
 * Reference impl (Python float64) is gold standard; per-SC relative error
 * budget < 1e-3 vs. Python reference output on the same input.
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
/* Complex numbers: ESP-DSP uses interleaved real/imag pairs (float[2*N]).    */
/* -------------------------------------------------------------------------- */

/* Pack complex float[2] into float[2*N]: out[2k]=re, out[2k+1]=im */

/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */

typedef enum {
    SHUTTER_FRAME_HT20 = 0,   /* 312.5 kHz subcarrier spacing */
    SHUTTER_FRAME_HT40 = 1,   /* 312.5 kHz */
    SHUTTER_FRAME_VHT  = 2,   /* 312.5 kHz */
    SHUTTER_FRAME_HE   = 3,   /* 78.125 kHz */
} shutter_frame_type_t;

typedef struct {
    float room_size_m;              /* max room dimension [m], default 8.0 */
    float margin_m;                 /* wall reflection margin [m], default 2.0 */
    float baseline_ema_alpha;       /* baseline EMA update coefficient, default 0.005 */
    float drift_threshold;          /* drift score threshold for recalibration, default 0.5 */
    float quiet_energy_threshold;   /* normalized dynamic energy for "quiet" detection, default 0.01 */
    int   n_subcarriers;            /* input CSI size (53 for HT20), required */
    int   fft_size;                 /* IFFT/FFT size, must be power of 2 >= n_subcarriers (default 128) */
} shutter_config_t;

#define SHUTTER_CONFIG_DEFAULT() { \
    .room_size_m            = 8.0f,   \
    .margin_m               = 2.0f,   \
    .baseline_ema_alpha     = 0.005f, \
    .drift_threshold        = 0.5f,   \
    .quiet_energy_threshold = 0.01f,  \
    .n_subcarriers          = 53,     \
    .fft_size               = 128,    \
}

/* -------------------------------------------------------------------------- */
/* Per-frame output metadata                                                   */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint32_t frame_count;
    int      gate_idx;                  /* CIR bin index cutoff */
    float    indoor_power_ratio;        /* power(τ ≤ gate) / total CIR power */
    float    snr_improvement_db;        /* 10 log10(indoor / outdoor) */
    bool     baseline_active;           /* true if baseline subtraction was applied */
    bool     calibrating;               /* true if in calibration collect phase */
    float    calibration_progress;      /* 0.0 ~ 1.0 during calibration */
    float    dynamic_energy_ratio;      /* power(h_dynamic) / power(h_baseline) */
    float    drift_score;               /* normalized drift indicator */
    bool     needs_recalibration;       /* true if drift exceeded threshold */
    bool     is_quiet;                  /* derived: dynamic_energy_ratio < quiet_energy_threshold */
} shutter_metadata_t;

/* -------------------------------------------------------------------------- */
/* Opaque state handle (allocated / freed by init / deinit)                   */
/* -------------------------------------------------------------------------- */

typedef struct shutter_state_s shutter_state_t;

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * Allocate and initialize shutter state. Input/output buffers are internally
 * allocated in PSRAM. Caller retains ownership of `cfg` only for the init call;
 * values are copied into the state.
 *
 * @param[out] out_state   Pointer to state handle, set on success.
 * @param[in]  cfg         Configuration; must have valid n_subcarriers, fft_size.
 * @return  ESP_OK on success, ESP_ERR_NO_MEM on allocation failure,
 *          ESP_ERR_INVALID_ARG on bad config.
 */
esp_err_t shutter_init(shutter_state_t **out_state, const shutter_config_t *cfg);

/**
 * Free all resources associated with the state handle.
 */
void shutter_deinit(shutter_state_t *state);

/* -------------------------------------------------------------------------- */
/* Calibration                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * Begin baseline calibration. The next `num_frames` calls to `shutter_process()`
 * will accumulate into the baseline buffer; on the last frame the baseline is
 * averaged and activated. During calibration, baseline subtraction is NOT
 * applied (raw gated CIR passes through).
 *
 * @param num_frames   Number of frames to average (typical 100-500, static env).
 */
esp_err_t shutter_start_calibration(shutter_state_t *state, int num_frames);

/**
 * Whether a baseline CIR has been established (either via finished calibration
 * or explicit `shutter_set_baseline()`).
 */
bool shutter_has_baseline(const shutter_state_t *state);

/**
 * Directly set baseline CIR (e.g., restored from NVS after reboot).
 * `baseline_cir` is N_fft complex (interleaved float[2 * fft_size]).
 */
void shutter_set_baseline(shutter_state_t *state, const float *baseline_cir_interleaved);

/**
 * Clear baseline, revert to gating-only mode.
 */
void shutter_clear_baseline(shutter_state_t *state);

/* -------------------------------------------------------------------------- */
/* Per-frame processing                                                        */
/* -------------------------------------------------------------------------- */

/**
 * Apply Spatial Shutter to one CSI frame.
 *
 * Input format: `H_freq` is N_sc complex (interleaved float[2 * n_subcarriers]).
 * Internally zero-padded to `fft_size` and processed.
 *
 * Output: `H_filtered` is also N_sc complex (interleaved float[2 * n_subcarriers])
 * — the first N_sc bins of the FFT-reconstructed signal.
 *
 * @param[in,out] state         Initialized state handle.
 * @param[in]     H_freq        Input CSI, interleaved float[2 * n_subcarriers].
 * @param[in]     frame_type    Frame type for subcarrier spacing lookup.
 * @param[out]    H_filtered    Filtered CSI, interleaved float[2 * n_subcarriers].
 * @param[out]    meta          Per-frame diagnostics (nullable, optional).
 * @return  ESP_OK on success.
 */
esp_err_t shutter_process(
    shutter_state_t *state,
    const float *H_freq,
    shutter_frame_type_t frame_type,
    float *H_filtered,
    shutter_metadata_t *meta
);

#ifdef __cplusplus
}
#endif
