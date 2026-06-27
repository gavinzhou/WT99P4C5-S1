/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * HyperFi Breathing Rate Estimation — streaming CSI → BPM
 *
 * Firmware mirror of the Python gold-standard `hyperfi/tools/breathing_analyzer.py`
 * (v3.1). See `hyperfi/docs/engineering/breathing-firmware-port-plan.md` (M4.3).
 *
 * Pipeline (v1, amplitude-only):
 *   CSI amp per SC (~300 fps)
 *     -> decimate to 10 Hz (30-frame mean)
 *     -> Butterworth band-pass 0.1–0.5 Hz (cascade biquad, forward-only)
 *     -> Top-K SC by in-band variance
 *     -> Welch PSD (Hann, 30 s window) per selected SC
 *     -> peak detect + harmonic correction + preferred-band (0.13–0.35 Hz) boost
 *     -> multi-SC voting (median outlier rejection)
 *     -> 5 s cadence: emit BPM + confidence + state
 *
 * v1 is amplitude-only (PHASE_WEIGHT = 0); phase fusion is v2 (post-PoC).
 * Patent: breathing estimation is P5.5 core; PoC disclosure is covered by the
 * MFR NDA (限定開示), so firmware implementation is permitted.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
    int   n_subcarriers;     /* CSI subcarriers fed in (53 for HT20)            */
    float resample_fs;       /* uniform grid rate after decimation, Hz (10.0)   */
    float band_lo_hz;        /* band-pass low cutoff  (0.10 Hz = 6 bpm)         */
    float band_hi_hz;        /* band-pass high cutoff (0.50 Hz = 30 bpm)        */
    float pref_lo_hz;        /* preferred band low  (0.13 Hz ≈ 8 bpm)           */
    float pref_hi_hz;        /* preferred band high (0.35 Hz ≈ 21 bpm)          */
    int   top_k;             /* SC ranked/kept by in-band variance (5)          */
    int   n_vote;            /* SC participating in BPM voting (7)              */
    float window_sec;        /* analysis window length (30 s)                   */
    float step_sec;          /* emit/update cadence (5 s)                       */
    int   fft_size;          /* Welch segment FFT size, power of two (256)      */
    float min_confidence;    /* below this, BPM is held / not published (1.5)   */
} breathing_config_t;

/* Defaults mirror breathing_analyzer.py constants. */
#define BREATHING_CONFIG_DEFAULT() (breathing_config_t){ \
    .n_subcarriers  = 53,    \
    .resample_fs    = 10.0f, \
    .band_lo_hz     = 0.10f, \
    .band_hi_hz     = 0.50f, \
    .pref_lo_hz     = 0.13f, \
    .pref_hi_hz     = 0.35f, \
    .top_k          = 5,     \
    .n_vote         = 7,     \
    .window_sec     = 30.0f, \
    .step_sec       = 5.0f,  \
    .fft_size       = 256,   \
    .min_confidence = 1.5f,  \
}

/* -------------------------------------------------------------------------- */
/* Output                                                                     */
/* -------------------------------------------------------------------------- */

typedef enum {
    BREATHING_COLD = 0,   /* no data yet                                       */
    BREATHING_WARMUP,     /* filling the first analysis window (~30 s)         */
    BREATHING_TRACKING,   /* valid BPM estimate available                      */
    BREATHING_LOST,       /* signal too weak / confidence below floor          */
} breathing_state_e;

typedef struct {
    breathing_state_e state;
    float    bpm;             /* latest breaths-per-minute (valid if TRACKING)  */
    float    confidence;      /* peak-to-noise ratio of the PSD vote            */
    uint32_t window_count;    /* number of completed analysis windows           */
    uint64_t last_update_us;  /* esp_timer timestamp of last emit               */
} breathing_result_t;

/* -------------------------------------------------------------------------- */
/* API (opaque handle, allocates internal + PSRAM buffers — cf. shutter.h)    */
/* -------------------------------------------------------------------------- */

typedef struct breathing_s breathing_t;

/**
 * Allocate + initialise. Returns ESP_ERR_NO_MEM on alloc failure,
 * ESP_ERR_INVALID_ARG on bad config (n_subcarriers/fft_size).
 */
esp_err_t breathing_init(breathing_t **out_state, const breathing_config_t *cfg);

/**
 * Feed one CSI frame. `amp[n_subcarriers]` = sqrt(I²+Q²) per SC.
 * `phase` is reserved for v2 fusion and MUST be NULL in v1 (ignored if non-NULL).
 * `ts_us` = frame timestamp (esp_timer_get_time). Non-blocking, cheap; the
 * heavy PSD runs internally on the step cadence, not every frame.
 */
void breathing_on_frame(breathing_t *state, const float *amp,
                        const float *phase, uint64_t ts_us);

/** Snapshot the latest estimate. Safe to call from any thread. */
void breathing_get(const breathing_t *state, breathing_result_t *out);

/** Free all buffers. */
void breathing_deinit(breathing_t *state);

/* -------------------------------------------------------------------------- */
/* Exposed for self-test (m_breathing_test.c): pure Welch-PSD estimator on a   */
/* 10 Hz signal. Mirrors gen_breathing_fixtures.py welch_estimate (nfft=256).  */
/* Returns BPM; writes peak/median confidence to *out_conf. Allocates scratch  */
/* internally (not perf-critical — test/diagnostic use).                       */
/* -------------------------------------------------------------------------- */
float breathing_estimate_welch(const breathing_config_t *cfg,
                              const float *sig, int n, float *out_conf);

#ifdef __cplusplus
}
#endif
