/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Stage I post-filter on the Collapse FSM (ADR-023 M3.4).
 *
 * Maintains a 30-second rolling buffer of (timestamp, collapse_index)
 * pairs at the 1 Hz window cadence (collapse_update). On the rising edge
 * of `fall_flag` (the FSM's fall_detected boolean), extracts a 4-D
 * feature vector [max_C, mean_C, std_C, spike_duration_sec], cosine-matches
 * it against a small bank of reference fall patterns, and emits a
 * fall_detector_event_t carrying the match name + confidence.
 *
 * Direct port of hyperfi/poincare/fall_detector.py.
 *
 * The detector NEVER downgrades fall_flag → it only enriches the alert
 * with metadata. Cosine_similarity < match_threshold ⇒ confidence = 0,
 * meaning "FSM said fall, but the 30-s shape does not look like any
 * known fall pattern" (likely false-positive).
 *
 * Buffer sizing
 * -------------
 *   window_sec = 30.0
 *   cadence    = 1 Hz (ADR-023 §7)
 *   capacity   = 32 entries × 12 B = 384 B  ← stack/struct OK, no PSRAM needed
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Reference fall patterns (4-D feature vectors)                               */
/* -------------------------------------------------------------------------- */

#define FALL_FEATURE_DIM                  4
#define FALL_DETECTOR_DEFAULT_PATTERN_N   4
#define FALL_DETECTOR_BUFFER_CAPACITY     32   /* > 30 entries + slack */

typedef struct {
    const char *name;
    float       feature[FALL_FEATURE_DIM];   /* [max_C, mean_C, std_C, spike_dur_sec] */
} fall_pattern_t;

/* Hardcoded synthesized patterns — same numbers as
 * hyperfi/poincare/fall_detector.py:DEFAULT_REFERENCE_PATTERNS.
 * Replace from real pilot data in Phase 2. */
extern const fall_pattern_t fall_default_patterns[FALL_DETECTOR_DEFAULT_PATTERN_N];

/* -------------------------------------------------------------------------- */
/* Config + state                                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
    float                  window_sec;        /* default 30.0 */
    float                  match_threshold;   /* default 0.7  (ADR-023 §455) */
    float                  spike_threshold;   /* default 0.12 (collapse_threshold) */
    int                    n_patterns;        /* size of patterns[] */
    const fall_pattern_t  *patterns;          /* not owned by detector */
} fall_detector_config_t;

#define FALL_DETECTOR_CONFIG_DEFAULT() {                          \
    .window_sec      = 30.0f,                                     \
    .match_threshold = 0.7f,                                      \
    .spike_threshold = 0.12f,                                     \
    .n_patterns      = FALL_DETECTOR_DEFAULT_PATTERN_N,            \
    .patterns        = fall_default_patterns,                     \
}

typedef struct {
    uint64_t ts_us;
    float    collapse_index;
} fall_detector_entry_t;

typedef struct {
    fall_detector_config_t cfg;
    fall_detector_entry_t  buf[FALL_DETECTOR_BUFFER_CAPACITY];
    int                    head;            /* next write index */
    int                    count;           /* current entries (≤ capacity) */
    bool                   prev_fall_flag;
} fall_detector_t;

/* -------------------------------------------------------------------------- */
/* Event                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool        triggered;                  /* true only on rising edge of fall_flag */
    uint64_t    event_ts_us;
    float       confidence;                 /* 0 if best_similarity < match_threshold */
    int         best_pattern_idx;           /* -1 if no match (e.g. zero feature norm) */
    const char *best_pattern_name;          /* "" if best_pattern_idx < 0 */
    float       best_similarity;            /* cosine similarity, [0..1] */
    float       feature[FALL_FEATURE_DIM];  /* the 4-D vec at trigger time */
} fall_detector_event_t;

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

/** Initialize. Pass cfg=NULL to use defaults. */
void fall_detector_init(fall_detector_t *d, const fall_detector_config_t *cfg);

/** Drop history, keep config. */
void fall_detector_reset(fall_detector_t *d);

/**
 * Push one (ts, C, fall_flag). On the rising edge of fall_flag,
 * out->triggered = true and event fields are populated. Otherwise
 * out->triggered = false.
 */
void fall_detector_update(
    fall_detector_t            *d,
    uint64_t                    ts_us,
    float                       collapse_index,
    bool                        fall_flag,
    fall_detector_event_t      *out);

#ifdef __cplusplus
}
#endif
