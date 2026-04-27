/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * fall_detector — Stage I post-filter (ADR-023 M3.4).
 * Direct port of hyperfi/poincare/fall_detector.py with float32 throughout.
 */

#include "fall_detector.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Default reference patterns — must match Python ref's                        */
/* DEFAULT_REFERENCE_PATTERNS in fall_detector.py.                             */
/* Order:                  [max_C, mean_C, std_C, spike_dur_sec]               */
/* -------------------------------------------------------------------------- */
const fall_pattern_t fall_default_patterns[FALL_DETECTOR_DEFAULT_PATTERN_N] = {
    { .name = "stand_collapse_fast", .feature = { 0.45f, 0.06f, 0.11f, 1.5f } },
    { .name = "stand_collapse_slow", .feature = { 0.22f, 0.07f, 0.06f, 2.5f } },
    { .name = "sit_to_floor",        .feature = { 0.30f, 0.05f, 0.07f, 1.0f } },
    { .name = "trip_then_fall",      .feature = { 0.35f, 0.08f, 0.09f, 2.0f } },
};

/* -------------------------------------------------------------------------- */
/* Init / reset                                                                */
/* -------------------------------------------------------------------------- */
void fall_detector_init(fall_detector_t *d, const fall_detector_config_t *cfg)
{
    if (!d) return;
    if (cfg) {
        d->cfg = *cfg;
    } else {
        const fall_detector_config_t def = FALL_DETECTOR_CONFIG_DEFAULT();
        d->cfg = def;
    }
    fall_detector_reset(d);
}

void fall_detector_reset(fall_detector_t *d)
{
    if (!d) return;
    d->head           = 0;
    d->count          = 0;
    d->prev_fall_flag = false;
    memset(d->buf, 0, sizeof(d->buf));
}

/* -------------------------------------------------------------------------- */
/* Buffer helpers                                                              */
/* -------------------------------------------------------------------------- */

/* Push (ts, C) at head, advance head, then evict any entries older than
 * window_sec relative to ts. */
static void buf_push(fall_detector_t *d, uint64_t ts_us, float C)
{
    d->buf[d->head].ts_us         = ts_us;
    d->buf[d->head].collapse_index = C;
    d->head = (d->head + 1) % FALL_DETECTOR_BUFFER_CAPACITY;
    if (d->count < FALL_DETECTOR_BUFFER_CAPACITY) {
        d->count++;
    }

    /* Evict from tail (oldest) entries where ts < ts_us - window_us. */
    const uint64_t window_us = (uint64_t)(d->cfg.window_sec * 1.0e6f);
    while (d->count > 0) {
        const int tail = (d->head - d->count + FALL_DETECTOR_BUFFER_CAPACITY)
                       % FALL_DETECTOR_BUFFER_CAPACITY;
        const uint64_t tail_ts = d->buf[tail].ts_us;
        if (ts_us > tail_ts && (ts_us - tail_ts) > window_us) {
            d->count--;
        } else {
            break;
        }
    }
}

/* Index into buffer in chronological order: 0 = oldest, count-1 = newest. */
static inline const fall_detector_entry_t *buf_at(
    const fall_detector_t *d, int chrono_idx)
{
    const int tail = (d->head - d->count + FALL_DETECTOR_BUFFER_CAPACITY)
                   % FALL_DETECTOR_BUFFER_CAPACITY;
    const int idx  = (tail + chrono_idx) % FALL_DETECTOR_BUFFER_CAPACITY;
    return &d->buf[idx];
}

/* -------------------------------------------------------------------------- */
/* Feature extraction                                                          */
/* -------------------------------------------------------------------------- */
static void compute_features(const fall_detector_t *d, float feat_out[4])
{
    if (d->count == 0) {
        feat_out[0] = feat_out[1] = feat_out[2] = feat_out[3] = 0.0f;
        return;
    }

    /* max, sum, sum_sq, longest spike run */
    float    max_C       = -INFINITY;
    float    sum_C       = 0.0f;
    int      run_start   = -1;        /* chronological index */
    float    spike_dur   = 0.0f;
    const float spike_th = d->cfg.spike_threshold;

    for (int i = 0; i < d->count; i++) {
        const fall_detector_entry_t *e = buf_at(d, i);
        const float C = e->collapse_index;
        if (C > max_C) max_C = C;
        sum_C += C;

        if (C > spike_th) {
            if (run_start < 0) run_start = i;
            const float run_dur =
                (float)(e->ts_us - buf_at(d, run_start)->ts_us) / 1.0e6f;
            if (run_dur > spike_dur) spike_dur = run_dur;
        } else {
            run_start = -1;
        }
    }

    const float mean_C = sum_C / (float)d->count;

    /* Population variance (ddof=0) — second pass for numerical stability. */
    float ss = 0.0f;
    for (int i = 0; i < d->count; i++) {
        const float dC = buf_at(d, i)->collapse_index - mean_C;
        ss += dC * dC;
    }
    const float var_C = ss / (float)d->count;
    const float std_C = sqrtf(var_C);

    feat_out[0] = max_C;
    feat_out[1] = mean_C;
    feat_out[2] = std_C;
    feat_out[3] = spike_dur;
}

/* -------------------------------------------------------------------------- */
/* Cosine match                                                                */
/* -------------------------------------------------------------------------- */
static int cosine_match(
    const fall_detector_t *d,
    const float            feat[4],
    float                 *out_best_sim)
{
    *out_best_sim = 0.0f;

    /* ||feat|| */
    float f_sq = 0.0f;
    for (int i = 0; i < FALL_FEATURE_DIM; i++) f_sq += feat[i] * feat[i];
    const float f_norm = sqrtf(f_sq);
    if (f_norm < 1e-10f) return -1;

    int   best_idx = -1;
    float best_sim = -1.0f;
    for (int p = 0; p < d->cfg.n_patterns; p++) {
        const fall_pattern_t *ref = &d->cfg.patterns[p];
        float r_sq = 0.0f, dot = 0.0f;
        for (int i = 0; i < FALL_FEATURE_DIM; i++) {
            r_sq += ref->feature[i] * ref->feature[i];
            dot  += feat[i] * ref->feature[i];
        }
        const float r_norm = sqrtf(r_sq);
        if (r_norm < 1e-10f) continue;
        const float sim = dot / (f_norm * r_norm);
        if (sim > best_sim) {
            best_sim = sim;
            best_idx = p;
        }
    }

    /* Cosine sim can be slightly negative if feature vectors point apart;
     * clamp to [0, 1] for the output to match Python ref behaviour. */
    if (best_sim < 0.0f) best_sim = 0.0f;
    *out_best_sim = best_sim;
    return best_idx;
}

/* -------------------------------------------------------------------------- */
/* Public update                                                               */
/* -------------------------------------------------------------------------- */
void fall_detector_update(
    fall_detector_t       *d,
    uint64_t               ts_us,
    float                  collapse_index,
    bool                   fall_flag,
    fall_detector_event_t *out)
{
    if (!d || !out) return;

    /* Always update buffer first so the rising-edge window includes the
     * newest sample at trigger time. */
    buf_push(d, ts_us, collapse_index);

    const bool rising_edge = (fall_flag && !d->prev_fall_flag);
    d->prev_fall_flag = fall_flag;

    /* Default empty event */
    memset(out, 0, sizeof(*out));
    out->triggered         = false;
    out->best_pattern_idx  = -1;
    out->best_pattern_name = "";

    if (!rising_edge) return;

    out->triggered    = true;
    out->event_ts_us  = ts_us;
    compute_features(d, out->feature);

    float best_sim = 0.0f;
    const int best_idx = cosine_match(d, out->feature, &best_sim);
    out->best_similarity = best_sim;

    if (best_idx < 0) {
        /* zero feature norm — cannot match */
        return;
    }

    out->best_pattern_idx  = best_idx;
    out->best_pattern_name = d->cfg.patterns[best_idx].name;
    out->confidence = (best_sim >= d->cfg.match_threshold) ? best_sim : 0.0f;
}
