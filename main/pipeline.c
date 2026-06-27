/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * pipeline — see pipeline.h.
 */

#include "pipeline.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "shutter.h"
#include "metrics.h"
#include "poincare.h"
#include "collapse.h"
#include "fall_detector.h"
#include "event_buffer.h"
#include "breathing.h"
#include "hf_config.h"

static const char *TAG = "pipeline";

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool                inited;

    pipeline_config_t   cfg;

    /* Sub-modules */
    shutter_state_t    *shutter;
    collapse_handle_t  *collapse;
    quiet_detector_t    quiet_det;
    fall_detector_t     fall_det;
    breathing_t        *breathing;          /* M4.3 — NULL if init failed */

    /* Per-frame scratch (PSRAM) */
    float              *h_freq_complex;     /* [2*N_sc]   = 424 B for HT20 */
    float              *h_clean_complex;    /* [2*N_sc] */
    float              *amp_per_sc;         /* [N_sc]     = 212 B for HT20 */

    /* Amplitude ring buffer (PSRAM) */
    int                 ring_capacity;
    uint64_t           *ring_ts_us;          /* [ring_capacity] */
    float              *ring_amp;            /* [ring_capacity * N_sc], row-major */
    int                 ring_head;           /* next write idx */
    int                 ring_count;          /* current entries */

    /* Per-window aggregation scratch (PSRAM) */
    float              *amp_matrix;          /* [max_window_frames * N_sc] */
    float              *mean_amp;            /* [N_sc] */
    int                 max_window_frames;   /* matches ring_capacity */

    /* Window emission state */
    uint64_t            last_emit_us;        /* 0 = never emitted */
    uint32_t            window_drop_count;   /* csi_drop equivalent */

    /* RSSI / noise rolling sums in current stride */
    int                 rssi_sum;
    int                 noise_sum;
    int                 health_count;

    /* Telemetry callback */
    pipeline_telemetry_cb_t cb;
    void                   *cb_ctx;
} pipeline_state_t;

static pipeline_state_t g_pipe;

/* -------------------------------------------------------------------------- */
/* Init / deinit                                                               */
/* -------------------------------------------------------------------------- */

static void *psram_calloc(size_t n, size_t sz)
{
    return heap_caps_calloc(n, sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

esp_err_t pipeline_init(const pipeline_config_t *cfg)
{
    if (g_pipe.inited) return ESP_ERR_INVALID_STATE;

    if (cfg) {
        g_pipe.cfg = *cfg;
    } else {
        const pipeline_config_t def = PIPELINE_CONFIG_DEFAULT();
        g_pipe.cfg = def;
    }
    const int N_sc = g_pipe.cfg.n_subcarriers;
    if (N_sc != PIPELINE_N_SC_HT20) {
        ESP_LOGE(TAG, "only HT20 (N_sc=%d) supported in M3.5", PIPELINE_N_SC_HT20);
        return ESP_ERR_INVALID_ARG;
    }
    if (g_pipe.cfg.ring_capacity < 100) g_pipe.cfg.ring_capacity = 100;

    /* ---- Runtime commissioning config (NVS, M4/PoC §3.1) ---- */
    const hf_config_t *hc = hf_config_get();

    /* ---- Shutter ---- */
    shutter_config_t sh_cfg = SHUTTER_CONFIG_DEFAULT();
    sh_cfg.n_subcarriers = N_sc;
    sh_cfg.room_size_m   = hc->room_size_m;
    sh_cfg.margin_m      = hc->margin_m;
    if (shutter_init(&g_pipe.shutter, &sh_cfg) != ESP_OK || !g_pipe.shutter) {
        ESP_LOGE(TAG, "shutter_init failed");
        goto err;
    }

    /* ---- Collapse ---- */
    collapse_config_t col_cfg = COLLAPSE_CONFIG_DEFAULT();
    col_cfg.collapse_threshold = hc->collapse_threshold;
    col_cfg.silence_threshold  = hc->silence_threshold;
    if (collapse_init(&g_pipe.collapse, &col_cfg) != ESP_OK || !g_pipe.collapse) {
        ESP_LOGE(TAG, "collapse_init failed");
        goto err;
    }

    /* ---- Quiet detector + Fall detector (transparent structs) ---- */
    quiet_detector_init(&g_pipe.quiet_det, NULL);
    fall_detector_config_t fd_cfg = FALL_DETECTOR_CONFIG_DEFAULT();
    fall_detector_init(&g_pipe.fall_det, &fd_cfg);

    /* ---- Breathing (M4.3) — soft-fail: optional, must not block pipeline ---- */
    breathing_config_t br_cfg = BREATHING_CONFIG_DEFAULT();
    br_cfg.n_subcarriers  = N_sc;
    br_cfg.min_confidence = hc->breathing_min_conf;
    if (breathing_init(&g_pipe.breathing, &br_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "breathing_init failed — breathing telemetry disabled");
        g_pipe.breathing = NULL;
    }

    /* ---- Per-frame scratch ---- */
    g_pipe.h_freq_complex  = psram_calloc(2 * N_sc, sizeof(float));
    g_pipe.h_clean_complex = psram_calloc(2 * N_sc, sizeof(float));
    g_pipe.amp_per_sc      = psram_calloc(N_sc, sizeof(float));

    /* ---- Ring buffer ---- */
    const int cap = g_pipe.cfg.ring_capacity;
    g_pipe.ring_capacity = cap;
    g_pipe.ring_ts_us = psram_calloc(cap, sizeof(uint64_t));
    g_pipe.ring_amp   = psram_calloc((size_t)cap * N_sc, sizeof(float));

    /* ---- Per-window scratch (size matches ring) ---- */
    g_pipe.max_window_frames = cap;
    g_pipe.amp_matrix = psram_calloc((size_t)cap * N_sc, sizeof(float));
    g_pipe.mean_amp   = psram_calloc(N_sc, sizeof(float));

    if (!g_pipe.h_freq_complex || !g_pipe.h_clean_complex ||
        !g_pipe.amp_per_sc || !g_pipe.ring_ts_us || !g_pipe.ring_amp ||
        !g_pipe.amp_matrix || !g_pipe.mean_amp) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        goto err;
    }

    g_pipe.ring_head = 0;
    g_pipe.ring_count = 0;
    g_pipe.last_emit_us = 0;
    g_pipe.window_drop_count = 0;
    g_pipe.rssi_sum = 0;
    g_pipe.noise_sum = 0;
    g_pipe.health_count = 0;
    g_pipe.cb = NULL;
    g_pipe.cb_ctx = NULL;

    g_pipe.inited = true;
    ESP_LOGI(TAG,
             "pipeline ready: N_sc=%d, window=%.1fs, stride=%dms, ring=%d entries (~%.1f KB)  shutter=%s",
             N_sc, (double)g_pipe.cfg.window_sec, g_pipe.cfg.window_stride_ms, cap,
             (double)((size_t)cap * N_sc * sizeof(float) +
                     cap * sizeof(uint64_t)) / 1024.0,
             g_pipe.cfg.bypass_shutter ? "BYPASSED (raw |H|)" : "active");
    return ESP_OK;

err:
    pipeline_deinit();
    return ESP_FAIL;
}

void pipeline_deinit(void)
{
    if (g_pipe.shutter) shutter_deinit(g_pipe.shutter);
    if (g_pipe.collapse) collapse_deinit(g_pipe.collapse);
    if (g_pipe.breathing) { breathing_deinit(g_pipe.breathing); g_pipe.breathing = NULL; }
    g_pipe.shutter = NULL;
    g_pipe.collapse = NULL;
    heap_caps_free(g_pipe.h_freq_complex);
    heap_caps_free(g_pipe.h_clean_complex);
    heap_caps_free(g_pipe.amp_per_sc);
    heap_caps_free(g_pipe.ring_ts_us);
    heap_caps_free(g_pipe.ring_amp);
    heap_caps_free(g_pipe.amp_matrix);
    heap_caps_free(g_pipe.mean_amp);
    memset(&g_pipe, 0, sizeof(g_pipe));
}

void pipeline_reset(void)
{
    if (!g_pipe.inited) return;
    g_pipe.ring_head = 0;
    g_pipe.ring_count = 0;
    g_pipe.last_emit_us = 0;
    g_pipe.rssi_sum = 0;
    g_pipe.noise_sum = 0;
    g_pipe.health_count = 0;
    quiet_detector_reset(&g_pipe.quiet_det);
    fall_detector_reset(&g_pipe.fall_det);
    collapse_reset(g_pipe.collapse);
    shutter_clear_baseline(g_pipe.shutter);
}

void pipeline_register_telemetry_cb(pipeline_telemetry_cb_t cb, void *ctx)
{
    g_pipe.cb     = cb;
    g_pipe.cb_ctx = ctx;
}

void pipeline_record_drop(void)
{
    g_pipe.window_drop_count++;
}

/* -------------------------------------------------------------------------- */
/* Ring helpers                                                                */
/* -------------------------------------------------------------------------- */

static inline void ring_push(uint64_t ts_us, const float *amp_row)
{
    const int N_sc = g_pipe.cfg.n_subcarriers;
    const int idx  = g_pipe.ring_head;
    g_pipe.ring_ts_us[idx] = ts_us;
    memcpy(&g_pipe.ring_amp[(size_t)idx * N_sc], amp_row,
           (size_t)N_sc * sizeof(float));
    g_pipe.ring_head = (idx + 1) % g_pipe.ring_capacity;
    if (g_pipe.ring_count < g_pipe.ring_capacity) g_pipe.ring_count++;
}

/* Build a contiguous (n_frames, N_sc) amp_matrix for entries with ts in
 * [now_us - window_us, now_us]. Returns n_frames written. */
static int ring_collect_window(uint64_t now_us, float *out_matrix)
{
    const int N_sc = g_pipe.cfg.n_subcarriers;
    const uint64_t window_us = (uint64_t)(g_pipe.cfg.window_sec * 1.0e6f);
    const int cap = g_pipe.ring_capacity;
    int n = 0;

    for (int i = 0; i < g_pipe.ring_count; i++) {
        const int idx = (g_pipe.ring_head - g_pipe.ring_count + i + cap) % cap;
        const uint64_t ts = g_pipe.ring_ts_us[idx];
        if (now_us > ts && (now_us - ts) > window_us) continue;
        memcpy(&out_matrix[(size_t)n * N_sc],
               &g_pipe.ring_amp[(size_t)idx * N_sc],
               (size_t)N_sc * sizeof(float));
        n++;
    }
    return n;
}

/* -------------------------------------------------------------------------- */
/* Window emission                                                             */
/* -------------------------------------------------------------------------- */

static void emit_window(uint64_t now_us)
{
    const int N_sc = g_pipe.cfg.n_subcarriers;

    const int n_frames = ring_collect_window(now_us, g_pipe.amp_matrix);
    if (n_frames < 2) {
        /* Need at least 2 frames for shape_corr; skip silently. */
        g_pipe.last_emit_us = now_us;
        return;
    }

    /* ---- M3.3 metrics ---- */
    metrics_result_t mres;
    if (metrics_compute(g_pipe.amp_matrix, n_frames, N_sc, &mres) != ESP_OK) {
        return;
    }
    const float G = metrics_dynamic_gain_G(g_pipe.amp_matrix, n_frames, N_sc);

    /* ---- Quiet detector ---- */
    bool is_quiet = false;
    float quiet_dur = 0.0f;
    quiet_detector_update(&g_pipe.quiet_det, mres.norm_cv, now_us,
                          &is_quiet, &quiet_dur);

    /* ---- Mean amp per SC over window ---- */
    for (int k = 0; k < N_sc; k++) g_pipe.mean_amp[k] = 0.0f;
    for (int i = 0; i < n_frames; i++) {
        const float *row = &g_pipe.amp_matrix[(size_t)i * N_sc];
        for (int k = 0; k < N_sc; k++) g_pipe.mean_amp[k] += row[k];
    }
    const float inv_n = 1.0f / (float)n_frames;
    for (int k = 0; k < N_sc; k++) g_pipe.mean_amp[k] *= inv_n;

    /* ---- Poincaré 53→8 + embed (separate from collapse to expose raw embed) ---- */
    float amp_select[PIPELINE_EMBED_DIM];
    poincare_select_8d(g_pipe.mean_amp, amp_select);
    float embed_now[PIPELINE_EMBED_DIM];
    poincare_embed(amp_select, PIPELINE_EMBED_DIM, 0.5f, embed_now);

    /* ---- Collapse FSM (M3.2) ---- */
    collapse_result_t cres;
    collapse_update(g_pipe.collapse, g_pipe.mean_amp, now_us, &cres);

    /* ---- Fall detector (M3.4) ---- */
    fall_detector_event_t fev;
    fall_detector_update(&g_pipe.fall_det, now_us, cres.collapse_index,
                         cres.fall_detected, &fev);

    /* ---- CSI fps over the stride ---- */
    float fps = 0.0f;
    if (g_pipe.last_emit_us > 0 && now_us > g_pipe.last_emit_us) {
        const float dt_sec = (float)(now_us - g_pipe.last_emit_us) / 1.0e6f;
        fps = (float)g_pipe.health_count / dt_sec;
    }

    /* ---- RSSI / noise averages over the stride ---- */
    int rssi_avg = 0, noise_avg = 0;
    if (g_pipe.health_count > 0) {
        rssi_avg  = g_pipe.rssi_sum  / g_pipe.health_count;
        noise_avg = g_pipe.noise_sum / g_pipe.health_count;
    }

    /* ---- Build telemetry snapshot ---- */
    pipeline_telemetry_t t = {0};
    t.timestamp_us         = now_us;
    t.n_frames_in_window   = n_frames;
    t.csi_fps              = fps;
    t.norm_cv              = mres.norm_cv;
    t.raw_cv               = mres.raw_cv;
    t.shape_corr           = mres.shape_corr;
    t.dynamic_gain_G       = G;
    t.quiet_period         = is_quiet;
    t.n_valid_subcarriers  = mres.n_valid_subcarriers;

    memcpy(t.poincare_embed, embed_now, sizeof(t.poincare_embed));

    t.collapse_index       = cres.collapse_index;
    t.fsm_state            = (cres.state == COLLAPSE_FSM_SPIKE_DETECTED)
                              ? PIPELINE_FSM_SPIKE_DETECTED
                              : PIPELINE_FSM_MONITORING;
    t.embedding_norm       = cres.embedding_norm;

    t.fall_detected        = cres.fall_detected;
    t.fall_event_rising_edge = fev.triggered;
    t.fall_confidence        = fev.confidence;
    t.fall_best_pattern_idx  = fev.best_pattern_idx;
    t.fall_best_pattern_name = (fev.best_pattern_name && fev.best_pattern_name[0])
                                ? fev.best_pattern_name : "";

    t.rssi_avg          = rssi_avg;
    t.noise_floor_avg   = noise_avg;
    t.csi_drop_count    = g_pipe.window_drop_count;

    /* M4.3: breathing snapshot (estimator updates on its own 5 s cadence) */
    if (g_pipe.breathing) {
        breathing_result_t br;
        breathing_get(g_pipe.breathing, &br);
        t.breathing_bpm        = br.bpm;
        t.breathing_confidence = br.confidence;
        t.breathing_state      = (uint32_t)br.state;
    }

    /* M3.6.4: push derived per-window snapshot into the Tier B buffer.
     * No-op if event_buffer isn't inited. */
    {
        ev_buf_window_t evw;
        memset(&evw, 0, sizeof(evw));
        evw.ts_us           = t.timestamp_us;
        evw.collapse_index  = t.collapse_index;
        evw.norm_cv         = t.norm_cv;
        evw.raw_cv          = t.raw_cv;
        evw.shape_corr      = t.shape_corr;
        evw.dynamic_gain_G  = t.dynamic_gain_G;
        evw.embedding_norm  = t.embedding_norm;
        for (int i = 0; i < EV_BUF_EMBED_DIM; i++)
            evw.poincare_embed[i] = t.poincare_embed[i];
        evw.rssi_avg        = t.rssi_avg;
        evw.fsm_state       = (uint16_t)t.fsm_state;
        evw.flags           = (uint16_t)((t.quiet_period   ? 0x1 : 0) |
                                         (t.fall_detected ? 0x2 : 0));
        event_buffer_push_window(&evw);
    }

    if (g_pipe.cb) g_pipe.cb(&t, g_pipe.cb_ctx);

    /* Reset stride counters */
    g_pipe.last_emit_us = now_us;
    g_pipe.rssi_sum     = 0;
    g_pipe.noise_sum    = 0;
    g_pipe.health_count = 0;
}

/* -------------------------------------------------------------------------- */
/* Per-frame entry                                                             */
/* -------------------------------------------------------------------------- */

void pipeline_on_csi_frame(
    uint64_t      ts_us,
    int8_t        rssi,
    int8_t        noise_floor,
    const int8_t *iq_data,
    int           n_sc)
{
    if (!g_pipe.inited || !iq_data || n_sc != g_pipe.cfg.n_subcarriers) {
        pipeline_record_drop();
        return;
    }

    /* Wire layout: int8 [Im0, Re0, Im1, Re1, ...]
     * Convert to interleaved float [Re0, Im0, Re1, Im1, ...] for shutter/ESP-DSP. */
    for (int k = 0; k < n_sc; k++) {
        const int8_t im = iq_data[2 * k];
        const int8_t re = iq_data[2 * k + 1];
        g_pipe.h_freq_complex[2 * k]     = (float)re;
        g_pipe.h_freq_complex[2 * k + 1] = (float)im;
    }

    const float *h_for_amp = g_pipe.h_freq_complex;
    if (!g_pipe.cfg.bypass_shutter) {
        /* M3.1 Spatial Shutter — HT20 (312.5 kHz spacing, 53 SC, 8-bit IQ wire) */
        if (shutter_process(g_pipe.shutter, g_pipe.h_freq_complex,
                             SHUTTER_FRAME_HT20,
                             g_pipe.h_clean_complex, NULL) != ESP_OK) {
            pipeline_record_drop();
            return;
        }
        h_for_amp = g_pipe.h_clean_complex;
    }
    /* Amplitude per SC (from raw H if shutter bypassed, else from H_clean) */
    for (int k = 0; k < n_sc; k++) {
        const float re = h_for_amp[2 * k];
        const float im = h_for_amp[2 * k + 1];
        g_pipe.amp_per_sc[k] = sqrtf(re * re + im * im);
    }

    ring_push(ts_us, g_pipe.amp_per_sc);

    /* M4.3: feed amplitude row to breathing estimator (decimates internally to
     * 10 Hz; heavy PSD runs on the 5 s cadence, ~0.3% CPU per frame). */
    if (g_pipe.breathing)
        breathing_on_frame(g_pipe.breathing, g_pipe.amp_per_sc, NULL, ts_us);

    /* M3.6.4: also push raw CSI into the Tier B rolling buffer.
     * No-op if event_buffer isn't inited. ~3 µs per frame. */
    {
        ev_buf_frame_t evf;
        memset(&evf, 0, sizeof(evf));
        evf.ts_us       = ts_us;
        evf.rssi        = rssi;
        evf.noise_floor = noise_floor;
        memcpy(evf.iq, iq_data, EV_BUF_IQ_BYTES);
        event_buffer_push_frame(&evf);
    }

    g_pipe.rssi_sum  += rssi;
    g_pipe.noise_sum += noise_floor;
    g_pipe.health_count++;

    /* Window emission boundary */
    const uint64_t stride_us = (uint64_t)g_pipe.cfg.window_stride_ms * 1000ULL;
    if (g_pipe.last_emit_us == 0) {
        /* First emission: wait until we have at least window_sec of data */
        const uint64_t window_us = (uint64_t)(g_pipe.cfg.window_sec * 1.0e6f);
        if (g_pipe.ring_count >= 2 &&
            (ts_us - g_pipe.ring_ts_us[
                (g_pipe.ring_head - g_pipe.ring_count +
                 g_pipe.ring_capacity) % g_pipe.ring_capacity]) >= window_us) {
            emit_window(ts_us);
        }
    } else if ((ts_us - g_pipe.last_emit_us) >= stride_us) {
        emit_window(ts_us);
    }
}
