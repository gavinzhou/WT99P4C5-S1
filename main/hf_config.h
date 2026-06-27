/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * hf_config — runtime-configurable detection parameters, persisted in NVS.
 *
 * MFR PoC (M4 / ROADMAP Phase 3) §3.1: the on-site engineer commissions each
 * room without recompiling. Parameters live in NVS (namespace "hfcfg") and are
 * loaded at boot; pipeline_init() applies them to collapse / shutter / breathing.
 * Falls back to compile-time defaults when NVS is empty or stale.
 *
 * v1 set: room geometry (used once the shutter is un-bypassed) + collapse fall
 * thresholds (per-room sensitivity) + breathing confidence floor.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float room_size_m;          /* monitoring radius [m] — 半径8m 可調 (spec §KPI) */
    float margin_m;             /* wall reflection margin [m] */
    float collapse_threshold;   /* C above this -> SPIKE_DETECTED (fall sensitivity) */
    float silence_threshold;    /* C below this counts toward silence */
    float breathing_min_conf;   /* breathing confidence floor (raised from 1.5: pure
                                 * noise scores ~1.7 on a single Welch window, M4.3) */
} hf_config_t;

#define HF_CONFIG_DEFAULT() (hf_config_t){ \
    .room_size_m        = 8.0f,  \
    .margin_m           = 2.0f,  \
    .collapse_threshold = 0.12f, \
    .silence_threshold  = 0.04f, \
    .breathing_min_conf = 3.0f,  \
}

/** Load from NVS into the singleton (falls back to defaults). Call once at boot
 *  before pipeline_init(). Safe if NVS is uninitialised — uses defaults. */
esp_err_t hf_config_load(void);

/** Persist the current singleton to NVS. */
esp_err_t hf_config_save(void);

/** Read-only access to the current config. Never NULL after hf_config_load(). */
const hf_config_t *hf_config_get(void);

/** Set one field by name (for the commissioning console). Keys:
 *  "room", "margin", "collapse_thr", "silence_thr", "breathing_min".
 *  Does NOT persist — call hf_config_save() after. Returns ESP_ERR_INVALID_ARG
 *  for an unknown key. */
esp_err_t hf_config_set(const char *key, float value);

/** Log the current config (commissioning aid). */
void hf_config_print(void);

#ifdef __cplusplus
}
#endif
