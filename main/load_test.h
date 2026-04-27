/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * load_test — pure airtime stress test for the C5 single-radio uplink
 * path (ADR-024 evaluation).
 *
 * Sends UDP datagrams from P4 → C5 STA radio → SoftAP, where SoftAP
 * has no listener on the dest port and silently drops them. This puts
 * pressure on the C5 radio's airtime without requiring any protocol
 * stack on the receiving side.
 *
 * The goal is to measure CSI fps degradation under varying upload
 * load to validate whether C5 single-radio is viable for "facility
 * router serves both CSI source AND uplink" production architecture.
 *
 * Cross-reference: existing csi_stats_task in main.cpp logs CSI fps
 * every ~5 s. Eyeball the monitor: load_test logs the actual upload
 * rate, csi_stats logs the fps achieved during that step.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char  *target_ip;          /* default "192.168.4.1" (TX SoftAP) */
    uint16_t     target_port;        /* default 9999 (no listener) */
    int          packet_size;        /* bytes per UDP datagram, ≤ 1400 */
    int          step_duration_sec;  /* per-step run time */
    const int   *target_kbps_steps;  /* array of target rates, kbps */
    int          n_steps;
    int          warmup_sec;         /* sleep before first step */
} load_test_config_t;

/* Default ramp:
 *   step 0: 0 kbps    — baseline / idle (existing csi_trigger only)
 *   step 1: 10        — ~MQTT telemetry × 10
 *   step 2: 100       — small batch upload
 *   step 3: 500       — half the M3.6 Tier B context burst rate
 *   step 4: 1000      — 1 Mbps stress (Tier B 60-s context spread over ~12 s)
 *   step 5: 0         — recovery (verify CSI fps returns to baseline)
 */
extern const int load_test_default_steps[6];

#define LOAD_TEST_CONFIG_DEFAULT() {                       \
    .target_ip          = "192.168.4.1",                  \
    .target_port        = 9999,                            \
    .packet_size        = 1400,                            \
    .step_duration_sec  = 30,                              \
    .target_kbps_steps  = load_test_default_steps,         \
    .n_steps            = 6,                               \
    .warmup_sec         = 20,                              \
}

/**
 * Spawn the load-test task. The task:
 *   1. Sleeps `warmup_sec` so CSI ingest reaches steady state first.
 *   2. Walks the kbps_steps[] one by one, sending UDP datagrams at
 *      the target rate for `step_duration_sec` each.
 *   3. Logs actual achieved kbps + send/fail counts per step.
 *   4. Self-deletes when done (no need to call a stop function).
 *
 * Returns ESP_OK if task spawned, ESP_ERR_NO_MEM if alloc failed,
 * ESP_ERR_INVALID_ARG on bad config.
 */
esp_err_t load_test_start(const load_test_config_t *cfg);

#ifdef __cplusplus
}
#endif
