/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.6.0 self-test (event_buffer rolling buffer) — ADR-023.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Synthetic-frame regression for event_buffer:
 *
 *   1. push N frames + M windows on a synthetic timeline (1 ms apart for
 *      frames, 1 s apart for windows).
 *   2. extract_sync at a known event_ts ± pre/post.
 *   3. verify n_frames / n_windows / ts ranges / content fidelity.
 *
 * Returns ESP_OK if all subtests pass, else ESP_FAIL.
 */
esp_err_t m36_run_self_test(void);

#ifdef __cplusplus
}
#endif
