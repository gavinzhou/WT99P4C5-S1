/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Breathing rate self-test (M4.3) — see m_breathing_test.c.
 */
#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Run breathing Welch-estimator fixtures at boot. ESP_OK if all pass. */
esp_err_t breathing_run_self_test(void);

#ifdef __cplusplus
}
#endif
