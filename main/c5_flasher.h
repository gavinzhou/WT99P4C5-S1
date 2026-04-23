/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * C5 Flasher via UART from P4 (ADR-020 方案 A PoC)
 *
 * Physical wiring (WT99P4C5-S1 v1.1, J6 ↔ J5 jumper wires):
 *   P4 GPIO32 (UART1 TX)  →  J5 RXD (C5 U0RXD, IO26)
 *   P4 GPIO33 (UART1 RX)  ←  J5 TXD (C5 U0TXD, IO27)
 *   P4 GPIO26 (BOOT ctrl) →  J5 BOOT (C5 IO28 strapping)
 *   P4 GPIO54 (C5 EN)     already connected via PCB
 *   GND                   shared via PCB (or jumper for safety)
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Smoke test: try to connect to C5 bootloader via UART and report chip info.
 * Does NOT flash anything. Use this to verify wiring before attempting full flash.
 *
 * @return ESP_OK if C5 bootloader responded, ESP_FAIL otherwise.
 */
esp_err_t c5_smoke_test(void);

/**
 * Full flash: put C5 in bootloader, write the embedded network_adapter binary,
 * then reset C5 to normal boot.
 *
 * @return ESP_OK on success.
 */
esp_err_t c5_full_flash(void);

/**
 * Reset C5 into normal boot (BOOT=high, EN pulse). Call this when skipping flash,
 * so the already-flashed network_adapter starts up correctly instead of leaving
 * the BOOT strapping pin floating (would cause C5 to enter download mode and
 * SDIO handshake to fail with ESP_ERR_TIMEOUT 0x107).
 */
void c5_release_normal_boot_public(void);

#ifdef __cplusplus
}
#endif
