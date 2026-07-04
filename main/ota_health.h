/* ota_health.h — M4-OTA post-update verification gate.
 *
 * With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, the first boot after an
 * esp_https_ota runs in ESP_OTA_IMG_PENDING_VERIFY: unless the app calls
 * esp_ota_mark_app_valid_cancel_rollback(), any reset returns to the
 * previous slot. This module decides when (and whether) to mark valid.
 *
 * Two-tier verdict:
 *   - fully_ok    → mark valid immediately (self-tests + uplink + CSI).
 *   - uplink_ok   → at timeout, mark valid anyway: MQTT reachable means a
 *                   broken detection path can still be fixed by another OTA,
 *                   while CSI absence may just be the facility AP being down.
 *   - neither     → mark invalid + reboot (bootloader rolls back).
 *
 * No-op when the running image is not PENDING_VERIFY (USB-flashed boots).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool (*fully_ok)(void *ctx);   /* full health — mark valid on first true */
    bool (*uplink_ok)(void *ctx);  /* minimum to keep image at timeout */
    void *ctx;
    uint32_t timeout_s;            /* poll deadline (0 → default 300 s) */
} ota_health_config_t;

/* Logs the running slot + OTA state, and if PENDING_VERIFY spawns the
 * verification task. Call once, early in app_main (after NVS init). */
esp_err_t ota_health_start(const ota_health_config_t *cfg);

#ifdef __cplusplus
}
#endif
