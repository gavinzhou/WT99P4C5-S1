/* ota_updater.h — M4-OTA firmware download + verify + apply.
 *
 * Downloads an app image over HTTPS (presigned S3 GET or any TLS URL,
 * cert bundle), streams it into the inactive OTA slot, verifies the
 * whole image sha256 against the expected digest by reading the slot
 * back, then sets the boot partition and reboots. With
 * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE the new image boots as
 * PENDING_VERIFY and must pass the ota_health gate.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Called on failure, or on success just before the reboot. */
typedef void (*ota_updater_done_cb)(esp_err_t result, void *ctx);

/* Kick off the download task. sha256_hex: 64 lowercase/uppercase hex chars.
 * Returns ESP_ERR_INVALID_STATE if an update is already running. */
esp_err_t ota_updater_start(const char *url, const char *sha256_hex,
                            ota_updater_done_cb cb, void *ctx);

bool ota_updater_in_progress(void);

#ifdef __cplusplus
}
#endif
