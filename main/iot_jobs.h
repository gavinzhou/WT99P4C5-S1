/* iot_jobs.h — M4-OTA AWS IoT Jobs client (firmware update channel).
 *
 * Piggybacks on the mqtt_publisher client (same pattern as event_uploader).
 * Only meaningful when the broker is AWS IoT Core (mTLS material in NVS);
 * against local mosquitto the $aws/... subscriptions are inert.
 *
 * Job document (created by tools/aws/create_ota_job.sh):
 *   { "operation": "fw_update", "version": "<esp_app_desc version>",
 *     "url": "<presigned S3 GET>", "sha256": "<64 hex>" }
 *
 * Flow: notify-next / start-next → version differs → persist {jobId,version}
 * in NVS → report IN_PROGRESS → ota_updater. After the reboot the health
 * gate (ota_health) decides; once the image state leaves PENDING_VERIFY we
 * reconcile: running version == target → SUCCEEDED, else (bootloader rolled
 * back) → FAILED.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once after mqtt_publisher_init() + event_uploader_init(). */
esp_err_t iot_jobs_init(void);

#ifdef __cplusplus
}
#endif
