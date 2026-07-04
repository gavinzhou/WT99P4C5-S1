/* ota_health.c — M4-OTA post-update verification gate. */

#include "ota_health.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_health";

#define OTA_HEALTH_DEFAULT_TIMEOUT_S 300
#define OTA_HEALTH_POLL_MS           5000

static ota_health_config_t s_cfg;

static void ota_health_task(void *arg)
{
    const uint32_t timeout_s = s_cfg.timeout_s ? s_cfg.timeout_s
                                               : OTA_HEALTH_DEFAULT_TIMEOUT_S;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_s * 1000);

    while (xTaskGetTickCount() < deadline) {
        if (s_cfg.fully_ok && s_cfg.fully_ok(s_cfg.ctx)) {
            ESP_LOGI(TAG, "health gate PASS — marking app valid (rollback cancelled)");
            esp_ota_mark_app_valid_cancel_rollback();
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(OTA_HEALTH_POLL_MS));
    }

    if (s_cfg.uplink_ok && s_cfg.uplink_ok(s_cfg.ctx)) {
        /* Uplink alive: keep the image even though full health never passed —
         * we can still push a fixed build over the air. Do NOT silently
         * pretend all is well: leave a loud marker for the soak log. */
        ESP_LOGW(TAG, "health gate TIMEOUT (%lus) but MQTT uplink OK — "
                      "marking valid; detection path needs investigation",
                 (unsigned long)timeout_s);
        esp_ota_mark_app_valid_cancel_rollback();
    } else {
        ESP_LOGE(TAG, "health gate FAILED (%lus, no uplink) — rolling back to previous slot",
                 (unsigned long)timeout_s);
        /* Marks the running app invalid and reboots; bootloader boots the
         * previous slot. Never returns. */
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    vTaskDelete(NULL);
}

esp_err_t ota_health_start(const ota_health_config_t *cfg)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_err_t err = esp_ota_get_state_partition(running, &state);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_ota_get_state_partition failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "running partition=%s state=%d", running->label, (int)state);

    if (state != ESP_OTA_IMG_PENDING_VERIFY) {
        return ESP_OK;  /* normal boot (USB-flashed or already verified) */
    }

    if (!cfg || !cfg->fully_ok || !cfg->uplink_ok) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    ESP_LOGW(TAG, "first boot after OTA (PENDING_VERIFY) — health gate armed, "
                  "timeout %lus",
             (unsigned long)(s_cfg.timeout_s ? s_cfg.timeout_s
                                             : OTA_HEALTH_DEFAULT_TIMEOUT_S));
    if (xTaskCreate(ota_health_task, "ota_health", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed — marking valid to avoid false rollback");
        esp_ota_mark_app_valid_cancel_rollback();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
