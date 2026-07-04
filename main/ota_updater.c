/* ota_updater.c — M4-OTA firmware download + verify + apply. */

#include "ota_updater.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_crt_bundle.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"

static const char *TAG = "ota_updater";

#define OTA_URL_MAX      1024   /* presigned S3 URLs carry long query strings */
#define OTA_READBACK_BUF 4096

static struct {
    char url[OTA_URL_MAX];
    uint8_t sha_expected[32];
    ota_updater_done_cb cb;
    void *cb_ctx;
    volatile bool in_progress;
} s;

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((int)c);
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    return -1;
}

static esp_err_t parse_sha256_hex(const char *hex, uint8_t out[32])
{
    if (!hex || strlen(hex) != 64) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < 32; i++) {
        int hi = hex_nibble(hex[2 * i]), lo = hex_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return ESP_ERR_INVALID_ARG;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return ESP_OK;
}

/* sha256 of the first `len` bytes of the partition the OTA wrote into. */
static esp_err_t partition_sha256(const esp_partition_t *part, size_t len,
                                  uint8_t out[32])
{
    uint8_t *buf = malloc(OTA_READBACK_BUF);
    if (!buf) return ESP_ERR_NO_MEM;

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0 /* SHA-256 */);

    esp_err_t err = ESP_OK;
    for (size_t off = 0; off < len; off += OTA_READBACK_BUF) {
        size_t n = len - off < OTA_READBACK_BUF ? len - off : OTA_READBACK_BUF;
        err = esp_partition_read(part, off, buf, n);
        if (err != ESP_OK) break;
        mbedtls_sha256_update(&ctx, buf, n);
    }
    if (err == ESP_OK) mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    free(buf);
    return err;
}

static void finish_fail(esp_https_ota_handle_t h, esp_err_t err)
{
    if (h) esp_https_ota_abort(h);
    ESP_LOGE(TAG, "update FAILED: 0x%x", err);
    if (s.cb) s.cb(err, s.cb_ctx);
    s.in_progress = false;
    vTaskDelete(NULL);
}

static void ota_updater_task(void *arg)
{
    ESP_LOGI(TAG, "starting download");

    esp_http_client_config_t http_cfg = {
        .url = s.url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,   /* presigned query string exceeds default */
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_https_ota_handle_t h = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &h);
    if (err != ESP_OK) finish_fail(NULL, err);

    int total = esp_https_ota_get_image_size(h);
    int last_logged = 0;
    while ((err = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int got = esp_https_ota_get_image_len_read(h);
        if (got - last_logged >= 256 * 1024) {
            last_logged = got;
            ESP_LOGI(TAG, "downloaded %d / %d bytes", got, total);
        }
    }
    if (err != ESP_OK) finish_fail(h, err);
    if (!esp_https_ota_is_complete_data_received(h)) {
        ESP_LOGE(TAG, "incomplete download");
        finish_fail(h, ESP_ERR_INVALID_SIZE);
    }

    /* Whole-image sha256: read the slot back and compare to the digest the
     * job/console supplied. IDF has already validated the app header; this
     * catches wrong-file-uploaded and truncated/corrupted objects. */
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    int len = esp_https_ota_get_image_len_read(h);
    uint8_t sha_actual[32];
    err = partition_sha256(part, (size_t)len, sha_actual);
    if (err != ESP_OK) finish_fail(h, err);
    if (memcmp(sha_actual, s.sha_expected, 32) != 0) {
        ESP_LOGE(TAG, "sha256 MISMATCH — refusing image (%d bytes in %s)",
                 len, part->label);
        finish_fail(h, ESP_ERR_INVALID_CRC);
    }
    ESP_LOGI(TAG, "sha256 verified (%d bytes in %s)", len, part->label);

    err = esp_https_ota_finish(h);   /* validates + sets boot partition */
    if (err != ESP_OK) finish_fail(NULL, err);

    ESP_LOGW(TAG, "update applied — rebooting in 3 s (new image boots "
                  "PENDING_VERIFY, ota_health gate decides)");
    if (s.cb) s.cb(ESP_OK, s.cb_ctx);
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

bool ota_updater_in_progress(void)
{
    return s.in_progress;
}

esp_err_t ota_updater_start(const char *url, const char *sha256_hex,
                            ota_updater_done_cb cb, void *ctx)
{
    if (!url || strlen(url) >= OTA_URL_MAX) return ESP_ERR_INVALID_ARG;
    if (s.in_progress) return ESP_ERR_INVALID_STATE;

    esp_err_t err = parse_sha256_hex(sha256_hex, s.sha_expected);
    if (err != ESP_OK) return err;

    strcpy(s.url, url);
    s.cb = cb;
    s.cb_ctx = ctx;
    s.in_progress = true;

    if (xTaskCreate(ota_updater_task, "ota_updater", 8192, NULL, 5, NULL) != pdPASS) {
        s.in_progress = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
