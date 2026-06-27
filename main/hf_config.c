/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * hf_config implementation — single NVS blob (namespace "hfcfg", key "cfg").
 */
#include "hf_config.h"

#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "hf_config";
#define NS  "hfcfg"
#define KEY "cfg"

static hf_config_t s_cfg = HF_CONFIG_DEFAULT();
static bool s_loaded = false;

esp_err_t hf_config_load(void)
{
    s_cfg = HF_CONFIG_DEFAULT();
    s_loaded = true;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no saved config (%s) — using defaults", esp_err_to_name(err));
        hf_config_print();
        return ESP_OK;
    }
    hf_config_t tmp;
    size_t len = sizeof(tmp);
    err = nvs_get_blob(h, KEY, &tmp, &len);
    nvs_close(h);
    if (err == ESP_OK && len == sizeof(tmp)) {
        s_cfg = tmp;
        ESP_LOGI(TAG, "loaded config from NVS");
    } else {
        ESP_LOGW(TAG, "saved config absent/stale (%s, len=%u) — defaults",
                 esp_err_to_name(err), (unsigned)len);
    }
    hf_config_print();
    return ESP_OK;
}

esp_err_t hf_config_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err)); return err; }
    err = nvs_set_blob(h, KEY, &s_cfg, sizeof(s_cfg));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) ESP_LOGI(TAG, "config saved to NVS");
    else               ESP_LOGE(TAG, "config save failed: %s", esp_err_to_name(err));
    return err;
}

const hf_config_t *hf_config_get(void)
{
    if (!s_loaded) { s_cfg = HF_CONFIG_DEFAULT(); s_loaded = true; }
    return &s_cfg;
}

esp_err_t hf_config_set(const char *key, float v)
{
    if (!key) return ESP_ERR_INVALID_ARG;
    if      (!strcmp(key, "room"))          s_cfg.room_size_m = v;
    else if (!strcmp(key, "margin"))        s_cfg.margin_m = v;
    else if (!strcmp(key, "collapse_thr"))  s_cfg.collapse_threshold = v;
    else if (!strcmp(key, "silence_thr"))   s_cfg.silence_threshold = v;
    else if (!strcmp(key, "breathing_min")) s_cfg.breathing_min_conf = v;
    else { ESP_LOGW(TAG, "unknown key '%s'", key); return ESP_ERR_INVALID_ARG; }
    ESP_LOGI(TAG, "set %s = %.4f (not yet saved)", key, v);
    return ESP_OK;
}

void hf_config_print(void)
{
    ESP_LOGI(TAG, "config: room=%.1fm margin=%.1fm collapse_thr=%.3f "
             "silence_thr=%.3f breathing_min=%.2f",
             s_cfg.room_size_m, s_cfg.margin_m, s_cfg.collapse_threshold,
             s_cfg.silence_threshold, s_cfg.breathing_min_conf);
}
