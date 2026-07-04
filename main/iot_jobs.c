/* iot_jobs.c — M4-OTA AWS IoT Jobs client. */

#include "iot_jobs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt_publisher.h"
#include "ota_updater.h"

static const char *TAG = "iot_jobs";

#define JOBS_NVS_NAMESPACE "iotjobs"
#define JOBS_NVS_KEY_JID   "jid"
#define JOBS_NVS_KEY_VER   "ver"
#define JOB_ID_MAX  80
#define VERSION_MAX 32

static struct {
    esp_mqtt_client_handle_t mqtt;
    const char *device_id;
    char topic_notify[128];       /* $aws/things/{id}/jobs/notify-next */
    char topic_start_acc[128];    /* $aws/things/{id}/jobs/start-next/accepted */
    bool inited;
    bool reconcile_spawned;
    char active_jid[JOB_ID_MAX];  /* job being downloaded right now */
} s;

/* ---- helpers ------------------------------------------------------------ */

static void report_status(const char *jid, const char *status, const char *detail)
{
    char topic[160], payload[256];
    snprintf(topic, sizeof(topic), "$aws/things/%s/jobs/%s/update",
             s.device_id, jid);
    if (detail && detail[0]) {
        snprintf(payload, sizeof(payload),
                 "{\"status\":\"%s\",\"statusDetails\":{\"detail\":\"%s\"}}",
                 status, detail);
    } else {
        snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}", status);
    }
    int id = esp_mqtt_client_publish(s.mqtt, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "job %s -> %s (msg_id=%d)", jid, status, id);
}

static void job_nvs_save(const char *jid, const char *ver)
{
    nvs_handle_t nh;
    if (nvs_open(JOBS_NVS_NAMESPACE, NVS_READWRITE, &nh) != ESP_OK) return;
    nvs_set_str(nh, JOBS_NVS_KEY_JID, jid);
    nvs_set_str(nh, JOBS_NVS_KEY_VER, ver);
    nvs_commit(nh);
    nvs_close(nh);
}

static bool job_nvs_load(char *jid, size_t jid_len, char *ver, size_t ver_len)
{
    nvs_handle_t nh;
    if (nvs_open(JOBS_NVS_NAMESPACE, NVS_READONLY, &nh) != ESP_OK) return false;
    esp_err_t e1 = nvs_get_str(nh, JOBS_NVS_KEY_JID, jid, &jid_len);
    esp_err_t e2 = nvs_get_str(nh, JOBS_NVS_KEY_VER, ver, &ver_len);
    nvs_close(nh);
    return e1 == ESP_OK && e2 == ESP_OK;
}

static void job_nvs_clear(void)
{
    nvs_handle_t nh;
    if (nvs_open(JOBS_NVS_NAMESPACE, NVS_READWRITE, &nh) != ESP_OK) return;
    nvs_erase_all(nh);
    nvs_commit(nh);
    nvs_close(nh);
}

/* ---- post-reboot reconciliation ----------------------------------------- */

/* Waits for the health gate's verdict (image state leaves PENDING_VERIFY),
 * then reports the persisted job as SUCCEEDED or FAILED. */
static void reconcile_task(void *arg)
{
    char jid[JOB_ID_MAX], ver[VERSION_MAX];
    if (!job_nvs_load(jid, sizeof(jid), ver, sizeof(ver))) {
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "pending job %s (target ver %s) — awaiting health verdict",
             jid, ver);

    const esp_partition_t *running = esp_ota_get_running_partition();
    for (int i = 0; i < 120; i++) {   /* ≤ 10 min */
        esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
        esp_ota_get_state_partition(running, &st);
        if (st != ESP_OTA_IMG_PENDING_VERIFY) break;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    const char *cur = esp_app_get_description()->version;
    if (strncmp(cur, ver, VERSION_MAX) == 0) {
        report_status(jid, "SUCCEEDED", NULL);
    } else {
        /* Running version isn't the target — bootloader rolled us back. */
        report_status(jid, "FAILED", "rolled back by health gate");
    }
    job_nvs_clear();
    vTaskDelete(NULL);
}

/* ---- job execution ------------------------------------------------------ */

static void ota_fail_cb(esp_err_t result, void *ctx)
{
    if (result == ESP_OK) return;   /* success path reboots; reconcile reports */
    if (s.active_jid[0]) {
        report_status(s.active_jid, "FAILED", esp_err_to_name(result));
        s.active_jid[0] = 0;
        job_nvs_clear();
    }
}

static void handle_execution(cJSON *exec)
{
    cJSON *jid_j = cJSON_GetObjectItem(exec, "jobId");
    cJSON *doc   = cJSON_GetObjectItem(exec, "jobDocument");
    if (!cJSON_IsString(jid_j) || !cJSON_IsObject(doc)) return;

    cJSON *url_j = cJSON_GetObjectItem(doc, "url");
    cJSON *sha_j = cJSON_GetObjectItem(doc, "sha256");
    cJSON *ver_j = cJSON_GetObjectItem(doc, "version");
    if (!cJSON_IsString(url_j) || !cJSON_IsString(sha_j) || !cJSON_IsString(ver_j)) {
        ESP_LOGW(TAG, "job %s: document missing url/sha256/version — rejecting",
                 jid_j->valuestring);
        report_status(jid_j->valuestring, "FAILED", "bad job document");
        return;
    }

    const char *cur = esp_app_get_description()->version;
    if (strcmp(cur, ver_j->valuestring) == 0) {
        ESP_LOGI(TAG, "job %s: already on version %s", jid_j->valuestring, cur);
        report_status(jid_j->valuestring, "SUCCEEDED", "already on target version");
        return;
    }
    if (ota_updater_in_progress()) {
        ESP_LOGW(TAG, "job %s: update already in progress — ignoring",
                 jid_j->valuestring);
        return;
    }

    ESP_LOGW(TAG, "job %s: fw update %s -> %s", jid_j->valuestring, cur,
             ver_j->valuestring);
    strlcpy(s.active_jid, jid_j->valuestring, sizeof(s.active_jid));
    job_nvs_save(s.active_jid, ver_j->valuestring);
    report_status(s.active_jid, "IN_PROGRESS", NULL);

    esp_err_t err = ota_updater_start(url_j->valuestring, sha_j->valuestring,
                                      ota_fail_cb, NULL);
    if (err != ESP_OK) ota_fail_cb(err, NULL);
}

static void handle_jobs_payload(const char *data, int len)
{
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return;
    cJSON *exec = cJSON_GetObjectItem(root, "execution");
    if (cJSON_IsObject(exec)) {
        handle_execution(exec);
    }
    cJSON_Delete(root);
}

/* ---- MQTT wiring --------------------------------------------------------- */

static void subscribe_and_kick(void)
{
    esp_mqtt_client_subscribe(s.mqtt, s.topic_notify, 1);
    esp_mqtt_client_subscribe(s.mqtt, s.topic_start_acc, 1);

    if (!s.reconcile_spawned) {
        s.reconcile_spawned = true;
        xTaskCreate(reconcile_task, "jobs_reconcile", 4096, NULL, 3, NULL);
    }

    /* Ask for the next queued job (covers jobs created while offline). */
    char topic[128];
    snprintf(topic, sizeof(topic), "$aws/things/%s/jobs/start-next", s.device_id);
    esp_mqtt_client_publish(s.mqtt, topic, "{}", 0, 1, 0);
}

static void jobs_mqtt_event_cb(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        subscribe_and_kick();
        break;
    case MQTT_EVENT_DATA:
        if (ev->topic_len > 0 &&
            (strncmp(ev->topic, s.topic_notify, ev->topic_len) == 0 ||
             strncmp(ev->topic, s.topic_start_acc, ev->topic_len) == 0)) {
            handle_jobs_payload(ev->data, ev->data_len);
        }
        break;
    default:
        break;
    }
}

esp_err_t iot_jobs_init(void)
{
    if (s.inited) return ESP_OK;
    s.mqtt      = (esp_mqtt_client_handle_t)mqtt_publisher_get_client();
    s.device_id = mqtt_publisher_get_device_id();
    if (!s.mqtt || !s.device_id) return ESP_ERR_INVALID_STATE;

    snprintf(s.topic_notify, sizeof(s.topic_notify),
             "$aws/things/%s/jobs/notify-next", s.device_id);
    snprintf(s.topic_start_acc, sizeof(s.topic_start_acc),
             "$aws/things/%s/jobs/start-next/accepted", s.device_id);

    esp_err_t err = esp_mqtt_client_register_event(s.mqtt, ESP_EVENT_ANY_ID,
                                                   jobs_mqtt_event_cb, NULL);
    if (err != ESP_OK) return err;
    s.inited = true;

    /* Same belt-and-suspenders as event_uploader: if the broker connected
     * before we registered, kick the subscribe now; the CONNECTED handler
     * covers every later reconnect. */
    if (mqtt_publisher_is_connected()) {
        subscribe_and_kick();
    }
    ESP_LOGI(TAG, "ready (thing=%s, fw ver=%s)", s.device_id,
             esp_app_get_description()->version);
    return ESP_OK;
}
