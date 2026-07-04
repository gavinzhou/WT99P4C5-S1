/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * V5+V6: SDIO throughput test (P4 ↔ C5 via ESP-Hosted → 5GHz → Mac)
 *        extends the ADR-020 方案 A PoC
 *
 * Flow:
 *   1. Flash C5 via UART (P4 self-programs C5) [optional]
 *   2. Init esp_wifi → ESP-Hosted SDIO handshake
 *   3. Connect C5 as STA to standalone C5 TX AP "HyperFi_CSI_5G" ch36
 *   4. Get IP from C5 TX (192.168.4.x)
 *   5. TCP blast test to Mac (192.168.4.3:5001), report Mbps
 *
 * Wiring (J6 ↔ J5 jumper wires):
 *   P4 GPIO32 → J5 RXD, P4 GPIO33 ← J5 TXD, P4 GPIO26 → J5 BOOT, GND shared
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "c5_flasher.h"
#include "esp_hosted.h"      /* esp_hosted_connect_to_slave() */
#include "esp_hosted_misc.h" /* esp_hosted_register_custom_callback() — ADR-022 backport */
#include "shutter_test.h"    /* ADR-023 M3.1 regression test */
#include "m32_test.h"        /* ADR-023 M3.2 regression test (Poincaré + Collapse) */
#include "m33_test.h"        /* ADR-023 M3.3 regression test (Metrics + Quiet Detector) */
#include "m34_test.h"        /* ADR-023 M3.4 regression test (Fall Detector — Stage I) */
#include "pipeline.h"        /* ADR-023 M3.5.0 live CSI → algorithm chain */
#include "m35_proto_test.h"  /* ADR-023 M3.5.1 protobuf encode/decode roundtrip */
#include "mqtt_publisher.h"  /* ADR-023 M3.5.2 ETH static IP + esp-mqtt publish */
#include "load_test.h"       /* ADR-024 eval — C5 WiFi uplink airtime stress */
#include "m36_test.h"        /* ADR-023 M3.6.0 event_buffer regression test */
#include "m_breathing_test.h" /* M4.3 breathing Welch-estimator regression test */
#include "hf_config.h"        /* M4/PoC §3.1 — NVS commissioning config */
#include "hf_console.h"       /* M4/PoC §3.1 — commissioning REPL */
#include "event_uploader.h"  /* ADR-023 M3.6.2 raw context cloud upload */
#include "event_orchestrator.h"  /* ADR-023 M3.6.4 fall→buffer→encode→upload glue */
#include "ota_health.h"      /* M4-OTA — post-OTA rollback health gate */
#include "bsp/wt99p4c5_s1_board.h"  /* bsp_eth_init() — direct USB-Ethernet to Mac */

/* Set 1 to spawn the C5 WiFi uplink load test after Wi-Fi STA connects.
 * The test ramps UDP traffic 0→1000 kbps to 192.168.4.1:9999 (discard
 * port) and lets you read CSI fps degradation from the existing
 * csi_stats_task lines. Total runtime ~3.5 minutes. Set back to 0 for
 * normal operation. See load_test.h. */
#ifndef HYPERFI_LOAD_TEST_ENABLED
#define HYPERFI_LOAD_TEST_ENABLED 0
#endif

/* Throughput test config */
#define TEST_SSID          "HyperFi_CSI_5G"
#define TEST_PASS          "hyperfi2026"
#define IPERF_SERVER_IP    "192.168.4.3"
#define IPERF_SERVER_PORT  5001
#define TEST_DURATION_S    30
#define BLAST_BUF_SIZE     1024

/* HyperFi CSI pass-through event ID — MUST match slave_csi_hook.h */
#define HYPERFI_CSI_EVENT_ID 0x2001

/* HyperFi slave diagnostic text channel — MUST match slave_diag.h.
 * Slave sends human-readable text over this event; host just printf()s. */
#define HYPERFI_SLAVE_DIAG_EVENT_ID 0x2002

/* On-wire CSI header layout — MUST match hyperfi_csi_wire_hdr_t on slave side */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    int8_t   rssi;
    int8_t   noise_floor;
    uint16_t seq;
    uint16_t len;
    uint8_t  mac[6];
    uint8_t  bw;
    uint8_t  reserved[3];
} hyperfi_csi_wire_hdr_t;

/* CSI stats — printed every ~1 sec via a simple counter */
static uint32_t s_csi_frames_received = 0;
static uint32_t s_csi_bytes_received  = 0;
static int64_t  s_csi_first_us        = 0;

/* M4-OTA: post-OTA health gate probes (see ota_health.h).
 * fully_ok  = boot self-tests passed + MQTT uplink up + CSI flowing.
 * uplink_ok = MQTT up — enough to keep the image (fix can come via OTA). */
static bool s_selftests_ok = false;

static bool ota_probe_fully_ok(void *ctx)
{
    return s_selftests_ok && mqtt_publisher_is_connected()
           && s_csi_frames_received >= 100;
}

static bool ota_probe_uplink_ok(void *ctx)
{
    return mqtt_publisher_is_connected();
}

/* Wi-Fi event group */
static EventGroupHandle_t s_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "app_main";

/* Set to 1 to re-flash C5 at every boot (~2 min delay).
 * Set to 0 to skip flash and go straight to ESP-Hosted SDIO init.
 *
 * 2026-04-23 note: skip-flash path is currently NOT reliable — C5 residual
 * state from previous boot causes SDIO handshake to fail (ESP_ERR_TIMEOUT
 * 0x107 on send_op_cond). BOOT=HIGH + EN reset from P4 side doesn't help.
 * Need to investigate (maybe add a longer post-reset delay, or check C5
 * partition integrity). For now, stay at 1 for every-boot fresh flash. */
#ifndef C5_DO_FLASH_ON_BOOT
#define C5_DO_FLASH_ON_BOOT  1
#endif

/* ========================================================================== */
/* HyperFi CSI consumer (ADR-021 M2 + ADR-022) — receives CSI events from      */
/* slave C5 via Custom RPC backport.                                           */
/* ========================================================================== */

/* Slave diagnostic text consumer (0x2002) — just printf the message. */
static void on_slave_diag(uint32_t msg_id, const uint8_t *data,
                          size_t data_len, void *ctx)
{
    (void)ctx;
    (void)msg_id;
    if (!data || data_len == 0) return;
    /* Slave sends null-terminated UTF-8. Trust the terminator but cap at 256
     * to defend against malformed frames. */
    char buf[260];
    size_t n = data_len < sizeof(buf) - 1 ? data_len : sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = 0;
    printf("[SLAVE] %s\n", buf);
}

/* Telemetry callback — print one console line + publish via MQTT.
 * C linkage so it matches pipeline_telemetry_cb_t exactly. */
extern "C" void log_telemetry_cb(const pipeline_telemetry_t *t, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG,
             "[telemetry] ts=%llu  C=%.4f state=%d quiet=%d  "
             "norm_cv=%.3f shape=%.3f  G=%.1f  fps=%.0f n=%d  "
             "rssi=%d  fall=%d conf=%.2f%s%s  br=%.1fbpm/c%.0f/s%u  mqtt=%s",
             (unsigned long long)t->timestamp_us,
             (double)t->collapse_index, (int)t->fsm_state, (int)t->quiet_period,
             (double)t->norm_cv, (double)t->shape_corr, (double)t->dynamic_gain_G,
             (double)t->csi_fps, t->n_frames_in_window,
             t->rssi_avg,
             (int)t->fall_detected, (double)t->fall_confidence,
             t->fall_event_rising_edge ? "  ★EVENT" : "",
             (t->fall_best_pattern_idx >= 0 && t->fall_event_rising_edge)
                 ? t->fall_best_pattern_name : "",
             (double)t->breathing_bpm, (double)t->breathing_confidence,
             (unsigned)t->breathing_state,
             mqtt_publisher_is_connected() ? "ON" : "OFF");

    /* M3.5.2: publish to MQTT (drops silently if broker disconnected). */
    mqtt_publisher_publish_telemetry(t);
    if (t->fall_event_rising_edge) {
        mqtt_publisher_publish_alert(t);
    }

    /* M3.6.4: kick off the rolling-buffer→encode→cloud chain. No-op when
     * fall_event_rising_edge is false, so it's cheap on every emit. */
    event_orchestrator_handle(t);
}

static void on_csi_from_slave(uint32_t msg_id, const uint8_t *data,
                              size_t data_len, void *ctx)
{
    (void)ctx;
    (void)msg_id;

    if (!data || data_len < sizeof(hyperfi_csi_wire_hdr_t)) {
        return;  /* malformed */
    }

    const hyperfi_csi_wire_hdr_t *hdr = (const hyperfi_csi_wire_hdr_t *)data;

    if (s_csi_first_us == 0) {
        s_csi_first_us = esp_timer_get_time();
        ESP_LOGI(TAG, "[CSI] ★ first frame arrived — mac=%02X%02X%02X%02X%02X%02X "
                 "rssi=%d seq=%u len=%u bw=%u",
                 hdr->mac[0], hdr->mac[1], hdr->mac[2],
                 hdr->mac[3], hdr->mac[4], hdr->mac[5],
                 hdr->rssi, hdr->seq, hdr->len, hdr->bw);
    }

    s_csi_frames_received++;
    s_csi_bytes_received += data_len;

    /* ADR-023 M3.5.0 — feed frame into live pipeline.
     * Wire IQ payload starts immediately after the header. */
    const size_t hdr_sz = sizeof(hyperfi_csi_wire_hdr_t);
    const size_t iq_len = data_len - hdr_sz;
    /* HT20 expects 53 SC × 2 (Im,Re) × 1B = 106 B. Defensive bound. */
    if (iq_len >= (size_t)(2 * 53)) {
        const int8_t *iq = (const int8_t *)(data + hdr_sz);
        pipeline_on_csi_frame(
            esp_timer_get_time(),
            hdr->rssi,
            hdr->noise_floor,
            iq,
            53);
    }
}

static void csi_stats_task(void *arg)
{
    (void)arg;
    uint32_t prev_frames = 0;
    uint32_t prev_bytes  = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t frames = s_csi_frames_received;
        uint32_t bytes  = s_csi_bytes_received;
        float fps = (frames - prev_frames) / 5.0f;
        float kbps = (bytes - prev_bytes) * 8.0f / 5000.0f;
        ESP_LOGI(TAG, "[CSI] stats: total=%lu frames (%.1f fps), %.1f kbps",
                 (unsigned long)frames, fps, kbps);
        prev_frames = frames;
        prev_bytes = bytes;
    }
}

/* CSI trigger task: periodically send UDP packets to the SoftAP gateway so the
 * AP emits unicast reply frames. ESP32-C5's CSI engine only fires on unicast
 * frames addressed to the STA, so we need this active traffic. 100 Hz is the
 * same rate used by the standalone c5_rx_2g4/5g firmware. */
static void csi_trigger_task(void *arg)
{
    (void)arg;
    /* Wait for IP */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    /* Extra settle time after IP acquired */
    vTaskDelay(pdMS_TO_TICKS(1000));

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "[CSI-trigger] socket() failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(5500);          /* same port as c5_tx_5g's UDP broadcast */
    inet_aton("192.168.4.1", &dest.sin_addr);   /* SoftAP gateway */

    uint8_t payload[16];
    memset(payload, 0x5A, sizeof(payload));
    uint32_t seq = 0;

    ESP_LOGI(TAG, "[CSI-trigger] pinging 192.168.4.1:5500 at 100Hz to force AP unicast replies");

    while (1) {
        memcpy(payload, &seq, sizeof(seq));
        sendto(sock, payload, sizeof(payload), 0,
               (struct sockaddr *)&dest, sizeof(dest));
        seq++;
        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz */
    }
}

/* Wi-Fi event handlers */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            /* HyperFi M2.9 fix: slave 2.12.6 + host 2.0.13 emits STA_START
             * multiple times during init; repeated esp_wifi_connect() calls
             * trigger a second netif_add inside esp_wifi_remote → lwip
             * "netif already added" assert. Gate with a static flag so only
             * the FIRST STA_START triggers connect. DISCONNECTED still retries. */
            static bool first_start_handled = false;
            if (!first_start_handled) {
                first_start_handled = true;
                esp_wifi_connect();
                ESP_LOGI(TAG, "[Wi-Fi] STA start → connecting to %s...", TEST_SSID);
            } else {
                ESP_LOGW(TAG, "[Wi-Fi] STA_START re-entry, skipping duplicate connect");
            }
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "[Wi-Fi] disconnected, retrying...");
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "[Wi-Fi] STA connected to AP");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "[Wi-Fi] ✓ GOT IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* TCP throughput test — connect retries until Mac nc is ready */
static void throughput_test_task(void *arg)
{
    (void)arg;

    /* Wait for IP */
    ESP_LOGI(TAG, "[V5] Waiting for Wi-Fi + IP...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    static uint8_t buf[BLAST_BUF_SIZE];
    memset(buf, 0xAA, sizeof(buf));

    int attempt = 0;
    while (1) {
        attempt++;
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "[V5] socket() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* 5-second connect timeout */
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(IPERF_SERVER_PORT);
        inet_aton(IPERF_SERVER_IP, &dest.sin_addr);

        ESP_LOGI(TAG, "[V5] attempt %d: connecting to %s:%d...",
                 attempt, IPERF_SERVER_IP, IPERF_SERVER_PORT);

        if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) < 0) {
            ESP_LOGW(TAG, "[V5] connect failed: errno=%d (probably Mac nc not started yet)", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* Connected — start blasting */
        ESP_LOGI(TAG, "[V5] ✓ Connected! Blasting for %d seconds...", TEST_DURATION_S);
        ESP_LOGI(TAG, "========================================");

        int64_t t_start = esp_timer_get_time();
        int64_t t_last_report = t_start;
        int64_t total_bytes = 0;
        int64_t bytes_since_report = 0;
        bool failed = false;

        while (1) {
            int64_t now = esp_timer_get_time();
            int64_t elapsed_us = now - t_start;
            if (elapsed_us >= (int64_t)TEST_DURATION_S * 1000000) break;

            int sent = send(sock, buf, BLAST_BUF_SIZE, 0);
            if (sent < 0) {
                ESP_LOGE(TAG, "[V5] send() failed at %lld ms: errno=%d",
                         elapsed_us / 1000, errno);
                failed = true;
                break;
            }
            total_bytes += sent;
            bytes_since_report += sent;

            if (now - t_last_report >= 1000000) {
                float mbps = bytes_since_report * 8.0f / (now - t_last_report);
                ESP_LOGI(TAG, "[V5] t=%2lld s — %7.2f Mbps  (total=%lld KB)",
                         elapsed_us / 1000000, mbps, total_bytes / 1024);
                bytes_since_report = 0;
                t_last_report = now;
            }
        }

        int64_t t_end = esp_timer_get_time();
        float elapsed_s = (t_end - t_start) / 1000000.0f;
        float avg_mbps = total_bytes * 8.0f / (t_end - t_start);

        ESP_LOGI(TAG, "========================================");
        if (failed) {
            ESP_LOGW(TAG, "  [V5] Test ABORTED after %.2f s", elapsed_s);
        } else {
            ESP_LOGI(TAG, "  [V5] ✓ TEST COMPLETE");
        }
        ESP_LOGI(TAG, "  Total: %lld bytes (%lld KB) in %.2f s",
                 total_bytes, total_bytes / 1024, elapsed_s);
        ESP_LOGI(TAG, "  Average: %.2f Mbps  (%.2f MB/s)",
                 avg_mbps, total_bytes / (elapsed_s * 1024 * 1024));
        ESP_LOGI(TAG, "  CSI requirement: ~0.6 Mbps (100fps × 768B)");
        ESP_LOGI(TAG, "  Margin: %.0fx", avg_mbps / 0.6f);
        ESP_LOGI(TAG, "========================================");

        close(sock);
        break;  /* test done — exit the retry loop */
    }

    ESP_LOGI(TAG, "[V5] Throughput test finished — idle.");
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    /* ---- NVS ---- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ---- M4/PoC §3.1: load commissioning config from NVS (defaults if empty) ---- */
    hf_config_load();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  V5+V6: SDIO Throughput Test");
    ESP_LOGI(TAG, "  Board: WT99P4C5-S1 (P4 + C5)");
    ESP_LOGI(TAG, "  Target: %s via %s (ch36 5G)", IPERF_SERVER_IP, TEST_SSID);
    ESP_LOGI(TAG, "========================================");

    /* ---- Boot self-tests (results feed the M4-OTA health gate below) ---- */
    bool st_ok = true;
    /* ADR-023 M3.1: Spatial Shutter (no-op if fixtures absent) */
    st_ok &= (shutter_run_self_test() == ESP_OK);
    /* ADR-023 M3.2: Poincaré + Collapse */
    st_ok &= (m32_run_self_test() == ESP_OK);
    /* ADR-023 M3.3: Metrics + Quiet Detector */
    st_ok &= (m33_run_self_test() == ESP_OK);
    /* ADR-023 M3.4: Fall Detector (Stage I post-filter) */
    st_ok &= (m34_run_self_test() == ESP_OK);
    /* ADR-023 M3.5.1: protobuf encode/decode roundtrip */
    st_ok &= (m35_run_proto_self_test() == ESP_OK);
    /* ADR-023 M3.6.0: event_buffer rolling buffer */
    st_ok &= (m36_run_self_test() == ESP_OK);
    /* M4.3: Breathing rate Welch-estimator */
    st_ok &= (breathing_run_self_test() == ESP_OK);
    s_selftests_ok = st_ok;

    /* ---- M4-OTA: arm the post-OTA rollback health gate.
     * No-op on normal (USB-flashed / already-verified) boots. On the first
     * boot after an esp_https_ota it must see fully_ok within 300 s, else
     * uplink decides between keep (WARN) and rollback reboot. Armed here —
     * before C5 flash / SDIO — so a hang later in bring-up still ends in
     * a watchdog/reset and the bootloader reverts to the previous slot. */
    {
        ota_health_config_t hcfg = {
            .fully_ok  = ota_probe_fully_ok,
            .uplink_ok = ota_probe_uplink_ok,
            .ctx       = NULL,
            .timeout_s = 300,
        };
        ota_health_start(&hcfg);
    }

    /* ---- ADR-023 M3.5.0: bring up live pipeline (CSI → algorithms → telemetry) ---- */
    if (pipeline_init(NULL) == ESP_OK) {
        pipeline_register_telemetry_cb(&log_telemetry_cb, NULL);
        ESP_LOGI(TAG, "[Pipeline] M3.5.0 ready — telemetry will emit at 1Hz");
    } else {
        ESP_LOGE(TAG, "[Pipeline] init failed — pipeline disabled");
    }

    /* ---- M4/PoC §3.1: on-site commissioning console (hf show/set/save) ---- */
    hf_console_init();

    /* ---- Step 1: Flash C5 via UART (conditional) ---- */
#if C5_DO_FLASH_ON_BOOT
    ESP_LOGI(TAG, "[Step 1] Flashing C5 via UART (~2 min)...");
    if (c5_full_flash() == ESP_OK) {
        ESP_LOGI(TAG, "[Step 1] ✓ C5 flash completed");
    } else {
        ESP_LOGE(TAG, "[Step 1] ✗ C5 flash FAILED — trying SDIO anyway");
    }
#else
    ESP_LOGI(TAG, "[Step 1] Skipped flash — releasing C5 to normal boot (BOOT=HIGH + EN reset)...");
    /* Must explicitly set BOOT high + reset C5 before SDIO init.
     * Otherwise BOOT pin floats, C5 may enter download mode,
     * SDIO handshake times out with ESP_ERR_TIMEOUT 0x107. */
    c5_release_normal_boot_public();
#endif

    /* ---- Step 2: Init Wi-Fi stack following official esp-hosted-mcu example order ----
     *
     * IMPORTANT ORDER (verified against examples/host_hosted_events/main/main.c):
     *   1. esp_netif_init()                         — lwip + netif core init
     *   2. esp_event_loop_create_default()          — event loop
     *   3. Register WIFI_EVENT + IP_EVENT handlers  — BEFORE any netif creation
     *   4. esp_hosted_connect_to_slave()            — explicit SDIO handshake
     *      (Previously we let esp_wifi_init() trigger SDIO implicitly, which caused
     *       v2.12.6 slave to auto-register its own netif during transport init,
     *       then our esp_netif_create_default_wifi_sta() became a duplicate →
     *       "netif already added" assert at esp_wifi_start → STA_START event.)
     *   5. esp_netif_create_default_wifi_sta()      — AFTER transport up
     *   6. esp_wifi_init / set_mode / set_config / start
     */
    ESP_LOGI(TAG, "[Step 2] Initializing Wi-Fi stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create event group + register handlers BEFORE netif creation (per example) */
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* Explicitly complete SDIO handshake BEFORE creating netif.
     * esp_hosted_host_init() constructor already called esp_hosted_init() before main.
     * esp_hosted_connect_to_slave() → esp_hosted_reconfigure() → transport_drv_reconfigure()
     * This is synchronous in v2.0.13 — returns after SDIO INIT event received. */
    ESP_LOGI(TAG, "[Step 2] Connecting to C5 slave via SDIO (esp_hosted_connect_to_slave)...");
    int hosted_err = esp_hosted_connect_to_slave();
    if (hosted_err != 0) {
        ESP_LOGE(TAG, "[Step 2] esp_hosted_connect_to_slave failed: %d", hosted_err);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "[Step 2] ✓ SDIO handshake complete");

    /* Register HyperFi CSI consumer (ADR-022 Custom RPC backport).
     * Must happen after SDIO handshake so the RPC channel is live;
     * can happen before or after esp_wifi_init(). Do it now for clarity. */
    esp_err_t csi_reg = esp_hosted_register_custom_callback(
            HYPERFI_CSI_EVENT_ID, on_csi_from_slave, NULL);
    if (csi_reg != ESP_OK) {
        ESP_LOGW(TAG, "[Step 2] esp_hosted_register_custom_callback failed: 0x%x", csi_reg);
    } else {
        ESP_LOGI(TAG, "[Step 2] ✓ HyperFi CSI consumer registered (event 0x%04X)",
                 HYPERFI_CSI_EVENT_ID);
    }

    /* HyperFi slave diagnostic text channel (M2.9): slave sends printf-style
     * messages over event 0x2002; we just echo to host console as [SLAVE] ... */
    esp_err_t diag_reg = esp_hosted_register_custom_callback(
            HYPERFI_SLAVE_DIAG_EVENT_ID, on_slave_diag, NULL);
    if (diag_reg != ESP_OK) {
        ESP_LOGW(TAG, "[Step 2] slave diag callback reg failed: 0x%x", diag_reg);
    } else {
        ESP_LOGI(TAG, "[Step 2] ✓ Slave diag consumer registered (event 0x%04X)",
                 HYPERFI_SLAVE_DIAG_EVENT_ID);
    }

    xTaskCreate(csi_stats_task, "csi_stats", 3072, NULL, 1, NULL);

    /* Now safe to create netif — transport is up, no race with framework auto-create */
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        ESP_LOGI(TAG, "[Step 2] WIFI_STA_DEF netif already exists (framework auto-created) — reusing");
    } else {
        sta_netif = esp_netif_create_default_wifi_sta();
        ESP_LOGI(TAG, "[Step 2] WIFI_STA_DEF netif created manually");
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[Step 2] esp_wifi_init failed: 0x%x", err);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "[Step 2] ✓ esp_wifi_init OK");

    /* ---- ADR-023 M3.5.2: bring up Ethernet (static IP) + esp-mqtt ----
     * ETH and Wi-Fi run independently: Wi-Fi via C5 carries CSI traffic,
     * Ethernet (USB-Eth direct to Mac) carries MQTT to local mosquitto.
     * Started here so MQTT can begin retrying broker connect in parallel
     * with Wi-Fi association below. */
    {
        const mqtt_publisher_config_t mqcfg = MQTT_PUBLISHER_CONFIG_DEFAULT();
        if (mqtt_publisher_init(&mqcfg) == ESP_OK) {
            ESP_LOGI(TAG, "[MQTT] M3.5.2 ready — broker=%s, telemetry will publish once connected",
                     mqcfg.broker_uri);
        } else {
            ESP_LOGW(TAG, "[MQTT] init failed — telemetry will only print to console");
        }

        /* M3.6.2: piggyback on the same MQTT client for raw-context upload
         * request/response. Subscribe to upload_url, ready to fire HTTPS PUT
         * once a fall_event triggers the M3.6.4 wiring. */
        const event_uploader_config_t ucfg = EVENT_UPLOADER_CONFIG_DEFAULT();
        if (event_uploader_init(&ucfg) == ESP_OK) {
            ESP_LOGI(TAG, "[Uploader] M3.6.2 ready — raw context PUT chain armed");
        } else {
            ESP_LOGW(TAG, "[Uploader] init failed — raw context upload disabled");
        }

        /* M3.6.4: orchestrator that turns fall_event_rising_edge into the
         * snapshot → encode → upload chain. */
        const event_orchestrator_config_t ocfg = EVENT_ORCHESTRATOR_CONFIG_DEFAULT();
        if (event_orchestrator_init(&ocfg) == ESP_OK) {
            ESP_LOGI(TAG, "[Orchestrator] M3.6.4 ready — fall→cloud chain wired");
        } else {
            ESP_LOGW(TAG, "[Orchestrator] init failed — fall events will not upload");
        }
    }

    /* ---- Step 3: Configure STA + connect to HyperFi_CSI_5G ---- */
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, TEST_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, TEST_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "[Step 3] ✓ Wi-Fi started, connecting to %s...", TEST_SSID);

    /* ---- Step 4: Launch CSI trigger + throughput tasks ---- */
    /* CSI-trigger actively sends UDP so AP replies trigger CSI on slave side. */
    xTaskCreate(csi_trigger_task, "csi_trig", 4096, NULL, 4, NULL);
    xTaskCreate(throughput_test_task, "thrpt", 8192, NULL, 5, NULL);

#if HYPERFI_LOAD_TEST_ENABLED
    /* ADR-024 evaluation — measure CSI fps degradation under varying
     * C5-radio uplink load. Sends UDP datagrams to 192.168.4.1:9999
     * (no listener; SoftAP drops). Eyeball the [CSI] stats lines and
     * the [load_test] step lines in the monitor to correlate fps vs
     * upload kbps. Disable by setting HYPERFI_LOAD_TEST_ENABLED to 0. */
    {
        const load_test_config_t lt_cfg = LOAD_TEST_CONFIG_DEFAULT();
        if (load_test_start(&lt_cfg) == ESP_OK) {
            ESP_LOGI(TAG, "[load_test] task spawned — warm-up %ds, then "
                          "ramp 0→1000 kbps × 30s/step",
                     lt_cfg.warmup_sec);
        } else {
            ESP_LOGW(TAG, "[load_test] failed to spawn");
        }
    }
#endif

    /* Idle loop — throughput task takes over */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        ESP_LOGI(TAG, "(uptime=%lld s)", (long long)(esp_log_timestamp() / 1000));
    }
}
