/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * mqtt_publisher — see mqtt_publisher.h.
 */

#include "mqtt_publisher.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "mqtt_client.h"

#include "bsp/wt99p4c5_s1_board.h"
#include "proto_codec.h"

static const char *TAG = "mqtt_pub";

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

#define DEVICE_ID_MAX  32
#define TOPIC_MAX      96

typedef struct {
    bool                       inited;
    esp_mqtt_client_handle_t   client;
    atomic_bool                connected;
    char                       device_id[DEVICE_ID_MAX];
    char                       topic_telemetry[TOPIC_MAX];
    char                       topic_event[TOPIC_MAX];

    /* Counters for visibility in monitor */
    uint32_t                   pub_ok;
    uint32_t                   pub_fail;
    uint32_t                   pub_drop_disconnected;
} mqtt_state_t;

static mqtt_state_t s = {0};

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void make_default_device_id(char *out, size_t n)
{
    uint8_t mac[6] = {0};
    /* ESP_MAC_BASE works pre-Wi-Fi-init (reads from eFuse directly).
     * Avoid ESP_MAC_WIFI_STA here because mqtt_publisher_init runs
     * before esp_wifi_start. */
    esp_err_t err = esp_read_mac(mac, ESP_MAC_BASE);
    if (err != ESP_OK) {
        snprintf(out, n, "p4-unknown");
        return;
    }
    snprintf(out, n, "p4-%02x%02x%02x", mac[3], mac[4], mac[5]);
}

static bool parse_ipv4(const char *s, esp_ip4_addr_t *out)
{
    unsigned int a, b, c, d;
    if (!s || sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
    if (a > 255 || b > 255 || c > 255 || d > 255)            return false;
    out->addr = ESP_IP4TOADDR(a, b, c, d);
    return true;
}

static esp_err_t configure_eth_static_ip(const char *ip_str,
                                          const char *netmask_str,
                                          const char *gateway_str)
{
    esp_netif_t *eth = esp_netif_get_handle_from_ifkey("ETH_DEF");
    if (!eth) {
        ESP_LOGE(TAG, "ETH_DEF netif not found — bsp_eth_init() must run first");
        return ESP_ERR_INVALID_STATE;
    }

    /* Stop DHCP client so static IP isn't overwritten when link comes up. */
    (void)esp_netif_dhcpc_stop(eth);   /* may already be stopped — ignore err */

    esp_netif_ip_info_t info = {0};
    if (!parse_ipv4(ip_str,      &info.ip)      ||
        !parse_ipv4(netmask_str, &info.netmask) ||
        !parse_ipv4(gateway_str, &info.gw)) {
        ESP_LOGE(TAG, "invalid IP literal in config (%s / %s / %s)",
                 ip_str, netmask_str, gateway_str);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_netif_set_ip_info(eth, &info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_set_ip_info: %d", err);
        return err;
    }
    ESP_LOGI(TAG, "ETH static IP %s/%s gw=%s set", ip_str, netmask_str, gateway_str);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* MQTT event handler                                                          */
/* -------------------------------------------------------------------------- */
static void mqtt_event_cb(void *handler_args, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base; (void)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        atomic_store(&s.connected, true);
        ESP_LOGI(TAG, "✓ MQTT_CONNECTED → broker accepted client_id=%s",
                 s.device_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        atomic_store(&s.connected, false);
        ESP_LOGW(TAG, "MQTT_DISCONNECTED — auto-reconnect engaged");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Init                                                                        */
/* -------------------------------------------------------------------------- */
esp_err_t mqtt_publisher_init(const mqtt_publisher_config_t *cfg)
{
    if (s.inited) return ESP_ERR_INVALID_STATE;

    mqtt_publisher_config_t local = MQTT_PUBLISHER_CONFIG_DEFAULT();
    if (cfg) {
        if (cfg->eth_ip)       local.eth_ip      = cfg->eth_ip;
        if (cfg->eth_netmask)  local.eth_netmask = cfg->eth_netmask;
        if (cfg->eth_gateway)  local.eth_gateway = cfg->eth_gateway;
        if (cfg->broker_uri)   local.broker_uri  = cfg->broker_uri;
        if (cfg->device_id)    local.device_id   = cfg->device_id;
    }

    /* ---- 1. Bring up Ethernet (BSP) ---- */
    esp_err_t err = bsp_eth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_eth_init: %d", err);
        return err;
    }

    /* ---- 2. Static IP on ETH netif ---- */
    err = configure_eth_static_ip(local.eth_ip, local.eth_netmask, local.eth_gateway);
    if (err != ESP_OK) return err;

    /* ---- 3. Identity + topics ---- */
    if (local.device_id) {
        strncpy(s.device_id, local.device_id, sizeof(s.device_id) - 1);
    } else {
        make_default_device_id(s.device_id, sizeof(s.device_id));
    }
    snprintf(s.topic_telemetry, sizeof(s.topic_telemetry),
             "hyperfi/%s/telemetry", s.device_id);
    snprintf(s.topic_event,     sizeof(s.topic_event),
             "hyperfi/%s/event",     s.device_id);
    ESP_LOGI(TAG, "device_id='%s'", s.device_id);
    ESP_LOGI(TAG, "topics: %s, %s", s.topic_telemetry, s.topic_event);

    /* ---- 4. esp-mqtt client ---- */
    esp_mqtt_client_config_t mc = {
        .broker.address.uri    = local.broker_uri,
        .credentials.client_id = s.device_id,
        .session.keepalive     = 60,
        .network.reconnect_timeout_ms = 2000,
    };
    s.client = esp_mqtt_client_init(&mc);
    if (!s.client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return ESP_FAIL;
    }
    err = esp_mqtt_client_register_event(s.client, ESP_EVENT_ANY_ID,
                                          mqtt_event_cb, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register_event: %d", err);
        return err;
    }
    /* Set inited BEFORE start so that if MQTT_CONNECTED fires synchronously
     * inside client_start, the event handler sees a fully-initialized state.
     * (s.connected starts as false from the zero-init of `s`.) */
    s.inited = true;
    err = esp_mqtt_client_start(s.client);
    if (err != ESP_OK) {
        s.inited = false;
        ESP_LOGE(TAG, "client_start: %d", err);
        return err;
    }
    ESP_LOGI(TAG, "esp-mqtt client started (broker=%s)", local.broker_uri);
    return ESP_OK;
}

bool mqtt_publisher_is_connected(void)
{
    return s.inited && atomic_load(&s.connected);
}

/* -------------------------------------------------------------------------- */
/* Publishing                                                                  */
/* -------------------------------------------------------------------------- */

static void publish_blob(const char *topic, const uint8_t *buf, int len, int qos)
{
    if (!s.inited) return;
    if (!atomic_load(&s.connected)) {
        s.pub_drop_disconnected++;
        return;
    }
    int msg_id = esp_mqtt_client_publish(s.client, topic,
                                          (const char *)buf, len, qos, 0);
    if (msg_id < 0) {
        s.pub_fail++;
        ESP_LOGW(TAG, "publish failed (topic=%s len=%d msg_id=%d)",
                 topic, len, msg_id);
    } else {
        s.pub_ok++;
    }
}

void mqtt_publisher_publish_telemetry(const pipeline_telemetry_t *t)
{
    if (!t) return;
    uint8_t buf[PROTO_TELEMETRY_BUF_SIZE];
    int n = proto_codec_encode_telemetry(t, buf, sizeof(buf));
    if (n <= 0) {
        s.pub_fail++;
        return;
    }
    publish_blob(s.topic_telemetry, buf, n, /*qos=*/0);
}

void mqtt_publisher_publish_alert(const pipeline_telemetry_t *t)
{
    if (!t) return;
    uint8_t buf[PROTO_ALERT_BUF_SIZE];
    int n = proto_codec_encode_alert(t, buf, sizeof(buf));
    if (n <= 0) {
        s.pub_fail++;
        return;
    }
    publish_blob(s.topic_event, buf, n, /*qos=*/1);
    ESP_LOGI(TAG, "★ alert published (%d B)", n);
}

/* -------------------------------------------------------------------------- */
/* Shared accessors                                                            */
/* -------------------------------------------------------------------------- */

void *mqtt_publisher_get_client(void)
{
    return s.inited ? (void *)s.client : NULL;
}

const char *mqtt_publisher_get_device_id(void)
{
    return s.inited ? s.device_id : NULL;
}
