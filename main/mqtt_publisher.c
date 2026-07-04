/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * mqtt_publisher — see mqtt_publisher.h.
 */

#include "mqtt_publisher.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "lwip/inet.h"
#include "mqtt_client.h"
#include "nvs.h"

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

    /* ETH config for auto-reconnect (TD-005) */
    char                       eth_ip_str[16];      /* "192.168.10.2" */
    char                       eth_netmask_str[16]; /* "255.255.255.0" */
    char                       eth_gateway_str[16]; /* "192.168.10.1" */

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

    /* M4-OTA: static IP disables DHCP, so no DNS arrives automatically —
     * without this the IoT Core / S3 hostnames never resolve. Gateway
     * (facility router / Mac internet sharing) first, public DNS backup
     * (1.1.1.1 — 8.8.8.8 is unreachable from mainland China test sites). */
    esp_netif_dns_info_t dns = {0};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4 = info.gw;
    (void)esp_netif_set_dns_info(eth, ESP_NETIF_DNS_MAIN, &dns);
    ip4addr_aton("1.1.1.1", (ip4_addr_t *)&dns.ip.u_addr.ip4);
    (void)esp_netif_set_dns_info(eth, ESP_NETIF_DNS_BACKUP, &dns);

    ESP_LOGI(TAG, "ETH static IP %s/%s gw=%s set (DNS: gw + 1.1.1.1)",
             ip_str, netmask_str, gateway_str);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* ETH event handler (TD-005 fix)                                             */
/* -------------------------------------------------------------------------- */
static void eth_reconnect_handler(void *handler_args, esp_event_base_t base,
                                  int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base; (void)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "ETH_CONNECTED → reapplying static IP (TD-005 auto-reconnect)");
        /* Re-apply static IP configuration when ETH link comes back */
        esp_err_t err = configure_eth_static_ip(s.eth_ip_str,
                                               s.eth_netmask_str,
                                               s.eth_gateway_str);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to re-apply static IP: %d", err);
        } else {
            ESP_LOGI(TAG, "Static IP re-applied: %s/%s gw=%s",
                     s.eth_ip_str, s.eth_netmask_str, s.eth_gateway_str);
        }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "ETH_DISCONNECTED → MQTT will auto-reconnect when ETH recovers");
        break;
    default:
        break;
    }
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

/* ========================================================================== */
/* M4-OTA: AWS IoT Core mTLS support.                                         */
/* NVS namespace "mqtttls" may carry a broker URI override plus X.509         */
/* material written at provisioning time (tools/aws/provision_device.sh):     */
/*   uri  (str)  e.g. mqtts://xxx-ats.iot.ap-northeast-1.amazonaws.com:8883   */
/*   ca   (blob) Amazon Root CA 1 (PEM)                                       */
/*   cert (blob) device certificate (PEM)                                     */
/*   key  (blob) device private key (PEM)                                     */
/* Absent keys → compile-time default (local mosquitto, no TLS). Buffers are  */
/* malloc'd once and kept for the client's lifetime (esp-mqtt keeps refs).    */
/* ========================================================================== */
static struct {
    char *uri;
    char *ca, *cert, *key;   /* null-terminated PEM */
} s_tls;

static char *nvs_load_str_or_blob(nvs_handle_t nh, const char *k, bool blob)
{
    size_t len = 0;
    esp_err_t err = blob ? nvs_get_blob(nh, k, NULL, &len)
                         : nvs_get_str(nh, k, NULL, &len);
    if (err != ESP_OK || len == 0) return NULL;
    char *buf = malloc(len + 1);
    if (!buf) return NULL;
    err = blob ? nvs_get_blob(nh, k, buf, &len)
               : nvs_get_str(nh, k, buf, &len);
    if (err != ESP_OK) { free(buf); return NULL; }
    buf[len] = 0;   /* PEM must be null-terminated for esp-mqtt/mbedtls */
    return buf;
}

static void mqtt_load_tls_from_nvs(void)
{
    nvs_handle_t nh;
    if (nvs_open("mqtttls", NVS_READONLY, &nh) != ESP_OK) return;
    s_tls.uri  = nvs_load_str_or_blob(nh, "uri",  false);
    s_tls.ca   = nvs_load_str_or_blob(nh, "ca",   true);
    s_tls.cert = nvs_load_str_or_blob(nh, "cert", true);
    s_tls.key  = nvs_load_str_or_blob(nh, "key",  true);
    nvs_close(nh);
    if (s_tls.uri) ESP_LOGI(TAG, "broker override from NVS: %s", s_tls.uri);
    if (s_tls.ca && s_tls.cert && s_tls.key) {
        ESP_LOGI(TAG, "X.509 mTLS material loaded from NVS");
    }
}

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

    /* Store ETH config for auto-reconnect (TD-005) */
    strncpy(s.eth_ip_str,      local.eth_ip,      sizeof(s.eth_ip_str) - 1);
    strncpy(s.eth_netmask_str, local.eth_netmask, sizeof(s.eth_netmask_str) - 1);
    strncpy(s.eth_gateway_str, local.eth_gateway, sizeof(s.eth_gateway_str) - 1);
    s.eth_ip_str[sizeof(s.eth_ip_str) - 1] = '\0';
    s.eth_netmask_str[sizeof(s.eth_netmask_str) - 1] = '\0';
    s.eth_gateway_str[sizeof(s.eth_gateway_str) - 1] = '\0';

    /* ---- 1. Bring up Ethernet (BSP) ---- */
    esp_err_t err = bsp_eth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_eth_init: %d", err);
        return err;
    }

    /* ---- 2. Static IP on ETH netif ---- */
    err = configure_eth_static_ip(local.eth_ip, local.eth_netmask, local.eth_gateway);
    if (err != ESP_OK) return err;

    /* ---- 2.5. Register ETH event handler for auto-reconnect (TD-005) ---- */
    err = esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_CONNECTED,
                                     eth_reconnect_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ETH_CONNECTED handler: %d", err);
        return err;
    }
    err = esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED,
                                     eth_reconnect_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ETH_DISCONNECTED handler: %d", err);
        return err;
    }
    ESP_LOGI(TAG, "ETH auto-reconnect handlers registered (TD-005 fix)");

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
    mqtt_load_tls_from_nvs();   /* M4-OTA: IoT Core URI + X.509 overrides */
    esp_mqtt_client_config_t mc = {
        .broker.address.uri    = s_tls.uri ? s_tls.uri : local.broker_uri,
        .credentials.client_id = s.device_id,
        .session.keepalive     = 60,
        .network.reconnect_timeout_ms = 5000,     /* TD-005: longer reconnect timeout */
        .network.disable_auto_reconnect = false,  /* TD-005: ensure auto-reconnect enabled */
        .network.timeout_ms = 10000,              /* TD-005: network operation timeout */
    };
    if (s_tls.ca && s_tls.cert && s_tls.key) {
        /* AWS IoT Core mutual TLS (ADR-004): server verified against the
         * Amazon root CA, device authenticated by per-device X.509 cert. */
        mc.broker.verification.certificate    = s_tls.ca;
        mc.credentials.authentication.certificate = s_tls.cert;
        mc.credentials.authentication.key     = s_tls.key;
    }
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
    ESP_LOGI(TAG, "esp-mqtt client started (broker=%s%s)",
             s_tls.uri ? s_tls.uri : local.broker_uri,
             (s_tls.ca && s_tls.cert && s_tls.key) ? ", mTLS" : "");
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

/* -------------------------------------------------------------------------- */
/* Cleanup (TD-005)                                                           */
/* -------------------------------------------------------------------------- */

void mqtt_publisher_deinit(void)
{
    if (!s.inited) return;

    /* Unregister ETH event handlers */
    esp_event_handler_unregister(ETH_EVENT, ETHERNET_EVENT_CONNECTED, eth_reconnect_handler);
    esp_event_handler_unregister(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, eth_reconnect_handler);

    /* Stop MQTT client */
    if (s.client) {
        esp_mqtt_client_stop(s.client);
        esp_mqtt_client_destroy(s.client);
        s.client = NULL;
    }

    atomic_store(&s.connected, false);
    s.inited = false;
    ESP_LOGI(TAG, "mqtt_publisher deinitialized");
}
