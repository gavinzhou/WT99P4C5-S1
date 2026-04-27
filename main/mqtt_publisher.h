/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * mqtt_publisher — ADR-023 M3.5.2.
 *
 * Brings up Ethernet (static IP, direct USB-Eth to Mac) + esp-mqtt client,
 * publishes pipeline_telemetry_t snapshots as TelemetryReport on
 *   hyperfi/{device_id}/telemetry
 * and fall events as AlertReport on
 *   hyperfi/{device_id}/event
 *
 * No TLS, anonymous broker — M3 PoC against local Mac mosquitto only.
 * Phase 2 will swap broker URI + add X.509 for AWS IoT Core (ADR-004).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Static IP for the P4's Ethernet netif (Mac side: 192.168.10.1). */
    const char *eth_ip;        /* default "192.168.10.2" */
    const char *eth_netmask;   /* default "255.255.255.0" */
    const char *eth_gateway;   /* default "192.168.10.1" */

    /* MQTT broker URI, e.g. "mqtt://192.168.10.1:1883". */
    const char *broker_uri;

    /* Client / topic identity. device_id should be unique per board. */
    const char *device_id;     /* default uses base MAC tail */
} mqtt_publisher_config_t;

#define MQTT_PUBLISHER_CONFIG_DEFAULT() {           \
    .eth_ip      = "192.168.10.2",                  \
    .eth_netmask = "255.255.255.0",                 \
    .eth_gateway = "192.168.10.1",                  \
    .broker_uri  = "mqtt://192.168.10.1:1883",      \
    .device_id   = NULL,                            \
}

/**
 * Initialize ETH netif (static IP), start ESP-MQTT client, register
 * connection events. Idempotent — safe to call once after esp_netif_init
 * + esp_event_loop_create_default. If cfg=NULL or any string is NULL,
 * defaults from MQTT_PUBLISHER_CONFIG_DEFAULT are used; default device_id
 * derives from base MAC last 3 bytes ("p4-AABBCC").
 *
 * Returns ESP_OK if ETH and MQTT client both started; ESP_FAIL on
 * unrecoverable setup error. The MQTT client retries broker connection
 * indefinitely on its own task — ESP_OK does NOT imply broker reachable.
 */
esp_err_t mqtt_publisher_init(const mqtt_publisher_config_t *cfg);

/** True once the MQTT client has reached MQTT_CONNECTED state. */
bool mqtt_publisher_is_connected(void);

/**
 * Encode a pipeline_telemetry_t as TelemetryReport and publish to
 * hyperfi/{device_id}/telemetry at QoS 0. Drops silently if not connected
 * (telemetry is best-effort; loss is acceptable in M3 PoC).
 */
void mqtt_publisher_publish_telemetry(const pipeline_telemetry_t *t);

/**
 * Encode + publish an AlertReport. QoS 1 for at-least-once delivery.
 * Caller should only invoke when t->fall_event_rising_edge is true.
 */
void mqtt_publisher_publish_alert(const pipeline_telemetry_t *t);

#ifdef __cplusplus
}
#endif
