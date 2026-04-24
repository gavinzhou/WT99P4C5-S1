/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * C5 Flasher Implementation (ADR-020 方案 A PoC)
 *
 * Uses ESP-Serial-Flasher library (espressif/esp-serial-flasher, v1.7+)
 * to let P4 act as a programmer for C5 via UART.
 */

#include "c5_flasher.h"

#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_loader.h"
#include "esp32_port.h"

static const char *TAG = "c5_flasher";

/* ------------- Wiring (see c5_flasher.h) ------------- */
#define PIN_P4_UART_TX     32   /* → J5 RXD */
#define PIN_P4_UART_RX     33   /* ← J5 TXD */
#define PIN_P4_C5_BOOT     26   /* → J5 BOOT (C5 IO28 strapping) */
#define PIN_P4_C5_EN       54   /* → C5 EN, PCB 直连 */
#define UART_PORT_NUM      1    /* P4 UART1 (UART0 is used by USB-UART) */
#define BOOT_BAUD_RATE     115200
/* 2026-04-24 Onboard-trace baud staircase (WT99P4C5-S1, P4↔C5 internal trace):
 *   115200   → 131.0 s  (initial, dupont wires on 2026-04-22)
 *   921600   →  21.7 s  ( 6.0×)
 *   1500000  →  15.6 s  ( 8.4×)
 *   2000000  →  13.6 s  ( 9.6× — C5 ROM UART ceiling) ← current
 *
 * Per-block ACK overhead (~5 ms / 1KB block × 1336 blocks ≈ 6.5 s) dominates
 * above 1.5 Mbps — the UART is no longer the bottleneck. Eliminating this
 * requires a flasher stub for C5, which esp-serial-flasher v1.11 does NOT
 * ship (esp_stubs.c ESP32C5_CHIP entry is `{}`; esp_loader_connect_with_stub()
 * returns UNSUPPORTED_CHIP). Tracked in hyperfi/docs/tech-debt.md TD-001.
 *
 * Safety: esp_loader_change_transmission_rate() returns non-success when
 * slave doesn't ACK. c5_full_flash() falls back to BOOT_BAUD_RATE (115200)
 * automatically — worst case we get a log warning, not a brick. */
#define HIGHER_BAUD_RATE   2000000

/* Embedded C5 firmware (see main/CMakeLists.txt EMBED_FILES) */
extern const uint8_t c5_fw_bin_start[] asm("_binary_c5_fw_bin_start");
extern const uint8_t c5_fw_bin_end[]   asm("_binary_c5_fw_bin_end");

/* ------------- Shared loader config ------------- */
static loader_esp32_config_t s_loader_cfg = {
    .baud_rate         = BOOT_BAUD_RATE,
    .uart_port         = UART_PORT_NUM,
    .uart_rx_pin       = PIN_P4_UART_RX,
    .uart_tx_pin       = PIN_P4_UART_TX,
    .reset_trigger_pin = PIN_P4_C5_EN,
    .gpio0_trigger_pin = PIN_P4_C5_BOOT,
};

/* ------------- Release C5 to normal boot ------------- */
static void c5_release_normal_boot(void)
{
    /* BOOT high = normal SPI boot */
    gpio_set_level(PIN_P4_C5_BOOT, 1);
    /* Pulse EN low to reset */
    gpio_set_level(PIN_P4_C5_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_P4_C5_EN, 1);
    ESP_LOGI(TAG, "C5 released to normal boot");
}

/* Public version: also configures GPIO as outputs (needed when skipping flash).
 * When c5_full_flash() is called, loader_port_esp32_init() configures the pins.
 * When skipping flash, we must configure them manually. */
void c5_release_normal_boot_public(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PIN_P4_C5_BOOT) | (1ULL << PIN_P4_C5_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    /* Set BOOT high FIRST, then pulse EN to reset C5.
     * This ensures C5 reads BOOT=high at reset and boots from SPI flash normally. */
    gpio_set_level(PIN_P4_C5_BOOT, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_P4_C5_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_P4_C5_EN, 1);
    /* Give C5 time to finish bootloader + run network_adapter before SDIO init */
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "C5 released to normal boot (public: BOOT=HIGH, EN pulse)");
}

/* ------------- Smoke test ------------- */
esp_err_t c5_smoke_test(void)
{
    ESP_LOGI(TAG, "=== C5 Smoke Test (no flash) ===");
    ESP_LOGI(TAG, "Wiring check: P4.IO%d→J5.RXD, P4.IO%d←J5.TXD, P4.IO%d→J5.BOOT",
             PIN_P4_UART_TX, PIN_P4_UART_RX, PIN_P4_C5_BOOT);

    esp_loader_error_t err = loader_port_esp32_init(&s_loader_cfg);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "loader_port_esp32_init FAILED (err=%d)", err);
        return ESP_FAIL;
    }

    esp_loader_connect_args_t conn = ESP_LOADER_CONNECT_DEFAULT();
    err = esp_loader_connect(&conn);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "✗ C5 bootloader NO RESPONSE (err=%d)", err);
        ESP_LOGE(TAG, "  Check: jumper wires connected? BOOT/EN/TX/RX correct direction?");
        loader_port_esp32_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ C5 bootloader RESPONDED");
    ESP_LOGI(TAG, "  target    = %d (6 = ESP32-C5 ✓)", esp_loader_get_target());

    c5_release_normal_boot();
    loader_port_esp32_deinit();
    ESP_LOGI(TAG, "=== C5 Smoke Test PASSED ===");
    return ESP_OK;
}

/* ------------- Full flash ------------- */
esp_err_t c5_full_flash(void)
{
    const size_t bin_size = c5_fw_bin_end - c5_fw_bin_start;
    ESP_LOGI(TAG, "=== C5 Full Flash (%u bytes) ===", (unsigned)bin_size);

    esp_loader_error_t err = loader_port_esp32_init(&s_loader_cfg);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "loader_port_esp32_init FAILED (err=%d)", err);
        return ESP_FAIL;
    }

    esp_loader_connect_args_t conn = ESP_LOADER_CONNECT_DEFAULT();
    err = esp_loader_connect(&conn);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "Connect failed (err=%d)", err);
        loader_port_esp32_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connected, target=%d", esp_loader_get_target());

    /* Bump baud rate for faster flash. WT99P4C5-S1 onboard P4↔C5 trace
     * is short and clean (vs 2026-04-22 dupont wires that capped at 115200).
     * 921600 → ~16s for 1.37MB vs ~131s at 115200 (8x). */
    uint32_t target_baud = HIGHER_BAUD_RATE;
    err = esp_loader_change_transmission_rate(target_baud);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGW(TAG, "esp_loader_change_transmission_rate(%u) failed (err=%d), staying at %u baud",
                 (unsigned)target_baud, err, BOOT_BAUD_RATE);
        target_baud = BOOT_BAUD_RATE;
    } else {
        err = loader_port_change_transmission_rate(target_baud);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGW(TAG, "loader_port_change_transmission_rate(%u) failed (err=%d); host UART stays at %u, slave expects %u — this WILL desync",
                     (unsigned)target_baud, err, BOOT_BAUD_RATE, (unsigned)target_baud);
            /* Best-effort: if host can't switch, we have to fall back too,
             * but the slave is already at new rate — expect failure. */
            target_baud = BOOT_BAUD_RATE;
        } else {
            ESP_LOGI(TAG, "✓ baud upgraded to %u", (unsigned)target_baud);
        }
    }

    ESP_LOGI(TAG, "Flashing at %u baud (~%us expected)...",
             (unsigned)target_baud,
             (unsigned)(bin_size * 10u / target_baud));

    err = esp_loader_flash_start(0x0, bin_size, 1024);
    if (err != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "flash_start FAILED (err=%d)", err);
        loader_port_esp32_deinit();
        return ESP_FAIL;
    }

    /* esp-serial-flasher pads the last partial block to 1024 bytes by writing
     * 0xFF into the caller's buffer. Our source (c5_fw_bin_start) is in flash-
     * mapped XIP memory (read-only). We MUST copy each chunk to a RAM buffer
     * before passing to esp_loader_flash_write().
     * (Bug found 2026-04-22 at 97% of full flash → store access fault.)
     */
    uint8_t *ram_buf = heap_caps_malloc(1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ram_buf) {
        ESP_LOGE(TAG, "Failed to alloc 1KB RAM buffer");
        loader_port_esp32_deinit();
        return ESP_FAIL;
    }

    const uint8_t *p = c5_fw_bin_start;
    size_t remaining = bin_size;
    int64_t t0 = esp_log_timestamp();
    while (remaining > 0) {
        size_t chunk = (remaining > 1024) ? 1024 : remaining;
        memcpy(ram_buf, p, chunk);          /* flash XIP → RAM copy */
        /* Pad unused portion with 0xFF ourselves so library doesn't write
         * to flash-mapped memory (belt-and-suspenders). */
        if (chunk < 1024) {
            memset(ram_buf + chunk, 0xFF, 1024 - chunk);
        }
        err = esp_loader_flash_write(ram_buf, chunk);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "flash_write FAILED at offset %u (err=%d)",
                     (unsigned)(bin_size - remaining), err);
            free(ram_buf);
            loader_port_esp32_deinit();
            return ESP_FAIL;
        }
        p += chunk;
        remaining -= chunk;
        if ((bin_size - remaining) % (64 * 1024) == 0) {
            ESP_LOGI(TAG, "  flash progress: %u/%u bytes",
                     (unsigned)(bin_size - remaining), (unsigned)bin_size);
        }
    }
    free(ram_buf);
    esp_loader_flash_finish(true);
    ESP_LOGI(TAG, "Flash complete in %d ms", (int)(esp_log_timestamp() - t0));

    c5_release_normal_boot();
    loader_port_esp32_deinit();
    ESP_LOGI(TAG, "=== C5 Full Flash DONE ===");
    return ESP_OK;
}
