/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * load_test — see load_test.h.
 */

#include "load_test.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char *TAG = "load_test";

const int load_test_default_steps[6] = { 0, 10, 100, 500, 1000, 0 };

typedef struct {
    load_test_config_t cfg;
    /* The kbps_steps pointer in cfg points to caller's storage; copy
     * the values we need so the task is self-contained even if the
     * caller's stack frame goes away. */
    int kbps_storage[16];
    int n_steps;
} state_t;

/* -------------------------------------------------------------------------- */
/* One step                                                                    */
/* -------------------------------------------------------------------------- */
static void run_step(int step_idx, int total, int target_kbps,
                     int sock, const struct sockaddr_in *dest,
                     const uint8_t *buf, int packet_size, int duration_sec)
{
    ESP_LOGI(TAG, "─── step %d/%d ───  target=%d kbps  pkt=%dB  dur=%ds",
             step_idx + 1, total, target_kbps, packet_size, duration_sec);

    if (target_kbps == 0) {
        ESP_LOGI(TAG, "    idle baseline (no extra traffic from this task)");
        vTaskDelay(pdMS_TO_TICKS(duration_sec * 1000));
        return;
    }

    /* Microseconds between sends to hit target_kbps with packet_size B/pkt. */
    const int64_t dt_us = (int64_t)packet_size * 8 * 1000LL / target_kbps;

    const int64_t step_start_us = esp_timer_get_time();
    const int64_t step_end_us   = step_start_us + (int64_t)duration_sec * 1000000LL;

    uint32_t pkts_sent = 0;
    uint32_t bytes_sent = 0;
    uint32_t fail_count = 0;
    int64_t  next_send_us = step_start_us;

    while (esp_timer_get_time() < step_end_us) {
        int n = sendto(sock, buf, packet_size, 0,
                       (const struct sockaddr *)dest, sizeof(*dest));
        if (n < 0) {
            fail_count++;
        } else {
            pkts_sent++;
            bytes_sent += (uint32_t)n;
        }

        next_send_us += dt_us;
        const int64_t now_us = esp_timer_get_time();
        if (next_send_us > now_us) {
            const int64_t sleep_us = next_send_us - now_us;
            if (sleep_us >= 1000) {
                vTaskDelay(pdMS_TO_TICKS(sleep_us / 1000));
            }
            /* sub-ms residue is left to natural call overhead */
        } else if ((pkts_sent & 0x3F) == 0) {
            /* Rate too high to keep up — yield occasionally so the
             * scheduler doesn't starve other tasks. */
            vTaskDelay(1);
        }
    }

    const int64_t actual_us = esp_timer_get_time() - step_start_us;
    const float actual_kbps =
        (float)bytes_sent * 8.0f / ((float)actual_us / 1000.0f);

    ESP_LOGI(TAG,
             "    step %d done: pkts=%lu bytes=%lu actual=%.1fkbps fail=%lu",
             step_idx + 1,
             (unsigned long)pkts_sent,
             (unsigned long)bytes_sent,
             (double)actual_kbps,
             (unsigned long)fail_count);
}

/* -------------------------------------------------------------------------- */
/* Task                                                                        */
/* -------------------------------------------------------------------------- */
static void load_test_task(void *arg)
{
    state_t *s = (state_t *)arg;

    /* Warm-up: let CSI ingest reach steady state before pressuring radio. */
    if (s->cfg.warmup_sec > 0) {
        ESP_LOGI(TAG, "warm-up: sleeping %ds before first step",
                 s->cfg.warmup_sec);
        vTaskDelay(pdMS_TO_TICKS(s->cfg.warmup_sec * 1000));
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        free(s);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(s->cfg.target_port);
    inet_aton(s->cfg.target_ip, &dest.sin_addr);

    uint8_t *buf = (uint8_t *)malloc((size_t)s->cfg.packet_size);
    if (!buf) {
        ESP_LOGE(TAG, "malloc(%d) failed", s->cfg.packet_size);
        close(sock); free(s); vTaskDelete(NULL); return;
    }
    memset(buf, 0xA5, (size_t)s->cfg.packet_size);

    ESP_LOGI(TAG,
             "═══ load test start: target=%s:%d size=%dB steps=%d ═══",
             s->cfg.target_ip, s->cfg.target_port,
             s->cfg.packet_size, s->n_steps);

    for (int i = 0; i < s->n_steps; i++) {
        run_step(i, s->n_steps, s->kbps_storage[i],
                 sock, &dest, buf, s->cfg.packet_size,
                 s->cfg.step_duration_sec);
    }

    ESP_LOGI(TAG, "═══ load test complete — eyeball [CSI] stats lines for fps ═══");

    free(buf);
    close(sock);
    free(s);
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Public entry                                                                */
/* -------------------------------------------------------------------------- */
esp_err_t load_test_start(const load_test_config_t *cfg)
{
    if (!cfg || !cfg->target_kbps_steps || cfg->n_steps <= 0
        || cfg->n_steps > 16 || cfg->packet_size <= 0
        || cfg->packet_size > 1400) {
        return ESP_ERR_INVALID_ARG;
    }
    state_t *s = (state_t *)calloc(1, sizeof(*s));
    if (!s) return ESP_ERR_NO_MEM;
    s->cfg     = *cfg;
    s->n_steps = cfg->n_steps;
    for (int i = 0; i < s->n_steps; i++) {
        s->kbps_storage[i] = cfg->target_kbps_steps[i];
    }
    /* Stack 4 KB is enough — only sendto + small buf. */
    BaseType_t r = xTaskCreate(load_test_task, "load_test", 4096, s, 5, NULL);
    if (r != pdPASS) {
        free(s);
        return ESP_FAIL;
    }
    return ESP_OK;
}
