/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * hf_console — on-site commissioning REPL over the console UART (M4/PoC §3.1).
 *
 * The engineer types over the USB-UART (CP2102N) console:
 *     hf                       show current config
 *     hf show                  same
 *     hf set <key> <value>     set a parameter (not yet persisted)
 *     hf save                  persist current config to NVS
 *   keys: room margin collapse_thr silence_thr breathing_min
 *
 * Coexists with the normal ESP_LOG output on the same UART.
 */
#include "hf_console.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_console.h"
#include "hf_config.h"

static const char *TAG = "hf_console";

static void usage(void)
{
    printf("usage: hf [show | set <key> <value> | save]\n"
           "  keys: room margin collapse_thr silence_thr breathing_min\n");
}

static int cmd_hf(int argc, char **argv)
{
    if (argc < 2 || !strcmp(argv[1], "show")) {
        hf_config_print();
        return 0;
    }
    if (!strcmp(argv[1], "save")) {
        return (hf_config_save() == ESP_OK) ? 0 : 1;
    }
    if (!strcmp(argv[1], "set")) {
        if (argc < 4) { usage(); return 1; }
        char *end = NULL;
        float v = strtof(argv[3], &end);
        if (end == argv[3]) { printf("bad value '%s'\n", argv[3]); return 1; }
        if (hf_config_set(argv[2], v) != ESP_OK) {
            printf("unknown key '%s'\n", argv[2]);
            usage();
            return 1;
        }
        printf("set %s=%.4f (run 'hf save' to persist)\n", argv[2], v);
        return 0;
    }
    usage();
    return 1;
}

void hf_console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "hf>";
    repl_cfg.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "console REPL init failed (%s) — commissioning disabled",
                 esp_err_to_name(err));
        return;
    }

    const esp_console_cmd_t cmd = {
        .command = "hf",
        .help    = "Commissioning config: hf [show|set <key> <val>|save]",
        .func    = &cmd_hf,
    };
    esp_console_cmd_register(&cmd);
    esp_console_register_help_command();
    esp_console_start_repl(repl);
    ESP_LOGI(TAG, "commissioning console ready — type 'hf' or 'help'");
}
