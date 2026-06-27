/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * hf_console — on-site commissioning REPL (M4/PoC §3.1). See hf_console.c.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Start the commissioning console REPL on the console UART. Call once, late in
 *  app_main (after hf_config_load + pipeline init). No-op on failure. */
void hf_console_init(void);

#ifdef __cplusplus
}
#endif
