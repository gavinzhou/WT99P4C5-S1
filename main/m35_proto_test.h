/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * M3.5.1 self-test (TelemetryReport encode/decode roundtrip), ADR-023.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Builds a synthetic pipeline_telemetry_t with known values, encodes it
 * via proto_codec_encode_telemetry, decodes back into a nanopb struct,
 * and verifies field-by-field fidelity within float ε.
 *
 * Returns ESP_OK on success, ESP_FAIL on any mismatch.
 */
esp_err_t m35_run_proto_self_test(void);

#ifdef __cplusplus
}
#endif
