/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * proto_codec — wire encoder/decoder for ADR-023 M3.5.1 telemetry messages.
 *
 * Maps the in-firmware pipeline_telemetry_t (pipeline.h) to the on-wire
 * hyperfi.csi.TelemetryReport / AlertReport (telemetry.proto) and back,
 * using nanopb static-allocation codegen.
 *
 * Buffer sizing (worst-case, encoded):
 *   TelemetryReport  ≤ 128 B
 *   AlertReport      ≤ 256 B  (TelemetryReport snapshot included)
 * Use TELEMETRY_BUF_SIZE / ALERT_BUF_SIZE for caller buffers.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pipeline.h"
#include "event_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_TELEMETRY_BUF_SIZE   192
#define PROTO_ALERT_BUF_SIZE       384

/* M3.6 EventRawContext upper bound:
 *   hdr (~120 B) + 12000 × ~135 B encoded frames + 60 × ~80 B windows ≈ 1.65 MB
 * Round up to 2 MB so PSRAM allocation is comfortable.
 * Caller obtains PSRAM via heap_caps_calloc(MALLOC_CAP_SPIRAM). */
#define PROTO_EVENT_BUF_SIZE       (2 * 1024 * 1024)

/**
 * Encode a TelemetryReport from a pipeline snapshot.
 * @return number of bytes written (>0), or -1 on encoder error.
 */
int proto_codec_encode_telemetry(
    const pipeline_telemetry_t *t,
    uint8_t                    *buf,
    size_t                      buf_size);

/**
 * Encode an AlertReport (TelemetryReport snapshot embedded).
 * Caller should ensure t->fall_event_rising_edge is true before calling.
 * @return number of bytes written, or -1 on error.
 */
int proto_codec_encode_alert(
    const pipeline_telemetry_t *t,
    uint8_t                    *buf,
    size_t                      buf_size);

/**
 * Round-trip helper: decode bytes → opaque telemetry summary used by
 * m35_proto_test.c to verify field-by-field fidelity.
 *
 * `out_struct` is a void* pointing to a hyperfi_csi_TelemetryReport
 * (sized by sizeof_telemetry_struct() to keep proto_codec.h independent
 * of nanopb headers).
 */
size_t proto_codec_telemetry_struct_size(void);
bool   proto_codec_decode_telemetry_struct(
    const uint8_t *buf,
    size_t         buf_size,
    void          *out_struct);

/* Field accessors used by the test harness — abstract the nanopb struct
 * layout so tests can compare against pipeline_telemetry_t without
 * pulling telemetry.pb.h into m35_proto_test.h. */
uint64_t proto_codec_get_timestamp_us(const void *s);
float    proto_codec_get_collapse_index_max(const void *s);
float    proto_codec_get_norm_cv(const void *s);
float    proto_codec_get_shape_corr(const void *s);
float    proto_codec_get_dynamic_gain(const void *s);
bool     proto_codec_get_quiet_period(const void *s);
int32_t  proto_codec_get_rssi(const void *s);
uint32_t proto_codec_get_n_frames_in_window(const void *s);
size_t   proto_codec_get_poincare_embed(const void *s, float out8[8]);
const char *proto_codec_get_fsm_state(const void *s);

/* -------------------------------------------------------------------------- */
/* M3.6 — EventRawContext encoder (streaming via nanopb pb_callback_t)         */
/* -------------------------------------------------------------------------- */

typedef struct {
    const char  *device_id;
    uint64_t     event_id;
    float        collapse_index_peak;
    float        confidence;
    const char  *matched_pattern;     /* may be NULL */
    int32_t      best_pattern_idx;
} proto_codec_event_meta_t;

/**
 * Encode an EventRawContext from a snapshot + alert metadata.
 * The repeated frames / windows fields stream from the snapshot via
 * nanopb callbacks — no intermediate buffer.
 *
 * @param[in]  snap   buffer snapshot owned by caller (must outlive encode)
 * @param[in]  meta   alert metadata, all fields used
 * @param[out] buf    output buffer (must be PSRAM, ≥ PROTO_EVENT_BUF_SIZE)
 * @param[in]  buf_size  capacity of buf
 * @return number of bytes written, or -1 on encoder error
 */
int proto_codec_encode_event_raw_context(
    const event_buffer_snapshot_t  *snap,
    const proto_codec_event_meta_t *meta,
    uint8_t                        *buf,
    size_t                          buf_size);

#ifdef __cplusplus
}
#endif
