/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Poincaré ball embedding + geodesic distance + Möbius addition
 *
 * Port of hyperfi/poincare/mapping.py to P4 C, per ADR-023 M3.2.
 *
 * Numerics: float32 (P4 hardware single-precision FPU).
 * Reference impl (Python float64) is the gold standard; per-op relative
 * error budget < 1e-4 vs. Python reference output.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

/* Dimension reduction: 53 input subcarriers → 8 D Poincaré embedding.
 * Indices are uniform-sampled across the 53 LLTF subcarriers.
 * MUST match Python fixture generator exactly. */
#define POINCARE_INPUT_DIM   53
#define POINCARE_EMBED_DIM   8

extern const int poincare_select_indices[POINCARE_EMBED_DIM];

/* Numerical clamps (mirror mapping.py:70-71, 73, 88-94) */
#define POINCARE_NORM_SQ_MAX  0.9999f
#define POINCARE_BALL_NORM_MAX 0.9999f
#define POINCARE_EPSILON       1e-10f

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

/**
 * Project an n_dim feature vector into the open Poincaré ball.
 *
 * Algorithm (mapping.py:22-45):
 *   norm = ||features||
 *   direction = features / norm
 *   radius = scale * tanh(norm)
 *   out = radius * direction
 *
 * @param[in]  features  Input vector, length n_dim
 * @param[in]  n_dim     Vector length (typically POINCARE_EMBED_DIM=8)
 * @param[in]  scale     Maximum radius (0 < scale < 1, typically 0.5)
 * @param[out] out       Output point in B^n, ||out|| < scale
 */
void poincare_embed(const float *features, int n_dim, float scale, float *out);

/**
 * Hyperbolic geodesic distance in the Poincaré ball (curvature c = -1).
 *
 * d(u, v) = arccosh(1 + 2 ||u - v||² / ((1 - ||u||²)(1 - ||v||²)))
 *
 * Norms are clamped to POINCARE_NORM_SQ_MAX before division to avoid
 * boundary singularity. arccosh argument is clamped to >= 1.
 *
 * @return Geodesic distance >= 0.0f
 */
float poincare_geodesic_distance(const float *u, const float *v, int n_dim);

/**
 * Möbius addition u ⊕ v in the Poincaré ball (mapping.py:77-95).
 * Used for parallel transport / origin adjustment in M3.3.
 *
 * out = ((1 + 2<u,v> + ||v||²) u + (1 - ||u||²) v)
 *       / (1 + 2<u,v> + ||u||² ||v||²)
 *
 * If ||out|| ≥ 1, out is rescaled to POINCARE_BALL_NORM_MAX × out / ||out||.
 *
 * @param[in]  u, v   Points in Poincaré ball, length n_dim
 * @param[in]  n_dim  Vector length
 * @param[out] out    Result, length n_dim, ||out|| < 1
 */
void poincare_mobius_addition(const float *u, const float *v, int n_dim, float *out);

/**
 * Select 8 features from a 53-element amplitude vector via fixed indices.
 * MUST match the Python fixture generator's dimension reduction.
 *
 * @param[in]  amp_full   Length-53 amplitude vector
 * @param[out] amp_select Length-8 selected vector
 */
void poincare_select_8d(const float *amp_full, float *amp_select);

#ifdef __cplusplus
}
#endif
