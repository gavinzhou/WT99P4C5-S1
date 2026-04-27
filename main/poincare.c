/*
 * SPDX-FileCopyrightText: 2026 PacPort Inc.
 * SPDX-License-Identifier: CC0-1.0
 *
 * Poincaré ball implementation — see poincare.h for API.
 *
 * Direct port of hyperfi/poincare/mapping.py per ADR-023 M3.2.
 * All operations are float32 (P4 hardware FPU).
 */

#include "poincare.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Subcarrier selection indices (uniform sampling 53 → 8)                      */
/* -------------------------------------------------------------------------- */

const int poincare_select_indices[POINCARE_EMBED_DIM] = {
    3, 9, 16, 22, 29, 35, 42, 48
};

/* -------------------------------------------------------------------------- */
/* Embed: features → Poincaré ball                                             */
/* -------------------------------------------------------------------------- */

void poincare_embed(const float *features, int n_dim, float scale, float *out)
{
    /* norm = ||features|| (with epsilon to avoid divide-by-zero, mapping.py:40) */
    float sum_sq = 0.0f;
    for (int i = 0; i < n_dim; i++) {
        sum_sq += features[i] * features[i];
    }
    const float norm = sqrtf(sum_sq) + POINCARE_EPSILON;

    /* radius = scale * tanh(norm) */
    const float radius = scale * tanhf(norm);

    /* out = (radius / norm) * features */
    const float k = radius / norm;
    for (int i = 0; i < n_dim; i++) {
        out[i] = k * features[i];
    }
}

/* -------------------------------------------------------------------------- */
/* Geodesic distance                                                           */
/* -------------------------------------------------------------------------- */

float poincare_geodesic_distance(const float *u, const float *v, int n_dim)
{
    float norm_u_sq = 0.0f;
    float norm_v_sq = 0.0f;
    float diff_sq   = 0.0f;
    for (int i = 0; i < n_dim; i++) {
        norm_u_sq += u[i] * u[i];
        norm_v_sq += v[i] * v[i];
        const float d = u[i] - v[i];
        diff_sq += d * d;
    }

    /* Clamp to avoid boundary singularity (mapping.py:70-71) */
    if (norm_u_sq > POINCARE_NORM_SQ_MAX) norm_u_sq = POINCARE_NORM_SQ_MAX;
    if (norm_v_sq > POINCARE_NORM_SQ_MAX) norm_v_sq = POINCARE_NORM_SQ_MAX;

    /* arg = 1 + 2 ||u-v||² / ((1 - ||u||²)(1 - ||v||²) + ε) */
    const float denom = (1.0f - norm_u_sq) * (1.0f - norm_v_sq) + POINCARE_EPSILON;
    float arg = 1.0f + 2.0f * diff_sq / denom;

    /* arccosh requires arg >= 1 (mapping.py:74) */
    if (arg < 1.0f) arg = 1.0f;

    return acoshf(arg);
}

/* -------------------------------------------------------------------------- */
/* Möbius addition u ⊕ v                                                       */
/* -------------------------------------------------------------------------- */

void poincare_mobius_addition(const float *u, const float *v, int n_dim, float *out)
{
    float norm_u_sq = 0.0f;
    float norm_v_sq = 0.0f;
    float uv = 0.0f;   /* <u, v> */
    for (int i = 0; i < n_dim; i++) {
        norm_u_sq += u[i] * u[i];
        norm_v_sq += v[i] * v[i];
        uv        += u[i] * v[i];
    }

    /* num_u_coef = (1 + 2<u,v> + ||v||²); num_v_coef = (1 - ||u||²) */
    const float num_u_coef = 1.0f + 2.0f * uv + norm_v_sq;
    const float num_v_coef = 1.0f - norm_u_sq;
    const float den = 1.0f + 2.0f * uv + norm_u_sq * norm_v_sq + POINCARE_EPSILON;

    float result_norm_sq = 0.0f;
    for (int i = 0; i < n_dim; i++) {
        const float t = (num_u_coef * u[i] + num_v_coef * v[i]) / den;
        out[i] = t;
        result_norm_sq += t * t;
    }

    /* Clamp to stay inside ball (mapping.py:91-94) */
    if (result_norm_sq >= 1.0f) {
        const float result_norm = sqrtf(result_norm_sq);
        const float k = POINCARE_BALL_NORM_MAX / result_norm;
        for (int i = 0; i < n_dim; i++) {
            out[i] *= k;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Subcarrier selection 53 → 8                                                 */
/* -------------------------------------------------------------------------- */

void poincare_select_8d(const float *amp_full, float *amp_select)
{
    for (int i = 0; i < POINCARE_EMBED_DIM; i++) {
        amp_select[i] = amp_full[poincare_select_indices[i]];
    }
}
