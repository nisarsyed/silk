#ifndef SILK_CONTACT_H
#define SILK_CONTACT_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Collision tolerance in world length units. This is intentionally
 * independent of SL_EPSILON, which is only a floating-point tolerance. */
#define SL_LINEAR_SLOP 0.005f
#define SL_SPECULATIVE_DISTANCE (4.0f * SL_LINEAR_SLOP)
#define SL_MANIFOLD_POINT_COUNT_MAX 2u

/* One contact between two shape surfaces. Anchors are world-oriented
 * offsets from their body origins; point is their midpoint for debug draw.
 * separation is signed along the manifold normal (negative overlaps).
 * id identifies the feature on both shapes and remains stable while those
 * features remain paired. Solver-owned fields start at zero. */
typedef struct sl_manifold_point {
    sl_vec2 anchor_a;
    sl_vec2 anchor_b;
    sl_vec2 point;
    float separation;
    float normal_impulse;
    float tangent_impulse;
    float normal_velocity;
    uint32_t id;
    bool persisted;
} sl_manifold_point;

/* World-space unit normal points from shape A toward shape B. */
typedef struct sl_manifold {
    sl_vec2 normal;
    uint32_t point_count;
    sl_manifold_point points[SL_MANIFOLD_POINT_COUNT_MAX];
} sl_manifold;

#ifdef __cplusplus
}
#endif

#endif /* SILK_CONTACT_H */
