#ifndef SILK_GEOMETRY_H
#define SILK_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

/* Closest-point geometry shared by the modules: cloth self contact
 * (Offset Geometric Contact) and rigid-body collision. All functions are
 * pure. */

/* Closest feature of a triangle to a point. */
typedef enum sl_feature_kind {
    SL_FEATURE_VERTEX = 0,
    SL_FEATURE_EDGE = 1,
    SL_FEATURE_FACE = 2
} sl_feature_kind;

typedef struct sl_triangle_closest {
    sl_feature_kind kind;
    /* Local corner indices of the feature: VERTEX uses a, EDGE uses (a, b)
     * with a < b, FACE uses none. */
    uint32_t a;
    uint32_t b;
    float bary[3]; /* closest point = sum bary_k t_k */
    float distance;
} sl_triangle_closest;

/* Closest point of triangle (t0, t1, t2) to p, with the feature it lies
 * in (Ericson, Real-Time Collision Detection, 5.1.5). Degenerate
 * triangles fall back to their closest edge. */
sl_triangle_closest sl_triangle_closest_point(sl_vec3 p, sl_vec3 t0, sl_vec3 t1,
                                              sl_vec3 t2);

/* Parameter in [0, 1] of the point on segment (a, b) closest to p. */
float sl_segment_closest_param(sl_vec3 p, sl_vec3 a, sl_vec3 b);

/* Closest points of segments (p0, p1) and (q0, q1): parameters s, t in
 * [0, 1] (Ericson 5.1.9). Parallel segments pick a deterministic pair. */
void sl_segment_closest_params(sl_vec3 p0, sl_vec3 p1, sl_vec3 q0, sl_vec3 q1,
                               float *s, float *t);

#endif /* SILK_GEOMETRY_H */
