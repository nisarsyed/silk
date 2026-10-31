#ifndef SILK_CONTACT_H
#define SILK_CONTACT_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

/* Geometry and energy kernels for Offset Geometric Contact (Chen et al.,
 * "Offset Geometric Contact", SIGGRAPH 2025). All functions are pure.
 *
 * A contact acts between a point and the closest point on another
 * primitive. It is written as a stencil of up to 4 vertices with weights
 * that sum to zero, so that the separation vector is
 *
 *     s = sum_k weight_k x_k,   d = |s|,
 *
 * and the force on vertex k is -weight_k g'(d) s / d. Weights are
 * re-derived from the current positions on every evaluation; the set of
 * vertices (the contact feature) only changes at detection. */

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

/* OGC feasible regions (paper Eqs. 8, 9, 15). x is the query point.
 *
 * Vertex block: (x - v) . (v - n) >= 0 for every neighbour n of v.
 * Edge block: x projects strictly inside (e0, e1), and for each opposite
 * vertex o of a face on the edge, (x - p) . (p - o) >= 0 with p the foot
 * of o on the edge line. */
bool sl_vertex_block_contains(sl_vec3 x, sl_vec3 v, const sl_vec3 *neighbours,
                              uint32_t neighbour_count);
bool sl_edge_block_contains(sl_vec3 x, sl_vec3 e0, sl_vec3 e1,
                            const sl_vec3 *opposite, uint32_t opposite_count);

/* Two-stage activation (paper Eq. 18) with tau = r / 2:
 *
 *     g(d) = k / 2 (r - d)^2              for tau <= d < r
 *     g(d) = -k' log(d) + b               for 0 < d < tau
 *
 * with k' = k tau^2 and b = k / 2 (r - tau)^2 + k' log(tau), which makes g
 * C2 at tau (the paper prints k' as tau k (tau - r)^2; C1 and C2 continuity
 * both require k tau (r - tau) = k tau^2). g = 0 for d >= r. k is N/m, r
 * and d meters, g joules. Returns g; *slope = g'(d) <= 0 and
 * *curvature = g''(d) >= 0. d must be > 0. */
float sl_contact_energy(float d, float r, float k, float *slope,
                        float *curvature);

#endif /* SILK_CONTACT_H */
