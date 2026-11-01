#ifndef SILK_CONTACT_H
#define SILK_CONTACT_H

#include <stdbool.h>
#include <stdint.h>

#include "geometry.h"
#include "silk/math.h"

/* Kernels for Offset Geometric Contact (Chen et al., "Offset Geometric
 * Contact", SIGGRAPH 2025): the offset-block feasibility tests and the
 * contact energy. All functions are pure; closest points are in
 * geometry.h.
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
