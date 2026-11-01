#include <math.h>
#include <string.h>

#include "contact.h"
#include "world_internal.h"

/* Offset Geometric Contact (Chen et al., SIGGRAPH 2025) for cloth-cloth and
 * self contact: detection with the paper's Algorithms 1 and 2 over a
 * spatial hash, conservative displacement bounds that make every solver
 * move intersection-free without CCD, and contact pairs evaluated as vertex
 * stencils (contact.h).
 *
 * Bounds extend the paper's Eq. 21 to be translation-relative per cloth.
 * The paper bounds each vertex's absolute displacement by
 * b_v = gamma_p * d_min (Wu et al. 2020: if every vertex moves less than
 * half its nearest non-incident distance, the straight path between the
 * states is intersection-free). That penalises rigid motion, the paper's
 * stated limitation (Fig. 22b): inside one sheet non-adjacent primitives
 * sit about half an edge apart, so a falling sheet exhausted its bounds
 * every few substeps and, at one sweep per substep, lost its velocity each
 * time (measured: 0.4 m/s instead of 9.8 m/s after 1 s of free fall).
 *
 * Distances within a cloth do not change under a common translation T_c of
 * that cloth. silk therefore constrains x_v = x_safe + T_c + delta_v with
 * |delta_v| <= b_v, b_v = gamma_p * (nearest same-cloth distance): every
 * same-cloth pair's relative displacement is delta_v - delta_w, at most
 * 2 b_v < d, exactly the paper's condition. Pairs across cloths see the
 * absolute displacement |T_c| + |delta_v|, so each cloth also gets
 * B_c = gamma_p * (lower bound on its distance to every other cloth), and
 * T_c is capped at B_c - max_v b'_v with b'_v = min(b_v, B_c), keeping
 * every absolute displacement within B_c. With one cloth B_c is unbounded.
 * T_c is the cloth's mean displacement, clamped to its cap. */

/* Relaxation gamma_p of the bound b_v = gamma_p * d_min (paper Eq. 21,
 * 0 < gamma_p < 0.5; the paper uses 0.45 to stay conservative under float
 * rounding). */
#define SL_BOUND_RELAXATION 0.45f

/* Cap on the motion part of the query radius, in triangle extents. The
 * query radius sets the candidate count of every detection, while bounds
 * mostly come from nearer primitives anyway. Three 30 x 30 sheets dropped
 * in a pile ran at 82 ms/step with a cap of 0.25, 61 ms with 0.1, 50 ms
 * with 0.05 and 51 ms with 0 (14.5 ms without self contact); a draped
 * 40 x 40 sheet stayed near 12 ms. All runs stayed intersection-free. */
#define SL_QUERY_MOTION_EXTENT 0.05f

/* Stand-in for "no other cloth": larger than any distance in the domain. */
#define SL_SELF_UNBOUNDED (8.0f * SL_POSITION_ABS_MAX)

static float sl_thickness_of(const sl_world_state *s, uint32_t v)
{
    return s->cloths[s->vertex_cloth[v]].thickness;
}

static float sl_friction_of(const sl_world_state *s, uint32_t v)
{
    return s->cloths[s->vertex_cloth[v]].friction;
}

/* ---------------------------------------------------------------------- */
/* BVH maintenance                                                         */

static sl_vec3 sl_vec3_min3(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_make(sl_min(a.x, b.x), sl_min(a.y, b.y), sl_min(a.z, b.z));
}

static sl_vec3 sl_vec3_max3(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_make(sl_max(a.x, b.x), sl_max(a.y, b.y), sl_max(a.z, b.z));
}

/* Element e's box overlaps [lo, hi]. */
static bool sl_element_overlaps(const sl_bvh *bvh, uint32_t e, sl_vec3 lo,
                                sl_vec3 hi)
{
    const sl_vec3 el = bvh->elem_lo[e];
    const sl_vec3 eh = bvh->elem_hi[e];
    return el.x <= hi.x && lo.x <= eh.x && el.y <= hi.y && lo.y <= eh.y &&
           el.z <= hi.z && lo.z <= eh.z;
}

void sl_self_build(sl_world_state *s)
{
    SL_ASSERT(sl_self_enabled(s));
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        s->triangle_bvh.centroid[t] = sl_vec3_scale(
            sl_vec3_add(sl_vec3_add(s->position[idx[0]], s->position[idx[1]]),
                        s->position[idx[2]]),
            1.0f / 3.0f);
    }
    sl_bvh_build(&s->triangle_bvh, s->triangle_count);
    for (uint32_t e = 0; e < s->edge_count; e++) {
        s->edge_bvh.centroid[e] =
            sl_vec3_scale(sl_vec3_add(s->position[s->edge_index[2u * e]],
                                      s->position[s->edge_index[2u * e + 1u]]),
                          0.5f);
    }
    sl_bvh_build(&s->edge_bvh, s->edge_count);
}

/* Refits both trees to the current positions (element boxes unexpanded;
 * queries expand themselves). */
static void sl_self_refit(sl_world_state *s)
{
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        const sl_vec3 a = s->position[idx[0]];
        const sl_vec3 b = s->position[idx[1]];
        const sl_vec3 c = s->position[idx[2]];
        s->triangle_bvh.elem_lo[t] = sl_vec3_min3(sl_vec3_min3(a, b), c);
        s->triangle_bvh.elem_hi[t] = sl_vec3_max3(sl_vec3_max3(a, b), c);
        const sl_vec3 center =
            sl_vec3_scale(sl_vec3_add(sl_vec3_add(a, b), c), 1.0f / 3.0f);
        s->triangle_bvh.sphere_center[t] = center;
        s->triangle_bvh.sphere_radius[t] =
            sqrtf(sl_max(sl_vec3_length_sq(sl_vec3_sub(a, center)),
                         sl_max(sl_vec3_length_sq(sl_vec3_sub(b, center)),
                                sl_vec3_length_sq(sl_vec3_sub(c, center)))));
    }
    sl_bvh_refit(&s->triangle_bvh);
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const sl_vec3 a = s->position[s->edge_index[2u * e]];
        const sl_vec3 b = s->position[s->edge_index[2u * e + 1u]];
        s->edge_bvh.elem_lo[e] = sl_vec3_min3(a, b);
        s->edge_bvh.elem_hi[e] = sl_vec3_max3(a, b);
        s->edge_bvh.sphere_center[e] = sl_vec3_scale(sl_vec3_add(a, b), 0.5f);
        s->edge_bvh.sphere_radius[e] = 0.5f * sl_vec3_length(sl_vec3_sub(b, a));
    }
    sl_bvh_refit(&s->edge_bvh);
}

/* ---------------------------------------------------------------------- */
/* OGC feasible regions over the mesh adjacency                            */

/* Vertex block (Eq. 8; Eq. 15 for the edge-only manifold, which has the
 * same neighbours here): x must not lie behind any plane through w normal
 * to an incident edge. */
static bool sl_vertex_feasible(const sl_world_state *s, sl_vec3 x, uint32_t w)
{
    const sl_vec3 xw = s->position[w];
    const sl_vec3 d = sl_vec3_sub(x, xw);
    for (uint32_t r = s->vertex_triangle_offset[w];
         r < s->vertex_triangle_offset[w + 1u]; r++) {
        const uint32_t *idx =
            s->triangle_index + 3u * (s->vertex_triangle[r] / 3u);
        for (uint32_t k = 0; k < 3u; k++) {
            if (idx[k] != w &&
                sl_vec3_dot(d, sl_vec3_sub(xw, s->position[idx[k]])) < 0.0f) {
                return false;
            }
        }
    }
    return true;
}

/* Edge block (Eq. 9): x projects inside (a, b) and lies on the outer side
 * of the plane normal to each adjacent face through the edge. */
static bool sl_edge_feasible(const sl_world_state *s, sl_vec3 x, uint32_t a,
                             uint32_t b)
{
    sl_vec3 opposite[2];
    uint32_t count = 0;
    for (uint32_t r = s->vertex_triangle_offset[a];
         r < s->vertex_triangle_offset[a + 1u] && count < 2u; r++) {
        const uint32_t *idx =
            s->triangle_index + 3u * (s->vertex_triangle[r] / 3u);
        if (idx[0] != b && idx[1] != b && idx[2] != b) {
            continue;
        }
        for (uint32_t k = 0; k < 3u; k++) {
            if (idx[k] != a && idx[k] != b) {
                opposite[count++] = s->position[idx[k]];
            }
        }
    }
    return sl_edge_block_contains(x, s->position[a], s->position[b], opposite,
                                  count);
}

/* ---------------------------------------------------------------------- */
/* Pair evaluation                                                         */

/* Separation s = sum_k w_k x_k of a pair at the current positions, with
 * its weights. Returns false for a degenerate configuration. */
static bool sl_pair_separation(const sl_world_state *s,
                               const sl_contact_pair *p, float w[4],
                               sl_vec3 *sep)
{
    const sl_vec3 *x = s->position;
    const uint32_t *v = p->vertex;
    switch ((sl_pair_kind)p->kind) {
    case SL_PAIR_VERTEX_FACE: {
        /* Projection onto the face plane; barycentrics are not clamped, so
         * the distance stays smooth while the feature is held fixed. */
        const sl_vec3 e0 = sl_vec3_sub(x[v[2]], x[v[1]]);
        const sl_vec3 e1 = sl_vec3_sub(x[v[3]], x[v[1]]);
        const sl_vec3 q = sl_vec3_sub(x[v[0]], x[v[1]]);
        const float d00 = sl_vec3_dot(e0, e0);
        const float d01 = sl_vec3_dot(e0, e1);
        const float d11 = sl_vec3_dot(e1, e1);
        const float d20 = sl_vec3_dot(q, e0);
        const float d21 = sl_vec3_dot(q, e1);
        const float denom = d00 * d11 - d01 * d01;
        if (!(denom > 1e-12f * d00 * d11)) {
            return false;
        }
        const float b1 = (d11 * d20 - d01 * d21) / denom;
        const float b2 = (d00 * d21 - d01 * d20) / denom;
        w[0] = 1.0f;
        w[1] = -(1.0f - b1 - b2);
        w[2] = -b1;
        w[3] = -b2;
        break;
    }
    case SL_PAIR_VERTEX_EDGE: {
        const sl_vec3 ab = sl_vec3_sub(x[v[2]], x[v[1]]);
        const float length_sq = sl_vec3_length_sq(ab);
        if (!(length_sq > 0.0f)) {
            return false;
        }
        const float t =
            sl_vec3_dot(sl_vec3_sub(x[v[0]], x[v[1]]), ab) / length_sq;
        w[0] = 1.0f;
        w[1] = -(1.0f - t);
        w[2] = -t;
        w[3] = 0.0f;
        break;
    }
    case SL_PAIR_VERTEX_VERTEX:
        w[0] = 1.0f;
        w[1] = -1.0f;
        w[2] = 0.0f;
        w[3] = 0.0f;
        break;
    case SL_PAIR_EDGE_EDGE: {
        float a;
        float b;
        sl_segment_closest_params(x[v[0]], x[v[1]], x[v[2]], x[v[3]], &a, &b);
        w[0] = 1.0f - a;
        w[1] = a;
        w[2] = -(1.0f - b);
        w[3] = -b;
        break;
    }
    case SL_PAIR_EDGE_VERTEX: {
        const sl_vec3 ab = sl_vec3_sub(x[v[1]], x[v[0]]);
        const float length_sq = sl_vec3_length_sq(ab);
        if (!(length_sq > 0.0f)) {
            return false;
        }
        const float t =
            sl_vec3_dot(sl_vec3_sub(x[v[2]], x[v[0]]), ab) / length_sq;
        w[0] = 1.0f - t;
        w[1] = t;
        w[2] = -1.0f;
        w[3] = 0.0f;
        break;
    }
    default:
        return false;
    }
    sl_vec3 sum = sl_vec3_zero();
    for (uint32_t k = 1; k < p->count; k++) {
        /* Relative to vertex 0; the weights sum to zero. */
        sum = sl_vec3_mul_add(sum, sl_vec3_sub(x[v[k]], x[v[0]]), w[k]);
    }
    *sep = sum;
    return sl_vec3_is_finite(sum);
}

bool sl_self_pair_points(const sl_world_state *s, uint32_t k, sl_vec3 *point_a,
                         sl_vec3 *point_b)
{
    SL_ASSERT(k < s->pair_count);
    const sl_contact_pair *p = &s->pairs[k];
    float w[4];
    sl_vec3 sep;
    if (!sl_pair_separation(s, p, w, &sep)) {
        return false;
    }
    /* s = a - b: a is vertex 0 for vertex pairs, and the point
     * w0 x0 + w1 x1 (w0 + w1 = 1) on the first edge for edge pairs. */
    const sl_vec3 *x = s->position;
    sl_vec3 a = x[p->vertex[0]];
    if (p->kind == SL_PAIR_EDGE_EDGE || p->kind == SL_PAIR_EDGE_VERTEX) {
        a = sl_vec3_mul_add(a, sl_vec3_sub(x[p->vertex[1]], a), w[1]);
    }
    *point_a = a;
    *point_b = sl_vec3_sub(a, sep);
    return true;
}

/* Distances below this fraction of the radius are clamped: the barrier is
 * then very stiff but finite. */
#define SL_CONTACT_DISTANCE_MIN_FRACTION 1.0e-4f

void sl_self_vertex(const sl_world_state *s, uint32_t i, sl_vec3 *force,
                    sl_mat3 *hessian)
{
    for (uint32_t r = s->pair_ref_offset[i]; r < s->pair_ref_offset[i + 1u];
         r++) {
        const sl_contact_pair *p = &s->pairs[s->pair_ref[r] / 4u];
        const uint32_t slot = s->pair_ref[r] % 4u;
        float w[4];
        sl_vec3 sep;
        if (!sl_pair_separation(s, p, w, &sep)) {
            continue;
        }
        const float length = sl_vec3_length(sep);
        if (!(length < p->radius) || !(length > 0.0f)) {
            continue;
        }
        const float d =
            sl_max(length, SL_CONTACT_DISTANCE_MIN_FRACTION * p->radius);
        float slope;
        float curvature;
        (void)sl_contact_energy(d, p->radius, s->self_stiffness, &slope,
                                &curvature);
        const float share = sl_pair_share(p);
        const sl_vec3 n = sl_vec3_scale(sep, 1.0f / length);
        *force = sl_vec3_mul_add(*force, n, -share * w[slot] * slope);
        *hessian = sl_mat3_mul_add(*hessian, sl_mat3_outer(n, n),
                                   share * w[slot] * w[slot] * curvature);
    }
}

double sl_self_energy(const sl_world_state *s)
{
    double energy = 0.0;
    for (uint32_t k = 0; k < s->pair_count; k++) {
        const sl_contact_pair *p = &s->pairs[k];
        float w[4];
        sl_vec3 sep;
        if (!sl_pair_separation(s, p, w, &sep)) {
            continue;
        }
        const float length = sl_vec3_length(sep);
        if (!(length < p->radius) || !(length > 0.0f)) {
            continue;
        }
        float slope;
        float curvature;
        const float d =
            sl_max(length, SL_CONTACT_DISTANCE_MIN_FRACTION * p->radius);
        const float g = sl_contact_energy(d, p->radius, s->self_stiffness,
                                          &slope, &curvature);

        energy += (double)g * (double)sl_pair_share(p);
    }
    return energy;
}

/* ---------------------------------------------------------------------- */
/* Detection                                                               */

/* Appends a pair unless an identical one exists among the owner's pairs
 * [first, pair_count). With SL_VERTEX_CONTACT_COUNT_MAX pairs already, the
 * farthest is replaced if the new one is closer. */
static void sl_pair_add(sl_world_state *s, uint32_t first,
                        const sl_contact_pair *pair, float distance,
                        float *distances)
{
    const uint32_t owned = s->pair_count - first;
    for (uint32_t k = first; k < s->pair_count; k++) {
        const sl_contact_pair *q = &s->pairs[k];
        if (q->kind == pair->kind &&
            memcmp(q->vertex, pair->vertex, sizeof(q->vertex)) == 0) {
            return;
        }
    }
    if (owned >= SL_VERTEX_CONTACT_COUNT_MAX) {
        uint32_t farthest = 0;
        for (uint32_t k = 1; k < owned; k++) {
            if (distances[k] > distances[farthest]) {
                farthest = k;
            }
        }
        s->self_dropped++;
        if (distance < distances[farthest]) {
            s->pairs[first + farthest] = *pair;
            distances[farthest] = distance;
        }
        return;
    }
    if (s->pair_count >= s->pair_capacity) {
        s->self_dropped++;
        return;
    }
    distances[owned] = distance;
    s->pairs[s->pair_count++] = *pair;
}

static sl_contact_pair sl_pair_make(sl_pair_kind kind, uint32_t count,
                                    const uint32_t *vertex, float radius,
                                    float friction)
{
    sl_contact_pair p;
    memset(&p, 0, sizeof(p));
    p.kind = (uint8_t)kind;
    p.count = (uint8_t)count;

    for (uint32_t k = 0; k < count; k++) {
        p.vertex[k] = vertex[k];
    }
    p.radius = radius;
    p.friction = friction;
    return p;
}

/* Paper Algorithm 1 for vertex v: a box query of half-width r_q on the
 * triangle tree. */
static void sl_detect_vertex(sl_world_state *s, uint32_t v, float query)
{
    const sl_bvh *bvh = &s->triangle_bvh;
    const sl_vec3 x = s->position[v];
    const sl_vec3 grow = sl_vec3_make(query, query, query);
    const uint32_t first = s->pair_count;
    float distances[SL_VERTEX_CONTACT_COUNT_MAX];
    float dmin = query;
    const float thickness = sl_thickness_of(s, v);
    const float friction = sl_friction_of(s, v);
    const sl_vec3 box_lo = sl_vec3_sub(x, grow);
    const sl_vec3 box_hi = sl_vec3_add(x, grow);
    sl_bvh_cursor cursor;
    sl_bvh_cursor_init(&cursor, bvh, box_lo, box_hi);
    uint32_t leaf_first;
    uint32_t leaf_count;
    while (sl_bvh_next_leaf(&cursor, bvh, &leaf_first, &leaf_count)) {
        for (uint32_t k = leaf_first; k < leaf_first + leaf_count; k++) {
            const uint32_t t = bvh->perm[k];
            if (!sl_element_overlaps(bvh, t, box_lo, box_hi)) {
                continue;
            }
            const float reach = bvh->sphere_radius[t] + query;
            if (sl_vec3_length_sq(sl_vec3_sub(x, bvh->sphere_center[t])) >=
                reach * reach) {
                continue;
            }
            const uint32_t *idx = s->triangle_index + 3u * t;
            if (idx[0] == v || idx[1] == v || idx[2] == v) {
                continue;
            }
            const sl_triangle_closest c = sl_triangle_closest_point(
                x, s->position[idx[0]], s->position[idx[1]],
                s->position[idx[2]]);
            if (!(c.distance < query)) {
                continue;
            }
            const uint32_t other_cloth = s->vertex_cloth[idx[0]];
            if (other_cloth == s->vertex_cloth[v]) {
                dmin = sl_min(dmin, c.distance);
                s->triangle_dmin[t] = sl_min(s->triangle_dmin[t], c.distance);
            } else {
                sl_cloth_record *a = &s->cloths[s->vertex_cloth[v]];
                sl_cloth_record *b = &s->cloths[other_cloth];
                a->inter_dmin = sl_min(a->inter_dmin, c.distance);
                b->inter_dmin = sl_min(b->inter_dmin, c.distance);
            }
            /* Pairs are kept inside the contact radius only (F_OGC, Eq. 12).
             * A primitive approaching between detections is held by its bound,
             * still intersection-free, until the next detection adds it. */
            const float radius = thickness + sl_thickness_of(s, idx[0]);
            if (!(c.distance < radius) || !(c.distance > 0.0f)) {
                continue;
            }
            const float mu = sqrtf(friction * sl_friction_of(s, idx[0]));
            if (c.kind == SL_FEATURE_VERTEX) {
                const uint32_t w = idx[c.a];
                if (!sl_vertex_feasible(s, x, w)) {
                    continue;
                }
                const uint32_t vs[2] = { v, w };
                const sl_contact_pair p =
                    sl_pair_make(SL_PAIR_VERTEX_VERTEX, 2u, vs, radius, mu);
                sl_pair_add(s, first, &p, c.distance, distances);
            } else if (c.kind == SL_FEATURE_EDGE) {
                uint32_t a = idx[c.a];
                uint32_t b = idx[c.b];
                if (a > b) {
                    const uint32_t tmp = a;
                    a = b;
                    b = tmp;
                }
                if (!sl_edge_feasible(s, x, a, b)) {
                    continue;
                }
                const uint32_t vs[3] = { v, a, b };
                const sl_contact_pair p =
                    sl_pair_make(SL_PAIR_VERTEX_EDGE, 3u, vs, radius, mu);
                sl_pair_add(s, first, &p, c.distance, distances);
            } else {
                const uint32_t vs[4] = { v, idx[0], idx[1], idx[2] };
                const sl_contact_pair p =
                    sl_pair_make(SL_PAIR_VERTEX_FACE, 4u, vs, radius, mu);
                sl_pair_add(s, first, &p, c.distance, distances);
            }
        }
    }
    s->vertex_dmin[v] = dmin;
}

/* Stages the contact of `owner` (edge o0, o1) against the other edge's
 * closest feature: its vertex w when the closest point sits at an end
 * (t <= 0 or t >= 1, feasible in w's vertex block for the point x on the
 * owner), else its interior (f0, f1). */
static void sl_stage_edge(sl_world_state *s, uint32_t owner, uint32_t o0,
                          uint32_t o1, uint32_t f0, uint32_t f1, float t,
                          sl_vec3 x, float distance, float radius, float mu)
{
    sl_contact_pair pair;
    if (t <= 0.0f || t >= 1.0f) {
        const uint32_t w = (t <= 0.0f) ? f0 : f1;
        if (!sl_vertex_feasible(s, x, w)) {
            return;
        }
        const uint32_t vs[3] = { o0, o1, w };
        pair = sl_pair_make(SL_PAIR_EDGE_VERTEX, 3u, vs, radius, mu);
    } else {
        const uint32_t vs[4] = { o0, o1, f0, f1 };
        pair = sl_pair_make(SL_PAIR_EDGE_EDGE, 4u, vs, radius, mu);
    }
    if (s->stage_count >= s->pair_capacity) {
        s->self_dropped++;
        return;
    }
    s->stage_pair[s->stage_count] = pair;
    s->stage_distance[s->stage_count] = distance;
    s->stage_owner[s->stage_count] = owner;
    s->stage_count++;
}

/* One unordered edge pair (e, f), paper Algorithm 2 from both sides:
 * updates both minimal distances and stages each side's contact. */
static void sl_edge_pair(sl_world_state *s, uint32_t e, uint32_t f, float query)
{
    const sl_bvh *bvh = &s->edge_bvh;
    /* Box then sphere cull (paper Alg. 2: radius r_q + l / 2 about the
     * midpoint), before the exact segment distance. */
    const sl_vec3 grow = sl_vec3_make(query, query, query);
    if (!sl_element_overlaps(bvh, f, sl_vec3_sub(bvh->elem_lo[e], grow),
                             sl_vec3_add(bvh->elem_hi[e], grow))) {
        return;
    }
    const float reach = bvh->sphere_radius[e] + bvh->sphere_radius[f] + query;
    if (sl_vec3_length_sq(sl_vec3_sub(
            bvh->sphere_center[e], bvh->sphere_center[f])) >= reach * reach) {
        return;
    }
    const uint32_t e0 = s->edge_index[2u * e];
    const uint32_t e1 = s->edge_index[2u * e + 1u];
    const uint32_t f0 = s->edge_index[2u * f];
    const uint32_t f1 = s->edge_index[2u * f + 1u];
    if (f0 == e0 || f0 == e1 || f1 == e0 || f1 == e1) {
        return;
    }
    const sl_vec3 p[2] = { s->position[e0], s->position[e1] };
    const sl_vec3 q[2] = { s->position[f0], s->position[f1] };
    float a;
    float b;
    sl_segment_closest_params(p[0], p[1], q[0], q[1], &a, &b);
    const sl_vec3 pc = sl_vec3_lerp(p[0], p[1], a);
    const sl_vec3 qc = sl_vec3_lerp(q[0], q[1], b);
    const float d = sl_vec3_length(sl_vec3_sub(pc, qc));
    if (!(d < query)) {
        return;
    }
    const uint32_t cloth_e = s->vertex_cloth[e0];
    const uint32_t cloth_f = s->vertex_cloth[f0];
    if (cloth_e == cloth_f) {
        s->edge_dmin[e] = sl_min(s->edge_dmin[e], d);
        s->edge_dmin[f] = sl_min(s->edge_dmin[f], d);
    } else {
        s->cloths[cloth_e].inter_dmin =
            sl_min(s->cloths[cloth_e].inter_dmin, d);
        s->cloths[cloth_f].inter_dmin =
            sl_min(s->cloths[cloth_f].inter_dmin, d);
    }
    const float radius = sl_thickness_of(s, e0) + sl_thickness_of(s, f0);
    if (!(d < radius) || !(d > 0.0f)) {
        return;
    }
    const float mu = sqrtf(sl_friction_of(s, e0) * sl_friction_of(s, f0));
    sl_stage_edge(s, e, e0, e1, f0, f1, b, pc, d, radius, mu);
    sl_stage_edge(s, f, f0, f1, e0, e1, a, qc, d, radius, mu);
}

/* All edge pairs within r_q through one self traversal of the edge tree:
 * each unordered pair is visited exactly once. */
static void sl_detect_edges(sl_world_state *s, float query)
{
    const sl_bvh *bvh = &s->edge_bvh;
    sl_bvh_pair_cursor cursor;
    sl_bvh_pair_cursor_init(&cursor, bvh, query);
    uint32_t a_first;
    uint32_t a_count;
    uint32_t b_first;
    uint32_t b_count;
    bool same;
    while (sl_bvh_next_leaf_pair(&cursor, bvh, &a_first, &a_count, &b_first,
                                 &b_count, &same)) {
        for (uint32_t i = a_first; i < a_first + a_count; i++) {
            const uint32_t j_begin = same ? i + 1u : b_first;
            for (uint32_t j = j_begin; j < b_first + b_count; j++) {
                const uint32_t x = bvh->perm[i];
                const uint32_t y = bvh->perm[j];
                sl_edge_pair(s, (x < y) ? x : y, (x < y) ? y : x, query);
            }
        }
    }
}

/* Moves staged edge contacts into the pair list grouped by owner in
 * ascending order, dropping duplicates and keeping each owner's
 * SL_VERTEX_CONTACT_COUNT_MAX closest. */
static void sl_commit_edges(sl_world_state *s)
{
    uint32_t *offset = s->owner_offset;
    memset(offset, 0, sizeof(uint32_t) * (s->edge_count + 1u));
    for (uint32_t k = 0; k < s->stage_count; k++) {
        offset[s->stage_owner[k] + 1u]++;
    }
    for (uint32_t e = 0; e < s->edge_count; e++) {
        offset[e + 1u] += offset[e];
    }
    for (uint32_t k = 0; k < s->stage_count; k++) {
        s->stage_order[offset[s->stage_owner[k]]++] = k;
    }
    for (uint32_t e = s->edge_count; e > 0u; e--) {
        offset[e] = offset[e - 1u];
    }
    offset[0] = 0u;
    float distances[SL_VERTEX_CONTACT_COUNT_MAX];
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const uint32_t first = s->pair_count;
        for (uint32_t k = offset[e]; k < offset[e + 1u]; k++) {
            const uint32_t staged = s->stage_order[k];
            sl_pair_add(s, first, &s->stage_pair[staged],
                        s->stage_distance[staged], distances);
        }
    }
}

void sl_self_index(sl_world_state *s)
{
    uint32_t *offset = s->pair_ref_offset;
    memset(offset, 0, sizeof(uint32_t) * (s->vertex_count + 1u));
    for (uint32_t k = 0; k < s->pair_count; k++) {
        const sl_contact_pair *p = &s->pairs[k];
        for (uint32_t slot = 0; slot < p->count; slot++) {
            offset[p->vertex[slot] + 1u]++;
        }
    }
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        offset[i + 1u] += offset[i];
    }
    /* Fill advancing offset[v] as v's cursor, then shift back. */
    for (uint32_t k = 0; k < s->pair_count; k++) {
        const sl_contact_pair *p = &s->pairs[k];
        for (uint32_t slot = 0; slot < p->count; slot++) {
            s->pair_ref[offset[p->vertex[slot]]++] = 4u * k + slot;
        }
    }
    for (uint32_t i = s->vertex_count; i > 0u; i--) {
        offset[i] = offset[i - 1u];
    }
    offset[0] = 0u;
}

void sl_self_detect(sl_world_state *s, float motion)
{
    SL_ASSERT(sl_self_enabled(s));
    s->self_detections++;
    s->pair_count = 0u;
    s->bound_exceeded = 0u;
    if (s->vertex_count == 0u) {
        return;
    }
    /* Query radius r_q = r_max + motion (the paper's choice). The motion
     * part is capped: within a sheet, non-adjacent primitives sit about
     * half an edge apart, so bounds cannot usefully exceed that, while the
     * candidates per query (and the work per detection) grow with r_q. */
    float thickness_max = 0.0f;
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        thickness_max = sl_max(thickness_max, s->cloths[c].thickness);
    }
    float extent_max = 0.0f;
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        for (uint32_t k = 0; k < 3u; k++) {
            const sl_vec3 d = sl_vec3_sub(s->position[idx[k]],
                                          s->position[idx[(k + 1u) % 3u]]);
            extent_max =
                sl_max(extent_max,
                       sl_max(sl_abs(d.x), sl_max(sl_abs(d.y), sl_abs(d.z))));
        }
    }
    const float query =
        2.0f * thickness_max +
        sl_min(sl_max(motion, 0.0f), SL_QUERY_MOTION_EXTENT * extent_max);
    s->query_radius = query;
    sl_self_refit(s);

    for (uint32_t t = 0; t < s->triangle_count; t++) {
        s->triangle_dmin[t] = query;
    }
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        sl_cloth_record *rec = &s->cloths[c];
        rec->inter_dmin = query;
        rec->box_lo = s->position[rec->vertex_offset];
        rec->box_hi = rec->box_lo;
        for (uint32_t i = rec->vertex_offset;
             i < rec->vertex_offset + rec->vertex_count; i++) {
            const sl_vec3 x = s->position[i];
            rec->box_lo = sl_vec3_make(sl_min(rec->box_lo.x, x.x),
                                       sl_min(rec->box_lo.y, x.y),
                                       sl_min(rec->box_lo.z, x.z));
            rec->box_hi = sl_vec3_make(sl_max(rec->box_hi.x, x.x),
                                       sl_max(rec->box_hi.y, x.y),
                                       sl_max(rec->box_hi.z, x.z));
        }
    }
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        sl_detect_vertex(s, v, query);
    }
    for (uint32_t e = 0; e < s->edge_count; e++) {
        s->edge_dmin[e] = query;
    }
    s->stage_count = 0u;
    sl_detect_edges(s, query);
    sl_commit_edges(s);

    /* Bounds (Eq. 21): the nearest of v's own distance to faces, its edges'
     * distances to edges, and its faces' distances to vertices. */
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        s->bound[v] = s->vertex_dmin[v];
        s->at_bound[v] = 0u;
    }
    for (uint32_t e = 0; e < s->edge_count; e++) {
        for (uint32_t k = 0; k < 2u; k++) {
            const uint32_t v = s->edge_index[2u * e + k];
            s->bound[v] = sl_min(s->bound[v], s->edge_dmin[e]);
        }
    }
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        for (uint32_t k = 0; k < 3u; k++) {
            const uint32_t v = s->triangle_index[3u * t + k];
            s->bound[v] = sl_min(s->bound[v], s->triangle_dmin[t]);
        }
    }
    /* B_c from the nearest other cloth: found pairs inside the query, or
     * the bounding-box gap when that is wider (a valid lower bound). */
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        sl_cloth_record *a = &s->cloths[c];
        float inter = SL_SELF_UNBOUNDED;
        for (uint32_t k = 0; k < s->cloth_count; k++) {
            if (k == c) {
                continue;
            }
            const sl_cloth_record *b = &s->cloths[k];
            const sl_vec3 gap =
                sl_vec3_make(sl_max(0.0f, sl_max(b->box_lo.x - a->box_hi.x,
                                                 a->box_lo.x - b->box_hi.x)),
                             sl_max(0.0f, sl_max(b->box_lo.y - a->box_hi.y,
                                                 a->box_lo.y - b->box_hi.y)),
                             sl_max(0.0f, sl_max(b->box_lo.z - a->box_hi.z,
                                                 a->box_lo.z - b->box_hi.z)));
            const float box_gap = sl_vec3_length(gap);
            inter = sl_min(inter, (box_gap >= query)
                                      ? box_gap
                                      : sl_min(a->inter_dmin, query));
        }
        a->translation_cap = SL_BOUND_RELAXATION * inter; /* B_c for now */
    }
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        const sl_cloth_record *rec = &s->cloths[s->vertex_cloth[v]];
        s->bound[v] =
            sl_min(SL_BOUND_RELAXATION * s->bound[v], rec->translation_cap);
        s->position_safe[v] = s->position[v];
    }
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        sl_cloth_record *rec = &s->cloths[c];
        float deviation = 0.0f;
        for (uint32_t i = rec->vertex_offset;
             i < rec->vertex_offset + rec->vertex_count; i++) {
            deviation = sl_max(deviation, s->bound[i]);
        }
        rec->translation_cap = sl_max(0.0f, rec->translation_cap - deviation);
    }
    sl_self_index(s);
}

uint32_t sl_self_constrain(sl_world_state *s)
{
    uint32_t moved = 0;
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        const sl_cloth_record *rec = &s->cloths[c];
        const uint32_t end = rec->vertex_offset + rec->vertex_count;
        /* T_c: mean displacement of the free vertices, clamped to its cap. */
        sl_vec3 shift = sl_vec3_zero();
        uint32_t free_count = 0;
        for (uint32_t i = rec->vertex_offset; i < end; i++) {
            if (s->pinned[i] == 0u) {
                shift = sl_vec3_add(
                    shift, sl_vec3_sub(s->position[i], s->position_safe[i]));
                free_count++;
            }
        }
        if (free_count == 0u) {
            continue;
        }
        shift = sl_vec3_scale(shift, 1.0f / (float)free_count);
        const float length = sl_vec3_length(shift);
        if (length > rec->translation_cap) {
            shift = sl_vec3_scale(shift, rec->translation_cap / length);
        }
        for (uint32_t i = rec->vertex_offset; i < end; i++) {
            if (s->pinned[i] != 0u) {
                continue;
            }
            const sl_vec3 anchor = sl_vec3_add(s->position_safe[i], shift);
            const sl_vec3 d = sl_vec3_sub(s->position[i], anchor);
            const float deviation = sl_vec3_length(d);
            if (deviation > s->bound[i]) {
                s->position[i] =
                    sl_vec3_mul_add(anchor, d, s->bound[i] / deviation);
                moved++;
                if (s->at_bound[i] == 0u) {
                    s->at_bound[i] = 1u;
                    s->bound_exceeded++;
                }
            }
        }
    }
    return moved;
}

void sl_self_friction(sl_world_state *s, float h)
{
    for (uint32_t k = 0; k < s->pair_count; k++) {
        const sl_contact_pair *p = &s->pairs[k];
        if (!(p->friction > 0.0f)) {
            continue;
        }
        float w[4];
        sl_vec3 sep;
        if (!sl_pair_separation(s, p, w, &sep)) {
            continue;
        }
        const float length = sl_vec3_length(sep);
        if (!(length < p->radius) || !(length > 0.0f)) {
            continue;
        }
        float slope;
        float curvature;
        (void)sl_contact_energy(
            sl_max(length, SL_CONTACT_DISTANCE_MIN_FRACTION * p->radius),
            p->radius, s->self_stiffness, &slope, &curvature);
        const sl_vec3 n = sl_vec3_scale(sep, 1.0f / length);
        sl_vec3 rel = sl_vec3_zero();
        for (uint32_t j = 0; j < p->count; j++) {
            rel = sl_vec3_mul_add(rel, s->velocity[p->vertex[j]], w[j]);
        }
        const sl_vec3 tangential =
            sl_vec3_mul_add(rel, n, -sl_vec3_dot(n, rel));
        const float speed = sl_vec3_length(tangential);
        float inverse = 0.0f;
        for (uint32_t j = 0; j < p->count; j++) {
            const uint32_t v = p->vertex[j];
            if (s->pinned[v] == 0u) {
                inverse += w[j] * w[j] / s->mass[v];
            }
        }
        if (!(speed > 0.0f) || !(inverse > 0.0f)) {
            continue;
        }
        const float reach =
            sl_pair_share(p) * p->friction * (-slope) * h * inverse;
        const float cut = (speed <= reach) ? 1.0f : reach / speed;
        const sl_vec3 impulse = sl_vec3_scale(tangential, -cut / inverse);
        for (uint32_t j = 0; j < p->count; j++) {
            const uint32_t v = p->vertex[j];
            if (s->pinned[v] == 0u) {
                s->velocity[v] =
                    sl_vec3_mul_add(s->velocity[v], impulse, w[j] / s->mass[v]);
            }
        }
    }
}
