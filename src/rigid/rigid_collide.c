#include <math.h>
#include <string.h>

#include "geometry.h"
#include "rigid.h"

/* Box narrow phase by the separating axis test (Gottschalk et al. 1996;
 * Ericson, Real-Time Collision Detection, 4.4.1) over the 15 candidate
 * axes, then contact generation from the chosen axis: a face axis clips
 * the other box's most opposed face against the reference face's sides
 * (Sutherland-Hodgman), an edge axis gives the closest points of the two
 * edges. Box 1 and 2 below mean the reference and incident boxes of a face
 * contact. */

/* Axis choice with hysteresis: a face of b replaces a face of a only when
 * its separation is larger by SL_SAT_FACE_TOLERANCE, and an edge axis
 * replaces the face only when larger by SL_SAT_EDGE_TOLERANCE. Nearly
 * equal axes would otherwise alternate between steps, and with them the
 * contact points and feature ids (tests/test_rigid.c pins both choices). */
#define SL_SAT_FACE_TOLERANCE 0.0005f
#define SL_SAT_EDGE_TOLERANCE 0.001f

/* Edge axes from nearly parallel edges, |a x b|^2 below this, are skipped:
 * their cross product is ill-conditioned and a face axis covers them. */
#define SL_SAT_PARALLEL_SQ 1.0e-6f

/* Feature ids. A face contact point lies on two features of the clipped
 * polygon: incident face edges E0..E3 (codes 0-3) and reference face side
 * planes P0..P3 (codes 4-7); with the reference and incident faces and
 * which box is the reference, that identifies it while the boxes keep
 * their relative arrangement. */
#define SL_ID_EDGE_CONTACT 0x10000u
#define SL_ID_PLANE_CONTACT 0x20000u

static float sl_get(sl_vec3 v, uint32_t k)
{
    return (k == 0u) ? v.x : ((k == 1u) ? v.y : v.z);
}

static sl_vec3 sl_column(sl_mat3 m, uint32_t k)
{
    return (k == 0u) ? m.c0 : ((k == 1u) ? m.c1 : m.c2);
}

/* +1 for x >= 0, else -1 (deterministic at 0). */
static float sl_sign(float x)
{
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

static void sl_manifold_clear(sl_manifold *out)
{
    memset(out, 0, sizeof(*out));
}

/* --------------------------------------------------------------------- */
/* Reduction                                                              */

/* Signed area of (a, b, c) about normal n, times two. */
static float sl_area(sl_vec3 a, sl_vec3 b, sl_vec3 c, sl_vec3 n)
{
    return sl_vec3_dot(sl_vec3_cross(sl_vec3_sub(b, a), sl_vec3_sub(c, a)), n);
}

/* Picks at most SL_MANIFOLD_POINT_COUNT_MAX of `count` candidates spanning
 * a large area: the deepest, the farthest from it, the one making the
 * largest triangle with those, and the one adding the most area to that
 * triangle. Ties go to the lower index, so the choice is deterministic.
 * Writes the chosen indices in ascending order to keep[] and returns how
 * many. */
static uint32_t sl_rigid_reduce(const sl_vec3 *x, const float *separation,
                                uint32_t count, sl_vec3 n, uint32_t *keep)
{
    if (count <= SL_MANIFOLD_POINT_COUNT_MAX) {
        for (uint32_t k = 0; k < count; k++) {
            keep[k] = k;
        }
        return count;
    }
    uint32_t i0 = 0;
    for (uint32_t k = 1; k < count; k++) {
        if (separation[k] < separation[i0]) {
            i0 = k;
        }
    }
    uint32_t i1 = (i0 == 0u) ? 1u : 0u;
    float best = -1.0f;
    for (uint32_t k = 0; k < count; k++) {
        const float d = sl_vec3_length_sq(sl_vec3_sub(x[k], x[i0]));
        if (k != i0 && d > best) {
            best = d;
            i1 = k;
        }
    }
    uint32_t i2 = UINT32_MAX;
    best = 0.0f;
    for (uint32_t k = 0; k < count; k++) {
        const float a = sl_abs(sl_area(x[i0], x[i1], x[k], n));
        if (k != i0 && k != i1 && a > best) {
            best = a;
            i2 = k;
        }
    }
    uint32_t chosen[4] = { i0, i1, i2, UINT32_MAX };
    uint32_t chosen_count = (i2 == UINT32_MAX) ? 2u : 3u;
    if (i2 != UINT32_MAX) {
        /* Area a point adds outside the triangle: across the edge where it
         * lies opposite the triangle's third vertex. */
        const uint32_t tri[3] = { i0, i1, i2 };
        const float orient = sl_sign(sl_area(x[i0], x[i1], x[i2], n));
        best = 0.0f;
        for (uint32_t k = 0; k < count; k++) {
            if (k == i0 || k == i1 || k == i2) {
                continue;
            }
            float added = 0.0f;
            for (uint32_t e = 0; e < 3u; e++) {
                const float a =
                    orient * sl_area(x[tri[e]], x[tri[(e + 1u) % 3u]], x[k], n);
                added = sl_max(added, -a);
            }
            if (added > best) {
                best = added;
                chosen[3] = k;
            }
        }
        if (chosen[3] != UINT32_MAX) {
            chosen_count = 4u;
        }
    }
    /* Ascending order, so the output order does not depend on the picks. */
    for (uint32_t a = 1; a < chosen_count; a++) {
        for (uint32_t b = a; b > 0u && chosen[b - 1u] > chosen[b]; b--) {
            const uint32_t t = chosen[b];
            chosen[b] = chosen[b - 1u];
            chosen[b - 1u] = t;
        }
    }
    for (uint32_t k = 0; k < chosen_count; k++) {
        keep[k] = chosen[k];
    }
    return chosen_count;
}

/* --------------------------------------------------------------------- */
/* Face contact                                                           */

typedef struct sl_clip_vertex {
    sl_vec3 x;
    uint8_t f0; /* features it lies on, f0 < f1 */
    uint8_t f1;
} sl_clip_vertex;

#define SL_CLIP_VERTEX_MAX 8u

/* Keeps the part of the convex polygon `in` with dot(x, normal) <= offset.
 * A vertex made on an edge takes that edge's shared feature and `plane`.
 * Clipping a convex polygon by a half-space adds at most one vertex. */
static uint32_t sl_clip(const sl_clip_vertex *in, uint32_t count,
                        sl_vec3 normal, float offset, uint8_t plane,
                        sl_clip_vertex *out)
{
    uint32_t n = 0;
    for (uint32_t e = 0; e < count; e++) {
        const sl_clip_vertex p = in[e];
        const sl_clip_vertex q = in[(e + 1u) % count];
        const float dp = sl_vec3_dot(p.x, normal) - offset;
        const float dq = sl_vec3_dot(q.x, normal) - offset;
        if (dp <= 0.0f) {
            SL_ASSERT(n < SL_CLIP_VERTEX_MAX);
            out[n++] = p;
        }
        if ((dp <= 0.0f) != (dq <= 0.0f)) {
            const float t = dp / (dp - dq);
            const uint8_t shared = (p.f0 == q.f0 || p.f0 == q.f1) ? p.f0 : p.f1;
            sl_clip_vertex v;
            v.x = sl_vec3_mul_add(p.x, sl_vec3_sub(q.x, p.x), t);
            v.f0 = (shared < plane) ? shared : plane;
            v.f1 = (shared < plane) ? plane : shared;
            SL_ASSERT(n < SL_CLIP_VERTEX_MAX);
            out[n++] = v;
        }
    }
    return n;
}

/* Face contact with reference face axis k of box 1. flip: box 1 is the
 * manifold's second box, so the normal and anchors swap. */
static uint32_t sl_face_contact(sl_vec3 p1, sl_quat q1, sl_mat3 m1, sl_vec3 h1,
                                uint32_t k, sl_vec3 p2, sl_quat q2, sl_mat3 m2,
                                sl_vec3 h2, bool flip, float margin,
                                sl_manifold *out)
{
    /* Reference face: the side of box 1 facing box 2. */
    const sl_vec3 axis = sl_column(m1, k);
    const float s = sl_sign(sl_vec3_dot(sl_vec3_sub(p2, p1), axis));
    const sl_vec3 n = sl_vec3_scale(axis, s);
    const uint32_t f_ref = 2u * k + ((s < 0.0f) ? 1u : 0u);
    const sl_vec3 face_center = sl_vec3_mul_add(p1, n, sl_get(h1, k));

    /* Incident face: the face of box 2 whose outward normal is most
     * opposed to n. */
    uint32_t j = 0;
    float best = -1.0f;
    for (uint32_t c = 0; c < 3u; c++) {
        const float d = sl_abs(sl_vec3_dot(sl_column(m2, c), n));
        if (d > best) {
            best = d;
            j = c;
        }
    }
    const float sigma = -sl_sign(sl_vec3_dot(sl_column(m2, j), n));
    const uint32_t f_inc = 2u * j + ((sigma < 0.0f) ? 1u : 0u);
    const uint32_t ju = (j + 1u) % 3u;
    const uint32_t jv = (j + 2u) % 3u;
    const sl_vec3 inc_center =
        sl_vec3_mul_add(p2, sl_column(m2, j), sigma * sl_get(h2, j));
    const sl_vec3 eu = sl_vec3_scale(sl_column(m2, ju), sl_get(h2, ju));
    const sl_vec3 ev = sl_vec3_scale(sl_column(m2, jv), sl_get(h2, jv));
    static const float k_su[4] = { 1.0f, -1.0f, -1.0f, 1.0f };
    static const float k_sv[4] = { 1.0f, 1.0f, -1.0f, -1.0f };
    sl_clip_vertex poly[SL_CLIP_VERTEX_MAX];
    sl_clip_vertex next[SL_CLIP_VERTEX_MAX];
    uint32_t count = 4u;
    for (uint32_t v = 0; v < 4u; v++) {
        poly[v].x = sl_vec3_mul_add(sl_vec3_mul_add(inc_center, eu, k_su[v]),
                                    ev, k_sv[v]);
        /* Vertex v joins edges v - 1 and v (edge v runs to vertex v + 1). */
        const uint8_t before = (uint8_t)((v + 3u) % 4u);
        const uint8_t after = (uint8_t)v;
        poly[v].f0 = (before < after) ? before : after;
        poly[v].f1 = (before < after) ? after : before;
    }

    /* Side planes of the reference face: +-u then +-v. */
    const uint32_t ku = (k + 1u) % 3u;
    const uint32_t kv = (k + 2u) % 3u;
    const sl_vec3 side_axis[2] = { sl_column(m1, ku), sl_column(m1, kv) };
    const float side_half[2] = { sl_get(h1, ku), sl_get(h1, kv) };
    for (uint32_t plane = 0; plane < 4u && count > 0u; plane++) {
        const float sign = ((plane & 1u) == 0u) ? 1.0f : -1.0f;
        const sl_vec3 normal = sl_vec3_scale(side_axis[plane / 2u], sign);
        const float offset = sl_vec3_dot(p1, normal) + side_half[plane / 2u];
        count =
            sl_clip(poly, count, normal, offset, (uint8_t)(4u + plane), next);
        memcpy(poly, next, sizeof(sl_clip_vertex) * count);
    }

    /* Points of the clipped polygon within the margin of the face. */
    sl_vec3 x[SL_CLIP_VERTEX_MAX];
    float separation[SL_CLIP_VERTEX_MAX];
    uint32_t id[SL_CLIP_VERTEX_MAX];
    uint32_t kept = 0;
    for (uint32_t v = 0; v < count; v++) {
        const float d = sl_vec3_dot(sl_vec3_sub(poly[v].x, face_center), n);
        if (d <= margin) {
            x[kept] = poly[v].x;
            separation[kept] = d;
            id[kept] = ((flip ? 1u : 0u) << 12) | (f_ref << 9) | (f_inc << 6) |
                       ((uint32_t)poly[v].f0 << 3) | (uint32_t)poly[v].f1;
            kept++;
        }
    }
    uint32_t keep[SL_MANIFOLD_POINT_COUNT_MAX];
    const uint32_t chosen = sl_rigid_reduce(x, separation, kept, n, keep);
    out->normal = flip ? sl_vec3_neg(n) : n;
    out->point_count = chosen;
    for (uint32_t c = 0; c < chosen; c++) {
        const uint32_t v = keep[c];
        const sl_vec3 on_incident = x[v];
        const sl_vec3 on_reference =
            sl_vec3_mul_add(on_incident, n, -separation[v]);
        const sl_vec3 local_1 =
            sl_quat_rotate_inv(q1, sl_vec3_sub(on_reference, p1));
        const sl_vec3 local_2 =
            sl_quat_rotate_inv(q2, sl_vec3_sub(on_incident, p2));
        sl_contact_point *point = &out->points[c];
        point->local_a = flip ? local_2 : local_1;
        point->local_b = flip ? local_1 : local_2;
        point->separation = separation[v];
        point->id = id[v];
    }
    return chosen;
}

/* --------------------------------------------------------------------- */
/* Edge contact                                                           */

/* The edge of a box parallel to axis i that is furthest along n, as a
 * center and its index (4 i + sign bits of the other two axes). */
static sl_vec3 sl_support_edge(sl_vec3 p, sl_mat3 m, sl_vec3 h, uint32_t i,
                               sl_vec3 n, uint32_t *index)
{
    const uint32_t i1 = (i + 1u) % 3u;
    const uint32_t i2 = (i + 2u) % 3u;
    const float s1 = sl_sign(sl_vec3_dot(sl_column(m, i1), n));
    const float s2 = sl_sign(sl_vec3_dot(sl_column(m, i2), n));
    *index = 4u * i + ((s1 < 0.0f) ? 1u : 0u) + ((s2 < 0.0f) ? 2u : 0u);
    sl_vec3 c = sl_vec3_mul_add(p, sl_column(m, i1), s1 * sl_get(h, i1));
    return sl_vec3_mul_add(c, sl_column(m, i2), s2 * sl_get(h, i2));
}

static uint32_t sl_edge_contact(sl_vec3 pa, sl_quat qa, sl_mat3 ma, sl_vec3 ha,
                                uint32_t i, sl_vec3 pb, sl_quat qb, sl_mat3 mb,
                                sl_vec3 hb, uint32_t j, float margin,
                                sl_manifold *out)
{
    const sl_vec3 ai = sl_column(ma, i);
    const sl_vec3 bj = sl_column(mb, j);
    sl_vec3 n = sl_vec3_normalize(sl_vec3_cross(ai, bj));
    if (sl_vec3_dot(n, sl_vec3_sub(pb, pa)) < 0.0f) {
        n = sl_vec3_neg(n);
    }
    uint32_t edge_a = 0;
    uint32_t edge_b = 0;
    const sl_vec3 ca = sl_support_edge(pa, ma, ha, i, n, &edge_a);
    const sl_vec3 cb = sl_support_edge(pb, mb, hb, j, sl_vec3_neg(n), &edge_b);
    const sl_vec3 da = sl_vec3_scale(ai, sl_get(ha, i));
    const sl_vec3 db = sl_vec3_scale(bj, sl_get(hb, j));
    float s = 0.0f;
    float t = 0.0f;
    sl_segment_closest_params(sl_vec3_sub(ca, da), sl_vec3_add(ca, da),
                              sl_vec3_sub(cb, db), sl_vec3_add(cb, db), &s, &t);
    const sl_vec3 on_a = sl_vec3_mul_add(sl_vec3_sub(ca, da), da, 2.0f * s);
    const sl_vec3 on_b = sl_vec3_mul_add(sl_vec3_sub(cb, db), db, 2.0f * t);
    const float separation = sl_vec3_dot(sl_vec3_sub(on_b, on_a), n);
    if (separation > margin) {
        return 0u;
    }
    out->normal = n;
    out->point_count = 1u;
    sl_contact_point *point = &out->points[0];
    point->local_a = sl_quat_rotate_inv(qa, sl_vec3_sub(on_a, pa));
    point->local_b = sl_quat_rotate_inv(qb, sl_vec3_sub(on_b, pb));
    point->separation = separation;
    point->id = SL_ID_EDGE_CONTACT | (edge_a << 4) | edge_b;
    return 1u;
}

/* --------------------------------------------------------------------- */
/* Box-box                                                                */

uint32_t sl_collide_box_box(sl_vec3 pa, sl_quat qa, sl_vec3 ha, sl_vec3 pb,
                            sl_quat qb, sl_vec3 hb, float margin,
                            sl_manifold *out)
{
    SL_ASSERT(out != NULL && margin >= 0.0f);
    sl_manifold_clear(out);
    const sl_mat3 ma = sl_quat_to_mat3(qa);
    const sl_mat3 mb = sl_quat_to_mat3(qb);
    const sl_vec3 d = sl_vec3_sub(pb, pa);
    float r[3][3];
    float ar[3][3];
    float t[3];
    float hap[3];
    float hbp[3];
    for (uint32_t a = 0; a < 3u; a++) {
        t[a] = sl_vec3_dot(d, sl_column(ma, a));
        hap[a] = sl_get(ha, a);
        hbp[a] = sl_get(hb, a);
        for (uint32_t b = 0; b < 3u; b++) {
            r[a][b] = sl_vec3_dot(sl_column(ma, a), sl_column(mb, b));
            /* The epsilon keeps rounding from creating a separating axis
             * where an edge pair is near parallel (Ericson 4.4.1). */
            ar[a][b] = sl_abs(r[a][b]) + 1.0e-6f;
        }
    }

    float sep_a = -INFINITY;
    uint32_t face_a = 0;
    for (uint32_t a = 0; a < 3u; a++) {
        const float rb =
            hbp[0] * ar[a][0] + hbp[1] * ar[a][1] + hbp[2] * ar[a][2];
        const float sep = sl_abs(t[a]) - (hap[a] + rb);
        if (sep > margin) {
            return 0u;
        }
        if (sep > sep_a) {
            sep_a = sep;
            face_a = a;
        }
    }
    float sep_b = -INFINITY;
    uint32_t face_b = 0;
    for (uint32_t b = 0; b < 3u; b++) {
        const float tb = t[0] * r[0][b] + t[1] * r[1][b] + t[2] * r[2][b];
        const float ra =
            hap[0] * ar[0][b] + hap[1] * ar[1][b] + hap[2] * ar[2][b];
        const float sep = sl_abs(tb) - (ra + hbp[b]);
        if (sep > margin) {
            return 0u;
        }
        if (sep > sep_b) {
            sep_b = sep;
            face_b = b;
        }
    }
    float sep_e = -INFINITY;
    uint32_t edge_a = 0;
    uint32_t edge_b = 0;
    for (uint32_t a = 0; a < 3u; a++) {
        const uint32_t a1 = (a + 1u) % 3u;
        const uint32_t a2 = (a + 2u) % 3u;
        for (uint32_t b = 0; b < 3u; b++) {
            const float length_sq = 1.0f - r[a][b] * r[a][b];
            if (!(length_sq > SL_SAT_PARALLEL_SQ)) {
                continue;
            }
            const uint32_t b1 = (b + 1u) % 3u;
            const uint32_t b2 = (b + 2u) % 3u;
            const float ra = hap[a1] * ar[a2][b] + hap[a2] * ar[a1][b];
            const float rb = hbp[b1] * ar[a][b2] + hbp[b2] * ar[a][b1];
            const float dist = sl_abs(t[a2] * r[a1][b] - t[a1] * r[a2][b]);
            const float sep = (dist - ra - rb) / sqrtf(length_sq);
            if (sep > margin) {
                return 0u;
            }
            if (sep > sep_e) {
                sep_e = sep;
                edge_a = a;
                edge_b = b;
            }
        }
    }

    const bool use_b = sep_b > sep_a + SL_SAT_FACE_TOLERANCE;
    const float sep_face = use_b ? sep_b : sep_a;
    if (sep_e > sep_face + SL_SAT_EDGE_TOLERANCE) {
        return sl_edge_contact(pa, qa, ma, ha, edge_a, pb, qb, mb, hb, edge_b,
                               margin, out);
    }
    if (use_b) {
        return sl_face_contact(pb, qb, mb, hb, face_b, pa, qa, ma, ha, true,
                               margin, out);
    }
    return sl_face_contact(pa, qa, ma, ha, face_a, pb, qb, mb, hb, false,
                           margin, out);
}

/* --------------------------------------------------------------------- */
/* Box-plane                                                              */

uint32_t sl_collide_box_plane(sl_vec3 pa, sl_quat qa, sl_vec3 ha,
                              const sl_rigid_plane *plane, float margin,
                              sl_manifold *out)
{
    SL_ASSERT(out != NULL && plane != NULL && margin >= 0.0f);
    sl_manifold_clear(out);
    sl_vec3 x[8];
    sl_vec3 local[8];
    float separation[8];
    uint32_t vertex[8];
    uint32_t count = 0;
    for (uint32_t v = 0; v < 8u; v++) {
        const sl_vec3 l = sl_vec3_make(((v & 1u) != 0u) ? -ha.x : ha.x,
                                       ((v & 2u) != 0u) ? -ha.y : ha.y,
                                       ((v & 4u) != 0u) ? -ha.z : ha.z);
        const sl_vec3 w = sl_vec3_add(pa, sl_quat_rotate(qa, l));
        const float d =
            sl_vec3_dot(sl_vec3_sub(w, plane->point), plane->normal);
        if (d <= margin) {
            x[count] = w;
            local[count] = l;
            separation[count] = d;
            vertex[count] = v;
            count++;
        }
    }
    uint32_t keep[SL_MANIFOLD_POINT_COUNT_MAX];
    const uint32_t chosen =
        sl_rigid_reduce(x, separation, count, plane->normal, keep);
    out->normal = sl_vec3_neg(plane->normal);
    out->point_count = chosen;
    for (uint32_t c = 0; c < chosen; c++) {
        const uint32_t v = keep[c];
        sl_contact_point *point = &out->points[c];
        point->local_a = local[v];
        point->local_b = sl_vec3_mul_add(x[v], plane->normal, -separation[v]);
        point->separation = separation[v];
        point->id = SL_ID_PLANE_CONTACT | vertex[v];
    }
    return chosen;
}
