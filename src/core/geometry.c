#include "geometry.h"

static sl_triangle_closest sl_closest_vertex(uint32_t corner, float distance)
{
    sl_triangle_closest c;
    c.kind = SL_FEATURE_VERTEX;
    c.a = corner;
    c.b = corner;
    c.bary[0] = (corner == 0u) ? 1.0f : 0.0f;
    c.bary[1] = (corner == 1u) ? 1.0f : 0.0f;
    c.bary[2] = (corner == 2u) ? 1.0f : 0.0f;
    c.distance = distance;
    return c;
}

/* Edge (a, b) of the triangle, a < b, at parameter t from a to b. */
static sl_triangle_closest sl_closest_edge(uint32_t a, uint32_t b, float t,
                                           float distance)
{
    sl_triangle_closest c;
    c.kind = SL_FEATURE_EDGE;
    c.a = a;
    c.b = b;
    c.bary[0] = 0.0f;
    c.bary[1] = 0.0f;
    c.bary[2] = 0.0f;
    c.bary[a] = 1.0f - t;
    c.bary[b] = t;
    c.distance = distance;
    return c;
}

float sl_segment_closest_param(sl_vec3 p, sl_vec3 a, sl_vec3 b)
{
    const sl_vec3 ab = sl_vec3_sub(b, a);
    const float length_sq = sl_vec3_length_sq(ab);
    if (!(length_sq > 0.0f)) {
        return 0.0f;
    }
    return sl_clamp(sl_vec3_dot(sl_vec3_sub(p, a), ab) / length_sq, 0.0f, 1.0f);
}

static float sl_point_distance(sl_vec3 p, const sl_vec3 t[3],
                               const float bary[3])
{
    sl_vec3 q = sl_vec3_scale(t[0], bary[0]);
    q = sl_vec3_mul_add(q, t[1], bary[1]);
    q = sl_vec3_mul_add(q, t[2], bary[2]);
    return sl_vec3_length(sl_vec3_sub(p, q));
}

/* Closest of the three edges; used for degenerate triangles. */
static sl_triangle_closest sl_closest_edge_of(sl_vec3 p, const sl_vec3 t[3])
{
    static const uint32_t k_edges[3][2] = { { 0u, 1u },
                                            { 0u, 2u },
                                            { 1u, 2u } };
    sl_triangle_closest best = sl_closest_vertex(0u, INFINITY);
    for (uint32_t e = 0; e < 3u; e++) {
        const uint32_t a = k_edges[e][0];
        const uint32_t b = k_edges[e][1];
        const float s = sl_segment_closest_param(p, t[a], t[b]);
        sl_triangle_closest c = sl_closest_edge(a, b, s, 0.0f);
        if (s <= 0.0f) {
            c = sl_closest_vertex(a, 0.0f);
        } else if (s >= 1.0f) {
            c = sl_closest_vertex(b, 0.0f);
        }
        c.distance = sl_point_distance(p, t, c.bary);
        if (c.distance < best.distance) {
            best = c;
        }
    }
    return best;
}

sl_triangle_closest sl_triangle_closest_point(sl_vec3 p, sl_vec3 t0, sl_vec3 t1,
                                              sl_vec3 t2)
{
    const sl_vec3 t[3] = { t0, t1, t2 };
    const sl_vec3 ab = sl_vec3_sub(t1, t0);
    const sl_vec3 ac = sl_vec3_sub(t2, t0);
    const sl_vec3 ap = sl_vec3_sub(p, t0);
    const float d1 = sl_vec3_dot(ab, ap);
    const float d2 = sl_vec3_dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        return sl_closest_vertex(0u, sl_vec3_length(ap));
    }
    const sl_vec3 bp = sl_vec3_sub(p, t1);
    const float d3 = sl_vec3_dot(ab, bp);
    const float d4 = sl_vec3_dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        return sl_closest_vertex(1u, sl_vec3_length(bp));
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        sl_triangle_closest c = sl_closest_edge(0u, 1u, d1 / (d1 - d3), 0.0f);
        c.distance = sl_point_distance(p, t, c.bary);
        return c;
    }
    const sl_vec3 cp = sl_vec3_sub(p, t2);
    const float d5 = sl_vec3_dot(ab, cp);
    const float d6 = sl_vec3_dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        return sl_closest_vertex(2u, sl_vec3_length(cp));
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        sl_triangle_closest c = sl_closest_edge(0u, 2u, d2 / (d2 - d6), 0.0f);
        c.distance = sl_point_distance(p, t, c.bary);
        return c;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        sl_triangle_closest c = sl_closest_edge(1u, 2u, w, 0.0f);
        c.distance = sl_point_distance(p, t, c.bary);
        return c;
    }
    const float sum = va + vb + vc;
    if (!(sum > 0.0f) || !sl_is_finite(sum)) {
        return sl_closest_edge_of(p, t);
    }
    const float v = vb / sum;
    const float w = vc / sum;
    sl_triangle_closest c;
    c.kind = SL_FEATURE_FACE;
    c.a = 0u;
    c.b = 0u;
    c.bary[0] = 1.0f - v - w;
    c.bary[1] = v;
    c.bary[2] = w;
    c.distance = sl_point_distance(p, t, c.bary);
    return c;
}

void sl_segment_closest_params(sl_vec3 p0, sl_vec3 p1, sl_vec3 q0, sl_vec3 q1,
                               float *s, float *t)
{
    SL_ASSERT(s != NULL && t != NULL);
    const sl_vec3 d1 = sl_vec3_sub(p1, p0);
    const sl_vec3 d2 = sl_vec3_sub(q1, q0);
    const sl_vec3 r = sl_vec3_sub(p0, q0);
    const float a = sl_vec3_dot(d1, d1);
    const float e = sl_vec3_dot(d2, d2);
    const float f = sl_vec3_dot(d2, r);
    if (!(a > 0.0f) && !(e > 0.0f)) {
        *s = 0.0f;
        *t = 0.0f;
        return;
    }
    if (!(a > 0.0f)) {
        *s = 0.0f;
        *t = sl_clamp(f / e, 0.0f, 1.0f);
        return;
    }
    const float c = sl_vec3_dot(d1, r);
    if (!(e > 0.0f)) {
        *t = 0.0f;
        *s = sl_clamp(-c / a, 0.0f, 1.0f);
        return;
    }
    const float b = sl_vec3_dot(d1, d2);
    const float denom = a * e - b * b;
    /* Near-parallel segments: start from s = 0 (Ericson's choice). */
    float sv = (denom > 1e-6f * a * e)
                   ? sl_clamp((b * f - c * e) / denom, 0.0f, 1.0f)
                   : 0.0f;
    float tv = (b * sv + f) / e;
    if (tv < 0.0f) {
        tv = 0.0f;
        sv = sl_clamp(-c / a, 0.0f, 1.0f);
    } else if (tv > 1.0f) {
        tv = 1.0f;
        sv = sl_clamp((b - c) / a, 0.0f, 1.0f);
    }
    *s = sv;
    *t = tv;
}
