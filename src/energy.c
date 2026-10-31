#include "energy.h"

bool sl_membrane_rest(sl_vec3 x0, sl_vec3 x1, sl_vec3 x2, float rest[4],
                      float *area)
{
    SL_ASSERT(rest != NULL);
    SL_ASSERT(area != NULL);
    const sl_vec3 e1 = sl_vec3_sub(x1, x0);
    const sl_vec3 e2 = sl_vec3_sub(x2, x0);
    const float twice_area = sl_vec3_length(sl_vec3_cross(e1, e2));
    const float length = sl_vec3_length(e1);
    if (!(0.5f * twice_area >= SL_TRIANGLE_AREA_MIN) || !(length > 0.0f)) {
        return false;
    }
    /* Rest frame: u along e1, v in-plane perpendicular. Then
     * Dm = [[length, e2.u], [0, height]] and rest = Dm^-1. */
    const float height = twice_area / length;
    const float e2u = sl_vec3_dot(e2, e1) / length;
    const float r[4] = { 1.0f / length, -e2u / (length * height), 0.0f,
                         1.0f / height };
    for (uint32_t i = 0; i < 4u; i++) {
        if (!sl_is_finite(r[i])) {
            return false;
        }
    }
    for (uint32_t i = 0; i < 4u; i++) {
        rest[i] = r[i];
    }
    *area = 0.5f * twice_area;
    return true;
}

typedef struct sl_membrane_strain {
    sl_vec3 f0, f1; /* columns of F */
    float g00, g01, g11;
} sl_membrane_strain;

static sl_membrane_strain sl_membrane_strain_make(const sl_vec3 x[3],
                                                  const float rest[4])
{
    const sl_vec3 e1 = sl_vec3_sub(x[1], x[0]);
    const sl_vec3 e2 = sl_vec3_sub(x[2], x[0]);
    sl_membrane_strain s;
    s.f0 = sl_vec3_mul_add(sl_vec3_scale(e1, rest[0]), e2, rest[2]);
    s.f1 = sl_vec3_mul_add(sl_vec3_scale(e1, rest[1]), e2, rest[3]);
    s.g00 = 0.5f * (sl_vec3_dot(s.f0, s.f0) - 1.0f);
    s.g01 = 0.5f * sl_vec3_dot(s.f0, s.f1);
    s.g11 = 0.5f * (sl_vec3_dot(s.f1, s.f1) - 1.0f);
    return s;
}

float sl_membrane_energy(const sl_vec3 x[3], const float rest[4], float area,
                         float mu, float lambda)
{
    const sl_membrane_strain s = sl_membrane_strain_make(x, rest);
    const float trace = s.g00 + s.g11;
    const float norm_sq = s.g00 * s.g00 + 2.0f * s.g01 * s.g01 + s.g11 * s.g11;
    return area * (mu * norm_sq + 0.5f * lambda * trace * trace);
}

/* Corner weight w_k, where F = sum_k x_k w_k^T with w1 = (a, b),
 * w2 = (c, d), and w0 = -(w1 + w2). */
static void sl_membrane_weight(const float rest[4], uint32_t corner, float *wx,
                               float *wy)
{
    if (corner == 1u) {
        *wx = rest[0];
        *wy = rest[1];
    } else if (corner == 2u) {
        *wx = rest[2];
        *wy = rest[3];
    } else {
        *wx = -(rest[0] + rest[2]);
        *wy = -(rest[1] + rest[3]);
    }
}

void sl_membrane_vertex(const sl_vec3 x[3], const float rest[4], float area,
                        float mu, float lambda, uint32_t corner,
                        sl_vec3 *gradient, sl_mat3 *hessian)
{
    SL_ASSERT(corner < 3u);
    SL_ASSERT(gradient != NULL);
    SL_ASSERT(hessian != NULL);
    const sl_membrane_strain s = sl_membrane_strain_make(x, rest);
    float wx;
    float wy;
    sl_membrane_weight(rest, corner, &wx, &wy);
    const float trace = s.g00 + s.g11;
    const float s00 = 2.0f * mu * s.g00 + lambda * trace;
    const float s01 = 2.0f * mu * s.g01;
    const float s11 = 2.0f * mu * s.g11 + lambda * trace;
    const float swx = s00 * wx + s01 * wy;
    const float swy = s01 * wx + s11 * wy;
    *gradient = sl_vec3_scale(
        sl_vec3_mul_add(sl_vec3_scale(s.f0, swx), s.f1, swy), area);

    const sl_vec3 u = sl_vec3_mul_add(sl_vec3_scale(s.f0, wx), s.f1, wy);
    const float wsw = sl_max(wx * swx + wy * swy, 0.0f);
    const float ww = wx * wx + wy * wy;
    sl_mat3 h = sl_mat3_diagonal(wsw);
    h = sl_mat3_mul_add(h, sl_mat3_outer(u, u), mu + lambda);
    h = sl_mat3_mul_add(h, sl_mat3_outer(s.f0, s.f0), mu * ww);
    h = sl_mat3_mul_add(h, sl_mat3_outer(s.f1, s.f1), mu * ww);
    *hessian = sl_mat3_scale(h, area);
}

/* cot of the angle between a and b; |a x b| > 0 is guaranteed by the
 * caller's area check. */
static float sl_cotangent(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_dot(a, b) / sl_vec3_length(sl_vec3_cross(a, b));
}

bool sl_bend_rest(const sl_vec3 x[4], float weight[4], float *scale)
{
    SL_ASSERT(weight != NULL);
    SL_ASSERT(scale != NULL);
    const sl_vec3 e0 = sl_vec3_sub(x[1], x[0]);
    const sl_vec3 e1 = sl_vec3_sub(x[2], x[0]);
    const sl_vec3 e2 = sl_vec3_sub(x[3], x[0]);
    const sl_vec3 e3 = sl_vec3_sub(x[2], x[1]);
    const sl_vec3 e4 = sl_vec3_sub(x[3], x[1]);
    const float area0 = 0.5f * sl_vec3_length(sl_vec3_cross(e0, e1));
    const float area1 = 0.5f * sl_vec3_length(sl_vec3_cross(e0, e2));
    if (!(area0 >= SL_TRIANGLE_AREA_MIN) || !(area1 >= SL_TRIANGLE_AREA_MIN)) {
        return false;
    }
    const float c01 = sl_cotangent(e0, e1);
    const float c02 = sl_cotangent(e0, e2);
    const float c03 = sl_cotangent(sl_vec3_neg(e0), e3);
    const float c04 = sl_cotangent(sl_vec3_neg(e0), e4);
    const float w[4] = { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
    /* Bergou et al. use 3 / (A0 + A1), which converges to three times the
     * continuum energy; 1 / (A0 + A1) makes stiffness the plate modulus. */
    const float s = 1.0f / (area0 + area1);
    for (uint32_t i = 0; i < 4u; i++) {
        if (!sl_is_finite(w[i])) {
            return false;
        }
    }
    if (!sl_is_finite(s)) {
        return false;
    }
    for (uint32_t i = 0; i < 4u; i++) {
        weight[i] = w[i];
    }
    *scale = s;
    return true;
}

static sl_vec3 sl_bend_sum(const sl_vec3 x[4], const float weight[4])
{
    /* Relative to x[0]: identical in exact arithmetic because the weights
     * sum to zero, and it removes absolute-position cancellation. */
    sl_vec3 sum = sl_vec3_zero();
    for (uint32_t j = 1; j < 4u; j++) {
        sum = sl_vec3_mul_add(sum, sl_vec3_sub(x[j], x[0]), weight[j]);
    }
    return sum;
}

float sl_bend_energy(const sl_vec3 x[4], const float weight[4], float scale,
                     float stiffness)
{
    const sl_vec3 sum = sl_bend_sum(x, weight);
    return 0.5f * stiffness * scale * sl_vec3_length_sq(sum);
}

void sl_bend_vertex(const sl_vec3 x[4], const float weight[4], float scale,
                    float stiffness, uint32_t slot, sl_vec3 *gradient,
                    float *hessian)
{
    SL_ASSERT(slot < 4u);
    SL_ASSERT(gradient != NULL);
    SL_ASSERT(hessian != NULL);
    const float k = stiffness * scale * weight[slot];
    *gradient = sl_vec3_scale(sl_bend_sum(x, weight), k);
    *hessian = k * weight[slot];
}

static float sl_round_distance(sl_vec3 center, float radius, sl_vec3 p,
                               sl_vec3 *normal)
{
    const sl_vec3 d = sl_vec3_sub(p, center);
    const float length = sl_vec3_length(d);
    if (length > 0.0f) {
        *normal = sl_vec3_scale(d, 1.0f / length);
    } else {
        *normal = sl_vec3_make(0.0f, 1.0f, 0.0f);
    }
    return length - radius;
}

float sl_collider_distance(const sl_collider *collider, sl_vec3 p,
                           sl_vec3 *normal)
{
    SL_ASSERT(collider != NULL);
    SL_ASSERT(normal != NULL);
    switch (collider->kind) {
    case SL_COLLIDER_PLANE:
        *normal = collider->point_b;
        return sl_vec3_dot(collider->point_b,
                           sl_vec3_sub(p, collider->point_a));
    case SL_COLLIDER_SPHERE:
        return sl_round_distance(collider->point_a, collider->radius, p,
                                 normal);
    case SL_COLLIDER_CAPSULE: {
        const sl_vec3 axis = sl_vec3_sub(collider->point_b, collider->point_a);
        const float length_sq = sl_vec3_length_sq(axis);
        float t = 0.0f;
        if (length_sq > 0.0f) {
            t = sl_clamp(sl_vec3_dot(sl_vec3_sub(p, collider->point_a), axis) /
                             length_sq,
                         0.0f, 1.0f);
        }
        const sl_vec3 closest = sl_vec3_mul_add(collider->point_a, axis, t);
        return sl_round_distance(closest, collider->radius, p, normal);
    }
    }
    SL_ASSERT(false);
    *normal = sl_vec3_make(0.0f, 1.0f, 0.0f);
    return 0.0f;
}
