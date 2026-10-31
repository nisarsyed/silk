#ifndef SILK_MATH_H
#define SILK_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "silk/assert.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SL_PI 3.14159265358979323846f

/* Near-unit absolute comparison and normalization tolerance (~8 ULP of
 * 1.0f); not a relative comparison or a collision tolerance. Pinned by
 * tests/test_math.c. */
#define SL_EPSILON 1e-6f

static inline float sl_min(float a, float b)
{
    return (a < b) ? a : b;
}

static inline float sl_max(float a, float b)
{
    return (a > b) ? a : b;
}

static inline float sl_clamp(float v, float lo, float hi)
{
    SL_ASSERT(lo <= hi);
    return sl_min(sl_max(v, lo), hi);
}

static inline float sl_abs(float v)
{
    return fabsf(v);
}

/* True when |a - b| <= eps; false if either operand is NaN. */
static inline bool sl_feq(float a, float b, float eps)
{
    return sl_abs(a - b) <= eps;
}

static inline bool sl_is_finite(float v)
{
    return isfinite(v);
}

/* Right-handed 3D vector. Units are given by each use site. */
typedef struct sl_vec3 {
    float x, y, z;
} sl_vec3;

static inline sl_vec3 sl_vec3_make(float x, float y, float z)
{
    sl_vec3 v = { x, y, z };
    return v;
}

static inline sl_vec3 sl_vec3_zero(void)
{
    return sl_vec3_make(0.0f, 0.0f, 0.0f);
}

static inline sl_vec3 sl_vec3_add(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline sl_vec3 sl_vec3_sub(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline sl_vec3 sl_vec3_scale(sl_vec3 v, float s)
{
    return sl_vec3_make(v.x * s, v.y * s, v.z * s);
}

/* a + b * s, the common accumulate form. */
static inline sl_vec3 sl_vec3_mul_add(sl_vec3 a, sl_vec3 b, float s)
{
    return sl_vec3_make(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
}

static inline sl_vec3 sl_vec3_neg(sl_vec3 v)
{
    return sl_vec3_make(-v.x, -v.y, -v.z);
}

static inline float sl_vec3_dot(sl_vec3 a, sl_vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline sl_vec3 sl_vec3_cross(sl_vec3 a, sl_vec3 b)
{
    return sl_vec3_make(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x);
}

static inline float sl_vec3_length_sq(sl_vec3 v)
{
    return sl_vec3_dot(v, v);
}

static inline float sl_vec3_length(sl_vec3 v)
{
    return sqrtf(sl_vec3_length_sq(v));
}

/* Unit vector along v, or zero when |v| < SL_EPSILON. */
static inline sl_vec3 sl_vec3_normalize(sl_vec3 v)
{
    const float length = sl_vec3_length(v);
    if (length < SL_EPSILON) {
        return sl_vec3_zero();
    }
    return sl_vec3_scale(v, 1.0f / length);
}

/* a at t = 0, b at t = 1; t is not clamped. */
static inline sl_vec3 sl_vec3_lerp(sl_vec3 a, sl_vec3 b, float t)
{
    return sl_vec3_mul_add(a, sl_vec3_sub(b, a), t);
}

static inline bool sl_vec3_is_finite(sl_vec3 v)
{
    return sl_is_finite(v.x) && sl_is_finite(v.y) && sl_is_finite(v.z);
}

/* 3x3 matrix stored as columns c0, c1, c2. */
typedef struct sl_mat3 {
    sl_vec3 c0, c1, c2;
} sl_mat3;

static inline sl_mat3 sl_mat3_make(sl_vec3 c0, sl_vec3 c1, sl_vec3 c2)
{
    sl_mat3 m = { c0, c1, c2 };
    return m;
}

static inline sl_mat3 sl_mat3_zero(void)
{
    return sl_mat3_make(sl_vec3_zero(), sl_vec3_zero(), sl_vec3_zero());
}

/* s * I */
static inline sl_mat3 sl_mat3_diagonal(float s)
{
    return sl_mat3_make(sl_vec3_make(s, 0.0f, 0.0f),
                        sl_vec3_make(0.0f, s, 0.0f),
                        sl_vec3_make(0.0f, 0.0f, s));
}

/* a * b^T */
static inline sl_mat3 sl_mat3_outer(sl_vec3 a, sl_vec3 b)
{
    return sl_mat3_make(sl_vec3_scale(a, b.x), sl_vec3_scale(a, b.y),
                        sl_vec3_scale(a, b.z));
}

static inline sl_mat3 sl_mat3_add(sl_mat3 a, sl_mat3 b)
{
    return sl_mat3_make(sl_vec3_add(a.c0, b.c0), sl_vec3_add(a.c1, b.c1),
                        sl_vec3_add(a.c2, b.c2));
}

static inline sl_mat3 sl_mat3_scale(sl_mat3 m, float s)
{
    return sl_mat3_make(sl_vec3_scale(m.c0, s), sl_vec3_scale(m.c1, s),
                        sl_vec3_scale(m.c2, s));
}

/* a + b * s */
static inline sl_mat3 sl_mat3_mul_add(sl_mat3 a, sl_mat3 b, float s)
{
    return sl_mat3_make(sl_vec3_mul_add(a.c0, b.c0, s),
                        sl_vec3_mul_add(a.c1, b.c1, s),
                        sl_vec3_mul_add(a.c2, b.c2, s));
}

static inline sl_vec3 sl_mat3_mul_vec3(sl_mat3 m, sl_vec3 v)
{
    sl_vec3 r = sl_vec3_scale(m.c0, v.x);
    r = sl_vec3_mul_add(r, m.c1, v.y);
    return sl_vec3_mul_add(r, m.c2, v.z);
}

static inline float sl_mat3_determinant(sl_mat3 m)
{
    return sl_vec3_dot(m.c0, sl_vec3_cross(m.c1, m.c2));
}

/* Solves m * x = b by Cramer's rule. Returns false, leaving *x untouched,
 * when |det(m)| <= det_min or the result is non-finite. det_min >= 0. */
static inline bool sl_mat3_solve(sl_mat3 m, sl_vec3 b, float det_min,
                                 sl_vec3 *x)
{
    SL_ASSERT(x != NULL);
    SL_ASSERT(det_min >= 0.0f);
    const float det = sl_mat3_determinant(m);
    if (!(sl_abs(det) > det_min)) {
        return false;
    }
    const float inv = 1.0f / det;
    const sl_vec3 r =
        sl_vec3_make(sl_vec3_dot(b, sl_vec3_cross(m.c1, m.c2)) * inv,
                     sl_vec3_dot(m.c0, sl_vec3_cross(b, m.c2)) * inv,
                     sl_vec3_dot(m.c0, sl_vec3_cross(m.c1, b)) * inv);
    if (!sl_vec3_is_finite(r)) {
        return false;
    }
    *x = r;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* SILK_MATH_H */
