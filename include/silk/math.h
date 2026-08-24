#ifndef SILK_MATH_H
#define SILK_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>

#define SL_PI 3.14159265358979323846f
#define SL_EPSILON 1e-6f

/* ------------------------------------------------------------------ */
/* Scalar utilities                                                    */
/* ------------------------------------------------------------------ */

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
    return sl_min(sl_max(v, lo), hi);
}

static inline float sl_abs(float v)
{
    return fabsf(v);
}

/* True when |a - b| <= eps. */
static inline bool sl_feq(float a, float b, float eps)
{
    return sl_abs(a - b) <= eps;
}

/* ------------------------------------------------------------------ */
/* 2D vectors                                                          */
/* ------------------------------------------------------------------ */

typedef struct sl_vec2 {
    float x, y;
} sl_vec2;

static inline sl_vec2 sl_vec2_make(float x, float y)
{
    sl_vec2 v = { x, y };
    return v;
}

static inline sl_vec2 sl_vec2_add(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_make(a.x + b.x, a.y + b.y);
}

static inline sl_vec2 sl_vec2_sub(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_make(a.x - b.x, a.y - b.y);
}

static inline sl_vec2 sl_vec2_scale(sl_vec2 v, float s)
{
    return sl_vec2_make(v.x * s, v.y * s);
}

static inline sl_vec2 sl_vec2_neg(sl_vec2 v)
{
    return sl_vec2_make(-v.x, -v.y);
}

static inline float sl_vec2_dot(sl_vec2 a, sl_vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

/* Scalar (z-component) of the 3D cross product a x b.
 * Positive when b is counter-clockwise from a. */
static inline float sl_vec2_cross(sl_vec2 a, sl_vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

static inline float sl_vec2_length_sq(sl_vec2 v)
{
    return sl_vec2_dot(v, v);
}

static inline float sl_vec2_length(sl_vec2 v)
{
    return sqrtf(sl_vec2_length_sq(v));
}

static inline float sl_vec2_distance(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_length(sl_vec2_sub(a, b));
}

/* Returns the unit vector of v. Zero-length input yields the zero vector
 * instead of NaN — a physics-safe default. The threshold compares squared
 * length against SL_EPSILON, so it is not scale-invariant by design. */
static inline sl_vec2 sl_vec2_normalize(sl_vec2 v)
{
    float len_sq = sl_vec2_length_sq(v);
    if (len_sq <= SL_EPSILON) {
        return sl_vec2_make(0.0f, 0.0f);
    }
    return sl_vec2_scale(v, 1.0f / sqrtf(len_sq));
}

/* Counter-clockwise perpendicular: (-y, x). */
static inline sl_vec2 sl_vec2_perp(sl_vec2 v)
{
    return sl_vec2_make(-v.y, v.x);
}

static inline sl_vec2 sl_vec2_lerp(sl_vec2 a, sl_vec2 b, float t)
{
    return sl_vec2_make(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}

/* ------------------------------------------------------------------ */
/* 2x2 matrices (row-major storage, column-vector convention)          */
/*                                                                     */
/*     | m00  m01 |   applied as: out = M * v                          */
/*     | m10  m11 |                                                    */
/* ------------------------------------------------------------------ */

typedef struct sl_mat2 {
    float m00, m01;
    float m10, m11;
} sl_mat2;

static inline sl_mat2 sl_mat2_identity(void)
{
    sl_mat2 m = { 1.0f, 0.0f, 0.0f, 1.0f };
    return m;
}

/* Counter-clockwise rotation matrix for positive angles. */
static inline sl_mat2 sl_mat2_rotation(float radians)
{
    float c = cosf(radians);
    float s = sinf(radians);
    sl_mat2 m = { c, -s, s, c };
    return m;
}

static inline sl_mat2 sl_mat2_multiply(sl_mat2 a, sl_mat2 b)
{
    sl_mat2 r;
    r.m00 = a.m00 * b.m00 + a.m01 * b.m10;
    r.m01 = a.m00 * b.m01 + a.m01 * b.m11;
    r.m10 = a.m10 * b.m00 + a.m11 * b.m10;
    r.m11 = a.m10 * b.m01 + a.m11 * b.m11;
    return r;
}

static inline sl_vec2 sl_mat2_mul_vec2(sl_mat2 m, sl_vec2 v)
{
    return sl_vec2_make(m.m00 * v.x + m.m01 * v.y, m.m10 * v.x + m.m11 * v.y);
}

static inline sl_mat2 sl_mat2_transpose(sl_mat2 m)
{
    sl_mat2 r = { m.m00, m.m10, m.m01, m.m11 };
    return r;
}

#ifdef __cplusplus
}
#endif

#endif /* SILK_MATH_H */
