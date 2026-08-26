#ifndef SILK_MATH_H
#define SILK_MATH_H

#include <math.h>
#include <stdbool.h>

#include "silk/assert.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SL_PI 3.14159265358979323846f

/* Comparison tolerance in length units (~8 ULP of 1.0f); pinned by
 * tests/test_math.c. */
#define SL_EPSILON 1e-6f
/* Normalize cutoff as squared length: vectors under SL_EPSILON in length
 * yield zero. */
#define SL_VEC2_LENGTH_EPS_SQ (SL_EPSILON * SL_EPSILON)

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

static inline float sl_vec2_distance_sq(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_length_sq(sl_vec2_sub(a, b));
}

static inline sl_vec2 sl_vec2_min(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_make(sl_min(a.x, b.x), sl_min(a.y, b.y));
}

static inline sl_vec2 sl_vec2_max(sl_vec2 a, sl_vec2 b)
{
    return sl_vec2_make(sl_max(a.x, b.x), sl_max(a.y, b.y));
}

/* Returns the unit vector of v. Inputs whose squared length falls below
 * SL_VEC2_LENGTH_EPS_SQ, or is not finite — infinite or NaN components,
 * or finite ones whose squares overflow — yield the zero vector instead
 * of NaN, a physics-safe default. */
static inline sl_vec2 sl_vec2_normalize(sl_vec2 v)
{
    float len_sq = sl_vec2_length_sq(v);
    if (!sl_is_finite(len_sq) || len_sq <= SL_VEC2_LENGTH_EPS_SQ) {
        return sl_vec2_make(0.0f, 0.0f);
    }
    return sl_vec2_scale(v, 1.0f / sqrtf(len_sq));
}

/* Counter-clockwise perpendicular: (-y, x). */
static inline sl_vec2 sl_vec2_perp(sl_vec2 v)
{
    return sl_vec2_make(-v.y, v.x);
}

static inline bool sl_vec2_is_finite(sl_vec2 v)
{
    return sl_is_finite(v.x) && sl_is_finite(v.y);
}

static inline sl_vec2 sl_vec2_lerp(sl_vec2 a, sl_vec2 b, float t)
{
    return sl_vec2_make(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}

/* Rotation stored as (cos, sin): applying it is four multiplies and no
 * trig, so hot loops rotate freely while the world integrates a single
 * scalar angle. c^2 + s^2 == 1 within rounding for anything built by
 * sl_rotation_make. */
typedef struct sl_rotation {
    float c, s;
} sl_rotation;

static inline sl_rotation sl_rotation_identity(void)
{
    sl_rotation q = { 1.0f, 0.0f };
    return q;
}

/* Counter-clockwise positive; libm trig, so cross-platform results
 * agree within tolerance rather than bitwise. */
static inline sl_rotation sl_rotation_make(float radians)
{
    sl_rotation q = { cosf(radians), sinf(radians) };
    return q;
}

static inline sl_vec2 sl_rotation_apply(sl_rotation q, sl_vec2 v)
{
    return sl_vec2_make(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/* Transpose equals inverse for rotations: rotates by the opposite
 * angle. */
static inline sl_vec2 sl_rotation_apply_inverse(sl_rotation q, sl_vec2 v)
{
    return sl_vec2_make(q.c * v.x + q.s * v.y, q.c * v.y - q.s * v.x);
}

static inline bool sl_rotation_is_finite(sl_rotation q)
{
    return sl_is_finite(q.c) && sl_is_finite(q.s);
}

/* Rigid body-to-world frame: world = rotation * local + position. */
typedef struct sl_transform {
    sl_vec2 position;
    sl_rotation rotation;
} sl_transform;

static inline sl_transform sl_transform_identity(void)
{
    sl_transform t;
    t.position = sl_vec2_make(0.0f, 0.0f);
    t.rotation = sl_rotation_identity();
    return t;
}

static inline sl_transform sl_transform_make(sl_vec2 position,
                                             sl_rotation rotation)
{
    sl_transform t;
    t.position = position;
    t.rotation = rotation;
    return t;
}

static inline sl_vec2 sl_transform_apply(sl_transform t, sl_vec2 local)
{
    return sl_vec2_add(t.position, sl_rotation_apply(t.rotation, local));
}

static inline sl_vec2 sl_transform_apply_inverse(sl_transform t, sl_vec2 world)
{
    return sl_rotation_apply_inverse(t.rotation,
                                     sl_vec2_sub(world, t.position));
}

static inline bool sl_transform_is_finite(sl_transform t)
{
    return sl_vec2_is_finite(t.position) && sl_rotation_is_finite(t.rotation);
}

/* Axis-aligned box in [lower, upper]; valid when finite and
 * lower <= upper per axis, so a point is a valid zero-area box. */
typedef struct sl_aabb {
    sl_vec2 lower;
    sl_vec2 upper;
} sl_aabb;

static inline sl_aabb sl_aabb_make(sl_vec2 lower, sl_vec2 upper)
{
    sl_aabb b;
    b.lower = lower;
    b.upper = upper;
    return b;
}

static inline bool sl_aabb_is_valid(sl_aabb b)
{
    return sl_vec2_is_finite(b.lower) && sl_vec2_is_finite(b.upper) &&
           b.lower.x <= b.upper.x && b.lower.y <= b.upper.y;
}

/* Boundary inclusive; NaN components read as outside. */
static inline bool sl_aabb_contains_point(sl_aabb b, sl_vec2 p)
{
    return p.x >= b.lower.x && p.x <= b.upper.x && p.y >= b.lower.y &&
           p.y <= b.upper.y;
}

/* Maps any finite angle into [-pi, pi], both bounds closed. Inputs
 * already inside pass through bitwise unchanged, so re-wrapping stored
 * state is stable; fmodf reduces first, so one +-2*pi correction covers
 * every finite input and non-finite input maps to NaN. */
static inline float sl_angle_wrap(float angle)
{
    float wrapped = fmodf(angle, 2.0f * SL_PI);
    if (wrapped > SL_PI) {
        wrapped -= 2.0f * SL_PI;
    } else if (wrapped < -SL_PI) {
        wrapped += 2.0f * SL_PI;
    }
    return wrapped;
}

/* Row-major storage, column-vector convention: out = M * v. */
typedef struct sl_mat2 {
    float m00, m01;
    float m10, m11;
} sl_mat2;

static inline sl_mat2 sl_mat2_identity(void)
{
    sl_mat2 m = { 1.0f, 0.0f, 0.0f, 1.0f };
    return m;
}

/* Counter-clockwise for positive angles; libm trig, so cross-platform
 * results agree within tolerance rather than bitwise. */
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

static inline sl_vec2 sl_mat2_multiply_vec2(sl_mat2 m, sl_vec2 v)
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
