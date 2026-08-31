#ifndef SILK_MATH_H
#define SILK_MATH_H

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "silk/assert.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SL_PI 3.14159265358979323846f

/* Near-unit absolute comparison and normalization tolerance (~8 ULP of
 * 1.0f); not a relative comparison or a collision/contact tolerance.
 * Pinned by tests/test_math.c. */
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

/* Clockwise perpendicular: (y, -x). */
static inline sl_vec2 sl_vec2_right_perp(sl_vec2 v)
{
    return sl_vec2_make(v.y, -v.x);
}

/* 2D forms of the 3D cross products (0, 0, scalar) x vector and
 * vector x (0, 0, scalar), respectively. The operand order is part of
 * the name because reversing a cross product reverses its sign. */
static inline sl_vec2 sl_scalar_cross_vec2(float scalar, sl_vec2 vector)
{
    return sl_vec2_scale(sl_vec2_perp(vector), scalar);
}

static inline sl_vec2 sl_vec2_cross_scalar(sl_vec2 vector, float scalar)
{
    return sl_vec2_scale(sl_vec2_right_perp(vector), scalar);
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
 * trig, so hot loops rotate freely while the world integrates this pair
 * directly. c^2 + s^2 == 1 within rounding for anything built by
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

/* Composition a * b applies b first, then a. */
static inline sl_rotation sl_rotation_mul(sl_rotation a, sl_rotation b)
{
    sl_rotation q = { a.c * b.c - a.s * b.s, a.s * b.c + a.c * b.s };
    return q;
}

/* Principal angle in [-pi, pi] for a finite, non-degenerate rotation. */
static inline float sl_rotation_angle(sl_rotation q)
{
    return atan2f(q.s, q.c);
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

/* Scale-safe unit normalization. Invalid rotations and rotations whose
 * largest component is no greater than SL_EPSILON become identity. */
static inline sl_rotation sl_rotation_normalize(sl_rotation q)
{
    const float scale = sl_max(sl_abs(q.c), sl_abs(q.s));
    if (!sl_is_finite(scale) || scale <= SL_EPSILON) {
        return sl_rotation_identity();
    }

    q.c /= scale;
    q.s /= scale;
    const float length = sqrtf(q.c * q.c + q.s * q.s);
    if (!sl_is_finite(length) || length <= SL_EPSILON) {
        return sl_rotation_identity();
    }

    q.c /= length;
    q.s /= length;
    return q;
}

/* Normalized small-angle update q + delta_angle * perp(q). For a unit
 * input, the angular advance is atan(delta_angle). A non-finite delta
 * leaves a valid input unchanged. The scale-safe step rotation keeps a
 * huge finite delta finite instead of overflowing the candidate. */
static inline sl_rotation sl_rotation_integrate(sl_rotation q,
                                                float delta_angle)
{
    if (!sl_is_finite(delta_angle)) {
        return q;
    }

    const float input_scale = sl_max(sl_abs(q.c), sl_abs(q.s));
    if (!sl_is_finite(input_scale) || input_scale <= SL_EPSILON) {
        return sl_rotation_identity();
    }

    if (delta_angle == 0.0f) {
        return q;
    }

    /* Dividing the complete q + delta * perp(q) candidate by a common
     * positive scale does not change its normalized result. It does keep
     * multiplication by a huge finite delta inside float range. */
    const float delta_scale = sl_max(1.0f, sl_abs(delta_angle));
    const float inverse_scale = 1.0f / delta_scale;
    const float scaled_delta = delta_angle * inverse_scale;
    const sl_rotation candidate = {
        q.c * inverse_scale - q.s * scaled_delta,
        q.s * inverse_scale + q.c * scaled_delta,
    };
    const float candidate_scale =
        sl_max(sl_abs(candidate.c), sl_abs(candidate.s));
    if (!sl_rotation_is_finite(candidate) || candidate_scale <= SL_EPSILON) {
        return q;
    }
    return sl_rotation_normalize(candidate);
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

/* Boundary contact counts as overlap. Invalid boxes never overlap. */
static inline bool sl_aabb_overlaps(sl_aabb a, sl_aabb b)
{
    if (!sl_aabb_is_valid(a) || !sl_aabb_is_valid(b)) {
        return false;
    }
    return a.lower.x <= b.upper.x && a.upper.x >= b.lower.x &&
           a.lower.y <= b.upper.y && a.upper.y >= b.lower.y;
}

/* Boundary inclusive: true when the complete inner box lies in outer. */
static inline bool sl_aabb_contains(sl_aabb outer, sl_aabb inner)
{
    if (!sl_aabb_is_valid(outer) || !sl_aabb_is_valid(inner)) {
        return false;
    }
    return outer.lower.x <= inner.lower.x && outer.lower.y <= inner.lower.y &&
           outer.upper.x >= inner.upper.x && outer.upper.y >= inner.upper.y;
}

/* Smallest valid AABB containing both valid inputs. */
static inline sl_aabb sl_aabb_union(sl_aabb a, sl_aabb b)
{
    SL_ASSERT(sl_aabb_is_valid(a));
    SL_ASSERT(sl_aabb_is_valid(b));
    return sl_aabb_make(sl_vec2_min(a.lower, b.lower),
                        sl_vec2_max(a.upper, b.upper));
}

/* Twice the sum of side lengths. Valid finite bounds can span more than
 * FLT_MAX; those boxes saturate to keep broad-phase cost values finite. */
static inline float sl_aabb_perimeter(sl_aabb b)
{
    if (!sl_aabb_is_valid(b)) {
        return 0.0f;
    }
    const double width = (double)b.upper.x - (double)b.lower.x;
    const double height = (double)b.upper.y - (double)b.lower.y;
    const double perimeter = 2.0 * (width + height);
    return (perimeter <= (double)FLT_MAX) ? (float)perimeter : FLT_MAX;
}

/* Expands a valid box uniformly. A negative/non-finite margin or an
 * unrepresentable bound leaves the input unchanged. */
static inline sl_aabb sl_aabb_extend(sl_aabb b, float margin)
{
    if (!sl_aabb_is_valid(b) || !sl_is_finite(margin) || margin < 0.0f) {
        return b;
    }

    const double lower_x = (double)b.lower.x - (double)margin;
    const double lower_y = (double)b.lower.y - (double)margin;
    const double upper_x = (double)b.upper.x + (double)margin;
    const double upper_y = (double)b.upper.y + (double)margin;
    if (lower_x < -(double)FLT_MAX || lower_y < -(double)FLT_MAX ||
        upper_x > (double)FLT_MAX || upper_y > (double)FLT_MAX) {
        return b;
    }
    return sl_aabb_make(sl_vec2_make((float)lower_x, (float)lower_y),
                        sl_vec2_make((float)upper_x, (float)upper_y));
}

typedef struct sl_segment_distance_result {
    sl_vec2 point_a;
    sl_vec2 point_b;
    float fraction_a;
    float fraction_b;
    float distance_sq;
} sl_segment_distance_result;

/* Closest points between finite segments [a1, a2] and [b1, b2]. Segments
 * no longer than SL_EPSILON are treated as points. Near-parallel segments
 * choose fraction_a == 0 deterministically before clamping fraction_b.
 * Returns false without changing out if input or any result is not finite
 * and representable as float. */
static inline bool sl_segment_distance(sl_vec2 a1, sl_vec2 a2, sl_vec2 b1,
                                       sl_vec2 b2,
                                       sl_segment_distance_result *out)
{
    if (out == NULL || !sl_vec2_is_finite(a1) || !sl_vec2_is_finite(a2) ||
        !sl_vec2_is_finite(b1) || !sl_vec2_is_finite(b2)) {
        return false;
    }

    const double d1_x = (double)a2.x - (double)a1.x;
    const double d1_y = (double)a2.y - (double)a1.y;
    const double d2_x = (double)b2.x - (double)b1.x;
    const double d2_y = (double)b2.y - (double)b1.y;
    const double r_x = (double)a1.x - (double)b1.x;
    const double r_y = (double)a1.y - (double)b1.y;
    const double a = d1_x * d1_x + d1_y * d1_y;
    const double e = d2_x * d2_x + d2_y * d2_y;
    const double f = d2_x * r_x + d2_y * r_y;
    const double length_eps_sq = (double)SL_VEC2_LENGTH_EPS_SQ;
    double fraction_a = 0.0;
    double fraction_b = 0.0;

    if (a <= length_eps_sq && e <= length_eps_sq) {
        fraction_a = 0.0;
        fraction_b = 0.0;
    } else if (a <= length_eps_sq) {
        fraction_b = f / e;
        fraction_b = (fraction_b < 0.0)   ? 0.0
                     : (fraction_b > 1.0) ? 1.0
                                          : fraction_b;
    } else {
        const double c = d1_x * r_x + d1_y * r_y;
        if (e <= length_eps_sq) {
            fraction_a = -c / a;
            fraction_a = (fraction_a < 0.0)   ? 0.0
                         : (fraction_a > 1.0) ? 1.0
                                              : fraction_a;
        } else {
            const double b = d1_x * d2_x + d1_y * d2_y;
            const double denominator = a * e - b * b;
            if (denominator > (double)FLT_EPSILON * a * e) {
                fraction_a = (b * f - c * e) / denominator;
                fraction_a = (fraction_a < 0.0)   ? 0.0
                             : (fraction_a > 1.0) ? 1.0
                                                  : fraction_a;
            }

            fraction_b = (b * fraction_a + f) / e;
            if (fraction_b < 0.0) {
                fraction_b = 0.0;
                fraction_a = -c / a;
                fraction_a = (fraction_a < 0.0)   ? 0.0
                             : (fraction_a > 1.0) ? 1.0
                                                  : fraction_a;
            } else if (fraction_b > 1.0) {
                fraction_b = 1.0;
                fraction_a = (b - c) / a;
                fraction_a = (fraction_a < 0.0)   ? 0.0
                             : (fraction_a > 1.0) ? 1.0
                                                  : fraction_a;
            }
        }
    }

    const double point_a_x = (double)a1.x + fraction_a * d1_x;
    const double point_a_y = (double)a1.y + fraction_a * d1_y;
    const double point_b_x = (double)b1.x + fraction_b * d2_x;
    const double point_b_y = (double)b1.y + fraction_b * d2_y;
    const double separation_x = point_b_x - point_a_x;
    const double separation_y = point_b_y - point_a_y;
    const double distance_sq =
        separation_x * separation_x + separation_y * separation_y;
    if (!isfinite(point_a_x) || !isfinite(point_a_y) || !isfinite(point_b_x) ||
        !isfinite(point_b_y) || !isfinite(distance_sq) ||
        point_a_x < -(double)FLT_MAX || point_a_x > (double)FLT_MAX ||
        point_a_y < -(double)FLT_MAX || point_a_y > (double)FLT_MAX ||
        point_b_x < -(double)FLT_MAX || point_b_x > (double)FLT_MAX ||
        point_b_y < -(double)FLT_MAX || point_b_y > (double)FLT_MAX ||
        distance_sq > (double)FLT_MAX) {
        return false;
    }

    const sl_segment_distance_result candidate = {
        .point_a = sl_vec2_make((float)point_a_x, (float)point_a_y),
        .point_b = sl_vec2_make((float)point_b_x, (float)point_b_y),
        .fraction_a = (float)fraction_a,
        .fraction_b = (float)fraction_b,
        .distance_sq = (float)distance_sq,
    };
    *out = candidate;
    return true;
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

/* Solves matrix * x == b. A matrix is singular when
 * |det(matrix)| <= FLT_EPSILON * max(|matrix_ij|)^2. Double
 * intermediates keep the criterion scale-relative for finite float input.
 * Returns false without changing out for singular input or a solution that
 * is not finite and representable as float. */
static inline bool sl_mat2_solve(sl_mat2 matrix, sl_vec2 b, sl_vec2 *out)
{
    if (out == NULL || !sl_vec2_is_finite(b) || !sl_is_finite(matrix.m00) ||
        !sl_is_finite(matrix.m01) || !sl_is_finite(matrix.m10) ||
        !sl_is_finite(matrix.m11)) {
        return false;
    }

    double scale = (double)sl_abs(matrix.m00);
    scale = ((double)sl_abs(matrix.m01) > scale) ? (double)sl_abs(matrix.m01)
                                                 : scale;
    scale = ((double)sl_abs(matrix.m10) > scale) ? (double)sl_abs(matrix.m10)
                                                 : scale;
    scale = ((double)sl_abs(matrix.m11) > scale) ? (double)sl_abs(matrix.m11)
                                                 : scale;
    if (scale == 0.0) {
        return false;
    }

    const double determinant = (double)matrix.m00 * (double)matrix.m11 -
                               (double)matrix.m01 * (double)matrix.m10;
    if (fabs(determinant) <= (double)FLT_EPSILON * scale * scale) {
        return false;
    }

    const double x =
        ((double)b.x * (double)matrix.m11 - (double)matrix.m01 * (double)b.y) /
        determinant;
    const double y =
        ((double)matrix.m00 * (double)b.y - (double)b.x * (double)matrix.m10) /
        determinant;
    if (!isfinite(x) || !isfinite(y) || x < -(double)FLT_MAX ||
        x > (double)FLT_MAX || y < -(double)FLT_MAX || y > (double)FLT_MAX) {
        return false;
    }

    *out = sl_vec2_make((float)x, (float)y);
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* SILK_MATH_H */
