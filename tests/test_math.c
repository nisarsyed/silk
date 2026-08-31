#include "silk_test.h"
#include "suites.h"
#include <float.h>
#include <math.h>
#include <stdint.h>

#include <silk/math.h>

/* Exact equality is used only where IEEE-754 guarantees it (construction,
 * integer-valued arithmetic, and small-integer matrix products); everything
 * flowing through sqrt/trig compares via SL_EXPECT_NEAR. Rotation cases
 * need the looser epsilon because SL_PI / 2 rounds by up to ~3e-8 rad, so
 * cos/sin land within ~1e-7 of 0 and 1 — 1e-6f leaves ~10x headroom. */
#define SL_TEST_EPS 1e-6f

static void test_scalar_min_max(void)
{
    SL_EXPECT(sl_min(1.0f, 2.0f) == 1.0f);
    SL_EXPECT(sl_min(-1.0f, -2.0f) == -2.0f);
    SL_EXPECT(sl_max(1.0f, 2.0f) == 2.0f);
}

static void test_scalar_clamp(void)
{
    SL_EXPECT(sl_clamp(0.5f, 0.0f, 1.0f) == 0.5f);
    SL_EXPECT(sl_clamp(-1.0f, 0.0f, 1.0f) == 0.0f);
    SL_EXPECT(sl_clamp(2.0f, 0.0f, 1.0f) == 1.0f);
}

static void test_scalar_feq(void)
{
    SL_EXPECT(sl_feq(1.0f, 1.0f + 1e-7f, SL_EPSILON));
    SL_EXPECT(!sl_feq(1.0f, 1.1f, SL_EPSILON));
}

static void test_vec2_make_and_components(void)
{
    sl_vec2 v = sl_vec2_make(3.0f, 4.0f);
    SL_EXPECT(v.x == 3.0f && v.y == 4.0f);
}

static void test_vec2_add_sub_neg_scale(void)
{
    sl_vec2 a = sl_vec2_make(1.0f, 2.0f);
    sl_vec2 b = sl_vec2_make(3.0f, -4.0f);

    sl_vec2 sum = sl_vec2_add(a, b);
    SL_EXPECT(sum.x == 4.0f && sum.y == -2.0f);

    sl_vec2 diff = sl_vec2_sub(a, b);
    SL_EXPECT(diff.x == -2.0f && diff.y == 6.0f);

    sl_vec2 neg = sl_vec2_neg(a);
    SL_EXPECT(neg.x == -1.0f && neg.y == -2.0f);

    sl_vec2 scaled = sl_vec2_scale(b, 2.0f);
    SL_EXPECT(scaled.x == 6.0f && scaled.y == -8.0f);
}

static void test_vec2_dot(void)
{
    /* Perpendicular vectors have zero dot product. */
    sl_vec2 x = sl_vec2_make(1.0f, 0.0f);
    sl_vec2 y = sl_vec2_make(0.0f, 1.0f);
    SL_EXPECT_NEAR(sl_vec2_dot(x, y), 0.0f, SL_EPSILON);

    /* Parallel: |a||b| when same direction. */
    sl_vec2 a = sl_vec2_make(3.0f, 0.0f);
    SL_EXPECT_NEAR(sl_vec2_dot(a, a), 9.0f, SL_EPSILON);
}

static void test_vec2_cross_sign(void)
{
    sl_vec2 x = sl_vec2_make(1.0f, 0.0f);
    sl_vec2 y = sl_vec2_make(0.0f, 1.0f);
    SL_EXPECT(sl_vec2_cross(x, y) > 0.0f); /* CCW */
    SL_EXPECT(sl_vec2_cross(y, x) < 0.0f); /* CW */
    SL_EXPECT_NEAR(sl_vec2_cross(x, y), 1.0f, SL_EPSILON);
    SL_EXPECT_NEAR(sl_vec2_cross(y, x), -1.0f, SL_EPSILON);
}

static void test_vec2_length_normalize(void)
{
    sl_vec2 v = sl_vec2_make(3.0f, 4.0f);
    SL_EXPECT_NEAR(sl_vec2_length(v), 5.0f, SL_EPSILON);

    sl_vec2 unit = sl_vec2_normalize(v);
    SL_EXPECT_NEAR(unit.x, 0.6f, SL_TEST_EPS);
    SL_EXPECT_NEAR(unit.y, 0.8f, SL_TEST_EPS);
    SL_EXPECT_NEAR(sl_vec2_length(unit), 1.0f, SL_TEST_EPS);

    /* Zero-safe normalize. */
    sl_vec2 zero = sl_vec2_normalize(sl_vec2_make(0.0f, 0.0f));
    SL_EXPECT(zero.x == 0.0f && zero.y == 0.0f);

    /* Squared length overflows to infinity: still the zero vector.
     * Regression pin — this held before the finiteness guard via
     * v * (1 / inf), and must keep holding now it is explicit. */
    sl_vec2 huge = sl_vec2_normalize(sl_vec2_make(3e19f, 1e19f));
    SL_EXPECT(huge.x == 0.0f && huge.y == 0.0f);

    /* Non-finite components used to produce NaN (inf * 0): now zero. */
    sl_vec2 infinite = sl_vec2_normalize(sl_vec2_make(INFINITY, 0.0f));
    SL_EXPECT(infinite.x == 0.0f && infinite.y == 0.0f);
    sl_vec2 nan_input = sl_vec2_normalize(sl_vec2_make(NAN, 1.0f));
    SL_EXPECT(nan_input.x == 0.0f && nan_input.y == 0.0f);
}

static void test_vec2_perp_is_orthogonal_ccw(void)
{
    sl_vec2 v = sl_vec2_make(2.0f, 1.0f);
    sl_vec2 p = sl_vec2_perp(v);
    SL_EXPECT(p.x == -1.0f && p.y == 2.0f);
    SL_EXPECT_NEAR(sl_vec2_dot(v, p), 0.0f, SL_EPSILON);
}

static void test_vec2_right_perp_and_scalar_cross_order(void)
{
    const sl_vec2 v = sl_vec2_make(2.0f, 1.0f);
    const sl_vec2 right = sl_vec2_right_perp(v);
    SL_EXPECT(right.x == 1.0f && right.y == -2.0f);
    SL_EXPECT(sl_vec2_dot(v, right) == 0.0f);

    const sl_vec2 scalar_vector = sl_scalar_cross_vec2(3.0f, v);
    const sl_vec2 vector_scalar = sl_vec2_cross_scalar(v, 3.0f);
    SL_EXPECT(scalar_vector.x == -3.0f && scalar_vector.y == 6.0f);
    SL_EXPECT(vector_scalar.x == 3.0f && vector_scalar.y == -6.0f);
}

static void test_vec2_lerp_endpoints(void)
{
    sl_vec2 a = sl_vec2_make(0.0f, 0.0f);
    sl_vec2 b = sl_vec2_make(10.0f, -10.0f);

    sl_vec2 at_a = sl_vec2_lerp(a, b, 0.0f);
    sl_vec2 mid = sl_vec2_lerp(a, b, 0.5f);
    sl_vec2 at_b = sl_vec2_lerp(a, b, 1.0f);

    SL_EXPECT(at_a.x == 0.0f && at_a.y == 0.0f);
    SL_EXPECT(mid.x == 5.0f && mid.y == -5.0f);
    SL_EXPECT(at_b.x == 10.0f && at_b.y == -10.0f);
}

static void test_vec2_distance(void)
{
    float d =
        sl_vec2_distance(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(3.0f, 4.0f));
    SL_EXPECT_NEAR(d, 5.0f, SL_EPSILON);
}

static void test_vec2_min_max_componentwise(void)
{
    sl_vec2 a = sl_vec2_make(1.0f, -5.0f);
    sl_vec2 b = sl_vec2_make(-2.0f, 3.0f);

    sl_vec2 lo = sl_vec2_min(a, b);
    SL_EXPECT(lo.x == -2.0f && lo.y == -5.0f);

    sl_vec2 hi = sl_vec2_max(a, b);
    SL_EXPECT(hi.x == 1.0f && hi.y == 3.0f);

    /* Equal inputs pass through unchanged. */
    sl_vec2 same = sl_vec2_max(a, a);
    SL_EXPECT(same.x == 1.0f && same.y == -5.0f);
}

static void test_vec2_distance_sq_matches_distance(void)
{
    /* Integer 3-4-5 triangle: exact in binary floating point. */
    sl_vec2 a = sl_vec2_make(0.0f, 0.0f);
    sl_vec2 b = sl_vec2_make(3.0f, 4.0f);
    SL_EXPECT(sl_vec2_distance_sq(a, b) == 25.0f);
    SL_EXPECT(sl_vec2_distance_sq(b, a) == 25.0f);
    SL_EXPECT(sl_vec2_distance_sq(a, a) == 0.0f);

    const float sq = sl_vec2_distance_sq(a, b);
    SL_EXPECT_NEAR(sl_vec2_length(sl_vec2_sub(b, a)), sqrtf(sq), SL_EPSILON);
}

static void test_rotation_identity_is_exact(void)
{
    sl_rotation q = sl_rotation_identity();
    SL_EXPECT(q.c == 1.0f && q.s == 0.0f);

    /* Applying identity reproduces the input bitwise: c*x - s*y with
     * (c, s) = (1, 0) is x - 0, exact for every finite v. */
    sl_vec2 v = sl_vec2_make(-13.5f, 4.25f);
    sl_vec2 r = sl_rotation_apply(q, v);
    SL_EXPECT(r.x == v.x && r.y == v.y);
}

/* Both constructors call cosf/sinf on the same argument, so their
 * coefficients must agree bitwise; anything else means one of them
 * stopped being the rotation it claims to be. */
static void test_rotation_make_matches_mat2_rotation(void)
{
    const float angles[] = { 0.7f, -2.1f, SL_PI / 2.0f, SL_PI };
    for (uint32_t i = 0u; i < sizeof(angles) / sizeof(angles[0]); ++i) {
        sl_rotation q = sl_rotation_make(angles[i]);
        sl_mat2 m = sl_mat2_rotation(angles[i]);
        SL_EXPECT(q.c == m.m00 && q.s == m.m10);
    }

    /* Quarter turn sends +x to +y. */
    sl_rotation quarter = sl_rotation_make(SL_PI / 2.0f);
    sl_vec2 r = sl_rotation_apply(quarter, sl_vec2_make(1.0f, 0.0f));
    SL_EXPECT_NEAR(r.x, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(r.y, 1.0f, SL_TEST_EPS);
}

static void test_rotation_apply_inverse_roundtrip(void)
{
    sl_rotation q = sl_rotation_make(0.7f);

    /* Unit length within rounding, per the type contract. */
    SL_EXPECT_NEAR(q.c * q.c + q.s * q.s, 1.0f, SL_TEST_EPS);

    sl_vec2 v = sl_vec2_make(3.0f, -2.0f);
    sl_vec2 back = sl_rotation_apply_inverse(q, sl_rotation_apply(q, v));
    SL_EXPECT_NEAR(back.x, v.x, SL_TEST_EPS);
    SL_EXPECT_NEAR(back.y, v.y, SL_TEST_EPS);

    /* The inverse alone acts as the opposite-angle rotation. */
    sl_vec2 inv_only = sl_rotation_apply_inverse(q, sl_vec2_make(1.0f, 0.0f));
    sl_rotation negated = sl_rotation_make(-0.7f);
    SL_EXPECT_NEAR(inv_only.x, negated.c, SL_TEST_EPS);
    SL_EXPECT_NEAR(inv_only.y, negated.s, SL_TEST_EPS);
}

static void test_rotation_mul_composes_and_reports_angle(void)
{
    const sl_rotation a = sl_rotation_make(0.3f);
    const sl_rotation b = sl_rotation_make(-0.8f);
    const sl_rotation product = sl_rotation_mul(a, b);
    const sl_rotation expected = sl_rotation_make(-0.5f);

    SL_EXPECT_NEAR(product.c, expected.c, SL_TEST_EPS);
    SL_EXPECT_NEAR(product.s, expected.s, SL_TEST_EPS);
    SL_EXPECT_NEAR(sl_rotation_angle(product), -0.5f, SL_TEST_EPS);

    const sl_vec2 v = sl_vec2_make(2.0f, -3.0f);
    const sl_vec2 composed = sl_rotation_apply(product, v);
    const sl_vec2 sequential = sl_rotation_apply(a, sl_rotation_apply(b, v));
    SL_EXPECT_NEAR(composed.x, sequential.x, SL_TEST_EPS);
    SL_EXPECT_NEAR(composed.y, sequential.y, SL_TEST_EPS);
}

static void test_rotation_normalize_is_scale_safe(void)
{
    const sl_rotation huge =
        sl_rotation_normalize((sl_rotation){ .c = FLT_MAX, .s = FLT_MAX });
    SL_EXPECT(sl_rotation_is_finite(huge));
    SL_EXPECT_NEAR(huge.c * huge.c + huge.s * huge.s, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(huge.c, sqrtf(0.5f), SL_TEST_EPS);
    SL_EXPECT_NEAR(huge.s, sqrtf(0.5f), SL_TEST_EPS);

    const sl_rotation unit = sl_rotation_make(0.7f);
    const sl_rotation normalized = sl_rotation_normalize(unit);
    SL_EXPECT_NEAR(normalized.c, unit.c, SL_TEST_EPS);
    SL_EXPECT_NEAR(normalized.s, unit.s, SL_TEST_EPS);

    const sl_rotation zero =
        sl_rotation_normalize((sl_rotation){ .c = 0.0f, .s = 0.0f });
    const sl_rotation infinite =
        sl_rotation_normalize((sl_rotation){ .c = INFINITY, .s = 1.0f });
    SL_EXPECT(zero.c == 1.0f && zero.s == 0.0f);
    SL_EXPECT(infinite.c == 1.0f && infinite.s == 0.0f);
}

static void test_rotation_integrate_small_and_extreme_deltas(void)
{
    const sl_rotation identity = sl_rotation_identity();
    const sl_rotation unchanged = sl_rotation_integrate(identity, 0.0f);
    SL_EXPECT(unchanged.c == identity.c && unchanged.s == identity.s);

    const sl_rotation integrated = sl_rotation_integrate(identity, 0.1f);
    const sl_rotation exact = sl_rotation_make(0.1f);
    SL_EXPECT_NEAR(integrated.c, exact.c, 1e-3f);
    SL_EXPECT_NEAR(integrated.s, exact.s, 1e-3f);
    SL_EXPECT_NEAR(sl_rotation_angle(integrated), atanf(0.1f), SL_TEST_EPS);

    const sl_rotation initial = sl_rotation_make(0.3f);
    const sl_rotation huge = sl_rotation_integrate(initial, FLT_MAX);
    SL_EXPECT(sl_rotation_is_finite(huge));
    SL_EXPECT_NEAR(huge.c * huge.c + huge.s * huge.s, 1.0f, SL_TEST_EPS);
    const sl_rotation quarter =
        sl_rotation_mul(initial, sl_rotation_make(0.5f * SL_PI));
    SL_EXPECT_NEAR(huge.c, quarter.c, SL_TEST_EPS);
    SL_EXPECT_NEAR(huge.s, quarter.s, SL_TEST_EPS);

    const sl_rotation nan_delta = sl_rotation_integrate(initial, NAN);
    const sl_rotation inf_delta = sl_rotation_integrate(initial, INFINITY);
    SL_EXPECT(nan_delta.c == initial.c && nan_delta.s == initial.s);
    SL_EXPECT(inf_delta.c == initial.c && inf_delta.s == initial.s);
}

/* Operand-order pin for the frame: rotate-then-translate. A
 * translate-first mistake turns local (1, 0) at position (10, 0) by a
 * quarter turn into ~(11, 0) instead of the correct ~(10, 1). */
static void test_transform_apply_rotates_then_translates(void)
{
    sl_transform t = sl_transform_make(sl_vec2_make(10.0f, 0.0f),
                                       sl_rotation_make(SL_PI / 2.0f));

    sl_vec2 world = sl_transform_apply(t, sl_vec2_make(1.0f, 0.0f));
    SL_EXPECT_NEAR(world.x, 10.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(world.y, 1.0f, SL_TEST_EPS);
}

static void test_transform_apply_inverse_roundtrip(void)
{
    sl_transform t =
        sl_transform_make(sl_vec2_make(10.0f, -20.0f), sl_rotation_make(0.7f));

    sl_vec2 world = sl_transform_apply(t, sl_vec2_make(3.0f, -2.0f));
    sl_vec2 local = sl_transform_apply_inverse(t, world);
    SL_EXPECT_NEAR(local.x, 3.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(local.y, -2.0f, SL_TEST_EPS);

    /* Identity transforms round-trip exactly. */
    sl_transform id = sl_transform_identity();
    sl_vec2 v = sl_vec2_make(1.5f, -2.5f);
    sl_vec2 out = sl_transform_apply(id, v);
    SL_EXPECT(out.x == v.x && out.y == v.y);
    sl_vec2 back = sl_transform_apply_inverse(id, v);
    SL_EXPECT(back.x == v.x && back.y == v.y);
}

static void test_aabb_contains_point_boundary_inclusive(void)
{
    sl_aabb box =
        sl_aabb_make(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(10.0f, 10.0f));

    /* All four corners and the center lie inside. */
    SL_EXPECT(sl_aabb_contains_point(box, sl_vec2_make(0.0f, 0.0f)));
    SL_EXPECT(sl_aabb_contains_point(box, sl_vec2_make(10.0f, 10.0f)));
    SL_EXPECT(sl_aabb_contains_point(box, sl_vec2_make(0.0f, 10.0f)));
    SL_EXPECT(sl_aabb_contains_point(box, sl_vec2_make(10.0f, 0.0f)));
    SL_EXPECT(sl_aabb_contains_point(box, sl_vec2_make(5.0f, 5.0f)));

    /* Just outside each of two edges. */
    SL_EXPECT(!sl_aabb_contains_point(box, sl_vec2_make(-0.001f, 5.0f)));
    SL_EXPECT(!sl_aabb_contains_point(box, sl_vec2_make(5.0f, 10.001f)));

    /* NaN components read as outside. */
    SL_EXPECT(!sl_aabb_contains_point(box, sl_vec2_make(NAN, 5.0f)));
    SL_EXPECT(!sl_aabb_contains_point(box, sl_vec2_make(5.0f, NAN)));
}

static void test_aabb_is_valid_rejects_inverted_or_non_finite(void)
{
    SL_EXPECT(sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(-1.0f, -2.0f), sl_vec2_make(3.0f, 4.0f))));
    /* A point is a valid zero-area box. */
    SL_EXPECT(sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(1.0f, 1.0f), sl_vec2_make(1.0f, 1.0f))));

    /* Inverted on one axis each. */
    SL_EXPECT(!sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(3.0f, 0.0f), sl_vec2_make(-3.0f, 4.0f))));
    SL_EXPECT(!sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(0.0f, 4.0f), sl_vec2_make(3.0f, -4.0f))));

    /* Non-finite bounds. */
    SL_EXPECT(!sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(NAN, 0.0f), sl_vec2_make(3.0f, 4.0f))));
    SL_EXPECT(!sl_aabb_is_valid(
        sl_aabb_make(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(INFINITY, 4.0f))));
}

static void test_aabb_set_operations_are_boundary_inclusive(void)
{
    const sl_aabb a =
        sl_aabb_make(sl_vec2_make(-2.0f, -1.0f), sl_vec2_make(2.0f, 3.0f));
    const sl_aabb touching =
        sl_aabb_make(sl_vec2_make(2.0f, 0.0f), sl_vec2_make(4.0f, 1.0f));
    const sl_aabb outside =
        sl_aabb_make(sl_vec2_make(2.1f, 0.0f), sl_vec2_make(4.0f, 1.0f));
    const sl_aabb inner =
        sl_aabb_make(sl_vec2_make(-2.0f, 0.0f), sl_vec2_make(1.0f, 3.0f));

    SL_EXPECT(sl_aabb_overlaps(a, touching));
    SL_EXPECT(!sl_aabb_overlaps(a, outside));
    SL_EXPECT(sl_aabb_contains(a, inner));
    SL_EXPECT(!sl_aabb_contains(inner, a));

    const sl_aabb combined = sl_aabb_union(a, outside);
    SL_EXPECT(combined.lower.x == -2.0f && combined.lower.y == -1.0f);
    SL_EXPECT(combined.upper.x == 4.0f && combined.upper.y == 3.0f);
    SL_EXPECT(sl_aabb_perimeter(a) == 16.0f);

    const sl_aabb extended = sl_aabb_extend(a, 0.5f);
    SL_EXPECT(extended.lower.x == -2.5f && extended.lower.y == -1.5f);
    SL_EXPECT(extended.upper.x == 2.5f && extended.upper.y == 3.5f);

    const sl_aabb invalid =
        sl_aabb_make(sl_vec2_make(NAN, 0.0f), sl_vec2_make(1.0f, 1.0f));
    SL_EXPECT(!sl_aabb_overlaps(a, invalid));
    SL_EXPECT(!sl_aabb_contains(a, invalid));
    SL_EXPECT(sl_aabb_perimeter(invalid) == 0.0f);

    const sl_aabb unchanged = sl_aabb_extend(a, -1.0f);
    SL_EXPECT(unchanged.lower.x == a.lower.x && unchanged.lower.y == a.lower.y);
    SL_EXPECT(unchanged.upper.x == a.upper.x && unchanged.upper.y == a.upper.y);

    const sl_aabb enormous = sl_aabb_make(sl_vec2_make(-FLT_MAX, -FLT_MAX),
                                          sl_vec2_make(FLT_MAX, FLT_MAX));
    SL_EXPECT(sl_aabb_perimeter(enormous) == FLT_MAX);
}

static void test_segment_distance_crossing_and_parallel(void)
{
    sl_segment_distance_result result = { 0 };
    SL_EXPECT(sl_segment_distance(
        sl_vec2_make(0.0f, 0.0f), sl_vec2_make(2.0f, 2.0f),
        sl_vec2_make(0.0f, 2.0f), sl_vec2_make(2.0f, 0.0f), &result));
    SL_EXPECT_NEAR(result.fraction_a, 0.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.fraction_b, 0.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.point_a.x, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.point_a.y, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.point_b.x, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.point_b.y, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(result.distance_sq, 0.0f, SL_TEST_EPS);

    SL_EXPECT(sl_segment_distance(
        sl_vec2_make(0.0f, 0.0f), sl_vec2_make(4.0f, 0.0f),
        sl_vec2_make(0.0f, 2.0f), sl_vec2_make(4.0f, 2.0f), &result));
    SL_EXPECT(result.fraction_a == 0.0f && result.fraction_b == 0.0f);
    SL_EXPECT(result.distance_sq == 4.0f);
}

static void test_segment_distance_handles_degenerate_and_invalid(void)
{
    sl_segment_distance_result result = { 0 };
    SL_EXPECT(sl_segment_distance(
        sl_vec2_make(1.0f, 1.0f), sl_vec2_make(1.0f, 1.0f),
        sl_vec2_make(0.0f, 0.0f), sl_vec2_make(2.0f, 0.0f), &result));
    SL_EXPECT(result.fraction_a == 0.0f);
    SL_EXPECT_NEAR(result.fraction_b, 0.5f, SL_TEST_EPS);
    SL_EXPECT(result.point_a.x == 1.0f && result.point_a.y == 1.0f);
    SL_EXPECT(result.point_b.x == 1.0f && result.point_b.y == 0.0f);
    SL_EXPECT(result.distance_sq == 1.0f);

    const sl_segment_distance_result sentinel = {
        .point_a = { 1.0f, 2.0f },
        .point_b = { 3.0f, 4.0f },
        .fraction_a = 0.25f,
        .fraction_b = 0.75f,
        .distance_sq = 5.0f,
    };
    result = sentinel;
    SL_EXPECT(!sl_segment_distance(
        sl_vec2_make(NAN, 0.0f), sl_vec2_make(0.0f, 0.0f),
        sl_vec2_make(0.0f, 0.0f), sl_vec2_make(1.0f, 0.0f), &result));
    SL_EXPECT(result.point_a.x == sentinel.point_a.x &&
              result.point_a.y == sentinel.point_a.y &&
              result.point_b.x == sentinel.point_b.x &&
              result.point_b.y == sentinel.point_b.y &&
              result.fraction_a == sentinel.fraction_a &&
              result.fraction_b == sentinel.fraction_b &&
              result.distance_sq == sentinel.distance_sq);

    SL_EXPECT(!sl_segment_distance(
        sl_vec2_make(FLT_MAX, 0.0f), sl_vec2_make(FLT_MAX, 0.0f),
        sl_vec2_make(-FLT_MAX, 0.0f), sl_vec2_make(-FLT_MAX, 0.0f), &result));
    SL_EXPECT(result.distance_sq == sentinel.distance_sq);
}

static void test_angle_wrap_identity_inside_range(void)
{
    /* In-range inputs pass through bitwise, both closed ends included:
     * fmodf returns its first argument whenever |x| < y. */
    SL_EXPECT(sl_angle_wrap(0.0f) == 0.0f);
    SL_EXPECT(sl_angle_wrap(0.5f) == 0.5f);
    SL_EXPECT(sl_angle_wrap(-1.25f) == -1.25f);
    SL_EXPECT(sl_angle_wrap(SL_PI / 2.0f) == SL_PI / 2.0f);
    SL_EXPECT(sl_angle_wrap(SL_PI) == SL_PI);
    SL_EXPECT(sl_angle_wrap(-SL_PI) == -SL_PI);
}

static void test_angle_wrap_single_turn(void)
{
    SL_EXPECT_NEAR(sl_angle_wrap(2.5f * SL_PI), 0.5f * SL_PI, SL_TEST_EPS);
    SL_EXPECT_NEAR(sl_angle_wrap(-2.5f * SL_PI), -0.5f * SL_PI, SL_TEST_EPS);
    SL_EXPECT_NEAR(sl_angle_wrap(2.0f * SL_PI), 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(sl_angle_wrap(-6.0f * SL_PI), 0.0f, SL_TEST_EPS);
}

static void test_angle_wrap_many_turns(void)
{
    /* Deep in the multi-turn regime fmodf reduces first; by then
     * 100*pi carries ~1e-4 of representation error from the argument
     * itself, so the tolerance pins reduction, not trigonometry. */
    SL_EXPECT_NEAR(sl_angle_wrap(100.0f * SL_PI + 0.5f), 0.5f, 1e-4f);
    SL_EXPECT_NEAR(sl_angle_wrap(-100.0f * SL_PI + 0.5f), 0.5f, 1e-4f);
}

static void test_mat2_identity(void)
{
    sl_mat2 id = sl_mat2_identity();
    sl_vec2 v = sl_vec2_make(7.5f, -2.0f);
    sl_vec2 r = sl_mat2_multiply_vec2(id, v);
    SL_EXPECT(r.x == v.x && r.y == v.y);
}

/* Operand order: matrix multiply is not commutative, and every rotation
 * product commutes, so this integer case is what actually pins which
 * operand is applied on the left. Exact equality is safe here: all
 * intermediate values are integers <= 50. */
static void test_mat2_multiply_operand_order(void)
{
    sl_mat2 a = { 1.0f, 2.0f, 3.0f, 4.0f };
    sl_mat2 b = { 5.0f, 6.0f, 7.0f, 8.0f };

    /* a * b = | 1*5+2*7  1*6+2*8 |   = | 19  22 |
               | 3*5+4*7  3*6+4*8 |     | 43  50 | */
    sl_mat2 ab = sl_mat2_multiply(a, b);
    SL_EXPECT(ab.m00 == 19.0f && ab.m01 == 22.0f);
    SL_EXPECT(ab.m10 == 43.0f && ab.m11 == 50.0f);

    /* b * a differs in every entry; a swapped implementation lands here. */
    sl_mat2 ba = sl_mat2_multiply(b, a);
    SL_EXPECT(ba.m00 == 23.0f && ba.m01 == 34.0f);
    SL_EXPECT(ba.m10 == 31.0f && ba.m11 == 46.0f);
}

static void test_mat2_rotation_90deg(void)
{
    /* Rotating +x axis by 90 degrees CCW gives +y axis. */
    sl_mat2 rot = sl_mat2_rotation(SL_PI / 2.0f);
    sl_vec2 r = sl_mat2_multiply_vec2(rot, sl_vec2_make(1.0f, 0.0f));
    SL_EXPECT_NEAR(r.x, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(r.y, 1.0f, SL_TEST_EPS);
}

static void test_mat2_rotation_composition(void)
{
    /* Two 90-degree rotations equal one 180-degree rotation. */
    sl_mat2 quarter = sl_mat2_rotation(SL_PI / 2.0f);
    sl_mat2 half = sl_mat2_rotation(SL_PI);

    sl_vec2 via_composition = sl_mat2_multiply_vec2(
        sl_mat2_multiply(quarter, quarter), sl_vec2_make(1.0f, 0.0f));
    sl_vec2 direct = sl_mat2_multiply_vec2(half, sl_vec2_make(1.0f, 0.0f));

    SL_EXPECT_NEAR(via_composition.x, direct.x, SL_TEST_EPS);
    SL_EXPECT_NEAR(via_composition.y, direct.y, SL_TEST_EPS);
}

static void test_mat2_transpose_inverse_for_rotation(void)
{
    /* For pure rotations, transpose equals inverse: R^T * R == I. */
    sl_mat2 rot = sl_mat2_rotation(0.7f);
    sl_mat2 rt_r = sl_mat2_multiply(sl_mat2_transpose(rot), rot);
    sl_mat2 id = sl_mat2_identity();

    SL_EXPECT_NEAR(rt_r.m00, id.m00, SL_TEST_EPS);
    SL_EXPECT_NEAR(rt_r.m01, id.m01, SL_TEST_EPS);
    SL_EXPECT_NEAR(rt_r.m10, id.m10, SL_TEST_EPS);
    SL_EXPECT_NEAR(rt_r.m11, id.m11, SL_TEST_EPS);
}

static void test_mat2_solve_known_system(void)
{
    const sl_mat2 matrix = { 3.0f, 2.0f, 1.0f, 2.0f };
    const sl_vec2 b = sl_vec2_make(7.0f, 5.0f);
    sl_vec2 solution = sl_vec2_make(0.0f, 0.0f);
    SL_EXPECT(sl_mat2_solve(matrix, b, &solution));
    SL_EXPECT_NEAR(solution.x, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(solution.y, 2.0f, SL_TEST_EPS);

    const sl_vec2 reconstructed = sl_mat2_multiply_vec2(matrix, solution);
    SL_EXPECT_NEAR(reconstructed.x, b.x, SL_TEST_EPS);
    SL_EXPECT_NEAR(reconstructed.y, b.y, SL_TEST_EPS);
}

static void test_mat2_solve_rejects_without_partial_output(void)
{
    const sl_vec2 sentinel = sl_vec2_make(7.0f, -9.0f);
    sl_vec2 output = sentinel;
    SL_EXPECT(!sl_mat2_solve((sl_mat2){ 1.0f, 2.0f, 2.0f, 4.0f },
                             sl_vec2_make(1.0f, 2.0f), &output));
    SL_EXPECT(output.x == sentinel.x && output.y == sentinel.y);

    const float almost_singular = 0.5f * FLT_EPSILON;
    SL_EXPECT(!sl_mat2_solve((sl_mat2){ 1.0f, 0.0f, 0.0f, almost_singular },
                             sl_vec2_make(1.0f, 1.0f), &output));
    SL_EXPECT(output.x == sentinel.x && output.y == sentinel.y);

    SL_EXPECT(!sl_mat2_solve((sl_mat2){ FLT_MIN, 0.0f, 0.0f, FLT_MIN },
                             sl_vec2_make(FLT_MAX, 1.0f), &output));
    SL_EXPECT(output.x == sentinel.x && output.y == sentinel.y);

    SL_EXPECT(!sl_mat2_solve((sl_mat2){ NAN, 0.0f, 0.0f, 1.0f },
                             sl_vec2_make(1.0f, 1.0f), &output));
    SL_EXPECT(output.x == sentinel.x && output.y == sentinel.y);
}

static const sl_test_case k_cases[] = {
    { "scalar_min_max", test_scalar_min_max },
    { "scalar_clamp", test_scalar_clamp },
    { "scalar_feq", test_scalar_feq },
    { "vec2_make_and_components", test_vec2_make_and_components },
    { "vec2_add_sub_neg_scale", test_vec2_add_sub_neg_scale },
    { "vec2_dot", test_vec2_dot },
    { "vec2_cross_sign", test_vec2_cross_sign },
    { "vec2_length_normalize", test_vec2_length_normalize },
    { "vec2_perp_is_orthogonal_ccw", test_vec2_perp_is_orthogonal_ccw },
    { "vec2_right_perp_and_scalar_cross_order",
      test_vec2_right_perp_and_scalar_cross_order },
    { "vec2_lerp_endpoints", test_vec2_lerp_endpoints },
    { "vec2_distance", test_vec2_distance },
    { "vec2_min_max_componentwise", test_vec2_min_max_componentwise },
    { "vec2_distance_sq_matches_distance",
      test_vec2_distance_sq_matches_distance },
    { "rotation_identity_is_exact", test_rotation_identity_is_exact },
    { "rotation_make_matches_mat2_rotation",
      test_rotation_make_matches_mat2_rotation },
    { "rotation_apply_inverse_roundtrip",
      test_rotation_apply_inverse_roundtrip },
    { "rotation_mul_composes_and_reports_angle",
      test_rotation_mul_composes_and_reports_angle },
    { "rotation_normalize_is_scale_safe",
      test_rotation_normalize_is_scale_safe },
    { "rotation_integrate_small_and_extreme_deltas",
      test_rotation_integrate_small_and_extreme_deltas },
    { "transform_apply_rotates_then_translates",
      test_transform_apply_rotates_then_translates },
    { "transform_apply_inverse_roundtrip",
      test_transform_apply_inverse_roundtrip },
    { "aabb_contains_point_boundary_inclusive",
      test_aabb_contains_point_boundary_inclusive },
    { "aabb_is_valid_rejects_inverted_or_non_finite",
      test_aabb_is_valid_rejects_inverted_or_non_finite },
    { "aabb_set_operations_are_boundary_inclusive",
      test_aabb_set_operations_are_boundary_inclusive },
    { "segment_distance_crossing_and_parallel",
      test_segment_distance_crossing_and_parallel },
    { "segment_distance_handles_degenerate_and_invalid",
      test_segment_distance_handles_degenerate_and_invalid },
    { "angle_wrap_identity_inside_range",
      test_angle_wrap_identity_inside_range },
    { "angle_wrap_single_turn", test_angle_wrap_single_turn },
    { "angle_wrap_many_turns", test_angle_wrap_many_turns },
    { "mat2_identity", test_mat2_identity },
    { "mat2_multiply_operand_order", test_mat2_multiply_operand_order },
    { "mat2_rotation_90deg", test_mat2_rotation_90deg },
    { "mat2_rotation_composition", test_mat2_rotation_composition },
    { "mat2_transpose_inverse_for_rotation",
      test_mat2_transpose_inverse_for_rotation },
    { "mat2_solve_known_system", test_mat2_solve_known_system },
    { "mat2_solve_rejects_without_partial_output",
      test_mat2_solve_rejects_without_partial_output },
};

int sl_math_suite(void)
{
    return sl_run_suite("math", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
