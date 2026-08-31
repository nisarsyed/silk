#include "silk_test.h"
#include "suites.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <silk/contact.h>
#include <silk/shape.h>

#include "collide.h"

#define SL_TEST_EPS 1e-5f

static sl_transform transform_at(float x, float y, float radians)
{
    return sl_transform_make(sl_vec2_make(x, y), sl_rotation_make(radians));
}

static sl_shape circle_make(float radius)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(radius, &shape));
    return shape;
}

static sl_shape box_make(float half_width, float half_height)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(half_width, half_height, &shape));
    return shape;
}

static void expect_vec2_near(sl_vec2 actual, sl_vec2 expected, float epsilon)
{
    SL_EXPECT_NEAR(actual.x, expected.x, epsilon);
    SL_EXPECT_NEAR(actual.y, expected.y, epsilon);
}

static void expect_manifold_finite(const sl_manifold *manifold)
{
    SL_EXPECT(manifold->point_count <= SL_MANIFOLD_POINT_COUNT_MAX);
    if (manifold->point_count > 0u) {
        SL_EXPECT(sl_vec2_is_finite(manifold->normal));
        SL_EXPECT_NEAR(sl_vec2_length(manifold->normal), 1.0f, 1e-4f);
    }
    for (uint32_t i = 0u; i < manifold->point_count; ++i) {
        const sl_manifold_point *point = &manifold->points[i];
        SL_EXPECT(sl_vec2_is_finite(point->anchor_a));
        SL_EXPECT(sl_vec2_is_finite(point->anchor_b));
        SL_EXPECT(sl_vec2_is_finite(point->point));
        SL_EXPECT(sl_is_finite(point->separation));
        SL_EXPECT(point->normal_impulse == 0.0f);
        SL_EXPECT(point->tangent_impulse == 0.0f);
        SL_EXPECT(point->normal_velocity == 0.0f);
        SL_EXPECT(!point->persisted);
    }
}

static void circles_overlap_analytically(void)
{
    const sl_shape circle = circle_make(1.0f);
    const sl_manifold manifold =
        sl_collide_shapes(&circle, transform_at(0.0f, 0.0f, 0.0f), &circle,
                          transform_at(1.9f, 0.0f, 0.0f));

    SL_EXPECT_INT_EQ(manifold.point_count, 1u);
    expect_vec2_near(manifold.normal, sl_vec2_make(1.0f, 0.0f), SL_TEST_EPS);
    SL_EXPECT_NEAR(manifold.points[0].separation, -0.1f, SL_TEST_EPS);
    expect_vec2_near(manifold.points[0].anchor_a, sl_vec2_make(1.0f, 0.0f),
                     SL_TEST_EPS);
    expect_vec2_near(manifold.points[0].anchor_b, sl_vec2_make(-1.0f, 0.0f),
                     SL_TEST_EPS);
    expect_vec2_near(manifold.points[0].point, sl_vec2_make(0.95f, 0.0f),
                     SL_TEST_EPS);
    expect_manifold_finite(&manifold);
}

static void circles_touch_coincide_and_speculate(void)
{
    const sl_shape circle = circle_make(1.0f);
    const sl_transform identity = transform_at(0.0f, 0.0f, 0.0f);

    const sl_manifold touching = sl_collide_shapes(
        &circle, identity, &circle, transform_at(2.0f, 0.0f, 0.0f));
    SL_EXPECT_INT_EQ(touching.point_count, 1u);
    SL_EXPECT_NEAR(touching.points[0].separation, 0.0f, SL_TEST_EPS);

    const sl_manifold coincident =
        sl_collide_shapes(&circle, identity, &circle, identity);
    SL_EXPECT_INT_EQ(coincident.point_count, 1u);
    expect_vec2_near(coincident.normal, sl_vec2_make(1.0f, 0.0f), SL_TEST_EPS);
    SL_EXPECT_NEAR(coincident.points[0].separation, -2.0f, SL_TEST_EPS);

    const sl_manifold speculative = sl_collide_shapes(
        &circle, identity, &circle, transform_at(2.015f, 0.0f, 0.0f));
    SL_EXPECT_INT_EQ(speculative.point_count, 1u);
    SL_EXPECT_NEAR(speculative.points[0].separation, 0.015f, SL_TEST_EPS);

    const sl_manifold separated = sl_collide_shapes(
        &circle, identity, &circle, transform_at(2.03f, 0.0f, 0.0f));
    SL_EXPECT_INT_EQ(separated.point_count, 0u);
}

static void polygon_circle_face_and_vertex_regions(void)
{
    const sl_shape box = box_make(1.0f, 1.0f);
    const sl_shape circle = circle_make(0.5f);
    const sl_transform identity = transform_at(0.0f, 0.0f, 0.0f);

    const sl_manifold face = sl_collide_shapes(&box, identity, &circle,
                                               transform_at(0.0f, 1.4f, 0.0f));
    SL_EXPECT_INT_EQ(face.point_count, 1u);
    expect_vec2_near(face.normal, sl_vec2_make(0.0f, 1.0f), SL_TEST_EPS);
    SL_EXPECT_NEAR(face.points[0].separation, -0.1f, SL_TEST_EPS);

    const float diagonal = 1.0f / sqrtf(2.0f);
    const sl_manifold vertex = sl_collide_shapes(
        &box, identity, &circle, transform_at(1.3f, 1.3f, 0.0f));
    SL_EXPECT_INT_EQ(vertex.point_count, 1u);
    expect_vec2_near(vertex.normal, sl_vec2_make(diagonal, diagonal), 1e-4f);
    SL_EXPECT_NEAR(vertex.points[0].separation,
                   sqrtf(0.3f * 0.3f + 0.3f * 0.3f) - 0.5f, 1e-4f);

    const sl_manifold outside = sl_collide_shapes(
        &box, identity, &circle, transform_at(1.4f, 1.4f, 0.0f));
    SL_EXPECT_INT_EQ(outside.point_count, 0u);
    expect_manifold_finite(&face);
    expect_manifold_finite(&vertex);
}

static void circle_polygon_flip_is_complete(void)
{
    const sl_shape box = box_make(1.0f, 1.0f);
    const sl_shape circle = circle_make(0.5f);
    const sl_transform box_transform = transform_at(2.0f, -1.0f, 0.2f);
    const sl_transform circle_transform = transform_at(1.8f, 0.3f, 0.0f);
    const sl_manifold polygon_first =
        sl_collide_shapes(&box, box_transform, &circle, circle_transform);
    const sl_manifold circle_first =
        sl_collide_shapes(&circle, circle_transform, &box, box_transform);

    SL_EXPECT_INT_EQ(polygon_first.point_count, 1u);
    SL_EXPECT_INT_EQ(circle_first.point_count, 1u);
    expect_vec2_near(circle_first.normal, sl_vec2_neg(polygon_first.normal),
                     SL_TEST_EPS);
    expect_vec2_near(circle_first.points[0].anchor_a,
                     polygon_first.points[0].anchor_b, SL_TEST_EPS);
    expect_vec2_near(circle_first.points[0].anchor_b,
                     polygon_first.points[0].anchor_a, SL_TEST_EPS);
    SL_EXPECT_NEAR(circle_first.points[0].separation,
                   polygon_first.points[0].separation, SL_TEST_EPS);
    SL_EXPECT_INT_EQ(circle_first.points[0].id,
                     ((polygon_first.points[0].id & 0xffu) << 8u) |
                         ((polygon_first.points[0].id >> 8u) & 0xffu));
}

static void resting_boxes_have_two_stable_points(void)
{
    const sl_shape box = box_make(1.0f, 1.0f);
    const sl_transform lower = transform_at(0.0f, 0.0f, 0.0f);
    const sl_manifold first =
        sl_collide_shapes(&box, lower, &box, transform_at(0.0f, 2.0f, 0.0f));
    const sl_manifold jittered =
        sl_collide_shapes(&box, lower, &box, transform_at(0.0f, 1.999f, 0.0f));

    SL_EXPECT_INT_EQ(first.point_count, 2u);
    SL_EXPECT_INT_EQ(jittered.point_count, 2u);
    expect_vec2_near(first.normal, sl_vec2_make(0.0f, 1.0f), SL_TEST_EPS);
    for (uint32_t i = 0u; i < 2u; ++i) {
        SL_EXPECT_NEAR(first.points[i].separation, 0.0f, SL_TEST_EPS);
        SL_EXPECT_INT_EQ(first.points[i].id, jittered.points[i].id);
    }
    expect_manifold_finite(&first);
    expect_manifold_finite(&jittered);
}

static void polygon_gap_uses_closest_features(void)
{
    const sl_shape box = box_make(1.0f, 1.0f);
    const sl_manifold close =
        sl_collide_shapes(&box, transform_at(0.0f, 0.0f, 0.0f), &box,
                          transform_at(0.0f, 2.015f, 0.0f));
    const sl_manifold far =
        sl_collide_shapes(&box, transform_at(0.0f, 0.0f, 0.0f), &box,
                          transform_at(0.0f, 2.03f, 0.0f));
    SL_EXPECT_INT_EQ(close.point_count, 1u);
    SL_EXPECT_NEAR(close.points[0].separation, 0.015f, SL_TEST_EPS);
    expect_vec2_near(close.normal, sl_vec2_make(0.0f, 1.0f), SL_TEST_EPS);
    SL_EXPECT_INT_EQ(far.point_count, 0u);
}

static void rotated_corner_and_large_coordinates_stay_finite(void)
{
    const sl_shape box = box_make(1.0f, 1.0f);
    const sl_manifold corner =
        sl_collide_shapes(&box, transform_at(0.0f, 0.0f, 0.0f), &box,
                          transform_at(1.7f, 1.7f, 0.25f * SL_PI));
    SL_EXPECT(corner.point_count > 0u);
    expect_manifold_finite(&corner);

    const sl_manifold large =
        sl_collide_shapes(&box, transform_at(8000.0f, -8000.0f, 0.17f), &box,
                          transform_at(8000.0f, -7998.2f, -0.11f));
    SL_EXPECT(large.point_count > 0u);
    expect_manifold_finite(&large);
}

static uint32_t prng_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    *state = x;
    return x;
}

static float prng_range(uint32_t *state, float lower, float upper)
{
    const float unit = (float)(prng_next(state) & 0xffffu) / 65535.0f;
    return lower + (upper - lower) * unit;
}

static void seeded_circle_properties_are_symmetric(void)
{
    uint32_t state = 0x6d2b79f5u;
    for (uint32_t i = 0u; i < 500u; ++i) {
        const float radius_a = prng_range(&state, 0.1f, 2.0f);
        const float radius_b = prng_range(&state, 0.1f, 2.0f);
        const float angle = prng_range(&state, -SL_PI, SL_PI);
        const float distance =
            prng_range(&state, 0.01f,
                       radius_a + radius_b + SL_SPECULATIVE_DISTANCE + 0.1f);
        const sl_shape circle_a = circle_make(radius_a);
        const sl_shape circle_b = circle_make(radius_b);
        const sl_transform transform_a =
            transform_at(prng_range(&state, -100.0f, 100.0f),
                         prng_range(&state, -100.0f, 100.0f), 0.0f);
        const sl_transform transform_b =
            transform_at(transform_a.position.x + cosf(angle) * distance,
                         transform_a.position.y + sinf(angle) * distance, 0.0f);
        const sl_manifold ab =
            sl_collide_shapes(&circle_a, transform_a, &circle_b, transform_b);
        const sl_manifold ba =
            sl_collide_shapes(&circle_b, transform_b, &circle_a, transform_a);

        SL_EXPECT_INT_EQ(ab.point_count, ba.point_count);
        expect_manifold_finite(&ab);
        expect_manifold_finite(&ba);
        if (ab.point_count == 1u && ba.point_count == 1u) {
            expect_vec2_near(ab.normal, sl_vec2_neg(ba.normal), 2e-4f);
            SL_EXPECT_NEAR(ab.points[0].separation, ba.points[0].separation,
                           2e-4f);
            expect_vec2_near(ab.points[0].anchor_a, ba.points[0].anchor_b,
                             2e-4f);
            expect_vec2_near(ab.points[0].anchor_b, ba.points[0].anchor_a,
                             2e-4f);
        }
    }
}

static void none_shape_never_collides(void)
{
    const sl_shape none = sl_shape_none();
    const sl_shape circle = circle_make(1.0f);
    const sl_transform identity = transform_at(0.0f, 0.0f, 0.0f);
    const sl_manifold manifold =
        sl_collide_shapes(&none, identity, &circle, identity);
    SL_EXPECT_INT_EQ(manifold.point_count, 0u);
    SL_EXPECT(memcmp(&manifold, &(sl_manifold){ 0 }, sizeof(manifold)) == 0);
}

static const sl_test_case k_cases[] = {
    { "circles overlap analytically", circles_overlap_analytically },
    { "circle boundary cases", circles_touch_coincide_and_speculate },
    { "polygon circle regions", polygon_circle_face_and_vertex_regions },
    { "circle polygon flip", circle_polygon_flip_is_complete },
    { "resting box contacts", resting_boxes_have_two_stable_points },
    { "polygon speculative gap", polygon_gap_uses_closest_features },
    { "rotated and large contacts",
      rotated_corner_and_large_coordinates_stay_finite },
    { "seeded circle properties", seeded_circle_properties_are_symmetric },
    { "none is inert", none_shape_never_collides },
};

int sl_collide_suite(void)
{
    return sl_run_suite("collide", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
