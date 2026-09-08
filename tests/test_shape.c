#include "silk_test.h"
#include "suites.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <silk/math.h>
#include <silk/shape.h>

/* Exact equality only where IEEE-754 guarantees it (zeroed records,
 * integer-valued construction, rejection paths); anything through trig,
 * sqrt, or fan summation compares via tolerances. Scale-dependent
 * oracles (geometry spanning ~100 units) pin at 1e-5. */
#define SL_TEST_EPS 1e-6f
#define SL_TEST_SCALE_EPS 1e-5f

static sl_transform tf_identity(void)
{
    return sl_transform_identity();
}

static sl_transform tf_at(float x, float y, float radians)
{
    return sl_transform_make(sl_vec2_make(x, y), sl_rotation_make(radians));
}

static sl_ray_hit hit_sentinel(void)
{
    sl_ray_hit hit;
    hit.fraction = -42.0f;
    hit.point = sl_vec2_make(-42.0f, -42.0f);
    hit.normal = sl_vec2_make(-42.0f, -42.0f);
    return hit;
}

static bool hit_untouched(const sl_ray_hit *hit)
{
    return hit->fraction == -42.0f && hit->point.x == -42.0f &&
           hit->point.y == -42.0f && hit->normal.x == -42.0f &&
           hit->normal.y == -42.0f;
}

/* A constructor that writes only its live union member leaves the tail
 * carrying whatever was on the stack, so byte-wise hashes, comparisons,
 * and serialization of shape records differ run to run. Checked
 * byte-wise because that is the property that matters; the layout
 * assert in shape.c pins sl_shape as padding-free. */
static void expect_payload_zeroed(const sl_shape *shape)
{
    const unsigned char *bytes = (const unsigned char *)shape;
    bool clean = true;
    for (size_t i = sizeof(sl_shape_kind); i < sizeof(sl_shape); ++i) {
        clean = clean && bytes[i] == 0u;
    }
    SL_EXPECT(clean);
}

/* Constructors build their record on their own frame, so a leaked tail
 * reads back as whatever that frame last held. Soil it first: on an
 * already-clean stack an uninitialized tail reads as zero and the
 * assertions above pass vacuously. */
static void soil_stack(void)
{
    volatile unsigned char scratch[sizeof(sl_shape) * 4u];
    for (size_t i = 0u; i < sizeof(scratch); ++i) {
        scratch[i] = (unsigned char)(0xA5u + i);
    }
}

static void none_shape_is_zeroed_and_inert(void)
{
    soil_stack();
    const sl_shape shape = sl_shape_none();
    SL_EXPECT(shape.kind == SL_SHAPE_NONE);
    expect_payload_zeroed(&shape);

    /* Zeroed record stays valid: kind NONE with dead payload bytes. */
    sl_shape zeroed;
    memset(&zeroed, 0, sizeof(zeroed));
    SL_EXPECT(zeroed.kind == SL_SHAPE_NONE);
    /* And sl_shape_none is exactly that record -- the representation
     * used for every shapeless body must have the same bytes. */
    SL_EXPECT(memcmp(&shape, &zeroed, sizeof(shape)) == 0);

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT(data.area == 0.0f);
    SL_EXPECT(data.centroid.x == 0.0f && data.centroid.y == 0.0f);
    SL_EXPECT(data.inertia_per_unit_mass == 0.0f);

    const sl_aabb box = sl_shape_aabb(&shape, tf_at(3.0f, -4.0f, 0.7f));
    SL_EXPECT(box.lower.x == 3.0f && box.lower.y == -4.0f);
    SL_EXPECT(box.upper.x == 3.0f && box.upper.y == -4.0f);

    SL_EXPECT(!sl_shape_contains_point(&shape, tf_identity(),
                                       sl_vec2_make(3.0f, -4.0f)));

    sl_ray_hit hit = hit_sentinel();
    const sl_ray ray = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(1.0f, 0.0f) };
    SL_EXPECT(!sl_shape_ray_cast(&shape, tf_identity(), ray, &hit));
    SL_EXPECT(hit_untouched(&hit));
}

static void constructors_zero_unused_payload(void)
{
    /* A circle leaves the 64-byte polygon arm of the union unused, and
     * a polygon leaves vertices[count..7] unused; neither may carry
     * stack residue in its output record. */
    soil_stack();
    sl_shape circle;
    SL_EXPECT(sl_shape_make_circle(1.5f, &circle));
    const unsigned char *bytes = (const unsigned char *)&circle;
    bool tail_clean = true;
    for (size_t i = offsetof(sl_shape, circle) + sizeof(sl_circle);
         i < sizeof(sl_shape); ++i) {
        tail_clean = tail_clean && bytes[i] == 0u;
    }
    SL_EXPECT(tail_clean);

    soil_stack();
    sl_shape tri;
    const sl_vec2 points[3] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(2.0f, 0.0f),
                                sl_vec2_make(0.0f, 2.0f) };
    SL_EXPECT(sl_shape_make_polygon(points, 3u, &tri));
    SL_EXPECT_INT_EQ(tri.polygon.count, 3u);
    bool vertex_tail_clean = true;
    for (uint32_t i = 3u; i < SL_POLYGON_VERTEX_COUNT_MAX; ++i) {
        const sl_vec2 v = tri.polygon.vertices[i];
        vertex_tail_clean = vertex_tail_clean && v.x == 0.0f && v.y == 0.0f;
    }
    SL_EXPECT(vertex_tail_clean);

    /* Same geometry built twice compares equal byte for byte, from
     * differently soiled frames -- the determinism the world needs. */
    soil_stack();
    sl_shape again;
    SL_EXPECT(sl_shape_make_polygon(points, 3u, &again));
    SL_EXPECT(memcmp(&tri, &again, sizeof(tri)) == 0);
}

static void circle_make_rejects_bad_radius(void)
{
    const float bad[] = { NAN,
                          INFINITY,
                          -INFINITY,
                          0.0f,
                          -1.0f,
                          SL_EPSILON,
                          SL_SHAPE_EXTENT_MAX + 1.0f,
                          1e20f };
    for (uint32_t i = 0u; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        sl_shape out = sl_shape_none();
        out.circle.radius = 5.0f; /* sentinel payload */
        SL_EXPECT(!sl_shape_make_circle(bad[i], &out));
        /* Untouched means the caller's record survives rejection. */
        SL_EXPECT(out.kind == SL_SHAPE_NONE && out.circle.radius == 5.0f);
    }

    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &shape));
    SL_EXPECT(shape.kind == SL_SHAPE_CIRCLE);
    SL_EXPECT(shape.circle.radius == 2.0f);
    SL_EXPECT(sl_shape_is_valid(&shape));
}

static void shape_extent_limit_bounds_derived_data(void)
{
    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(SL_SHAPE_EXTENT_MAX, &circle));
    const sl_mass_data circle_data = sl_shape_mass_data(&circle);
    SL_EXPECT(sl_is_finite(circle_data.area));
    SL_EXPECT(sl_is_finite(circle_data.inertia_per_unit_mass));

    const sl_vec2 at_limit[4] = {
        { 0.0f, -SL_SHAPE_EXTENT_MAX },
        { SL_SHAPE_EXTENT_MAX, 0.0f },
        { 0.0f, SL_SHAPE_EXTENT_MAX },
        { -SL_SHAPE_EXTENT_MAX, 0.0f },
    };
    sl_shape polygon = sl_shape_none();
    SL_EXPECT(sl_shape_make_polygon(at_limit, 4u, &polygon));
    const sl_mass_data polygon_data = sl_shape_mass_data(&polygon);
    SL_EXPECT(sl_is_finite(polygon_data.area));
    SL_EXPECT(sl_is_finite(polygon_data.inertia_per_unit_mass));

    const float outside = SL_SHAPE_EXTENT_MAX + 1.0f;
    const sl_vec2 over_limit[4] = {
        { 0.0f, -outside },
        { outside, 0.0f },
        { 0.0f, outside },
        { -outside, 0.0f },
    };
    sl_shape untouched = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(over_limit, 4u, &untouched));
    SL_EXPECT(untouched.kind == SL_SHAPE_NONE);
}

static void far_origin_polygon_is_canonical_and_valid(void)
{
    /* At 1e7, binary32 spacing is one world unit. The first centroid
     * subtraction therefore leaves a one-third-unit residual even
     * though this triangle has ordinary local extents. Construction
     * must refine that residual before publishing the record. */
    const sl_vec2 points[3] = {
        { 9999998.0f, 9999999.0f },
        { 10000003.0f, 9999999.0f },
        { 10000000.0f, 10000002.0f },
    };
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_polygon(points, 3u, &shape));
    SL_EXPECT(sl_shape_is_valid(&shape));

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT(sl_is_finite(data.area));
    SL_EXPECT(sl_vec2_is_finite(data.centroid));
    SL_EXPECT(sl_is_finite(data.inertia_per_unit_mass));
}

static void circle_mass_data_matches_closed_form(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &shape));

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT_NEAR(data.area, SL_PI * 4.0f, SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(data.centroid.x, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.centroid.y, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.inertia_per_unit_mass, 2.0f, SL_TEST_EPS);
}

static void circle_aabb_ignores_rotation(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &shape));

    const float angles[] = { 0.0f, 0.7f, SL_PI / 2.0f, -2.3f };
    for (uint32_t i = 0u; i < sizeof(angles) / sizeof(angles[0]); ++i) {
        const sl_aabb box =
            sl_shape_aabb(&shape, tf_at(10.0f, -5.0f, angles[i]));
        SL_EXPECT_NEAR(box.lower.x, 8.0f, SL_TEST_EPS);
        SL_EXPECT_NEAR(box.lower.y, -7.0f, SL_TEST_EPS);
        SL_EXPECT_NEAR(box.upper.x, 12.0f, SL_TEST_EPS);
        SL_EXPECT_NEAR(box.upper.y, -3.0f, SL_TEST_EPS);
    }
}

static void box_make_matches_closed_form(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &shape));
    SL_EXPECT(shape.kind == SL_SHAPE_POLYGON);

    /* Canonical CCW corners, recentered onto the origin. */
    const sl_polygon *p = &shape.polygon;
    SL_EXPECT_INT_EQ((int)p->count, 4);
    SL_EXPECT(p->vertices[0].x == -1.0f && p->vertices[0].y == -0.5f);
    SL_EXPECT(p->vertices[1].x == 1.0f && p->vertices[1].y == -0.5f);
    SL_EXPECT(p->vertices[2].x == 1.0f && p->vertices[2].y == 0.5f);
    SL_EXPECT(p->vertices[3].x == -1.0f && p->vertices[3].y == 0.5f);
    SL_EXPECT(sl_shape_is_valid(&shape));
}

static void box_mass_data_matches_closed_form(void)
{
    /* Half-extents (1, 0.5): area 2, I/m = (w^2 + h^2)/12 = 5/12. */
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &shape));

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT_NEAR(data.area, 2.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.centroid.x, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.centroid.y, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.inertia_per_unit_mass, 5.0f / 12.0f, SL_TEST_EPS);
}

static void right_triangle_mass_data_matches_closed_form(void)
{
    /* Legs 3 and 3 about the origin corner: area 4.5, sides 3/3/3*sqrt2,
     * I/m = (a^2 + b^2 + c^2)/36 = 1. Centroid (1, 1) shifts storage to
     * integer coordinates, so vertices compare exactly. */
    const sl_vec2 points[3] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(3.0f, 0.0f),
                                sl_vec2_make(0.0f, 3.0f) };
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_polygon(points, 3u, &shape));

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT_NEAR(data.area, 4.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(data.inertia_per_unit_mass, 1.0f, SL_TEST_EPS);

    const sl_polygon *p = &shape.polygon;
    SL_EXPECT(p->vertices[0].x == -1.0f && p->vertices[0].y == -1.0f);
    SL_EXPECT(p->vertices[1].x == 2.0f && p->vertices[1].y == -1.0f);
    SL_EXPECT(p->vertices[2].x == -1.0f && p->vertices[2].y == 2.0f);
}

static void polygon_make_recenters_offset_input(void)
{
    /* Same triangle as above, parked at (10, 10): storage must land on
     * the identical centered coordinates, bit for bit. */
    const sl_vec2 points[3] = { sl_vec2_make(10.0f, 10.0f),
                                sl_vec2_make(13.0f, 10.0f),
                                sl_vec2_make(10.0f, 13.0f) };
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_polygon(points, 3u, &shape));

    const sl_polygon *p = &shape.polygon;
    SL_EXPECT(p->vertices[0].x == -1.0f && p->vertices[0].y == -1.0f);
    SL_EXPECT(p->vertices[1].x == 2.0f && p->vertices[1].y == -1.0f);
    SL_EXPECT(p->vertices[2].x == -1.0f && p->vertices[2].y == 2.0f);
}

static void polygon_accepts_octagon_at_vertex_count_max(void)
{
    /* Regular octagon, circumradius 1: area 2*sqrt(2), I/m =
     * (R^2/6)*(2 + cos(pi/4)) ~= 0.4511845. */
    sl_vec2 points[SL_POLYGON_VERTEX_COUNT_MAX];
    for (uint32_t k = 0u; k < SL_POLYGON_VERTEX_COUNT_MAX; ++k) {
        const double angle = (double)k * (2.0 * (double)SL_PI) / 8.0;
        points[k] = sl_vec2_make((float)cos(angle), (float)sin(angle));
    }

    sl_shape shape = sl_shape_none();
    SL_EXPECT(
        sl_shape_make_polygon(points, SL_POLYGON_VERTEX_COUNT_MAX, &shape));
    SL_EXPECT(sl_shape_is_valid(&shape));

    const sl_mass_data data = sl_shape_mass_data(&shape);
    SL_EXPECT_NEAR(data.area, 2.0f * sqrtf(2.0f), SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(data.inertia_per_unit_mass, 0.4511845f, SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(data.centroid.x, 0.0f, SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(data.centroid.y, 0.0f, SL_TEST_SCALE_EPS);
}

static void polygon_make_rejects_bad_count(void)
{
    const sl_vec2 points[SL_POLYGON_VERTEX_COUNT_MAX] = {
        sl_vec2_make(-1.0f, -1.0f), sl_vec2_make(1.0f, -1.0f),
        sl_vec2_make(1.0f, 1.0f),   sl_vec2_make(-1.0f, 1.0f),
        sl_vec2_make(0.0f, 0.0f),   sl_vec2_make(0.0f, 0.0f),
        sl_vec2_make(0.0f, 0.0f),   sl_vec2_make(0.0f, 0.0f)
    };

    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 2u, &out));
    SL_EXPECT(!sl_shape_make_polygon(points, 9u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
    SL_EXPECT(!sl_shape_make_polygon(NULL, 4u, &out));
    SL_EXPECT(!sl_shape_make_polygon(points, 4u, NULL));
}

static void polygon_make_rejects_non_finite_vertex(void)
{
    const sl_vec2 points[4] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(NAN, 0.0f),
                                sl_vec2_make(1.0f, 1.0f),
                                sl_vec2_make(0.0f, 1.0f) };
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 4u, &out));

    const sl_vec2 infinite[4] = { sl_vec2_make(0.0f, 0.0f),
                                  sl_vec2_make(INFINITY, 0.0f),
                                  sl_vec2_make(1.0f, 1.0f),
                                  sl_vec2_make(0.0f, 1.0f) };
    SL_EXPECT(!sl_shape_make_polygon(infinite, 4u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_clockwise_winding(void)
{
    /* The same square as make_box, listed clockwise: CW is rejected,
     * not silently flipped -- in the sandbox's y-down frame a visually
     * CCW loop IS mathematically CW, and hiding that would bury the
     * frame bug. */
    const sl_vec2 points[4] = { sl_vec2_make(-1.0f, -1.0f),
                                sl_vec2_make(-1.0f, 1.0f),
                                sl_vec2_make(1.0f, 1.0f),
                                sl_vec2_make(1.0f, -1.0f) };
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 4u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_collinear_vertex(void)
{
    /* Three collinear bottoms: zero-length turn at (1, 0). */
    const sl_vec2 points[4] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(1.0f, 0.0f),
                                sl_vec2_make(2.0f, 0.0f),
                                sl_vec2_make(1.0f, 1.0f) };
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 4u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_near_duplicate_vertices(void)
{
    /* Fifth vertex one epsilon-class step from the first: the closing
     * edge falls below the resolvable-edge cutoff. */
    const sl_vec2 points[5] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(4.0f, 0.0f),
                                sl_vec2_make(4.0f, 4.0f),
                                sl_vec2_make(0.0f, 4.0f),
                                sl_vec2_make(1e-7f, 0.0f) };
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 5u, &out));

    /* Exact repeat of the first corner: same cutoff, zero length. */
    const sl_vec2 repeated[5] = { sl_vec2_make(0.0f, 0.0f),
                                  sl_vec2_make(4.0f, 0.0f),
                                  sl_vec2_make(4.0f, 4.0f),
                                  sl_vec2_make(0.0f, 4.0f),
                                  sl_vec2_make(0.0f, 0.0f) };
    SL_EXPECT(!sl_shape_make_polygon(repeated, 5u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_reflex_vertex(void)
{
    /* Simple but concave: (2, 1) turns right while every other vertex
     * turns left. */
    const sl_vec2 points[5] = { sl_vec2_make(0.0f, 0.0f),
                                sl_vec2_make(4.0f, 0.0f),
                                sl_vec2_make(4.0f, 4.0f),
                                sl_vec2_make(2.0f, 1.0f),
                                sl_vec2_make(0.0f, 4.0f) };
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(points, 5u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_self_intersecting_star(void)
{
    /* Regular pentagon vertices visited 0, 2, 4, 1, 3: every turn is
     * left and below pi (so strict convexity alone cannot see it), but
     * the total turn is 4*pi -- the simplicity rule's reason to exist. */
    sl_vec2 pentagon[5];
    for (uint32_t k = 0u; k < 5u; ++k) {
        const double angle = (double)k * (2.0 * (double)SL_PI) / 5.0;
        pentagon[k] = sl_vec2_make((float)cos(angle), (float)sin(angle));
    }
    const uint32_t star_order[5] = { 0u, 2u, 4u, 1u, 3u };
    sl_vec2 star[5];
    for (uint32_t i = 0u; i < 5u; ++i) {
        star[i] = pentagon[star_order[i]];
    }

    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_polygon(star, 5u, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void polygon_make_rejects_overflowing_geometry(void)
{
    /* Extents whose edge lengths overflow squared length. */
    sl_shape out = sl_shape_none();
    SL_EXPECT(!sl_shape_make_box(1e20f, 1e20f, &out));
    SL_EXPECT(!sl_shape_make_box(NAN, 1.0f, &out));
    SL_EXPECT(out.kind == SL_SHAPE_NONE);
}

static void shape_is_valid_accepts_made_rejects_tampered(void)
{
    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &box));
    SL_EXPECT(sl_shape_is_valid(&box));

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.5f, &circle));
    SL_EXPECT(sl_shape_is_valid(&circle));
    SL_EXPECT(sl_shape_is_valid(&box));

    const sl_shape none = sl_shape_none();
    SL_EXPECT(sl_shape_is_valid(&none));

    /* Nudged vertex: recentering recomputation drifts beyond tolerance. */
    sl_shape nudged = box;
    nudged.polygon.vertices[0].x += 0.25f;
    SL_EXPECT(!sl_shape_is_valid(&nudged));

    /* Translating every vertex leaves finiteness, edge lengths,
     * convexity and winding untouched -- only the centering check
     * catches it, so it is not redundant with the geometry rules. */
    sl_shape shifted = box;
    for (uint32_t i = 0u; i < shifted.polygon.count; ++i) {
        shifted.polygon.vertices[i].x += 100.0f;
    }
    SL_EXPECT(!sl_shape_is_valid(&shifted));

    sl_shape bad_count = box;
    bad_count.polygon.count = 9u;
    SL_EXPECT(!sl_shape_is_valid(&bad_count));

    sl_shape bad_radius = circle;
    bad_radius.circle.radius = 0.0f;
    SL_EXPECT(!sl_shape_is_valid(&bad_radius));
    bad_radius.circle.radius = SL_SHAPE_EXTENT_MAX + 1.0f;
    SL_EXPECT(!sl_shape_is_valid(&bad_radius));

    sl_shape oversized = box;
    for (uint32_t i = 0u; i < oversized.polygon.count; ++i) {
        oversized.polygon.vertices[i] = sl_vec2_scale(
            oversized.polygon.vertices[i], 2.0f * SL_SHAPE_EXTENT_MAX);
    }
    SL_EXPECT(!sl_shape_is_valid(&oversized));

    sl_shape bad_kind = box;
    bad_kind.kind = (sl_shape_kind)7;
    SL_EXPECT(!sl_shape_is_valid(&bad_kind));

    SL_EXPECT(!sl_shape_is_valid(NULL));
}

static void polygon_aabb_bounds_rotated_box(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &shape));

    /* Quarter turn swaps the half-extents. */
    const sl_aabb quarter =
        sl_shape_aabb(&shape, tf_at(0.0f, 0.0f, SL_PI / 2.0f));
    SL_EXPECT_NEAR(quarter.lower.x, -0.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(quarter.lower.y, -1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(quarter.upper.x, 0.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(quarter.upper.y, 1.0f, SL_TEST_EPS);

    /* Diagonal orientation: |cos|*hw + |sin|*hh per axis =
     * (1 + 0.5)*sqrt(2)/2 = 1.0606602. */
    const sl_aabb diagonal =
        sl_shape_aabb(&shape, tf_at(2.0f, -1.0f, SL_PI / 4.0f));
    SL_EXPECT_NEAR(diagonal.upper.x - diagonal.lower.x, 2.1213204f,
                   SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(diagonal.lower.x, 2.0f - 1.0606602f, SL_TEST_SCALE_EPS);
    SL_EXPECT_NEAR(diagonal.upper.y, -1.0f + 1.0606602f, SL_TEST_SCALE_EPS);
}

static void contains_point_circle_boundary_inclusive(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &shape));
    const sl_transform tf = tf_at(5.0f, 5.0f, 0.0f);

    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(5.0f, 5.0f)));
    /* Boundary inclusivity needs probes whose squared distance computes
     * exactly: integer offsets on the axes give dist_sq == r^2 bitwise. */
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(7.0f, 5.0f)));
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(3.0f, 5.0f)));
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(5.0f, 7.0f)));
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(5.0f, 3.0f)));
    /* Diagonal probes stay clear of the boundary: float trig constants
     * cannot reconstruct an exact boundary point. */
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(6.0f, 6.0f)));
    SL_EXPECT(!sl_shape_contains_point(&shape, tf, sl_vec2_make(7.001f, 5.0f)));
    SL_EXPECT(!sl_shape_contains_point(&shape, tf, sl_vec2_make(NAN, 5.0f)));
}

static void contains_point_polygon_uses_inverse_transform(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &shape));
    const sl_transform tf = tf_at(10.0f, 20.0f, 0.7f);

    /* Center maps into the shape. Local-frame probes are scaled clear
     * of the boundary: mapping exact corners through double trig and
     * back through float loses ULPs, which the boundary-inclusive test
     * must not be asked to absorb. */
    SL_EXPECT(sl_shape_contains_point(&shape, tf, sl_vec2_make(10.0f, 20.0f)));

    const double c = cos(0.7);
    const double s = sin(0.7);
    for (uint32_t i = 0u; i < 4u; ++i) {
        const double lx = ((i == 1u || i == 2u) ? 1.0 : -1.0) * 0.9;
        const double ly = ((i >= 2u) ? 0.5 : -0.5) * 0.9;
        const sl_vec2 world = sl_vec2_make((float)(10.0 + lx * c - ly * s),
                                           (float)(20.0 + lx * s + ly * c));
        SL_EXPECT(sl_shape_contains_point(&shape, tf, world));
    }

    /* Outside probes built in the local frame: half a length out along
     * local +y and three along local +x. */
    const sl_vec2 outside_near =
        sl_vec2_make((float)(10.0 - 0.55 * s), (float)(20.0 + 0.55 * c));
    SL_EXPECT(!sl_shape_contains_point(&shape, tf, outside_near));
    const sl_vec2 outside_far =
        sl_vec2_make((float)(10.0 + 3.0 * c), (float)(20.0 + 3.0 * s));
    SL_EXPECT(!sl_shape_contains_point(&shape, tf, outside_far));
}

static void ray_cast_circle_hits_front_face_with_outward_normal(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &shape));
    const sl_transform tf = tf_at(5.0f, 0.0f, 0.0f);

    const sl_ray through = { sl_vec2_make(0.0f, 0.0f),
                             sl_vec2_make(10.0f, 0.0f) };
    sl_ray_hit hit = hit_sentinel();
    SL_EXPECT(sl_shape_ray_cast(&shape, tf, through, &hit));
    SL_EXPECT_NEAR(hit.fraction, 0.4f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.x, 4.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.y, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.x, -1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.y, 0.0f, SL_TEST_EPS);

    /* Grazing tangent above the center: discriminant exactly zero. */
    const sl_ray graze = { sl_vec2_make(0.0f, 1.0f),
                           sl_vec2_make(10.0f, 0.0f) };
    hit = hit_sentinel();
    SL_EXPECT(sl_shape_ray_cast(&shape, tf, graze, &hit));
    SL_EXPECT_NEAR(hit.fraction, 0.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.x, 5.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.y, 1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.x, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.y, 1.0f, SL_TEST_EPS);
}

static void ray_cast_circle_from_inside_or_boundary_misses(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &shape));
    const sl_transform tf = tf_at(5.0f, 0.0f, 0.0f);

    const sl_ray from_center = { sl_vec2_make(5.0f, 0.0f),
                                 sl_vec2_make(3.0f, 0.0f) };
    sl_ray_hit hit = hit_sentinel();
    SL_EXPECT(!sl_shape_ray_cast(&shape, tf, from_center, &hit));
    SL_EXPECT(hit_untouched(&hit));

    /* Origin exactly on the boundary: the Box2D convention treats it as
     * inside, so it misses. */
    const sl_ray from_boundary = { sl_vec2_make(6.0f, 0.0f),
                                   sl_vec2_make(-3.0f, 0.0f) };
    hit = hit_sentinel();
    SL_EXPECT(!sl_shape_ray_cast(&shape, tf, from_boundary, &hit));
    SL_EXPECT(hit_untouched(&hit));
}

static void ray_cast_polygon_hits_edge_with_outward_normal(void)
{
    /* Box (2, 0.5) rotated a quarter turn at (5, 0): world half-extents
     * (0.5, 2); the ray enters the world-left face, locally the top. */
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(2.0f, 0.5f, &shape));
    const sl_transform tf = tf_at(5.0f, 0.0f, SL_PI / 2.0f);

    const sl_ray ray = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(10.0f, 0.0f) };
    sl_ray_hit hit = hit_sentinel();
    SL_EXPECT(sl_shape_ray_cast(&shape, tf, ray, &hit));
    SL_EXPECT_NEAR(hit.fraction, 0.45f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.x, 4.5f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.point.y, 0.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.x, -1.0f, SL_TEST_EPS);
    SL_EXPECT_NEAR(hit.normal.y, 0.0f, SL_TEST_EPS);
}

static void ray_cast_polygon_from_inside_or_parallel_misses(void)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(2.0f, 0.5f, &shape));

    const sl_ray inside = { sl_vec2_make(5.0f, 0.0f),
                            sl_vec2_make(3.0f, 0.0f) };
    sl_ray_hit hit = hit_sentinel();
    SL_EXPECT(
        !sl_shape_ray_cast(&shape, tf_at(5.0f, 0.0f, 0.0f), inside, &hit));
    SL_EXPECT(hit_untouched(&hit));

    /* Above the top face, parallel to both horizontal edges. */
    const sl_ray parallel = { sl_vec2_make(0.0f, 1.0f),
                              sl_vec2_make(10.0f, 0.0f) };
    hit = hit_sentinel();
    SL_EXPECT(
        !sl_shape_ray_cast(&shape, tf_at(5.0f, 0.0f, 0.0f), parallel, &hit));
    SL_EXPECT(hit_untouched(&hit));
}

static void ray_cast_rejects_malformed_ray(void)
{
    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    const sl_transform tf = tf_at(5.0f, 0.0f, 0.0f);
    sl_shape none = sl_shape_none();

    const sl_ray valid = { sl_vec2_make(0.0f, 0.0f),
                           sl_vec2_make(10.0f, 0.0f) };
    const sl_ray rays[] = {
        { sl_vec2_make(NAN, 0.0f), sl_vec2_make(10.0f, 0.0f) },
        { sl_vec2_make(0.0f, INFINITY), sl_vec2_make(10.0f, 0.0f) },
        { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(NAN, 0.0f) },
        { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f) },
        { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(1e-7f, 0.0f) },
    };

    for (uint32_t i = 0u; i < sizeof(rays) / sizeof(rays[0]); ++i) {
        sl_ray_hit hit = hit_sentinel();
        SL_EXPECT(!sl_shape_ray_cast(&circle, tf, rays[i], &hit));
        SL_EXPECT(hit_untouched(&hit));
    }

    /* NONE swallows well-formed rays too. */
    sl_ray_hit hit = hit_sentinel();
    SL_EXPECT(!sl_shape_ray_cast(&none, tf, valid, &hit));
    SL_EXPECT(hit_untouched(&hit));
}

/* Seeded property sweep: xorshift32 (Marsaglia) driving random convex
 * polygons through construction, validation, mass properties, bounds,
 * containment, and ray casts against double-precision oracles. One
 * ellipse per polygon -- semi-axes sampled independently, vertices on
 * that ellipse at strictly increasing central angles -- so convexity
 * holds by construction and make == true is a hard assertion, never
 * rejection sampling. Worst-case turn angle lands near 1.5e-3 rad
 * (100:1 axes, most compressed gaps at n = 8), three orders above the
 * ~1e-6 convexity threshold. Seed committed so failures reproduce. */
#define SHAPE_SEED 0x5EEDFACEu
#define SHAPE_SAMPLES 256u
#define SHAPE_PROBES 16u

static uint32_t prop_rng_state = SHAPE_SEED;

static uint32_t prop_rng_next(void)
{
    prop_rng_state ^= prop_rng_state << 13u;
    prop_rng_state ^= prop_rng_state >> 17u;
    prop_rng_state ^= prop_rng_state << 5u;
    return prop_rng_state;
}

/* Uniform in [0, 1): 24 random bits land exactly in float's mantissa. */
static double prop_rng_unit(void)
{
    return (double)(prop_rng_next() >> 8) * (1.0 / 16777216.0);
}

static double prop_rng_signed(double magnitude)
{
    return (prop_rng_unit() * 2.0 - 1.0) * magnitude;
}

static double d_cross(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}

/* Fan mass about the origin in double precision: the oracle for the
 * engine's float pipeline. */
static void d_fan_mass(const sl_polygon *p, double *area, double *cx,
                       double *cy, double *inertia_centroid)
{
    double area_twice = 0.0;
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double inertia_num = 0.0;
    for (uint32_t i = 0u; i < p->count; ++i) {
        const uint32_t next = (i + 1u == p->count) ? 0u : i + 1u;
        const double x1 = (double)p->vertices[i].x;
        const double y1 = (double)p->vertices[i].y;
        const double x2 = (double)p->vertices[next].x;
        const double y2 = (double)p->vertices[next].y;
        const double d = d_cross(x1, y1, x2, y2);
        area_twice += d;
        centroid_x += (x1 + x2) * d;
        centroid_y += (y1 + y2) * d;
        inertia_num +=
            d * (x1 * x1 + y1 * y1 + (x1 * x2 + y1 * y2) + x2 * x2 + y2 * y2);
    }
    *area = area_twice / 2.0;
    *cx = centroid_x / (3.0 * area_twice);
    *cy = centroid_y / (3.0 * area_twice);
    /* Per unit mass: parallel axis to the centroid, then divide off
     * the area -- omitting this division reports total inertia and
     * reads as an exact engine-oracle ratio of *area. */
    const double about_origin = inertia_num / 12.0;
    const double per_unit_mass =
        (about_origin - *area * (*cx * *cx + *cy * *cy)) / *area;
    *inertia_centroid = per_unit_mass;
}

static void random_convex_polygons_satisfy_invariants(void)
{
    bool make_ok = true;
    bool valid_ok = true;
    bool centroid_ok = true;
    bool mass_ok = true;
    bool bounds_ok = true;
    bool query_ok = true;
    bool ray_ok = true;

    for (uint32_t sample = 0u; sample < SHAPE_SAMPLES; ++sample) {
        const uint32_t n =
            3u + prop_rng_next() % (SL_POLYGON_VERTEX_COUNT_MAX - 2u);
        const double spacing = (2.0 * (double)SL_PI) / (double)n;
        const double axis_a = 0.5 + 49.5 * prop_rng_unit();
        const double axis_b = 0.5 + 49.5 * prop_rng_unit();
        const double shift_x = prop_rng_signed(100.0);
        const double shift_y = prop_rng_signed(100.0);

        sl_vec2 points[SL_POLYGON_VERTEX_COUNT_MAX];
        double raw_x[SL_POLYGON_VERTEX_COUNT_MAX];
        double raw_y[SL_POLYGON_VERTEX_COUNT_MAX];
        for (uint32_t k = 0u; k < n; ++k) {
            const double angle =
                (double)k * spacing + prop_rng_signed(0.4 * spacing);
            raw_x[k] = axis_a * cos(angle) + shift_x;
            raw_y[k] = axis_b * sin(angle) + shift_y;
            points[k] = sl_vec2_make((float)raw_x[k], (float)raw_y[k]);
        }

        sl_shape shape = sl_shape_none();
        if (!sl_shape_make_polygon(points, n, &shape)) {
            make_ok = false;
            continue;
        }
        if (!sl_shape_is_valid(&shape)) {
            valid_ok = false;
            continue;
        }
        const sl_polygon *stored = &shape.polygon;

        float extent = 0.0f;
        for (uint32_t i = 0u; i < stored->count; ++i) {
            extent = sl_max(extent, sl_abs(stored->vertices[i].x));
            extent = sl_max(extent, sl_abs(stored->vertices[i].y));
        }

        const sl_mass_data data = sl_shape_mass_data(&shape);
        if (sl_vec2_length(data.centroid) > 1e-5f * extent) {
            centroid_ok = false;
        }

        double oracle_area = 0.0;
        double oracle_cx = 0.0;
        double oracle_cy = 0.0;
        double oracle_ipm = 0.0;
        d_fan_mass(stored, &oracle_area, &oracle_cx, &oracle_cy, &oracle_ipm);
        if (fabs((double)data.area - oracle_area) > 1e-4 * fabs(oracle_area) ||
            fabs((double)data.inertia_per_unit_mass - oracle_ipm) >
                1e-4 * fabs(oracle_ipm)) {
            mass_ok = false;
        }

        /* Queries run under a rotating, displaced transform. */
        const sl_transform tf =
            tf_at(3.0f, -2.0f, 0.35f * (float)(sample % 3u));

        const sl_aabb box = sl_shape_aabb(&shape, tf);
        for (uint32_t i = 0u; i < stored->count; ++i) {
            if (!sl_aabb_contains_point(
                    box, sl_transform_apply(tf, stored->vertices[i]))) {
                bounds_ok = false;
            }
        }

        /* Probe points agree with the double half-plane oracle wherever
         * they are not borderline (within 1e-4*extent of an edge). */
        for (uint32_t probe = 0u; probe < SHAPE_PROBES; ++probe) {
            const double u = -0.05 + 1.1 * prop_rng_unit();
            const double v = -0.05 + 1.1 * prop_rng_unit();
            const sl_vec2 point =
                sl_vec2_make((float)((double)box.lower.x +
                                     u * (double)(box.upper.x - box.lower.x)),
                             (float)((double)box.lower.y +
                                     v * (double)(box.upper.y - box.lower.y)));

            const sl_vec2 local = sl_transform_apply_inverse(tf, point);
            const double px = (double)local.x;
            const double py = (double)local.y;
            double min_signed = 1e300;
            for (uint32_t i = 0u; i < stored->count; ++i) {
                const uint32_t next = (i + 1u == stored->count) ? 0u : i + 1u;
                const double ex = (double)stored->vertices[next].x -
                                  (double)stored->vertices[i].x;
                const double ey = (double)stored->vertices[next].y -
                                  (double)stored->vertices[i].y;
                const double rx = px - (double)stored->vertices[i].x;
                const double ry = py - (double)stored->vertices[i].y;
                const double edge_len = sqrt(ex * ex + ey * ey);
                const double signed_dist = d_cross(ex, ey, rx, ry) / edge_len;
                if (signed_dist < min_signed) {
                    min_signed = signed_dist;
                }
            }
            if (fabs(min_signed) < 1e-4 * (double)extent) {
                continue;
            }
            const bool oracle_inside = min_signed >= 0.0;
            if (sl_shape_contains_point(&shape, tf, point) != oracle_inside) {
                query_ok = false;
            }
        }

        /* A ray from outside aimed at the shape center hits strictly
         * between the endpoints with an outward unit normal; from the
         * center outward it misses. */
        const double reach = 2.0 * sqrt((double)extent * (double)extent +
                                        shift_x * shift_x + shift_y * shift_y) +
                             10.0;
        const double phi = 2.0 * (double)SL_PI * prop_rng_unit();
        const sl_vec2 origin =
            sl_vec2_make((float)(reach * cos(phi)), (float)(reach * sin(phi)));
        /* Aim through the body center: the transform position is the
         * centroid, so the crossing lands strictly between the
         * endpoints. */
        const sl_vec2 delta = sl_vec2_sub(tf.position, origin);
        const sl_ray inbound = { origin, delta };
        sl_ray_hit hit = hit_sentinel();
        if (!sl_shape_ray_cast(&shape, tf, inbound, &hit)) {
            ray_ok = false;
        } else if (!(hit.fraction > 0.0f && hit.fraction < 1.0f) ||
                   fabs(1.0 - (double)sl_vec2_length(hit.normal)) > 1e-4 ||
                   !(sl_vec2_dot(hit.normal,
                                 sl_vec2_sub(hit.point, tf.position)) > 0.0f)) {
            ray_ok = false;
        }

        /* From the shape center outward: inside-origin convention
         * misses. */
        const sl_ray outbound = { tf.position,
                                  sl_vec2_make((float)reach, (float)reach) };
        if (sl_shape_ray_cast(&shape, tf, outbound, &hit)) {
            ray_ok = false;
        }
    }

    SL_EXPECT(make_ok);
    SL_EXPECT(valid_ok);
    SL_EXPECT(centroid_ok);
    SL_EXPECT(mass_ok);
    SL_EXPECT(bounds_ok);
    SL_EXPECT(query_ok);
    SL_EXPECT(ray_ok);
}

static void ray_cast_circle_preserves_small_radius_on_long_rays(void)
{
    const float radii[] = { 0.001f, 1.0f, SL_SHAPE_EXTENT_MAX };
    const sl_transform transform = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
    const sl_ray ray = { { -9216.0f, 0.0f }, { 18432.0f, 0.0f } };
    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_shape circle = sl_shape_none();
        SL_EXPECT(sl_shape_make_circle(radii[i], &circle));
        sl_ray_hit hit;
        SL_EXPECT(sl_shape_ray_cast(&circle, transform, ray, &hit));
        SL_EXPECT_NEAR(hit.point.x, -radii[i], SL_EPSILON);
        SL_EXPECT_NEAR(hit.point.y, 0.0f, SL_EPSILON);
        SL_EXPECT_NEAR(hit.normal.x, -1.0f, SL_EPSILON);
        SL_EXPECT_NEAR(hit.normal.y, 0.0f, SL_EPSILON);
        SL_EXPECT_NEAR(hit.fraction, (9216.0f - radii[i]) / 18432.0f,
                       SL_EPSILON);
    }
    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    sl_ray_hit hit;
    SL_EXPECT(sl_shape_ray_cast(
        &circle, transform, (sl_ray){ { -9216.0f, 0.5f }, { 18432.0f, 0.0f } },
        &hit));
    SL_EXPECT_NEAR(hit.point.x, -sqrtf(0.75f), SL_EPSILON);
    SL_EXPECT_NEAR(hit.normal.y, 0.5f, SL_EPSILON);
    SL_EXPECT_NEAR(sl_vec2_length(hit.normal), 1.0f, SL_EPSILON);
}

static const sl_test_case k_cases[] = {
    { "long circle rays preserve radius",
      ray_cast_circle_preserves_small_radius_on_long_rays },
    { "none_shape_is_zeroed_and_inert", none_shape_is_zeroed_and_inert },
    { "constructors_zero_unused_payload", constructors_zero_unused_payload },
    { "circle_make_rejects_bad_radius", circle_make_rejects_bad_radius },
    { "shape_extent_limit_bounds_derived_data",
      shape_extent_limit_bounds_derived_data },
    { "far_origin_polygon_is_canonical_and_valid",
      far_origin_polygon_is_canonical_and_valid },
    { "circle_mass_data_matches_closed_form",
      circle_mass_data_matches_closed_form },
    { "circle_aabb_ignores_rotation", circle_aabb_ignores_rotation },
    { "box_make_matches_closed_form", box_make_matches_closed_form },
    { "box_mass_data_matches_closed_form", box_mass_data_matches_closed_form },
    { "right_triangle_mass_data_matches_closed_form",
      right_triangle_mass_data_matches_closed_form },
    { "polygon_make_recenters_offset_input",
      polygon_make_recenters_offset_input },
    { "polygon_accepts_octagon_at_vertex_count_max",
      polygon_accepts_octagon_at_vertex_count_max },
    { "polygon_make_rejects_bad_count", polygon_make_rejects_bad_count },
    { "polygon_make_rejects_non_finite_vertex",
      polygon_make_rejects_non_finite_vertex },
    { "polygon_make_rejects_clockwise_winding",
      polygon_make_rejects_clockwise_winding },
    { "polygon_make_rejects_collinear_vertex",
      polygon_make_rejects_collinear_vertex },
    { "polygon_make_rejects_near_duplicate_vertices",
      polygon_make_rejects_near_duplicate_vertices },
    { "polygon_make_rejects_reflex_vertex",
      polygon_make_rejects_reflex_vertex },
    { "polygon_make_rejects_self_intersecting_star",
      polygon_make_rejects_self_intersecting_star },
    { "polygon_make_rejects_overflowing_geometry",
      polygon_make_rejects_overflowing_geometry },
    { "shape_is_valid_accepts_made_rejects_tampered",
      shape_is_valid_accepts_made_rejects_tampered },
    { "polygon_aabb_bounds_rotated_box", polygon_aabb_bounds_rotated_box },
    { "contains_point_circle_boundary_inclusive",
      contains_point_circle_boundary_inclusive },
    { "contains_point_polygon_uses_inverse_transform",
      contains_point_polygon_uses_inverse_transform },
    { "ray_cast_circle_hits_front_face_with_outward_normal",
      ray_cast_circle_hits_front_face_with_outward_normal },
    { "ray_cast_circle_from_inside_or_boundary_misses",
      ray_cast_circle_from_inside_or_boundary_misses },
    { "ray_cast_polygon_hits_edge_with_outward_normal",
      ray_cast_polygon_hits_edge_with_outward_normal },
    { "ray_cast_polygon_from_inside_or_parallel_misses",
      ray_cast_polygon_from_inside_or_parallel_misses },
    { "ray_cast_rejects_malformed_ray", ray_cast_rejects_malformed_ray },
    { "random_convex_polygons_satisfy_invariants",
      random_convex_polygons_satisfy_invariants },
};

int sl_shape_suite(void)
{
    return sl_run_suite("shape", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
