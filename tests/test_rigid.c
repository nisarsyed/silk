#include <math.h>
#include <string.h>

#include "fixture.h"
#include "rigid.h"
#include "silk_test.h"
#include "suites.h"

static sl_quat sl_rot(float x, float y, float z, float angle)
{
    return sl_quat_from_axis_angle(sl_vec3_normalize(sl_vec3_make(x, y, z)),
                                   angle);
}

static sl_vec3 sl_world_point(sl_vec3 p, sl_quat q, sl_vec3 local)
{
    return sl_vec3_add(p, sl_quat_rotate(q, local));
}

/* True when world point w lies in the box, grown by tol. */
static bool sl_in_box(sl_vec3 w, sl_vec3 p, sl_quat q, sl_vec3 h, float tol)
{
    const sl_vec3 l = sl_quat_rotate_inv(q, sl_vec3_sub(w, p));
    return sl_abs(l.x) <= h.x + tol && sl_abs(l.y) <= h.y + tol &&
           sl_abs(l.z) <= h.z + tol;
}

/* Every point: on or in both boxes, separation consistent with the
 * normal, normal unit. */
static void sl_expect_consistent(const sl_manifold *m, sl_vec3 pa, sl_quat qa,
                                 sl_vec3 ha, sl_vec3 pb, sl_quat qb, sl_vec3 hb)
{
    SL_EXPECT_NEAR(sl_vec3_length(m->normal), 1.0f, 1e-4f);
    for (uint32_t k = 0; k < m->point_count; k++) {
        const sl_contact_point *c = &m->points[k];
        const sl_vec3 wa = sl_world_point(pa, qa, c->local_a);
        const sl_vec3 wb = sl_world_point(pb, qb, c->local_b);
        SL_EXPECT(sl_in_box(wa, pa, qa, ha, 1e-3f));
        SL_EXPECT(sl_in_box(wb, pb, qb, hb, 1e-3f));
        SL_EXPECT_NEAR(sl_vec3_dot(sl_vec3_sub(wb, wa), m->normal),
                       c->separation, 1e-4f);
        for (uint32_t j = 0; j < k; j++) {
            SL_EXPECT(m->points[j].id != c->id);
        }
    }
}

static void test_quat_rotation(void)
{
    uint32_t seed = 0x51F15EEDu;
    for (uint32_t k = 0; k < 200u; k++) {
        const sl_quat a = sl_rot(sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, 0.1f, 1.0f),
                                 sl_test_randf(&seed, -3.0f, 3.0f));
        const sl_quat b = sl_rot(sl_test_randf(&seed, 0.1f, 1.0f),
                                 sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, -3.0f, 3.0f));
        const sl_vec3 v = sl_vec3_make(sl_test_randf(&seed, -2.0f, 2.0f),
                                       sl_test_randf(&seed, -2.0f, 2.0f),
                                       sl_test_randf(&seed, -2.0f, 2.0f));
        const sl_vec3 r = sl_quat_rotate(a, v);
        const sl_vec3 m = sl_mat3_mul_vec3(sl_quat_to_mat3(a), v);
        SL_EXPECT_NEAR(r.x, m.x, 1e-5f);
        SL_EXPECT_NEAR(r.y, m.y, 1e-5f);
        SL_EXPECT_NEAR(r.z, m.z, 1e-5f);
        SL_EXPECT_NEAR(sl_vec3_length(r), sl_vec3_length(v), 1e-5f);
        const sl_vec3 back = sl_quat_rotate_inv(a, r);
        SL_EXPECT_NEAR(back.x, v.x, 1e-5f);
        SL_EXPECT_NEAR(back.y, v.y, 1e-5f);
        SL_EXPECT_NEAR(back.z, v.z, 1e-5f);
        /* (a b) v = a (b v) */
        const sl_vec3 ab = sl_quat_rotate(sl_quat_mul(a, b), v);
        const sl_vec3 a_b = sl_quat_rotate(a, sl_quat_rotate(b, v));
        SL_EXPECT_NEAR(ab.x, a_b.x, 1e-5f);
        SL_EXPECT_NEAR(ab.y, a_b.y, 1e-5f);
        SL_EXPECT_NEAR(ab.z, a_b.z, 1e-5f);
    }
    const sl_quat zero = sl_quat_normalize(sl_quat_make(0, 0, 0, 0));
    SL_EXPECT(zero.w == 1.0f && zero.x == 0.0f);
    const sl_quat bad = sl_quat_normalize(sl_quat_make(NAN, 0, 0, 1));
    SL_EXPECT(bad.w == 1.0f);
}

static void test_box_resting_on_box(void)
{
    /* A 1 m cube on a 2 m cube, 1 cm deep: the bottom face, 4 points. */
    const sl_quat id = sl_quat_identity();
    const sl_vec3 pa = sl_vec3_zero();
    const sl_vec3 ha = sl_vec3_make(1.0f, 1.0f, 1.0f);
    const sl_vec3 pb = sl_vec3_make(0.0f, 1.49f, 0.0f);
    const sl_vec3 hb = sl_vec3_make(0.5f, 0.5f, 0.5f);
    sl_manifold m;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, id, ha, pb, id, hb, 0.0f, &m), 4);
    SL_EXPECT_NEAR(m.normal.y, 1.0f, 1e-6f);
    for (uint32_t k = 0; k < m.point_count; k++) {
        SL_EXPECT_NEAR(m.points[k].separation, -0.01f, 1e-5f);
        SL_EXPECT_NEAR(m.points[k].local_a.y, 1.0f, 1e-5f);
        SL_EXPECT_NEAR(m.points[k].local_b.y, -0.5f, 1e-5f);
        SL_EXPECT_NEAR(sl_abs(m.points[k].local_b.x), 0.5f, 1e-5f);
        SL_EXPECT_NEAR(sl_abs(m.points[k].local_b.z), 0.5f, 1e-5f);
    }
    sl_expect_consistent(&m, pa, id, ha, pb, id, hb);
    /* Equal faces: a stays the reference (hysteresis), flip bit clear. */
    for (uint32_t k = 0; k < m.point_count; k++) {
        SL_EXPECT((m.points[k].id & (1u << 12)) == 0u);
    }
    /* Swapped arguments: the normal flips. */
    sl_manifold s;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pb, id, hb, pa, id, ha, 0.0f, &s), 4);
    SL_EXPECT_NEAR(s.normal.y, -1.0f, 1e-6f);
    sl_expect_consistent(&s, pb, id, hb, pa, id, ha);
}

static void test_offset_face_is_clipped(void)
{
    /* Two 2 m cubes, the upper shifted 1.5 m in x: points lie in the 0.5 m
     * wide overlap. */
    const sl_quat id = sl_quat_identity();
    const sl_vec3 h = sl_vec3_make(1.0f, 1.0f, 1.0f);
    const sl_vec3 pa = sl_vec3_zero();
    const sl_vec3 pb = sl_vec3_make(1.5f, 1.995f, 0.0f);
    sl_manifold m;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, id, h, pb, id, h, 0.0f, &m), 4);
    for (uint32_t k = 0; k < m.point_count; k++) {
        const sl_vec3 w = sl_world_point(pa, id, m.points[k].local_a);
        SL_EXPECT(w.x >= 0.5f - 1e-5f && w.x <= 1.0f + 1e-5f);
        SL_EXPECT_NEAR(m.points[k].separation, -0.005f, 1e-5f);
    }
    sl_expect_consistent(&m, pa, id, h, pb, id, h);
}

static void test_rotated_face_reduces_to_four(void)
{
    /* Equal cubes, the upper turned 45 degrees about y: the overlap is an
     * octagon, reduced to 4 points spanning most of it. */
    const sl_quat id = sl_quat_identity();
    const sl_quat turn = sl_rot(0.0f, 1.0f, 0.0f, 0.25f * SL_PI);
    const sl_vec3 h = sl_vec3_make(1.0f, 1.0f, 1.0f);
    const sl_vec3 pa = sl_vec3_zero();
    const sl_vec3 pb = sl_vec3_make(0.0f, 1.99f, 0.0f);
    sl_manifold m;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, id, h, pb, turn, h, 0.0f, &m), 4);
    sl_expect_consistent(&m, pa, id, h, pb, turn, h);
    /* Area of the kept quadrilateral (shoelace in xz) against the octagon,
     * 8 (sqrt(2) - 1) = 3.31 m^2: the four points keep over half. */
    sl_vec3 w[4];
    for (uint32_t k = 0; k < 4u; k++) {
        w[k] = sl_world_point(pa, id, m.points[k].local_a);
    }
    /* Order around the centroid for the shoelace formula. */
    float angle[4];
    for (uint32_t k = 0; k < 4u; k++) {
        angle[k] = atan2f(w[k].z, w[k].x);
    }
    for (uint32_t a = 1; a < 4u; a++) {
        for (uint32_t b = a; b > 0u && angle[b - 1u] > angle[b]; b--) {
            const float t = angle[b];
            angle[b] = angle[b - 1u];
            angle[b - 1u] = t;
            const sl_vec3 v = w[b];
            w[b] = w[b - 1u];
            w[b - 1u] = v;
        }
    }
    float area = 0.0f;
    for (uint32_t k = 0; k < 4u; k++) {
        const sl_vec3 p = w[k];
        const sl_vec3 q = w[(k + 1u) % 4u];
        area += p.x * q.z - q.x * p.z;
    }
    SL_EXPECT(0.5f * sl_abs(area) > 0.5f * 8.0f * (sqrtf(2.0f) - 1.0f));
}

static void test_crossed_edges(void)
{
    /* Two cubes on edge, the lower turned about z and the upper about x,
     * so their edges cross at right angles: one point, normal +y. */
    const sl_vec3 h = sl_vec3_make(1.0f, 1.0f, 1.0f);
    const sl_quat qa = sl_rot(0.0f, 0.0f, 1.0f, 0.25f * SL_PI);
    const sl_quat qb = sl_rot(1.0f, 0.0f, 0.0f, 0.25f * SL_PI);
    const float reach = sqrtf(2.0f);
    const sl_vec3 pa = sl_vec3_zero();
    const sl_vec3 pb = sl_vec3_make(0.0f, 2.0f * reach - 0.01f, 0.0f);
    sl_manifold m;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, qa, h, pb, qb, h, 0.0f, &m), 1);
    SL_EXPECT_NEAR(m.normal.y, 1.0f, 1e-5f);
    SL_EXPECT_NEAR(m.points[0].separation, -0.01f, 1e-5f);
    SL_EXPECT((m.points[0].id & 0x10000u) != 0u);
    const sl_vec3 w = sl_world_point(pa, qa, m.points[0].local_a);
    SL_EXPECT_NEAR(w.x, 0.0f, 1e-5f);
    SL_EXPECT_NEAR(w.z, 0.0f, 1e-5f);
    SL_EXPECT_NEAR(w.y, reach, 1e-5f);
    sl_expect_consistent(&m, pa, qa, h, pb, qb, h);
}

static void test_margin_keeps_speculative_points(void)
{
    const sl_quat id = sl_quat_identity();
    const sl_vec3 h = sl_vec3_make(0.5f, 0.5f, 0.5f);
    const sl_vec3 pa = sl_vec3_zero();
    const sl_vec3 pb = sl_vec3_make(0.0f, 1.02f, 0.0f); /* 2 cm gap */
    sl_manifold m;
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, id, h, pb, id, h, 0.01f, &m), 0);
    SL_EXPECT_INT_EQ(sl_collide_box_box(pa, id, h, pb, id, h, 0.05f, &m), 4);
    for (uint32_t k = 0; k < m.point_count; k++) {
        SL_EXPECT_NEAR(m.points[k].separation, 0.02f, 1e-5f);
    }
}

static void test_box_on_plane(void)
{
    const sl_rigid_plane ground = { sl_vec3_zero(),
                                    sl_vec3_make(0.0f, 1.0f, 0.0f) };
    const sl_vec3 h = sl_vec3_make(0.5f, 0.5f, 0.5f);
    sl_manifold m;
    /* Flat, 1 cm deep: 4 points, normal from the box into the plane. */
    SL_EXPECT_INT_EQ(sl_collide_box_plane(sl_vec3_make(0.0f, 0.49f, 0.0f),
                                          sl_quat_identity(), h, &ground, 0.0f,
                                          &m),
                     4);
    SL_EXPECT_NEAR(m.normal.y, -1.0f, 1e-6f);
    for (uint32_t k = 0; k < m.point_count; k++) {
        SL_EXPECT_NEAR(m.points[k].separation, -0.01f, 1e-5f);
        SL_EXPECT_NEAR(m.points[k].local_b.y, 0.0f, 1e-6f);
    }
    /* Tilted about x: the lower edge, 2 points. */
    const sl_quat tilt = sl_rot(1.0f, 0.0f, 0.0f, 0.3f);
    const float low = 0.5f * (cosf(0.3f) + sinf(0.3f));
    SL_EXPECT_INT_EQ(sl_collide_box_plane(sl_vec3_make(0.0f, low - 0.01f, 0.0f),
                                          tilt, h, &ground, 0.0f, &m),
                     2);
    /* Tilted about two axes: one corner. */
    const sl_quat twist = sl_quat_mul(sl_rot(0.0f, 0.0f, 1.0f, 0.4f), tilt);
    sl_manifold probe;
    (void)sl_collide_box_plane(sl_vec3_make(0.0f, 2.0f, 0.0f), twist, h,
                               &ground, 10.0f, &probe);
    float lowest = INFINITY;
    for (uint32_t k = 0; k < probe.point_count; k++) {
        lowest = sl_min(lowest, probe.points[k].separation);
    }
    const float depth_y = 2.0f - lowest; /* height of the lowest corner */
    SL_EXPECT_INT_EQ(
        sl_collide_box_plane(sl_vec3_make(0.0f, depth_y - 0.01f, 0.0f), twist,
                             h, &ground, 0.0f, &m),
        1);
    SL_EXPECT_NEAR(m.points[0].separation, -0.01f, 1e-4f);
}

static void test_random_pairs_are_consistent(void)
{
    /* Random boxes and poses near each other. Contacts must lie on both
     * boxes with consistent separations; "no contact" must mean no vertex
     * of either box is inside the other. */
    uint32_t seed = 0xB0C5u;
    uint32_t touching = 0;
    for (uint32_t k = 0; k < 2000u; k++) {
        const sl_vec3 ha = sl_vec3_make(sl_test_randf(&seed, 0.1f, 1.0f),
                                        sl_test_randf(&seed, 0.1f, 1.0f),
                                        sl_test_randf(&seed, 0.1f, 1.0f));
        const sl_vec3 hb = sl_vec3_make(sl_test_randf(&seed, 0.1f, 1.0f),
                                        sl_test_randf(&seed, 0.1f, 1.0f),
                                        sl_test_randf(&seed, 0.1f, 1.0f));
        const sl_quat qa = sl_rot(sl_test_randf(&seed, -1.0f, 1.0f),
                                  sl_test_randf(&seed, -1.0f, 1.0f),
                                  sl_test_randf(&seed, 0.1f, 1.0f),
                                  sl_test_randf(&seed, -3.0f, 3.0f));
        const sl_quat qb = sl_rot(sl_test_randf(&seed, 0.1f, 1.0f),
                                  sl_test_randf(&seed, -1.0f, 1.0f),
                                  sl_test_randf(&seed, -1.0f, 1.0f),
                                  sl_test_randf(&seed, -3.0f, 3.0f));
        const sl_vec3 pa = sl_vec3_zero();
        const sl_vec3 pb = sl_vec3_make(sl_test_randf(&seed, -1.5f, 1.5f),
                                        sl_test_randf(&seed, -1.5f, 1.5f),
                                        sl_test_randf(&seed, -1.5f, 1.5f));
        sl_manifold m;
        const uint32_t n = sl_collide_box_box(pa, qa, ha, pb, qb, hb, 0.0f, &m);
        SL_EXPECT(n <= SL_MANIFOLD_POINT_COUNT_MAX);
        if (n > 0u) {
            touching++;
            sl_expect_consistent(&m, pa, qa, ha, pb, qb, hb);
            continue;
        }
        for (uint32_t v = 0; v < 8u; v++) {
            const sl_vec3 sa =
                sl_vec3_make((v & 1u) ? -1.0f : 1.0f, (v & 2u) ? -1.0f : 1.0f,
                             (v & 4u) ? -1.0f : 1.0f);
            const sl_vec3 la =
                sl_vec3_make(sa.x * ha.x, sa.y * ha.y, sa.z * ha.z);
            const sl_vec3 lb =
                sl_vec3_make(sa.x * hb.x, sa.y * hb.y, sa.z * hb.z);
            SL_EXPECT(
                !sl_in_box(sl_world_point(pa, qa, la), pb, qb, hb, -1e-4f));
            SL_EXPECT(
                !sl_in_box(sl_world_point(pb, qb, lb), pa, qa, ha, -1e-4f));
        }
    }
    /* Both outcomes are exercised. */
    SL_EXPECT(touching > 200u && touching < 1800u);
}

static void test_feature_ids_are_stable(void)
{
    /* A smaller box on a larger one, nudged and twisted slightly: no
     * features coincide, so every point keeps its id. */
    const sl_vec3 ha = sl_vec3_make(1.0f, 0.5f, 1.0f);
    const sl_vec3 hb = sl_vec3_make(0.4f, 0.3f, 0.3f);
    const sl_vec3 pa = sl_vec3_zero();
    const sl_quat id = sl_quat_identity();
    uint32_t seed = 0x1D5u;
    for (uint32_t k = 0; k < 20u; k++) {
        const sl_vec3 pb =
            sl_vec3_make(sl_test_randf(&seed, -0.5f, 0.5f), 0.79f,
                         sl_test_randf(&seed, -0.5f, 0.5f));
        const sl_quat qb =
            sl_rot(0.0f, 1.0f, 0.0f, sl_test_randf(&seed, -1.0f, 1.0f));
        sl_manifold m0;
        sl_manifold m1;
        SL_EXPECT(sl_collide_box_box(pa, id, ha, pb, qb, hb, 0.0f, &m0) > 0u);
        const sl_vec3 moved = sl_vec3_add(pb, sl_vec3_make(1e-3f, 0.0f, 0.0f));
        const sl_quat turned = sl_quat_mul(sl_rot(0.0f, 1.0f, 0.0f, 2e-3f), qb);
        SL_EXPECT_INT_EQ(
            sl_collide_box_box(pa, id, ha, moved, turned, hb, 0.0f, &m1),
            (int)m0.point_count);
        for (uint32_t p = 0; p < m0.point_count; p++) {
            SL_EXPECT_INT_EQ(m1.points[p].id, m0.points[p].id);
        }
    }
}

static void test_collide_is_deterministic_and_persistent(void)
{
    sl_body_set set;
    SL_EXPECT(sl_body_set_init(&set, 4u, 16u));
    const sl_vec3 h = sl_vec3_make(0.5f, 0.5f, 0.5f);
    uint32_t a = 0;
    uint32_t b = 0;
    SL_EXPECT(sl_body_add_box(&set, sl_vec3_make(0.0f, 0.49f, 0.0f),
                              sl_quat_identity(), h, 1000.0f, &a));
    SL_EXPECT(sl_body_add_box(&set, sl_vec3_make(0.1f, 1.48f, 0.0f),
                              sl_quat_identity(), h, 1000.0f, &b));
    const sl_rigid_plane ground = { sl_vec3_zero(),
                                    sl_vec3_make(0.0f, 1.0f, 0.0f) };
    sl_rigid_collide(&set, &ground, 1u, 0.005f);
    /* Sorted by pair: (0, 1), then (0, plane), then nothing for 1. */
    SL_EXPECT_INT_EQ(set.manifold_count, 2);
    SL_EXPECT_INT_EQ(set.manifolds[0].body_a, 0);
    SL_EXPECT_INT_EQ(set.manifolds[0].body_b, 1);
    SL_EXPECT_INT_EQ(set.manifolds[1].body_b, SL_RIGID_STATIC_BASE);
    /* A solver writes warm data; a small move keeps every point's data.
     * These equal boxes stand flush in z, so a twist can change a corner's
     * feature id (SL_CONTACT_MATCH_DISTANCE): the data follows by
     * distance. */
    for (uint32_t i = 0; i < set.manifold_count; i++) {
        for (uint32_t p = 0; p < set.manifolds[i].point_count; p++) {
            set.manifolds[i].points[p].warm[0] = (float)(10u * i + p + 1u);
        }
    }
    sl_manifold before[2];
    memcpy(before, set.manifolds, sizeof(before));
    set.position[b].x += 1e-4f;
    set.orientation[b] = sl_rot(0.0f, 1.0f, 0.0f, 1e-3f);
    sl_rigid_collide(&set, &ground, 1u, 0.005f);
    SL_EXPECT_INT_EQ(set.manifold_count, 2);
    for (uint32_t i = 0; i < 2u; i++) {
        SL_EXPECT_INT_EQ(set.manifolds[i].point_count, before[i].point_count);
        for (uint32_t p = 0; p < set.manifolds[i].point_count; p++) {
            SL_EXPECT(set.manifolds[i].points[p].warm[0] ==
                      before[i].points[p].warm[0]);
        }
    }
    /* Lifted away and back: the pair's warm data starts over. */
    const sl_vec3 rest = set.position[b];
    set.position[b].y += 1.0f;
    sl_rigid_collide(&set, &ground, 1u, 0.005f);
    SL_EXPECT_INT_EQ(set.manifold_count, 1);
    set.position[b] = rest;
    sl_rigid_collide(&set, &ground, 1u, 0.005f);
    SL_EXPECT_INT_EQ(set.manifold_count, 2);
    for (uint32_t p = 0; p < set.manifolds[0].point_count; p++) {
        SL_EXPECT(set.manifolds[0].points[p].warm[0] == 0.0f);
    }
    sl_body_set_destroy(&set);
}

/* A pile of random boxes: the broad phase finds exactly the pairs a brute
 * force O(n^2) narrow phase finds, in the same order, and two runs give
 * identical bytes. */
static uint32_t sl_random_pile(sl_body_set *set, uint32_t count, uint32_t seed)
{
    uint32_t body = 0;
    for (uint32_t k = 0; k < count; k++) {
        const sl_vec3 p = sl_vec3_make(sl_test_randf(&seed, -2.0f, 2.0f),
                                       sl_test_randf(&seed, 0.0f, 4.0f),
                                       sl_test_randf(&seed, -2.0f, 2.0f));
        const sl_vec3 h = sl_vec3_make(sl_test_randf(&seed, 0.05f, 0.3f),
                                       sl_test_randf(&seed, 0.05f, 0.3f),
                                       sl_test_randf(&seed, 0.05f, 0.3f));
        const sl_quat q = sl_rot(sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, 0.1f, 1.0f),
                                 sl_test_randf(&seed, -1.0f, 1.0f),
                                 sl_test_randf(&seed, -3.0f, 3.0f));
        /* Every tenth body static, to exercise the static-pair skip. */
        const float density = (k % 10u == 0u) ? 0.0f : 500.0f;
        (void)sl_body_add_box(set, p, q, h, density, &body);
    }
    return set->count;
}

static void test_broad_phase_matches_brute_force(void)
{
    const uint32_t n = 300u;
    const float margin = 0.01f;
    sl_body_set set;
    SL_EXPECT(sl_body_set_init(&set, n, 4096u));
    SL_EXPECT_INT_EQ(sl_random_pile(&set, n, 0x7A11u), (int)n);
    sl_rigid_collide(&set, NULL, 0u, margin);
    SL_EXPECT_INT_EQ(set.dropped, 0);
    uint32_t expected = 0;
    uint32_t at = 0;
    for (uint32_t a = 0; a < n; a++) {
        for (uint32_t b = a + 1u; b < n; b++) {
            if (set.inverse_mass[a] == 0.0f && set.inverse_mass[b] == 0.0f) {
                continue;
            }
            sl_manifold m;
            if (sl_collide_box_box(set.position[a], set.orientation[a],
                                   set.half_extent[a], set.position[b],
                                   set.orientation[b], set.half_extent[b],
                                   margin, &m) == 0u) {
                continue;
            }
            expected++;
            if (at < set.manifold_count) {
                SL_EXPECT_INT_EQ(set.manifolds[at].body_a, (int)a);
                SL_EXPECT_INT_EQ(set.manifolds[at].body_b, (int)b);
                SL_EXPECT_INT_EQ(set.manifolds[at].point_count,
                                 (int)m.point_count);
            }
            at++;
        }
    }
    SL_EXPECT_INT_EQ(set.manifold_count, (int)expected);
    SL_EXPECT(expected > 20u);

    sl_body_set again;
    SL_EXPECT(sl_body_set_init(&again, n, 4096u));
    (void)sl_random_pile(&again, n, 0x7A11u);
    sl_rigid_collide(&again, NULL, 0u, margin);
    SL_EXPECT_INT_EQ(again.manifold_count, (int)set.manifold_count);
    SL_EXPECT(memcmp(again.manifolds, set.manifolds,
                     sizeof(sl_manifold) * set.manifold_count) == 0);
    sl_body_set_destroy(&again);
    sl_body_set_destroy(&set);
}

static void test_capacity_and_validation(void)
{
    sl_body_set set;
    SL_EXPECT(!sl_body_set_init(&set, 0u, 4u));
    SL_EXPECT(!sl_body_set_init(&set, SL_BODY_COUNT_MAX + 1u, 4u));
    SL_EXPECT(!sl_body_set_init(&set, 4u, 0u));
    SL_EXPECT(sl_body_set_init(&set, 2u, 1u));
    uint32_t body = 0;
    const sl_quat id = sl_quat_identity();
    SL_EXPECT(!sl_body_add_box(&set, sl_vec3_zero(), id,
                               sl_vec3_make(0.0f, 1.0f, 1.0f), 1.0f, &body));
    SL_EXPECT(!sl_body_add_box(&set, sl_vec3_make(NAN, 0.0f, 0.0f), id,
                               sl_vec3_make(1.0f, 1.0f, 1.0f), 1.0f, &body));
    SL_EXPECT(!sl_body_add_box(&set, sl_vec3_zero(), id,
                               sl_vec3_make(1.0f, 1.0f, 1.0f), -1.0f, &body));
    SL_EXPECT_INT_EQ(set.count, 0);
    /* Mass properties: a 1 m cube of 1000 kg/m^3 has m = 1000 kg and
     * I = m / 6 about each axis. */
    SL_EXPECT(sl_body_add_box(&set, sl_vec3_zero(), id,
                              sl_vec3_make(0.5f, 0.5f, 0.5f), 1000.0f, &body));
    SL_EXPECT_NEAR(set.inverse_mass[body], 1e-3f, 1e-9f);
    SL_EXPECT_NEAR(set.inverse_inertia[body].x, 6.0f / 1000.0f, 1e-8f);
    /* Density 0 is static. */
    SL_EXPECT(sl_body_add_box(&set, sl_vec3_make(0.0f, 0.9f, 0.0f), id,
                              sl_vec3_make(0.5f, 0.5f, 0.5f), 0.0f, &body));
    SL_EXPECT(set.inverse_mass[body] == 0.0f);
    SL_EXPECT(!sl_body_add_box(&set, sl_vec3_zero(), id,
                               sl_vec3_make(1.0f, 1.0f, 1.0f), 1.0f, &body));
    /* Capacity 1: a box pair and a plane contact; one is dropped. */
    const sl_rigid_plane ground = { sl_vec3_make(0.0f, -0.49f, 0.0f),
                                    sl_vec3_make(0.0f, 1.0f, 0.0f) };
    sl_rigid_collide(&set, &ground, 1u, 0.0f);
    SL_EXPECT_INT_EQ(set.manifold_count, 1);
    SL_EXPECT_INT_EQ(set.dropped, 1);
    sl_body_set_destroy(&set);
}

static const sl_test_case k_cases[] = {
    { "quat_rotation", test_quat_rotation },
    { "box_resting_on_box", test_box_resting_on_box },
    { "offset_face_is_clipped", test_offset_face_is_clipped },
    { "rotated_face_reduces", test_rotated_face_reduces_to_four },
    { "crossed_edges", test_crossed_edges },
    { "margin_speculative", test_margin_keeps_speculative_points },
    { "box_on_plane", test_box_on_plane },
    { "random_pairs_consistent", test_random_pairs_are_consistent },
    { "feature_ids_stable", test_feature_ids_are_stable },
    { "deterministic_persistent",
      test_collide_is_deterministic_and_persistent },
    { "broad_phase_brute_force", test_broad_phase_matches_brute_force },
    { "capacity_and_validation", test_capacity_and_validation },
};

int sl_rigid_suite(void)
{
    return sl_run_suite("rigid", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
