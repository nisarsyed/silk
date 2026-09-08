#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <silk/shape.h>
#include <silk/step.h>
#include <silk/world.h>

/* Hand-derived expectations on clean fractions compare within 1e-6;
 * rejection paths and invariant encodings compare exactly. */
static const float k_eps = 1e-6f;

static sl_world make_world(float gravity_y, float linear_drag,
                           float angular_drag)
{
    sl_world_config config = { .body_capacity = 8u,
                               .gravity = sl_vec2_make(0.0f, gravity_y),
                               .linear_drag = linear_drag,
                               .angular_drag = angular_drag };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    return world;
}

static void desc_defaults_are_dynamic_point_particle(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    /* A fully zeroed descriptor is a valid spawn: dynamic, upright, at
     * rest -- except mass, which has no sound zero for a dynamic body
     * and is set explicitly. */
    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_get_type(&world, h) == SL_BODY_DYNAMIC);
    SL_EXPECT(sl_world_body_get_angle(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_NONE);
    SL_EXPECT(sl_world_body_get_friction(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_restitution(&world, h) == 0.0f);

    sl_world_destroy(&world);
}

static void materials_validate_pack_and_reset(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);
    sl_body_desc first = { .mass = 1.0f, .friction = 0.25f };
    sl_body_desc middle = { .mass = 2.0f,
                            .friction = 0.5f,
                            .restitution = 0.4f };
    sl_body_desc last = { .mass = 3.0f,
                          .friction = 0.75f,
                          .restitution = 1.0f };
    const sl_body_handle first_handle = sl_world_body_create(&world, &first);
    const sl_body_handle middle_handle = sl_world_body_create(&world, &middle);
    const sl_body_handle last_handle = sl_world_body_create(&world, &last);
    SL_EXPECT(!sl_body_handle_is_null(first_handle));
    SL_EXPECT(!sl_body_handle_is_null(middle_handle));
    SL_EXPECT(!sl_body_handle_is_null(last_handle));

    SL_EXPECT(sl_world_body_set_friction(&world, middle_handle, 0.6f));
    SL_EXPECT(sl_world_body_set_restitution(&world, middle_handle, 0.8f));
    SL_EXPECT(sl_world_body_get_friction(&world, middle_handle) == 0.6f);
    SL_EXPECT(sl_world_body_get_restitution(&world, middle_handle) == 0.8f);

    const float bad_friction[] = { -1.0f, NAN, INFINITY, -INFINITY };
    for (uint32_t i = 0u; i < sizeof(bad_friction) / sizeof(bad_friction[0]);
         ++i) {
        SL_EXPECT(!sl_world_body_set_friction(&world, middle_handle,
                                              bad_friction[i]));
        SL_EXPECT(sl_world_body_get_friction(&world, middle_handle) == 0.6f);
        sl_body_desc bad = { .mass = 1.0f, .friction = bad_friction[i] };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &bad)));
    }

    const float bad_restitution[] = { -0.1f, 1.1f, NAN, INFINITY };
    for (uint32_t i = 0u;
         i < sizeof(bad_restitution) / sizeof(bad_restitution[0]); ++i) {
        SL_EXPECT(!sl_world_body_set_restitution(&world, middle_handle,
                                                 bad_restitution[i]));
        SL_EXPECT(sl_world_body_get_restitution(&world, middle_handle) == 0.8f);
        sl_body_desc bad = { .mass = 1.0f, .restitution = bad_restitution[i] };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &bad)));
    }
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 3u);

    /* Swap-removing the first row moves the last row and every material
     * column together. */
    sl_world_body_destroy(&world, first_handle);
    SL_EXPECT(sl_world_body_get_friction(&world, last_handle) == 0.75f);
    SL_EXPECT(sl_world_body_get_restitution(&world, last_handle) == 1.0f);

    sl_world_reset(&world);
    SL_EXPECT(!sl_world_body_is_valid(&world, middle_handle));
    sl_body_desc zero_material = { .mass = 1.0f };
    const sl_body_handle reset_handle =
        sl_world_body_create(&world, &zero_material);
    SL_EXPECT(sl_world_body_get_friction(&world, reset_handle) == 0.0f);
    SL_EXPECT(sl_world_body_get_restitution(&world, reset_handle) == 0.0f);
    const uint32_t dense = world.state->slots[reset_handle.index].dense;
    SL_EXPECT(world.state->delta_positions[dense].x == 0.0f &&
              world.state->delta_positions[dense].y == 0.0f);
    SL_EXPECT(world.state->delta_rotations[dense].c == 1.0f &&
              world.state->delta_rotations[dense].s == 0.0f);

    sl_world_destroy(&world);
}

static void create_static_stores_infinite_mass_as_zero(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .position = sl_vec2_make(2.0f, -1.0f),
                          .mass = 0.0f,
                          .type = SL_BODY_STATIC };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_get_type(&world, h) == SL_BODY_STATIC);
    SL_EXPECT(sl_world_body_get_mass(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_mass(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == 0.0f);

    sl_world_destroy(&world);
}

static void create_rejects_static_or_kinematic_with_mass(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    const float masses[3] = { 1.0f, 1e-30f, INFINITY };
    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_body_desc stat = { .mass = masses[i], .type = SL_BODY_STATIC };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &stat)));
        sl_body_desc kine = { .mass = masses[i], .type = SL_BODY_KINEMATIC };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &kine)));
    }
    SL_EXPECT_INT_EQ((int)sl_world_body_count(&world), 0);

    sl_world_destroy(&world);
}

static void create_rejects_static_with_velocity(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc linear = { .velocity = sl_vec2_make(0.0f, 1.0f),
                            .type = SL_BODY_STATIC };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &linear)));

    sl_body_desc spin = { .angular_velocity = -0.5f, .type = SL_BODY_STATIC };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &spin)));

    /* Kinematic accepts both. */
    sl_body_desc kine = { .velocity = sl_vec2_make(0.0f, 1.0f),
                          .mass = 0.0f,
                          .type = SL_BODY_KINEMATIC,
                          .angular_velocity = -0.5f };
    SL_EXPECT(!sl_body_handle_is_null(sl_world_body_create(&world, &kine)));

    sl_world_destroy(&world);
}

static void create_rejects_unknown_type(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f, .type = (sl_body_type)7u };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    SL_EXPECT_INT_EQ((int)sl_world_body_count(&world), 0);

    sl_world_destroy(&world);
}

static void create_rejects_non_finite_angle_or_spin(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    const float bad[3] = { NAN, INFINITY, -INFINITY };
    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_body_desc angle = { .mass = 1.0f, .angle = bad[i] };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &angle)));
        sl_body_desc spin = { .mass = 1.0f, .angular_velocity = bad[i] };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &spin)));
    }

    sl_world_destroy(&world);
}

static void create_wraps_angle(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f, .angle = 2.5f * SL_PI };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.5f * SL_PI, k_eps);

    /* Construction wraps before sin/cos; recovery pays atan2 and is
     * therefore a tolerance comparison rather than stored scalar identity. */
    SL_EXPECT(sl_world_body_set_angle(&world, h, 1.234f));
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 1.234f, k_eps);

    const uint32_t dense = world.state->slots[h.index].dense;
    const sl_rotation before = world.state->rotations[dense];
    SL_EXPECT(!sl_world_body_set_angle(&world, h, NAN));
    SL_EXPECT(world.state->rotations[dense].c == before.c &&
              world.state->rotations[dense].s == before.s);

    sl_world_destroy(&world);
}

static void create_with_shape_derives_inertia(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &circle));

    /* I = m * r^2 / 2 = 1 * 2 = 2. */
    sl_body_desc desc = { .mass = 1.0f, .shape = &circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 2.0f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_inv_inertia(&world, h), 0.5f, k_eps);

    sl_world_destroy(&world);
}

static void create_rejects_tampered_shape_before_measuring_it(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 1.0f, &box));

    /* The complete descriptor is rejected before inertia measurement;
     * otherwise that measurement would trust count while walking the
     * fixed vertex array. */
    sl_shape tampered = box;
    tampered.polygon.count = 4000000u;
    SL_EXPECT(!sl_shape_is_valid(&tampered));

    sl_body_desc desc = { .mass = 1.0f, .shape = &tampered };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    /* Refused before a slot was consumed. */
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0u);

    sl_world_destroy(&world);
}

static void create_normalizes_circle_payload(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &circle));
    sl_shape dirty_circle = circle;
    const size_t tail = offsetof(sl_shape, circle) + sizeof(sl_circle);
    memset((unsigned char *)&dirty_circle + tail, 0xAB,
           sizeof(dirty_circle) - tail);
    SL_EXPECT(sl_shape_is_valid(&dirty_circle));
    SL_EXPECT(memcmp(&dirty_circle, &circle, sizeof(circle)) != 0);

    sl_body_desc desc = { .mass = 1.0f, .shape = &dirty_circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(memcmp(sl_world_body_get_shape(&world, h), &circle,
                     sizeof(circle)) == 0);

    sl_world_destroy(&world);
}

static void create_normalizes_polygon_vertex_tail(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &box));
    sl_shape dirty_box = box;
    const size_t tail = offsetof(sl_shape, polygon.vertices) +
                        (size_t)box.polygon.count * sizeof(sl_vec2);
    memset((unsigned char *)&dirty_box + tail, 0xCD, sizeof(dirty_box) - tail);
    SL_EXPECT(sl_shape_is_valid(&dirty_box));
    SL_EXPECT(memcmp(&dirty_box, &box, sizeof(box)) != 0);

    sl_body_desc desc = { .mass = 1.0f, .shape = &dirty_box };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(memcmp(sl_world_body_get_shape(&world, h), &box, sizeof(box)) ==
              0);

    sl_world_destroy(&world);
}

static void set_shape_normalizes_none_payload(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_shape dirty_none = sl_shape_none();
    const size_t payload = offsetof(sl_shape, circle);
    memset((unsigned char *)&dirty_none + payload, 0xEF,
           sizeof(dirty_none) - payload);
    SL_EXPECT(sl_shape_is_valid(&dirty_none));
    const sl_shape none = sl_shape_none();
    SL_EXPECT(memcmp(&dirty_none, &none, sizeof(none)) != 0);
    SL_EXPECT(sl_world_body_set_shape(&world, h, &dirty_none));
    SL_EXPECT(memcmp(sl_world_body_get_shape(&world, h), &none, sizeof(none)) ==
              0);

    sl_world_destroy(&world);
}

static void set_mass_rejects_when_banked_force_would_overflow(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    /* Accepted at this mass: 1e30 / 1e10 is a finite acceleration. */
    sl_body_desc desc = { .mass = 1e10f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(1e30f, 0.0f)));

    /* The new mass is legal on its own, and so is its derived inertia.
     * What is not legal is the force already banked against the old
     * inverse: 1e30 * 1e20 is infinity, and accepting the mass would
     * hand exactly that to the next step. */
    SL_EXPECT(!sl_world_body_set_mass(&world, h, 1e-20f));
    SL_EXPECT(sl_world_body_get_mass(&world, h) == 1e10f);
    sl_world_step(&world, 0.1f);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, h)));

    /* The step consumed the accumulator, so the objection is gone and
     * the very same mass now holds -- the guard is about what is
     * banked, not about the value being set. */
    SL_EXPECT(sl_world_body_set_mass(&world, h, 1e-20f));

    sl_world_destroy(&world);
}

static void set_shape_rejects_when_banked_torque_would_overflow(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    /* Shapeless means zero inv_inertia, so any finite torque is
     * accepted: it integrates to nothing. */
    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_apply_torque(&world, h, 1e30f));

    /* Attaching a tiny circle gives that torque somewhere to go, and
     * 1e30 / 5e-11 is not finite. */
    sl_shape tiny = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1e-5f, &tiny));
    SL_EXPECT(!sl_world_body_set_shape(&world, h, &tiny));
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_NONE);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);

    sl_world_step(&world, 0.1f);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 0.0f);
    SL_EXPECT(sl_is_finite(sl_world_body_get_angle(&world, h)));

    /* Same attachment holds once the step has drained the torque. */
    SL_EXPECT(sl_world_body_set_shape(&world, h, &tiny));

    sl_world_destroy(&world);
}

static void set_shape_on_dynamic_body_sets_inertia(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 3.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);

    /* Unit square: I/m = 1/6 about the centroid, so I = 3/6 = 0.5. */
    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box));
    SL_EXPECT(sl_world_body_set_shape(&world, h, &box));
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 0.5f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_inv_inertia(&world, h), 2.0f, k_eps);

    sl_world_destroy(&world);
}

static void set_shape_null_detaches_and_zeroes_inertia(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &circle));
    sl_body_desc desc = { .mass = 1.0f, .shape = &circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_set_shape(&world, h, NULL));
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_NONE);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == 0.0f);

    /* An explicit NONE record detaches identically. */
    SL_EXPECT(sl_world_body_set_shape(&world, h, &circle));
    const sl_shape none = sl_shape_none();
    SL_EXPECT(sl_world_body_set_shape(&world, h, &none));
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_NONE);

    sl_world_destroy(&world);
}

static void set_shape_rejects_invalid_shape(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    sl_body_desc desc = { .mass = 1.0f, .shape = &circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_shape tampered = circle;
    tampered.circle.radius = -1.0f;
    SL_EXPECT(!sl_world_body_set_shape(&world, h, &tampered));
    tampered.circle.radius = NAN;
    SL_EXPECT(!sl_world_body_set_shape(&world, h, &tampered));

    sl_shape bad_polygon = sl_shape_none();
    const sl_vec2 points[4] = { sl_vec2_make(-1.0f, -1.0f),
                                sl_vec2_make(-1.0f, 1.0f),
                                sl_vec2_make(1.0f, 1.0f),
                                sl_vec2_make(1.0f, -1.0f) };
    SL_EXPECT(!sl_shape_make_polygon(points, 4u, &bad_polygon));
    SL_EXPECT(bad_polygon.kind == SL_SHAPE_NONE);

    /* Rejections leave the attached shape and derived inertia intact. */
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_CIRCLE);
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 0.5f, k_eps);

    sl_world_destroy(&world);
}

static void set_shape_on_static_keeps_zero_inertia(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 0.0f, .type = SL_BODY_STATIC };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 1.0f, &box));
    SL_EXPECT(sl_world_body_set_shape(&world, h, &box));
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_POLYGON);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == 0.0f);

    sl_world_destroy(&world);
}

static void set_mass_rescales_inertia(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box));
    sl_body_desc desc = { .mass = 6.0f, .shape = &box };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 1.0f, k_eps);

    SL_EXPECT(sl_world_body_set_mass(&world, h, 12.0f));
    SL_EXPECT_NEAR(sl_world_body_get_mass(&world, h), 12.0f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 2.0f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_inv_inertia(&world, h), 0.5f, k_eps);

    sl_world_destroy(&world);
}

static void set_mass_rejected_on_non_dynamic(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc stat = { .type = SL_BODY_STATIC };
    sl_body_handle hs = sl_world_body_create(&world, &stat);
    SL_EXPECT(!sl_body_handle_is_null(hs));
    SL_EXPECT(!sl_world_body_set_mass(&world, hs, 5.0f));

    sl_body_desc kine = { .type = SL_BODY_KINEMATIC };
    sl_body_handle hk = sl_world_body_create(&world, &kine);
    SL_EXPECT(!sl_body_handle_is_null(hk));
    SL_EXPECT(!sl_world_body_set_mass(&world, hk, 5.0f));

    sl_world_destroy(&world);
}

/* Masses exist whose own inverse is representable but whose derived
 * inertia is denormal-small, so the inverse overflows: unit-box I/m =
 * 1/6 puts that window near m = 1e-38 (I ~ 1.67e-39, 1/I ~ 6e38 >
 * FLT_MAX). Rejection must leave every derived column untouched. */
static void set_mass_rejects_inertia_inverse_overflow(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box));
    sl_body_desc desc = { .mass = 6.0f, .shape = &box };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    const float old_mass = sl_world_body_get_mass(&world, h);
    const float old_inertia = sl_world_body_get_inertia(&world, h);
    const float old_inv_inertia = sl_world_body_get_inv_inertia(&world, h);

    /* Sanity for the window itself: the mass stays representable while
     * its derived inertia's inverse overflows. */
    volatile float tiny_mass = 1e-38f;
    SL_EXPECT(1.0f / tiny_mass < FLT_MAX);
    SL_EXPECT(1.0f / (tiny_mass / 6.0f) > FLT_MAX);
    SL_EXPECT(!sl_world_body_set_mass(&world, h, tiny_mass));
    SL_EXPECT(sl_world_body_get_mass(&world, h) == old_mass);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == old_inertia);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == old_inv_inertia);

    sl_world_destroy(&world);
}

static void set_angular_velocity_only_zero_on_static(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .type = SL_BODY_STATIC };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(!sl_world_body_set_angular_velocity(&world, h, 2.0f));
    SL_EXPECT(sl_world_body_set_angular_velocity(&world, h, 0.0f));
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 0.0f);

    sl_world_destroy(&world);
}

static void apply_torque_accumulates_on_dynamic(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_apply_torque(&world, h, 1.5f));
    SL_EXPECT(sl_world_body_apply_torque(&world, h, 2.5f));
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 4.0f);

    sl_world_destroy(&world);
}

static void apply_torque_rejected_on_non_dynamic(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc stat = { .type = SL_BODY_STATIC };
    sl_body_handle hs = sl_world_body_create(&world, &stat);
    SL_EXPECT(!sl_body_handle_is_null(hs));
    SL_EXPECT(!sl_world_body_apply_torque(&world, hs, 1.0f));

    sl_body_desc kine = { .type = SL_BODY_KINEMATIC };
    sl_body_handle hk = sl_world_body_create(&world, &kine);
    SL_EXPECT(!sl_body_handle_is_null(hk));
    SL_EXPECT(!sl_world_body_apply_torque(&world, hk, 1.0f));

    SL_EXPECT(sl_world_body_get_torque(&world, hs) == 0.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, hk) == 0.0f);

    sl_world_destroy(&world);
}

static void apply_torque_rejects_non_finite_and_overflow(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(!sl_world_body_apply_torque(&world, h, NAN));
    SL_EXPECT(!sl_world_body_apply_torque(&world, h, INFINITY));

    /* Accumulation overflow: two thirds-scale torques sum past
     * FLT_MAX (halves would land on exactly FLT_MAX, which fits). */
    const float big_torque = 0.75f * FLT_MAX;
    SL_EXPECT(sl_world_body_apply_torque(&world, h, big_torque));
    SL_EXPECT(!sl_world_body_apply_torque(&world, h, big_torque));
    SL_EXPECT(sl_world_body_get_torque(&world, h) == big_torque);

    /* Angular acceleration overflow through a tiny inertia: r = 1e-5
     * gives I/m = 5e-11, so tau = 1e30 integrates to infinity. */
    sl_shape needle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1e-5f, &needle));
    sl_body_desc spinner = { .mass = 1.0f, .shape = &needle };
    sl_body_handle s = sl_world_body_create(&world, &spinner);
    SL_EXPECT(!sl_body_handle_is_null(s));
    SL_EXPECT(!sl_world_body_apply_torque(&world, s, 1e30f));

    sl_world_destroy(&world);
}

static void apply_force_at_point_adds_cross_product_torque(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .position = sl_vec2_make(1.0f, 1.0f), .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    /* Force (0, 2) one length to the right of the center:
     * torque = cross((1, 0), (0, 2)) = 2. */
    SL_EXPECT(sl_world_body_apply_force_at_point(
        &world, h, sl_vec2_make(0.0f, 2.0f), sl_vec2_make(2.0f, 1.0f)));
    SL_EXPECT(sl_world_body_get_force(&world, h).y == 2.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 2.0f);

    /* Through the center: pure force, zero torque. */
    SL_EXPECT(sl_world_body_apply_force_at_point(
        &world, h, sl_vec2_make(0.0f, 2.0f), sl_vec2_make(1.0f, 1.0f)));
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 2.0f);
    SL_EXPECT(sl_world_body_get_force(&world, h).y == 4.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(SL_SHAPE_EXTENT_MAX, &circle));
    sl_body_desc edge = { .position = sl_vec2_make(SL_POSITION_ABS_MAX, 0.0f),
                          .mass = 1.0f,
                          .shape = &circle };
    sl_body_handle boundary = sl_world_body_create(&world, &edge);
    SL_EXPECT(!sl_body_handle_is_null(boundary));

    /* A valid body's surface reaches one shape extent beyond the legal
     * center domain. That point remains a valid force application. */
    const sl_vec2 surface =
        sl_vec2_make(SL_POSITION_ABS_MAX + SL_SHAPE_EXTENT_MAX, 0.0f);
    SL_EXPECT(sl_world_body_apply_force_at_point(
        &world, boundary, sl_vec2_make(0.0f, 1.0f), surface));
    SL_EXPECT_NEAR(sl_world_body_get_torque(&world, boundary),
                   SL_SHAPE_EXTENT_MAX, k_eps);

    sl_world_destroy(&world);
}

static void apply_force_at_point_rejects_atomically(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    /* Banked torque at three quarters of FLT_MAX: a force whose moment
     * (two lengths times the same scale) would push the accumulator
     * past infinity fails as a whole -- the force half alone would
     * have succeeded, so this pins write atomicity. */
    const float big_torque = 0.75f * FLT_MAX;
    SL_EXPECT(sl_world_body_apply_torque(&world, h, big_torque));

    SL_EXPECT(!sl_world_body_apply_force_at_point(
        &world, h, sl_vec2_make(0.0f, big_torque), sl_vec2_make(2.0f, 0.0f)));
    SL_EXPECT(sl_world_body_get_force(&world, h).x == 0.0f);
    SL_EXPECT(sl_world_body_get_force(&world, h).y == 0.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, h) == big_torque);

    /* And in the other order: a force that fails leaves no torque. */
    sl_body_desc d2 = { .mass = 1.0f };
    sl_body_handle g = sl_world_body_create(&world, &d2);
    SL_EXPECT(!sl_body_handle_is_null(g));
    SL_EXPECT(!sl_world_body_apply_force_at_point(
        &world, g, sl_vec2_make(NAN, 0.0f), sl_vec2_make(1.0f, 0.0f)));
    SL_EXPECT(sl_world_body_get_force(&world, g).x == 0.0f);
    SL_EXPECT(sl_world_body_get_force(&world, g).y == 0.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, g) == 0.0f);

    SL_EXPECT(!sl_world_body_apply_force_at_point(
        &world, g, sl_vec2_make(1.0f, 0.0f),
        sl_vec2_make(SL_POSITION_ABS_MAX + SL_SHAPE_EXTENT_MAX + 1.0f, 0.0f)));
    SL_EXPECT(sl_world_body_get_force(&world, g).x == 0.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, g) == 0.0f);

    /* Non-dynamic bodies refuse the whole operation. */
    sl_body_desc stat = { .type = SL_BODY_STATIC };
    sl_body_handle hs = sl_world_body_create(&world, &stat);
    SL_EXPECT(!sl_body_handle_is_null(hs));
    SL_EXPECT(!sl_world_body_apply_force_at_point(
        &world, hs, sl_vec2_make(1.0f, 0.0f), sl_vec2_make(2.0f, 0.0f)));

    sl_world_destroy(&world);
}

static void apply_force_rejected_on_non_dynamic(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc stat = { .type = SL_BODY_STATIC };
    sl_body_handle hs = sl_world_body_create(&world, &stat);
    SL_EXPECT(!sl_body_handle_is_null(hs));
    SL_EXPECT(!sl_world_body_apply_force(&world, hs, sl_vec2_make(1.0f, 0.0f)));

    sl_body_desc kine = { .type = SL_BODY_KINEMATIC };
    sl_body_handle hk = sl_world_body_create(&world, &kine);
    SL_EXPECT(!sl_body_handle_is_null(hk));
    SL_EXPECT(!sl_world_body_apply_force(&world, hk, sl_vec2_make(1.0f, 0.0f)));

    SL_EXPECT(sl_world_body_get_force(&world, hs).x == 0.0f);
    SL_EXPECT(sl_world_body_get_force(&world, hk).x == 0.0f);

    sl_world_destroy(&world);
}

static void get_transform_matches_position_and_angle(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .position = sl_vec2_make(7.0f, -3.0f),
                          .mass = 1.0f,
                          .angle = SL_PI / 2.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    const sl_transform tf = sl_world_body_get_transform(&world, h);
    SL_EXPECT(tf.position.x == 7.0f && tf.position.y == -3.0f);
    SL_EXPECT_NEAR(tf.rotation.c, 0.0f, k_eps);
    SL_EXPECT_NEAR(tf.rotation.s, 1.0f, k_eps);
    const uint32_t dense = world.state->slots[h.index].dense;
    SL_EXPECT(tf.rotation.c == world.state->rotations[dense].c);
    SL_EXPECT(tf.rotation.s == world.state->rotations[dense].s);

    /* Local +x maps to world +y under this frame. */
    const sl_vec2 local_x = sl_transform_apply(tf, sl_vec2_make(1.0f, 0.0f));
    SL_EXPECT_NEAR(local_x.x, 7.0f, k_eps);
    SL_EXPECT_NEAR(local_x.y, -2.0f, k_eps);

    sl_world_destroy(&world);
}

static void get_shape_reflects_set_shape(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &box));
    SL_EXPECT(sl_world_body_set_shape(&world, h, &box));

    const sl_shape *attached = sl_world_body_get_shape(&world, h);
    SL_EXPECT(attached->kind == SL_SHAPE_POLYGON);
    SL_EXPECT_INT_EQ((int)attached->polygon.count, 4);
    SL_EXPECT(attached->polygon.vertices[0].x == -1.0f &&
              attached->polygon.vertices[0].y == -0.5f);

    sl_world_destroy(&world);
}

static void destroy_swap_moves_angular_and_shape_rows(void)
{
    sl_world world = make_world(0.0f, 0.0f, 0.0f);

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 1.0f, &box));

    /* Row order after creation: A at 0, B at 1, C at 2. Destroying A
     * pulls C into row 0 -- every swapped column must follow. */
    sl_body_desc da = { .mass = 1.0f, .shape = &circle, .angle = 0.25f };
    sl_body_handle ha = sl_world_body_create(&world, &da);
    SL_EXPECT(!sl_body_handle_is_null(ha));

    sl_body_desc db = { .position = sl_vec2_make(9.0f, 9.0f),
                        .mass = 2.0f,
                        .angular_velocity = 0.75f };
    sl_body_handle hb = sl_world_body_create(&world, &db);
    SL_EXPECT(!sl_body_handle_is_null(hb));

    sl_body_desc dc = { .mass = 3.0f, .shape = &box, .angle = -1.5f };
    sl_body_handle hc = sl_world_body_create(&world, &dc);
    SL_EXPECT(!sl_body_handle_is_null(hc));

    sl_world_body_destroy(&world, ha);

    /* C now lives behind A's old handle slot generation-wise; validate
     * through its own (still-live) handle. */
    SL_EXPECT(sl_world_body_is_valid(&world, hc));
    SL_EXPECT(!sl_world_body_is_valid(&world, ha));
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, hc), -1.5f, k_eps);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, hc) == 0.0f);
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, hc), 2.0f, k_eps);
    SL_EXPECT(sl_world_body_get_shape(&world, hc)->kind == SL_SHAPE_POLYGON);
    /* B keeps its own row untouched. */
    SL_EXPECT(sl_world_body_get_shape(&world, hb)->kind == SL_SHAPE_NONE);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, hb) == 0.75f);

    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "desc_defaults_are_dynamic_point_particle",
      desc_defaults_are_dynamic_point_particle },
    { "materials_validate_pack_and_reset", materials_validate_pack_and_reset },
    { "create_static_stores_infinite_mass_as_zero",
      create_static_stores_infinite_mass_as_zero },
    { "create_rejects_static_or_kinematic_with_mass",
      create_rejects_static_or_kinematic_with_mass },
    { "create_rejects_static_with_velocity",
      create_rejects_static_with_velocity },
    { "create_rejects_unknown_type", create_rejects_unknown_type },
    { "create_rejects_non_finite_angle_or_spin",
      create_rejects_non_finite_angle_or_spin },
    { "create_wraps_angle", create_wraps_angle },
    { "create_with_shape_derives_inertia", create_with_shape_derives_inertia },
    { "create_rejects_tampered_shape_before_measuring_it",
      create_rejects_tampered_shape_before_measuring_it },
    { "create_normalizes_circle_payload", create_normalizes_circle_payload },
    { "create_normalizes_polygon_vertex_tail",
      create_normalizes_polygon_vertex_tail },
    { "set_shape_normalizes_none_payload", set_shape_normalizes_none_payload },
    { "set_mass_rejects_when_banked_force_would_overflow",
      set_mass_rejects_when_banked_force_would_overflow },
    { "set_shape_rejects_when_banked_torque_would_overflow",
      set_shape_rejects_when_banked_torque_would_overflow },
    { "set_shape_on_dynamic_body_sets_inertia",
      set_shape_on_dynamic_body_sets_inertia },
    { "set_shape_null_detaches_and_zeroes_inertia",
      set_shape_null_detaches_and_zeroes_inertia },
    { "set_shape_rejects_invalid_shape", set_shape_rejects_invalid_shape },
    { "set_shape_on_static_keeps_zero_inertia",
      set_shape_on_static_keeps_zero_inertia },
    { "set_mass_rescales_inertia", set_mass_rescales_inertia },
    { "set_mass_rejected_on_non_dynamic", set_mass_rejected_on_non_dynamic },
    { "set_mass_rejects_inertia_inverse_overflow",
      set_mass_rejects_inertia_inverse_overflow },
    { "set_angular_velocity_only_zero_on_static",
      set_angular_velocity_only_zero_on_static },
    { "apply_torque_accumulates_on_dynamic",
      apply_torque_accumulates_on_dynamic },
    { "apply_torque_rejected_on_non_dynamic",
      apply_torque_rejected_on_non_dynamic },
    { "apply_torque_rejects_non_finite_and_overflow",
      apply_torque_rejects_non_finite_and_overflow },
    { "apply_force_at_point_adds_cross_product_torque",
      apply_force_at_point_adds_cross_product_torque },
    { "apply_force_at_point_rejects_atomically",
      apply_force_at_point_rejects_atomically },
    { "apply_force_rejected_on_non_dynamic",
      apply_force_rejected_on_non_dynamic },
    { "get_transform_matches_position_and_angle",
      get_transform_matches_position_and_angle },
    { "get_shape_reflects_set_shape", get_shape_reflects_set_shape },
    { "destroy_swap_moves_angular_and_shape_rows",
      destroy_swap_moves_angular_and_shape_rows },
};

int sl_body_suite(void)
{
    return sl_run_suite("body", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
