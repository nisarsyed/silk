#include "silk_test.h"
#include "suites.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <silk/step.h>

static const float k_dt = 1.0f / 60.0f;

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

static sl_body_handle body_make(sl_world *world, sl_body_type type,
                                sl_vec2 position, float mass,
                                const sl_shape *shape)
{
    const sl_body_desc desc = {
        .position = position,
        .mass = mass,
        .type = type,
        .shape = shape,
    };
    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(body));
    return body;
}

static sl_joint_handle distance_make(sl_world *world, sl_body_handle body_a,
                                     sl_body_handle body_b, float length,
                                     bool collide_connected)
{
    const sl_joint_desc desc = {
        .kind = SL_JOINT_DISTANCE,
        .body_a = body_a,
        .body_b = body_b,
        .collide_connected = collide_connected,
        .distance = { .length = length },
    };
    const sl_joint_handle joint = sl_world_joint_create(world, &desc);
    SL_EXPECT(!sl_joint_handle_is_null(joint));
    return joint;
}

static sl_joint_handle revolute_make(sl_world *world, sl_body_handle body_a,
                                     sl_body_handle body_b,
                                     sl_vec2 local_anchor_a,
                                     sl_vec2 local_anchor_b,
                                     bool collide_connected)
{
    const sl_joint_desc desc = {
        .kind = SL_JOINT_REVOLUTE,
        .body_a = body_a,
        .body_b = body_b,
        .local_anchor_a = local_anchor_a,
        .local_anchor_b = local_anchor_b,
        .collide_connected = collide_connected,
    };
    const sl_joint_handle joint = sl_world_joint_create(world, &desc);
    SL_EXPECT(!sl_joint_handle_is_null(joint));
    return joint;
}

static void config_and_descriptor_validation_are_atomic(void)
{
    sl_world disabled = { 0 };
    const sl_world_config disabled_config = { .body_capacity = 2u };
    SL_EXPECT(sl_world_init(&disabled, &disabled_config));
    SL_EXPECT_INT_EQ(sl_world_joint_capacity(&disabled), 0u);
    const sl_body_handle disabled_a = body_make(
        &disabled, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle disabled_b = body_make(
        &disabled, SL_BODY_DYNAMIC, sl_vec2_make(1.0f, 0.0f), 1.0f, NULL);
    const sl_joint_desc disabled_desc = {
        .kind = SL_JOINT_DISTANCE,
        .body_a = disabled_a,
        .body_b = disabled_b,
        .distance = { .length = 1.0f },
    };
    SL_EXPECT(sl_joint_handle_is_null(
        sl_world_joint_create(&disabled, &disabled_desc)));
    sl_world_destroy(&disabled);

    const sl_world_config config = {
        .body_capacity = 4u,
        .joint_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT_NEAR(world.joint_hertz, SL_JOINT_HERTZ_DEFAULT, 0.0f);
    SL_EXPECT_NEAR(world.joint_damping_ratio, SL_JOINT_DAMPING_RATIO_DEFAULT,
                   0.0f);
    const sl_body_handle dynamic = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(1.0f, 0.0f), 1.0f, NULL);
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle fixed_b =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(2.0f, 0.0f), 0.0f, NULL);

    sl_joint_desc desc = {
        .kind = SL_JOINT_DISTANCE,
        .body_a = fixed,
        .body_b = dynamic,
        .distance = { .length = 1.0f },
    };
#define EXPECT_REJECTED(candidate)                                             \
    do {                                                                       \
        const uint32_t count_before = sl_world_joint_count(&world);            \
        const uint32_t free_before = world.joint_free_count;                   \
        SL_EXPECT(sl_joint_handle_is_null(                                     \
            sl_world_joint_create(&world, &(candidate))));                     \
        SL_EXPECT_INT_EQ(sl_world_joint_count(&world), count_before);          \
        SL_EXPECT_INT_EQ(world.joint_free_count, free_before);                 \
    } while (false)

    sl_joint_desc invalid = desc;
    invalid.body_b = fixed;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.body_a = fixed_b;
    invalid.body_b = fixed;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.body_b = sl_body_handle_null();
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.kind = (sl_joint_kind)77;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.local_anchor_a.x = NAN;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.local_anchor_b.y = SL_SHAPE_EXTENT_MAX + 1.0f;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.distance.length = SL_LINEAR_SLOP * 0.5f;
    EXPECT_REJECTED(invalid);
    invalid = desc;
    invalid.distance.length = 2.0f * SL_POSITION_ABS_MAX + 1.0f;
    EXPECT_REJECTED(invalid);

    const sl_joint_handle joint = sl_world_joint_create(&world, &desc);
    SL_EXPECT(sl_world_joint_is_valid(&world, joint));
    SL_EXPECT_INT_EQ(sl_world_joint_count(&world), 1u);
    SL_EXPECT_INT_EQ(sl_world_joint_at(&world, 0u).index, joint.index);
    const sl_joint_desc stored = sl_world_joint_get_desc(&world, joint);
    SL_EXPECT(stored.kind == SL_JOINT_DISTANCE);
    SL_EXPECT(stored.body_a.index == fixed.index);
    SL_EXPECT(stored.body_b.index == dynamic.index);
    SL_EXPECT(stored.distance.length == 1.0f);

    EXPECT_REJECTED(desc);
#undef EXPECT_REJECTED

    sl_world_destroy(&world);

    sl_world rejected = { 0 };
    sl_world_config invalid_config = config;
    invalid_config.joint_capacity = SL_JOINT_COUNT_MAX + 1u;
    SL_EXPECT(!sl_world_init(&rejected, &invalid_config));
    invalid_config = config;
    invalid_config.joint_hertz = INFINITY;
    SL_EXPECT(!sl_world_init(&rejected, &invalid_config));
    invalid_config = config;
    invalid_config.joint_damping_ratio = -1.0f;
    SL_EXPECT(!sl_world_init(&rejected, &invalid_config));
}

static void handles_swap_remove_reset_and_body_destroy(void)
{
    const sl_world_config config = {
        .body_capacity = 4u,
        .joint_capacity = 3u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle a = body_make(&world, SL_BODY_DYNAMIC,
                                       sl_vec2_make(1.0f, 0.0f), 1.0f, NULL);
    const sl_body_handle b = body_make(&world, SL_BODY_DYNAMIC,
                                       sl_vec2_make(2.0f, 0.0f), 1.0f, NULL);
    const sl_body_handle c = body_make(&world, SL_BODY_DYNAMIC,
                                       sl_vec2_make(3.0f, 0.0f), 1.0f, NULL);
    const sl_joint_handle first = distance_make(&world, fixed, a, 1.0f, true);
    const sl_joint_handle middle = distance_make(&world, a, b, 1.0f, true);
    const sl_joint_handle last = distance_make(&world, b, c, 1.0f, true);

    sl_world_joint_destroy(&world, middle);
    SL_EXPECT(!sl_world_joint_is_valid(&world, middle));
    SL_EXPECT(sl_world_joint_is_valid(&world, first));
    SL_EXPECT(sl_world_joint_is_valid(&world, last));
    SL_EXPECT(sl_world_joint_at(&world, 1u).index == last.index);
    sl_world_joint_destroy(&world, middle);
    sl_world_joint_destroy(&world, sl_joint_handle_null());
    sl_world_joint_destroy(&world,
                           (sl_joint_handle){ world.joint_capacity + 1u, 9u });

    const sl_joint_handle reused = distance_make(&world, a, b, 1.0f, true);
    SL_EXPECT(reused.index == middle.index);
    SL_EXPECT(reused.generation != middle.generation);

    sl_world_body_destroy(&world, b);
    SL_EXPECT(!sl_world_joint_is_valid(&world, last));
    SL_EXPECT(!sl_world_joint_is_valid(&world, reused));
    SL_EXPECT(sl_world_joint_is_valid(&world, first));
    SL_EXPECT_INT_EQ(sl_world_joint_count(&world), 1u);

    sl_world_reset(&world);
    SL_EXPECT(!sl_world_joint_is_valid(&world, first));
    SL_EXPECT_INT_EQ(sl_world_joint_count(&world), 0u);
    SL_EXPECT_INT_EQ(world.joint_free_count, world.joint_capacity);
    sl_world_destroy(&world);
}

static void collision_suppression_requeries_without_motion(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 3u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle fixed = body_make(
        &world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, &circle);
    const sl_body_handle dynamic = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(1.5f, 0.0f), 1.0f, &circle);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);

    const sl_joint_handle first =
        revolute_make(&world, fixed, dynamic, sl_vec2_make(0.0f, 0.0f),
                      sl_vec2_make(-1.5f, 0.0f), false);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    const sl_joint_handle second =
        revolute_make(&world, fixed, dynamic, sl_vec2_make(0.0f, 0.0f),
                      sl_vec2_make(-1.5f, 0.0f), false);
    sl_world_joint_destroy(&world, first);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);

    sl_world_joint_destroy(&world, second);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world), 0u);

    const sl_joint_handle colliding =
        revolute_make(&world, fixed, dynamic, sl_vec2_make(0.0f, 0.0f),
                      sl_vec2_make(-1.5f, 0.0f), true);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    const sl_joint_handle suppressing =
        revolute_make(&world, fixed, dynamic, sl_vec2_make(0.0f, 0.0f),
                      sl_vec2_make(-1.5f, 0.0f), false);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    sl_world_joint_destroy(&world, suppressing);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    sl_world_joint_destroy(&world, colliding);
    sl_world_destroy(&world);
}

static void revolute_keeps_anchor_and_leaves_rotation_free(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape box = box_make(0.25f, 0.5f);
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle dynamic = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(1.0f, 0.0f), 1.0f, &box);
    SL_EXPECT(sl_world_body_set_angular_velocity(&world, dynamic, 2.0f));
    (void)revolute_make(&world, fixed, dynamic, sl_vec2_make(0.0f, 0.0f),
                        sl_vec2_make(-1.0f, 0.0f), true);
    for (uint32_t i = 0u; i < 60u; ++i) {
        sl_world_step(&world, k_dt);
    }
    const sl_transform transform = sl_world_body_get_transform(&world, dynamic);
    const sl_vec2 anchor =
        sl_transform_apply(transform, sl_vec2_make(-1.0f, 0.0f));
    SL_EXPECT(sl_vec2_length(anchor) < 2.0f * SL_LINEAR_SLOP);
    SL_EXPECT(
        sl_is_finite(sl_world_body_get_angular_velocity(&world, dynamic)));
    SL_EXPECT(sl_abs(sl_world_body_get_angle(&world, dynamic)) > 0.1f);
    sl_world_destroy(&world);
}

static void coincident_distance_uses_deterministic_axis(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle dynamic = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.0f), 1.0f, NULL);
    (void)distance_make(&world, fixed, dynamic, 1.0f, true);
    for (uint32_t i = 0u; i < 120u; ++i) {
        sl_world_step(&world, k_dt);
    }
    const sl_vec2 position = sl_world_body_get_position(&world, dynamic);
    SL_EXPECT_NEAR(position.x, 1.0f, 0.02f);
    SL_EXPECT_NEAR(position.y, 0.0f, 1e-6f);
    sl_world_destroy(&world);
}

static void distance_supports_load_and_reports_full_step_impulse(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle bob = body_make(&world, SL_BODY_DYNAMIC,
                                         sl_vec2_make(0.0f, -1.0f), 1.0f, NULL);
    const sl_joint_handle joint = distance_make(&world, fixed, bob, 1.0f, true);
    for (uint32_t i = 0u; i < 600u; ++i) {
        sl_world_step(&world, k_dt);
    }
    const sl_vec2 position = sl_world_body_get_position(&world, bob);
    const sl_vec2 velocity = sl_world_body_get_velocity(&world, bob);
    const sl_vec2 impulse = sl_world_joint_get_linear_impulse(&world, joint);
    SL_EXPECT(sl_vec2_is_finite(position));
    SL_EXPECT(sl_vec2_is_finite(velocity));
    SL_EXPECT(sl_abs(sl_vec2_length(position) - 1.0f) < 0.02f);
    SL_EXPECT(sl_abs(velocity.y) < 2e-3f);
    SL_EXPECT_NEAR(impulse.y, 10.0f * k_dt, 0.02f);
    sl_world_destroy(&world);
}

static void pendulum_period_matches_small_angle_model(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const float length = 1.0f;
    const float theta = 0.08f;
    const sl_body_handle fixed =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle bob = body_make(
        &world, SL_BODY_DYNAMIC,
        sl_vec2_make(length * sinf(theta), -length * cosf(theta)), 1.0f, NULL);
    (void)distance_make(&world, fixed, bob, length, true);

    float previous_x = sl_world_body_get_position(&world, bob).x;
    float first_crossing = -1.0f;
    float second_crossing = -1.0f;
    for (uint32_t step = 1u; step <= 360u; ++step) {
        sl_world_step(&world, k_dt);
        const float x = sl_world_body_get_position(&world, bob).x;
        if (previous_x > 0.0f && x <= 0.0f) {
            const float crossing = (float)step * k_dt;
            if (first_crossing < 0.0f) {
                first_crossing = crossing;
            } else {
                second_crossing = crossing;
                break;
            }
        }
        previous_x = x;
    }
    const float expected = 2.0f * SL_PI * sqrtf(length / 10.0f);
    SL_EXPECT(first_crossing > 0.0f && second_crossing > first_crossing);
    SL_EXPECT_NEAR(second_crossing - first_crossing, expected,
                   0.05f * expected);
    sl_world_destroy(&world);
}

static void eight_link_chain_reports_total_top_load(void)
{
    enum { LINK_COUNT = 8 };
    const sl_world_config config = {
        .body_capacity = LINK_COUNT + 1u,
        .joint_capacity = LINK_COUNT,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_body_handle previous =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    sl_joint_handle top = sl_joint_handle_null();
    for (uint32_t i = 0u; i < LINK_COUNT; ++i) {
        const sl_body_handle body =
            body_make(&world, SL_BODY_DYNAMIC,
                      sl_vec2_make(0.0f, -(float)(i + 1u)), 1.0f, NULL);
        const sl_joint_handle joint =
            distance_make(&world, previous, body, 1.0f, true);
        if (i == 0u) {
            top = joint;
        }
        previous = body;
    }
    for (uint32_t i = 0u; i < 600u; ++i) {
        sl_world_step(&world, k_dt);
    }
    float impulse_sum = 0.0f;
    const uint32_t sample_count = 240u;
    for (uint32_t i = 0u; i < sample_count; ++i) {
        sl_world_step(&world, k_dt);
        impulse_sum += sl_world_joint_get_linear_impulse(&world, top).y;
    }
    const float average = impulse_sum / (float)sample_count;
    const float expected = (float)LINK_COUNT * 10.0f * k_dt;
    SL_EXPECT_NEAR(average, expected, 0.15f * expected);
    sl_world_destroy(&world);
}

static void identical_joint_worlds_are_bitwise_deterministic(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world a = { 0 };
    sl_world b = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    SL_EXPECT(sl_world_init(&b, &config));
    const sl_body_handle fixed_a =
        body_make(&a, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle bob_a =
        body_make(&a, SL_BODY_DYNAMIC, sl_vec2_make(0.25f, -1.0f), 1.0f, NULL);
    const sl_body_handle fixed_b =
        body_make(&b, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), 0.0f, NULL);
    const sl_body_handle bob_b =
        body_make(&b, SL_BODY_DYNAMIC, sl_vec2_make(0.25f, -1.0f), 1.0f, NULL);
    const float length = sqrtf(1.0625f);
    const sl_joint_handle joint_a =
        distance_make(&a, fixed_a, bob_a, length, true);
    const sl_joint_handle joint_b =
        distance_make(&b, fixed_b, bob_b, length, true);
    for (uint32_t i = 0u; i < 240u; ++i) {
        sl_world_step(&a, k_dt);
        sl_world_step(&b, k_dt);
    }
    const sl_vec2 position_a = sl_world_body_get_position(&a, bob_a);
    const sl_vec2 position_b = sl_world_body_get_position(&b, bob_b);
    const sl_vec2 velocity_a = sl_world_body_get_velocity(&a, bob_a);
    const sl_vec2 velocity_b = sl_world_body_get_velocity(&b, bob_b);
    const sl_vec2 impulse_a = sl_world_joint_get_linear_impulse(&a, joint_a);
    const sl_vec2 impulse_b = sl_world_joint_get_linear_impulse(&b, joint_b);
    SL_EXPECT(position_a.x == position_b.x && position_a.y == position_b.y);
    SL_EXPECT(velocity_a.x == velocity_b.x && velocity_a.y == velocity_b.y);
    SL_EXPECT(impulse_a.x == impulse_b.x && impulse_a.y == impulse_b.y);
    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static void extreme_valid_mass_ratio_stays_finite(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .joint_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_desc heavy_desc = {
        .position = { 0.0f, 0.0f },
        .mass = 1e12f,
    };
    const sl_body_desc light_desc = {
        .position = { 1.0f, 0.0f },
        .velocity = { 0.0f, 5.0f },
        .mass = 1e-12f,
    };
    const sl_body_handle heavy = sl_world_body_create(&world, &heavy_desc);
    const sl_body_handle light = sl_world_body_create(&world, &light_desc);
    SL_EXPECT(!sl_body_handle_is_null(heavy));
    SL_EXPECT(!sl_body_handle_is_null(light));
    const sl_joint_handle joint =
        distance_make(&world, heavy, light, 1.0f, true);
    for (uint32_t i = 0u; i < 240u; ++i) {
        sl_world_step(&world, k_dt);
    }
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_position(&world, heavy)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_position(&world, light)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, heavy)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, light)));
    SL_EXPECT(
        sl_vec2_is_finite(sl_world_joint_get_linear_impulse(&world, joint)));
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "config and descriptor atomicity",
      config_and_descriptor_validation_are_atomic },
    { "handles, reset, body destroy",
      handles_swap_remove_reset_and_body_destroy },
    { "collision suppression requery",
      collision_suppression_requeries_without_motion },
    { "revolute free rotation",
      revolute_keeps_anchor_and_leaves_rotation_free },
    { "coincident distance axis", coincident_distance_uses_deterministic_axis },
    { "distance load and step impulse",
      distance_supports_load_and_reports_full_step_impulse },
    { "pendulum period", pendulum_period_matches_small_angle_model },
    { "eight-link top load", eight_link_chain_reports_total_top_load },
    { "extreme mass ratio", extreme_valid_mass_ratio_stays_finite },
    { "joint determinism", identical_joint_worlds_are_bitwise_deterministic },
};

int sl_joint_suite(void)
{
    return sl_run_suite("joint", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
