#include "silk_test.h"
#include "suites.h"

#include <float.h>
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
                                sl_vec2 position, float angle, sl_vec2 velocity,
                                float mass, const sl_shape *shape,
                                float friction, float restitution)
{
    const sl_body_desc desc = {
        .position = position,
        .velocity = velocity,
        .mass = mass,
        .type = type,
        .angle = angle,
        .shape = shape,
        .friction = friction,
        .restitution = restitution,
    };
    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(body));
    return body;
}

static void resting_ball_stays_supported(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape ground_shape = box_make(5.0f, 0.5f);
    const sl_shape ball_shape = circle_make(0.5f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, 0.6f, 0.0f);
    const sl_body_handle ball =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.5f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &ball_shape, 0.6f, 0.0f);

    for (uint32_t i = 0u; i < 120u; ++i) {
        sl_world_step(&world, k_dt);
    }
    const sl_vec2 position = sl_world_body_get_position(&world, ball);
    const sl_vec2 velocity = sl_world_body_get_velocity(&world, ball);
    SL_EXPECT_NEAR(position.y, 0.5f, 2.0f * SL_LINEAR_SLOP);
    SL_EXPECT(sl_abs(velocity.y) < 1e-3f);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    sl_world_destroy(&world);
}

static void two_point_normal_block_avoids_contact_order_torque(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .gravity = { 0.0f, -10.0f },
        .substep_count = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape ground_shape = box_make(5.0f, 0.5f);
    const sl_shape box_shape = box_make(0.5f, 0.5f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, 0.0f, 0.0f);
    const sl_body_handle box =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.5f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &box_shape, 0.0f, 0.0f);

    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    SL_EXPECT_INT_EQ(sl_world_contact_at(&world, 0u)->manifold.point_count, 2u);
    SL_EXPECT(sl_abs(sl_world_body_get_velocity(&world, box).y) < 1e-3f);
    SL_EXPECT(sl_abs(sl_world_body_get_angular_velocity(&world, box)) < 2e-3f);
    sl_world_destroy(&world);
}

static void ten_box_stack_stays_finite_and_centered(void)
{
    enum { BOX_COUNT = 10 };
    const sl_world_config config = {
        .body_capacity = BOX_COUNT + 1u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape ground_shape = box_make(8.0f, 0.5f);
    const sl_shape box_shape = box_make(0.5f, 0.5f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, 0.7f, 0.0f);
    sl_body_handle boxes[BOX_COUNT];
    for (uint32_t i = 0u; i < BOX_COUNT; ++i) {
        boxes[i] = body_make(
            &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.5f + (float)i), 0.0f,
            sl_vec2_make(0.0f, 0.0f), 1.0f, &box_shape, 0.7f, 0.0f);
    }

    for (uint32_t i = 0u; i < 600u; ++i) {
        sl_world_step(&world, k_dt);
    }
    float drift_max = 0.0f;
    for (uint32_t i = 0u; i < BOX_COUNT; ++i) {
        const sl_vec2 position = sl_world_body_get_position(&world, boxes[i]);
        const sl_vec2 velocity = sl_world_body_get_velocity(&world, boxes[i]);
        drift_max = sl_max(drift_max, sl_abs(position.x));
        SL_EXPECT(sl_vec2_is_finite(position));
        SL_EXPECT(sl_vec2_is_finite(velocity));
    }
    SL_EXPECT(drift_max < 0.01f);
    sl_world_destroy(&world);
}

static float bounce_apex(float restitution)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape ground_shape = box_make(5.0f, 0.5f);
    const sl_shape ball_shape = circle_make(0.5f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, 0.0f,
                    restitution);
    const sl_body_handle ball = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 5.0f), 0.0f,
        sl_vec2_make(0.0f, 0.0f), 1.0f, &ball_shape, 0.0f, restitution);

    bool rising = false;
    float apex = -FLT_MAX;
    for (uint32_t i = 0u; i < 360u; ++i) {
        sl_world_step(&world, k_dt);
        const sl_vec2 velocity = sl_world_body_get_velocity(&world, ball);
        const sl_vec2 position = sl_world_body_get_position(&world, ball);
        if (velocity.y > 0.0f) {
            rising = true;
        }
        if (rising) {
            apex = sl_max(apex, position.y);
            if (velocity.y <= 0.0f) {
                break;
            }
        }
    }
    sl_world_destroy(&world);
    return rising ? apex : 0.5f;
}

static void restitution_zero_stops_and_one_returns_height(void)
{
    const float inelastic_apex = bounce_apex(0.0f);
    const float elastic_apex = bounce_apex(1.0f);
    SL_EXPECT(inelastic_apex <= 0.52f);
    SL_EXPECT_NEAR(elastic_apex, 5.0f, 0.25f);
}

static float incline_travel(float friction)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const float angle = SL_PI / 6.0f;
    const sl_vec2 tangent = sl_vec2_make(cosf(angle), sinf(angle));
    const sl_vec2 normal = sl_vec2_make(-sinf(angle), cosf(angle));
    const sl_shape ground_shape = box_make(6.0f, 0.25f);
    const sl_shape box_shape = box_make(0.25f, 0.25f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), angle,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, friction,
                    0.0f);
    const sl_vec2 start = sl_vec2_scale(normal, 0.5f);
    const sl_body_handle box =
        body_make(&world, SL_BODY_DYNAMIC, start, angle,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &box_shape, friction, 0.0f);
    for (uint32_t i = 0u; i < 240u; ++i) {
        sl_world_step(&world, k_dt);
    }
    const sl_vec2 delta =
        sl_vec2_sub(sl_world_body_get_position(&world, box), start);
    const float travel = sl_vec2_dot(delta, tangent);
    sl_world_destroy(&world);
    return travel;
}

static void friction_holds_or_slides_on_incline(void)
{
    const float holding = incline_travel(0.7f);
    const float sliding = incline_travel(0.3f);
    SL_EXPECT(sl_abs(holding) < 0.05f);
    SL_EXPECT(sliding < -0.2f);
}

static void kinematic_paddle_pushes_without_reaction(void)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape box_shape = box_make(0.5f, 0.5f);
    const sl_body_handle paddle =
        body_make(&world, SL_BODY_KINEMATIC, sl_vec2_make(0.0f, 0.0f), 0.0f,
                  sl_vec2_make(1.0f, 0.0f), 0.0f, &box_shape, 0.0f, 0.0f);
    const sl_body_handle box =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(1.0f, 0.0f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &box_shape, 0.0f, 0.0f);
    for (uint32_t i = 0u; i < 60u; ++i) {
        sl_world_step(&world, k_dt);
    }
    SL_EXPECT(sl_world_body_get_velocity(&world, paddle).x == 1.0f);
    SL_EXPECT(sl_world_body_get_velocity(&world, box).x > 0.5f);
    SL_EXPECT(sl_world_body_get_position(&world, box).x > 1.5f);
    sl_world_destroy(&world);
}

static void existing_contact_observes_material_changes(void)
{
    const sl_world_config config = {
        .body_capacity = 2u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape ground_shape = box_make(4.0f, 0.5f);
    const sl_shape box_shape = box_make(0.5f, 0.5f);
    const sl_body_handle ground =
        body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 0.0f, &ground_shape, 0.0f, 0.0f);
    const sl_body_handle box =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.5f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &box_shape, 0.0f, 0.0f);
    sl_world_step(&world, k_dt);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    SL_EXPECT(sl_world_contact_at(&world, 0u)->friction == 0.0f);

    SL_EXPECT(sl_world_body_set_friction(&world, ground, 1.0f));
    SL_EXPECT(sl_world_body_set_friction(&world, box, 1.0f));
    SL_EXPECT(
        sl_world_body_set_velocity(&world, box, sl_vec2_make(2.0f, 0.0f)));
    for (uint32_t i = 0u; i < 10u; ++i) {
        sl_world_step(&world, k_dt);
    }
    SL_EXPECT(sl_world_contact_at(&world, 0u)->friction == 1.0f);
    SL_EXPECT(sl_abs(sl_world_body_get_velocity(&world, box).x) < 2.0f);
    sl_world_destroy(&world);
}

static float penetration_push_distance(sl_body_type partner_type)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    (void)body_make(&world, partner_type, sl_vec2_make(0.0f, 0.0f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &circle, 0.0f, 0.0f);
    const sl_body_handle body =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(1.99f, 0.0f), 0.0f,
                  sl_vec2_make(0.0f, 0.0f), 1.0f, &circle, 0.0f, 0.0f);
    sl_world_step(&world, k_dt);
    const float distance = sl_world_body_get_position(&world, body).x - 1.99f;
    sl_world_destroy(&world);
    return distance;
}

static void static_partner_uses_stiffer_softness(void)
{
    const float static_distance = penetration_push_distance(SL_BODY_STATIC);
    const float kinematic_distance =
        penetration_push_distance(SL_BODY_KINEMATIC);
    SL_EXPECT(static_distance > kinematic_distance);
}

static void tuning_validation_and_extreme_mass_stay_finite(void)
{
    const sl_world_config defaults = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &defaults));
    SL_EXPECT(world.contact_hertz == SL_CONTACT_HERTZ_DEFAULT);
    SL_EXPECT(world.contact_damping_ratio == SL_CONTACT_DAMPING_RATIO_DEFAULT);
    SL_EXPECT(world.contact_push_velocity_max ==
              SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT);
    SL_EXPECT(world.restitution_threshold == SL_RESTITUTION_THRESHOLD_DEFAULT);
    sl_world_destroy(&world);

    sl_world_config invalid = defaults;
    invalid.contact_hertz = -1.0f;
    SL_EXPECT(!sl_world_init(&world, &invalid));
    invalid = defaults;
    invalid.contact_damping_ratio = NAN;
    SL_EXPECT(!sl_world_init(&world, &invalid));
    invalid = defaults;
    invalid.contact_push_velocity_max = INFINITY;
    SL_EXPECT(!sl_world_init(&world, &invalid));
    invalid = defaults;
    invalid.restitution_threshold = -1.0f;
    SL_EXPECT(!sl_world_init(&world, &invalid));
    invalid = defaults;
    invalid.contact_hertz = FLT_MAX;
    SL_EXPECT(!sl_world_init(&world, &invalid));
    invalid = defaults;
    invalid.contact_damping_ratio = FLT_MAX;
    SL_EXPECT(!sl_world_init(&world, &invalid));

    SL_EXPECT(sl_world_init(&world, &defaults));
    const sl_shape large_circle = circle_make(100.0f);
    const float tiny_mass = 1.5f / FLT_MAX;
    const sl_body_handle a = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(-50.0f, 0.0f), 0.0f,
        sl_vec2_make(1.0f, 0.0f), tiny_mass, &large_circle, 0.0f, 0.0f);
    const sl_body_handle b = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(50.0f, 0.0f), 0.0f,
        sl_vec2_make(-1.0f, 0.0f), tiny_mass, &large_circle, 0.0f, 0.0f);
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, a)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, b)));
    sl_world_destroy(&world);
}

static uint32_t random_next(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static void seeded_pile_and_identical_runs_are_finite(void)
{
    enum { BODY_COUNT = 20 };
    const sl_world_config config = {
        .body_capacity = BODY_COUNT + 1u,
        .gravity = { 0.0f, -10.0f },
    };
    sl_world a = { 0 };
    sl_world b = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    SL_EXPECT(sl_world_init(&b, &config));
    const sl_shape ground = box_make(8.0f, 0.5f);
    const sl_shape box = box_make(0.35f, 0.35f);
    (void)body_make(&a, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground, 0.6f, 0.1f);
    (void)body_make(&b, SL_BODY_STATIC, sl_vec2_make(0.0f, -0.5f), 0.0f,
                    sl_vec2_make(0.0f, 0.0f), 0.0f, &ground, 0.6f, 0.1f);
    sl_body_handle bodies_a[BODY_COUNT];
    sl_body_handle bodies_b[BODY_COUNT];
    uint32_t random = 0x4d595df4u;
    for (uint32_t i = 0u; i < BODY_COUNT; ++i) {
        const float x =
            ((float)(random_next(&random) & 1023u) / 1023.0f - 0.5f) * 4.0f;
        const float angle =
            ((float)(random_next(&random) & 255u) / 255.0f - 0.5f) * 0.2f;
        const sl_vec2 position = sl_vec2_make(x, 0.5f + 0.8f * (float)i);
        bodies_a[i] =
            body_make(&a, SL_BODY_DYNAMIC, position, angle,
                      sl_vec2_make(0.0f, 0.0f), 1.0f, &box, 0.6f, 0.1f);
        bodies_b[i] =
            body_make(&b, SL_BODY_DYNAMIC, position, angle,
                      sl_vec2_make(0.0f, 0.0f), 1.0f, &box, 0.6f, 0.1f);
    }
    for (uint32_t step = 0u; step < 2000u; ++step) {
        sl_world_step(&a, k_dt);
        sl_world_step(&b, k_dt);
    }
    for (uint32_t i = 0u; i < BODY_COUNT; ++i) {
        const sl_vec2 position_a = sl_world_body_get_position(&a, bodies_a[i]);
        const sl_vec2 position_b = sl_world_body_get_position(&b, bodies_b[i]);
        const sl_vec2 velocity_a = sl_world_body_get_velocity(&a, bodies_a[i]);
        const sl_vec2 velocity_b = sl_world_body_get_velocity(&b, bodies_b[i]);
        SL_EXPECT(sl_vec2_is_finite(position_a));
        SL_EXPECT(sl_vec2_is_finite(velocity_a));
        SL_EXPECT(position_a.x == position_b.x && position_a.y == position_b.y);
        SL_EXPECT(velocity_a.x == velocity_b.x && velocity_a.y == velocity_b.y);
        SL_EXPECT(sl_world_body_get_angle(&a, bodies_a[i]) ==
                  sl_world_body_get_angle(&b, bodies_b[i]));
    }
    SL_EXPECT_INT_EQ(sl_world_contact_count(&a), sl_world_contact_count(&b));
    sl_world_destroy(&b);
    sl_world_destroy(&a);
}

static const sl_test_case k_cases[] = {
    { "resting ball", resting_ball_stays_supported },
    { "two point normal block",
      two_point_normal_block_avoids_contact_order_torque },
    { "ten box stack", ten_box_stack_stays_finite_and_centered },
    { "restitution", restitution_zero_stops_and_one_returns_height },
    { "friction incline", friction_holds_or_slides_on_incline },
    { "kinematic paddle", kinematic_paddle_pushes_without_reaction },
    { "material refresh", existing_contact_observes_material_changes },
    { "static softness", static_partner_uses_stiffer_softness },
    { "tuning and extreme mass",
      tuning_validation_and_extreme_mass_stay_finite },
    { "seeded pile determinism", seeded_pile_and_identical_runs_are_finite },
};

int sl_solver_suite(void)
{
    return sl_run_suite("solver", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
