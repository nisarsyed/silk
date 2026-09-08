#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

#include <math.h>
#include <silk/query.h>
#include <silk/silk.h>
#include <silk/step.h>
#include <string.h>

static const float k_dt = 1.0f / 60.0f;

static sl_body_handle body_add(sl_world *world, sl_body_type type, float x,
                               float y, const sl_shape *shape)
{
    const sl_body_desc desc = { .type = type,
                                .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                                .position = { x, y },
                                .shape = shape,
                                .friction = 0.6f };
    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(body));
    return body;
}

static void steps_run(sl_world *world, uint32_t count)
{
    for (uint32_t i = 0u; i < count; ++i) {
        sl_world_step(world, k_dt);
    }
}

static sl_joint_handle join(sl_world *world, sl_body_handle a, sl_body_handle b)
{
    const sl_joint_desc desc = { .kind = SL_JOINT_DISTANCE,
                                 .body_a = a,
                                 .body_b = b,
                                 .distance = { .length = 1.0f } };
    const sl_joint_handle joint = sl_world_joint_create(world, &desc);
    SL_EXPECT(!sl_joint_handle_is_null(joint));
    return joint;
}

static void fixture_make(sl_world *world, bool chains)
{
    const sl_world_config config = { .body_capacity = 24u,
                                     .joint_capacity = 24u,
                                     .gravity = { 0.0f, -10.0f },
                                     .sleep_enabled = true };
    SL_EXPECT(sl_world_init(world, &config));
    sl_shape shape, ground;
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &shape));
    SL_EXPECT(sl_shape_make_box(12.0f, 0.5f, &ground));
    const sl_body_handle support =
        body_add(world, SL_BODY_STATIC, 0.0f, chains ? 5.0f : -0.5f,
                 chains ? NULL : &ground);
    for (uint32_t pile = 0u; pile < 3u; ++pile) {
        sl_body_handle previous = support;
        for (uint32_t i = 0u; i < 3u; ++i) {
            const float x = chains ? 0.0f : (float)pile * 3.0f;
            const float y = chains ? 4.0f - (float)i : 0.5f + (float)i;
            const sl_body_handle body =
                body_add(world, SL_BODY_DYNAMIC, x, y, chains ? NULL : &shape);
            if (chains) {
                (void)join(world, previous, body);
            }
            previous = body;
        }
        if (chains) {
            break;
        }
    }
}

static void settling_replay_and_snapshots(void)
{
    for (uint32_t mode = 0u; mode < 2u; ++mode) {
        sl_world a = { 0 }, b = { 0 };
        fixture_make(&a, mode != 0u);
        fixture_make(&b, mode != 0u);
        for (uint32_t step = 0u; step < 1200u; ++step) {
            sl_world_step(&a, k_dt);
            sl_world_step(&b, k_dt);
            sl_query_result result;
            const sl_aabb bounds = { .lower = { -20.0f, -20.0f },
                                     .upper = { 20.0f, 20.0f } };
            SL_EXPECT(sl_world_query_aabb(&a, bounds, 0u, NULL, 0u, &result));
            SL_EXPECT(
                sl_replay_check(&a, &b, "sleep/query", 0x6400u + mode, step));
        }
        sl_world_stats stats = sl_world_get_stats(&a);
        SL_EXPECT_INT_EQ(stats.awake_dynamic_count, 0u);
        SL_EXPECT(stats.sleeping_dynamic_count > 0u);
        SL_EXPECT_INT_EQ(stats.step.dynamic_body_count, 0u);
        SL_EXPECT_INT_EQ(stats.step.contact_constraint_count, 0u);
        SL_EXPECT_INT_EQ(stats.step.joint_constraint_count, 0u);
        sl_transform transforms[24];
        for (uint32_t i = 0u; i < sl_world_body_count(&a); ++i) {
            transforms[i] =
                sl_world_body_get_transform(&a, sl_world_body_at(&a, i));
        }
        const float joint_cache =
            mode != 0u ? a.state->joint_distance_impulses[0] : 0.0f;
        steps_run(&a, 100u);
        for (uint32_t i = 0u; i < sl_world_body_count(&a); ++i) {
            const sl_transform transform =
                sl_world_body_get_transform(&a, sl_world_body_at(&a, i));
            SL_EXPECT(memcmp(&transforms[i], &transform, sizeof(transform)) ==
                      0);
        }
        if (mode != 0u) {
            SL_EXPECT(a.state->joint_distance_impulses[0] == joint_cache);
            SL_EXPECT(a.state->joint_linear_impulses[0].x == 0.0f &&
                      a.state->joint_linear_impulses[0].y == 0.0f);
        }
        const sl_body_handle body = sl_world_body_at(&a, 1u);
        a.state->work_total.body_wakes = UINT64_MAX;
        SL_EXPECT(sl_world_body_wake(&a, body));
        SL_EXPECT(sl_world_body_is_awake(&a, body));
        SL_EXPECT(a.state->work_total.body_wakes == UINT64_MAX);
        SL_EXPECT_INT_EQ(sl_world_get_stats(&a).awake_dynamic_count, 3u);
        sl_world_destroy(&a);
        sl_world_destroy(&b);
    }
}

static void mutation_wakes(void)
{
    for (uint32_t trigger = 0u; trigger < 14u; ++trigger) {
        sl_world world = { 0 };
        fixture_make(&world, true);
        steps_run(&world, 300u);
        SL_EXPECT_INT_EQ(sl_world_get_stats(&world).awake_dynamic_count, 0u);
        const sl_body_handle support = sl_world_body_at(&world, 0u);
        const sl_body_handle body = sl_world_body_at(&world, 1u);
        const sl_body_handle last = sl_world_body_at(&world, 3u);
        sl_shape circle;
        SL_EXPECT(sl_shape_make_circle(0.1f, &circle));
        switch (trigger) {
        case 0u:
            SL_EXPECT(sl_world_body_wake(&world, body));
            break;
        case 1u:
            SL_EXPECT(sl_world_body_apply_force(&world, body,
                                                (sl_vec2){ 1.0f, 0.0f }));
            break;
        case 2u:
            SL_EXPECT(sl_world_body_apply_torque(&world, body, 1.0f));
            break;
        case 3u:
            SL_EXPECT(sl_world_body_set_velocity(&world, body,
                                                 (sl_vec2){ 0.1f, 0.0f }));
            break;
        case 4u:
            SL_EXPECT(sl_world_body_set_angular_velocity(&world, body, 0.1f));
            break;
        case 5u:
            SL_EXPECT(sl_world_body_set_angle(&world, body, 0.1f));
            break;
        case 6u:
            SL_EXPECT(sl_world_body_set_shape(&world, body, &circle));
            break;
        case 7u:
            SL_EXPECT(sl_world_body_set_mass(&world, body, 2.0f));
            break;
        case 8u:
            SL_EXPECT(sl_world_body_set_friction(&world, body, 0.7f));
            break;
        case 9u:
            SL_EXPECT(sl_world_body_set_restitution(&world, body, 0.2f));
            break;
        case 10u:
            SL_EXPECT(sl_world_body_set_position(&world, support,
                                                 (sl_vec2){ 0.001f, 5.0f }));
            break;
        case 11u:
            sl_world_joint_destroy(&world, sl_world_joint_at(&world, 0u));
            break;
        case 12u:
            sl_world_body_destroy(&world, support);
            break;
        case 13u:
            SL_EXPECT(sl_world_body_apply_force_at_point(
                &world, body, (sl_vec2){ 1.0f, 0.0f },
                (sl_vec2){ 0.0f, 4.0f }));
            break;
        default:
            SL_EXPECT(false);
            break;
        }
        SL_EXPECT(sl_world_body_is_awake(&world, last));
        SL_EXPECT_INT_EQ(sl_world_get_stats(&world).awake_dynamic_count, 3u);
        sl_world_destroy(&world);
    }
}

static void rejection_noops_and_dt(void)
{
    sl_world world = { 0 };
    fixture_make(&world, true);
    steps_run(&world, 300u);
    const sl_body_handle body = sl_world_body_at(&world, 1u);
    const uint32_t row = world.state->slots[body.index].dense;
    const uint32_t quiet = world.state->quiet_steps[row];
    SL_EXPECT(sl_world_body_set_position(
        &world, body, sl_world_body_get_position(&world, body)));
    SL_EXPECT(
        sl_world_body_set_velocity(&world, body, (sl_vec2){ 0.0f, 0.0f }));
    SL_EXPECT(sl_world_body_set_angle(&world, body,
                                      sl_world_body_get_angle(&world, body)));
    SL_EXPECT(sl_world_body_set_mass(&world, body, 1.0f));
    SL_EXPECT(sl_world_body_set_shape(&world, body, NULL));
    SL_EXPECT(sl_world_body_apply_force(&world, body, (sl_vec2){ 0.0f, 0.0f }));
    SL_EXPECT(sl_world_body_apply_torque(&world, body, 0.0f));
    SL_EXPECT(!sl_world_body_set_mass(&world, body, NAN));
    SL_EXPECT(
        !sl_world_body_apply_force(&world, body, (sl_vec2){ INFINITY, 0.0f }));
    SL_EXPECT(!sl_world_body_wake(&world, sl_body_handle_null()));
    SL_EXPECT(!sl_world_body_wake(&world, sl_world_body_at(&world, 0u)));
    SL_EXPECT_INT_EQ(world.state->quiet_steps[row], quiet);
    SL_EXPECT(!sl_world_body_is_awake(&world, body));
#ifdef NDEBUG
    sl_world_step(&world, NAN);
    sl_world_step(&world, 0.0f);
    SL_EXPECT(!sl_world_body_is_awake(&world, body));
#endif
    sl_world_step(&world, k_dt * 0.5f);
    SL_EXPECT(sl_world_body_is_awake(&world, body));
    sl_world_reset(&world);
    SL_EXPECT(!sl_world_body_is_awake(&world, body));
    SL_EXPECT_INT_EQ(sl_world_get_stats(&world).sleeping_dynamic_count, 0u);
    sl_world_destroy(&world);
    const float bad[] = { -1.0f, NAN, INFINITY };
    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_world_config config = { .body_capacity = 1u,
                                   .sleep_speed_max = bad[i] };
        SL_EXPECT(!sl_world_init(&world, &config));
        config.sleep_speed_max = 0.0f;
        config.sleep_angular_speed_max = bad[i];
        SL_EXPECT(!sl_world_init(&world, &config));
        config.sleep_angular_speed_max = 0.0f;
        config.sleep_time_min = bad[i];
        SL_EXPECT(!sl_world_init(&world, &config));
    }
}

static void contact_support_and_kinematics(void)
{
    for (uint32_t mode = 0u; mode < 4u; ++mode) {
        sl_world world = { 0 };
        const sl_world_config config = { .body_capacity = 8u,
                                         .sleep_enabled = true,
                                         .gravity = { 0.0f, -10.0f } };
        SL_EXPECT(sl_world_init(&world, &config));
        sl_shape ground, box;
        SL_EXPECT(sl_shape_make_box(5.0f, 0.5f, &ground));
        SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box));
        const sl_body_handle support =
            body_add(&world, mode == 2u ? SL_BODY_KINEMATIC : SL_BODY_STATIC,
                     0.0f, -0.5f, &ground);
        const sl_body_handle body =
            body_add(&world, SL_BODY_DYNAMIC, 0.0f, 0.5f, &box);
        steps_run(&world, 180u);
        SL_EXPECT(!sl_world_body_is_awake(&world, body));
        const sl_contact cached = world.state->contacts[0];
        sl_body_handle found[8];
        sl_query_result matches;
        const sl_world_stats before = sl_world_get_stats(&world);
        SL_EXPECT(sl_world_query_point(&world,
                                       sl_world_body_get_position(&world, body),
                                       0u, found, 8u, &matches));
        const sl_world_stats after = sl_world_get_stats(&world);
        SL_EXPECT(before.cumulative.wake_visits ==
                  after.cumulative.wake_visits);
        SL_EXPECT(memcmp(&cached, &world.state->contacts[0], sizeof(cached)) ==
                  0);
        if (mode == 0u) {
            const uint64_t moves = before.cumulative.proxy_moves;
            SL_EXPECT(sl_world_body_set_position(&world, support,
                                                 (sl_vec2){ 0.001f, -0.5f }));
            SL_EXPECT(sl_world_get_stats(&world).cumulative.proxy_moves ==
                      moves);
        } else if (mode == 1u) {
            sl_world_body_destroy(&world, support);
        } else if (mode == 2u) {
            SL_EXPECT(sl_world_body_set_velocity(&world, support,
                                                 (sl_vec2){ 0.1f, 0.0f }));
        } else {
            (void)body_add(&world, SL_BODY_DYNAMIC, 0.0f, 1.49f, &box);
            sl_world_step(&world, k_dt);
        }
        SL_EXPECT(sl_world_body_is_awake(&world, body));
        steps_run(&world, 30u);
        if (mode == 1u) {
            SL_EXPECT(sl_world_body_get_position(&world, body).y < 0.0f);
        }
        if (mode == 2u) {
            SL_EXPECT(sl_world_body_is_awake(&world, body));
        }
        sl_world_destroy(&world);
    }
}

static void guards_and_new_joint(void)
{
    sl_world world = { 0 };
    sl_world_config config = { .body_capacity = 4u,
                               .joint_capacity = 4u,
                               .sleep_enabled = true,
                               .joint_hertz = 0.0001f };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle a =
        body_add(&world, SL_BODY_DYNAMIC, 0.0f, 0.0f, NULL);
    const sl_body_handle b =
        body_add(&world, SL_BODY_DYNAMIC, 3.0f, 0.0f, NULL);
    steps_run(&world, 40u);
    SL_EXPECT(!sl_world_body_is_awake(&world, a));
    const sl_joint_handle joint = join(&world, a, b);
    SL_EXPECT(sl_world_body_is_awake(&world, a) &&
              sl_world_body_is_awake(&world, b));
    steps_run(&world, 120u);
    SL_EXPECT(sl_world_body_is_awake(&world, a));
    SL_EXPECT(sl_vec2_distance(sl_world_body_get_position(&world, a),
                               sl_world_body_get_position(&world, b)) > 2.0f);
    sl_world_joint_destroy(&world, joint);
    sl_world_body_destroy(&world, b);
    const sl_body_handle replacement =
        body_add(&world, SL_BODY_DYNAMIC, 10.0f, 0.0f, NULL);
    SL_EXPECT_INT_EQ(replacement.index, b.index);
    SL_EXPECT(world.state->body_islands[replacement.index] ==
              SL_BODY_DENSE_NONE);
    SL_EXPECT(!sl_world_body_wake(&world, b));
    sl_world_destroy(&world);

    config = (sl_world_config){ .body_capacity = 2u,
                                .sleep_enabled = true,
                                .contact_push_velocity_max = 0.00001f };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_shape circle;
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    (void)body_add(&world, SL_BODY_STATIC, 0.0f, 0.0f, &circle);
    const sl_body_handle trapped =
        body_add(&world, SL_BODY_DYNAMIC, 0.0f, 0.1f, &circle);
    steps_run(&world, 120u);
    SL_EXPECT(sl_world_body_is_awake(&world, trapped));
    sl_world_destroy(&world);
    config.sleep_time_min = 3.0e38f;
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle forever =
        body_add(&world, SL_BODY_DYNAMIC, 0.0f, 0.0f, NULL);
    steps_run(&world, 40u);
    SL_EXPECT(sl_world_body_is_awake(&world, forever));
    SL_EXPECT_INT_EQ(world.state->sleep_steps_required, 0u);
    sl_world_destroy(&world);
}

static void seeded_activation_replay(void)
{
    sl_world a = { 0 }, b = { 0 };
    fixture_make(&a, false);
    fixture_make(&b, false);
    steps_run(&a, 300u);
    steps_run(&b, 300u);
    uint32_t seed = 0x64abcdefu;
    for (uint32_t operation = 0u; operation < 160u; ++operation) {
        seed = seed * 1664525u + 1013904223u;
        const uint32_t row = 1u + seed % (sl_world_body_count(&a) - 1u);
        const sl_body_handle body_a = sl_world_body_at(&a, row);
        const sl_body_handle body_b = sl_world_body_at(&b, row);
        SL_EXPECT_INT_EQ(body_a.index, body_b.index);
        const float value = (float)(seed % 11u) * 0.1f;
        switch (operation % 5u) {
        case 0u:
            SL_EXPECT(sl_world_body_apply_force(&a, body_a,
                                                (sl_vec2){ value, 0.0f }));
            SL_EXPECT(sl_world_body_apply_force(&b, body_b,
                                                (sl_vec2){ value, 0.0f }));
            break;
        case 1u:
            SL_EXPECT(sl_world_body_set_position(&a, body_a,
                                                 (sl_vec2){ value, 3.0f }));
            SL_EXPECT(sl_world_body_set_position(&b, body_b,
                                                 (sl_vec2){ value, 3.0f }));
            break;
        case 2u:
            sl_world_body_destroy(&a, body_a);
            sl_world_body_destroy(&b, body_b);
            (void)body_add(&a, SL_BODY_DYNAMIC, value, 4.0f, NULL);
            (void)body_add(&b, SL_BODY_DYNAMIC, value, 4.0f, NULL);
            break;
        case 3u: {
            const sl_body_handle other = sl_world_body_at(&a, 1u);
            const sl_joint_desc desc = { .kind = SL_JOINT_REVOLUTE,
                                         .body_a = body_a,
                                         .body_b = other };
            const sl_joint_handle ja = sl_world_joint_create(&a, &desc);
            const sl_joint_handle jb = sl_world_joint_create(&b, &desc);
            SL_EXPECT_INT_EQ(ja.index, jb.index);
            SL_EXPECT_INT_EQ(ja.generation, jb.generation);
            break;
        }
        case 4u:
            SL_EXPECT(sl_world_body_wake(&a, body_a));
            SL_EXPECT(sl_world_body_wake(&b, body_b));
            break;
        default:
            SL_EXPECT(false);
            break;
        }
        SL_EXPECT(sl_replay_check(&a, &b, "activation churn", 0x64abcdefu,
                                  operation));
        sl_world_step(&a, k_dt);
        sl_world_step(&b, k_dt);
        SL_EXPECT(
            sl_replay_check(&a, &b, "activation step", 0x64abcdefu, operation));
    }
    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static const sl_test_case k_cases[] = {
    { "seeded activation mutation replay", seeded_activation_replay },
    { "support changes, contacts and moving kinematics",
      contact_support_and_kinematics },
    { "settling guards and new joint propagation", guards_and_new_joint },
    { "settled piles/chains, replay and exact holds",
      settling_replay_and_snapshots },
    { "mutation wake propagation", mutation_wakes },
    { "rejection, no-ops and timestep changes", rejection_noops_and_dt },
};
int sl_sleep_suite(void)
{
    return sl_run_suite("sleep", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
