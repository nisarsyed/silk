#include "silk_test.h"
#include "suites.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <silk/step.h>
#include <silk/world.h>

/* Trajectory expectations are hand-derived on clean fractions (dt = 0.1,
 * dt * drag = 1) and compare within 1e-6; determinism and packing tests
 * compare bitwise, where exactness is the property under test. */
static const float k_eps = 1e-6f;
static const float k_dt = 0.1f;

static void test_stepper_init_rejects_bad_timestep(void)
{
    sl_stepper stepper;

    const float bad[] = { NAN, INFINITY, -INFINITY, 0.0f, -0.1f };
    for (uint32_t i = 0u; i < 5u; ++i) {
        SL_EXPECT(!sl_stepper_init(&stepper, bad[i]));
        SL_EXPECT(stepper.timestep == 0.0f && stepper.remainder == 0.0f);
    }

    SL_EXPECT(sl_stepper_init(&stepper, 1.0f / 60.0f));
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));
}

static void test_step_moves_body_under_gravity(void)
{
    sl_world_config config = { .body_capacity = 2u,
                               .gravity = sl_vec2_make(0.0f, -10.0f),
                               .linear_drag = 0.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    /* From rest: v(n) = -n, p(n) = -0.1 * n(n+1)/2 at dt = 0.1. */
    const float expected_v[3] = { -1.0f, -2.0f, -3.0f };
    const float expected_p[3] = { -0.1f, -0.3f, -0.6f };
    for (uint32_t n = 0u; n < 3u; ++n) {
        sl_world_step(&world, k_dt);
        sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
        sl_vec2 position = sl_world_body_get_position(&world, h);
        SL_EXPECT_NEAR(velocity.y, expected_v[n], k_eps);
        SL_EXPECT_NEAR(position.y, expected_p[n], k_eps);
        SL_EXPECT(velocity.x == 0.0f && position.x == 0.0f);
    }

    sl_world_destroy(&world);
}

static void test_step_applies_force_via_inv_mass(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* mass 2 kg: accumulated f = (4, 0) must integrate a = f/m = 2. */
    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 2.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(1.5f, 0.0f)));
    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(2.5f, 0.0f)));

    sl_vec2 accumulated = sl_world_body_get_force(&world, h);
    SL_EXPECT(accumulated.x == 4.0f && accumulated.y == 0.0f);

    sl_world_step(&world, k_dt);
    sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT_NEAR(velocity.x, 0.2f, k_eps);
    SL_EXPECT_NEAR(position.x, 0.02f, k_eps);

    sl_world_destroy(&world);
}

static void test_force_clears_after_step(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(10.0f, 0.0f)));
    sl_world_step(&world, k_dt);

    sl_vec2 impulse_v = sl_world_body_get_velocity(&world, h);
    SL_EXPECT_NEAR(impulse_v.x, 1.0f, k_eps);

    /* Force was consumed: a force-free step changes velocity by exactly
     * nothing (drag is zero) and moves position by v * dt. */
    sl_vec2 cleared = sl_world_body_get_force(&world, h);
    SL_EXPECT(cleared.x == 0.0f && cleared.y == 0.0f);

    sl_vec2 coast_p = sl_world_body_get_position(&world, h);
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_world_body_get_velocity(&world, h).x == impulse_v.x);
    sl_vec2 drifted_p = sl_world_body_get_position(&world, h);
    SL_EXPECT_NEAR(drifted_p.x - coast_p.x, impulse_v.x * k_dt, k_eps);

    sl_world_destroy(&world);
}

static void test_apply_force_rejects_non_finite(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(3.0f, 4.0f)));

    SL_EXPECT(!sl_world_body_apply_force(&world, h, sl_vec2_make(NAN, 0.0f)));
    SL_EXPECT(
        !sl_world_body_apply_force(&world, h, sl_vec2_make(0.0f, INFINITY)));
    sl_vec2 unchanged = sl_world_body_get_force(&world, h);
    SL_EXPECT(unchanged.x == 3.0f && unchanged.y == 4.0f);

    sl_world_destroy(&world);
}

static void test_apply_force_rejects_overflowing_sum(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    sl_body_handle c = sl_world_body_create(&world, &desc);

    /* Each application is finite on its own; the accumulation overflows
     * and must be rejected whole — y is untouched when x overflows. */
    SL_EXPECT(
        sl_world_body_apply_force(&world, h, sl_vec2_make(FLT_MAX, 0.0f)));
    SL_EXPECT(
        !sl_world_body_apply_force(&world, h, sl_vec2_make(1.0e38f, 5.0f)));
    sl_vec2 saturated = sl_world_body_get_force(&world, h);
    SL_EXPECT(saturated.x == FLT_MAX && saturated.y == 0.0f);

    /* Exact cancellation stays inside range and must stay legal. */
    SL_EXPECT(
        sl_world_body_apply_force(&world, c, sl_vec2_make(FLT_MAX, 0.0f)));
    SL_EXPECT(
        sl_world_body_apply_force(&world, c, sl_vec2_make(-FLT_MAX, 0.0f)));
    sl_vec2 cancelled = sl_world_body_get_force(&world, c);
    SL_EXPECT(cancelled.x == 0.0f && cancelled.y == 0.0f);

    /* Stepping the saturated accumulator keeps all state finite. */
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, h)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_position(&world, h)));

    sl_world_destroy(&world);
}

static void test_step_holds_last_finite_state_on_overflow(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    /* Each value is legal state; their product with dt is not. The
     * angular side is the sharper one: wrapping an infinite angle
     * yields NaN, which would leave the documented [-pi, pi] range and
     * trip the finite-transform assert in every later shape query. */
    SL_EXPECT(sl_world_body_set_velocity(&world, h, sl_vec2_make(3e38f, 0.0f)));
    SL_EXPECT(sl_world_body_set_angular_velocity(&world, h, 3e38f));
    sl_world_step(&world, 2.0f);

    const sl_vec2 position = sl_world_body_get_position(&world, h);
    const float angle = sl_world_body_get_angle(&world, h);
    SL_EXPECT(sl_vec2_is_finite(position));
    SL_EXPECT(sl_is_finite(angle));
    /* Uncommitted, not clamped: the row keeps the pose it arrived with. */
    SL_EXPECT(position.x == 0.0f && position.y == 0.0f);
    SL_EXPECT(angle == 0.0f);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, h)));
    SL_EXPECT(sl_is_finite(sl_world_body_get_angular_velocity(&world, h)));
    SL_EXPECT_INT_EQ(sl_world_get_step_rejection_count(&world), 1u);

    sl_world_destroy(&world);
}

static void test_step_rejection_count_saturates_and_reset_clears(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = {
        .position = sl_vec2_make(SL_POSITION_ABS_MAX - 0.5f, 0.0f),
        .velocity = sl_vec2_make(1.0f, 0.0f),
        .mass = 1.0f,
    };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_apply_force(&world, h, sl_vec2_make(1.0f, 0.0f)));

    sl_world_step(&world, 1.0f);
    const sl_vec2 position = sl_world_body_get_position(&world, h);
    const sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(position.x == desc.position.x && position.y == desc.position.y);
    SL_EXPECT(velocity.x == desc.velocity.x && velocity.y == desc.velocity.y);
    SL_EXPECT(sl_world_body_get_force(&world, h).x == 0.0f);
    SL_EXPECT_INT_EQ(sl_world_get_step_rejection_count(&world), 1u);

    /* Reach the representational limit without billions of steps, then
     * prove another rejection cannot wrap the diagnostic to zero. */
    world.step_rejection_count = UINT32_MAX - 1u;
    sl_world_step(&world, 1.0f);
    SL_EXPECT_INT_EQ(sl_world_get_step_rejection_count(&world), UINT32_MAX);
    sl_world_step(&world, 1.0f);
    SL_EXPECT_INT_EQ(sl_world_get_step_rejection_count(&world), UINT32_MAX);

    sl_world_reset(&world);
    SL_EXPECT_INT_EQ(sl_world_get_step_rejection_count(&world), 0u);

    sl_world_destroy(&world);
}

static void test_apply_force_rejects_acceleration_overflow(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* Mass 1e-35 keeps a representable inverse, and the force is finite
     * on its own — but f / m overflows. Application must reject instead
     * of banking infinity for the stepper to consume. */
    sl_body_desc tiny = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1e-35f };
    sl_body_handle h = sl_world_body_create(&world, &tiny);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(!sl_world_body_apply_force(&world, h, sl_vec2_make(3e38f, 0.0f)));
    sl_vec2 unchanged = sl_world_body_get_force(&world, h);
    SL_EXPECT(unchanged.x == 0.0f && unchanged.y == 0.0f);

    /* Stepping the untouched body keeps all state finite. */
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_velocity(&world, h)));
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_position(&world, h)));

    sl_world_destroy(&world);
}

static void test_drag_halves_velocity_per_step(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .gravity = sl_vec2_make(0.0f, 0.0f),
                               .linear_drag = 10.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT(sl_world_get_linear_drag(&world) == 10.0f);

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(1.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    /* dt * drag = 1 => scale 1 / (1 + 1) = 0.5 per step; v halves and p
     * integrates the halved series. */
    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_velocity(&world, h).x, 0.5f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_position(&world, h).x, 0.05f, k_eps);

    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_velocity(&world, h).x, 0.25f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_position(&world, h).x, 0.075f, k_eps);

    sl_world_destroy(&world);
}

static void test_drag_extreme_coefficient_stable(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .gravity = sl_vec2_make(0.0f, 0.0f),
                               .linear_drag = 1e30f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(1.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    sl_world_step(&world, k_dt);
    sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(sl_vec2_is_finite(velocity));
    SL_EXPECT(sl_abs(velocity.x) < 1e-20f);
    SL_EXPECT(velocity.x > 0.0f); /* sign preserved, never reversed */

    sl_world_destroy(&world);
}

static void test_advance_runs_steps_and_carries_remainder(void)
{
    sl_world_config config = { .body_capacity = 2u,
                               .gravity = sl_vec2_make(0.0f, -10.0f),
                               .linear_drag = 0.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    sl_stepper stepper;
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));

    /* Half a step: nothing moves, time is banked. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.05f), 0u);
    SL_EXPECT(sl_world_body_get_position(&world, h).y == 0.0f);
    SL_EXPECT(stepper.remainder > 0.0f && stepper.remainder < k_dt);

    /* Doubling a float is exact, so the banked half plus another half
     * completes exactly one step. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.05f), 1u);
    SL_EXPECT_NEAR(sl_world_body_get_velocity(&world, h).y, -1.0f, k_eps);

    /* 0.25 s buys two full steps and banks the leftover fraction. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.25f), 2u);
    SL_EXPECT_NEAR(sl_world_body_get_velocity(&world, h).y, -3.0f, k_eps);
    SL_EXPECT(stepper.remainder > 0.0f && stepper.remainder < k_dt);

    sl_world_destroy(&world);
}

static void test_advance_drops_debt_at_cap(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_stepper stepper;
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));

    /* 10 s demands 100 steps; the cap executes 8 and sheds the debt. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 10.0f),
                     SL_STEP_COUNT_MAX);
    SL_EXPECT(stepper.remainder == 0.0f);

    /* No hidden debt: less than one step of new time runs nothing. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.09f), 0u);

    sl_world_destroy(&world);
}

static void test_advance_ignores_garbage_frame_time(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_stepper stepper;
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));

    const float garbage[] = { NAN, INFINITY, -INFINITY, -1.0f };
    for (uint32_t i = 0u; i < 4u; ++i) {
        SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, garbage[i]), 0u);
        SL_EXPECT(stepper.remainder == 0.0f);
    }

    sl_world_destroy(&world);
}

/* Whole-record bitwise compare. sl_shape carries no padding (pinned by
 * the layout assert in shape.c) and every constructor zeroes the unused
 * arm of the union, so two records for the same shape agree byte for
 * byte -- tail included. A field-wise compare would skip that tail,
 * which is precisely where record drift hides. */
static bool shapes_match(const sl_shape *a, const sl_shape *b)
{
    return memcmp(a, b, sizeof(*a)) == 0;
}

/* Full SoA state of two worlds compared bitwise, row by row. Every
 * packed array belongs here -- a missing array hides swap-remove drift
 * from the churn stress below. */
static bool twin_worlds_in_sync(const sl_world *a, const sl_world *b)
{
    if (a->body_count != b->body_count || a->free_count != b->free_count ||
        a->step_rejection_count != b->step_rejection_count) {
        return false;
    }
    for (uint32_t i = 0u; i < a->body_count; ++i) {
        const bool rows_match =
            a->positions[i].x == b->positions[i].x &&
            a->positions[i].y == b->positions[i].y &&
            a->velocities[i].x == b->velocities[i].x &&
            a->velocities[i].y == b->velocities[i].y &&
            a->forces[i].x == b->forces[i].x &&
            a->forces[i].y == b->forces[i].y && a->masses[i] == b->masses[i] &&
            a->inv_masses[i] == b->inv_masses[i] &&
            a->angles[i] == b->angles[i] &&
            a->angular_velocities[i] == b->angular_velocities[i] &&
            a->torques[i] == b->torques[i] &&
            a->inertias[i] == b->inertias[i] &&
            a->inv_inertias[i] == b->inv_inertias[i] &&
            a->types[i] == b->types[i] &&
            shapes_match(&a->shapes[i], &b->shapes[i]);
        if (!rows_match) {
            return false;
        }
    }
    return true;
}

static void test_determinism_identical_sequences_bitwise(void)
{
    sl_world_config config = { .body_capacity = 8u,
                               .gravity = sl_vec2_make(0.5f, -10.0f),
                               .linear_drag = 2.0f };
    sl_world a = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    sl_world b = { 0 };
    SL_EXPECT(sl_world_init(&b, &config));

    sl_stepper sa;
    sl_stepper sb;
    SL_EXPECT(sl_stepper_init(&sa, k_dt));
    SL_EXPECT(sl_stepper_init(&sb, k_dt));

    sl_body_handle ha[8];
    sl_body_handle hb[8];
    uint32_t count = 0u;

    sl_shape circle_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.5f, &circle_shape));
    sl_shape box_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.5f, &box_shape));

    for (uint32_t round = 0u; round < 24u; ++round) {
        if (count < 8u && round % 3u != 2u) {
            /* Mixed population: every fourth slot is kinematic, slot 6
             * is the one static, and shapes alternate circle/box. The
             * twin worlds see byte-identical descriptors. */
            sl_body_desc desc = { .position =
                                      sl_vec2_make((float)round, -(float)round),
                                  .velocity =
                                      sl_vec2_make(0.1f * (float)round, 0.0f),
                                  .mass = 1.0f + 0.5f * (float)round };
            desc.shape = (count % 2u == 0u) ? &circle_shape : &box_shape;
            if (count % 4u == 3u) {
                desc.type = SL_BODY_KINEMATIC;
                desc.mass = 0.0f;
            } else if (count == 6u) {
                desc.type = SL_BODY_STATIC;
                desc.mass = 0.0f;
                desc.velocity = sl_vec2_make(0.0f, 0.0f);
                desc.angular_velocity = 0.0f;
            }
            const bool dynamic_body = desc.type == SL_BODY_DYNAMIC;

            ha[count] = sl_world_body_create(&a, &desc);
            hb[count] = sl_world_body_create(&b, &desc);
            SL_EXPECT(!sl_body_handle_is_null(ha[count]));
            SL_EXPECT(!sl_body_handle_is_null(hb[count]));
            /* Forces and torques land on dynamics only; rejection is
             * itself part of the deterministic contract under churn. */
            const sl_vec2 force = sl_vec2_make(2.0f, -1.0f);
            SL_EXPECT(sl_world_body_apply_force(&a, ha[count], force) ==
                      dynamic_body);
            SL_EXPECT(sl_world_body_apply_force(&b, hb[count], force) ==
                      dynamic_body);
            SL_EXPECT(sl_world_body_apply_torque(&a, ha[count], 0.75f) ==
                      dynamic_body);
            SL_EXPECT(sl_world_body_apply_torque(&b, hb[count], 0.75f) ==
                      dynamic_body);
            count++;
        } else if (count > 0u) {
            count--;
            sl_world_body_destroy(&a, ha[count]);
            sl_world_body_destroy(&b, hb[count]);
        }

        SL_EXPECT_INT_EQ(sl_world_advance(&a, &sa, 0.25f),
                         sl_world_advance(&b, &sb, 0.25f));
        SL_EXPECT(twin_worlds_in_sync(&a, &b));
    }

    /* Finish with capped advances shedding debt identically. */
    for (uint32_t i = 0u; i < 5u; ++i) {
        const uint32_t steps_a = sl_world_advance(&a, &sa, 10.0f);
        const uint32_t steps_b = sl_world_advance(&b, &sb, 10.0f);
        SL_EXPECT_INT_EQ(steps_a, steps_b);
        SL_EXPECT_INT_EQ(steps_a, SL_STEP_COUNT_MAX);
        SL_EXPECT(sa.remainder == 0.0f && sb.remainder == 0.0f);
        SL_EXPECT(twin_worlds_in_sync(&a, &b));
    }

    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static void test_results_independent_of_packed_layout(void)
{
    sl_shape circle_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle_shape));
    sl_shape box_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box_shape));

    /* Angular state and shapes ride along: packing must not perturb
     * rotation any more than translation. */
    const sl_body_desc descs[6] = {
        { .position = sl_vec2_make(1.0f, 0.0f),
          .velocity = sl_vec2_make(0.5f, 0.0f),
          .mass = 1.0f,
          .angular_velocity = 0.15f,
          .shape = &circle_shape },
        { .position = sl_vec2_make(2.0f, 0.0f),
          .velocity = sl_vec2_make(-0.5f, 0.0f),
          .mass = 2.0f,
          .angular_velocity = -0.4f },
        { .position = sl_vec2_make(3.0f, 0.0f),
          .velocity = sl_vec2_make(0.25f, 0.0f),
          .mass = 3.0f,
          .angular_velocity = 0.8f,
          .shape = &box_shape },
        { .position = sl_vec2_make(4.0f, 0.0f),
          .velocity = sl_vec2_make(-0.25f, 0.0f),
          .mass = 4.0f,
          .angular_velocity = -1.2f },
        { .position = sl_vec2_make(5.0f, 0.0f),
          .velocity = sl_vec2_make(0.75f, 0.0f),
          .mass = 5.0f,
          .angular_velocity = 0.05f,
          .shape = &circle_shape },
        { .position = sl_vec2_make(6.0f, 0.0f),
          .velocity = sl_vec2_make(-0.75f, 0.0f),
          .mass = 6.0f,
          .angular_velocity = 2.0f,
          .shape = &box_shape },
    };

    const sl_world_config config = { .body_capacity = 8u,
                                     .gravity = sl_vec2_make(0.0f, -10.0f),
                                     .linear_drag = 3.0f };

    /* World A: tags 0..5 created in order, then tag 2 destroyed —
     * packed rows become tags 0, 1, 5, 4, 3. */
    sl_world a = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    sl_body_handle ha[6];
    for (uint32_t i = 0u; i < 6u; ++i) {
        ha[i] = sl_world_body_create(&a, &descs[i]);
        SL_EXPECT(!sl_body_handle_is_null(ha[i]));
    }
    sl_world_body_destroy(&a, ha[2]);

    /* World B: same six tags, permuted creation order, tag 2 destroyed —
     * packed rows become tags 5, 3, 1, 4, 0. */
    sl_world b = { 0 };
    SL_EXPECT(sl_world_init(&b, &config));
    const uint32_t order[6] = { 5u, 3u, 1u, 4u, 2u, 0u };
    sl_body_handle hb[6];
    for (uint32_t i = 0u; i < 6u; ++i) {
        hb[order[i]] = sl_world_body_create(&b, &descs[order[i]]);
        SL_EXPECT(!sl_body_handle_is_null(hb[order[i]]));
    }
    sl_world_body_destroy(&b, hb[2]);

    /* 0.1f banked against a 0.1f timestep is exactly one step per call
     * (exact subtraction at the boundary). */
    sl_stepper sa;
    sl_stepper sb;
    SL_EXPECT(sl_stepper_init(&sa, k_dt));
    SL_EXPECT(sl_stepper_init(&sb, k_dt));
    for (uint32_t i = 0u; i < 10u; ++i) {
        SL_EXPECT_INT_EQ(sl_world_advance(&a, &sa, k_dt), 1u);
        SL_EXPECT_INT_EQ(sl_world_advance(&b, &sb, k_dt), 1u);
    }

    /* Each body ran the identical arithmetic regardless of its row, so
     * matching tags agree bitwise. */
    const uint32_t alive_tags[5] = { 0u, 1u, 3u, 4u, 5u };
    for (uint32_t t = 0u; t < 5u; ++t) {
        const uint32_t tag = alive_tags[t];
        SL_EXPECT(sl_world_body_is_valid(&a, ha[tag]));
        SL_EXPECT(sl_world_body_is_valid(&b, hb[tag]));
        sl_vec2 pos_a = sl_world_body_get_position(&a, ha[tag]);
        sl_vec2 pos_b = sl_world_body_get_position(&b, hb[tag]);
        sl_vec2 vel_a = sl_world_body_get_velocity(&a, ha[tag]);
        sl_vec2 vel_b = sl_world_body_get_velocity(&b, hb[tag]);
        SL_EXPECT(pos_a.x == pos_b.x && pos_a.y == pos_b.y);
        SL_EXPECT(vel_a.x == vel_b.x && vel_a.y == vel_b.y);
        SL_EXPECT(sl_world_body_get_angle(&a, ha[tag]) ==
                  sl_world_body_get_angle(&b, hb[tag]));
        SL_EXPECT(sl_world_body_get_angular_velocity(&a, ha[tag]) ==
                  sl_world_body_get_angular_velocity(&b, hb[tag]));
    }

    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static void test_config_rejects_bad_gravity_or_drag(void)
{
    sl_world world = { 0 };

    sl_world_config nan_gravity = { .body_capacity = 4u,
                                    .gravity = sl_vec2_make(NAN, 0.0f),
                                    .linear_drag = 0.0f };
    SL_EXPECT(!sl_world_init(&world, &nan_gravity));
    SL_EXPECT(world.memory == NULL && world.body_capacity == 0u);

    sl_world_config inf_gravity = { .body_capacity = 4u,
                                    .gravity = sl_vec2_make(0.0f, INFINITY),
                                    .linear_drag = 0.0f };
    SL_EXPECT(!sl_world_init(&world, &inf_gravity));
    SL_EXPECT(world.memory == NULL);

    sl_world_config neg_drag = { .body_capacity = 4u };
    neg_drag.linear_drag = -1.0f;
    SL_EXPECT(!sl_world_init(&world, &neg_drag));

    sl_world_config nan_drag = { .body_capacity = 4u };
    nan_drag.linear_drag = NAN;
    SL_EXPECT(!sl_world_init(&world, &nan_drag));

    sl_world_config inf_drag = { .body_capacity = 4u };
    inf_drag.linear_drag = INFINITY;
    SL_EXPECT(!sl_world_init(&world, &inf_drag));

    sl_world_config neg_angular = { .body_capacity = 4u };
    neg_angular.angular_drag = -0.5f;
    SL_EXPECT(!sl_world_init(&world, &neg_angular));

    sl_world_config nan_angular = { .body_capacity = 4u };
    nan_angular.angular_drag = NAN;
    SL_EXPECT(!sl_world_init(&world, &nan_angular));

    sl_world_config inf_angular = { .body_capacity = 4u };
    inf_angular.angular_drag = INFINITY;
    SL_EXPECT(!sl_world_init(&world, &inf_angular));

    /* Zeroed trailing fields are the documented defaults. */
    sl_world_config defaults = { .body_capacity = 2u };
    SL_EXPECT(sl_world_init(&world, &defaults));
    sl_vec2 gravity = sl_world_get_gravity(&world);
    SL_EXPECT(gravity.x == 0.0f && gravity.y == 0.0f);
    SL_EXPECT(sl_world_get_linear_drag(&world) == 0.0f);
    SL_EXPECT(sl_world_get_angular_drag(&world) == 0.0f);

    /* Defaults coast: pure linear motion, exactly one timestep of arc. */
    sl_body_desc desc = { .position = sl_vec2_make(1.0f, 1.0f),
                          .velocity = sl_vec2_make(2.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    sl_world_step(&world, k_dt);
    sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT_NEAR(position.x, 1.2f, k_eps);
    SL_EXPECT(position.y == 1.0f);

    sl_world_destroy(&world);
}

/* Seeded churn stress with integration in the mix: xorshift32 drives
 * create/destroy/apply-force/advance/reset against a reference pool
 * model and a bitwise twin world. Seed committed for reproducibility. */
#define CHURN_SEED 0xB0A57EEDu
#define CHURN_OPS 2048u
#define CHURN_CAPACITY 32u

/* One generator for both churn tests; each owns its state so the two
 * streams stay independent and reproducible from their seed alone. */
static uint32_t xorshift32(uint32_t *state)
{
    *state ^= *state << 13u;
    *state ^= *state >> 17u;
    *state ^= *state << 5u;
    return *state;
}

static uint32_t model_generation_bump(uint32_t generation)
{
    generation++;
    return (generation == 0u) ? 1u : generation;
}

static void test_churn_step_stress_matches_model(void)
{
    sl_world_config config = { .body_capacity = CHURN_CAPACITY,
                               .gravity = sl_vec2_make(0.2f, -9.8f),
                               .linear_drag = 1.5f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_world twin = { 0 };
    SL_EXPECT(sl_world_init(&twin, &config));

    sl_stepper stepper;
    sl_stepper twin_stepper;
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));
    SL_EXPECT(sl_stepper_init(&twin_stepper, k_dt));

    uint32_t rng = CHURN_SEED;
    bool alive[CHURN_CAPACITY] = { false };
    bool moved[CHURN_CAPACITY] = { false };
    uint32_t generation[CHURN_CAPACITY];
    int64_t tag[CHURN_CAPACITY];
    uint32_t live = 0u;
    int64_t next_tag = 0;
    for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
        generation[i] = 1u;
        tag[i] = -1;
    }

    static const float k_frames[4] = { 0.0f, 0.005f, 0.01f, 0.25f };

    bool counts_match = true;
    bool validity_matches = true;
    bool payloads_intact = true;
    bool forces_settled = true;
    bool twins_in_sync = true;

    for (uint32_t op = 0u; op < CHURN_OPS; ++op) {
        const uint32_t roll = xorshift32(&rng) % 40u;
        if (roll == 0u) {
            sl_world_reset(&world);
            sl_world_reset(&twin);
            for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
                generation[i] = model_generation_bump(generation[i]);
                alive[i] = false;
                moved[i] = false;
                tag[i] = -1;
            }
            live = 0u;
        } else if (roll <= 15u) {
            sl_body_desc desc = { .position = sl_vec2_make((float)next_tag,
                                                           (float)(-next_tag)),
                                  .velocity = sl_vec2_make(0.0f, 0.0f),
                                  .mass = 1.0f };
            sl_body_handle h = sl_world_body_create(&world, &desc);
            sl_body_handle ht = sl_world_body_create(&twin, &desc);
            if (live == CHURN_CAPACITY) {
                counts_match = counts_match && sl_body_handle_is_null(h) &&
                               sl_body_handle_is_null(ht);
            } else {
                counts_match = counts_match && !sl_body_handle_is_null(h) &&
                               !sl_body_handle_is_null(ht);
                if (!sl_body_handle_is_null(h)) {
                    validity_matches = validity_matches && !alive[h.index] &&
                                       h.generation == generation[h.index];
                    alive[h.index] = true;
                    moved[h.index] = false;
                    tag[h.index] = next_tag;
                    live++;
                }
            }
            next_tag++;
        } else if (roll <= 27u) {
            uint32_t victim = CHURN_CAPACITY;
            for (uint32_t probe = 0u; probe < 8u && victim == CHURN_CAPACITY;
                 ++probe) {
                const uint32_t candidate = xorshift32(&rng) % CHURN_CAPACITY;
                if (alive[candidate]) {
                    victim = candidate;
                }
            }
            if (victim != CHURN_CAPACITY) {
                sl_body_handle h = { victim, generation[victim] };
                validity_matches =
                    validity_matches && sl_world_body_is_valid(&world, h);
                sl_world_body_destroy(&world, h);
                sl_world_body_destroy(&twin, h);
                alive[victim] = false;
                generation[victim] = model_generation_bump(generation[victim]);
                live--;
            }
        } else if (roll <= 33u) {
            uint32_t target = CHURN_CAPACITY;
            for (uint32_t probe = 0u; probe < 8u && target == CHURN_CAPACITY;
                 ++probe) {
                const uint32_t candidate = xorshift32(&rng) % CHURN_CAPACITY;
                if (alive[candidate]) {
                    target = candidate;
                }
            }
            if (target != CHURN_CAPACITY) {
                sl_body_handle h = { target, generation[target] };
                sl_vec2 force =
                    sl_vec2_make((float)(roll % 7u) - 3.0f, (float)(roll % 5u));
                SL_EXPECT(sl_world_body_apply_force(&world, h, force));
                SL_EXPECT(sl_world_body_apply_force(&twin, h, force));
            }
        } else {
            /* Identical steppers fed identical frames stay identical. */
            const float frame_time = k_frames[xorshift32(&rng) % 4u];
            const uint32_t stepped =
                sl_world_advance(&world, &stepper, frame_time);
            const uint32_t stepped_twin =
                sl_world_advance(&twin, &twin_stepper, frame_time);
            SL_EXPECT_INT_EQ(stepped, stepped_twin);
            if (stepped > 0u) {
                for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
                    moved[i] = moved[i] || alive[i];
                }
                /* Every executed step consumes both accumulators. */
                for (uint32_t row = 0u; row < world.body_count; ++row) {
                    forces_settled = forces_settled &&
                                     world.forces[row].x == 0.0f &&
                                     world.forces[row].y == 0.0f &&
                                     world.torques[row] == 0.0f;
                }
            }
        }

        const uint32_t reported = sl_world_body_count(&world);
        counts_match = counts_match && reported == live &&
                       reported == sl_world_body_count(&twin);
        for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
            sl_body_handle probe = { i, generation[i] };
            const bool engine_alive = sl_world_body_is_valid(&world, probe);
            validity_matches =
                validity_matches && engine_alive == alive[i] &&
                engine_alive == sl_world_body_is_valid(&twin, probe);
            if (engine_alive && alive[i]) {
                sl_vec2 position = sl_world_body_get_position(&world, probe);
                if (!moved[i]) {
                    payloads_intact = payloads_intact &&
                                      position.x == (float)tag[i] &&
                                      position.y == (float)(-tag[i]);
                }
                payloads_intact =
                    payloads_intact && sl_vec2_is_finite(position) &&
                    sl_vec2_is_finite(
                        sl_world_body_get_velocity(&world, probe));
            }
        }
        twins_in_sync = twins_in_sync && twin_worlds_in_sync(&world, &twin);
    }

    SL_EXPECT(counts_match);
    SL_EXPECT(validity_matches);
    SL_EXPECT(payloads_intact);
    SL_EXPECT(forces_settled);
    SL_EXPECT(twins_in_sync);
    SL_EXPECT(next_tag > 0);

    sl_world_destroy(&world);
    sl_world_destroy(&twin);
}

/* Circle r = 2, m = 2: I = m*r^2/2 = 4; tau = 8 drives alpha =
 * tau/I = 2 rad/s^2, so dt = 0.1 yields omega = 0.2, theta = 0.02. */
static void test_torque_integrates_angular_velocity_by_inv_inertia(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &circle));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 2.0f,
                          .shape = &circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT_NEAR(sl_world_body_get_inertia(&world, h), 4.0f, k_eps);

    SL_EXPECT(sl_world_body_apply_torque(&world, h, 3.0f));
    SL_EXPECT(sl_world_body_apply_torque(&world, h, 5.0f));
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 8.0f);

    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_angular_velocity(&world, h), 0.2f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.02f, k_eps);

    sl_world_destroy(&world);
}

static void test_torque_clears_after_step(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(2.0f, &circle));
    sl_body_desc desc = { .mass = 2.0f, .shape = &circle };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_apply_torque(&world, h, 8.0f));
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 0.0f);

    /* Torque-free step leaves the spin bitwise intact (no drag). */
    const float spin = sl_world_body_get_angular_velocity(&world, h);
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == spin);

    sl_world_destroy(&world);
}

/* Mirror of the linear drag test: divisor (1 + dt * angular_drag) = 2
 * at dt = 0.1, drag 10 halves spin every step; theta accumulates the
 * post-drag arc 0.05 then 0.075. */
static void test_angular_drag_halves_spin_per_step(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .linear_drag = 0.0f,
                               .angular_drag = 10.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .mass = 1.0f, .angular_velocity = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_angular_velocity(&world, h), 0.5f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.05f, k_eps);

    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_angular_velocity(&world, h), 0.25f, k_eps);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.075f, k_eps);

    sl_world_destroy(&world);
}

static void test_angle_wraps_after_full_turn(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* Five half-turns per second at dt = 0.1: a quarter-turn arc per
     * step, crossing +-pi repeatedly over 100 steps (~8 turns). */
    sl_body_desc desc = { .mass = 1.0f, .angular_velocity = 5.0f * SL_PI };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    for (uint32_t i = 0u; i < 100u; ++i) {
        sl_world_step(&world, k_dt);
        const float angle = sl_world_body_get_angle(&world, h);
        SL_EXPECT(angle <= SL_PI + k_eps && angle >= -SL_PI - k_eps);
    }
    /* Four quarter-arc steps land within one arc of a full turn. */
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.0f, 0.5f);

    sl_world_destroy(&world);
}

static void test_angle_stays_bounded_under_large_step_rotation(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* One step rotating 6.5*pi: the fmodf reduction path. */
    sl_body_desc desc = { .mass = 1.0f, .angular_velocity = 65.0f * SL_PI };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_world_step(&world, k_dt);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.5f * SL_PI, 1e-4f);

    sl_world_destroy(&world);
}

static void test_static_body_ignores_gravity_and_force(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .gravity = sl_vec2_make(0.0f, -10.0f),
                               .linear_drag = 5.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(3.0f, 4.0f),
                          .type = SL_BODY_STATIC };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(sl_world_body_get_mass(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_mass(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_type(&world, h) == SL_BODY_STATIC);

    SL_EXPECT(!sl_world_body_apply_force(&world, h, sl_vec2_make(1.0f, 0.0f)));
    SL_EXPECT(!sl_world_body_apply_torque(&world, h, 1.0f));
    SL_EXPECT(!sl_world_body_set_velocity(&world, h, sl_vec2_make(1.0f, 0.0f)));
    SL_EXPECT(!sl_world_body_set_angular_velocity(&world, h, 1.0f));
    SL_EXPECT(sl_world_body_set_velocity(&world, h, sl_vec2_make(0.0f, 0.0f)));

    for (uint32_t i = 0u; i < 5u; ++i) {
        sl_world_step(&world, k_dt);
        const sl_vec2 position = sl_world_body_get_position(&world, h);
        SL_EXPECT(position.x == 3.0f && position.y == 4.0f);
    }

    sl_world_destroy(&world);
}

static void test_kinematic_moves_without_drag_or_gravity(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .gravity = sl_vec2_make(0.0f, -10.0f),
                               .linear_drag = 10.0f,
                               .angular_drag = 10.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .velocity = sl_vec2_make(2.0f, 0.0f),
                          .mass = 0.0f,
                          .type = SL_BODY_KINEMATIC,
                          .angular_velocity = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));

    SL_EXPECT(!sl_world_body_apply_force(&world, h, sl_vec2_make(5.0f, 5.0f)));
    SL_EXPECT(!sl_world_body_apply_torque(&world, h, 5.0f));

    sl_world_step(&world, k_dt);
    const sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT_NEAR(position.x, 0.2f, k_eps);
    SL_EXPECT(position.y == 0.0f);
    const sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(velocity.x == 2.0f && velocity.y == 0.0f);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.1f, k_eps);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 1.0f);

    sl_world_destroy(&world);
}

/* A dynamic body without inertia still integrates its spin and accepts
 * torque (inv_inertia = 0 makes the torque term exactly zero); angular
 * drag is off here so the spin survives bitwise. */
static void test_shapeless_dynamic_spins_kinematically(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .mass = 1.0f, .angular_velocity = 3.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);

    SL_EXPECT(sl_world_body_apply_torque(&world, h, 100.0f));
    sl_world_step(&world, k_dt);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 3.0f);
    SL_EXPECT_NEAR(sl_world_body_get_angle(&world, h), 0.3f, k_eps);
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 0.0f);

    sl_world_destroy(&world);
}

/* Mixed-type churn: same pool model as above, but descriptors roll
 * dynamic / kinematic / static and none / circle / box shapes. Statics
 * must never leave their tag position bitwise, accumulators settle on
 * every row, and the twin world stays in lockstep throughout. */
#define MIXED_SEED 0xFEEDFACEu
#define MIXED_OPS 1024u
#define MIXED_CAPACITY 24u

static void test_churn_mixed_types_stress_matches_model(void)
{
    sl_world_config config = { .body_capacity = MIXED_CAPACITY,
                               .gravity = sl_vec2_make(0.2f, -9.8f),
                               .linear_drag = 1.5f,
                               .angular_drag = 0.7f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_world twin = { 0 };
    SL_EXPECT(sl_world_init(&twin, &config));

    sl_stepper stepper;
    sl_stepper twin_stepper;
    SL_EXPECT(sl_stepper_init(&stepper, k_dt));
    SL_EXPECT(sl_stepper_init(&twin_stepper, k_dt));

    sl_shape circle_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.25f, &circle_shape));
    sl_shape box_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.75f, 0.5f, &box_shape));

    uint32_t rng = MIXED_SEED;
    bool alive[MIXED_CAPACITY] = { false };
    sl_body_type types[MIXED_CAPACITY] = { 0 };
    sl_vec2 start[MIXED_CAPACITY];
    uint32_t generation[MIXED_CAPACITY];
    uint32_t live = 0u;
    int64_t next_tag = 0;
    for (uint32_t i = 0u; i < MIXED_CAPACITY; ++i) {
        generation[i] = 1u;
    }

    static const float k_frames[4] = { 0.0f, 0.005f, 0.01f, 0.35f };

    bool counts_match = true;
    bool validity_matches = true;
    bool statics_frozen = true;
    bool payloads_finite = true;
    bool accumulators_settled = true;
    bool twins_in_sync = true;

    for (uint32_t op = 0u; op < MIXED_OPS; ++op) {
        const uint32_t roll = xorshift32(&rng) % 32u;
        if (roll == 0u) {
            sl_world_reset(&world);
            sl_world_reset(&twin);
            for (uint32_t i = 0u; i < MIXED_CAPACITY; ++i) {
                generation[i] = model_generation_bump(generation[i]);
                alive[i] = false;
            }
            live = 0u;
        } else if (roll <= 13u) {
            const uint32_t type_roll = xorshift32(&rng) % 8u;
            const uint32_t shape_roll = xorshift32(&rng) % 3u;
            sl_body_desc desc = {
                .position = sl_vec2_make((float)next_tag, (float)(-next_tag)),
                .velocity = sl_vec2_make(0.0f, 0.0f),
                .mass = 1.0f + 0.25f * (float)(type_roll % 3u)
            };
            if (shape_roll == 1u) {
                desc.shape = &circle_shape;
            } else if (shape_roll == 2u) {
                desc.shape = &box_shape;
            }
            if (type_roll <= 4u) {
                desc.velocity = sl_vec2_make(0.3f, -0.2f);
                desc.angular_velocity = 0.4f;
            } else if (type_roll <= 6u) {
                desc.type = SL_BODY_KINEMATIC;
                desc.mass = 0.0f;
                desc.velocity = sl_vec2_make(0.5f, 0.0f);
                desc.angular_velocity = 1.0f;
            } else {
                desc.type = SL_BODY_STATIC;
                desc.mass = 0.0f;
            }

            sl_body_handle h = sl_world_body_create(&world, &desc);
            sl_body_handle ht = sl_world_body_create(&twin, &desc);
            const bool created = !sl_body_handle_is_null(h);
            counts_match = counts_match &&
                           created == !sl_body_handle_is_null(ht) &&
                           created == (live < MIXED_CAPACITY);
            if (created) {
                validity_matches = validity_matches && !alive[h.index] &&
                                   h.generation == generation[h.index];
                alive[h.index] = true;
                types[h.index] = desc.type;
                start[h.index] = desc.position;
                live++;
            }
            next_tag++;
        } else if (roll <= 22u) {
            uint32_t victim = MIXED_CAPACITY;
            for (uint32_t probe = 0u; probe < 8u && victim == MIXED_CAPACITY;
                 ++probe) {
                const uint32_t candidate = xorshift32(&rng) % MIXED_CAPACITY;
                if (alive[candidate]) {
                    victim = candidate;
                }
            }
            if (victim != MIXED_CAPACITY) {
                sl_body_handle h = { victim, generation[victim] };
                sl_world_body_destroy(&world, h);
                sl_world_body_destroy(&twin, h);
                alive[victim] = false;
                generation[victim] = model_generation_bump(generation[victim]);
                live--;
            }
        } else if (roll <= 26u) {
            uint32_t target = MIXED_CAPACITY;
            for (uint32_t probe = 0u; probe < 8u && target == MIXED_CAPACITY;
                 ++probe) {
                const uint32_t candidate = xorshift32(&rng) % MIXED_CAPACITY;
                if (alive[candidate]) {
                    target = candidate;
                }
            }
            if (target != MIXED_CAPACITY) {
                sl_body_handle h = { target, generation[target] };
                const bool expects_force = types[target] == SL_BODY_DYNAMIC;
                const sl_vec2 force =
                    sl_vec2_make((float)(roll % 7u) - 3.0f, (float)(roll % 5u));
                SL_EXPECT(sl_world_body_apply_force(&world, h, force) ==
                          expects_force);
                SL_EXPECT(sl_world_body_apply_force(&twin, h, force) ==
                          expects_force);
                SL_EXPECT(sl_world_body_apply_torque(
                              &world, h, (float)(roll % 3u) - 1.0f) ==
                          expects_force);
                SL_EXPECT(sl_world_body_apply_torque(
                              &twin, h, (float)(roll % 3u) - 1.0f) ==
                          expects_force);
            }
        } else {
            const float frame_time = k_frames[xorshift32(&rng) % 4u];
            const uint32_t stepped =
                sl_world_advance(&world, &stepper, frame_time);
            const uint32_t stepped_twin =
                sl_world_advance(&twin, &twin_stepper, frame_time);
            SL_EXPECT_INT_EQ(stepped, stepped_twin);
            if (stepped > 0u) {
                for (uint32_t row = 0u; row < world.body_count; ++row) {
                    accumulators_settled = accumulators_settled &&
                                           world.forces[row].x == 0.0f &&
                                           world.forces[row].y == 0.0f &&
                                           world.torques[row] == 0.0f;
                }
            }
        }

        counts_match =
            counts_match && sl_world_body_count(&world) == live &&
            sl_world_body_count(&world) == sl_world_body_count(&twin);
        for (uint32_t i = 0u; i < MIXED_CAPACITY; ++i) {
            sl_body_handle probe = { i, generation[i] };
            const bool engine_alive = sl_world_body_is_valid(&world, probe);
            validity_matches =
                validity_matches && engine_alive == alive[i] &&
                engine_alive == sl_world_body_is_valid(&twin, probe);
            if (!engine_alive || !alive[i]) {
                continue;
            }
            const sl_vec2 position = sl_world_body_get_position(&world, probe);
            payloads_finite =
                payloads_finite && sl_vec2_is_finite(position) &&
                sl_vec2_is_finite(sl_world_body_get_velocity(&world, probe)) &&
                sl_is_finite(sl_world_body_get_angle(&world, probe)) &&
                sl_is_finite(sl_world_body_get_angular_velocity(&world, probe));
            /* Statics are pinned by contract, not by luck: they never
             * integrate, so their tag position survives bitwise even
             * after hundreds of steps under gravity. Ungated on
             * purpose -- excusing rows that have been stepped would
             * skip exactly the case this is here to catch. */
            if (types[i] == SL_BODY_STATIC) {
                statics_frozen = statics_frozen && position.x == start[i].x &&
                                 position.y == start[i].y;
            }
        }
        twins_in_sync = twins_in_sync && twin_worlds_in_sync(&world, &twin);
    }

    SL_EXPECT(counts_match);
    SL_EXPECT(validity_matches);
    SL_EXPECT(statics_frozen);
    SL_EXPECT(payloads_finite);
    SL_EXPECT(accumulators_settled);
    SL_EXPECT(twins_in_sync);
    SL_EXPECT(next_tag > 0);

    sl_world_destroy(&world);
    sl_world_destroy(&twin);
}

static const sl_test_case k_cases[] = {
    { "stepper_init_rejects_bad_timestep",
      test_stepper_init_rejects_bad_timestep },
    { "step_moves_body_under_gravity", test_step_moves_body_under_gravity },
    { "step_applies_force_via_inv_mass", test_step_applies_force_via_inv_mass },
    { "force_clears_after_step", test_force_clears_after_step },
    { "apply_force_rejects_non_finite", test_apply_force_rejects_non_finite },
    { "apply_force_rejects_overflowing_sum",
      test_apply_force_rejects_overflowing_sum },
    { "apply_force_rejects_acceleration_overflow",
      test_apply_force_rejects_acceleration_overflow },
    { "step_holds_last_finite_state_on_overflow",
      test_step_holds_last_finite_state_on_overflow },
    { "step_rejection_count_saturates_and_reset_clears",
      test_step_rejection_count_saturates_and_reset_clears },
    { "drag_halves_velocity_per_step", test_drag_halves_velocity_per_step },
    { "drag_extreme_coefficient_stable", test_drag_extreme_coefficient_stable },
    { "advance_runs_steps_and_carries_remainder",
      test_advance_runs_steps_and_carries_remainder },
    { "advance_drops_debt_at_cap", test_advance_drops_debt_at_cap },
    { "advance_ignores_garbage_frame_time",
      test_advance_ignores_garbage_frame_time },
    { "determinism_identical_sequences_bitwise",
      test_determinism_identical_sequences_bitwise },
    { "results_independent_of_packed_layout",
      test_results_independent_of_packed_layout },
    { "config_rejects_bad_gravity_or_drag",
      test_config_rejects_bad_gravity_or_drag },
    { "torque_integrates_angular_velocity_by_inv_inertia",
      test_torque_integrates_angular_velocity_by_inv_inertia },
    { "torque_clears_after_step", test_torque_clears_after_step },
    { "angular_drag_halves_spin_per_step",
      test_angular_drag_halves_spin_per_step },
    { "angle_wraps_after_full_turn", test_angle_wraps_after_full_turn },
    { "angle_stays_bounded_under_large_step_rotation",
      test_angle_stays_bounded_under_large_step_rotation },
    { "static_body_ignores_gravity_and_force",
      test_static_body_ignores_gravity_and_force },
    { "kinematic_moves_without_drag_or_gravity",
      test_kinematic_moves_without_drag_or_gravity },
    { "shapeless_dynamic_spins_kinematically",
      test_shapeless_dynamic_spins_kinematically },
    { "churn_step_stress_matches_model", test_churn_step_stress_matches_model },
    { "churn_mixed_types_stress_matches_model",
      test_churn_mixed_types_stress_matches_model },
};

int sl_step_suite(void)
{
    return sl_run_suite("step", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
