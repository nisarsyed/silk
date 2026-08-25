#include "silk_test.h"
#include "suites.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          1.0f };
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
    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          2.0f };
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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          1.0f };
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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          1.0f };
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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          1.0f };
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

static void test_drag_halves_velocity_per_step(void)
{
    sl_world_config config = { .body_capacity = 1u,
                               .gravity = sl_vec2_make(0.0f, 0.0f),
                               .linear_drag = 10.0f };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT(sl_world_get_linear_drag(&world) == 10.0f);

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(1.0f, 0.0f),
                          1.0f };
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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(1.0f, 0.0f),
                          1.0f };
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

    sl_body_desc desc = { sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f),
                          1.0f };
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

/* Full SoA state of two worlds compared bitwise, row by row. */
static bool twin_worlds_in_sync(const sl_world *a, const sl_world *b)
{
    if (a->body_count != b->body_count || a->free_count != b->free_count) {
        return false;
    }
    for (uint32_t i = 0u; i < a->body_count; ++i) {
        const bool rows_match = a->positions[i].x == b->positions[i].x &&
                                a->positions[i].y == b->positions[i].y &&
                                a->velocities[i].x == b->velocities[i].x &&
                                a->velocities[i].y == b->velocities[i].y &&
                                a->forces[i].x == b->forces[i].x &&
                                a->forces[i].y == b->forces[i].y &&
                                a->masses[i] == b->masses[i] &&
                                a->inv_masses[i] == b->inv_masses[i];
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

    for (uint32_t round = 0u; round < 24u; ++round) {
        if (count < 8u && round % 3u != 2u) {
            sl_body_desc desc = { sl_vec2_make((float)round, -(float)round),
                                  sl_vec2_make(0.1f * (float)round, 0.0f),
                                  1.0f + 0.5f * (float)round };
            ha[count] = sl_world_body_create(&a, &desc);
            hb[count] = sl_world_body_create(&b, &desc);
            SL_EXPECT(!sl_body_handle_is_null(ha[count]));
            SL_EXPECT(!sl_body_handle_is_null(hb[count]));
            SL_EXPECT(sl_world_body_apply_force(&a, ha[count],
                                                sl_vec2_make(2.0f, -1.0f)));
            SL_EXPECT(sl_world_body_apply_force(&b, hb[count],
                                                sl_vec2_make(2.0f, -1.0f)));
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
    const sl_body_desc descs[6] = {
        { sl_vec2_make(1.0f, 0.0f), sl_vec2_make(0.5f, 0.0f), 1.0f },
        { sl_vec2_make(2.0f, 0.0f), sl_vec2_make(-0.5f, 0.0f), 2.0f },
        { sl_vec2_make(3.0f, 0.0f), sl_vec2_make(0.25f, 0.0f), 3.0f },
        { sl_vec2_make(4.0f, 0.0f), sl_vec2_make(-0.25f, 0.0f), 4.0f },
        { sl_vec2_make(5.0f, 0.0f), sl_vec2_make(0.75f, 0.0f), 5.0f },
        { sl_vec2_make(6.0f, 0.0f), sl_vec2_make(-0.75f, 0.0f), 6.0f },
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

    /* Zeroed trailing fields are the documented defaults. */
    sl_world_config defaults = { .body_capacity = 2u };
    SL_EXPECT(sl_world_init(&world, &defaults));
    sl_vec2 gravity = sl_world_get_gravity(&world);
    SL_EXPECT(gravity.x == 0.0f && gravity.y == 0.0f);
    SL_EXPECT(sl_world_get_linear_drag(&world) == 0.0f);

    /* Defaults coast: pure linear motion, exactly one timestep of arc. */
    sl_body_desc desc = { sl_vec2_make(1.0f, 1.0f), sl_vec2_make(2.0f, 0.0f),
                          1.0f };
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

static uint32_t rng_state = CHURN_SEED;

static uint32_t rng_next(void)
{
    rng_state ^= rng_state << 13u;
    rng_state ^= rng_state >> 17u;
    rng_state ^= rng_state << 5u;
    return rng_state;
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
        const uint32_t roll = rng_next() % 40u;
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
            sl_body_desc desc = { sl_vec2_make((float)next_tag,
                                               (float)(-next_tag)),
                                  sl_vec2_make(0.0f, 0.0f), 1.0f };
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
                const uint32_t candidate = rng_next() % CHURN_CAPACITY;
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
                const uint32_t candidate = rng_next() % CHURN_CAPACITY;
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
            const float frame_time = k_frames[rng_next() % 4u];
            const uint32_t stepped =
                sl_world_advance(&world, &stepper, frame_time);
            const uint32_t stepped_twin =
                sl_world_advance(&twin, &twin_stepper, frame_time);
            SL_EXPECT_INT_EQ(stepped, stepped_twin);
            if (stepped > 0u) {
                for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
                    moved[i] = moved[i] || alive[i];
                }
                /* Every executed step consumes the accumulators. */
                for (uint32_t row = 0u; row < world.body_count; ++row) {
                    forces_settled = forces_settled &&
                                     world.forces[row].x == 0.0f &&
                                     world.forces[row].y == 0.0f;
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

static const sl_test_case k_cases[] = {
    { "stepper_init_rejects_bad_timestep",
      test_stepper_init_rejects_bad_timestep },
    { "step_moves_body_under_gravity", test_step_moves_body_under_gravity },
    { "step_applies_force_via_inv_mass", test_step_applies_force_via_inv_mass },
    { "force_clears_after_step", test_force_clears_after_step },
    { "apply_force_rejects_non_finite", test_apply_force_rejects_non_finite },
    { "apply_force_rejects_overflowing_sum",
      test_apply_force_rejects_overflowing_sum },
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
    { "churn_step_stress_matches_model", test_churn_step_stress_matches_model },
};

int sl_step_suite(void)
{
    return sl_run_suite("step", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
