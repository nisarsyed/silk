#include "fixture.h"
#include "silk_test.h"
#include "suites.h"

static void test_stepper_rejects_bad_timestep(void)
{
    sl_stepper stepper;
    SL_EXPECT(!sl_stepper_init(&stepper, 0.0f));
    SL_EXPECT(stepper.timestep == 0.0f && stepper.remainder == 0.0f);
    SL_EXPECT(!sl_stepper_init(&stepper, -1.0f));
    SL_EXPECT(!sl_stepper_init(&stepper, NAN));
    SL_EXPECT(sl_stepper_init(&stepper, 0.01f));
}

static void test_advance_accumulates_and_caps(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(16u, 16u);
    sl_cloth_desc d = sl_test_cloth_desc();
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 1u, 0.0f, false));
    sl_stepper stepper;
    SL_EXPECT(sl_stepper_init(&stepper, 0.25f));
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.1f), 0);
    SL_EXPECT_NEAR(stepper.remainder, 0.1f, 1e-7f);
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 0.4f), 2);
    SL_EXPECT_NEAR(stepper.remainder, 0.0f, 1e-6f);
    /* Ten steps of debt: eight run, the rest is dropped. */
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, 2.5f),
                     SL_STEP_COUNT_MAX);
    SL_EXPECT(stepper.remainder >= 0.0f && stepper.remainder < 0.25f);
    SL_EXPECT_NEAR(stepper.remainder, 0.0f, 0.0f);
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, NAN), 0);
    SL_EXPECT_INT_EQ(sl_world_advance(&world, &stepper, -1.0f), 0);
    sl_world_destroy(&world);
}

#ifdef NDEBUG
static void test_invalid_dt_is_ignored_in_release(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(16u, 16u);
    sl_cloth_desc d = sl_test_cloth_desc();
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 1u, 0.0f, false));
    const uint64_t hash = sl_test_world_hash(&world);
    sl_world_step(&world, 0.0f);
    sl_world_step(&world, -1.0f);
    sl_world_step(&world, NAN);
    SL_EXPECT(sl_test_world_hash(&world) == hash);
    sl_world_destroy(&world);
}
#endif

static const sl_test_case k_cases[] = {
    { "stepper_rejects_bad_timestep", test_stepper_rejects_bad_timestep },
    { "advance_accumulates_and_caps", test_advance_accumulates_and_caps },
#ifdef NDEBUG
    { "invalid_dt_ignored_release", test_invalid_dt_is_ignored_in_release },
#endif
};

int sl_step_suite(void)
{
    return sl_run_suite("step", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
