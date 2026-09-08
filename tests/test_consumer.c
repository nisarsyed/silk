#include "silk_test.h"
#include "suites.h"
#include <silk/query.h>
#include <silk/step.h>
#include <silk/world.h>

/* This translation unit is compiled without the private src include path. */
static void public_lifecycle(void)
{
    sl_world world = { 0 };
    const sl_world_config invalid = { 0 };
    SL_EXPECT(!sl_world_init(&world, &invalid));
    SL_EXPECT_INT_EQ(sl_world_body_capacity(&world), 0u);
    sl_world_destroy(&world);
    const sl_world_config config = { .body_capacity = 2u,
                                     .joint_capacity = 1u };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(0.25f, &shape));
    const sl_body_desc desc = { .mass = 1.0f, .shape = &shape };
    const sl_body_handle a = sl_world_body_create(&world, &desc);
    const sl_body_handle b = sl_world_body_create(&world, &desc);
    const sl_joint_desc joint_desc = { .kind = SL_JOINT_REVOLUTE,
                                       .body_a = a,
                                       .body_b = b };
    const sl_joint_handle joint = sl_world_joint_create(&world, &joint_desc);
    SL_EXPECT(sl_world_body_is_awake(&world, a));
    SL_EXPECT(sl_world_body_wake(&world, a));
    SL_EXPECT(sl_world_body_is_valid(&world, a));
    SL_EXPECT(sl_world_joint_is_valid(&world, joint));
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT(sl_vec2_is_finite(sl_world_body_get_position(&world, a)));
    SL_EXPECT_INT_EQ(sl_world_get_stats(&world).joint_count, 1u);
    sl_query_result matches;
    sl_body_handle found[2];
    SL_EXPECT(sl_world_query_aabb(&world,
                                  (sl_aabb){ { -2.0f, -2.0f }, { 2.0f, 2.0f } },
                                  0u, found, 2u, &matches));
    SL_EXPECT_INT_EQ(matches.count, 2u);
    sl_world_reset(&world);
    SL_EXPECT(!sl_world_body_is_valid(&world, a));
    SL_EXPECT(!sl_world_joint_is_valid(&world, joint));
    sl_world_destroy(&world);
    sl_world_destroy(&world);
    SL_EXPECT(!sl_world_body_is_valid(&world, a));
}
static const sl_test_case k_cases[] = { { "public_lifecycle",
                                          public_lifecycle } };
int sl_consumer_suite(void)
{
    return sl_run_suite("consumer", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
