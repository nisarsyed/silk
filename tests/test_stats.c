#include "replay.h"
#include "silk_test.h"
#include "stats.h"
#include "suites.h"
#include <math.h>
#include <silk/step.h>

static sl_body_handle ball(sl_world *world, float x)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &shape));
    const sl_body_desc desc = { .mass = 1.0f,
                                .position = { x, 0.0f },
                                .shape = &shape };
    return sl_world_body_create(world, &desc);
}
static void tiny_work_and_lifetime(void)
{
    const sl_world_config config = { .body_capacity = 2u,
                                     .contact_capacity = 1u };
    sl_world world = { 0 }, twin = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT(sl_world_init(&twin, &config));
    const sl_body_handle a = ball(&world, 0.0f);
    (void)ball(&world, 2.1f);
    (void)ball(&twin, 0.0f);
    (void)ball(&twin, 2.1f);
    sl_world_stats stats = sl_world_get_stats(&world);
    SL_EXPECT_INT_EQ(stats.body_count, 2u);
    SL_EXPECT_INT_EQ(stats.body_count_high, 2u);
    SL_EXPECT(stats.cumulative.proxy_creates == 2u);
    SL_EXPECT_INT_EQ(stats.step.substep_count, 0u);
    sl_world_step(&world, 1.0f / 60.0f);
    sl_world_step(&twin, 1.0f / 60.0f);
    stats = sl_world_get_stats(&world);
    SL_EXPECT(stats.step.work.tree_node_visits == 6u);
    SL_EXPECT(stats.step.work.pair_candidates == 4u);
    SL_EXPECT(stats.step.work.pair_probes == 2u);
    SL_EXPECT(stats.step.work.proxy_creates == 0u);
    SL_EXPECT_INT_EQ(stats.step.dynamic_body_count, 2u);
    SL_EXPECT_INT_EQ(stats.step.contact_constraint_count, 0u);
    SL_EXPECT_INT_EQ(stats.pair_count, 1u);
    SL_EXPECT_INT_EQ(stats.pair_capacity, 2u);
    SL_EXPECT_INT_EQ(stats.contact_count_high, 1u);
    const sl_contact *snapshot = sl_world_contact_at(&world, 0u);
    for (uint32_t i = 0u; i < 10u; ++i) {
        (void)sl_world_get_stats(&world);
    }
    SL_EXPECT(snapshot == sl_world_contact_at(&world, 0u));
    sl_replay_mismatch mismatch = { 0 };
    SL_EXPECT(sl_replay_compare(&world, &twin, &mismatch));
    SL_EXPECT(!sl_world_body_set_position(&world, a, (sl_vec2){ NAN, 0.0f }));
#ifdef NDEBUG
    sl_world_step(&world, NAN);
    sl_world_step(&world, 0.0f);
#endif
    SL_EXPECT(sl_replay_compare(&world, &twin, &mismatch));
    sl_world_step(&world, 1.0f / 60.0f);
    sl_world_step(&twin, 1.0f / 60.0f);
    stats = sl_world_get_stats(&world);
    SL_EXPECT(stats.step.work.tree_node_visits == 0u &&
              stats.step.work.pair_candidates == 0u);
    SL_EXPECT(sl_replay_compare(&world, &twin, &mismatch));
    SL_EXPECT(sl_world_body_set_position(&world, a, (sl_vec2){ -5.0f, 0.0f }));
    stats = sl_world_get_stats(&world);
    SL_EXPECT(stats.cumulative.proxy_moves == 1u);
    SL_EXPECT(stats.step.work.proxy_moves == 0u);
    sl_world_body_destroy(&world, a);
    stats = sl_world_get_stats(&world);
    SL_EXPECT(stats.cumulative.proxy_destroys == 1u);
    SL_EXPECT_INT_EQ(stats.body_count, 1u);
    SL_EXPECT_INT_EQ(stats.body_count_high, 2u);
    sl_world_reset(&world);
    stats = sl_world_get_stats(&world);
    SL_EXPECT_INT_EQ(stats.body_count, 0u);
    SL_EXPECT_INT_EQ(stats.body_capacity, 2u);
    SL_EXPECT_INT_EQ(stats.body_count_high, 0u);
    SL_EXPECT_INT_EQ(stats.step.substep_count, 0u);
    SL_EXPECT(stats.cumulative.pair_probes == 0u &&
              stats.cumulative.proxy_moves == 0u);
    sl_world_destroy(&world);
    sl_world_destroy(&twin);
}
static void memory_accounting(void)
{
    const uint32_t capacities[] = { 1u, 3u, 4u, SL_BODY_COUNT_MAX };
    for (uint32_t i = 0u; i < 4u; ++i) {
        for (uint32_t joints = 0u; joints < 3u; ++joints) {
            const sl_world_config config = {
                .body_capacity = capacities[i],
                .joint_capacity = joints == 2u ? SL_JOINT_COUNT_MAX : joints
            };
            sl_world_memory_breakdown m = { 0 };
            SL_EXPECT(sl_world_memory_breakdown_get(&config, &m));
            SL_EXPECT(m.body_bytes + m.broadphase_bytes + m.contact_bytes +
                          m.pair_bytes + m.contact_solver_bytes +
                          m.joint_bytes + m.padding_bytes ==
                      m.arena_bytes);
            SL_EXPECT(m.arena_bytes == sl_world_memory_bytes(&config));
            SL_EXPECT(m.world_bytes == sizeof(sl_world));
            SL_EXPECT((m.joint_bytes == 0u) == (joints == 0u));
            /* 169 payload bytes: 16 ownership + 32 vec2 + 16 rotation +
             * 32 scalar + 1 type + 72 shape, before slice padding. */
            SL_EXPECT(m.body_bytes == (size_t)capacities[i] * 169u);
        }
    }
    const sl_world_config invalid = { .body_capacity = 0u };
    sl_world_memory_breakdown sentinel = { .arena_bytes = 123u };
    SL_EXPECT(!sl_world_memory_breakdown_get(&invalid, &sentinel));
    SL_EXPECT(sentinel.arena_bytes == 123u && sentinel.body_bytes == 0u);
    SL_EXPECT(!sl_world_memory_breakdown_get(NULL, &sentinel));
    SL_EXPECT(!sl_world_memory_breakdown_get(&invalid, NULL));
}
static void saturation_and_empty_step(void)
{
    SL_EXPECT(sl_stats_sum(UINT64_MAX - 1u, 2u) == UINT64_MAX);
    const sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    world.stats.cumulative.proxy_creates = UINT64_MAX;
    (void)ball(&world, 0.0f);
    SL_EXPECT(sl_world_get_stats(&world).cumulative.proxy_creates ==
              UINT64_MAX);
    world.stats.cumulative.proxy_moves = UINT64_MAX;
    const sl_body_handle body = sl_world_body_at(&world, 0u);
    SL_EXPECT(
        sl_world_body_set_velocity(&world, body, (sl_vec2){ 20.0f, 0.0f }));
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT(sl_world_get_stats(&world).step.work.proxy_moves == 1u);
    SL_EXPECT(sl_world_get_stats(&world).cumulative.proxy_moves == UINT64_MAX);
    sl_world_reset(&world);
    sl_world_step(&world, 1.0f / 60.0f);
    const sl_world_stats stats = sl_world_get_stats(&world);
    SL_EXPECT_INT_EQ(stats.step.substep_count, 4u);
    SL_EXPECT_INT_EQ(stats.step.dynamic_body_count, 0u);
    SL_EXPECT(stats.step.work.pair_probes == 0u);
    sl_world_destroy(&world);
}
static const sl_test_case k_cases[] = {
    { "tiny work and snapshot lifetime", tiny_work_and_lifetime },
    { "exact memory categories", memory_accounting },
    { "counter saturation and empty step", saturation_and_empty_step },
};
int sl_stats_suite(void)
{
    return sl_run_suite("stats", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
