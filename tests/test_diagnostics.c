#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <silk/step.h>
#include <string.h>

static sl_body_handle add_body(sl_world *world, sl_body_type type, float x,
                               bool shaped)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &shape));
    const sl_body_desc desc = { .type = type,
                                .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                                .position = { x, 0.0f },
                                .shape = shaped ? &shape : NULL };
    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(body));
    return body;
}

static void expect_unassigned(const sl_world *world, sl_body_handle body)
{
    sl_island_stats value = { 23u, 17u, 11u, 5u };
    SL_EXPECT(!sl_world_body_get_island_stats(world, body, &value));
    SL_EXPECT_INT_EQ(value.id, 23u);
    SL_EXPECT_INT_EQ(value.dynamic_body_count, 17u);
    SL_EXPECT_INT_EQ(value.contact_count, 11u);
    SL_EXPECT_INT_EQ(value.joint_count, 5u);
}

static void island_snapshot_lifecycle(void)
{
    const sl_world_config config = { .body_capacity = 6u,
                                     .contact_capacity = 16u,
                                     .joint_capacity = 1u };
    sl_world world = { 0 }, twin = { 0 };
    sl_world *worlds[] = { &world, &twin };
    sl_body_handle bodies[2][5];
    for (uint32_t i = 0u; i < 2u; ++i) {
        SL_EXPECT(sl_world_init(worlds[i], &config));
        bodies[i][0] = add_body(worlds[i], SL_BODY_DYNAMIC, 0.0f, true);
        bodies[i][1] = add_body(worlds[i], SL_BODY_DYNAMIC, 1.5f, true);
        bodies[i][2] = add_body(worlds[i], SL_BODY_DYNAMIC, 10.0f, false);
        bodies[i][3] = add_body(worlds[i], SL_BODY_STATIC, 20.0f, false);
        bodies[i][4] = add_body(worlds[i], SL_BODY_KINEMATIC, 30.0f, false);
        const sl_joint_desc desc = { .body_a = bodies[i][0],
                                     .body_b = bodies[i][1],
                                     .kind = SL_JOINT_DISTANCE,
                                     .distance = { .length = 1.5f },
                                     .collide_connected = true };
        SL_EXPECT(
            !sl_joint_handle_is_null(sl_world_joint_create(worlds[i], &desc)));
        expect_unassigned(worlds[i], bodies[i][0]);
        sl_world_step(worlds[i], 1.0f / 60.0f);
    }
    const sl_contact *contact = sl_world_contact_at(&world, 0u);
    unsigned char contents[sizeof(*contact)];
    memcpy(contents, contact, sizeof(contents));
    sl_island_stats first = { 0 }, other = { 0 };
    for (uint32_t i = 0u; i < 10u; ++i) {
        SL_EXPECT(sl_world_body_get_island_stats(&world, bodies[0][0], &first));
        SL_EXPECT(sl_world_body_get_island_stats(&world, bodies[0][1], &other));
        SL_EXPECT_INT_EQ(first.id, other.id);
        SL_EXPECT_INT_EQ(first.dynamic_body_count, 2u);
        SL_EXPECT_INT_EQ(first.contact_count, 1u);
        SL_EXPECT_INT_EQ(first.joint_count, 1u);
        SL_EXPECT(sl_world_body_get_island_stats(&world, bodies[0][2], &other));
        SL_EXPECT(first.id != other.id);
        SL_EXPECT_INT_EQ(other.dynamic_body_count, 1u);
        SL_EXPECT_INT_EQ(other.contact_count, 0u);
        SL_EXPECT_INT_EQ(other.joint_count, 0u);
    }
    SL_EXPECT(contact == sl_world_contact_at(&world, 0u));
    SL_EXPECT(memcmp(contents, contact, sizeof(contents)) == 0);
    sl_replay_mismatch mismatch = { 0 };
    SL_EXPECT(sl_replay_compare(&world, &twin, &mismatch));
    SL_EXPECT(!sl_world_body_get_island_stats(&world, bodies[0][0], NULL));
    expect_unassigned(&world, bodies[0][3]);
    expect_unassigned(&world, bodies[0][4]);
    expect_unassigned(&world, sl_body_handle_null());
    sl_world_body_destroy(&world, bodies[0][1]);
    expect_unassigned(&world, bodies[0][1]);
    const sl_body_handle replacement =
        add_body(&world, SL_BODY_DYNAMIC, 50.0f, true);
    SL_EXPECT_INT_EQ(replacement.index, bodies[0][1].index);
    expect_unassigned(&world, replacement);
    SL_EXPECT(sl_world_body_get_island_stats(&world, bodies[0][0], &other));
    SL_EXPECT_INT_EQ(other.dynamic_body_count,
                     2u); /* Last build, before deletion. */
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT(sl_world_body_get_island_stats(&world, bodies[0][0], &other));
    SL_EXPECT_INT_EQ(other.dynamic_body_count, 1u);
    SL_EXPECT_INT_EQ(other.joint_count, 0u);
    sl_world_reset(&world);
    expect_unassigned(&world, replacement);
    const sl_body_handle reset_body =
        add_body(&world, SL_BODY_DYNAMIC, 0.0f, false);
    expect_unassigned(&world, reset_body);
    sl_world_destroy(&world);
    expect_unassigned(&world, reset_body);
    sl_world_destroy(&twin);
}

static void sleeping_body_remains_assigned(void)
{
    const sl_world_config config = { .body_capacity = 1u,
                                     .sleep_enabled = true };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_body_handle body = add_body(&world, SL_BODY_DYNAMIC, 0.0f, false);
    for (uint32_t step = 0u; step < 60u; ++step) {
        sl_world_step(&world, 1.0f / 60.0f);
    }
    SL_EXPECT(!sl_world_body_is_awake(&world, body));
    sl_island_stats island = { 0 };
    SL_EXPECT(sl_world_body_get_island_stats(&world, body, &island));
    SL_EXPECT_INT_EQ(island.dynamic_body_count, 1u);
    SL_EXPECT_INT_EQ(island.contact_count, 0u);
    SL_EXPECT_INT_EQ(island.joint_count, 0u);
    SL_EXPECT(!sl_world_body_is_awake(&world, body));
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "sleeping body remains assigned", sleeping_body_remains_assigned },
    { "copied island lifecycle and read-only state",
      island_snapshot_lifecycle },
};
int sl_diagnostics_suite(void)
{
    return sl_run_suite("diagnostics", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
