#include "adapter.h"
#include "context.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"

#include <float.h>
#include <math.h>
#include <string.h>

static sl_wasm_context *make_context(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c != NULL) {
        uint32_t *u = sl_wasm_input_u32(c);
        u[0] = 2u;
        SL_EXPECT(sl_wasm_world_bytes(c) > 0u);
        SL_EXPECT(sl_wasm_world_init(c));
    }
    return c;
}
static void body_input(sl_wasm_context *c)
{
    memset(sl_wasm_input_f32(c), 0, 32u * sizeof(float));
    memset(sl_wasm_input_u32(c), 0, 16u * sizeof(uint32_t));
    sl_wasm_input_f32(c)[4] = 1.0f;
}
static void lifecycle_and_rejection(void)
{
    sl_wasm_context *c = make_context();
    if (c == NULL) {
        return;
    }
    SL_EXPECT(!sl_wasm_world_init(c));
    body_input(c);
    SL_EXPECT(sl_wasm_shape_circle(c, 1.0f));
    SL_EXPECT(sl_wasm_body_create(c));
    const uint32_t i = sl_wasm_output_u32(c)[0];
    const uint32_t g = sl_wasm_output_u32(c)[1];
    SL_EXPECT(sl_wasm_body_read(c, i, g));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[10], 0.5f, 1e-6f);
    const float bad[] = { NAN, INFINITY, -INFINITY, 0.0f, -1.0f, FLT_MIN };
    for (uint32_t n = 0u; n < sizeof(bad) / sizeof(bad[0]); ++n) {
        SL_EXPECT(!sl_wasm_world_step(c, bad[n]));
        SL_EXPECT(sl_wasm_body_read(c, i, g));
        SL_EXPECT(sl_wasm_output_f32(c)[0] == 0.0f);
    }
    SL_EXPECT(!sl_wasm_body_set_position(c, i, g, NAN, 3.0f));
    SL_EXPECT(!sl_wasm_body_set_velocity(c, i, g, 1.0f, INFINITY));
    SL_EXPECT(sl_wasm_body_apply_force(c, i, g, 4.0f, 0.0f));
    SL_EXPECT(
        !sl_wasm_body_apply_force_at_point(c, i, g, 0.0f, 1.0f, FLT_MAX, 0.0f));
    SL_EXPECT(sl_wasm_world_step(c, 1.0f / 60.0f));
    SL_EXPECT(sl_wasm_body_read(c, i, g));
    SL_EXPECT(sl_wasm_output_f32(c)[0] > 0.0f);
    SL_EXPECT(sl_wasm_output_f32(c)[12] == 0.0f);
    SL_EXPECT(sl_wasm_body_create(c));
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 2u);
    SL_EXPECT(!sl_wasm_body_at(c, 2u));
    SL_EXPECT(sl_wasm_body_destroy(c, i, g));
    SL_EXPECT(!sl_wasm_body_read(c, i, g));
    SL_EXPECT(!sl_wasm_body_set_mass(c, i, g, 2.0f));
    SL_EXPECT(!sl_wasm_body_destroy(c, i, g));
    SL_EXPECT(sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], i);
    SL_EXPECT(sl_wasm_output_u32(c)[1] != g);
    const uint32_t replacement = sl_wasm_output_u32(c)[1];
    sl_wasm_world_reset(c);
    SL_EXPECT(!sl_wasm_body_valid(c, i, replacement));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 0u);
    sl_wasm_world_dispose(c);
    sl_wasm_world_dispose(c);
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT(!sl_wasm_world_step(c, 1.0f / 60.0f));
    sl_wasm_context_destroy(c);
}
static void config_and_shape_atomicity(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c == NULL) {
        return;
    }
    uint32_t *u = sl_wasm_input_u32(c);
    float *f = sl_wasm_input_f32(c);
    SL_EXPECT(!sl_wasm_world_init(c));
    u[0] = 1u;
    u[4] = 2u;
    SL_EXPECT(!sl_wasm_world_init(c));
    u[4] = 0u;
    f[0] = INFINITY;
    SL_EXPECT(!sl_wasm_world_init(c));
    f[0] = 0.0f;
    SL_EXPECT(sl_wasm_world_init(c));
    SL_EXPECT(sl_wasm_shape_box(c, 2.0f, 1.0f));
    sl_wasm_shape_mass(c);
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], 8.0f, 1e-6f);
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[3], 5.0f / 3.0f, 1e-6f);
    SL_EXPECT(sl_wasm_shape_aabb(c));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], -2.0f, 1e-6f);
    SL_EXPECT(sl_wasm_shape_contains(c));
    f[3] = -3.0f;
    f[5] = 6.0f;
    SL_EXPECT(sl_wasm_shape_ray(c));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], 1.0f / 6.0f, 1e-6f);
    f[2] = NAN;
    SL_EXPECT(!sl_wasm_shape_aabb(c));
    SL_EXPECT(!sl_wasm_shape_ray(c));
    f[2] = 0.0f;
    sl_wasm_shape_read(c);
    float before[17];
    memcpy(before, sl_wasm_output_f32(c) + 32, sizeof(before));
    SL_EXPECT(!sl_wasm_shape_circle(c, NAN));
    SL_EXPECT(!sl_wasm_shape_polygon(c, UINT32_MAX));
    u[0] = 2u;
    u[1] = 9u;
    SL_EXPECT(!sl_wasm_shape_load(c));
    sl_wasm_shape_read(c);
    SL_EXPECT(memcmp(before, sl_wasm_output_f32(c) + 32, sizeof(before)) == 0);
    body_input(c);
    f[4] = 0.0f;
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 0u);
    f[4] = 1.0f;
    SL_EXPECT(sl_wasm_body_create(c));
    sl_wasm_context_destroy(c);
}
static void independent_replay(void)
{
    sl_wasm_context *a = make_context(), *b = make_context();
    if (a == NULL || b == NULL) {
        sl_wasm_context_destroy(a);
        sl_wasm_context_destroy(b);
        return;
    }
    body_input(a);
    body_input(b);
    SL_EXPECT(sl_wasm_body_create(a));
    SL_EXPECT(sl_wasm_body_create(b));
    const uint32_t i = sl_wasm_output_u32(a)[0], g = sl_wasm_output_u32(a)[1];
    for (uint32_t n = 0u; n < 180u; ++n) {
        SL_EXPECT(sl_wasm_body_apply_force(a, i, g, 1.0f, -2.0f));
        SL_EXPECT(sl_wasm_body_apply_force(b, i, g, 1.0f, -2.0f));
        SL_EXPECT(sl_wasm_world_step(a, 1.0f / 60.0f));
        SL_EXPECT(sl_wasm_world_step(b, 1.0f / 60.0f));
        SL_EXPECT(sl_wasm_snapshot_refresh(a, 1u));
        SL_EXPECT(sl_wasm_query_point(a, 0u, 2u));
        sl_replay_mismatch mismatch = { 0 };
        SL_EXPECT(sl_replay_compare(&a->world, &b->world, &mismatch));
        SL_EXPECT(sl_wasm_body_read(a, i, g));
        SL_EXPECT(sl_wasm_body_read(b, i, g));
        SL_EXPECT(memcmp(sl_wasm_output_f32(a), sl_wasm_output_f32(b),
                         64u * sizeof(float)) == 0);
    }
    sl_wasm_world_reset(a);
    SL_EXPECT_INT_EQ(sl_wasm_body_count(b), 1u);
    sl_wasm_context_destroy(a);
    sl_wasm_context_destroy(b);
}
static void bulk_queries_and_memory(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c == NULL) {
        return;
    }
    uint32_t *u = sl_wasm_input_u32(c);
    float *f = sl_wasm_input_f32(c);
    u[0] = 4u;
    u[1] = 16u;
    u[2] = 2u;
    const size_t required = sl_wasm_adapter_bytes(c);
    SL_EXPECT(required ==
              sl_wasm_context_bytes() + 144u * 4u + 136u * 16u + 60u * 2u);
    SL_EXPECT(sl_wasm_world_init(c));
    SL_EXPECT(sl_wasm_adapter_bytes(c) == required);
    body_input(c);
    SL_EXPECT(sl_wasm_shape_circle(c, 1.0f));
    SL_EXPECT(sl_wasm_body_create(c));
    const sl_body_handle first = { sl_wasm_output_u32(c)[0],
                                   sl_wasm_output_u32(c)[1] };
    f[0] = 0.25f;
    f[4] = 0.0f;
    u[0] = SL_BODY_STATIC;
    SL_EXPECT(sl_wasm_body_create(c));
    f[0] = 5.0f;
    f[4] = 1.0f;
    u[0] = SL_BODY_DYNAMIC;
    SL_EXPECT(sl_wasm_body_create(c));
    f[0] = 0.0f;
    f[1] = 0.0f;
    SL_EXPECT(sl_wasm_query_point(c, 0u, 1u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 2u);
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[1], 1u);
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[2], 1u);
    SL_EXPECT_INT_EQ(sl_wasm_query_u32(c)[0], first.index);
    uint32_t query_before[8], output_before[32];
    memcpy(query_before, sl_wasm_query_u32(c), sizeof(query_before));
    memcpy(output_before, sl_wasm_output_u32(c), sizeof(output_before));
    SL_EXPECT(!sl_wasm_query_point(c, 8u, 1u));
    SL_EXPECT(!sl_wasm_query_point(c, 0u, 5u));
    f[0] = NAN;
    SL_EXPECT(!sl_wasm_query_point(c, 0u, 1u));
    SL_EXPECT(
        memcmp(query_before, sl_wasm_query_u32(c), sizeof(query_before)) == 0);
    SL_EXPECT(memcmp(output_before, sl_wasm_output_u32(c),
                     sizeof(output_before)) == 0);
    f[0] = 0.0f;
    SL_EXPECT(sl_wasm_query_point(c, SL_QUERY_DYNAMIC, 4u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 1u);
    SL_EXPECT(sl_wasm_query_point(c, 0u, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 2u);
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[2], 0u);
    f[0] = -3.0f;
    f[2] = 6.0f;
    f[3] = 0.0f;
    SL_EXPECT(sl_wasm_query_ray(c, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 1u);
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[1], first.index);
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], 1.0f / 3.0f, 1e-6f);
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 3u);
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[3], 3u);
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[3], 0u);
    c->body_f32[0] = 100.0f;
    SL_EXPECT(sl_world_body_get_position(&c->world, first).x == 0.0f);
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 0u));
    SL_EXPECT(c->body_f32[0] == 0.0f);
    SL_EXPECT(sl_wasm_shape_circle(c, 2.0f));
    SL_EXPECT(sl_wasm_body_set_shape(c, first.index, first.generation));
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[3], 1u);
    SL_EXPECT_NEAR(sl_wasm_geometry_f32(c)[first.index], 2.0f, 1e-6f);
    SL_EXPECT(sl_wasm_world_step(c, 1.0f / 60.0f));
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 1u));
    SL_EXPECT(sl_wasm_output_u32(c)[1] > 0u);
    const sl_contact *contact = sl_world_contact_at(&c->world, 0u);
    SL_EXPECT(sl_wasm_contacts_f32(c)[0] == contact->friction);
    SL_EXPECT_INT_EQ(sl_wasm_contacts_u32(c)[0], contact->body_a.index);
    for (uint32_t row = 0u; row < 3u; ++row) {
        const sl_body_handle h = sl_world_body_at(&c->world, row);
        const sl_transform t = sl_world_body_get_transform(&c->world, h);
        SL_EXPECT(sl_wasm_snapshot_f32(c)[row] == t.position.x);
        SL_EXPECT(sl_wasm_snapshot_f32(c)[4u + row] == t.position.y);
        SL_EXPECT(sl_wasm_snapshot_f32(c)[8u + row] == t.rotation.c);
        SL_EXPECT_INT_EQ(sl_wasm_snapshot_u32(c)[row], h.index);
    }
    SL_EXPECT(sl_wasm_memory_read(c));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[10],
                     sl_world_memory_bytes(&c->config));
    SL_EXPECT(sl_wasm_stats_read(c));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[2], 3u);
    SL_EXPECT(sl_wasm_work_data(c)[13u + 9u] > 0u);
    SL_EXPECT(sl_wasm_body_destroy(c, first.index, first.generation));
    body_input(c);
    SL_EXPECT(sl_wasm_shape_box(c, 0.25f, 0.5f));
    SL_EXPECT(sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], first.index);
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 0u));
    SL_EXPECT_INT_EQ(sl_wasm_geometry_u32(c)[first.index], SL_SHAPE_POLYGON);
    SL_EXPECT_INT_EQ(sl_wasm_geometry_u32(c)[4u + first.index], 4u);
    sl_wasm_world_reset(c);
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 1u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 0u);
    sl_wasm_world_dispose(c);
    SL_EXPECT(sl_wasm_snapshot_f32(c) == NULL);
    sl_wasm_context_destroy(c);
}
static void joints_counters_and_advance(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c == NULL) {
        return;
    }
    uint32_t *u = sl_wasm_input_u32(c);
    float *f = sl_wasm_input_f32(c);
    u[0] = 2u;
    u[2] = 1u;
    SL_EXPECT(sl_wasm_world_init(c));
    body_input(c);
    SL_EXPECT(sl_wasm_body_create(c));
    const sl_body_handle a = { sl_wasm_output_u32(c)[0],
                               sl_wasm_output_u32(c)[1] };
    f[0] = 2.0f;
    SL_EXPECT(sl_wasm_body_create(c));
    const sl_body_handle b = { sl_wasm_output_u32(c)[0],
                               sl_wasm_output_u32(c)[1] };
    memset(f, 0, 32u * sizeof(float));
    u[0] = SL_JOINT_DISTANCE;
    u[1] = a.index;
    u[2] = a.generation;
    u[3] = b.index;
    u[4] = b.generation;
    u[5] = 0u;
    f[4] = NAN;
    SL_EXPECT(!sl_wasm_joint_create(c));
    f[4] = 2.0f;
    SL_EXPECT(sl_wasm_joint_create(c));
    const sl_joint_handle joint = { sl_wasm_output_u32(c)[0],
                                    sl_wasm_output_u32(c)[1] };
    SL_EXPECT(!sl_wasm_joint_create(c));
    SL_EXPECT(sl_wasm_joint_read(c, joint.index, joint.generation));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[4], 2.0f, 1e-6f);
    SL_EXPECT(!sl_wasm_joint_at(c, 1u));
    SL_EXPECT(sl_wasm_world_advance(c, 1.0f / 60.0f, 0.0f, 1.0f));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], 8u);
    SL_EXPECT(sl_wasm_dropped_time(c) > 0.8);
    SL_EXPECT(!sl_wasm_world_advance(c, FLT_MIN, 0.0f, 1.0f));
    SL_EXPECT(!sl_wasm_world_advance(c, 1.0f / 60.0f, -1.0f, 1.0f));
    SL_EXPECT(sl_wasm_snapshot_refresh(c, 1u));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[2], 1u);
    SL_EXPECT_INT_EQ(sl_wasm_joints_u32(c)[0], joint.index);
    SL_EXPECT(sl_wasm_island_read(c, a.index, a.generation));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[1], 2u);
    SL_EXPECT(sl_wasm_body_destroy(c, a.index, a.generation));
    SL_EXPECT(!sl_wasm_joint_valid(c, joint.index, joint.generation));
    SL_EXPECT(!sl_wasm_joint_read(c, joint.index, joint.generation));
    sl_wasm_context_destroy(c);
    const sl_world_work work = { .tree_node_visits = UINT64_C(9007199254740993),
                                 .pair_candidates = UINT64_MAX,
                                 .contact_drops =
                                     UINT64_C(18446744073709551614) };
    uint64_t packed[13] = { 0 };
    sl_wasm_work_pack(packed, &work);
    SL_EXPECT(packed[0] == UINT64_C(9007199254740993));
    SL_EXPECT(packed[1] == UINT64_MAX);
    SL_EXPECT(packed[12] == UINT64_C(18446744073709551614));
}
static const sl_test_case k_cases[] = {
    { "bounded bulk queries and exact memory", bulk_queries_and_memory },
    { "joints exact counters and bounded advance",
      joints_counters_and_advance },
    { "adapter lifecycle and guarded stepping", lifecycle_and_rejection },
    { "adapter config and shape rejection", config_and_shape_atomicity },
    { "adapter independent replay", independent_replay },
};
int sl_wasm_suite(void)
{
    return sl_run_suite("wasm", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
