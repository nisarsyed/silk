#include "../bench/fixtures.h"
#include "../bench/render/driver.h"
#include "../bench/render/scene.h"
#include "../wasm/context.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <silk/step.h>
#include <stdlib.h>

static void invalid_arguments(void)
{
    sl_world world = { 0 };
    sl_world_config config = { .body_capacity = 37u };
    SL_EXPECT(!sl_render_scene_build(NULL, &config, 0u, 1u, false));
    SL_EXPECT(!sl_render_scene_build(&world, NULL, 0u, 1u, false));
    const uint32_t invalid_copies[] = { 0u, 3u, 17u, UINT32_MAX };
    for (uint32_t i = 0u; i < 4u; ++i) {
        SL_EXPECT(!sl_render_scene_build(&world, &config, 0u, invalid_copies[i],
                                         false));
    }
    const uint32_t invalid_fixtures[] = { 2u, 4u, 5u, 6u, 7u, UINT32_MAX };
    for (uint32_t i = 0u; i < 6u; ++i) {
        SL_EXPECT(!sl_render_scene_build(&world, &config, invalid_fixtures[i],
                                         1u, false));
    }
    SL_EXPECT_INT_EQ(config.body_capacity, 37u);
    sl_world_destroy(&world);
}
static void body_matches(const sl_world *source, sl_body_handle a,
                         const sl_world *world, sl_body_handle b, float offset)
{
    const sl_vec2 pa = sl_world_body_get_position(source, a);
    const sl_vec2 pb = sl_world_body_get_position(world, b);
    const sl_vec2 va = sl_world_body_get_velocity(source, a);
    const sl_vec2 vb = sl_world_body_get_velocity(world, b);
    /* Exact values are the property: copy one source descriptor and perform
     * the contract's single binary32 x translation, with no JS reconstruction.
     */
    SL_EXPECT(pb.x == pa.x + offset && pb.y == pa.y);
    SL_EXPECT(vb.x == va.x && vb.y == va.y);
    SL_EXPECT_INT_EQ(sl_world_body_get_type(source, a),
                     sl_world_body_get_type(world, b));
    SL_EXPECT(sl_world_body_get_mass(source, a) ==
              sl_world_body_get_mass(world, b));
    SL_EXPECT(sl_world_body_get_angle(source, a) ==
              sl_world_body_get_angle(world, b));
    SL_EXPECT(sl_world_body_get_angular_velocity(source, a) ==
              sl_world_body_get_angular_velocity(world, b));
    SL_EXPECT(sl_world_body_get_friction(source, a) ==
              sl_world_body_get_friction(world, b));
    SL_EXPECT(sl_world_body_get_restitution(source, a) ==
              sl_world_body_get_restitution(world, b));
    const sl_shape *sa = sl_world_body_get_shape(source, a);
    const sl_shape *sb = sl_world_body_get_shape(world, b);
    SL_EXPECT_INT_EQ(sa->kind, sb->kind);
    if (sa->kind == SL_SHAPE_CIRCLE && sb->kind == SL_SHAPE_CIRCLE) {
        SL_EXPECT(sa->circle.radius == sb->circle.radius);
    } else if (sa->kind == SL_SHAPE_POLYGON && sb->kind == SL_SHAPE_POLYGON) {
        SL_EXPECT_INT_EQ(sa->polygon.count, sb->polygon.count);
        for (uint32_t v = 0u; v < sa->polygon.count; ++v) {
            SL_EXPECT(sa->polygon.vertices[v].x == sb->polygon.vertices[v].x);
            SL_EXPECT(sa->polygon.vertices[v].y == sb->polygon.vertices[v].y);
        }
    }
}
static void scaling_matrix(void)
{
    const uint32_t fixtures[] = { 0u, 1u, 3u };
    const uint32_t tiers[] = { 1u, 2u, 4u, 8u, 16u };
    for (uint32_t sleeping = 0u; sleeping < 2u; ++sleeping) {
        for (uint32_t f = 0u; f < 3u; ++f) {
            sl_bench_scene *source = calloc(1u, sizeof(*source));
            SL_EXPECT(source != NULL);
            if (source == NULL) {
                return;
            }
            sl_world original = { 0 };
            const bool built = sl_bench_scene_build(
                source, &original, fixtures[f], sleeping != 0u);
            SL_EXPECT(built);
            if (!built) {
                free(source);
                return;
            }
            const uint32_t joint_count = sl_world_joint_count(&original);
            for (uint32_t t = 0u; t < 5u; ++t) {
                sl_world world = { 0 }, twin = { 0 };
                sl_world_config config = { 0 }, twin_config = { 0 };
                const uint32_t copies = tiers[t];
                const bool ok =
                    sl_render_scene_build(&world, &config, fixtures[f], copies,
                                          sleeping != 0u) &&
                    sl_render_scene_build(&twin, &twin_config, fixtures[f],
                                          copies, sleeping != 0u);
                SL_EXPECT(ok);
                if (!ok) {
                    sl_world_destroy(&world);
                    sl_world_destroy(&twin);
                    sl_world_destroy(&original);
                    free(source);
                    return;
                }
                SL_EXPECT_INT_EQ(sl_world_body_count(&world),
                                 source->body_count * copies);
                SL_EXPECT_INT_EQ(config.body_capacity,
                                 source->config.body_capacity * copies);
                SL_EXPECT_INT_EQ(config.contact_capacity,
                                 sl_world_contact_capacity(&original) * copies);
                SL_EXPECT_INT_EQ(config.joint_capacity,
                                 source->config.joint_capacity * copies);
                SL_EXPECT_INT_EQ(sl_world_joint_count(&world),
                                 joint_count * copies);
                for (uint32_t copy = 0u; copy < copies; ++copy) {
                    const float offset =
                        32.0f * ((float)copy - (float)(copies - 1u) * 0.5f);
                    const uint32_t first = copy * source->body_count;
                    for (uint32_t row = 0u; row < source->body_count; ++row) {
                        body_matches(&original, source->bodies[row], &world,
                                     sl_world_body_at(&world, first + row),
                                     offset);
                    }
                    for (uint32_t row = 0u; row < joint_count; ++row) {
                        const sl_joint_desc a = sl_world_joint_get_desc(
                            &original, sl_world_joint_at(&original, row));
                        const sl_joint_desc b = sl_world_joint_get_desc(
                            &world, sl_world_joint_at(
                                        &world, copy * joint_count + row));
                        SL_EXPECT_INT_EQ(b.body_a.index,
                                         first + a.body_a.index);
                        SL_EXPECT_INT_EQ(b.body_b.index,
                                         first + a.body_b.index);
                        SL_EXPECT_INT_EQ(a.kind, b.kind);
                        if (a.kind == SL_JOINT_DISTANCE) {
                            SL_EXPECT(a.distance.length == b.distance.length);
                        }
                        SL_EXPECT(a.local_anchor_a.x == b.local_anchor_a.x);
                        SL_EXPECT(a.local_anchor_a.y == b.local_anchor_a.y);
                        SL_EXPECT(a.local_anchor_b.x == b.local_anchor_b.x);
                        SL_EXPECT(a.local_anchor_b.y == b.local_anchor_b.y);
                        SL_EXPECT(a.collide_connected == b.collide_connected);
                    }
                }
                if (copies == 1u) {
                    SL_EXPECT(sl_replay_check(&original, &world, source->name,
                                              source->seed, 0u));
                }
                for (uint32_t step = 0u; step < 3u; ++step) {
                    sl_world_step(&world, SL_BENCH_TIMESTEP);
                    sl_world_step(&twin, SL_BENCH_TIMESTEP);
                    SL_EXPECT(sl_replay_check(&world, &twin, source->name,
                                              source->seed, step));
                    SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world), 0u);
                }
                sl_world_destroy(&world);
                sl_world_destroy(&twin);
            }
            sl_world_destroy(&original);
            free(source);
        }
    }
}
static void driver_bounds(void)
{
    SL_EXPECT(sl_render_study_create(0u, 1u, 0u, 0u) == NULL);
    SL_EXPECT(sl_render_study_create(0u, 1u, 0u,
                                     SL_RENDER_STEP_COUNT_MAX + 1u) == NULL);
    SL_EXPECT(sl_render_study_create(0u, 1u, 2u, 1u) == NULL);
    SL_EXPECT(sl_render_study_create(4u, 1u, 0u, 1u) == NULL);
    SL_EXPECT(sl_render_study_create(0u, 3u, 0u, 1u) == NULL);
    SL_EXPECT(!sl_render_study_step(NULL));
    SL_EXPECT(sl_render_study_adapter(NULL) == NULL);
    SL_EXPECT(sl_render_study_world(NULL) == NULL);
    SL_EXPECT(sl_render_study_status(NULL) == NULL);
    SL_EXPECT(sl_render_study_drops(NULL) == 0u);
    sl_render_study_destroy(NULL);
    sl_render_study *largest =
        sl_render_study_create(1u, 16u, 0u, SL_RENDER_STEP_COUNT_MAX);
    SL_EXPECT(largest != NULL);
    if (largest != NULL) {
        SL_EXPECT_INT_EQ(sl_render_study_status(largest)[5], 32048u);
        SL_EXPECT_INT_EQ(sl_render_study_status(largest)[6], 262144u);
        SL_EXPECT(
            sl_wasm_snapshot_refresh(sl_render_study_adapter(largest), 1u));
        sl_render_study_destroy(largest);
    }
    // Demonstrate stepping beyond the original CLI cap, then exact rejection
    // at this owner's declared bound. Sleep affects only test execution cost.
    sl_render_study *study = sl_render_study_create(0u, 1u, 1u, 10001u);
    SL_EXPECT(study != NULL);
    if (study == NULL) {
        return;
    }
    sl_wasm_context *adapter = sl_render_study_adapter(study);
    const void *buffers = adapter->buffers;
    const size_t bytes = adapter->buffer_bytes;
    SL_EXPECT(sl_render_study_world(study) == &adapter->world);
    for (uint32_t i = 0u; i < 10001u; ++i) {
        SL_EXPECT(sl_render_study_step(study));
    }
    SL_EXPECT_INT_EQ(sl_render_study_status(study)[4], 10001u);
    SL_EXPECT(sl_render_study_drops(study) == 0u);
    SL_EXPECT(adapter->buffers == buffers && adapter->buffer_bytes == bytes);
    SL_EXPECT(sl_wasm_snapshot_refresh(adapter, 1u));
    const sl_vec2 position = sl_world_body_get_position(
        &adapter->world, sl_world_body_at(&adapter->world, 1u));
    SL_EXPECT(!sl_render_study_step(study));
    const sl_vec2 after = sl_world_body_get_position(
        &adapter->world, sl_world_body_at(&adapter->world, 1u));
    SL_EXPECT(position.x == after.x && position.y == after.y);
    SL_EXPECT_INT_EQ(sl_render_study_status(study)[4], 10001u);
    sl_render_study_destroy(study);
}
static const sl_test_case k_cases[] = {
    { "reject unsupported scaling inputs", invalid_arguments },
    { "long study driver is bounded and keeps fixed storage", driver_bounds },
    { "all frozen physics tiers preserve descriptors and replay",
      scaling_matrix }
};
int sl_render_scene_suite(void)
{
    return sl_run_suite("render scenes", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
