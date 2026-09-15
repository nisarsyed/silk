#include "../bench/fixtures.h"
#include "../bench/render/driver.h"
#include "../bench/render/overlay.h"
#include "../bench/render/scene.h"
#include "../wasm/context.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <float.h>
#include <math.h>
#include <silk/step.h>
#include <stdlib.h>
#include <string.h>

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
static void check_overlay(sl_render_study *study)
{
    const uint32_t *status = sl_render_study_status(study);
    const uint32_t b = status[5], c = status[6], j = status[7];
    const uint32_t lines = 4u * b + 2u * c + j + 6u;
    const uint32_t markers = 2u * c + 2u * j + 2u;
    const float *rect = sl_render_study_settings(study) + 17;
    const double scale = fmin(1280.0 / ((double)rect[2] - (double)rect[0]),
                              720.0 / ((double)rect[3] - (double)rect[1]));
    sl_render_overlay out = {
        .lines = calloc((size_t)lines * 5u + 1u, sizeof(float)),
        .markers = calloc((size_t)markers * 3u + 1u, sizeof(float)),
        .colors = calloc((size_t)b + 1u, sizeof(float)),
        .line_capacity = lines,
        .marker_capacity = markers,
        .color_count = b,
        .scale = scale,
        .offset_x = 640.0 - scale * ((double)rect[0] + (double)rect[2]) * 0.5,
        .offset_y = 360.0 + scale * ((double)rect[1] + (double)rect[3]) * 0.5
    };
    SL_EXPECT(out.lines != NULL && out.markers != NULL && out.colors != NULL);
    if (out.lines == NULL || out.markers == NULL || out.colors == NULL) {
        free(out.lines);
        free(out.markers);
        free(out.colors);
        return;
    }
    out.lines[5u * lines] = 73.0f;
    out.markers[3u * markers] = 79.0f;
    out.colors[b] = 83.0f;
    sl_wasm_context *adapter = sl_render_study_adapter(study);
    SL_EXPECT(sl_wasm_snapshot_refresh(adapter, 1u));
    SL_EXPECT(sl_render_overlay_prepare(study, &out));
    const uint32_t *snapshot = sl_wasm_snapshot_u32(adapter);
    const uint32_t *flags = sl_render_study_diagnostic_u32(study);
    uint32_t proxies = 0u, points = 0u;
    for (uint32_t row = 0u; row < b; ++row) {
        const uint32_t slot = snapshot[row];
        const float color = snapshot[2u * b + row] == (uint32_t)SL_BODY_STATIC
                                ? 0.0f
                            : snapshot[3u * b + row] == 0u ? 2.0f
                            : snapshot[4u * b + row] == UINT32_MAX
                                ? 1.0f
                                : (float)(3u + snapshot[4u * b + row] % 8u);
        SL_EXPECT(out.colors[row] == color);
        if ((flags[slot] & 8u) != 0u) {
            for (uint32_t edge = 0u; edge < 4u; ++edge) {
                SL_EXPECT(out.lines[5u * (4u * proxies + edge) + 4u] ==
                          ((flags[slot] & 7u) != 0u ? 15.0f : 13.0f));
            }
            ++proxies;
        }
    }
    const sl_world *world = sl_render_study_world(study);
    for (uint32_t row = 0u; row < sl_world_contact_count(world); ++row) {
        points += sl_world_contact_at(world, row)->manifold.point_count;
    }
    const uint32_t joints = sl_world_joint_count(world), hit = flags[b + 2u];
    SL_EXPECT_INT_EQ(out.line_count, 4u * proxies + points + joints + 5u + hit);
    SL_EXPECT_INT_EQ(out.marker_count, points + 2u * joints + 1u + hit);
    for (uint32_t point = 0u; point < points; ++point) {
        const float *line = out.lines + 5u * (4u * proxies + point);
        SL_EXPECT(line[4] == 12.0f);
        const double length = hypot((double)line[2] - (double)line[0],
                                    (double)line[3] - (double)line[1]);
        SL_EXPECT(fabs(length - 12.0) < 0.001);
    }
    for (uint32_t row = 0u; row < joints; ++row) {
        const sl_joint_desc joint =
            sl_world_joint_get_desc(world, sl_world_joint_at(world, row));
        const sl_transform a = sl_world_body_get_transform(world, joint.body_a);
        const double x = (double)a.position.x +
                         (double)a.rotation.c * (double)joint.local_anchor_a.x -
                         (double)a.rotation.s * (double)joint.local_anchor_a.y;
        const double y = (double)a.position.y +
                         (double)a.rotation.s * (double)joint.local_anchor_a.x +
                         (double)a.rotation.c * (double)joint.local_anchor_a.y;
        const float *line = out.lines + 5u * (4u * proxies + points + row);
        SL_EXPECT_NEAR(line[0], (float)(out.offset_x + x * scale), 0.0001f);
        SL_EXPECT_NEAR(line[1], (float)(out.offset_y - y * scale), 0.0001f);
        SL_EXPECT(line[4] == 14.0f);
    }
    SL_EXPECT(out.lines[5u * lines] == 73.0f);
    SL_EXPECT(out.markers[3u * markers] == 79.0f);
    SL_EXPECT(out.colors[b] == 83.0f);
    --out.line_capacity;
    SL_EXPECT(!sl_render_overlay_prepare(study, &out));
    ++out.line_capacity;
    --out.marker_capacity;
    SL_EXPECT(!sl_render_overlay_prepare(study, &out));
    ++out.marker_capacity;
    --out.color_count;
    SL_EXPECT(!sl_render_overlay_prepare(study, &out));
    ++out.color_count;
    out.scale = (double)NAN;
    SL_EXPECT(!sl_render_overlay_prepare(study, &out));
    out.scale = DBL_MAX;
    SL_EXPECT(!sl_render_overlay_prepare(study, &out));
    out.scale = scale;
    SL_EXPECT(sl_render_overlay_prepare(study, &out));
    SL_EXPECT(!sl_render_overlay_prepare(NULL, &out));
    SL_EXPECT(!sl_render_overlay_prepare(study, NULL));
    free(out.lines);
    free(out.markers);
    free(out.colors);
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
    check_overlay(study);
    sl_render_study_destroy(study);
}
static void diagnostic_columns(void)
{
    SL_EXPECT(!sl_render_study_diagnostics(NULL));
    SL_EXPECT(sl_render_study_diagnostic_f32(NULL) == NULL);
    SL_EXPECT(sl_render_study_diagnostic_u32(NULL) == NULL);
    SL_EXPECT(sl_render_study_diagnostic_bytes(NULL) == 0u);
    sl_render_study *study = sl_render_study_create(1u, 1u, 0u, 3u);
    SL_EXPECT(study != NULL);
    if (study == NULL) {
        return;
    }
    sl_world twin = { 0 };
    sl_world_config config = { 0 };
    const bool built = sl_render_scene_build(&twin, &config, 1u, 1u, false);
    SL_EXPECT(built);
    if (!built) {
        sl_render_study_destroy(study);
        return;
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        SL_EXPECT(sl_render_study_step(study));
        sl_world_step(&twin, SL_BENCH_TIMESTEP);
    }
    const uint32_t capacity = config.body_capacity;
    sl_body_handle *handles = calloc(capacity, sizeof(*handles));
    uint32_t *expected = calloc(capacity, sizeof(*expected));
    SL_EXPECT(handles != NULL && expected != NULL);
    if (handles == NULL || expected == NULL) {
        free(handles);
        free(expected);
        sl_world_destroy(&twin);
        sl_render_study_destroy(study);
        return;
    }
    SL_EXPECT(sl_render_study_diagnostics(study));
    SL_EXPECT(sl_replay_check(&twin, sl_render_study_world(study),
                              "diagnostic const refresh", SL_BENCH_RAIN_SEED,
                              3u));
    const float *values = sl_render_study_diagnostic_f32(study);
    const uint32_t *words = sl_render_study_diagnostic_u32(study);
    SL_EXPECT(sl_render_study_diagnostic_bytes(study) ==
              (size_t)capacity * 20u + 32u);
    for (uint32_t row = 0u; row < sl_world_body_count(&twin); ++row) {
        const sl_body_handle body = sl_world_body_at(&twin, row);
        sl_aabb bounds;
        const bool has_proxy =
            sl_world_body_get_proxy_aabb(&twin, body, &bounds);
        SL_EXPECT(((words[body.index] & 8u) != 0u) == has_proxy);
        if (has_proxy) {
            expected[body.index] = 8u;
            SL_EXPECT(values[body.index] == bounds.lower.x);
            SL_EXPECT(values[capacity + body.index] == bounds.lower.y);
            SL_EXPECT(values[2u * capacity + body.index] == bounds.upper.x);
            SL_EXPECT(values[3u * capacity + body.index] == bounds.upper.y);
        }
    }
    sl_query_result query = { 0 };
    SL_EXPECT(sl_world_query_point(&twin, (sl_vec2){ 0.0f, 12.0f },
                                   SL_QUERY_ALL, handles, capacity, &query));
    SL_EXPECT_INT_EQ(words[capacity], query.count);
    for (uint32_t i = 0u; i < query.count; ++i) {
        expected[handles[i].index] |= 1u;
    }
    SL_EXPECT(sl_world_query_aabb(
        &twin, (sl_aabb){ { -1.0f, 11.0f }, { 1.0f, 13.0f } }, SL_QUERY_ALL,
        handles, capacity, &query));
    SL_EXPECT_INT_EQ(words[capacity + 1u], query.count);
    for (uint32_t i = 0u; i < query.count; ++i) {
        expected[handles[i].index] |= 2u;
    }
    sl_query_ray_result ray = { 0 };
    SL_EXPECT(sl_world_query_ray(&twin,
                                 (sl_ray){ { -9.0f, 12.0f }, { 18.0f, 0.0f } },
                                 SL_QUERY_ALL, &ray));
    SL_EXPECT_INT_EQ(words[capacity + 2u], ray.hit ? 1u : 0u);
    if (ray.hit) {
        expected[ray.body.index] |= 4u;
    }
    SL_EXPECT(values[4u * capacity] == ray.geometry.fraction);
    SL_EXPECT(values[4u * capacity + 1u] == ray.geometry.point.x);
    SL_EXPECT(values[4u * capacity + 2u] == ray.geometry.point.y);
    SL_EXPECT(values[4u * capacity + 3u] == ray.geometry.normal.x);
    SL_EXPECT(values[4u * capacity + 4u] == ray.geometry.normal.y);
    for (uint32_t i = 0u; i < capacity; ++i) {
        SL_EXPECT_INT_EQ(words[i], expected[i]);
    }
    free(handles);
    free(expected);
    sl_world_destroy(&twin);
    sl_render_study_destroy(study);
}
static void direct_poses(void)
{
    float sentinel = 71.0f;
    SL_EXPECT(!sl_render_study_poses(NULL, &sentinel, 1u));
    SL_EXPECT(sentinel == 71.0f);
    const uint32_t fixtures[] = { 0u, 1u, 3u };
    const uint32_t tiers[] = { 1u, 16u };
    for (uint32_t fixture = 0u; fixture < 3u; ++fixture) {
        for (uint32_t tier = 0u; tier < 2u; ++tier) {
            for (uint32_t sleep = 0u; sleep < 2u; ++sleep) {
                sl_render_study *study = sl_render_study_create(
                    fixtures[fixture], tiers[tier], sleep, 3u);
                SL_EXPECT(study != NULL);
                if (study == NULL) {
                    continue;
                }
                sl_wasm_context *adapter = sl_render_study_adapter(study);
                const uint32_t count = sl_world_body_count(&adapter->world);
                const uint32_t capacity = sl_render_study_status(study)[5];
                float *storage = calloc((size_t)count * 4u + 2u, sizeof(float));
                SL_EXPECT(storage != NULL);
                if (storage == NULL) {
                    sl_render_study_destroy(study);
                    continue;
                }
                storage[0] = 71.0f;
                storage[4u * count + 1u] = 79.0f;
                SL_EXPECT(!sl_render_study_poses(study, NULL, count));
                SL_EXPECT(!sl_render_study_poses(study, &sentinel, count - 1u));
                SL_EXPECT(!sl_render_study_poses(study, &sentinel, count + 1u));
                SL_EXPECT(sentinel == 71.0f);
                for (uint32_t step = 0u; step < 3u; ++step) {
                    SL_EXPECT(sl_render_study_step(study));
                    SL_EXPECT(sl_wasm_snapshot_refresh(adapter, 0u));
                    SL_EXPECT(
                        sl_render_study_poses(study, storage + 1u, count));
                    const float *snapshot = sl_wasm_snapshot_f32(adapter);
                    for (uint32_t row = 0u; row < count; ++row) {
                        for (uint32_t column = 0u; column < 4u; ++column) {
                            // Packing must preserve every float bit, including
                            // signed zero; this is not an approximate oracle.
                            SL_EXPECT(memcmp(storage + 1u + 4u * row + column,
                                             snapshot + column * capacity + row,
                                             sizeof(float)) == 0);
                        }
                    }
                    SL_EXPECT(storage[0] == 71.0f);
                    SL_EXPECT(storage[4u * count + 1u] == 79.0f);
                    SL_EXPECT_INT_EQ(sl_render_study_status(study)[4],
                                     step + 1u);
                }
                check_overlay(study);
                free(storage);
                sl_render_study_destroy(study);
            }
        }
    }
}
static const sl_test_case k_cases[] = {
    { "co-located poses preserve snapshot bits and output bounds",
      direct_poses },
    { "reject unsupported scaling inputs", invalid_arguments },
    { "diagnostic columns match public queries without changing physics",
      diagnostic_columns },
    { "long study driver is bounded and keeps fixed storage", driver_bounds },
    { "all frozen physics tiers preserve descriptors and replay",
      scaling_matrix }
};
int sl_render_scene_suite(void)
{
    return sl_run_suite("render scenes", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
