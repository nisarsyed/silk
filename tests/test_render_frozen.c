#include "../bench/fixtures.h"
#include "../bench/render/driver.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <math.h>
#include <silk/query.h>
#include <silk/step.h>
#include <stdlib.h>

static void invalid_inputs(void)
{
    SL_EXPECT(sl_render_frozen_create(NULL, 256u) == NULL);
    SL_EXPECT(sl_render_frozen_f32(NULL) == NULL);
    SL_EXPECT(sl_render_frozen_u32(NULL) == NULL);
    SL_EXPECT_INT_EQ(sl_render_frozen_count(NULL), 0u);
    SL_EXPECT(sl_render_frozen_bytes(NULL) == 0u);
    sl_render_frozen_destroy(NULL);
    const uint32_t fixtures[] = { 0u, 1u, 3u };
    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_render_study *study =
            sl_render_study_create(fixtures[i], 1u, 0u, 121u);
        SL_EXPECT(study != NULL);
        if (study == NULL) {
            continue;
        }
        SL_EXPECT(sl_render_frozen_create(study, 256u) == NULL);
        for (uint32_t step = 0u; step < 120u; ++step) {
            SL_EXPECT(sl_render_study_step(study));
        }
        if (fixtures[i] != 1u) {
            SL_EXPECT(sl_render_frozen_create(study, 256u) == NULL);
        }
        const uint32_t invalid[] = { 0u, 128u, 255u, 257u, 65537u, UINT32_MAX };
        for (uint32_t j = 0u; j < 6u; ++j) {
            SL_EXPECT(sl_render_frozen_create(study, invalid[j]) == NULL);
        }
        SL_EXPECT(sl_render_study_step(study));
        SL_EXPECT(sl_render_frozen_create(study, 256u) == NULL);
        sl_render_study_destroy(study);
    }
}

static bool copy_instances(sl_world *out, const sl_world *source,
                           uint32_t count, const float *bounds,
                           uint32_t *expected)
{
    sl_body_handle rows[SL_BENCH_RAIN_CIRCLE_COUNT];
    uint32_t moving = 0u;
    for (uint32_t row = 0u; row < SL_BENCH_RAIN_BODY_COUNT; ++row) {
        const sl_body_handle body = sl_world_body_at(source, row);
        if (sl_world_body_get_type(source, body) != SL_BODY_STATIC) {
            if (moving == SL_BENCH_RAIN_CIRCLE_COUNT) {
                return false;
            }
            rows[moving++] = body;
        }
    }
    SL_EXPECT_INT_EQ(moving, SL_BENCH_RAIN_CIRCLE_COUNT);
    if (moving != SL_BENCH_RAIN_CIRCLE_COUNT) {
        return false;
    }
    const uint32_t tiles = (count + moving - 1u) / moving;
    for (uint32_t row = 0u; row < count; ++row) {
        const sl_body_handle body = rows[row % moving];
        const float offset =
            32.0f * ((float)(row / moving) - 0.5f * (float)(tiles - 1u));
        sl_vec2 position = sl_world_body_get_position(source, body);
        position.x += offset;
        const sl_body_desc desc = { .position = position,
                                    .mass = 1.0f,
                                    .shape =
                                        sl_world_body_get_shape(source, body) };
        const sl_body_handle created = sl_world_body_create(out, &desc);
        if (sl_body_handle_is_null(created)) {
            return false;
        }
        SL_EXPECT_INT_EQ(created.index, row);
        sl_aabb proxy;
        if (sl_world_body_get_proxy_aabb(source, body, &proxy)) {
            expected[row] = 8u;
            // The exact copied bits, not approximate geometry, are tested.
            SL_EXPECT(bounds[row] == proxy.lower.x + offset);
            SL_EXPECT(bounds[count + row] == proxy.lower.y);
            SL_EXPECT(bounds[2u * count + row] == proxy.upper.x + offset);
            SL_EXPECT(bounds[3u * count + row] == proxy.upper.y);
        }
    }
    return true;
}

static void compare_queries(sl_render_study *study, uint32_t count)
{
    sl_render_frozen *frozen = sl_render_frozen_create(study, count);
    SL_EXPECT(frozen != NULL);
    if (frozen == NULL) {
        return;
    }
    SL_EXPECT_INT_EQ(sl_render_frozen_count(frozen), count);
    SL_EXPECT(sl_render_frozen_bytes(frozen) >= (size_t)count * 20u + 32u);
    const float *values = sl_render_frozen_f32(frozen);
    const uint32_t *words = sl_render_frozen_u32(frozen);
    for (uint32_t i = 0u; i < count * 4u + 5u; ++i) {
        SL_EXPECT(isfinite(values[i]));
    }
    // A separate, unstepped query-only world checks the public world query
    // semantics on exactly the displayed circles. No runtime helper world is
    // added by the implementation, and the source simulation remains intact.
    sl_world world = { 0 };
    const sl_world_config config = { .body_capacity = count,
                                     .contact_capacity = 1u };
    const bool initialized = sl_world_init(&world, &config);
    sl_body_handle *handles = calloc(count, sizeof(*handles));
    uint32_t *expected = calloc(count, sizeof(*expected));
    SL_EXPECT(initialized && handles != NULL && expected != NULL);
    if (initialized && handles != NULL && expected != NULL &&
        copy_instances(&world, sl_render_study_world(study), count, values,
                       expected)) {
        sl_query_result result = { 0 };
        SL_EXPECT(sl_world_query_point(&world, (sl_vec2){ 0.0f, 12.0f },
                                       SL_QUERY_ALL, handles, count, &result));
        SL_EXPECT(!result.truncated);
        SL_EXPECT_INT_EQ(result.count, words[count]);
        for (uint32_t i = 0u; i < result.count; ++i) {
            expected[handles[i].index] |= 1u;
        }
        SL_EXPECT(sl_world_query_aabb(
            &world, (sl_aabb){ { -1.0f, 11.0f }, { 1.0f, 13.0f } },
            SL_QUERY_ALL, handles, count, &result));
        SL_EXPECT(!result.truncated);
        SL_EXPECT_INT_EQ(result.count, words[count + 1u]);
        for (uint32_t i = 0u; i < result.count; ++i) {
            expected[handles[i].index] |= 2u;
        }
        const uint32_t tiles = (count + SL_BENCH_RAIN_CIRCLE_COUNT - 1u) /
                               SL_BENCH_RAIN_CIRCLE_COUNT;
        const float half_width = 9.0f + 16.0f * (float)(tiles - 1u);
        sl_query_ray_result hit = { 0 };
        SL_EXPECT(sl_world_query_ray(
            &world,
            (sl_ray){ { -half_width, 12.0f }, { 2.0f * half_width, 0.0f } },
            SL_QUERY_ALL, &hit));
        SL_EXPECT_INT_EQ(words[count + 2u], hit.hit ? 1u : 0u);
        if (hit.hit) {
            expected[hit.body.index] |= 4u;
        }
        const float ray[] = { hit.geometry.fraction, hit.geometry.point.x,
                              hit.geometry.point.y, hit.geometry.normal.x,
                              hit.geometry.normal.y };
        for (uint32_t i = 0u; i < 5u; ++i) {
            SL_EXPECT(values[4u * count + i] == ray[i]);
        }
        for (uint32_t i = 0u; i < count; ++i) {
            SL_EXPECT_INT_EQ(words[i], expected[i]);
        }
    } else {
        SL_EXPECT(false);
    }
    free(expected);
    free(handles);
    sl_world_destroy(&world);
    sl_render_frozen_destroy(frozen);
}

static void tiers_match_public_queries(void)
{
    sl_render_study *study = sl_render_study_create(1u, 1u, 0u, 120u);
    sl_render_study *twin = sl_render_study_create(1u, 1u, 0u, 120u);
    SL_EXPECT(study != NULL && twin != NULL);
    if (study != NULL && twin != NULL) {
        for (uint32_t step = 0u; step < 120u; ++step) {
            SL_EXPECT(sl_render_study_step(study));
            SL_EXPECT(sl_render_study_step(twin));
        }
        for (uint32_t count = 256u; count <= 65536u; count *= 2u) {
            compare_queries(study, count);
        }
        SL_EXPECT(sl_replay_check(
            sl_render_study_world(study), sl_render_study_world(twin),
            "frozen render queries", SL_BENCH_RAIN_SEED, 120u));
    }
    sl_render_study_destroy(twin);
    sl_render_study_destroy(study);
}

static const sl_test_case k_cases[] = {
    { "frozen queries reject invalid source stages and tiers", invalid_inputs },
    { "all render tiers match public queries without simulation mutation",
      tiers_match_public_queries }
};
int sl_render_frozen_suite(void)
{
    return sl_run_suite("frozen render queries", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
