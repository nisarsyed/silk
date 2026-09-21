#include "../bench/fixtures.h"
#include "../bench/quality.h"
#include "../bench/wasm/driver.h"
#include "../wasm/context.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <stdlib.h>

static void driver_rejection(void)
{
    SL_EXPECT(sl_wasm_bench_create(SL_BENCH_FIXTURE_COUNT, 0u, 0u, 1u) == NULL);
    SL_EXPECT(sl_wasm_bench_create(0u, 2u, 0u, 1u) == NULL);
    SL_EXPECT(sl_wasm_bench_create(0u, 0u, 10001u, 1u) == NULL);
    SL_EXPECT(sl_wasm_bench_create(0u, 0u, 0u, 0u) == NULL);
    SL_EXPECT(sl_wasm_bench_create(0u, 0u, 0u, 10001u) == NULL);
    SL_EXPECT(!sl_wasm_bench_prepare(NULL));
    SL_EXPECT(!sl_wasm_bench_mutate(NULL));
    SL_EXPECT(!sl_wasm_bench_step(NULL));
    SL_EXPECT(!sl_wasm_bench_sample(NULL));
    SL_EXPECT(!sl_wasm_bench_report(NULL));
    sl_wasm_bench_destroy(NULL);
    sl_wasm_bench *b = sl_wasm_bench_create(5u, 0u, 0u, 1u);
    SL_EXPECT(b != NULL);
    if (b == NULL) {
        return;
    }
    SL_EXPECT(sl_wasm_bench_report(b));
    const uint64_t digest = sl_wasm_bench_counters(b)[0];
    SL_EXPECT(!sl_wasm_bench_step(b));
    SL_EXPECT(!sl_wasm_bench_sample(b));
    SL_EXPECT(!sl_wasm_bench_mutate(b));
    SL_EXPECT(sl_wasm_bench_report(b));
    SL_EXPECT(sl_wasm_bench_counters(b)[0] == digest);
    SL_EXPECT_INT_EQ(sl_wasm_bench_words(b)[4], 0u);
    SL_EXPECT(sl_wasm_bench_prepare(b));
    SL_EXPECT(!sl_wasm_bench_prepare(b));
    SL_EXPECT(!sl_wasm_bench_step(b));
    SL_EXPECT(sl_wasm_bench_mutate(b));
    SL_EXPECT(!sl_wasm_bench_mutate(b));
    SL_EXPECT(sl_wasm_bench_step(b));
    SL_EXPECT(!sl_wasm_bench_step(b));
    SL_EXPECT(sl_wasm_bench_sample(b));
    SL_EXPECT(!sl_wasm_bench_sample(b));
    SL_EXPECT(!sl_wasm_bench_prepare(b));
    SL_EXPECT(sl_wasm_bench_report(b));
    SL_EXPECT_INT_EQ(sl_wasm_bench_words(b)[4], 1u);
    SL_EXPECT_INT_EQ(sl_wasm_bench_words(b)[7], 1u);
    sl_wasm_bench_destroy(b);
}
static void fixture_matrix_replay(void)
{
    for (uint32_t sleeping = 0u; sleeping < 2u; ++sleeping) {
        for (uint32_t fixture = 0u; fixture < SL_BENCH_FIXTURE_COUNT;
             ++fixture) {
            sl_wasm_bench *b = sl_wasm_bench_create(fixture, sleeping, 2u, 8u);
            sl_bench_scene *reference = calloc(1u, sizeof(*reference));
            sl_world world = { 0 };
            SL_EXPECT(b != NULL && reference != NULL);
            if (b == NULL || reference == NULL) {
                sl_wasm_bench_destroy(b);
                free(reference);
                return;
            }
            const bool built = sl_bench_scene_build(reference, &world, fixture,
                                                    sleeping != 0u);
            SL_EXPECT(built);
            if (!built) {
                sl_wasm_bench_destroy(b);
                free(reference);
                return;
            }
            sl_wasm_context *c = sl_wasm_bench_adapter(b);
            const void *storage = c->buffers;
            const size_t bytes = c->buffer_bytes;
            sl_bench_quality quality = { 0 };
            for (uint32_t i = 0u; i < 10u; ++i) {
                if (i == 2u) {
                    sl_bench_reference_capture(reference);
                }
                SL_EXPECT(sl_bench_scene_mutate(reference, i));
                sl_world_step(&world, SL_BENCH_TIMESTEP);
                if (i >= 2u) {
                    SL_EXPECT(
                        sl_bench_quality_sample(reference, &quality, true));
                }
                SL_EXPECT(sl_wasm_bench_prepare(b));
                SL_EXPECT(sl_wasm_bench_mutate(b));
                SL_EXPECT(sl_wasm_bench_step(b));
                SL_EXPECT(sl_wasm_bench_sample(b));
                SL_EXPECT(sl_wasm_snapshot_refresh(c, 1u));
                SL_EXPECT(sl_replay_check(&world, &c->world,
                                          sl_bench_fixture_name(fixture),
                                          reference->seed, i));
                SL_EXPECT(c->buffers == storage);
                SL_EXPECT(c->buffer_bytes == bytes);
            }
            SL_EXPECT(sl_wasm_bench_report(b));
            const uint64_t *counters = sl_wasm_bench_counters(b);
            SL_EXPECT(counters[0] == sl_bench_digest(&world, reference->bodies,
                                                     reference->body_count));
            SL_EXPECT(counters[1] == 0u);
            const double *values = sl_wasm_bench_quality(b);
            SL_EXPECT_NEAR(values[0], quality.penetration_max, 1e-8);
            SL_EXPECT_NEAR(values[2], quality.translation_drift_max, 1e-8);
            SL_EXPECT_NEAR(values[6], quality.joint_error_max, 1e-8);
            SL_EXPECT_NEAR(values[7], quality.support_force_sum / 8.0, 1e-8);
            SL_EXPECT_INT_EQ(sl_wasm_bench_words(b)[7], quality.window_steps);
            SL_EXPECT_INT_EQ(sl_wasm_bench_words(b)[8],
                             sl_world_body_capacity(&world));
            sl_wasm_bench_destroy(b);
            sl_world_destroy(&world);
            free(reference);
        }
    }
}
static const sl_test_case k_cases[] = {
    { "bounded driver rejects invalid phase and capacity", driver_rejection },
    { "all fixtures preserve public and private replay", fixture_matrix_replay }
};
int sl_wasm_bench_suite(void)
{
    return sl_run_suite("wasm benchmark", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
