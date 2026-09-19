#ifndef SILK_BENCH_FIXTURES_H
#define SILK_BENCH_FIXTURES_H
#include <silk/world.h>

/* Shared fixed workloads. This is benchmark tooling, not an engine API. */
#define SL_BENCH_PYRAMID_ROW_COUNT 20u
#define SL_BENCH_PYRAMID_BODY_COUNT                                            \
    (1u + (SL_BENCH_PYRAMID_ROW_COUNT * (SL_BENCH_PYRAMID_ROW_COUNT + 1u)) / 2u)
#define SL_BENCH_RAIN_CIRCLE_COUNT 2000u
#define SL_BENCH_RAIN_BODY_COUNT (3u + SL_BENCH_RAIN_CIRCLE_COUNT)
#define SL_BENCH_RAIN_CONTACT_CAPACITY 16384u
#define SL_BENCH_BODY_COUNT_MAX SL_BENCH_RAIN_BODY_COUNT
#define SL_BENCH_RAIN_SEED UINT32_C(0xC001D00D)

#define SL_BENCH_FIXTURE_COUNT 7u
#define SL_BENCH_STEP_COUNT_MAX 10000u
#define SL_BENCH_QUALITY_WINDOW 60u
#define SL_BENCH_SUBSTEP_COUNT 4u
#define SL_BENCH_TIMESTEP (1.0f / 60.0f)

typedef struct sl_bench_scene {
    sl_world *world;
    sl_world_config config;
    sl_transform reference[SL_BENCH_BODY_COUNT_MAX];
    sl_body_handle bodies[SL_BENCH_BODY_COUNT_MAX];
    uint32_t body_count;
    uint32_t seed;
    const char *name;
} sl_bench_scene;

typedef struct sl_bench_quality {
    float penetration_max;
    float cached_penetration_max;
    float translation_drift_max;
    float rotation_drift_max;
    float linear_speed_max;
    float angular_speed_max;
    float joint_error_max;
    double support_force_sum;
    double supported_weight;
    uint32_t window_steps;
} sl_bench_quality;

/* Fixture IDs and names are stable in native report order. Invalid IDs return
 * NULL. Build requires a zeroed/uninitialized world shell. Invalid arguments
 * leave inputs unchanged; setup failure releases the newly initialized world.
 * The caller owns and destroys the world after a successful build.
 * The scene borrows the shell through its lifetime and allocates no storage.
 * Reference/handle arrays cover at most SL_BENCH_BODY_COUNT_MAX bodies.
 */
const char *sl_bench_fixture_name(uint32_t fixture);
bool sl_bench_scene_build(sl_bench_scene *scene, sl_world *world,
                          uint32_t fixture, bool sleep_enabled);
/* Step index spans warm-up + measured steps, each bounded by the public
 * benchmark limit. Fixed seeded mutation is outside step timing regions. */
bool sl_bench_scene_mutate(sl_bench_scene *scene, uint32_t step);
void sl_bench_reference_capture(sl_bench_scene *scene);
bool sl_bench_quality_sample(sl_bench_scene *scene, sl_bench_quality *quality,
                             bool window);
#endif
