/* Deterministic rigid-body performance baseline.
 *
 * Scene construction and stepping depend only on committed constants and the
 * PRNG below. Wall-clock samples surround complete sl_world_step calls and
 * never enter simulation state. The checksum hashes six IEEE-754 binary32
 * fields per body (position, angle, linear velocity, angular velocity), in
 * ascending body-slot order captured while each scene is built. */

#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <silk/shape.h>
#include <silk/step.h>
#include <silk/world.h>

#include "sl_bench_config.h"

#define BENCH_PYRAMID_ROW_COUNT 20u
#define BENCH_PYRAMID_BODY_COUNT                                               \
    (1u + (BENCH_PYRAMID_ROW_COUNT * (BENCH_PYRAMID_ROW_COUNT + 1u)) / 2u)
#define BENCH_RAIN_CIRCLE_COUNT 2000u
#define BENCH_RAIN_BODY_COUNT (3u + BENCH_RAIN_CIRCLE_COUNT)
#define BENCH_RAIN_CONTACT_CAPACITY 16384u
#define BENCH_BODY_COUNT_MAX BENCH_RAIN_BODY_COUNT
#define BENCH_RAIN_SEED UINT32_C(0xC001D00D)

static const uint32_t k_warmup_step_count = 120u;
static const uint32_t k_measured_step_count = 600u;
static const uint32_t k_substep_count = 4u;
static const float k_timestep = 1.0f / 60.0f;
static const sl_vec2 k_gravity = { 0.0f, -9.81f };

typedef struct bench_scene {
    sl_world world;
    sl_body_handle bodies[BENCH_BODY_COUNT_MAX];
    uint32_t body_count;
    uint32_t seed;
    const char *name;
} bench_scene;

typedef struct bench_result {
    double average_ms;
    double maximum_ms;
    uint64_t checksum;
    uint64_t dropped_contact_count;
    uint32_t contact_count;
} bench_result;

static uint32_t rng_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    *state = x;
    return x;
}

/* Exactly representable uniform variate in [0, 1). */
static float rng_unit(uint32_t *state)
{
    return (float)(rng_next(state) >> 8u) * (1.0f / 16777216.0f);
}

static float rng_range(uint32_t *state, float low, float high)
{
    return low + (high - low) * rng_unit(state);
}

static bool scene_body_add(bench_scene *scene, const sl_body_desc *desc)
{
    if (scene->body_count >= BENCH_BODY_COUNT_MAX) {
        return false;
    }
    const sl_body_handle body = sl_world_body_create(&scene->world, desc);
    if (sl_body_handle_is_null(body)) {
        return false;
    }
    if (scene->body_count > 0u &&
        scene->bodies[scene->body_count - 1u].index >= body.index) {
        return false;
    }
    scene->bodies[scene->body_count] = body;
    scene->body_count += 1u;
    return true;
}

static bool scene_world_init(bench_scene *scene, const char *name,
                             uint32_t body_capacity, uint32_t contact_capacity,
                             uint32_t seed)
{
    memset(scene, 0, sizeof(*scene));
    const sl_world_config config = {
        .body_capacity = body_capacity,
        .contact_capacity = contact_capacity,
        .gravity = k_gravity,
        .substep_count = k_substep_count,
    };
    if (!sl_world_init(&scene->world, &config)) {
        return false;
    }
    scene->name = name;
    scene->seed = seed;
    return true;
}

static bool pyramid_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "pyramid", BENCH_PYRAMID_BODY_COUNT, 0u, 0u)) {
        return false;
    }

    sl_shape ground = sl_shape_none();
    sl_shape box = sl_shape_none();
    if (!sl_shape_make_box(12.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box)) {
        return false;
    }
    const sl_body_desc ground_desc = {
        .position = { 0.0f, -0.5f },
        .type = SL_BODY_STATIC,
        .shape = &ground,
        .friction = 0.6f,
    };
    if (!scene_body_add(scene, &ground_desc)) {
        return false;
    }

    for (uint32_t row = 0u; row < BENCH_PYRAMID_ROW_COUNT; ++row) {
        const uint32_t row_body_count = BENCH_PYRAMID_ROW_COUNT - row;
        for (uint32_t column = 0u; column < row_body_count; ++column) {
            const sl_body_desc desc = {
                .position = {
                    ((float)column - 0.5f * (float)(row_body_count - 1u)) *
                        1.01f,
                    0.5f + (float)row * 1.01f,
                },
                .mass = 1.0f,
                .shape = &box,
                .friction = 0.6f,
            };
            if (!scene_body_add(scene, &desc)) {
                return false;
            }
        }
    }
    return scene->body_count == BENCH_PYRAMID_BODY_COUNT;
}

static bool rain_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "rain", BENCH_RAIN_BODY_COUNT,
                          BENCH_RAIN_CONTACT_CAPACITY, BENCH_RAIN_SEED)) {
        return false;
    }

    sl_shape floor_shape = sl_shape_none();
    sl_shape wall_shape = sl_shape_none();
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_box(8.4f, 0.2f, &floor_shape) ||
        !sl_shape_make_box(0.2f, 12.0f, &wall_shape) ||
        !sl_shape_make_circle(0.15f, &circle)) {
        return false;
    }
    const sl_body_desc floor_desc = {
        .position = { 0.0f, -0.2f },
        .type = SL_BODY_STATIC,
        .shape = &floor_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    const sl_body_desc wall_left_desc = {
        .position = { -8.2f, 12.0f },
        .type = SL_BODY_STATIC,
        .shape = &wall_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    const sl_body_desc wall_right_desc = {
        .position = { 8.2f, 12.0f },
        .type = SL_BODY_STATIC,
        .shape = &wall_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    if (!scene_body_add(scene, &floor_desc) ||
        !scene_body_add(scene, &wall_left_desc) ||
        !scene_body_add(scene, &wall_right_desc)) {
        return false;
    }

    uint32_t rng = BENCH_RAIN_SEED;
    for (uint32_t i = 0u; i < BENCH_RAIN_CIRCLE_COUNT; ++i) {
        const uint32_t column = i % 40u;
        const uint32_t row = i / 40u;
        const sl_body_desc desc = {
            .position = {
                -7.41f + (float)column * 0.38f +
                    rng_range(&rng, -0.025f, 0.025f),
                0.8f + (float)row * 0.38f +
                    rng_range(&rng, -0.025f, 0.025f),
            },
            .velocity = {
                rng_range(&rng, -0.1f, 0.1f),
                rng_range(&rng, -0.05f, 0.05f),
            },
            .mass = 0.1f,
            .shape = &circle,
            .friction = 0.4f,
            .restitution = 0.1f,
        };
        if (!scene_body_add(scene, &desc)) {
            return false;
        }
    }
    return scene->body_count == BENCH_RAIN_BODY_COUNT;
}

static bool timer_now(struct timespec *out)
{
    if (timespec_get(out, TIME_UTC) != TIME_UTC) {
        return false;
    }
    return out->tv_nsec >= 0L && out->tv_nsec < 1000000000L;
}

static bool timer_elapsed_ms(const struct timespec *start,
                             const struct timespec *end, double *out)
{
    const double seconds = difftime(end->tv_sec, start->tv_sec);
    const double nanoseconds = (double)end->tv_nsec - (double)start->tv_nsec;
    const double elapsed_ms = seconds * 1000.0 + nanoseconds * 1e-6;
    if (!isfinite(elapsed_ms) || elapsed_ms < 0.0) {
        return false;
    }
    *out = elapsed_ms;
    return true;
}

static uint64_t checksum_float(uint64_t hash, float value)
{
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    return (hash ^ (uint64_t)bits) * UINT64_C(1099511628211);
}

static uint64_t scene_checksum(const bench_scene *scene)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    for (uint32_t i = 0u; i < scene->body_count; ++i) {
        const sl_body_handle body = scene->bodies[i];
        const sl_vec2 position =
            sl_world_body_get_position(&scene->world, body);
        const sl_vec2 velocity =
            sl_world_body_get_velocity(&scene->world, body);
        hash = checksum_float(hash, position.x);
        hash = checksum_float(hash, position.y);
        hash =
            checksum_float(hash, sl_world_body_get_angle(&scene->world, body));
        hash = checksum_float(hash, velocity.x);
        hash = checksum_float(hash, velocity.y);
        hash = checksum_float(
            hash, sl_world_body_get_angular_velocity(&scene->world, body));
    }
    return hash;
}

static bool scene_run(bench_scene *scene, bench_result *result)
{
    uint64_t dropped_contact_count = 0u;
    for (uint32_t i = 0u; i < k_warmup_step_count; ++i) {
        sl_world_step(&scene->world, k_timestep);
        dropped_contact_count +=
            (uint64_t)sl_world_contact_drop_count(&scene->world);
    }

    double total_ms = 0.0;
    double maximum_ms = 0.0;
    for (uint32_t i = 0u; i < k_measured_step_count; ++i) {
        struct timespec start = { 0 };
        struct timespec end = { 0 };
        if (!timer_now(&start)) {
            return false;
        }
        sl_world_step(&scene->world, k_timestep);
        if (!timer_now(&end)) {
            return false;
        }
        double elapsed_ms = 0.0;
        if (!timer_elapsed_ms(&start, &end, &elapsed_ms)) {
            return false;
        }
        total_ms += elapsed_ms;
        if (elapsed_ms > maximum_ms) {
            maximum_ms = elapsed_ms;
        }
        dropped_contact_count +=
            (uint64_t)sl_world_contact_drop_count(&scene->world);
    }

    result->average_ms = total_ms / (double)k_measured_step_count;
    result->maximum_ms = maximum_ms;
    result->checksum = scene_checksum(scene);
    result->dropped_contact_count = dropped_contact_count;
    result->contact_count = sl_world_contact_count(&scene->world);
    return isfinite(result->average_ms) && isfinite(result->maximum_ms);
}

static const char *metadata_or_none(const char *value)
{
    return value[0] == '\0' ? "(none)" : value;
}

static void metadata_print(void)
{
    printf("silk rigid-body benchmark\n");
    printf("compiler: %s %s\n", SL_BENCH_COMPILER_ID,
           SL_BENCH_COMPILER_VERSION);
    printf("build_mode: %s\n", metadata_or_none(SL_BENCH_BUILD_MODE));
    printf("cmake_c_flags: %s\n", metadata_or_none(SL_BENCH_C_FLAGS));
    printf("target_warning_flags: %s\n",
           metadata_or_none(SL_BENCH_WARNING_FLAGS));
    printf("host: %s %s (%s)\n", SL_BENCH_HOST_SYSTEM, SL_BENCH_HOST_VERSION,
           SL_BENCH_HOST_PROCESSOR);
    printf("checksum: FNV-1a over binary32 position/angle/velocity, "
           "ascending body-slot order\n");
}

static void result_print(const bench_scene *scene, const bench_result *result)
{
    printf("scene: %s\n", scene->name);
    printf("  seed: 0x%08" PRIx32 "\n", scene->seed);
    printf("  timestep_seconds: %.9g\n", (double)k_timestep);
    printf("  substeps: %" PRIu32 "\n", k_substep_count);
    printf("  gravity: (%.9g, %.9g)\n", (double)k_gravity.x,
           (double)k_gravity.y);
    printf("  body_count: %" PRIu32 "\n", scene->body_count);
    printf("  body_capacity: %" PRIu32 "\n",
           sl_world_body_capacity(&scene->world));
    printf("  contact_capacity: %" PRIu32 "\n",
           sl_world_contact_capacity(&scene->world));
    printf("  final_contact_count: %" PRIu32 "\n", result->contact_count);
    printf("  dropped_contacts_all_steps: %" PRIu64 "\n",
           result->dropped_contact_count);
    printf("  warmup_steps: %" PRIu32 "\n", k_warmup_step_count);
    printf("  measured_steps: %" PRIu32 "\n", k_measured_step_count);
    printf("  average_ms_per_step: %.6f\n", result->average_ms);
    printf("  maximum_ms_per_step: %.6f\n", result->maximum_ms);
    printf("  checksum: 0x%016" PRIx64 "\n", result->checksum);
}

static bool binary32_supported(void)
{
    return sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
           FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128;
}

static bool benchmark_run(bool (*build)(bench_scene *))
{
    bench_scene scene;
    if (!build(&scene)) {
        fprintf(stderr, "failed to construct benchmark scene\n");
        sl_world_destroy(&scene.world);
        return false;
    }

    bench_result result = { 0 };
    const bool ran = scene_run(&scene, &result);
    if (!ran) {
        fprintf(stderr,
                "rejected an invalid or backward timing sample for %s\n",
                scene.name);
        sl_world_destroy(&scene.world);
        return false;
    }
    result_print(&scene, &result);
    sl_world_destroy(&scene.world);
    return true;
}

int main(void)
{
    if (!binary32_supported()) {
        fprintf(stderr, "checksum requires IEEE-754 binary32 float\n");
        return EXIT_FAILURE;
    }
    metadata_print();
    if (!benchmark_run(pyramid_build) || !benchmark_run(rain_build)) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
