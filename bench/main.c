/* Deterministic benchmark matrix. Timing surrounds complete world steps and
 * never enters simulation state. One semantic digest covers public state;
 * JSON is the report format for every invocation. */

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

#include "quality.h"
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
    sl_world_config config;
    sl_transform reference[BENCH_BODY_COUNT_MAX];
    sl_body_handle bodies[BENCH_BODY_COUNT_MAX];
    uint32_t body_count;
    uint32_t seed;
    const char *name;
} bench_scene;

#define BENCH_STEP_COUNT_MAX 10000u
#define BENCH_QUALITY_WINDOW 60u

typedef struct bench_options {
    const char *scene;
    uint32_t warmup;
    uint32_t steps;
} bench_options;
typedef struct bench_quality {
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
} bench_quality;
typedef struct bench_result {
    double average_ms;
    double median_ms;
    double p95_ms;
    double maximum_ms;
    double mutation_average_ms;
    uint64_t digest;
    uint64_t dropped_contact_count;
    sl_world_stats stats;
    sl_world_memory_breakdown memory;
    bench_quality quality;
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
                             sl_world_config config, uint32_t seed)
{
    memset(scene, 0, sizeof(*scene));
    if (!sl_world_init(&scene->world, &config)) {
        return false;
    }
    scene->config = config;
    scene->name = name;
    scene->seed = seed;
    return true;
}

static bool pyramid_build(bench_scene *scene)
{
    if (!scene_world_init(
            scene, "pyramid",
            (sl_world_config){ .body_capacity = BENCH_PYRAMID_BODY_COUNT,
                               .gravity = k_gravity,
                               .substep_count = k_substep_count },
            0u)) {
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
    if (!scene_world_init(
            scene, "rain",
            (sl_world_config){ .body_capacity = BENCH_RAIN_BODY_COUNT,
                               .contact_capacity = BENCH_RAIN_CONTACT_CAPACITY,
                               .gravity = k_gravity,
                               .substep_count = k_substep_count },
            BENCH_RAIN_SEED)) {
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

static bool fixture_body(bench_scene *scene, sl_shape *shape, float x, float y,
                         float mass)
{
    const sl_body_desc desc = { .position = { x, y },
                                .mass = mass,
                                .type = mass == 0.0f ? SL_BODY_STATIC
                                                     : SL_BODY_DYNAMIC,
                                .shape = shape,
                                .friction = 0.6f };
    return scene_body_add(scene, &desc);
}
static bool piles_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "piles",
                          (sl_world_config){ .body_capacity = 161u,
                                             .contact_capacity = 2048u,
                                             .joint_capacity = 0u,
                                             .gravity = { 0.0f, -9.81f },
                                             .substep_count = k_substep_count },
                          UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), box = sl_shape_none();
    if (!sl_shape_make_box(26.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box) ||
        !fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f)) {
        return false;
    }
    for (uint32_t pile = 0u; pile < 16u; ++pile) {
        for (uint32_t row = 0u; row < 10u; ++row) {
            if (!fixture_body(scene, &box, 3.0f * ((float)pile - 7.5f),
                              0.5f + (float)row, 1.0f)) {
                return false;
            }
        }
    }
    return true;
}
static bool chains_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "chains",
                          (sl_world_config){ .body_capacity = 104u,
                                             .contact_capacity = 1024u,
                                             .joint_capacity = 96u,
                                             .gravity = { 0.0f, -9.81f },
                                             .substep_count = k_substep_count },
                          UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape box = sl_shape_none();
    if (!sl_shape_make_box(0.25f, 0.4f, &box)) {
        return false;
    }
    for (uint32_t chain = 0u; chain < 8u; ++chain) {
        sl_body_handle previous = sl_body_handle_null();
        for (uint32_t row = 0u; row < 13u; ++row) {
            if (!fixture_body(scene, &box, 4.0f * (float)chain,
                              14.0f - (float)row, row == 0u ? 0.0f : 1.0f)) {
                return false;
            }
            const sl_body_handle body = scene->bodies[scene->body_count - 1u];
            if (row > 0u) {
                const sl_joint_desc joint = { .body_a = previous,
                                              .body_b = body,
                                              .kind = (row & 1u)
                                                          ? SL_JOINT_DISTANCE
                                                          : SL_JOINT_REVOLUTE,
                                              .local_anchor_a = { 0.0f, -0.4f },
                                              .local_anchor_b = { 0.0f, 0.6f },
                                              .distance = { .length = 0.005f },
                                              .collide_connected = false };
                if (sl_joint_handle_is_null(
                        sl_world_joint_create(&scene->world, &joint))) {
                    return false;
                }
            }
            previous = body;
        }
    }
    return true;
}
static bool churn_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "churn",
                          (sl_world_config){ .body_capacity = 128u,
                                             .contact_capacity = 2048u,
                                             .joint_capacity = 0u,
                                             .gravity = { 0.0f, 0.0f },
                                             .substep_count = k_substep_count },
                          UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_circle(0.25f, &circle)) {
        return false;
    }
    for (uint32_t i = 0u; i < 128u; ++i) {
        if (!fixture_body(scene, &circle, 0.6f * (float)(i % 16u),
                          0.6f * (float)(i / 16u), 1.0f)) {
            return false;
        }
    }
    return true;
}
static bool table_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "table",
                          (sl_world_config){ .body_capacity = 5u,
                                             .contact_capacity = 64u,
                                             .joint_capacity = 0u,
                                             .gravity = { 0.0f, -9.81f },
                                             .substep_count = k_substep_count },
                          UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), leg = sl_shape_none(),
             top = sl_shape_none(), load = sl_shape_none();
    return sl_shape_make_box(8.0f, 0.5f, &ground) &&
           sl_shape_make_box(0.4f, 1.0f, &leg) &&
           sl_shape_make_box(3.0f, 0.2f, &top) &&
           sl_shape_make_box(0.5f, 0.5f, &load) &&
           fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f) &&
           fixture_body(scene, &leg, -2.0f, 1.0f, 1.0f) &&
           fixture_body(scene, &leg, 2.0f, 1.0f, 1.0f) &&
           fixture_body(scene, &top, 0.0f, 2.2f, 2.0f) &&
           fixture_body(scene, &load, 0.0f, 2.9f, 5.0f);
}
static bool inverted_build(bench_scene *scene)
{
    if (!scene_world_init(scene, "inverted",
                          (sl_world_config){ .body_capacity = 7u,
                                             .contact_capacity = 128u,
                                             .joint_capacity = 0u,
                                             .gravity = { 0.0f, -9.81f },
                                             .substep_count = k_substep_count },
                          UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), box = sl_shape_none();
    if (!sl_shape_make_box(8.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box) ||
        !fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f)) {
        return false;
    }
    float mass = 0.25f;
    for (uint32_t row = 0u; row < 6u; ++row) {
        if (!fixture_body(scene, &box, 0.0f, 0.5f + (float)row, mass)) {
            return false;
        }
        mass *= 2.0f;
    }
    return true;
}
typedef struct bench_fixture {
    const char *name;
    bool (*build)(bench_scene *);
} bench_fixture;
static const bench_fixture k_fixtures[] = {
    { "pyramid", pyramid_build },  { "rain", rain_build },
    { "piles", piles_build },      { "chains", chains_build },
    { "churn", churn_build },      { "table", table_build },
    { "inverted", inverted_build }
};
#define BENCH_FIXTURE_COUNT                                                    \
    ((uint32_t)(sizeof(k_fixtures) / sizeof(k_fixtures[0])))
static bool scene_mutate(bench_scene *scene, uint32_t step)
{
    if (strcmp(scene->name, "churn") != 0) {
        return true;
    }
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_circle(0.25f, &circle)) {
        return false;
    }
    for (uint32_t j = 0u; j < 4u; ++j) {
        const uint32_t slot = (step * 4u + j) % 128u;
        sl_world_body_destroy(&scene->world, scene->bodies[slot]);
        const sl_body_desc desc = { .mass = 1.0f,
                                    .shape = &circle,
                                    .friction = 0.6f,
                                    .position = { 0.6f * (float)(slot % 16u),
                                                  0.6f *
                                                      (float)(slot / 16u) } };
        const sl_body_handle body = sl_world_body_create(&scene->world, &desc);
        if (sl_body_handle_is_null(body) || body.index != slot) {
            return false;
        }
        scene->bodies[slot] =
            body; /* immediate LIFO reuse preserves slot order */
        if (!sl_world_body_set_position(
                &scene->world, body,
                (sl_vec2){ desc.position.x + ((step & 1u) ? 0.15f : -0.15f),
                           desc.position.y })) {
            return false;
        }
    }
    return true;
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

static int sample_compare(const void *a, const void *b)
{
    const double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}
static void reference_capture(bench_scene *scene)
{
    for (uint32_t i = 0u; i < scene->body_count; ++i) {
        scene->reference[i] =
            sl_world_body_get_transform(&scene->world, scene->bodies[i]);
    }
}
static bool quality_sample(bench_scene *scene, bench_quality *q, bool window)
{
    const sl_world *world = &scene->world;
    double support_force = 0.0;
    for (uint32_t i = 0u; i < sl_world_contact_count(world); ++i) {
        const sl_contact *c = sl_world_contact_at(world, i);
        const sl_shape *a = sl_world_body_get_shape(world, c->body_a);
        const sl_shape *b = sl_world_body_get_shape(world, c->body_b);
        const float depth = sl_bench_penetration(
            a, sl_world_body_get_transform(world, c->body_a), b,
            sl_world_body_get_transform(world, c->body_b));
        q->penetration_max = sl_max(q->penetration_max, depth);
        for (uint32_t j = 0u; j < c->manifold.point_count; ++j) {
            const sl_manifold_point *point = &c->manifold.points[j];
            q->cached_penetration_max =
                sl_max(q->cached_penetration_max, -point->separation);
            if (sl_world_body_get_type(world, c->body_a) == SL_BODY_STATIC) {
                support_force +=
                    (double)(point->normal_impulse * c->manifold.normal.y) *
                    (double)k_substep_count / (double)k_timestep;
            } else if (sl_world_body_get_type(world, c->body_b) ==
                       SL_BODY_STATIC) {
                support_force -=
                    (double)(point->normal_impulse * c->manifold.normal.y) *
                    (double)k_substep_count / (double)k_timestep;
            }
        }
    }
    for (uint32_t i = 0u; i < sl_world_joint_count(world); ++i) {
        const sl_joint_handle handle = sl_world_joint_at(world, i);
        const sl_joint_desc d = sl_world_joint_get_desc(world, handle);
        const sl_vec2 a = sl_transform_apply(
            sl_world_body_get_transform(world, d.body_a), d.local_anchor_a);
        const sl_vec2 b = sl_transform_apply(
            sl_world_body_get_transform(world, d.body_b), d.local_anchor_b);
        float error = sl_vec2_length(sl_vec2_sub(a, b));
        if (d.kind == SL_JOINT_DISTANCE) {
            error = sl_abs(error - d.distance.length);
        }
        q->joint_error_max = sl_max(q->joint_error_max, error);
    }
    q->supported_weight = 0.0;
    for (uint32_t i = 0u; i < scene->body_count; ++i) {
        const sl_body_handle body = scene->bodies[i];
        const sl_transform t = sl_world_body_get_transform(world, body);
        const sl_vec2 v = sl_world_body_get_velocity(world, body);
        const float w = sl_world_body_get_angular_velocity(world, body);
        if (!sl_vec2_is_finite(t.position) || !sl_vec2_is_finite(v) ||
            !isfinite(w) || !isfinite(t.rotation.c) ||
            !isfinite(t.rotation.s)) {
            return false;
        }
        q->supported_weight += (double)sl_world_body_get_mass(world, body) *
                               -(double)scene->config.gravity.y;
        if (window) {
            q->translation_drift_max =
                sl_max(q->translation_drift_max,
                       sl_vec2_length(sl_vec2_sub(
                           t.position, scene->reference[i].position)));
            q->rotation_drift_max =
                sl_max(q->rotation_drift_max,
                       sl_abs(sl_angle_wrap(
                           sl_rotation_angle(t.rotation) -
                           sl_rotation_angle(scene->reference[i].rotation))));
            q->linear_speed_max =
                sl_max(q->linear_speed_max, sl_vec2_length(v));
            q->angular_speed_max = sl_max(q->angular_speed_max, sl_abs(w));
        }
    }
    if (window) {
        q->support_force_sum += support_force;
        q->window_steps += 1u;
    }
    return isfinite(q->support_force_sum) && isfinite(q->supported_weight) &&
           isfinite(q->penetration_max) && isfinite(q->joint_error_max);
}
static bool scene_run(bench_scene *scene, const bench_options *options,
                      bench_result *result)
{
    double *samples = malloc((size_t)options->steps * sizeof(*samples));
    if (samples == NULL) {
        return false;
    }
    bool valid = true;
    for (uint32_t i = 0u; i < options->warmup; ++i) {
        if (!scene_mutate(scene, i)) {
            valid = false;
            break;
        }
        sl_world_step(&scene->world, k_timestep);
        result->dropped_contact_count +=
            sl_world_contact_drop_count(&scene->world);
    }
    double total_ms = 0.0, mutation_ms = 0.0;
    const uint32_t window = options->steps < BENCH_QUALITY_WINDOW
                                ? options->steps
                                : BENCH_QUALITY_WINDOW;
    for (uint32_t i = 0u; valid && i < options->steps; ++i) {
        if (i == options->steps - window) {
            reference_capture(scene);
        }
        struct timespec start = { 0 }, end = { 0 };
        double elapsed = 0.0;
        if (strcmp(scene->name, "churn") == 0) {
            if (!timer_now(&start) ||
                !scene_mutate(scene, options->warmup + i) || !timer_now(&end) ||
                !timer_elapsed_ms(&start, &end, &elapsed)) {
                valid = false;
                break;
            }
            mutation_ms += elapsed;
        }
        if (!timer_now(&start)) {
            valid = false;
            break;
        }
        sl_world_step(&scene->world, k_timestep);
        if (!timer_now(&end) || !timer_elapsed_ms(&start, &end, &samples[i])) {
            valid = false;
            break;
        }
        total_ms += samples[i];
        result->dropped_contact_count +=
            sl_world_contact_drop_count(&scene->world);
        valid = quality_sample(scene, &result->quality,
                               i >= options->steps - window);
    }
    if (valid) {
        qsort(samples, options->steps, sizeof(*samples), sample_compare);
        result->average_ms = total_ms / (double)options->steps;
        const uint32_t mid = options->steps / 2u;
        result->median_ms = (options->steps & 1u) != 0u
                                ? samples[mid]
                                : 0.5 * (samples[mid - 1u] + samples[mid]);
        result->p95_ms = samples[(95u * options->steps + 99u) / 100u - 1u];
        result->maximum_ms = samples[options->steps - 1u];
        result->mutation_average_ms = mutation_ms / (double)options->steps;
        result->digest =
            sl_bench_digest(&scene->world, scene->bodies, scene->body_count);
        result->stats = sl_world_get_stats(&scene->world);
        valid =
            sl_world_memory_breakdown_get(&scene->config, &result->memory) &&
            isfinite(result->average_ms) &&
            isfinite(result->mutation_average_ms) &&
            result->dropped_contact_count == 0u;
    }
    free(samples);
    return valid;
}

static bool binary32_supported(void)
{
    return sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
           FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128;
}

static void json_string(const char *value)
{
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)value; *p != 0u; ++p) {
        if (*p == '"' || *p == '\\') {
            putchar('\\');
            putchar(*p);
        } else if (*p < 32u) {
            printf("\\u%04x", (unsigned int)*p);
        } else {
            putchar(*p);
        }
    }
    putchar('"');
}
static void work_print(const sl_world_work *w)
{
    printf("{\"tree_node_visits\":%" PRIu64 ",\"pair_candidates\":%" PRIu64
           ",\"pair_probes\":%" PRIu64 ",\"proxy_creates\":%" PRIu64
           ",\"proxy_destroys\":%" PRIu64 ",\"proxy_moves\":%" PRIu64
           ",\"contact_drops\":%" PRIu64 "}",
           w->tree_node_visits, w->pair_candidates, w->pair_probes,
           w->proxy_creates, w->proxy_destroys, w->proxy_moves,
           w->contact_drops);
}
static void result_json(const bench_scene *scene, const bench_result *r,
                        const bench_options *o)
{
    printf("{\"scene\":");
    json_string(scene->name);
    printf(",\"settings\":{\"fixture_version\":1,\"friction\":%.9g,"
           "\"restitution\":%.9g,\"seed\":%" PRIu32 ",\"warmup_steps\":%" PRIu32
           ",\"measured_steps\":%" PRIu32
           ",\"dt_seconds\":%.9g,\"substeps\":%" PRIu32
           ",\"body_capacity\":%" PRIu32 ",\"contact_capacity\":%" PRIu32
           ",\"joint_capacity\":%" PRIu32
           ",\"gravity\":[%.9g,%.9g],\"linear_drag\":0,\"angular_drag\":0,"
           "\"sleep_enabled\":false"
           ",\"linear_speed_max\":%.9g,\"contact_hertz\":%.9g,\"contact_"
           "damping_ratio\":%.9g"
           ",\"contact_push_velocity_max\":%.9g,\"restitution_threshold\":%.9g,"
           "\"joint_hertz\":%.9g,\"joint_damping_ratio\":%.9g}",
           strcmp(scene->name, "rain") == 0 ? (double)0.4f : (double)0.6f,
           strcmp(scene->name, "rain") == 0 ? (double)0.1f : 0.0, scene->seed,
           o->warmup, o->steps, (double)k_timestep, k_substep_count,
           r->stats.body_capacity, r->stats.contact_capacity,
           r->stats.joint_capacity, (double)scene->config.gravity.x,
           (double)scene->config.gravity.y, (double)SL_LINEAR_SPEED_MAX_DEFAULT,
           (double)SL_CONTACT_HERTZ_DEFAULT,
           (double)SL_CONTACT_DAMPING_RATIO_DEFAULT,
           (double)SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT,
           (double)SL_RESTITUTION_THRESHOLD_DEFAULT,
           (double)SL_JOINT_HERTZ_DEFAULT,
           (double)SL_JOINT_DAMPING_RATIO_DEFAULT);
    printf(",\"timing_ms\":{\"average\":%.9g,\"median\":%.9g,\"p95\":%.9g,"
           "\"max\":%.9g,\"mutation_average\":%.9g}",
           r->average_ms, r->median_ms, r->p95_ms, r->maximum_ms,
           r->mutation_average_ms);
    printf(",\"semantic_digest\":\"%016" PRIx64 "\",\"drops\":%" PRIu64,
           r->digest, r->dropped_contact_count);
    printf(",\"counts\":{\"bodies\":%" PRIu32 ",\"contacts\":%" PRIu32
           ",\"joints\":%" PRIu32 ",\"pairs\":%" PRIu32
           ",\"pair_capacity\":%" PRIu32 ",\"body_high\":%" PRIu32
           ",\"contact_high\":%" PRIu32 ",\"joint_high\":%" PRIu32 "}",
           r->stats.body_count, r->stats.contact_count, r->stats.joint_count,
           r->stats.pair_count, r->stats.pair_capacity,
           r->stats.body_count_high, r->stats.contact_count_high,
           r->stats.joint_count_high);
    printf(",\"step\":{\"dynamic_bodies\":%" PRIu32
           ",\"kinematic_bodies\":%" PRIu32 ",\"contact_constraints\":%" PRIu32
           ",\"joint_constraints\":%" PRIu32 ",\"substeps\":%" PRIu32
           ",\"work\":",
           r->stats.step.dynamic_body_count, r->stats.step.kinematic_body_count,
           r->stats.step.contact_constraint_count,
           r->stats.step.joint_constraint_count, r->stats.step.substep_count);
    work_print(&r->stats.step.work);
    printf("},\"cumulative_work\":");
    work_print(&r->stats.cumulative);
    const sl_world_memory_breakdown *m = &r->memory;
    printf(",\"memory_bytes\":{\"world_state\":%zu,\"body\":%zu,\"broadphase\":"
           "%zu,\"contact\":%zu,"
           "\"pair\":%zu,\"contact_solver\":%zu,\"joint\":%zu,\"padding\":%zu,"
           "\"arena\":%zu,\"world\":%zu}",
           m->world_state_bytes, m->body_bytes, m->broadphase_bytes,
           m->contact_bytes, m->pair_bytes, m->contact_solver_bytes,
           m->joint_bytes, m->padding_bytes, m->arena_bytes, m->world_bytes);
    const bench_quality *q = &r->quality;
    printf(",\"quality\":{\"window_steps\":%" PRIu32
           ",\"penetration_max\":%.9g,\"cached_penetration_max\":%.9g"
           ",\"translation_drift_max\":%.9g,\"rotation_drift_max\":%.9g,"
           "\"linear_speed_max\":%.9g,\"angular_speed_max\":%.9g"
           ",\"joint_error_max\":%.9g,\"support_force_mean\":%.9g,\"supported_"
           "weight\":%.9g}}",
           q->window_steps, (double)q->penetration_max,
           (double)q->cached_penetration_max, (double)q->translation_drift_max,
           (double)q->rotation_drift_max, (double)q->linear_speed_max,
           (double)q->angular_speed_max, (double)q->joint_error_max,
           q->support_force_sum / (double)q->window_steps, q->supported_weight);
}
static bool count_parse(const char *s, uint32_t *out, bool zero)
{
    uint32_t value = 0u;
    if (*s == '\0') {
        return false;
    }
    for (uint32_t i = 0u; s[i] != '\0'; ++i) {
        if (i >= 5u || s[i] < '0' || s[i] > '9') {
            return false;
        }
        value = value * 10u + (uint32_t)(s[i] - '0');
        if (value > BENCH_STEP_COUNT_MAX) {
            return false;
        }
    }
    if (!zero && value == 0u) {
        return false;
    }
    *out = value;
    return true;
}
static bool options_parse(int argc, char **argv, bench_options *o)
{
    *o = (bench_options){ .scene = "all",
                          .warmup = k_warmup_step_count,
                          .steps = k_measured_step_count };
    uint32_t seen = 0u;
    if (argc > 7 || (argc & 1) == 0) {
        return false;
    }
    for (int i = 1; i < argc; i += 2) {
        uint32_t bit = 0u;
        if (strcmp(argv[i], "--scene") == 0) {
            bit = 1u;
            bool found = strcmp(argv[i + 1], "all") == 0;
            for (uint32_t j = 0u; j < BENCH_FIXTURE_COUNT; ++j) {
                if (strcmp(argv[i + 1], k_fixtures[j].name) == 0) {
                    found = true;
                }
            }
            if (!found) {
                return false;
            }
            o->scene = argv[i + 1];
        } else if (strcmp(argv[i], "--warmup") == 0) {
            bit = 2u;
            if (!count_parse(argv[i + 1], &o->warmup, true)) {
                return false;
            }
        } else if (strcmp(argv[i], "--steps") == 0) {
            bit = 4u;
            if (!count_parse(argv[i + 1], &o->steps, false)) {
                return false;
            }
        } else {
            return false;
        }
        if ((seen & bit) != 0u) {
            return false;
        }
        seen |= bit;
    }
    return true;
}
int main(int argc, char **argv)
{
    bench_options options;
    if (!options_parse(argc, argv, &options) || !binary32_supported()) {
        fprintf(stderr, "usage: sl_bench [--scene "
                        "all|pyramid|rain|piles|chains|churn|table|inverted] "
                        "[--warmup 0..10000] [--steps 1..10000]\n");
        return EXIT_FAILURE;
    }
    printf("{\"schema_version\":2,\"metadata\":{\"compiler\":");
    json_string(SL_BENCH_COMPILER_ID);
    printf(",\"compiler_version\":");
    json_string(SL_BENCH_COMPILER_VERSION);
    printf(",\"build\":");
    json_string(SL_BENCH_BUILD_MODE);
    printf(",\"flags\":");
    json_string(SL_BENCH_C_FLAGS);
    printf(",\"warnings\":");
    json_string(SL_BENCH_WARNING_FLAGS);
    printf(",\"host\":");
    json_string(SL_BENCH_HOST_SYSTEM);
    printf(",\"host_version\":");
    json_string(SL_BENCH_HOST_VERSION);
    printf(",\"processor\":");
    json_string(SL_BENCH_HOST_PROCESSOR);
    printf(",\"revision\":");
    json_string(SL_BENCH_REVISION);
    printf("},\"results\":[");
    bool first = true;
    for (uint32_t i = 0u; i < BENCH_FIXTURE_COUNT; ++i) {
        if (strcmp(options.scene, "all") != 0 &&
            strcmp(options.scene, k_fixtures[i].name) != 0) {
            continue;
        }
        bench_scene *scene = calloc(1u, sizeof(*scene));
        bench_result result = { 0 };
        if (scene == NULL) {
            return EXIT_FAILURE;
        }
        const bool valid =
            k_fixtures[i].build(scene) && scene_run(scene, &options, &result);
        if (valid) {
            if (!first) {
                putchar(',');
            }
            result_json(scene, &result, &options);
        }
        sl_world_destroy(&scene->world);
        free(scene);
        if (!valid) {
            fprintf(
                stderr,
                "benchmark failed: %s (setup, timing, finite-state or drops)\n",
                k_fixtures[i].name);
            return EXIT_FAILURE;
        }
        first = false;
    }
    printf("]}\n");
    return EXIT_SUCCESS;
}
