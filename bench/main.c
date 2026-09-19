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

#include "fixtures.h"
#include "quality.h"
#include "sl_bench_config.h"

static const uint32_t k_warmup_step_count = 120u;
static const uint32_t k_measured_step_count = 600u;

typedef struct bench_options {
    bool sleep_enabled;
    const char *scene;
    uint32_t warmup;
    uint32_t steps;
} bench_options;
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
    sl_bench_quality quality;
} bench_result;

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
static bool scene_run(sl_bench_scene *scene, const bench_options *options,
                      bench_result *result)
{
    double *samples = malloc((size_t)options->steps * sizeof(*samples));
    if (samples == NULL) {
        return false;
    }
    bool valid = true;
    for (uint32_t i = 0u; i < options->warmup; ++i) {
        if (!sl_bench_scene_mutate(scene, i)) {
            valid = false;
            break;
        }
        sl_world_step(scene->world, SL_BENCH_TIMESTEP);
        result->dropped_contact_count +=
            sl_world_contact_drop_count(scene->world);
    }
    double total_ms = 0.0, mutation_ms = 0.0;
    const uint32_t window = options->steps < SL_BENCH_QUALITY_WINDOW
                                ? options->steps
                                : SL_BENCH_QUALITY_WINDOW;
    for (uint32_t i = 0u; valid && i < options->steps; ++i) {
        if (i == options->steps - window) {
            sl_bench_reference_capture(scene);
        }
        struct timespec start = { 0 }, end = { 0 };
        double elapsed = 0.0;
        if (strcmp(scene->name, "churn") == 0) {
            if (!timer_now(&start) ||
                !sl_bench_scene_mutate(scene, options->warmup + i) ||
                !timer_now(&end) || !timer_elapsed_ms(&start, &end, &elapsed)) {
                valid = false;
                break;
            }
            mutation_ms += elapsed;
        }
        if (!timer_now(&start)) {
            valid = false;
            break;
        }
        sl_world_step(scene->world, SL_BENCH_TIMESTEP);
        if (!timer_now(&end) || !timer_elapsed_ms(&start, &end, &samples[i])) {
            valid = false;
            break;
        }
        total_ms += samples[i];
        result->dropped_contact_count +=
            sl_world_contact_drop_count(scene->world);
        valid = sl_bench_quality_sample(scene, &result->quality,
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
            sl_bench_digest(scene->world, scene->bodies, scene->body_count);
        result->stats = sl_world_get_stats(scene->world);
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
    printf(
        "{\"tree_node_visits\":%" PRIu64 ",\"pair_candidates\":%" PRIu64
        ",\"pair_probes\":%" PRIu64 ",\"proxy_creates\":%" PRIu64
        ",\"proxy_destroys\":%" PRIu64 ",\"proxy_moves\":%" PRIu64
        ",\"wake_visits\":%" PRIu64 ",\"body_wakes\":%" PRIu64
        ",\"body_sleeps\":%" PRIu64 ",\"graph_body_visits\":%" PRIu64
        ",\"graph_constraint_visits\":%" PRIu64
        ",\"graph_parent_probes\":%" PRIu64 ",\"contact_drops\":%" PRIu64 "}",
        w->tree_node_visits, w->pair_candidates, w->pair_probes,
        w->proxy_creates, w->proxy_destroys, w->proxy_moves, w->wake_visits,
        w->body_wakes, w->body_sleeps, w->graph_body_visits,
        w->graph_constraint_visits, w->graph_parent_probes, w->contact_drops);
}
static void result_json(const sl_bench_scene *scene, const bench_result *r,
                        const bench_options *o)
{
    printf("{\"scene\":");
    json_string(scene->name);
    printf(
        ",\"settings\":{\"fixture_version\":1,\"friction\":%.9g,"
        "\"restitution\":%.9g,\"seed\":%" PRIu32 ",\"warmup_steps\":%" PRIu32
        ",\"measured_steps\":%" PRIu32
        ",\"dt_seconds\":%.9g,\"substeps\":%" PRIu32
        ",\"body_capacity\":%" PRIu32 ",\"contact_capacity\":%" PRIu32
        ",\"joint_capacity\":%" PRIu32
        ",\"gravity\":[%.9g,%.9g],\"linear_drag\":0,\"angular_drag\":0,"
        "\"sleep_enabled\":%s,\"sleep_speed_max\":%.9g,\"sleep_angular_speed_"
        "max\":%.9g,\"sleep_time_min\":%.9g"
        ",\"linear_speed_max\":%.9g,\"contact_hertz\":%.9g,\"contact_"
        "damping_ratio\":%.9g"
        ",\"contact_push_velocity_max\":%.9g,\"restitution_threshold\":%.9g,"
        "\"joint_hertz\":%.9g,\"joint_damping_ratio\":%.9g}",
        strcmp(scene->name, "rain") == 0 ? (double)0.4f : (double)0.6f,
        strcmp(scene->name, "rain") == 0 ? (double)0.1f : 0.0, scene->seed,
        o->warmup, o->steps, (double)SL_BENCH_TIMESTEP, SL_BENCH_SUBSTEP_COUNT,
        r->stats.body_capacity, r->stats.contact_capacity,
        r->stats.joint_capacity, (double)scene->config.gravity.x,
        (double)scene->config.gravity.y, o->sleep_enabled ? "true" : "false",
        (double)SL_SLEEP_SPEED_MAX_DEFAULT,
        (double)SL_SLEEP_ANGULAR_SPEED_MAX_DEFAULT,
        (double)SL_SLEEP_TIME_MIN_DEFAULT, (double)SL_LINEAR_SPEED_MAX_DEFAULT,
        (double)SL_CONTACT_HERTZ_DEFAULT,
        (double)SL_CONTACT_DAMPING_RATIO_DEFAULT,
        (double)SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT,
        (double)SL_RESTITUTION_THRESHOLD_DEFAULT,
        (double)SL_JOINT_HERTZ_DEFAULT, (double)SL_JOINT_DAMPING_RATIO_DEFAULT);
    printf(",\"timing_ms\":{\"average\":%.9g,\"median\":%.9g,\"p95\":%.9g,"
           "\"max\":%.9g,\"mutation_average\":%.9g}",
           r->average_ms, r->median_ms, r->p95_ms, r->maximum_ms,
           r->mutation_average_ms);
    printf(",\"semantic_digest\":\"%016" PRIx64 "\",\"drops\":%" PRIu64,
           r->digest, r->dropped_contact_count);
    printf(",\"counts\":{\"awake_dynamics\":%" PRIu32
           ",\"sleeping_dynamics\":%" PRIu32 ",\"bodies\":%" PRIu32
           ",\"contacts\":%" PRIu32 ",\"joints\":%" PRIu32 ",\"pairs\":%" PRIu32
           ",\"pair_capacity\":%" PRIu32 ",\"body_high\":%" PRIu32
           ",\"contact_high\":%" PRIu32 ",\"joint_high\":%" PRIu32 "}",
           r->stats.awake_dynamic_count, r->stats.sleeping_dynamic_count,
           r->stats.body_count, r->stats.contact_count, r->stats.joint_count,
           r->stats.pair_count, r->stats.pair_capacity,
           r->stats.body_count_high, r->stats.contact_count_high,
           r->stats.joint_count_high);
    printf(",\"step\":{\"dynamic_bodies\":%" PRIu32
           ",\"kinematic_bodies\":%" PRIu32 ",\"contact_constraints\":%" PRIu32
           ",\"joint_constraints\":%" PRIu32 ",\"substeps\":%" PRIu32
           ",\"islands\":%" PRIu32 ",\"island_bodies_max\":%" PRIu32
           ",\"islands_executed\":%" PRIu32 ",\"islands_skipped\":%" PRIu32
           ",\"work\":",
           r->stats.step.dynamic_body_count, r->stats.step.kinematic_body_count,
           r->stats.step.contact_constraint_count,
           r->stats.step.joint_constraint_count, r->stats.step.substep_count,
           r->stats.step.island_count, r->stats.step.island_body_count_max,
           r->stats.step.island_executed_count,
           r->stats.step.island_skipped_count);
    work_print(&r->stats.step.work);
    printf("},\"cumulative_work\":");
    work_print(&r->stats.cumulative);
    const sl_world_memory_breakdown *m = &r->memory;
    printf(",\"memory_bytes\":{\"sleep\":%zu,\"island\":%zu,\"world_state\":%"
           "zu,\"body\":%zu,"
           "\"broadphase\":"
           "%zu,\"contact\":%zu,"
           "\"pair\":%zu,\"contact_solver\":%zu,\"joint\":%zu,\"padding\":%zu,"
           "\"arena\":%zu,\"world\":%zu}",
           m->sleep_bytes, m->island_bytes, m->world_state_bytes, m->body_bytes,
           m->broadphase_bytes, m->contact_bytes, m->pair_bytes,
           m->contact_solver_bytes, m->joint_bytes, m->padding_bytes,
           m->arena_bytes, m->world_bytes);
    const sl_bench_quality *q = &r->quality;
    printf(",\"quality\":{\"window_steps\":%" PRIu32
           ",\"penetration_max\":%.9g,\"cached_penetration_max\":%.9g"
           ",\"translation_drift_max\":%.9g,\"rotation_drift_max\":%.9g,"
           "\"linear_speed_max\":%.9g,\"angular_speed_max\":%.9g"
           ",\"joint_error_max\":%.9g,\"cached_support_force_mean\":%.9g,"
           "\"supported_"
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
        if (value > SL_BENCH_STEP_COUNT_MAX) {
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
    if (argc > 9 || (argc & 1) == 0) {
        return false;
    }
    for (int i = 1; i < argc; i += 2) {
        uint32_t bit = 0u;
        if (strcmp(argv[i], "--sleep") == 0) {
            bit = 8u;
            if (strcmp(argv[i + 1], "on") == 0) {
                o->sleep_enabled = true;
            } else if (strcmp(argv[i + 1], "off") == 0) {
                o->sleep_enabled = false;
            } else {
                return false;
            }
        } else if (strcmp(argv[i], "--scene") == 0) {
            bit = 1u;
            bool found = strcmp(argv[i + 1], "all") == 0;
            for (uint32_t j = 0u; j < SL_BENCH_FIXTURE_COUNT; ++j) {
                if (strcmp(argv[i + 1], sl_bench_fixture_name(j)) == 0) {
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
        fprintf(stderr,
                "usage: sl_bench [--scene "
                "all|pyramid|rain|piles|chains|churn|table|inverted] "
                "[--warmup 0..10000] [--steps 1..10000] [--sleep off|on]\n");
        return EXIT_FAILURE;
    }
    printf("{\"schema_version\":4,\"metadata\":{\"compiler\":");
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
    for (uint32_t i = 0u; i < SL_BENCH_FIXTURE_COUNT; ++i) {
        if (strcmp(options.scene, "all") != 0 &&
            strcmp(options.scene, sl_bench_fixture_name(i)) != 0) {
            continue;
        }
        sl_bench_scene *scene = calloc(1u, sizeof(*scene));
        sl_world world = { 0 };
        bench_result result = { 0 };
        if (scene == NULL) {
            return EXIT_FAILURE;
        }
        const bool valid =
            sl_bench_scene_build(scene, &world, i, options.sleep_enabled) &&
            scene_run(scene, &options, &result);
        if (valid) {
            if (!first) {
                putchar(',');
            }
            result_json(scene, &result, &options);
        }
        sl_world_destroy(&world);
        free(scene);
        if (!valid) {
            fprintf(
                stderr,
                "benchmark failed: %s (setup, timing, finite-state or drops)\n",
                sl_bench_fixture_name(i));
            return EXIT_FAILURE;
        }
        first = false;
    }
    printf("]}\n");
    return EXIT_SUCCESS;
}
