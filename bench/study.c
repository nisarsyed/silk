/* Public-only adversarial fixtures for the solver-order investigation.
 * Fixed geometry and events complement the unchanged full benchmark matrix. */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <silk/step.h>

#include "quality.h"
#include "sl_bench_config.h"

#define STUDY_BODY_MAX 16u
#define STUDY_WARMUP 120u
#define STUDY_STEPS 600u
#define STUDY_WINDOW 60u

static const float k_dt = 1.0f / 60.0f;

typedef struct study_scene {
    sl_world world;
    sl_body_handle bodies[STUDY_BODY_MAX];
    sl_transform reference[STUDY_BODY_MAX];
    sl_transform event_reference[STUDY_BODY_MAX];
    uint32_t count;
    uint32_t kind;
} study_scene;

static bool body_add(study_scene *scene, float x, float y, float hx, float hy,
                     float mass, float vx, float restitution)
{
    sl_shape shape = sl_shape_none();
    if (scene->count >= STUDY_BODY_MAX || !sl_shape_make_box(hx, hy, &shape)) {
        return false;
    }
    const sl_body_desc desc = { .position = { x, y },
                                .velocity = { vx, 0.0f },
                                .shape = &shape,
                                .mass = mass,
                                .type = mass == 0.0f ? SL_BODY_STATIC
                                                     : SL_BODY_DYNAMIC,
                                .friction = 0.6f,
                                .restitution = restitution };
    const sl_body_handle body = sl_world_body_create(&scene->world, &desc);
    if (sl_body_handle_is_null(body)) {
        return false;
    }
    scene->bodies[scene->count++] = body;
    return true;
}

static bool scene_build(study_scene *scene, uint32_t kind, bool sleeping)
{
    scene->kind = kind;
    const sl_world_config config = { .body_capacity = STUDY_BODY_MAX,
                                     .contact_capacity = 128u,
                                     .gravity = { 0.0f,
                                                  kind == 2u ? 0.0f : -9.81f },
                                     .substep_count = 4u,
                                     .sleep_enabled = sleeping };
    if (!sl_world_init(&scene->world, &config)) {
        return false;
    }
    if (kind == 2u) {
        /* Zero initial momentum, elastic head-on collision during warmup. */
        return body_add(scene, -2.0f, 0.0f, 0.5f, 0.5f, 1.0f, 3.0f, 1.0f) &&
               body_add(scene, 2.0f, 0.0f, 0.5f, 0.5f, 3.0f, -1.0f, 1.0f);
    }
    if (!body_add(scene, 0.0f, -6.5f, 16.0f, 0.5f, 0.0f, 0.0f, 0.0f)) {
        return false;
    }
    if (kind == 0u) {
        /* Wide beam transfers a centered load to two separated supports. */
        return body_add(scene, -2.0f, -1.5f, 0.4f, 1.5f, 0.0f, 0.0f, 0.0f) &&
               body_add(scene, 2.0f, -1.5f, 0.4f, 1.5f, 0.0f, 0.0f, 0.0f) &&
               body_add(scene, 0.0f, 0.2f, 2.5f, 0.2f, 2.0f, 0.0f, 0.0f) &&
               body_add(scene, 0.0f, 0.9f, 0.5f, 0.5f, 5.0f, 0.0f, 0.0f);
    }
    if (!body_add(scene, 0.0f, -0.5f, 4.0f, 0.5f, 0.0f, 0.0f, 0.0f)) {
        return false;
    }
    if (kind == 3u) {
        /* Equal-height horizontal edge is deliberately ambiguous for layering.
         */
        return body_add(scene, -0.499f, 0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f) &&
               body_add(scene, 0.499f, 0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f) &&
               body_add(scene, 0.0f, 1.2f, 1.0f, 0.2f, 2.0f, 0.0f, 0.0f);
    }
    for (uint32_t i = 0u; i < 6u; ++i) {
        if (!body_add(scene, 0.0f, 0.5f + (float)i, 0.5f, 0.5f, 1.0f, 0.0f,
                      0.0f)) {
            return false;
        }
    }
    return true;
}

static void json_string(const char *text)
{
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)text; *p != 0u; ++p) {
        if (*p == '"' || *p == '\\') {
            putchar('\\');
            putchar(*p);
        } else if (*p < 32u) {
            printf("\\u%04x", (unsigned)*p);
        } else {
            putchar(*p);
        }
    }
    putchar('"');
}

static bool scene_run(study_scene *scene)
{
    double time_ms = 0.0, penetration = 0.0, travel = 0.0, speed = 0.0;
    double spin = 0.0, rotation = 0.0, support = 0.0, momentum_error = 0.0;
    double drop = 0.0, displacement = 0.0, excursion = 0.0;
    double samples[10] = { 0 };
    uint64_t constraints = 0u;
    for (uint32_t step = 0u; step < STUDY_WARMUP + STUDY_STEPS; ++step) {
        if (step == STUDY_WARMUP) {
            for (uint32_t i = 0u; i < scene->count; ++i) {
                scene->event_reference[i] = sl_world_body_get_transform(
                    &scene->world, scene->bodies[i]);
            }
        }
        /* Events occur at the first measured step, after settling warmup. */
        if (step == STUDY_WARMUP && scene->kind == 1u) {
            sl_world_body_destroy(&scene->world, scene->bodies[1]);
        }
        if (step == STUDY_WARMUP && scene->kind == 4u &&
            !sl_world_body_set_velocity(&scene->world,
                                        scene->bodies[scene->count - 1u],
                                        (sl_vec2){ 3.0f, 0.0f })) {
            return false;
        }
        if (step == STUDY_WARMUP + STUDY_STEPS - STUDY_WINDOW) {
            for (uint32_t i = 0u; i < scene->count; ++i) {
                if (sl_world_body_is_valid(&scene->world, scene->bodies[i])) {
                    scene->reference[i] = sl_world_body_get_transform(
                        &scene->world, scene->bodies[i]);
                }
            }
        }
        struct timespec a, b;
        if (timespec_get(&a, TIME_UTC) != TIME_UTC) {
            return false;
        }
        sl_world_step(&scene->world, k_dt);
        if (timespec_get(&b, TIME_UTC) != TIME_UTC || a.tv_nsec < 0L ||
            a.tv_nsec >= 1000000000L || b.tv_nsec < 0L ||
            b.tv_nsec >= 1000000000L) {
            return false;
        }
        const double elapsed = difftime(b.tv_sec, a.tv_sec) * 1000.0 +
                               ((double)b.tv_nsec - (double)a.tv_nsec) * 1e-6;
        if (!isfinite(elapsed) || elapsed < 0.0 ||
            sl_world_contact_drop_count(&scene->world) != 0u) {
            return false;
        }
        const bool measured = step >= STUDY_WARMUP;
        const bool window = step >= STUDY_WARMUP + STUDY_STEPS - STUDY_WINDOW;
        if (measured) {
            time_ms += elapsed;
            constraints +=
                sl_world_get_stats(&scene->world).step.contact_constraint_count;
        }
        double px = 0.0, py = 0.0, frame_penetration = 0.0;
        for (uint32_t i = 0u; i < scene->count; ++i) {
            const sl_body_handle body = scene->bodies[i];
            if (!sl_world_body_is_valid(&scene->world, body)) {
                continue;
            }
            const sl_transform t =
                sl_world_body_get_transform(&scene->world, body);
            const sl_vec2 v = sl_world_body_get_velocity(&scene->world, body);
            const float w =
                sl_world_body_get_angular_velocity(&scene->world, body);
            if (!sl_vec2_is_finite(t.position) || !sl_vec2_is_finite(v) ||
                !isfinite(t.rotation.c) || !isfinite(t.rotation.s) ||
                !isfinite(w)) {
                return false;
            }
            const double mass =
                (double)sl_world_body_get_mass(&scene->world, body);
            if (measured && mass > 0.0) {
                const sl_transform initial = scene->event_reference[i];
                drop = fmax(drop,
                            (double)initial.position.y - (double)t.position.y);
                displacement =
                    fmax(displacement, fabs((double)t.position.x -
                                            (double)initial.position.x));
                excursion = fmax(
                    excursion,
                    fabs(atan2(
                        (double)t.rotation.s * (double)initial.rotation.c -
                            (double)t.rotation.c * (double)initial.rotation.s,
                        (double)t.rotation.c * (double)initial.rotation.c +
                            (double)t.rotation.s *
                                (double)initial.rotation.s)));
            }
            px += mass * (double)v.x;
            py += mass * (double)v.y;
            if (window) {
                speed = fmax(speed, hypot((double)v.x, (double)v.y));
                spin = fmax(spin, fabs((double)w));
                const sl_transform before = scene->reference[i];
                travel = fmax(
                    travel,
                    hypot((double)t.position.x - (double)before.position.x,
                          (double)t.position.y - (double)before.position.y));
                rotation = fmax(
                    rotation,
                    fabs(atan2(
                        (double)t.rotation.s * (double)before.rotation.c -
                            (double)t.rotation.c * (double)before.rotation.s,
                        (double)t.rotation.c * (double)before.rotation.c +
                            (double)t.rotation.s * (double)before.rotation.s)));
            }
            for (uint32_t j = 0u; j < i; ++j) {
                const sl_body_handle other = scene->bodies[j];
                if (!sl_world_body_is_valid(&scene->world, other) ||
                    (mass == 0.0 &&
                     sl_world_body_get_mass(&scene->world, other) == 0.0f)) {
                    continue;
                }
                frame_penetration = fmax(
                    frame_penetration,
                    (double)sl_bench_penetration(
                        sl_world_body_get_shape(&scene->world, body), t,
                        sl_world_body_get_shape(&scene->world, other),
                        sl_world_body_get_transform(&scene->world, other)));
            }
        }
        if (scene->kind == 2u) {
            momentum_error = fmax(momentum_error, hypot(px, py));
        }
        if (measured) {
            penetration = fmax(penetration, frame_penetration);
            if ((step - STUDY_WARMUP + 1u) % STUDY_WINDOW == 0u) {
                samples[(step - STUDY_WARMUP) / STUDY_WINDOW] =
                    frame_penetration;
            }
        }
        if (window) {
            for (uint32_t i = 0u; i < sl_world_contact_count(&scene->world);
                 ++i) {
                const sl_contact *c = sl_world_contact_at(&scene->world, i);
                const bool a_static =
                    sl_world_body_get_type(&scene->world, c->body_a) ==
                    SL_BODY_STATIC;
                const bool b_static =
                    sl_world_body_get_type(&scene->world, c->body_b) ==
                    SL_BODY_STATIC;
                if (a_static == b_static) {
                    continue;
                }
                for (uint32_t j = 0u; j < c->manifold.point_count; ++j) {
                    support += (double)c->manifold.points[j].normal_impulse *
                               (double)c->manifold.normal.y *
                               (a_static ? 1.0 : -1.0) * 4.0 / (double)k_dt /
                               (double)STUDY_WINDOW;
                }
            }
        }
    }
    sl_body_handle live[STUDY_BODY_MAX];
    uint32_t count = 0u;
    for (uint32_t i = 0u; i < scene->count; ++i) {
        if (sl_world_body_is_valid(&scene->world, scene->bodies[i])) {
            live[count++] = scene->bodies[i];
        }
    }
    const sl_world_stats stats = sl_world_get_stats(&scene->world);
    printf("\"drop_max\":%.9g,\"horizontal_motion_max\":%.9g,\"rotation_"
           "excursion_max\":%.9g,",
           drop, displacement, excursion);
    printf("\"average_ms\":%.9g,\"digest\":\"%016" PRIx64 "\","
           "\"penetration_max\":%.9g,\"translation_drift_max\":%.9g,"
           "\"rotation_drift_max\":%.9g,\"linear_speed_max\":%.9g,"
           "\"angular_speed_max\":%.9g,\"cached_support_force_mean\":%.9g,"
           "\"momentum_error_max\":%.9g,\"prepared_contacts\":%" PRIu64 ","
           "\"sleeping_dynamics\":%" PRIu32 ",\"penetration_samples\":[",
           time_ms / (double)STUDY_STEPS,
           sl_bench_digest(&scene->world, live, count), penetration, travel,
           rotation, speed, spin, support, momentum_error, constraints,
           stats.sleeping_dynamic_count);
    for (uint32_t i = 0u; i < 10u; ++i) {
        printf("%s%.9g", i == 0u ? "" : ",", samples[i]);
    }
    printf("]}");
    return true;
}

int main(int argc, char **argv)
{
    const bool sleeping = argc == 3 && strcmp(argv[1], "--sleep") == 0 &&
                          strcmp(argv[2], "on") == 0;
    if (argc != 1 && !(argc == 3 && strcmp(argv[1], "--sleep") == 0 &&
                       (sleeping || strcmp(argv[2], "off") == 0))) {
        return 1;
    }
    const char *names[] = { "bridge", "collapse", "zero-gravity", "cycle",
                            "topple" };
    printf("{\"study_version\":1,\"revision\":");
    json_string(SL_BENCH_REVISION);
    printf(",\"compiler\":");
    json_string(SL_BENCH_COMPILER_VERSION);
    printf(",\"compiler_id\":");
    json_string(SL_BENCH_COMPILER_ID);
    printf(",\"host\":");
    json_string(SL_BENCH_HOST_SYSTEM);
    printf(",\"host_version\":");
    json_string(SL_BENCH_HOST_VERSION);
    printf(",\"processor\":");
    json_string(SL_BENCH_HOST_PROCESSOR);
    printf(",\"flags\":");
    json_string(SL_BENCH_C_FLAGS);
    printf(",\"body_capacity\":16,\"contact_capacity\":128,\"seed\":0,"
           "\"quality_window\":60,\"sleep_enabled\":%s,\"warmup\":120,"
           "\"steps\":600,\"substeps\":4,"
           "\"dt\":%.9g,\"results\":[",
           sleeping ? "true" : "false", (double)k_dt);
    for (uint32_t i = 0u; i < 5u; ++i) {
        study_scene scene = { 0 };
        printf("%s{\"scene\":\"%s\",", i == 0u ? "" : ",", names[i]);
        const bool valid =
            scene_build(&scene, i, sleeping) && scene_run(&scene);
        sl_world_destroy(&scene.world);
        if (!valid) {
            return 1;
        }
    }
    printf("]}\n");
    return 0;
}
