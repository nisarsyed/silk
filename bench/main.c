/* Deterministic cloth benchmark. Each scene runs warm-up and measured
 * steps on a fresh world, reports wall-clock milliseconds per step (timing
 * lives here, never in silk), and an FNV-1a hash of the final positions
 * and velocities. Every scene runs twice; differing hashes are a
 * determinism failure and make the process exit nonzero.
 *
 * With --threads N (N > 1), each scene runs once more on an N-thread pool
 * (examples/pool); its hash must equal the serial one, since results may
 * not depend on the thread count.
 *
 * usage: sl_bench [--steps N] [--warmup N] [--scene NAME] [--threads N] */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pool.h"
#include "silk/mesh.h"
#include "silk/silk.h"
#include "silk/step.h"
#include "silk/world.h"

#define SB_DT (1.0f / 60.0f)

typedef struct sb_scene {
    const char *name;
    const char *about;
    uint32_t cells;
    bool (*build)(sl_world *world, uint32_t cells);
} sb_scene;

static sl_cloth_desc sb_cotton(void)
{
    sl_cloth_desc d;
    memset(&d, 0, sizeof(d));
    d.density = 0.2f;
    d.membrane_mu = 2.0e3f;
    d.membrane_lambda = 2.0e3f;
    d.bending_stiffness = 1.0e-4f;
    d.damping = 20.0f;
    d.thickness = 0.005f;
    d.friction = 0.5f;
    d.drag = 0.6f;
    return d;
}

/* The pool scenes run on, or NULL for the calling thread only. */
static sp_pool *sb_pool;

static void sb_parallel(sl_world_config *c)
{
    c->parallel_for = (sb_pool != NULL) ? sp_pool_parallel_for : NULL;
    c->parallel_context = sb_pool;
}

static bool sb_add_grid(sl_world *world, sl_vec3 origin, sl_vec3 u, sl_vec3 v,
                        uint32_t cells, uint32_t vertex_extra,
                        uint32_t triangle_extra)
{
    const uint32_t n = sl_mesh_grid_vertex_count(cells, cells);
    const uint32_t tn = sl_mesh_grid_triangle_count(cells, cells);
    sl_world_config c;
    memset(&c, 0, sizeof(c));
    sb_parallel(&c);
    c.vertex_capacity = n + vertex_extra;
    c.triangle_capacity = tn + triangle_extra;
    c.cloth_capacity = 1u;
    c.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    sl_vec3 *positions = malloc(sizeof(sl_vec3) * n);
    uint32_t *indices = malloc(sizeof(uint32_t) * 3u * tn);
    sl_cloth_desc d = sb_cotton();
    d.positions = positions;
    d.vertex_count = n;
    d.indices = indices;
    d.triangle_count = tn;
    uint32_t cloth = 0;
    const bool ok = positions != NULL && indices != NULL &&
                    sl_mesh_grid(origin, u, v, cells, cells, positions, n,
                                 indices, 3u * tn) &&
                    sl_world_init(world, &c) &&
                    sl_world_cloth_add(world, &d, &cloth);
    free(positions);
    free(indices);
    return ok;
}

/* 1 m curtain hung from its two top corners. */
static bool sb_build_curtain(sl_world *world, uint32_t cells)
{
    if (!sb_add_grid(world, sl_vec3_make(-0.5f, 1.5f, 0.0f),
                     sl_vec3_make(1.0f, 0.0f, 0.0f),
                     sl_vec3_make(0.0f, -1.0f, 0.0f), cells, 0u, 0u)) {
        return false;
    }
    return sl_world_vertex_pin_set(world, 0u, 0u, true) &&
           sl_world_vertex_pin_set(world, 0u, cells, true);
}

/* 1.2 m sheet dropped on a sphere above the ground. */
static bool sb_build_drape(sl_world *world, uint32_t cells)
{
    if (!sb_add_grid(world, sl_vec3_make(-0.6f, 1.0f, -0.6f),
                     sl_vec3_make(1.2f, 0.0f, 0.0f),
                     sl_vec3_make(0.0f, 0.0f, 1.2f), cells, 0u, 0u)) {
        return false;
    }
    const sl_collider sphere = { SL_COLLIDER_SPHERE,
                                 sl_vec3_make(0.0f, 0.5f, 0.0f), sl_vec3_zero(),
                                 0.3f, 0.5f };
    const sl_collider ground = { SL_COLLIDER_PLANE, sl_vec3_zero(),
                                 sl_vec3_make(0.0f, 1.0f, 0.0f), 0.0f, 0.5f };
    uint32_t id = 0;
    return sl_world_collider_add(world, &sphere, &id) &&
           sl_world_collider_add(world, &ground, &id);
}

/* 1.5 m x 1 m flag pinned along its pole edge in a 6 m/s wind. */
static bool sb_build_flag(sl_world *world, uint32_t cells)
{
    if (!sb_add_grid(world, sl_vec3_make(0.0f, 2.0f, 0.0f),
                     sl_vec3_make(1.5f, 0.0f, 0.0f),
                     sl_vec3_make(0.0f, -1.0f, 0.0f), cells, 0u, 0u)) {
        return false;
    }
    for (uint32_t j = 0; j <= cells; j++) {
        if (!sl_world_vertex_pin_set(world, 0u, j * (cells + 1u), true)) {
            return false;
        }
    }
    return sl_world_wind_set(world, sl_vec3_make(6.0f, 0.0f, 1.5f));
}

/* Three 1.2 m sheets, each 15 cm above and turned 30 degrees from the
 * last, dropped on a sphere: cloth-cloth contact (Offset Geometric
 * Contact). */
static bool sb_build_pile(sl_world *world, uint32_t cells)
{
    const uint32_t layers = 3u;
    const uint32_t n = sl_mesh_grid_vertex_count(cells, cells);
    const uint32_t tn = sl_mesh_grid_triangle_count(cells, cells);
    sl_world_config c;
    memset(&c, 0, sizeof(c));
    sb_parallel(&c);
    c.vertex_capacity = layers * n;
    c.triangle_capacity = layers * tn;
    c.cloth_capacity = layers;
    c.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    c.self_contact_capacity = 8u * layers * n;
    sl_vec3 *positions = malloc(sizeof(sl_vec3) * n);
    uint32_t *indices = malloc(sizeof(uint32_t) * 3u * tn);
    bool ok = positions != NULL && indices != NULL && sl_world_init(world, &c);
    for (uint32_t k = 0; ok && k < layers; k++) {
        const float angle = 0.52f * (float)k;
        const sl_vec3 u =
            sl_vec3_make(1.2f * cosf(angle), 0.0f, 1.2f * sinf(angle));
        const sl_vec3 v =
            sl_vec3_make(-1.2f * sinf(angle), 0.0f, 1.2f * cosf(angle));
        const sl_vec3 center =
            sl_vec3_make(0.0f, 1.1f + 0.15f * (float)k, 0.0f);
        const sl_vec3 origin =
            sl_vec3_sub(center, sl_vec3_scale(sl_vec3_add(u, v), 0.5f));
        sl_cloth_desc d = sb_cotton();
        d.thickness = 0.004f;
        d.positions = positions;
        d.vertex_count = n;
        d.indices = indices;
        d.triangle_count = tn;
        uint32_t cloth = 0;
        ok = sl_mesh_grid(origin, u, v, cells, cells, positions, n, indices,
                          3u * tn) &&
             sl_world_cloth_add(world, &d, &cloth);
    }
    free(positions);
    free(indices);
    const sl_collider sphere = { SL_COLLIDER_SPHERE,
                                 sl_vec3_make(0.0f, 0.55f, 0.0f),
                                 sl_vec3_zero(), 0.3f, 0.5f };
    uint32_t id = 0;
    return ok && sl_world_collider_add(world, &sphere, &id);
}

static const sb_scene k_scenes[] = {
    { "curtain-32", "1 m curtain, two corner pins, 32 x 32 cells", 32u,
      sb_build_curtain },
    { "curtain-64", "1 m curtain, two corner pins, 64 x 64 cells", 64u,
      sb_build_curtain },
    { "drape-64", "1.2 m sheet onto sphere and ground, 64 x 64 cells", 64u,
      sb_build_drape },
    { "flag-48", "1.5 x 1 m flag in 6 m/s wind, 48 x 48 cells", 48u,
      sb_build_flag },
    { "pile-30", "3 sheets dropped on a sphere, self contact, 30 x 30 cells",
      30u, sb_build_pile },
};

static double sb_seconds(void)
{
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC) {
        return 0.0;
    }
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static uint64_t sb_hash(const sl_world *world)
{
    uint64_t hash = 1469598103934665603u;
    const uint32_t cloths = sl_world_stats_get(world).cloth_count;
    for (uint32_t c = 0; c < cloths; c++) {
        const uint32_t n = sl_world_cloth_vertex_count(world, c);
        const sl_vec3 *arrays[2] = { sl_world_cloth_positions(world, c),
                                     sl_world_cloth_velocities(world, c) };
        for (uint32_t a = 0; a < 2u; a++) {
            const unsigned char *bytes = (const unsigned char *)arrays[a];
            for (size_t k = 0; k < sizeof(sl_vec3) * n; k++) {
                hash = (hash ^ bytes[k]) * 1099511628211u;
            }
        }
    }
    return hash;
}

/* Returns false when the scene fails to build. */
static bool sb_run(const sb_scene *scene, uint32_t warmup, uint32_t steps,
                   double *ms_per_step, uint64_t *hash, sl_world_stats *stats)
{
    sl_world world = { 0 };
    if (!scene->build(&world, scene->cells)) {
        sl_world_destroy(&world);
        return false;
    }
    for (uint32_t k = 0; k < warmup; k++) {
        sl_world_step(&world, SB_DT);
    }
    const double start = sb_seconds();
    for (uint32_t k = 0; k < steps; k++) {
        sl_world_step(&world, SB_DT);
    }
    const double elapsed = sb_seconds() - start;
    *ms_per_step = (steps > 0u) ? 1000.0 * elapsed / (double)steps : 0.0;
    *hash = sb_hash(&world);
    *stats = sl_world_stats_get(&world);
    sl_world_destroy(&world);
    return true;
}

static uint32_t sb_parse_count(const char *text, uint32_t fallback)
{
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value > 100000ul) {
        return fallback;
    }
    return (uint32_t)value;
}

int main(int argc, char **argv)
{
    uint32_t steps = 120u;
    uint32_t warmup = 30u;
    const char *only = NULL;
    uint32_t threads = 1u;
    for (int a = 1; a + 1 < argc; a += 2) {
        if (strcmp(argv[a], "--steps") == 0) {
            steps = sb_parse_count(argv[a + 1], steps);
        } else if (strcmp(argv[a], "--warmup") == 0) {
            warmup = sb_parse_count(argv[a + 1], warmup);
        } else if (strcmp(argv[a], "--scene") == 0) {
            only = argv[a + 1];
        } else if (strcmp(argv[a], "--threads") == 0) {
            threads = sb_parse_count(argv[a + 1], threads);
        }
    }
    const int version = sl_version();
    printf("silk %d.%d.%d bench: %u warm-up + %u measured steps of 1/60 s, "
           "%u iteration(s) per substep, substeps from cloth stiffness\n",
           (version >> 16) & 0xff, (version >> 8) & 0xff, version & 0xff,
           warmup, steps, SL_ITERATION_COUNT_DEFAULT);
    sp_pool *pool = (threads > 1u) ? sp_pool_create(threads) : NULL;
    if (threads > 1u) {
        if (pool == NULL) {
            printf("thread pool creation failed\n");
            return 1;
        }
        printf("threaded runs: %u thread(s)\n", sp_pool_thread_count(pool));
    }
    int status = 0;
    uint32_t ran = 0;
    for (size_t s = 0; s < sizeof(k_scenes) / sizeof(k_scenes[0]); s++) {
        const sb_scene *scene = &k_scenes[s];
        if (only != NULL && strcmp(only, scene->name) != 0) {
            continue;
        }
        double ms[2] = { 0.0, 0.0 };
        uint64_t hash[2] = { 0u, 0u };
        sl_world_stats stats;
        if (!sb_run(scene, warmup, steps, &ms[0], &hash[0], &stats) ||
            !sb_run(scene, warmup, steps, &ms[1], &hash[1], &stats)) {
            printf("%-12s BUILD FAILED\n", scene->name);
            status = 1;
            continue;
        }
        const uint32_t n = stats.vertex_count;
        const double best = (ms[0] < ms[1]) ? ms[0] : ms[1];
        const double sweeps =
            (double)stats.substep_count * (double)SL_ITERATION_COUNT_DEFAULT;
        printf("%-12s %6u verts %2u substeps %8.3f ms/step %6.1f "
               "ns/vertex-sweep contacts %5u hash %016llx%s%s  (%s)\n",
               scene->name, n, stats.substep_count, best,
               1e6 * best / ((double)n * sweeps), stats.contact_count,
               (unsigned long long)hash[0],
               (hash[0] == hash[1]) ? "" : " NONDETERMINISTIC",
               stats.substep_limited ? " SUBSTEPS-CAPPED" : "", scene->about);
        if (hash[0] != hash[1]) {
            status = 1;
        }
        if (pool != NULL) {
            double ms_threaded = 0.0;
            uint64_t hash_threaded = 0u;
            sb_pool = pool;
            const bool built = sb_run(scene, warmup, steps, &ms_threaded,
                                      &hash_threaded, &stats);
            sb_pool = NULL;
            printf("%-12s threaded %8.3f ms/step (%.2fx) hash %016llx%s\n", "",
                   ms_threaded, (ms_threaded > 0.0) ? best / ms_threaded : 0.0,
                   (unsigned long long)hash_threaded,
                   (built && hash_threaded == hash[0]) ? ""
                                                       : " THREAD-MISMATCH");
            if (!built || hash_threaded != hash[0]) {
                status = 1;
            }
        }
        ran++;
    }
    sp_pool_destroy(pool);
    if (ran == 0u) {
        printf("no scene matched\n");
        return 1;
    }
    return status;
}
