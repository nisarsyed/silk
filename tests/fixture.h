#ifndef SILK_TEST_FIXTURE_H
#define SILK_TEST_FIXTURE_H

/* Shared scene builders for the suites. Header-only; every helper is
 * static so each suite gets its own copy. */

#include <stdlib.h>
#include <string.h>

#include "silk/mesh.h"
#include "silk/step.h"
#include "silk/world.h"

/* xorshift32 (Marsaglia 2003); seeds are committed in each suite. */
static inline uint32_t sl_test_rand(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/* Uniform in [lo, hi]. */
static inline float sl_test_randf(uint32_t *state, float lo, float hi)
{
    const float unit = (float)(sl_test_rand(state) >> 8) / 16777215.0f;
    return lo + (hi - lo) * unit;
}

/* Light cotton defaults used across the suites. mu = 2e3 N/m keeps the
 * default 16 x 1 solver energy-stable at the 5 cm edges the suites use
 * (mu h^2 / (density edge^2) = 4.3; see test_solver.c). */
static inline sl_cloth_desc sl_test_cloth_desc(void)
{
    sl_cloth_desc d;
    memset(&d, 0, sizeof(d));
    d.density = 0.2f;
    d.membrane_mu = 2.0e3f;
    d.membrane_lambda = 2.0e3f;
    d.bending_stiffness = 1.0e-4f;
    d.damping = 0.0f;
    d.thickness = 0.005f;
    d.friction = 0.5f;
    d.drag = 0.0f;
    return d;
}

typedef struct sl_test_grid {
    sl_vec3 *positions;
    uint32_t *indices;
    uint32_t vertex_count;
    uint32_t triangle_count;
} sl_test_grid;

/* Allocates and fills a grid; free with sl_test_grid_free. */
static inline sl_test_grid sl_test_grid_make(sl_vec3 origin, sl_vec3 u,
                                             sl_vec3 v, uint32_t cells_u,
                                             uint32_t cells_v)
{
    sl_test_grid g;
    g.vertex_count = sl_mesh_grid_vertex_count(cells_u, cells_v);
    g.triangle_count = sl_mesh_grid_triangle_count(cells_u, cells_v);
    g.positions = malloc(sizeof(sl_vec3) * g.vertex_count);
    g.indices = malloc(sizeof(uint32_t) * 3u * g.triangle_count);
    if (g.positions == NULL || g.indices == NULL ||
        !sl_mesh_grid(origin, u, v, cells_u, cells_v, g.positions,
                      g.vertex_count, g.indices, 3u * g.triangle_count)) {
        free(g.positions);
        free(g.indices);
        memset(&g, 0, sizeof(g));
    }
    return g;
}

static inline void sl_test_grid_free(sl_test_grid *g)
{
    free(g->positions);
    free(g->indices);
    memset(g, 0, sizeof(*g));
}

static inline sl_world_config sl_test_config(uint32_t vertices,
                                             uint32_t triangles)
{
    sl_world_config c;
    memset(&c, 0, sizeof(c));
    c.vertex_capacity = vertices;
    c.triangle_capacity = triangles;
    c.cloth_capacity = 4u;
    c.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    return c;
}

/* A world holding one vertical sheet of side meters in the xy plane, top
 * edge at y = top, optionally pinned at both top corners. */
static inline bool sl_test_sheet(sl_world *world, const sl_world_config *c,
                                 const sl_cloth_desc *desc, float side,
                                 uint32_t cells, float top, bool pin_corners)
{
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.5f * side, top, 0.0f), sl_vec3_make(side, 0.0f, 0.0f),
        sl_vec3_make(0.0f, -side, 0.0f), cells, cells);
    sl_cloth_desc d = *desc;
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    const bool ok = g.positions != NULL && sl_world_init(world, c) &&
                    sl_world_cloth_add(world, &d, &cloth) && cloth == 0u;
    sl_test_grid_free(&g);
    if (ok && pin_corners) {
        (void)sl_world_vertex_pin_set(world, 0u, 0u, true);
        (void)sl_world_vertex_pin_set(world, 0u, cells, true);
    }
    return ok;
}

/* FNV-1a over the bit patterns of every position and velocity. */
static inline uint64_t sl_test_world_hash(const sl_world *world)
{
    uint64_t hash = 1469598103934665603u;
    const sl_world_stats stats = sl_world_stats_get(world);
    for (uint32_t c = 0; c < stats.cloth_count; c++) {
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

#endif /* SILK_TEST_FIXTURE_H */
