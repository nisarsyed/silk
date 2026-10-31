/* Minimal headless silk program: a 1 m flag in the wind for two seconds.
 * Uses only the installed public headers; prints the free corner. */
#include <stdio.h>
#include <string.h>

#include <silk/mesh.h>
#include <silk/silk.h>
#include <silk/step.h>
#include <silk/world.h>

#define CELLS 16u

int main(void)
{
    sl_vec3 positions[(CELLS + 1u) * (CELLS + 1u)];
    uint32_t indices[3u * 2u * CELLS * CELLS];
    const uint32_t n = sl_mesh_grid_vertex_count(CELLS, CELLS);
    const uint32_t tn = sl_mesh_grid_triangle_count(CELLS, CELLS);
    if (!sl_mesh_grid(sl_vec3_make(0.0f, 2.0f, 0.0f),
                      sl_vec3_make(1.0f, 0.0f, 0.0f),
                      sl_vec3_make(0.0f, -1.0f, 0.0f), CELLS, CELLS, positions,
                      n, indices, 3u * tn)) {
        return 1;
    }

    sl_world_config config;
    memset(&config, 0, sizeof(config));
    config.vertex_capacity = n;
    config.triangle_capacity = tn;
    config.cloth_capacity = 1u;
    config.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    config.wind = sl_vec3_make(5.0f, 0.0f, 1.0f);
    sl_world world = { 0 };
    if (!sl_world_init(&world, &config)) {
        return 1;
    }

    sl_cloth_desc cloth;
    memset(&cloth, 0, sizeof(cloth));
    cloth.positions = positions;
    cloth.vertex_count = n;
    cloth.indices = indices;
    cloth.triangle_count = tn;
    cloth.density = 0.2f;        /* kg/m^2 */
    cloth.membrane_mu = 2000.0f; /* N/m */
    cloth.membrane_lambda = 2000.0f;
    cloth.bending_stiffness = 1e-4f; /* N*m */
    cloth.damping = 20.0f;           /* 1/s */
    cloth.thickness = 0.005f;        /* m */
    cloth.drag = 0.6f;               /* kg/m^3 */
    uint32_t id = 0;
    bool ok = sl_world_cloth_add(&world, &cloth, &id);
    for (uint32_t j = 0; ok && j <= CELLS; j++) {
        ok = sl_world_vertex_pin_set(&world, id, j * (CELLS + 1u), true);
    }

    sl_stepper stepper;
    ok = ok && sl_stepper_init(&stepper, 1.0f / 60.0f);
    for (uint32_t frame = 0; ok && frame < 120u; frame++) {
        (void)sl_world_advance(&world, &stepper, 1.0f / 60.0f);
    }
    if (ok) {
        const sl_vec3 corner = sl_world_cloth_positions(&world, id)[CELLS];
        const sl_world_stats stats = sl_world_stats_get(&world);
        const int version = sl_version();
        printf("silk %d.%d.%d: free corner at (%.3f, %.3f, %.3f) m after "
               "2 s, %u substeps per step\n",
               (version >> 16) & 0xff, (version >> 8) & 0xff, version & 0xff,
               (double)corner.x, (double)corner.y, (double)corner.z,
               stats.substep_count);
    }
    sl_world_destroy(&world);
    return ok ? 0 : 1;
}
