/* Verify the extern-C boundary from C++; this is not a C++ engine wrapper. */
#include <silk/assert.h>
#include <silk/math.h>
#include <silk/mesh.h>
#include <silk/silk.h>
#include <silk/step.h>
#include <silk/world.h>

int main()
{
    sl_vec3 positions[4] = {};
    uint32_t indices[6] = {};
    if (!sl_mesh_grid(sl_vec3_make(0.0f, 1.0f, 0.0f),
                      sl_vec3_make(1.0f, 0.0f, 0.0f),
                      sl_vec3_make(0.0f, -1.0f, 0.0f), 1u, 1u, positions, 4u,
                      indices, 6u)) {
        return 1;
    }
    sl_world_config config = {};
    config.vertex_capacity = 4u;
    config.triangle_capacity = 2u;
    config.cloth_capacity = 1u;
    config.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    sl_world world = {};
    if (sl_version() != ((SL_VERSION_MAJOR << 16) | (SL_VERSION_MINOR << 8) |
                         SL_VERSION_PATCH) ||
        !sl_world_init(&world, &config)) {
        return 1;
    }
    sl_cloth_desc cloth = {};
    cloth.positions = positions;
    cloth.vertex_count = 4u;
    cloth.indices = indices;
    cloth.triangle_count = 2u;
    cloth.density = 0.2f;
    cloth.membrane_mu = 1000.0f;
    cloth.membrane_lambda = 1000.0f;
    cloth.thickness = 0.005f;
    uint32_t id = 0;
    const bool added = sl_world_cloth_add(&world, &cloth, &id);
    sl_world_step(&world, 1.0f / 60.0f);
    const bool valid =
        added && sl_world_cloth_positions(&world, id)[3].y < 0.0f;
    sl_world_destroy(&world);
    return valid ? 0 : 1;
}
