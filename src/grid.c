#include "silk/mesh.h"

static bool sl_grid_side_valid(uint32_t cells)
{
    return cells >= 1u && cells <= SL_MESH_GRID_CELL_COUNT_MAX;
}

uint32_t sl_mesh_grid_vertex_count(uint32_t cells_u, uint32_t cells_v)
{
    if (!sl_grid_side_valid(cells_u) || !sl_grid_side_valid(cells_v)) {
        return 0u;
    }
    return (cells_u + 1u) * (cells_v + 1u);
}

uint32_t sl_mesh_grid_triangle_count(uint32_t cells_u, uint32_t cells_v)
{
    if (!sl_grid_side_valid(cells_u) || !sl_grid_side_valid(cells_v)) {
        return 0u;
    }
    return 2u * cells_u * cells_v;
}

bool sl_mesh_grid(sl_vec3 origin, sl_vec3 extent_u, sl_vec3 extent_v,
                  uint32_t cells_u, uint32_t cells_v, sl_vec3 *positions,
                  uint32_t position_count, uint32_t *indices,
                  uint32_t index_count)
{
    const uint32_t vertex_count = sl_mesh_grid_vertex_count(cells_u, cells_v);
    const uint32_t triangle_count =
        sl_mesh_grid_triangle_count(cells_u, cells_v);
    if (vertex_count == 0u || positions == NULL || indices == NULL ||
        position_count != vertex_count || index_count != 3u * triangle_count ||
        !sl_vec3_is_finite(origin) || !sl_vec3_is_finite(extent_u) ||
        !sl_vec3_is_finite(extent_v)) {
        return false;
    }
    const uint32_t row = cells_u + 1u;
    for (uint32_t j = 0; j <= cells_v; j++) {
        const float t = (float)j / (float)cells_v;
        for (uint32_t i = 0; i <= cells_u; i++) {
            const float s = (float)i / (float)cells_u;
            sl_vec3 p = sl_vec3_mul_add(origin, extent_u, s);
            positions[j * row + i] = sl_vec3_mul_add(p, extent_v, t);
        }
    }
    uint32_t k = 0;
    for (uint32_t j = 0; j < cells_v; j++) {
        for (uint32_t i = 0; i < cells_u; i++) {
            const uint32_t a = j * row + i;
            const uint32_t b = a + 1u;
            const uint32_t c = a + row + 1u;
            const uint32_t d = a + row;
            const bool even = ((i + j) & 1u) == 0u;
            const uint32_t tri[6] = { a, b, even ? c : d, even ? a : b, c, d };
            for (uint32_t m = 0; m < 6u; m++) {
                indices[k++] = tri[m];
            }
        }
    }
    return true;
}
