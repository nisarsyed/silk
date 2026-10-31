#ifndef SILK_MESH_H
#define SILK_MESH_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Largest grid side, in cells. Keeps every count below SL_VERTEX_COUNT_MAX
 * and SL_TRIANGLE_COUNT_MAX for square grids up to this size. */
#define SL_MESH_GRID_CELL_COUNT_MAX 1023u

/* Counts for a cells_u x cells_v grid; 0 when either side is outside
 * [1, SL_MESH_GRID_CELL_COUNT_MAX]. */
uint32_t sl_mesh_grid_vertex_count(uint32_t cells_u, uint32_t cells_v);
uint32_t sl_mesh_grid_triangle_count(uint32_t cells_u, uint32_t cells_v);

/* Fills a planar parallelogram grid spanning origin + s * extent_u +
 * t * extent_v for s, t in [0, 1]. Vertex (i, j), i along u, is at index
 * j * (cells_u + 1) + i. Each cell splits along alternating diagonals so
 * the mesh has no preferred shear direction. Returns false, writing
 * nothing, when the counts do not match exactly or any input is
 * non-finite. */
bool sl_mesh_grid(sl_vec3 origin, sl_vec3 extent_u, sl_vec3 extent_v,
                  uint32_t cells_u, uint32_t cells_v, sl_vec3 *positions,
                  uint32_t position_count, uint32_t *indices,
                  uint32_t index_count);

#ifdef __cplusplus
}
#endif

#endif /* SILK_MESH_H */
