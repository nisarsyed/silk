#ifndef SILK_RENDER_RAYLIB_H
#define SILK_RENDER_RAYLIB_H
#include <stdbool.h>
#include <stdint.h>
/* Private comparison ABI. One renderer per module. Setup allocates all storage;
 * returned buffers live until disposal. The TS harness supplies validated
 * triangle meshes, ordered batches and finite transforms before drawing. */
bool sl_render_init(uint32_t instances, uint32_t vertices, uint32_t batches,
                    uint32_t width, uint32_t height);
float *sl_render_vertices(void);
float *sl_render_poses(void);
uint32_t *sl_render_batches(void);
bool sl_render_draw(float scale, float offset_x, float offset_y);
void sl_render_dispose(void);
#endif
