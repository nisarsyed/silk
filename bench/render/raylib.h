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
/* Optional diagnostic storage, allocated once after renderer initialization.
 * Lines are x/y/x/y/color, markers x/y/color, all in drawing-buffer pixels.
 * Palette is 16 RGB triples; marker mesh is 90 two-component unit vertices.
 * counts validates active prefixes. After any buffer write, the caller must
 * obtain a successful counts result before drawing; failure retains counts,
 * not a backup of the caller-written buffers. */
bool sl_render_overlay_init(uint32_t lines, uint32_t markers);
float *sl_render_overlay_lines(void);
float *sl_render_overlay_markers(void);
float *sl_render_overlay_colors(void);
float *sl_render_overlay_mesh(void);
uint32_t *sl_render_overlay_palette(void);
bool sl_render_overlay_counts(uint32_t lines, uint32_t markers);
#endif
