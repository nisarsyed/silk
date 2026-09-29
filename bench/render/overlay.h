#ifndef SILK_RENDER_OVERLAY_H
#define SILK_RENDER_OVERLAY_H
#include "driver.h"

/* Private transient drawing commands, not entity state. The caller allocates
 * all arrays during setup. Lines use five floats (ax/ay/bx/by/palette index),
 * markers three (x/y/palette index), colors one per body in packed row order.
 * Coordinates are drawing-buffer pixels. Camera arithmetic uses double to
 * match the TS command builder before storing binary32 commands. */
typedef struct sl_render_overlay {
    float *lines;
    float *markers;
    float *colors;
    uint32_t line_capacity;
    uint32_t marker_capacity;
    uint32_t color_count;
    uint32_t line_count;
    uint32_t marker_count;
    double scale;
    double offset_x;
    double offset_y;
} sl_render_overlay;

/* Allocation-free; world state is unchanged. On failure the entire output
 * frame is invalid and must not be drawn. Capacities must cover the configured
 * worst case, so no primitive is silently dropped or buffers grown. */
bool sl_render_overlay_prepare(sl_render_study *study, sl_render_overlay *out);
#endif
