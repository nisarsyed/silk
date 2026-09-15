#include "raylib.h"
#include <math.h>
#include <raylib.h>
#include <rlgl.h>
#include <stdlib.h>

static float *vertices;
static float *poses;
static uint32_t *batches;
static uint32_t instance_count;
static uint32_t vertex_count;
static uint32_t batch_count;
static bool ready;
static float *overlay_lines, *overlay_markers, *overlay_colors;
static float overlay_mesh[180];
static uint32_t overlay_palette[48];
static uint32_t line_capacity, marker_capacity, line_count, marker_count;

static void overlay_color(float code)
{
    const uint32_t at = (uint32_t)code * 3u;
    rlColor4ub((unsigned char)overlay_palette[at],
               (unsigned char)overlay_palette[at + 1u],
               (unsigned char)overlay_palette[at + 2u], 255u);
}
static void overlay_draw(void)
{
    rlBegin(RL_TRIANGLES);
    for (uint32_t i = 0u; i < line_count; ++i) {
        const float *p = overlay_lines + 5u * i;
        const float dx = p[2] - p[0], dy = p[3] - p[1];
        const float length_sq = dx * dx + dy * dy;
        const float inverse = length_sq > 0.0f ? 0.5f / sqrtf(length_sq) : 0.0f;
        const float nx = -dy * inverse, ny = dx * inverse;
        overlay_color(p[4]);
        rlVertex2f(p[0] - nx, p[1] - ny);
        rlVertex2f(p[2] - nx, p[3] - ny);
        rlVertex2f(p[2] + nx, p[3] + ny);
        rlVertex2f(p[0] - nx, p[1] - ny);
        rlVertex2f(p[2] + nx, p[3] + ny);
        rlVertex2f(p[0] + nx, p[1] + ny);
    }
    for (uint32_t i = 0u; i < marker_count; ++i) {
        const float *p = overlay_markers + 3u * i;
        overlay_color(p[2]);
        for (uint32_t v = 0u; v < 90u; ++v) {
            rlVertex2f(p[0] + 3.0f * overlay_mesh[2u * v],
                       p[1] + 3.0f * overlay_mesh[2u * v + 1u]);
        }
    }
    rlEnd();
}

bool sl_render_init(uint32_t instances, uint32_t vertices_count,
                    uint32_t batches_count, uint32_t width, uint32_t height)
{
    if (ready || instances == 0u || instances > 65536u ||
        vertices_count == 0u || vertices_count > 65536u * 90u ||
        batches_count == 0u || batches_count > instances || width == 0u ||
        height == 0u || width > 8192u || height > 8192u) {
        return false;
    }
    vertices = calloc((size_t)vertices_count * 2u, sizeof(float));
    poses = calloc((size_t)instances * 4u, sizeof(float));
    batches = calloc((size_t)batches_count * 5u, sizeof(uint32_t));
    if (vertices == NULL || poses == NULL || batches == NULL) {
        sl_render_dispose();
        return false;
    }
    instance_count = instances;
    vertex_count = vertices_count;
    batch_count = batches_count;
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow((int)width, (int)height, "Silk renderer comparison");
    ready = IsWindowReady();
    if (!ready) {
        sl_render_dispose();
        return false;
    }
    SetTargetFPS(0);
    return true;
}
float *sl_render_vertices(void)
{
    return vertices;
}
float *sl_render_poses(void)
{
    return poses;
}
uint32_t *sl_render_batches(void)
{
    return batches;
}

bool sl_render_draw(float scale, float offset_x, float offset_y)
{
    if (!ready || !isfinite(scale) || scale <= 0.0f || !isfinite(offset_x) ||
        !isfinite(offset_y)) {
        return false;
    }
    /* Validate ranges before submitting any drawing. These bounded checks are
     * included in CPU submission, not hidden in a separate timing region. */
    for (uint32_t i = 0u; i < batch_count; ++i) {
        const uint32_t *b = batches + 5u * i;
        if (b[0] > instance_count || b[1] > instance_count - b[0] ||
            b[2] > vertex_count || b[3] > vertex_count - b[2] ||
            b[3] % 3u != 0u || b[4] > 1u) {
            return false;
        }
    }
    BeginDrawing();
    ClearBackground((Color){ 16u, 24u, 32u, 255u });
    rlDisableBackfaceCulling();
    rlSetTexture(rlGetTextureIdDefault());
    rlBegin(RL_TRIANGLES);
    rlTexCoord2f(0.5f, 0.5f);
    rlNormal3f(0.0f, 0.0f, 1.0f);
    for (uint32_t i = 0u; i < batch_count; ++i) {
        const uint32_t *b = batches + 5u * i;
        if (b[4] == 0u) {
            rlColor4ub(113u, 146u, 165u, 255u);
        } else {
            rlColor4ub(235u, 189u, 112u, 255u);
        }
        for (uint32_t j = b[0]; j < b[0] + b[1]; ++j) {
            const float *p = poses + 4u * j;
            if (overlay_colors != NULL) {
                overlay_color(overlay_colors[j]);
            }
            for (uint32_t v = b[2]; v < b[2] + b[3]; ++v) {
                const float x = vertices[2u * v];
                const float y = vertices[2u * v + 1u];
                rlVertex2f(offset_x + scale * (p[0] + p[2] * x - p[3] * y),
                           offset_y - scale * (p[1] + p[3] * x + p[2] * y));
            }
        }
    }
    rlEnd();
    if (overlay_lines != NULL) {
        overlay_draw();
    }
    rlSetTexture(0u);
    EndDrawing();
    return true;
}
void sl_render_dispose(void)
{
    if (ready) {
        CloseWindow();
    }
    free(overlay_lines);
    free(overlay_markers);
    free(overlay_colors);
    overlay_lines = NULL;
    overlay_markers = NULL;
    overlay_colors = NULL;
    line_capacity = 0u;
    marker_capacity = 0u;
    line_count = 0u;
    marker_count = 0u;
    free(vertices);
    free(poses);
    free(batches);
    vertices = NULL;
    poses = NULL;
    batches = NULL;
    instance_count = 0u;
    vertex_count = 0u;
    batch_count = 0u;
    ready = false;
}

bool sl_render_overlay_init(uint32_t lines, uint32_t markers)
{
    /* Derived from the existing maximum B/C/J capacities, never dynamic growth.
     */
    if (!ready || overlay_lines != NULL || lines == 0u || markers == 0u ||
        lines > 4u * 65536u + 2u * 262144u + 65536u + 6u ||
        markers > 2u * 262144u + 2u * 65536u + 2u) {
        return false;
    }
    overlay_lines = calloc((size_t)lines * 5u, sizeof(float));
    overlay_markers = calloc((size_t)markers * 3u, sizeof(float));
    overlay_colors = calloc(instance_count, sizeof(float));
    if (overlay_lines == NULL || overlay_markers == NULL ||
        overlay_colors == NULL) {
        free(overlay_lines);
        free(overlay_markers);
        free(overlay_colors);
        overlay_lines = NULL;
        overlay_markers = NULL;
        overlay_colors = NULL;
        return false;
    }
    line_capacity = lines;
    marker_capacity = markers;
    return true;
}
float *sl_render_overlay_lines(void)
{
    return overlay_lines;
}
float *sl_render_overlay_markers(void)
{
    return overlay_markers;
}
float *sl_render_overlay_colors(void)
{
    return overlay_colors;
}
float *sl_render_overlay_mesh(void)
{
    return overlay_mesh;
}
uint32_t *sl_render_overlay_palette(void)
{
    return overlay_palette;
}
static bool color_valid(float code)
{
    return isfinite(code) && code >= 0.0f && code < 16.0f &&
           floorf(code) == code;
}
bool sl_render_overlay_counts(uint32_t lines, uint32_t markers)
{
    if (overlay_lines == NULL || lines > line_capacity ||
        markers > marker_capacity) {
        return false;
    }
    for (uint32_t i = 0u; i < 48u; ++i) {
        if (overlay_palette[i] > 255u) {
            return false;
        }
    }
    for (uint32_t i = 0u; i < 180u; ++i) {
        if (!isfinite(overlay_mesh[i]) || overlay_mesh[i] < -1.0f ||
            overlay_mesh[i] > 1.0f) {
            return false;
        }
    }
    for (uint32_t i = 0u; i < instance_count; ++i) {
        if (!color_valid(overlay_colors[i])) {
            return false;
        }
    }
    for (uint32_t i = 0u; i < lines; ++i) {
        const float *p = overlay_lines + 5u * i;
        for (uint32_t j = 0u; j < 4u; ++j) {
            if (!isfinite(p[j])) {
                return false;
            }
        }
        const float dx = p[2] - p[0], dy = p[3] - p[1];
        if (!isfinite(dx * dx + dy * dy) || !color_valid(p[4])) {
            return false;
        }
    }
    for (uint32_t i = 0u; i < markers; ++i) {
        const float *p = overlay_markers + 3u * i;
        if (!isfinite(p[0]) || !isfinite(p[1]) || !color_valid(p[2])) {
            return false;
        }
    }
    line_count = lines;
    marker_count = markers;
    return true;
}
