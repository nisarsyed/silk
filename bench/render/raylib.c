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
            for (uint32_t v = b[2]; v < b[2] + b[3]; ++v) {
                const float x = vertices[2u * v];
                const float y = vertices[2u * v + 1u];
                rlVertex2f(offset_x + scale * (p[0] + p[2] * x - p[3] * y),
                           offset_y - scale * (p[1] + p[3] * x + p[2] * y));
            }
        }
    }
    rlEnd();
    rlSetTexture(0u);
    EndDrawing();
    return true;
}
void sl_render_dispose(void)
{
    if (ready) {
        CloseWindow();
    }
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
