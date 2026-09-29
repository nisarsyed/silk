#include "../fixtures.h"
#include "driver.h"
#include <stdlib.h>

struct sl_render_frozen {
    float *floats;
    uint32_t *words;
    uint32_t count;
};

sl_render_frozen *sl_render_frozen_create(sl_render_study *study,
                                          uint32_t instances)
{
    if (study == NULL || sl_render_study_failed(study) || instances < 256u ||
        instances > SL_BODY_COUNT_MAX || (instances & (instances - 1u)) != 0u) {
        return NULL;
    }
    const uint32_t *status = sl_render_study_status(study);
    if (status[0] != 1u || status[1] != 1u || status[2] != 0u ||
        status[4] != 120u) {
        return NULL;
    }
    const sl_world *world = sl_render_study_world(study);
    if (sl_world_body_count(world) != SL_BENCH_RAIN_BODY_COUNT) {
        return NULL;
    }
    sl_body_handle source[SL_BENCH_RAIN_CIRCLE_COUNT];
    uint32_t source_count = 0u;
    for (uint32_t row = 0u; row < SL_BENCH_RAIN_BODY_COUNT; ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        if (sl_world_body_get_type(world, body) == SL_BODY_STATIC ||
            sl_world_body_get_shape(world, body)->kind == SL_SHAPE_NONE) {
            continue;
        }
        if (source_count == SL_BENCH_RAIN_CIRCLE_COUNT) {
            return NULL;
        }
        source[source_count++] = body;
    }
    if (source_count != SL_BENCH_RAIN_CIRCLE_COUNT) {
        return NULL;
    }
    sl_render_frozen *out = calloc(1u, sizeof(*out));
    if (out == NULL) {
        return NULL;
    }
    out->floats = calloc((size_t)instances * 4u + 5u, sizeof(float));
    out->words = calloc((size_t)instances + 3u, sizeof(uint32_t));
    if (out->floats == NULL || out->words == NULL) {
        sl_render_frozen_destroy(out);
        return NULL;
    }
    out->count = instances;
    const uint32_t tiles = (instances + source_count - 1u) / source_count;
    const float half_spread = 16.0f * (float)(tiles - 1u);
    const sl_vec2 center = { 0.0f, 12.0f };
    const sl_aabb box = { { -1.0f, 11.0f }, { 1.0f, 13.0f } };
    const sl_ray ray = { { -9.0f - half_spread, 12.0f },
                         { 18.0f + 2.0f * half_spread, 0.0f } };
    sl_ray_hit nearest = { 0 };
    uint32_t nearest_row = UINT32_MAX;
    for (uint32_t row = 0u; row < instances; ++row) {
        const sl_body_handle body = source[row % source_count];
        const float offset = 32.0f * (float)(row / source_count) - half_spread;
        sl_transform transform = sl_world_body_get_transform(world, body);
        transform.position.x += offset;
        const sl_shape *shape = sl_world_body_get_shape(world, body);
        sl_aabb proxy;
        if (sl_world_body_get_proxy_aabb(world, body, &proxy)) {
            out->words[row] = 8u;
            out->floats[row] = proxy.lower.x + offset;
            out->floats[instances + row] = proxy.lower.y;
            out->floats[2u * instances + row] = proxy.upper.x + offset;
            out->floats[3u * instances + row] = proxy.upper.y;
        }
        if (sl_shape_contains_point(shape, transform, center)) {
            out->words[row] |= 1u;
            ++out->words[instances];
        }
        if (sl_aabb_overlaps(sl_shape_aabb(shape, transform), box)) {
            out->words[row] |= 2u;
            ++out->words[instances + 1u];
        }
        sl_ray_hit hit;
        if (sl_shape_ray_cast(shape, transform, ray, &hit) &&
            (nearest_row == UINT32_MAX || hit.fraction < nearest.fraction)) {
            nearest = hit;
            nearest_row = row;
        }
    }
    if (nearest_row != UINT32_MAX) {
        out->words[nearest_row] |= 4u;
        out->words[instances + 2u] = 1u;
    }
    const float values[] = { nearest.fraction, nearest.point.x, nearest.point.y,
                             nearest.normal.x, nearest.normal.y };
    for (uint32_t i = 0u; i < 5u; ++i) {
        out->floats[4u * instances + i] = values[i];
    }
    return out;
}
void sl_render_frozen_destroy(sl_render_frozen *frozen)
{
    if (frozen != NULL) {
        free(frozen->floats);
        free(frozen->words);
        free(frozen);
    }
}
uint32_t sl_render_frozen_count(const sl_render_frozen *frozen)
{
    return frozen != NULL ? frozen->count : 0u;
}
const float *sl_render_frozen_f32(const sl_render_frozen *frozen)
{
    return frozen != NULL ? frozen->floats : NULL;
}
const uint32_t *sl_render_frozen_u32(const sl_render_frozen *frozen)
{
    return frozen != NULL ? frozen->words : NULL;
}
size_t sl_render_frozen_bytes(const sl_render_frozen *frozen)
{
    return frozen != NULL ? sizeof(*frozen) + (size_t)frozen->count * 20u + 32u
                          : 0u;
}
