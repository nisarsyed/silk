#include "driver.h"
#include "../../wasm/context.h"
#include "../fixtures.h"
#include "scene.h"
#include <stdlib.h>
#include <string.h>

struct sl_render_study {
    sl_wasm_context adapter;
    uint32_t status[10];
    float settings[21];
    float *diagnostic_f32;
    uint32_t *diagnostic_u32;
    uint64_t drops;
};
sl_render_study *sl_render_study_create(uint32_t fixture, uint32_t copies,
                                        uint32_t sleep_enabled,
                                        uint32_t step_limit)
{
    if (sleep_enabled > 1u || step_limit == 0u ||
        step_limit > SL_RENDER_STEP_COUNT_MAX) {
        return NULL;
    }
    sl_render_study *study = calloc(1u, sizeof(*study));
    if (study == NULL) {
        return NULL;
    }
    sl_wasm_context *adapter = &study->adapter;
    if (!sl_render_scene_build(&adapter->world, &adapter->config, fixture,
                               copies, sleep_enabled != 0u) ||
        !sl_wasm_storage_init(adapter)) {
        sl_render_study_destroy(study);
        return NULL;
    }
    const uint32_t capacity = adapter->config.body_capacity;
    study->diagnostic_f32 = calloc((size_t)capacity * 4u + 5u, sizeof(float));
    study->diagnostic_u32 = calloc((size_t)capacity + 3u, sizeof(uint32_t));
    if (study->diagnostic_f32 == NULL || study->diagnostic_u32 == NULL) {
        sl_render_study_destroy(study);
        return NULL;
    }
    const uint32_t status[] = { fixture,
                                copies,
                                sleep_enabled,
                                step_limit,
                                0u,
                                adapter->config.body_capacity,
                                adapter->config.contact_capacity,
                                adapter->config.joint_capacity,
                                sl_world_get_substep_count(&adapter->world),
                                fixture == 0u   ? 0u
                                : fixture == 1u ? SL_BENCH_RAIN_SEED
                                                : UINT32_C(0x59B3AC01) };
    for (uint32_t i = 0u; i < 10u; ++i) {
        study->status[i] = status[i];
    }
    const sl_vec2 gravity = sl_world_get_gravity(&adapter->world);
    const float settings[] = { gravity.x,
                               gravity.y,
                               sl_world_get_linear_drag(&adapter->world),
                               sl_world_get_angular_drag(&adapter->world),
                               sl_world_get_linear_speed_max(&adapter->world),
                               SL_CONTACT_HERTZ_DEFAULT,
                               SL_CONTACT_DAMPING_RATIO_DEFAULT,
                               SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT,
                               SL_RESTITUTION_THRESHOLD_DEFAULT,
                               SL_JOINT_HERTZ_DEFAULT,
                               SL_JOINT_DAMPING_RATIO_DEFAULT,
                               SL_SLEEP_SPEED_MAX_DEFAULT,
                               SL_SLEEP_ANGULAR_SPEED_MAX_DEFAULT,
                               SL_SLEEP_TIME_MIN_DEFAULT,
                               fixture == 1u ? 0.4f : 0.6f,
                               fixture == 1u ? 0.1f : 0.0f,
                               SL_BENCH_TIMESTEP };
    for (uint32_t i = 0u; i < 17u; ++i) {
        study->settings[i] = settings[i];
    }
    const float camera[][4] = { { -13.0f, -1.0f, 13.0f, 22.0f },
                                { -9.0f, -1.0f, 9.0f, 25.0f },
                                { -1.0f, -1.0f, 29.0f, 16.0f } };
    const uint32_t camera_index = fixture == 3u ? 2u : fixture;
    for (uint32_t i = 0u; i < 4u; ++i) {
        study->settings[17u + i] = camera[camera_index][i];
    }
    const float half_spread = 16.0f * (float)(copies - 1u);
    study->settings[17] -= half_spread;
    study->settings[19] += half_spread;
    return study;
}
void sl_render_study_destroy(sl_render_study *study)
{
    if (study != NULL) {
        free(study->diagnostic_f32);
        free(study->diagnostic_u32);
        sl_wasm_world_dispose(&study->adapter);
        free(study);
    }
}
bool sl_render_study_step(sl_render_study *study)
{
    if (study == NULL || study->status[4] >= study->status[3]) {
        return false;
    }
    sl_world_step(&study->adapter.world, SL_BENCH_TIMESTEP);
    study->drops += sl_world_contact_drop_count(&study->adapter.world);
    ++study->status[4];
    return true;
}
sl_wasm_context *sl_render_study_adapter(sl_render_study *study)
{
    return study != NULL ? &study->adapter : NULL;
}
sl_world *sl_render_study_world(sl_render_study *study)
{
    return study != NULL ? &study->adapter.world : NULL;
}
const uint32_t *sl_render_study_status(const sl_render_study *study)
{
    return study != NULL ? study->status : NULL;
}
const float *sl_render_study_settings(const sl_render_study *study)
{
    return study != NULL ? study->settings : NULL;
}
uint64_t sl_render_study_drops(const sl_render_study *study)
{
    return study != NULL ? study->drops : 0u;
}
size_t sl_render_study_bytes(void)
{
    return sizeof(sl_render_study);
}

bool sl_render_study_diagnostics(sl_render_study *study)
{
    if (study == NULL) {
        return false;
    }
    sl_wasm_context *adapter = &study->adapter;
    const sl_world *world = &adapter->world;
    const uint32_t capacity = adapter->config.body_capacity;
    uint32_t *flags = study->diagnostic_u32;
    float *bounds = study->diagnostic_f32;
    memset(flags, 0, ((size_t)capacity + 3u) * sizeof(*flags));
    const uint32_t count = sl_world_body_count(world);
    for (uint32_t row = 0u; row < count; ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        sl_aabb box;
        if (sl_world_body_get_proxy_aabb(world, body, &box)) {
            flags[body.index] = 8u;
            bounds[body.index] = box.lower.x;
            bounds[capacity + body.index] = box.lower.y;
            bounds[2u * capacity + body.index] = box.upper.x;
            bounds[3u * capacity + body.index] = box.upper.y;
        }
    }
    const float *rect = study->settings + 17;
    const sl_vec2 center = { (rect[0] + rect[2]) * 0.5f,
                             (rect[1] + rect[3]) * 0.5f };
    sl_query_result result;
    if (!sl_world_query_point(world, center, SL_QUERY_ALL,
                              adapter->query_handles, capacity, &result)) {
        return false;
    }
    if (result.truncated || result.count > capacity) {
        return false;
    }
    flags[capacity] = result.count;
    for (uint32_t i = 0u; i < result.count; ++i) {
        flags[adapter->query_handles[i].index] |= 1u;
    }
    const sl_aabb box = { { center.x - 1.0f, center.y - 1.0f },
                          { center.x + 1.0f, center.y + 1.0f } };
    if (!sl_world_query_aabb(world, box, SL_QUERY_ALL, adapter->query_handles,
                             capacity, &result)) {
        return false;
    }
    if (result.truncated || result.count > capacity) {
        return false;
    }
    flags[capacity + 1u] = result.count;
    for (uint32_t i = 0u; i < result.count; ++i) {
        flags[adapter->query_handles[i].index] |= 2u;
    }
    const sl_ray ray = { { rect[0], center.y }, { rect[2] - rect[0], 0.0f } };
    sl_query_ray_result hit;
    if (!sl_world_query_ray(world, ray, SL_QUERY_ALL, &hit)) {
        return false;
    }
    flags[capacity + 2u] = hit.hit ? 1u : 0u;
    if (hit.hit) {
        flags[hit.body.index] |= 4u;
    }
    const float values[] = { hit.geometry.fraction, hit.geometry.point.x,
                             hit.geometry.point.y, hit.geometry.normal.x,
                             hit.geometry.normal.y };
    for (uint32_t i = 0u; i < 5u; ++i) {
        bounds[4u * capacity + i] = values[i];
    }
    return true;
}
const float *sl_render_study_diagnostic_f32(const sl_render_study *study)
{
    return study != NULL ? study->diagnostic_f32 : NULL;
}
const uint32_t *sl_render_study_diagnostic_u32(const sl_render_study *study)
{
    return study != NULL ? study->diagnostic_u32 : NULL;
}
size_t sl_render_study_diagnostic_bytes(const sl_render_study *study)
{
    return study != NULL
               ? (size_t)study->adapter.config.body_capacity * 20u + 32u
               : 0u;
}
