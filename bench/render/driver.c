#include "driver.h"
#include "../../wasm/context.h"
#include "../fixtures.h"
#include "scene.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct sl_render_study {
    sl_wasm_context adapter;
    uint32_t status[10];
    float settings[21];
    float *diagnostic_f32;
    uint32_t *diagnostic_u32;
    uint64_t drops;
    uint32_t pointer_status[3];
    sl_vec2 grab_local;
    sl_vec2 grab_target;
    bool failed;
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
    study->pointer_status[1] = UINT32_MAX;
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
    if (!sl_render_study_validate(study)) {
        sl_render_study_destroy(study);
        return NULL;
    }
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
    if (study == NULL || study->failed ||
        study->status[4] >= study->status[3]) {
        return false;
    }
    if (study->pointer_status[0] != 0u &&
        study->pointer_status[1] != UINT32_MAX) {
        const sl_body_handle body = { study->pointer_status[1],
                                      study->pointer_status[2] };
        sl_world *world = &study->adapter.world;
        if (!sl_world_body_is_valid(world, body)) {
            study->pointer_status[0] = 0u;
            study->pointer_status[1] = UINT32_MAX;
            study->pointer_status[2] = 0u;
        } else {
            const sl_transform transform =
                sl_world_body_get_transform(world, body);
            const sl_vec2 point =
                sl_transform_apply(transform, study->grab_local);
            // Match the native sandbox: 15 N/m, applied once per fixed step
            // at the original local grab point, including its induced torque.
            const sl_vec2 force =
                sl_vec2_scale(sl_vec2_sub(study->grab_target, point), 15.0f);
            if (!sl_world_body_apply_force_at_point(world, body, force,
                                                    point)) {
                study->failed = true;
                return false;
            }
        }
    }
    sl_world_step(&study->adapter.world, SL_BENCH_TIMESTEP);
    study->drops += sl_world_contact_drop_count(&study->adapter.world);
    ++study->status[4];
    return sl_render_study_validate(study);
}
static bool state_finite(const sl_world *world)
{
    const uint32_t bodies = sl_world_body_count(world);
    for (uint32_t row = 0u; row < bodies; ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        const sl_transform t = sl_world_body_get_transform(world, body);
        if (!sl_vec2_is_finite(t.position) || !isfinite(t.rotation.c) ||
            !isfinite(t.rotation.s) ||
            !sl_vec2_is_finite(sl_world_body_get_velocity(world, body)) ||
            !isfinite(sl_world_body_get_angular_velocity(world, body))) {
            return false;
        }
        sl_aabb bounds;
        if (sl_world_body_get_proxy_aabb(world, body, &bounds) &&
            (!sl_vec2_is_finite(bounds.lower) ||
             !sl_vec2_is_finite(bounds.upper))) {
            return false;
        }
    }
    const uint32_t contacts = sl_world_contact_count(world);
    for (uint32_t row = 0u; row < contacts; ++row) {
        const sl_contact *contact = sl_world_contact_at(world, row);
        const sl_manifold *m = &contact->manifold;
        if (!isfinite(contact->friction) || !isfinite(contact->restitution) ||
            !sl_vec2_is_finite(m->normal) ||
            m->point_count > SL_MANIFOLD_POINT_COUNT_MAX) {
            return false;
        }
        for (uint32_t i = 0u; i < m->point_count; ++i) {
            const sl_manifold_point *p = &m->points[i];
            if (!sl_vec2_is_finite(p->anchor_a) ||
                !sl_vec2_is_finite(p->anchor_b) ||
                !sl_vec2_is_finite(p->point) || !isfinite(p->separation) ||
                !isfinite(p->normal_impulse) || !isfinite(p->tangent_impulse) ||
                !isfinite(p->normal_velocity)) {
                return false;
            }
        }
    }
    const uint32_t joints = sl_world_joint_count(world);
    for (uint32_t row = 0u; row < joints; ++row) {
        if (!sl_vec2_is_finite(sl_world_joint_get_linear_impulse(
                world, sl_world_joint_at(world, row)))) {
            return false;
        }
    }
    return true;
}
bool sl_render_study_validate(sl_render_study *study)
{
    if (study == NULL || study->failed) {
        return false;
    }
    study->failed = !state_finite(&study->adapter.world);
    return !study->failed;
}
bool sl_render_study_failed(const sl_render_study *study)
{
    return study == NULL || study->failed;
}
sl_wasm_context *sl_render_study_adapter(sl_render_study *study)
{
    return study != NULL ? &study->adapter : NULL;
}
sl_world *sl_render_study_world(sl_render_study *study)
{
    return study != NULL ? &study->adapter.world : NULL;
}
bool sl_render_study_poses(const sl_render_study *study, float *poses,
                           uint32_t count)
{
    if (study == NULL || poses == NULL ||
        count != sl_world_body_count(&study->adapter.world)) {
        return false;
    }
    const sl_world *world = &study->adapter.world;
    for (uint32_t row = 0u; row < count; ++row) {
        const sl_transform transform =
            sl_world_body_get_transform(world, sl_world_body_at(world, row));
        poses[4u * row] = transform.position.x;
        poses[4u * row + 1u] = transform.position.y;
        poses[4u * row + 2u] = transform.rotation.c;
        poses[4u * row + 3u] = transform.rotation.s;
    }
    return true;
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

bool sl_render_study_pointer(sl_render_study *study, uint32_t action, float x,
                             float y)
{
    if (study == NULL || study->failed || study->status[1] != 1u ||
        study->status[4] >= study->status[3] || action > 3u || !isfinite(x) ||
        !isfinite(y) || fabsf(x) > SL_POSITION_ABS_MAX ||
        fabsf(y) > SL_POSITION_ABS_MAX) {
        return false;
    }
    const sl_vec2 target = { x, y };
    if (action == 0u) {
        if (study->pointer_status[0] != 0u) {
            return false;
        }
        sl_wasm_context *adapter = &study->adapter;
        sl_world *world = &adapter->world;
        sl_query_result result = { 0 };
        if (!sl_world_query_point(world, target, SL_QUERY_DYNAMIC,
                                  adapter->query_handles,
                                  adapter->config.body_capacity, &result) ||
            result.truncated) {
            return false;
        }
        sl_body_handle selected = sl_body_handle_null();
        float nearest = FLT_MAX;
        // Query order is ascending body slot. Strict comparison retains that
        // deterministic tie order, matching the sandbox's closest-center pick.
        for (uint32_t i = 0u; i < result.count; ++i) {
            const sl_body_handle body = adapter->query_handles[i];
            const sl_vec2 position = sl_world_body_get_position(world, body);
            const float distance =
                sl_vec2_length_sq(sl_vec2_sub(target, position));
            if (distance < nearest) {
                nearest = distance;
                selected = body;
            }
        }
        sl_vec2 local = { 0.0f, 0.0f };
        if (!sl_body_handle_is_null(selected)) {
            local = sl_transform_apply_inverse(
                sl_world_body_get_transform(world, selected), target);
            if (!sl_world_body_wake(world, selected)) {
                return false;
            }
        }
        study->pointer_status[0] = 1u;
        study->pointer_status[1] = selected.index;
        study->pointer_status[2] = selected.generation;
        study->grab_local = local;
        study->grab_target = target;
    } else if (action == 1u) {
        if (study->pointer_status[0] == 0u) {
            return false;
        }
        study->grab_target = target;
    } else {
        // Release/cancel apply no force; no force is banked by input events.
        study->pointer_status[0] = 0u;
        study->pointer_status[1] = UINT32_MAX;
        study->pointer_status[2] = 0u;
    }
    return true;
}
const uint32_t *sl_render_study_pointer_status(const sl_render_study *study)
{
    return study != NULL ? study->pointer_status : NULL;
}
