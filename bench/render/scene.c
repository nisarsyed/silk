#include "scene.h"
#include "../fixtures.h"
#include <stdlib.h>

static bool copy_bodies(sl_world *world, const sl_bench_scene *source,
                        sl_body_handle *handles, float offset)
{
    for (uint32_t row = 0u; row < source->body_count; ++row) {
        const sl_body_handle body = source->bodies[row];
        sl_vec2 position = sl_world_body_get_position(source->world, body);
        position.x += offset;
        const sl_body_desc desc = {
            .position = position,
            .velocity = sl_world_body_get_velocity(source->world, body),
            .mass = sl_world_body_get_mass(source->world, body),
            .type = sl_world_body_get_type(source->world, body),
            .angle = sl_world_body_get_angle(source->world, body),
            .angular_velocity =
                sl_world_body_get_angular_velocity(source->world, body),
            .friction = sl_world_body_get_friction(source->world, body),
            .restitution = sl_world_body_get_restitution(source->world, body),
            .shape = sl_world_body_get_shape(source->world, body)
        };
        if (body.index >= SL_BENCH_BODY_COUNT_MAX) {
            return false;
        }
        handles[body.index] = sl_world_body_create(world, &desc);
        if (sl_body_handle_is_null(handles[body.index])) {
            return false;
        }
    }
    return true;
}
static bool copy_joints(sl_world *world, const sl_bench_scene *source,
                        const sl_body_handle *handles)
{
    const uint32_t count = sl_world_joint_count(source->world);
    for (uint32_t row = 0u; row < count; ++row) {
        sl_joint_desc desc = sl_world_joint_get_desc(
            source->world, sl_world_joint_at(source->world, row));
        if (desc.body_a.index >= SL_BENCH_BODY_COUNT_MAX ||
            desc.body_b.index >= SL_BENCH_BODY_COUNT_MAX) {
            return false;
        }
        desc.body_a = handles[desc.body_a.index];
        desc.body_b = handles[desc.body_b.index];
        if (sl_joint_handle_is_null(sl_world_joint_create(world, &desc))) {
            return false;
        }
    }
    return true;
}
bool sl_render_scene_build(sl_world *world, sl_world_config *config,
                           uint32_t fixture, uint32_t copies,
                           bool sleep_enabled)
{
    if (world == NULL || config == NULL ||
        (fixture != 0u && fixture != 1u && fixture != 3u) ||
        (copies != 1u && copies != 2u && copies != 4u && copies != 8u &&
         copies != 16u)) {
        return false;
    }
    sl_bench_scene *source = calloc(1u, sizeof(*source));
    if (source == NULL) {
        return false;
    }
    sl_world original = { 0 };
    sl_world *fixture_world = copies == 1u ? world : &original;
    if (!sl_bench_scene_build(source, fixture_world, fixture, sleep_enabled)) {
        free(source);
        return false;
    }
    sl_world_config resolved = source->config;
    resolved.contact_capacity = sl_world_contact_capacity(fixture_world);
    if (copies == 1u) {
        *config = resolved;
        free(source);
        return true;
    }
    /* Upper bounds are 32048 bodies, 262144 contacts, and 1536 joints.
     * The source world and 48 KiB fixture records are setup-only overhead.
     * The bounded handle map is reused for each copy and never retained. */
    resolved.body_capacity *= copies;
    resolved.contact_capacity *= copies;
    resolved.joint_capacity *= copies;
    bool ok = sl_world_init(world, &resolved);
    sl_body_handle handles[SL_BENCH_BODY_COUNT_MAX] = { 0 };
    for (uint32_t i = 0u; ok && i < copies; ++i) {
        const float offset = 32.0f * ((float)i - (float)(copies - 1u) * 0.5f);
        ok = copy_bodies(world, source, handles, offset) &&
             copy_joints(world, source, handles);
    }
    sl_world_destroy(&original);
    free(source);
    if (!ok) {
        sl_world_destroy(world);
        return false;
    }
    *config = resolved;
    return true;
}
