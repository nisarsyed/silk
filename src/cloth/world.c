#include <stdlib.h>
#include <string.h>

#include "arena.h"
#include "world_internal.h"

/* Lays out every array; returns the total size. Pass base == NULL to
 * measure without assigning. */
static size_t sl_world_layout(sl_world_state *s, unsigned char *base)
{
    const size_t v = s->vertex_capacity;
    const size_t t = s->triangle_capacity;
    const size_t b = s->bend_capacity;
    size_t at = 0;
#define SL_TAKE(field, count)                                                  \
    s->field = sl_arena_take(base, &at, (count), sizeof(*s->field))
    SL_TAKE(position, v);
    SL_TAKE(position_start, v);
    SL_TAKE(velocity, v);
    SL_TAKE(inertia_target, v);
    SL_TAKE(pin_target, v);
    SL_TAKE(pin_origin, v);
    SL_TAKE(mass, v);
    SL_TAKE(vertex_cloth, v);
    SL_TAKE(pinned, v);
    SL_TAKE(contact, v);
    SL_TAKE(color_scratch, v);
    SL_TAKE(color_order, v);
    SL_TAKE(vertex_triangle_offset, v + 1u);
    SL_TAKE(vertex_triangle, 3u * t);
    SL_TAKE(vertex_bend_offset, v + 1u);
    SL_TAKE(vertex_bend, 4u * b);
    SL_TAKE(triangle_index, 3u * t);
    SL_TAKE(triangle_rest, 4u * t);
    SL_TAKE(triangle_area, t);
    SL_TAKE(bend_index, 4u * b);
    SL_TAKE(bend_weight, 4u * b);
    SL_TAKE(bend_scale, b);
    SL_TAKE(edge_index, 2u * (size_t)s->edge_capacity);

    SL_TAKE(cloths, s->cloth_capacity);
    /* Self contact: nothing when the capacity is 0. */
    const size_t pc = s->pair_capacity;
    const size_t on = (pc > 0u) ? 1u : 0u;
    const size_t tri_bvh = on * t;
    const size_t edge_bvh = on * (size_t)s->edge_capacity;
    s->triangle_bvh.capacity = (uint32_t)tri_bvh;
    s->edge_bvh.capacity = (uint32_t)edge_bvh;
    SL_TAKE(pairs, pc);
    SL_TAKE(pair_ref_offset, on * (v + 1u));
    SL_TAKE(pair_ref, 4u * pc);
    SL_TAKE(vertex_dmin, on * v);
    SL_TAKE(triangle_dmin, on * t);
    SL_TAKE(edge_dmin, on * (size_t)s->edge_capacity);
    SL_TAKE(bound, on * v);
    SL_TAKE(at_bound, on * v);
    SL_TAKE(stage_pair, pc);
    SL_TAKE(stage_distance, pc);
    SL_TAKE(stage_owner, pc);
    SL_TAKE(stage_order, pc);
    SL_TAKE(owner_offset, on * ((size_t)s->edge_capacity + 1u));
    SL_TAKE(position_safe, on * v);
    SL_TAKE(position_next, on * v);
    SL_TAKE(triangle_bvh.node_lo, 2u * tri_bvh);
    SL_TAKE(triangle_bvh.node_hi, 2u * tri_bvh);
    SL_TAKE(triangle_bvh.node_first, 2u * tri_bvh);
    SL_TAKE(triangle_bvh.node_count_of, 2u * tri_bvh);
    SL_TAKE(triangle_bvh.perm, tri_bvh);
    SL_TAKE(triangle_bvh.centroid, tri_bvh);
    SL_TAKE(triangle_bvh.elem_lo, tri_bvh);
    SL_TAKE(triangle_bvh.elem_hi, tri_bvh);
    SL_TAKE(triangle_bvh.sphere_center, tri_bvh);
    SL_TAKE(triangle_bvh.sphere_radius, tri_bvh);
    SL_TAKE(edge_bvh.node_lo, 2u * edge_bvh);
    SL_TAKE(edge_bvh.node_hi, 2u * edge_bvh);
    SL_TAKE(edge_bvh.node_first, 2u * edge_bvh);
    SL_TAKE(edge_bvh.node_count_of, 2u * edge_bvh);
    SL_TAKE(edge_bvh.perm, edge_bvh);
    SL_TAKE(edge_bvh.centroid, edge_bvh);
    SL_TAKE(edge_bvh.elem_lo, edge_bvh);
    SL_TAKE(edge_bvh.elem_hi, edge_bvh);
    SL_TAKE(edge_bvh.sphere_center, edge_bvh);
    SL_TAKE(edge_bvh.sphere_radius, edge_bvh);
#undef SL_TAKE
    return sl_arena_align(at);
}

static bool sl_vec3_in_domain(sl_vec3 v)
{
    return sl_vec3_is_finite(v) && sl_abs(v.x) <= SL_POSITION_ABS_MAX &&
           sl_abs(v.y) <= SL_POSITION_ABS_MAX &&
           sl_abs(v.z) <= SL_POSITION_ABS_MAX;
}

static bool sl_positive_or_default(float value, float fallback, float *out)
{
    if (value == 0.0f) {
        *out = fallback;
        return true;
    }
    if (!sl_is_finite(value) || !(value > 0.0f)) {
        return false;
    }
    *out = value;
    return true;
}

bool sl_world_init(sl_world *world, const sl_world_config *config)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(world->state == NULL);
    SL_ASSERT(config != NULL);
    if (world == NULL || config == NULL || world->state != NULL) {
        return false;
    }
    sl_world_state s;
    memset(&s, 0, sizeof(s));
    if (config->vertex_capacity < 1u ||
        config->vertex_capacity > SL_VERTEX_COUNT_MAX ||
        config->triangle_capacity < 1u ||
        config->triangle_capacity > SL_TRIANGLE_COUNT_MAX ||
        config->cloth_capacity < 1u ||
        config->cloth_capacity > SL_CLOTH_COUNT_MAX ||
        config->self_contact_capacity > SL_SELF_CONTACT_COUNT_MAX ||
        config->substep_count > SL_SUBSTEP_COUNT_MAX ||
        config->iteration_count > SL_ITERATION_COUNT_MAX ||
        !sl_vec3_is_finite(config->gravity) ||
        !sl_vec3_is_finite(config->wind) ||
        !sl_positive_or_default(config->speed_max, SL_SPEED_MAX_DEFAULT,
                                &s.speed_max) ||
        !sl_positive_or_default(config->collision_stiffness,
                                SL_COLLISION_STIFFNESS_DEFAULT,
                                &s.collision_stiffness) ||
        !sl_positive_or_default(config->self_contact_stiffness,
                                SL_SELF_CONTACT_STIFFNESS_DEFAULT,
                                &s.self_stiffness)) {
        return false;
    }
    s.gravity = config->gravity;
    s.wind = config->wind;
    s.substep_count = (config->substep_count == 0u) ? SL_SUBSTEP_COUNT_DEFAULT
                                                    : config->substep_count;
    s.iteration_count = (config->iteration_count == 0u)
                            ? SL_ITERATION_COUNT_DEFAULT
                            : config->iteration_count;
    s.vertex_capacity = config->vertex_capacity;
    s.triangle_capacity = config->triangle_capacity;
    /* Interior edges of a manifold mesh number at most 3T / 2. */
    s.bend_capacity = (3u * config->triangle_capacity + 1u) / 2u;
    /* Each triangle introduces at most three new edges. */
    s.edge_capacity = 3u * config->triangle_capacity;
    s.cloth_capacity = config->cloth_capacity;
    s.pair_capacity = config->self_contact_capacity;
    s.parallel_for = config->parallel_for;
    s.parallel_context = config->parallel_context;

    const size_t bytes = sl_world_layout(&s, NULL);
    sl_world_state *state = malloc(sizeof(*state));
    unsigned char *arena = malloc(bytes);
    if (state == NULL || arena == NULL) {
        free(state);
        free(arena);
        return false;
    }
    /* Zero everything once so no array is ever read uninitialized. */
    memset(arena, 0, bytes);
    *state = s;
    (void)sl_world_layout(state, arena);
    state->arena = arena;
    state->arena_bytes = bytes;
    world->state = state;
    return true;
}

void sl_world_destroy(sl_world *world)
{
    SL_ASSERT(world != NULL);
    if (world == NULL || world->state == NULL) {
        return;
    }
    free(world->state->arena);
    free(world->state);
    world->state = NULL;
}

bool sl_collider_validate(const sl_collider *desc, sl_collider *out)
{
    if (desc == NULL || !sl_vec3_in_domain(desc->point_a) ||
        !sl_is_finite(desc->friction) || !(desc->friction >= 0.0f)) {
        return false;
    }
    sl_collider c = *desc;
    switch (desc->kind) {
    case SL_COLLIDER_PLANE: {
        if (!sl_vec3_is_finite(desc->point_b)) {
            return false;
        }
        const float length = sl_vec3_length(desc->point_b);
        if (!(length >= SL_EPSILON) || !sl_is_finite(length)) {
            return false;
        }
        c.point_b = sl_vec3_scale(desc->point_b, 1.0f / length);
        c.radius = 0.0f;
        break;
    }
    case SL_COLLIDER_SPHERE:
        c.point_b = sl_vec3_zero();
        if (!sl_is_finite(desc->radius) || !(desc->radius > 0.0f) ||
            desc->radius > SL_POSITION_ABS_MAX) {
            return false;
        }
        break;
    case SL_COLLIDER_CAPSULE:
        if (!sl_vec3_in_domain(desc->point_b) || !sl_is_finite(desc->radius) ||
            !(desc->radius > 0.0f) || desc->radius > SL_POSITION_ABS_MAX) {
            return false;
        }
        break;
    default:
        return false;
    }
    *out = c;
    return true;
}

bool sl_world_collider_add(sl_world *world, const sl_collider *desc,
                           uint32_t *collider)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    SL_ASSERT(collider != NULL);
    sl_world_state *s = world->state;
    sl_collider c;
    if (s->collider_count >= SL_COLLIDER_COUNT_MAX ||
        !sl_collider_validate(desc, &c)) {
        return false;
    }
    s->colliders[s->collider_count] = c;
    *collider = s->collider_count;
    s->collider_count++;
    return true;
}

bool sl_world_collider_set(sl_world *world, uint32_t collider,
                           const sl_collider *desc)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    sl_world_state *s = world->state;
    sl_collider c;
    if (collider >= s->collider_count || !sl_collider_validate(desc, &c)) {
        return false;
    }
    s->colliders[collider] = c;
    return true;
}

bool sl_world_collider_get(const sl_world *world, uint32_t collider,
                           sl_collider *desc)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    SL_ASSERT(desc != NULL);
    if (collider >= world->state->collider_count) {
        return false;
    }
    *desc = world->state->colliders[collider];
    return true;
}

bool sl_world_gravity_set(sl_world *world, sl_vec3 gravity)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    if (!sl_vec3_is_finite(gravity)) {
        return false;
    }
    world->state->gravity = gravity;
    return true;
}

bool sl_world_wind_set(sl_world *world, sl_vec3 wind)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    if (!sl_vec3_is_finite(wind)) {
        return false;
    }
    world->state->wind = wind;
    return true;
}

/* Global vertex index, or UINT32_MAX when out of range. */
static uint32_t sl_vertex_global(const sl_world_state *s, uint32_t cloth,
                                 uint32_t vertex)
{
    if (cloth >= s->cloth_count || vertex >= s->cloths[cloth].vertex_count) {
        return UINT32_MAX;
    }
    return s->cloths[cloth].vertex_offset + vertex;
}

bool sl_world_vertex_pin_set(sl_world *world, uint32_t cloth, uint32_t vertex,
                             bool pinned)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    sl_world_state *s = world->state;
    const uint32_t i = sl_vertex_global(s, cloth, vertex);
    if (i == UINT32_MAX) {
        return false;
    }
    s->pinned[i] = (uint8_t)(pinned ? 1u : 0u);
    s->pin_target[i] = s->position[i];
    if (pinned) {
        s->velocity[i] = sl_vec3_zero();
    }
    return true;
}

bool sl_world_vertex_target_set(sl_world *world, uint32_t cloth,
                                uint32_t vertex, sl_vec3 target)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    sl_world_state *s = world->state;
    const uint32_t i = sl_vertex_global(s, cloth, vertex);
    if (i == UINT32_MAX || s->pinned[i] == 0u || !sl_vec3_in_domain(target)) {
        return false;
    }
    s->pin_target[i] = target;
    return true;
}

const sl_vec3 *sl_world_cloth_positions(const sl_world *world, uint32_t cloth)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    const sl_world_state *s = world->state;
    if (cloth >= s->cloth_count) {
        return NULL;
    }
    return s->position + s->cloths[cloth].vertex_offset;
}

const sl_vec3 *sl_world_cloth_velocities(const sl_world *world, uint32_t cloth)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    const sl_world_state *s = world->state;
    if (cloth >= s->cloth_count) {
        return NULL;
    }
    return s->velocity + s->cloths[cloth].vertex_offset;
}

uint32_t sl_world_cloth_vertex_count(const sl_world *world, uint32_t cloth)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    const sl_world_state *s = world->state;
    return (cloth < s->cloth_count) ? s->cloths[cloth].vertex_count : 0u;
}

bool sl_world_vertex_is_pinned(const sl_world *world, uint32_t cloth,
                               uint32_t vertex)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    const uint32_t i = sl_vertex_global(world->state, cloth, vertex);
    return i != UINT32_MAX && world->state->pinned[i] != 0u;
}

sl_world_stats sl_world_stats_get(const sl_world *world)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    const sl_world_state *s = world->state;
    sl_world_stats stats;
    memset(&stats, 0, sizeof(stats));
    stats.vertex_count = s->vertex_count;
    stats.vertex_capacity = s->vertex_capacity;
    stats.triangle_count = s->triangle_count;
    stats.triangle_capacity = s->triangle_capacity;
    stats.bend_count = s->bend_count;
    stats.cloth_count = s->cloth_count;
    stats.cloth_capacity = s->cloth_capacity;
    stats.collider_count = s->collider_count;
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        if (s->cloths[c].color_count > stats.color_count_max) {
            stats.color_count_max = s->cloths[c].color_count;
        }
    }
    stats.contact_count = s->contact_count;
    stats.substep_count = s->substep_count_last;
    stats.substep_limited = s->substep_limited;
    stats.stiffness_rate_max = s->stiffness_rate_max;
    stats.self_contact_count = s->pair_count;
    stats.self_contact_capacity = s->pair_capacity;
    stats.self_detection_count = s->self_detections;
    stats.self_contact_dropped = s->self_dropped;
    stats.self_bound_hits = s->self_bound_hits;
    stats.arena_bytes = s->arena_bytes;
    return stats;
}
