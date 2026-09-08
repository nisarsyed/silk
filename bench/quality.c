#include "quality.h"
#include <float.h>
#include <math.h>
#include <string.h>

static void projection(const sl_shape *shape, sl_transform transform,
                       sl_vec2 axis, float *low, float *high)
{
    if (shape->kind == SL_SHAPE_CIRCLE) {
        const float center = sl_vec2_dot(transform.position, axis);
        *low = center - shape->circle.radius;
        *high = center + shape->circle.radius;
        return;
    }
    *low = FLT_MAX;
    *high = -FLT_MAX;
    for (uint32_t i = 0u; i < shape->polygon.count; ++i) {
        const float p = sl_vec2_dot(
            sl_transform_apply(transform, shape->polygon.vertices[i]), axis);
        *low = sl_min(*low, p);
        *high = sl_max(*high, p);
    }
}
static float axis_overlap(const sl_shape *a, sl_transform ta, const sl_shape *b,
                          sl_transform tb, sl_vec2 axis)
{
    const float length = sl_vec2_length(axis);
    if (length <= SL_EPSILON) {
        return FLT_MAX;
    }
    axis = sl_vec2_scale(axis, 1.0f / length);
    float a_low, a_high, b_low, b_high;
    projection(a, ta, axis, &a_low, &a_high);
    projection(b, tb, axis, &b_low, &b_high);
    return sl_min(a_high - b_low, b_high - a_low);
}
float sl_bench_penetration(const sl_shape *a, sl_transform ta,
                           const sl_shape *b, sl_transform tb)
{
    if (a->kind == SL_SHAPE_NONE || b->kind == SL_SHAPE_NONE) {
        return 0.0f;
    }
    if (a->kind == SL_SHAPE_CIRCLE && b->kind == SL_SHAPE_CIRCLE) {
        return sl_max(
            0.0f, a->circle.radius + b->circle.radius -
                      sl_vec2_length(sl_vec2_sub(ta.position, tb.position)));
    }
    float overlap = FLT_MAX;
    const sl_shape *shapes[2] = { a, b };
    const sl_transform transforms[2] = { ta, tb };
    for (uint32_t side = 0u; side < 2u; ++side) {
        const sl_shape *poly = shapes[side];
        if (poly->kind != SL_SHAPE_POLYGON) {
            continue;
        }
        const sl_transform t = transforms[side];
        float closest_sq = FLT_MAX;
        sl_vec2 closest = { 0 };
        for (uint32_t i = 0u; i < poly->polygon.count; ++i) {
            const sl_vec2 v = sl_transform_apply(t, poly->polygon.vertices[i]);
            const sl_vec2 next = sl_transform_apply(
                t, poly->polygon.vertices[(i + 1u) % poly->polygon.count]);
            overlap =
                sl_min(overlap,
                       axis_overlap(a, ta, b, tb,
                                    sl_vec2_right_perp(sl_vec2_sub(next, v))));
            const sl_vec2 delta =
                sl_vec2_sub(v, transforms[1u - side].position);
            const float distance_sq = sl_vec2_dot(delta, delta);
            if (distance_sq < closest_sq) {
                closest_sq = distance_sq;
                closest = delta;
            }
        }
        if (shapes[1u - side]->kind == SL_SHAPE_CIRCLE) {
            overlap = sl_min(overlap, axis_overlap(a, ta, b, tb, closest));
        }
    }
    return sl_max(0.0f, overlap);
}
static uint64_t integer_hash(uint64_t hash, uint32_t value)
{
    return (hash ^ value) * UINT64_C(1099511628211);
}
static uint64_t float_hash(uint64_t hash, float value)
{
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    return integer_hash(hash, bits);
}
static uint64_t vector_hash(uint64_t hash, sl_vec2 value)
{
    return float_hash(float_hash(hash, value.x), value.y);
}
static uint64_t handle_hash(uint64_t hash, sl_body_handle handle)
{
    return integer_hash(integer_hash(hash, handle.index), handle.generation);
}
uint64_t sl_bench_digest(const sl_world *world, const sl_body_handle *bodies,
                         uint32_t count)
{
    uint64_t hash = integer_hash(UINT64_C(14695981039346656037), count);
    for (uint32_t i = 0u; i < count; ++i) {
        const sl_body_handle body = bodies[i];
        hash = handle_hash(hash, body);
        const sl_transform t = sl_world_body_get_transform(world, body);
        hash = vector_hash(hash, t.position);
        hash = float_hash(float_hash(hash, t.rotation.c), t.rotation.s);
        hash = vector_hash(hash, sl_world_body_get_velocity(world, body));
        hash =
            float_hash(hash, sl_world_body_get_angular_velocity(world, body));
        hash = float_hash(hash, sl_world_body_get_mass(world, body));
        hash = float_hash(hash, sl_world_body_get_inertia(world, body));
        hash = float_hash(hash, sl_world_body_get_inv_mass(world, body));
        hash = float_hash(hash, sl_world_body_get_inv_inertia(world, body));
        hash = float_hash(hash, sl_world_body_get_torque(world, body));
        sl_aabb bounds;
        const bool proxy = sl_world_body_get_proxy_aabb(world, body, &bounds);
        hash = integer_hash(hash, proxy ? 1u : 0u);
        if (proxy) {
            hash = vector_hash(vector_hash(hash, bounds.lower), bounds.upper);
        }

        hash = float_hash(hash, sl_world_body_get_friction(world, body));
        hash = float_hash(hash, sl_world_body_get_restitution(world, body));
        hash =
            integer_hash(hash, (uint32_t)sl_world_body_get_type(world, body));
        const sl_shape *shape = sl_world_body_get_shape(world, body);
        hash = integer_hash(hash, (uint32_t)shape->kind);
        if (shape->kind == SL_SHAPE_CIRCLE) {
            hash = float_hash(hash, shape->circle.radius);
        }
        if (shape->kind == SL_SHAPE_POLYGON) {
            hash = integer_hash(hash, shape->polygon.count);
            for (uint32_t j = 0u; j < shape->polygon.count; ++j) {
                hash = vector_hash(hash, shape->polygon.vertices[j]);
            }
        }
    }
    hash = integer_hash(hash, sl_world_contact_count(world));
    for (uint32_t i = 0u; i < sl_world_contact_count(world); ++i) {
        const sl_contact *c = sl_world_contact_at(world, i);
        hash = handle_hash(handle_hash(hash, c->body_a), c->body_b);
        hash = float_hash(float_hash(hash, c->friction), c->restitution);
        hash = integer_hash(hash, c->touching ? 1u : 0u);
        hash = vector_hash(hash, c->manifold.normal);
        hash = integer_hash(hash, c->manifold.point_count);
        for (uint32_t j = 0u; j < c->manifold.point_count; ++j) {
            const sl_manifold_point *p = &c->manifold.points[j];
            hash = vector_hash(
                vector_hash(vector_hash(hash, p->anchor_a), p->anchor_b),
                p->point);
            hash =
                float_hash(float_hash(hash, p->separation), p->normal_velocity);
            hash = float_hash(float_hash(hash, p->normal_impulse),
                              p->tangent_impulse);
            hash =
                integer_hash(integer_hash(hash, p->id), p->persisted ? 1u : 0u);
        }
    }
    hash = integer_hash(hash, sl_world_joint_count(world));
    for (uint32_t i = 0u; i < sl_world_joint_count(world); ++i) {
        const sl_joint_handle h = sl_world_joint_at(world, i);
        const sl_joint_desc d = sl_world_joint_get_desc(world, h);
        hash = integer_hash(integer_hash(hash, h.index), h.generation);
        hash = integer_hash(hash, (uint32_t)d.kind);
        hash = handle_hash(handle_hash(hash, d.body_a), d.body_b);
        hash =
            vector_hash(vector_hash(hash, d.local_anchor_a), d.local_anchor_b);
        hash = integer_hash(hash, d.collide_connected ? 1u : 0u);
        if (d.kind == SL_JOINT_DISTANCE) {
            hash = float_hash(hash, d.distance.length);
        }
        hash = vector_hash(hash, sl_world_joint_get_linear_impulse(world, h));
    }
    return hash;
}
