#include "collide.h"

#include <float.h>
#include <string.h>

#include "silk/assert.h"

typedef enum sl_feature_kind {
    SL_FEATURE_CIRCLE = 0u,
    SL_FEATURE_VERTEX = 1u,
    SL_FEATURE_EDGE = 2u,
} sl_feature_kind;

typedef struct sl_world_polygon {
    uint32_t count;
    sl_vec2 vertices[SL_POLYGON_VERTEX_COUNT_MAX];
    sl_vec2 normals[SL_POLYGON_VERTEX_COUNT_MAX];
} sl_world_polygon;

typedef struct sl_face_separation {
    float separation;
    uint32_t edge;
} sl_face_separation;

typedef struct sl_clip_vertex {
    sl_vec2 point;
    uint8_t feature_ref;
    uint8_t feature_inc;
} sl_clip_vertex;

static uint8_t sl_feature_make(sl_feature_kind kind, uint32_t index)
{
    SL_ASSERT(index < SL_POLYGON_VERTEX_COUNT_MAX);
    return (uint8_t)(((uint32_t)kind << 4u) | index);
}

static uint32_t sl_feature_pair(uint8_t feature_a, uint8_t feature_b)
{
    return ((uint32_t)feature_a << 8u) | (uint32_t)feature_b;
}

static uint32_t sl_feature_pair_flip(uint32_t id)
{
    return ((id & 0xffu) << 8u) | ((id >> 8u) & 0xffu);
}

static sl_manifold sl_manifold_empty(void)
{
    sl_manifold manifold;
    memset(&manifold, 0, sizeof(manifold));
    return manifold;
}

static sl_manifold_point sl_manifold_point_make(sl_vec2 point_a,
                                                sl_vec2 point_b,
                                                sl_transform transform_a,
                                                sl_transform transform_b,
                                                sl_vec2 normal, uint32_t id)
{
    sl_manifold_point point;
    memset(&point, 0, sizeof(point));
    point.anchor_a = sl_vec2_sub(point_a, transform_a.position);
    point.anchor_b = sl_vec2_sub(point_b, transform_b.position);
    point.point = sl_vec2_scale(sl_vec2_add(point_a, point_b), 0.5f);
    point.separation = sl_vec2_dot(sl_vec2_sub(point_b, point_a), normal);
    point.id = id;
    return point;
}

static sl_manifold sl_manifold_flip(sl_manifold manifold)
{
    manifold.normal = sl_vec2_neg(manifold.normal);
    for (uint32_t i = 0u; i < manifold.point_count; ++i) {
        const sl_vec2 anchor_a = manifold.points[i].anchor_a;
        manifold.points[i].anchor_a = manifold.points[i].anchor_b;
        manifold.points[i].anchor_b = anchor_a;
        manifold.points[i].id = sl_feature_pair_flip(manifold.points[i].id);
    }
    return manifold;
}

static sl_world_polygon sl_polygon_to_world(const sl_polygon *polygon,
                                            sl_transform transform)
{
    sl_world_polygon world;
    world.count = polygon->count;
    for (uint32_t i = 0u; i < polygon->count; ++i) {
        const uint32_t next = (i + 1u) % polygon->count;
        world.vertices[i] = sl_transform_apply(transform, polygon->vertices[i]);
        const sl_vec2 edge =
            sl_vec2_sub(polygon->vertices[next], polygon->vertices[i]);
        world.normals[i] = sl_rotation_apply(
            transform.rotation, sl_vec2_normalize(sl_vec2_right_perp(edge)));
    }
    return world;
}

static sl_face_separation
sl_polygon_separation(const sl_world_polygon *reference,
                      const sl_world_polygon *other)
{
    sl_face_separation result = { -FLT_MAX, 0u };
    for (uint32_t i = 0u; i < reference->count; ++i) {
        float separation = FLT_MAX;
        for (uint32_t j = 0u; j < other->count; ++j) {
            const float candidate = sl_vec2_dot(
                reference->normals[i],
                sl_vec2_sub(other->vertices[j], reference->vertices[i]));
            separation = sl_min(separation, candidate);
        }
        if (separation > result.separation) {
            result.separation = separation;
            result.edge = i;
        }
    }
    return result;
}

static uint32_t sl_incident_edge(const sl_world_polygon *polygon,
                                 sl_vec2 reference_normal)
{
    float minimum_dot = FLT_MAX;
    uint32_t edge = 0u;
    for (uint32_t i = 0u; i < polygon->count; ++i) {
        const float candidate =
            sl_vec2_dot(reference_normal, polygon->normals[i]);
        if (candidate < minimum_dot) {
            minimum_dot = candidate;
            edge = i;
        }
    }
    return edge;
}

/* Keep dot(normal, point) >= offset. The two-input/two-output bound is
 * fixed; equality remains inside so touching endpoints are retained. */
static uint32_t sl_clip_lower(const sl_clip_vertex input[2], uint32_t count,
                              sl_vec2 normal, float offset,
                              uint8_t reference_vertex, uint8_t incident_edge,
                              sl_clip_vertex output[2])
{
    SL_ASSERT(count <= 2u);
    if (count < 2u) {
        if (count == 1u && sl_vec2_dot(normal, input[0].point) >= offset) {
            output[0] = input[0];
            return 1u;
        }
        return 0u;
    }

    const float distance_0 = sl_vec2_dot(normal, input[0].point) - offset;
    const float distance_1 = sl_vec2_dot(normal, input[1].point) - offset;
    uint32_t output_count = 0u;
    if (distance_0 >= 0.0f) {
        output[output_count++] = input[0];
    }
    if ((distance_0 < 0.0f && distance_1 >= 0.0f) ||
        (distance_1 < 0.0f && distance_0 >= 0.0f)) {
        const float fraction = distance_0 / (distance_0 - distance_1);
        output[output_count].point =
            sl_vec2_lerp(input[0].point, input[1].point, fraction);
        output[output_count].feature_ref = reference_vertex;
        output[output_count].feature_inc = incident_edge;
        output_count += 1u;
    }
    if (distance_1 >= 0.0f && output_count < 2u) {
        output[output_count++] = input[1];
    }
    return output_count;
}

static uint8_t sl_segment_feature(float fraction, uint32_t edge, uint32_t count)
{
    if (fraction <= SL_EPSILON) {
        return sl_feature_make(SL_FEATURE_VERTEX, edge);
    }
    if (fraction >= 1.0f - SL_EPSILON) {
        return sl_feature_make(SL_FEATURE_VERTEX, (edge + 1u) % count);
    }
    return sl_feature_make(SL_FEATURE_EDGE, edge);
}

static sl_manifold sl_collide_circles(const sl_circle *circle_a,
                                      sl_transform transform_a,
                                      const sl_circle *circle_b,
                                      sl_transform transform_b)
{
    sl_manifold manifold = sl_manifold_empty();
    const sl_vec2 delta =
        sl_vec2_sub(transform_b.position, transform_a.position);
    const float distance_sq = sl_vec2_length_sq(delta);
    const float radius = circle_a->radius + circle_b->radius;
    const float limit = radius + SL_SPECULATIVE_DISTANCE;
    if (distance_sq > limit * limit) {
        return manifold;
    }

    const float distance = sqrtf(distance_sq);
    const sl_vec2 normal = (distance > SL_EPSILON)
                               ? sl_vec2_scale(delta, 1.0f / distance)
                               : sl_vec2_make(1.0f, 0.0f);
    const sl_vec2 point_a = sl_vec2_add(
        transform_a.position, sl_vec2_scale(normal, circle_a->radius));
    const sl_vec2 point_b = sl_vec2_sub(
        transform_b.position, sl_vec2_scale(normal, circle_b->radius));
    manifold.normal = normal;
    manifold.point_count = 1u;
    manifold.points[0] = sl_manifold_point_make(
        point_a, point_b, transform_a, transform_b, normal,
        sl_feature_pair(sl_feature_make(SL_FEATURE_CIRCLE, 0u),
                        sl_feature_make(SL_FEATURE_CIRCLE, 0u)));
    return manifold;
}

static sl_manifold sl_collide_polygon_circle(const sl_polygon *polygon,
                                             sl_transform transform_a,
                                             const sl_circle *circle,
                                             sl_transform transform_b)
{
    sl_manifold manifold = sl_manifold_empty();
    const sl_world_polygon world = sl_polygon_to_world(polygon, transform_a);
    const sl_vec2 center = transform_b.position;
    float maximum_separation = -FLT_MAX;
    uint32_t face = 0u;
    for (uint32_t i = 0u; i < world.count; ++i) {
        const float separation = sl_vec2_dot(
            world.normals[i], sl_vec2_sub(center, world.vertices[i]));
        if (separation > maximum_separation) {
            maximum_separation = separation;
            face = i;
        }
    }
    if (maximum_separation > circle->radius + SL_SPECULATIVE_DISTANCE) {
        return manifold;
    }

    const uint32_t next = (face + 1u) % world.count;
    const sl_vec2 vertex_1 = world.vertices[face];
    const sl_vec2 vertex_2 = world.vertices[next];
    const sl_vec2 edge = sl_vec2_sub(vertex_2, vertex_1);
    const float edge_length_sq = sl_vec2_length_sq(edge);
    SL_ASSERT(edge_length_sq > SL_VEC2_LENGTH_EPS_SQ);
    const float fraction = sl_clamp(
        sl_vec2_dot(sl_vec2_sub(center, vertex_1), edge) / edge_length_sq, 0.0f,
        1.0f);

    sl_vec2 normal = world.normals[face];
    sl_vec2 point_a;
    uint8_t feature_a;
    if (maximum_separation <= 0.0f) {
        point_a =
            sl_vec2_sub(center, sl_vec2_scale(normal, maximum_separation));
        feature_a = sl_feature_make(SL_FEATURE_EDGE, face);
    } else {
        point_a = sl_vec2_lerp(vertex_1, vertex_2, fraction);
        const sl_vec2 delta = sl_vec2_sub(center, point_a);
        const float distance_sq = sl_vec2_length_sq(delta);
        const float limit = circle->radius + SL_SPECULATIVE_DISTANCE;
        if (distance_sq > limit * limit) {
            return manifold;
        }
        const float distance = sqrtf(distance_sq);
        if (distance > SL_EPSILON) {
            normal = sl_vec2_scale(delta, 1.0f / distance);
        }
        feature_a = sl_segment_feature(fraction, face, world.count);
    }

    const sl_vec2 point_b =
        sl_vec2_sub(center, sl_vec2_scale(normal, circle->radius));
    const float separation = sl_vec2_dot(sl_vec2_sub(point_b, point_a), normal);
    if (separation > SL_SPECULATIVE_DISTANCE) {
        return manifold;
    }
    manifold.normal = normal;
    manifold.point_count = 1u;
    manifold.points[0] = sl_manifold_point_make(
        point_a, point_b, transform_a, transform_b, normal,
        sl_feature_pair(feature_a, sl_feature_make(SL_FEATURE_CIRCLE, 0u)));
    return manifold;
}

static sl_manifold sl_collide_polygon_closest(const sl_world_polygon *world_a,
                                              sl_transform transform_a,
                                              const sl_world_polygon *world_b,
                                              sl_transform transform_b,
                                              sl_vec2 fallback_normal)
{
    sl_manifold manifold = sl_manifold_empty();
    sl_segment_distance_result best;
    memset(&best, 0, sizeof(best));
    float best_distance_sq = FLT_MAX;
    uint32_t best_edge_a = 0u;
    uint32_t best_edge_b = 0u;

    for (uint32_t i = 0u; i < world_a->count; ++i) {
        const uint32_t next_a = (i + 1u) % world_a->count;
        for (uint32_t j = 0u; j < world_b->count; ++j) {
            const uint32_t next_b = (j + 1u) % world_b->count;
            sl_segment_distance_result candidate;
            const bool valid = sl_segment_distance(
                world_a->vertices[i], world_a->vertices[next_a],
                world_b->vertices[j], world_b->vertices[next_b], &candidate);
            SL_ASSERT(valid);
            if (valid && candidate.distance_sq < best_distance_sq) {
                best = candidate;
                best_distance_sq = candidate.distance_sq;
                best_edge_a = i;
                best_edge_b = j;
            }
        }
    }

    if (best_distance_sq > SL_SPECULATIVE_DISTANCE * SL_SPECULATIVE_DISTANCE) {
        return manifold;
    }
    const float distance = sqrtf(best_distance_sq);
    const sl_vec2 normal =
        (distance > SL_EPSILON)
            ? sl_vec2_scale(sl_vec2_sub(best.point_b, best.point_a),
                            1.0f / distance)
            : fallback_normal;
    manifold.normal = normal;
    manifold.point_count = 1u;
    manifold.points[0] = sl_manifold_point_make(
        best.point_a, best.point_b, transform_a, transform_b, normal,
        sl_feature_pair(
            sl_segment_feature(best.fraction_a, best_edge_a, world_a->count),
            sl_segment_feature(best.fraction_b, best_edge_b, world_b->count)));
    return manifold;
}

static sl_manifold sl_collide_polygon_faces(const sl_world_polygon *reference,
                                            sl_transform transform_ref,
                                            uint32_t reference_edge,
                                            const sl_world_polygon *incident,
                                            sl_transform transform_inc)
{
    sl_manifold manifold = sl_manifold_empty();
    const uint32_t reference_next = (reference_edge + 1u) % reference->count;
    const sl_vec2 vertex_1 = reference->vertices[reference_edge];
    const sl_vec2 vertex_2 = reference->vertices[reference_next];
    const sl_vec2 normal = reference->normals[reference_edge];
    const sl_vec2 tangent = sl_vec2_normalize(sl_vec2_sub(vertex_2, vertex_1));
    const uint32_t incident_edge = sl_incident_edge(incident, normal);
    const uint32_t incident_next = (incident_edge + 1u) % incident->count;

    sl_clip_vertex input[2] = {
        { incident->vertices[incident_edge],
          sl_feature_make(SL_FEATURE_EDGE, reference_edge),
          sl_feature_make(SL_FEATURE_VERTEX, incident_edge) },
        { incident->vertices[incident_next],
          sl_feature_make(SL_FEATURE_EDGE, reference_edge),
          sl_feature_make(SL_FEATURE_VERTEX, incident_next) },
    };
    sl_clip_vertex clipped_1[2];
    sl_clip_vertex clipped_2[2];
    uint32_t count = sl_clip_lower(
        input, 2u, tangent, sl_vec2_dot(tangent, vertex_1),
        sl_feature_make(SL_FEATURE_VERTEX, reference_edge),
        sl_feature_make(SL_FEATURE_EDGE, incident_edge), clipped_1);
    count = sl_clip_lower(
        clipped_1, count, sl_vec2_neg(tangent), -sl_vec2_dot(tangent, vertex_2),
        sl_feature_make(SL_FEATURE_VERTEX, reference_next),
        sl_feature_make(SL_FEATURE_EDGE, incident_edge), clipped_2);

    manifold.normal = normal;
    for (uint32_t i = 0u; i < count; ++i) {
        const float separation =
            sl_vec2_dot(normal, sl_vec2_sub(clipped_2[i].point, vertex_1));
        if (separation <= SL_SPECULATIVE_DISTANCE) {
            const sl_vec2 point_inc = clipped_2[i].point;
            const sl_vec2 point_ref =
                sl_vec2_sub(point_inc, sl_vec2_scale(normal, separation));
            manifold.points[manifold.point_count++] = sl_manifold_point_make(
                point_ref, point_inc, transform_ref, transform_inc, normal,
                sl_feature_pair(clipped_2[i].feature_ref,
                                clipped_2[i].feature_inc));
        }
    }
    if (manifold.point_count == 0u) {
        manifold.normal = sl_vec2_make(0.0f, 0.0f);
    }
    return manifold;
}

static sl_manifold sl_collide_polygons(const sl_polygon *polygon_a,
                                       sl_transform transform_a,
                                       const sl_polygon *polygon_b,
                                       sl_transform transform_b)
{
    const sl_world_polygon world_a =
        sl_polygon_to_world(polygon_a, transform_a);
    const sl_world_polygon world_b =
        sl_polygon_to_world(polygon_b, transform_b);
    const sl_face_separation separation_a =
        sl_polygon_separation(&world_a, &world_b);
    const sl_face_separation separation_b =
        sl_polygon_separation(&world_b, &world_a);
    const float maximum_separation =
        sl_max(separation_a.separation, separation_b.separation);
    if (maximum_separation > SL_SPECULATIVE_DISTANCE) {
        return sl_manifold_empty();
    }

    const bool reference_is_a =
        separation_a.separation >= separation_b.separation;
    const sl_vec2 fallback_normal =
        reference_is_a ? world_a.normals[separation_a.edge]
                       : sl_vec2_neg(world_b.normals[separation_b.edge]);
    if (maximum_separation > 0.1f * SL_LINEAR_SLOP) {
        return sl_collide_polygon_closest(&world_a, transform_a, &world_b,
                                          transform_b, fallback_normal);
    }

    if (reference_is_a) {
        return sl_collide_polygon_faces(
            &world_a, transform_a, separation_a.edge, &world_b, transform_b);
    }
    return sl_manifold_flip(sl_collide_polygon_faces(
        &world_b, transform_b, separation_b.edge, &world_a, transform_a));
}

sl_manifold sl_collide_shapes(const sl_shape *shape_a, sl_transform transform_a,
                              const sl_shape *shape_b, sl_transform transform_b)
{
    SL_ASSERT(shape_a != NULL);
    SL_ASSERT(shape_b != NULL);
    SL_ASSERT(sl_shape_is_valid(shape_a));
    SL_ASSERT(sl_shape_is_valid(shape_b));
    SL_ASSERT(sl_transform_is_finite(transform_a));
    SL_ASSERT(sl_transform_is_finite(transform_b));

    if (shape_a->kind == SL_SHAPE_NONE || shape_b->kind == SL_SHAPE_NONE) {
        return sl_manifold_empty();
    }
    if (shape_a->kind == SL_SHAPE_CIRCLE && shape_b->kind == SL_SHAPE_CIRCLE) {
        return sl_collide_circles(&shape_a->circle, transform_a,
                                  &shape_b->circle, transform_b);
    }
    if (shape_a->kind == SL_SHAPE_POLYGON && shape_b->kind == SL_SHAPE_CIRCLE) {
        return sl_collide_polygon_circle(&shape_a->polygon, transform_a,
                                         &shape_b->circle, transform_b);
    }
    if (shape_a->kind == SL_SHAPE_CIRCLE && shape_b->kind == SL_SHAPE_POLYGON) {
        return sl_manifold_flip(sl_collide_polygon_circle(
            &shape_b->polygon, transform_b, &shape_a->circle, transform_a));
    }
    SL_ASSERT(shape_a->kind == SL_SHAPE_POLYGON);
    SL_ASSERT(shape_b->kind == SL_SHAPE_POLYGON);
    return sl_collide_polygons(&shape_a->polygon, transform_a,
                               &shape_b->polygon, transform_b);
}
