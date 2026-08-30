#ifndef SILK_SHAPE_H
#define SILK_SHAPE_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Matches Box2D's polygon cap. Bounds every per-edge loop (containment,
 * ray cast, later SAT) and fixes the shape record size, keeping the
 * world's one-shape-per-body array inside its memory budget at
 * SL_BODY_COUNT_MAX; anything rounder than an octagon is a circle. */
#define SL_POLYGON_VERTEX_COUNT_MAX 8u

/* Maximum radius from a shape's centroid, in world length units. Silk is
 * tuned for ordinary body extents around 0.1-10 units; this cap leaves
 * two orders of magnitude for exceptional geometry while bounding every
 * area and inertia derivation far below float overflow. Together with
 * SL_POSITION_ABS_MAX it also bounds future contact-coordinate deltas.
 * Pinned by tests/test_shape.c. */
#define SL_SHAPE_EXTENT_MAX 1024.0f

/* "kind", not "type": body type (static/kinematic/dynamic) is a world
 * concern. Zero is the shapeless body -- a point particle -- so a
 * zeroed record is a valid NONE. */
typedef enum sl_shape_kind {
    SL_SHAPE_NONE = 0,
    SL_SHAPE_CIRCLE,
    SL_SHAPE_POLYGON
} sl_shape_kind;

/* Centered on the body origin; radius in
 * (SL_EPSILON, SL_SHAPE_EXTENT_MAX]. */
typedef struct sl_circle {
    float radius;
} sl_circle;

/* CCW, strictly convex at every vertex, centroid at the body origin --
 * sl_shape_make_polygon establishes all three; treat fields read-only.
 * Edge i runs vertices[i] -> vertices[(i + 1) % count]; its outward
 * normal direction is (e.y, -e.x) for edge vector e. Vertices are
 * body-local. */
typedef struct sl_polygon {
    uint32_t count; /* in [3, SL_POLYGON_VERTEX_COUNT_MAX] */
    sl_vec2 vertices[SL_POLYGON_VERTEX_COUNT_MAX];
} sl_polygon;

/* Tagged record stored by value in the world's per-body array. The
 * world rebuilds validated input from its active fields so inactive
 * payload bytes stay normalized. Supported ABIs must lay this public record
 * out in exactly 72 bytes; shape.c deliberately rejects any other size at
 * compile time because layout drift changes both the by-value ABI and the
 * world's fixed memory budget. Consumers always want the whole shape, so a
 * per-row record is the natural granularity. Renderers: circle ->
 * transform.position +- circle.radius; polygon ->
 * sl_transform_apply(transform, vertices[i]). */
typedef struct sl_shape {
    sl_shape_kind kind;
    union {
        sl_circle circle;
        sl_polygon polygon;
    };
} sl_shape;

/* Mass-independent properties; the world derives inertia = mass *
 * inertia_per_unit_mass about the body origin, which constructed shapes
 * keep at the centroid. NONE yields all zeros. */
typedef struct sl_mass_data {
    float area; /* length^2 */
    /* Body-local; zero within rounding for constructed shapes. */
    sl_vec2 centroid;
    float inertia_per_unit_mass; /* I / m about the centroid, length^2 */
} sl_mass_data;

/* Segment origin + f * translation with f in [0, 1]. translation is not
 * a unit direction: pass (to - from) as is, with length > SL_EPSILON.
 * World frame. */
typedef struct sl_ray {
    sl_vec2 origin;
    sl_vec2 translation;
} sl_ray;

/* Hit at origin + fraction * translation; normal unit, outward, world
 * frame. Fields are meaningful only when the cast returns true. */
typedef struct sl_ray_hit {
    float fraction;
    sl_vec2 point;
    sl_vec2 normal;
} sl_ray_hit;

/* The shapeless shape: kind NONE with the payload zeroed. Every byte,
 * not just the live union member, so byte-wise comparison, hashing, and
 * serialization of shape records stay deterministic. The world uses
 * this same representation for point particles. sl_shape has no padding
 * (see the layout assert in shape.c), so the initializer covers the
 * whole record. */
static inline sl_shape sl_shape_none(void)
{
    sl_shape s = { 0 };
    s.kind = SL_SHAPE_NONE;
    return s;
}

/* Constructors return false -- leaving *out untouched -- on rejection;
 * contracts on each function below. */
bool sl_shape_make_circle(float radius, sl_shape *out);
bool sl_shape_make_polygon(const sl_vec2 *points, uint32_t count,
                           sl_shape *out);
bool sl_shape_make_box(float half_width, float half_height, sl_shape *out);

/* Structural check for a record from any source: NONE is valid, every
 * constructed shape passes, tampered records fail. The world gates
 * shape attachment on it. */
bool sl_shape_is_valid(const sl_shape *shape);

/* Requires a valid shape. */
sl_mass_data sl_shape_mass_data(const sl_shape *shape);

/* World-frame bounds; NONE degenerates to {position, position}. */
sl_aabb sl_shape_aabb(const sl_shape *shape, sl_transform transform);

/* Boundary inclusive; NONE contains nothing; non-finite points are
 * outside. */
bool sl_shape_contains_point(const sl_shape *shape, sl_transform transform,
                             sl_vec2 point);

/* False -- leaving *hit untouched -- for malformed rays, NONE, an origin
 * inside or on the boundary, and misses; grazing counts as a hit.
 * fraction lands in [0, 1]. */
bool sl_shape_ray_cast(const sl_shape *shape, sl_transform transform,
                       sl_ray ray, sl_ray_hit *hit);

#ifdef __cplusplus
}
#endif

#endif /* SILK_SHAPE_H */
