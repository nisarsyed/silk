#include "silk/shape.h"

#include <math.h>
#include <stddef.h>

/* enum (4 B) + union { circle 4 B, polygon 4 + 8*8 = 68 B }: the record
 * the world's per-body array is sized against; a drift here changes the
 * engine's memory budget. */
_Static_assert(sizeof(sl_shape) == 72u, "sl_shape layout drifted");

static bool circle_radius_valid(float radius)
{
    return sl_is_finite(radius) && radius > SL_EPSILON &&
           radius <= SL_SHAPE_EXTENT_MAX;
}

static sl_vec2 edge_between(const sl_vec2 *vertices, uint32_t count, uint32_t i)
{
    const uint32_t next = (i + 1u == count) ? 0u : i + 1u;
    return sl_vec2_sub(vertices[next], vertices[i]);
}

/* Signed triangle fan about ref. With D_i = cross(v_i - ref,
 * v_{i+1} - ref): area = sum(D)/2; centroid = ref + sum(D*(e1 + e2)) /
 * (3*sum(D)); inertia about ref = sum(D*(|e1|^2 + e1.e2 + |e2|^2))/12.
 * All sums share one sign convention: CCW input yields positive area.
 * Callers derive centroidal values via the parallel axis theorem. */
static void polygon_mass_about(const sl_vec2 *vertices, uint32_t count,
                               sl_vec2 ref, float *area, sl_vec2 *centroid,
                               float *inertia_about_ref)
{
    float area_twice = 0.0f;
    sl_vec2 centroid_num = sl_vec2_make(0.0f, 0.0f);
    float inertia_num = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        const uint32_t next = (i + 1u == count) ? 0u : i + 1u;
        const sl_vec2 e1 = sl_vec2_sub(vertices[i], ref);
        const sl_vec2 e2 = sl_vec2_sub(vertices[next], ref);
        const float d = sl_vec2_cross(e1, e2);
        area_twice += d;
        centroid_num =
            sl_vec2_add(centroid_num, sl_vec2_scale(sl_vec2_add(e1, e2), d));
        inertia_num += d * (sl_vec2_dot(e1, e1) + sl_vec2_dot(e1, e2) +
                            sl_vec2_dot(e2, e2));
    }
    *area = area_twice / 2.0f;
    /* centroid_num carries D unhalved, so the divisor is 3*sum(D),
     * not 3*area -- writing 3*area here doubles the offset. */
    *centroid = sl_vec2_add(
        ref, sl_vec2_scale(centroid_num, 1.0f / (3.0f * area_twice)));
    *inertia_about_ref = inertia_num / 12.0f;
}

/* Validation in one pass, so make_polygon and sl_shape_is_valid
 * exercise identical rules:
 *   1. count in [3, SL_POLYGON_VERTEX_COUNT_MAX], before touching points
 *   2. every point finite
 *   3. every edge resolvable: finite length above the normalize cutoff
 *      and at most 2*SL_SHAPE_EXTENT_MAX; the lower bound rules out
 *      near-duplicate vertices and the upper bound is an early numeric
 *      guard implied by rule 7
 *   4. strictly convex CCW turns -- written as !(cross > threshold) so
 *      an overflowing threshold also rejects
 *   5. total turn below 3*pi: a simple convex loop turns exactly 2*pi,
 *      while all-left-turn self-intersections (pentagram windings) turn
 *      2*pi*k for k >= 2; rule 4 bounds each atan2f to (0, pi)
 *   6. positive finite fan area and finite centroid about points[0],
 *      which avoids cancellation for far-from-origin input
 *   7. every vertex within SL_SHAPE_EXTENT_MAX of the centroid
 * On success *centroid carries the fan centroid: sl_shape_make_polygon
 * subtracts it to recenter, sl_shape_is_valid checks a stored record is
 * already centered. */
static bool polygon_check(const sl_vec2 *points, uint32_t count,
                          sl_vec2 *centroid)
{
    if (count < 3u || count > SL_POLYGON_VERTEX_COUNT_MAX) {
        return false;
    }
    for (uint32_t i = 0u; i < count; ++i) {
        if (!sl_vec2_is_finite(points[i])) {
            return false;
        }
    }

    sl_vec2 edges[SL_POLYGON_VERTEX_COUNT_MAX];
    float lengths[SL_POLYGON_VERTEX_COUNT_MAX];
    const float edge_length_max = 2.0f * SL_SHAPE_EXTENT_MAX;
    const float edge_length_max_sq = edge_length_max * edge_length_max;
    for (uint32_t i = 0u; i < count; ++i) {
        edges[i] = edge_between(points, count, i);
        const float len_sq = sl_vec2_length_sq(edges[i]);
        if (!sl_is_finite(len_sq) || len_sq <= SL_VEC2_LENGTH_EPS_SQ ||
            len_sq > edge_length_max_sq) {
            return false;
        }
        lengths[i] = sqrtf(len_sq);
    }

    /* sin of each turn must exceed SL_EPSILON (~8 ULP of 1.0), keeping
     * collinear rounding noise on the rejected side of a real turn. */
    for (uint32_t i = 0u; i < count; ++i) {
        const uint32_t prev = (i == 0u) ? count - 1u : i - 1u;
        const float cross = sl_vec2_cross(edges[prev], edges[i]);
        const float threshold = SL_EPSILON * lengths[prev] * lengths[i];
        if (!(cross > threshold)) {
            return false;
        }
    }

    float total_turn = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        const uint32_t prev = (i == 0u) ? count - 1u : i - 1u;
        total_turn += atan2f(sl_vec2_cross(edges[prev], edges[i]),
                             sl_vec2_dot(edges[prev], edges[i]));
    }
    if (!(total_turn < 3.0f * SL_PI)) {
        return false;
    }

    float area = 0.0f;
    float inertia_about_first = 0.0f;
    *centroid = sl_vec2_make(0.0f, 0.0f);
    polygon_mass_about(points, count, points[0], &area, centroid,
                       &inertia_about_first);
    if (!sl_is_finite(area) || area <= 0.0f || !sl_vec2_is_finite(*centroid)) {
        return false;
    }
    const float extent_max_sq = SL_SHAPE_EXTENT_MAX * SL_SHAPE_EXTENT_MAX;
    for (uint32_t i = 0u; i < count; ++i) {
        const sl_vec2 centered = sl_vec2_sub(points[i], *centroid);
        const float extent_sq = sl_vec2_length_sq(centered);
        if (!sl_is_finite(extent_sq) || extent_sq > extent_max_sq) {
            return false;
        }
    }
    return true;
}

static bool polygon_record_valid(const sl_polygon *polygon)
{
    /* Rules 1-7 on the stored vertices; rule 1 fires before any
     * indexing, so a tampered count cannot read out of bounds. */
    sl_vec2 centroid;
    if (!polygon_check(polygon->vertices, polygon->count, &centroid)) {
        return false;
    }

    /* Rules 1-7 are translation invariant, so they cannot see a record
     * whose geometry is sound but whose centroid was moved off the body
     * origin. Recentering an already-centered polygon shifts by a few
     * ULP-scale terms of the extent at most. */
    float extent = 0.0f;
    for (uint32_t i = 0u; i < polygon->count; ++i) {
        extent = sl_max(extent, sl_abs(polygon->vertices[i].x));
        extent = sl_max(extent, sl_abs(polygon->vertices[i].y));
    }
    const float tolerance = (float)polygon->count * SL_EPSILON * extent;
    return sl_abs(centroid.x) <= tolerance && sl_abs(centroid.y) <= tolerance;
}

/* Rule 8: finite, centroid-centered storage at the body origin. The
 * vertex tail past count is zeroed so two records with the same geometry
 * are also identical byte for byte. */
static bool polygon_build(const sl_vec2 *points, uint32_t count,
                          sl_polygon *out)
{
    sl_vec2 centroid;
    if (!polygon_check(points, count, &centroid)) {
        return false;
    }

    sl_polygon built = { 0 };
    built.count = count;
    for (uint32_t i = 0u; i < count; ++i) {
        built.vertices[i] = sl_vec2_sub(points[i], centroid);
    }

    /* Subtracting a far-from-origin centroid rounds at the input's
     * coarser spacing and can leave an ordinary-scale stored polygon
     * visibly off-center. Re-derive once after the first shift, where
     * coordinates are local and precise, then gate on the one predicate
     * sl_shape_is_valid applies: a degenerate fan leaves residual
     * non-finite, which its rule 2 rejects. Only the residual is wanted
     * here; area and inertia are recomputed per query. */
    float area = 0.0f;
    float inertia_about_origin = 0.0f;
    sl_vec2 residual = sl_vec2_make(0.0f, 0.0f);
    polygon_mass_about(built.vertices, built.count, sl_vec2_make(0.0f, 0.0f),
                       &area, &residual, &inertia_about_origin);
    for (uint32_t i = 0u; i < count; ++i) {
        built.vertices[i] = sl_vec2_sub(built.vertices[i], residual);
    }
    if (!polygon_record_valid(&built)) {
        return false;
    }
    *out = built;
    return true;
}

bool sl_shape_make_circle(float radius, sl_shape *out)
{
    if (out == NULL || !circle_radius_valid(radius)) {
        return false;
    }
    /* Whole-record init: the unused polygon tail of the union must not
     * carry indeterminate bytes into comparisons or serialization. */
    sl_shape s = { 0 };
    s.kind = SL_SHAPE_CIRCLE;
    s.circle.radius = radius;
    *out = s;
    return true;
}

bool sl_shape_make_polygon(const sl_vec2 *points, uint32_t count, sl_shape *out)
{
    if (points == NULL || out == NULL) {
        return false;
    }
    sl_polygon built;
    if (!polygon_build(points, count, &built)) {
        return false;
    }
    sl_shape s = { 0 };
    s.kind = SL_SHAPE_POLYGON;
    s.polygon = built;
    *out = s;
    return true;
}

bool sl_shape_make_box(float half_width, float half_height, sl_shape *out)
{
    /* Canonical CCW corners; every rejection rule (non-finite,
     * degenerate or inverted extents, overflowing geometry) fires
     * inside sl_shape_make_polygon. */
    const sl_vec2 corners[4] = {
        sl_vec2_make(-half_width, -half_height),
        sl_vec2_make(half_width, -half_height),
        sl_vec2_make(half_width, half_height),
        sl_vec2_make(-half_width, half_height),
    };
    return sl_shape_make_polygon(corners, 4u, out);
}

bool sl_shape_is_valid(const sl_shape *shape)
{
    if (shape == NULL) {
        return false;
    }
    switch (shape->kind) {
    case SL_SHAPE_NONE:
        return true;
    case SL_SHAPE_CIRCLE:
        return circle_radius_valid(shape->circle.radius);
    case SL_SHAPE_POLYGON:
        return polygon_record_valid(&shape->polygon);
    default:
        return false;
    }
}

sl_mass_data sl_shape_mass_data(const sl_shape *shape)
{
    SL_ASSERT(shape != NULL);
    sl_mass_data data;
    data.area = 0.0f;
    data.centroid = sl_vec2_make(0.0f, 0.0f);
    data.inertia_per_unit_mass = 0.0f;

    switch (shape->kind) {
    case SL_SHAPE_NONE:
        break;
    case SL_SHAPE_CIRCLE: {
        const float radius = shape->circle.radius;
        data.area = SL_PI * radius * radius;
        data.inertia_per_unit_mass = radius * radius / 2.0f;
        break;
    }
    case SL_SHAPE_POLYGON: {
        const sl_polygon *p = &shape->polygon;
        SL_ASSERT(p->count >= 3u && p->count <= SL_POLYGON_VERTEX_COUNT_MAX);
        float area = 0.0f;
        float inertia_about_origin = 0.0f;
        sl_vec2 centroid = sl_vec2_make(0.0f, 0.0f);
        polygon_mass_about(p->vertices, p->count, sl_vec2_make(0.0f, 0.0f),
                           &area, &centroid, &inertia_about_origin);
        /* Constructed polygons are centroid-centered, so the parallel
         * axis term reduces to area * |centroid|^2 ~ rounding. */
        const float inertia_centroid =
            inertia_about_origin - area * sl_vec2_dot(centroid, centroid);
        data.area = area;
        data.centroid = centroid;
        data.inertia_per_unit_mass = inertia_centroid / area;
        break;
    }
    default:
        SL_ASSERT(false);
        break;
    }
    SL_ASSERT(sl_is_finite(data.area));
    SL_ASSERT(sl_vec2_is_finite(data.centroid));
    SL_ASSERT(sl_is_finite(data.inertia_per_unit_mass));
    return data;
}

sl_aabb sl_shape_aabb(const sl_shape *shape, sl_transform transform)
{
    SL_ASSERT(shape != NULL);
    SL_ASSERT(sl_transform_is_finite(transform));

    sl_aabb box;
    box.lower = sl_vec2_make(0.0f, 0.0f);
    box.upper = sl_vec2_make(0.0f, 0.0f);

    switch (shape->kind) {
    case SL_SHAPE_NONE:
        box.lower = transform.position;
        box.upper = transform.position;
        break;
    case SL_SHAPE_CIRCLE: {
        const sl_vec2 reach =
            sl_vec2_make(shape->circle.radius, shape->circle.radius);
        box.lower = sl_vec2_sub(transform.position, reach);
        box.upper = sl_vec2_add(transform.position, reach);
        break;
    }
    case SL_SHAPE_POLYGON: {
        const sl_polygon *p = &shape->polygon;
        SL_ASSERT(p->count >= 3u && p->count <= SL_POLYGON_VERTEX_COUNT_MAX);
        const sl_vec2 first = sl_transform_apply(transform, p->vertices[0]);
        sl_vec2 lo = first;
        sl_vec2 hi = first;
        for (uint32_t i = 1u; i < p->count; ++i) {
            const sl_vec2 v = sl_transform_apply(transform, p->vertices[i]);
            lo = sl_vec2_min(lo, v);
            hi = sl_vec2_max(hi, v);
        }
        box.lower = lo;
        box.upper = hi;
        break;
    }
    default:
        SL_ASSERT(false);
        break;
    }

    SL_ASSERT(sl_aabb_is_valid(box));
    return box;
}

bool sl_shape_contains_point(const sl_shape *shape, sl_transform transform,
                             sl_vec2 point)
{
    SL_ASSERT(shape != NULL);
    SL_ASSERT(sl_transform_is_finite(transform));

    /* The per-edge tests below would silently ACCEPT NaN comparisons;
     * non-finite points are outside by contract. */
    if (!sl_vec2_is_finite(point)) {
        return false;
    }

    switch (shape->kind) {
    case SL_SHAPE_NONE:
        return false;
    case SL_SHAPE_CIRCLE:
        return sl_vec2_distance_sq(point, transform.position) <=
               shape->circle.radius * shape->circle.radius;
    case SL_SHAPE_POLYGON: {
        const sl_polygon *p = &shape->polygon;
        SL_ASSERT(p->count >= 3u && p->count <= SL_POLYGON_VERTEX_COUNT_MAX);
        const sl_vec2 local = sl_transform_apply_inverse(transform, point);
        for (uint32_t i = 0u; i < p->count; ++i) {
            const sl_vec2 edge = edge_between(p->vertices, p->count, i);
            const sl_vec2 rel = sl_vec2_sub(local, p->vertices[i]);
            /* Interior sits left of every CCW edge. */
            if (sl_vec2_cross(edge, rel) < 0.0f) {
                return false;
            }
        }
        return true;
    }
    default:
        SL_ASSERT(false);
        return false;
    }
}

static bool ray_valid(sl_ray ray)
{
    return sl_vec2_is_finite(ray.origin) &&
           sl_vec2_is_finite(ray.translation) &&
           sl_vec2_length_sq(ray.translation) > SL_VEC2_LENGTH_EPS_SQ;
}

static bool circle_ray_cast(const sl_shape *shape, sl_transform transform,
                            sl_ray ray, sl_ray_hit *hit)
{
    const float radius = shape->circle.radius;
    /* u: ray origin relative to the circle center. */
    const sl_vec2 u = sl_vec2_sub(ray.origin, transform.position);
    const sl_vec2 d = ray.translation;

    const float dd = sl_vec2_dot(d, d);
    const float uc = sl_vec2_dot(u, u) - radius * radius;
    /* The containment pre-check already returned for origins inside or
     * on the boundary, so uc > 0 holds here. */
    if (uc <= 0.0f) {
        return false;
    }
    const float ub = sl_vec2_dot(u, d);
    if (ub >= 0.0f) {
        return false; /* receding: nearest approach lies behind */
    }
    const float discriminant = ub * ub - dd * uc;
    if (discriminant < 0.0f) {
        return false;
    }
    /* uc > 0 and ub < 0 keep the near root in [0, 1] iff it exists. */
    const float fraction = (-ub - sqrtf(discriminant)) / dd;
    if (fraction > 1.0f) {
        return false;
    }
    hit->fraction = fraction;
    hit->point = sl_vec2_add(ray.origin, sl_vec2_scale(d, fraction));
    hit->normal = sl_vec2_normalize(sl_vec2_add(u, sl_vec2_scale(d, fraction)));
    return true;
}

static bool polygon_ray_cast(const sl_shape *shape, sl_transform transform,
                             sl_ray ray, sl_ray_hit *hit)
{
    const sl_polygon *p = &shape->polygon;
    /* Clip in the local frame; the direction rotates without moving. */
    const sl_vec2 origin = sl_transform_apply_inverse(transform, ray.origin);
    const sl_vec2 direction =
        sl_rotation_apply_inverse(transform.rotation, ray.translation);

    /* Unnormalized outward normals keep the crossing ratio
     * scale-invariant; only the final hit normal pays a sqrt. */
    float enter = 0.0f;
    float exit = 1.0f;
    uint32_t enter_index = 0u;
    bool entered = false;

    for (uint32_t i = 0u; i < p->count; ++i) {
        const sl_vec2 edge = edge_between(p->vertices, p->count, i);
        const sl_vec2 normal = sl_vec2_make(edge.y, -edge.x);
        const sl_vec2 rel = sl_vec2_sub(origin, p->vertices[i]);
        const float numerator = sl_vec2_dot(normal, rel);
        const float denominator = sl_vec2_dot(normal, direction);

        if (denominator == 0.0f) {
            /* Interior sits strictly left of every edge (numerator < 0
             * means inside this half-plane). Parallel and inside cannot
             * bound the segment; parallel and outside never crosses. */
            if (numerator >= 0.0f) {
                return false;
            }
            continue;
        }

        const float t = -numerator / denominator;
        if (denominator > 0.0f) {
            /* f(t) increases through zero: leaving the half-plane. */
            exit = sl_min(exit, t);
        } else {
            /* Entering the half-plane. */
            if (t > exit) {
                return false;
            }
            if (!entered || t > enter) {
                enter = t;
                enter_index = i;
                entered = true;
            }
        }
        if (enter > exit) {
            return false;
        }
    }

    if (!entered || enter > 1.0f) {
        return false;
    }

    hit->fraction = enter;
    hit->point = sl_vec2_add(ray.origin, sl_vec2_scale(ray.translation, enter));
    const sl_vec2 edge = edge_between(p->vertices, p->count, enter_index);
    const sl_vec2 local_normal =
        sl_vec2_normalize(sl_vec2_make(edge.y, -edge.x));
    hit->normal = sl_rotation_apply(transform.rotation, local_normal);
    return true;
}

bool sl_shape_ray_cast(const sl_shape *shape, sl_transform transform,
                       sl_ray ray, sl_ray_hit *hit)
{
    SL_ASSERT(shape != NULL);
    SL_ASSERT(hit != NULL);
    SL_ASSERT(sl_transform_is_finite(transform));

    if (!ray_valid(ray)) {
        return false;
    }
    /* Box2D convention: origins inside or on the boundary miss. */
    if (sl_shape_contains_point(shape, transform, ray.origin)) {
        return false;
    }

    switch (shape->kind) {
    case SL_SHAPE_NONE:
        return false;
    case SL_SHAPE_CIRCLE:
        return circle_ray_cast(shape, transform, ray, hit);
    case SL_SHAPE_POLYGON:
        return polygon_ray_cast(shape, transform, ray, hit);
    default:
        SL_ASSERT(false);
        return false;
    }
}
