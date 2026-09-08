#include "silk/query.h"
#include "tree.h"
#include "world_internal.h"

static bool query_valid(const sl_world *world, uint32_t mask)
{
    return world != NULL && world->state != NULL &&
           (mask & ~SL_QUERY_ALL) == 0u;
}

static bool buffer_valid(const sl_world *world, uint32_t mask,
                         const sl_body_handle *bodies, uint32_t capacity,
                         const sl_query_result *result)
{
    return query_valid(world, mask) && result != NULL &&
           capacity <= SL_BODY_COUNT_MAX && (capacity == 0u || bodies != NULL);
}

/* Each body belongs to one root. Combined candidates fit the existing
 * body-capacity buffer; no query instrumentation enters simulation counters. */
static uint32_t candidates(const sl_world_state *world, sl_aabb bounds,
                           uint32_t mask)
{
    const uint32_t bits[SL_TREE_ROOT_COUNT] = { SL_QUERY_STATIC,
                                                SL_QUERY_KINEMATIC,
                                                SL_QUERY_DYNAMIC };
    const uint32_t selected = mask == 0u ? SL_QUERY_ALL : mask;
    uint32_t count = 0u;
    for (uint32_t root = 0u; root < SL_TREE_ROOT_COUNT; ++root) {
        if ((selected & bits[root]) == 0u)
            continue;
        const sl_tree_query_result result = sl_tree_query(
            world->tree, (sl_tree_root_kind)root, bounds,
            world->query_slots + count, world->body_capacity - count);
        SL_ASSERT(!result.overflow);
        SL_ASSERT(result.count <= world->body_capacity - count);
        count += result.count;
    }
    return count;
}

static void swap(uint32_t *a, uint32_t *b)
{
    const uint32_t value = *a;
    *a = *b;
    *b = value;
}

/* Nonrecursive heapsort: O(n log n) worst case, O(1) additional memory. */
static void sift(uint32_t *slots, uint32_t count, uint32_t root)
{
    while (root < count / 2u) {
        uint32_t child = 2u * root + 1u;
        if (child + 1u < count && slots[child] < slots[child + 1u])
            ++child;
        if (slots[root] >= slots[child])
            break;
        swap(&slots[root], &slots[child]);
        root = child;
    }
}
static void slots_sort(uint32_t *slots, uint32_t count)
{
    for (uint32_t i = count / 2u; i > 0u; --i)
        sift(slots, count, i - 1u);
    for (uint32_t i = count; i > 1u; --i) {
        swap(&slots[0], &slots[i - 1u]);
        sift(slots, i - 1u, 0u);
    }
}

static sl_transform body_transform(const sl_world_state *world, uint32_t dense)
{
    const sl_transform transform = { world->positions[dense],
                                     world->rotations[dense] };
    return transform;
}
static sl_body_handle body_handle(const sl_world_state *world, uint32_t slot)
{
    const sl_body_handle handle = { slot, world->slots[slot].generation };
    return handle;
}

static void query_buffer(const sl_world_state *world, sl_aabb bounds,
                         bool point_query, uint32_t mask,
                         sl_body_handle *bodies, uint32_t capacity,
                         sl_query_result *result)
{
    const uint32_t count = candidates(world, bounds, mask);
    slots_sort(world->query_slots, count);
    uint32_t matches = 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        const uint32_t slot = world->query_slots[i];
        const uint32_t dense = world->slots[slot].dense;
        const sl_shape *shape = &world->shapes[dense];
        if (shape->kind == SL_SHAPE_NONE)
            continue;
        const sl_transform transform = body_transform(world, dense);
        const bool match =
            point_query
                ? sl_shape_contains_point(shape, transform, bounds.lower)
                : sl_aabb_overlaps(sl_shape_aabb(shape, transform), bounds);
        if (!match)
            continue;
        if (matches < capacity)
            bodies[matches] = body_handle(world, slot);
        ++matches;
    }
    *result = (sl_query_result){ matches, matches > capacity };
}

bool sl_world_query_aabb(const sl_world *world, sl_aabb bounds,
                         uint32_t type_mask, sl_body_handle *bodies,
                         uint32_t capacity, sl_query_result *result)
{
    if (!buffer_valid(world, type_mask, bodies, capacity, result) ||
        !sl_aabb_is_valid(bounds))
        return false;
    query_buffer(world->state, bounds, false, type_mask, bodies, capacity,
                 result);
    return true;
}

bool sl_world_query_point(const sl_world *world, sl_vec2 point,
                          uint32_t type_mask, sl_body_handle *bodies,
                          uint32_t capacity, sl_query_result *result)
{
    if (!buffer_valid(world, type_mask, bodies, capacity, result) ||
        !sl_vec2_is_finite(point))
        return false;
    query_buffer(world->state, (sl_aabb){ point, point }, true, type_mask,
                 bodies, capacity, result);
    return true;
}

bool sl_world_query_ray(const sl_world *owner, sl_ray ray, uint32_t type_mask,
                        sl_query_ray_result *result)
{
    if (!query_valid(owner, type_mask) || result == NULL ||
        !sl_vec2_is_finite(ray.origin) || !sl_vec2_is_finite(ray.translation) ||
        sl_abs(ray.origin.x) > SL_QUERY_RAY_COORDINATE_MAX ||
        sl_abs(ray.origin.y) > SL_QUERY_RAY_COORDINATE_MAX)
        return false;
    /* Double addition validates even FLT_MAX translations before binary32
     * endpoint/length calculations. No float state is changed by validation. */
    const double end_x = (double)ray.origin.x + (double)ray.translation.x;
    const double end_y = (double)ray.origin.y + (double)ray.translation.y;
    const double limit = (double)SL_QUERY_RAY_COORDINATE_MAX;
    if (end_x < -limit || end_x > limit || end_y < -limit || end_y > limit ||
        sl_vec2_length_sq(ray.translation) <= SL_EPSILON * SL_EPSILON)
        return false;
    const sl_vec2 end = { (float)end_x, (float)end_y };
    const sl_aabb bounds = {
        { sl_min(ray.origin.x, end.x), sl_min(ray.origin.y, end.y) },
        { sl_max(ray.origin.x, end.x), sl_max(ray.origin.y, end.y) }
    };
    const sl_world_state *world = owner->state;
    const uint32_t count = candidates(world, bounds, type_mask);
    sl_query_ray_result best = { 0 };
    for (uint32_t i = 0u; i < count; ++i) {
        const uint32_t slot = world->query_slots[i];
        const uint32_t dense = world->slots[slot].dense;
        sl_ray_hit hit;
        if (!sl_shape_ray_cast(&world->shapes[dense],
                               body_transform(world, dense), ray, &hit))
            continue;
        if (!best.hit || hit.fraction < best.geometry.fraction ||
            (hit.fraction == best.geometry.fraction &&
             slot < best.body.index)) {
            best.hit = true;
            best.body = body_handle(world, slot);
            best.geometry = hit;
        }
    }
    *result = best;
    return true;
}
