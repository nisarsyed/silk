#include <stdlib.h>
#include <string.h>

#include "rigid.h"
#include "silk/world.h"

/* Bound on manifold capacity, so arena sizes stay far from overflow. */
#define SL_MANIFOLD_CAPACITY_MAX (1u << 20)

static size_t sl_rigid_align(size_t value)
{
    return (value + 15u) & ~(size_t)15u;
}

static void *sl_rigid_take(unsigned char *base, size_t *cursor, size_t count,
                           size_t size)
{
    const size_t at = sl_rigid_align(*cursor);
    *cursor = at + count * size;
    return (base != NULL) ? (void *)(base + at) : NULL;
}

/* Lays out every array; with base NULL, only measures. */
static size_t sl_body_set_layout(sl_body_set *set, unsigned char *base)
{
    size_t at = 0;
    const size_t n = set->capacity;
    const size_t m = set->manifold_capacity;
#define SL_TAKE(field, count)                                                  \
    set->field = sl_rigid_take(base, &at, (count), sizeof(*set->field))
    SL_TAKE(position, n);
    SL_TAKE(orientation, n);
    SL_TAKE(half_extent, n);
    SL_TAKE(velocity, n);
    SL_TAKE(angular_velocity, n);
    SL_TAKE(inverse_mass, n);
    SL_TAKE(inverse_inertia, n);
    SL_TAKE(manifolds, m);
    SL_TAKE(manifolds_previous, m);
    SL_TAKE(pair_key, m);
    SL_TAKE(pair_scratch, m);
    SL_TAKE(bvh.node_lo, 2u * n);
    SL_TAKE(bvh.node_hi, 2u * n);
    SL_TAKE(bvh.node_first, 2u * n);
    SL_TAKE(bvh.node_count_of, 2u * n);
    SL_TAKE(bvh.perm, n);
    SL_TAKE(bvh.centroid, n);
    SL_TAKE(bvh.elem_lo, n);
    SL_TAKE(bvh.elem_hi, n);
#undef SL_TAKE
    return sl_rigid_align(at);
}

bool sl_body_set_init(sl_body_set *set, uint32_t capacity,
                      uint32_t manifold_capacity)
{
    SL_ASSERT(set != NULL);
    if (set == NULL || capacity < 1u || capacity > SL_BODY_COUNT_MAX ||
        manifold_capacity < 1u ||
        manifold_capacity > SL_MANIFOLD_CAPACITY_MAX) {
        return false;
    }
    sl_body_set s;
    memset(&s, 0, sizeof(s));
    s.capacity = capacity;
    s.manifold_capacity = manifold_capacity;
    const size_t bytes = sl_body_set_layout(&s, NULL);
    unsigned char *arena = malloc(bytes);
    if (arena == NULL) {
        return false;
    }
    /* Zero once so no array is ever read uninitialized. */
    memset(arena, 0, bytes);
    (void)sl_body_set_layout(&s, arena);
    s.bvh.capacity = capacity;
    s.arena = arena;
    *set = s;
    return true;
}

void sl_body_set_destroy(sl_body_set *set)
{
    SL_ASSERT(set != NULL);
    if (set == NULL) {
        return;
    }
    free(set->arena);
    memset(set, 0, sizeof(*set));
}

static bool sl_rigid_in_domain(sl_vec3 v)
{
    return sl_vec3_is_finite(v) && sl_abs(v.x) <= SL_POSITION_ABS_MAX &&
           sl_abs(v.y) <= SL_POSITION_ABS_MAX &&
           sl_abs(v.z) <= SL_POSITION_ABS_MAX;
}

bool sl_body_add_box(sl_body_set *set, sl_vec3 position, sl_quat orientation,
                     sl_vec3 half_extent, float density, uint32_t *body)
{
    SL_ASSERT(set != NULL && body != NULL);
    const bool orientation_ok =
        sl_is_finite(orientation.x) && sl_is_finite(orientation.y) &&
        sl_is_finite(orientation.z) && sl_is_finite(orientation.w);
    if (set->count >= set->capacity || !sl_rigid_in_domain(position) ||
        !orientation_ok || !sl_vec3_is_finite(half_extent) ||
        !(half_extent.x > 0.0f) || !(half_extent.y > 0.0f) ||
        !(half_extent.z > 0.0f) || half_extent.x > SL_POSITION_ABS_MAX ||
        half_extent.y > SL_POSITION_ABS_MAX ||
        half_extent.z > SL_POSITION_ABS_MAX || !sl_is_finite(density) ||
        density < 0.0f) {
        return false;
    }
    const uint32_t i = set->count++;
    set->position[i] = position;
    set->orientation[i] = sl_quat_normalize(orientation);
    set->half_extent[i] = half_extent;
    set->velocity[i] = sl_vec3_zero();
    set->angular_velocity[i] = sl_vec3_zero();
    /* Solid box: m = rho 8 hx hy hz, I_x = m (hy^2 + hz^2) / 3, and so on
     * (half extents, so the usual m (w^2 + d^2) / 12 with w = 2 hy). */
    const float hx = half_extent.x;
    const float hy = half_extent.y;
    const float hz = half_extent.z;
    const float mass = density * 8.0f * hx * hy * hz;
    if (mass > 0.0f && sl_is_finite(mass)) {
        set->inverse_mass[i] = 1.0f / mass;
        set->inverse_inertia[i] =
            sl_vec3_make(3.0f / (mass * (hy * hy + hz * hz)),
                         3.0f / (mass * (hx * hx + hz * hz)),
                         3.0f / (mass * (hx * hx + hy * hy)));
    } else {
        set->inverse_mass[i] = 0.0f;
        set->inverse_inertia[i] = sl_vec3_zero();
    }
    *body = i;
    return true;
}

/* --------------------------------------------------------------------- */
/* Broad phase                                                            */

static bool sl_rigid_boxes_overlap(const sl_bvh *bvh, uint32_t a, uint32_t b)
{
    const sl_vec3 al = bvh->elem_lo[a];
    const sl_vec3 ah = bvh->elem_hi[a];
    const sl_vec3 bl = bvh->elem_lo[b];
    const sl_vec3 bh = bvh->elem_hi[b];
    return al.x <= bh.x && bl.x <= ah.x && al.y <= bh.y && bl.y <= ah.y &&
           al.z <= bh.z && bl.z <= ah.z;
}

/* Sorts keys[0, count) ascending: LSD radix sort, four 8-bit digits,
 * through scratch. Stable, allocation-free, O(count). */
static void sl_rigid_sort(uint32_t *keys, uint32_t *scratch, uint32_t count)
{
    uint32_t *from = keys;
    uint32_t *to = scratch;
    for (uint32_t shift = 0; shift < 32u; shift += 8u) {
        uint32_t offset[257];
        memset(offset, 0, sizeof(offset));
        for (uint32_t k = 0; k < count; k++) {
            offset[((from[k] >> shift) & 0xFFu) + 1u]++;
        }
        for (uint32_t d = 0; d < 256u; d++) {
            offset[d + 1u] += offset[d];
        }
        for (uint32_t k = 0; k < count; k++) {
            to[offset[(from[k] >> shift) & 0xFFu]++] = from[k];
        }
        uint32_t *swap = from;
        from = to;
        to = swap;
    }
    /* Four passes end back in keys. */
    SL_ASSERT(from == keys);
}

/* Candidate body pairs whose margin-expanded bounding boxes overlap, as
 * sorted keys (a << 16 | b, a < b). Pairs of two static bodies are
 * skipped. Returns the count. */
static uint32_t sl_rigid_pairs(sl_body_set *set, float margin)
{
    sl_bvh *bvh = &set->bvh;
    const sl_vec3 grow =
        sl_vec3_make(0.5f * margin, 0.5f * margin, 0.5f * margin);
    for (uint32_t i = 0; i < set->count; i++) {
        const sl_mat3 m = sl_quat_to_mat3(set->orientation[i]);
        const sl_vec3 h = set->half_extent[i];
        /* World half extents of the oriented box: sum_k |axis_k| h_k. */
        const sl_vec3 e = sl_vec3_make(
            sl_abs(m.c0.x) * h.x + sl_abs(m.c1.x) * h.y + sl_abs(m.c2.x) * h.z,
            sl_abs(m.c0.y) * h.x + sl_abs(m.c1.y) * h.y + sl_abs(m.c2.y) * h.z,
            sl_abs(m.c0.z) * h.x + sl_abs(m.c1.z) * h.y + sl_abs(m.c2.z) * h.z);
        const sl_vec3 extent = sl_vec3_add(e, grow);
        bvh->centroid[i] = set->position[i];
        bvh->elem_lo[i] = sl_vec3_sub(set->position[i], extent);
        bvh->elem_hi[i] = sl_vec3_add(set->position[i], extent);
    }
    sl_bvh_build(bvh, set->count);
    sl_bvh_refit(bvh);
    uint32_t count = 0;
    sl_bvh_pair_cursor cursor;
    sl_bvh_pair_cursor_init(&cursor, bvh, 0.0f);
    uint32_t a_first;
    uint32_t a_count;
    uint32_t b_first;
    uint32_t b_count;
    bool same;
    while (sl_bvh_next_leaf_pair(&cursor, bvh, &a_first, &a_count, &b_first,
                                 &b_count, &same)) {
        for (uint32_t i = a_first; i < a_first + a_count; i++) {
            const uint32_t j_begin = same ? i + 1u : b_first;
            for (uint32_t j = j_begin; j < b_first + b_count; j++) {
                const uint32_t x = bvh->perm[i];
                const uint32_t y = bvh->perm[j];
                if (!sl_rigid_boxes_overlap(bvh, x, y) ||
                    (set->inverse_mass[x] == 0.0f &&
                     set->inverse_mass[y] == 0.0f)) {
                    continue;
                }
                if (count >= set->manifold_capacity) {
                    set->dropped++;
                    continue;
                }
                const uint32_t lo = (x < y) ? x : y;
                const uint32_t hi = (x < y) ? y : x;
                set->pair_key[count++] = (lo << 16) | hi;
            }
        }
    }
    sl_rigid_sort(set->pair_key, set->pair_scratch, count);
    return count;
}

/* --------------------------------------------------------------------- */
/* Pipeline                                                               */

static void sl_rigid_emit(sl_body_set *set, const sl_manifold *candidate,
                          uint32_t a, uint32_t b)
{
    if (set->manifold_count >= set->manifold_capacity) {
        set->dropped++;
        return;
    }
    sl_manifold *out = &set->manifolds[set->manifold_count++];
    *out = *candidate;
    out->body_a = a;
    out->body_b = b;
}

static uint64_t sl_manifold_key(const sl_manifold *m)
{
    return ((uint64_t)m->body_a << 32) | (uint64_t)m->body_b;
}

/* A new point without an id match takes the warm data of the nearest
 * unmatched previous point whose anchor on body a lies within this
 * distance (m). Ids change when features coincide: two equal boxes
 * stacked flush have side planes on top of each other, and a 0.001 rad
 * twist turned a corner from (incident edge, side plane) into (side plane,
 * side plane), dropping its warm data every step (tests/test_rigid.c). */
#define SL_CONTACT_MATCH_DISTANCE 0.005f

/* Copies warm data from the previous detection to points of the same pair:
 * first by feature id, then by anchor distance. Both lists are sorted by
 * (body_a, body_b). */
static void sl_rigid_carry(sl_body_set *set)
{
    uint32_t j = 0;
    for (uint32_t i = 0; i < set->manifold_count; i++) {
        sl_manifold *now = &set->manifolds[i];
        const uint64_t key = sl_manifold_key(now);
        while (j < set->manifold_count_previous &&
               sl_manifold_key(&set->manifolds_previous[j]) < key) {
            j++;
        }
        if (j >= set->manifold_count_previous ||
            sl_manifold_key(&set->manifolds_previous[j]) != key) {
            continue;
        }
        const sl_manifold *before = &set->manifolds_previous[j];
        bool used[SL_MANIFOLD_POINT_COUNT_MAX] = { false };
        bool matched[SL_MANIFOLD_POINT_COUNT_MAX] = { false };
        for (uint32_t p = 0; p < now->point_count; p++) {
            for (uint32_t q = 0; q < before->point_count; q++) {
                if (!used[q] && before->points[q].id == now->points[p].id) {
                    memcpy(now->points[p].warm, before->points[q].warm,
                           sizeof(now->points[p].warm));
                    used[q] = true;
                    matched[p] = true;
                    break;
                }
            }
        }
        for (uint32_t p = 0; p < now->point_count; p++) {
            if (matched[p]) {
                continue;
            }
            uint32_t nearest = UINT32_MAX;
            float best = SL_CONTACT_MATCH_DISTANCE * SL_CONTACT_MATCH_DISTANCE;
            for (uint32_t q = 0; q < before->point_count; q++) {
                const float d = sl_vec3_length_sq(sl_vec3_sub(
                    before->points[q].local_a, now->points[p].local_a));
                if (!used[q] && d <= best) {
                    best = d;
                    nearest = q;
                }
            }
            if (nearest != UINT32_MAX) {
                memcpy(now->points[p].warm, before->points[nearest].warm,
                       sizeof(now->points[p].warm));
                used[nearest] = true;
            }
        }
    }
}

void sl_rigid_collide(sl_body_set *set, const sl_rigid_plane *planes,
                      uint32_t plane_count, float margin)
{
    SL_ASSERT(set != NULL && margin >= 0.0f);
    SL_ASSERT(plane_count == 0u || planes != NULL);
    sl_manifold *swap = set->manifolds_previous;
    set->manifolds_previous = set->manifolds;
    set->manifolds = swap;
    set->manifold_count_previous = set->manifold_count;
    set->manifold_count = 0u;
    set->dropped = 0u;
    if (set->count == 0u) {
        return;
    }
    const uint32_t pair_count = sl_rigid_pairs(set, margin);
    /* Emit in (body_a, body_b) order: body pairs of a (sorted keys), then
     * a's planes, whose indices start at SL_RIGID_STATIC_BASE. */
    uint32_t k = 0;
    sl_manifold candidate;
    for (uint32_t a = 0; a < set->count; a++) {
        for (; k < pair_count && (set->pair_key[k] >> 16) == a; k++) {
            const uint32_t b = set->pair_key[k] & 0xFFFFu;
            if (sl_collide_box_box(set->position[a], set->orientation[a],
                                   set->half_extent[a], set->position[b],
                                   set->orientation[b], set->half_extent[b],
                                   margin, &candidate) > 0u) {
                sl_rigid_emit(set, &candidate, a, b);
            }
        }
        if (set->inverse_mass[a] == 0.0f) {
            continue;
        }
        for (uint32_t p = 0; p < plane_count; p++) {
            if (sl_collide_box_plane(set->position[a], set->orientation[a],
                                     set->half_extent[a], &planes[p], margin,
                                     &candidate) > 0u) {
                sl_rigid_emit(set, &candidate, a, SL_RIGID_STATIC_BASE + p);
            }
        }
    }
    SL_ASSERT(k == pair_count);
    sl_rigid_carry(set);
}
