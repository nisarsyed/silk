#include "bvh.h"

static float sl_axis(sl_vec3 v, uint32_t axis)
{
    return (axis == 0u) ? v.x : ((axis == 1u) ? v.y : v.z);
}

/* Strict weak order on elements by centroid along axis, index tie-break. */
static bool sl_bvh_less(const sl_bvh *b, uint32_t axis, uint32_t e, uint32_t f)
{
    const float ke = sl_axis(b->centroid[e], axis);
    const float kf = sl_axis(b->centroid[f], axis);
    return (ke < kf) || (ke == kf && e < f);
}

/* Reorders perm[begin, end) so that perm[nth] holds the element of that
 * rank and every element before it is not greater (Hoare quickselect,
 * median-of-three pivot). Iterative; each pass shrinks the range. */
static void sl_bvh_select(sl_bvh *b, uint32_t begin, uint32_t end, uint32_t nth,
                          uint32_t axis)
{
    uint32_t *p = b->perm;
    uint32_t lo = begin;
    uint32_t hi = end - 1u;
    while (hi > lo) {
        const uint32_t mid = lo + (hi - lo) / 2u;
        /* Median of three into p[mid]. */
        if (sl_bvh_less(b, axis, p[mid], p[lo])) {
            const uint32_t t = p[mid];
            p[mid] = p[lo];
            p[lo] = t;
        }
        if (sl_bvh_less(b, axis, p[hi], p[lo])) {
            const uint32_t t = p[hi];
            p[hi] = p[lo];
            p[lo] = t;
        }
        if (sl_bvh_less(b, axis, p[hi], p[mid])) {
            const uint32_t t = p[hi];
            p[hi] = p[mid];
            p[mid] = t;
        }
        const uint32_t pivot = p[mid];
        uint32_t i = lo;
        uint32_t j = hi;
        while (i <= j) {
            while (sl_bvh_less(b, axis, p[i], pivot)) {
                i++;
            }
            while (sl_bvh_less(b, axis, pivot, p[j])) {
                j--;
            }
            if (i <= j) {
                const uint32_t t = p[i];
                p[i] = p[j];
                p[j] = t;
                i++;
                if (j == 0u) {
                    break;
                }
                j--;
            }
        }
        /* Now [lo, j] <= pivot <= [i, hi]. */
        if (nth <= j) {
            hi = j;
        } else if (nth >= i) {
            lo = i;
        } else {
            return;
        }
    }
}

void sl_bvh_build(sl_bvh *b, uint32_t element_count)
{
    SL_ASSERT(element_count <= b->capacity);
    b->element_count = element_count;
    b->node_count = 0u;
    if (element_count == 0u) {
        return;
    }
    for (uint32_t e = 0; e < element_count; e++) {
        b->perm[e] = e;
    }
    /* Explicit stack of (node, begin, end); depth-first, so it never holds
     * more than depth + 1 entries. */
    uint32_t stack[SL_BVH_DEPTH_MAX][3];
    uint32_t depth = 0;
    b->node_count = 1u;
    stack[depth][0] = 0u;
    stack[depth][1] = 0u;
    stack[depth][2] = element_count;
    depth++;
    while (depth > 0u) {
        depth--;
        const uint32_t node = stack[depth][0];
        const uint32_t begin = stack[depth][1];
        const uint32_t end = stack[depth][2];
        if (end - begin <= SL_BVH_LEAF_MAX) {
            b->node_first[node] = begin;
            b->node_count_of[node] = end - begin;
            continue;
        }
        sl_vec3 lo = b->centroid[b->perm[begin]];
        sl_vec3 hi = lo;
        for (uint32_t k = begin + 1u; k < end; k++) {
            const sl_vec3 c = b->centroid[b->perm[k]];
            lo = sl_vec3_make(sl_min(lo.x, c.x), sl_min(lo.y, c.y),
                              sl_min(lo.z, c.z));
            hi = sl_vec3_make(sl_max(hi.x, c.x), sl_max(hi.y, c.y),
                              sl_max(hi.z, c.z));
        }
        const sl_vec3 extent = sl_vec3_sub(hi, lo);
        uint32_t axis = 0u;
        if (extent.y > extent.x && extent.y >= extent.z) {
            axis = 1u;
        } else if (extent.z > extent.x && extent.z > extent.y) {
            axis = 2u;
        }
        const uint32_t mid = begin + (end - begin) / 2u;
        sl_bvh_select(b, begin, end, mid, axis);
        const uint32_t left = b->node_count;
        b->node_count += 2u;
        SL_ASSERT(b->node_count <= 2u * b->capacity);
        b->node_first[node] = left;
        b->node_count_of[node] = 0u;
        SL_ASSERT(depth + 2u <= SL_BVH_DEPTH_MAX);
        stack[depth][0] = left + 1u;
        stack[depth][1] = mid;
        stack[depth][2] = end;
        depth++;
        stack[depth][0] = left;
        stack[depth][1] = begin;
        stack[depth][2] = mid;
        depth++;
    }
}

void sl_bvh_refit(sl_bvh *b)
{
    /* Children follow their parent, so a reverse sweep is bottom-up. */
    for (uint32_t n = b->node_count; n > 0u; n--) {
        const uint32_t node = n - 1u;
        const uint32_t first = b->node_first[node];
        const uint32_t count = b->node_count_of[node];
        sl_vec3 lo;
        sl_vec3 hi;
        if (count == 0u) {
            lo = b->node_lo[first];
            hi = b->node_hi[first];
            const sl_vec3 lo2 = b->node_lo[first + 1u];
            const sl_vec3 hi2 = b->node_hi[first + 1u];
            lo = sl_vec3_make(sl_min(lo.x, lo2.x), sl_min(lo.y, lo2.y),
                              sl_min(lo.z, lo2.z));
            hi = sl_vec3_make(sl_max(hi.x, hi2.x), sl_max(hi.y, hi2.y),
                              sl_max(hi.z, hi2.z));
        } else {
            lo = b->elem_lo[b->perm[first]];
            hi = b->elem_hi[b->perm[first]];
            for (uint32_t k = first + 1u; k < first + count; k++) {
                const sl_vec3 l = b->elem_lo[b->perm[k]];
                const sl_vec3 h = b->elem_hi[b->perm[k]];
                lo = sl_vec3_make(sl_min(lo.x, l.x), sl_min(lo.y, l.y),
                                  sl_min(lo.z, l.z));
                hi = sl_vec3_make(sl_max(hi.x, h.x), sl_max(hi.y, h.y),
                                  sl_max(hi.z, h.z));
            }
        }
        b->node_lo[node] = lo;
        b->node_hi[node] = hi;
    }
}

void sl_bvh_cursor_init(sl_bvh_cursor *c, const sl_bvh *b, sl_vec3 lo,
                        sl_vec3 hi)
{
    c->lo = lo;
    c->hi = hi;
    c->depth = 0u;
    if (b->node_count > 0u) {
        c->stack[c->depth++] = 0u;
    }
}

static bool sl_box_overlap(sl_vec3 alo, sl_vec3 ahi, sl_vec3 blo, sl_vec3 bhi)
{
    return alo.x <= bhi.x && blo.x <= ahi.x && alo.y <= bhi.y &&
           blo.y <= ahi.y && alo.z <= bhi.z && blo.z <= ahi.z;
}

bool sl_bvh_next_leaf(sl_bvh_cursor *c, const sl_bvh *b, uint32_t *first,
                      uint32_t *count)
{
    while (c->depth > 0u) {
        const uint32_t node = c->stack[--c->depth];
        if (!sl_box_overlap(c->lo, c->hi, b->node_lo[node], b->node_hi[node])) {
            continue;
        }
        if (b->node_count_of[node] > 0u) {
            *first = b->node_first[node];
            *count = b->node_count_of[node];
            return true;
        }
        SL_ASSERT(c->depth + 2u <= SL_BVH_DEPTH_MAX);
        c->stack[c->depth++] = b->node_first[node] + 1u;
        c->stack[c->depth++] = b->node_first[node];
    }
    return false;
}

void sl_bvh_pair_cursor_init(sl_bvh_pair_cursor *c, const sl_bvh *b, float grow)
{
    c->grow = grow;
    c->depth = 0u;
    if (b->node_count > 0u) {
        c->stack[0][0] = 0u;
        c->stack[0][1] = 0u;
        c->depth = 1u;
    }
}

static float sl_box_volume(sl_vec3 lo, sl_vec3 hi)
{
    const sl_vec3 d = sl_vec3_sub(hi, lo);
    return d.x * d.y * d.z;
}

static void sl_pair_push(sl_bvh_pair_cursor *c, uint32_t a, uint32_t b)
{
    SL_ASSERT(c->depth < SL_BVH_PAIR_STACK_MAX);
    if (c->depth < SL_BVH_PAIR_STACK_MAX) {
        c->stack[c->depth][0] = a;
        c->stack[c->depth][1] = b;
        c->depth++;
    }
}

bool sl_bvh_next_leaf_pair(sl_bvh_pair_cursor *c, const sl_bvh *b,
                           uint32_t *a_first, uint32_t *a_count,
                           uint32_t *b_first, uint32_t *b_count, bool *same)
{
    const sl_vec3 grow = sl_vec3_make(c->grow, c->grow, c->grow);
    while (c->depth > 0u) {
        c->depth--;
        const uint32_t a = c->stack[c->depth][0];
        const uint32_t n = c->stack[c->depth][1];
        const bool a_leaf = b->node_count_of[a] > 0u;
        const bool n_leaf = b->node_count_of[n] > 0u;
        if (a == n) {
            if (a_leaf) {
                *a_first = b->node_first[a];
                *a_count = b->node_count_of[a];
                *b_first = *a_first;
                *b_count = *a_count;
                *same = true;
                return true;
            }
            const uint32_t l = b->node_first[a];
            sl_pair_push(c, l, l + 1u);
            sl_pair_push(c, l + 1u, l + 1u);
            sl_pair_push(c, l, l);
            continue;
        }
        if (!sl_box_overlap(sl_vec3_sub(b->node_lo[a], grow),
                            sl_vec3_add(b->node_hi[a], grow), b->node_lo[n],
                            b->node_hi[n])) {
            continue;
        }
        if (a_leaf && n_leaf) {
            *a_first = b->node_first[a];
            *a_count = b->node_count_of[a];
            *b_first = b->node_first[n];
            *b_count = b->node_count_of[n];
            *same = false;
            return true;
        }
        const bool split_a =
            !a_leaf &&
            (n_leaf || sl_box_volume(b->node_lo[a], b->node_hi[a]) >=
                           sl_box_volume(b->node_lo[n], b->node_hi[n]));
        if (split_a) {
            const uint32_t l = b->node_first[a];
            sl_pair_push(c, l + 1u, n);
            sl_pair_push(c, l, n);
        } else {
            const uint32_t l = b->node_first[n];
            sl_pair_push(c, a, l + 1u);
            sl_pair_push(c, a, l);
        }
    }
    return false;
}

void sl_bvh_pair_cursor_start(sl_bvh_pair_cursor *c, float grow, uint32_t a,
                              uint32_t b)
{
    c->grow = grow;
    c->stack[0][0] = a;
    c->stack[0][1] = b;
    c->depth = 1u;
}

/* One expansion step of the self traversal for node pair (a, n), by the
 * rules of sl_bvh_next_leaf_pair: writes the child pairs to out and
 * returns their count. A pair that yields a leaf pair is kept whole
 * (*keep); a pair whose boxes are too far apart yields nothing. */
static uint32_t sl_pair_expand(const sl_bvh *b, sl_vec3 grow, uint32_t a,
                               uint32_t n, uint32_t out[3][2], bool *keep)
{
    const bool a_leaf = b->node_count_of[a] > 0u;
    const bool n_leaf = b->node_count_of[n] > 0u;
    *keep = false;
    if (a == n) {
        if (a_leaf) {
            *keep = true;
            return 0u;
        }
        const uint32_t l = b->node_first[a];
        out[0][0] = l;
        out[0][1] = l;
        out[1][0] = l + 1u;
        out[1][1] = l + 1u;
        out[2][0] = l;
        out[2][1] = l + 1u;
        return 3u;
    }
    if (!sl_box_overlap(sl_vec3_sub(b->node_lo[a], grow),
                        sl_vec3_add(b->node_hi[a], grow), b->node_lo[n],
                        b->node_hi[n])) {
        return 0u;
    }
    if (a_leaf && n_leaf) {
        *keep = true;
        return 0u;
    }
    const bool split_a =
        !a_leaf && (n_leaf || sl_box_volume(b->node_lo[a], b->node_hi[a]) >=
                                  sl_box_volume(b->node_lo[n], b->node_hi[n]));
    const uint32_t l = b->node_first[split_a ? a : n];
    out[0][0] = split_a ? l : a;
    out[0][1] = split_a ? n : l;
    out[1][0] = split_a ? l + 1u : a;
    out[1][1] = split_a ? n : l + 1u;
    return 2u;
}

uint32_t sl_bvh_pair_frontier(const sl_bvh *b, float grow, uint32_t (*pairs)[2],
                              uint32_t (*scratch)[2], uint32_t capacity,
                              uint32_t target)
{
    SL_ASSERT(capacity >= 1u);
    if (b->node_count == 0u) {
        return 0u;
    }
    const sl_vec3 g = sl_vec3_make(grow, grow, grow);
    pairs[0][0] = 0u;
    pairs[0][1] = 0u;
    uint32_t count = 1u;
    /* Each level at least halves the remaining depth of some pair, so the
     * tree depth bounds the levels. */
    for (uint32_t level = 0; level < 2u * SL_BVH_DEPTH_MAX; level++) {
        if (count >= target || 3u * count > capacity) {
            break;
        }
        uint32_t next = 0u;
        bool expanded = false;
        for (uint32_t k = 0; k < count; k++) {
            uint32_t out[3][2];
            bool keep;
            const uint32_t children =
                sl_pair_expand(b, g, pairs[k][0], pairs[k][1], out, &keep);
            if (keep) {
                scratch[next][0] = pairs[k][0];
                scratch[next][1] = pairs[k][1];
                next++;
            }
            for (uint32_t j = 0; j < children; j++) {
                scratch[next][0] = out[j][0];
                scratch[next][1] = out[j][1];
                next++;
            }
            expanded = expanded || !keep; /* split or dropped */
        }
        for (uint32_t k = 0; k < next; k++) {
            pairs[k][0] = scratch[k][0];
            pairs[k][1] = scratch[k][1];
        }
        count = next;
        if (!expanded) {
            break;
        }
    }
    return count;
}
