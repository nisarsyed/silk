#ifndef SILK_BVH_H
#define SILK_BVH_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

/* Bounding volume hierarchy over a fixed set of elements (triangles or
 * edges of the cloth meshes). The tree is built once from element
 * centroids when the topology changes and refit to new element boxes at
 * every contact detection: cloth keeps its surface neighbourhoods, so a
 * topology-built tree stays tight under deformation, and a refit is O(n).
 *
 * Build: median split on the longest centroid axis, leaves of at most
 * SL_BVH_LEAF_MAX elements, children allocated as consecutive pairs after
 * their parent (so a reverse sweep refits bottom-up). The split is a
 * deterministic quickselect, ties broken by element index, so the tree is
 * a pure function of the centroids. Median splits bound the depth by
 * ceil(log2(n)) + 1, which SL_BVH_DEPTH_MAX covers for any count the world
 * allows. All storage is caller-provided. */

#define SL_BVH_LEAF_MAX 8u
#define SL_BVH_DEPTH_MAX 64u

typedef struct sl_bvh {
    uint32_t capacity;      /* elements; nodes hold 2 * capacity */
    uint32_t element_count; /* elements in the current tree */
    uint32_t node_count;
    sl_vec3 *node_lo; /* node boxes, m */
    sl_vec3 *node_hi;
    /* Internal nodes: left child index (right is first + 1), count 0.
     * Leaves: start into perm and count in [1, SL_BVH_LEAF_MAX]. */
    uint32_t *node_first;
    uint32_t *node_count_of;
    uint32_t *perm;    /* element indices in leaf order */
    sl_vec3 *centroid; /* per element, filled before sl_bvh_build */
    sl_vec3 *elem_lo;  /* per element, filled before sl_bvh_refit */
    sl_vec3 *elem_hi;
    /* Per element bounding sphere, maintained by the caller for its own
     * candidate culls (the tree does not read them). */
    sl_vec3 *sphere_center;
    float *sphere_radius;
} sl_bvh;

/* Builds the tree over elements [0, element_count) from bvh->centroid.
 * element_count <= capacity. Node boxes are left for sl_bvh_refit. */
void sl_bvh_build(sl_bvh *bvh, uint32_t element_count);

/* Recomputes every node box from bvh->elem_lo / elem_hi. */
void sl_bvh_refit(sl_bvh *bvh);

/* Depth-first walk over the leaves whose boxes overlap [lo, hi]. */
typedef struct sl_bvh_cursor {
    uint32_t stack[SL_BVH_DEPTH_MAX];
    uint32_t depth;
    sl_vec3 lo;
    sl_vec3 hi;
} sl_bvh_cursor;

void sl_bvh_cursor_init(sl_bvh_cursor *cursor, const sl_bvh *bvh, sl_vec3 lo,
                        sl_vec3 hi);

/* Advances to the next overlapping leaf, writing its perm range
 * [*first, *first + *count). Returns false when the walk is done. */
bool sl_bvh_next_leaf(sl_bvh_cursor *cursor, const sl_bvh *bvh, uint32_t *first,
                      uint32_t *count);

/* Self traversal: every pair of leaves (a, b) whose boxes come within
 * `grow` of each other, each unordered pair once, a leaf paired with
 * itself included. A node pair splits the node with more volume (an
 * internal one when the other is a leaf); a node paired with itself
 * yields its children's three pairs. Each step pushes at most three
 * pairs and a path splits each tree at most its depth, so the stack stays
 * within 3 * 2 * depth. */
#define SL_BVH_PAIR_STACK_MAX (6u * SL_BVH_DEPTH_MAX)

typedef struct sl_bvh_pair_cursor {
    uint32_t stack[SL_BVH_PAIR_STACK_MAX][2];
    uint32_t depth;
    float grow;
} sl_bvh_pair_cursor;

void sl_bvh_pair_cursor_init(sl_bvh_pair_cursor *cursor, const sl_bvh *bvh,
                             float grow);

/* Advances to the next leaf pair, writing both perm ranges; *same is true
 * when both ranges are the same leaf. Returns false when done. */
bool sl_bvh_next_leaf_pair(sl_bvh_pair_cursor *cursor, const sl_bvh *bvh,
                           uint32_t *a_first, uint32_t *a_count,
                           uint32_t *b_first, uint32_t *b_count, bool *same);

#endif /* SILK_BVH_H */
