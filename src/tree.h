#ifndef SILK_TREE_H
#define SILK_TREE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "silk/math.h"
#include "silk/world.h"

#define SL_TREE_NODE_NONE UINT32_MAX
#define SL_TREE_HEIGHT_FREE UINT32_MAX

typedef enum sl_tree_root_kind {
    SL_TREE_ROOT_STATIC = 0,
    SL_TREE_ROOT_KINEMATIC,
    SL_TREE_ROOT_DYNAMIC,
    SL_TREE_ROOT_COUNT
} sl_tree_root_kind;

/* parent doubles as the next-free link while height is
 * SL_TREE_HEIGHT_FREE. Leaves have no children and height zero; internal
 * nodes have height >= 1 and no user data. */
typedef struct sl_tree_node {
    sl_aabb aabb;
    uint32_t parent;
    uint32_t child_1;
    uint32_t child_2;
    uint32_t height;
    uint32_t user;
} sl_tree_node;

typedef struct sl_tree_query_result {
    uint32_t count; /* total matches, possibly greater than output capacity */
    bool overflow;
} sl_tree_query_result;

/* Internal owning view over caller-provided memory. All three roots share
 * nodes; stack is single-threaded traversal scratch sized to node_capacity. */
typedef struct sl_tree {
    sl_tree_node *nodes;
    uint32_t *stack;
    uint32_t roots[SL_TREE_ROOT_COUNT];
    uint32_t proxy_capacity;
    uint32_t node_capacity;
    uint32_t node_count;
    uint32_t proxy_count;
    uint32_t free_list;
    void *memory;
} sl_tree;

/* Exact bytes for 2 * proxy_capacity nodes plus a traversal stack of the
 * same count. Returns zero outside [1, SL_BODY_COUNT_MAX]. */
size_t sl_tree_memory_bytes(uint32_t proxy_capacity);

/* Initializes over 8-byte-aligned caller memory of at least the exact byte
 * count. Rejection leaves *tree zeroed and does not touch memory. */
bool sl_tree_init(sl_tree *tree, void *memory, size_t memory_bytes,
                  uint32_t proxy_capacity);

/* Returns SL_TREE_NODE_NONE on invalid input or capacity exhaustion without
 * mutation. user is returned by queries and conventionally holds a body slot.
 */
uint32_t sl_tree_proxy_create(sl_tree *tree, sl_tree_root_kind root,
                              sl_aabb aabb, uint32_t user);

/* Invalid, stale, or wrong-root proxy ids are rejected without mutation. */
bool sl_tree_proxy_destroy(sl_tree *tree, sl_tree_root_kind root,
                           uint32_t proxy);
bool sl_tree_proxy_move(sl_tree *tree, sl_tree_root_kind root, uint32_t proxy,
                        sl_aabb aabb);

/* Inclusive query in deterministic depth-first order. At most out_capacity
 * users are written, while count reports every match and overflow reports
 * count > out_capacity. The arena-backed stack makes traversal allocation-free.
 */
sl_tree_query_result sl_tree_query(sl_tree *tree, sl_tree_root_kind root,
                                   sl_aabb aabb, uint32_t *out_users,
                                   uint32_t out_capacity);

uint32_t sl_tree_height(const sl_tree *tree, sl_tree_root_kind root);

#endif /* SILK_TREE_H */
