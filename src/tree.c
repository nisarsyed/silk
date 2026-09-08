#include "tree.h"

#include <stdint.h>
#include <string.h>

#include "silk/assert.h"

#define SL_TREE_CARVE_ALIGN ((size_t)8u)

_Static_assert(sizeof(sl_tree_node) == 36u,
               "tree node layout changed; update the arena budget");
_Static_assert(_Alignof(sl_tree_node) <= SL_TREE_CARVE_ALIGN &&
                   SL_TREE_CARVE_ALIGN % _Alignof(sl_tree_node) == 0u,
               "tree node alignment violates the arena contract");
_Static_assert(_Alignof(uint32_t) <= SL_TREE_CARVE_ALIGN &&
                   SL_TREE_CARVE_ALIGN % _Alignof(uint32_t) == 0u,
               "tree stack alignment violates the arena contract");
_Static_assert((SL_TREE_CARVE_ALIGN & (SL_TREE_CARVE_ALIGN - 1u)) == 0u,
               "tree arena alignment must be a power of two");

static size_t sl_tree_align_up(size_t bytes)
{
    return (bytes + SL_TREE_CARVE_ALIGN - 1u) & ~(SL_TREE_CARVE_ALIGN - 1u);
}

static size_t sl_tree_slice_bytes(size_t count, size_t element_size)
{
    return sl_tree_align_up(count * element_size);
}

static bool sl_tree_root_valid(sl_tree_root_kind root)
{
    return root >= SL_TREE_ROOT_STATIC && root < SL_TREE_ROOT_COUNT;
}

static bool sl_tree_node_is_leaf(const sl_tree_node *node)
{
    return node->height == 0u;
}

size_t sl_tree_memory_bytes(uint32_t proxy_capacity)
{
    if (proxy_capacity < 1u || proxy_capacity > SL_BODY_COUNT_MAX) {
        return 0u;
    }
    const size_t node_capacity = 2u * (size_t)proxy_capacity;
    return sl_tree_slice_bytes(node_capacity, sizeof(sl_tree_node)) +
           sl_tree_slice_bytes(node_capacity, sizeof(uint32_t));
}

bool sl_tree_init(sl_tree *tree, void *memory, size_t memory_bytes,
                  uint32_t proxy_capacity)
{
    if (tree == NULL) {
        return false;
    }
    memset(tree, 0, sizeof(*tree));
    const size_t required = sl_tree_memory_bytes(proxy_capacity);
    if (required == 0u || memory == NULL || memory_bytes < required ||
        ((uintptr_t)memory & (SL_TREE_CARVE_ALIGN - 1u)) != 0u) {
        return false;
    }

    memset(memory, 0, required);
    const uint32_t node_capacity = 2u * proxy_capacity;
    unsigned char *cursor = memory;
    tree->nodes = (sl_tree_node *)cursor;
    cursor += sl_tree_slice_bytes((size_t)node_capacity, sizeof(sl_tree_node));
    tree->stack = (uint32_t *)cursor;
    for (uint32_t i = 0u; i < SL_TREE_ROOT_COUNT; ++i) {
        tree->roots[i] = SL_TREE_NODE_NONE;
    }
    for (uint32_t i = 0u; i < node_capacity; ++i) {
        tree->nodes[i].parent =
            (i + 1u < node_capacity) ? i + 1u : SL_TREE_NODE_NONE;
        tree->nodes[i].child_1 = SL_TREE_NODE_NONE;
        tree->nodes[i].child_2 = SL_TREE_NODE_NONE;
        tree->nodes[i].height = SL_TREE_HEIGHT_FREE;
        tree->nodes[i].user = SL_TREE_NODE_NONE;
    }
    tree->proxy_capacity = proxy_capacity;
    tree->node_capacity = node_capacity;
    tree->free_list = 0u;
    tree->memory = memory;
    return true;
}

static uint32_t sl_tree_node_allocate(sl_tree *tree)
{
    if (tree->free_list == SL_TREE_NODE_NONE) {
        return SL_TREE_NODE_NONE;
    }
    const uint32_t node_id = tree->free_list;
    sl_tree_node *node = &tree->nodes[node_id];
    tree->free_list = node->parent;
    node->parent = SL_TREE_NODE_NONE;
    node->child_1 = SL_TREE_NODE_NONE;
    node->child_2 = SL_TREE_NODE_NONE;
    node->height = 0u;
    node->user = SL_TREE_NODE_NONE;
    tree->node_count += 1u;
    return node_id;
}

static void sl_tree_node_free(sl_tree *tree, uint32_t node_id)
{
    SL_ASSERT(node_id < tree->node_capacity);
    sl_tree_node *node = &tree->nodes[node_id];
    SL_ASSERT(node->height != SL_TREE_HEIGHT_FREE);
    node->aabb =
        sl_aabb_make(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f));
    node->child_1 = SL_TREE_NODE_NONE;
    node->child_2 = SL_TREE_NODE_NONE;
    node->height = SL_TREE_HEIGHT_FREE;
    node->user = SL_TREE_NODE_NONE;
    node->parent = tree->free_list;
    tree->free_list = node_id;
    tree->node_count -= 1u;
}

static void sl_tree_node_refit(sl_tree *tree, uint32_t node_id)
{
    sl_tree_node *node = &tree->nodes[node_id];
    SL_ASSERT(!sl_tree_node_is_leaf(node));
    const sl_tree_node *child_1 = &tree->nodes[node->child_1];
    const sl_tree_node *child_2 = &tree->nodes[node->child_2];
    node->height = 1u + ((child_1->height > child_2->height) ? child_1->height
                                                             : child_2->height);
    node->aabb = sl_aabb_union(child_1->aabb, child_2->aabb);
}

static void sl_tree_replace_child(sl_tree *tree, sl_tree_root_kind root,
                                  uint32_t parent, uint32_t old_child,
                                  uint32_t new_child)
{
    if (parent == SL_TREE_NODE_NONE) {
        SL_ASSERT(tree->roots[root] == old_child);
        tree->roots[root] = new_child;
    } else {
        sl_tree_node *parent_node = &tree->nodes[parent];
        if (parent_node->child_1 == old_child) {
            parent_node->child_1 = new_child;
        } else {
            SL_ASSERT(parent_node->child_2 == old_child);
            parent_node->child_2 = new_child;
        }
    }
    tree->nodes[new_child].parent = parent;
}

/* AVL-style rotations preserve leaf order and use child_1 on equal heights,
 * making every mutation sequence deterministic. Returns the new subtree root.
 */
static uint32_t sl_tree_balance(sl_tree *tree, sl_tree_root_kind root,
                                uint32_t node_a_id)
{
    sl_tree_node *node_a = &tree->nodes[node_a_id];
    if (sl_tree_node_is_leaf(node_a) || node_a->height < 2u) {
        return node_a_id;
    }

    const uint32_t node_b_id = node_a->child_1;
    const uint32_t node_c_id = node_a->child_2;
    sl_tree_node *node_b = &tree->nodes[node_b_id];
    sl_tree_node *node_c = &tree->nodes[node_c_id];
    const int64_t balance = (int64_t)node_c->height - (int64_t)node_b->height;

    if (balance > 1) {
        const uint32_t node_f_id = node_c->child_1;
        const uint32_t node_g_id = node_c->child_2;
        sl_tree_node *node_f = &tree->nodes[node_f_id];
        sl_tree_node *node_g = &tree->nodes[node_g_id];
        const uint32_t old_parent = node_a->parent;
        sl_tree_replace_child(tree, root, old_parent, node_a_id, node_c_id);
        node_c->child_1 = node_a_id;
        node_a->parent = node_c_id;

        if (node_f->height > node_g->height) {
            node_c->child_2 = node_f_id;
            node_a->child_2 = node_g_id;
            node_f->parent = node_c_id;
            node_g->parent = node_a_id;
        } else {
            node_c->child_2 = node_g_id;
            node_a->child_2 = node_f_id;
            node_g->parent = node_c_id;
            node_f->parent = node_a_id;
        }
        sl_tree_node_refit(tree, node_a_id);
        sl_tree_node_refit(tree, node_c_id);
        return node_c_id;
    }

    if (balance < -1) {
        const uint32_t node_d_id = node_b->child_1;
        const uint32_t node_e_id = node_b->child_2;
        sl_tree_node *node_d = &tree->nodes[node_d_id];
        sl_tree_node *node_e = &tree->nodes[node_e_id];
        const uint32_t old_parent = node_a->parent;
        sl_tree_replace_child(tree, root, old_parent, node_a_id, node_b_id);
        node_b->child_2 = node_a_id;
        node_a->parent = node_b_id;

        if (node_d->height > node_e->height) {
            node_b->child_1 = node_d_id;
            node_a->child_1 = node_e_id;
            node_d->parent = node_b_id;
            node_e->parent = node_a_id;
        } else {
            node_b->child_1 = node_e_id;
            node_a->child_1 = node_d_id;
            node_e->parent = node_b_id;
            node_d->parent = node_a_id;
        }
        sl_tree_node_refit(tree, node_a_id);
        sl_tree_node_refit(tree, node_b_id);
        return node_b_id;
    }
    return node_a_id;
}

static void sl_tree_refit_upward(sl_tree *tree, sl_tree_root_kind root,
                                 uint32_t node_id)
{
    uint32_t steps = 0u;
    while (node_id != SL_TREE_NODE_NONE && steps < tree->node_capacity) {
        steps += 1u;
        node_id = sl_tree_balance(tree, root, node_id);
        sl_tree_node *node = &tree->nodes[node_id];
        if (!sl_tree_node_is_leaf(node)) {
            sl_tree_node_refit(tree, node_id);
        }
        node_id = node->parent;
    }
    SL_ASSERT(node_id == SL_TREE_NODE_NONE);
}

static void sl_tree_leaf_insert(sl_tree *tree, sl_tree_root_kind root,
                                uint32_t leaf_id)
{
    const uint32_t root_id = tree->roots[root];
    if (root_id == SL_TREE_NODE_NONE) {
        tree->roots[root] = leaf_id;
        tree->nodes[leaf_id].parent = SL_TREE_NODE_NONE;
        return;
    }

    const sl_aabb leaf_aabb = tree->nodes[leaf_id].aabb;
    uint32_t sibling_id = root_id;
    uint32_t steps = 0u;
    while (!sl_tree_node_is_leaf(&tree->nodes[sibling_id])) {
        if (steps >= tree->node_capacity) {
            SL_ASSERT(false);
            return;
        }
        steps += 1u;
        const sl_tree_node *sibling = &tree->nodes[sibling_id];
        const uint32_t child_1_id = sibling->child_1;
        const uint32_t child_2_id = sibling->child_2;
        const sl_aabb combined = sl_aabb_union(sibling->aabb, leaf_aabb);
        const float combined_cost = sl_aabb_perimeter(combined);
        const float direct_cost = 2.0f * combined_cost;
        const float inheritance_cost =
            2.0f * (combined_cost - sl_aabb_perimeter(sibling->aabb));

        const sl_tree_node *child_1 = &tree->nodes[child_1_id];
        const sl_tree_node *child_2 = &tree->nodes[child_2_id];
        const float cost_1 =
            (sl_tree_node_is_leaf(child_1)
                 ? sl_aabb_perimeter(sl_aabb_union(leaf_aabb, child_1->aabb))
                 : sl_aabb_perimeter(sl_aabb_union(leaf_aabb, child_1->aabb)) -
                       sl_aabb_perimeter(child_1->aabb)) +
            inheritance_cost;
        const float cost_2 =
            (sl_tree_node_is_leaf(child_2)
                 ? sl_aabb_perimeter(sl_aabb_union(leaf_aabb, child_2->aabb))
                 : sl_aabb_perimeter(sl_aabb_union(leaf_aabb, child_2->aabb)) -
                       sl_aabb_perimeter(child_2->aabb)) +
            inheritance_cost;
        if (direct_cost < cost_1 && direct_cost < cost_2) {
            break;
        }
        sibling_id = (cost_1 <= cost_2) ? child_1_id : child_2_id;
    }

    const uint32_t old_parent = tree->nodes[sibling_id].parent;
    const uint32_t new_parent_id = sl_tree_node_allocate(tree);
    SL_ASSERT(new_parent_id != SL_TREE_NODE_NONE);
    sl_tree_node *new_parent = &tree->nodes[new_parent_id];
    new_parent->parent = old_parent;
    new_parent->aabb = sl_aabb_union(leaf_aabb, tree->nodes[sibling_id].aabb);
    new_parent->height = tree->nodes[sibling_id].height + 1u;
    new_parent->child_1 = sibling_id;
    new_parent->child_2 = leaf_id;
    new_parent->user = SL_TREE_NODE_NONE;
    tree->nodes[sibling_id].parent = new_parent_id;
    tree->nodes[leaf_id].parent = new_parent_id;

    if (old_parent == SL_TREE_NODE_NONE) {
        tree->roots[root] = new_parent_id;
    } else {
        sl_tree_node *parent = &tree->nodes[old_parent];
        if (parent->child_1 == sibling_id) {
            parent->child_1 = new_parent_id;
        } else {
            SL_ASSERT(parent->child_2 == sibling_id);
            parent->child_2 = new_parent_id;
        }
    }
    sl_tree_refit_upward(tree, root, new_parent_id);
}

static void sl_tree_leaf_remove(sl_tree *tree, sl_tree_root_kind root,
                                uint32_t leaf_id)
{
    if (tree->roots[root] == leaf_id) {
        tree->roots[root] = SL_TREE_NODE_NONE;
        tree->nodes[leaf_id].parent = SL_TREE_NODE_NONE;
        return;
    }

    const uint32_t parent_id = tree->nodes[leaf_id].parent;
    SL_ASSERT(parent_id != SL_TREE_NODE_NONE);
    const sl_tree_node *parent = &tree->nodes[parent_id];
    const uint32_t sibling_id =
        (parent->child_1 == leaf_id) ? parent->child_2 : parent->child_1;
    const uint32_t grand_parent_id = parent->parent;
    if (grand_parent_id == SL_TREE_NODE_NONE) {
        tree->roots[root] = sibling_id;
        tree->nodes[sibling_id].parent = SL_TREE_NODE_NONE;
    } else {
        sl_tree_node *grand_parent = &tree->nodes[grand_parent_id];
        if (grand_parent->child_1 == parent_id) {
            grand_parent->child_1 = sibling_id;
        } else {
            SL_ASSERT(grand_parent->child_2 == parent_id);
            grand_parent->child_2 = sibling_id;
        }
        tree->nodes[sibling_id].parent = grand_parent_id;
    }
    tree->nodes[leaf_id].parent = SL_TREE_NODE_NONE;
    sl_tree_node_free(tree, parent_id);
    if (grand_parent_id != SL_TREE_NODE_NONE) {
        sl_tree_refit_upward(tree, root, grand_parent_id);
    }
}

static bool sl_tree_proxy_belongs_to(const sl_tree *tree,
                                     sl_tree_root_kind root, uint32_t proxy)
{
    if (tree == NULL || tree->nodes == NULL || !sl_tree_root_valid(root) ||
        proxy >= tree->node_capacity ||
        tree->nodes[proxy].height == SL_TREE_HEIGHT_FREE ||
        !sl_tree_node_is_leaf(&tree->nodes[proxy])) {
        return false;
    }

    uint32_t node_id = proxy;
    for (uint32_t steps = 0u; steps < tree->node_capacity; ++steps) {
        const uint32_t parent = tree->nodes[node_id].parent;
        if (parent == SL_TREE_NODE_NONE) {
            return tree->roots[root] == node_id;
        }
        if (parent >= tree->node_capacity) {
            return false;
        }
        node_id = parent;
    }
    return false;
}

uint32_t sl_tree_proxy_create(sl_tree *tree, sl_tree_root_kind root,
                              sl_aabb aabb, uint32_t user)
{
    if (tree == NULL || tree->nodes == NULL || !sl_tree_root_valid(root) ||
        !sl_aabb_is_valid(aabb) || user == SL_TREE_NODE_NONE ||
        tree->proxy_count >= tree->proxy_capacity) {
        return SL_TREE_NODE_NONE;
    }
    const uint32_t leaf_id = sl_tree_node_allocate(tree);
    if (leaf_id == SL_TREE_NODE_NONE) {
        return SL_TREE_NODE_NONE;
    }
    sl_tree_node *leaf = &tree->nodes[leaf_id];
    leaf->aabb = aabb;
    leaf->user = user;
    sl_tree_leaf_insert(tree, root, leaf_id);
    tree->proxy_count += 1u;
    return leaf_id;
}

bool sl_tree_proxy_destroy(sl_tree *tree, sl_tree_root_kind root,
                           uint32_t proxy)
{
    if (!sl_tree_proxy_belongs_to(tree, root, proxy)) {
        return false;
    }
    sl_tree_leaf_remove(tree, root, proxy);
    sl_tree_node_free(tree, proxy);
    tree->proxy_count -= 1u;
    return true;
}

bool sl_tree_proxy_move(sl_tree *tree, sl_tree_root_kind root, uint32_t proxy,
                        sl_aabb aabb)
{
    if (!sl_tree_proxy_belongs_to(tree, root, proxy) ||
        !sl_aabb_is_valid(aabb)) {
        return false;
    }
    sl_tree_leaf_remove(tree, root, proxy);
    tree->nodes[proxy].aabb = aabb;
    sl_tree_leaf_insert(tree, root, proxy);
    return true;
}

sl_tree_query_result sl_tree_query(sl_tree *tree, sl_tree_root_kind root,
                                   sl_aabb aabb, uint32_t *out_users,
                                   uint32_t out_capacity)
{
    sl_tree_query_result result = { 0 };
    if (tree == NULL || tree->nodes == NULL || !sl_tree_root_valid(root) ||
        !sl_aabb_is_valid(aabb) || (out_capacity > 0u && out_users == NULL)) {
        return result;
    }
    const uint32_t root_id = tree->roots[root];
    if (root_id == SL_TREE_NODE_NONE) {
        return result;
    }

    uint32_t stack_count = 0u;
    tree->stack[stack_count++] = root_id;
    while (stack_count > 0u) {
        const uint32_t node_id = tree->stack[--stack_count];
        result.node_visits += 1u;
        const sl_tree_node *node = &tree->nodes[node_id];
        if (!sl_aabb_overlaps(node->aabb, aabb)) {
            continue;
        }
        if (sl_tree_node_is_leaf(node)) {
            if (result.count < out_capacity) {
                out_users[result.count] = node->user;
            }
            result.count += 1u;
            continue;
        }
        SL_ASSERT(stack_count + 2u <= tree->node_capacity);
        tree->stack[stack_count++] = node->child_2;
        tree->stack[stack_count++] = node->child_1;
    }
    result.overflow = result.count > out_capacity;
    return result;
}

uint32_t sl_tree_height(const sl_tree *tree, sl_tree_root_kind root)
{
    if (tree == NULL || tree->nodes == NULL || !sl_tree_root_valid(root)) {
        return 0u;
    }
    const uint32_t root_id = tree->roots[root];
    return (root_id == SL_TREE_NODE_NONE) ? 0u : tree->nodes[root_id].height;
}
