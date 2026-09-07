#include "contact_world.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "collide.h"
#include "joint.h"
#include "silk/assert.h"
#include "stats.h"
#include "tree.h"

#define SL_CONTACT_CARVE_ALIGN ((size_t)8u)

/* Both nonzero states participate in pair deduplication until all moved
 * proxies have been queried. RETRY survives the final queue compaction. */
#define SL_PROXY_MOVED 1u
#define SL_PROXY_RETRY 2u

#define SL_CONTACT_ALIGNMENT_ASSERT(type)                                      \
    _Static_assert(_Alignof(type) <= SL_CONTACT_CARVE_ALIGN &&                 \
                       SL_CONTACT_CARVE_ALIGN % _Alignof(type) == 0u,          \
                   #type " alignment violates the contact arena contract")

_Static_assert((SL_CONTACT_CARVE_ALIGN & (SL_CONTACT_CARVE_ALIGN - 1u)) == 0u,
               "contact arena alignment must be a power of two");
SL_CONTACT_ALIGNMENT_ASSERT(sl_tree);
SL_CONTACT_ALIGNMENT_ASSERT(sl_aabb);
SL_CONTACT_ALIGNMENT_ASSERT(uint32_t);
SL_CONTACT_ALIGNMENT_ASSERT(uint8_t);
SL_CONTACT_ALIGNMENT_ASSERT(sl_contact);
SL_CONTACT_ALIGNMENT_ASSERT(uint64_t);

_Static_assert(sizeof(sl_contact) == 136u,
               "contact layout changed; re-pin the world arena budget");

#undef SL_CONTACT_ALIGNMENT_ASSERT

static size_t contact_align_up(size_t bytes)
{
    return (bytes + SL_CONTACT_CARVE_ALIGN - 1u) &
           ~(SL_CONTACT_CARVE_ALIGN - 1u);
}

static bool size_add(size_t a, size_t b, size_t *out)
{
    if (a > SIZE_MAX - b) {
        return false;
    }
    *out = a + b;
    return true;
}

static bool slice_size(size_t count, size_t element_size, size_t *out)
{
    if (count != 0u && element_size > SIZE_MAX / count) {
        return false;
    }
    const size_t bytes = count * element_size;
    if (bytes > SIZE_MAX - (SL_CONTACT_CARVE_ALIGN - 1u)) {
        return false;
    }
    *out = contact_align_up(bytes);
    return true;
}

static unsigned char *contact_carve(unsigned char *base, size_t *offset,
                                    size_t count, size_t element_size)
{
    size_t bytes = 0u;
    const bool valid = slice_size(count, element_size, &bytes);
    SL_ASSERT(valid);
    if (!valid) {
        return NULL;
    }
    unsigned char *slice = base + *offset;
    *offset += bytes;
    return slice;
}

uint32_t sl_contact_pair_capacity(uint32_t contact_capacity)
{
    if (contact_capacity < 1u || contact_capacity > SL_CONTACT_COUNT_MAX) {
        return 0u;
    }
    const uint32_t required = 2u * contact_capacity;
    uint32_t capacity = 1u;
    while (capacity < required) {
        capacity <<= 1u;
    }
    return capacity;
}

size_t sl_contact_world_memory_layout(uint32_t body_capacity,
                                      uint32_t contact_capacity,
                                      sl_world_memory_breakdown *out)
{
    if (body_capacity < 1u || body_capacity > SL_BODY_COUNT_MAX) {
        return 0u;
    }
    const uint32_t pair_capacity = sl_contact_pair_capacity(contact_capacity);
    if (pair_capacity == 0u) {
        return 0u;
    }

    size_t total = 0u;
    size_t payload = 0u;
    size_t slice = 0u;
#define ADD_SLICE(count, type)                                                 \
    do {                                                                       \
        if (!slice_size((size_t)(count), sizeof(type), &slice) ||              \
            !size_add(total, slice, &total)) {                                 \
            return 0u;                                                         \
        }                                                                      \
        payload += (size_t)(count) * sizeof(type);                             \
    } while (false)

    ADD_SLICE(1u, sl_tree);
    const size_t tree_bytes = sl_tree_memory_bytes(body_capacity);
    if (tree_bytes == 0u || !size_add(total, tree_bytes, &total)) {
        return 0u;
    }
    /* 2*body_capacity makes both tree arrays exact 8-byte multiples. */
    payload += tree_bytes;
    ADD_SLICE(body_capacity, sl_aabb);
    ADD_SLICE(body_capacity, uint32_t);
    ADD_SLICE(body_capacity, uint8_t);
    ADD_SLICE(body_capacity, uint32_t);
    ADD_SLICE(body_capacity, uint32_t);
    out->broadphase_bytes = payload;
    ADD_SLICE(contact_capacity, sl_contact);
    out->contact_bytes = (size_t)contact_capacity * sizeof(sl_contact);
    ADD_SLICE(pair_capacity, uint64_t);
    out->pair_bytes = (size_t)pair_capacity * sizeof(uint64_t);
    out->padding_bytes = total - payload;

#undef ADD_SLICE
    return total;
}

size_t sl_contact_world_memory_bytes(uint32_t body_capacity,
                                     uint32_t contact_capacity)
{
    sl_world_memory_breakdown ignored = { 0 };
    return sl_contact_world_memory_layout(body_capacity, contact_capacity,
                                          &ignored);
}

static sl_tree *world_tree(sl_world *world)
{
    return (sl_tree *)world->tree;
}

bool sl_contact_world_init(sl_world *world, void *memory, size_t memory_bytes)
{
    SL_ASSERT(world != NULL);
    const size_t required = sl_contact_world_memory_bytes(
        world->body_capacity, world->contact_capacity);
    if (memory == NULL || required == 0u || memory_bytes < required ||
        ((uintptr_t)memory & (SL_CONTACT_CARVE_ALIGN - 1u)) != 0u) {
        return false;
    }

    unsigned char *base = memory;
    size_t offset = 0u;
    world->tree = contact_carve(base, &offset, 1u, sizeof(sl_tree));
    const size_t tree_bytes = sl_tree_memory_bytes(world->body_capacity);
    void *tree_memory = contact_carve(base, &offset, tree_bytes, 1u);
    world->proxy_aabbs = (sl_aabb *)contact_carve(
        base, &offset, world->body_capacity, sizeof(sl_aabb));
    world->proxies = (uint32_t *)contact_carve(
        base, &offset, world->body_capacity, sizeof(uint32_t));
    world->moved = (uint8_t *)contact_carve(base, &offset, world->body_capacity,
                                            sizeof(uint8_t));
    world->moved_slots = (uint32_t *)contact_carve(
        base, &offset, world->body_capacity, sizeof(uint32_t));
    world->query_slots = (uint32_t *)contact_carve(
        base, &offset, world->body_capacity, sizeof(uint32_t));
    world->contacts = (sl_contact *)contact_carve(
        base, &offset, world->contact_capacity, sizeof(sl_contact));
    world->pair_keys = (uint64_t *)contact_carve(
        base, &offset, world->pair_capacity, sizeof(uint64_t));
    SL_ASSERT(offset == required);

    if (!sl_tree_init(world_tree(world), tree_memory, tree_bytes,
                      world->body_capacity)) {
        return false;
    }
    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        world->proxies[i] = SL_TREE_NODE_NONE;
    }
    return true;
}

static sl_tree_root_kind body_root(uint8_t type)
{
    switch ((sl_body_type)type) {
    case SL_BODY_DYNAMIC:
        return SL_TREE_ROOT_DYNAMIC;
    case SL_BODY_KINEMATIC:
        return SL_TREE_ROOT_KINEMATIC;
    case SL_BODY_STATIC:
        return SL_TREE_ROOT_STATIC;
    }
    SL_ASSERT(false);
    return SL_TREE_ROOT_STATIC;
}

static sl_aabb body_tight_aabb(const sl_world *world, uint32_t dense)
{
    const sl_transform transform =
        sl_transform_make(world->positions[dense], world->rotations[dense]);
    return sl_aabb_extend(sl_shape_aabb(&world->shapes[dense], transform),
                          SL_SPECULATIVE_DISTANCE);
}

static void moved_mark(sl_world *world, uint32_t dense, uint32_t slot)
{
    if (world->moved[dense] != 0u) {
        return;
    }
    SL_ASSERT(world->moved_count < world->body_capacity);
    if (world->moved_count >= world->body_capacity) {
        return;
    }
    world->moved[dense] = SL_PROXY_MOVED;
    world->moved_slots[world->moved_count] = slot;
    world->moved_count += 1u;
}

static void moved_remove(sl_world *world, uint32_t dense, uint32_t slot)
{
    if (world->moved[dense] == 0u) {
        return;
    }
    uint32_t write = 0u;
    for (uint32_t read = 0u; read < world->moved_count; ++read) {
        if (world->moved_slots[read] != slot) {
            world->moved_slots[write] = world->moved_slots[read];
            write += 1u;
        }
    }
    world->moved_count = write;
    world->moved[dense] = 0u;
}

bool sl_contact_body_create(sl_world *world, uint32_t dense, uint32_t slot)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(dense < world->body_count);
    SL_ASSERT(slot < world->body_capacity);
    world->proxies[dense] = SL_TREE_NODE_NONE;
    world->moved[dense] = 0u;
    world->proxy_aabbs[dense] =
        sl_aabb_make(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f));

    if (world->shapes[dense].kind == SL_SHAPE_NONE) {
        return true;
    }
    const sl_aabb fat_aabb =
        sl_aabb_extend(body_tight_aabb(world, dense), SL_AABB_MARGIN);
    const uint32_t proxy = sl_tree_proxy_create(
        world_tree(world), body_root(world->types[dense]), fat_aabb, slot);
    if (proxy == SL_TREE_NODE_NONE) {
        return false;
    }
    SL_WORK_ADD(world, proxy_creates, 1u);
    world->proxies[dense] = proxy;
    world->proxy_aabbs[dense] = fat_aabb;
    moved_mark(world, dense, slot);
    return true;
}

void sl_contact_body_destroy(sl_world *world, uint32_t dense, uint32_t slot)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(dense < world->body_count);
    moved_remove(world, dense, slot);
    if (world->proxies[dense] == SL_TREE_NODE_NONE) {
        return;
    }
    const bool destroyed =
        sl_tree_proxy_destroy(world_tree(world), body_root(world->types[dense]),
                              world->proxies[dense]);
    SL_ASSERT(destroyed);
    (void)destroyed;
    SL_WORK_ADD(world, proxy_destroys, 1u);
    world->proxies[dense] = SL_TREE_NODE_NONE;
}

bool sl_contact_body_update(sl_world *world, uint32_t dense, uint32_t slot)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(dense < world->body_count);
    SL_ASSERT(slot < world->body_capacity);

    if (world->shapes[dense].kind == SL_SHAPE_NONE) {
        sl_contact_body_destroy(world, dense, slot);
        world->proxy_aabbs[dense] =
            sl_aabb_make(sl_vec2_make(0.0f, 0.0f), sl_vec2_make(0.0f, 0.0f));
        return true;
    }
    if (world->proxies[dense] == SL_TREE_NODE_NONE) {
        return sl_contact_body_create(world, dense, slot);
    }

    const sl_aabb tight_aabb = body_tight_aabb(world, dense);
    if (sl_aabb_contains(world->proxy_aabbs[dense], tight_aabb)) {
        return true;
    }
    const sl_aabb fat_aabb = sl_aabb_extend(tight_aabb, SL_AABB_MARGIN);
    const bool moved =
        sl_tree_proxy_move(world_tree(world), body_root(world->types[dense]),
                           world->proxies[dense], fat_aabb);
    SL_ASSERT(moved);
    if (!moved) {
        return false;
    }
    SL_WORK_ADD(world, proxy_moves, 1u);
    world->proxy_aabbs[dense] = fat_aabb;
    moved_mark(world, dense, slot);
    return true;
}

void sl_contact_world_reset(sl_world *world)
{
    SL_ASSERT(world != NULL);
    sl_tree *tree = world_tree(world);
    void *tree_memory = tree->memory;
    const size_t tree_bytes = sl_tree_memory_bytes(world->body_capacity);
    const bool initialized =
        sl_tree_init(tree, tree_memory, tree_bytes, world->body_capacity);
    SL_ASSERT(initialized);
    (void)initialized;
    world->contact_count = 0u;
    world->contact_drop_count = 0u;
    world->moved_count = 0u;
    memset(world->moved, 0,
           (size_t)world->body_capacity * sizeof(world->moved[0]));
    memset(world->pair_keys, 0,
           (size_t)world->pair_capacity * sizeof(world->pair_keys[0]));
    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        world->proxies[i] = SL_TREE_NODE_NONE;
    }
}

static uint64_t pair_key(uint32_t slot_a, uint32_t slot_b)
{
    const uint32_t low = (slot_a < slot_b) ? slot_a : slot_b;
    const uint32_t high = (slot_a < slot_b) ? slot_b : slot_a;
    return ((uint64_t)low << 32u) | (uint64_t)high;
}

static uint32_t pair_hash(uint64_t key)
{
    key ^= key >> 30u;
    key *= UINT64_C(0xbf58476d1ce4e5b9);
    key ^= key >> 27u;
    key *= UINT64_C(0x94d049bb133111eb);
    key ^= key >> 31u;
    return (uint32_t)key;
}

static bool pair_find(sl_world *world, uint64_t key, uint32_t *out)
{
    SL_ASSERT(key != 0u);
    const uint32_t mask = world->pair_capacity - 1u;
    uint32_t index = pair_hash(key) & mask;
    for (uint32_t probe = 0u; probe < world->pair_capacity; ++probe) {
        const uint64_t stored = world->pair_keys[index];
        if (stored == 0u) {
            SL_WORK_ADD(world, pair_probes, (uint64_t)probe + 1u);
            *out = index;
            return false;
        }
        if (stored == key) {
            SL_WORK_ADD(world, pair_probes, (uint64_t)probe + 1u);
            *out = index;
            return true;
        }
        index = (index + 1u) & mask;
    }
    SL_ASSERT(false);
    SL_WORK_ADD(world, pair_probes, world->pair_capacity);
    *out = 0u;
    return false;
}

static bool pair_insert(sl_world *world, uint64_t key)
{
    uint32_t index = 0u;
    if (pair_find(world, key, &index)) {
        return false;
    }
    world->pair_keys[index] = key;
    return true;
}

static void pair_remove(sl_world *world, uint64_t key)
{
    uint32_t index = 0u;
    const bool found = pair_find(world, key, &index);
    SL_ASSERT(found);
    if (!found) {
        return;
    }

    const uint32_t mask = world->pair_capacity - 1u;
    world->pair_keys[index] = 0u;
    index = (index + 1u) & mask;
    for (uint32_t probe = 0u; probe < world->pair_capacity; ++probe) {
        const uint64_t shifted = world->pair_keys[index];
        if (shifted == 0u) {
            SL_WORK_ADD(world, pair_probes, (uint64_t)probe + 1u);
            return;
        }
        world->pair_keys[index] = 0u;
        const bool inserted = pair_insert(world, shifted);
        SL_ASSERT(inserted);
        (void)inserted;
        index = (index + 1u) & mask;
    }
    SL_ASSERT(false);
}

static float friction_mix(float friction_a, float friction_b)
{
    float mixed = sqrtf(friction_a) * sqrtf(friction_b);
    if (!sl_is_finite(mixed)) {
        const float low = sl_min(friction_a, friction_b);
        const float high = sl_max(friction_a, friction_b);
        mixed = (high == 0.0f) ? 0.0f : high * sqrtf(low / high);
    }
    SL_ASSERT(sl_is_finite(mixed) && mixed >= 0.0f);
    return (sl_is_finite(mixed) && mixed >= 0.0f) ? mixed : FLT_MAX;
}

static void manifold_persist(sl_manifold *manifold, const sl_manifold *previous)
{
    for (uint32_t i = 0u; i < manifold->point_count; ++i) {
        sl_manifold_point *point = &manifold->points[i];
        point->persisted = false;
        for (uint32_t j = 0u; j < previous->point_count; ++j) {
            const sl_manifold_point *old = &previous->points[j];
            if (point->id == old->id) {
                point->normal_impulse = old->normal_impulse;
                point->tangent_impulse = old->tangent_impulse;
                point->persisted = true;
                break;
            }
        }
    }
}

static void contact_refresh(sl_world *world, sl_contact *contact)
{
    const uint32_t dense_a = world->slots[contact->body_a.index].dense;
    const uint32_t dense_b = world->slots[contact->body_b.index].dense;
    const sl_transform transform_a =
        sl_transform_make(world->positions[dense_a], world->rotations[dense_a]);
    const sl_transform transform_b =
        sl_transform_make(world->positions[dense_b], world->rotations[dense_b]);
    const sl_manifold previous = contact->manifold;
    contact->manifold = sl_collide_shapes(&world->shapes[dense_a], transform_a,
                                          &world->shapes[dense_b], transform_b);
    manifold_persist(&contact->manifold, &previous);
    contact->friction =
        friction_mix(world->frictions[dense_a], world->frictions[dense_b]);
    contact->restitution =
        sl_max(world->restitutions[dense_a], world->restitutions[dense_b]);
    contact->touching = contact->manifold.point_count > 0u;
}

static void contact_remove(sl_world *world, uint32_t row)
{
    SL_ASSERT(row < world->contact_count);
    const sl_contact *contact = &world->contacts[row];
    pair_remove(world, pair_key(contact->body_a.index, contact->body_b.index));
    const uint32_t last = world->contact_count - 1u;
    if (row != last) {
        world->contacts[row] = world->contacts[last];
    }
    world->contact_count = last;
}

void sl_contact_pair_destroy(sl_world *world, uint32_t slot_a, uint32_t slot_b)
{
    SL_ASSERT(world != NULL);
    if (slot_a == slot_b || slot_a >= world->body_capacity ||
        slot_b >= world->body_capacity) {
        return;
    }
    uint32_t index = 0u;
    if (!pair_find(world, pair_key(slot_a, slot_b), &index)) {
        return;
    }
    (void)index;
    for (uint32_t row = 0u; row < world->contact_count; ++row) {
        const sl_contact *contact = &world->contacts[row];
        if (pair_key(contact->body_a.index, contact->body_b.index) ==
            pair_key(slot_a, slot_b)) {
            contact_remove(world, row);
            return;
        }
    }
    SL_ASSERT(false);
}

void sl_contact_body_mark_moved(sl_world *world, uint32_t slot)
{
    SL_ASSERT(world != NULL);
    if (slot >= world->body_capacity ||
        world->slots[slot].dense == SL_BODY_DENSE_NONE) {
        return;
    }
    const uint32_t dense = world->slots[slot].dense;
    if (world->proxies[dense] != SL_TREE_NODE_NONE) {
        moved_mark(world, dense, slot);
    }
}

static bool contact_is_live(const sl_world *world, const sl_contact *contact)
{
    if (!sl_world_body_is_valid(world, contact->body_a) ||
        !sl_world_body_is_valid(world, contact->body_b)) {
        return false;
    }
    const uint32_t dense_a = world->slots[contact->body_a.index].dense;
    const uint32_t dense_b = world->slots[contact->body_b.index].dense;
    if (world->shapes[dense_a].kind == SL_SHAPE_NONE ||
        world->shapes[dense_b].kind == SL_SHAPE_NONE ||
        world->proxies[dense_a] == SL_TREE_NODE_NONE ||
        world->proxies[dense_b] == SL_TREE_NODE_NONE) {
        return false;
    }
    return sl_joint_pair_should_collide(world, contact->body_a.index,
                                        contact->body_b.index) &&
           sl_aabb_overlaps(world->proxy_aabbs[dense_a],
                            world->proxy_aabbs[dense_b]);
}

static void contacts_update(sl_world *world)
{
    uint32_t row = 0u;
    while (row < world->contact_count) {
        sl_contact *contact = &world->contacts[row];
        if (!contact_is_live(world, contact)) {
            contact_remove(world, row);
            continue;
        }
        contact_refresh(world, contact);
        row += 1u;
    }
}

static bool contact_create(sl_world *world, uint32_t slot_a, uint32_t slot_b)
{
    SL_ASSERT(slot_a != slot_b);
    if (world->contact_count >= world->contact_capacity) {
        return false;
    }
    const uint32_t low = (slot_a < slot_b) ? slot_a : slot_b;
    const uint32_t high = (slot_a < slot_b) ? slot_b : slot_a;
    const uint64_t key = pair_key(low, high);
    if (!pair_insert(world, key)) {
        return true;
    }

    sl_contact *contact = &world->contacts[world->contact_count];
    memset(contact, 0, sizeof(*contact));
    contact->body_a = (sl_body_handle){ low, world->slots[low].generation };
    contact->body_b = (sl_body_handle){ high, world->slots[high].generation };
    contact_refresh(world, contact);
    world->contact_count += 1u;
    if (world->contact_count > world->stats.contact_count_high) {
        world->stats.contact_count_high = world->contact_count;
    }
    return true;
}

static void pair_candidate(sl_world *world, uint32_t slot, uint32_t other_slot,
                           bool *dropped)
{
    if (other_slot == slot || other_slot >= world->body_capacity ||
        world->slots[other_slot].dense == SL_BODY_DENSE_NONE) {
        return;
    }
    const uint32_t dense = world->slots[slot].dense;
    const uint32_t other_dense = world->slots[other_slot].dense;
    if (world->moved[other_dense] != 0u && other_slot < slot) {
        return;
    }
    if (world->shapes[other_dense].kind == SL_SHAPE_NONE ||
        (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC &&
         world->types[other_dense] != (uint8_t)SL_BODY_DYNAMIC) ||
        !sl_joint_pair_should_collide(world, slot, other_slot)) {
        return;
    }
    uint32_t index = 0u;
    if (pair_find(world, pair_key(slot, other_slot), &index)) {
        return;
    }
    if (!contact_create(world, slot, other_slot)) {
        world->contact_drop_count += 1u;
        SL_WORK_ADD(world, contact_drops, 1u);
        *dropped = true;
    }
}

static void query_root(sl_world *world, uint32_t slot, sl_tree_root_kind root,
                       bool *dropped)
{
    const uint32_t dense = world->slots[slot].dense;
    const sl_tree_query_result result =
        sl_tree_query(world_tree(world), root, world->proxy_aabbs[dense],
                      world->query_slots, world->body_capacity);
    SL_WORK_ADD(world, tree_node_visits, result.node_visits);
    SL_WORK_ADD(world, pair_candidates, result.count);
    SL_ASSERT(!result.overflow && result.count <= world->body_capacity);
    const uint32_t count = (result.count <= world->body_capacity)
                               ? result.count
                               : world->body_capacity;
    for (uint32_t i = 0u; i < count; ++i) {
        pair_candidate(world, slot, world->query_slots[i], dropped);
    }
}

static void pairs_update(sl_world *world)
{
    const uint32_t snapshot_count = world->moved_count;
    for (uint32_t i = 0u; i < snapshot_count; ++i) {
        const uint32_t slot = world->moved_slots[i];
        if (slot >= world->body_capacity ||
            world->slots[slot].dense == SL_BODY_DENSE_NONE) {
            continue;
        }
        const uint32_t dense = world->slots[slot].dense;
        if (world->proxies[dense] == SL_TREE_NODE_NONE ||
            world->moved[dense] == 0u) {
            continue;
        }
        bool dropped = false;
        if (world->types[dense] == (uint8_t)SL_BODY_DYNAMIC) {
            query_root(world, slot, SL_TREE_ROOT_STATIC, &dropped);
            query_root(world, slot, SL_TREE_ROOT_KINEMATIC, &dropped);
            query_root(world, slot, SL_TREE_ROOT_DYNAMIC, &dropped);
        } else {
            query_root(world, slot, SL_TREE_ROOT_DYNAMIC, &dropped);
        }
        if (dropped) {
            world->moved[dense] = SL_PROXY_RETRY;
        }
    }

    /* Queries are finished, so flags can now be cleared and retries packed
     * into the same queue without changing duplicate suppression or order.
     * next_count never exceeds i: writes cannot overwrite unread slots. */
    uint32_t next_count = 0u;
    for (uint32_t i = 0u; i < snapshot_count; ++i) {
        const uint32_t slot = world->moved_slots[i];
        if (slot >= world->body_capacity ||
            world->slots[slot].dense == SL_BODY_DENSE_NONE) {
            continue;
        }
        const uint32_t dense = world->slots[slot].dense;
        if (world->moved[dense] == SL_PROXY_RETRY) {
            world->moved_slots[next_count] = slot;
            next_count += 1u;
            world->moved[dense] = SL_PROXY_MOVED;
        } else {
            world->moved[dense] = 0u;
        }
    }
    world->moved_count = next_count;
}

void sl_contact_step_begin(sl_world *world)
{
    SL_ASSERT(world != NULL);
    world->contact_drop_count = 0u;
    contacts_update(world);
    pairs_update(world);
}

void sl_contact_step_end(sl_world *world)
{
    SL_ASSERT(world != NULL);
    for (uint32_t dense = 0u; dense < world->body_count; ++dense) {
        const bool updated =
            sl_contact_body_update(world, dense, world->slot_of[dense]);
        SL_ASSERT(updated);
        (void)updated;
    }
}

uint32_t sl_world_contact_count(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->contact_count;
}

uint32_t sl_world_contact_capacity(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->contact_capacity;
}

uint32_t sl_world_contact_drop_count(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->contact_drop_count;
}

const sl_contact *sl_world_contact_at(const sl_world *world, uint32_t row)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(row < world->contact_count);
    return &world->contacts[row];
}

bool sl_world_body_get_proxy_aabb(const sl_world *world, sl_body_handle handle,
                                  sl_aabb *out)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(out != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->proxies[dense] == SL_TREE_NODE_NONE) {
        return false;
    }
    *out = world->proxy_aabbs[dense];
    return true;
}
