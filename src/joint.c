#include "joint.h"
#include "world_internal.h"

#include <stdint.h>
#include <string.h>

#include "contact_world.h"
#include "silk/assert.h"

#define SL_JOINT_CARVE_ALIGN ((size_t)8u)
#define SL_JOINT_KIND_INVALID UINT32_MAX

typedef struct sl_joint_softness {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} sl_joint_softness;

typedef struct sl_joint_constraint {
    uint32_t joint_row;
    uint32_t dense_a;
    uint32_t dense_b;
    uint32_t kind;
    sl_vec2 anchor_a;
    sl_vec2 anchor_b;
    sl_vec2 base_delta;
    sl_vec2 axis;
    sl_vec2 impulse;
    sl_vec2 step_impulse;
    sl_joint_softness softness;
    float distance_length;
    float axial_mass;
    float axial_impulse;
} sl_joint_constraint;

_Static_assert(_Alignof(sl_joint_constraint) <= SL_JOINT_CARVE_ALIGN &&
                   SL_JOINT_CARVE_ALIGN % _Alignof(sl_joint_constraint) == 0u,
               "joint constraint alignment violates the arena contract");
_Static_assert(sizeof(sl_joint_constraint) == 88u,
               "joint constraint layout changed; re-pin the arena budget");

static size_t align_up(size_t bytes)
{
    return (bytes + SL_JOINT_CARVE_ALIGN - 1u) & ~(SL_JOINT_CARVE_ALIGN - 1u);
}

static bool slice_size(size_t count, size_t element_size, size_t *out)
{
    if (count != 0u && element_size > SIZE_MAX / count) {
        return false;
    }
    const size_t bytes = count * element_size;
    if (bytes > SIZE_MAX - (SL_JOINT_CARVE_ALIGN - 1u)) {
        return false;
    }
    *out = align_up(bytes);
    return true;
}

static bool size_add(size_t a, size_t b, size_t *out)
{
    if (a > SIZE_MAX - b) {
        return false;
    }
    *out = a + b;
    return true;
}

static unsigned char *carve(unsigned char *base, size_t *offset, size_t count,
                            size_t element_size)
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

size_t sl_joint_memory_layout(uint32_t body_capacity, uint32_t joint_capacity,
                              size_t *payload)
{
    *payload = 0u;
    if (body_capacity < 1u || body_capacity > SL_BODY_COUNT_MAX ||
        joint_capacity > SL_JOINT_COUNT_MAX) {
        return 0u;
    }
    if (joint_capacity == 0u) {
        return 0u;
    }

    size_t total = 0u;
    size_t slice = 0u;
#define ADD_SLICE(count, type)                                                 \
    do {                                                                       \
        if (!slice_size((size_t)(count), sizeof(type), &slice) ||              \
            !size_add(total, slice, &total)) {                                 \
            return 0u;                                                         \
        }                                                                      \
        *payload += (size_t)(count) * sizeof(type);                            \
    } while (false)

    ADD_SLICE(joint_capacity, sl_joint_slot);
    ADD_SLICE(joint_capacity, uint32_t);
    ADD_SLICE(joint_capacity, uint32_t);
    ADD_SLICE(joint_capacity, uint8_t);
    ADD_SLICE(joint_capacity, sl_body_handle);
    ADD_SLICE(joint_capacity, sl_body_handle);
    ADD_SLICE(joint_capacity, sl_vec2);
    ADD_SLICE(joint_capacity, sl_vec2);
    ADD_SLICE(joint_capacity, uint8_t);
    ADD_SLICE(joint_capacity, float);
    ADD_SLICE(joint_capacity, float);
    ADD_SLICE(joint_capacity, sl_vec2);
    ADD_SLICE(joint_capacity, sl_vec2);
    ADD_SLICE(body_capacity, uint32_t);
    ADD_SLICE(body_capacity, uint32_t);
    ADD_SLICE(2u * (size_t)joint_capacity, uint32_t);
    ADD_SLICE(2u * (size_t)joint_capacity, uint32_t);
    ADD_SLICE(joint_capacity, sl_joint_constraint);

#undef ADD_SLICE
    return total;
}

size_t sl_joint_memory_bytes(uint32_t body_capacity, uint32_t joint_capacity)
{
    size_t payload = 0u;
    return sl_joint_memory_layout(body_capacity, joint_capacity, &payload);
}

bool sl_joint_world_init(sl_world_state *world, void *memory,
                         size_t memory_bytes)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u) {
        return memory == NULL && memory_bytes == 0u;
    }
    const size_t required =
        sl_joint_memory_bytes(world->body_capacity, world->joint_capacity);
    if (memory == NULL || required == 0u || memory_bytes < required ||
        ((uintptr_t)memory & (SL_JOINT_CARVE_ALIGN - 1u)) != 0u) {
        return false;
    }

    unsigned char *base = memory;
    size_t offset = 0u;
    const size_t capacity = world->joint_capacity;
    world->joint_slots =
        (sl_joint_slot *)carve(base, &offset, capacity, sizeof(sl_joint_slot));
    world->joint_slot_of =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->joint_free_indices =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->joint_kinds =
        (uint8_t *)carve(base, &offset, capacity, sizeof(uint8_t));
    world->joint_bodies_a = (sl_body_handle *)carve(base, &offset, capacity,
                                                    sizeof(sl_body_handle));
    world->joint_bodies_b = (sl_body_handle *)carve(base, &offset, capacity,
                                                    sizeof(sl_body_handle));
    world->joint_local_anchors_a =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->joint_local_anchors_b =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->joint_collide_connected =
        (uint8_t *)carve(base, &offset, capacity, sizeof(uint8_t));
    world->joint_distance_lengths =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->joint_distance_impulses =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->joint_revolute_impulses =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->joint_linear_impulses =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->body_joint_heads = (uint32_t *)carve(
        base, &offset, world->body_capacity, sizeof(uint32_t));
    world->body_joint_counts = (uint32_t *)carve(
        base, &offset, world->body_capacity, sizeof(uint32_t));
    world->joint_edge_prevs =
        (uint32_t *)carve(base, &offset, 2u * capacity, sizeof(uint32_t));
    world->joint_edge_nexts =
        (uint32_t *)carve(base, &offset, 2u * capacity, sizeof(uint32_t));
    world->joint_constraints =
        carve(base, &offset, capacity, sizeof(sl_joint_constraint));
    SL_ASSERT(offset == required);

    world->joint_count = 0u;
    world->joint_free_count = world->joint_capacity;
    world->joint_constraint_count = 0u;
    for (uint32_t i = 0u; i < world->joint_capacity; ++i) {
        world->joint_slots[i].dense = SL_BODY_DENSE_NONE;
        world->joint_slots[i].generation = 1u;
        world->joint_slot_of[i] = SL_BODY_DENSE_NONE;
        world->joint_free_indices[i] = world->joint_capacity - 1u - i;
        world->joint_edge_prevs[2u * i] = SL_JOINT_EDGE_NONE;
        world->joint_edge_prevs[2u * i + 1u] = SL_JOINT_EDGE_NONE;
        world->joint_edge_nexts[2u * i] = SL_JOINT_EDGE_NONE;
        world->joint_edge_nexts[2u * i + 1u] = SL_JOINT_EDGE_NONE;
    }
    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        world->body_joint_heads[i] = SL_JOINT_EDGE_NONE;
        world->body_joint_counts[i] = 0u;
    }
    return true;
}

static uint32_t generation_bump(uint32_t generation)
{
    generation += 1u;
    return (generation == 0u) ? 1u : generation;
}

void sl_joint_world_reset(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u) {
        return;
    }
    world->joint_count = 0u;
    world->joint_free_count = world->joint_capacity;
    world->joint_constraint_count = 0u;
    for (uint32_t i = 0u; i < world->joint_capacity; ++i) {
        world->joint_slots[i].dense = SL_BODY_DENSE_NONE;
        world->joint_slots[i].generation =
            generation_bump(world->joint_slots[i].generation);
        world->joint_slot_of[i] = SL_BODY_DENSE_NONE;
        world->joint_free_indices[i] = world->joint_capacity - 1u - i;
        world->joint_edge_prevs[2u * i] = SL_JOINT_EDGE_NONE;
        world->joint_edge_prevs[2u * i + 1u] = SL_JOINT_EDGE_NONE;
        world->joint_edge_nexts[2u * i] = SL_JOINT_EDGE_NONE;
        world->joint_edge_nexts[2u * i + 1u] = SL_JOINT_EDGE_NONE;
    }
    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        world->body_joint_heads[i] = SL_JOINT_EDGE_NONE;
        world->body_joint_counts[i] = 0u;
    }
}

static bool joint_kind_valid(sl_joint_kind kind)
{
    return kind == SL_JOINT_DISTANCE || kind == SL_JOINT_REVOLUTE;
}

static bool anchor_valid(sl_vec2 anchor)
{
    return sl_vec2_is_finite(anchor) &&
           sl_abs(anchor.x) <= SL_SHAPE_EXTENT_MAX &&
           sl_abs(anchor.y) <= SL_SHAPE_EXTENT_MAX;
}

static float axial_mass_make(const sl_world_state *world, uint32_t dense_a,
                             uint32_t dense_b, sl_vec2 anchor_a,
                             sl_vec2 anchor_b, sl_vec2 axis);
static sl_mat2 effective_matrix_make(const sl_world_state *world,
                                     uint32_t dense_a, uint32_t dense_b,
                                     sl_vec2 anchor_a, sl_vec2 anchor_b);

static bool effective_matrix_solvable(sl_mat2 matrix)
{
    sl_vec2 column = sl_vec2_make(0.0f, 0.0f);
    return sl_mat2_solve(matrix, sl_vec2_make(1.0f, 0.0f), &column) &&
           sl_mat2_solve(matrix, sl_vec2_make(0.0f, 1.0f), &column);
}

static void edge_link(sl_world_state *world, uint32_t joint_slot,
                      uint32_t endpoint, uint32_t body_slot)
{
    const uint32_t edge = 2u * joint_slot + endpoint;
    const uint32_t head = world->body_joint_heads[body_slot];
    world->joint_edge_prevs[edge] = SL_JOINT_EDGE_NONE;
    world->joint_edge_nexts[edge] = head;
    if (head != SL_JOINT_EDGE_NONE) {
        world->joint_edge_prevs[head] = edge;
    }
    world->body_joint_heads[body_slot] = edge;
    world->body_joint_counts[body_slot] += 1u;
}

static void edge_unlink(sl_world_state *world, uint32_t joint_slot,
                        uint32_t endpoint, uint32_t body_slot)
{
    const uint32_t edge = 2u * joint_slot + endpoint;
    const uint32_t previous = world->joint_edge_prevs[edge];
    const uint32_t next = world->joint_edge_nexts[edge];
    if (previous == SL_JOINT_EDGE_NONE) {
        SL_ASSERT(world->body_joint_heads[body_slot] == edge);
        world->body_joint_heads[body_slot] = next;
    } else {
        world->joint_edge_nexts[previous] = next;
    }
    if (next != SL_JOINT_EDGE_NONE) {
        world->joint_edge_prevs[next] = previous;
    }
    world->joint_edge_prevs[edge] = SL_JOINT_EDGE_NONE;
    world->joint_edge_nexts[edge] = SL_JOINT_EDGE_NONE;
    SL_ASSERT(world->body_joint_counts[body_slot] > 0u);
    world->body_joint_counts[body_slot] -= 1u;
}

bool sl_joint_pair_should_collide(const sl_world_state *world,
                                  uint32_t body_slot_a, uint32_t body_slot_b)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u || body_slot_a >= world->body_capacity ||
        body_slot_b >= world->body_capacity || body_slot_a == body_slot_b) {
        return true;
    }

    uint32_t scan = body_slot_a;
    uint32_t other = body_slot_b;
    if (world->body_joint_counts[body_slot_b] <
        world->body_joint_counts[body_slot_a]) {
        scan = body_slot_b;
        other = body_slot_a;
    }
    uint32_t edge = world->body_joint_heads[scan];
    for (uint32_t visited = 0u;
         edge != SL_JOINT_EDGE_NONE && visited < world->body_joint_counts[scan];
         ++visited) {
        const uint32_t joint_slot = edge / 2u;
        const uint32_t dense = world->joint_slots[joint_slot].dense;
        SL_ASSERT(dense != SL_BODY_DENSE_NONE);
        const sl_body_handle body_a = world->joint_bodies_a[dense];
        const sl_body_handle body_b = world->joint_bodies_b[dense];
        const uint32_t connected =
            (body_a.index == scan) ? body_b.index : body_a.index;
        if (connected == other && world->joint_collide_connected[dense] == 0u) {
            return false;
        }
        edge = world->joint_edge_nexts[edge];
    }
    return true;
}

bool sl_joint_is_valid(const sl_world_state *world, sl_joint_handle handle)
{
    SL_ASSERT(world != NULL);
    if (handle.generation == 0u || handle.index >= world->joint_capacity) {
        return false;
    }
    const sl_joint_slot *slot = &world->joint_slots[handle.index];
    return slot->generation == handle.generation &&
           slot->dense != SL_BODY_DENSE_NONE;
}

sl_joint_handle sl_world_joint_create(sl_world *owner,
                                      const sl_joint_desc *desc)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(desc != NULL);
    if (world->joint_capacity == 0u || world->joint_free_count == 0u ||
        !joint_kind_valid(desc->kind) ||
        !sl_body_is_valid(world, desc->body_a) ||
        !sl_body_is_valid(world, desc->body_b) ||
        desc->body_a.index == desc->body_b.index ||
        !anchor_valid(desc->local_anchor_a) ||
        !anchor_valid(desc->local_anchor_b)) {
        return sl_joint_handle_null();
    }
    const uint32_t dense_a = world->slots[desc->body_a.index].dense;
    const uint32_t dense_b = world->slots[desc->body_b.index].dense;
    if (world->types[dense_a] != (uint8_t)SL_BODY_DYNAMIC &&
        world->types[dense_b] != (uint8_t)SL_BODY_DYNAMIC) {
        return sl_joint_handle_null();
    }

    float distance_length = 0.0f;
    const sl_vec2 offset_a =
        sl_rotation_apply(world->rotations[dense_a], desc->local_anchor_a);
    const sl_vec2 offset_b =
        sl_rotation_apply(world->rotations[dense_b], desc->local_anchor_b);
    if (desc->kind == SL_JOINT_DISTANCE) {
        distance_length = desc->distance.length;
        if (!sl_is_finite(distance_length) ||
            distance_length < SL_LINEAR_SLOP ||
            distance_length > 2.0f * SL_POSITION_ABS_MAX) {
            return sl_joint_handle_null();
        }
        const sl_vec2 anchor_a =
            sl_transform_apply(sl_transform_make(world->positions[dense_a],
                                                 world->rotations[dense_a]),
                               desc->local_anchor_a);
        const sl_vec2 anchor_b =
            sl_transform_apply(sl_transform_make(world->positions[dense_b],
                                                 world->rotations[dense_b]),
                               desc->local_anchor_b);
        const sl_vec2 delta = sl_vec2_sub(anchor_b, anchor_a);
        const float length_sq = sl_vec2_length_sq(delta);
        if (!sl_vec2_is_finite(delta) || !sl_is_finite(length_sq)) {
            return sl_joint_handle_null();
        }
        sl_vec2 axis = sl_vec2_normalize(delta);
        if (sl_vec2_length_sq(axis) == 0.0f) {
            axis = sl_vec2_make(1.0f, 0.0f);
        }
        if (axial_mass_make(world, dense_a, dense_b, offset_a, offset_b,
                            axis) == 0.0f) {
            return sl_joint_handle_null();
        }
    } else {
        const sl_mat2 matrix =
            effective_matrix_make(world, dense_a, dense_b, offset_a, offset_b);
        if (!effective_matrix_solvable(matrix)) {
            return sl_joint_handle_null();
        }
    }

    const uint32_t slot =
        world->joint_free_indices[world->joint_free_count - 1u];
    const uint32_t dense = world->joint_count;
    world->joint_free_count -= 1u;
    world->joint_count += 1u;
    if (world->joint_count > world->joint_count_high) {
        world->joint_count_high = world->joint_count;
    }
    world->joint_slots[slot].dense = dense;
    world->joint_slot_of[dense] = slot;
    world->joint_kinds[dense] = (uint8_t)desc->kind;
    world->joint_bodies_a[dense] = desc->body_a;
    world->joint_bodies_b[dense] = desc->body_b;
    world->joint_local_anchors_a[dense] = desc->local_anchor_a;
    world->joint_local_anchors_b[dense] = desc->local_anchor_b;
    world->joint_collide_connected[dense] = desc->collide_connected ? 1u : 0u;
    world->joint_distance_lengths[dense] = distance_length;
    world->joint_distance_impulses[dense] = 0.0f;
    world->joint_revolute_impulses[dense] = sl_vec2_make(0.0f, 0.0f);
    world->joint_linear_impulses[dense] = sl_vec2_make(0.0f, 0.0f);
    edge_link(world, slot, 0u, desc->body_a.index);
    edge_link(world, slot, 1u, desc->body_b.index);

    if (!desc->collide_connected) {
        sl_contact_pair_destroy(world, desc->body_a.index, desc->body_b.index);
    }
    return (sl_joint_handle){ slot, world->joint_slots[slot].generation };
}

void sl_joint_destroy(sl_world_state *world, sl_joint_handle handle)
{
    SL_ASSERT(world != NULL);
    if (!sl_joint_is_valid(world, handle)) {
        return;
    }
    sl_joint_slot *slot = &world->joint_slots[handle.index];
    const uint32_t dense = slot->dense;
    const uint32_t last = world->joint_count - 1u;
    const uint32_t body_a = world->joint_bodies_a[dense].index;
    const uint32_t body_b = world->joint_bodies_b[dense].index;
    const bool suppressed = world->joint_collide_connected[dense] == 0u;
    edge_unlink(world, handle.index, 0u, body_a);
    edge_unlink(world, handle.index, 1u, body_b);

    if (dense != last) {
        world->joint_kinds[dense] = world->joint_kinds[last];
        world->joint_bodies_a[dense] = world->joint_bodies_a[last];
        world->joint_bodies_b[dense] = world->joint_bodies_b[last];
        world->joint_local_anchors_a[dense] =
            world->joint_local_anchors_a[last];
        world->joint_local_anchors_b[dense] =
            world->joint_local_anchors_b[last];
        world->joint_collide_connected[dense] =
            world->joint_collide_connected[last];
        world->joint_distance_lengths[dense] =
            world->joint_distance_lengths[last];
        world->joint_distance_impulses[dense] =
            world->joint_distance_impulses[last];
        world->joint_revolute_impulses[dense] =
            world->joint_revolute_impulses[last];
        world->joint_linear_impulses[dense] =
            world->joint_linear_impulses[last];
        const uint32_t moved_slot = world->joint_slot_of[last];
        world->joint_slot_of[dense] = moved_slot;
        world->joint_slots[moved_slot].dense = dense;
    }
    world->joint_count = last;
    slot->dense = SL_BODY_DENSE_NONE;
    slot->generation = generation_bump(slot->generation);
    world->joint_free_indices[world->joint_free_count] = handle.index;
    world->joint_free_count += 1u;

    if (suppressed && sl_joint_pair_should_collide(world, body_a, body_b)) {
        sl_contact_body_mark_moved(world, body_a);
        sl_contact_body_mark_moved(world, body_b);
    }
}

void sl_joint_body_destroy(sl_world_state *world, uint32_t body_slot)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u || body_slot >= world->body_capacity) {
        return;
    }
    while (world->body_joint_heads[body_slot] != SL_JOINT_EDGE_NONE) {
        const uint32_t joint_slot = world->body_joint_heads[body_slot] / 2u;
        const sl_joint_handle handle = {
            joint_slot, world->joint_slots[joint_slot].generation
        };
        sl_joint_destroy(world, handle);
    }
}

void sl_joint_body_cache_clear(sl_world_state *world, uint32_t body_slot)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u || body_slot >= world->body_capacity) {
        return;
    }
    uint32_t edge = world->body_joint_heads[body_slot];
    for (uint32_t visited = 0u; edge != SL_JOINT_EDGE_NONE &&
                                visited < world->body_joint_counts[body_slot];
         ++visited) {
        const uint32_t joint_slot = edge / 2u;
        const uint32_t dense = world->joint_slots[joint_slot].dense;
        SL_ASSERT(dense != SL_BODY_DENSE_NONE);
        world->joint_distance_impulses[dense] = 0.0f;
        world->joint_revolute_impulses[dense] = sl_vec2_make(0.0f, 0.0f);
        world->joint_linear_impulses[dense] = sl_vec2_make(0.0f, 0.0f);
        edge = world->joint_edge_nexts[edge];
    }
}

uint32_t sl_world_joint_count(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    return world != NULL ? world->joint_count : 0u;
}

uint32_t sl_world_joint_capacity(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    return world != NULL ? world->joint_capacity : 0u;
}

sl_joint_handle sl_world_joint_at(const sl_world *owner, uint32_t row)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(row < world->joint_count);
    const uint32_t slot = world->joint_slot_of[row];
    return (sl_joint_handle){ slot, world->joint_slots[slot].generation };
}

sl_joint_desc sl_world_joint_get_desc(const sl_world *owner,
                                      sl_joint_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_joint_is_valid(world, handle));
    const uint32_t dense = world->joint_slots[handle.index].dense;
    sl_joint_desc desc;
    memset(&desc, 0, sizeof(desc));
    desc.kind = (sl_joint_kind)world->joint_kinds[dense];
    desc.body_a = world->joint_bodies_a[dense];
    desc.body_b = world->joint_bodies_b[dense];
    desc.local_anchor_a = world->joint_local_anchors_a[dense];
    desc.local_anchor_b = world->joint_local_anchors_b[dense];
    desc.collide_connected = world->joint_collide_connected[dense] != 0u;
    if (desc.kind == SL_JOINT_DISTANCE) {
        desc.distance.length = world->joint_distance_lengths[dense];
    }
    return desc;
}

sl_vec2 sl_world_joint_get_linear_impulse(const sl_world *owner,
                                          sl_joint_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_joint_is_valid(world, handle));
    return world->joint_linear_impulses[world->joint_slots[handle.index].dense];
}

typedef struct sl_joint_velocity_pair {
    sl_vec2 linear_a;
    sl_vec2 linear_b;
    float angular_a;
    float angular_b;
    float inverse_mass_a;
    float inverse_mass_b;
    float inverse_inertia_a;
    float inverse_inertia_b;
    bool dynamic_a;
    bool dynamic_b;
} sl_joint_velocity_pair;

static sl_joint_velocity_pair
velocity_pair_load(const sl_world_state *world,
                   const sl_joint_constraint *constraint)
{
    const uint32_t dense_a = constraint->dense_a;
    const uint32_t dense_b = constraint->dense_b;
    const sl_joint_velocity_pair pair = {
        .linear_a = world->velocities[dense_a],
        .linear_b = world->velocities[dense_b],
        .angular_a = world->angular_velocities[dense_a],
        .angular_b = world->angular_velocities[dense_b],
        .inverse_mass_a = world->inv_masses[dense_a],
        .inverse_mass_b = world->inv_masses[dense_b],
        .inverse_inertia_a = world->inv_inertias[dense_a],
        .inverse_inertia_b = world->inv_inertias[dense_b],
        .dynamic_a = world->types[dense_a] == (uint8_t)SL_BODY_DYNAMIC,
        .dynamic_b = world->types[dense_b] == (uint8_t)SL_BODY_DYNAMIC,
    };
    return pair;
}

static void velocity_pair_store(sl_world_state *world,
                                const sl_joint_constraint *constraint,
                                const sl_joint_velocity_pair *pair)
{
    if (pair->dynamic_a) {
        world->velocities[constraint->dense_a] = pair->linear_a;
        world->angular_velocities[constraint->dense_a] = pair->angular_a;
    }
    if (pair->dynamic_b) {
        world->velocities[constraint->dense_b] = pair->linear_b;
        world->angular_velocities[constraint->dense_b] = pair->angular_b;
    }
}

static bool impulse_apply(sl_joint_velocity_pair *pair, sl_vec2 anchor_a,
                          sl_vec2 anchor_b, sl_vec2 impulse)
{
    if (!sl_vec2_is_finite(impulse)) {
        return false;
    }
    sl_joint_velocity_pair candidate = *pair;
    if (candidate.dynamic_a) {
        candidate.linear_a =
            sl_vec2_sub(candidate.linear_a,
                        sl_vec2_scale(impulse, candidate.inverse_mass_a));
        candidate.angular_a -=
            candidate.inverse_inertia_a * sl_vec2_cross(anchor_a, impulse);
    }
    if (candidate.dynamic_b) {
        candidate.linear_b =
            sl_vec2_add(candidate.linear_b,
                        sl_vec2_scale(impulse, candidate.inverse_mass_b));
        candidate.angular_b +=
            candidate.inverse_inertia_b * sl_vec2_cross(anchor_b, impulse);
    }
    if (!sl_vec2_is_finite(candidate.linear_a) ||
        !sl_vec2_is_finite(candidate.linear_b) ||
        !sl_is_finite(candidate.angular_a) ||
        !sl_is_finite(candidate.angular_b)) {
        return false;
    }
    *pair = candidate;
    return true;
}

static sl_vec2 relative_velocity(const sl_joint_velocity_pair *pair,
                                 sl_vec2 anchor_a, sl_vec2 anchor_b)
{
    const sl_vec2 velocity_a = sl_vec2_add(
        pair->linear_a, sl_scalar_cross_vec2(pair->angular_a, anchor_a));
    const sl_vec2 velocity_b = sl_vec2_add(
        pair->linear_b, sl_scalar_cross_vec2(pair->angular_b, anchor_b));
    return sl_vec2_sub(velocity_b, velocity_a);
}

static sl_vec2 separation_current(const sl_world_state *world,
                                  const sl_joint_constraint *constraint);

static sl_joint_softness softness_make(float hertz, float damping_ratio,
                                       float h)
{
    /* Implicit damped correction in projected-impulse form. The 1/8-rate
     * hertz cap bounds h*omega to pi/4, which is deliberately more
     * conservative for bilateral chains than the contact solver's cap. */
    const float omega = 2.0f * SL_PI * hertz;
    const float h_omega = h * omega;
    const float a_1 = 2.0f * damping_ratio + h_omega;
    const float a_2 = h_omega * a_1;
    const float a_3 = 1.0f / (1.0f + a_2);
    const sl_joint_softness softness = {
        .bias_rate = omega / a_1,
        .mass_scale = a_2 * a_3,
        .impulse_scale = a_3,
    };
    const bool valid =
        sl_is_finite(softness.bias_rate) && softness.bias_rate >= 0.0f &&
        sl_is_finite(softness.mass_scale) && softness.mass_scale >= 0.0f &&
        softness.mass_scale <= 1.0f && sl_is_finite(softness.impulse_scale) &&
        softness.impulse_scale >= 0.0f && softness.impulse_scale <= 1.0f;
    SL_ASSERT(valid);
    return valid ? softness : (sl_joint_softness){ 0.0f, 1.0f, 0.0f };
}

static float axial_mass_make(const sl_world_state *world, uint32_t dense_a,
                             uint32_t dense_b, sl_vec2 anchor_a,
                             sl_vec2 anchor_b, sl_vec2 axis)
{
    const float cross_a = sl_vec2_cross(anchor_a, axis);
    const float cross_b = sl_vec2_cross(anchor_b, axis);
    const float inverse_mass =
        world->inv_masses[dense_a] + world->inv_masses[dense_b] +
        world->inv_inertias[dense_a] * cross_a * cross_a +
        world->inv_inertias[dense_b] * cross_b * cross_b;
    if (!sl_is_finite(inverse_mass) || inverse_mass <= 0.0f) {
        return 0.0f;
    }
    const float mass = 1.0f / inverse_mass;
    return (sl_is_finite(mass) && mass > 0.0f) ? mass : 0.0f;
}

static sl_mat2 effective_matrix_make(const sl_world_state *world,
                                     uint32_t dense_a, uint32_t dense_b,
                                     sl_vec2 anchor_a, sl_vec2 anchor_b)
{
    const float inverse_mass =
        world->inv_masses[dense_a] + world->inv_masses[dense_b];
    const float inertia_a = world->inv_inertias[dense_a];
    const float inertia_b = world->inv_inertias[dense_b];
    const sl_mat2 matrix = {
        inverse_mass + inertia_a * anchor_a.y * anchor_a.y +
            inertia_b * anchor_b.y * anchor_b.y,
        -inertia_a * anchor_a.x * anchor_a.y -
            inertia_b * anchor_b.x * anchor_b.y,
        -inertia_a * anchor_a.x * anchor_a.y -
            inertia_b * anchor_b.x * anchor_b.y,
        inverse_mass + inertia_a * anchor_a.x * anchor_a.x +
            inertia_b * anchor_b.x * anchor_b.x,
    };
    if (!sl_is_finite(matrix.m00) || !sl_is_finite(matrix.m01) ||
        !sl_is_finite(matrix.m10) || !sl_is_finite(matrix.m11)) {
        return (sl_mat2){ 0.0f, 0.0f, 0.0f, 0.0f };
    }
    return matrix;
}

void sl_joint_prepare(sl_world_state *world, float h, float inverse_h)
{
    SL_ASSERT(world != NULL);
    if (world->joint_capacity == 0u) {
        world->joint_constraint_count = 0u;
        return;
    }
    sl_joint_constraint *constraints = world->joint_constraints;
    const float hertz = sl_min(world->joint_hertz, 0.125f * inverse_h);
    const sl_joint_softness softness =
        softness_make(hertz, world->joint_damping_ratio, h);
    for (uint32_t row = 0u; row < world->joint_count; ++row) {
        sl_joint_constraint *constraint = &constraints[row];
        memset(constraint, 0, sizeof(*constraint));
        constraint->joint_row = row;
        constraint->dense_a =
            world->slots[world->joint_bodies_a[row].index].dense;
        constraint->dense_b =
            world->slots[world->joint_bodies_b[row].index].dense;
        constraint->kind = world->joint_kinds[row];
        constraint->anchor_a =
            sl_rotation_apply(world->rotations[constraint->dense_a],
                              world->joint_local_anchors_a[row]);
        constraint->anchor_b =
            sl_rotation_apply(world->rotations[constraint->dense_b],
                              world->joint_local_anchors_b[row]);
        constraint->base_delta =
            sl_vec2_sub(world->positions[constraint->dense_b],
                        world->positions[constraint->dense_a]);
        constraint->softness = softness;
        constraint->step_impulse = sl_vec2_make(0.0f, 0.0f);
        if (constraint->kind == (uint32_t)SL_JOINT_DISTANCE) {
            const sl_vec2 delta = sl_vec2_add(
                constraint->base_delta,
                sl_vec2_sub(constraint->anchor_b, constraint->anchor_a));
            constraint->axis = sl_vec2_normalize(delta);
            if (sl_vec2_length_sq(constraint->axis) == 0.0f) {
                constraint->axis = sl_vec2_make(1.0f, 0.0f);
            }
            constraint->distance_length = world->joint_distance_lengths[row];
            constraint->axial_mass = axial_mass_make(
                world, constraint->dense_a, constraint->dense_b,
                constraint->anchor_a, constraint->anchor_b, constraint->axis);
            constraint->axial_impulse = world->joint_distance_impulses[row];
            if (constraint->axial_mass == 0.0f) {
                constraint->kind = SL_JOINT_KIND_INVALID;
                constraint->axial_impulse = 0.0f;
            }
        } else {
            constraint->impulse = world->joint_revolute_impulses[row];
            const sl_mat2 matrix = effective_matrix_make(
                world, constraint->dense_a, constraint->dense_b,
                constraint->anchor_a, constraint->anchor_b);
            if (!effective_matrix_solvable(matrix)) {
                constraint->kind = SL_JOINT_KIND_INVALID;
                constraint->impulse = sl_vec2_make(0.0f, 0.0f);
            }
        }
    }
    world->joint_constraint_count = world->joint_count;
}

void sl_joint_warm_start(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    sl_joint_constraint *constraints = world->joint_constraints;
    for (uint32_t row = 0u; row < world->joint_constraint_count; ++row) {
        sl_joint_constraint *constraint = &constraints[row];
        if (constraint->kind == SL_JOINT_KIND_INVALID) {
            continue;
        }
        sl_joint_velocity_pair pair = velocity_pair_load(world, constraint);
        const sl_vec2 anchor_a = sl_rotation_apply(
            world->delta_rotations[constraint->dense_a], constraint->anchor_a);
        const sl_vec2 anchor_b = sl_rotation_apply(
            world->delta_rotations[constraint->dense_b], constraint->anchor_b);
        sl_vec2 impulse = constraint->impulse;
        if (constraint->kind == (uint32_t)SL_JOINT_DISTANCE) {
            const sl_vec2 axis =
                sl_vec2_normalize(separation_current(world, constraint));
            const sl_vec2 fallback =
                (sl_vec2_length_sq(axis) > 0.0f) ? axis : constraint->axis;
            impulse = sl_vec2_scale(fallback, constraint->axial_impulse);
        }
        const sl_vec2 step_impulse =
            sl_vec2_add(constraint->step_impulse, impulse);
        if (sl_vec2_is_finite(step_impulse) &&
            impulse_apply(&pair, anchor_a, anchor_b, impulse)) {
            constraint->step_impulse = step_impulse;
            velocity_pair_store(world, constraint, &pair);
        }
    }
}

static sl_vec2 separation_current(const sl_world_state *world,
                                  const sl_joint_constraint *constraint)
{
    const sl_vec2 anchor_a = sl_rotation_apply(
        world->delta_rotations[constraint->dense_a], constraint->anchor_a);
    const sl_vec2 anchor_b = sl_rotation_apply(
        world->delta_rotations[constraint->dense_b], constraint->anchor_b);
    return sl_vec2_add(
        constraint->base_delta,
        sl_vec2_sub(
            sl_vec2_add(world->delta_positions[constraint->dense_b], anchor_b),
            sl_vec2_add(world->delta_positions[constraint->dense_a],
                        anchor_a)));
}

static void distance_solve(const sl_world_state *world,
                           sl_joint_constraint *constraint,
                           sl_joint_velocity_pair *pair, bool use_bias)
{
    const sl_vec2 anchor_a = sl_rotation_apply(
        world->delta_rotations[constraint->dense_a], constraint->anchor_a);
    const sl_vec2 anchor_b = sl_rotation_apply(
        world->delta_rotations[constraint->dense_b], constraint->anchor_b);
    const sl_vec2 separation = separation_current(world, constraint);
    sl_vec2 axis = sl_vec2_normalize(separation);
    if (sl_vec2_length_sq(axis) == 0.0f) {
        axis = constraint->axis;
    }
    /* Match the mass to the rotated Jacobian. Each solve adds two cross
     * products and a reciprocal; storage and O(joints * substeps) work stay
     * unchanged. */
    constraint->axial_mass =
        axial_mass_make(world, constraint->dense_a, constraint->dense_b,
                        anchor_a, anchor_b, axis);
    if (constraint->axial_mass == 0.0f || sl_vec2_length_sq(axis) == 0.0f) {
        return;
    }
    const sl_vec2 velocity = relative_velocity(pair, anchor_a, anchor_b);
    const float axial_velocity = sl_vec2_dot(velocity, axis);
    const float old_impulse = constraint->axial_impulse;
    float bias = 0.0f;
    float mass_scale = 1.0f;
    float impulse_scale = 0.0f;
    if (use_bias) {
        const float error =
            sl_vec2_length(separation) - constraint->distance_length;
        bias = constraint->softness.bias_rate * error;
        mass_scale = constraint->softness.mass_scale;
        impulse_scale = constraint->softness.impulse_scale;
    }
    const float applied =
        -constraint->axial_mass * mass_scale * (axial_velocity + bias) -
        impulse_scale * old_impulse;
    const float next_impulse = old_impulse + applied;
    const sl_vec2 vector = sl_vec2_scale(axis, applied);
    const sl_vec2 step_impulse = sl_vec2_add(constraint->step_impulse, vector);
    if (!sl_is_finite(applied) || !sl_is_finite(next_impulse) ||
        !sl_vec2_is_finite(step_impulse) ||
        !impulse_apply(pair, anchor_a, anchor_b, vector)) {
        return;
    }
    constraint->axial_impulse = next_impulse;
    constraint->step_impulse = step_impulse;
}

static void revolute_solve(const sl_world_state *world,
                           sl_joint_constraint *constraint,
                           sl_joint_velocity_pair *pair, bool use_bias)
{
    const sl_vec2 anchor_a = sl_rotation_apply(
        world->delta_rotations[constraint->dense_a], constraint->anchor_a);
    const sl_vec2 anchor_b = sl_rotation_apply(
        world->delta_rotations[constraint->dense_b], constraint->anchor_b);
    const sl_vec2 velocity = relative_velocity(pair, anchor_a, anchor_b);
    sl_vec2 bias = sl_vec2_make(0.0f, 0.0f);
    float mass_scale = 1.0f;
    float impulse_scale = 0.0f;
    if (use_bias) {
        bias = sl_vec2_scale(separation_current(world, constraint),
                             constraint->softness.bias_rate);
        mass_scale = constraint->softness.mass_scale;
        impulse_scale = constraint->softness.impulse_scale;
    }
    const sl_vec2 rhs =
        sl_vec2_neg(sl_vec2_scale(sl_vec2_add(velocity, bias), mass_scale));
    sl_vec2 applied = sl_vec2_make(0.0f, 0.0f);
    const sl_mat2 effective_matrix = effective_matrix_make(
        world, constraint->dense_a, constraint->dense_b, anchor_a, anchor_b);
    if (!sl_vec2_is_finite(rhs) ||
        !sl_mat2_solve(effective_matrix, rhs, &applied)) {
        return;
    }
    /* Cached lambda already has impulse units; only velocity and bias need
     * the effective-mass conversion. */
    applied =
        sl_vec2_sub(applied, sl_vec2_scale(constraint->impulse, impulse_scale));
    const sl_vec2 next_impulse = sl_vec2_add(constraint->impulse, applied);
    const sl_vec2 step_impulse = sl_vec2_add(constraint->step_impulse, applied);
    if (!sl_vec2_is_finite(next_impulse) || !sl_vec2_is_finite(step_impulse) ||
        !impulse_apply(pair, anchor_a, anchor_b, applied)) {
        return;
    }
    constraint->impulse = next_impulse;
    constraint->step_impulse = step_impulse;
}

void sl_joint_solve(sl_world_state *world, bool use_bias)
{
    SL_ASSERT(world != NULL);
    sl_joint_constraint *constraints = world->joint_constraints;
    for (uint32_t row = 0u; row < world->joint_constraint_count; ++row) {
        sl_joint_constraint *constraint = &constraints[row];
        if (constraint->kind == SL_JOINT_KIND_INVALID) {
            continue;
        }
        sl_joint_velocity_pair pair = velocity_pair_load(world, constraint);
        if (constraint->kind == (uint32_t)SL_JOINT_DISTANCE) {
            distance_solve(world, constraint, &pair, use_bias);
        } else {
            revolute_solve(world, constraint, &pair, use_bias);
        }
        velocity_pair_store(world, constraint, &pair);
    }
}

void sl_joint_store(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    sl_joint_constraint *constraints = world->joint_constraints;
    for (uint32_t row = 0u; row < world->joint_constraint_count; ++row) {
        const sl_joint_constraint *constraint = &constraints[row];
        const bool valid = constraint->kind != SL_JOINT_KIND_INVALID;
        if (world->joint_kinds[constraint->joint_row] ==
            (uint8_t)SL_JOINT_DISTANCE) {
            world->joint_distance_impulses[constraint->joint_row] =
                valid ? constraint->axial_impulse : 0.0f;
        } else {
            world->joint_revolute_impulses[constraint->joint_row] =
                valid ? constraint->impulse : sl_vec2_make(0.0f, 0.0f);
        }
        /* The cache is one substep's lambda. The public value instead sums
         * every successful application over all substeps, so a steady load
         * reports force * dt rather than force * h. */
        world->joint_linear_impulses[constraint->joint_row] =
            valid ? constraint->step_impulse : sl_vec2_make(0.0f, 0.0f);
    }
}

bool sl_world_joint_is_valid(const sl_world *world, sl_joint_handle handle)
{
    SL_ASSERT(world != NULL);
    return world->state != NULL && sl_joint_is_valid(world->state, handle);
}
void sl_world_joint_destroy(sl_world *world, sl_joint_handle handle)
{
    SL_ASSERT(world != NULL);
    if (world->state != NULL)
        sl_joint_destroy(world->state, handle);
}
