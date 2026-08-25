#include "silk/world.h"

#include <stdlib.h>
#include <string.h>

/* All array slices are cut at multiples of 8 bytes — the largest member
 * alignment in the world (pointers, sl_vec2, sl_body_slot) — so carving
 * needs no per-type padding math and malloc's max_align_t guarantee
 * covers every slice. */
#define SL_CARVE_ALIGN ((size_t)8)

static size_t align_up(size_t bytes)
{
    return (bytes + (SL_CARVE_ALIGN - (size_t)1)) &
           ~(SL_CARVE_ALIGN - (size_t)1);
}

/* Generation counters are 1-based; a bump that lands on 0 skips to 1 so
 * the null-handle encoding (generation == 0) stays unreachable. */
static uint32_t generation_bump(uint32_t generation)
{
    generation++;
    return (generation == 0u) ? 1u : generation;
}

/* Shared by create and set_mass: exactly the masses whose full state can
 * be stored soundly. 1/mass must stay representable — FLT_TRUE_MIN-class
 * masses pass the finite/positive checks yet overflow their inverse. */
static bool body_mass_valid(float mass)
{
    if (!sl_is_finite(mass) || mass <= 0.0f) {
        return false;
    }
    return sl_is_finite(1.0f / mass);
}

static bool body_desc_valid(const sl_body_desc *desc)
{
    return sl_vec2_is_finite(desc->position) &&
           sl_vec2_is_finite(desc->velocity) && body_mass_valid(desc->mass);
}

static sl_body_handle handle_for(const sl_world *world, uint32_t dense)
{
    sl_body_handle h = { world->slot_of[dense],
                         world->slots[world->slot_of[dense]].generation };
    return h;
}

bool sl_world_init(sl_world *world, const sl_world_config *config)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(config != NULL);

    memset(world, 0, sizeof(*world));

    if (config->body_capacity < 1u ||
        config->body_capacity > SL_BODY_COUNT_MAX) {
        return false;
    }
    if (!sl_vec2_is_finite(config->gravity) ||
        !sl_is_finite(config->linear_drag) || config->linear_drag < 0.0f) {
        return false;
    }

    const size_t capacity = (size_t)config->body_capacity;
    const size_t slots_bytes = align_up(capacity * sizeof(sl_body_slot));
    const size_t index_bytes = align_up(capacity * sizeof(uint32_t));
    const size_t vec2_bytes = align_up(capacity * sizeof(sl_vec2));
    const size_t float_bytes = align_up(capacity * sizeof(float));
    const size_t total_bytes =
        slots_bytes + 2u * index_bytes + 3u * vec2_bytes + 2u * float_bytes;

    unsigned char *base = malloc(total_bytes);
    if (base == NULL) {
        return false;
    }

    size_t offset = 0;
    world->slots = (sl_body_slot *)(base + offset);
    offset += slots_bytes;
    world->slot_of = (uint32_t *)(base + offset);
    offset += index_bytes;
    world->free_indices = (uint32_t *)(base + offset);
    offset += index_bytes;
    world->positions = (sl_vec2 *)(base + offset);
    offset += vec2_bytes;
    world->velocities = (sl_vec2 *)(base + offset);
    offset += vec2_bytes;
    world->masses = (float *)(base + offset);
    offset += float_bytes;
    world->inv_masses = (float *)(base + offset);
    offset += float_bytes;
    world->forces = (sl_vec2 *)(base + offset);
    world->memory = base;

    world->body_count = 0u;
    world->body_capacity = config->body_capacity;
    world->free_count = config->body_capacity;
    world->gravity = config->gravity;
    world->linear_drag = config->linear_drag;

    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        /* Descending push => LIFO pops hand out slots 0,1,2,... */
        world->free_indices[i] = world->body_capacity - 1u - i;
        world->slots[i].dense = SL_BODY_DENSE_NONE;
        world->slots[i].generation = 1u;
        world->slot_of[i] = SL_BODY_DENSE_NONE;
    }

    return true;
}

void sl_world_destroy(sl_world *world)
{
    SL_ASSERT(world != NULL);

    free(world->memory);
    memset(world, 0, sizeof(*world));
}

void sl_world_reset(sl_world *world)
{
    SL_ASSERT(world != NULL);

    world->body_count = 0u;
    world->free_count = world->body_capacity;

    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        /* Bump past every handle ever issued against this slot. */
        world->slots[i].dense = SL_BODY_DENSE_NONE;
        world->slots[i].generation =
            generation_bump(world->slots[i].generation);
        world->slot_of[i] = SL_BODY_DENSE_NONE;
        world->free_indices[i] = world->body_capacity - 1u - i;
    }
}

sl_body_handle sl_world_body_create(sl_world *world, const sl_body_desc *desc)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(desc != NULL);

    if (!body_desc_valid(desc) || world->free_count == 0u) {
        return sl_body_handle_null();
    }

    const uint32_t slot_id = world->free_indices[world->free_count - 1u];
    world->free_count--;

    const uint32_t dense = world->body_count;
    world->body_count++;

    world->slots[slot_id].dense = dense;
    world->slot_of[dense] = slot_id;

    world->positions[dense] = desc->position;
    world->velocities[dense] = desc->velocity;
    world->masses[dense] = desc->mass;
    world->inv_masses[dense] = 1.0f / desc->mass;
    world->forces[dense] = sl_vec2_make(0.0f, 0.0f);

    return handle_for(world, dense);
}

void sl_world_body_destroy(sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);

    /* Tolerant contract: null, malformed, and stale handles all land here
     * as no-ops instead of corrupting the pool. */
    if (handle.generation == 0u || handle.index >= world->body_capacity) {
        return;
    }

    sl_body_slot *slot = &world->slots[handle.index];
    if (slot->generation != handle.generation ||
        slot->dense == SL_BODY_DENSE_NONE) {
        return;
    }

    /* Swap-remove: the last packed body moves into the hole and only its
     * back-pointer needs repair. */
    const uint32_t dense = slot->dense;
    const uint32_t last = world->body_count - 1u;
    if (dense != last) {
        world->positions[dense] = world->positions[last];
        world->velocities[dense] = world->velocities[last];
        world->masses[dense] = world->masses[last];
        world->inv_masses[dense] = world->inv_masses[last];
        world->forces[dense] = world->forces[last];

        const uint32_t moved_slot = world->slot_of[last];
        world->slot_of[dense] = moved_slot;
        world->slots[moved_slot].dense = dense;
    }
    world->body_count--;

    slot->dense = SL_BODY_DENSE_NONE;
    slot->generation = generation_bump(slot->generation);
    world->free_indices[world->free_count] = handle.index;
    world->free_count++;
}

bool sl_world_body_is_valid(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);

    if (handle.generation == 0u || handle.index >= world->body_capacity) {
        return false;
    }

    const sl_body_slot *slot = &world->slots[handle.index];
    return slot->generation == handle.generation &&
           slot->dense != SL_BODY_DENSE_NONE;
}

uint32_t sl_world_body_count(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->body_count;
}

uint32_t sl_world_body_capacity(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->body_capacity;
}

sl_vec2 sl_world_get_gravity(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->gravity;
}

float sl_world_get_linear_drag(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->linear_drag;
}

sl_body_handle sl_world_body_first(const sl_world *world)
{
    SL_ASSERT(world != NULL);

    if (world->body_count == 0u) {
        return sl_body_handle_null();
    }
    return handle_for(world, 0u);
}

sl_body_handle sl_world_body_next(const sl_world *world, sl_body_handle current)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, current));

    const uint32_t next_dense = world->slots[current.index].dense + 1u;
    if (next_dense >= world->body_count) {
        return sl_body_handle_null();
    }
    return handle_for(world, next_dense);
}

sl_vec2 sl_world_body_get_position(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->positions[world->slots[handle.index].dense];
}

sl_vec2 sl_world_body_get_velocity(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->velocities[world->slots[handle.index].dense];
}

float sl_world_body_get_mass(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->masses[world->slots[handle.index].dense];
}

float sl_world_body_get_inv_mass(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->inv_masses[world->slots[handle.index].dense];
}

bool sl_world_body_set_position(sl_world *world, sl_body_handle handle,
                                sl_vec2 position)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!sl_vec2_is_finite(position)) {
        return false;
    }
    world->positions[world->slots[handle.index].dense] = position;
    return true;
}

bool sl_world_body_set_velocity(sl_world *world, sl_body_handle handle,
                                sl_vec2 velocity)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!sl_vec2_is_finite(velocity)) {
        return false;
    }
    world->velocities[world->slots[handle.index].dense] = velocity;
    return true;
}

bool sl_world_body_set_mass(sl_world *world, sl_body_handle handle, float mass)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!body_mass_valid(mass)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    world->masses[dense] = mass;
    world->inv_masses[dense] = 1.0f / mass;
    return true;
}

bool sl_world_body_apply_force(sl_world *world, sl_body_handle handle,
                               sl_vec2 force)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!sl_vec2_is_finite(force)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    world->forces[dense] = sl_vec2_add(world->forces[dense], force);
    return true;
}

sl_vec2 sl_world_body_get_force(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->forces[world->slots[handle.index].dense];
}
