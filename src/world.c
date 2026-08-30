#include "silk/world.h"

#include <stdlib.h>
#include <string.h>

/* All array slices are cut at multiples of 8 bytes -- the largest member
 * alignment in the world (pointers, sl_vec2, sl_body_slot) -- so carving
 * needs no per-type padding math and malloc's max_align_t guarantee
 * covers every slice. */
#define SL_CARVE_ALIGN ((size_t)8)

static size_t align_up(size_t bytes)
{
    return (bytes + (SL_CARVE_ALIGN - (size_t)1)) &
           ~(SL_CARVE_ALIGN - (size_t)1);
}

static size_t slice_bytes(size_t capacity, size_t element_size)
{
    return align_up(capacity * element_size);
}

static unsigned char *carve(unsigned char *base, size_t *offset,
                            size_t capacity, size_t element_size)
{
    unsigned char *slice = base + *offset;
    *offset += slice_bytes(capacity, element_size);
    return slice;
}

/* Single source for init's allocation and the public budget query:
 * one slot array, two index arrays, three vec2 arrays, seven float
 * arrays, one type byte per body, and one shape record per body. */
static size_t world_memory_bytes_for(size_t capacity)
{
    return slice_bytes(capacity, sizeof(sl_body_slot)) +
           2u * slice_bytes(capacity, sizeof(uint32_t)) +
           3u * slice_bytes(capacity, sizeof(sl_vec2)) +
           7u * slice_bytes(capacity, sizeof(float)) +
           slice_bytes(capacity, sizeof(uint8_t)) +
           slice_bytes(capacity, sizeof(sl_shape));
}

size_t sl_world_memory_bytes(uint32_t body_capacity)
{
    if (body_capacity < 1u || body_capacity > SL_BODY_COUNT_MAX) {
        return 0u;
    }
    return world_memory_bytes_for((size_t)body_capacity);
}

/* Generation counters are 1-based; a bump that lands on 0 skips to 1 so
 * the null-handle encoding (generation == 0) stays unreachable. */
static uint32_t generation_bump(uint32_t generation)
{
    generation++;
    return (generation == 0u) ? 1u : generation;
}

/* Dynamic masses only: exactly the masses whose full state can be
 * stored soundly. 1/mass must stay representable -- FLT_TRUE_MIN-class
 * masses pass the finite/positive checks yet overflow their inverse. */
static bool body_mass_valid(float mass)
{
    if (!sl_is_finite(mass) || mass <= 0.0f) {
        return false;
    }
    return sl_is_finite(1.0f / mass);
}

/* Static and kinematic bodies encode infinite mass as exact zero; any
 * other value is external data worth rejecting, not reinterpreting. */
static bool body_mass_valid_for(sl_body_type type, float mass)
{
    if (type == SL_BODY_DYNAMIC) {
        return body_mass_valid(mass);
    }
    return mass == 0.0f;
}

static bool position_valid(sl_vec2 position)
{
    return sl_vec2_is_finite(position) &&
           sl_abs(position.x) <= SL_POSITION_ABS_MAX &&
           sl_abs(position.y) <= SL_POSITION_ABS_MAX;
}

static bool body_type_valid(sl_body_type type)
{
    return type == SL_BODY_DYNAMIC || type == SL_BODY_KINEMATIC ||
           type == SL_BODY_STATIC;
}

/* Inertia about the body origin: finite, non-negative, with a
 * representable inverse unless it is the zero (infinite) encoding. */
static bool body_inertia_valid(float inertia)
{
    if (!sl_is_finite(inertia) || inertia < 0.0f) {
        return false;
    }
    return inertia == 0.0f || sl_is_finite(1.0f / inertia);
}

/* Zero encodes infinite resistance -- for mass on non-dynamic rows, for
 * inertia on shapeless ones -- so its inverse is zero, not a division. */
static float inverse_or_zero(float value)
{
    return (value > 0.0f) ? 1.0f / value : 0.0f;
}

/* The world's standing invariant, one axis each: whatever is banked in
 * an accumulator must still produce a finite acceleration through the
 * body's inverse. apply_force and apply_torque check it when the
 * accumulator moves; set_mass and set_shape check it when the inverse
 * moves. Skip either side and the guard is only half enforced. */
static bool linear_accel_finite(sl_vec2 force, float inv_mass)
{
    return sl_vec2_is_finite(sl_vec2_scale(force, inv_mass));
}

static bool angular_accel_finite(float torque, float inv_inertia)
{
    return sl_is_finite(torque * inv_inertia);
}

/* Inertia derived from mass and the attached shape; shapeless bodies
 * spin freely (zero inertia). Callers gate on body_inertia_valid. */
static float body_inertia_for(float mass, const sl_shape *shape)
{
    if (shape == NULL || shape->kind == SL_SHAPE_NONE) {
        return 0.0f;
    }
    /* Constructed shapes are centroid-centered, so the body origin is
     * the center of mass and no parallel-axis term is needed. */
    return mass * sl_shape_mass_data(shape).inertia_per_unit_mass;
}

static bool body_desc_valid(const sl_body_desc *desc)
{
    if (!position_valid(desc->position) || !sl_vec2_is_finite(desc->velocity) ||
        !body_type_valid(desc->type) ||
        !body_mass_valid_for(desc->type, desc->mass)) {
        return false;
    }
    if (desc->type == SL_BODY_STATIC &&
        !(desc->velocity.x == 0.0f && desc->velocity.y == 0.0f &&
          desc->angular_velocity == 0.0f)) {
        return false;
    }
    if (!sl_is_finite(desc->angle) || !sl_is_finite(desc->angular_velocity)) {
        return false;
    }
    if (desc->shape != NULL && !sl_shape_is_valid(desc->shape)) {
        return false;
    }
    return true;
}

/* inertias and inv_inertias only ever move together. */
static void body_write_inertia(sl_world *world, uint32_t dense, float inertia)
{
    world->inertias[dense] = inertia;
    world->inv_inertias[dense] = inverse_or_zero(inertia);
}

/* Both accumulator writers ask the same question: does adding this
 * much keep the sum, and the acceleration the stepper derives from it,
 * finite? On success *out carries the value to bank. */
static bool force_sum_ok(const sl_world *world, uint32_t dense, sl_vec2 force,
                         sl_vec2 *out)
{
    const sl_vec2 summed = sl_vec2_add(world->forces[dense], force);
    /* The stepper consumes force * inv_mass, so a finite accumulation
     * can still overflow through a tiny mass; reject it here rather
     * than bank infinity. */
    if (!sl_vec2_is_finite(summed) ||
        !linear_accel_finite(summed, world->inv_masses[dense])) {
        return false;
    }
    *out = summed;
    return true;
}

static bool torque_sum_ok(const sl_world *world, uint32_t dense, float torque,
                          float *out)
{
    const float summed = world->torques[dense] + torque;
    /* Mirror of the linear guard. Zero inertia means infinite
     * resistance, which integrates to nothing and accepts. */
    if (!sl_is_finite(summed) ||
        !angular_accel_finite(summed, world->inv_inertias[dense])) {
        return false;
    }
    *out = summed;
    return true;
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
    /* A live world carries an arena pointer that init would drop on the
     * floor; zero-init makes the read defined and the misuse loud. */
    SL_ASSERT(world->memory == NULL);

    memset(world, 0, sizeof(*world));

    if (config->body_capacity < 1u ||
        config->body_capacity > SL_BODY_COUNT_MAX) {
        return false;
    }
    if (!sl_vec2_is_finite(config->gravity) ||
        !sl_is_finite(config->linear_drag) || config->linear_drag < 0.0f ||
        !sl_is_finite(config->angular_drag) || config->angular_drag < 0.0f) {
        return false;
    }

    const size_t capacity = (size_t)config->body_capacity;
    const size_t total_bytes = world_memory_bytes_for(capacity);

    unsigned char *base = malloc(total_bytes);
    if (base == NULL) {
        return false;
    }

    /* Carved in declaration order; the closing assert fires if an array
     * joins one list but not the other. */
    size_t offset = 0;
    world->slots =
        (sl_body_slot *)carve(base, &offset, capacity, sizeof(sl_body_slot));
    world->slot_of =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->free_indices =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->positions =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->velocities =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->masses = (float *)carve(base, &offset, capacity, sizeof(float));
    world->inv_masses = (float *)carve(base, &offset, capacity, sizeof(float));
    world->forces = (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->angles = (float *)carve(base, &offset, capacity, sizeof(float));
    world->angular_velocities =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->torques = (float *)carve(base, &offset, capacity, sizeof(float));
    world->inertias = (float *)carve(base, &offset, capacity, sizeof(float));
    world->inv_inertias =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->types = (uint8_t *)carve(base, &offset, capacity, sizeof(uint8_t));
    world->shapes =
        (sl_shape *)carve(base, &offset, capacity, sizeof(sl_shape));
    SL_ASSERT(offset == total_bytes);
    world->memory = base;

    world->body_count = 0u;
    world->body_capacity = config->body_capacity;
    world->free_count = config->body_capacity;
    world->gravity = config->gravity;
    world->linear_drag = config->linear_drag;
    world->angular_drag = config->angular_drag;

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
    world->step_rejection_count = 0u;

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

    /* Descriptor first: body_inertia_for measures the shape, walking
     * polygon.count vertices, so the record has to clear
     * sl_shape_is_valid before it is touched -- a tampered count read
     * here would run off the end of the vertex array. */
    if (!body_desc_valid(desc) || world->free_count == 0u) {
        return sl_body_handle_null();
    }
    /* Derived inertia is validated before any slot is consumed, so a
     * rejection here leaves the pool untouched. */
    const float inertia = (desc->type == SL_BODY_DYNAMIC)
                              ? body_inertia_for(desc->mass, desc->shape)
                              : 0.0f;
    if (!body_inertia_valid(inertia)) {
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
    /* body_mass_valid_for pins a non-dynamic mass at exactly zero, so
     * the encoding falls out of inverse_or_zero without a type test. */
    world->inv_masses[dense] = inverse_or_zero(desc->mass);
    world->forces[dense] = sl_vec2_make(0.0f, 0.0f);

    world->angles[dense] = sl_angle_wrap(desc->angle);
    world->angular_velocities[dense] = desc->angular_velocity;
    world->torques[dense] = 0.0f;
    body_write_inertia(world, dense, inertia);

    world->types[dense] = (uint8_t)desc->type;
    world->shapes[dense] =
        (desc->shape != NULL) ? *desc->shape : sl_shape_none();

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
     * back-pointer needs repair. Every packed array must appear in both
     * this copy chain and create's initialization. */
    const uint32_t dense = slot->dense;
    const uint32_t last = world->body_count - 1u;
    if (dense != last) {
        world->positions[dense] = world->positions[last];
        world->velocities[dense] = world->velocities[last];
        world->masses[dense] = world->masses[last];
        world->inv_masses[dense] = world->inv_masses[last];
        world->forces[dense] = world->forces[last];
        world->angles[dense] = world->angles[last];
        world->angular_velocities[dense] = world->angular_velocities[last];
        world->torques[dense] = world->torques[last];
        world->inertias[dense] = world->inertias[last];
        world->inv_inertias[dense] = world->inv_inertias[last];
        world->types[dense] = world->types[last];
        world->shapes[dense] = world->shapes[last];

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

float sl_world_get_angular_drag(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->angular_drag;
}

uint32_t sl_world_get_step_rejection_count(const sl_world *world)
{
    SL_ASSERT(world != NULL);
    return world->step_rejection_count;
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

sl_body_handle sl_world_body_at(const sl_world *world, uint32_t row)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(row < world->body_count);
    return handle_for(world, row);
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

sl_body_type sl_world_body_get_type(const sl_world *world,
                                    sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return (sl_body_type)world->types[world->slots[handle.index].dense];
}

float sl_world_body_get_angle(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->angles[world->slots[handle.index].dense];
}

float sl_world_body_get_angular_velocity(const sl_world *world,
                                         sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->angular_velocities[world->slots[handle.index].dense];
}

float sl_world_body_get_inertia(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->inertias[world->slots[handle.index].dense];
}

float sl_world_body_get_inv_inertia(const sl_world *world,
                                    sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->inv_inertias[world->slots[handle.index].dense];
}

float sl_world_body_get_torque(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->torques[world->slots[handle.index].dense];
}

const sl_shape *sl_world_body_get_shape(const sl_world *world,
                                        sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return &world->shapes[world->slots[handle.index].dense];
}

sl_transform sl_world_body_get_transform(const sl_world *world,
                                         sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    return sl_transform_make(world->positions[dense],
                             sl_rotation_make(world->angles[dense]));
}

bool sl_world_body_set_position(sl_world *world, sl_body_handle handle,
                                sl_vec2 position)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!position_valid(position)) {
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
    const uint32_t dense = world->slots[handle.index].dense;
    /* A static body never moves: only the exact zero velocity keeps its
     * state self-consistent. */
    if (world->types[dense] == (uint8_t)SL_BODY_STATIC &&
        !(velocity.x == 0.0f && velocity.y == 0.0f)) {
        return false;
    }
    world->velocities[dense] = velocity;
    return true;
}

bool sl_world_body_set_angle(sl_world *world, sl_body_handle handle,
                             float angle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!sl_is_finite(angle)) {
        return false;
    }
    world->angles[world->slots[handle.index].dense] = sl_angle_wrap(angle);
    return true;
}

bool sl_world_body_set_angular_velocity(sl_world *world, sl_body_handle handle,
                                        float angular_velocity)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    if (!sl_is_finite(angular_velocity)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] == (uint8_t)SL_BODY_STATIC &&
        angular_velocity != 0.0f) {
        return false;
    }
    world->angular_velocities[dense] = angular_velocity;
    return true;
}

bool sl_world_body_set_mass(sl_world *world, sl_body_handle handle, float mass)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC ||
        !body_mass_valid(mass)) {
        return false;
    }
    /* Re-derive from the currently attached shape; reject before any
     * write so a failing derivation leaves state untouched. */
    const float inertia = body_inertia_for(mass, &world->shapes[dense]);
    if (!body_inertia_valid(inertia)) {
        return false;
    }
    /* Whatever is already banked cleared the guard against the OLD
     * inverses. Shrinking mass or inertia raises both, so a force or
     * torque that was safe to accept can now overflow the integrator:
     * re-check it here, or the next step stores infinity. */
    const float inv_mass = inverse_or_zero(mass);
    const float inv_inertia = inverse_or_zero(inertia);
    if (!linear_accel_finite(world->forces[dense], inv_mass) ||
        !angular_accel_finite(world->torques[dense], inv_inertia)) {
        return false;
    }
    world->masses[dense] = mass;
    world->inv_masses[dense] = inv_mass;
    body_write_inertia(world, dense, inertia);
    return true;
}

bool sl_world_body_set_shape(sl_world *world, sl_body_handle handle,
                             const sl_shape *shape)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));

    const sl_shape record = (shape != NULL) ? *shape : sl_shape_none();
    if (!sl_shape_is_valid(&record)) {
        return false;
    }

    const uint32_t dense = world->slots[handle.index].dense;
    const bool dynamic = world->types[dense] == (uint8_t)SL_BODY_DYNAMIC;
    const float inertia =
        dynamic ? body_inertia_for(world->masses[dense], &record) : 0.0f;
    if (!body_inertia_valid(inertia)) {
        return false;
    }
    /* Mass is untouched, but a smaller shape raises inv_inertia, so the
     * banked torque has to clear the guard again -- same reason as
     * set_mass. */
    if (!angular_accel_finite(world->torques[dense],
                              inverse_or_zero(inertia))) {
        return false;
    }
    world->shapes[dense] = record;
    body_write_inertia(world, dense, inertia);
    return true;
}

bool sl_world_body_apply_force(sl_world *world, sl_body_handle handle,
                               sl_vec2 force)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    sl_vec2 summed;
    if (!sl_vec2_is_finite(force) ||
        !force_sum_ok(world, dense, force, &summed)) {
        return false;
    }
    world->forces[dense] = summed;
    return true;
}

sl_vec2 sl_world_body_get_force(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    return world->forces[world->slots[handle.index].dense];
}

bool sl_world_body_apply_torque(sl_world *world, sl_body_handle handle,
                                float torque)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    float summed;
    if (!sl_is_finite(torque) ||
        !torque_sum_ok(world, dense, torque, &summed)) {
        return false;
    }
    world->torques[dense] = summed;
    return true;
}

bool sl_world_body_apply_force_at_point(sl_world *world, sl_body_handle handle,
                                        sl_vec2 force, sl_vec2 point)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_world_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    if (!sl_vec2_is_finite(force) || !position_valid(point)) {
        return false;
    }

    /* Every check runs before either accumulator moves: both change or
     * neither does. */
    const sl_vec2 arm = sl_vec2_sub(point, world->positions[dense]);
    sl_vec2 force_sum;
    float torque_sum;
    if (!force_sum_ok(world, dense, force, &force_sum) ||
        !torque_sum_ok(world, dense, sl_vec2_cross(arm, force), &torque_sum)) {
        return false;
    }

    world->forces[dense] = force_sum;
    world->torques[dense] = torque_sum;
    return true;
}
