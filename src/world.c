#include "silk/world.h"
#include "world_internal.h"

#include <stdlib.h>
#include <string.h>

#include "contact_world.h"
#include "joint.h"
#include "solver.h"

/* malloc aligns the base for every fundamental type. Every carved type's
 * alignment must divide 8, so advancing by 8-byte multiples preserves that
 * alignment without per-type padding math. The gates below make this arena
 * assumption fail at compile time on an incompatible ABI. */
#define SL_CARVE_ALIGN ((size_t)8)

#define SL_CARVE_ALIGNMENT_ASSERT(type)                                        \
    _Static_assert(_Alignof(type) <= SL_CARVE_ALIGN &&                         \
                       SL_CARVE_ALIGN % _Alignof(type) == 0u,                  \
                   #type " alignment violates the arena contract")

_Static_assert((SL_CARVE_ALIGN & (SL_CARVE_ALIGN - 1u)) == 0u,
               "arena alignment must be a power of two");
SL_CARVE_ALIGNMENT_ASSERT(sl_world_state);
SL_CARVE_ALIGNMENT_ASSERT(sl_body_slot);
SL_CARVE_ALIGNMENT_ASSERT(uint32_t);
SL_CARVE_ALIGNMENT_ASSERT(sl_vec2);
SL_CARVE_ALIGNMENT_ASSERT(sl_rotation);
SL_CARVE_ALIGNMENT_ASSERT(float);
SL_CARVE_ALIGNMENT_ASSERT(uint8_t);
SL_CARVE_ALIGNMENT_ASSERT(sl_shape);

#undef SL_CARVE_ALIGNMENT_ASSERT

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
 * one slot array, two index arrays, four vec2 arrays, two rotation arrays,
 * eight float arrays, one type byte, and one shape record per body. */
static size_t world_body_layout(size_t capacity, size_t *payload)
{
    size_t total = 0u;
    *payload = 0u;
#define BODY_SLICE(number, type)                                               \
    do {                                                                       \
        total += (number) * slice_bytes(capacity, sizeof(type));               \
        *payload += (number) * capacity * sizeof(type);                        \
    } while (false)
    BODY_SLICE(1u, sl_body_slot);
    BODY_SLICE(2u, uint32_t);
    BODY_SLICE(4u, sl_vec2);
    BODY_SLICE(2u, sl_rotation);
    BODY_SLICE(8u, float);
    BODY_SLICE(1u, uint8_t);
    BODY_SLICE(1u, sl_shape);
#undef BODY_SLICE
    return total;
}
/* Four slot arrays and seven-word descriptors: 44 B/body + 4 B/contact
 * + 4 B/joint. Each slice is independently aligned like the body arrays. */
static size_t island_layout(uint32_t bodies, uint32_t contacts, uint32_t joints,
                            size_t *payload)
{
    *payload = (size_t)bodies * (4u * sizeof(uint32_t) + sizeof(sl_island)) +
               ((size_t)contacts + joints) * sizeof(uint32_t);
    return 4u * slice_bytes(bodies, sizeof(uint32_t)) +
           slice_bytes(bodies, sizeof(sl_island)) +
           slice_bytes(contacts, sizeof(uint32_t)) +
           slice_bytes(joints, sizeof(uint32_t));
}

static size_t sleep_layout(size_t capacity)
{
    return slice_bytes(capacity, sizeof(uint32_t)) +
           2u * slice_bytes(capacity, sizeof(float)) +
           slice_bytes(capacity, sizeof(uint8_t));
}

static size_t world_memory_bytes_for(size_t capacity)
{
    size_t payload = 0u;
    return world_body_layout(capacity, &payload);
}

static bool world_config_resolve(const sl_world_config *config,
                                 sl_world_config *resolved)
{
    if (config == NULL || config->body_capacity < 1u ||
        config->body_capacity > SL_BODY_COUNT_MAX) {
        return false;
    }
    if (!sl_is_finite(config->sleep_speed_max) ||
        config->sleep_speed_max < 0.0f ||
        !sl_is_finite(config->sleep_angular_speed_max) ||
        config->sleep_angular_speed_max < 0.0f ||
        !sl_is_finite(config->sleep_time_min) ||
        config->sleep_time_min < 0.0f) {
        return false;
    }
    if (!sl_vec2_is_finite(config->gravity) ||
        !sl_is_finite(config->linear_drag) || config->linear_drag < 0.0f ||
        !sl_is_finite(config->angular_drag) || config->angular_drag < 0.0f ||
        config->substep_count > SL_SUBSTEP_COUNT_MAX ||
        !sl_is_finite(config->linear_speed_max) ||
        config->linear_speed_max < 0.0f ||
        !sl_is_finite(config->contact_hertz) || config->contact_hertz < 0.0f ||
        !sl_is_finite(config->contact_damping_ratio) ||
        config->contact_damping_ratio < 0.0f ||
        !sl_is_finite(config->contact_push_velocity_max) ||
        config->contact_push_velocity_max < 0.0f ||
        !sl_is_finite(config->restitution_threshold) ||
        config->restitution_threshold < 0.0f ||
        config->joint_capacity > SL_JOINT_COUNT_MAX ||
        !sl_is_finite(config->joint_hertz) || config->joint_hertz < 0.0f ||
        !sl_is_finite(config->joint_damping_ratio) ||
        config->joint_damping_ratio < 0.0f) {
        return false;
    }

    uint32_t contacts = config->contact_capacity;
    if (contacts == 0u) {
        if (config->body_capacity > SL_CONTACT_COUNT_MAX / 4u) {
            return false;
        }
        contacts = 4u * config->body_capacity;
    }
    if (contacts < 1u || contacts > SL_CONTACT_COUNT_MAX) {
        return false;
    }

    *resolved = *config;
    resolved->sleep_speed_max = config->sleep_speed_max == 0.0f
                                    ? SL_SLEEP_SPEED_MAX_DEFAULT
                                    : config->sleep_speed_max;
    resolved->sleep_angular_speed_max = config->sleep_angular_speed_max == 0.0f
                                            ? SL_SLEEP_ANGULAR_SPEED_MAX_DEFAULT
                                            : config->sleep_angular_speed_max;
    resolved->sleep_time_min = config->sleep_time_min == 0.0f
                                   ? SL_SLEEP_TIME_MIN_DEFAULT
                                   : config->sleep_time_min;
    resolved->contact_capacity = contacts;
    resolved->substep_count = (config->substep_count == 0u)
                                  ? SL_SUBSTEP_COUNT_DEFAULT
                                  : config->substep_count;
    resolved->linear_speed_max = (config->linear_speed_max == 0.0f)
                                     ? SL_LINEAR_SPEED_MAX_DEFAULT
                                     : config->linear_speed_max;
    resolved->contact_hertz = (config->contact_hertz == 0.0f)
                                  ? SL_CONTACT_HERTZ_DEFAULT
                                  : config->contact_hertz;
    resolved->contact_damping_ratio = (config->contact_damping_ratio == 0.0f)
                                          ? SL_CONTACT_DAMPING_RATIO_DEFAULT
                                          : config->contact_damping_ratio;
    resolved->contact_push_velocity_max =
        (config->contact_push_velocity_max == 0.0f)
            ? SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT
            : config->contact_push_velocity_max;
    resolved->restitution_threshold = (config->restitution_threshold == 0.0f)
                                          ? SL_RESTITUTION_THRESHOLD_DEFAULT
                                          : config->restitution_threshold;
    resolved->joint_hertz = (config->joint_hertz == 0.0f)
                                ? SL_JOINT_HERTZ_DEFAULT
                                : config->joint_hertz;
    resolved->joint_damping_ratio = (config->joint_damping_ratio == 0.0f)
                                        ? SL_JOINT_DAMPING_RATIO_DEFAULT
                                        : config->joint_damping_ratio;

    /* Static contacts double hertz before the runtime substep-rate cap.
     * Validate that path and the worst capped h*omega softness terms now. */
    const float doubled_hertz = 2.0f * resolved->contact_hertz;
    const float omega = 2.0f * SL_PI * doubled_hertz;
    const float h_omega_max = 0.5f * SL_PI;
    const float a_1_max = 2.0f * resolved->contact_damping_ratio + h_omega_max;
    const float a_2_max = h_omega_max * a_1_max;
    if (!sl_is_finite(doubled_hertz) || !sl_is_finite(omega) ||
        !sl_is_finite(a_1_max) || !sl_is_finite(a_2_max) ||
        !sl_is_finite(1.0f + a_2_max)) {
        return false;
    }

    /* The joint cap is one eighth of the substep rate, so h*omega is at
     * most pi/4. Validate the raw tuning and that worst capped path. */
    const float joint_omega = 2.0f * SL_PI * resolved->joint_hertz;
    const float joint_h_omega_max = 0.25f * SL_PI;
    const float joint_a_1_max =
        2.0f * resolved->joint_damping_ratio + joint_h_omega_max;
    const float joint_a_2_max = joint_h_omega_max * joint_a_1_max;
    if (!sl_is_finite(joint_omega) || !sl_is_finite(joint_a_1_max) ||
        !sl_is_finite(joint_a_2_max) || !sl_is_finite(1.0f + joint_a_2_max)) {
        return false;
    }
    return true;
}

bool sl_world_memory_breakdown_get(const sl_world_config *config,
                                   sl_world_memory_breakdown *out)
{
    sl_world_config resolved;
    if (out == NULL || !world_config_resolve(config, &resolved)) {
        return false;
    }
    sl_world_memory_breakdown result = { 0 };
    const size_t body =
        world_body_layout(resolved.body_capacity, &result.body_bytes);
    const size_t contact = sl_contact_world_memory_layout(
        resolved.body_capacity, resolved.contact_capacity, &result);
    const size_t solver = sl_solver_memory_bytes(resolved.contact_capacity);
    const size_t joint = sl_joint_memory_layout(
        resolved.body_capacity, resolved.joint_capacity, &result.joint_bytes);
    const size_t island =
        island_layout(resolved.body_capacity, resolved.contact_capacity,
                      resolved.joint_capacity, &result.island_bytes);
    if (contact == 0u || solver == 0u ||
        (resolved.joint_capacity > 0u && joint == 0u) ||
        body > SIZE_MAX - contact || body + contact > SIZE_MAX - solver ||
        body + contact + solver > SIZE_MAX - joint) {
        return false;
    }
    /* Solver records are pinned to an 8-byte multiple in solver.c. */
    result.sleep_bytes = (size_t)resolved.body_capacity * 13u;
    result.padding_bytes +=
        sleep_layout(resolved.body_capacity) - result.sleep_bytes;
    result.contact_solver_bytes = solver;
    result.padding_bytes += body - result.body_bytes + joint -
                            result.joint_bytes + island - result.island_bytes;
    result.world_state_bytes = sizeof(sl_world_state);
    result.padding_bytes +=
        align_up(sizeof(sl_world_state)) - sizeof(sl_world_state);
    result.arena_bytes = align_up(sizeof(sl_world_state)) + body + contact +
                         solver + joint + island +
                         sleep_layout(resolved.body_capacity);
    result.world_bytes = sizeof(sl_world);
    *out = result;
    return true;
}
size_t sl_world_memory_bytes(const sl_world_config *config)
{
    sl_world_memory_breakdown result;
    return sl_world_memory_breakdown_get(config, &result) ? result.arena_bytes
                                                          : 0u;
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

/* Comparisons reject NaN and infinities as well as coordinates outside
 * the supported body-center domain. */
static bool position_valid(sl_vec2 position)
{
    return sl_abs(position.x) <= SL_POSITION_ABS_MAX &&
           sl_abs(position.y) <= SL_POSITION_ABS_MAX;
}

/* A point on a body reaches one shape extent past its center, so the
 * force-point domain is wider than the body-center domain. */
static bool point_valid(sl_vec2 point)
{
    const float point_abs_max = SL_POSITION_ABS_MAX + SL_SHAPE_EXTENT_MAX;
    return sl_abs(point.x) <= point_abs_max && sl_abs(point.y) <= point_abs_max;
}

static bool body_type_valid(sl_body_type type)
{
    return type == SL_BODY_DYNAMIC || type == SL_BODY_KINEMATIC ||
           type == SL_BODY_STATIC;
}

/* Rebuilds a validated shape from only its live fields. The caller owns
 * NULL-to-NONE policy and validation; keeping those outside makes this
 * writer's contract match sl_shape_is_valid and lets create validate its
 * complete descriptor before deriving any state. */
static void shape_write_normalized(const sl_shape *shape, sl_shape *out)
{
    SL_ASSERT(shape != NULL);
    SL_ASSERT(out != NULL);
    SL_ASSERT(sl_shape_is_valid(shape));

    switch (shape->kind) {
    case SL_SHAPE_NONE:
        *out = sl_shape_none();
        return;
    case SL_SHAPE_CIRCLE: {
        const bool written = sl_shape_make_circle(shape->circle.radius, out);
        SL_ASSERT(written);
        (void)written;
        return;
    }
    case SL_SHAPE_POLYGON: {
        const uint32_t count = shape->polygon.count;
        SL_ASSERT(count >= 3u && count <= SL_POLYGON_VERTEX_COUNT_MAX);

        sl_shape normalized = sl_shape_none();
        normalized.kind = SL_SHAPE_POLYGON;
        normalized.polygon.count = count;
        for (uint32_t i = 0u; i < count; ++i) {
            normalized.polygon.vertices[i] = shape->polygon.vertices[i];
        }
        *out = normalized;
        return;
    }
    }

    /* Unreachable for a shape that passed sl_shape_is_valid. No default
     * above: adding a shape kind must produce a compiler warning here. */
    SL_ASSERT(false);
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
    SL_ASSERT(shape != NULL);
    if (shape->kind == SL_SHAPE_NONE) {
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
    if (!sl_is_finite(desc->friction) || desc->friction < 0.0f ||
        !sl_is_finite(desc->restitution) || desc->restitution < 0.0f ||
        desc->restitution > 1.0f) {
        return false;
    }
    if (desc->shape != NULL && !sl_shape_is_valid(desc->shape)) {
        return false;
    }
    return true;
}

/* inertias and inv_inertias only ever move together. */
static void body_write_inertia(sl_world_state *world, uint32_t dense,
                               float inertia)
{
    world->inertias[dense] = inertia;
    world->inv_inertias[dense] = inverse_or_zero(inertia);
}

/* Both accumulator writers ask the same question: does adding this
 * much keep the sum, and the acceleration the stepper derives from it,
 * finite? On success *out carries the value to bank. */
static bool force_sum_ok(const sl_world_state *world, uint32_t dense,
                         sl_vec2 force, sl_vec2 *out)
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

static bool torque_sum_ok(const sl_world_state *world, uint32_t dense,
                          float torque, float *out)
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

static sl_body_handle handle_for(const sl_world_state *world, uint32_t dense)
{
    sl_body_handle h = { world->slot_of[dense],
                         world->slots[world->slot_of[dense]].generation };
    return h;
}

bool sl_world_init(sl_world *owner, const sl_world_config *config)
{
    SL_ASSERT(owner != NULL);
    SL_ASSERT(config != NULL);
    /* A live world carries an arena pointer that init would drop on the
     * floor; zero-init makes the read defined and the misuse loud. */
    SL_ASSERT(owner->state == NULL);

    owner->state = NULL;

    sl_world_config resolved;
    if (!world_config_resolve(config, &resolved)) {
        return false;
    }

    const size_t capacity = (size_t)config->body_capacity;
    const size_t body_bytes = world_memory_bytes_for(capacity);
    const size_t contact_bytes = sl_contact_world_memory_bytes(
        config->body_capacity, resolved.contact_capacity);
    const size_t solver_bytes =
        sl_solver_memory_bytes(resolved.contact_capacity);
    const size_t joint_bytes =
        sl_joint_memory_bytes(config->body_capacity, resolved.joint_capacity);
    size_t island_payload = 0u;
    const size_t island_bytes =
        island_layout(resolved.body_capacity, resolved.contact_capacity,
                      resolved.joint_capacity, &island_payload);
    const size_t total_bytes = align_up(sizeof(sl_world_state)) + body_bytes +
                               contact_bytes + solver_bytes + joint_bytes +
                               island_bytes + sleep_layout(capacity);

    unsigned char *base = malloc(total_bytes);
    if (base == NULL) {
        return false;
    }
    /* The complete arena is persistent world state, including inactive
     * packed rows and alignment gaps. Give every byte a deterministic
     * initial value before carving typed slices out of it. */
    memset(base, 0, total_bytes);

    /* Carved in declaration order; the closing assert fires if an array
     * joins one list but not the other. */
    sl_world_state *world = (sl_world_state *)base;
    size_t offset = align_up(sizeof(*world));
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
    world->delta_positions =
        (sl_vec2 *)carve(base, &offset, capacity, sizeof(sl_vec2));
    world->rotations =
        (sl_rotation *)carve(base, &offset, capacity, sizeof(sl_rotation));
    world->delta_rotations =
        (sl_rotation *)carve(base, &offset, capacity, sizeof(sl_rotation));
    world->angular_velocities =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->torques = (float *)carve(base, &offset, capacity, sizeof(float));
    world->inertias = (float *)carve(base, &offset, capacity, sizeof(float));
    world->inv_inertias =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->frictions = (float *)carve(base, &offset, capacity, sizeof(float));
    world->restitutions =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->types = (uint8_t *)carve(base, &offset, capacity, sizeof(uint8_t));
    world->shapes =
        (sl_shape *)carve(base, &offset, capacity, sizeof(sl_shape));
    SL_ASSERT(offset == align_up(sizeof(*world)) + body_bytes);

    world->body_count = 0u;
    world->body_capacity = config->body_capacity;
    world->free_count = config->body_capacity;
    world->contact_count = 0u;
    world->contact_capacity = resolved.contact_capacity;
    world->contact_drop_count = 0u;
    world->pair_capacity = sl_contact_pair_capacity(resolved.contact_capacity);
    world->moved_count = 0u;
    world->joint_count = 0u;
    world->joint_capacity = resolved.joint_capacity;
    world->joint_free_count = resolved.joint_capacity;
    world->joint_constraint_count = 0u;
    world->gravity = config->gravity;
    world->linear_drag = config->linear_drag;
    world->angular_drag = config->angular_drag;
    world->substep_count = resolved.substep_count;
    world->linear_speed_max = resolved.linear_speed_max;
    world->contact_hertz = resolved.contact_hertz;
    world->contact_damping_ratio = resolved.contact_damping_ratio;
    world->contact_push_velocity_max = resolved.contact_push_velocity_max;
    world->restitution_threshold = resolved.restitution_threshold;
    world->joint_hertz = resolved.joint_hertz;
    world->joint_damping_ratio = resolved.joint_damping_ratio;

    void *contact_memory = carve(base, &offset, contact_bytes, 1u);
    if (!sl_contact_world_init(world, contact_memory, contact_bytes)) {
        free(base);
        owner->state = NULL;
        return false;
    }
    void *solver_memory = carve(base, &offset, solver_bytes, 1u);
    if (!sl_solver_init(world, solver_memory, solver_bytes)) {
        free(base);
        owner->state = NULL;
        return false;
    }
    void *joint_memory = NULL;
    if (joint_bytes > 0u) {
        joint_memory = carve(base, &offset, joint_bytes, 1u);
    }
    world->island_parents =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->island_sizes =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->body_islands =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->island_bodies =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->islands =
        (sl_island *)carve(base, &offset, capacity, sizeof(sl_island));
    world->island_contacts = (uint32_t *)carve(
        base, &offset, resolved.contact_capacity, sizeof(uint32_t));
    world->island_joints = (uint32_t *)carve(
        base, &offset, resolved.joint_capacity, sizeof(uint32_t));
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        world->body_islands[slot] = SL_BODY_DENSE_NONE;
    }
    world->quiet_steps =
        (uint32_t *)carve(base, &offset, capacity, sizeof(uint32_t));
    world->quiet_translation =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->quiet_rotation =
        (float *)carve(base, &offset, capacity, sizeof(float));
    world->sleeping =
        (uint8_t *)carve(base, &offset, capacity, sizeof(uint8_t));
    world->sleep_enabled = resolved.sleep_enabled;
    world->sleep_speed_max = resolved.sleep_speed_max;
    world->sleep_angular_speed_max = resolved.sleep_angular_speed_max;
    world->sleep_time_min = resolved.sleep_time_min;
    SL_ASSERT(offset == total_bytes);
    if (!sl_joint_world_init(world, joint_memory, joint_bytes)) {
        free(base);
        owner->state = NULL;
        return false;
    }

    for (uint32_t i = 0u; i < world->body_capacity; ++i) {
        /* Descending push => LIFO pops hand out slots 0,1,2,... */
        world->free_indices[i] = world->body_capacity - 1u - i;
        world->slots[i].dense = SL_BODY_DENSE_NONE;
        world->slots[i].generation = 1u;
        world->slot_of[i] = SL_BODY_DENSE_NONE;
    }

    owner->state = world;
    return true;
}

void sl_world_destroy(sl_world *world)
{
    SL_ASSERT(world != NULL);
    free(world->state);
    world->state = NULL;
}

void sl_world_reset(sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);

    world->sleep_dt = 0.0f;
    world->sleep_steps_required = 0u;
    world->wake_count = 0u;
    world->wake_batch = false;
    world->island_count = 0u;
    world->island_body_count = 0u;
    world->island_contact_count = 0u;
    world->island_joint_count = 0u;
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        world->body_islands[slot] = SL_BODY_DENSE_NONE;
    }
    sl_joint_world_reset(world);
    sl_contact_world_reset(world);
    memset(&world->step_stats, 0, sizeof(world->step_stats));
    memset(&world->work_total, 0, sizeof(world->work_total));
    world->body_count_high = 0u;
    world->contact_count_high = 0u;
    world->joint_count_high = 0u;
    memset(&world->step_work, 0, sizeof(world->step_work));
    world->stats_stepping = false;
    world->contact_constraint_count = 0u;
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

sl_body_handle sl_world_body_create(sl_world *owner, const sl_body_desc *desc)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(desc != NULL);

    /* Validate the complete descriptor before deriving inertia, whose
     * polygon path walks shape->polygon.count vertices. */
    if (!body_desc_valid(desc) || world->free_count == 0u) {
        return sl_body_handle_null();
    }
    /* Derived inertia is validated before any slot is consumed, so a
     * rejection here leaves the pool untouched. */
    const float inertia = (desc->type == SL_BODY_DYNAMIC && desc->shape != NULL)
                              ? body_inertia_for(desc->mass, desc->shape)
                              : 0.0f;
    if (!body_inertia_valid(inertia)) {
        return sl_body_handle_null();
    }

    const uint32_t slot_id = world->free_indices[world->free_count - 1u];
    world->free_count--;
    world->body_islands[slot_id] = SL_BODY_DENSE_NONE;

    const uint32_t dense = world->body_count;
    world->body_count++;
    world->quiet_steps[dense] = 0u;
    world->quiet_translation[dense] = 0.0f;
    world->quiet_rotation[dense] = 0.0f;
    world->sleeping[dense] = 0u;
    if (world->body_count > world->body_count_high) {
        world->body_count_high = world->body_count;
    }

    world->slots[slot_id].dense = dense;
    world->slot_of[dense] = slot_id;

    world->positions[dense] = desc->position;
    world->velocities[dense] = desc->velocity;
    world->masses[dense] = desc->mass;
    /* body_mass_valid_for pins a non-dynamic mass at exactly zero, so
     * the encoding falls out of inverse_or_zero without a type test. */
    world->inv_masses[dense] = inverse_or_zero(desc->mass);
    world->forces[dense] = sl_vec2_make(0.0f, 0.0f);
    world->delta_positions[dense] = sl_vec2_make(0.0f, 0.0f);

    world->rotations[dense] = sl_rotation_make(sl_angle_wrap(desc->angle));
    world->delta_rotations[dense] = sl_rotation_identity();
    world->angular_velocities[dense] = desc->angular_velocity;
    world->torques[dense] = 0.0f;
    body_write_inertia(world, dense, inertia);

    world->types[dense] = (uint8_t)desc->type;
    world->frictions[dense] = desc->friction;
    world->restitutions[dense] = desc->restitution;
    if (desc->shape == NULL) {
        world->shapes[dense] = sl_shape_none();
    } else {
        shape_write_normalized(desc->shape, &world->shapes[dense]);
    }

    const sl_body_handle handle = handle_for(world, dense);
    if (!sl_contact_body_create(world, dense, slot_id)) {
        sl_world_body_destroy(owner, handle);
        return sl_body_handle_null();
    }
    return handle;
}

void sl_world_body_destroy(sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
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
    sl_sleep_body_changed(world, handle.index);
    sl_joint_body_destroy(world, handle.index);
    sl_contact_body_destroy(world, dense, handle.index);
    if (dense != last) {
        world->quiet_steps[dense] = world->quiet_steps[last];
        world->quiet_translation[dense] = world->quiet_translation[last];
        world->quiet_rotation[dense] = world->quiet_rotation[last];
        world->sleeping[dense] = world->sleeping[last];
        world->positions[dense] = world->positions[last];
        world->velocities[dense] = world->velocities[last];
        world->masses[dense] = world->masses[last];
        world->inv_masses[dense] = world->inv_masses[last];
        world->forces[dense] = world->forces[last];
        world->delta_positions[dense] = world->delta_positions[last];
        world->rotations[dense] = world->rotations[last];
        world->delta_rotations[dense] = world->delta_rotations[last];
        world->angular_velocities[dense] = world->angular_velocities[last];
        world->torques[dense] = world->torques[last];
        world->inertias[dense] = world->inertias[last];
        world->inv_inertias[dense] = world->inv_inertias[last];
        world->frictions[dense] = world->frictions[last];
        world->restitutions[dense] = world->restitutions[last];
        world->types[dense] = world->types[last];
        world->shapes[dense] = world->shapes[last];
        world->proxy_aabbs[dense] = world->proxy_aabbs[last];
        world->proxies[dense] = world->proxies[last];
        world->moved[dense] = world->moved[last];

        const uint32_t moved_slot = world->slot_of[last];
        world->slot_of[dense] = moved_slot;
        world->slots[moved_slot].dense = dense;
    }
    world->body_count--;

    world->body_islands[handle.index] = SL_BODY_DENSE_NONE;
    slot->dense = SL_BODY_DENSE_NONE;
    slot->generation = generation_bump(slot->generation);
    world->free_indices[world->free_count] = handle.index;
    world->free_count++;
}

bool sl_body_is_valid(const sl_world_state *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);

    if (handle.generation == 0u || handle.index >= world->body_capacity) {
        return false;
    }

    const sl_body_slot *slot = &world->slots[handle.index];
    return slot->generation == handle.generation &&
           slot->dense != SL_BODY_DENSE_NONE;
}

uint32_t sl_world_body_count(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    return world != NULL ? world->body_count : 0u;
}

uint32_t sl_world_body_capacity(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    return world != NULL ? world->body_capacity : 0u;
}

sl_vec2 sl_world_get_gravity(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    return world->gravity;
}

float sl_world_get_linear_drag(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    return world->linear_drag;
}

float sl_world_get_angular_drag(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    return world->angular_drag;
}

uint32_t sl_world_get_substep_count(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    return world->substep_count;
}

float sl_world_get_linear_speed_max(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    return world->linear_speed_max;
}

sl_body_handle sl_world_body_first(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);

    if (world->body_count == 0u) {
        return sl_body_handle_null();
    }
    return handle_for(world, 0u);
}

sl_body_handle sl_world_body_next(const sl_world *owner, sl_body_handle current)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, current));

    const uint32_t next_dense = world->slots[current.index].dense + 1u;
    if (next_dense >= world->body_count) {
        return sl_body_handle_null();
    }
    return handle_for(world, next_dense);
}

sl_body_handle sl_world_body_at(const sl_world *owner, uint32_t row)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(row < world->body_count);
    return handle_for(world, row);
}

sl_vec2 sl_world_body_get_position(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->positions[world->slots[handle.index].dense];
}

sl_vec2 sl_world_body_get_velocity(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->velocities[world->slots[handle.index].dense];
}

float sl_world_body_get_mass(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->masses[world->slots[handle.index].dense];
}

float sl_world_body_get_inv_mass(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->inv_masses[world->slots[handle.index].dense];
}

sl_body_type sl_world_body_get_type(const sl_world *owner,
                                    sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return (sl_body_type)world->types[world->slots[handle.index].dense];
}

float sl_world_body_get_angle(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return sl_rotation_angle(
        world->rotations[world->slots[handle.index].dense]);
}

float sl_world_body_get_angular_velocity(const sl_world *owner,
                                         sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->angular_velocities[world->slots[handle.index].dense];
}

float sl_world_body_get_inertia(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->inertias[world->slots[handle.index].dense];
}

float sl_world_body_get_inv_inertia(const sl_world *owner,
                                    sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->inv_inertias[world->slots[handle.index].dense];
}

float sl_world_body_get_torque(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->torques[world->slots[handle.index].dense];
}

float sl_world_body_get_friction(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->frictions[world->slots[handle.index].dense];
}

float sl_world_body_get_restitution(const sl_world *owner,
                                    sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->restitutions[world->slots[handle.index].dense];
}

const sl_shape *sl_world_body_get_shape(const sl_world *owner,
                                        sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return &world->shapes[world->slots[handle.index].dense];
}

sl_transform sl_world_body_get_transform(const sl_world *owner,
                                         sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    return sl_transform_make(world->positions[dense], world->rotations[dense]);
}

bool sl_world_body_set_position(sl_world *owner, sl_body_handle handle,
                                sl_vec2 position)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    if (!position_valid(position)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->positions[dense].x != position.x ||
        world->positions[dense].y != position.y) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->positions[dense] = position;
    sl_joint_body_cache_clear(world, handle.index);
    const bool updated = sl_contact_body_update(world, dense, handle.index);
    SL_ASSERT(updated);
    (void)updated;
    return true;
}

bool sl_world_body_set_velocity(sl_world *owner, sl_body_handle handle,
                                sl_vec2 velocity)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
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
    if (world->velocities[dense].x != velocity.x ||
        world->velocities[dense].y != velocity.y) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->velocities[dense] = velocity;
    return true;
}

bool sl_world_body_set_angle(sl_world *owner, sl_body_handle handle,
                             float angle)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    if (!sl_is_finite(angle)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    const sl_rotation rotation = sl_rotation_make(sl_angle_wrap(angle));
    if (sl_angle_wrap(angle) != sl_rotation_angle(world->rotations[dense]) &&
        (world->rotations[dense].c != rotation.c ||
         world->rotations[dense].s != rotation.s)) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->rotations[dense] = rotation;
    sl_joint_body_cache_clear(world, handle.index);
    const bool updated = sl_contact_body_update(world, dense, handle.index);
    SL_ASSERT(updated);
    (void)updated;
    return true;
}

bool sl_world_body_set_angular_velocity(sl_world *owner, sl_body_handle handle,
                                        float angular_velocity)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    if (!sl_is_finite(angular_velocity)) {
        return false;
    }
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] == (uint8_t)SL_BODY_STATIC &&
        angular_velocity != 0.0f) {
        return false;
    }
    if (world->angular_velocities[dense] != angular_velocity) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->angular_velocities[dense] = angular_velocity;
    return true;
}

bool sl_world_body_set_friction(sl_world *owner, sl_body_handle handle,
                                float friction)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    if (!sl_is_finite(friction) || friction < 0.0f) {
        return false;
    }
    if (world->frictions[world->slots[handle.index].dense] != friction) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->frictions[world->slots[handle.index].dense] = friction;
    return true;
}

bool sl_world_body_set_restitution(sl_world *owner, sl_body_handle handle,
                                   float restitution)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    if (!sl_is_finite(restitution) || restitution < 0.0f ||
        restitution > 1.0f) {
        return false;
    }
    if (world->restitutions[world->slots[handle.index].dense] != restitution) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->restitutions[world->slots[handle.index].dense] = restitution;
    return true;
}

bool sl_world_body_set_mass(sl_world *owner, sl_body_handle handle, float mass)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
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
    if (world->masses[dense] != mass) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->masses[dense] = mass;
    world->inv_masses[dense] = inv_mass;
    body_write_inertia(world, dense, inertia);
    sl_joint_body_cache_clear(world, handle.index);
    return true;
}

static bool shape_equal(const sl_shape *a, const sl_shape *b)
{
    if (a->kind != b->kind) {
        return false;
    }
    if (a->kind == SL_SHAPE_CIRCLE) {
        return a->circle.radius == b->circle.radius;
    }
    if (a->kind == SL_SHAPE_POLYGON) {
        if (a->polygon.count != b->polygon.count) {
            return false;
        }
        for (uint32_t i = 0u; i < a->polygon.count; ++i) {
            if (a->polygon.vertices[i].x != b->polygon.vertices[i].x ||
                a->polygon.vertices[i].y != b->polygon.vertices[i].y) {
                return false;
            }
        }
    }
    return true;
}

bool sl_world_body_set_shape(sl_world *owner, sl_body_handle handle,
                             const sl_shape *shape)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));

    if (shape != NULL && !sl_shape_is_valid(shape)) {
        return false;
    }

    const uint32_t dense = world->slots[handle.index].dense;
    const bool dynamic = world->types[dense] == (uint8_t)SL_BODY_DYNAMIC;
    const float inertia = (dynamic && shape != NULL)
                              ? body_inertia_for(world->masses[dense], shape)
                              : 0.0f;
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
    const sl_shape none = sl_shape_none();
    if (!shape_equal(&world->shapes[dense], shape == NULL ? &none : shape)) {
        sl_sleep_body_changed(world, handle.index);
    }
    if (shape == NULL) {
        world->shapes[dense] = sl_shape_none();
    } else {
        shape_write_normalized(shape, &world->shapes[dense]);
    }
    body_write_inertia(world, dense, inertia);
    sl_joint_body_cache_clear(world, handle.index);
    const bool updated = sl_contact_body_update(world, dense, handle.index);
    SL_ASSERT(updated);
    (void)updated;
    return true;
}

bool sl_world_body_apply_force(sl_world *owner, sl_body_handle handle,
                               sl_vec2 force)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    sl_vec2 summed;
    if (!sl_vec2_is_finite(force) ||
        !force_sum_ok(world, dense, force, &summed)) {
        return false;
    }
    if (force.x != 0.0f || force.y != 0.0f) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->forces[dense] = summed;
    return true;
}

sl_vec2 sl_world_body_get_force(const sl_world *owner, sl_body_handle handle)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    return world->forces[world->slots[handle.index].dense];
}

bool sl_world_body_apply_torque(sl_world *owner, sl_body_handle handle,
                                float torque)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    float summed;
    if (!sl_is_finite(torque) ||
        !torque_sum_ok(world, dense, torque, &summed)) {
        return false;
    }
    if (torque != 0.0f) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->torques[dense] = summed;
    return true;
}

bool sl_world_body_apply_force_at_point(sl_world *owner, sl_body_handle handle,
                                        sl_vec2 force, sl_vec2 point)
{
    SL_ASSERT(owner != NULL);
    sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_body_is_valid(world, handle));
    const uint32_t dense = world->slots[handle.index].dense;
    if (world->types[dense] != (uint8_t)SL_BODY_DYNAMIC) {
        return false;
    }
    if (!sl_vec2_is_finite(force) || !point_valid(point)) {
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

    if (force.x != 0.0f || force.y != 0.0f) {
        sl_sleep_body_changed(world, handle.index);
    }
    world->forces[dense] = force_sum;
    world->torques[dense] = torque_sum;
    return true;
}

sl_world_stats sl_world_get_stats(const sl_world *owner)
{
    SL_ASSERT(owner != NULL);
    const sl_world_state *world = owner->state;
    SL_ASSERT(world != NULL);
    sl_world_stats result = {
        .step = world->step_stats,
        .cumulative = world->work_total,
        .body_count_high = world->body_count_high,
        .contact_count_high = world->contact_count_high,
        .joint_count_high = world->joint_count_high,
    };
    for (uint32_t row = 0u; row < world->body_count; ++row) {
        if (world->types[row] == (uint8_t)SL_BODY_DYNAMIC) {
            result.awake_dynamic_count += world->sleeping[row] == 0u ? 1u : 0u;
            result.sleeping_dynamic_count +=
                world->sleeping[row] != 0u ? 1u : 0u;
        }
    }
    result.body_count = world->body_count;
    result.body_capacity = world->body_capacity;
    result.contact_count = world->contact_count;
    result.contact_capacity = world->contact_capacity;
    result.joint_count = world->joint_count;
    result.joint_capacity = world->joint_capacity;
    result.pair_count = world->contact_count;
    result.pair_capacity = world->pair_capacity;
    return result;
}

bool sl_world_body_is_valid(const sl_world *world, sl_body_handle handle)
{
    SL_ASSERT(world != NULL);
    return world->state != NULL && sl_body_is_valid(world->state, handle);
}
