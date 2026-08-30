#ifndef SILK_WORLD_H
#define SILK_WORLD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "silk/math.h"
#include "silk/shape.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds worst-case world allocation: 141 bytes per body (seven packed
 * scalar/vec2 arrays, two index arrays, one slot array, one type byte,
 * and a 72-byte shape record) lands at ~8.8 MiB at this cap. Raise only
 * on profiling evidence, together with the budget test. */
#define SL_BODY_COUNT_MAX 65536u

/* Maximum absolute body-center coordinate. IEEE-754 binary32 spacing at
 * 8192 is 0.0009765625; a shape adds at most SL_SHAPE_EXTENT_MAX, and the
 * largest delta between two resulting points is at most 18432, where
 * spacing is 0.001953125 -- under half the 0.005-unit contact slop planned
 * for Phase 3. Typical bodies should span 0.1-10 world units. Pinned by
 * tests/test_world.c and tests/test_step.c. */
#define SL_POSITION_ABS_MAX 8192.0f

/* slots[i].dense marker for a free slot; also the liveness bit.
 * Invariant: body_count + free_count == body_capacity. */
#define SL_BODY_DENSE_NONE UINT32_MAX

/* index means nothing unless generation != 0; generations are never
 * issued as 0, so any zeroed handle is the null handle. */
typedef struct sl_body_handle {
    uint32_t index;
    uint32_t generation;
} sl_body_handle;

static inline sl_body_handle sl_body_handle_null(void)
{
    sl_body_handle h = { UINT32_MAX, 0 };
    return h;
}

static inline bool sl_body_handle_is_null(sl_body_handle h)
{
    return h.generation == 0;
}

/* Zero is dynamic so a zeroed sl_body_desc keeps its historical
 * meaning. Static bodies never move (inv_mass = inv_inertia = 0);
 * kinematic bodies move only by their velocities and ignore forces,
 * gravity, and drag. */
typedef enum sl_body_type {
    SL_BODY_DYNAMIC = 0,
    SL_BODY_KINEMATIC = 1,
    SL_BODY_STATIC = 2
} sl_body_type;

typedef struct sl_body_desc {
    /* World units; each component in
     * [-SL_POSITION_ABS_MAX, SL_POSITION_ABS_MAX]. */
    sl_vec2 position;
    sl_vec2 velocity; /* world units / second; {0, 0} for static */
    /* Kilograms: > 0 with representable inverse for dynamic; exactly 0
     * encodes infinite mass for static and kinematic. */
    float mass;
    /* Fields below zero-fill to: dynamic, upright, at rest, point
     * particle. */
    sl_body_type type;
    /* Radians, CCW positive; any finite value is accepted and stored
     * wrapped to [-pi, pi]. */
    float angle;
    /* Radians / second; must be 0 for static. */
    float angular_velocity;
    /* NULL or kind SL_SHAPE_NONE means a point particle. Rebuilt from
     * its active fields into a zero-filled record at create; the pointer
     * and inactive payload bytes are not retained. */
    const sl_shape *shape;
} sl_body_desc;

typedef struct sl_world_config {
    uint32_t body_capacity; /* in [1, SL_BODY_COUNT_MAX], fixed at init */
    sl_vec2 gravity;        /* world units / second^2; default {0, 0} */
    float linear_drag;      /* 1 / seconds, >= 0; default 0 */
    float angular_drag;     /* 1 / seconds, >= 0; default 0 */
} sl_world_config;

typedef struct sl_body_slot {
    uint32_t dense;      /* row in [0, body_count) or SL_BODY_DENSE_NONE */
    uint32_t generation; /* bumped on destroy (wrap skips 0) */
} sl_body_slot;

/* Owning runtime object: initialize from zero, never copy after init, and
 * treat every field below as engine-private. Copying duplicates arena
 * ownership and makes destruction unsafe. */
typedef struct sl_world {
    uint32_t body_count;    /* rows used by every packed array below */
    uint32_t body_capacity; /* slot count, fixed at init */
    uint32_t free_count;

    sl_vec2 gravity;    /* world units / second^2 */
    float linear_drag;  /* 1 / seconds, >= 0 */
    float angular_drag; /* 1 / seconds, >= 0 */

    /* Dual indexing: slots[handle.index] -> packed row, slot_of[row] ->
     * owning slot. Swap-remove repairs exactly one slot_of entry. */
    sl_body_slot *slots;
    uint32_t *slot_of;
    /* LIFO of free slot ids, depth == free_count; pops yield 0,1,2,... */
    uint32_t *free_indices;

    sl_vec2 *positions;  /* world units */
    sl_vec2 *velocities; /* world units / second */
    float *masses;       /* kilograms, > 0; 0 encodes infinite mass */
    float *inv_masses;   /* 1 / mass or 0 */
    sl_vec2 *forces;     /* kilograms * world units / second^2; the
                          * stepper clears them every step */

    float *angles;             /* radians, wrapped to [-pi, pi] */
    float *angular_velocities; /* radians / second */
    float *torques;            /* force units * length; the stepper
                                * clears them every step */
    /* About the body origin, which constructed shapes keep at the
     * centroid; 0 encodes infinite inertia or no shape. */
    float *inertias;
    float *inv_inertias; /* 1 / inertia or 0; create / set_mass /
                          * set_shape are its only writers */
    uint8_t *types;      /* holds sl_body_type */
    /* Whole-record per-body array: consumers (mass data, bounds,
     * containment, later contacts) always want the complete shape, so
     * the record is the SoA granularity they use. */
    sl_shape *shapes;

    void *memory; /* backing block carved into every array above */
} sl_world;

/* Bytes sl_world_init allocates for the given capacity; 0 when
 * capacity is outside [1, SL_BODY_COUNT_MAX]. Lets callers size memory
 * and lets tests pin the budget. */
size_t sl_world_memory_bytes(uint32_t body_capacity);

/* *world must be zero-initialized or previously destroyed: re-initializing
 * a live world leaks its arena, so the assert fires in debug builds.
 * Returns false, leaving *world zeroed, when body_capacity is outside
 * [1, SL_BODY_COUNT_MAX], gravity is non-finite, linear_drag or
 * angular_drag is non-finite or negative, or allocation fails.
 * The full arena, including inactive rows and alignment gaps, starts
 * zeroed. Everything is allocated here; nothing allocates during
 * simulation. */
bool sl_world_init(sl_world *world, const sl_world_config *config);

/* Frees all engine-owned memory and zeroes *world; safe to repeat. */
void sl_world_destroy(sl_world *world);

/* Destroys every body and bumps every generation, so handles held from
 * before a reset never validate afterwards. */
void sl_world_reset(sl_world *world);

/* Returns the null handle -- leaving the world unchanged -- when any
 * descriptor rule fails: position outside SL_POSITION_ABS_MAX, velocity
 * non-finite, unknown type,
 * a mass violating its type's rule (positive with representable inverse
 * for dynamic, exactly 0 for static and kinematic), a static carrying
 * velocity or spin, a non-finite angle or angular velocity, a shape
 * sl_shape_is_valid rejects, or a derived inertia whose inverse would
 * overflow; also when the world is full. */
sl_body_handle sl_world_body_create(sl_world *world, const sl_body_desc *desc);

/* Tolerant no-op for null, malformed, and stale handles, so deferred
 * destruction never corrupts the pool. */
void sl_world_body_destroy(sl_world *world, sl_body_handle handle);

/* True when handle names a live body in this world; safe on arbitrary
 * handles, where the accessors would assert. */
bool sl_world_body_is_valid(const sl_world *world, sl_body_handle handle);

uint32_t sl_world_body_count(const sl_world *world);
uint32_t sl_world_body_capacity(const sl_world *world);

sl_vec2 sl_world_get_gravity(const sl_world *world);
float sl_world_get_linear_drag(const sl_world *world);
float sl_world_get_angular_drag(const sl_world *world);

/* Read-only walks visit live bodies in packed order: deterministic for
 * a given operation sequence, not creation order across destroys. Both
 * ends read as the null handle. Destroy is a swap-remove -- the last
 * packed body lands behind the cursor -- so handles cannot express
 * removal during such a walk; destroying walks use at() with a row
 * cursor instead, retesting the row a removal just refilled:
 *
 *     for (uint32_t row = 0u; row < sl_world_body_count(&world);) {
 *         sl_body_handle body = sl_world_body_at(&world, row);
 *         if (remove(&world, body)) {
 *             sl_world_body_destroy(&world, body);
 *         } else {
 *             row++;
 *         }
 *     }
 */
sl_body_handle sl_world_body_first(const sl_world *world);
sl_body_handle sl_world_body_next(const sl_world *world,
                                  sl_body_handle current);

/* Handle of the body at packed row; asserts row < body_count. Pairs
 * with destroy() for removal-during-traversal walks (see above). */
sl_body_handle sl_world_body_at(const sl_world *world, uint32_t row);

/* Accessors assert handle validity. Setters return false -- leaving
 * state unchanged -- for values create() would reject, true otherwise. */
sl_vec2 sl_world_body_get_position(const sl_world *world,
                                   sl_body_handle handle);
sl_vec2 sl_world_body_get_velocity(const sl_world *world,
                                   sl_body_handle handle);
/* 0 means infinite mass (static or kinematic body). */
float sl_world_body_get_mass(const sl_world *world, sl_body_handle handle);
float sl_world_body_get_inv_mass(const sl_world *world, sl_body_handle handle);
sl_body_type sl_world_body_get_type(const sl_world *world,
                                    sl_body_handle handle);
/* Radians in [-pi, pi]. */
float sl_world_body_get_angle(const sl_world *world, sl_body_handle handle);
float sl_world_body_get_angular_velocity(const sl_world *world,
                                         sl_body_handle handle);
/* About the body origin; 0 for static, kinematic, and shapeless
 * bodies. */
float sl_world_body_get_inertia(const sl_world *world, sl_body_handle handle);
float sl_world_body_get_inv_inertia(const sl_world *world,
                                    sl_body_handle handle);
float sl_world_body_get_torque(const sl_world *world, sl_body_handle handle);

/* Points into packed storage: valid until the next create / destroy /
 * reset / set_shape. Never NULL; kind SL_SHAPE_NONE marks a point
 * particle. */
const sl_shape *sl_world_body_get_shape(const sl_world *world,
                                        sl_body_handle handle);

/* sl_transform_make(position, sl_rotation_make(angle)): pays libm trig
 * per call, so renderers rotate cheaply while stored state stays
 * trig-free. */
sl_transform sl_world_body_get_transform(const sl_world *world,
                                         sl_body_handle handle);

bool sl_world_body_set_position(sl_world *world, sl_body_handle handle,
                                sl_vec2 position);
/* A static body accepts only the zero velocity. */
bool sl_world_body_set_velocity(sl_world *world, sl_body_handle handle,
                                sl_vec2 velocity);
/* Any finite angle; stored wrapped to [-pi, pi]. */
bool sl_world_body_set_angle(sl_world *world, sl_body_handle handle,
                             float angle);
/* A static body accepts only zero. */
bool sl_world_body_set_angular_velocity(sl_world *world, sl_body_handle handle,
                                        float angular_velocity);
/* Dynamic bodies only; re-derives inertia from the attached shape.
 * Also false -- unchanged -- when the new mass or inertia would turn an
 * already-banked force or torque into an infinite acceleration: both
 * apply_force and apply_torque accepted those against the old
 * inverses, and lowering an inverse's denominator must not smuggle
 * infinity past the guard they enforced. */
bool sl_world_body_set_mass(sl_world *world, sl_body_handle handle, float mass);
/* Rebuilds shape from its active fields into a zero-filled record (NULL
 * or kind SL_SHAPE_NONE detaches, restoring a point particle) and
 * re-derives inertia. Inactive union bytes and polygon vertices past
 * count are not copied. Any body may carry a shape; only dynamic bodies
 * gain inertia from it. False -- unchanged -- for shapes
 * sl_shape_is_valid rejects, an inertia with no finite inverse, or a
 * new inertia that would make the banked torque overflow (see
 * sl_world_body_set_mass). */
bool sl_world_body_set_shape(sl_world *world, sl_body_handle handle,
                             const sl_shape *shape);

/* Adds force to the body's accumulator; the stepper consumes and clears
 * it each step. Dynamic bodies only -- a force on a body that cannot
 * respond is rejected, not silently dropped. Returns false -- leaving
 * the accumulator unchanged -- for non-dynamic bodies, non-finite
 * force, an accumulation that would overflow to infinity, or an
 * accumulated acceleration that overflows at the body's current mass.
 * Rejection happens at application time; integration itself neither
 * clamps nor saturates (see step.h). */
bool sl_world_body_apply_force(sl_world *world, sl_body_handle handle,
                               sl_vec2 force);
sl_vec2 sl_world_body_get_force(const sl_world *world, sl_body_handle handle);

/* Dynamic bodies only. Mirrors apply_force: rejects non-finite torque,
 * an overflowing accumulation, or an angular acceleration that would
 * overflow at the body's current inertia. Torque units follow force
 * units times length. */
bool sl_world_body_apply_torque(sl_world *world, sl_body_handle handle,
                                float torque);

/* Accumulates the force and the torque cross(point - position, force)
 * together: both accumulators change or neither does. Dynamic bodies
 * only, under the same overflow rules as apply_force and apply_torque.
 * Each point component must lie within SL_POSITION_ABS_MAX +
 * SL_SHAPE_EXTENT_MAX, the coordinate domain reachable by a point on a
 * valid body. */
bool sl_world_body_apply_force_at_point(sl_world *world, sl_body_handle handle,
                                        sl_vec2 force, sl_vec2 point);

#ifdef __cplusplus
}
#endif

#endif /* SILK_WORLD_H */
