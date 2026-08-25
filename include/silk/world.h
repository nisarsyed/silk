#ifndef SILK_WORLD_H
#define SILK_WORLD_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds worst-case world allocation to single-digit MiB; raise on
 * profiling evidence only. */
#define SL_BODY_COUNT_MAX 65536u

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

typedef struct sl_body_desc {
    sl_vec2 position; /* world units */
    sl_vec2 velocity; /* world units / second */
    float mass;       /* kilograms, > 0, representable inverse */
} sl_body_desc;

typedef struct sl_world_config {
    uint32_t body_capacity; /* in [1, SL_BODY_COUNT_MAX], fixed at init */
    sl_vec2 gravity;        /* world units / second^2; default {0, 0} */
    float linear_drag;      /* 1 / seconds, >= 0; default 0 */
} sl_world_config;

typedef struct sl_body_slot {
    uint32_t dense;      /* row in [0, body_count) or SL_BODY_DENSE_NONE */
    uint32_t generation; /* bumped on destroy (wrap skips 0) */
} sl_body_slot;

typedef struct sl_world {
    uint32_t body_count;    /* rows used by every packed array below */
    uint32_t body_capacity; /* slot count, fixed at init */
    uint32_t free_count;

    sl_vec2 gravity;   /* world units / second^2 */
    float linear_drag; /* 1 / seconds, >= 0 */

    /* Dual indexing: slots[handle.index] -> packed row, slot_of[row] ->
     * owning slot. Swap-remove repairs exactly one slot_of entry. */
    sl_body_slot *slots;
    uint32_t *slot_of;
    /* LIFO of free slot ids, depth == free_count; pops yield 0,1,2,... */
    uint32_t *free_indices;

    sl_vec2 *positions;  /* world units */
    sl_vec2 *velocities; /* world units / second */
    float *masses;       /* kilograms, > 0 */
    float *inv_masses;   /* 1 / mass; set_mass is its only writer */
    sl_vec2 *forces;     /* kilograms * world units / second^2; the
                          * stepper clears them every step */

    void *memory; /* backing block carved into every array above */
} sl_world;

/* *world must be zero-initialized or previously destroyed: re-initializing
 * a live world leaks its arena, so the assert fires in debug builds.
 * Returns false, leaving *world zeroed, when body_capacity is outside
 * [1, SL_BODY_COUNT_MAX], gravity is non-finite, linear_drag is
 * non-finite or negative, or allocation fails. Everything is allocated
 * here; nothing allocates during simulation. */
bool sl_world_init(sl_world *world, const sl_world_config *config);

/* Frees all engine-owned memory and zeroes *world; safe to repeat. */
void sl_world_destroy(sl_world *world);

/* Destroys every body and bumps every generation, so handles held from
 * before a reset never validate afterwards. */
void sl_world_reset(sl_world *world);

/* Returns the null handle — leaving the world unchanged — when position
 * or velocity is non-finite, mass is non-finite, <= 0, or overflows its
 * inverse, or when the world is full. */
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

/* Read-only walks visit live bodies in packed order: deterministic for
 * a given operation sequence, not creation order across destroys. Both
 * ends read as the null handle. Destroy is a swap-remove — the last
 * packed body lands behind the cursor — so handles cannot express
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

/* Accessors assert handle validity. Setters return false — leaving
 * state unchanged — for values create() would reject, true otherwise. */
sl_vec2 sl_world_body_get_position(const sl_world *world,
                                   sl_body_handle handle);
sl_vec2 sl_world_body_get_velocity(const sl_world *world,
                                   sl_body_handle handle);
float sl_world_body_get_mass(const sl_world *world, sl_body_handle handle);
float sl_world_body_get_inv_mass(const sl_world *world, sl_body_handle handle);

bool sl_world_body_set_position(sl_world *world, sl_body_handle handle,
                                sl_vec2 position);
bool sl_world_body_set_velocity(sl_world *world, sl_body_handle handle,
                                sl_vec2 velocity);
bool sl_world_body_set_mass(sl_world *world, sl_body_handle handle, float mass);

/* Adds force to the body's accumulator; the stepper consumes and clears
 * it each step. Returns false — leaving the accumulator unchanged — for
 * non-finite force, an accumulation that would overflow to infinity, or
 * an accumulated acceleration that overflows at the body's current mass.
 * This rejects at application time only; integration itself neither
 * clamps nor saturates (see step.h). */
bool sl_world_body_apply_force(sl_world *world, sl_body_handle handle,
                               sl_vec2 force);
sl_vec2 sl_world_body_get_force(const sl_world *world, sl_body_handle handle);

#ifdef __cplusplus
}
#endif

#endif /* SILK_WORLD_H */
