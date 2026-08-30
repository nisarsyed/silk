#ifndef SILK_STEP_H
#define SILK_STEP_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/world.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds steps executed per advance; frame time beyond the cap is
 * dropped rather than carried, so debt never accumulates. Pinned by
 * tests/test_step.c. */
#define SL_STEP_COUNT_MAX 8u

/* Advances the simulation by exactly one fixed step of dt seconds:
 *
 *     dynamic:    v <- (v + (gravity + f/m) * dt) / (1 + dt * linear_drag)
 *                 w <- (w + t/I * dt)           / (1 + dt * angular_drag)
 *     kinematic:  v, w unchanged
 *     static:     v, w are zero and stay zero
 *     all:        p <- p + v * dt
 *                 a <- wrap(a + w * dt)          (a stored in [-pi, pi])
 *                 f <- 0, t <- 0
 *
 * Semi-implicit Euler -- position uses the post-drag velocity. The drag
 * forms divide instead of subtracting, so any drag >= 0 stays stable.
 * Body type gates gravity and both drags only; the remaining
 * differences are world invariants (non-dynamic rows carry
 * inv_mass = inv_inertia = 0 and empty accumulators, static rows carry
 * zero velocities), which integration neither reads nor disturbs.
 * Angular drag gates on body type, not inertia: a dynamic body with
 * infinite rotational inertia still damps toward zero spin.
 * Integration neither clamps nor saturates: force, torque, velocity,
 * angular velocity, and gravity magnitudes are expected to stay within
 * float range across a step. Linear axes commit independently: a
 * non-finite candidate velocity leaves that axis unchanged; a position
 * inside SL_POSITION_ABS_MAX commits with its finite velocity; and a
 * position outside the domain holds. On that last case a dynamic velocity
 * component is zeroed, preventing unbounded outward accumulation and
 * allowing inward acceleration to recover on the next step, so the body
 * halts up to velocity * dt inside the bound rather than against it;
 * kinematic velocity remains controller-owned. Spin commits when finite and
 * angle holds when wrapping would be non-finite. dt is the caller-selected
 * fixed timestep; Silk supplies no default. It must be identical across the
 * calls of a run, and timing policy lives above this function. Consumed
 * forces and torques always clear. */
void sl_world_step(sl_world *world, float dt);

typedef struct sl_stepper {
    float timestep;  /* fixed dt in seconds, finite > 0 */
    float remainder; /* carried seconds, always in [0, timestep) */
} sl_stepper;

/* Returns false, leaving *stepper zeroed, when timestep is non-finite
 * or <= 0. */
bool sl_stepper_init(sl_stepper *stepper, float timestep);

/* Accumulates frame_time seconds and runs floor((remainder +
 * frame_time) / timestep) fixed steps, capped at SL_STEP_COUNT_MAX --
 * leftover beyond the cap is dropped, never carried. stepper must come
 * from sl_stepper_init first. Non-finite or negative frame_time reads
 * as 0. Returns the number of steps executed.
 * Deterministic for a given call sequence; no wall-clock time enters. */
uint32_t sl_world_advance(sl_world *world, sl_stepper *stepper,
                          float frame_time);

#ifdef __cplusplus
}
#endif

#endif /* SILK_STEP_H */
