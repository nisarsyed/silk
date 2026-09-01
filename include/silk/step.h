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

/* Advances the simulation by exactly one fixed step of dt seconds, divided
 * into the world's configured n substeps with h = dt / n:
 *
 *     dynamic:    v <- (v + (gravity + f/m) * h) / (1 + h * linear_drag)
 *                 w <- (w + t/I * h)             / (1 + h * angular_drag)
 *     kinematic:  v, w ignore acceleration and drag
 *     static:     v, w are zero and stay zero
 *     moving:     v <- linear_cap(v), w <- angular_cap(w, dt)
 *     moving:     dp <- dp + v * h
 *                 dq <- normalize(dq + w * h * perp(dq))
 *     finalize:   p <- p + dp, q <- normalize(dq * q)
 *                 f <- 0, t <- 0
 *
 * Before integration, persistent broad-phase contacts are reaped, refreshed,
 * and discovered. Joints and touching contacts are prepared once. Each
 * substep warm-starts and solves joints before biased contact impulses, then
 * performs the same joint-before-contact order for the unbiased relax after
 * delta integration. Two-point manifolds solve their coupled normal LCP in
 * the biased pass. Restitution runs once after all substeps. After
 * finalization, shaped bodies that escaped their fat proxy AABBs are moved and
 * queued for the following step.
 *
 * Semi-implicit Euler -- each delta uses the post-drag velocity. The drag
 * forms divide instead of subtracting, so any drag >= 0 stays stable; force
 * and torque remain active for all substeps and clear once at finalization.
 * Body type gates gravity and both drags only; the remaining
 * differences are world invariants (non-dynamic rows carry
 * inv_mass = inv_inertia = 0 and empty accumulators, static rows carry
 * zero velocities), which integration neither reads nor disturbs.
 * Angular drag gates on body type, not inertia: a dynamic body with infinite
 * rotational inertia still damps toward zero spin. Before integration and
 * constraint work, every dynamic and kinematic linear velocity is capped to
 * linear_speed_max and angular velocity to SL_ROTATION_PER_STEP_MAX / dt.
 * Joint and contact impulses are validated atomically and may raise a finite
 * post-solver velocity above the free-integration cap.
 *
 * Linear axes test base position plus candidate accumulated delta on every
 * substep. An out-of-domain dynamic axis holds and zeroes its velocity;
 * kinematic motion holds while controller velocity remains. Rotation advances
 * by atan(w * h) per substep. Non-finite velocity candidates retain the prior
 * finite state before the cap. Invalid dt or derived h/inverse terms assert in
 * debug and return without mutation in release. dt must be identical across
 * calls of a run; timing policy lives above this function. */
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
