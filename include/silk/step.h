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
 *     v <- (v + (gravity + f/m) * dt) / (1 + dt * linear_drag)
 *     p <- p + v * dt
 *
 * Semi-implicit Euler — position uses the post-drag velocity. The drag
 * form divides instead of subtracting, so any drag >= 0 stays stable.
 * dt is THE fixed timestep and must be identical across the calls of a
 * run; timing policy lives above this function. Consumed forces clear. */
void sl_world_step(sl_world *world, float dt);

typedef struct sl_stepper {
    float timestep;  /* fixed dt in seconds, finite > 0 */
    float remainder; /* carried seconds, always in [0, timestep) */
} sl_stepper;

/* Returns false, leaving *stepper zeroed, when timestep is non-finite
 * or <= 0. */
bool sl_stepper_init(sl_stepper *stepper, float timestep);

/* Accumulates frame_time seconds and runs floor((remainder +
 * frame_time) / timestep) fixed steps, capped at SL_STEP_COUNT_MAX —
 * leftover beyond the cap is dropped, never carried. Non-finite or
 * negative frame_time reads as 0. Returns the number of steps executed.
 * Deterministic for a given call sequence; no wall-clock time enters. */
uint32_t sl_world_advance(sl_world *world, sl_stepper *stepper,
                          float frame_time);

#ifdef __cplusplus
}
#endif

#endif /* SILK_STEP_H */
