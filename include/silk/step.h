#ifndef SILK_STEP_H
#define SILK_STEP_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/world.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds steps executed per advance; frame time beyond the cap is dropped
 * rather than carried, so debt never accumulates. Pinned by
 * tests/test_step.c. */
#define SL_STEP_COUNT_MAX 8u

/* Advances the world by one fixed step of dt seconds, split into n
 * substeps of h = dt / n, n from the configured minimum raised as needed
 * for SL_STIFFNESS_STEP_MAX (see sl_world_stats). Each substep solves
 * backward Euler in its
 * variational form with Vertex Block Descent (Chen et al., SIGGRAPH 2024):
 *
 *     minimize  G(x) = sum_i m_i / (2 h^2) |x_i - y_i|^2 + E(x)
 *     y_i       = x_i + h v_i + h^2 (gravity + drag_i / m_i)
 *
 * where E sums StVK membrane, quadratic bending, and penalty contact
 * energies. Every iteration visits each cloth's vertices color by color in
 * ascending vertex order and replaces x_i by one Newton step on G restricted
 * to x_i. Iterations start from x = y. Then v = (x - x_start) / h;
 * Coulomb friction acts on contacting vertices' tangential velocity, edge
 * stretch damping acts on v, and v is capped at speed_max.
 *
 * With self contact enabled (sl_world_config.self_contact_capacity), the
 * step adds Offset Geometric Contact (Chen et al., SIGGRAPH 2025): contacts
 * between vertices and faces and between edges are detected at the start
 * of the step and enter each vertex block as a two-stage barrier energy,
 * with Coulomb friction at velocity level. Every move the solver makes is
 * kept within per-vertex displacement bounds from the last detection
 * (relative to each cloth's common translation), which guarantees that
 * no edge passes through a triangle without continuous collision
 * detection; contacts are re-detected once 1% of the vertices reach their
 * bounds. Pinned vertices move kinematically and are not bounded: a target
 * that drags cloth through itself or another cloth is not prevented.
 *
 * Coloring makes the result independent of how a color is partitioned
 * across workers, so it is reproducible bit for bit on one platform and
 * build for a given call sequence. A skipped vertex solve (singular or
 * non-finite block) leaves that vertex unchanged; coordinates are clamped
 * to SL_POSITION_ABS_MAX. Non-finite or non-positive dt asserts in debug
 * and returns without mutation in release. */
void sl_world_step(sl_world *world, float dt);

typedef struct sl_stepper {
    float timestep;  /* fixed dt in seconds, finite > 0 */
    float remainder; /* carried seconds, always in [0, timestep) */
} sl_stepper;

/* Returns false, leaving *stepper zeroed, when timestep is non-finite or
 * <= 0. */
bool sl_stepper_init(sl_stepper *stepper, float timestep);

/* Accumulates frame_time seconds and runs floor((remainder + frame_time) /
 * timestep) fixed steps, capped at SL_STEP_COUNT_MAX; leftover beyond the
 * cap is dropped, never carried. Non-finite or negative frame_time reads as
 * 0. Returns the number of steps executed. */
uint32_t sl_world_advance(sl_world *world, sl_stepper *stepper,
                          float frame_time);

#ifdef __cplusplus
}
#endif

#endif /* SILK_STEP_H */
