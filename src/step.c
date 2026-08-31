#include "silk/step.h"

#include <string.h>

/* Commits one linear axis. A held dynamic axis drops to zero velocity so
 * outward acceleration cannot bank indefinitely: the body halts at the
 * last in-domain position, up to velocity * dt inside the bound rather
 * than against it. Kinematic velocity stays controller-owned. */
static inline void position_axis_commit(float velocity, float dt, bool dynamic,
                                        float *position_stored,
                                        float *velocity_stored)
{
    if (!sl_is_finite(velocity)) {
        return;
    }
    *velocity_stored = velocity;
    const float position = *position_stored + velocity * dt;
    if (sl_abs(position) <= SL_POSITION_ABS_MAX) {
        *position_stored = position;
    } else if (dynamic) {
        *velocity_stored = 0.0f;
    }
}

void sl_world_step(sl_world *world, float dt)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_is_finite(dt));
    SL_ASSERT(dt > 0.0f);

    /* (1 + dt * drag) >= 1 for every accepted world, so each scale is
     * in (0, 1] and can never reverse a velocity. */
    const float drag_scale = 1.0f / (1.0f + dt * world->linear_drag);
    const float angular_drag_scale = 1.0f / (1.0f + dt * world->angular_drag);

    for (uint32_t i = 0u; i < world->body_count; ++i) {
        const uint8_t type = world->types[i];
        const bool dynamic = (type == (uint8_t)SL_BODY_DYNAMIC);
        const bool moving = dynamic || (type == (uint8_t)SL_BODY_KINEMATIC);
        /* The type gates only gravity and the two drags; everything
         * else is an invariant create and the setters maintain. */
        SL_ASSERT(dynamic || world->inv_masses[i] == 0.0f);
        SL_ASSERT(dynamic || world->inv_inertias[i] == 0.0f);
        SL_ASSERT(moving || (world->velocities[i].x == 0.0f &&
                             world->velocities[i].y == 0.0f &&
                             world->angular_velocities[i] == 0.0f));

        const sl_vec2 gravity =
            dynamic ? world->gravity : sl_vec2_make(0.0f, 0.0f);
        const sl_vec2 force =
            dynamic ? world->forces[i] : sl_vec2_make(0.0f, 0.0f);
        const float torque = dynamic ? world->torques[i] : 0.0f;

        const sl_vec2 acceleration =
            sl_vec2_add(gravity, sl_vec2_scale(force, world->inv_masses[i]));
        const float angular_acceleration = torque * world->inv_inertias[i];

        if (moving) {
            sl_vec2 velocity = sl_vec2_add(world->velocities[i],
                                           sl_vec2_scale(acceleration, dt));
            float spin =
                world->angular_velocities[i] + angular_acceleration * dt;
            if (dynamic) {
                velocity = sl_vec2_scale(velocity, drag_scale);
                spin = spin * angular_drag_scale;
            }
            /* Axes commit independently so one boundary cannot freeze
             * motion along the other. */
            position_axis_commit(velocity.x, dt, dynamic,
                                 &world->positions[i].x,
                                 &world->velocities[i].x);
            position_axis_commit(velocity.y, dt, dynamic,
                                 &world->positions[i].y,
                                 &world->velocities[i].y);
            if (sl_is_finite(spin)) {
                world->angular_velocities[i] = spin;
                world->rotations[i] =
                    sl_rotation_integrate(world->rotations[i], spin * dt);
            }
        }

        /* Accumulators are consumed whether or not their body moved:
         * non-dynamic rows carry zeros, and zero-clearing is bitwise
         * stable. */
        world->forces[i] = sl_vec2_make(0.0f, 0.0f);
        world->torques[i] = 0.0f;
    }
}

bool sl_stepper_init(sl_stepper *stepper, float timestep)
{
    SL_ASSERT(stepper != NULL);

    if (!sl_is_finite(timestep) || timestep <= 0.0f) {
        memset(stepper, 0, sizeof(*stepper));
        return false;
    }

    stepper->timestep = timestep;
    stepper->remainder = 0.0f;
    return true;
}

uint32_t sl_world_advance(sl_world *world, sl_stepper *stepper,
                          float frame_time)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(stepper != NULL);
    SL_ASSERT(sl_is_finite(stepper->timestep));
    SL_ASSERT(stepper->timestep > 0.0f);

    float available = stepper->remainder;
    if (sl_is_finite(frame_time) && frame_time > 0.0f) {
        available += frame_time;
    }

    uint32_t steps = 0u;
    while (available >= stepper->timestep && steps < SL_STEP_COUNT_MAX) {
        sl_world_step(world, stepper->timestep);
        available -= stepper->timestep;
        steps++;
    }

    /* At the cap, drop whatever is left: a slow consumer sheds debt
     * instead of stepping forever on the next frame. */
    stepper->remainder = (steps == SL_STEP_COUNT_MAX) ? 0.0f : available;
    return steps;
}
