#include "silk/step.h"

#include <string.h>

void sl_world_step(sl_world *world, float dt)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_is_finite(dt));
    SL_ASSERT(dt > 0.0f);

    /* (1 + dt * drag) >= 1 for every accepted world, so the scale is in
     * (0, 1] and can never reverse a velocity. */
    const float drag_scale = 1.0f / (1.0f + dt * world->linear_drag);

    for (uint32_t i = 0u; i < world->body_count; ++i) {
        const sl_vec2 acceleration =
            sl_vec2_add(world->gravity,
                        sl_vec2_scale(world->forces[i], world->inv_masses[i]));
        world->velocities[i] = sl_vec2_scale(
            sl_vec2_add(world->velocities[i], sl_vec2_scale(acceleration, dt)),
            drag_scale);
        world->positions[i] = sl_vec2_add(
            world->positions[i], sl_vec2_scale(world->velocities[i], dt));
        world->forces[i] = sl_vec2_make(0.0f, 0.0f);
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
