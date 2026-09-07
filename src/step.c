#include "silk/step.h"

#include <string.h>

#include "contact_world.h"
#include "joint.h"
#include "solver.h"

/* Scale-safe Euclidean cap. Dividing by the largest component before the
 * length avoids overflow for any finite externally supplied velocity. */
static sl_vec2 linear_velocity_cap(sl_vec2 velocity, float speed_max)
{
    const float component_max = sl_max(sl_abs(velocity.x), sl_abs(velocity.y));
    if (component_max == 0.0f) {
        return velocity;
    }
    const sl_vec2 scaled = sl_vec2_scale(velocity, 1.0f / component_max);
    const float scaled_length = sl_vec2_length(scaled);
    if (component_max <= speed_max / scaled_length) {
        return velocity;
    }
    const float factor = (speed_max / component_max) / scaled_length;
    return sl_vec2_scale(velocity, factor);
}

static float angular_velocity_cap(float velocity, float speed_max)
{
    return sl_clamp(velocity, -speed_max, speed_max);
}

/* Invalid arithmetic retains the previous finite velocity. Valid world state
 * makes this exceptional (an extreme dt is the usual cause), and the speed
 * policy still caps the retained state before it feeds integration. */
static float velocity_integrate(float velocity, float acceleration, float h,
                                float drag_scale)
{
    const float candidate = (velocity + acceleration * h) * drag_scale;
    return sl_is_finite(candidate) ? candidate : velocity;
}

/* Advances one accumulated linear delta. A held dynamic axis drops to zero
 * velocity so outward acceleration cannot bank indefinitely. Kinematic
 * velocity remains controller-owned while its delta stays at the last valid
 * value. */
static void position_delta_integrate(float base_position, float velocity,
                                     float h, bool dynamic, float *delta,
                                     float *velocity_stored)
{
    const float candidate_delta = *delta + velocity * h;
    const float candidate_position = base_position + candidate_delta;
    if (sl_is_finite(candidate_delta) &&
        sl_abs(candidate_position) <= SL_POSITION_ABS_MAX) {
        *delta = candidate_delta;
    } else if (dynamic) {
        *velocity_stored = 0.0f;
    }
}

void sl_world_step(sl_world *world, float dt)
{
    SL_ASSERT(world != NULL);
    if (world == NULL) {
        return;
    }
    const bool world_valid =
        world->substep_count >= 1u &&
        world->substep_count <= SL_SUBSTEP_COUNT_MAX &&
        sl_is_finite(world->linear_speed_max) &&
        world->linear_speed_max > 0.0f && sl_is_finite(world->contact_hertz) &&
        world->contact_hertz > 0.0f &&
        sl_is_finite(world->contact_damping_ratio) &&
        world->contact_damping_ratio > 0.0f &&
        sl_is_finite(world->contact_push_velocity_max) &&
        world->contact_push_velocity_max > 0.0f &&
        sl_is_finite(world->restitution_threshold) &&
        world->restitution_threshold > 0.0f &&
        sl_is_finite(world->joint_hertz) && world->joint_hertz > 0.0f &&
        sl_is_finite(world->joint_damping_ratio) &&
        world->joint_damping_ratio > 0.0f;
    SL_ASSERT(world_valid);
    if (!world_valid) {
        return;
    }
    const bool dt_valid = sl_is_finite(dt) && dt > 0.0f;
    SL_ASSERT(dt_valid);
    if (!dt_valid) {
        return;
    }

    const float h = dt / (float)world->substep_count;
    const float inverse_dt = 1.0f / dt;
    const float inverse_h = 1.0f / h;
    const bool derived_valid = sl_is_finite(h) && h > 0.0f &&
                               sl_is_finite(inverse_dt) && inverse_dt > 0.0f &&
                               sl_is_finite(inverse_h) && inverse_h > 0.0f;
    SL_ASSERT(derived_valid);
    if (!derived_valid) {
        return;
    }

    /* Each divisor is >= 1 for an initialized world. Overflow produces a
     * scale of exact zero, still a stable and finite drag result. */
    const float linear_drag_scale = 1.0f / (1.0f + h * world->linear_drag);
    const float angular_drag_scale = 1.0f / (1.0f + h * world->angular_drag);
    const float angular_speed_max = SL_ROTATION_PER_STEP_MAX * inverse_dt;

    memset(&world->step_work, 0, sizeof(world->step_work));
    world->stats_stepping = true;
    sl_contact_step_begin(world);
    sl_joint_prepare(world, h, inverse_h);
    sl_solver_prepare(world, h, inverse_h);

    /* Deltas retain precision while the base transforms remain fixed for all
     * substeps and, later, for every solver iteration. */
    for (uint32_t i = 0u; i < world->body_count; ++i) {
        world->delta_positions[i] = sl_vec2_make(0.0f, 0.0f);
        world->delta_rotations[i] = sl_rotation_identity();
    }

    /* One constraint sweep at each smaller step follows the error-reduction
     * result of Macklin et al., "Small Steps in Physics Simulation" (SCA
     * 2019). Silk reuses one frame manifold/Jacobian preparation so the
     * substeps add bounded solver work without repeating broad-phase work. */
    for (uint32_t substep = 0u; substep < world->substep_count; ++substep) {
        for (uint32_t i = 0u; i < world->body_count; ++i) {
            const uint8_t type = world->types[i];
            const bool dynamic = type == (uint8_t)SL_BODY_DYNAMIC;
            const bool moving = dynamic || type == (uint8_t)SL_BODY_KINEMATIC;
            SL_ASSERT(dynamic || world->inv_masses[i] == 0.0f);
            SL_ASSERT(dynamic || world->inv_inertias[i] == 0.0f);
            SL_ASSERT(moving || (world->velocities[i].x == 0.0f &&
                                 world->velocities[i].y == 0.0f &&
                                 world->angular_velocities[i] == 0.0f));
            if (!moving) {
                continue;
            }

            sl_vec2 velocity = world->velocities[i];
            float angular_velocity = world->angular_velocities[i];
            if (dynamic) {
                const sl_vec2 acceleration = sl_vec2_add(
                    world->gravity,
                    sl_vec2_scale(world->forces[i], world->inv_masses[i]));
                const float angular_acceleration =
                    world->torques[i] * world->inv_inertias[i];
                velocity.x = velocity_integrate(velocity.x, acceleration.x, h,
                                                linear_drag_scale);
                velocity.y = velocity_integrate(velocity.y, acceleration.y, h,
                                                linear_drag_scale);
                angular_velocity =
                    velocity_integrate(angular_velocity, angular_acceleration,
                                       h, angular_drag_scale);
            }
            world->velocities[i] =
                linear_velocity_cap(velocity, world->linear_speed_max);
            world->angular_velocities[i] =
                angular_velocity_cap(angular_velocity, angular_speed_max);
        }

        sl_joint_warm_start(world);
        sl_solver_warm_start(world);
        sl_joint_solve(world, true);
        sl_solver_solve(world, inverse_h, true);

        for (uint32_t i = 0u; i < world->body_count; ++i) {
            const uint8_t type = world->types[i];
            const bool dynamic = type == (uint8_t)SL_BODY_DYNAMIC;
            const bool moving = dynamic || type == (uint8_t)SL_BODY_KINEMATIC;
            if (!moving) {
                continue;
            }
            position_delta_integrate(
                world->positions[i].x, world->velocities[i].x, h, dynamic,
                &world->delta_positions[i].x, &world->velocities[i].x);
            position_delta_integrate(
                world->positions[i].y, world->velocities[i].y, h, dynamic,
                &world->delta_positions[i].y, &world->velocities[i].y);
            world->delta_rotations[i] = sl_rotation_integrate(
                world->delta_rotations[i], world->angular_velocities[i] * h);
        }

        sl_joint_solve(world, false);
        sl_solver_solve(world, inverse_h, false);
    }

    sl_solver_restitution(world);
    sl_joint_store(world);
    sl_solver_store(world);

    sl_world_step_stats completed = { 0 };
    for (uint32_t i = 0u; i < world->body_count; ++i) {
        completed.dynamic_body_count +=
            world->types[i] == (uint8_t)SL_BODY_DYNAMIC ? 1u : 0u;
        completed.kinematic_body_count +=
            world->types[i] == (uint8_t)SL_BODY_KINEMATIC ? 1u : 0u;
        world->positions[i] =
            sl_vec2_add(world->positions[i], world->delta_positions[i]);
        world->rotations[i] = sl_rotation_normalize(
            sl_rotation_mul(world->delta_rotations[i], world->rotations[i]));
        world->delta_positions[i] = sl_vec2_make(0.0f, 0.0f);
        world->delta_rotations[i] = sl_rotation_identity();
        world->forces[i] = sl_vec2_make(0.0f, 0.0f);
        world->torques[i] = 0.0f;
    }
    sl_contact_step_end(world);
    world->stats_stepping = false;
    completed.work = world->step_work;
    completed.substep_count = world->substep_count;
    completed.contact_constraint_count = world->contact_constraint_count;
    completed.joint_constraint_count = world->joint_constraint_count;
    world->stats.step = completed;
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
