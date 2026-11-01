#include "silk/step.h"

#include <string.h>

#include "world_internal.h"

void sl_world_step(sl_world *world, float dt)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    SL_ASSERT(sl_is_finite(dt) && dt > 0.0f);
    sl_world_state *s = world->state;
    if (!sl_is_finite(dt) || !(dt > 0.0f)) {
        return;
    }
    bool limited = false;
    const uint32_t substeps = sl_solver_substep_count(s, dt, &limited);
    const float h = dt / (float)substeps;
    if (!(h > 0.0f)) {
        return;
    }
    s->substep_count_last = substeps;
    s->substep_limited = limited;
    s->self_detections = 0u;
    s->self_dropped = 0u;
    s->self_bound_hits = 0u;
    s->self_overflows = 0u;
    s->self_inline = 0u;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->pinned[i] != 0u) {
            s->pin_origin[i] = s->position[i];
        }
    }
    if (sl_self_enabled(s)) {
        /* One detection per step. The paper widens the query by the
         * inertial motion; with translation-relative bounds only motion
         * relative to the cloth's mean matters, dt |v - mean(v)|. Bounds
         * then keep every substep intersection-free and trigger
         * re-detection. */
        float spread = 0.0f;
        for (uint32_t c = 0; c < s->cloth_count; c++) {
            const sl_cloth_record *rec = &s->cloths[c];
            sl_vec3 mean = sl_vec3_zero();
            for (uint32_t i = rec->vertex_offset;
                 i < rec->vertex_offset + rec->vertex_count; i++) {
                mean = sl_vec3_add(mean, s->velocity[i]);
            }
            mean = sl_vec3_scale(mean, 1.0f / (float)rec->vertex_count);
            for (uint32_t i = rec->vertex_offset;
                 i < rec->vertex_offset + rec->vertex_count; i++) {
                spread = sl_max(
                    spread, sl_vec3_length(sl_vec3_sub(s->velocity[i], mean)));
            }
        }
        s->self_motion = dt * spread;
        sl_self_detect(s, s->self_motion);
    }
    for (uint32_t sub = 0; sub < substeps; sub++) {
        const float alpha = (float)(sub + 1u) / (float)substeps;
        sl_solver_substep_begin(s, h, alpha);
        for (uint32_t it = 0; it < s->iteration_count; it++) {
            sl_solver_sweep(s, h);
            if (it + 1u < s->iteration_count) {
                sl_solver_redetect(s);
            }
        }
        sl_solver_substep_end(s, h);
    }
    /* Anchors reached their targets; the next step starts from there. */
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->attach_stiffness[i] > 0.0f) {
            s->pin_origin[i] = s->pin_target[i];
        }
    }
}

bool sl_stepper_init(sl_stepper *stepper, float timestep)
{
    SL_ASSERT(stepper != NULL);
    memset(stepper, 0, sizeof(*stepper));
    if (!sl_is_finite(timestep) || !(timestep > 0.0f)) {
        return false;
    }
    stepper->timestep = timestep;
    return true;
}

uint32_t sl_world_advance(sl_world *world, sl_stepper *stepper,
                          float frame_time)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(stepper != NULL && stepper->timestep > 0.0f);
    if (!sl_is_finite(frame_time) || frame_time < 0.0f) {
        frame_time = 0.0f;
    }
    float time = stepper->remainder + frame_time;
    uint32_t steps = 0;
    while (time >= stepper->timestep && steps < SL_STEP_COUNT_MAX) {
        sl_world_step(world, stepper->timestep);
        time -= stepper->timestep;
        steps++;
    }
    if (time >= stepper->timestep) {
        time = 0.0f; /* drop debt beyond the cap */
    }
    stepper->remainder = time;
    return steps;
}
