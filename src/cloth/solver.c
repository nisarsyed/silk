#include <math.h>

#include "energy.h"
#include "vertex_block.h"
#include "world_internal.h"

/* Per-vertex work of a substep, as parallel tasks over all vertices. Each
 * task writes only its own vertex. */
typedef struct sl_substep_task {
    sl_world_state *state;
    float h;
    float alpha;
} sl_substep_task;

/* Adds drag forces from the velocities at the start of the substep to the
 * inertia targets. Per triangle: f = -drag * A * |v_rel| * (v_rel . n) n
 * with v_rel the mean corner velocity minus wind, split evenly over the
 * corners. Each corner's velocity change h * f / (3 m) is limited to
 * |v_rel . n|, so explicit drag never reverses the relative normal motion.
 *
 * Serially this loops over triangles. With threads, sl_drag_vertices
 * gathers per vertex over its incident triangles in ascending triangle
 * order: each vertex receives exactly the same additions in the same
 * order, so the result is identical, at three force evaluations per
 * triangle instead of one (10% slower serially on a 64 x 64 curtain, so
 * the serial path keeps the loop over triangles). */
static void sl_solver_drag(sl_world_state *s, const sl_cloth_record *rec,
                           float h)
{
    for (uint32_t t = rec->triangle_offset;
         t < rec->triangle_offset + rec->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        const sl_vec3 x0 = s->position_start[idx[0]];
        const sl_vec3 normal_area = sl_vec3_scale(
            sl_vec3_cross(sl_vec3_sub(s->position_start[idx[1]], x0),
                          sl_vec3_sub(s->position_start[idx[2]], x0)),
            0.5f);
        const float area = sl_vec3_length(normal_area);
        if (!(area >= SL_TRIANGLE_AREA_MIN)) {
            continue;
        }
        const sl_vec3 n = sl_vec3_scale(normal_area, 1.0f / area);
        sl_vec3 v = sl_vec3_add(s->velocity[idx[0]], s->velocity[idx[1]]);
        v = sl_vec3_scale(sl_vec3_add(v, s->velocity[idx[2]]), 1.0f / 3.0f);
        const sl_vec3 rel = sl_vec3_sub(v, s->wind);
        const float rel_n = sl_vec3_dot(rel, n);
        const float force =
            rec->drag * area * sl_vec3_length(rel) * rel_n / 3.0f;
        for (uint32_t k = 0; k < 3u; k++) {
            const uint32_t i = idx[k];
            if (s->pinned[i] != 0u) {
                continue;
            }
            /* Velocity change along -n, limited to |rel_n|. */
            float dv = h * force / s->mass[i];
            if (sl_abs(dv) > sl_abs(rel_n)) {
                dv = rel_n;
            }
            s->inertia_target[i] =
                sl_vec3_mul_add(s->inertia_target[i], n, -h * dv);
        }
    }
}

static void sl_drag_vertices(uint32_t begin, uint32_t end, void *context)
{
    const sl_substep_task *task = context;
    sl_world_state *s = task->state;
    const float h = task->h;
    for (uint32_t i = begin; i < end; i++) {
        const sl_cloth_record *rec = &s->cloths[s->vertex_cloth[i]];
        if (!(rec->drag > 0.0f) || s->pinned[i] != 0u) {
            continue;
        }
        sl_vec3 target = s->inertia_target[i];
        for (uint32_t r = s->vertex_triangle_offset[i];
             r < s->vertex_triangle_offset[i + 1u]; r++) {
            const uint32_t *idx =
                s->triangle_index + 3u * (s->vertex_triangle[r] / 3u);
            const sl_vec3 x0 = s->position_start[idx[0]];
            const sl_vec3 normal_area = sl_vec3_scale(
                sl_vec3_cross(sl_vec3_sub(s->position_start[idx[1]], x0),
                              sl_vec3_sub(s->position_start[idx[2]], x0)),
                0.5f);
            const float area = sl_vec3_length(normal_area);
            if (!(area >= SL_TRIANGLE_AREA_MIN)) {
                continue;
            }
            const sl_vec3 n = sl_vec3_scale(normal_area, 1.0f / area);
            sl_vec3 v = sl_vec3_add(s->velocity[idx[0]], s->velocity[idx[1]]);
            v = sl_vec3_scale(sl_vec3_add(v, s->velocity[idx[2]]), 1.0f / 3.0f);
            const sl_vec3 rel = sl_vec3_sub(v, s->wind);
            const float rel_n = sl_vec3_dot(rel, n);
            const float force =
                rec->drag * area * sl_vec3_length(rel) * rel_n / 3.0f;
            /* Velocity change along -n, limited to |rel_n|. */
            float dv = h * force / s->mass[i];
            if (sl_abs(dv) > sl_abs(rel_n)) {
                dv = rel_n;
            }
            target = sl_vec3_mul_add(target, n, -h * dv);
        }
        s->inertia_target[i] = target;
    }
}

uint32_t sl_solver_substep_count(const sl_world_state *s, float dt,
                                 bool *limited)
{
    SL_ASSERT(dt > 0.0f);
    SL_ASSERT(limited != NULL);
    /* Smallest n with (dt / n)^2 * sigma_max <= SL_STIFFNESS_STEP_MAX. */
    const double need = ceil((double)dt * sqrt((double)s->stiffness_rate_max /
                                               (double)SL_STIFFNESS_STEP_MAX));
    uint32_t count = s->substep_count;
    *limited = false;
    if (need > (double)SL_SUBSTEP_COUNT_MAX) {
        *limited = true;
        count = SL_SUBSTEP_COUNT_MAX;
    } else if (need > (double)count) {
        count = (uint32_t)need;
    }
    return count;
}

/* Keeps every vertex within the bounds of the last detection, which makes
 * the move from that state intersection-free without CCD. */
static void sl_solver_constrain(sl_world_state *s)
{
    if (sl_self_enabled(s)) {
        s->self_bound_hits += sl_self_constrain(s);
    }
}

static void sl_begin_vertices(uint32_t begin, uint32_t end, void *context)
{
    const sl_substep_task *task = context;
    sl_world_state *s = task->state;
    const float h = task->h;
    for (uint32_t i = begin; i < end; i++) {
        s->position_start[i] = s->position[i];
        if (s->pinned[i] != 0u) {
            s->position[i] =
                sl_vec3_lerp(s->pin_origin[i], s->pin_target[i], task->alpha);
            s->inertia_target[i] = s->position[i];
            continue;
        }
        const sl_vec3 inertial =
            sl_vec3_mul_add(s->position[i], s->velocity[i], h);
        s->inertia_target[i] = sl_vec3_mul_add(inertial, s->gravity, h * h);
    }
}

/* Initial guess y, exact for free flight and rigid sliding. Measured
 * alternatives are in docs/solver.md: VBD's per-vertex adaptive guess fed
 * measured accelerations back into the guess and amplified elastic
 * vibration into divergence on unconverged substeps; x + h v lagged far
 * behind gravity on stiff cloth; starting contacting vertices from x + h v
 * made resting contact flicker. */
static void sl_guess_vertices(uint32_t begin, uint32_t end, void *context)
{
    const sl_substep_task *task = context;
    sl_world_state *s = task->state;
    for (uint32_t i = begin; i < end; i++) {
        if (s->pinned[i] == 0u) {
            /* The guess is y. An attachment's spring k / 2 |x - a|^2 then
             * joins the inertia term w / 2 |x - y|^2, w = m / h^2, as one
             * quadratic: weight w + k (vertex_block.h) about
             * (w y + k a) / (w + k), up to a constant, so the block solve
             * has no attachment term. Starting from that blended target
             * instead of y left 16-iteration sweeps 5% off the equilibrium
             * of a sheet held by four springs, against 0.04%. */
            s->position[i] = s->inertia_target[i];
            const float k = s->attach_stiffness[i];
            if (k > 0.0f) {
                const sl_vec3 anchor = sl_vec3_lerp(
                    s->pin_origin[i], s->pin_target[i], task->alpha);
                const float w = s->mass[i] / (task->h * task->h);
                s->inertia_target[i] =
                    sl_vec3_lerp(s->inertia_target[i], anchor, k / (w + k));
            }
        }
        s->contact[i] = 0u;
    }
}

void sl_solver_substep_begin(sl_world_state *s, float h, float alpha)
{
    SL_ASSERT(h > 0.0f);
    SL_ASSERT(alpha >= 0.0f && alpha <= 1.0f);
    /* Contacts were detected at the step's start; refresh them once enough
     * vertices have used up their bounds (the substep start is
     * intersection-free). */
    sl_solver_redetect(s);
    sl_substep_task task = { s, h, alpha };
    sl_parallel(s, sl_begin_vertices, s->vertex_count, SL_PARALLEL_GRAIN_LOOP,
                &task);
    bool drag = false;
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        drag = drag || s->cloths[c].drag > 0.0f;
    }
    if (drag && s->parallel_for != NULL) {
        sl_parallel(s, sl_drag_vertices, s->vertex_count,
                    SL_PARALLEL_GRAIN_BLOCK, &task);
    } else if (drag) {
        for (uint32_t c = 0; c < s->cloth_count; c++) {
            if (s->cloths[c].drag > 0.0f) {
                sl_solver_drag(s, &s->cloths[c], h);
            }
        }
    }
    sl_parallel(s, sl_guess_vertices, s->vertex_count, SL_PARALLEL_GRAIN_LOOP,
                &task);
    /* OGC Eq. 28: the guess stays within the bounds. */
    sl_solver_constrain(s);
}

/* gamma_e of the paper: re-detect once 1% of the vertices are held at
 * their bounds. */
#define SL_REDETECT_FRACTION 0.01f

void sl_solver_redetect(sl_world_state *s)
{
    if (!sl_self_enabled(s)) {
        return;
    }
    const float threshold =
        sl_max(1.0f, SL_REDETECT_FRACTION * (float)s->vertex_count);
    if ((float)s->bound_exceeded >= threshold) {
        sl_self_detect(s, s->self_motion);
    }
}

static void sl_solver_sweep_blocks(sl_world_state *s, float h);

void sl_solver_sweep(sl_world_state *s, float h)
{
    SL_ASSERT(h > 0.0f);
    sl_solver_sweep_blocks(s, h);
    sl_solver_constrain(s);
}

/* Vertices of one color share no triangle or bending stencil, so without
 * self contact a block reads no vertex its color writes: solving them
 * concurrently, in place, gives exactly the serial result. */
static void sl_sweep_color(uint32_t begin, uint32_t end, void *context)
{
    const sl_sweep_task *task = context;
    sl_world_state *s = task->state;
    for (uint32_t k = begin; k < end; k++) {
        const uint32_t i = task->order[k];
        if (s->pinned[i] == 0u) {
            s->position[i] = sl_vertex_block(s, task->rec, i, task->h, false);
        }
    }
}

static void sl_solver_sweep_blocks(sl_world_state *s, float h)
{
    SL_ASSERT(h > 0.0f);
    if (sl_self_enabled(s)) {
        sl_solver_sweep_self(s, h);
        return;
    }
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        const sl_cloth_record *rec = &s->cloths[c];
        for (uint32_t color = 0; color < rec->color_count; color++) {
            const uint32_t begin =
                rec->vertex_offset + rec->color_offsets[color];
            const uint32_t end =
                rec->vertex_offset + rec->color_offsets[color + 1u];
            sl_sweep_task task = { s, rec, s->color_order + begin, h };
            sl_parallel(s, sl_sweep_color, end - begin, SL_PARALLEL_GRAIN_BLOCK,
                        &task);
        }
    }
}

/* Removes the fraction 1 - exp(-damping h) of every edge's relative
 * velocity along the edge, as equal and opposite mass-weighted impulses
 * (a pinned end has infinite mass). Rigid translation and rotation have no
 * relative velocity along any edge and are untouched; linear momentum is
 * conserved and kinetic energy can only fall, so the damping is stable
 * whether or not the sweeps converged. Implicit Rayleigh damping inside the
 * vertex blocks was measured to do the opposite: it stiffens the blocks,
 * slows convergence, and unconverged substeps then gained energy.
 *
 * Serial, in ascending edge order, on purpose. Consecutive edges share
 * vertices, so one pass carries the damping along chains across the whole
 * sheet and suppresses long waves. Edge coloring would make the pass
 * parallel, but in color order a pass reaches only a few edges: the 1.2 m
 * stiff drape (tests/test_solver.c, sphere_drape_outside) then gained
 * kinetic energy from unconverged substeps faster than damping removed it,
 * its hanging skirt flapped at up to 24 m/s instead of settling under
 * 1.5 m/s, and the test failed on x86-64. Every impulse still removed
 * energy; there was simply far less damping per pass. */
static void sl_solver_damp(sl_world_state *s, const sl_cloth_record *rec,
                           float h)
{
    const float fraction = 1.0f - expf(-rec->damping * h);
    for (uint32_t e = rec->edge_offset; e < rec->edge_offset + rec->edge_count;
         e++) {
        const uint32_t a = s->edge_index[2u * e];
        const uint32_t b = s->edge_index[2u * e + 1u];
        const float wa = (s->pinned[a] != 0u) ? 0.0f : 1.0f / s->mass[a];
        const float wb = (s->pinned[b] != 0u) ? 0.0f : 1.0f / s->mass[b];
        if (!(wa + wb > 0.0f)) {
            continue;
        }
        const sl_vec3 n =
            sl_vec3_normalize(sl_vec3_sub(s->position[b], s->position[a]));
        const float rate =
            sl_vec3_dot(sl_vec3_sub(s->velocity[b], s->velocity[a]), n);
        const float impulse = fraction * rate / (wa + wb);
        s->velocity[a] = sl_vec3_mul_add(s->velocity[a], n, impulse * wa);
        s->velocity[b] = sl_vec3_mul_add(s->velocity[b], n, -impulse * wb);
    }
}

/* Coulomb friction on the substep velocity of a contacting vertex. Each
 * collider it penetrates presses with the penalty force lambda = k * depth;
 * the tangential velocity shrinks by at most mu lambda h / m and stops when
 * it is slower (static friction). Done per vertex against its own load,
 * this is exact in one pass and only removes kinetic energy. Two measured
 * alternatives failed (docs/solver.md): friction inside the Gauss-Seidel
 * blocks could not hold cloth, because every neighbour starts from y
 * already slipped; re-deriving positions from the corrected velocity
 * stopped creep but pumped energy into draped cloth. Positions are left as
 * solved, so held cloth creeps by the tangential part of h^2 g per
 * substep: at most h |g| sin(slope), about 3.5 mm/s on a 20 degree slope
 * with the default 16 substeps at 60 Hz. */
static void sl_solver_friction(sl_world_state *s, const sl_cloth_record *rec,
                               uint32_t i, float h)
{
    sl_vec3 v = s->velocity[i];
    bool changed = false;
    for (uint32_t c = 0; c < s->collider_count; c++) {
        const sl_collider *collider = &s->colliders[c];
        sl_vec3 n;
        const float depth =
            rec->thickness - sl_collider_distance(collider, s->position[i], &n);
        const float mu = sqrtf(rec->friction * collider->friction);
        if (!(depth > 0.0f) || !(mu > 0.0f)) {
            continue;
        }
        const sl_vec3 tangential = sl_vec3_mul_add(v, n, -sl_vec3_dot(n, v));
        const float speed = sl_vec3_length(tangential);
        if (!(speed > 0.0f)) {
            continue;
        }
        const float reach =
            mu * s->collision_stiffness * depth * h / s->mass[i];
        const float cut = (speed <= reach) ? 1.0f : reach / speed;
        v = sl_vec3_mul_add(v, tangential, -cut);
        changed = true;
    }
    if (changed) {
        s->velocity[i] = v;
    }
}

/* Velocity from the substep's displacement, then collider friction. */
static void sl_end_vertices(uint32_t begin, uint32_t end, void *context)
{
    const sl_substep_task *task = context;
    sl_world_state *s = task->state;
    const float inv_h = 1.0f / task->h;
    for (uint32_t i = begin; i < end; i++) {
        s->velocity[i] = sl_vec3_scale(
            sl_vec3_sub(s->position[i], s->position_start[i]), inv_h);
        const sl_cloth_record *rec = &s->cloths[s->vertex_cloth[i]];
        if (rec->friction > 0.0f && s->contact[i] != 0u && s->pinned[i] == 0u) {
            sl_solver_friction(s, rec, i, task->h);
        }
    }
}

static void sl_clamp_speeds(uint32_t begin, uint32_t end, void *context)
{
    const sl_substep_task *task = context;
    sl_world_state *s = task->state;
    for (uint32_t i = begin; i < end; i++) {
        const float speed = sl_vec3_length(s->velocity[i]);
        if (speed > s->speed_max) {
            s->velocity[i] =
                sl_vec3_scale(s->velocity[i], s->speed_max / speed);
        }
    }
}

void sl_solver_substep_end(sl_world_state *s, float h)
{
    SL_ASSERT(h > 0.0f);
    sl_substep_task task = { s, h, 1.0f };
    sl_parallel(s, sl_end_vertices, s->vertex_count, SL_PARALLEL_GRAIN_LOOP,
                &task);
    uint32_t contacts = 0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        contacts += s->contact[i];
    }
    if (sl_self_enabled(s)) {
        sl_self_friction(s, h);
    }
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        if (s->cloths[c].damping > 0.0f) {
            sl_solver_damp(s, &s->cloths[c], h);
        }
    }
    sl_parallel(s, sl_clamp_speeds, s->vertex_count, SL_PARALLEL_GRAIN_LOOP,
                &task);
    s->contact_count = contacts;
}

double sl_solver_energy(const sl_world_state *s, float h)
{
    double energy = 0.0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->pinned[i] != 0u) {
            continue;
        }
        /* With an attachment this includes its spring (sl_guess_vertices),
         * up to a constant. */
        const sl_vec3 d = sl_vec3_sub(s->position[i], s->inertia_target[i]);
        const float weight = s->mass[i] / (h * h) + s->attach_stiffness[i];
        energy += 0.5 * (double)weight * (double)sl_vec3_length_sq(d);
    }
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        const sl_cloth_record *rec = &s->cloths[c];
        for (uint32_t t = rec->triangle_offset;
             t < rec->triangle_offset + rec->triangle_count; t++) {
            const uint32_t *idx = s->triangle_index + 3u * t;
            const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                                    s->position[idx[2]] };
            energy += (double)sl_membrane_energy(
                xs, s->triangle_rest + 4u * t, s->triangle_area[t],
                rec->membrane_mu, rec->membrane_lambda);
        }
        for (uint32_t b = rec->bend_offset;
             b < rec->bend_offset + rec->bend_count; b++) {
            const uint32_t *idx = s->bend_index + 4u * b;
            const sl_vec3 xs[4] = { s->position[idx[0]], s->position[idx[1]],
                                    s->position[idx[2]], s->position[idx[3]] };
            energy += (double)sl_bend_energy(xs, s->bend_weight + 4u * b,
                                             s->bend_scale[b],
                                             rec->bending_stiffness);
        }
        for (uint32_t i = rec->vertex_offset;
             i < rec->vertex_offset + rec->vertex_count; i++) {
            if (s->pinned[i] != 0u) {
                continue;
            }
            for (uint32_t k = 0; k < s->collider_count; k++) {
                sl_vec3 n;
                const float depth =
                    rec->thickness -
                    sl_collider_distance(&s->colliders[k], s->position[i], &n);
                if (depth > 0.0f) {
                    energy += 0.5 * (double)s->collision_stiffness *
                              (double)depth * (double)depth;
                }
            }
        }
    }
    if (sl_self_enabled(s)) {
        energy += sl_self_energy(s);
    }
    return energy;
}
