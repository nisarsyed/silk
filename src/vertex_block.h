#ifndef SILK_VERTEX_BLOCK_H
#define SILK_VERTEX_BLOCK_H

#include "energy.h"
#include "world_internal.h"

/* The VBD vertex block solve, shared by the contact-free sweep (solver.c)
 * and the self-contact sweep (solver_self.c). It lives in a header so each
 * sweep has a single call site in its own translation unit and the solve
 * is inlined into both loops: with two call sites in one file, or one call
 * site holding an untaken branch to the self-contact terms, the compiler
 * stopped inlining it and the contact-free path ran 27% slower (5.0 vs
 * 6.4 ms/step on a 32 x 32 curtain). `self` is a constant per caller. */

static inline sl_vec3 sl_position_clamp(sl_vec3 p)
{
    const float m = SL_POSITION_ABS_MAX;
    return sl_vec3_make(sl_clamp(p.x, -m, m), sl_clamp(p.y, -m, m),
                        sl_clamp(p.z, -m, m));
}

/* One VBD block update of vertex i: a Newton step on the incremental
 * potential restricted to x_i. Returns the new position (x_i itself when
 * the block is singular); the caller stores it. Writes only s->contact[i],
 * so blocks of one color may run concurrently. */
static inline sl_vec3 sl_vertex_block(sl_world_state *s,
                                      const sl_cloth_record *rec, uint32_t i,
                                      float h, bool self)
{
    const float inertia = s->mass[i] / (h * h);
    const sl_vec3 x = s->position[i];
    sl_vec3 force =
        sl_vec3_scale(sl_vec3_sub(s->inertia_target[i], x), inertia);
    sl_mat3 hessian = sl_mat3_diagonal(inertia);

    for (uint32_t r = s->vertex_triangle_offset[i];
         r < s->vertex_triangle_offset[i + 1u]; r++) {
        const uint32_t t = s->vertex_triangle[r] / 3u;
        const uint32_t corner = s->vertex_triangle[r] % 3u;
        const uint32_t *idx = s->triangle_index + 3u * t;
        const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                                s->position[idx[2]] };
        sl_vec3 gradient;
        sl_mat3 block;
        sl_membrane_vertex(xs, s->triangle_rest + 4u * t, s->triangle_area[t],
                           rec->membrane_mu, rec->membrane_lambda, corner,
                           &gradient, &block);
        force = sl_vec3_sub(force, gradient);
        hessian = sl_mat3_add(hessian, block);
    }

    if (rec->bending_stiffness > 0.0f) {
        for (uint32_t r = s->vertex_bend_offset[i];
             r < s->vertex_bend_offset[i + 1u]; r++) {
            const uint32_t b = s->vertex_bend[r] / 4u;
            const uint32_t slot = s->vertex_bend[r] % 4u;
            const uint32_t *idx = s->bend_index + 4u * b;
            const sl_vec3 xs[4] = { s->position[idx[0]], s->position[idx[1]],
                                    s->position[idx[2]], s->position[idx[3]] };
            sl_vec3 gradient;
            float block;
            sl_bend_vertex(xs, s->bend_weight + 4u * b, s->bend_scale[b],
                           rec->bending_stiffness, slot, &gradient, &block);
            force = sl_vec3_sub(force, gradient);
            hessian = sl_mat3_add(hessian, sl_mat3_diagonal(block));
        }
    }

    /* Contacts: penalty energy k / 2 * depth^2 along the collider normal.
     * Friction is applied at velocity level in sl_solver_substep_end. */
    for (uint32_t c = 0; c < s->collider_count; c++) {
        sl_vec3 n;
        const float depth =
            rec->thickness - sl_collider_distance(&s->colliders[c], x, &n);
        if (!(depth > 0.0f)) {
            continue;
        }
        const float k = s->collision_stiffness;
        force = sl_vec3_mul_add(force, n, k * depth);
        hessian = sl_mat3_mul_add(hessian, sl_mat3_outer(n, n), k);
        s->contact[i] = 1u;
    }

    if (self) {
        sl_self_vertex(s, i, &force, &hessian);
    }

    sl_vec3 step;
    if (sl_mat3_solve(hessian, force, 0.0f, &step)) {
        return sl_position_clamp(sl_vec3_add(x, step));
    }
    return x;
}

/* One color of one cloth, as a parallel task over its vertices. */
typedef struct sl_sweep_task {
    sl_world_state *state;
    const sl_cloth_record *rec;
    const uint32_t *order; /* the color's vertices, ascending */
    float h;
} sl_sweep_task;

#endif /* SILK_VERTEX_BLOCK_H */
