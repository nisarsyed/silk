/* Head-to-head cloth solver comparison (research harness, not part of the
 * library). Every solver runs on the same silk world: mesh, rest data,
 * inertial prediction y (with drag), pins, colliders, velocity-level
 * friction, edge stretch damping and speed cap are shared, so only the
 * elastic solve differs:
 *
 *   vbd          silk itself: colored Gauss-Seidel vertex Newton blocks
 *                (Chen et al. 2024), StVK + quadratic bending, penalty
 *                contact.
 *   xpbd         Macklin et al. 2016 / small steps 2019: edge distance
 *                constraints with compliance 1 / k, k = mu + lambda (the
 *                spring stiffness whose energy matches StVK under uniform
 *                stretch on this grid), the same quadratic bending as a
 *                3-row constraint, colliders by projection.
 *   sipd-jacobi  Luo et al. 2026 (arXiv 2609.09834), Algorithm 1: Jacobi
 *                successive substitution with pairwise tension tensors
 *                L_ij = c_ij A P F^+, the analytic +/- split, and a global
 *                backtracking line search. Cloth triangles use the
 *                pseudo-inverse F^+ = (F^T F)^-1 F^T; for StVK
 *                P F^+ = mu F F^T + lambda/2 tr(C) Pi - (mu + lambda) Pi,
 *                Pi = F C^-1 F^T. Bending enters as scalar pair tensors.
 *   sipd-gs      the same update applied vertex by vertex in silk's color
 *                order (the paper's "Our-GS"), no line search.
 *
 * Substep counts are fixed per run (silk's automatic stability substeps are
 * bypassed) so that solvers are compared at equal cost.
 *
 * usage: sl_compare [--quick] [--no-sipd] [--cells N]
 * Scenes scale with the grid; edge length is 1.2 m / N for the drape and
 * 1 m / N for the others. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "energy.h"
#include "silk/mesh.h"
#include "silk/step.h"
#include "silk/world.h"
#include "world_internal.h"

#define SC_DT (1.0f / 60.0f)
#define SC_LINE_SEARCH_MAX 6u

typedef enum sc_solver {
    SC_VBD = 0,
    SC_XPBD = 1,
    SC_SIPD_JACOBI = 2,
    SC_SIPD_GS = 3,
    SC_SOLVER_COUNT = 4
} sc_solver;

static const char *const k_solver_names[SC_SOLVER_COUNT] = { "vbd", "xpbd",
                                                             "sipd-jacobi",
                                                             "sipd-gs" };

typedef struct sc_sim {
    sl_world world;
    sc_solver solver;
    uint32_t substeps;
    uint32_t iterations;
    float spring_k;     /* XPBD edge stiffness, N/m */
    float *edge_rest;   /* XPBD rest lengths */
    float *lambda;      /* XPBD multipliers: edges, then 3 per stencil */
    float *corrected;   /* XPBD normal projection per substep, m */
    float *pair_weight; /* SIPD c_ij: 6 per triangle, (corner, other) */
    sl_vec3 *gradient;  /* SIPD Jacobi scratch */
    sl_mat3 *block;     /* SIPD Jacobi scratch */
    sl_vec3 *base;      /* SIPD Jacobi scratch */
} sc_sim;

/* ---------------------------------------------------------------------- */
/* Scene construction                                                      */

static sl_cloth_desc sc_material(float drag)
{
    sl_cloth_desc d;
    memset(&d, 0, sizeof(d));
    d.density = 0.2f;
    d.membrane_mu = 2.0e3f;
    d.membrane_lambda = 2.0e3f;
    d.bending_stiffness = 1.0e-4f;
    d.damping = 20.0f;
    d.thickness = 0.005f;
    d.friction = 0.5f;
    d.drag = drag;
    return d;
}

static bool sc_alloc(sc_sim *sim)
{
    const sl_world_state *s = sim->world.state;
    sim->edge_rest = calloc(s->edge_count + 1u, sizeof(float));
    sim->lambda =
        calloc(s->edge_count + 3u * s->bend_count + 1u, sizeof(float));
    sim->corrected = calloc(s->vertex_count, sizeof(float));
    sim->pair_weight = calloc(6u * s->triangle_count, sizeof(float));
    sim->gradient = calloc(s->vertex_count, sizeof(sl_vec3));
    sim->block = calloc(s->vertex_count, sizeof(sl_mat3));
    sim->base = calloc(s->vertex_count, sizeof(sl_vec3));
    return sim->edge_rest != NULL && sim->lambda != NULL &&
           sim->corrected != NULL && sim->pair_weight != NULL &&
           sim->gradient != NULL && sim->block != NULL && sim->base != NULL;
}

/* SIPD pair weights: for corner i with other corners j1, j2, solve
 * [xi_ij1 xi_ij2] c = -g_i in the rest plane, so that
 * sum_j c_ij A P F^+ (y_j - y_i) = -A P g_i, the FEM force. */
static void sc_pair_weights(sc_sim *sim)
{
    const sl_world_state *s = sim->world.state;
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const float *r = s->triangle_rest + 4u * t;
        /* Dm = rest^-1: columns are the rest edges X1 - X0 and X2 - X0. */
        const float det = r[0] * r[3] - r[1] * r[2];
        const float dm00 = r[3] / det;
        const float dm01 = -r[1] / det;
        const float dm10 = -r[2] / det;
        const float dm11 = r[0] / det;
        const float px[3] = { 0.0f, dm00, dm01 };
        const float py[3] = { 0.0f, dm10, dm11 };
        const float gx[3] = { -(r[0] + r[2]), r[0], r[2] };
        const float gy[3] = { -(r[1] + r[3]), r[1], r[3] };
        for (uint32_t i = 0; i < 3u; i++) {
            const uint32_t j1 = (i + 1u) % 3u;
            const uint32_t j2 = (i + 2u) % 3u;
            const float ax = px[j1] - px[i];
            const float ay = py[j1] - py[i];
            const float bx = px[j2] - px[i];
            const float by = py[j2] - py[i];
            const float d = ax * by - bx * ay;
            /* c1 a + c2 b = -g */
            const float c1 = (-gx[i] * by + gy[i] * bx) / d;
            const float c2 = (-ax * gy[i] + ay * gx[i]) / d;
            sim->pair_weight[6u * t + 2u * i] = c1;
            sim->pair_weight[6u * t + 2u * i + 1u] = c2;
        }
    }
}

static bool sc_build(sc_sim *sim, sc_solver solver, uint32_t substeps,
                     uint32_t iterations, sl_vec3 origin, sl_vec3 u, sl_vec3 v,
                     uint32_t cells, float drag)
{
    memset(sim, 0, sizeof(*sim));
    sim->solver = solver;
    sim->substeps = substeps;
    sim->iterations = iterations;
    const uint32_t n = sl_mesh_grid_vertex_count(cells, cells);
    const uint32_t tn = sl_mesh_grid_triangle_count(cells, cells);
    sl_vec3 *positions = malloc(sizeof(sl_vec3) * n);
    uint32_t *indices = malloc(sizeof(uint32_t) * 3u * tn);
    sl_world_config c;
    memset(&c, 0, sizeof(c));
    c.vertex_capacity = n;
    c.triangle_capacity = tn;
    c.cloth_capacity = 1u;
    c.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    c.substep_count = substeps;
    c.iteration_count = iterations;
    sl_cloth_desc d = sc_material(drag);
    d.positions = positions;
    d.vertex_count = n;
    d.indices = indices;
    d.triangle_count = tn;
    uint32_t cloth = 0;
    bool ok = positions != NULL && indices != NULL &&
              sl_mesh_grid(origin, u, v, cells, cells, positions, n, indices,
                           3u * tn) &&
              sl_world_init(&sim->world, &c) &&
              sl_world_cloth_add(&sim->world, &d, &cloth);
    free(positions);
    free(indices);
    if (!ok || !sc_alloc(sim)) {
        return false;
    }
    sl_world_state *s = sim->world.state;
    s->stiffness_rate_max = 0.0f; /* fixed substeps: equal-cost runs */
    sim->spring_k = d.membrane_mu + d.membrane_lambda;
    for (uint32_t e = 0; e < s->edge_count; e++) {
        sim->edge_rest[e] = sl_vec3_length(
            sl_vec3_sub(s->position[s->edge_index[2u * e]],
                        s->position[s->edge_index[2u * e + 1u]]));
    }
    sc_pair_weights(sim);
    return true;
}

static void sc_free(sc_sim *sim)
{
    sl_world_destroy(&sim->world);
    free(sim->edge_rest);
    free(sim->lambda);
    free(sim->corrected);
    free(sim->pair_weight);
    free(sim->gradient);
    free(sim->block);
    free(sim->base);
    memset(sim, 0, sizeof(*sim));
}

/* ---------------------------------------------------------------------- */
/* XPBD                                                                    */

static float sc_inv_mass(const sl_world_state *s, uint32_t i)
{
    return (s->pinned[i] != 0u) ? 0.0f : 1.0f / s->mass[i];
}

static void sc_xpbd_sweep(sc_sim *sim, float h)
{
    sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    const float alpha_edge = 1.0f / (sim->spring_k * h * h);
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const uint32_t a = s->edge_index[2u * e];
        const uint32_t b = s->edge_index[2u * e + 1u];
        const float wa = sc_inv_mass(s, a);
        const float wb = sc_inv_mass(s, b);
        const sl_vec3 d = sl_vec3_sub(s->position[a], s->position[b]);
        const float length = sl_vec3_length(d);
        if (!(wa + wb > 0.0f) || !(length > 0.0f)) {
            continue;
        }
        const sl_vec3 n = sl_vec3_scale(d, 1.0f / length);
        const float constraint = length - sim->edge_rest[e];
        const float dl = (-constraint - alpha_edge * sim->lambda[e]) /
                         (wa + wb + alpha_edge);
        sim->lambda[e] += dl;
        s->position[a] = sl_vec3_mul_add(s->position[a], n, wa * dl);
        s->position[b] = sl_vec3_mul_add(s->position[b], n, -wb * dl);
    }
    if (rec->bending_stiffness > 0.0f) {
        for (uint32_t b = 0; b < s->bend_count; b++) {
            const uint32_t *idx = s->bend_index + 4u * b;
            const float *w = s->bend_weight + 4u * b;
            const float alpha =
                1.0f / (rec->bending_stiffness * s->bend_scale[b] * h * h);
            float denom = alpha;
            sl_vec3 c = sl_vec3_zero();
            for (uint32_t k = 0; k < 4u; k++) {
                denom += w[k] * w[k] * sc_inv_mass(s, idx[k]);
                c = sl_vec3_mul_add(
                    c, sl_vec3_sub(s->position[idx[k]], s->position[idx[0]]),
                    w[k]);
            }
            float *lam = sim->lambda + s->edge_count + 3u * b;
            const sl_vec3 dl = sl_vec3_make((-c.x - alpha * lam[0]) / denom,
                                            (-c.y - alpha * lam[1]) / denom,
                                            (-c.z - alpha * lam[2]) / denom);
            lam[0] += dl.x;
            lam[1] += dl.y;
            lam[2] += dl.z;
            for (uint32_t k = 0; k < 4u; k++) {
                s->position[idx[k]] = sl_vec3_mul_add(
                    s->position[idx[k]], dl, w[k] * sc_inv_mass(s, idx[k]));
            }
        }
    }
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->pinned[i] != 0u) {
            continue;
        }
        for (uint32_t c = 0; c < s->collider_count; c++) {
            sl_vec3 n;
            const float depth =
                rec->thickness -
                sl_collider_distance(&s->colliders[c], s->position[i], &n);
            if (depth > 0.0f) {
                s->position[i] = sl_vec3_mul_add(s->position[i], n, depth);
                sim->corrected[i] += depth;
                s->contact[i] = 1u;
            }
        }
    }
}

/* ---------------------------------------------------------------------- */
/* SIPD                                                                    */

/* P F^+ for StVK split into its positive and negative semidefinite parts. */
static void sc_sipd_tension(const sl_vec3 xs[3], const float rest[4], float mu,
                            float lambda, sl_mat3 *positive, sl_mat3 *negative)
{
    const sl_vec3 e1 = sl_vec3_sub(xs[1], xs[0]);
    const sl_vec3 e2 = sl_vec3_sub(xs[2], xs[0]);
    const sl_vec3 f0 = sl_vec3_mul_add(sl_vec3_scale(e1, rest[0]), e2, rest[2]);
    const sl_vec3 f1 = sl_vec3_mul_add(sl_vec3_scale(e1, rest[1]), e2, rest[3]);
    const float c00 = sl_vec3_dot(f0, f0);
    const float c01 = sl_vec3_dot(f0, f1);
    const float c11 = sl_vec3_dot(f1, f1);
    const float det = c00 * c11 - c01 * c01;
    const sl_mat3 ff =
        sl_mat3_add(sl_mat3_outer(f0, f0), sl_mat3_outer(f1, f1));
    sl_mat3 pi = sl_mat3_zero();
    if (det > 1e-12f) {
        /* Pi = F C^-1 F^T with C^-1 = [c11 -c01; -c01 c00] / det. */
        pi = sl_mat3_mul_add(pi, sl_mat3_outer(f0, f0), c11 / det);
        pi = sl_mat3_mul_add(pi, sl_mat3_outer(f1, f1), c00 / det);
        pi = sl_mat3_mul_add(pi, sl_mat3_outer(f0, f1), -c01 / det);
        pi = sl_mat3_mul_add(pi, sl_mat3_outer(f1, f0), -c01 / det);
    }
    *positive =
        sl_mat3_mul_add(sl_mat3_scale(ff, mu), pi, 0.5f * lambda * (c00 + c11));
    *negative = sl_mat3_scale(pi, -(mu + lambda));
}

/* Adds triangle t's pair terms for corner i to (gradient, block). */
static void sc_sipd_triangle(const sc_sim *sim, uint32_t t, uint32_t corner,
                             sl_vec3 *gradient, sl_mat3 *block)
{
    const sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    const uint32_t *idx = s->triangle_index + 3u * t;
    const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                            s->position[idx[2]] };
    sl_mat3 positive;
    sl_mat3 negative;
    sc_sipd_tension(xs, s->triangle_rest + 4u * t, rec->membrane_mu,
                    rec->membrane_lambda, &positive, &negative);
    const float area = s->triangle_area[t];
    for (uint32_t m = 0; m < 2u; m++) {
        const uint32_t j = (corner + 1u + m) % 3u;
        const float c = sim->pair_weight[6u * t + 2u * corner + m];
        const sl_mat3 pair =
            sl_mat3_scale(sl_mat3_add(positive, negative), c * area);
        const sl_vec3 d = sl_vec3_sub(xs[j], xs[corner]);
        *gradient = sl_vec3_sub(*gradient, sl_mat3_mul_vec3(pair, d));
        /* L+ = A (c+ (PF+)+ + c- (PF+)-) */
        const sl_mat3 implicit_part = (c > 0.0f)
                                          ? sl_mat3_scale(positive, c * area)
                                          : sl_mat3_scale(negative, c * area);
        *block = sl_mat3_add(*block, implicit_part);
    }
}

static void sc_sipd_bend(const sc_sim *sim, uint32_t b, uint32_t slot,
                         sl_vec3 *gradient, sl_mat3 *block)
{
    const sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    const uint32_t *idx = s->bend_index + 4u * b;
    const float *w = s->bend_weight + 4u * b;
    const float k = rec->bending_stiffness * s->bend_scale[b];
    for (uint32_t j = 0; j < 4u; j++) {
        if (j == slot) {
            continue;
        }
        const float pair = -k * w[slot] * w[j]; /* L_ij = pair * I */
        const sl_vec3 d =
            sl_vec3_sub(s->position[idx[j]], s->position[idx[slot]]);
        *gradient = sl_vec3_mul_add(*gradient, d, -pair);
        if (pair > 0.0f) {
            *block = sl_mat3_add(*block, sl_mat3_diagonal(pair));
        }
    }
}

/* Gradient of the incremental potential and SIPD's implicit matrix for
 * vertex i, with penalty contact (as in silk). */
static void sc_sipd_vertex_system(sc_sim *sim, uint32_t i, float h,
                                  sl_vec3 *gradient, sl_mat3 *block)
{
    sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    const float inertia = s->mass[i] / (h * h);
    *gradient = sl_vec3_scale(sl_vec3_sub(s->position[i], s->inertia_target[i]),
                              inertia);
    *block = sl_mat3_diagonal(inertia);
    for (uint32_t r = s->vertex_triangle_offset[i];
         r < s->vertex_triangle_offset[i + 1u]; r++) {
        sc_sipd_triangle(sim, s->vertex_triangle[r] / 3u,
                         s->vertex_triangle[r] % 3u, gradient, block);
    }
    if (rec->bending_stiffness > 0.0f) {
        for (uint32_t r = s->vertex_bend_offset[i];
             r < s->vertex_bend_offset[i + 1u]; r++) {
            sc_sipd_bend(sim, s->vertex_bend[r] / 4u, s->vertex_bend[r] % 4u,
                         gradient, block);
        }
    }
    for (uint32_t c = 0; c < s->collider_count; c++) {
        sl_vec3 n;
        const float depth =
            rec->thickness -
            sl_collider_distance(&s->colliders[c], s->position[i], &n);
        if (depth > 0.0f) {
            *gradient =
                sl_vec3_mul_add(*gradient, n, -s->collision_stiffness * depth);
            *block = sl_mat3_mul_add(*block, sl_mat3_outer(n, n),
                                     s->collision_stiffness);
            s->contact[i] = 1u;
        }
    }
}

static void sc_sipd_jacobi_sweep(sc_sim *sim, float h)
{
    sl_world_state *s = sim->world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        sim->gradient[i] = sl_vec3_zero();
        sim->base[i] = s->position[i];
        if (s->pinned[i] != 0u) {
            continue;
        }
        sl_vec3 g;
        sl_mat3 a;
        sc_sipd_vertex_system(sim, i, h, &g, &a);
        sl_vec3 step = sl_vec3_zero();
        if (sl_mat3_solve(a, sl_vec3_neg(g), 0.0f, &step)) {
            sim->gradient[i] = step; /* reused as the search direction */
        }
    }
    /* Global backtracking line search on the incremental potential. */
    const double before = sl_solver_energy(s, h);
    float alpha = 1.0f;
    for (uint32_t k = 0; k < SC_LINE_SEARCH_MAX; k++) {
        for (uint32_t i = 0; i < s->vertex_count; i++) {
            s->position[i] =
                sl_vec3_mul_add(sim->base[i], sim->gradient[i], alpha);
        }
        if (sl_solver_energy(s, h) <= before) {
            return;
        }
        alpha *= 0.5f;
    }
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        s->position[i] = sim->base[i];
    }
}

static void sc_sipd_gs_sweep(sc_sim *sim, float h)
{
    sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    for (uint32_t color = 0; color < rec->color_count; color++) {
        for (uint32_t k = rec->color_offsets[color];
             k < rec->color_offsets[color + 1u]; k++) {
            const uint32_t i = s->color_order[k];
            if (s->pinned[i] != 0u) {
                continue;
            }
            sl_vec3 g;
            sl_mat3 a;
            sc_sipd_vertex_system(sim, i, h, &g, &a);
            sl_vec3 step;
            if (sl_mat3_solve(a, sl_vec3_neg(g), 0.0f, &step)) {
                s->position[i] = sl_vec3_add(s->position[i], step);
            }
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Shared substep end for the non-silk solvers: silk's velocity update,
 * velocity-level friction (normal load from penalty depth, or for XPBD from
 * the projection impulse), edge stretch damping and speed cap. */

static void sc_substep_end(sc_sim *sim, float h)
{
    sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        s->velocity[i] = sl_vec3_scale(
            sl_vec3_sub(s->position[i], s->position_start[i]), 1.0f / h);
        if (s->contact[i] == 0u || s->pinned[i] != 0u) {
            continue;
        }
        for (uint32_t c = 0; c < s->collider_count; c++) {
            const sl_collider *collider = &s->colliders[c];
            sl_vec3 n;
            const float depth =
                rec->thickness -
                sl_collider_distance(collider, s->position[i], &n);
            float load_dv; /* mu-free velocity budget: lambda h / m */
            if (sim->solver == SC_XPBD) {
                if (!(depth > -1e-6f)) {
                    continue;
                }
                load_dv = sim->corrected[i] / h;
            } else {
                if (!(depth > 0.0f)) {
                    continue;
                }
                load_dv = s->collision_stiffness * depth * h / s->mass[i];
            }
            const float mu = sqrtf(rec->friction * collider->friction);
            const sl_vec3 v = s->velocity[i];
            const sl_vec3 t = sl_vec3_mul_add(v, n, -sl_vec3_dot(n, v));
            const float speed = sl_vec3_length(t);
            if (speed > 0.0f) {
                const float reach = mu * load_dv;
                const float cut = (speed <= reach) ? 1.0f : reach / speed;
                s->velocity[i] = sl_vec3_mul_add(v, t, -cut);
            }
        }
    }
    const float fraction = 1.0f - expf(-rec->damping * h);
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const uint32_t a = s->edge_index[2u * e];
        const uint32_t b = s->edge_index[2u * e + 1u];
        const float wa = sc_inv_mass(s, a);
        const float wb = sc_inv_mass(s, b);
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
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        const float speed = sl_vec3_length(s->velocity[i]);
        if (speed > s->speed_max) {
            s->velocity[i] =
                sl_vec3_scale(s->velocity[i], s->speed_max / speed);
        }
    }
}

static void sc_step(sc_sim *sim)
{
    if (sim->solver == SC_VBD) {
        sl_world_step(&sim->world, SC_DT);
        return;
    }
    sl_world_state *s = sim->world.state;
    const float h = SC_DT / (float)sim->substeps;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->pinned[i] != 0u) {
            s->pin_origin[i] = s->position[i];
        }
    }
    for (uint32_t sub = 0; sub < sim->substeps; sub++) {
        sl_solver_substep_begin(s, h, (float)(sub + 1u) / (float)sim->substeps);
        memset(sim->lambda, 0,
               sizeof(float) * (s->edge_count + 3u * s->bend_count));
        memset(sim->corrected, 0, sizeof(float) * s->vertex_count);
        for (uint32_t it = 0; it < sim->iterations; it++) {
            if (sim->solver == SC_XPBD) {
                sc_xpbd_sweep(sim, h);
            } else if (sim->solver == SC_SIPD_JACOBI) {
                sc_sipd_jacobi_sweep(sim, h);
            } else {
                sc_sipd_gs_sweep(sim, h);
            }
        }
        sc_substep_end(sim, h);
    }
}

/* ---------------------------------------------------------------------- */
/* Metrics                                                                 */

/* Kinetic + gravitational + the solver's own elastic energy, joules. */
static double sc_energy(const sc_sim *sim)
{
    const sl_world_state *s = sim->world.state;
    const sl_cloth_record *rec = &s->cloths[0];
    double e = 0.0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        e += (double)s->mass[i] *
             (0.5 * (double)sl_vec3_length_sq(s->velocity[i]) +
              9.81 * (double)s->position[i].y);
    }
    if (sim->solver == SC_XPBD) {
        for (uint32_t k = 0; k < s->edge_count; k++) {
            const double d = (double)sl_vec3_length(sl_vec3_sub(
                                 s->position[s->edge_index[2u * k]],
                                 s->position[s->edge_index[2u * k + 1u]])) -
                             (double)sim->edge_rest[k];
            e += 0.5 * (double)sim->spring_k * d * d;
        }
    } else {
        for (uint32_t t = 0; t < s->triangle_count; t++) {
            const uint32_t *idx = s->triangle_index + 3u * t;
            const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                                    s->position[idx[2]] };
            e += (double)sl_membrane_energy(
                xs, s->triangle_rest + 4u * t, s->triangle_area[t],
                rec->membrane_mu, rec->membrane_lambda);
        }
    }
    for (uint32_t b = 0; b < s->bend_count; b++) {
        const uint32_t *idx = s->bend_index + 4u * b;
        const sl_vec3 xs[4] = { s->position[idx[0]], s->position[idx[1]],
                                s->position[idx[2]], s->position[idx[3]] };
        e += (double)sl_bend_energy(xs, s->bend_weight + 4u * b,
                                    s->bend_scale[b], rec->bending_stiffness);
    }
    return e;
}

static float sc_speed_max(const sc_sim *sim)
{
    const sl_world_state *s = sim->world.state;
    float v = 0.0f;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        v = sl_max(v, sl_vec3_length(s->velocity[i]));
    }
    return v;
}

static bool sc_finite(const sc_sim *sim)
{
    const sl_world_state *s = sim->world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (!sl_vec3_is_finite(s->position[i]) ||
            sl_abs(s->position[i].x) >= SL_POSITION_ABS_MAX ||
            sl_abs(s->position[i].y) >= SL_POSITION_ABS_MAX) {
            return false;
        }
    }
    return true;
}

static double sc_seconds(void)
{
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC) {
        return 0.0;
    }
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

typedef struct sc_result {
    float fall;         /* free-fall speed / exact, 0.5 s */
    float strain_mean;  /* hanging curtain, 4 s */
    float strain_off;   /* max off the pins */
    float hang_speed;   /* residual speed, hanging */
    double swing_gain;  /* (peak - initial) / swing drop energy */
    double drape_gain;  /* (peak - initial) / drape drop energy */
    float drape_depth;  /* deepest penetration below the sphere, m */
    float drape_speed;  /* residual speed, drape */
    double ms_per_step; /* curtain */
    bool finite;
} sc_result;

static void sc_run(sc_solver solver, uint32_t substeps, uint32_t iterations,
                   uint32_t cells, sc_result *out)
{
    memset(out, 0, sizeof(*out));
    out->finite = true;
    sc_sim sim;
    /* Free fall. */
    if (sc_build(&sim, solver, substeps, iterations,
                 sl_vec3_make(-0.5f, 1.0f, 0.0f),
                 sl_vec3_make(1.0f, 0.0f, 0.0f),
                 sl_vec3_make(0.0f, -1.0f, 0.0f), cells, 0.0f)) {
        for (uint32_t k = 0; k < 30u; k++) {
            sc_step(&sim);
        }
        const uint32_t mid = (cells / 2u) * (cells + 1u) + cells / 2u;
        out->fall = -sim.world.state->velocity[mid].y / (9.81f * 0.5f);
        out->finite = out->finite && sc_finite(&sim);
    }
    sc_free(&sim);

    /* Hanging curtain from two corners: stretch, cost. */
    if (sc_build(&sim, solver, substeps, iterations,
                 sl_vec3_make(-0.5f, 1.0f, 0.0f),
                 sl_vec3_make(1.0f, 0.0f, 0.0f),
                 sl_vec3_make(0.0f, -1.0f, 0.0f), cells, 0.6f)) {
        sl_world_state *s = sim.world.state;
        (void)sl_world_vertex_pin_set(&sim.world, 0u, 0u, true);
        (void)sl_world_vertex_pin_set(&sim.world, 0u, cells, true);
        float *rest = malloc(sizeof(float) * s->edge_count);
        for (uint32_t e = 0; e < s->edge_count && rest != NULL; e++) {
            rest[e] = sim.edge_rest[e];
        }
        const double start = sc_seconds();
        for (uint32_t k = 0; k < 240u; k++) {
            sc_step(&sim);
        }
        out->ms_per_step = 1000.0 * (sc_seconds() - start) / 240.0;
        double sum = 0.0;
        for (uint32_t e = 0; e < s->edge_count && rest != NULL; e++) {
            const uint32_t a = s->edge_index[2u * e];
            const uint32_t b = s->edge_index[2u * e + 1u];
            const float strain =
                sl_vec3_length(sl_vec3_sub(s->position[a], s->position[b])) /
                    rest[e] -
                1.0f;
            sum += (double)strain;
            if (s->pinned[a] == 0u && s->pinned[b] == 0u) {
                out->strain_off = sl_max(out->strain_off, strain);
            }
        }
        out->strain_mean = (float)(sum / (double)s->edge_count);
        out->hang_speed = sc_speed_max(&sim);
        out->finite = out->finite && sc_finite(&sim);
        free(rest);
    }
    sc_free(&sim);

    /* Sheet swinging down from a hinged edge. */
    if (sc_build(&sim, solver, substeps, iterations,
                 sl_vec3_make(-0.5f, 1.0f, 0.0f),
                 sl_vec3_make(1.0f, 0.0f, 0.0f), sl_vec3_make(0.0f, 0.0f, 1.0f),
                 cells, 0.6f)) {
        for (uint32_t i = 0; i <= cells; i++) {
            (void)sl_world_vertex_pin_set(&sim.world, 0u, i, true);
        }
        const double initial = sc_energy(&sim);
        double peak = initial;
        double low = initial;
        for (uint32_t k = 0; k < 180u; k++) {
            sc_step(&sim);
            const double e = sc_energy(&sim);
            peak = fmax(peak, e);
            low = fmin(low, e);
        }
        /* Gain relative to the energy the swing can release (0.98 J). */
        out->swing_gain = (peak - initial) / (0.2 * 9.81 * 0.5);
        out->finite = out->finite && sc_finite(&sim);
        (void)low;
    }
    sc_free(&sim);

    /* 1.2 m sheet dropped on a 0.3 m sphere. */
    if (sc_build(&sim, solver, substeps, iterations,
                 sl_vec3_make(-0.6f, 0.5f, -0.6f),
                 sl_vec3_make(1.2f, 0.0f, 0.0f), sl_vec3_make(0.0f, 0.0f, 1.2f),
                 cells, 0.6f)) {
        const sl_collider sphere = { SL_COLLIDER_SPHERE, sl_vec3_zero(),
                                     sl_vec3_zero(), 0.3f, 0.5f };
        uint32_t id = 0;
        (void)sl_world_collider_add(&sim.world, &sphere, &id);
        const double initial = sc_energy(&sim);
        double peak = initial;
        for (uint32_t k = 0; k < 180u; k++) {
            sc_step(&sim);
            peak = fmax(peak, sc_energy(&sim));
            const sl_world_state *s = sim.world.state;
            for (uint32_t i = 0; i < s->vertex_count; i++) {
                out->drape_depth = sl_max(
                    out->drape_depth, 0.3f - sl_vec3_length(s->position[i]));
            }
        }
        /* Relative to the sheet's drop energy onto the pole (0.56 J). */
        out->drape_gain = (peak - initial) / (0.288 * 9.81 * 0.2);
        out->drape_speed = sc_speed_max(&sim);
        out->finite = out->finite && sc_finite(&sim);
    }
    sc_free(&sim);
}

int main(int argc, char **argv)
{
    bool quick = false;
    bool sipd = true;
    uint32_t cells = 24u;
    for (int a = 1; a < argc; a++) {
        if (strcmp(argv[a], "--quick") == 0) {
            quick = true;
        } else if (strcmp(argv[a], "--no-sipd") == 0) {
            sipd = false;
        } else if (a + 1 < argc && strcmp(argv[a], "--cells") == 0) {
            const long value = strtol(argv[++a], NULL, 10);
            if (value >= 4 && value <= 128) {
                cells = (uint32_t)value;
            }
        }
    }
    static const uint32_t budgets[][2] = {
        /* substeps, iterations */
        { 8u, 1u }, { 16u, 1u }, { 32u, 1u }, { 64u, 1u },
        { 8u, 4u }, { 16u, 4u }, { 4u, 16u },
    };
    const size_t budget_count =
        quick ? 2u : sizeof(budgets) / sizeof(budgets[0]);
    printf("cloth solver comparison: %u x %u grid (%.1f cm edges), "
           "mu = lambda = 2e3 N/m, 0.2 kg/m^2, 1/60 s steps\n\n",
           cells, cells, 120.0 / (double)cells);
    printf("| solver | substeps x iters | ms/step | free fall | mean strain "
           "| max strain off pins | hang speed | swing gain | drape gain | "
           "drape depth | drape speed |\n");
    printf("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
           "---: | ---: |\n");
    for (size_t b = 0; b < budget_count; b++) {
        for (int v = 0; v < (int)SC_SOLVER_COUNT; v++) {
            if (!sipd && (v == (int)SC_SIPD_JACOBI || v == (int)SC_SIPD_GS)) {
                continue;
            }
            sc_result r;
            sc_run((sc_solver)v, budgets[b][0], budgets[b][1], cells, &r);
            printf("| %s | %u x %u | %.2f | %.0f%% | %.2f%% | %.1f%% | %.2f "
                   "| %+.0f%% | %+.0f%% | %.1f mm | %.2f |%s\n",
                   k_solver_names[v], budgets[b][0], budgets[b][1],
                   r.ms_per_step, 100.0 * (double)r.fall,
                   100.0 * (double)r.strain_mean, 100.0 * (double)r.strain_off,
                   (double)r.hang_speed, 100.0 * r.swing_gain,
                   100.0 * r.drape_gain, 1000.0 * (double)r.drape_depth,
                   (double)r.drape_speed, r.finite ? "" : " NONFINITE");
            fflush(stdout);
        }
    }
    return 0;
}
