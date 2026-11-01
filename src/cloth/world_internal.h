#ifndef SILK_WORLD_INTERNAL_H
#define SILK_WORLD_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

#include "bvh.h"
#include "parallel.h"
#include "silk/world.h"

/* One cloth's slice of the shared pools. Every range is contiguous and
 * cloths are laid out in insertion order. color_offsets index color_order
 * relative to vertex_offset: color c owns color_order entries
 * [vertex_offset + color_offsets[c], vertex_offset + color_offsets[c + 1]).
 */
typedef struct sl_cloth_record {
    uint32_t vertex_offset;
    uint32_t vertex_count;
    uint32_t triangle_offset;
    uint32_t triangle_count;
    uint32_t bend_offset;
    uint32_t bend_count;
    uint32_t edge_offset;
    uint32_t edge_count;
    uint32_t color_count;
    uint32_t color_offsets[SL_CLOTH_COLOR_COUNT_MAX + 1u];
    float membrane_mu;
    float membrane_lambda;
    float bending_stiffness;
    float damping;
    float thickness;
    float friction;
    float drag;
    float stiffness_rate_max; /* sigma, 1/s^2, see SL_STIFFNESS_STEP_MAX */
    /* Self contact, per detection: nearest primitive of another cloth (a
     * lower bound, m), the cloth's bounding box, and the cap on its common
     * translation (see sl_self_constrain). */
    float inter_dmin;
    float translation_cap;
    sl_vec3 box_lo;
    sl_vec3 box_hi;
} sl_cloth_record;

/* One self-contact pair as a vertex stencil (see contact.h): the separation
 * s = sum_k weight_k x_k is re-derived from current positions on every
 * evaluation, the vertices fixed at detection. Every pair pushes all of its
 * vertices, so the contact forces are the gradient of one energy
 * sum_pairs share * g(d) and come in action-reaction sets. Edge pairs
 * (EDGE_EDGE, EDGE_VERTEX) are staged once from each edge of an unordered
 * edge pair and carry share 1/2, so a pair accepted from both edges acts
 * once. Each side passes its own feasibility test, so one side can be
 * rejected; pushing only the detecting edge then left the other edge with
 * no reaction, and a gathered curtain (examples/sandbox scene 1) gained
 * energy until it flew apart. Transient AoS: a pair is always consumed
 * whole. */
typedef enum sl_pair_kind {
    SL_PAIR_VERTEX_FACE = 0,   /* v, t0, t1, t2 */
    SL_PAIR_VERTEX_EDGE = 1,   /* v, e0, e1 */
    SL_PAIR_VERTEX_VERTEX = 2, /* v, w */
    SL_PAIR_EDGE_EDGE = 3,     /* e0, e1, f0, f1 */
    SL_PAIR_EDGE_VERTEX = 4    /* e0, e1, w */
} sl_pair_kind;

typedef struct sl_contact_pair {
    uint32_t vertex[4];
    uint8_t kind;   /* sl_pair_kind */
    uint8_t count;  /* vertices used */
    float radius;   /* m */
    float friction; /* combined Coulomb coefficient */
} sl_contact_pair;

/* Weight of a pair in the contact energy (see sl_contact_pair). */
static inline float sl_pair_share(const sl_contact_pair *p)
{
    return (p->kind == SL_PAIR_EDGE_EDGE || p->kind == SL_PAIR_EDGE_VERTEX)
               ? 0.5f
               : 1.0f;
}

/* Vertex state is SoA by field; each field is a packed array of vec3 (or
 * scalars) indexed by the global vertex index. Triangle, bending and edge
 * data are SoA likewise. Memory per vertex capacity is 6 vec3 + float +
 * 3 bytes + 3 uint32 = 91 bytes; per triangle capacity 3 + 4 + 1 words
 * plus 3 references, 1.5 bending stencils of 4 + 4 + 1 words and 4
 * references each, and 3 edges of 2 words: 146 bytes. A 64 x 64 cell grid
 * (4225 vertices, 8192 triangles) therefore needs about 1.6 MB. Self
 * contact adds its own arrays, including one vec3 per vertex for
 * position_next. */
struct sl_world_state {
    /* Configuration, validated at init. */
    sl_vec3 gravity;
    sl_vec3 wind;
    uint32_t substep_count;
    uint32_t iteration_count;
    float speed_max;
    float collision_stiffness;

    uint32_t vertex_capacity;
    uint32_t triangle_capacity;
    uint32_t bend_capacity;
    uint32_t edge_capacity;
    uint32_t cloth_capacity;
    uint32_t vertex_count;
    uint32_t triangle_count;
    uint32_t bend_count;
    uint32_t edge_count;
    uint32_t cloth_count;
    uint32_t collider_count;
    uint32_t contact_count; /* last sweep of the last step */
    uint32_t substep_count_last;
    bool substep_limited;
    float stiffness_rate_max; /* over all cloths */

    /* Self contact; every array below is empty when the capacity is 0. */
    uint32_t pair_capacity;
    uint32_t pair_count;
    float self_stiffness; /* N/m */
    /* Parallel execution, from the config; NULL runs serially. */
    sl_parallel_for_fn *parallel_for;
    void *parallel_context;
    float query_radius;       /* m, of the last detection */
    float self_motion;        /* m, inertial motion estimate of the substep */
    uint32_t self_detections; /* this step */
    uint32_t self_dropped;    /* this step */
    uint32_t self_bound_hits; /* this step */
    uint32_t bound_exceeded;  /* distinct vertices held, since detection */
    sl_contact_pair *pairs;
    uint32_t *pair_ref_offset; /* vertex_capacity + 1: pairs pushing a vertex */
    uint32_t *pair_ref;        /* pair * 4 + slot */
    /* Nearest same-cloth primitives (other cloths are tracked per cloth),
     * capped at the query radius, m. */
    float *vertex_dmin;   /* nearest non-incident triangle */
    float *triangle_dmin; /* nearest non-incident vertex */
    float *edge_dmin;     /* nearest non-adjacent edge */
    float *bound;         /* bound on deviation from the cloth's shift */
    uint8_t *at_bound;    /* 1 once held by its bound, per detection */
    /* Edge-edge staging: each unordered edge pair is examined once and
     * yields a contact for each side; contacts are then grouped by owning
     * edge (counting sort) for de-duplication and the per-owner cap. */
    sl_contact_pair *stage_pair;
    float *stage_distance;
    uint32_t *stage_owner;
    uint32_t *stage_order;  /* pair_capacity: staged indices by owner */
    uint32_t *owner_offset; /* edge_capacity + 1 */
    uint32_t stage_count;
    sl_vec3 *position_safe; /* positions at the last detection */
    /* Positions solved in the current color, committed after it, so that
     * same-color vertices joined by a contact read each other's
     * start-of-color positions whatever the order or thread (see
     * solver_self.c). */
    sl_vec3 *position_next;
    sl_bvh triangle_bvh; /* over all triangles */
    sl_bvh edge_bvh;     /* over all unique edges */

    /* Vertex fields. */
    sl_vec3 *position;
    sl_vec3 *position_start; /* x at the start of the current substep */
    sl_vec3 *velocity;
    sl_vec3 *inertia_target; /* y */
    sl_vec3 *pin_target;     /* pinned: position at the end of the step */
    sl_vec3 *pin_origin;     /* pinned: position at the start of the step */
    float *mass;             /* kg, > 0 */
    uint32_t *vertex_cloth;  /* owning cloth index */
    uint8_t *pinned;         /* 0 or 1 */
    uint8_t *contact;       /* 0 or 1: touched a collider in the latest sweep */
    uint8_t *color_scratch; /* cloth add only */
    uint32_t *color_order;  /* global vertex indices, see sl_cloth_record */
    /* CSR adjacency. Triangle references encode triangle * 3 + corner and
     * bending references encode stencil * 4 + slot, both global. Offsets
     * have vertex_capacity + 1 entries. */
    uint32_t *vertex_triangle_offset;
    uint32_t *vertex_triangle;
    uint32_t *vertex_bend_offset;
    uint32_t *vertex_bend;

    /* Triangle fields. rest holds the 2x2 inverse rest matrix row-major
     * (a, b, c, d) so the deformation gradient is
     * F = [x1 - x0, x2 - x0] * [[a, b], [c, d]]. */
    uint32_t *triangle_index; /* 3 per triangle, global vertex indices */
    float *triangle_rest;     /* 4 per triangle, 1/m */
    float *triangle_area;     /* rest area, m^2 */

    /* Bending stencils on interior edges: slots 0, 1 are the edge and 2, 3
     * the opposite wing vertices. */
    uint32_t *bend_index; /* 4 per stencil */
    float *bend_weight;   /* 4 per stencil, dimensionless cotangent sums */
    float *bend_scale;    /* per stencil, 1/m^2 */

    /* Unique mesh edges, 2 global vertex indices each, for damping. */
    uint32_t *edge_index;

    sl_cloth_record *cloths;
    sl_collider colliders[SL_COLLIDER_COUNT_MAX];

    void *arena;
    size_t arena_bytes;
};

/* Substeps one step of dt needs: at least the configured count, enough to
 * keep h^2 * stiffness_rate_max <= SL_STIFFNESS_STEP_MAX, and at most
 * SL_SUBSTEP_COUNT_MAX. Sets *limited when the bound needed more. */
uint32_t sl_solver_substep_count(const sl_world_state *state, float dt,
                                 bool *limited);

/* Self contact (self_contact.c). Detection runs at x, which must be
 * intersection-free, and refreshes pairs, bounds and position_safe. */
/* Inline: it is tested inside the per-vertex solve, where an out-of-line
 * call measurably slowed the self-contact-free path (5.0 -> 6.4 ms/step on
 * a 32 x 32 curtain). */
static inline bool sl_self_enabled(const sl_world_state *state)
{
    return state->pair_capacity > 0u;
}
void sl_self_detect(sl_world_state *state, float motion);
/* Adds vertex i's self-contact gradient terms (as force) and PSD Hessian
 * block. */
void sl_self_vertex(const sl_world_state *state, uint32_t i, sl_vec3 *force,
                    sl_mat3 *hessian);
/* Moves every vertex back inside its bound (see self_contact.c); returns
 * the number moved. */
uint32_t sl_self_constrain(sl_world_state *state);
/* Velocity-level Coulomb friction over all pairs. */
void sl_self_friction(sl_world_state *state, float h);
/* Self-contact energy at the current positions, joules. */
double sl_self_energy(const sl_world_state *state);
/* Rebuilds both BVHs over the current topology (after a cloth add). */
void sl_self_build(sl_world_state *state);
/* Rebuilds the vertex -> pair references after pairs change. */
void sl_self_index(sl_world_state *state);
/* Closest points of pair k at the current positions (see
 * sl_self_contact); false when they are undefined. */
bool sl_self_pair_points(const sl_world_state *state, uint32_t k,
                         sl_vec3 *point_a, sl_vec3 *point_b);

/* Validates and normalizes a collider description. */
bool sl_collider_validate(const sl_collider *desc, sl_collider *out);

/* Substep phases, exposed for tests. A substep is begin, iteration_count
 * sweeps, then end. */
void sl_solver_substep_begin(sl_world_state *state, float h, float alpha);
void sl_solver_sweep(sl_world_state *state, float h);
void sl_solver_substep_end(sl_world_state *state, float h);

/* One sweep including self-contact terms (solver_self.c). */
void sl_solver_sweep_self(sl_world_state *state, float h);

/* Runs task over [0, count) through the world's parallel_for (see
 * core/parallel.h). */
static inline void sl_parallel(const sl_world_state *state, sl_task_fn *task,
                               uint32_t count, uint32_t grain, void *context)
{
    sl_parallel_run(state->parallel_for, state->parallel_context, task, count,
                    grain, context);
}

/* Re-detects contacts mid-substep when at least
 * SL_REDETECT_FRACTION of the vertices hit their bounds (OGC gamma_e). */
void sl_solver_redetect(sl_world_state *state);

/* Incremental potential G(x) of the current substep: inertia, membrane,
 * bending, and contact energy, without damping or friction. Joules. */
double sl_solver_energy(const sl_world_state *state, float h);

#endif /* SILK_WORLD_INTERNAL_H */
