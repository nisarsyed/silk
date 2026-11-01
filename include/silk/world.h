#ifndef SILK_WORLD_H
#define SILK_WORLD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "silk/math.h"
#include "silk/parallel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Units are SI throughout: meters, kilograms, seconds, newtons. The world
 * frame is right-handed; gravity is a configuration value, so "up" is
 * whatever the caller makes it. */

/* Storage bounds, fixed at init. Raise only with a measured memory budget. */
#define SL_VERTEX_COUNT_MAX (1u << 20)
#define SL_TRIANGLE_COUNT_MAX (1u << 21)
#define SL_CLOTH_COUNT_MAX 256u
#define SL_COLLIDER_COUNT_MAX 32u

/* Per-vertex topology bounds. They cap the work of one vertex solve and the
 * greedy coloring; a regular grid uses 6 triangles and 7 colors.
 * Meshes exceeding either bound are rejected. Pinned by tests/test_mesh.c. */
#define SL_VERTEX_TRIANGLE_COUNT_MAX 32u
#define SL_CLOTH_COLOR_COUNT_MAX 64u

/* Solver work bounds. At equal cost (substeps x iterations), many
 * substeps of one sweep each stretch a hanging sheet least ("small steps",
 * Macklin et al. 2019): a sweep cannot remove low-frequency error, while a
 * smaller h makes each vertex block better conditioned. */
/* 128 covers mu = 2e3 N/m cloth down to 1.5 cm edges at 60 Hz (a 1 m,
 * 64 x 64 sheet needs 68 substeps under SL_STIFFNESS_STEP_MAX). */
#define SL_SUBSTEP_COUNT_MAX 128u
#define SL_ITERATION_COUNT_MAX 64u
#define SL_SUBSTEP_COUNT_DEFAULT 4u
#define SL_ITERATION_COUNT_DEFAULT 1u

/* Stability bound on h^2 * sigma, where sigma (1/s^2) is a vertex's rest
 * elastic stiffness over its inertia: the trace of its elastic Hessian
 * block at rest divided by 3 m. A sweep that starts from the inertial
 * prediction behaves explicitly on mid-frequency stretch modes it cannot
 * remove, so like a CFL limit it needs small h on stiff, fine cloth. On
 * regular grids sigma_max = 8 mu / (density edge^2) for lambda = mu. A
 * 1.2 m sheet dropped on a sphere (docs/solver.md) first gained energy at
 * h^2 sigma_max = 30 for mu = 1e4 N/m and at 43 for mu = 2e3 N/m, from 24
 * to 64 cells; 20 leaves a margin under the stiffer case. sl_world_step
 * raises the substep count to meet it. Pinned by tests/test_solver.c. */
#define SL_STIFFNESS_STEP_MAX 20.0f

/* Velocity cap applied after every substep, m/s. A cloth vertex moving
 * faster than this is almost certainly numerical, not physical. */
#define SL_SPEED_MAX_DEFAULT 100.0f

/* Penalty contact stiffness, N/m. Contact is solved implicitly inside each
 * vertex block, so stiffness trades penetration depth against conditioning,
 * not stability. With 1e4 N/m a 1 g vertex resting under 9.81 m/s^2 sinks
 * about 1 micrometer into its thickness shell. Pinned by tests/test_solver.c.
 */
#define SL_COLLISION_STIFFNESS_DEFAULT 1.0e4f

/* Self contact (Offset Geometric Contact; see sl_world_step).
 *
 * Two cloths, or two parts of one cloth, are kept at least a contact
 * radius r = thickness_a + thickness_b apart, the sum of the two cloths'
 * shell half-thicknesses. Keep r below the distance between non-adjacent
 * edges of a flat patch (0.7 of an edge on a regular grid): edge-edge
 * contact is defined on the wireframe and would push them apart; vertex
 * contact never acts within a flat sheet, at any r. The stiffness default
 * matches the paper's 1e4 N/m sphere-drape scenes; the barrier stage below r /
 * 2 makes the force unbounded regardless. A vertex keeps at most
 * SL_VERTEX_CONTACT_COUNT_MAX contact pairs (the closest), and an edge as
 * many edge-edge pairs. */
#define SL_SELF_CONTACT_COUNT_MAX (1u << 22)
#define SL_SELF_CONTACT_STIFFNESS_DEFAULT 1.0e4f
#define SL_VERTEX_CONTACT_COUNT_MAX 16u

/* Every stored and accepted coordinate lies in [-max, max] meters. float
 * spacing at 1e4 is about 1 mm, the coarsest resolution we accept for cloth.
 */
#define SL_POSITION_ABS_MAX 1.0e4f

/* Optional parallel execution: sl_parallel_for_fn and sl_task_fn are in
 * silk/parallel.h. */

typedef struct sl_world_config {
    uint32_t vertex_capacity;   /* in [1, SL_VERTEX_COUNT_MAX] */
    uint32_t triangle_capacity; /* in [1, SL_TRIANGLE_COUNT_MAX] */
    uint32_t cloth_capacity;    /* in [1, SL_CLOTH_COUNT_MAX] */
    sl_vec3 gravity;            /* m/s^2, finite */
    sl_vec3 wind;               /* air velocity, m/s, finite */
    /* 0 selects the defaults; otherwise within the matching maximum.
     * substep_count is a minimum: each step uses at least enough substeps
     * to satisfy SL_STIFFNESS_STEP_MAX, up to SL_SUBSTEP_COUNT_MAX. More
     * substeps or iterations reduce stretch error. */
    uint32_t substep_count;
    uint32_t iteration_count;
    /* 0 selects the documented defaults; otherwise finite and > 0. */
    float speed_max;           /* m/s */
    float collision_stiffness; /* N/m */
    /* Cloth-cloth and self contact (Offset Geometric Contact). The maximum
     * number of contact pairs kept per detection, in
     * [0, SL_SELF_CONTACT_COUNT_MAX]; 0 disables self contact and its
     * storage. Pairs beyond capacity are dropped and counted. */
    uint32_t self_contact_capacity;
    /* Quadratic-stage stiffness of the contact energy, N/m. 0 selects
     * SL_SELF_CONTACT_STIFFNESS_DEFAULT; otherwise finite and > 0. */
    float self_contact_stiffness;
    /* Parallel execution (see sl_parallel_for_fn); NULL runs serially.
     * parallel_context is passed through unchanged and must outlive the
     * world. */
    sl_parallel_for_fn *parallel_for;
    void *parallel_context;
} sl_world_config;

/* Opaque storage owned by the world after a successful init. */
typedef struct sl_world_state sl_world_state;

/* Caller-owned handle. Zero-initialize, then pass to sl_world_init. */
typedef struct sl_world {
    sl_world_state *state;
} sl_world;

/* A triangle mesh added as one cloth. The rest shape is the given
 * positions: each triangle's rest metric is taken from its own 3D shape and
 * bending treats the rest surface as flat (see sl_world_cloth_add).
 * Pointers are read during the call only. */
typedef struct sl_cloth_desc {
    const sl_vec3 *positions; /* vertex_count positions, meters */
    uint32_t vertex_count;
    /* 3 * triangle_count indices into positions. Winding is free. */
    const uint32_t *indices;
    uint32_t triangle_count;
    /* Areal density, kg/m^2, finite > 0. Cotton shirting is ~0.15. */
    float density;
    /* St. Venant-Kirchhoff membrane Lame parameters, N/m, finite >= 0.
     * mu resists shear and stretch; lambda resists area change. */
    float membrane_mu;
    float membrane_lambda;
    /* Bending modulus, N*m (J), finite >= 0. */
    float bending_stiffness;
    /* Stretch damping rate, 1/s, finite >= 0: each substep removes the
     * fraction 1 - exp(-damping * h) of every edge's relative velocity
     * along the edge. Rigid motion is never damped. */
    float damping;
    /* Collision shell half-thickness, meters, finite > 0. */
    float thickness;
    /* Coulomb coefficient, finite >= 0, combined with a collider's as
     * sqrt(cloth * collider). */
    float friction;
    /* Aerodynamic drag, kg/m^3 (half air density times the drag
     * coefficient; about 0.6 for a flag in air), finite >= 0. */
    float drag;
} sl_cloth_desc;

typedef enum sl_collider_kind {
    SL_COLLIDER_PLANE = 0,
    SL_COLLIDER_SPHERE = 1,
    SL_COLLIDER_CAPSULE = 2
} sl_collider_kind;

/* A static analytic collider. Moving one between steps is allowed; within a
 * step it is treated as static, so friction sees no collider velocity. */
typedef struct sl_collider {
    sl_collider_kind kind;
    /* PLANE: a point on the plane. SPHERE: center. CAPSULE: segment start.
     */
    sl_vec3 point_a;
    /* PLANE: outward normal, non-zero, normalized on accept. CAPSULE:
     * segment end. SPHERE: unused. */
    sl_vec3 point_b;
    /* SPHERE, CAPSULE: meters, finite > 0. PLANE: unused. */
    float radius;
    float friction; /* finite >= 0 */
} sl_collider;

typedef struct sl_world_stats {
    uint32_t vertex_count;
    uint32_t vertex_capacity;
    uint32_t triangle_count;
    uint32_t triangle_capacity;
    uint32_t bend_count; /* interior-edge bending stencils */
    uint32_t cloth_count;
    uint32_t cloth_capacity;
    uint32_t collider_count;
    uint32_t color_count_max; /* largest per-cloth color count */
    /* Vertices in contact during the final sweep of the last step. */
    uint32_t contact_count;
    /* Substeps the last step used, and whether stability needed more than
     * SL_SUBSTEP_COUNT_MAX (the step then ran the maximum anyway). */
    uint32_t substep_count;
    bool substep_limited;
    /* Largest per-vertex rest stiffness rate over all cloths, 1/s^2. */
    float stiffness_rate_max;
    /* Self contact during the last step: pairs at its final detection,
     * detections run, pairs dropped for capacity, and vertex moves cut
     * short by their conservative displacement bounds. */
    uint32_t self_contact_count;
    uint32_t self_contact_capacity;
    uint32_t self_detection_count;
    uint32_t self_contact_dropped;
    uint32_t self_bound_hits;
    size_t arena_bytes; /* exact runtime allocation */
} sl_world_stats;

/* Allocates every runtime array once. Returns false without allocating when
 * config is invalid or allocation fails. world must be zeroed or destroyed.
 */
bool sl_world_init(sl_world *world, const sl_world_config *config);

/* Releases storage and zeroes *world. Safe on a zeroed world. */
void sl_world_destroy(sl_world *world);

/* Adds a cloth mesh, writing its index (0, 1, ...) to *cloth. Returns false
 * without mutation when the desc is invalid or capacity is short. Rejects
 * out-of-range indices, repeated corners, triangles with area below
 * 1e-12 m^2, non-finite or out-of-domain positions, vertices referenced by
 * no triangle, edges shared by more than two triangles, and vertices beyond
 * the topology bounds. Vertex masses lump density * area / 3 per corner.
 * Velocities start at zero and no vertex is pinned. */
bool sl_world_cloth_add(sl_world *world, const sl_cloth_desc *desc,
                        uint32_t *cloth);

/* Adds a collider, writing its index to *collider. Returns false without
 * mutation when the desc is invalid or SL_COLLIDER_COUNT_MAX is reached. */
bool sl_world_collider_add(sl_world *world, const sl_collider *desc,
                           uint32_t *collider);

/* Replaces collider's shape. Same validation as add; false leaves it as is.
 */
bool sl_world_collider_set(sl_world *world, uint32_t collider,
                           const sl_collider *desc);

/* Returns false for an out-of-range index. */
bool sl_world_collider_get(const sl_world *world, uint32_t collider,
                           sl_collider *desc);

/* Replace world-level forcing. Return false on non-finite input. */
bool sl_world_gravity_set(sl_world *world, sl_vec3 gravity);
bool sl_world_wind_set(sl_world *world, sl_vec3 wind);

/* Pinned vertices are kinematic: the solver never moves them and they carry
 * infinite mass. Pinning holds the current position as the target;
 * unpinning keeps the velocity of the last step. Returns false for an
 * out-of-range cloth or vertex. */
bool sl_world_vertex_pin_set(sl_world *world, uint32_t cloth, uint32_t vertex,
                             bool pinned);

/* Moves a pinned vertex to target over the next step, linearly across its
 * substeps. Returns false for an unpinned or out-of-range vertex, or a
 * non-finite or out-of-domain target. */
bool sl_world_vertex_target_set(sl_world *world, uint32_t cloth,
                                uint32_t vertex, sl_vec3 target);

/* Read-only views of one cloth's vertices, in the order they were given.
 * Pointers stay valid until sl_world_destroy; the contents change on every
 * mutating call. NULL for an out-of-range cloth. */
const sl_vec3 *sl_world_cloth_positions(const sl_world *world, uint32_t cloth);
const sl_vec3 *sl_world_cloth_velocities(const sl_world *world, uint32_t cloth);

/* 0 for an out-of-range cloth. */
uint32_t sl_world_cloth_vertex_count(const sl_world *world, uint32_t cloth);

/* false for an out-of-range cloth or vertex. */
bool sl_world_vertex_is_pinned(const sl_world *world, uint32_t cloth,
                               uint32_t vertex);

sl_world_stats sl_world_stats_get(const sl_world *world);

#ifdef __cplusplus
}
#endif

#endif /* SILK_WORLD_H */
