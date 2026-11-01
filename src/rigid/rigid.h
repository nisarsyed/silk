#ifndef SILK_RIGID_H
#define SILK_RIGID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bvh.h"
#include "silk/math.h"

/* Rigid-body collision: the part of rigid bodies shared by whichever solver
 * Milestone 4 adopts (docs/rigid.md). Internal until a solver exists; the
 * head-to-head harness and the tests use it directly.
 *
 * Units are meters, kilograms, and seconds. Bodies are boxes for now; the
 * shape storage is a half-extent per body. */

/* Body indices fit in 16 bits, so a pair key fits in 32. */
#define SL_BODY_COUNT_MAX 65535u

/* A manifold whose second index is at least SL_RIGID_STATIC_BASE is
 * against static plane (index - SL_RIGID_STATIC_BASE). */
#define SL_RIGID_STATIC_BASE 0x10000u

/* Contact points per manifold: a face contact clips to at most 8 points
 * and keeps the 4 that span the largest area (sl_rigid_reduce). */
#define SL_MANIFOLD_POINT_COUNT_MAX 4u

/* Per-point storage owned by the solver and carried across steps for
 * points matched by feature id: three impulses (Soft Step) or three
 * multipliers and three penalty stiffnesses (augmented Lagrangian). */
#define SL_CONTACT_WARM_COUNT 6u

typedef struct sl_contact_point {
    sl_vec3 local_a;  /* point on body a, in a's frame, m */
    sl_vec3 local_b;  /* point on body b, in b's frame (world for a plane) */
    float separation; /* along the normal at detection, m; < 0 overlaps */
    uint32_t id;      /* feature id, stable while the features persist */
    float warm[SL_CONTACT_WARM_COUNT];
} sl_contact_point;

typedef struct sl_manifold {
    uint32_t body_a; /* body_a < body_b */
    uint32_t body_b; /* a body, or SL_RIGID_STATIC_BASE + plane */
    sl_vec3 normal;  /* world, unit, from a toward b */
    uint32_t point_count;
    sl_contact_point points[SL_MANIFOLD_POINT_COUNT_MAX];
} sl_manifold;

/* A static plane: points x with dot(x - point, normal) >= 0 are outside. */
typedef struct sl_rigid_plane {
    sl_vec3 point;
    sl_vec3 normal; /* unit */
} sl_rigid_plane;

/* Bodies (SoA) and the contact state of the last detection. All storage is
 * one allocation made at init. */
typedef struct sl_body_set {
    uint32_t capacity;
    uint32_t count;
    uint32_t manifold_capacity;
    sl_vec3 *position;         /* center of mass, world, m */
    sl_quat *orientation;      /* body to world, unit */
    sl_vec3 *half_extent;      /* box, m, each > 0 */
    sl_vec3 *velocity;         /* m/s */
    sl_vec3 *angular_velocity; /* world, rad/s */
    float *inverse_mass;       /* 1/kg; 0 is static */
    sl_vec3 *inverse_inertia;  /* principal, body frame, 1/(kg m^2) */
    /* Contacts: manifolds sorted by (body_a, body_b), double-buffered so
     * the previous detection's warm data can be carried over. */
    sl_manifold *manifolds;
    sl_manifold *manifolds_previous;
    uint32_t manifold_count;
    uint32_t manifold_count_previous;
    uint32_t dropped; /* candidate pairs or manifolds beyond capacity */
    /* Broad phase: a BVH over body bounding boxes rebuilt per detection
     * (rigid bodies do not keep neighbourhoods), and sorted pair keys. */
    sl_bvh bvh;
    uint32_t *pair_key;
    uint32_t *pair_scratch;
    void *arena;
} sl_body_set;

/* Allocates storage for `capacity` bodies (in [1, SL_BODY_COUNT_MAX]) and
 * `manifold_capacity` manifolds (>= 1). Returns false, allocating nothing,
 * on invalid sizes or allocation failure. */
bool sl_body_set_init(sl_body_set *set, uint32_t capacity,
                      uint32_t manifold_capacity);
void sl_body_set_destroy(sl_body_set *set);

/* Adds a box of the given half extents (m, finite, > 0) and density
 * (kg/m^3, finite, >= 0; 0 makes it static) at rest. Returns false without
 * change when full or invalid; *body receives the index. */
bool sl_body_add_box(sl_body_set *set, sl_vec3 position, sl_quat orientation,
                     sl_vec3 half_extent, float density, uint32_t *body);

/* Detects all contacts within `margin` (m, >= 0; a positive margin keeps
 * nearly touching points as speculative contacts with separation > 0):
 * every box pair, and every box against every plane. Matched points keep
 * their warm data; new points start at zero. */
void sl_rigid_collide(sl_body_set *set, const sl_rigid_plane *planes,
                      uint32_t plane_count, float margin);

/* Narrow phase, pure. Each fills `out` (body indices and warm data
 * zeroed) and returns its point count; 0 means no contact within margin.
 * The normal points from the first box toward the second shape. */
uint32_t sl_collide_box_box(sl_vec3 pa, sl_quat qa, sl_vec3 ha, sl_vec3 pb,
                            sl_quat qb, sl_vec3 hb, float margin,
                            sl_manifold *out);
uint32_t sl_collide_box_plane(sl_vec3 pa, sl_quat qa, sl_vec3 ha,
                              const sl_rigid_plane *plane, float margin,
                              sl_manifold *out);

#endif /* SILK_RIGID_H */
