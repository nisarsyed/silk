#ifndef SILK_WORLD_INTERNAL_H
#define SILK_WORLD_INTERNAL_H
#include "silk/world.h"

/* slots[i].dense marker for a free slot; also the liveness bit.
 * Invariant: body_count + free_count == body_capacity. */
#define SL_BODY_DENSE_NONE UINT32_MAX

typedef struct sl_body_slot {
    uint32_t dense;      /* row in [0, body_count) or SL_BODY_DENSE_NONE */
    uint32_t generation; /* bumped on destroy (wrap skips 0) */
} sl_body_slot;

typedef struct sl_joint_slot {
    uint32_t dense;      /* packed row or SL_BODY_DENSE_NONE when free */
    uint32_t generation; /* bumped on destroy/reset; wrap skips zero */
} sl_joint_slot;

typedef struct sl_world_state {
    uint32_t body_count;    /* rows used by every packed array below */
    uint32_t body_capacity; /* slot count, fixed at init */
    uint32_t free_count;
    uint32_t contact_count;
    uint32_t contact_capacity;
    uint32_t contact_drop_count;
    uint32_t pair_capacity;
    uint32_t moved_count;
    uint32_t joint_count;
    uint32_t joint_capacity;
    uint32_t joint_free_count;
    uint32_t joint_constraint_count;

    sl_vec2 gravity;        /* world units / second^2 */
    float linear_drag;      /* 1 / seconds, >= 0 */
    float angular_drag;     /* 1 / seconds, >= 0 */
    float linear_speed_max; /* world units / second, finite > 0 */
    uint32_t substep_count; /* in [1, SL_SUBSTEP_COUNT_MAX] */
    float contact_hertz;
    float contact_damping_ratio;
    float contact_push_velocity_max;
    float restitution_threshold;
    float joint_hertz;
    float joint_damping_ratio;

    /* Dual indexing: slots[handle.index] -> packed row, slot_of[row] ->
     * owning slot. Swap-remove repairs exactly one slot_of entry. */
    sl_body_slot *slots;
    uint32_t *slot_of;
    /* LIFO of free slot ids, depth == free_count; pops yield 0,1,2,... */
    uint32_t *free_indices;

    sl_vec2 *positions;  /* world units */
    sl_vec2 *velocities; /* world units / second */
    float *masses;       /* kilograms, > 0; 0 encodes infinite mass */
    float *inv_masses;   /* 1 / mass or 0 */
    sl_vec2 *forces;     /* kilograms * world units / second^2; the
                          * stepper clears them every step */
    /* Solver/integration scratch, cleared after every valid step. */
    sl_vec2 *delta_positions;

    sl_rotation *rotations; /* unit (cos, sin), body to world */
    /* Relative rotation, identity between steps. */
    sl_rotation *delta_rotations;
    float *angular_velocities; /* radians / second */
    float *torques;            /* force units * length; the stepper
                                * clears them every step */
    /* About the body origin, which constructed shapes keep at the
     * centroid; 0 encodes infinite inertia or no shape. */
    float *inertias;
    float *inv_inertias; /* 1 / inertia or 0; create / set_mass /
                          * set_shape are its only writers */
    float *frictions;    /* finite >= 0 */
    float *restitutions; /* finite in [0, 1] */
    uint8_t *types;      /* holds sl_body_type */
    /* Whole-record per-body array: consumers (mass data, bounds,
     * containment, later contacts) always want the complete shape, so
     * the record is the SoA granularity they use. */
    sl_shape *shapes;

    /* Broad-phase state. Proxy ids and moved flags follow packed body rows;
     * moved buffers carry stable body slots. tree is an internal sl_tree. */
    sl_aabb *proxy_aabbs;
    uint32_t *proxies;
    uint8_t *moved;
    uint32_t *moved_slots;
    uint32_t *query_slots;
    void *tree;

    /* Packed persistent records and an open-addressed set of canonical
     * body-slot pair keys. */
    sl_contact *contacts;
    uint64_t *pair_keys;

    /* Internal AoS scratch: complete constraint records are consumed
     * together during every scalar solver stage. */
    void *contact_constraints;
    uint32_t contact_constraint_count;

    /* Persistent joint state is SoA. Joint slots remain stable while packed
     * rows swap-remove; edge ids are 2 * slot + endpoint and link through
     * stable body-slot adjacency heads. */
    sl_joint_slot *joint_slots;
    uint32_t *joint_slot_of;
    uint32_t *joint_free_indices;
    uint8_t *joint_kinds;
    sl_body_handle *joint_bodies_a;
    sl_body_handle *joint_bodies_b;
    sl_vec2 *joint_local_anchors_a;
    sl_vec2 *joint_local_anchors_b;
    uint8_t *joint_collide_connected;
    float *joint_distance_lengths;
    float *joint_distance_impulses;
    sl_vec2 *joint_revolute_impulses;
    sl_vec2 *joint_linear_impulses;
    uint32_t *body_joint_heads;
    uint32_t *body_joint_counts;
    uint32_t *joint_edge_prevs;
    uint32_t *joint_edge_nexts;

    /* Complete transient records are consumed together by each scalar
     * solver stage, so AoS is the appropriate scratch granularity. */
    void *joint_constraints;

    /* Fixed diagnostic storage; never participates in physics decisions.
     * Cost: one step snapshot + two work counters + three uint32_t + bool and
     * ABI padding. Measured arm64 diagnostic cost: 208 bytes;
     * arena size and per-entity storage are unaffected. */
    sl_world_step_stats step_stats;
    sl_world_work work_total;
    sl_world_work step_work;
    uint32_t body_count_high;
    uint32_t contact_count_high;
    uint32_t joint_count_high;
    bool stats_stepping;

} sl_world_state;

bool sl_body_is_valid(const sl_world_state *world, sl_body_handle handle);
bool sl_joint_is_valid(const sl_world_state *world, sl_joint_handle handle);
void sl_joint_destroy(sl_world_state *world, sl_joint_handle handle);
#endif
