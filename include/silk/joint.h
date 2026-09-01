#ifndef SILK_JOINT_H
#define SILK_JOINT_H

#include <stdbool.h>
#include <stdint.h>

#include "silk/body.h"
#include "silk/math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds the opt-in joint pool and its two adjacency edges per slot. */
#define SL_JOINT_COUNT_MAX 65536u

/* Joint handles use the same slot/generation lifetime contract as bodies.
 * A zero generation is never issued and therefore encodes null. */
typedef struct sl_joint_handle {
    uint32_t index;
    uint32_t generation;
} sl_joint_handle;

static inline sl_joint_handle sl_joint_handle_null(void)
{
    sl_joint_handle handle = { UINT32_MAX, 0u };
    return handle;
}

static inline bool sl_joint_handle_is_null(sl_joint_handle handle)
{
    return handle.generation == 0u;
}

typedef enum sl_joint_kind {
    SL_JOINT_DISTANCE = 0,
    SL_JOINT_REVOLUTE = 1
} sl_joint_kind;

typedef struct sl_distance_joint_desc {
    /* World units. World creation accepts values from its linear collision
     * slop through twice its absolute position bound. */
    float length;
} sl_distance_joint_desc;

typedef struct sl_joint_desc {
    sl_joint_kind kind;
    sl_body_handle body_a;
    sl_body_handle body_b;
    /* Body-local coordinates. Each component is bounded by
     * SL_SHAPE_EXTENT_MAX. */
    sl_vec2 local_anchor_a;
    sl_vec2 local_anchor_b;
    /* False suppresses collision while any such joint connects the pair. */
    bool collide_connected;
    union {
        sl_distance_joint_desc distance;
    };
} sl_joint_desc;

#ifdef __cplusplus
}
#endif

#endif /* SILK_JOINT_H */
