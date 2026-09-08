#ifndef SILK_QUERY_H
#define SILK_QUERY_H
#include "silk/world.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Zero selects every body type; any unknown bit is rejected. */
#define SL_QUERY_DYNAMIC UINT32_C(1)
#define SL_QUERY_KINEMATIC UINT32_C(2)
#define SL_QUERY_STATIC UINT32_C(4)
#define SL_QUERY_ALL (SL_QUERY_DYNAMIC | SL_QUERY_KINEMATIC | SL_QUERY_STATIC)

/* Ray origin and endpoint components must lie within this world-frame bound.
 * It includes the largest supported shape beyond the body-center limit and
 * bounds intermediate shape-ray arithmetic. Translation length > SL_EPSILON. */
#define SL_QUERY_RAY_COORDINATE_MAX (SL_POSITION_ABS_MAX + SL_SHAPE_EXTENT_MAX)

typedef struct sl_query_result {
    uint32_t count; /* total matches, including those that did not fit */
    bool truncated;
} sl_query_result;

typedef struct sl_query_ray_result {
    bool hit;
    sl_body_handle body;
    sl_ray_hit geometry;
} sl_query_ray_result;

/* All queries require an initialized world. They reuse private scratch but
 * preserve physics, diagnostics and contact/shape snapshot lifetimes.
 * Same-world queries are non-reentrant and cannot run concurrently with other
 * operations. Shapeless bodies are skipped; activation state is not a query
 * filter.
 *
 * False leaves every output untouched for invalid geometry, unknown mask bits,
 * missing required outputs, or capacity > SL_BODY_COUNT_MAX. Buffer queries
 * permit NULL bodies only when capacity is zero. Outputs must not overlap each
 * other or world-owned storage. Results contain ascending body slots;
 * truncation writes that order's prefix. Caller-owned handles follow body
 * lifetime rules. Finite point/AABB coordinates have no additional bound. */
bool sl_world_query_aabb(const sl_world *world, sl_aabb bounds,
                         uint32_t type_mask, sl_body_handle *bodies,
                         uint32_t capacity, sl_query_result *result);
bool sl_world_query_point(const sl_world *world, sl_vec2 point,
                          uint32_t type_mask, sl_body_handle *bodies,
                          uint32_t capacity, sl_query_result *result);

/* Inclusive tight shape-AABB overlap above; point containment includes shape
 * boundaries. Rays use sl_shape_ray_cast conventions: origins inside/on each
 * shape miss that shape, grazing hits, and fraction is in [0,1]. Exact fraction
 * ties choose the lowest body slot. A valid miss returns true and zeroes
 * result. Malformed, too-short or out-of-bound rays return false without output
 * writes. */
bool sl_world_query_ray(const sl_world *world, sl_ray ray, uint32_t type_mask,
                        sl_query_ray_result *result);
#ifdef __cplusplus
}
#endif
#endif /* SILK_QUERY_H */
