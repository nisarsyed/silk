#ifndef SILK_BENCH_QUALITY_H
#define SILK_BENCH_QUALITY_H
#include <silk/world.h>
/* Independent current-transform geometry, not cached manifold separation. */
float sl_bench_penetration(const sl_shape *a, sl_transform ta,
                           const sl_shape *b, sl_transform tb);
uint64_t sl_bench_digest(const sl_world *world, const sl_body_handle *bodies,
                         uint32_t count);
#endif
