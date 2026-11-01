#ifndef SILK_CORE_PARALLEL_H
#define SILK_CORE_PARALLEL_H

#include <stddef.h>
#include <stdint.h>

#include "silk/parallel.h"

/* Smallest ranges worth handing to a thread (sl_parallel_for_fn's grain),
 * sized so a range is a few microseconds of work, several times a spinning
 * pool's dispatch cost.
 *
 * Vertex block solves cost about 150 ns each, so 32 is about 5 us. A
 * 64 x 64 curtain has colors of a few hundred vertices: a grain of 128 left
 * most of 8 threads idle (1.85x), while 16 to 64 all gave about 2.15x.
 * Plain per-vertex loops cost a few ns per vertex: at a grain of 32 they
 * cost more to dispatch than to run. */
#define SL_PARALLEL_GRAIN_BLOCK 32u
#define SL_PARALLEL_GRAIN_LOOP 2048u

/* Runs task over [0, count) through parallel_for, or on the calling thread
 * when it is NULL. The host decides how to split, with grain as the hint
 * (a pool runs a count under its grain inline). Tasks write only data
 * owned by their own indices, so the result never depends on the
 * partition. */
static inline void sl_parallel_run(sl_parallel_for_fn *parallel_for,
                                   void *parallel_context, sl_task_fn *task,
                                   uint32_t count, uint32_t grain,
                                   void *context)
{
    if (count == 0u) {
        return;
    }
    if (parallel_for != NULL) {
        parallel_for(task, count, grain, context, parallel_context);
    } else {
        task(0u, count, context);
    }
}

#endif /* SILK_CORE_PARALLEL_H */
