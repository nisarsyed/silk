#ifndef SILK_PARALLEL_H
#define SILK_PARALLEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Optional parallel execution. silk never creates threads; a world given a
 * parallel_for (sl_world_config) runs its independent work through it.
 *
 * sl_world_step calls parallel_for(task, count, grain, task_context,
 * parallel_context) and expects it to call task(begin, end, task_context)
 * on disjoint ranges that together cover [0, count) exactly once, in any
 * order and on any threads, and to return only after every call has
 * finished, with their writes visible to the caller. grain is a hint: the
 * smallest range worth running on its own. Results are bitwise identical
 * for every partition, thread count, and order, including running the whole
 * range on the calling thread. A NULL parallel_for runs everything on the
 * calling thread. */
typedef void sl_task_fn(uint32_t begin, uint32_t end, void *task_context);
typedef void sl_parallel_for_fn(sl_task_fn *task, uint32_t count,
                                uint32_t grain, void *task_context,
                                void *parallel_context);

#ifdef __cplusplus
}
#endif

#endif /* SILK_PARALLEL_H */
