#ifndef SILK_EXAMPLE_POOL_H
#define SILK_EXAMPLE_POOL_H

#include <stdint.h>

#include "silk/world.h"

/* A small thread pool implementing sl_parallel_for_fn, for the benchmark and
 * the sandbox. It is example code, not part of silk: the engine never
 * creates threads, and a host can supply any implementation.
 *
 * thread_count counts the calling thread, which also works during a
 * parallel_for; 1 runs everything on the caller. Workers spin briefly
 * between dispatches, since a step issues hundreds of short ones, then
 * sleep until the next. Threads need POSIX; elsewhere the pool runs on the
 * calling thread only (sp_pool_thread_count reports 1). */
typedef struct sp_pool sp_pool;

sp_pool *sp_pool_create(uint32_t thread_count);
void sp_pool_destroy(sp_pool *pool);
uint32_t sp_pool_thread_count(const sp_pool *pool);

/* An sl_parallel_for_fn; parallel_context is the pool. Not reentrant: one
 * parallel_for at a time per pool. */
void sp_pool_parallel_for(sl_task_fn *task, uint32_t count, uint32_t grain,
                          void *task_context, void *parallel_context);

#endif /* SILK_EXAMPLE_POOL_H */
