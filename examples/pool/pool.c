#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif

#include "pool.h"

#include <stdbool.h>
#include <stdlib.h>

/* Atomic loads a worker spins for the next dispatch before sleeping, about
 * a quarter of a millisecond. It must outlast the serial work between
 * dispatches (drag and edge damping take about 100 us per substep on a
 * 64 x 64 curtain): at 20000 loads, workers fell asleep every substep and
 * most of their time went to wake-ups. Idle workers still sleep within a
 * millisecond, so a paused sandbox does not keep cores busy. */
#define SP_SPIN_MAX 200000u

/* Ranges per thread for one dispatch: more than one evens out uneven
 * ranges and slower cores, at a small claiming cost. */
#define SP_RANGES_PER_THREAD 4u

#if defined(_WIN32)

struct sp_pool {
    uint32_t thread_count;
};

sp_pool *sp_pool_create(uint32_t thread_count)
{
    (void)thread_count;
    sp_pool *pool = calloc(1u, sizeof(*pool));
    if (pool != NULL) {
        pool->thread_count = 1u;
    }
    return pool;
}

void sp_pool_destroy(sp_pool *pool)
{
    free(pool);
}

uint32_t sp_pool_thread_count(const sp_pool *pool)
{
    return pool->thread_count;
}

void sp_pool_parallel_for(sl_task_fn *task, uint32_t count, uint32_t grain,
                          void *task_context, void *parallel_context)
{
    (void)grain;
    (void)parallel_context;
    task(0u, count, task_context);
}

#else

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>

/* Upper bound on threads, so a dispatch's range count fits in 16 bits of
 * the claim word. */
#define SP_THREAD_COUNT_MAX 256u

/* The claim word packs the dispatch generation (bits 32-63), its range
 * count (16-31), and the next unclaimed range (0-15). A worker claims a
 * range with one compare-and-swap that also checks the generation, so a
 * worker still finishing one dispatch can never claim a range of the next.
 * The caller therefore waits only for its ranges, never for every thread. */
static uint32_t sp_generation(uint64_t word)
{
    return (uint32_t)(word >> 32);
}

struct sp_pool {
    uint32_t thread_count;
    pthread_t *threads;
    /* The current dispatch. Written before the claim word is published and
     * read only by a worker holding a claimed range, which keeps the caller
     * from starting the next dispatch until the range is done. */
    sl_task_fn *task;
    void *task_context;
    uint32_t count;
    uint32_t range; /* elements per claimed range */
    _Atomic uint64_t claim;
    atomic_uint done; /* ranges of the current dispatch finished */
    atomic_uint sleepers;
    atomic_bool stop;
    pthread_mutex_t lock;
    pthread_cond_t wake;
};

/* Claims and runs ranges of dispatch `generation` until none remain. */
static void sp_pool_work(sp_pool *pool, uint32_t generation)
{
    uint64_t word = atomic_load_explicit(&pool->claim, memory_order_acquire);
    for (;;) {
        const uint32_t next = (uint32_t)(word & 0xFFFFu);
        const uint32_t range_count = (uint32_t)((word >> 16) & 0xFFFFu);
        if (sp_generation(word) != generation || next >= range_count) {
            return;
        }
        if (!atomic_compare_exchange_weak_explicit(
                &pool->claim, &word, word + 1u, memory_order_acq_rel,
                memory_order_acquire)) {
            continue; /* word now holds the current value */
        }
        const uint32_t begin = next * pool->range;
        const uint32_t end = (pool->count - begin < pool->range)
                                 ? pool->count
                                 : begin + pool->range;
        pool->task(begin, end, pool->task_context);
        atomic_fetch_add_explicit(&pool->done, 1u, memory_order_release);
        word = atomic_load_explicit(&pool->claim, memory_order_acquire);
    }
}

static void *sp_pool_worker(void *argument)
{
    sp_pool *pool = argument;
    uint32_t seen = sp_generation(atomic_load(&pool->claim));
    for (;;) {
        uint32_t now = seen;
        for (uint32_t spin = 0; spin < SP_SPIN_MAX; spin++) {
            now = sp_generation(
                atomic_load_explicit(&pool->claim, memory_order_acquire));
            if (now != seen ||
                atomic_load_explicit(&pool->stop, memory_order_relaxed)) {
                break;
            }
        }
        if (now == seen && !atomic_load(&pool->stop)) {
            /* Sleep. The dispatcher publishes the claim word before it reads
             * sleepers, and the worker registers before it re-reads the
             * word under the lock, so no wake-up is lost. */
            pthread_mutex_lock(&pool->lock);
            atomic_fetch_add(&pool->sleepers, 1u);
            while (sp_generation(atomic_load(&pool->claim)) == seen &&
                   !atomic_load(&pool->stop)) {
                pthread_cond_wait(&pool->wake, &pool->lock);
            }
            atomic_fetch_sub(&pool->sleepers, 1u);
            pthread_mutex_unlock(&pool->lock);
            now = sp_generation(atomic_load(&pool->claim));
        }
        if (atomic_load(&pool->stop)) {
            return NULL;
        }
        seen = now;
        sp_pool_work(pool, now);
    }
}

sp_pool *sp_pool_create(uint32_t thread_count)
{
    if (thread_count < 1u) {
        thread_count = 1u;
    }
    if (thread_count > SP_THREAD_COUNT_MAX) {
        thread_count = SP_THREAD_COUNT_MAX;
    }
    sp_pool *pool = calloc(1u, sizeof(*pool));
    if (pool == NULL) {
        return NULL;
    }
    pool->threads = calloc(thread_count, sizeof(*pool->threads));
    if (pool->threads == NULL || pthread_mutex_init(&pool->lock, NULL) != 0) {
        free(pool->threads);
        free(pool);
        return NULL;
    }
    if (pthread_cond_init(&pool->wake, NULL) != 0) {
        pthread_mutex_destroy(&pool->lock);
        free(pool->threads);
        free(pool);
        return NULL;
    }
    atomic_init(&pool->claim, 0u);
    atomic_init(&pool->done, 0u);
    atomic_init(&pool->sleepers, 0u);
    atomic_init(&pool->stop, false);
    pool->thread_count = 1u;
    for (uint32_t t = 1u; t < thread_count; t++) {
        if (pthread_create(&pool->threads[t], NULL, sp_pool_worker, pool) !=
            0) {
            break; /* run with the threads that started */
        }
        pool->thread_count++;
    }
    return pool;
}

void sp_pool_destroy(sp_pool *pool)
{
    if (pool == NULL) {
        return;
    }
    pthread_mutex_lock(&pool->lock);
    atomic_store(&pool->stop, true);
    pthread_cond_broadcast(&pool->wake);
    pthread_mutex_unlock(&pool->lock);
    for (uint32_t t = 1u; t < pool->thread_count; t++) {
        pthread_join(pool->threads[t], NULL);
    }
    pthread_cond_destroy(&pool->wake);
    pthread_mutex_destroy(&pool->lock);
    free(pool->threads);
    free(pool);
}

uint32_t sp_pool_thread_count(const sp_pool *pool)
{
    return pool->thread_count;
}

void sp_pool_parallel_for(sl_task_fn *task, uint32_t count, uint32_t grain,
                          void *task_context, void *parallel_context)
{
    sp_pool *pool = parallel_context;
    if (pool->thread_count <= 1u || count <= grain) {
        task(0u, count, task_context);
        return;
    }
    const uint32_t target = pool->thread_count * SP_RANGES_PER_THREAD;
    uint32_t range = (count + target - 1u) / target;
    if (range < grain) {
        range = grain;
    }
    const uint32_t range_count = (count + range - 1u) / range;
    pool->task = task;
    pool->task_context = task_context;
    pool->count = count;
    pool->range = range;
    atomic_store_explicit(&pool->done, 0u, memory_order_relaxed);
    const uint32_t generation = sp_generation(atomic_load_explicit(
                                    &pool->claim, memory_order_relaxed)) +
                                1u;
    /* Publish (seq_cst also orders it before the sleepers read below). */
    atomic_store(&pool->claim,
                 ((uint64_t)generation << 32) | ((uint64_t)range_count << 16));
    if (atomic_load(&pool->sleepers) > 0u) {
        pthread_mutex_lock(&pool->lock);
        pthread_cond_broadcast(&pool->wake);
        pthread_mutex_unlock(&pool->lock);
    }
    sp_pool_work(pool, generation);
    while (atomic_load_explicit(&pool->done, memory_order_acquire) <
           range_count) {
        sched_yield();
    }
}

#endif
