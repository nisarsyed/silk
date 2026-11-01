#ifndef SILK_CORE_ARENA_H
#define SILK_CORE_ARENA_H

#include <stddef.h>

/* Init-time arena layout. A module lays out every array twice through
 * sl_arena_take: with base NULL to measure the total, then over one zeroed
 * allocation. Every array starts 16-byte aligned. */

static inline size_t sl_arena_align(size_t value)
{
    const size_t align = 16u;
    return (value + align - 1u) & ~(align - 1u);
}

/* Reserves count * size bytes at *cursor; with base NULL, only measures. */
static inline void *sl_arena_take(unsigned char *base, size_t *cursor,
                                  size_t count, size_t size)
{
    const size_t at = sl_arena_align(*cursor);
    *cursor = at + count * size;
    return (base != NULL) ? (void *)(base + at) : NULL;
}

#endif /* SILK_CORE_ARENA_H */
