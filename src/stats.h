#ifndef SILK_STATS_INTERNAL_H
#define SILK_STATS_INTERNAL_H
#include "silk/world.h"
static inline uint64_t sl_stats_sum(uint64_t a, uint64_t b)
{
    return b > UINT64_MAX - a ? UINT64_MAX : a + b;
}
/* Evaluates amount once. Step-local work is independent of a saturated
 * cumulative counter, so a long-lived world's last step stays useful. */
#define SL_WORK_ADD(world, field, amount)                                      \
    do {                                                                       \
        const uint64_t sl_work_amount = (uint64_t)(amount);                    \
        (world)->stats.cumulative.field =                                      \
            sl_stats_sum((world)->stats.cumulative.field, sl_work_amount);     \
        if ((world)->stats_stepping) {                                         \
            (world)->step_work.field =                                         \
                sl_stats_sum((world)->step_work.field, sl_work_amount);        \
        }                                                                      \
    } while (false)
#endif
