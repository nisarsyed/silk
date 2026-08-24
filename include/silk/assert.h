#ifndef SILK_ASSERT_H
#define SILK_ASSERT_H

#include <assert.h>

/* Fails loudly if cond is false. Use to enforce arguments, returns,
 * and invariants — check both the expected and the unexpected.
 * Compiled out when NDEBUG is defined (release builds). */
#define SL_ASSERT(cond) assert(cond)

#endif /* SILK_ASSERT_H */
