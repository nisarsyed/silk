#ifndef SILK_TEST_REPLAY_H
#define SILK_TEST_REPLAY_H
#include "silk/world.h"
/* Test-only semantic comparison. Values hold integer values or binary32 bits.
 * On success out is untouched. Context is supplied by the fixture runner. */
typedef struct sl_replay_mismatch {
    const char *fixture;
    uint32_t seed;
    uint32_t operation;
    uint32_t entity;
    const char *field_name;
    uint64_t value_a;
    uint64_t value_b;
} sl_replay_mismatch;
bool sl_replay_compare(const sl_world *a, const sl_world *b,
                       sl_replay_mismatch *out);
/* Compare and print the first mismatch with fixture context on failure. */
bool sl_replay_check(const sl_world *a, const sl_world *b, const char *fixture,
                     uint32_t seed, uint32_t operation);
#endif
