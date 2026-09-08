#ifndef SILK_TEST_REPLAY_H
#define SILK_TEST_REPLAY_H
#include "silk/world.h"
/* Test-only replay boundary for a fixed platform, build and call sequence.
 * Compare float representations and reject non-finite live state. Cover
 * configuration, generations/packed ownership/free lists, live body/shape
 * fields, ordered contacts and joint descriptors/caches, pair occupancy,
 * moved/retry queues, tree topology/free links, joint adjacency, retained
 * island membership and ordered ranges. Sleep policy, flags, quiet-step/travel
 * accumulators and fixed timestep are included. Exclude addresses, padding,
 * inactive payloads, unused queue tails and scratch overwritten before use.
 * Keep private-storage migrations inside this helper, not in fixtures.
 *
 * Values hold integers or binary32 bits; the first mismatch wins. On success
 * out is untouched. The fixture runner supplies seed/operation context. */
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
