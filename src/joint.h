#ifndef SILK_JOINT_INTERNAL_H
#define SILK_JOINT_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "silk/world.h"

#define SL_JOINT_EDGE_NONE UINT32_MAX

size_t sl_joint_memory_bytes(uint32_t body_capacity, uint32_t joint_capacity);
bool sl_joint_world_init(sl_world *world, void *memory, size_t memory_bytes);
void sl_joint_world_reset(sl_world *world);
void sl_joint_body_destroy(sl_world *world, uint32_t body_slot);
void sl_joint_body_cache_clear(sl_world *world, uint32_t body_slot);

/* True unless at least one live joint on the pair suppresses collision. */
bool sl_joint_pair_should_collide(const sl_world *world, uint32_t body_slot_a,
                                  uint32_t body_slot_b);

void sl_joint_prepare(sl_world *world, float h, float inverse_h);
void sl_joint_warm_start(sl_world *world);
void sl_joint_solve(sl_world *world, bool use_bias);
void sl_joint_store(sl_world *world);

#endif /* SILK_JOINT_INTERNAL_H */
