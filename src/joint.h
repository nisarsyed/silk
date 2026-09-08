#ifndef SILK_JOINT_INTERNAL_H
#define SILK_JOINT_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "world_internal.h"

#define SL_JOINT_EDGE_NONE UINT32_MAX

size_t sl_joint_memory_layout(uint32_t body_capacity, uint32_t joint_capacity,
                              size_t *payload);
size_t sl_joint_memory_bytes(uint32_t body_capacity, uint32_t joint_capacity);
bool sl_joint_world_init(sl_world_state *world, void *memory,
                         size_t memory_bytes);
void sl_joint_world_reset(sl_world_state *world);
void sl_joint_body_destroy(sl_world_state *world, uint32_t body_slot);
void sl_joint_body_cache_clear(sl_world_state *world, uint32_t body_slot);

/* True unless at least one live joint on the pair suppresses collision. */
bool sl_joint_pair_should_collide(const sl_world_state *world,
                                  uint32_t body_slot_a, uint32_t body_slot_b);

void sl_joint_prepare(sl_world_state *world, float h, float inverse_h);
void sl_joint_warm_start(sl_world_state *world);
void sl_joint_solve(sl_world_state *world, bool use_bias);
void sl_joint_store(sl_world_state *world);

#endif /* SILK_JOINT_INTERNAL_H */
