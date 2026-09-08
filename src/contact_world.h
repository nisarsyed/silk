#ifndef SILK_CONTACT_WORLD_H
#define SILK_CONTACT_WORLD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "world_internal.h"

uint32_t sl_contact_pair_capacity(uint32_t contact_capacity);
size_t sl_contact_world_memory_bytes(uint32_t body_capacity,
                                     uint32_t contact_capacity);
size_t sl_contact_world_memory_layout(uint32_t body_capacity,
                                      uint32_t contact_capacity,
                                      sl_world_memory_breakdown *out);
bool sl_contact_world_init(sl_world_state *world, void *memory,
                           size_t memory_bytes);
void sl_contact_world_reset(sl_world_state *world);

bool sl_contact_body_create(sl_world_state *world, uint32_t dense,
                            uint32_t slot);
void sl_contact_body_destroy(sl_world_state *world, uint32_t dense,
                             uint32_t slot);
bool sl_contact_body_update(sl_world_state *world, uint32_t dense,
                            uint32_t slot);
void sl_contact_body_mark_moved(sl_world_state *world, uint32_t slot);
void sl_contact_pair_destroy(sl_world_state *world, uint32_t slot_a,
                             uint32_t slot_b);

void sl_contact_step_begin(sl_world_state *world);
void sl_contact_step_end(sl_world_state *world);

#endif /* SILK_CONTACT_WORLD_H */
