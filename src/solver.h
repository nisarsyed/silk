#ifndef SILK_SOLVER_H
#define SILK_SOLVER_H

#include <stdbool.h>
#include <stddef.h>

#include "silk/world.h"

size_t sl_solver_memory_bytes(uint32_t contact_capacity);
bool sl_solver_init(sl_world *world, void *memory, size_t memory_bytes);

void sl_solver_prepare(sl_world *world, float h, float inverse_h);
void sl_solver_warm_start(sl_world *world);
void sl_solver_solve(sl_world *world, float inverse_h, bool use_bias);
void sl_solver_restitution(sl_world *world);
void sl_solver_store(sl_world *world);

#endif /* SILK_SOLVER_H */
