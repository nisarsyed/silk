#ifndef SILK_RENDER_SCENE_H
#define SILK_RENDER_SCENE_H
#include <silk/world.h>

/* Comparison-only setup, using the shared C fixtures and public world API.
 * Accepts pyramid/rain/chains IDs (0/1/3) and copy counts 1/2/4/8/16.
 * Copies are inserted in increasing order at x = 32*(i-(copies-1)/2).
 * Capacities multiply exactly; every copy reuses the original seeded fixture.
 * world must be a zeroed/uninitialized shell. Invalid arguments leave outputs
 * unchanged; allocation/setup failure destroys the new world and preserves
 * config. Success returns resolved capacities in config. No retained helper
 * storage: the caller owns the world. All allocations occur during setup.
 */
bool sl_render_scene_build(sl_world *world, sl_world_config *config,
                           uint32_t fixture, uint32_t copies,
                           bool sleep_enabled);
#endif
