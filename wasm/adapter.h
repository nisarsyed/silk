#ifndef SILK_WASM_ADAPTER_H
#define SILK_WASM_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Private binding ABI, not an installed engine header. A context owns its
 * world and fixed marshalling buffers. All pointers stay inside the wrapper.
 * Inputs/outputs are explicit arrays of binary32/uint32, never C struct bytes.
 * See protocol.md for field order. Calls are synchronous and non-reentrant.
 * Creating/freeing contexts and initializing worlds are the only allocators.
 */
typedef struct sl_wasm_context sl_wasm_context;
sl_wasm_context *sl_wasm_context_create(void);
void sl_wasm_context_destroy(sl_wasm_context *context);
size_t sl_wasm_context_bytes(void);
float *sl_wasm_input_f32(sl_wasm_context *context);
uint32_t *sl_wasm_input_u32(sl_wasm_context *context);
const float *sl_wasm_output_f32(const sl_wasm_context *context);
const uint32_t *sl_wasm_output_u32(const sl_wasm_context *context);
size_t sl_wasm_world_bytes(const sl_wasm_context *context);
bool sl_wasm_world_init(sl_wasm_context *context);
void sl_wasm_world_reset(sl_wasm_context *context);
void sl_wasm_world_dispose(sl_wasm_context *context);
bool sl_wasm_world_step(sl_wasm_context *context, float dt);
uint32_t sl_wasm_body_count(const sl_wasm_context *context);
bool sl_wasm_body_valid(const sl_wasm_context *context, uint32_t index,
                        uint32_t generation);
bool sl_wasm_body_create(sl_wasm_context *context);
bool sl_wasm_body_at(sl_wasm_context *context, uint32_t row);
bool sl_wasm_body_next(sl_wasm_context *context, uint32_t index,
                       uint32_t generation);
bool sl_wasm_body_read(sl_wasm_context *context, uint32_t index,
                       uint32_t generation);
bool sl_wasm_body_destroy(sl_wasm_context *context, uint32_t index,
                          uint32_t generation);
bool sl_wasm_body_set_shape(sl_wasm_context *context, uint32_t index,
                            uint32_t generation);
bool sl_wasm_body_set_position(sl_wasm_context *context, uint32_t index,
                               uint32_t generation, float x, float y);
bool sl_wasm_body_set_velocity(sl_wasm_context *context, uint32_t index,
                               uint32_t generation, float x, float y);
bool sl_wasm_body_apply_force(sl_wasm_context *context, uint32_t index,
                              uint32_t generation, float x, float y);
bool sl_wasm_body_apply_force_at_point(sl_wasm_context *context, uint32_t index,
                                       uint32_t generation, float fx, float fy,
                                       float px, float py);
bool sl_wasm_body_set_angle(sl_wasm_context *context, uint32_t index,
                            uint32_t generation, float value);
bool sl_wasm_body_set_angular_velocity(sl_wasm_context *context, uint32_t index,
                                       uint32_t generation, float value);
bool sl_wasm_body_set_mass(sl_wasm_context *context, uint32_t index,
                           uint32_t generation, float value);
bool sl_wasm_body_set_friction(sl_wasm_context *context, uint32_t index,
                               uint32_t generation, float value);
bool sl_wasm_body_set_restitution(sl_wasm_context *context, uint32_t index,
                                  uint32_t generation, float value);
bool sl_wasm_body_apply_torque(sl_wasm_context *context, uint32_t index,
                               uint32_t generation, float value);
bool sl_wasm_body_wake(sl_wasm_context *context, uint32_t index,
                       uint32_t generation);
void sl_wasm_shape_none(sl_wasm_context *context);
bool sl_wasm_shape_circle(sl_wasm_context *context, float radius);
bool sl_wasm_shape_box(sl_wasm_context *context, float x, float y);
bool sl_wasm_shape_polygon(sl_wasm_context *context, uint32_t count);
bool sl_wasm_shape_load(sl_wasm_context *context);
void sl_wasm_shape_read(sl_wasm_context *context);
void sl_wasm_shape_mass(sl_wasm_context *context);
bool sl_wasm_shape_aabb(sl_wasm_context *context);
bool sl_wasm_shape_contains(sl_wasm_context *context);
bool sl_wasm_shape_ray(sl_wasm_context *context);

#endif
