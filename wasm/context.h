#ifndef SILK_WASM_CONTEXT_H
#define SILK_WASM_CONTEXT_H

#include "adapter.h"
#include <silk/query.h>
#include <silk/step.h>

/* Adapter-private ownership; no private engine headers or storage offsets. */
struct sl_wasm_context {
    sl_world world;
    sl_shape shape;
    sl_world_config config;
    float input_f32[32];
    uint32_t input_u32[16];
    float output_f32[64];
    uint32_t output_u32[32];
    uint64_t work[26];
    double dropped_time;
    void *buffers;
    size_t buffer_bytes;
    float *body_f32, *geometry_f32, *contact_f32, *joint_f32;
    uint32_t *body_u32, *geometry_u32, *geometry_generation;
    uint32_t *query_u32, *contact_u32, *joint_u32;
    sl_body_handle *query_handles;
};

size_t sl_wasm_storage_bytes(const sl_world_config *config);
bool sl_wasm_storage_init(sl_wasm_context *context);
void sl_wasm_storage_dispose(sl_wasm_context *context);
/* Used by stats marshalling and native large-counter boundary tests. */
void sl_wasm_work_pack(uint64_t *out, const sl_world_work *work);
void sl_wasm_contact_pack(float *floats, uint32_t *words, uint32_t stride,
                          uint32_t row, const sl_contact *contact);
void sl_wasm_joint_pack(float *floats, uint32_t *words, uint32_t stride,
                        uint32_t row, const sl_world *world,
                        sl_joint_handle handle);
#endif
