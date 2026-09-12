#ifndef SILK_WASM_RUNTIME_H
#define SILK_WASM_RUNTIME_H
#include <stdint.h>
/* Emscripten-only linear-memory diagnostics, outside portable adapter logic. */
uint32_t sl_wasm_heap_base(void);
uint32_t sl_wasm_static_end(void);
uint32_t sl_wasm_allocator_used(void);
#endif
