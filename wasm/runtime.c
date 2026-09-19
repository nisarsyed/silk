#include "runtime.h"
#include <malloc.h>
#include <stdint.h>

/* Addresses of wasm-ld linker symbols, not C objects to dereference. */
extern unsigned char __heap_base;
extern unsigned char __data_end;
uint32_t sl_wasm_heap_base(void)
{
    return (uint32_t)(uintptr_t)&__heap_base;
}
uint32_t sl_wasm_static_end(void)
{
    return (uint32_t)(uintptr_t)&__data_end;
}
uint32_t sl_wasm_allocator_used(void)
{
    return (uint32_t)mallinfo().uordblks;
}
