#ifndef SILK_BENCH_WASM_DRIVER_H
#define SILK_BENCH_WASM_DRIVER_H
#include "adapter.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Developer benchmark ABI, separate from the distributed runtime package.
 * A driver owns its adapter/world and fixed fixture/reference/report storage.
 * The adapter pointer is private borrowed state valid until driver destruction.
 */
typedef struct sl_wasm_bench sl_wasm_bench;
sl_wasm_bench *sl_wasm_bench_create(uint32_t fixture, uint32_t sleep_enabled,
                                    uint32_t warmup, uint32_t steps);
void sl_wasm_bench_destroy(sl_wasm_bench *bench);
sl_wasm_context *sl_wasm_bench_adapter(sl_wasm_bench *bench);
size_t sl_wasm_bench_bytes(void);
/* One bounded iteration is prepare -> mutate -> step -> sample. Invalid order
 * returns false without mutation. Timers belong to the host. Step includes
 * only phase validation, sl_world_step, and the phase write; drops/quality are
 * sampled afterwards. Fixture mutation and reference capture are separate.
 */
bool sl_wasm_bench_prepare(sl_wasm_bench *bench);
bool sl_wasm_bench_mutate(sl_wasm_bench *bench);
bool sl_wasm_bench_step(sl_wasm_bench *bench);
bool sl_wasm_bench_sample(sl_wasm_bench *bench);
bool sl_wasm_bench_report(sl_wasm_bench *bench);
const uint32_t *sl_wasm_bench_words(const sl_wasm_bench *bench);
const double *sl_wasm_bench_quality(const sl_wasm_bench *bench);
const uint64_t *sl_wasm_bench_counters(const sl_wasm_bench *bench);
#endif
