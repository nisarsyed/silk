#ifndef SILK_RENDER_DRIVER_H
#define SILK_RENDER_DRIVER_H
#include "../../wasm/adapter.h"
#include <silk/world.h>

/* Separate developer driver: six minutes at 60 Hz bounds the longest study
 * interval (five minutes) plus one minute of setup/warm-up headroom. Host
 * protocol still rebuilds the world between disposable warm-up and measurement.
 * This does not change SL_BENCH_STEP_COUNT_MAX or the native quality profile.
 */
#define SL_RENDER_STEP_COUNT_MAX 21600u

typedef struct sl_render_study sl_render_study;
sl_render_study *sl_render_study_create(uint32_t fixture, uint32_t copies,
                                        uint32_t sleep_enabled,
                                        uint32_t step_limit);
void sl_render_study_destroy(sl_render_study *study);
/* Invalid/finished input rejects without a step. Each successful call performs
 * one binary32(1/60) step at the fixture's four substeps. No
 * clocks/allocations. Contact drops are accumulated exactly and exposed even
 * after a failed run.
 */
bool sl_render_study_step(sl_render_study *study);
/* Borrowed private ABI: these do not escape the JS study owner. The world is
 * also available to the co-located raylib candidate without a JS pose copy. */
sl_wasm_context *sl_render_study_adapter(sl_render_study *study);
sl_world *sl_render_study_world(sl_render_study *study);
/* fixture, copies, sleep, step_limit, completed_steps, B/C/J capacities. */
const uint32_t *sl_render_study_status(const sl_render_study *study);
uint64_t sl_render_study_drops(const sl_render_study *study);
size_t sl_render_study_bytes(void);
#endif
