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
/* Check evolving public state: transforms/velocities, proxy bounds, live
 * contact floats and joint impulses. Failure is latched; later steps reject.
 * step performs this check after each executed step (inside measured CPU
 * work), retaining its completed-step/drop counters even on numeric failure.
 * Initialization/shape/configuration inputs already use engine validation. */
bool sl_render_study_validate(sl_render_study *study);
bool sl_render_study_failed(const sl_render_study *study);
/* Borrowed private ABI: these do not escape the JS study owner. The world is
 * also available to the co-located raylib candidate without a JS pose copy. */
sl_wasm_context *sl_render_study_adapter(sl_render_study *study);
sl_world *sl_render_study_world(sl_render_study *study);
/* Write 16 bytes per body directly into a co-located renderer's pose buffer,
 * in packed row order: x/y/cos/sin. Caller supplies 4*count floats; count must
 * equal the world body count. Invalid inputs leave output untouched. No
 * allocation, snapshot packing, trigonometry or simulation mutation. */
bool sl_render_study_poses(const sl_render_study *study, float *poses,
                           uint32_t count);
/* fixture, copies, sleep, step_limit, completed_steps, B/C/J capacities,
 * resolved substeps, seed. Settings: gravity x/y; linear/angular drag; speed
 * cap; contact hertz/damping/push/restitution threshold; joint hertz/damping;
 * sleep speed/angular speed/time; friction/restitution; fixed dt; camera
 * left/bottom/right/top. */
const uint32_t *sl_render_study_status(const sl_render_study *study);
const float *sl_render_study_settings(const sl_render_study *study);
uint64_t sl_render_study_drops(const sl_render_study *study);
size_t sl_render_study_bytes(void);
/* Diagnostic arrays: four proxy-AABB float columns keyed by body slot, then
 * ray fraction/point x/y/normal x/y. Words: per-slot point/AABB/ray/proxy bits
 * (1/2/4/8), then point count, AABB count, ray-hit flag. All queries use the
 * fixed camera center and SL_QUERY_ALL with full body-capacity output.
 * Refresh is const with respect to physics and allocates no storage. */
bool sl_render_study_diagnostics(sl_render_study *study);
const float *sl_render_study_diagnostic_f32(const sl_render_study *study);
const uint32_t *sl_render_study_diagnostic_u32(const sl_render_study *study);
size_t sl_render_study_diagnostic_bytes(const sl_render_study *study);

/* Setup-only render tiers: the original one-copy awake rain at exactly 120
 * steps, repeated in dynamic-row order into 256..65536 power-of-two instances.
 * Queries use public C shape predicates on translated instances, all types,
 * with one output slot per instance. Static/trimmed shapes are not queried.
 * Proxy bounds are the translated frozen source bounds. Ray ties retain the
 * first instance in draw order. The independent copied result owns no world
 * pointers and remains valid until destroy; creation never steps/mutates it.
 * Float/word layouts match the diagnostic columns above, keyed by instance.
 * Temporary creation memory and O(instances) query work belong to setup.
 */
typedef struct sl_render_frozen sl_render_frozen;
sl_render_frozen *sl_render_frozen_create(sl_render_study *study,
                                          uint32_t instances);
void sl_render_frozen_destroy(sl_render_frozen *frozen);
uint32_t sl_render_frozen_count(const sl_render_frozen *frozen);
const float *sl_render_frozen_f32(const sl_render_frozen *frozen);
const uint32_t *sl_render_frozen_u32(const sl_render_frozen *frozen);
size_t sl_render_frozen_bytes(const sl_render_frozen *frozen);
#endif
