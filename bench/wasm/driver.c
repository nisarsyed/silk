#include "driver.h"
#include "../fixtures.h"
#include "../quality.h"
#include "context.h"
#include <stdlib.h>

enum { PHASE_READY, PHASE_PREPARED, PHASE_MUTATED, PHASE_STEPPED };
struct sl_wasm_bench {
    sl_wasm_context adapter;
    sl_bench_scene scene;
    sl_bench_quality quality;
    uint64_t drops;
    uint32_t fixture, warmup, steps, index, phase;
    bool failed;
    uint32_t words[13];
    double values[25];
    uint64_t counters[2];
};
sl_wasm_bench *sl_wasm_bench_create(uint32_t fixture, uint32_t sleep_enabled,
                                    uint32_t warmup, uint32_t steps)
{
    if (fixture >= SL_BENCH_FIXTURE_COUNT || sleep_enabled > 1u ||
        warmup > SL_BENCH_STEP_COUNT_MAX || steps == 0u ||
        steps > SL_BENCH_STEP_COUNT_MAX) {
        return NULL;
    }
    sl_wasm_bench *b = calloc(1u, sizeof(*b));
    if (b == NULL) {
        return NULL;
    }
    if (!sl_bench_scene_build(&b->scene, &b->adapter.world, fixture,
                              sleep_enabled != 0u)) {
        free(b);
        return NULL;
    }
    b->adapter.config = b->scene.config;
    b->adapter.config.contact_capacity =
        sl_world_contact_capacity(&b->adapter.world);
    if (!sl_wasm_storage_init(&b->adapter)) {
        sl_wasm_world_dispose(&b->adapter);
        free(b);
        return NULL;
    }
    b->fixture = fixture;
    b->warmup = warmup;
    b->steps = steps;
    return b;
}
void sl_wasm_bench_destroy(sl_wasm_bench *b)
{
    if (b != NULL) {
        sl_wasm_world_dispose(&b->adapter);
        free(b);
    }
}
sl_wasm_context *sl_wasm_bench_adapter(sl_wasm_bench *b)
{
    return b != NULL ? &b->adapter : NULL;
}
size_t sl_wasm_bench_bytes(void)
{
    return sizeof(sl_wasm_bench);
}
static bool phase_valid(const sl_wasm_bench *b, uint32_t phase)
{
    return b != NULL && !b->failed && b->index < b->warmup + b->steps &&
           b->phase == phase;
}
static uint32_t window_start(const sl_wasm_bench *b)
{
    const uint32_t window =
        b->steps < SL_BENCH_QUALITY_WINDOW ? b->steps : SL_BENCH_QUALITY_WINDOW;
    return b->warmup + b->steps - window;
}
bool sl_wasm_bench_prepare(sl_wasm_bench *b)
{
    if (!phase_valid(b, PHASE_READY)) {
        return false;
    }
    if (b->index == window_start(b)) {
        sl_bench_reference_capture(&b->scene);
    }
    b->phase = PHASE_PREPARED;
    return true;
}
bool sl_wasm_bench_mutate(sl_wasm_bench *b)
{
    if (!phase_valid(b, PHASE_PREPARED)) {
        return false;
    }
    if (!sl_bench_scene_mutate(&b->scene, b->index)) {
        b->failed = true;
        return false;
    }
    b->phase = PHASE_MUTATED;
    return true;
}
bool sl_wasm_bench_step(sl_wasm_bench *b)
{
    if (!phase_valid(b, PHASE_MUTATED)) {
        return false;
    }
    sl_world_step(&b->adapter.world, SL_BENCH_TIMESTEP);
    b->phase = PHASE_STEPPED;
    return true;
}
bool sl_wasm_bench_sample(sl_wasm_bench *b)
{
    if (!phase_valid(b, PHASE_STEPPED)) {
        return false;
    }
    b->drops += sl_world_contact_drop_count(&b->adapter.world);
    if (b->index >= b->warmup &&
        !sl_bench_quality_sample(&b->scene, &b->quality,
                                 b->index >= window_start(b))) {
        b->failed = true;
        return false;
    }
    ++b->index;
    b->phase = PHASE_READY;
    return true;
}
bool sl_wasm_bench_report(sl_wasm_bench *b)
{
    if (b == NULL) {
        return false;
    }
    const sl_bench_quality *q = &b->quality;
    const sl_vec2 gravity = sl_world_get_gravity(&b->adapter.world);
    const double values[] = {
        (double)q->penetration_max,
        (double)q->cached_penetration_max,
        (double)q->translation_drift_max,
        (double)q->rotation_drift_max,
        (double)q->linear_speed_max,
        (double)q->angular_speed_max,
        (double)q->joint_error_max,
        q->window_steps != 0u ? q->support_force_sum / (double)q->window_steps
                              : 0.0,
        q->supported_weight,
        (double)gravity.x,
        (double)gravity.y,
        (double)sl_world_get_linear_drag(&b->adapter.world),
        (double)sl_world_get_angular_drag(&b->adapter.world),
        (double)sl_world_get_linear_speed_max(&b->adapter.world),
        (double)SL_CONTACT_HERTZ_DEFAULT,
        (double)SL_CONTACT_DAMPING_RATIO_DEFAULT,
        (double)SL_CONTACT_PUSH_VELOCITY_MAX_DEFAULT,
        (double)SL_RESTITUTION_THRESHOLD_DEFAULT,
        (double)SL_JOINT_HERTZ_DEFAULT,
        (double)SL_JOINT_DAMPING_RATIO_DEFAULT,
        (double)SL_SLEEP_SPEED_MAX_DEFAULT,
        (double)SL_SLEEP_ANGULAR_SPEED_MAX_DEFAULT,
        (double)SL_SLEEP_TIME_MIN_DEFAULT,
        b->fixture == 1u ? (double)0.4f : (double)0.6f,
        b->fixture == 1u ? (double)0.1f : 0.0
    };
    for (uint32_t i = 0u; i < 25u; ++i) {
        b->values[i] = values[i];
    }
    const uint32_t words[] = { b->fixture,
                               b->scene.seed,
                               b->warmup,
                               b->steps,
                               b->index,
                               b->phase,
                               b->failed ? 1u : 0u,
                               q->window_steps,
                               b->adapter.config.body_capacity,
                               b->adapter.config.contact_capacity,
                               b->adapter.config.joint_capacity,
                               sl_world_get_substep_count(&b->adapter.world),
                               b->scene.config.sleep_enabled ? 1u : 0u };
    for (uint32_t i = 0u; i < 13u; ++i) {
        b->words[i] = words[i];
    }
    b->counters[0] = sl_bench_digest(&b->adapter.world, b->scene.bodies,
                                     b->scene.body_count);
    b->counters[1] = b->drops;
    return true;
}
const uint32_t *sl_wasm_bench_words(const sl_wasm_bench *b)
{
    return b->words;
}
const double *sl_wasm_bench_quality(const sl_wasm_bench *b)
{
    return b->values;
}
const uint64_t *sl_wasm_bench_counters(const sl_wasm_bench *b)
{
    return b->counters;
}
