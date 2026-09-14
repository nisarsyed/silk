#include "driver.h"
#include "../../wasm/context.h"
#include "../fixtures.h"
#include "scene.h"
#include <stdlib.h>

struct sl_render_study {
    sl_wasm_context adapter;
    uint32_t status[8];
    uint64_t drops;
};
sl_render_study *sl_render_study_create(uint32_t fixture, uint32_t copies,
                                        uint32_t sleep_enabled,
                                        uint32_t step_limit)
{
    if (sleep_enabled > 1u || step_limit == 0u ||
        step_limit > SL_RENDER_STEP_COUNT_MAX) {
        return NULL;
    }
    sl_render_study *study = calloc(1u, sizeof(*study));
    if (study == NULL) {
        return NULL;
    }
    sl_wasm_context *adapter = &study->adapter;
    if (!sl_render_scene_build(&adapter->world, &adapter->config, fixture,
                               copies, sleep_enabled != 0u) ||
        !sl_wasm_storage_init(adapter)) {
        sl_render_study_destroy(study);
        return NULL;
    }
    const uint32_t status[] = { fixture,
                                copies,
                                sleep_enabled,
                                step_limit,
                                0u,
                                adapter->config.body_capacity,
                                adapter->config.contact_capacity,
                                adapter->config.joint_capacity };
    for (uint32_t i = 0u; i < 8u; ++i) {
        study->status[i] = status[i];
    }
    return study;
}
void sl_render_study_destroy(sl_render_study *study)
{
    if (study != NULL) {
        sl_wasm_world_dispose(&study->adapter);
        free(study);
    }
}
bool sl_render_study_step(sl_render_study *study)
{
    if (study == NULL || study->status[4] >= study->status[3]) {
        return false;
    }
    sl_world_step(&study->adapter.world, SL_BENCH_TIMESTEP);
    study->drops += sl_world_contact_drop_count(&study->adapter.world);
    ++study->status[4];
    return true;
}
sl_wasm_context *sl_render_study_adapter(sl_render_study *study)
{
    return study != NULL ? &study->adapter : NULL;
}
sl_world *sl_render_study_world(sl_render_study *study)
{
    return study != NULL ? &study->adapter.world : NULL;
}
const uint32_t *sl_render_study_status(const sl_render_study *study)
{
    return study != NULL ? study->status : NULL;
}
uint64_t sl_render_study_drops(const sl_render_study *study)
{
    return study != NULL ? study->drops : 0u;
}
size_t sl_render_study_bytes(void)
{
    return sizeof(sl_render_study);
}
