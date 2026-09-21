#include "adapter.h"
#include "silk_test.h"
#include "suites.h"

#include <float.h>
#include <math.h>
#include <string.h>

static sl_wasm_context *make_context(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c != NULL) {
        uint32_t *u = sl_wasm_input_u32(c);
        u[0] = 2u;
        SL_EXPECT(sl_wasm_world_bytes(c) > 0u);
        SL_EXPECT(sl_wasm_world_init(c));
    }
    return c;
}
static void body_input(sl_wasm_context *c)
{
    memset(sl_wasm_input_f32(c), 0, 32u * sizeof(float));
    memset(sl_wasm_input_u32(c), 0, 16u * sizeof(uint32_t));
    sl_wasm_input_f32(c)[4] = 1.0f;
}
static void lifecycle_and_rejection(void)
{
    sl_wasm_context *c = make_context();
    if (c == NULL) {
        return;
    }
    SL_EXPECT(!sl_wasm_world_init(c));
    body_input(c);
    SL_EXPECT(sl_wasm_shape_circle(c, 1.0f));
    SL_EXPECT(sl_wasm_body_create(c));
    const uint32_t i = sl_wasm_output_u32(c)[0];
    const uint32_t g = sl_wasm_output_u32(c)[1];
    SL_EXPECT(sl_wasm_body_read(c, i, g));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[10], 0.5f, 1e-6f);
    const float bad[] = { NAN, INFINITY, -INFINITY, 0.0f, -1.0f, FLT_MIN };
    for (uint32_t n = 0u; n < sizeof(bad) / sizeof(bad[0]); ++n) {
        SL_EXPECT(!sl_wasm_world_step(c, bad[n]));
        SL_EXPECT(sl_wasm_body_read(c, i, g));
        SL_EXPECT(sl_wasm_output_f32(c)[0] == 0.0f);
    }
    SL_EXPECT(!sl_wasm_body_set_position(c, i, g, NAN, 3.0f));
    SL_EXPECT(!sl_wasm_body_set_velocity(c, i, g, 1.0f, INFINITY));
    SL_EXPECT(sl_wasm_body_apply_force(c, i, g, 4.0f, 0.0f));
    SL_EXPECT(
        !sl_wasm_body_apply_force_at_point(c, i, g, 0.0f, 1.0f, FLT_MAX, 0.0f));
    SL_EXPECT(sl_wasm_world_step(c, 1.0f / 60.0f));
    SL_EXPECT(sl_wasm_body_read(c, i, g));
    SL_EXPECT(sl_wasm_output_f32(c)[0] > 0.0f);
    SL_EXPECT(sl_wasm_output_f32(c)[12] == 0.0f);
    SL_EXPECT(sl_wasm_body_create(c));
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 2u);
    SL_EXPECT(!sl_wasm_body_at(c, 2u));
    SL_EXPECT(sl_wasm_body_destroy(c, i, g));
    SL_EXPECT(!sl_wasm_body_read(c, i, g));
    SL_EXPECT(!sl_wasm_body_set_mass(c, i, g, 2.0f));
    SL_EXPECT(!sl_wasm_body_destroy(c, i, g));
    SL_EXPECT(sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_output_u32(c)[0], i);
    SL_EXPECT(sl_wasm_output_u32(c)[1] != g);
    const uint32_t replacement = sl_wasm_output_u32(c)[1];
    sl_wasm_world_reset(c);
    SL_EXPECT(!sl_wasm_body_valid(c, i, replacement));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 0u);
    sl_wasm_world_dispose(c);
    sl_wasm_world_dispose(c);
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT(!sl_wasm_world_step(c, 1.0f / 60.0f));
    sl_wasm_context_destroy(c);
}
static void config_and_shape_atomicity(void)
{
    sl_wasm_context *c = sl_wasm_context_create();
    SL_EXPECT(c != NULL);
    if (c == NULL) {
        return;
    }
    uint32_t *u = sl_wasm_input_u32(c);
    float *f = sl_wasm_input_f32(c);
    SL_EXPECT(!sl_wasm_world_init(c));
    u[0] = 1u;
    u[4] = 2u;
    SL_EXPECT(!sl_wasm_world_init(c));
    u[4] = 0u;
    f[0] = INFINITY;
    SL_EXPECT(!sl_wasm_world_init(c));
    f[0] = 0.0f;
    SL_EXPECT(sl_wasm_world_init(c));
    SL_EXPECT(sl_wasm_shape_box(c, 2.0f, 1.0f));
    sl_wasm_shape_mass(c);
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], 8.0f, 1e-6f);
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[3], 5.0f / 3.0f, 1e-6f);
    SL_EXPECT(sl_wasm_shape_aabb(c));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], -2.0f, 1e-6f);
    SL_EXPECT(sl_wasm_shape_contains(c));
    f[3] = -3.0f;
    f[5] = 6.0f;
    SL_EXPECT(sl_wasm_shape_ray(c));
    SL_EXPECT_NEAR(sl_wasm_output_f32(c)[0], 1.0f / 6.0f, 1e-6f);
    f[2] = NAN;
    SL_EXPECT(!sl_wasm_shape_aabb(c));
    SL_EXPECT(!sl_wasm_shape_ray(c));
    f[2] = 0.0f;
    sl_wasm_shape_read(c);
    float before[17];
    memcpy(before, sl_wasm_output_f32(c) + 32, sizeof(before));
    SL_EXPECT(!sl_wasm_shape_circle(c, NAN));
    SL_EXPECT(!sl_wasm_shape_polygon(c, UINT32_MAX));
    u[0] = 2u;
    u[1] = 9u;
    SL_EXPECT(!sl_wasm_shape_load(c));
    sl_wasm_shape_read(c);
    SL_EXPECT(memcmp(before, sl_wasm_output_f32(c) + 32, sizeof(before)) == 0);
    body_input(c);
    f[4] = 0.0f;
    SL_EXPECT(!sl_wasm_body_create(c));
    SL_EXPECT_INT_EQ(sl_wasm_body_count(c), 0u);
    f[4] = 1.0f;
    SL_EXPECT(sl_wasm_body_create(c));
    sl_wasm_context_destroy(c);
}
static void independent_replay(void)
{
    sl_wasm_context *a = make_context(), *b = make_context();
    if (a == NULL || b == NULL) {
        sl_wasm_context_destroy(a);
        sl_wasm_context_destroy(b);
        return;
    }
    body_input(a);
    body_input(b);
    SL_EXPECT(sl_wasm_body_create(a));
    SL_EXPECT(sl_wasm_body_create(b));
    const uint32_t i = sl_wasm_output_u32(a)[0], g = sl_wasm_output_u32(a)[1];
    for (uint32_t n = 0u; n < 180u; ++n) {
        SL_EXPECT(sl_wasm_body_apply_force(a, i, g, 1.0f, -2.0f));
        SL_EXPECT(sl_wasm_body_apply_force(b, i, g, 1.0f, -2.0f));
        SL_EXPECT(sl_wasm_world_step(a, 1.0f / 60.0f));
        SL_EXPECT(sl_wasm_world_step(b, 1.0f / 60.0f));
        SL_EXPECT(sl_wasm_body_read(a, i, g));
        SL_EXPECT(sl_wasm_body_read(b, i, g));
        SL_EXPECT(memcmp(sl_wasm_output_f32(a), sl_wasm_output_f32(b),
                         64u * sizeof(float)) == 0);
    }
    sl_wasm_world_reset(a);
    SL_EXPECT_INT_EQ(sl_wasm_body_count(b), 1u);
    sl_wasm_context_destroy(a);
    sl_wasm_context_destroy(b);
}
static const sl_test_case k_cases[] = {
    { "adapter lifecycle and guarded stepping", lifecycle_and_rejection },
    { "adapter config and shape rejection", config_and_shape_atomicity },
    { "adapter independent replay", independent_replay },
};
int sl_wasm_suite(void)
{
    return sl_run_suite("wasm", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
