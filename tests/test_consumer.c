/* Compiled without the private src/ include path: everything here must be
 * reachable through the installed public headers alone. */
#include <silk/assert.h>
#include <silk/math.h>
#include <silk/mesh.h>
#include <silk/silk.h>
#include <silk/step.h>
#include <silk/world.h>

#include "silk_test.h"
#include "suites.h"

static void test_public_api_round_trip(void)
{
    sl_vec3 positions[9];
    uint32_t indices[24];
    SL_EXPECT(sl_mesh_grid(
        sl_vec3_make(0.0f, 1.0f, 0.0f), sl_vec3_make(1.0f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, -1.0f, 0.0f), 2u, 2u, positions, 9u, indices, 24u));
    sl_world_config config = { 0 };
    config.vertex_capacity = 9u;
    config.triangle_capacity = 8u;
    config.cloth_capacity = 1u;
    config.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_cloth_desc desc = { 0 };
    desc.positions = positions;
    desc.vertex_count = 9u;
    desc.indices = indices;
    desc.triangle_count = 8u;
    desc.density = 0.2f;
    desc.membrane_mu = 1000.0f;
    desc.membrane_lambda = 1000.0f;
    desc.thickness = 0.005f;
    uint32_t cloth = 99u;
    SL_EXPECT(sl_world_cloth_add(&world, &desc, &cloth));
    SL_EXPECT_INT_EQ(cloth, 0);
    SL_EXPECT(sl_world_vertex_pin_set(&world, cloth, 0u, true));
    sl_world_step(&world, 1.0f / 60.0f);
    const sl_vec3 *p = sl_world_cloth_positions(&world, cloth);
    SL_EXPECT(p != NULL && p[0].y == 1.0f && p[8].y < 0.0f);
    /* Diagnostics: no colliders and no self contact. */
    SL_EXPECT_INT_EQ(sl_world_vertex_flags(&world, cloth, 8u), 0);
    SL_EXPECT_INT_EQ(sl_world_self_contacts(&world, NULL, 0u), 0);
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "public_api_round_trip", test_public_api_round_trip },
};

int sl_consumer_suite(void)
{
    return sl_run_suite("consumer", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
