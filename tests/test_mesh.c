#include "fixture.h"
#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

static void test_grid_counts_and_layout(void)
{
    SL_EXPECT_INT_EQ(sl_mesh_grid_vertex_count(3u, 2u), 12);
    SL_EXPECT_INT_EQ(sl_mesh_grid_triangle_count(3u, 2u), 12);
    SL_EXPECT_INT_EQ(sl_mesh_grid_vertex_count(0u, 2u), 0);
    SL_EXPECT_INT_EQ(
        sl_mesh_grid_vertex_count(SL_MESH_GRID_CELL_COUNT_MAX + 1u, 1u), 0);
    SL_EXPECT(sl_mesh_grid_vertex_count(SL_MESH_GRID_CELL_COUNT_MAX,
                                        SL_MESH_GRID_CELL_COUNT_MAX) <=
              SL_VERTEX_COUNT_MAX);
    SL_EXPECT(sl_mesh_grid_triangle_count(SL_MESH_GRID_CELL_COUNT_MAX,
                                          SL_MESH_GRID_CELL_COUNT_MAX) <=
              SL_TRIANGLE_COUNT_MAX);
    sl_vec3 p[12];
    uint32_t idx[36];
    SL_EXPECT(sl_mesh_grid(sl_vec3_zero(), sl_vec3_make(3.0f, 0.0f, 0.0f),
                           sl_vec3_make(0.0f, 0.0f, 2.0f), 3u, 2u, p, 12u, idx,
                           36u));
    SL_EXPECT_NEAR(p[5].x, 1.0f, 1e-6f); /* (i, j) = (1, 1) */
    SL_EXPECT_NEAR(p[5].z, 1.0f, 1e-6f);
    SL_EXPECT(!sl_mesh_grid(sl_vec3_zero(), sl_vec3_make(1.0f, 0.0f, 0.0f),
                            sl_vec3_make(0.0f, 1.0f, 0.0f), 3u, 2u, p, 11u, idx,
                            36u));
}

/* Recovers each vertex's color from the solver order. */
static void sl_colors_of(const sl_world_state *s, uint32_t cloth,
                         uint32_t *color)
{
    const sl_cloth_record *rec = &s->cloths[cloth];
    for (uint32_t c = 0; c < rec->color_count; c++) {
        for (uint32_t k = rec->color_offsets[c]; k < rec->color_offsets[c + 1u];
             k++) {
            color[s->color_order[rec->vertex_offset + k] - rec->vertex_offset] =
                c;
        }
    }
}

static void test_grid_cloth_topology(void)
{
    const uint32_t n = 12u;
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(1000u, 1000u);
    sl_cloth_desc desc = sl_test_cloth_desc();
    SL_EXPECT(sl_test_sheet(&world, &c, &desc, 2.0f, n, 0.0f, false));
    const sl_world_stats st = sl_world_stats_get(&world);
    SL_EXPECT_INT_EQ(st.vertex_count, (n + 1u) * (n + 1u));
    SL_EXPECT_INT_EQ(st.triangle_count, 2u * n * n);
    SL_EXPECT_INT_EQ(st.bend_count, 3u * n * n - 2u * n); /* interior edges */
    SL_EXPECT(st.color_count_max >= 4u && st.color_count_max <= 16u);
    SL_EXPECT(st.arena_bytes > 0u);

    const sl_world_state *s = world.state;
    double mass = 0.0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        SL_EXPECT(s->mass[i] > 0.0f);
        mass += (double)s->mass[i];
    }
    SL_EXPECT_NEAR(mass, 0.2 * 4.0, 1e-5);

    uint32_t color[169];
    uint32_t seen[169] = { 0 };
    sl_colors_of(s, 0u, color);
    for (uint32_t k = 0; k < s->vertex_count; k++) {
        seen[s->color_order[k]]++;
    }
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        SL_EXPECT_INT_EQ(seen[i], 1);
    }
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *v = s->triangle_index + 3u * t;
        SL_EXPECT(color[v[0]] != color[v[1]] && color[v[1]] != color[v[2]] &&
                  color[v[0]] != color[v[2]]);
    }
    for (uint32_t b = 0; b < s->bend_count; b++) {
        const uint32_t *v = s->bend_index + 4u * b;
        for (uint32_t p = 0; p < 4u; p++) {
            for (uint32_t q = p + 1u; q < 4u; q++) {
                SL_EXPECT(color[v[p]] != color[v[q]]);
            }
        }
    }
    /* Within a color, vertices ascend: the documented sweep order. */
    const sl_cloth_record *rec = &s->cloths[0];
    for (uint32_t col = 0; col < rec->color_count; col++) {
        for (uint32_t k = rec->color_offsets[col] + 1u;
             k < rec->color_offsets[col + 1u]; k++) {
            SL_EXPECT(s->color_order[k - 1u] < s->color_order[k]);
        }
    }
    /* CSR: every triangle corner and stencil slot appears exactly once. */
    SL_EXPECT_INT_EQ(s->vertex_triangle_offset[s->vertex_count],
                     3u * s->triangle_count);
    SL_EXPECT_INT_EQ(s->vertex_bend_offset[s->vertex_count],
                     4u * s->bend_count);
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        for (uint32_t r = s->vertex_triangle_offset[i];
             r < s->vertex_triangle_offset[i + 1u]; r++) {
            SL_EXPECT_INT_EQ(s->triangle_index[s->vertex_triangle[r]], i);
        }
        for (uint32_t r = s->vertex_bend_offset[i];
             r < s->vertex_bend_offset[i + 1u]; r++) {
            SL_EXPECT_INT_EQ(s->bend_index[s->vertex_bend[r]], i);
        }
    }
    sl_world_destroy(&world);
    SL_EXPECT(world.state == NULL);
}

/* Two triangles sharing edge 0-1. */
static const sl_vec3 k_quad[4] = {
    { 0.0f, 0.0f, 0.0f },
    { 1.0f, 0.0f, 0.0f },
    { 0.5f, 1.0f, 0.0f },
    { 0.5f, -1.0f, 0.0f },
};

static bool sl_try_add(sl_world *w, const sl_vec3 *p, uint32_t n,
                       const uint32_t *idx, uint32_t tn)
{
    sl_cloth_desc d = sl_test_cloth_desc();
    d.positions = p;
    d.vertex_count = n;
    d.indices = idx;
    d.triangle_count = tn;
    uint32_t cloth = 0;
    return sl_world_cloth_add(w, &d, &cloth);
}

static void sl_expect_unchanged(const sl_world *w, sl_world_stats before,
                                uint64_t hash)
{
    const sl_world_stats after = sl_world_stats_get(w);
    SL_EXPECT_INT_EQ(after.vertex_count, before.vertex_count);
    SL_EXPECT_INT_EQ(after.triangle_count, before.triangle_count);
    SL_EXPECT_INT_EQ(after.bend_count, before.bend_count);
    SL_EXPECT_INT_EQ(after.cloth_count, before.cloth_count);
    SL_EXPECT(sl_test_world_hash(w) == hash);
}

static void test_invalid_meshes_rejected_without_mutation(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(64u, 64u);
    SL_EXPECT(sl_world_init(&world, &c));
    const uint32_t good[6] = { 0, 1, 2, 1, 0, 3 };
    SL_EXPECT(sl_try_add(&world, k_quad, 4u, good, 2u));
    const sl_world_stats before = sl_world_stats_get(&world);
    const uint64_t hash = sl_test_world_hash(&world);
    SL_EXPECT_INT_EQ(before.bend_count, 1);

    const uint32_t out_of_range[6] = { 0, 1, 2, 1, 0, 4 };
    SL_EXPECT(!sl_try_add(&world, k_quad, 4u, out_of_range, 2u));
    const uint32_t repeated[6] = { 0, 1, 1, 1, 0, 3 };
    SL_EXPECT(!sl_try_add(&world, k_quad, 4u, repeated, 2u));
    const uint32_t unreferenced[3] = { 0, 1, 2 };
    SL_EXPECT(!sl_try_add(&world, k_quad, 4u, unreferenced, 1u));
    const uint32_t duplicate[6] = { 0, 1, 2, 2, 1, 0 };
    SL_EXPECT(!sl_try_add(&world, k_quad, 3u, duplicate, 2u));
    /* Three triangles on edge 0-1. */
    const sl_vec3 fin[5] = {
        k_quad[0], k_quad[1], k_quad[2], k_quad[3], { 0.5f, 0.0f, 1.0f }
    };
    const uint32_t nonmanifold[9] = { 0, 1, 2, 1, 0, 3, 0, 1, 4 };
    SL_EXPECT(!sl_try_add(&world, fin, 5u, nonmanifold, 3u));
    const sl_vec3 collinear[4] = {
        k_quad[0], k_quad[1], { 2.0f, 0.0f, 0.0f }, k_quad[3]
    };
    SL_EXPECT(!sl_try_add(&world, collinear, 4u, good, 2u));
    sl_vec3 nan_point[4] = { k_quad[0], k_quad[1], k_quad[2], k_quad[3] };
    nan_point[3].y = NAN;
    SL_EXPECT(!sl_try_add(&world, nan_point, 4u, good, 2u));
    sl_vec3 far_point[4] = { k_quad[0], k_quad[1], k_quad[2], k_quad[3] };
    far_point[3].x = 2.0f * SL_POSITION_ABS_MAX;
    SL_EXPECT(!sl_try_add(&world, far_point, 4u, good, 2u));

    sl_cloth_desc d = sl_test_cloth_desc();
    d.positions = k_quad;
    d.vertex_count = 4u;
    d.indices = good;
    d.triangle_count = 2u;
    uint32_t cloth = 0;
    d.density = 0.0f;
    SL_EXPECT(!sl_world_cloth_add(&world, &d, &cloth));
    d = sl_test_cloth_desc();
    d.positions = k_quad;
    d.vertex_count = 4u;
    d.indices = good;
    d.triangle_count = 2u;
    d.membrane_mu = -1.0f;
    SL_EXPECT(!sl_world_cloth_add(&world, &d, &cloth));
    d.membrane_mu = 1.0f;
    d.thickness = 0.0f;
    SL_EXPECT(!sl_world_cloth_add(&world, &d, &cloth));
    d.thickness = 0.01f;
    d.drag = INFINITY;
    SL_EXPECT(!sl_world_cloth_add(&world, &d, &cloth));
    sl_expect_unchanged(&world, before, hash);

    /* The world still accepts a valid mesh after the rejections, and the
     * second cloth's adjacency continues the first's. */
    SL_EXPECT(sl_try_add(&world, k_quad, 4u, good, 2u));
    const sl_world_state *s = world.state;
    SL_EXPECT_INT_EQ(s->vertex_triangle_offset[4], 6);
    SL_EXPECT_INT_EQ(s->vertex_triangle_offset[8], 12);
    SL_EXPECT_INT_EQ(s->vertex_bend_offset[4], 4);
    SL_EXPECT_INT_EQ(s->vertex_bend_offset[8], 8);
    SL_EXPECT_INT_EQ(s->triangle_index[6], 4);
    sl_world_destroy(&world);
}

static void test_capacity_limits(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(5u, 64u);
    c.cloth_capacity = 1u;
    SL_EXPECT(sl_world_init(&world, &c));
    const uint32_t good[6] = { 0, 1, 2, 1, 0, 3 };
    SL_EXPECT(sl_try_add(&world, k_quad, 4u, good, 2u));
    SL_EXPECT(!sl_try_add(&world, k_quad, 4u, good, 2u)); /* cloths full */
    sl_world_destroy(&world);

    c.cloth_capacity = 2u;
    SL_EXPECT(sl_world_init(&world, &c));
    SL_EXPECT(sl_try_add(&world, k_quad, 4u, good, 2u));
    SL_EXPECT(!sl_try_add(&world, k_quad, 4u, good, 2u)); /* vertices full */
    sl_world_destroy(&world);

    sl_world_config bad = sl_test_config(0u, 1u);
    SL_EXPECT(!sl_world_init(&world, &bad));
    bad = sl_test_config(1u, 1u);
    bad.iteration_count = SL_ITERATION_COUNT_MAX + 1u;
    SL_EXPECT(!sl_world_init(&world, &bad));
    bad.iteration_count = 0u;
    bad.substep_count = SL_SUBSTEP_COUNT_MAX + 1u;
    SL_EXPECT(!sl_world_init(&world, &bad));
    bad.substep_count = 0u;
    bad.speed_max = -1.0f;
    SL_EXPECT(!sl_world_init(&world, &bad));
    bad.speed_max = 0.0f;
    bad.gravity.x = NAN;
    SL_EXPECT(!sl_world_init(&world, &bad));
    SL_EXPECT(world.state == NULL);
}

/* A fan of `count` triangles around vertex 0. */
static bool sl_try_fan(uint32_t count)
{
    sl_vec3 p[SL_VERTEX_TRIANGLE_COUNT_MAX + 3u];
    uint32_t idx[3u * (SL_VERTEX_TRIANGLE_COUNT_MAX + 1u)];
    p[0] = sl_vec3_zero();
    for (uint32_t k = 0; k <= count; k++) {
        const float a = SL_PI * (float)k / (float)count; /* half disc */
        p[k + 1u] = sl_vec3_make(cosf(a), sinf(a), 0.0f);
    }
    for (uint32_t k = 0; k < count; k++) {
        idx[3u * k] = 0u;
        idx[3u * k + 1u] = k + 1u;
        idx[3u * k + 2u] = k + 2u;
    }
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(64u, 64u);
    bool ok = sl_world_init(&world, &c);
    ok = ok && sl_try_add(&world, p, count + 2u, idx, count);
    sl_world_destroy(&world);
    return ok;
}

static void test_vertex_valence_bound(void)
{
    SL_EXPECT(sl_try_fan(SL_VERTEX_TRIANGLE_COUNT_MAX));
    SL_EXPECT(!sl_try_fan(SL_VERTEX_TRIANGLE_COUNT_MAX + 1u));
}

static void test_vertex_queries(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(64u, 64u);
    SL_EXPECT(sl_world_init(&world, &c));
    const uint32_t good[6] = { 0, 1, 2, 1, 0, 3 };
    SL_EXPECT(sl_try_add(&world, k_quad, 4u, good, 2u));
    SL_EXPECT_INT_EQ(sl_world_cloth_vertex_count(&world, 0u), 4);
    SL_EXPECT_INT_EQ(sl_world_cloth_vertex_count(&world, 1u), 0);
    SL_EXPECT(sl_world_cloth_positions(&world, 1u) == NULL);
    SL_EXPECT(sl_world_cloth_positions(&world, 0u)[2].y == 1.0f);
    SL_EXPECT(!sl_world_vertex_pin_set(&world, 0u, 4u, true));
    SL_EXPECT(!sl_world_vertex_pin_set(&world, 1u, 0u, true));
    SL_EXPECT(!sl_world_vertex_target_set(&world, 0u, 2u, sl_vec3_zero()));
    SL_EXPECT(sl_world_vertex_pin_set(&world, 0u, 2u, true));
    SL_EXPECT(sl_world_vertex_is_pinned(&world, 0u, 2u));
    SL_EXPECT(!sl_world_vertex_target_set(&world, 0u, 2u,
                                          sl_vec3_make(NAN, 0.0f, 0.0f)));
    SL_EXPECT(sl_world_vertex_target_set(&world, 0u, 2u, sl_vec3_zero()));
    SL_EXPECT(sl_world_vertex_pin_set(&world, 0u, 2u, false));
    SL_EXPECT(!sl_world_vertex_is_pinned(&world, 0u, 2u));
    SL_EXPECT(!sl_world_gravity_set(&world, sl_vec3_make(0.0f, NAN, 0.0f)));
    SL_EXPECT(!sl_world_wind_set(&world, sl_vec3_make(INFINITY, 0.0f, 0.0f)));
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "grid_counts_and_layout", test_grid_counts_and_layout },
    { "grid_cloth_topology", test_grid_cloth_topology },
    { "invalid_meshes_rejected",
      test_invalid_meshes_rejected_without_mutation },
    { "capacity_limits", test_capacity_limits },
    { "vertex_valence_bound", test_vertex_valence_bound },
    { "vertex_queries", test_vertex_queries },
};

int sl_mesh_suite(void)
{
    return sl_run_suite("mesh", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
