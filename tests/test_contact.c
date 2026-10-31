#include "contact.h"
#include "fixture.h"
#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

static sl_vec3 sl_rand_point(uint32_t *seed, float extent)
{
    return sl_vec3_make(sl_test_randf(seed, -extent, extent),
                        sl_test_randf(seed, -extent, extent),
                        sl_test_randf(seed, -extent, extent));
}

static void test_triangle_closest_matches_sampling(void)
{
    uint32_t seed = 0xC0000001u;
    for (int trial = 0; trial < 200; trial++) {
        const sl_vec3 t[3] = { sl_rand_point(&seed, 1.0f),
                               sl_rand_point(&seed, 1.0f),
                               sl_rand_point(&seed, 1.0f) };
        const sl_vec3 p = sl_rand_point(&seed, 1.5f);
        const sl_triangle_closest c =
            sl_triangle_closest_point(p, t[0], t[1], t[2]);
        /* Brute force over a barycentric lattice. */
        float best = INFINITY;
        const int n = 60;
        for (int i = 0; i <= n; i++) {
            for (int j = 0; i + j <= n; j++) {
                const float a = (float)i / (float)n;
                const float b = (float)j / (float)n;
                sl_vec3 q = sl_vec3_scale(t[0], 1.0f - a - b);
                q = sl_vec3_mul_add(q, t[1], a);
                q = sl_vec3_mul_add(q, t[2], b);
                best = sl_min(best, sl_vec3_length(sl_vec3_sub(p, q)));
            }
        }
        SL_EXPECT(c.distance <= best + 1e-5f);
        SL_EXPECT(c.distance >= best - 0.05f);
        /* The feature matches the barycentric zero pattern. */
        uint32_t zeros = 0;
        for (uint32_t k = 0; k < 3u; k++) {
            zeros += (c.bary[k] == 0.0f) ? 1u : 0u;
        }
        if (c.kind == SL_FEATURE_VERTEX) {
            SL_EXPECT_INT_EQ(zeros, 2);
        } else if (c.kind == SL_FEATURE_EDGE) {
            SL_EXPECT(zeros >= 1u && c.a < c.b);
        }
    }
}

static void test_segment_closest_matches_sampling(void)
{
    uint32_t seed = 0xC0000002u;
    for (int trial = 0; trial < 200; trial++) {
        const sl_vec3 p0 = sl_rand_point(&seed, 1.0f);
        const sl_vec3 p1 = sl_rand_point(&seed, 1.0f);
        const sl_vec3 q0 = sl_rand_point(&seed, 1.0f);
        const sl_vec3 q1 = sl_rand_point(&seed, 1.0f);
        float s;
        float t;
        sl_segment_closest_params(p0, p1, q0, q1, &s, &t);
        SL_EXPECT(s >= 0.0f && s <= 1.0f && t >= 0.0f && t <= 1.0f);
        const float d = sl_vec3_length(
            sl_vec3_sub(sl_vec3_lerp(p0, p1, s), sl_vec3_lerp(q0, q1, t)));
        float best = INFINITY;
        for (int i = 0; i <= 100; i++) {
            for (int j = 0; j <= 100; j++) {
                best =
                    sl_min(best, sl_vec3_length(sl_vec3_sub(
                                     sl_vec3_lerp(p0, p1, (float)i / 100.0f),
                                     sl_vec3_lerp(q0, q1, (float)j / 100.0f))));
            }
        }
        SL_EXPECT(d <= best + 1e-5f);
    }
    /* Parallel segments pick a deterministic pair at the true distance. */
    float s;
    float t;
    sl_segment_closest_params(sl_vec3_zero(), sl_vec3_make(1.0f, 0.0f, 0.0f),
                              sl_vec3_make(0.5f, 0.2f, 0.0f),
                              sl_vec3_make(1.5f, 0.2f, 0.0f), &s, &t);
    const float d = sl_vec3_length(sl_vec3_sub(
        sl_vec3_make(s, 0.0f, 0.0f), sl_vec3_make(0.5f + t, 0.2f, 0.0f)));
    SL_EXPECT_NEAR(d, 0.2f, 1e-6f);
}

static void test_contact_energy_two_stage(void)
{
    const float r = 0.01f;
    const float k = 1.0e4f;
    float slope;
    float curvature;
    SL_EXPECT(sl_contact_energy(r, r, k, &slope, &curvature) == 0.0f);
    SL_EXPECT(slope == 0.0f && curvature == 0.0f);
    SL_EXPECT(sl_contact_energy(2.0f * r, r, k, &slope, &curvature) == 0.0f);
    /* C2 at tau = r / 2: value, slope and curvature agree from both sides. */
    const float tau = 0.5f * r;
    float s_lo;
    float c_lo;
    float s_hi;
    float c_hi;
    const float g_lo =
        sl_contact_energy(tau * (1.0f - 1e-5f), r, k, &s_lo, &c_lo);
    const float g_hi =
        sl_contact_energy(tau * (1.0f + 1e-5f), r, k, &s_hi, &c_hi);
    /* Values differ only by the slope across the 2e-5 tau gap. */
    SL_EXPECT_NEAR(g_lo, g_hi, 1.1f * sl_abs(s_hi) * 2e-5f * tau);
    SL_EXPECT_NEAR(s_lo, s_hi, 1e-3f * sl_abs(s_hi));
    SL_EXPECT_NEAR(c_lo, c_hi, 1e-3f * c_hi);
    /* Slope and curvature are the derivatives (central differences). */
    const float points[3] = { 0.2f * r, 0.6f * r, 0.9f * r };
    for (uint32_t i = 0; i < 3u; i++) {
        const float d = points[i];
        const float h = 1e-3f * r;
        float sm;
        float cm;
        float sp;
        float cp;
        const double gp = (double)sl_contact_energy(d + h, r, k, &sp, &cp);
        const double gm = (double)sl_contact_energy(d - h, r, k, &sm, &cm);
        float s0;
        float c0;
        (void)sl_contact_energy(d, r, k, &s0, &c0);
        SL_EXPECT_NEAR(s0, (gp - gm) / (2.0 * (double)h),
                       1e-2 * fabs((double)s0));
        SL_EXPECT_NEAR(c0, (sp - sm) / (2.0f * h), 1e-2f * c0);
        SL_EXPECT(s0 < 0.0f && c0 > 0.0f);
    }
    /* The barrier stage's force grows without bound as d -> 0. */
    float near_slope;
    (void)sl_contact_energy(1e-6f * r, r, k, &near_slope, &curvature);
    (void)sl_contact_energy(tau, r, k, &slope, &curvature);
    SL_EXPECT(near_slope < 1.0e5f * slope);
}

/* One sheet in a world with self contact enabled. */
static bool sl_contact_world(sl_world *w, uint32_t capacity, float thickness,
                             float side, uint32_t cells, float top)
{
    sl_world_config c = sl_test_config(2000u, 4000u);
    c.self_contact_capacity = capacity;
    sl_cloth_desc d = sl_test_cloth_desc();
    d.thickness = thickness;
    return sl_test_sheet(w, &c, &d, side, cells, top, false);
}

static void test_flat_sheet_does_not_repel_itself(void)
{
    /* OGC's vertex blocks only admit points off the surface, so no vertex
     * of a flat sheet touches the sheet at any radius. Edge-edge contact
     * lives on the wireframe, which has no faces: non-adjacent edges of
     * this grid are 0.71 of an edge apart, so below that radius a flat
     * sheet has no contact at all, and above it only edge pairs. */
    sl_world world = { 0 };
    SL_EXPECT(sl_contact_world(&world, 4096u, 0.03f, 1.0f, 10u, 1.0f));
    sl_self_detect(world.state, 0.0f); /* r = 0.6 edges */
    SL_EXPECT_INT_EQ(world.state->pair_count, 0);
    sl_world_destroy(&world);

    SL_EXPECT(sl_contact_world(&world, 4096u, 0.04f, 1.0f, 10u, 1.0f));
    const sl_world_state *s = world.state;
    sl_self_detect(world.state, 0.0f); /* r = 0.8 edges */
    SL_EXPECT(s->pair_count > 0u);
    for (uint32_t k = 0; k < s->pair_count; k++) {
        SL_EXPECT_INT_EQ(s->pairs[k].kind, SL_PAIR_EDGE_EDGE);
    }
    sl_world_destroy(&world);
}

/* Brute-force distance from vertex v to the nearest triangle of its own
 * cloth that does not contain it. */
static float sl_brute_vertex_distance(const sl_world_state *s, uint32_t v)
{
    float best = INFINITY;
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        if (idx[0] == v || idx[1] == v || idx[2] == v ||
            s->vertex_cloth[idx[0]] != s->vertex_cloth[v]) {
            continue;
        }
        best = sl_min(best, sl_triangle_closest_point(
                                s->position[v], s->position[idx[0]],
                                s->position[idx[1]], s->position[idx[2]])
                                .distance);
    }
    return best;
}

/* Brute-force distance from edge e to the nearest edge of its own cloth
 * that shares no vertex with it. */
static float sl_brute_edge_distance(const sl_world_state *s, uint32_t e)
{
    const uint32_t e0 = s->edge_index[2u * e];
    const uint32_t e1 = s->edge_index[2u * e + 1u];
    float best = INFINITY;
    for (uint32_t f = 0; f < s->edge_count; f++) {
        const uint32_t f0 = s->edge_index[2u * f];
        const uint32_t f1 = s->edge_index[2u * f + 1u];
        if (f0 == e0 || f0 == e1 || f1 == e0 || f1 == e1 ||
            s->vertex_cloth[f0] != s->vertex_cloth[e0]) {
            continue;
        }
        float a;
        float b;
        sl_segment_closest_params(s->position[e0], s->position[e1],
                                  s->position[f0], s->position[f1], &a, &b);
        best = sl_min(best,
                      sl_vec3_length(sl_vec3_sub(
                          sl_vec3_lerp(s->position[e0], s->position[e1], a),
                          sl_vec3_lerp(s->position[f0], s->position[f1], b))));
    }
    return best;
}

static void test_broad_phase_finds_every_near_triangle(void)
{
    /* Crumple a sheet deterministically; every vertex's detected nearest
     * non-incident distance must equal brute force (capped at the query
     * radius), i.e. the spatial hash misses nothing. */
    sl_world world = { 0 };
    SL_EXPECT(sl_contact_world(&world, 8192u, 0.01f, 1.0f, 12u, 1.0f));
    sl_world_state *s = world.state;
    uint32_t seed = 0xC0000003u;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        const sl_vec3 p = s->position[i];
        s->position[i] = sl_vec3_make(p.x + sl_test_randf(&seed, -0.02f, 0.02f),
                                      p.y + 0.15f * sinf(9.0f * p.x) +
                                          sl_test_randf(&seed, -0.02f, 0.02f),
                                      0.25f * cosf(7.0f * p.y) +
                                          sl_test_randf(&seed, -0.03f, 0.03f));
    }
    sl_self_detect(s, 0.08f);
    const float query = s->query_radius;
    /* The motion term widens the query past the 2 cm contact radius. */
    SL_EXPECT(query > 0.02f);
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        const float brute = sl_min(sl_brute_vertex_distance(s, v), query);
        /* bound = gamma_p * min(vertex, edge, triangle distances) <=
         * gamma_p * vertex distance; equality on the vertex term is checked
         * through vertex_dmin, which the bounds step does not touch. */
        SL_EXPECT_NEAR(s->vertex_dmin[v], brute, 1e-6f);
    }
    /* Edge pairs come from a self traversal of the edge tree; it must
     * reach every pair within the query radius as well. */
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const float brute = sl_min(sl_brute_edge_distance(s, e), query);
        SL_EXPECT_NEAR(s->edge_dmin[e], brute, 1e-6f);
    }
    sl_world_destroy(&world);
}

/* Segment-triangle crossing (Moller-Trumbore), endpoints excluded. */
static bool sl_segment_crosses(sl_vec3 p, sl_vec3 q, sl_vec3 a, sl_vec3 b,
                               sl_vec3 c)
{
    const sl_vec3 e1 = sl_vec3_sub(b, a);
    const sl_vec3 e2 = sl_vec3_sub(c, a);
    const sl_vec3 d = sl_vec3_sub(q, p);
    const sl_vec3 h = sl_vec3_cross(d, e2);
    const float det = sl_vec3_dot(e1, h);
    if (sl_abs(det) < 1e-14f) {
        return false;
    }
    const float f = 1.0f / det;
    const sl_vec3 o = sl_vec3_sub(p, a);
    const float u = f * sl_vec3_dot(o, h);
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    const sl_vec3 r = sl_vec3_cross(o, e1);
    const float w = f * sl_vec3_dot(d, r);
    if (w < 0.0f || u + w > 1.0f) {
        return false;
    }
    const float t = f * sl_vec3_dot(e2, r);
    return t > 0.0f && t < 1.0f;
}

/* Edges crossing non-incident triangles anywhere in the world. */
static uint32_t sl_crossings(const sl_world_state *s)
{
    uint32_t count = 0;
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const uint32_t a = s->edge_index[2u * e];
        const uint32_t b = s->edge_index[2u * e + 1u];
        for (uint32_t t = 0; t < s->triangle_count; t++) {
            const uint32_t *idx = s->triangle_index + 3u * t;
            if (idx[0] == a || idx[1] == a || idx[2] == a || idx[0] == b ||
                idx[1] == b || idx[2] == b) {
                continue;
            }
            count += sl_segment_crosses(
                         s->position[a], s->position[b], s->position[idx[0]],
                         s->position[idx[1]], s->position[idx[2]])
                         ? 1u
                         : 0u;
        }
    }
    return count;
}

static void sl_add_flat(sl_world *w, float y, float side, uint32_t cells)
{
    sl_cloth_desc d = sl_test_cloth_desc();
    d.thickness = 0.004f;
    d.damping = 20.0f;
    d.drag = 0.3f;
    sl_test_grid g =
        sl_test_grid_make(sl_vec3_make(-0.5f * side, y, -0.5f * side),
                          sl_vec3_make(side, 0.0f, 0.0f),
                          sl_vec3_make(0.0f, 0.0f, side), cells, cells);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    SL_EXPECT(sl_world_cloth_add(w, &d, &cloth));
    sl_test_grid_free(&g);
}

/* Two sheets dropped onto a sphere; returns the worst crossing count over
 * all steps and the peak contact pair count. */
static uint32_t sl_stack_drop(uint32_t capacity, uint32_t *pairs_max)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = capacity;
    (void)sl_world_init(&world, &c);
    sl_add_flat(&world, 0.4f, 0.8f, 10u);
    sl_add_flat(&world, 0.5f, 0.7f, 10u);
    const sl_collider sphere = { SL_COLLIDER_SPHERE, sl_vec3_zero(),
                                 sl_vec3_zero(), 0.2f, 0.5f };
    uint32_t id = 0;
    (void)sl_world_collider_add(&world, &sphere, &id);
    uint32_t worst = 0;
    *pairs_max = 0;
    for (uint32_t k = 0; k < 90u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
        const uint32_t crossings = sl_crossings(world.state);
        worst = (crossings > worst) ? crossings : worst;
        const uint32_t pairs = sl_world_stats_get(&world).self_contact_count;
        *pairs_max = (pairs > *pairs_max) ? pairs : *pairs_max;
    }
    sl_world_destroy(&world);
    return worst;
}

static void test_stacked_sheets_never_intersect(void)
{
    uint32_t pairs = 0;
    /* Without self contact the upper sheet falls through the lower one. */
    SL_EXPECT(sl_stack_drop(0u, &pairs) > 0u);
    SL_EXPECT_INT_EQ(pairs, 0);
    /* With it, no edge ever crosses a triangle, at any step. */
    SL_EXPECT_INT_EQ(sl_stack_drop(4096u, &pairs), 0);
    SL_EXPECT(pairs > 10u);
}

static float sl_fall_speed(uint32_t capacity)
{
    sl_world world = { 0 };
    (void)sl_contact_world(&world, capacity, 0.003f, 1.0f, 12u, 1.0f);
    for (uint32_t k = 0; k < 30u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
    }
    const float speed = -world.state->velocity[84].y;
    sl_world_destroy(&world);
    return speed;
}

static void test_bounds_allow_rigid_motion(void)
{
    /* Translation-relative bounds: a falling sheet is not slowed by its
     * own displacement bounds (absolute bounds held it to 0.4 m/s). */
    const float free_fall = sl_fall_speed(0u);
    SL_EXPECT_NEAR(sl_fall_speed(4096u), free_fall, 1e-3f * free_fall);
}

static uint64_t sl_stack_hash(sl_test_scramble *scramble)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = 4096u;
    if (scramble != NULL) {
        c.parallel_for = sl_test_scrambled_for;
        c.parallel_context = scramble;
    }
    (void)sl_world_init(&world, &c);
    sl_add_flat(&world, 0.4f, 0.8f, 8u);
    sl_add_flat(&world, 0.45f, 0.7f, 8u);
    const sl_collider ground = { SL_COLLIDER_PLANE, sl_vec3_zero(),
                                 sl_vec3_make(0.0f, 1.0f, 0.0f), 0.0f, 0.5f };
    uint32_t id = 0;
    (void)sl_world_collider_add(&world, &ground, &id);
    for (uint32_t k = 0; k < 40u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
    }
    const uint64_t hash = sl_test_world_hash(&world);
    sl_world_destroy(&world);
    return hash;
}

static void test_self_contact_is_deterministic(void)
{
    SL_EXPECT(sl_stack_hash(NULL) == sl_stack_hash(NULL));
}

/* One 0.6 m sheet folded in half along z: the far half lies mirrored 6 mm
 * above the near half, inside the 8 mm contact radius, so the cloth
 * touches itself from the first step and contact pairs join vertices of
 * one cloth, some of the same color (up to 90 pairs; bending reopens the
 * fold within 30 steps). It falls onto the ground. Solving the
 * self-contact sweep in place makes the scrambled run differ. */
static uint64_t sl_fold_hash(sl_test_scramble *scramble)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = 4096u;
    if (scramble != NULL) {
        c.parallel_for = sl_test_scrambled_for;
        c.parallel_context = scramble;
    }
    (void)sl_world_init(&world, &c);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.thickness = 0.004f;
    d.damping = 20.0f;
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.3f, 0.2f, -0.3f), sl_vec3_make(0.6f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, 0.0f, 0.6f), 12u, 12u);
    for (uint32_t i = 0; i < g.vertex_count; i++) {
        if (g.positions[i].z > 0.001f) {
            g.positions[i].z = -g.positions[i].z;
            g.positions[i].y += 0.006f;
        }
    }
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    SL_EXPECT(sl_world_cloth_add(&world, &d, &cloth));
    sl_test_grid_free(&g);
    const sl_collider ground = { SL_COLLIDER_PLANE, sl_vec3_zero(),
                                 sl_vec3_make(0.0f, 1.0f, 0.0f), 0.0f, 0.5f };
    uint32_t id = 0;
    (void)sl_world_collider_add(&world, &ground, &id);
    uint32_t pairs_max = 0;
    for (uint32_t k = 0; k < 30u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
        const uint32_t pairs = sl_world_stats_get(&world).self_contact_count;
        pairs_max = (pairs > pairs_max) ? pairs : pairs_max;
    }

    SL_EXPECT(pairs_max > 50u);
    const uint64_t hash = sl_test_world_hash(&world);
    sl_world_destroy(&world);
    return hash;
}

static void test_self_contact_parallel_matches_serial(void)
{
    /* Same-color contact partners read start-of-color positions
     * (solver_self.c), so any partition and order gives the serial
     * result: for contact within one folded cloth, and between cloths. */
    const uint64_t fold = sl_fold_hash(NULL);
    const uint64_t stack = sl_stack_hash(NULL);
    const uint32_t seeds[2] = { 0x9E3779B9u, 11u };
    for (uint32_t k = 0; k < 2u; k++) {
        sl_test_scramble scramble = { seeds[k], 0u };
        SL_EXPECT(sl_fold_hash(&scramble) == fold);
        SL_EXPECT(sl_stack_hash(&scramble) == stack);
        SL_EXPECT(scramble.calls > 1000u);
    }
}

static void test_capacity_and_validation(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(16u, 16u);
    c.self_contact_capacity = SL_SELF_CONTACT_COUNT_MAX + 1u;
    SL_EXPECT(!sl_world_init(&world, &c));
    c.self_contact_capacity = 8u;
    c.self_contact_stiffness = -1.0f;
    SL_EXPECT(!sl_world_init(&world, &c));
    c.self_contact_stiffness = NAN;
    SL_EXPECT(!sl_world_init(&world, &c));
    SL_EXPECT(world.state == NULL);

    /* A tiny capacity drops pairs and says so, and stays intersection-free
     * in the sense that it never writes past capacity. */
    uint32_t pairs = 0;
    sl_world w2 = { 0 };
    sl_world_config c2 = sl_test_config(400u, 800u);
    c2.self_contact_capacity = 4u;
    SL_EXPECT(sl_world_init(&w2, &c2));
    sl_add_flat(&w2, 0.4f, 0.8f, 10u);
    sl_add_flat(&w2, 0.405f, 0.7f, 10u);
    sl_world_step(&w2, 1.0f / 60.0f);
    const sl_world_stats st = sl_world_stats_get(&w2);
    SL_EXPECT(st.self_contact_count <= 4u);
    SL_EXPECT(st.self_contact_dropped > 0u);
    SL_EXPECT_INT_EQ(st.self_contact_capacity, 4);
    SL_EXPECT(st.self_detection_count >= 1u);
    (void)pairs;
    sl_world_destroy(&w2);
}

static const sl_test_case k_cases[] = {
    { "triangle_closest_sampling", test_triangle_closest_matches_sampling },
    { "segment_closest_sampling", test_segment_closest_matches_sampling },
    { "contact_energy_two_stage", test_contact_energy_two_stage },
    { "flat_sheet_no_self_repulsion", test_flat_sheet_does_not_repel_itself },
    { "broad_phase_complete", test_broad_phase_finds_every_near_triangle },
    { "stacked_sheets_never_intersect", test_stacked_sheets_never_intersect },
    { "bounds_allow_rigid_motion", test_bounds_allow_rigid_motion },
    { "self_contact_deterministic", test_self_contact_is_deterministic },
    { "self_contact_parallel", test_self_contact_parallel_matches_serial },
    { "capacity_and_validation", test_capacity_and_validation },
};

int sl_contact_suite(void)
{
    return sl_run_suite("contact", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
