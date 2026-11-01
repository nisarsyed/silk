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

/* Brute-force distance from triangle t to the nearest vertex of its own
 * cloth that it does not contain. */
static float sl_brute_triangle_distance(const sl_world_state *s, uint32_t t)
{
    const uint32_t *idx = s->triangle_index + 3u * t;
    float best = INFINITY;
    for (uint32_t v = 0; v < s->vertex_count; v++) {
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

/* Detects at the world's positions and checks every nearest distance the
 * bounds use against brute force (capped at the query radius). Returns the
 * number of vertices with a triangle inside the query radius. */
static uint32_t sl_check_broad_phase(sl_world_state *s, float motion)
{
    sl_self_detect(s, motion);
    const float query = s->query_radius;
    uint32_t near = 0u;
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        const float brute = sl_min(sl_brute_vertex_distance(s, v), query);
        /* bound = gamma_p * min(vertex, edge, triangle distances); each
         * term is checked through its own array, which the bounds step
         * does not touch. */
        SL_EXPECT_NEAR(s->vertex_dmin[v], brute, 1e-6f);
        near += (brute < query) ? 1u : 0u;
    }
    /* Edge pairs come from a self traversal of the edge tree; it must
     * reach every pair within the query radius as well. */
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const float brute = sl_min(sl_brute_edge_distance(s, e), query);
        SL_EXPECT_NEAR(s->edge_dmin[e], brute, 1e-6f);
    }
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const float brute = sl_min(sl_brute_triangle_distance(s, t), query);
        SL_EXPECT_NEAR(s->triangle_dmin[t], brute, 1e-6f);
    }
    return near;
}

static void test_broad_phase_finds_every_near_triangle(void)
{
    /* Crumple a sheet deterministically; the detected distances must equal
     * brute force, i.e. the broad phase misses nothing. */
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
    (void)sl_check_broad_phase(s, 0.08f);
    /* The motion term widens the query past the 2 cm contact radius. */
    SL_EXPECT(s->query_radius > 0.02f);
    sl_world_destroy(&world);

    /* The crumple leaves few vertices near anything (7 of 169), so a sheet
     * folded 7 mm onto itself, inside the 8 mm query radius, checks the
     * same with most vertices near another layer, across several detection
     * chunks. Boxes 7 mm apart also catch a leaf cull missing a millimetre
     * of its margin. */
    SL_EXPECT(sl_contact_world(&world, 8192u, 0.004f, 0.6f, 20u, 0.5f));
    s = world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->position[i].y < 0.2f) {
            s->position[i].y = 0.4f - s->position[i].y;
            s->position[i].z = 0.007f;
        }
    }
    SL_EXPECT(s->vertex_count > 3u * SL_DETECT_CHUNK_VERTICES);
    SL_EXPECT(sl_check_broad_phase(s, 0.0f) > s->vertex_count / 3u);
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

/* Leaf pairs of a self traversal from (a, b), as sorted keys
 * (lower leaf start << 32 | higher), appended at keys[*count]. */
static void sl_collect_leaf_pairs(const sl_bvh *bvh, float grow, uint32_t a,
                                  uint32_t b, uint64_t *keys, uint32_t *count,
                                  uint32_t capacity)
{
    sl_bvh_pair_cursor cursor;
    sl_bvh_pair_cursor_start(&cursor, grow, a, b);
    uint32_t a_first;
    uint32_t a_count;
    uint32_t b_first;
    uint32_t b_count;
    bool same;
    while (sl_bvh_next_leaf_pair(&cursor, bvh, &a_first, &a_count, &b_first,
                                 &b_count, &same)) {
        SL_EXPECT(same == (a_first == b_first));
        const uint32_t lo = (a_first < b_first) ? a_first : b_first;
        const uint32_t hi = (a_first < b_first) ? b_first : a_first;
        if (*count < capacity) {
            keys[*count] = ((uint64_t)lo << 32u) | hi;
        }
        (*count)++;
    }
}

static void sl_sort_keys(uint64_t *keys, uint32_t count)
{
    for (uint32_t i = 1; i < count; i++) {
        const uint64_t key = keys[i];
        uint32_t j = i;
        for (; j > 0u && keys[j - 1u] > key; j--) {
            keys[j] = keys[j - 1u];
        }
        keys[j] = key;
    }
}

static void test_traversal_frontier_covers_once(void)
{
    /* The edge tree of a crumpled sheet: the traversals started at the
     * frontier's pairs must yield exactly the leaf pairs of one full
     * traversal, each once, at any frontier size. */
    sl_world world = { 0 };
    SL_EXPECT(sl_contact_world(&world, 8192u, 0.01f, 1.0f, 12u, 1.0f));
    sl_world_state *s = world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        const sl_vec3 p = s->position[i];
        s->position[i] = sl_vec3_make(p.x, p.y + 0.15f * sinf(9.0f * p.x),
                                      0.25f * cosf(7.0f * p.y));
    }
    sl_self_detect(s, 0.08f);
    const sl_bvh *bvh = &s->edge_bvh;
    const float grow = s->query_radius;
    const uint32_t capacity = 1u << 16;
    uint64_t *full = malloc(sizeof(uint64_t) * capacity);
    uint64_t *part = malloc(sizeof(uint64_t) * capacity);
    uint32_t (*pairs)[2] = malloc(sizeof(*pairs) * SL_DETECT_FRONTIER_MAX);
    uint32_t (*scratch)[2] = malloc(sizeof(*scratch) * SL_DETECT_FRONTIER_MAX);
    SL_EXPECT(full != NULL && part != NULL && pairs != NULL && scratch != NULL);
    if (full != NULL && part != NULL && pairs != NULL && scratch != NULL) {
        uint32_t full_count = 0u;
        sl_collect_leaf_pairs(bvh, grow, 0u, 0u, full, &full_count, capacity);
        SL_EXPECT(full_count > 100u && full_count <= capacity);
        sl_sort_keys(full, full_count);
        const uint32_t targets[5] = { 1u, 3u, 40u, 256u, 100000u };
        const uint32_t limits[5] = { 1u, 8u, 64u, SL_DETECT_FRONTIER_MAX,
                                     SL_DETECT_FRONTIER_MAX };
        for (uint32_t t = 0; t < 5u; t++) {
            const uint32_t n = sl_bvh_pair_frontier(bvh, grow, pairs, scratch,
                                                    limits[t], targets[t]);
            SL_EXPECT(n >= 1u && n <= limits[t]);
            if (t >= 2u) {
                SL_EXPECT(n > 1u); /* the frontier did split */
            }
            uint32_t part_count = 0u;
            for (uint32_t k = 0; k < n; k++) {
                sl_collect_leaf_pairs(bvh, grow, pairs[k][0], pairs[k][1], part,
                                      &part_count, capacity);
            }
            SL_EXPECT_INT_EQ(part_count, full_count);
            if (part_count == full_count) {
                sl_sort_keys(part, part_count);
                SL_EXPECT(memcmp(part, full, sizeof(uint64_t) * full_count) ==
                          0);
            }
        }
    }
    free(full);
    free(part);
    free(pairs);
    free(scratch);
    sl_world_destroy(&world);
}

/* The stacked-sheet drop with the detection record buffer cut to
 * `records`: its hash, and how many chunks ran twice or inline. */
static uint64_t sl_records_hash(uint32_t records, sl_test_scramble *scramble,
                                uint32_t *reruns, uint32_t *inlined)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = 4096u;
    if (scramble != NULL) {
        c.parallel_for = sl_test_scrambled_for;
        c.parallel_context = scramble;
    }
    SL_EXPECT(sl_world_init(&world, &c));
    sl_world_state *s = world.state;
    if (records < s->record_capacity) {
        s->record_capacity = records;
    }
    sl_add_flat(&world, 0.4f, 0.8f, 10u);
    sl_add_flat(&world, 0.45f, 0.7f, 10u);
    const sl_collider sphere = { SL_COLLIDER_SPHERE, sl_vec3_zero(),
                                 sl_vec3_zero(), 0.2f, 0.5f };
    uint32_t id = 0;
    SL_EXPECT(sl_world_collider_add(&world, &sphere, &id));
    *reruns = 0u;
    *inlined = 0u;
    for (uint32_t k = 0; k < 60u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
        *reruns += s->self_overflows;
        *inlined += s->self_inline;
    }
    const uint64_t hash = sl_test_world_hash(&world);
    sl_world_destroy(&world);
    return hash;
}

static void test_detection_overflow_is_exact(void)
{
    /* Chunks that outgrow their record regions run again, or inline in the
     * merge when the buffer is short; neither may change the result. */
    uint32_t reruns = 0u;
    uint32_t inlined = 0u;
    const uint64_t full = sl_records_hash(UINT32_MAX, NULL, &reruns, &inlined);
    SL_EXPECT_INT_EQ(inlined, 0);
    SL_EXPECT(sl_records_hash(64u, NULL, &reruns, &inlined) == full);
    SL_EXPECT(reruns > 0u);
    SL_EXPECT(sl_records_hash(0u, NULL, &reruns, &inlined) == full);
    SL_EXPECT(inlined > 0u);
    sl_test_scramble scramble = { 0x2545F491u, 0u };
    SL_EXPECT(sl_records_hash(64u, &scramble, &reruns, &inlined) == full);
    SL_EXPECT(reruns > 0u && scramble.calls > 1000u);
}

/* A 12 x 12 sheet draped on a sphere, then one vertex dragged at 6 m/s
 * up, across and back down through the sheet, held by a pin (spring 0) or
 * an attachment. Returns the worst crossing count over the drag. */
static uint32_t sl_grab_drag(float spring)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    c.self_contact_capacity = 4096u;
    (void)sl_world_init(&world, &c);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.thickness = 0.004f;
    d.damping = 20.0f;
    d.drag = 0.3f;
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.4f, 0.6f, -0.4f), sl_vec3_make(0.8f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, 0.0f, 0.8f), 12u, 12u);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t id = 0;
    SL_EXPECT(sl_world_cloth_add(&world, &d, &id));
    sl_test_grid_free(&g);
    const sl_collider sphere = { SL_COLLIDER_SPHERE,
                                 sl_vec3_make(0.0f, 0.3f, 0.0f), sl_vec3_zero(),
                                 0.2f, 0.5f };
    SL_EXPECT(sl_world_collider_add(&world, &sphere, &id));
    for (uint32_t k = 0; k < 60u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
    }
    const uint32_t held = 3u * 13u + 6u;
    if (spring > 0.0f) {
        SL_EXPECT(sl_world_vertex_attach_set(&world, 0u, held, spring));
    } else {
        SL_EXPECT(sl_world_vertex_pin_set(&world, 0u, held, true));
    }
    sl_vec3 target = sl_world_cloth_positions(&world, 0u)[held];
    const sl_vec3 legs[3] = { sl_vec3_make(0.0f, 1.0f, 0.0f),
                              sl_vec3_make(0.3f, 0.0f, 1.0f),
                              sl_vec3_make(-0.2f, -1.0f, -0.5f) };
    const float lengths[3] = { 0.4f, 0.6f, 0.5f };
    uint32_t worst = 0u;
    for (uint32_t leg = 0; leg < 3u; leg++) {
        const sl_vec3 step = sl_vec3_scale(sl_vec3_normalize(legs[leg]),
                                           6.0f / 60.0f); /* 6 m/s */
        const uint32_t steps = (uint32_t)(lengths[leg] / 0.1f + 0.5f);
        for (uint32_t k = 0; k < steps; k++) {
            target = sl_vec3_add(target, step);
            SL_EXPECT(sl_world_vertex_target_set(&world, 0u, held, target));
            sl_world_step(&world, 1.0f / 60.0f);
            const uint32_t crossings = sl_crossings(world.state);
            worst = (crossings > worst) ? crossings : worst;
        }
    }
    sl_world_destroy(&world);
    return worst;
}

static void test_grab_by_attachment_never_crosses(void)
{
    /* A pinned vertex is not bounded: dragged fast, its triangles pass
     * through the sheet (the sandbox grab glitch). An attached vertex
     * keeps its bounds and trails the anchor instead, at any stiffness. */
    SL_EXPECT(sl_grab_drag(0.0f) > 0u);
    SL_EXPECT_INT_EQ(sl_grab_drag(100.0f), 0);
    SL_EXPECT_INT_EQ(sl_grab_drag(1.0e4f), 0);
}

static void test_bounds_keep_a_rounding_floor(void)
{
    /* Two flat sheets 50 nm apart, under the bound floor (64 ulps of the
     * 0.4 m coordinates, 3 um): no relative motion is allowed at all, since
     * rounding alone could carry a vertex across. 1 mm apart they get
     * bounds as usual. */
    const float gaps[2] = { 5.0e-8f, 1.0e-3f };
    for (uint32_t k = 0; k < 2u; k++) {
        sl_world world = { 0 };
        sl_world_config c = sl_test_config(400u, 800u);
        c.self_contact_capacity = 4096u;
        SL_EXPECT(sl_world_init(&world, &c));
        sl_add_flat(&world, 0.4f, 0.8f, 6u);
        sl_add_flat(&world, 0.4f + gaps[k], 0.7f, 6u);
        sl_world_state *s = world.state;
        sl_self_detect(s, 0.0f);
        float bound_max = 0.0f;
        for (uint32_t v = 0; v < s->vertex_count; v++) {
            bound_max = sl_max(bound_max, s->bound[v]);
        }
        if (k == 0u) {
            SL_EXPECT(bound_max == 0.0f);
            SL_EXPECT(s->cloths[0].translation_cap == 0.0f);
        } else {
            SL_EXPECT(bound_max > 1e-4f);
        }
        sl_world_destroy(&world);
    }

    /* The same within one cloth: a sheet folded 50 nm onto itself. Every
     * vertex nearer than the floor to a triangle of its own gets no bound. */
    sl_world world = { 0 };
    SL_EXPECT(sl_contact_world(&world, 8192u, 0.004f, 0.6f, 20u, 0.5f));
    sl_world_state *s = world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        if (s->position[i].y < 0.2f) {
            s->position[i].y = 0.4f - s->position[i].y;
            s->position[i].z = 5.0e-8f;
        }
    }
    sl_self_detect(s, 0.0f);
    uint32_t touching = 0u;
    for (uint32_t v = 0; v < s->vertex_count; v++) {
        if (s->vertex_dmin[v] < 1e-6f) {
            touching++;
            SL_EXPECT(s->bound[v] == 0.0f);
        }
    }
    SL_EXPECT(touching > s->vertex_count / 3u);
    sl_world_destroy(&world);
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

/* Exact equality: the query copies positions, so equality is the
 * property under test. */
static bool sl_vec3_equal(sl_vec3 a, sl_vec3 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/* Is p exactly one of the world's vertex positions? */
static bool sl_is_vertex(const sl_world *w, sl_vec3 p)
{
    for (uint32_t c = 0; c < sl_world_stats_get(w).cloth_count; c++) {
        const sl_vec3 *x = sl_world_cloth_positions(w, c);
        for (uint32_t i = 0; i < sl_world_cloth_vertex_count(w, c); i++) {
            if (sl_vec3_equal(x[i], p)) {
                return true;
            }
        }
    }
    return false;
}

/* Distances from p to the nearest triangle and to the nearest edge. */
static float sl_triangle_gap(const sl_world_state *s, sl_vec3 p)
{
    float gap = INFINITY;
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        gap = sl_min(gap, sl_triangle_closest_point(p, s->position[idx[0]],
                                                    s->position[idx[1]],
                                                    s->position[idx[2]])
                              .distance);
    }
    return gap;
}

static float sl_edge_gap(const sl_world_state *s, sl_vec3 p)
{
    float gap = INFINITY;
    for (uint32_t e = 0; e < s->edge_count; e++) {
        const sl_vec3 a = s->position[s->edge_index[2u * e]];
        const sl_vec3 b = s->position[s->edge_index[2u * e + 1u]];
        const float t = sl_segment_closest_param(p, a, b);
        gap = sl_min(gap, sl_vec3_length(sl_vec3_sub(
                              p, sl_vec3_mul_add(a, sl_vec3_sub(b, a), t))));
    }
    return gap;
}

static void test_self_contact_query(void)
{
    /* Two sheets dropped flat onto the ground, the upper one resting on
     * the lower inside the 8 mm contact radius. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = 4096u;
    SL_EXPECT(sl_world_init(&world, &c));
    sl_add_flat(&world, 0.1f, 0.8f, 10u);
    sl_add_flat(&world, 0.15f, 0.7f, 10u);
    const sl_collider ground = { SL_COLLIDER_PLANE, sl_vec3_zero(),
                                 sl_vec3_make(0.0f, 1.0f, 0.0f), 0.0f, 0.5f };
    uint32_t id = 0;
    SL_EXPECT(sl_world_collider_add(&world, &ground, &id));
    SL_EXPECT_INT_EQ(sl_world_self_contacts(&world, NULL, 0u), 0);
    for (uint32_t k = 0; k < 60u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
    }
    const uint32_t count = sl_world_stats_get(&world).self_contact_count;
    SL_EXPECT(count > 10u && count <= 4096u);
    sl_self_contact *all = malloc(sizeof(sl_self_contact) * count);
    SL_EXPECT(all != NULL);
    if (all == NULL) {
        sl_world_destroy(&world);
        return;
    }
    /* No degenerate primitive in a flat grid, so every pair is listed. */
    SL_EXPECT_INT_EQ(sl_world_self_contacts(&world, all, count), count);
    uint32_t kinds[2] = { 0u, 0u };
    uint32_t pushing = 0u;
    float gap_max = 0.0f; /* of a closest point from its primitive, m */
    for (uint32_t k = 0; k < count; k++) {
        const sl_self_contact *p = &all[k];
        SL_EXPECT(sl_vec3_is_finite(p->point_a) &&
                  sl_vec3_is_finite(p->point_b));
        SL_EXPECT_NEAR(p->radius, 0.008f, 1e-6f);
        SL_EXPECT(p->kind == SL_SELF_CONTACT_VERTEX_FACET ||
                  p->kind == SL_SELF_CONTACT_EDGE_EDGE);
        kinds[p->kind == SL_SELF_CONTACT_EDGE_EDGE ? 1 : 0]++;
        /* The sheets lie flat: the closest points are one above the
         * other, and a vertex pair starts at a vertex. */
        const sl_vec3 d = sl_vec3_sub(p->point_a, p->point_b);
        SL_EXPECT(sl_abs(d.y) > 0.9f * sl_vec3_length(d));
        SL_EXPECT(sl_vec3_length(d) < 2.0f * p->radius);
        if (p->kind == SL_SELF_CONTACT_VERTEX_FACET) {
            SL_EXPECT(sl_is_vertex(&world, p->point_a));
            gap_max = sl_max(gap_max, sl_triangle_gap(world.state, p->point_b));
        } else {
            gap_max = sl_max(gap_max, sl_edge_gap(world.state, p->point_a));
            gap_max = sl_max(gap_max, sl_edge_gap(world.state, p->point_b));
        }
        pushing += (sl_vec3_length(d) < p->radius) ? 1u : 0u;
    }
    SL_EXPECT(kinds[0] > 0u && kinds[1] > 0u);
    SL_EXPECT(pushing > 0u);
    /* Each closest point lies on its primitive up to float rounding
     * (measured 3e-8 m). */
    SL_EXPECT(gap_max < 1e-6f);
    /* A short buffer gets the leading pairs, in the same order. */
    sl_self_contact head[3];
    SL_EXPECT_INT_EQ(sl_world_self_contacts(&world, head, 3u), 3);
    for (uint32_t k = 0; k < 3u; k++) {
        SL_EXPECT(sl_vec3_equal(head[k].point_a, all[k].point_a) &&
                  sl_vec3_equal(head[k].point_b, all[k].point_b) &&
                  head[k].kind == all[k].kind);
    }
    free(all);
    sl_world_destroy(&world);

    /* Disabled self contact lists nothing and flags no bound. */
    SL_EXPECT(sl_contact_world(&world, 0u, 0.004f, 0.6f, 6u, 0.5f));
    sl_world_step(&world, 1.0f / 60.0f);
    sl_self_contact one;
    SL_EXPECT_INT_EQ(sl_world_self_contacts(&world, &one, 1u), 0);
    SL_EXPECT_INT_EQ(sl_world_vertex_flags(&world, 0u, 0u), 0);
    sl_world_destroy(&world);
}

/* Counts the vertices carrying flag over every cloth. */
static uint32_t sl_flag_count(const sl_world *w, uint32_t flag)
{
    uint32_t count = 0u;
    for (uint32_t c = 0; c < sl_world_stats_get(w).cloth_count; c++) {
        for (uint32_t i = 0; i < sl_world_cloth_vertex_count(w, c); i++) {
            count += ((sl_world_vertex_flags(w, c, i) & flag) != 0u) ? 1u : 0u;
        }
    }
    return count;
}

static void test_vertex_flags(void)
{
    /* Two sheets dropped on a sphere above the ground: the flags match the
     * collider contact count and the distinct bound-held vertices at every
     * step, and both occur. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(400u, 800u);
    c.self_contact_capacity = 4096u;
    SL_EXPECT(sl_world_init(&world, &c));
    sl_add_flat(&world, 0.4f, 0.8f, 10u);
    sl_add_flat(&world, 0.5f, 0.7f, 10u);
    const sl_collider sphere = { SL_COLLIDER_SPHERE, sl_vec3_zero(),
                                 sl_vec3_zero(), 0.2f, 0.5f };
    uint32_t id = 0;
    SL_EXPECT(sl_world_collider_add(&world, &sphere, &id));
    uint32_t touching_max = 0u;
    uint32_t held_max = 0u;
    for (uint32_t k = 0; k < 90u; k++) {
        sl_world_step(&world, 1.0f / 60.0f);
        const uint32_t touching =
            sl_flag_count(&world, SL_VERTEX_COLLIDER_CONTACT);
        const uint32_t held = sl_flag_count(&world, SL_VERTEX_AT_BOUND);
        SL_EXPECT_INT_EQ(touching, sl_world_stats_get(&world).contact_count);
        SL_EXPECT_INT_EQ(held, world.state->bound_exceeded);
        touching_max = (touching > touching_max) ? touching : touching_max;
        held_max = (held > held_max) ? held : held_max;
    }
    SL_EXPECT(touching_max > 0u);
    SL_EXPECT(held_max > 0u);
    SL_EXPECT_INT_EQ(sl_world_vertex_flags(&world, 2u, 0u), 0);
    SL_EXPECT_INT_EQ(sl_world_vertex_flags(&world, 0u, 121u), 0);
    sl_world_destroy(&world);
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
    { "traversal_frontier_covers_once", test_traversal_frontier_covers_once },
    { "detection_overflow_is_exact", test_detection_overflow_is_exact },
    { "grab_by_attachment_never_crosses",
      test_grab_by_attachment_never_crosses },
    { "bounds_keep_a_rounding_floor", test_bounds_keep_a_rounding_floor },
    { "self_contact_query", test_self_contact_query },
    { "vertex_flags", test_vertex_flags },
};

int sl_contact_suite(void)
{
    return sl_run_suite("contact", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
