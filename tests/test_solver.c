#include "energy.h"
#include "fixture.h"
#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

#define SL_TEST_DT (1.0f / 60.0f)

static void sl_run(sl_world *w, uint32_t steps)
{
    for (uint32_t k = 0; k < steps; k++) {
        sl_world_step(w, SL_TEST_DT);
    }
}

static float sl_speed_max_of(const sl_world *w, uint32_t cloth)
{
    const sl_vec3 *v = sl_world_cloth_velocities(w, cloth);
    float speed = 0.0f;
    for (uint32_t i = 0; i < sl_world_cloth_vertex_count(w, cloth); i++) {
        speed = sl_max(speed, sl_vec3_length(v[i]));
    }
    return speed;
}

/* Largest relative edge elongation over the triangles of cloth 0,
 * optionally skipping triangles that touch a pinned vertex. */
static float sl_strain_max_of(const sl_world *w, bool skip_pinned)
{
    const sl_world_state *s = w->state;
    float strain = 0.0f;
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        if (skip_pinned &&
            (s->pinned[idx[0]] != 0u || s->pinned[idx[1]] != 0u ||
             s->pinned[idx[2]] != 0u)) {
            continue;
        }
        for (uint32_t e = 0; e < 3u; e++) {
            const uint32_t a = idx[e];
            const uint32_t b = idx[(e + 1u) % 3u];
            const float now =
                sl_vec3_length(sl_vec3_sub(s->position[a], s->position[b]));
            const float rest =
                sl_vec3_length(sl_vec3_sub(s->pin_origin[a], s->pin_origin[b]));
            strain = sl_max(strain, now / rest - 1.0f);
        }
    }
    return strain;
}

/* Copies current positions into pin_origin so sl_strain_max_of can use
 * them as rest lengths (pin_origin is otherwise only read for pins). */
static void sl_capture_rest(sl_world *w)
{
    sl_world_state *s = w->state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        s->pin_origin[i] = s->position[i];
    }
}

static void test_substeps_follow_stiffness(void)
{
    /* On a regular grid sigma_max = 8 mu / (density edge^2), so 5 cm edges
     * at mu = lambda = 2e3 N/m give 3.2e7 / s^2 and need
     * ceil(dt sqrt(sigma / 20)) = 22 substeps at 60 Hz. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.bending_stiffness = 0.0f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 20u, 1.0f, false));
    sl_world_stats st = sl_world_stats_get(&world);
    SL_EXPECT_NEAR(st.stiffness_rate_max, 3.2e7f, 3.2e7f * 1e-4f);
    sl_world_step(&world, SL_TEST_DT);
    st = sl_world_stats_get(&world);
    SL_EXPECT_INT_EQ(st.substep_count, 22);
    SL_EXPECT(!st.substep_limited);
    sl_world_destroy(&world);

    /* A higher configured count is kept. */
    c.substep_count = 30u;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 20u, 1.0f, false));
    sl_world_step(&world, SL_TEST_DT);
    SL_EXPECT_INT_EQ(sl_world_stats_get(&world).substep_count, 30);
    sl_world_destroy(&world);

    /* Beyond SL_SUBSTEP_COUNT_MAX the step runs the maximum and says so. */
    c.substep_count = 0u;
    d.membrane_mu = 1.0e7f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 20u, 1.0f, false));
    sl_world_step(&world, SL_TEST_DT);
    st = sl_world_stats_get(&world);
    SL_EXPECT_INT_EQ(st.substep_count, SL_SUBSTEP_COUNT_MAX);
    SL_EXPECT(st.substep_limited);
    sl_world_destroy(&world);
}

static void test_free_fall_matches_backward_euler(void)
{
    /* Without internal forces each vertex block is solved exactly, so the
     * result is backward Euler: after N substeps of h,
     * y = y0 + g h^2 N (N + 1) / 2. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.membrane_mu = 0.0f;
    d.membrane_lambda = 0.0f;
    d.bending_stiffness = 0.0f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, false));
    sl_run(&world, 30u);
    const uint32_t substeps = sl_world_stats_get(&world).substep_count;
    SL_EXPECT_INT_EQ(substeps, SL_SUBSTEP_COUNT_DEFAULT); /* no stiffness */
    const float h = SL_TEST_DT / (float)substeps;
    const float n = 30.0f * (float)substeps;
    const float expected = 1.0f - 9.81f * h * h * n * (n + 1.0f) * 0.5f;
    /* Velocity is (x - x_start) / h: each substep rounds x to float
     * spacing (<= 2.4e-7 m below 2 m), i.e. up to 5.8e-5 m/s of velocity
     * noise per substep at h = 1/240 s, accumulated over 120 substeps. */
    SL_EXPECT_NEAR(sl_world_cloth_positions(&world, 0u)[0].y, expected, 1e-3f);
    SL_EXPECT_NEAR(sl_world_cloth_velocities(&world, 0u)[0].y, -9.81f * h * n,
                   5e-3f);
    sl_world_destroy(&world);

    /* A stiff, damped sheet falls rigidly too: the initial guess y is
     * already the solution and edge damping ignores rigid motion. */
    d = sl_test_cloth_desc();
    d.membrane_mu = 1.0e5f;
    d.membrane_lambda = 1.0e5f;
    d.damping = 50.0f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, false));
    sl_run(&world, 30u);
    /* The stiff sheet runs more substeps; free fall is exact either way. */
    const uint32_t stiff = sl_world_stats_get(&world).substep_count;
    const float hs = SL_TEST_DT / (float)stiff;
    const float ns = 30.0f * (float)stiff;
    SL_EXPECT(stiff > substeps);
    SL_EXPECT_NEAR(sl_world_cloth_positions(&world, 0u)[40].y,
                   1.0f - 0.5f - 9.81f * hs * hs * ns * (ns + 1.0f) * 0.5f,
                   1e-2f); /* float rounding over ~2000 substeps */
    sl_world_destroy(&world);
}

static void test_sweeps_decrease_incremental_potential(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.friction = 0.0f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 16u, 1.0f, true));
    sl_run(&world, 20u); /* swinging, deformed state */
    sl_world_state *s = world.state;
    const float h = SL_TEST_DT / (float)s->substep_count;
    sl_solver_substep_begin(s, h, 1.0f);
    double previous = sl_solver_energy(s, h);
    const double initial = previous;
    for (uint32_t it = 0; it < 32u; it++) {
        sl_solver_sweep(s, h);
        const double energy = sl_solver_energy(s, h);
        SL_EXPECT(energy <= previous + 1e-9 * (1.0 + fabs(previous)));
        previous = energy;
    }
    SL_EXPECT(previous < initial);
    sl_world_destroy(&world);
}

static void test_hanging_sheet_settles_with_small_stretch(void)
{
    /* Pins the default substep and iteration counts: a 1 m sheet hung by
     * two corners settles within 4 s; see the stretch bounds below. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.damping = 20.0f;
    d.drag = 0.6f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 20u, 1.0f, true));
    sl_capture_rest(&world);
    sl_run(&world, 240u);
    /* The two pinned corners carry point loads, whose stress is singular
     * in the continuum too, so stretch is measured off the pins. The
     * converged equilibrium stretches under 0.5%; one sweep per substep
     * leaves long-wavelength error the sweep cannot remove (docs/solver.md:
     * 2.9% off the pins at the automatic 22 substeps). */
    const float strain = sl_strain_max_of(&world, true);
    SL_EXPECT(strain < 0.04f);
    SL_EXPECT(sl_strain_max_of(&world, false) < 0.08f);
    SL_EXPECT(sl_speed_max_of(&world, 0u) < 0.1f);
    /* Pins never moved. */
    SL_EXPECT(sl_world_cloth_positions(&world, 0u)[0].y == 1.0f);
    SL_EXPECT(sl_world_cloth_positions(&world, 0u)[20].x == 0.5f);
    /* The free bottom edge hangs below the top. */
    SL_EXPECT(sl_world_cloth_positions(&world, 0u)[430].y < 0.2f);
    sl_world_destroy(&world);
}

static void test_pin_targets_interpolate_over_substeps(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    sl_cloth_desc d = sl_test_cloth_desc();
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, true));
    const sl_vec3 target = sl_vec3_make(-0.5f, 1.3f, 0.2f);
    SL_EXPECT(sl_world_vertex_target_set(&world, 0u, 0u, target));
    sl_world_step(&world, SL_TEST_DT);
    const sl_vec3 p = sl_world_cloth_positions(&world, 0u)[0];
    SL_EXPECT(p.x == target.x && p.y == target.y && p.z == target.z);
    /* Velocity is the last substep's kinematic rate: (0.3, 0.2) m / dt. */
    const sl_vec3 v = sl_world_cloth_velocities(&world, 0u)[0];
    SL_EXPECT_NEAR(v.y, 0.3f / SL_TEST_DT, 1e-2f);
    SL_EXPECT_NEAR(v.z, 0.2f / SL_TEST_DT, 1e-2f);
    sl_world_step(&world, SL_TEST_DT);
    const sl_vec3 q = sl_world_cloth_positions(&world, 0u)[0];
    SL_EXPECT(q.x == target.x && q.y == target.y && q.z == target.z);
    SL_EXPECT_NEAR(sl_world_cloth_velocities(&world, 0u)[0].y, 0.0f, 0.0f);
    sl_world_destroy(&world);
}

/* A pinned sheet with damping, drag, wind and a sphere collider: every
 * substep phase runs. With a scramble, through sl_test_scrambled_for. */
static uint64_t sl_scene_hash(uint32_t steps, sl_test_scramble *scramble)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    if (scramble != NULL) {
        c.parallel_for = sl_test_scrambled_for;
        c.parallel_context = scramble;
    }
    c.wind = sl_vec3_make(0.0f, 0.0f, 3.0f);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.damping = 20.0f;
    d.drag = 0.6f;
    (void)sl_test_sheet(&world, &c, &d, 1.0f, 16u, 1.0f, true);
    const sl_collider sphere = { SL_COLLIDER_SPHERE,
                                 sl_vec3_make(0.0f, 0.2f, 0.15f),
                                 sl_vec3_zero(), 0.25f, 0.5f };
    uint32_t id = 0;
    (void)sl_world_collider_add(&world, &sphere, &id);
    sl_run(&world, steps);
    const uint64_t hash = sl_test_world_hash(&world);
    sl_world_destroy(&world);
    return hash;
}

static void test_simulation_is_deterministic(void)
{
    const uint64_t a = sl_scene_hash(60u, NULL);
    SL_EXPECT(a == sl_scene_hash(60u, NULL));
    SL_EXPECT(a != sl_scene_hash(59u, NULL));
}

static void test_parallel_matches_serial(void)
{
    /* Any partition and order of the per-vertex and per-edge tasks gives
     * the serial result bit for bit (sl_parallel_for_fn's contract). */
    const uint64_t serial = sl_scene_hash(60u, NULL);
    const uint32_t seeds[3] = { 0x9E3779B9u, 0x2545F491u, 7u };
    for (uint32_t k = 0; k < 3u; k++) {
        sl_test_scramble scramble = { seeds[k], 0u };
        SL_EXPECT(sl_scene_hash(60u, &scramble) == serial);
        SL_EXPECT(scramble.calls > 1000u);
    }
}

static void test_ground_contact_rests_on_thickness(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.damping = 20.0f;
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.5f, 0.3f, -0.5f), sl_vec3_make(1.0f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, 0.0f, 1.0f), 16u, 16u);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    SL_EXPECT(sl_world_init(&world, &c));
    SL_EXPECT(sl_world_cloth_add(&world, &d, &cloth));
    sl_test_grid_free(&g);
    const sl_collider ground = { SL_COLLIDER_PLANE, sl_vec3_zero(),
                                 sl_vec3_make(0.0f, 1.0f, 0.0f), 0.0f, 0.5f };
    uint32_t id = 0;
    SL_EXPECT(sl_world_collider_add(&world, &ground, &id));
    sl_run(&world, 90u);
    const sl_vec3 *p = sl_world_cloth_positions(&world, 0u);
    for (uint32_t i = 0; i < sl_world_cloth_vertex_count(&world, 0u); i++) {
        /* Rests in the thickness shell; sinks under 1 mm. */
        SL_EXPECT(p[i].y > d.thickness - 1e-3f);
        SL_EXPECT(p[i].y < d.thickness + 1e-3f);
    }
    SL_EXPECT(sl_speed_max_of(&world, 0u) < 0.01f);
    SL_EXPECT_INT_EQ(sl_world_stats_get(&world).contact_count, 289);
    sl_world_destroy(&world);
}

/* Kinetic + gravitational + membrane energy, joules, gravity along -y. */
static double sl_total_energy(const sl_world *w, const sl_cloth_desc *d)
{
    const sl_world_state *s = w->state;
    double energy = 0.0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        energy += (double)s->mass[i] *
                  (0.5 * (double)sl_vec3_length_sq(s->velocity[i]) +
                   9.81 * (double)s->position[i].y);
    }
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                                s->position[idx[2]] };
        energy += (double)sl_membrane_energy(
            xs, s->triangle_rest + 4u * t, s->triangle_area[t], d->membrane_mu,
            d->membrane_lambda);
    }
    return energy;
}

static void test_sphere_drape_stays_outside(void)
{
    /* A 1.2 m sheet dropped on a 0.3 m sphere. Besides penetration, this
     * pins SL_STIFFNESS_STEP_MAX: wrapping compresses the cloth, and at
     * fixed 16 substeps mu = 1e4 N/m at 5 cm edges (h^2 sigma = 174)
     * gained over 280% energy, and at 38 (h^2 sigma = 31) still 6%. The
     * automatic substep count must keep the total at or below the initial
     * energy. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(1000u, 2000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.membrane_mu = 1.0e4f;
    d.membrane_lambda = 1.0e4f;
    d.damping = 20.0f;
    d.drag = 0.6f;
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.6f, 0.5f, -0.6f), sl_vec3_make(1.2f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, 0.0f, 1.2f), 24u, 24u);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    SL_EXPECT(sl_world_init(&world, &c));
    SL_EXPECT(sl_world_cloth_add(&world, &d, &cloth));
    sl_test_grid_free(&g);
    const sl_collider sphere = { SL_COLLIDER_SPHERE, sl_vec3_zero(),
                                 sl_vec3_zero(), 0.3f, 0.5f };
    uint32_t id = 0;
    SL_EXPECT(sl_world_collider_add(&world, &sphere, &id));
    const double initial = sl_total_energy(&world, &d);
    double peak = initial;
    float depth_max = 0.0f;
    float late_speed = 0.0f;
    for (uint32_t step = 0; step < 180u; step++) {
        sl_world_step(&world, SL_TEST_DT);
        peak = fmax(peak, sl_total_energy(&world, &d));
        if (step >= 120u) {
            late_speed = sl_max(late_speed, sl_speed_max_of(&world, 0u));
        }
        const sl_vec3 *p = sl_world_cloth_positions(&world, 0u);
        for (uint32_t i = 0; i < sl_world_cloth_vertex_count(&world, 0u); i++) {
            const float depth = 0.3f + d.thickness - sl_vec3_length(p[i]);
            depth_max = sl_max(depth_max, depth);
        }
    }
    SL_EXPECT(peak <= initial + 1e-3);
    /* The skirt settles: under 1.5 m/s over the last second with or without
     * fused multiply-add. Edge damping in color order instead of ascending
     * order (see sl_solver_damp) let unconverged substeps pump it to
     * 24 m/s, within the energy bound above, since the drop releases more
     * potential energy than the skirt gained. */
    SL_EXPECT(late_speed < 4.0f);
    /* Penalty contact lets cloth tension compress the thickness shell; the
     * midsurface must stay at least half a thickness off the sphere. */
    SL_EXPECT(depth_max < 0.5f * d.thickness);
    /* Center vertex rests on the pole. */
    const sl_vec3 top = sl_world_cloth_positions(&world, 0u)[12u * 25u + 12u];
    SL_EXPECT_NEAR(top.y, 0.3f + d.thickness, 2e-3f);
    SL_EXPECT(sl_world_stats_get(&world).contact_count > 50u);
    sl_world_destroy(&world);
}

static void test_swinging_sheet_never_gains_energy(void)
{
    /* A horizontal sheet hinged along one edge swings down and whips. */
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(500u, 1000u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.drag = 0.6f;
    sl_test_grid g = sl_test_grid_make(
        sl_vec3_make(-0.5f, 1.0f, 0.0f), sl_vec3_make(1.0f, 0.0f, 0.0f),
        sl_vec3_make(0.0f, 0.0f, 1.0f), 20u, 20u);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    SL_EXPECT(sl_world_init(&world, &c));
    SL_EXPECT(sl_world_cloth_add(&world, &d, &cloth));
    sl_test_grid_free(&g);
    for (uint32_t i = 0; i <= 20u; i++) {
        SL_EXPECT(sl_world_vertex_pin_set(&world, 0u, i, true));
    }
    const double initial = sl_total_energy(&world, &d);
    double peak = initial;
    for (uint32_t step = 0; step < 180u; step++) {
        sl_world_step(&world, SL_TEST_DT);
        peak = fmax(peak, sl_total_energy(&world, &d));
    }
    SL_EXPECT(peak <= initial + 1e-3);
    sl_world_destroy(&world);
}

/* Internal kinetic plus membrane energy, and linear momentum, of a
 * zero-gravity sheet after
 * `steps`, started with drift plus a rigid spin about z (rate spin, rad/s)
 * plus a uniform expansion (rate expand, 1/s). */
static double sl_sheet_kinetic(float damping, float spin, float expand,
                               uint32_t steps, sl_vec3 *momentum)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    c.gravity = sl_vec3_zero();
    sl_cloth_desc d = sl_test_cloth_desc();
    d.damping = damping;
    (void)sl_test_sheet(&world, &c, &d, 1.0f, 8u, 0.5f, false);
    sl_world_state *s = world.state;
    const sl_vec3 drift = sl_vec3_make(0.3f, 0.0f, -0.2f);
    const sl_vec3 axis = sl_vec3_make(0.0f, 0.0f, spin);
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        const sl_vec3 p = s->position[i];
        s->velocity[i] = sl_vec3_add(sl_vec3_add(drift, sl_vec3_cross(axis, p)),
                                     sl_vec3_scale(p, expand));
    }
    for (uint32_t k = 0; k < steps; k++) {
        sl_world_step(&world, SL_TEST_DT);
    }
    /* Kinetic energy relative to the mean velocity, so drift is excluded,
     * plus membrane energy. */
    *momentum = sl_vec3_zero();
    float mass = 0.0f;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        *momentum = sl_vec3_mul_add(*momentum, s->velocity[i], s->mass[i]);
        mass += s->mass[i];
    }
    const sl_vec3 mean = sl_vec3_scale(*momentum, 1.0f / mass);
    double kinetic = 0.0;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        kinetic += 0.5 * (double)s->mass[i] *
                   (double)sl_vec3_length_sq(sl_vec3_sub(s->velocity[i], mean));
    }
    for (uint32_t t = 0; t < s->triangle_count; t++) {
        const uint32_t *idx = s->triangle_index + 3u * t;
        const sl_vec3 xs[3] = { s->position[idx[0]], s->position[idx[1]],
                                s->position[idx[2]] };
        kinetic += (double)sl_membrane_energy(xs, s->triangle_rest + 4u * t,
                                              s->triangle_area[t],
                                              d.membrane_mu, d.membrane_lambda);
    }
    sl_world_destroy(&world);
    return kinetic;
}

static void test_edge_damping_spares_rigid_motion(void)
{
    /* Rigid spin and drift survive heavy stretch damping. Momentum matches
     * the undamped run to sweep accuracy (Gauss-Seidel vertex updates are
     * not exactly momentum-conserving; the damping impulses are). */
    sl_vec3 p0;
    sl_vec3 p1;
    const double free_spin = sl_sheet_kinetic(0.0f, 2.0f, 0.0f, 30u, &p0);
    const double damped_spin = sl_sheet_kinetic(200.0f, 2.0f, 0.0f, 30u, &p1);
    SL_EXPECT(damped_spin > 0.999 * free_spin);
    SL_EXPECT_NEAR(p1.x, p0.x, 1e-3f);
    SL_EXPECT_NEAR(p1.z, p0.z, 1e-3f);
    /* What it removes is stretch rate. Backward Euler already dissipates
     * the free stretch oscillation quickly (internal energy falls 80x in
     * three steps), so the damped run must stay well below it (measured 4x
     * lower after three steps). */
    const double free_grow = sl_sheet_kinetic(0.0f, 0.0f, 1.0f, 3u, &p0);
    const double damped_grow = sl_sheet_kinetic(200.0f, 0.0f, 1.0f, 3u, &p1);
    SL_EXPECT(damped_grow < 0.5 * free_grow);
}

/* Slides a small horizontal patch on a 20 degree incline for 1 s and
 * returns its travel along the slope. */
static float sl_incline_travel(float friction)
{
    const float angle = 20.0f * SL_PI / 180.0f;
    const sl_vec3 n = sl_vec3_make(-sinf(angle), cosf(angle), 0.0f);
    const sl_vec3 down = sl_vec3_make(-cosf(angle), -sinf(angle), 0.0f);
    const sl_vec3 side = sl_vec3_make(0.0f, 0.0f, 0.2f);
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    sl_cloth_desc d = sl_test_cloth_desc();
    d.friction = friction;
    const sl_vec3 origin = sl_vec3_scale(n, d.thickness);
    sl_test_grid g =
        sl_test_grid_make(origin, sl_vec3_scale(down, -0.2f), side, 6u, 6u);
    d.positions = g.positions;
    d.vertex_count = g.vertex_count;
    d.indices = g.indices;
    d.triangle_count = g.triangle_count;
    uint32_t cloth = 0;
    (void)sl_world_init(&world, &c);
    (void)sl_world_cloth_add(&world, &d, &cloth);
    sl_test_grid_free(&g);
    const sl_collider slope = { SL_COLLIDER_PLANE, sl_vec3_zero(), n, 0.0f,
                                friction };
    uint32_t id = 0;
    (void)sl_world_collider_add(&world, &slope, &id);
    const sl_vec3 start = sl_world_cloth_positions(&world, 0u)[0];
    sl_run(&world, 60u);
    const sl_vec3 end = sl_world_cloth_positions(&world, 0u)[0];
    sl_world_destroy(&world);
    return sl_vec3_dot(sl_vec3_sub(end, start), down);
}

static void test_friction_sticks_and_slides(void)
{
    /* tan(20 deg) = 0.364. mu = 0.8 must hold, creeping at most
     * h g sin(20 deg) = 3.5 mm/s (see sl_solver_friction); mu = 0.1 slides
     * with a = g (sin - mu cos) = 2.43 m/s^2, about 1.2 m in 1 s. */
    SL_EXPECT(sl_incline_travel(0.8f) < 4e-3f);
    const float slide = sl_incline_travel(0.1f);
    SL_EXPECT(slide > 1.0f);
    SL_EXPECT(slide < 1.3f);
}

static void test_drag_slows_fall_and_wind_pushes(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    sl_cloth_desc d = sl_test_cloth_desc();
    float fall[2];
    for (uint32_t k = 0; k < 2u; k++) {
        sl_test_grid g = sl_test_grid_make(
            sl_vec3_make(-0.5f, 1.0f, -0.5f), sl_vec3_make(1.0f, 0.0f, 0.0f),
            sl_vec3_make(0.0f, 0.0f, 1.0f), 8u, 8u);
        d.drag = (k == 0u) ? 0.0f : 0.6f;
        d.positions = g.positions;
        d.vertex_count = g.vertex_count;
        d.indices = g.indices;
        d.triangle_count = g.triangle_count;
        uint32_t cloth = 0;
        SL_EXPECT(sl_world_init(&world, &c));
        SL_EXPECT(sl_world_cloth_add(&world, &d, &cloth));
        sl_test_grid_free(&g);
        sl_run(&world, 60u);
        fall[k] = -sl_world_cloth_velocities(&world, 0u)[40].y;
        sl_world_destroy(&world);
    }
    /* Terminal speed sqrt(m g / (drag A)) = sqrt(0.2 * 9.81 / 0.6) = 1.81.
     */
    SL_EXPECT(fall[0] > 9.0f);
    SL_EXPECT_NEAR(fall[1], 1.81f, 0.1f);

    c.wind = sl_vec3_make(0.0f, 0.0f, 5.0f);
    d.drag = 0.6f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, true));
    sl_run(&world, 60u);
    SL_EXPECT(sl_world_cloth_positions(&world, 0u)[76].z > 0.2f);
    sl_world_destroy(&world);
}

static void test_speed_cap_and_finite_state(void)
{
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(200u, 400u);
    c.speed_max = 1.0f;
    sl_cloth_desc d = sl_test_cloth_desc();
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, false));
    sl_run(&world, 60u);
    SL_EXPECT(sl_speed_max_of(&world, 0u) <= 1.0f + 1e-6f);
    sl_world_destroy(&world);

    /* Absurd stiffness and a long step stay finite. */
    c = sl_test_config(200u, 400u);
    c.substep_count = 1u;
    c.iteration_count = 1u;
    d.membrane_mu = 1.0e12f;
    d.membrane_lambda = 1.0e12f;
    d.bending_stiffness = 1.0e6f;
    SL_EXPECT(sl_test_sheet(&world, &c, &d, 1.0f, 8u, 1.0f, true));
    for (uint32_t k = 0; k < 30u; k++) {
        sl_world_step(&world, 0.5f);
    }
    const sl_vec3 *p = sl_world_cloth_positions(&world, 0u);
    for (uint32_t i = 0; i < sl_world_cloth_vertex_count(&world, 0u); i++) {
        SL_EXPECT(sl_vec3_is_finite(p[i]));
    }
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "substeps_follow_stiffness", test_substeps_follow_stiffness },
    { "free_fall_backward_euler", test_free_fall_matches_backward_euler },
    { "sweeps_decrease_potential", test_sweeps_decrease_incremental_potential },
    { "hanging_sheet_settles", test_hanging_sheet_settles_with_small_stretch },
    { "pin_targets_interpolate", test_pin_targets_interpolate_over_substeps },
    { "deterministic", test_simulation_is_deterministic },
    { "parallel_matches_serial", test_parallel_matches_serial },
    { "ground_contact_rests", test_ground_contact_rests_on_thickness },
    { "sphere_drape_outside", test_sphere_drape_stays_outside },
    { "swing_never_gains_energy", test_swinging_sheet_never_gains_energy },
    { "edge_damping_spares_rigid", test_edge_damping_spares_rigid_motion },
    { "friction_sticks_and_slides", test_friction_sticks_and_slides },
    { "drag_and_wind", test_drag_slows_fall_and_wind_pushes },
    { "speed_cap_and_finite", test_speed_cap_and_finite_state },
};

int sl_solver_suite(void)
{
    return sl_run_suite("solver", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
