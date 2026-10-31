#include "energy.h"
#include "fixture.h"
#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

static sl_vec3 sl_rand_vec3(uint32_t *seed, float lo, float hi)
{
    return sl_vec3_make(sl_test_randf(seed, lo, hi),
                        sl_test_randf(seed, lo, hi),
                        sl_test_randf(seed, lo, hi));
}

static float sl_vec3_component(sl_vec3 v, uint32_t axis)
{
    return (axis == 0u) ? v.x : ((axis == 1u) ? v.y : v.z);
}

static sl_vec3 sl_vec3_bump(sl_vec3 v, uint32_t axis, float d)
{
    if (axis == 0u) {
        v.x += d;
    } else if (axis == 1u) {
        v.y += d;
    } else {
        v.z += d;
    }
    return v;
}

/* A random non-degenerate rest triangle and rest data. */
static void sl_rand_triangle(uint32_t *seed, sl_vec3 x[3], float rest[4],
                             float *area)
{
    do {
        for (uint32_t k = 0; k < 3u; k++) {
            x[k] = sl_rand_vec3(seed, -0.5f, 0.5f);
        }
    } while (!sl_membrane_rest(x[0], x[1], x[2], rest, area) || *area < 0.02f);
}

static void test_membrane_rest_is_stress_free(void)
{
    uint32_t seed = 0xE0000001u;
    for (int trial = 0; trial < 50; trial++) {
        sl_vec3 x[3];
        float rest[4];
        float area;
        sl_rand_triangle(&seed, x, rest, &area);
        SL_EXPECT_NEAR(sl_membrane_energy(x, rest, area, 3.0f, 2.0f), 0.0f,
                       1e-6f);
        for (uint32_t k = 0; k < 3u; k++) {
            sl_vec3 g;
            sl_mat3 h;
            sl_membrane_vertex(x, rest, area, 3.0f, 2.0f, k, &g, &h);
            SL_EXPECT_NEAR(sl_vec3_length(g), 0.0f, 1e-5f);
        }
    }
}

static void test_membrane_rigid_motion_is_free(void)
{
    uint32_t seed = 0xE0000002u;
    sl_vec3 x[3];
    float rest[4];
    float area;
    sl_rand_triangle(&seed, x, rest, &area);
    /* Rotate 90 degrees about z and translate. */
    sl_vec3 y[3];
    for (uint32_t k = 0; k < 3u; k++) {
        y[k] = sl_vec3_make(-x[k].y + 3.0f, x[k].x - 1.0f, x[k].z + 2.0f);
    }
    SL_EXPECT_NEAR(sl_membrane_energy(y, rest, area, 5.0f, 5.0f), 0.0f, 1e-5f);
}

static void test_membrane_stretch_matches_closed_form(void)
{
    /* Uniaxial stretch s along x of a unit right triangle: G = diag(e, 0)
     * with e = (s^2 - 1) / 2, so E = A (mu e^2 + lambda e^2 / 2). */
    const sl_vec3 x[3] = { sl_vec3_make(0.0f, 0.0f, 0.0f),
                           sl_vec3_make(1.0f, 0.0f, 0.0f),
                           sl_vec3_make(0.0f, 1.0f, 0.0f) };
    float rest[4];
    float area;
    SL_EXPECT(sl_membrane_rest(x[0], x[1], x[2], rest, &area));
    SL_EXPECT_NEAR(area, 0.5f, 1e-7f);
    const float s = 1.1f;
    const sl_vec3 y[3] = { x[0], sl_vec3_make(s, 0.0f, 0.0f), x[2] };
    const float e = 0.5f * (s * s - 1.0f);
    const float mu = 7.0f;
    const float lambda = 3.0f;
    SL_EXPECT_NEAR(sl_membrane_energy(y, rest, area, mu, lambda),
                   0.5f * (mu * e * e + 0.5f * lambda * e * e), 1e-6f);
}

static void test_membrane_gradient_matches_finite_difference(void)
{
    uint32_t seed = 0xE0000003u;
    const float mu = 2.0f;
    const float lambda = 1.0f;
    const float d = 1e-3f;
    for (int trial = 0; trial < 40; trial++) {
        sl_vec3 x[3];
        float rest[4];
        float area;
        sl_rand_triangle(&seed, x, rest, &area);
        for (uint32_t k = 0; k < 3u; k++) {
            x[k] = sl_vec3_add(x[k], sl_rand_vec3(&seed, -0.1f, 0.1f));
        }
        for (uint32_t k = 0; k < 3u; k++) {
            sl_vec3 g;
            sl_mat3 h;
            sl_membrane_vertex(x, rest, area, mu, lambda, k, &g, &h);
            for (uint32_t axis = 0; axis < 3u; axis++) {
                sl_vec3 p[3] = { x[0], x[1], x[2] };
                sl_vec3 m[3] = { x[0], x[1], x[2] };
                p[k] = sl_vec3_bump(p[k], axis, d);
                m[k] = sl_vec3_bump(m[k], axis, -d);
                const double fd =
                    ((double)sl_membrane_energy(p, rest, area, mu, lambda) -
                     (double)sl_membrane_energy(m, rest, area, mu, lambda)) /
                    (2.0 * (double)d);
                SL_EXPECT_NEAR(sl_vec3_component(g, axis), fd,
                               2e-3 + 1e-2 * fabs(fd));
            }
        }
    }
}

static void test_membrane_hessian_matches_finite_difference(void)
{
    /* Under uniform stretch S is positive, the clamp is inactive, and the
     * block is the exact second derivative. */
    uint32_t seed = 0xE0000004u;
    const float mu = 2.0f;
    const float lambda = 1.0f;
    const float d = 1e-3f;
    for (int trial = 0; trial < 20; trial++) {
        sl_vec3 x[3];
        float rest[4];
        float area;
        sl_rand_triangle(&seed, x, rest, &area);
        for (uint32_t k = 0; k < 3u; k++) {
            x[k] = sl_vec3_scale(x[k], 1.2f);
        }
        for (uint32_t k = 0; k < 3u; k++) {
            sl_vec3 g;
            sl_mat3 h;
            sl_membrane_vertex(x, rest, area, mu, lambda, k, &g, &h);
            const sl_vec3 cols[3] = { h.c0, h.c1, h.c2 };
            for (uint32_t axis = 0; axis < 3u; axis++) {
                sl_vec3 p[3] = { x[0], x[1], x[2] };
                sl_vec3 m[3] = { x[0], x[1], x[2] };
                p[k] = sl_vec3_bump(p[k], axis, d);
                m[k] = sl_vec3_bump(m[k], axis, -d);
                sl_vec3 gp;
                sl_vec3 gm;
                sl_mat3 unused;
                sl_membrane_vertex(p, rest, area, mu, lambda, k, &gp, &unused);
                sl_membrane_vertex(m, rest, area, mu, lambda, k, &gm, &unused);
                const sl_vec3 fd = sl_vec3_scale(sl_vec3_sub(gp, gm), 0.5f / d);
                const float scale = sl_vec3_length(cols[axis]);
                SL_EXPECT_NEAR(cols[axis].x, fd.x, 1e-2f * (1.0f + scale));
                SL_EXPECT_NEAR(cols[axis].y, fd.y, 1e-2f * (1.0f + scale));
                SL_EXPECT_NEAR(cols[axis].z, fd.z, 1e-2f * (1.0f + scale));
            }
        }
    }
}

static void test_membrane_hessian_is_psd_under_compression(void)
{
    uint32_t seed = 0xE0000005u;
    for (int trial = 0; trial < 40; trial++) {
        sl_vec3 x[3];
        float rest[4];
        float area;
        sl_rand_triangle(&seed, x, rest, &area);
        for (uint32_t k = 0; k < 3u; k++) {
            x[k] = sl_vec3_add(sl_vec3_scale(x[k], 0.6f),
                               sl_rand_vec3(&seed, -0.05f, 0.05f));
        }
        for (uint32_t k = 0; k < 3u; k++) {
            sl_vec3 g;
            sl_mat3 h;
            sl_membrane_vertex(x, rest, area, 4.0f, 1.0f, k, &g, &h);
            for (int probe = 0; probe < 10; probe++) {
                const sl_vec3 v = sl_rand_vec3(&seed, -1.0f, 1.0f);
                SL_EXPECT(sl_vec3_dot(v, sl_mat3_mul_vec3(h, v)) >= -1e-6f);
            }
        }
    }
}

/* Flat unit-square stencil: edge (0,0)-(1,0), wings above and below. */
static void sl_flat_stencil(sl_vec3 x[4])
{
    x[0] = sl_vec3_make(0.0f, 0.0f, 0.0f);
    x[1] = sl_vec3_make(1.0f, 0.0f, 0.0f);
    x[2] = sl_vec3_make(0.3f, 0.8f, 0.0f);
    x[3] = sl_vec3_make(0.6f, -0.7f, 0.0f);
}

static void test_bend_weights_annihilate_affine_images(void)
{
    sl_vec3 x[4];
    sl_flat_stencil(x);
    float w[4];
    float scale;
    SL_EXPECT(sl_bend_rest(x, w, &scale));
    SL_EXPECT_NEAR(w[0] + w[1] + w[2] + w[3], 0.0f, 1e-5f);
    SL_EXPECT_NEAR(sl_bend_energy(x, w, scale, 1.0f), 0.0f, 1e-10f);
    /* Shear, stretch, rotate, translate: still no bending energy. */
    sl_vec3 y[4];
    for (uint32_t k = 0; k < 4u; k++) {
        const sl_vec3 p = x[k];
        y[k] = sl_vec3_make(1.3f * p.x + 0.4f * p.y + 2.0f, 0.2f * p.x + 5.0f,
                            0.5f * p.x - 0.9f * p.y - 1.0f);
    }
    SL_EXPECT_NEAR(sl_bend_energy(y, w, scale, 1.0f), 0.0f, 1e-8f);
    /* Folding the wing out of plane costs energy. */
    y[0] = x[0];
    y[1] = x[1];
    y[2] = sl_vec3_make(0.3f, 0.6f, 0.4f);
    y[3] = x[3];
    SL_EXPECT(sl_bend_energy(y, w, scale, 1.0f) > 1e-3f);
}

static void test_bend_gradient_matches_finite_difference(void)
{
    uint32_t seed = 0xE0000006u;
    sl_vec3 rest[4];
    sl_flat_stencil(rest);
    float w[4];
    float scale;
    SL_EXPECT(sl_bend_rest(rest, w, &scale));
    const float d = 1e-3f;
    for (int trial = 0; trial < 20; trial++) {
        sl_vec3 x[4];
        for (uint32_t k = 0; k < 4u; k++) {
            x[k] = sl_vec3_add(rest[k], sl_rand_vec3(&seed, -0.2f, 0.2f));
        }
        for (uint32_t k = 0; k < 4u; k++) {
            sl_vec3 g;
            float h;
            sl_bend_vertex(x, w, scale, 0.7f, k, &g, &h);
            SL_EXPECT_NEAR(h, 0.7f * scale * w[k] * w[k], 1e-6f);
            for (uint32_t axis = 0; axis < 3u; axis++) {
                sl_vec3 p[4] = { x[0], x[1], x[2], x[3] };
                sl_vec3 m[4] = { x[0], x[1], x[2], x[3] };
                p[k] = sl_vec3_bump(p[k], axis, d);
                m[k] = sl_vec3_bump(m[k], axis, -d);
                const double fd = ((double)sl_bend_energy(p, w, scale, 0.7f) -
                                   (double)sl_bend_energy(m, w, scale, 0.7f)) /
                                  (2.0 * (double)d);
                SL_EXPECT_NEAR(sl_vec3_component(g, axis), fd,
                               1e-3 + 1e-2 * fabs(fd));
            }
        }
    }
}

static void test_bend_cylinder_matches_plate_energy(void)
{
    /* Roll a flat 1 m x 1 m, 40 x 40 grid onto a cylinder of radius
     * 0.5 m along u. The plate energy is k / 2 * kappa^2 * area = 2 J for
     * k = 1. Interior edges miss half a cell along each border, so the
     * discrete sum sits ~2.5% low at this resolution (it converges from
     * below: 0.946 at 20 cells, 0.983 at 60). */
    const float radius = 0.5f;
    const uint32_t cells = 40u;
    sl_world world = { 0 };
    sl_world_config c = sl_test_config(2000u, 4000u);
    sl_cloth_desc desc = sl_test_cloth_desc();
    desc.bending_stiffness = 1.0f;
    SL_EXPECT(sl_test_sheet(&world, &c, &desc, 1.0f, cells, 0.0f, false));
    sl_world_state *s = world.state;
    for (uint32_t i = 0; i < s->vertex_count; i++) {
        /* Sheet spans x in [-0.5, 0.5]; arc length u = x + 0.5. */
        const sl_vec3 p = s->position[i];
        const float angle = (p.x + 0.5f) / radius;
        s->position[i] = sl_vec3_make(radius * sinf(angle), p.y,
                                      radius - radius * cosf(angle));
    }
    double energy = 0.0;
    for (uint32_t b = 0; b < s->bend_count; b++) {
        const uint32_t *idx = s->bend_index + 4u * b;
        const sl_vec3 x[4] = { s->position[idx[0]], s->position[idx[1]],
                               s->position[idx[2]], s->position[idx[3]] };
        energy += (double)sl_bend_energy(x, s->bend_weight + 4u * b,
                                         s->bend_scale[b], 1.0f);
    }
    const double plate = 0.5 / (double)(radius * radius);
    SL_EXPECT(energy / plate > 0.96);
    SL_EXPECT(energy / plate < 1.0);
    sl_world_destroy(&world);
}

static void test_collider_distances(void)
{
    sl_collider plane = { SL_COLLIDER_PLANE, sl_vec3_make(0.0f, 1.0f, 0.0f),
                          sl_vec3_make(0.0f, 2.0f, 0.0f), 0.0f, 0.3f };
    sl_collider c;
    SL_EXPECT(sl_collider_validate(&plane, &c));
    sl_vec3 n;
    SL_EXPECT_NEAR(sl_collider_distance(&c, sl_vec3_make(5.0f, 3.5f, 1.0f), &n),
                   2.5f, 1e-6f);
    SL_EXPECT_NEAR(n.y, 1.0f, 0.0f);

    const sl_collider sphere = { SL_COLLIDER_SPHERE,
                                 sl_vec3_make(1.0f, 0.0f, 0.0f), sl_vec3_zero(),
                                 0.5f, 0.0f };
    SL_EXPECT(sl_collider_validate(&sphere, &c));
    SL_EXPECT_NEAR(sl_collider_distance(&c, sl_vec3_make(1.0f, 0.2f, 0.0f), &n),
                   -0.3f, 1e-6f);
    SL_EXPECT_NEAR(n.y, 1.0f, 1e-6f);

    const sl_collider capsule = { SL_COLLIDER_CAPSULE, sl_vec3_zero(),
                                  sl_vec3_make(0.0f, 0.0f, 2.0f), 0.25f, 0.0f };
    SL_EXPECT(sl_collider_validate(&capsule, &c));
    SL_EXPECT_NEAR(sl_collider_distance(&c, sl_vec3_make(1.0f, 0.0f, 1.0f), &n),
                   0.75f, 1e-6f);
    SL_EXPECT_NEAR(n.x, 1.0f, 1e-6f);
    SL_EXPECT_NEAR(sl_collider_distance(&c, sl_vec3_make(0.0f, 0.0f, 3.0f), &n),
                   0.75f, 1e-6f);
    SL_EXPECT_NEAR(n.z, 1.0f, 1e-6f);

    sl_collider bad = sphere;
    bad.radius = -1.0f;
    SL_EXPECT(!sl_collider_validate(&bad, &c));
    bad = plane;
    bad.point_b = sl_vec3_zero();
    SL_EXPECT(!sl_collider_validate(&bad, &c));
    bad = plane;
    bad.friction = NAN;
    SL_EXPECT(!sl_collider_validate(&bad, &c));
}

static const sl_test_case k_cases[] = {
    { "membrane_rest_is_stress_free", test_membrane_rest_is_stress_free },
    { "membrane_rigid_motion_is_free", test_membrane_rigid_motion_is_free },
    { "membrane_stretch_closed_form",
      test_membrane_stretch_matches_closed_form },
    { "membrane_gradient_fd",
      test_membrane_gradient_matches_finite_difference },
    { "membrane_hessian_fd", test_membrane_hessian_matches_finite_difference },
    { "membrane_hessian_psd_compressed",
      test_membrane_hessian_is_psd_under_compression },
    { "bend_affine_images_free", test_bend_weights_annihilate_affine_images },
    { "bend_gradient_fd", test_bend_gradient_matches_finite_difference },
    { "bend_cylinder_plate_energy", test_bend_cylinder_matches_plate_energy },
    { "collider_distances", test_collider_distances },
};

int sl_energy_suite(void)
{
    return sl_run_suite("energy", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
