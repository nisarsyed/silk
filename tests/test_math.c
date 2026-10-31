#include <float.h>

#include "fixture.h"
#include "silk/math.h"
#include "silk_test.h"
#include "suites.h"

static void test_epsilon_is_near_unit_ulps(void)
{
    /* SL_EPSILON is 8 to 9 ULP of 1.0f (FLT_EPSILON = 2^-23): sl_feq
     * accepts 8 ULP and rejects 9. Checked through sl_feq rather than by
     * comparing constants, which MSVC /W4 rejects as constant conditions
     * (C4127). */
    SL_EXPECT(sl_feq(1.0f, 1.0f + 8.0f * FLT_EPSILON, SL_EPSILON));
    SL_EXPECT(!sl_feq(1.0f, 1.0f + 9.0f * FLT_EPSILON, SL_EPSILON));
    SL_EXPECT(!sl_feq(1.0f, NAN, SL_EPSILON));
}

static void test_vec3_algebra(void)
{
    const sl_vec3 a = sl_vec3_make(1.0f, 2.0f, 3.0f);
    const sl_vec3 b = sl_vec3_make(-4.0f, 0.5f, 2.0f);
    SL_EXPECT_NEAR(sl_vec3_dot(a, b), -4.0f + 1.0f + 6.0f, 0.0f);
    const sl_vec3 c = sl_vec3_cross(a, b);
    SL_EXPECT_NEAR(sl_vec3_dot(c, a), 0.0f, 1e-5f);
    SL_EXPECT_NEAR(sl_vec3_dot(c, b), 0.0f, 1e-5f);
    const sl_vec3 x = sl_vec3_make(1.0f, 0.0f, 0.0f);
    const sl_vec3 y = sl_vec3_make(0.0f, 1.0f, 0.0f);
    const sl_vec3 z = sl_vec3_cross(x, y);
    SL_EXPECT_NEAR(z.z, 1.0f, 0.0f); /* right-handed */
    SL_EXPECT_NEAR(sl_vec3_length(sl_vec3_make(3.0f, 4.0f, 12.0f)), 13.0f,
                   1e-5f);
    const sl_vec3 m = sl_vec3_lerp(a, b, 0.5f);
    SL_EXPECT_NEAR(m.x, -1.5f, 1e-6f);
}

static void test_vec3_normalize_small_is_zero(void)
{
    const sl_vec3 n = sl_vec3_normalize(sl_vec3_make(0.0f, 3.0f, 4.0f));
    SL_EXPECT_NEAR(sl_vec3_length(n), 1.0f, SL_EPSILON);
    const sl_vec3 z = sl_vec3_normalize(sl_vec3_make(1e-7f, 0.0f, 0.0f));
    SL_EXPECT(z.x == 0.0f && z.y == 0.0f && z.z == 0.0f);
    SL_EXPECT(!sl_vec3_is_finite(sl_vec3_make(0.0f, INFINITY, 0.0f)));
}

static void test_mat3_solve_random_spd(void)
{
    uint32_t seed = 0x5EED0001u;
    for (int trial = 0; trial < 200; trial++) {
        sl_mat3 m = sl_mat3_diagonal(sl_test_randf(&seed, 0.1f, 2.0f));
        for (int k = 0; k < 3; k++) {
            const sl_vec3 a = sl_vec3_make(sl_test_randf(&seed, -1.0f, 1.0f),
                                           sl_test_randf(&seed, -1.0f, 1.0f),
                                           sl_test_randf(&seed, -1.0f, 1.0f));
            m = sl_mat3_add(m, sl_mat3_outer(a, a));
        }
        const sl_vec3 x = sl_vec3_make(sl_test_randf(&seed, -5.0f, 5.0f),
                                       sl_test_randf(&seed, -5.0f, 5.0f),
                                       sl_test_randf(&seed, -5.0f, 5.0f));
        const sl_vec3 b = sl_mat3_mul_vec3(m, x);
        sl_vec3 r = sl_vec3_zero();
        SL_EXPECT(sl_mat3_solve(m, b, 0.0f, &r));
        SL_EXPECT_NEAR(r.x, x.x, 1e-3f);
        SL_EXPECT_NEAR(r.y, x.y, 1e-3f);
        SL_EXPECT_NEAR(r.z, x.z, 1e-3f);
    }
}

static void test_mat3_solve_singular_untouched(void)
{
    const sl_mat3 m = sl_mat3_outer(sl_vec3_make(1.0f, 2.0f, 3.0f),
                                    sl_vec3_make(1.0f, 2.0f, 3.0f));
    sl_vec3 r = sl_vec3_make(7.0f, 7.0f, 7.0f);
    SL_EXPECT(!sl_mat3_solve(m, sl_vec3_make(1.0f, 0.0f, 0.0f), 1e-6f, &r));
    SL_EXPECT(r.x == 7.0f && r.y == 7.0f && r.z == 7.0f);
}

static const sl_test_case k_cases[] = {
    { "epsilon_is_near_unit_ulps", test_epsilon_is_near_unit_ulps },
    { "vec3_algebra", test_vec3_algebra },
    { "vec3_normalize_small_is_zero", test_vec3_normalize_small_is_zero },
    { "mat3_solve_random_spd", test_mat3_solve_random_spd },
    { "mat3_solve_singular_untouched", test_mat3_solve_singular_untouched },
};

int sl_math_suite(void)
{
    return sl_run_suite("math", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
