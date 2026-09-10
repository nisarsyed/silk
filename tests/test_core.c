#include "silk_test.h"
#include "suites.h"
#include <silk/silk.h>

static void test_version_matches_macros(void)
{
    SL_EXPECT_INT_EQ(SL_VERSION_MAJOR, SL_TEST_VERSION_MAJOR);
    SL_EXPECT_INT_EQ(SL_VERSION_MINOR, SL_TEST_VERSION_MINOR);
    SL_EXPECT_INT_EQ(SL_VERSION_PATCH, SL_TEST_VERSION_PATCH);
    int expected =
        (SL_VERSION_MAJOR << 16) | (SL_VERSION_MINOR << 8) | SL_VERSION_PATCH;
    SL_EXPECT_INT_EQ(sl_version(), expected);
}

static const sl_test_case k_cases[] = {
    { "version_matches_macros", test_version_matches_macros },
};

int sl_core_suite(void)
{
    return sl_run_suite("core", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
