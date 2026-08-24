#include "silk_test.h"
#include <silk/silk.h>

static void test_version_matches_macros(void)
{
    int expected = (SL_VERSION_MAJOR << 16) | (SL_VERSION_MINOR << 8) | SL_VERSION_PATCH;
    SL_EXPECT_INT_EQ(sl_version(), expected);
}

static const sl_test_case k_cases[] = {
    { "version_matches_macros", test_version_matches_macros },
};

int sl_core_suite(void)
{
    return sl_run_suite("core", k_cases, (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
