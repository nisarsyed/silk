#define SILK_TEST_IMPLEMENTATION
#include "silk_test.h"
#include "suites.h"

int main(void)
{
    int failures = 0;

    failures += sl_core_suite();
    failures += sl_math_suite();
    failures += sl_shape_suite();
    failures += sl_collide_suite();
    failures += sl_body_suite();
    failures += sl_world_suite();
    failures += sl_step_suite();

    if (failures == 0) {
        printf("all tests passed\n");
        return 0;
    }
    printf("%d failure(s)\n", failures);
    return 1;
}
