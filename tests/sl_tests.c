#define SILK_TEST_IMPLEMENTATION
#include "silk_test.h"
#include "suites.h"

int main(void)
{
    int failures = 0;

    failures += sl_core_suite();
    failures += sl_math_suite();
    failures += sl_energy_suite();
    failures += sl_mesh_suite();
    failures += sl_solver_suite();
    failures += sl_contact_suite();
    failures += sl_rigid_suite();
    failures += sl_step_suite();
    failures += sl_consumer_suite();

    if (failures == 0) {
        printf("all tests passed\n");
        return 0;
    }
    printf("%d failure(s)\n", failures);
    return 1;
}
