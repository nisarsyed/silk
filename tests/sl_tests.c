#define SILK_TEST_IMPLEMENTATION
#include "silk_test.h"
#include "suites.h"

int main(void)
{
    int failures = 0;

#ifdef SL_TEST_WASM_ADAPTER
    failures += sl_wasm_suite();
    failures += sl_wasm_bench_suite();
#endif

    failures += sl_replay_suite();
    failures += sl_sleep_suite();
    failures += sl_island_suite();
    failures += sl_diagnostics_suite();
    failures += sl_stats_suite();
    failures += sl_consumer_suite();
    failures += sl_query_suite();
    failures += sl_core_suite();
    failures += sl_math_suite();
    failures += sl_shape_suite();
    failures += sl_collide_suite();
    failures += sl_contact_suite();
    failures += sl_solver_suite();
    failures += sl_joint_suite();
    failures += sl_tree_suite();
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
