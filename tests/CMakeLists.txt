add_executable(sl_tests
    sl_tests.c
    replay.c
    test_replay.c
    test_stats.c
    test_core.c
    test_math.c
    test_shape.c
    test_collide.c
    test_contact.c
    test_solver.c
    test_joint.c
    test_tree.c
    test_body.c
    test_world.c
    test_step.c
)
target_link_libraries(sl_tests PRIVATE silk silk_warnings)
target_include_directories(sl_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sl_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)

# One entry: the runner has no per-suite filter, so per-suite registrations
# would each re-execute the whole binary.
add_test(NAME all COMMAND sl_tests)
