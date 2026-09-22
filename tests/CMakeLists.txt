add_executable(sl_tests
    sl_tests.c
    replay.c
    test_replay.c
    test_query.c
    test_sleep.c
    test_island.c
    test_diagnostics.c
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
if(SL_BUILD_WASM_ADAPTER)
    target_link_libraries(sl_tests PRIVATE silk_render_scene silk_warnings)
else()
    target_link_libraries(sl_tests PRIVATE silk silk_warnings)
endif()
target_compile_definitions(sl_tests PRIVATE
    SL_TEST_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    SL_TEST_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    SL_TEST_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)
target_include_directories(sl_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sl_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)

# One entry: the runner has no per-suite filter, so per-suite registrations
# would each re-execute the whole binary.
add_test(NAME all COMMAND sl_tests)

# Compile the public consumer independently: no private src include directory.
add_library(sl_consumer OBJECT test_consumer.c)
target_link_libraries(sl_consumer PRIVATE silk silk_warnings)
target_sources(sl_tests PRIVATE $<TARGET_OBJECTS:sl_consumer>)

if(EMSCRIPTEN)
    sl_wasm_executable(sl_tests)
endif()

if(SL_BUILD_WASM_ADAPTER)
    target_sources(sl_tests PRIVATE test_wasm.c test_wasm_bench.c test_render_scene.c test_render_frozen.c)
    target_compile_definitions(sl_tests PRIVATE SL_TEST_WASM_ADAPTER=1)
endif()
