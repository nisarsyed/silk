add_executable(sl_tests
    sl_tests.c
    test_core.c
    test_math.c
    test_energy.c
    test_mesh.c
    test_solver.c
    test_contact.c
    test_rigid.c
    test_step.c
)
target_link_libraries(sl_tests PRIVATE silk silk_warnings)
target_compile_definitions(sl_tests PRIVATE
    SL_TEST_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    SL_TEST_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    SL_TEST_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)
target_include_directories(sl_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sl_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/src/core
    ${PROJECT_SOURCE_DIR}/src/cloth
    ${PROJECT_SOURCE_DIR}/src/rigid
)

# One entry: the runner has no per-suite filter, so per-suite registrations
# would each re-execute the whole binary.
add_test(NAME all COMMAND sl_tests)

# Compile the public consumer independently: no private src include directory.
add_library(sl_consumer OBJECT test_consumer.c)
target_link_libraries(sl_consumer PRIVATE silk silk_warnings)
target_include_directories(sl_consumer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(sl_tests PRIVATE $<TARGET_OBJECTS:sl_consumer>)
