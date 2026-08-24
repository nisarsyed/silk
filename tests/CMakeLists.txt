add_executable(sl_tests
    sl_tests.c
    test_core.c
    test_math.c
)
target_link_libraries(sl_tests PRIVATE silk silk_warnings)
target_include_directories(sl_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# One entry: the runner has no per-suite filter, so per-suite registrations
# would each re-execute the whole binary.
add_test(NAME all COMMAND sl_tests)
