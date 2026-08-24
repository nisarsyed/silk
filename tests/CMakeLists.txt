add_executable(sl_tests
    sl_tests.c
    test_core.c
    test_math.c
)
target_link_libraries(sl_tests PRIVATE silk)
target_include_directories(sl_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME core COMMAND sl_tests)
add_test(NAME math COMMAND sl_tests)
