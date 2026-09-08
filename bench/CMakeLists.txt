# An explicit revision permits source-archive builds. Otherwise capture the
# configured Git source, including tracked dirty state; reconfigure after edits.
set(SL_BENCH_REVISION "" CACHE STRING "Source revision for benchmark reports")
if(SL_BENCH_REVISION STREQUAL "")
    find_package(Git QUIET)
    set(SL_BENCH_REVISION "unknown")
    if(Git_FOUND)
        execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
            OUTPUT_VARIABLE SL_BENCH_GIT_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET RESULT_VARIABLE SL_BENCH_GIT_RESULT)
        if(SL_BENCH_GIT_RESULT EQUAL 0)
            set(SL_BENCH_REVISION "${SL_BENCH_GIT_REVISION}")
            execute_process(COMMAND "${GIT_EXECUTABLE}" diff --quiet HEAD
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" RESULT_VARIABLE SL_BENCH_DIRTY)
            if(NOT SL_BENCH_DIRTY EQUAL 0)
                string(APPEND SL_BENCH_REVISION "-dirty")
            endif()
        endif()
    endif()
endif()
add_executable(sl_bench main.c quality.c)
target_link_libraries(sl_bench PRIVATE silk silk_warnings)

# Preserve exactly the build and host metadata CMake used. file(GENERATE)
# makes this work for both single-config and multi-config generators.
get_target_property(
    SL_BENCH_WARNING_OPTIONS silk_warnings INTERFACE_COMPILE_OPTIONS)
if(NOT SL_BENCH_WARNING_OPTIONS)
    set(SL_BENCH_WARNING_OPTIONS "")
endif()
string(JOIN " " SL_BENCH_WARNING_FLAGS ${SL_BENCH_WARNING_OPTIONS})

foreach(SL_BENCH_METADATA_VAR
        SL_BENCH_REVISION
        CMAKE_C_COMPILER_ID
        CMAKE_C_COMPILER_VERSION
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS_MINSIZEREL
        CMAKE_HOST_SYSTEM_NAME
        CMAKE_HOST_SYSTEM_VERSION
        CMAKE_HOST_SYSTEM_PROCESSOR
        SL_BENCH_WARNING_FLAGS)
    string(REPLACE "\\" "\\\\" ${SL_BENCH_METADATA_VAR}_ESCAPED
        "${${SL_BENCH_METADATA_VAR}}")
    string(REPLACE "\"" "\\\"" ${SL_BENCH_METADATA_VAR}_ESCAPED
        "${${SL_BENCH_METADATA_VAR}_ESCAPED}")
endforeach()

set(SL_BENCH_CONFIG_FLAGS
    "$<$<CONFIG:Debug>:${CMAKE_C_FLAGS_DEBUG_ESCAPED}>"
    "$<$<CONFIG:Release>:${CMAKE_C_FLAGS_RELEASE_ESCAPED}>"
    "$<$<CONFIG:RelWithDebInfo>:${CMAKE_C_FLAGS_RELWITHDEBINFO_ESCAPED}>"
    "$<$<CONFIG:MinSizeRel>:${CMAKE_C_FLAGS_MINSIZEREL_ESCAPED}>")
string(JOIN "" SL_BENCH_CONFIG_FLAGS ${SL_BENCH_CONFIG_FLAGS})

set(SL_BENCH_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/config/$<CONFIG>")
file(GENERATE
    OUTPUT "${SL_BENCH_CONFIG_DIR}/sl_bench_config.h"
    CONTENT
"#ifndef SL_BENCH_CONFIG_H
#define SL_BENCH_CONFIG_H
#define SL_BENCH_REVISION \"${SL_BENCH_REVISION_ESCAPED}\"
#define SL_BENCH_COMPILER_ID \"${CMAKE_C_COMPILER_ID_ESCAPED}\"
#define SL_BENCH_COMPILER_VERSION \"${CMAKE_C_COMPILER_VERSION_ESCAPED}\"
#define SL_BENCH_BUILD_MODE \"$<CONFIG>\"
#define SL_BENCH_C_FLAGS \"${CMAKE_C_FLAGS_ESCAPED} ${SL_BENCH_CONFIG_FLAGS}\"
#define SL_BENCH_WARNING_FLAGS \"${SL_BENCH_WARNING_FLAGS_ESCAPED}\"
#define SL_BENCH_HOST_SYSTEM \"${CMAKE_HOST_SYSTEM_NAME_ESCAPED}\"
#define SL_BENCH_HOST_VERSION \"${CMAKE_HOST_SYSTEM_VERSION_ESCAPED}\"
#define SL_BENCH_HOST_PROCESSOR \"${CMAKE_HOST_SYSTEM_PROCESSOR_ESCAPED}\"
#endif
")
target_include_directories(sl_bench PRIVATE "${SL_BENCH_CONFIG_DIR}")

add_executable(sl_bench_check check.c quality.c)
target_link_libraries(sl_bench_check PRIVATE silk silk_warnings)
