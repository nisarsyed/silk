cmake_minimum_required(VERSION 3.28)
project(silk VERSION 0.3.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(SL_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SL_BUILD_TESTS "Build the silk test suite" ${PROJECT_IS_TOP_LEVEL})
if(SL_SANITIZERS)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    else()
        message(FATAL_ERROR "SL_SANITIZERS requires Clang or GCC")
    endif()
endif()

if(SL_BUILD_TESTS)
    enable_testing()
endif()

# Strict warning set shared by every target that compiles engine code,
# so public headers are exercised under -Werror everywhere.
add_library(silk_warnings INTERFACE)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(silk_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wdouble-promotion
        -Wformat=2
        -Wundef
        -Wmissing-prototypes
        -Wstrict-prototypes
        -Wvla
        -Werror
    )
elseif(MSVC)
    target_compile_options(silk_warnings INTERFACE /W4 /WX)
endif()

# Executables land in one predictable place per preset (<build>/bin),
# independent of their depth in the source tree.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(src)
if(SL_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Deterministic rigid-body performance harness: an off-by-default developer
# executable, never a CTest entry. Wall-clock timing remains outside silk.
option(SL_BUILD_BENCH "Build the deterministic benchmark executable" OFF)
if(SL_BUILD_BENCH)
    add_subdirectory(bench)
endif()

# Interactive raylib sandbox: an off-by-default dev tool, not a tested
# artifact — enabling it fetches raylib at configure time and needs a
# display to run, but it is compile-checked in CI (the sandbox job).
# See examples/sandbox/CMakeLists.txt.
option(SL_SANDBOX "Build the raylib sandbox example" OFF)
if(SL_SANDBOX)
    add_subdirectory(examples/sandbox)
endif()
