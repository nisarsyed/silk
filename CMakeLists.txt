cmake_minimum_required(VERSION 3.28)
project(silk VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(SL_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(SL_SANITIZERS)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    else()
        message(FATAL_ERROR "SL_SANITIZERS requires Clang or GCC")
    endif()
endif()

enable_testing()

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
        -Werror
    )
elseif(MSVC)
    target_compile_options(silk_warnings INTERFACE /W4 /WX)
endif()

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(examples/sandbox)
