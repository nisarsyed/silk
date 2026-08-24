add_library(silk STATIC
    silk.c
)
add_library(silk::silk ALIAS silk)

target_include_directories(silk PUBLIC ${PROJECT_SOURCE_DIR}/include)

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(silk PRIVATE
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
    target_compile_options(silk PRIVATE /W4 /WX)
endif()
