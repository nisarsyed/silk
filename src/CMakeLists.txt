add_library(silk STATIC
    silk.c
    world.c
    query.c
    island.c
    step.c
    shape.c
    solver.c
    joint.c
    collide.c
    contact.c
    tree.c
)
add_library(silk::silk ALIAS silk)

target_include_directories(silk PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(silk PRIVATE silk_warnings)

# Public headers call <math.h> functions (sqrtf, sinf, cosf); glibc keeps
# them in libm, so consumers of silk link it transitively. On macOS libm
# folds into libSystem (linking it is harmless); on Windows they live in
# the CRT and no libm exists.
if(UNIX)
    target_link_libraries(silk PUBLIC m)
endif()
