add_library(silk STATIC
    silk.c
    world.c
    query.c
    sleep.c
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

include(GNUInstallDirs)
target_include_directories(silk PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(silk PUBLIC c_std_17)
# Static PRIVATE dependencies otherwise appear as LINK_ONLY export entries.
# Keep first-party warnings local to this build and out of both export forms.
target_link_libraries(silk PRIVATE $<BUILD_LOCAL_INTERFACE:silk_warnings>)

# Public headers call <math.h> functions (sqrtf, sinf, cosf); glibc keeps
# them in libm, so consumers of silk link it transitively. On macOS libm
# folds into libSystem (linking it is harmless); on Windows they live in
# the CRT and no libm exists.
if(UNIX)
    target_link_libraries(silk PUBLIC m)
endif()
