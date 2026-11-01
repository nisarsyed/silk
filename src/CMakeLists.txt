# silk is a core plus modules. Each is an object library that sees the
# public headers, core's private headers, and its own -- nothing else -- so
# the core cannot depend on a module and modules cannot depend on each
# other: such an include fails to compile. Consumers link the one static
# library they combine into.
#
#   core   shared by every module: geometry, the BVH, arena layout, parallel
#          dispatch, version
#   cloth  the VBD cloth solver, Offset Geometric Contact, the world and step
#   rigid  rigid bodies: box collision, contact persistence, broad phase
function(silk_module name)
    add_library(silk_${name} OBJECT ${ARGN})
    target_include_directories(silk_${name} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/${name}
    )
    target_compile_features(silk_${name} PRIVATE c_std_17)
    target_link_libraries(silk_${name} PRIVATE silk_warnings)
endfunction()

silk_module(core
    core/bvh.c
    core/geometry.c
    core/silk.c
)
silk_module(cloth
    cloth/contact.c
    cloth/energy.c
    cloth/grid.c
    cloth/mesh.c
    cloth/self_contact.c
    cloth/solver.c
    cloth/solver_self.c
    cloth/step.c
    cloth/world.c
)
silk_module(rigid
    rigid/rigid.c
    rigid/rigid_collide.c
)

add_library(silk STATIC
    $<TARGET_OBJECTS:silk_core>
    $<TARGET_OBJECTS:silk_cloth>
    $<TARGET_OBJECTS:silk_rigid>
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

# Public headers call <math.h> functions (sqrtf, fabsf); glibc keeps them in
# libm, so consumers of silk link it transitively. On macOS libm folds into
# libSystem (linking it is harmless); on Windows they live in the CRT and no
# libm exists.
if(UNIX)
    target_link_libraries(silk PUBLIC m)
endif()
