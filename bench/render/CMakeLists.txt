if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "Renderer comparison requires the pinned Emscripten toolchain")
endif()
include(FetchContent)
# Match examples/sandbox. The 6.0 config parser's CUSTOMIZE_BUILD behavior is
# unsuitable here too; retain its defaults and explicitly select WebGL 2.
set(PLATFORM Web CACHE STRING "" FORCE)
set(OPENGL_VERSION "ES 3.0" CACHE STRING "" FORCE)
set(GRAPHICS GRAPHICS_API_OPENGL_ES3 CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CUSTOMIZE_BUILD OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 6.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(raylib)
# Emscripten EM_ASM in third-party raylib requires GNU mode. The comparison
# adapter and silk retain C17 with extensions disabled.
set_target_properties(raylib PROPERTIES C_EXTENSIONS ON)
# The pin exports ccall through a transitive link option. Supply one combined
# list so it cannot override this private module's typed-array exports.
get_target_property(SL_RAYLIB_LINK_OPTIONS raylib INTERFACE_LINK_OPTIONS)
list(REMOVE_ITEM SL_RAYLIB_LINK_OPTIONS "-sEXPORTED_RUNTIME_METHODS=ccall")
set_property(TARGET raylib PROPERTY INTERFACE_LINK_OPTIONS "${SL_RAYLIB_LINK_OPTIONS}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS raylib.h)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/raylib.h" SL_RENDER_HEADER)
string(REGEX MATCHALL "sl_render_[a-z_0-9]+\\(" SL_RENDER_NAMES "${SL_RENDER_HEADER}")
set(SL_RENDER_EXPORTS "")
foreach(SL_RENDER_NAME IN LISTS SL_RENDER_NAMES)
    string(REPLACE "(" "" SL_RENDER_NAME "${SL_RENDER_NAME}")
    list(APPEND SL_RENDER_EXPORTS "_${SL_RENDER_NAME}")
endforeach()
list(JOIN SL_RENDER_EXPORTS "," SL_RENDER_EXPORT_FLAGS)
add_executable(silk_render_raylib raylib.c)
target_include_directories(silk_render_raylib SYSTEM PRIVATE "${raylib_SOURCE_DIR}/src")
target_link_libraries(silk_render_raylib PRIVATE raylib silk_warnings)
set_target_properties(silk_render_raylib PROPERTIES OUTPUT_NAME raylib SUFFIX .mjs
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/render-study")
target_link_options(silk_render_raylib PRIVATE --no-entry
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web
    -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0 -sINITIAL_MEMORY=67108864
    -sSTACK_SIZE=1048576 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
    -sINCOMING_MODULE_JS_API=canvas,locateFile,print,printErr,onAbort
    -sEXPORTED_RUNTIME_METHODS=ccall,HEAPF32,HEAPU32
    "-sEXPORTED_FUNCTIONS=${SL_RENDER_EXPORT_FLAGS}"
    "$<$<CONFIG:Debug>:-sASSERTIONS=2>" "$<$<CONFIG:Release>:-sASSERTIONS=0>")

configure_file("${raylib_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/render-study/licenses/raylib.txt" COPYONLY)
