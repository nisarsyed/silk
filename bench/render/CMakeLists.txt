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
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SL_RENDER_HEADER "")
foreach(SL_HEADER raylib.h driver.h ../../wasm/adapter.h ../../wasm/runtime.h)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SL_HEADER}")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${SL_HEADER}" SL_HEADER_CONTENT)
    string(APPEND SL_RENDER_HEADER "${SL_HEADER_CONTENT}")
endforeach()
string(REGEX MATCHALL "sl_(render|wasm)_[a-z_0-9]+\\(" SL_RENDER_NAMES "${SL_RENDER_HEADER}")
set(SL_RENDER_EXPORTS "_sl_version")
foreach(SL_RENDER_NAME IN LISTS SL_RENDER_NAMES)
    string(REPLACE "(" "" SL_RENDER_NAME "${SL_RENDER_NAME}")
    list(APPEND SL_RENDER_EXPORTS "_${SL_RENDER_NAME}")
endforeach()
list(JOIN SL_RENDER_EXPORTS "," SL_RENDER_EXPORT_FLAGS)
add_executable(silk_render_raylib raylib.c driver.c scene.c overlay.c ../fixtures.c ../quality.c
    ../../wasm/adapter.c ../../wasm/snapshot.c ../../wasm/operations.c ../../wasm/runtime.c)
target_include_directories(silk_render_raylib SYSTEM PRIVATE "${raylib_SOURCE_DIR}/src")
target_link_libraries(silk_render_raylib PRIVATE raylib silk silk_warnings)
set_target_properties(silk_render_raylib PROPERTIES OUTPUT_NAME raylib SUFFIX .mjs
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/render-study")
target_link_options(silk_render_raylib PRIVATE --no-entry
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web
    -sIMPORTED_MEMORY=1 -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0 -sINITIAL_MEMORY=2097152
    -sSTACK_SIZE=1048576 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
    -sINCOMING_MODULE_JS_API=canvas,locateFile,wasmMemory,print,printErr,onAbort
    -sEXPORTED_RUNTIME_METHODS=ccall,HEAPF32,HEAPU32
    "-sEXPORTED_FUNCTIONS=${SL_RENDER_EXPORT_FLAGS}"
    "$<$<CONFIG:Debug>:-sASSERTIONS=2>" "$<$<CONFIG:Release>:-sASSERTIONS=0>")
set_property(TARGET silk_render_raylib APPEND PROPERTY LINK_DEPENDS
    "${PROJECT_SOURCE_DIR}/wasm/provision_memory.py")
add_custom_command(TARGET silk_render_raylib POST_BUILD
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/wasm/provision_memory.py"
        "$<TARGET_FILE_DIR:silk_render_raylib>/raylib.wasm"
    VERBATIM)

configure_file("${raylib_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/render-study/licenses/raylib.txt" COPYONLY)
add_custom_target(silk_render_raylib_metadata ALL
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/bench/render/metadata.py"
        --root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
        --compiler "${CMAKE_C_COMPILER}" --mode "$<CONFIG>" --kind raylib
        --raylib-source "${raylib_SOURCE_DIR}"
    DEPENDS silk_render_raylib
    VERBATIM)
