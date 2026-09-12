# Opt-in wrapper around the SDK toolchain, also used by CMake try_compile.
# Native presets never load this file or look for Emscripten/Node.
set(SL_EMSCRIPTEN_VERSION "6.0.9")
set(SL_EMSCRIPTEN_ROOT "$ENV{EMSDK}/upstream/emscripten")
if(NOT EXISTS "${SL_EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake")
    message(FATAL_ERROR
        "Silk WASM requires Emscripten ${SL_EMSCRIPTEN_VERSION}. Install/activate that SDK and source emsdk_env.sh (EMSDK is unset or invalid). See docs/wasm.md.")
endif()
file(READ "${SL_EMSCRIPTEN_ROOT}/emscripten-version.txt" SL_EMSCRIPTEN_ACTUAL)
string(STRIP "${SL_EMSCRIPTEN_ACTUAL}" SL_EMSCRIPTEN_ACTUAL)
string(REPLACE "\"" "" SL_EMSCRIPTEN_ACTUAL "${SL_EMSCRIPTEN_ACTUAL}")
if(NOT SL_EMSCRIPTEN_ACTUAL STREQUAL SL_EMSCRIPTEN_VERSION)
    message(FATAL_ERROR
        "Silk WASM requires Emscripten ${SL_EMSCRIPTEN_VERSION}; found ${SL_EMSCRIPTEN_ACTUAL}. Activate the pinned SDK and configure a clean build directory. See docs/wasm.md.")
endif()

if(EXISTS "$ENV{EMSDK_NODE}")
    set(SL_NODE_EXECUTABLE "$ENV{EMSDK_NODE}")
else()
    find_program(SL_NODE_EXECUTABLE NAMES node nodejs NO_CMAKE_FIND_ROOT_PATH)
endif()
if(NOT SL_NODE_EXECUTABLE)
    message(FATAL_ERROR "Silk WASM tests require Node on PATH. Source emsdk_env.sh or install Node. See docs/wasm.md.")
endif()
execute_process(COMMAND "${SL_NODE_EXECUTABLE}" --version
    RESULT_VARIABLE SL_NODE_RESULT OUTPUT_VARIABLE SL_NODE_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT SL_NODE_RESULT EQUAL 0)
    message(FATAL_ERROR "Cannot execute Node: ${SL_NODE_EXECUTABLE}")
endif()
set(CMAKE_CROSSCOMPILING_EMULATOR "${SL_NODE_EXECUTABLE}" CACHE FILEPATH
    "Node runtime for WASM C tests")
include("${SL_EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake")
