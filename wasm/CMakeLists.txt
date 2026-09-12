add_library(silk_wasm_adapter STATIC adapter.c snapshot.c operations.c)
target_link_libraries(silk_wasm_adapter PUBLIC silk PRIVATE silk_warnings)
target_include_directories(silk_wasm_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(EMSCRIPTEN)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS adapter.h runtime.h)
    # The binding ABI exports explicit functions; no C aggregate ABI or embind.
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/adapter.h" SL_WASM_HEADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/runtime.h" SL_WASM_RUNTIME_HEADER)
    string(APPEND SL_WASM_HEADER "${SL_WASM_RUNTIME_HEADER}")
    string(REGEX MATCHALL "sl_wasm_[a-z_0-9]+\\(" SL_WASM_NAMES "${SL_WASM_HEADER}")
    set(SL_WASM_EXPORTS "_sl_version")
    foreach(SL_WASM_NAME IN LISTS SL_WASM_NAMES)
        string(REPLACE "(" "" SL_WASM_NAME "${SL_WASM_NAME}")
        list(APPEND SL_WASM_EXPORTS "_${SL_WASM_NAME}")
    endforeach()
    list(JOIN SL_WASM_EXPORTS "," SL_WASM_EXPORT_FLAGS)
    add_executable(silk_wasm adapter.c snapshot.c operations.c runtime.c)
    target_link_libraries(silk_wasm PRIVATE silk silk_warnings)
    set_target_properties(silk_wasm PROPERTIES
        OUTPUT_NAME silk SUFFIX .mjs
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/package")
    target_link_options(silk_wasm PRIVATE
        --no-entry -sMODULARIZE=1 -sEXPORT_ES6=1
        -sENVIRONMENT=web,worker,node
        -sIMPORTED_MEMORY=1 -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0
        -sINITIAL_MEMORY=2097152 -sSTACK_SIZE=1048576
        -sFILESYSTEM=0
        "-sEXPORTED_FUNCTIONS=${SL_WASM_EXPORT_FLAGS}"
        -sEXPORTED_RUNTIME_METHODS=HEAPF32,HEAPU32
        -sINCOMING_MODULE_JS_API=locateFile,wasmBinary,wasmMemory,print,printErr,onAbort
        "$<$<CONFIG:Debug>:-sASSERTIONS=2>"
        "$<$<CONFIG:Debug>:-sSTACK_OVERFLOW_CHECK=2>"
        "$<$<CONFIG:Release>:-sASSERTIONS=0>"
    )
    set_property(TARGET silk_wasm APPEND PROPERTY LINK_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/provision_memory.py")
    add_custom_command(TARGET silk_wasm POST_BUILD
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/provision_memory.py"
            "${CMAKE_BINARY_DIR}/package/silk.wasm"
        VERBATIM)
    configure_file(package.json.in "${CMAKE_BINARY_DIR}/package/package.json" @ONLY)
    configure_file(index.d.ts "${CMAKE_BINARY_DIR}/package/index.d.ts" COPYONLY)
    configure_file(README.md "${CMAKE_BINARY_DIR}/package/README.md" COPYONLY)
    configure_file("${PROJECT_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/package/LICENSE" COPYONLY)
    foreach(SL_NOTICE README.md emscripten.txt musl.txt compiler-rt.txt)
        configure_file("licenses/${SL_NOTICE}" "${CMAKE_BINARY_DIR}/package/licenses/${SL_NOTICE}" COPYONLY)
    endforeach()
    configure_file(views.mjs "${CMAKE_BINARY_DIR}/package/views.mjs" COPYONLY)
    configure_file(index.mjs "${CMAKE_BINARY_DIR}/package/index.mjs" COPYONLY)
endif()
