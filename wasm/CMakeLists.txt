add_library(silk_wasm_adapter STATIC adapter.c snapshot.c operations.c)
target_link_libraries(silk_wasm_adapter PUBLIC silk PRIVATE silk_warnings)
target_include_directories(silk_wasm_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Native boundary tests exercise the same fixture/driver code under sanitizers.
add_library(silk_wasm_bench_adapter STATIC ../bench/wasm/driver.c ../bench/fixtures.c ../bench/quality.c)
target_link_libraries(silk_wasm_bench_adapter PUBLIC silk_wasm_adapter PRIVATE silk_warnings)

# Browser study setup reuses the existing fixture object code; it does not
# alter the native/WASM step-only benchmark target or distributed package.
add_library(silk_render_scene STATIC ../bench/render/scene.c ../bench/render/driver.c ../bench/render/overlay.c ../bench/render/frozen.c)
target_link_libraries(silk_render_scene PUBLIC silk_wasm_bench_adapter PRIVATE silk_warnings)

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
    add_executable(silk_wasm_bench adapter.c snapshot.c operations.c runtime.c
        ../bench/wasm/driver.c ../bench/fixtures.c ../bench/quality.c)
    target_include_directories(silk_wasm_bench PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ../bench/wasm/driver.h)
    file(READ "${PROJECT_SOURCE_DIR}/bench/wasm/driver.h" SL_BENCH_HEADER)
    string(REGEX MATCHALL "sl_wasm_[a-z_0-9]+\\(" SL_BENCH_NAMES "${SL_BENCH_HEADER}")
    set(SL_BENCH_EXPORTS "${SL_WASM_EXPORTS}")
    foreach(SL_BENCH_NAME IN LISTS SL_BENCH_NAMES)
        string(REPLACE "(" "" SL_BENCH_NAME "${SL_BENCH_NAME}")
        list(APPEND SL_BENCH_EXPORTS "_${SL_BENCH_NAME}")
    endforeach()
    list(JOIN SL_BENCH_EXPORTS "," SL_BENCH_EXPORT_FLAGS)
    set_target_properties(silk_wasm_bench PROPERTIES
        OUTPUT_NAME bench SUFFIX .mjs
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/benchmark")
    target_link_options(silk_wasm PRIVATE "-sEXPORTED_FUNCTIONS=${SL_WASM_EXPORT_FLAGS}")
    target_link_options(silk_wasm_bench PRIVATE "-sEXPORTED_FUNCTIONS=${SL_BENCH_EXPORT_FLAGS}")
    set_target_properties(silk_wasm PROPERTIES
        OUTPUT_NAME silk SUFFIX .mjs
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/package")
    add_executable(silk_wasm_study adapter.c snapshot.c operations.c runtime.c
        ../bench/render/driver.c ../bench/render/scene.c ../bench/render/frozen.c ../bench/fixtures.c ../bench/quality.c)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ../bench/render/driver.h)
    file(READ "${PROJECT_SOURCE_DIR}/bench/render/driver.h" SL_STUDY_HEADER)
    string(REGEX MATCHALL "sl_render_[a-z_0-9]+\\(" SL_STUDY_NAMES "${SL_STUDY_HEADER}")
    set(SL_STUDY_EXPORTS "${SL_WASM_EXPORTS}")
    foreach(SL_STUDY_NAME IN LISTS SL_STUDY_NAMES)
        string(REPLACE "(" "" SL_STUDY_NAME "${SL_STUDY_NAME}")
        list(APPEND SL_STUDY_EXPORTS "_${SL_STUDY_NAME}")
    endforeach()
    list(JOIN SL_STUDY_EXPORTS "," SL_STUDY_EXPORT_FLAGS)
    target_link_options(silk_wasm_study PRIVATE "-sEXPORTED_FUNCTIONS=${SL_STUDY_EXPORT_FLAGS}")
    set_target_properties(silk_wasm_study PROPERTIES OUTPUT_NAME study SUFFIX .mjs
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/render-physics")
    configure_file(views.mjs "${CMAKE_BINARY_DIR}/render-physics/views.mjs" COPYONLY)
    configure_file(../bench/render/study.mjs "${CMAKE_BINARY_DIR}/render-physics/driver.mjs" COPYONLY)
    configure_file(../bench/render/study_owner.mjs "${CMAKE_BINARY_DIR}/render-physics/owner.mjs" COPYONLY)
    add_custom_target(silk_wasm_study_metadata ALL
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/bench/render/metadata.py"
            --root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
            --compiler "${CMAKE_C_COMPILER}" --mode "$<CONFIG>" --kind physics
        DEPENDS silk_wasm_study
        VERBATIM)
    foreach(SL_WASM_TARGET silk_wasm silk_wasm_bench silk_wasm_study)
        target_link_libraries(${SL_WASM_TARGET} PRIVATE silk silk_warnings)
        target_link_options(${SL_WASM_TARGET} PRIVATE
            --no-entry -sMODULARIZE=1 -sEXPORT_ES6=1
            -sENVIRONMENT=web,worker,node
            -sIMPORTED_MEMORY=1 -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0
            -sINITIAL_MEMORY=2097152 -sSTACK_SIZE=1048576
            -sFILESYSTEM=0
            -sEXPORTED_RUNTIME_METHODS=HEAPF32,HEAPU32
            -sINCOMING_MODULE_JS_API=locateFile,wasmBinary,wasmMemory,print,printErr,onAbort
            "$<$<CONFIG:Debug>:-sASSERTIONS=2>"
            "$<$<CONFIG:Debug>:-sSTACK_OVERFLOW_CHECK=2>"
            "$<$<CONFIG:Release>:-sASSERTIONS=0>"
        )
        set_property(TARGET ${SL_WASM_TARGET} APPEND PROPERTY LINK_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/provision_memory.py")
        add_custom_command(TARGET ${SL_WASM_TARGET} POST_BUILD
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/provision_memory.py"
                "$<TARGET_FILE_DIR:${SL_WASM_TARGET}>/$<TARGET_FILE_BASE_NAME:${SL_WASM_TARGET}>.wasm"
            VERBATIM)
    endforeach()
    configure_file(package.json.in "${CMAKE_BINARY_DIR}/package/package.json" @ONLY)
    configure_file(index.d.ts "${CMAKE_BINARY_DIR}/package/index.d.ts" COPYONLY)
    configure_file(README.md "${CMAKE_BINARY_DIR}/package/README.md" COPYONLY)
    configure_file("${PROJECT_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/package/LICENSE" COPYONLY)
    foreach(SL_NOTICE README.md emscripten.txt musl.txt compiler-rt.txt fmaf.txt)
        configure_file("licenses/${SL_NOTICE}" "${CMAKE_BINARY_DIR}/package/licenses/${SL_NOTICE}" COPYONLY)
    endforeach()
    configure_file(views.mjs "${CMAKE_BINARY_DIR}/benchmark/views.mjs" COPYONLY)
    configure_file(../bench/wasm/driver.mjs "${CMAKE_BINARY_DIR}/benchmark/driver.mjs" COPYONLY)
    configure_file(../bench/wasm/run.mjs "${CMAKE_BINARY_DIR}/benchmark/run.mjs" COPYONLY)
    configure_file(../bench/wasm/browser.html "${CMAKE_BINARY_DIR}/benchmark/browser.html" COPYONLY)
    configure_file(../bench/wasm/browser.mjs "${CMAKE_BINARY_DIR}/benchmark/browser.mjs" COPYONLY)
    add_custom_target(silk_wasm_bench_metadata ALL
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/bench/wasm/metadata.py"
            --root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
            --compiler "${CMAKE_C_COMPILER}" --mode "$<CONFIG>"
        DEPENDS silk_wasm_bench
        VERBATIM)
    configure_file(views.mjs "${CMAKE_BINARY_DIR}/package/views.mjs" COPYONLY)
    configure_file(index.mjs "${CMAKE_BINARY_DIR}/package/index.mjs" COPYONLY)
endif()
