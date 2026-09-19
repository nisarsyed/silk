if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "Silk's baseline WASM target requires wasm32")
endif()

# Apply only to first-party WASM executables. The static library retains the
# same portable C compilation and warning policy as native consumers.
function(sl_wasm_executable target)
    target_link_options(${target} PRIVATE
        -sENVIRONMENT=node
        -sALLOW_MEMORY_GROWTH=0
        -sABORTING_MALLOC=0
        -sINITIAL_MEMORY=268435456
        -sSTACK_SIZE=1048576
        -sEXIT_RUNTIME=1
        "$<$<CONFIG:Debug>:-sASSERTIONS=2>"
        "$<$<CONFIG:Debug>:-sSTACK_OVERFLOW_CHECK=2>"
        "$<$<CONFIG:Release>:-sASSERTIONS=0>"
    )
endfunction()
