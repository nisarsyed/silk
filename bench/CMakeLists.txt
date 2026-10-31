add_executable(sl_bench main.c)
target_link_libraries(sl_bench PRIVATE silk silk_pool silk_warnings)

# Research harness comparing VBD, XPBD and SIPD on silk's own scenes. It
# reads private state, so it sees src/; it is never part of the library.
add_executable(sl_compare compare.c)
target_link_libraries(sl_compare PRIVATE silk silk_warnings)
target_include_directories(sl_compare PRIVATE ${PROJECT_SOURCE_DIR}/src)
