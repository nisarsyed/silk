add_executable(silk_headless main.c)
target_link_libraries(silk_headless PRIVATE silk::silk silk_warnings)
