# Thread pool for the benchmark and sandbox (example code, not part of silk).
# It only needs silk's headers; its users link silk themselves.
add_library(silk_pool STATIC pool.c)
target_include_directories(silk_pool PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(silk_pool PRIVATE
    $<TARGET_PROPERTY:silk,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(silk_pool PRIVATE silk_warnings)
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(silk_pool PRIVATE Threads::Threads)
endif()
