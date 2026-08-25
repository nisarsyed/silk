include(FetchContent)

# Pinned to raylib 6.0 — the current stable release as of Aug 2026:
# https://github.com/raysan5/raylib/releases/tag/6.0
set(SANDBOX_RAYLIB_TAG 6.0)

FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG ${SANDBOX_RAYLIB_TAG}
    GIT_SHALLOW TRUE
)

# Trim raylib to the library itself; the sandbox ships no extras.
# BUILD_EXAMPLES already defaults off under FetchContent (raylib keys it
# on PROJECT_IS_TOP_LEVEL); pinned anyway so a default flip cannot
# drag examples into this build.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib)

add_executable(sandbox main.c)
target_link_libraries(sandbox PRIVATE raylib silk silk_warnings)

# Raylib headers are third-party and not ours to lint: expose them as
# SYSTEM so their internals stay outside the strict warning set, while
# main.c keeps the full set and owns its conversions with explicit
# casts. Linking raylib propagates its include dirs; this re-adds the
# same directory with SYSTEM precedence.
target_include_directories(sandbox SYSTEM PRIVATE ${raylib_SOURCE_DIR}/src)
