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
#
# Do NOT set CUSTOMIZE_BUILD on the 6.0 tag: it switches raylib to
# EXTERNAL_CONFIG_FLAGS, and 6.0's ParseConfigHeader.cmake promotes every
# uncommented "#define SUPPORT_X 0" in config.h to ON — including
# SUPPORT_CUSTOM_FRAME_CONTROL, which removes the buffer swap and event
# polling from EndDrawing() and leaves a window that never renders.
# Fixed upstream after 6.0 (raysan5/raylib#5844, 2026-05-08); revisit
# once a release carries it. Audio stays compiled in; miniaudio dlopens
# its backends at runtime, so no audio dev packages are needed to build.
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
