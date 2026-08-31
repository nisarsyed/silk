# silk

A portable, data-oriented physics and simulation engine written in pure C17.
The core is renderer-independent and has no third-party dependencies.

Silk currently provides the Phase 2 simulation foundation:

- 2D vectors, matrices, rotations, transforms, and AABBs
- circles and convex polygons with mass, bounds, point, and ray queries
- an allocation-bounded SoA world with generational body handles
- dynamic, kinematic, and static bodies with linear and angular state
- force, torque, gravity, drag, and semi-implicit Euler integration
- a deterministic fixed-step API and bounded accumulator helper

Collision detection and response are not implemented yet; they are the next
phase. See [ROADMAP.md](ROADMAP.md) for the progression.

## Requirements

- CMake 3.28 or newer
- A C17 compiler
- Git and network access only when configuring the optional sandbox, which
  fetches raylib 6.0

## Build and test

Configure, build, and test a debug build:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

The release, sanitizer, and Windows presets use the same workflow:

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release

cmake --preset sanitize
cmake --build --preset sanitize
ctest --preset sanitize

cmake --preset windows
cmake --build --preset windows
ctest --preset windows
```

Executables are written to `build/<preset>/bin`. The test binary can also be
run directly as `./build/debug/bin/sl_tests`.

Available CMake options:

| Option | Default | Purpose |
| --- | --- | --- |
| `SL_BUILD_TESTS` | On at the top level; off as a subproject | Build and register `sl_tests` |
| `SL_SANITIZERS` | `OFF` | Enable AddressSanitizer and UndefinedBehaviorSanitizer |
| `SL_SANDBOX` | `OFF` | Fetch raylib and build the interactive sandbox |

## Interactive sandbox

The sandbox is a raylib debug playground for the current engine features. It
uses metres for simulation, renders at 100 pixels per metre, and steps at 60 Hz.
On Linux, raylib also needs the platform OpenGL and window-system development
packages.

```sh
cmake --preset sandbox
cmake --build --preset sandbox
./build/sandbox/bin/sandbox
```

Use `sandbox-release` in place of `sandbox` to compile the example with release
settings.

Controls:

- Left-drag empty space to spawn and launch a body.
- Press `B` to switch between circles and boxes.
- Left-hold a body to tether it; off-center grabs apply torque.
- Press `Space` to pause or resume, `.` to step while paused, and `R` to reset.

Bodies intentionally pass through the preview ground and one another until the
Phase 3 contact-response layer lands.

## Use as a CMake subproject

Silk currently exports its target from the source tree:

```cmake
add_subdirectory(path/to/silk)
target_link_libraries(my_app PRIVATE silk::silk)
```

Include the subsystem headers you use, such as `<silk/math.h>`,
`<silk/shape.h>`, `<silk/contact.h>`, `<silk/world.h>`, and `<silk/step.h>`.
The core chooses no timestep: call `sl_world_step` with one fixed value
throughout a run, or
initialize `sl_stepper` with your application's fixed value. The sandbox's
1/60-second timestep is an example policy, not a library default.

## Contributing

Engineering principles, verification requirements, and repository workflow are
in [CONTRIBUTING.md](CONTRIBUTING.md). Instructions for coding agents are in
[AGENTS.md](AGENTS.md). Silk is available under the [MIT License](LICENSE).
