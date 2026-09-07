# silk

A portable, data-oriented physics and simulation engine written in pure C17.
The core is renderer-independent and has no third-party dependencies.

Silk 0.3.0 provides the Phase 3 2D rigid-body engine:

- 2D vectors, matrices, rotations, transforms, and AABBs
- circles and convex polygons with mass, bounds, point, and ray queries
- a bounded SoA world with generational body and joint handles, with runtime
  storage allocated at initialization and allocation-free stepping
- dynamic, kinematic, and static bodies with linear and angular state
- force, torque, gravity, drag, and substepped semi-implicit Euler integration
- a deterministic fixed-step API and bounded accumulator helper
- circle and polygon contact manifolds with stable feature IDs and speculative
  contacts, backed by a dynamic AABB-tree broad phase and persistent contact pool
- Soft Step sequential impulses with warm starting, an unbiased relax pass,
  per-body Coulomb friction, and restitution
- distance and revolute joints with optional connected-body collision suppression
- a raylib collision sandbox and deterministic pyramid/rain benchmark executable

See [ROADMAP.md](ROADMAP.md) for the merged implementation trail and next phase.

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
| `SL_BUILD_BENCH` | `OFF` | Build `sl_bench`, separate from CTest |

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
- Press `C` to toggle contact points and normals; gold points are speculative
  contacts with positive separation.
- Press `A` to toggle exact fat broad-phase proxy AABBs.
- Press `J` to spawn a bounded six-link revolute chain anchored at the cursor;
  joint anchors and connections are drawn automatically.
- Press `Space` to pause or resume, `.` to step while paused, and `R` to reset.

The deterministic default scene combines falling bodies with a ten-row box
pyramid on colliding static ground. Shapes use friction 0.6 and restitution
0.1, with four solver substeps per fixed step. The HUD reports body, contact,
and joint counts/capacities and dropped contacts.

## Deterministic benchmark

Build and run the optional benchmark in release mode:

```sh
cmake --preset release -DSL_BUILD_BENCH=ON
cmake --build --preset release
./build/release/bin/sl_bench
```

`sl_bench` runs a 20-row box pyramid and a seeded 2,000-circle rain scene.
Each scene uses 120 warm-up steps followed by 600 measured steps at 1/60 s
with four substeps. It reports average and maximum time per complete step,
body/contact capacities, final contact count, dropped contacts, compiler/build/
host metadata, and an FNV-1a checksum of binary32 body position, angle, and
linear/angular velocity in ascending body-slot order.

Run the same executable again to compare checksums. Timing surrounds engine
calls and never enters simulation state; wall-clock results vary with the host
and scheduling. Checksums require IEEE-754 binary32 floats and are a regression
aid for the same platform and build, not a cross-platform numerical guarantee.
The harness rejects unavailable, invalid, or backward clock samples. It is a
developer tool and is not registered as a CTest test.

The expanded matrix adds disconnected piles, chains, churn, a table and an
inverted-mass stack. Use `--scene all --format json` or the standard-library
`bench/report.py` repeat-run tool. Reports include work/memory diagnostics,
percentiles and physical-quality metrics; see [benchmark documentation](docs/benchmarks.md)
for bounded options, schema, fixtures, quality limits and profiling commands.

## Use as a CMake subproject

Silk currently exports its target from the source tree:

```cmake
add_subdirectory(path/to/silk)
target_link_libraries(my_app PRIVATE silk::silk)
```

Include the subsystem headers you use, such as `<silk/math.h>`,
`<silk/shape.h>`, `<silk/contact.h>`, `<silk/joint.h>`, `<silk/world.h>`, and
`<silk/step.h>`. `<silk/silk.h>` exposes the version macros and `sl_version()`.
The core chooses no timestep: call `sl_world_step` with one fixed value
throughout a run, or initialize `sl_stepper` with your application's fixed
value. The sandbox's 1/60-second timestep is an example policy, not a library
default.

`sl_world_config.substep_count` defaults to four and accepts one through eight.
Body friction and restitution default to zero; joint storage is opt-in through
a nonzero `joint_capacity`. Size body, contact, and joint pools at initialization
for the workload. Contact-pool exhaustion drops new pairs and is observable
through `sl_world_contact_drop_count()`.

## Current limits

- No continuous collision detection (CCD) or bullet bodies; fast objects can
  tunnel despite speculative contacts and free-integration speed caps.
- No sleeping or islands, sensors, or contact callbacks.
- Distance and revolute joints have no limits, motors, or user-facing springs.
- The CPU solver is scalar and single-threaded; SIMD and threading are deferred.
- Contacts are read-only snapshots. A pointer returned by
  `sl_world_contact_at()` is valid only until the next non-const world operation,
  as specified in [world.h](include/silk/world.h); consume or copy it before
  stepping or mutating the world.

Simulation is deterministic for a fixed call sequence on a given platform and
build. Cross-platform numerical comparisons require explicit tolerances;
bitwise equivalence across compilers or platforms is not promised.

## Acknowledgments

Erin Catto's [Solver2D article](https://box2d.org/posts/2024/02/solver2d/)
describes the substepping, soft-constraint, and relaxation techniques used in
this solver family, including Ross Nordby's mass-independent softness
parameters. Silk's solver design and its local two-point contact solve are
documented in [PR #49](https://github.com/nisarsyed/silk/pull/49).

## Contributing

Engineering principles, verification requirements, and repository workflow are
in [CONTRIBUTING.md](CONTRIBUTING.md). Instructions for coding agents are in
[AGENTS.md](AGENTS.md). Silk is available under the [MIT License](LICENSE).
