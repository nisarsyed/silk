# silk

A portable, data-oriented physics and simulation engine written in C.
The core is renderer-independent and has no third-party dependencies.

Silk provides:

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
- deterministic constraint islands, opt-in whole-island sleeping and wake propagation
- bounded world queries and copied work/memory diagnostics
- a raylib collision sandbox and deterministic seven-fixture benchmark executable

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
| `SL_BUILD_EXAMPLE` | `OFF` | Build the public headless example |
| `SL_INSTALL` | top-level only | Install the static library, headers and CMake package |
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
- Left-hold a body to wake and tether it; exact shape containment selects the
  nearest center, with body-slot ties. Off-center grabs apply torque.
- Press `C` to toggle contact points and normals; gold points are speculative
  contacts with positive separation.
- Press `A` to toggle tight shape bounds (yellow) and fat proxies (blue).
- Press `1`–`4` for playground, settled piles, moving support, or joint chain.
- Press `S` to toggle sleeping and rebuild the selected scene; sleeping starts on.
- Press `M` in the support scene to start/stop its fixed-step motion.
- Press `I` for last-build island colors; sleeping bodies are dim in both modes.
- Press `Q` to cycle point, closest-ray, and AABB inspection. Right-drag defines
  rays/boxes; release selects the first result without waking it. Queries show
  up to eight handles and explicitly report truncation. Picking uses a separate
  full-capacity buffer, so demo truncation does not restrict selection.
- Press `J` to spawn a bounded six-link revolute chain anchored at the cursor;
  joint anchors and connections are drawn automatically.
- Press `Space` to pause or resume, `.` to step while paused, and `R` to rebuild the selected scene.

The deterministic default scene combines falling bodies with a ten-row box
pyramid on colliding static ground. Shapes use friction 0.6 and restitution
0.1, with four solver substeps per fixed step. The HUD reports body, contact,
and joint counts/capacities, awake/asleep counts, last-step work and island
counts, cumulative work/drops, and exact allocated memory plus the owning shell.
Selected-body values and island diagnostics use copied public accessors. Island
IDs describe the last completed build and may reflect topology before a mutation;
they are not persistent handles. Query inspection does not wake bodies.

The piles/support fixtures use restitution zero; the scene geometry and seed are
fixed in the sandbox source. Pause and single-step apply support motion and
tether forces only when a simulation step executes. Reset rebuilds the selected
fixture and clears all old handles, queries, and interaction state.

## Deterministic benchmark

Build and run the optional benchmark in release mode:

```sh
cmake --preset release -DSL_BUILD_BENCH=ON
cmake --build --preset release
./build/release/bin/sl_bench
```

`sl_bench` runs seven fixed fixtures: pyramid, rain, disconnected piles,
chains, churn, a table and an inverted-mass stack. Each uses 120 warm-up steps
and 600 measured steps at 1/60 s with four substeps. JSON reports include step
timing, work/memory diagnostics, physical-quality metrics and a semantic digest
of public body, contact and joint state.

Use `--scene`, `--warmup` and `--steps` to select a bounded workload.
The standard-library `bench/report.py` tool repeats runs and validates reports.
Digests check determinism within the same platform and build; physical-quality
limits assess cross-build results. See [benchmark documentation](docs/benchmarks.md)
for the schema, fixtures, quality limits and profiling commands.

## Use from C

Build the public headless example, which checks initialization and pool failure,
creates bodies and a joint, steps, inspects contacts, queries sleeping bodies,
wakes a component, resets and cleans up:

```sh
cmake --preset debug -DSL_BUILD_EXAMPLE=ON
cmake --build --preset debug
./build/debug/bin/silk_headless
```

An existing CMake project can embed Silk without enabling its tests, examples
or installation rules:

```cmake
add_subdirectory(path/to/silk)
target_link_libraries(my_app PRIVATE silk::silk)
```

For an installed static package:

```sh
cmake --preset release -DSL_INSTALL=ON
cmake --build --preset release
cmake --install build/release --prefix "$PWD/build/silk-prefix"
```

In the consuming project:

```cmake
cmake_minimum_required(VERSION 3.28)
project(my_app LANGUAGES C)
find_package(silk 0.3.0 CONFIG REQUIRED)
add_executable(my_app main.c)
target_link_libraries(my_app PRIVATE silk::silk)
```

Configure that project with `-DCMAKE_PREFIX_PATH=/path/to/silk-prefix`. The
prefix can be moved before configuring a fresh consumer. The target supplies
public include paths, the required C standard and platform math linkage; Silk's
first-party warnings do not become consumer warnings. Install includes only
the archive, public headers and CMake metadata. `SL_INSTALL` defaults on for a
top-level build and off when embedded; `SL_BUILD_EXAMPLE` defaults off.

The package uses exact version matching when a version is requested. The
development tree remains version 0.3.0 until the planned release; this is not a
stable binary ABI promise. Rebuild applications against matching headers and
archive. Public headers are independently compiled, with an additional C++
linkage smoke check for the existing `extern "C"` guards; no C++ wrapper API is
provided.

Run the same external checks used in compiler CI with:

```sh
python3 tools/check_consumer.py --output build/consumer-checks --config Debug
python3 tools/check_consumer.py --output build/consumer-release --config Release
```

The check copies a clean source tree and consumer, builds both integration
paths, moves the installation and removes the original source/build paths from
reach before rebuilding the installed consumer. It checks every public header,
linkage, version rejection and downstream compile options without fetching
any dependencies.

A world is a zero-initialized owning shell: call `sl_world_init` and
`sl_world_destroy`, and never copy it while initialized. Storage is private;
inspect bodies, joints and diagnostics through the public accessors. Handles
belong to the world lifetime that created them. Reset invalidates old handles;
numerical handle collisions across worlds or after destroy/reinitialize are
possible. Walk packed rows with `sl_world_body_at()` when removing bodies during
traversal, as documented in the header.

Include the subsystem headers you use, such as `<silk/math.h>`,
`<silk/shape.h>`, `<silk/contact.h>`, `<silk/joint.h>`, `<silk/world.h>`,
`<silk/step.h>`, and `<silk/query.h>`. `<silk/silk.h>` exposes the version macros
and `sl_version()`.
The core chooses no timestep: call `sl_world_step` with one fixed value
throughout a run, or initialize `sl_stepper` with your application's fixed
value. The sandbox's 1/60-second timestep is an example policy, not a library
default.

World queries provide tight-AABB overlap, shape point containment and closest
ray hits through `<silk/query.h>`. Buffer queries return ascending body slots,
total matches and truncation; they accept zero capacity for counting. Queries
preserve snapshots and simulation state but share scratch, so same-world
operations must not run concurrently. See the header for type masks and ray bounds.

`sl_world_config.substep_count` defaults to four and accepts one through eight.
Gravity and drag default to zero; sleeping defaults off. Enabling sleep selects
0.02 units/s, 0.01 radians/s and 0.5 seconds when its thresholds are zero.
Choose consistent length/mass/time units; angles and angular speeds use radians.
Body friction and restitution default to zero; joint storage is opt-in through
a nonzero `joint_capacity`. Size body, contact, and joint pools at initialization
for the workload. Zero contact capacity selects up to four times body capacity,
bounded by `SL_CONTACT_COUNT_MAX`. Initialization validates before allocating;
body/joint creation returns a null handle on capacity failure. Recoverable
invalid input returns failure without partial mutation. Contact-pool exhaustion
drops new pairs and is observable
through `sl_world_contact_drop_count()`.

For source migration from the original 0.3.0 tag toward the planned 0.4 release,
replace direct world-storage access with public accessors, scoped handles and
copied statistics. Do not copy the owning world or depend on packed addresses.
Query results contain handles, not retained body pointers. Shape pointers last
until the next create/destroy/reset/shape replacement; contact pointers last
until the next non-const world operation. Consume or copy snapshots before those
operations. Queries reuse scratch, so serialize all operations on the same world.
There are no compatibility aliases for the removed public storage layout.

## Current limits

- No continuous collision detection (CCD) or bullet bodies; fast objects can
  tunnel despite speculative contacts and free-integration speed caps.
- No sensors or contact callbacks. Islands and opt-in sleeping are supported.
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
