# Silk — Roadmap

Living document, updated as milestones land. It records direction, not dates or
commitments. Engineering principles and contribution policy live in
[CONTRIBUTING.md](CONTRIBUTING.md).

**Status:** Phase 4 · 2D engine maturity complete (0.4.0).
Phase 5 · WebAssembly is next.

## Baseline

| Concern | Direction |
| --- | --- |
| Language | C17, portable ISO C only (`C_EXTENSIONS OFF`) |
| Build | CMake ≥ 3.28 with `debug` / `release` / `sanitize` / `windows` / `sandbox` / `sandbox-release` presets |
| Toolchain | GCC + Clang on Linux CI; Clang on macOS CI; MSVC on Windows CI; warnings-as-errors on the core |
| Dependencies | None in the core; raylib linked only by the sandbox example |
| API | `sl_` prefix, snake_case; SoA simulation state; generational body and joint handles |
| Simulation | Caller-selected fixed timestep (sandbox: 1/60 s), bounded accumulator, 1–8 substeps (default 4), semi-implicit Euler with stored rotations, gravity + drag, Soft Step contacts and joints, friction + restitution; deterministic for a fixed call sequence on a given platform/build |
| Tests | Hand-rolled framework (`tests/silk_test.h`), CTest-integrated; seeded deterministic PRNGs only |
| License | MIT |

## Milestone 1 — Foundations

- [x] Repository setup — public repo, MIT license
- [x] Project scaffold: CMake targets (core, tests, sandbox), test framework, sandbox stub ([#1](https://github.com/nisarsyed/silk/pull/1), merged)
- [x] Tooling pass — CMake presets, sanitizers, strict warnings, GitHub Actions CI, `SL_ASSERT` ([#3](https://github.com/nisarsyed/silk/pull/3), merged); added during the eng-standards review, beyond the original plan
- [x] 2D math library: `vec2`, `mat2`, scalar utilities, epsilon comparison + tests ([#2](https://github.com/nisarsyed/silk/pull/2), merged)
- [x] Simulation state: SoA world, body pool, generational handles + tests ([#8](https://github.com/nisarsyed/silk/pull/8), merged)
- [x] Integration & forces: fixed-timestep loop, force accumulation, determinism tests ([#9](https://github.com/nisarsyed/silk/pull/9), merged)
- [x] Raylib sandbox: interactive demo — spawn/drag bodies, debug draw, pause/step

## Milestone 2 — Basic simulation

Phase 2 delivery: transforms, shape geometry, mass properties, body types,
and rotational integration — everything Phase 3 collision consumes.

- [x] Math: rotation, transform, AABB primitives, angle wrap
- [x] 2D shapes & geometry: circle + convex polygon shapes, mass properties, point/ray queries
- [x] Rigid-body state: body types (static/kinematic), angular state, shape attachment, rotational integration
- [x] Sandbox: shaped bodies, static ground, engine picking

## Milestone 3 — 2D rigid-body physics

Phase 3 implementation is merged: collision detection and response, materials,
broad-phase, the first joints, and a deterministic performance baseline, on a
Soft Step substepped solver. The 0.3.0 implementation is scalar and
single-threaded.

- [x] Math: robust rotation integration and composition, 2x2 solve, AABB set operations, and segment distance ([issue #28](https://github.com/nisarsyed/silk/issues/28); [PR #42](https://github.com/nisarsyed/silk/pull/42), merged)
- [x] Rigid-body state: stored rotations, substepped integration, per-body friction and restitution, and bounded speed controls ([issues #29](https://github.com/nisarsyed/silk/issues/29), [#32](https://github.com/nisarsyed/silk/issues/32); [PRs #43](https://github.com/nisarsyed/silk/pull/43), [#47](https://github.com/nisarsyed/silk/pull/47), merged)
- [x] Collision detection: circle and polygon manifolds with stable feature IDs and speculative distance; dynamic AABB-tree broad phase; persistent contact pool and pair set ([issues #30](https://github.com/nisarsyed/silk/issues/30), [#31](https://github.com/nisarsyed/silk/issues/31), [#33](https://github.com/nisarsyed/silk/issues/33); [PRs #45](https://github.com/nisarsyed/silk/pull/45), [#46](https://github.com/nisarsyed/silk/pull/46), [#48](https://github.com/nisarsyed/silk/pull/48), merged)
- [x] Collision response: Soft Step sequential impulses with warm starting, relax pass, friction, and restitution ([issue #34](https://github.com/nisarsyed/silk/issues/34); [PR #49](https://github.com/nisarsyed/silk/pull/49), merged)
- [x] Constraints: SoA joint pool with generational handles; distance and revolute joints ([issue #36](https://github.com/nisarsyed/silk/issues/36); [PR #52](https://github.com/nisarsyed/silk/pull/52), merged)
- [x] Sandbox: collision scene, materials, and contact/AABB/joint debug drawing ([issue #37](https://github.com/nisarsyed/silk/issues/37); [PR #53](https://github.com/nisarsyed/silk/pull/53), merged)
- [x] Benchmark: deterministic pyramid and rain scenes with warm-up, per-step timing, and a state checksum ([issue #35](https://github.com/nisarsyed/silk/issues/35); [PR #51](https://github.com/nisarsyed/silk/pull/51), merged)
- [x] Wrap: version 0.3.0, README, roadmap, attribution, and release verification ([issue #38](https://github.com/nisarsyed/silk/issues/38); [PR #55](https://github.com/nisarsyed/silk/pull/55))

The 0.3.0 implementation limits are explicit: no CCD/bullets, sleeping/islands,
sensors, contact callbacks, joint limits/motors/user-facing springs, SIMD, or
threading.
Contacts are read-only snapshots valid only until the next non-const world
operation. These describe Phase 3's delivered state; future scope is recorded
below.

## Milestone 4 — 2D engine maturity

Phase 4 delivers 0.4.0. Correctness, API cleanup, diagnostics, and performance
are complementary workstreams; performance is one part of engine maturity.
The [Phase 4 milestone](https://github.com/nisarsyed/silk/milestone/2) and
[execution tracker #56](https://github.com/nisarsyed/silk/issues/56) track the
work, dependencies, and short gh-stack PR chains. Merges remain with the
maintainer; items are checked only after their implementation reaches `main`.

- [x] Correctness: deterministic replay and mutation regression coverage ([#57](https://github.com/nisarsyed/silk/issues/57))
- [x] Measurement: deterministic work counters, exact memory diagnostics, expanded benchmark workloads, and developer-tool CI ([#58](https://github.com/nisarsyed/silk/issues/58), [#59](https://github.com/nisarsyed/silk/issues/59), [#60](https://github.com/nisarsyed/silk/issues/60))
- [x] Public API: separate world ownership from private storage and add bounded spatial queries ([#61](https://github.com/nisarsyed/silk/issues/61), [#62](https://github.com/nisarsyed/silk/issues/62))
- [x] Activation: deterministic constraint islands, opt-in sleeping, and complete wake propagation ([#63](https://github.com/nisarsyed/silk/issues/63), [#64](https://github.com/nisarsyed/silk/issues/64))
- [x] Measured optimization round: investigate algorithmic cost, memory access, compiler vectorization, targeted SIMD, and solver ordering; assess threading feasibility ([#65](https://github.com/nisarsyed/silk/issues/65), [#66](https://github.com/nisarsyed/silk/issues/66))
- [x] Diagnostics and consumption: public-API sandbox diagnostics and a tested, documented C consumer contract ([#67](https://github.com/nisarsyed/silk/issues/67), [#68](https://github.com/nisarsyed/silk/issues/68))
- [x] Wrap: version 0.4.0, documentation, migration notes, and release verification ([#69](https://github.com/nisarsyed/silk/issues/69))

Merged delivery trail: [replay #72](https://github.com/nisarsyed/silk/pull/72),
[stats #73](https://github.com/nisarsyed/silk/pull/73),
[benchmarks #74](https://github.com/nisarsyed/silk/pull/74),
[CI #75](https://github.com/nisarsyed/silk/pull/75),
[ownership #77](https://github.com/nisarsyed/silk/pull/77),
[queries #78](https://github.com/nisarsyed/silk/pull/78),
[islands #80](https://github.com/nisarsyed/silk/pull/80),
[sleep #81](https://github.com/nisarsyed/silk/pull/81),
[worksets #83](https://github.com/nisarsyed/silk/pull/83),
[solver study #84](https://github.com/nisarsyed/silk/pull/84),
[sandbox #85](https://github.com/nisarsyed/silk/pull/85), and
[consumer #86](https://github.com/nisarsyed/silk/pull/86).

The measured round adopted empty joint-stage skipping without changing physics
or memory. Query compaction and explicit SIMD were rejected after matched
measurements; compiler vectorization defaults remain. Graph ordering and the
bounded two-pass solver adaptation were rejected on cost or physical-quality
evidence. Threading feasibility was assessed and implementation deferred.
The [benchmark guide](docs/benchmarks.md) links reproducible positive and
negative decisions. The core remains portable C and single-threaded.

CCD/bullets, sensors/contact events, joint limits/motors/user-facing springs,
serialization, and WASM implementation remain outside Phase 4's committed
scope. SIMD investigation and threading feasibility are in scope under the
decision gates above; neither is a required production backend for 0.4.0.

## Phase Progression

Condensed from the technical specification. Near-term phases stay itemized; later ones remain coarse until they become near-term.

| Phase | Theme | Scope |
| --- | --- | --- |
| 1 | Foundation | Milestone 1 above |
| 2 | Basic simulation *(complete)* | Particle system; forces & integration; 2D shapes & geometry; basic simulation loop |
| 3 | 2D rigid-body physics *(complete)* | Collision detection & resolution; friction & restitution; broad-phase & spatial acceleration; constraints & joints |
| 4 | 2D engine maturity *(complete)* | Correctness & determinism; public API cleanup & queries; islands/sleeping; diagnostics; benchmarks followed by a measured optimization round |
| 5 | WebAssembly *(next)* | WASM build target; C ↔ JavaScript API boundary; browser demos & debug visualization; measure and optimize the actual WASM/browser build |
| 6 | GPU foundation | Select a concrete particle-based simulation workload; build and optimize GPU memory/compute infrastructure for its demonstrated requirements |
| 7 | WebGPU | WebGPU execution and browser demonstrations of the selected workload; optimize suitable GPU workloads; broad-phase/constraint experiments stay outside the portable core |
| 8 | 3D foundation | 3D math & quaternions; rigid bodies, orientation & angular dynamics; shapes, collision & CCD; joints; spatial acceleration |
| 9 | Native GPU | Vulkan/Metal backends behind the GPU abstraction; GPU parallel solvers; CPU/GPU scheduling |
| 10 | Production core | Serialization & scene management; physics debugger; record/replay; cross-platform runtime; broader production validation & optimization |

Optimization recurs as execution targets and workloads change. Phase 4's
native CPU evidence is a baseline for Phase 5, which measures the actual
WASM/browser build. Phases 6–7 optimize workloads suited to GPU/WebGPU
execution. Phase 10 broadens validation and optimization across production
scenarios. Each round needs evidence from its own target and workload;
earlier results do not establish performance on later backends.

## Future Simulation Modules

These are staged planning opportunities, not committed module deliveries.
They build on the core without coupling optional modules into it. Neither
cloth versus fluids nor a solver algorithm is selected here.

- **After Phase 4:** small CPU 2D prototypes could explore a net or a fluid
  simulation and later become browser demonstrations. A 2D net can explore
  constrained particles and interaction in a plane; it is not full cloth
  simulation, including 3D deformation and self-collision.
- **When planning Phases 6–7:** select one concrete particle-based simulation
  workload so GPU infrastructure serves an actual requirement. GPU particles
  describe an execution workload; they do not by themselves constitute a
  fluid solver. Choose the physical model and algorithm only when that
  workload's requirements are understood.
- **After the Phase 8 3D foundation:** consider full 3D cloth/fluid interaction
  with rigid bodies, including the additional collision and coupling work
  required by the chosen module.

Other possible modules remain soft bodies, rope and hair, ragdolls and
articulated bodies, destruction/fracture, particle effects, smoke and fire,
and buoyancy/aerodynamics. Their scope and delivery stages are undecided.

## Execution Backends & Distribution

- **Data model** — contiguous SoA arrays (`positions[]`, `velocities[]`, …) processable by CPU scalar/SIMD/multithreaded and GPU compute paths alike.
- **Backends** — CPU first (correctness before acceleration); GPU as a first-class backend — WebGPU in browsers, Vulkan/Metal native — never hardcoded into simulation algorithms.
- **Renderer independence** — the core produces simulation state and debug geometry; raylib/WebGPU/future renderers consume it.
- **Distribution targets** — native static/shared libraries (`.a`/`.so`/`.dylib`/`.dll`), WASM package (`physics.wasm` + JS glue + TypeScript definitions), static web demo, source releases & docs.

## Non-Goals

- Third-party dependencies in the core
- Rendering or platform APIs inside the core
- Wall-clock time or non-deterministic stepping in simulation results
- Speculative GPU/rendering abstractions before a concrete requirement exists

## Definition of Success

The project succeeds when the same fundamental C simulation engine can:

1. Run a deterministic 2D rigid-body simulation on a native CPU
2. Run the same simulation through WebAssembly in a browser
3. Visualize simulations through an independent rendering layer
4. Support 3D rigid-body simulation
5. Execute suitable workloads through GPU compute
6. Run GPU simulations through WebGPU in the browser
7. Support specialized simulation modules without coupling them to the rigid-body core
8. Provide native and WASM distribution paths
9. Maintain automated testing, benchmarking, profiling, and validation
10. Keep a clean public API and a platform-independent core
