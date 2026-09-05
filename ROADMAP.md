# Silk — Roadmap

Living document, updated as milestones land. It records direction, not dates or
commitments. Engineering principles and contribution policy live in
[CONTRIBUTING.md](CONTRIBUTING.md).

**Status:** Phase 3 · 2D rigid-body physics complete (0.3.0).
Phase 4 · 2D engine maturity is next.

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
Soft Step substepped solver. SIMD and threading remain deferred.

- [x] Math: robust rotation integration and composition, 2x2 solve, AABB set operations, and segment distance ([issue #28](https://github.com/nisarsyed/silk/issues/28); [PR #42](https://github.com/nisarsyed/silk/pull/42), merged)
- [x] Rigid-body state: stored rotations, substepped integration, per-body friction and restitution, and bounded speed controls ([issues #29](https://github.com/nisarsyed/silk/issues/29), [#32](https://github.com/nisarsyed/silk/issues/32); [PRs #43](https://github.com/nisarsyed/silk/pull/43), [#47](https://github.com/nisarsyed/silk/pull/47), merged)
- [x] Collision detection: circle and polygon manifolds with stable feature IDs and speculative distance; dynamic AABB-tree broad phase; persistent contact pool and pair set ([issues #30](https://github.com/nisarsyed/silk/issues/30), [#31](https://github.com/nisarsyed/silk/issues/31), [#33](https://github.com/nisarsyed/silk/issues/33); [PRs #45](https://github.com/nisarsyed/silk/pull/45), [#46](https://github.com/nisarsyed/silk/pull/46), [#48](https://github.com/nisarsyed/silk/pull/48), merged)
- [x] Collision response: Soft Step sequential impulses with warm starting, relax pass, friction, and restitution ([issue #34](https://github.com/nisarsyed/silk/issues/34); [PR #49](https://github.com/nisarsyed/silk/pull/49), merged)
- [x] Constraints: SoA joint pool with generational handles; distance and revolute joints ([issue #36](https://github.com/nisarsyed/silk/issues/36); [PR #52](https://github.com/nisarsyed/silk/pull/52), merged)
- [x] Sandbox: collision scene, materials, and contact/AABB/joint debug drawing ([issue #37](https://github.com/nisarsyed/silk/issues/37); [PR #53](https://github.com/nisarsyed/silk/pull/53), merged)
- [x] Benchmark: deterministic pyramid and rain scenes with warm-up, per-step timing, and a state checksum ([issue #35](https://github.com/nisarsyed/silk/issues/35); [PR #51](https://github.com/nisarsyed/silk/pull/51), merged)
- [x] Wrap: version 0.3.0, README, roadmap, attribution, and release verification ([issue #38](https://github.com/nisarsyed/silk/issues/38); [PR #55](https://github.com/nisarsyed/silk/pull/55))

The remaining limits are explicit: no CCD/bullets, sleeping/islands, sensors,
contact callbacks, joint limits/motors/user-facing springs, SIMD, or threading.
Contacts are read-only snapshots valid only until the next non-const world
operation. These are deferred capabilities, not part of Phase 3's delivery.

## Phase Progression

Condensed from the technical specification. Near-term phases stay itemized; later ones remain coarse until they become near-term.

| Phase | Theme | Scope |
| --- | --- | --- |
| 1 | Foundation | Milestone 1 above |
| 2 | Basic simulation *(complete)* | Particle system; forces & integration; 2D shapes & geometry; basic simulation loop |
| 3 | 2D rigid-body physics *(complete)* | Collision detection & resolution; friction & restitution; broad-phase & spatial acceleration; constraints & joints |
| 4 | 2D engine maturity *(next)* | Determinism hardening; memory & data-oriented optimization; debug rendering; profiling & benchmarking; public engine API |
| 5 | WebAssembly | WASM build target; C ↔ JavaScript API boundary; browser demo infrastructure & web debug visualization |
| 6 | GPU foundation | GPU buffer/memory model; compute pipeline abstraction; GPU particle simulation as the first workload |
| 7 | WebGPU | WebGPU backend & compute pipelines; browser GPU particles; broad-phase/constraint experiments — stays outside the portable core |
| 8 | 3D foundation | 3D math & quaternions; rigid bodies, orientation & angular dynamics; shapes, collision & CCD; joints; spatial acceleration |
| 9 | Native GPU | Vulkan/Metal backends behind the GPU abstraction; GPU parallel solvers; CPU/GPU scheduling |
| 10 | Production core | Serialization & scene management; physics debugger; record/replay; cross-platform runtime; validation & optimization |

## Future Simulation Modules

Intentionally outside the primary progression; built on the core without coupling to it:

Soft bodies (PBD / XPBD / FEM) · cloth (rigid-body collision, self-collision, GPU) · fluids (SPH, FLIP/PIC hybrids, grid-based, GPU) · rope & hair · ragdolls & articulated bodies · destruction/fracture · particle effects, smoke, fire · buoyancy & aerodynamics

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
