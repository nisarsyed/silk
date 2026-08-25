# Silk — Roadmap

Living document, updated as milestones land. It records direction, not dates or commitments. Design principles and coding law live in [AGENTS.md](AGENTS.md).

**Status:** Phase 2 · Basic simulation (in progress)

## Baseline

| Concern | Direction |
| --- | --- |
| Language | C17, portable ISO C only (`C_EXTENSIONS OFF`) |
| Build | CMake ≥ 3.28 with `debug` / `release` / `sanitize` / `sandbox` presets |
| Toolchain | Clang + GCC on Linux/macOS CI; warnings-as-errors on the core |
| Dependencies | None in the core; raylib linked only by the sandbox example |
| API | `sl_` prefix, snake_case; SoA simulation state; generational body handles |
| Simulation | Fixed-timestep accumulator (default 1/60 s), semi-implicit Euler, gravity + drag; deterministic fixed timestep |
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

## Phase Progression

Condensed from the technical specification. Near-term phases stay itemized; later ones remain coarse until they become near-term.

| Phase | Theme | Scope |
| --- | --- | --- |
| 1 | Foundation | Milestone 1 above |
| 2 | Basic simulation *(current)* | Particle system; forces & integration; 2D shapes & geometry; basic simulation loop |
| 3 | 2D rigid-body physics | Collision detection & resolution; friction & restitution; broad-phase & spatial acceleration; constraints & joints |
| 4 | 2D engine maturity | Determinism hardening; memory & data-oriented optimization; debug rendering; profiling & benchmarking; public engine API |
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
