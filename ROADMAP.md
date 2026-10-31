# Silk — Roadmap

This is a living document, updated as milestones land. It records direction,
not dates or commitments. Engineering principles and contribution policy live
in [CONTRIBUTING.md](CONTRIBUTING.md). The solver's design evidence lives in
[docs/solver.md](docs/solver.md).

## Vision

Silk aims to be a general-purpose, deterministic physics engine in portable
C17: cloth, rigid bodies, soft bodies, and rods in one world, interacting
with each other. Most engines bolt separate solvers together; silk builds
every module on one solver family, block descent (VBD and its extensions),
so modules share contact, collision, and scheduling, and coupling between
them comes from the same solve rather than glue code.

The target is the best that a single consumer machine allows, not AAA scale.
Scenes of thousands to tens of thousands of elements should run in real time
on a laptop-class CPU and, later, in a browser. Scale beyond that waits for a
GPU backend; it does not drive the design.

**Status:** 0.1.0 is a 3D cloth engine: the VBD cloth solver (Milestone 1)
and Offset Geometric Contact for self and cloth–cloth contact (Milestone 2).
Performance is next, then rigid bodies, which will shape the shared core.

Silk was an unreleased 2D rigid-body engine through 0.4.0. In October 2026 it
was rewritten from scratch as a 3D engine on the same engineering baseline,
and numbering restarted at 0.1.0. The 2D history remains in git, and its
browser work is archived at the `archive/wasm-2d*` tags.

## How silk grows

- **Research before code.** Each new module starts with a survey of current
  methods and, where the choice is close, a head-to-head prototype on
  shared scenes, recorded in `docs/`. The cloth solver was chosen this way
  (VBD over XPBD and SIPD).
- **One core, several modules.** The code splits into a core and modules when
  the second module exists, so that a real second consumer defines what is
  shared. The expected core is math, the arena and capacities, the
  fixed-timestep stepper, a world holding several kinds of objects,
  colliders and the broad phase, contact, graph coloring, and the colored
  sweep scheduler. Cloth becomes the first module.
- **Coupling is the point.** A module is done when it interacts with the
  existing ones in the same solve, not when it runs alone.
- **The baseline holds for every module:** determinism, no allocation per
  step, bounded work, SoA state, and no core dependencies.

## Baseline

| Concern | Direction |
| --- | --- |
| Language | C17, portable ISO C only (`C_EXTENSIONS OFF`) |
| Build | CMake ≥ 3.28 with `debug` / `release` / `sanitize` / `windows` / `sandbox` / `sandbox-release` presets |
| Toolchain | GCC and Clang on Linux CI; Clang on macOS CI; MSVC on Windows CI; warnings are errors in the core |
| Dependencies | None in the core; raylib 6.0 only for the sandbox |
| API | `sl_` prefix, snake_case; SoA state; one arena allocated at init |
| Simulation | Fixed timestep; block descent on the variational implicit-Euler energy; substeps chosen from stiffness; deterministic for a fixed call sequence on a given platform and build |
| Tests | Hand-rolled framework (`tests/silk_test.h`) integrated with CTest; deterministic seeded PRNGs only |
| License | MIT |

## Done

### Milestone 1 — VBD cloth core (0.1.0)

- [x] Solver survey; VBD chosen, with OGC planned for contact ([docs/solver.md](docs/solver.md))
- [x] Head-to-head check: VBD, XPBD, and SIPD (SIGGRAPH Asia 2026) on shared scenes (`sl_compare`). VBD kept; SIPD rejected as conditionally stable and slower on CPU; XPBD stretches less per sweep but needs more substeps to stay stable with contact or at fine resolution ([docs/solver.md](docs/solver.md#head-to-head-comparison-october-2026))
- [x] 3D math: `vec3`, `mat3`, guarded 3×3 solve
- [x] Mesh ingestion: validation without partial mutation, CSR adjacency, interior-edge bending stencils, unique edges, greedy coloring, lumped mass; grid generator
- [x] Energies: StVK membrane with a PSD-clamped vertex Hessian; quadratic bending calibrated to the plate modulus; finite-difference and closed-form tests
- [x] Solver: colored Gauss–Seidel vertex blocks; pinned targets interpolated over substeps; drag and wind; edge stretch damping; velocity-level Coulomb friction; automatic stability substeps
- [x] Colliders: plane, sphere, and capsule with penalty contact
- [x] Verification: free fall, energy stability (drape and swing), friction stick and slide, determinism, and capacity and validation tests
- [x] Tooling: deterministic benchmark, headless example, consumer and package checks, CI
- [x] raylib 3D sandbox: scenes, grabbing, orbit camera, scripted capture

### Milestone 2 — Self contact (0.1.0)

- [x] Broad phase: deterministic BVHs over triangles and edges, built from the topology and refit at each detection, with a self traversal for edge pairs. A spatial hash was tried first and rejected on measured cost.
- [x] Offset Geometric Contact (Chen et al., SIGGRAPH 2025): vertex–facet and edge–edge contacts in the paper's offset blocks, its two-stage barrier, and displacement bounds instead of CCD. Bounds are translation-relative per cloth so moving cloth is not braked ([docs/solver.md](docs/solver.md#self-contact-offset-geometric-contact)).
- [x] Conservative contact forces: every pair pushes its whole stencil
- [x] Cloth–cloth contact between separate cloths in one world
- [x] Velocity-level Coulomb friction for self contact
- [x] Sandbox: self contact in every scene, a pile of three sheets, one step per frame at most, and a self-contact toggle; benchmark pile scene

## Next

### Milestone 3 — Performance

Make what exists fast, in a form the future core can reuse.

- [x] Deterministic threading through a host-supplied `parallel_for`: colored sweeps and per-vertex substep work, bitwise identical for any thread count (2.5× on the 64×64 curtain with 8 threads)
- [ ] Parallel edge damping that keeps its strength: in color order it damped long waves far less and a stiff drape went unstable ([docs/solver.md](docs/solver.md#edge-damping-stays-serial))
- [ ] Cheaper dense piles: fewer redetections and parallel detection
- [ ] SIMD for the membrane and bending kernels, kept only if measurement shows a gain
- [x] Faster CI: change-aware quick and full tiers, and optimized debug and sanitizer builds
- [ ] Contact debug drawing in the sandbox

### Milestone 4 — Core and rigid bodies

Rigid bodies are the backbone of a general-purpose engine and the second
module that defines the core. Threading (Milestone 3) lands first, so the
rigid solver is designed against it.

- [x] Research: rigid-body solvers surveyed, with claims checked against independent evidence ([docs/rigid.md](docs/rigid.md)). AVBD is not adopted on the evidence alone: its joints and coupling are strong, but its paper parameters are tuned to one set of units and its contact stability on a CPU is unproven against Soft Step.
- [ ] Collision foundation shared by either solver: convex contact generation (box first), contacts persisting across steps, and a broad phase over bodies
- [ ] Head-to-head at equal CPU time: VBD-family rigid bodies (compliant augmented Lagrangian) against Soft Step, decided by the rule in [docs/rigid.md](docs/rigid.md#proposed-experiment). If Soft Step wins, rigid bodies are a second solver coupled to cloth by staggering.
- [ ] Split the code into the core and a cloth module, with rigid bodies as the second consumer
- [ ] Rigid bodies: box, sphere, capsule, and convex-hull shapes; mass properties; contact with friction; stable stacking
- [ ] Joints: ball, hinge, prismatic, and fixed, with limits and motors
- [ ] Sleeping and islands, if measurement shows a need
- [ ] Two-way cloth–rigid coupling: cloth draped on and pushed by moving rigid bodies

### Milestone 5 — Demo

Show the engine where people can try it.

- [ ] WebAssembly build of the core and modules, started fresh with the lessons from the archived 2D stack: a pinned Emscripten toolchain, fixed-step frame pacing, and mobile CPUs being far slower
- [ ] Browser demo: cloth and rigid bodies together, desktop first
- [ ] Recorded clips of the sandbox scenes for the README

### Milestone 6 — Soft bodies

- [ ] Tetrahedral mesh ingestion and a tetrahedralization path for closed surface meshes
- [ ] Hyperelastic volume energies (Neo-Hookean first) in the shared solver
- [ ] Surface contact through the existing OGC contact
- [ ] Coupling with cloth and rigid bodies

### Milestone 7 — Rods

- [ ] Ropes, cables, and hair as elastic rods in the shared solver
- [ ] Edge contact with cloth, bodies, and other rods
- [ ] Attachment to rigid bodies and cloth

### Ongoing — Cloth quality

Cloth work that can land between milestones:

- [ ] Stretch error: evaluate multilevel or coarse-correction remedies (MGPBD, SIGGRAPH 2025) against more substeps, decided on evidence; XPBD's lower stretch per sweep in `sl_compare` is the bar to beat
- [ ] Revisit initial-guess and acceleration ideas (adaptive guess, Chebyshev, Anderson) under the automatic substep rule, keeping the energy-stability tests
- [ ] Dihedral-angle bending for curved rest shapes
- [ ] Anisotropic warp and weft materials
- [ ] Triangle-mesh and SDF colliders; moving-collider velocity in friction
- [ ] Seams between cloths

## Later

- Scene queries: raycasts, overlaps, and shape casts across all modules.
- Character controller built on the queries.
- Particles, granular media, and fluids. These share little with the
  block-descent core: fluids need a global pressure solve over a volume and
  usually a GPU, so they would be a separate solver on the shared world and
  colliders.
- A GPU backend for the color sweeps (WebGPU, Vulkan, or Metal), behind no
  abstraction until a backend exists.

## Non-goals

- AAA scale: millions of elements, or GPU-first design.
- Third-party dependencies in the core.
- Rendering or platform APIs inside the core.
- Wall-clock time or non-deterministic stepping in simulation results.
- Speculative abstractions, dispatch layers, or GPU and rendering backends
  before a concrete requirement exists.
