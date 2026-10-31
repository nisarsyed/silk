# Silk — Roadmap

This is a living document, updated as milestones land. It records direction,
not dates or commitments. Engineering principles and contribution policy live
in [CONTRIBUTING.md](CONTRIBUTING.md). The solver's design evidence lives in
[docs/solver.md](docs/solver.md).

**Status:** the 3D cloth rewrite (0.1.0) and self contact (Milestone 2)
are implemented. Convergence and performance are next.

Silk was a 2D rigid-body engine through 0.4.0 (Phases 1–4 and the Phase 5
WebAssembly plan), never released. The cloth engine restarts numbering at
0.1.0. In October 2026 the project changed direction: silk is now a
3D cloth simulation engine, rewritten from scratch on the same engineering
baseline. The 2D history remains in git.

## Baseline

| Concern | Direction |
| --- | --- |
| Language | C17, portable ISO C only (`C_EXTENSIONS OFF`) |
| Build | CMake ≥ 3.28 with `debug` / `release` / `sanitize` / `windows` / `sandbox` / `sandbox-release` presets |
| Toolchain | GCC and Clang on Linux CI; Clang on macOS CI; MSVC on Windows CI; warnings are errors in the core |
| Dependencies | None in the core; raylib 6.0 only for the sandbox |
| API | `sl_` prefix, snake_case; SoA vertex, triangle, and stencil state; one arena allocated at init |
| Simulation | Fixed timestep; Vertex Block Descent on the variational implicit-Euler energy; substeps chosen from cloth stiffness; deterministic for a fixed call sequence on a given platform and build |
| Tests | Hand-rolled framework (`tests/silk_test.h`) integrated with CTest; deterministic seeded PRNGs only |
| License | MIT |

## Milestone 1 — VBD cloth core (0.1.0)

- [x] Solver survey; VBD chosen, with OGC planned for contact ([docs/solver.md](docs/solver.md))
- [x] Head-to-head check: VBD, XPBD, and SIPD (SIGGRAPH Asia 2026) on shared scenes (`sl_compare`). VBD kept; SIPD rejected as conditionally stable and slower on CPU; XPBD stretches less per sweep but needs more substeps to stay stable with contact or at fine resolution ([docs/solver.md](docs/solver.md#head-to-head-comparison-october-2026))
- [x] 3D math: `vec3`, `mat3`, guarded 3×3 solve
- [x] Mesh ingestion: validation without partial mutation, CSR adjacency, interior-edge bending stencils, unique edges, greedy coloring, lumped mass; grid generator
- [x] Energies: StVK membrane with a PSD-clamped vertex Hessian; quadratic bending calibrated to the plate modulus; finite-difference and closed-form tests
- [x] Solver: colored Gauss–Seidel vertex blocks; pinned targets interpolated over substeps; drag and wind; edge stretch damping; velocity-level Coulomb friction; automatic stability substeps
- [x] Colliders: plane, sphere, and capsule with penalty contact
- [x] Verification: free fall, energy stability (drape and swing), friction stick and slide, determinism, and capacity and validation tests
- [x] Tooling: deterministic benchmark, headless example, consumer and package checks, CI
- [x] raylib 3D sandbox: four scenes, grabbing, orbit camera, scripted capture

## Milestone 2 — Self-contact

- [x] Broad phase: deterministic BVHs over triangles and edges, built from the topology and refit at each detection, with a self traversal for edge pairs. A spatial hash was tried first and rejected on measured cost.
- [x] Offset Geometric Contact (Chen et al., SIGGRAPH 2025): vertex–facet and edge–edge contacts in the paper's offset blocks, its two-stage barrier, and displacement bounds instead of CCD. Bounds are made translation-relative per cloth so moving cloth is not braked ([docs/solver.md](docs/solver.md#self-contact-offset-geometric-contact)).
- [x] Cloth–cloth contact between separate cloths in one world
- [x] Velocity-level Coulomb friction for self contact
- [x] Sandbox: self contact in every scene and a pile of three sheets; benchmark pile scene
- [ ] Contact debug drawing in the sandbox
- [ ] Cheaper dense piles: fewer redetections and parallel detection

## Milestone 3 — Convergence and performance

- [ ] Deterministic threading across a color, with results identical to the serial order
- [ ] SIMD for the membrane and bending kernels, gated on measurement
- [ ] Stretch error: evaluate multilevel or coarse-correction remedies (MGPBD, SIGGRAPH 2025) against more substeps, decided on evidence; XPBD's lower stretch per sweep in `sl_compare` is the bar to beat
- [ ] Revisit initial-guess and acceleration ideas (adaptive guess, Chebyshev, Anderson) under the automatic substep rule, keeping the energy-stability tests

## Milestone 4 — Garment features

- [ ] Dihedral-angle bending for curved rest shapes
- [ ] Anisotropic warp and weft materials
- [ ] Triangle-mesh and SDF colliders; moving-collider velocity in friction
- [ ] Attachments to animated bodies, and seams between cloths

## Later

- WebAssembly build and a browser demo.
- A GPU backend for the color sweeps (WebGPU, Vulkan, or Metal), behind no
  abstraction until a backend exists.
- Coupling with rigid bodies, possibly through Augmented VBD.

## Non-goals

- Third-party dependencies in the core.
- Rendering or platform APIs inside the core.
- Wall-clock time or non-deterministic stepping in simulation results.
- Speculative GPU or rendering abstractions before a concrete requirement exists.
