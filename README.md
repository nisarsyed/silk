# silk

A portable, data-oriented 3D cloth simulation engine written in C. The core is
renderer-independent, deterministic, and has no third-party dependencies.

silk simulates triangle-mesh cloth with **Vertex Block Descent** (VBD; Chen et
al., SIGGRAPH 2024). VBD solves each implicit-Euler step as an energy
minimization through per-vertex Newton steps, run color by color, so a color
can be split across workers without changing the result. See
[docs/solver.md](docs/solver.md) for how the method was chosen, the departures
from the paper, and the measurements behind them.

silk provides:

- 3D vectors and 3×3 matrices with a guarded linear solve.
- Cloth from any manifold triangle mesh: StVK membrane, quadratic isometric
  bending calibrated to the plate modulus, and lumped mass from areal density.
- Pinned (kinematic) vertices with targets that move smoothly across substeps,
  for grabbing, attaching, and animating.
- Analytic plane, sphere, and capsule colliders with penalty contact and
  Coulomb friction.
- Penetration-free self contact and cloth-cloth contact using Offset
  Geometric Contact (Chen et al., SIGGRAPH 2025), without continuous
  collision detection. Turn it on with `self_contact_capacity`.
- Gravity, wind, and quadratic aerodynamic drag, plus edge stretch damping
  that never damps rigid motion.
- Automatic substepping from each cloth's stiffness, with the substep count
  reported in the world stats.
- A bounded world allocated once at initialization; stepping never allocates.
- A deterministic fixed-step API and a bounded accumulator helper.
- A raylib 3D sandbox, a deterministic benchmark, and a headless example.

See [ROADMAP.md](ROADMAP.md) for direction and next milestones.

## Version 0.1.0

0.1.0 is the first development version of silk as a 3D cloth engine. It was
rewritten from scratch over an earlier 2D rigid-body engine that reached
0.4.0 in git history but was never released, so numbering restarts. Nothing
from the 2D API remains. Until 1.0, the API may change between versions.

## Requirements

- CMake 3.28 or newer.
- A C17 compiler.
- Git and network access, only to configure the optional sandbox, which
  fetches raylib 6.0.

## Build and test

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

The `release`, `sanitize`, and `windows` presets use the same three commands.
Executables are written to `build/<preset>/bin`; the test binary can also run
directly as `./build/debug/bin/sl_tests`.

| Option | Default | Purpose |
| --- | --- | --- |
| `SL_BUILD_TESTS` | On at the top level; off as a subproject | Build and register `sl_tests` |
| `SL_SANITIZERS` | `OFF` | Enable AddressSanitizer and UndefinedBehaviorSanitizer |
| `SL_SANDBOX` | `OFF` | Fetch raylib and build the interactive sandbox |
| `SL_BUILD_EXAMPLE` | `OFF` | Build the public headless example |
| `SL_INSTALL` | On at the top level only | Install the static library, headers, and CMake package |
| `SL_BUILD_BENCH` | `OFF` | Build `sl_bench`, kept separate from CTest |

## Interactive sandbox

```sh
cmake --preset sandbox-release
cmake --build --preset sandbox-release
./build/sandbox-release/bin/sandbox
```

On Linux, raylib also needs the OpenGL and window-system development packages.

The sandbox has five scenes, all with self contact on: a pleated curtain
gathered on rings, a sheet draped over a sphere, a flag on a pole, a towel on a rail, and
a pile of three sheets dropped on a sphere.

Controls:

- `1`–`5`: switch scene.
- `Space`: pause or resume. `S`: single-step while paused. `R`: reset.
- `W`: toggle wind (magenta arrow).
- `C`: toggle self contact (reloads the scene).
- Left-drag: grab and pull the cloth. Releasing it throws it.
- Right-drag: orbit. Wheel: zoom.

The HUD shows the substep count the stiffness rule chose, the milliseconds
per step, and the self-contact pairs and detections per step. The sandbox
runs at most one 1/60 s step per rendered frame, so a scene that costs more
than real time (the pile, about 60 ms per step) plays in slow motion,
flagged in the HUD, while the window stays responsive. Use the
`sandbox-release` build: the debug build is 10–17 times slower.

The sandbox solves on four threads by default. `--threads N` changes that;
use at most the number of performance cores (8 on an M4 Pro).

For a scripted capture, `sandbox --scene 2 --frames 150 --screenshot drape.png`
runs 150 fixed steps, saves the frame, and exits.

## Deterministic benchmark

```sh
cmake --preset release -DSL_BUILD_BENCH=ON
cmake --build --preset release
./build/release/bin/sl_bench
```

`sl_bench` runs five scenes: a 32×32 and a 64×64 curtain, a 64×64 sphere
drape, a 48×48 flag, and a pile of three 30×30 sheets with self contact. For each it reports milliseconds per step,
nanoseconds per vertex-sweep, contacts, and a hash of the final state. Every
scene runs twice, and the process exits nonzero if the hashes differ. With
`--threads N`, each scene also runs on an N-thread pool, which must reproduce
the serial hash. Use `--scene`, `--warmup`, and `--steps` to choose a smaller
workload.

On an Apple M4 Pro, a vertex-sweep costs about 130 ns on one thread.
Substeps scale with resolution under the stability rule, so cost grows
faster than the vertex count:

| Scene | Substeps | 1 thread | 8 threads |
| --- | --- | --- | --- |
| 32×32 curtain | 34 | 5.0 ms | 2.8 ms |
| 64×64 curtain | 68 | 40.6 ms | 16.1 ms |

Edge damping and self-contact detection are still single-threaded, so the
pile gains little from threads. Details are in
[docs/solver.md](docs/solver.md#threading).

The same build also produces `sl_compare`, which runs VBD, XPBD, and SIPD
head to head on shared scenes. Results and the decision they support are in
[docs/solver.md](docs/solver.md#head-to-head-comparison-october-2026).

## Use from C

```c
sl_world world = { 0 };
sl_world_config config = { 0 };
config.vertex_capacity = 4096;
config.triangle_capacity = 8192;
config.cloth_capacity = 1;
config.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
sl_world_init(&world, &config);

sl_cloth_desc cloth = { 0 };
cloth.positions = positions;   /* your mesh, meters */
cloth.vertex_count = vertex_count;
cloth.indices = indices;
cloth.triangle_count = triangle_count;
cloth.density = 0.2f;          /* kg/m^2 */
cloth.membrane_mu = 2000.0f;   /* N/m */
cloth.membrane_lambda = 2000.0f;
cloth.bending_stiffness = 1e-4f;
cloth.damping = 20.0f;         /* 1/s */
cloth.thickness = 0.005f;      /* m */
uint32_t id;
sl_world_cloth_add(&world, &cloth, &id);
sl_world_vertex_pin_set(&world, id, 0, true);

sl_stepper stepper;
sl_stepper_init(&stepper, 1.0f / 60.0f);
sl_world_advance(&world, &stepper, frame_seconds);
const sl_vec3 *p = sl_world_cloth_positions(&world, id);
```

`examples/headless/main.c` is a complete program. Build it with
`-DSL_BUILD_EXAMPLE=ON`.

To use several threads, give the world a `parallel_for` callback that runs
a task over ranges of an index space on your own threads (see
`sl_parallel_for_fn` in `silk/world.h`). silk never creates threads, and
results are bitwise identical however the work is split.
`examples/pool/pool.c` is a small pool implementing it:

```c
sp_pool *pool = sp_pool_create(8);
config.parallel_for = sp_pool_parallel_for;
config.parallel_context = pool;
```

An existing CMake project can embed silk without enabling its tests,
examples, or install rules:

```cmake
add_subdirectory(path/to/silk)
target_link_libraries(my_app PRIVATE silk::silk)
```

To use an installed, relocatable static package:

```sh
cmake --preset release -DSL_INSTALL=ON
cmake --build --preset release
cmake --install build/release --prefix "$PWD/build/silk-prefix"
```

```cmake
find_package(silk 0.1.0 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE silk::silk)
```

The package matches versions exactly. `tools/check_consumer.py` checks the
embedded, installed, and relocated consumers.

## Current limits

- **Self-contact cost.** Self contact costs 1.3× to 4× the contact-free
  step, rising with how many layers touch: three piled sheets take about
  50 ms per step. Pinned vertices are moved by the caller and are not
  bounded, so a pin dragged through cloth can still cross it. See
  [docs/solver.md](docs/solver.md#self-contact-offset-geometric-contact).
- **Analytic colliders only.** Colliders are planes, spheres, and capsules,
  treated as static within a step.
- **Stretch error.** One sweep per substep leaves some long-wavelength stretch,
  about 3% on a hanging sheet at the automatic substep count. Raise
  `substep_count` or `iteration_count` to reduce it.
- **Flat rest bending.** Bending assumes the rest surface is flat.
- **Single-threaded scalar code.** The color ordering keeps threading and SIMD
  open without changing results.
