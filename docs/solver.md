# Solver design notes

This records why silk's cloth solver looks the way it does: the research
choice, what was taken from the papers, and what was changed after measurement.
Code comments point here for the evidence behind a decision. Numbers come from
the scenes in `tests/test_solver.c` and `bench/main.c` (1/60 s steps, light
cotton: 0.2 kg/m², thickness 5 mm), with one Gauss–Seidel sweep per substep
unless stated.

## Choice of method

The rewrite started from a survey of current cloth solvers (October 2026):

| Method | Venue | Fit for silk |
| --- | --- | --- |
| XPBD | 2016 | Mature, robust, but does not converge to the true implicit solution and stalls at high resolution (MGPBD, SIGGRAPH 2025, documents the stall). |
| Projective dynamics | 2014 | Robust large steps; needs a prefactored sparse system and refactoring on topology change. |
| IPC / C-IPC / GIPC | 2020–2024 | Penetration-free guarantees, but offline cost. |
| B-spline cloth FEM | SIGGRAPH 2026 | Accuracy-focused with a global linear solve, not a real-time design. |
| **Vertex Block Descent (VBD)** | SIGGRAPH 2024 | Local, per-vertex Newton steps on the variational implicit-Euler energy; graph-colored Gauss–Seidel parallelises without changing results; the cloth solver in NVIDIA Newton. |
| Offset Geometric Contact (OGC) | SIGGRAPH 2025 | Penetration-free cloth self-contact designed for VBD; over 100× faster than prior guaranteed methods. |
| Augmented VBD | SIGGRAPH 2025 | Extends VBD with hard constraints and rigid bodies. |

VBD is the core, with OGC planned for self-contact. It matches the engine
constraints: purely local and allocation-free per step, deterministic in a fixed
color order, and parallel across a color without changing the result, which
keeps the SIMD, threading, GPU, and WASM paths open.

## Head-to-head comparison (October 2026)

No published study compares current cloth solvers directly. To check the
choice above, `bench/compare.c` (`sl_compare`, built with `SL_BUILD_BENCH`)
runs four solvers on silk's own scenes. Everything except the elastic solve is
shared: mesh, rest data, inertial prediction `y`, drag, pins, colliders,
velocity-level friction, edge damping, and the speed cap.

- **VBD:** silk itself.
- **XPBD:** Macklin et al. (2016, 2019). Edge distance constraints with
  stiffness `k = mu + lambda`, the spring constant whose energy matches StVK
  under uniform stretch on this grid. The same quadratic bending is posed as a
  3-row compliant constraint. Colliders are handled by projection.
- **SIPD-Jacobi:** Luo et al., SIGGRAPH Asia 2026 (arXiv 2609.09834),
  following Algorithm 1 of the paper's source: Jacobi successive substitution
  with pairwise tension tensors, the analytic split into positive and negative
  parts, and a global backtracking line search. The paper only covers
  tetrahedra (its cloth demo is a membrane without bending). silk extends it to
  triangles with `F⁺ = (FᵀF)⁻¹Fᵀ`, which gives, for StVK,
  `PF⁺ = mu·FFᵀ + ½·lambda·tr(C)·Π − (mu + lambda)·Π`, where Π is the
  projector onto the deformed plane. The resulting pair forces match silk's FEM
  gradient to float precision.
- **SIPD-GS:** the same update applied vertex by vertex in silk's color order,
  without line search (the paper's "Our-GS").

Each run uses a fixed substep count, bypassing silk's automatic rule. The four
scenes are free fall for 0.5 s, a curtain hung from two corners for 4 s, a
sheet swinging down from a hinged edge for 3 s, and a 1.2 m sheet dropped on a
sphere for 3 s. All use `mu = lambda = 2e3 N/m`, 0.2 kg/m², drag 0.6, edge
damping 20/s, and 1/60 s steps, single-threaded on an Apple M4 Pro.

In the tables below:

- **Gain** is the peak energy above the initial value, as a fraction of the
  energy the motion releases: 0.98 J for the swing, 0.56 J for the drape. Any
  positive value means the solver created energy.
- **Stretch** is mean edge strain / maximum edge strain off the pins, measured
  on the hanging curtain.

**24 × 24 cells (5 cm edges):**

| solver | substeps × sweeps | ms/step | stretch | swing gain | drape gain |
| --- | --- | ---: | --- | ---: | ---: |
| VBD | 8 × 1 | 0.68 | 0.58% / 16.1% | +131% | +592% |
| XPBD | 8 × 1 | 0.49 | 0.36% / 10.0% | +8% | +680% |
| VBD | 16 × 1 | 1.36 | 0.26% / 8.0% | 0 | 0 |
| XPBD | 16 × 1 | 1.03 | 0.12% / 3.7% | 0 | +91% |
| SIPD-GS | 16 × 1 | 2.09 | 0.31% / 9.3% | 0 | diverged |
| SIPD-Jacobi | 16 × 1 | 2.59 | 0.46% / 33% | diverged | diverged |
| VBD | 32 × 1 | 2.61 | 0.08% / 2.9% | 0 | 0 |
| XPBD | 32 × 1 | 2.24 | 0.04% / 1.2% | 0 | 0 |
| SIPD-GS | 32 × 1 | 4.05 | 0.08% / 2.8% | 0 | 0 |
| VBD | 8 × 4 | 2.05 | 0.27% / 8.3% | 0 | 0 |
| XPBD | 8 × 4 | 1.45 | 0.12% / 3.7% | 0 | +70% |
| SIPD-Jacobi | 16 × 4 | 8.14 | 0.13% / 4.3% | 0 | 0 |

**48 × 48 cells (2.5 cm edges), VBD and XPBD only:**

| solver | substeps × sweeps | ms/step | stretch | swing gain | drape gain |
| --- | --- | ---: | --- | ---: | ---: |
| VBD | 16 × 1 | 5.56 | 0.40% / 20.9% | +184% | +737% |
| XPBD | 16 × 1 | 4.31 | diverged | +890% | diverged |
| VBD | 32 × 1 | 10.52 | 0.30% / 15.7% | 0 | 0 |
| XPBD | 32 × 1 | 8.18 | diverged | +1124% | diverged |
| VBD | 64 × 1 | 21.20 | 0.04% / 2.7% | 0 | 0 |
| XPBD | 64 × 1 | 16.51 | 0.04% / 2.5% | 0 | 0 |
| VBD | 16 × 4 | 16.82 | 0.22% / 12.3% | 0 | 0 |
| XPBD | 16 × 4 | 11.93 | 0.13% / 7.4% | 0 | +17% |

"Diverged" means speeds of 25–100 m/s, the cap, or non-finite state.

### Findings

- **SIPD does not fit silk.** Below about 64 sweeps per frame both variants
  diverged; the Jacobi variant diverged even with the line search. SIPD treats
  the compressive part of the stress explicitly, which makes it only
  conditionally stable in `h`. The paper itself runs 20 iterations at 1 ms
  steps, about 340 sweeps per 1/60 s frame. On this single-threaded CPU it was
  also 1.5–1.9× slower per sweep than VBD. The paper's 1.6× speed-up over VBD
  is a GPU register and occupancy effect that this target does not have.
- **XPBD is cheaper per sweep and stretches less at equal sweeps.** Its sweeps
  cost about 25% less than VBD's, and stretch at the same substep count was
  roughly half. The materials differ slightly: an edge-spring network rather
  than StVK, calibrated only for uniform stretch.
- **VBD stays energy-stable at lower budgets wherever there is contact or fine
  resolution.**
  - In the 24-cell drape, XPBD gained energy at impact up to 16 × 1 and at
    8 × 4. Disabling friction did not change this; projected contacts stretch
    neighbouring edges faster than one sweep resolves.
  - At 48 cells, XPBD with one sweep per substep diverged even in plain free
    fall up to 32 substeps; it stayed still in zero gravity and was fine at
    24 cells. That is the same unconverged-`y` mechanism found for VBD above,
    with no stiffness rule to control it. With hard constraints (zero
    compliance) it diverged too.
  - The cheapest stable configurations favour VBD: 16 × 1 against 32 × 1 at
    24 cells (1.36 ms against 2.24 ms), and 32 × 1 against 64 × 1 at 48 cells
    (10.5 ms against 16.5 ms).
- **Decision: keep VBD.** Robust contact matters most for cloth. VBD is also
  the solver OGC self-contact is designed for, and it keeps a physically
  parameterised StVK material. XPBD's lower stretch per sweep is the reference
  point for the stretch-error work in the roadmap.

### Caveats

This is one material, one machine, and two resolutions. The XPBD and SIPD
implementations are this repository's: SIPD's cloth extension is new, and
XPBD's material calibration is approximate. Nothing here measures the GPU
setting SIPD targets. To reproduce:

```sh
./build/release/bin/sl_compare
./build/release/bin/sl_compare --no-sipd --cells 48
```

## What is implemented

- **Energy.** StVK membrane (Lamé `mu`, `lambda` in N/m) on each triangle's own
  rest metric, and the quadratic isometric bending model of Bergou et al.
  (2006). That paper's `3 / (A0 + A1)` scale converges to three times the plate
  energy on a rolled cylinder. silk uses `1 / (A0 + A1)`, so `bending_stiffness`
  is the plate modulus (`tests/test_energy.c`, cylinder within 3% at 40 cells
  and converging from below).
- **Vertex block.** Inertia plus the per-element gradient and Hessian, with the
  StVK geometric term `wᵀSw` clamped at zero so every block is positive
  definite. Analytic plane, sphere, and capsule colliders add a penalty
  `k/2 · depth²` inside the block.
- **Order.** Greedy coloring per cloth. Sweeps go color by color, in ascending
  vertex order within a color.
- **Drag.** Per-triangle quadratic drag against the wind, applied explicitly
  to the inertial target. Each corner's velocity change is capped so drag can
  never reverse the relative normal velocity.

## Departures from the paper, with evidence

### Initial guess: `y`, not the adaptive guess

VBD's adaptive guess keeps the part of the last measured acceleration that lies
along gravity. On a free-falling 20×20 sheet (4 substeps × 4 sweeps), it fed
elastic vibration back into the next guess. Acceleration spread grew from
7e-4 m/s² to 9 m/s² in 120 substeps, and the sheet tore apart in free fall.

Guesses that avoid extrapolation are stable but cannot move rigid modes,
because Gauss–Seidel cannot translate a stiff sheet. With `x + h·v`, a free
sheet reached 1–4% of free-fall speed after 0.5 s; starting from the previous
position, 0%.

The inertial target `y = x + h·v + h²·g` is exact in free flight and when
sliding rigidly, and `tests/test_solver.c` pins free fall to backward Euler.
Starting contacting vertices from `x + h·v` instead made resting contact
flicker: the ground scene settled with only half its vertices in contact and
5 mm/s jitter.

### Damping: edge stretch impulses, not implicit Rayleigh damping

Rayleigh damping inside the vertex block uses the vertex's own Hessian times
its own displacement, so it damps rigid translation. With `mu = 1e4 N/m` the
sheet could barely fall. A strain-rate form, zero for rigid motion, fixes that,
but it still stiffens every block, so one sweep converges less. On a sheet
swinging from one edge, energy gain grew with the damping coefficient: +0.5 J
undamped, +433 J at 0.005 s, and +1.6 MJ at 0.05 s.

silk instead applies equal and opposite impulses along every edge after the
substep, removing `1 − exp(−damping·h)` of the relative velocity along the
edge. That conserves momentum, leaves rigid spin intact (`tests/test_solver.c`),
and can only remove kinetic energy.

### Friction: velocity level, per vertex

Friction inside the block, as the smoothed lagged-Coulomb term from IPC and
VBD, could not hold a sheet on a 20° slope at μ = 0.8. Every neighbour starts at
`y`, already slipped, so stopping one vertex fights the stretched membrane
rather than that vertex's own load. A closed-form Coulomb projection in the
block failed for the same reason.

silk applies Coulomb friction to each contacting vertex's tangential velocity
after the substep, using the penalty force at its final penetration as the
normal load. The slope test holds at μ = 0.8 and slides 1.22 m in 1 s at μ = 0.1
(theory: 1.2 m).

Re-deriving positions from the corrected velocity stopped all creep but pumped
energy into a sphere drape (1.41 J → 2.08 J; still 1.81 J when limited to
stuck vertices). Positions are therefore left as solved. Held cloth creeps by
at most `h·|g|·sin(slope)` per second of contact, about 3.5 mm/s on 20° at
16 substeps.

### Automatic substeps for stability

One sweep that starts at `y` acts explicitly on the mid-frequency stretch
modes it cannot remove. Stability therefore follows a CFL-like ratio.
`sigma` is each vertex's rest elastic stiffness (the trace of its rest Hessian
block) over `3m`, which is `8·mu / (density·edge²)` on a regular grid with
`lambda = mu`. Sphere drapes from 24 to 64 cells:

| `mu` (N/m) | first energy gain at `h²·sigma` | stable at |
| --- | --- | --- |
| 2e3 | 43 (+45%) | 39 |
| 1e4 | 30 (+6%) | 23 |

Heavy edge damping, up to 1e5/s, did not move the threshold, and neither did
a per-vertex line search: the gain is not a local overshoot.
`SL_STIFFNESS_STEP_MAX = 20` keeps a margin below the stiffer case.
`sl_world_step` raises the configured minimum substep count to satisfy it,
up to `SL_SUBSTEP_COUNT_MAX`, and reports the count used and whether the cap
was hit.

At equal cost, small steps beat iterations for stretch error. On a 20×20
sheet hung from two corners for 4 s (`mu = 2e3 N/m`, automatic substeps
bypassed):

| substeps × sweeps | mean edge strain | max strain off the pins |
| --- | --- | --- |
| 1 × 16 | 2.19% | 40.0% |
| 4 × 4 | 0.67% | 16.6% |
| 16 × 1 | 0.14% | 4.4% |
| 22 × 1 (the automatic count) | 0.09% | 2.9% |
| 64 × 1 | 0.02% | 0.6% |

### No Chebyshev acceleration

At two sweeps per substep and `rho = 0.9`, Chebyshev's second weight
`2 / (2 − rho²) ≈ 1.68` diverged (velocities hit the speed cap). With the
small-steps default of one sweep, it never runs. It was removed rather than
shipped as a parameter that can explode.

## Self contact: Offset Geometric Contact

silk implements Offset Geometric Contact (OGC; Chen et al., SIGGRAPH 2025)
from the paper's text: `src/self_contact.c`, `src/contact.c`, `src/bvh.c`.

### Taken from the paper

- **Contact sets.** Vertex–facet contacts come from the face set `F_OGC`
  (Eq. 12). Edge–edge contacts come from `E_OGC` on the edge-only wireframe
  (Eq. 17). Every contact passes the constructive vertex and edge block
  tests (Eqs. 8, 9, 15).
- **Contact geometry.** Each contact pushes along the separation from the
  closest point to that point. Its feature (vertex, edge, or face interior)
  is fixed at detection. Vertex–facet contacts push both sides. Each
  edge–edge contact pushes only its owning edge, which is how the paper
  treats a pair seen from both sides.
- **Activation.** The two-stage function is quadratic above `τ = r/2` and
  logarithmic below (Eq. 18). The paper prints the log-stage coefficient as
  `τ·k(τ−r)²`; C¹ and C² continuity at τ both require
  `k′ = τ·k(r−τ) = k·τ²`, which `tests/test_contact.c` pins.
- **Bounds.** Conservative displacement bounds use `γ_p = 0.45` (Eq. 21),
  and truncation replaces CCD. Contacts are re-detected once `γ_e = 1%` of
  the vertices are held by their bounds. The paper counts vertices, so
  silk counts distinct vertices, not truncation events.

### Departures, with evidence

- **Translation-relative bounds.** With the paper's absolute bounds, a
  falling sheet used up its bounds within a few substeps: non-adjacent
  primitives of one sheet sit half an edge apart. At one sweep per substep,
  each truncation erased the substep's motion, so the sheet reached
  0.4 m/s after 1 s of free fall instead of 9.8 m/s. The paper names this
  as its limitation (Fig. 22b) and suggests using displacement directions.
  silk bounds each vertex's deviation from its cloth's common translation
  `T_c`. Same-cloth distances do not change under a common translation, so
  every same-cloth pair still moves less than `2b < d`, which is the
  paper's condition. Across cloths, `T_c` is capped so absolute
  displacements stay within a cloth-level bound `B_c`. `B_c` comes from
  inter-cloth distances found inside the query, or from the bounding-box
  gap between cloths when that is larger. Free fall with self contact on
  now matches it off exactly (9.893 m/s both).
- **Detection once per step, not per substep.** The query widens by the
  step's relative motion, `dt·|v − mean(v)|`. The bounds keep the 16–40
  substeps intersection-free, and the 1% rule re-detects as needed:
  1–4 detections per step in the scenes below.
- **Edge pairs push both edges.** OGC stages each edge–edge contact once
  from each edge, and silk first pushed only the detecting edge. Each side
  passes its own feasibility test, though (a vertex-block test when its
  closest point is an endpoint), so one side can be rejected. The other
  edge then felt no reaction, and the forces were no longer the gradient of
  any energy. The sandbox curtain, gathered on kinematic rings, gained
  energy until vertices hit the 100 m/s speed cap and the sheet flew apart
  (472 ms/step, tens of thousands of pairs dropped for capacity). Every
  pair now pushes its whole stencil, and edge pairs carry weight 1/2, so a
  pair accepted from both edges acts exactly as before. The same curtain
  stays at rest after the gather (34 ms/step while still flat, see Known
  limits).
- **Velocity-level friction.** Friction uses the same velocity-level
  Coulomb step as the colliders (see Friction above). Each pair's
  tangential relative velocity is reduced by at most `μ·λ·h·W`, with `λ` the
  barrier force and `W` the stencil's effective inverse mass. That only
    removes kinetic energy and conserves momentum for every pair.
- **Broad phase.** A spatial hash first scanned 130 to 590 edges per
  bucket in a two-layer drape, because its cells must exceed the longest
  edge plus the query. It is replaced by two BVHs, over triangles and
  edges. Each is built once from the topology (deterministic median split)
  and refit at every detection. Edge pairs come from one self traversal, so
  each unordered pair is visited exactly once.
- **Query cap.** The query's motion term is capped at 0.05 triangle
  extents. Three-sheet pile cost by cap: 82 ms/step at 0.25, 61 at 0.1, 50
  at 0.05, 51 at 0. All runs stayed intersection-free.
- **Flat sheets.** Vertex contacts never act within a flat sheet, at any
  radius (`tests/test_contact.c`). Edge–edge contacts live on the
  wireframe, so the radius must stay below the spacing of non-adjacent
  edges, about 0.7 of an edge on a regular grid.

### Results

A brute-force check of every edge against every non-incident triangle found
no crossings with self contact on, in any scene and at any checked step:
stacked sheets on a sphere, a sheet folded onto itself by a pinned edge,
and drapes. With it off, the same scenes had 225–2763 crossings.

Cost per 1/60 s step, single-threaded on an Apple M4 Pro, release build:

| Scene | without | with self contact | detections/step |
| --- | ---: | ---: | ---: |
| 32×32 sheet in free fall | 4.8 ms | 6.0 ms | 1.0 |
| 40×40 sheet draped on a sphere | 6.9 ms | 12.2 ms | 2.8 |
| two 32×32 sheets on a sphere | 12.2 ms | 27.7 ms | 3.8 |
| three 30×30 sheets piled on a sphere | 11.6 ms | 53.5 ms | 7 |

The pile cost rose from 48.6 ms when edge pairs began pushing both edges.

### Known limits

- Pinned vertices are kinematic and unbounded. A target that drags cloth
  through cloth is not prevented.
- An exactly planar sheet compressed within its plane (a flat curtain
  gathered on rings, with nothing to push it out of plane) crumples inside
  the plane. Coplanar triangles then overlap at distance 0, the vertices
  there get zero bounds and freeze, and detection runs every substep. A
  1 cm out-of-plane pleat removes it: 8.8 ms/step with one detection per
  step instead of 34 ms with 27.
- Vertices held at their bounds wait for the next detection. In dense
  piles this shows as brief sticking.
- There is no smoothing of nearly parallel edge–edge contacts (IPC's
  mollifier).
- The force jumps where a point enters a block through an open boundary on
  a concave side, as the paper's own limitation notes.

## Threading

A world given a `parallel_for` callback (`sl_world_config`) runs its
independent work through it. silk never creates threads: the host supplies
the callback, and `examples/pool` is a small pool for the benchmark and
sandbox. Portable C17 has no thread API the core may use (POSIX is excluded,
and macOS has no `<threads.h>`), which is also how Box2D and Box3D handle
it. The contract: results are bitwise identical for any partition, order,
and thread count, including running everything on the calling thread.

### What runs in parallel

- **The sweep, one color at a time.** The mesh coloring separates vertices
  that share a triangle or bending stencil, so without self contact a block
  reads no vertex its color writes, and a color can be solved concurrently
  in place with exactly the serial result. Contact-free hashes did not
  change.
- **The self-contact sweep, buffered per color.** A contact pair can join
  two vertices of the same color, which the mesh coloring cannot prevent.
  In place, one block would read a partner while another thread writes it.
  Each color is solved into a buffer and committed afterwards, so
  same-color contact partners see start-of-color positions (a Jacobi step
  within the color) while mesh neighbours see the latest positions as
  before. This changed self-contact results once, by design. A test folds
  one sheet onto itself so same-color pairs exist; solving in place makes
  its scrambled run differ.

- **Drag, gathered per vertex.** Drag scatters each triangle's force to its
  corners. With threads, each vertex sums its incident triangles in
  ascending order instead, receiving the same additions in the same order,
  so results are identical. That evaluates each triangle three times (10%
  slower on one thread), so the serial path keeps the loop over triangles.
- **Per-vertex substep loops:** the inertial targets, initial guess,
  velocities, collider friction, and speed clamp.

Still serial: edge damping (below), self-contact detection, bounds and
contact friction, and the detection-time BVH work. In the pile scene,
detection is about 70% of the step, so threading gains little there (issue
#117).

### Edge damping stays serial

Damping applies impulses edge by edge in ascending order, each seeing the
velocities earlier edges changed. Edge coloring (edges of one color share no
vertex) would let a color run in parallel, and it was tried: greedy coloring
at cloth add, at most 65 colors, 7 on a grid. It was reverted.

- **Effect:** in color order, damping got much weaker. Consecutive edges in
  ascending order share vertices, so one pass carries the damping along
  chains across the sheet and suppresses long waves; in color order a pass
  reaches only a few edges. Each impulse still removed energy, but far less
  overall.
- **Measured:** the stiff 1.2 m drape onto a sphere (`sphere_drape_outside`,
  μ = 10⁴ N/m) then gained kinetic energy from unconverged substeps faster
  than damping removed it. Its hanging skirt flapped at up to 24 m/s instead
  of settling under 1.5 m/s.
- **How it slipped through:** the test's energy bound still passed (the drop
  releases more potential energy than the skirt gained), and its final
  contact count failed only on x86-64, where the arithmetic differs from
  arm64's fused multiply-adds. The test now also bounds the skirt's speed
  over the last second.

Parallel coloring had looked like the biggest win (curtain 3.5× on 8
threads, against 2.5× without it), and it made the single-threaded path
faster too. Damping that keeps its strength under a parallel order needs a
different formulation, which is future work.

### Granularity

The engine passes a grain hint: 32 vertices per range for block solves
(about 5 µs), and 2048 vertices for plain loops. On
the 64×64 curtain, a grain of 128 left most of 8 threads idle (1.85×); 16 to
64 performed alike.

Each step issues hundreds of short dispatches (colors × substeps), so the
pool's dispatch latency matters:

- Workers that slept on a condition variable between dispatches spent most
  of their time waking up.
- Workers now spin for about a quarter of a millisecond, longer than the
  serial work between dispatches (drag and damping took about 100 µs per
  substep on the 64×64 curtain before drag was parallel).
- The pool's first version also made the caller wait for every worker to
  check in after each dispatch. One descheduled thread then stalled every
  color, and runs varied from 1.07× to 2.19×. Claims now carry the dispatch
  generation, so the caller waits only for its own ranges.

### Results

Best of three runs, Apple M4 Pro (8 performance and 4 efficiency cores),
release build, ms per 1/60 s step:

| Scene | 1 thread | 4 threads | 8 threads |
| --- | ---: | ---: | ---: |
| 32×32 curtain | 5.0 | 2.6 (1.9×) | 2.8 (1.8×) |
| 64×64 curtain | 40.6 | 18.5 (2.2×) | 16.1 (2.5×) |
| 64×64 drape | 36.0 | 16.1 (2.2×) | 13.9 (2.6×) |
| 48×48 flag | 15.1 | 7.3 (2.1×) | 6.3 (2.4×) |
| three 30×30 sheets (self contact) | 57.8 | 49.2 (1.2×) | 49.9 (1.2×) |

Single-threaded times and contact-free hashes are unchanged from before
threading. The threaded hash matches the serial one in every scene and at
every thread count tried (2–12). Beyond the performance cores, speed falls:
the 64×64 curtain runs at 1.85× on 10 threads and 1.7× on 12, because every
color waits for its slowest range and efficiency cores are slower. Use at
most the number of performance cores.

`tests/` checks the contract without threads: a `parallel_for` that cuts
ranges unevenly and runs them back to front must reproduce the serial hash,
both without self contact and with a self-folded sheet. CI also runs the
benchmark on four threads against the serial hashes, and ThreadSanitizer
found no races in the pool.

## Known limits

- **Stretch error.** One sweep per substep leaves long-wavelength stretch
  error. The hanging sheet above stretches 2.9% off the pins at the automatic
  22 substeps, against 0.6% at 64. More substeps or iterations reduce it.
  Multigrid (MGPBD-style) or global coarse corrections are the known remedies.
- **Colliders.** Colliders are analytic only. Self and cloth-cloth
  contact are covered above.
- **Moving colliders.** A collider is static within a step, so friction
  ignores its velocity.
- **Bending rest shape.** Bending assumes a flat rest surface.
- **Precision.** Velocity comes from float position differences: about
  `ulp(|x|)/h` of noise per substep.
