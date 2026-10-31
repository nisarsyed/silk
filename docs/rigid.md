# Rigid-body solver research

This records the research behind silk's rigid-body solver choice (Milestone 4),
done before any rigid-body code exists. It surveys Augmented Vertex Block
Descent (AVBD), the lead candidate in the roadmap, and its alternatives, and
separates what papers claim from independent evidence. Its conclusion is an
experiment, not a choice: the decision waits on the head-to-head below.

Sources were read in November 2026. Claims marked *verified* were checked
against the primary source (paper text, code, or the issue or post itself).

## Requirements

What a rigid-body solver must do in silk, from the [roadmap](../ROADMAP.md)
and the engineering baseline:

- Stable stacks and piles, chains, joints with limits and motors, and Coulomb
  friction, in real time on one laptop-class CPU (Apple M4 Pro reference).
- Bitwise determinism per platform and build, no allocation per step, and
  bounded work.
- Physical parameters a user can set without tuning per scene: the same
  constants across densities, sizes, and units.
- Two-way coupling with the VBD cloth (and later soft bodies and rods),
  ideally in the same solve. The roadmap's definition of done for a module is
  that it couples.

## Candidates

### AVBD (Giles, Diaz, Yuksel, SIGGRAPH 2025)

VBD's block Gauss–Seidel with a 6×6 block per rigid body (position and
rotation increment, LDLᵀ solve), plus an augmented Lagrangian for hard
constraints: per-constraint stiffness `k` ramped by `k ← k + β|C|`, a dual
update `λ ← kC + λ` after each sweep, Baumgarte-like stabilization `α`, and
warm starting of `k` and `λ` across steps through `γ` (paper Eqs. 8–19).
Inequalities clamp `λ`. Contacts are 3-row constraints with a Coulomb cone;
static friction freezes the contact anchors. The block Hessian replaces the
geometric-stiffness term with a diagonal approximation so it stays SPD.

**Claims** (paper, *verified*):

- Paper defaults `β = 10`, `α = 0.95`, `γ = 0.99`, with "similar results for
  any β ∈ [1, 1000]" (p. 4).
- Table 1, RTX 4090, 110k blocks: AVBD 4 iterations 3.5 ms; sequential
  impulse 15 iterations 14.1 ms; XPBD 6 substeps 19.5 ms; VBD 15 iterations
  10.8 ms. 510k blocks: 10.3, 78.4, 524.7, and 26.5 ms. The paper states that
  "the performance differences between methods is mainly due to the number of
  iterations used": per iteration, AVBD costs about the same as the others.
  Budgets are set per method to stop piles collapsing, not equal time.
- A 50-link chain holding a 50,000:1 mass, and two-way coupling with cloth:
  a flag on a jointed pole (Fig. 5) and 35k jointed bodies falling on a
  10k-vertex cloth in 16 ms (Fig. 14).

**What the claims do not cover:**

- All timings are on a GPU. There are no CPU timings.
- The "sequential impulse" baseline is Catto's 2006 projected Gauss–Seidel,
  not Soft Step or TGS, which are what modern engines ship.
- The cloth in the coupling scenes is mass-spring, without self contact.
  No coupling code is public.
- There are no energy, penetration, or friction metrics.

**Reference code** (`savant117/avbd-demo2d`, `avbd-demo3d`, MIT, *verified*):

- Single-threaded, sequential Gauss–Seidel.
- Box shapes only, an O(n²) broad phase, and contact manifolds allocated
  with `new` every step.
- The 3D demo ships `betaLin = 1e4`, `betaAng = 100`, `α = 0.99`, `γ = 0.999`,
  outside the paper's tested β range. Its comment says the best β "will depend
  on the length, mass, and constraint function scales".
- The 3D angular mass block uses the body-frame `diagonal(moment)` where the
  paper specifies the rotated inertia (`solver.cpp:202`). This looks wrong for
  non-cubic bodies.
- Joints are ball joints and an angular lock. There are no limits or motors.

**Independent evidence:**

- *Verified:* the parameters do not transfer across densities or units.
  avbd-demo2d #11 (open) reports that two or three stacked boxes go unstable
  at density 1000. Giles replied that "in the original paper we used the same
  density and units for all our scenes, and tuned the parameters for those",
  that the remedy is scaling β or adding iterations, and that a
  reformulation is in progress.
- *Verified:* Joona Aalto (Avian physics engine, October 2025) wrote up a few
  weeks of hands-on testing, qualitative only.
  - Positive: joints are "really impressive", with little stretching, and
    vertex coloring parallelizes better than the constraint coloring of dual
    solvers.
  - Negative: "the parameters are really fiddly and scene-dependent"; "the
    body solve order has a significant impact on results"; wide SIMD is
    harder for the primal update; the stiffness is mass-dependent rather
    than a frequency and damping ratio.
  - Contact: "for contact scenarios, I was struggling to get results that
    were even close to being as stable as our existing TGS_Soft".
- *Verified:* NVIDIA Newton ships AVBD rigid bodies in its experimental
  `SolverVBD`, coupled with VBD cloth and soft bodies. In August 2026 it
  added an opt-in "compliant ALM" (PR #3769, issue #3333) and is
  deprecating the legacy AVBD hard/soft path.
  - Each constraint keeps its authored physical stiffness, and the solver
    picks a separate numerical penalty and a persistent multiplier, with
    projected Coulomb friction.
  - This keeps AVBD's structure and fixes its parameter problem by
    separating physical authoring from numerical enforcement.
- Projective Affine Body Dynamics (Eurographics 2026) is the only third-party
  paper comparing against AVBD. It compares convergence, not time: AVBD's
  Newton blocks converge in fewer iterations, and the two settle similarly.
- No engine author has published a wall-clock comparison of AVBD against a
  modern impulse solver. Jolt's author has not evaluated it, and Rapier has
  not planned it. Box3D (June 2026) and Box2D 3.2 (October 2026) keep Soft
  Step.

### Soft Step (TGS-soft; Box2D v3, Box3D)

Sequential impulses at velocity level with soft constraints, set by a
frequency and damping ratio. Each substep integrates velocities, warm
starts, solves with a positional bias, integrates positions, then relaxes
without bias. Restitution is applied last.

- Catto's Solver2D study compared eight Gauss–Seidel solvers on the same
  scenes and budget, and chose it. He found that "smaller time steps are more
  effective than more iterations".
- **Box3D** (June 2026, MIT, C17) is a 3D reference implementation. It uses
  graph coloring with an overflow set, a wide-SIMD contact solver, and 4
  substeps by default.
- *Verified:* its determinism is cross-platform and independent of thread
  count, with floating-point contraction disabled.
- *Verified:* its documented limits are that "extreme mass ratios between
  connected bodies can cause joint stretching" and "long joint chains will
  stretch less with more sub-steps".
- silk's 2D engine used it, with warm starting, a relax pass and four
  substeps, so the team knows it in practice.
- **Weakness for silk:** it works on velocities and impulses, while VBD cloth
  works on positions and energy. Two-way coupling with cloth would be
  staggered or go through proxies, not one solve. Jolt couples its soft
  bodies the same way, through a separate pass.

### Others, and why not

- **XPBD for rigid bodies** (Müller et al. 2020): 20–40 substeps. Its own
  paper says tall stacks can go unstable. Catto found friction and
  far-from-origin problems, and AVBD's Table 1 found piles collapsing at 26
  substeps. It brings no coupling advantage over AVBD. At most a baseline.
- **PhysX TGS and Jolt:** shipped impulse solvers in the same family as Soft
  Step, with no advantage over it for silk.
- **Rigid IPC and Affine Body Dynamics:** penetration-free by construction
  but far from real time on a CPU. Rigid IPC reports itself two to three
  orders of magnitude slower than game solvers.
- **MuJoCo:** soft convex contact in generalized coordinates with a global
  factorization. Excellent for robots, but it does not scale to large piles
  or share a solve with cloth.

## Assessment

| | AVBD family | Soft Step |
| --- | --- | --- |
| Contact stacks and piles | Unproven on CPU; one hands-on report finds it less stable than Soft Step | Proven in shipped engines |
| Joints, chains, mass ratios | Strongest claim; confirmed for joints by an independent tester | Documented weakness, eased by substeps |
| Parameters | Paper constants are tuned to one set of units (author-confirmed); compliant ALM addresses it, so far only in Newton | Frequency and damping ratio, independent of units |
| CPU cost | Unmeasured; per-iteration cost comparable to impulses on GPU | Known, and SIMD-friendly |
| Determinism | Achievable with deterministic coloring; results depend on solve order | Proven cross-platform |
| Coupling with VBD cloth | Same solve: rigid 6×6 blocks next to cloth 3×3 blocks | Staggered only |
| Fit with silk's code | Reuses the VBD loop, coloring, contact, and substep rule | A second, separate solver |

Neither wins on the evidence. AVBD is the only method that fits the "one
solve" vision, and its joint and mass-ratio claims have independent support.
But its contact stability, CPU cost, and parameter robustness are exactly
where the independent evidence is weak or negative, and its published
comparisons are against an outdated baseline. Soft Step is proven for
contact, deterministic, and familiar, but it would make silk two solvers
coupled by staggering.

The roadmap's AVBD lead holds only with a refinement: use the compliant ALM
formulation (physical stiffness separate from the numerical penalty), not the
paper's raw β ramp. Even then, it must beat Soft Step on contact to be chosen.

## Proposed experiment

Settle it by measurement on silk's own terms, as the cloth solver was settled
by `sl_compare`. Build a prototype harness with two solvers sharing one
collision pipeline:

- **A. VBD-family rigid bodies:** 6×6 blocks in silk's colored Gauss–Seidel,
  with the compliant ALM formulation. The paper's β ramp is a variant of the
  same code.
- **B. Soft Step,** following Box3D's stage structure (4 substeps, warm
  starting, relax pass).

Compare at **equal wall-clock time per step on the M4 Pro CPU**, not equal
iterations. Use one set of constants for every scene; per-scene tuning counts
as a failure.

| Scene | Measures |
| --- | --- |
| 30-box stack, and a 1,000–5,000-box pyramid | Drift, penetration, settling, jitter at rest |
| The same stack at densities 1, 1000, and 10⁵ | Parameter robustness (the reported AVBD failure) |
| 50-link chain holding a 50,000:1 mass | Stretch (AVBD's strongest claim) |
| Box on a slope at the friction threshold | Stick and slide against μ |
| Boxes resting on, and dragged across, a silk cloth | Two-way coupling quality (B staggered) |
| The stack offset 10 km from the origin | Precision far from the origin |
| Each scene with reversed body order | Order sensitivity |

**Decision rule.**

- Choose A if it matches B on the stack, pile, and friction scenes at equal
  time, passes the density sweep with one set of constants, and keeps its
  lead on the chain and coupling scenes.
- Otherwise use B for rigid bodies, couple them to cloth by staggering, and
  revisit A when compliant ALM matures.

Either way, the harness needs convex contact generation (box first), contact
persistence across steps, and a broad phase over bodies. Those are needed by
whichever solver wins, so they are built first, as the shared part of
Milestone 4.

## Sources

- AVBD paper and project page:
  <https://graphics.cs.utah.edu/research/projects/avbd/>
- Reference code: <https://github.com/savant117/avbd-demo2d>,
  <https://github.com/savant117/avbd-demo3d>
- Density and units issue:
  <https://github.com/savant117/avbd-demo2d/issues/11>
- Avian 0.4 post (AVBD evaluation): <https://joonaa.dev/blog/09/avian-0-4>
- Newton compliant ALM: <https://github.com/newton-physics/newton/issues/3333>,
  <https://github.com/newton-physics/newton/pull/3769>
- Projective Affine Body Dynamics: <https://arxiv.org/abs/2609.02675>
- Solver2D: <https://box2d.org/posts/2024/02/solver2d/>
- Box3D: <https://box2d.org/posts/2026/06/announcing-box3d/>,
  <https://github.com/erincatto/box3d> (`docs/faq.md`, `docs/loose_ends.md`,
  `docs/simulation.md`)
- XPBD rigid bodies:
  <https://matthias-research.github.io/pages/publications/PBDBodies.pdf>
- Rigid IPC: <https://ipc-sim.github.io/rigid-ipc/>; ABD:
  <https://arxiv.org/abs/2201.10022>
- MuJoCo computation:
  <https://mujoco.readthedocs.io/en/stable/computation/index.html>
