# Reproducible benchmark matrix

Configure with `cmake --preset release -DSL_BUILD_BENCH=ON`, then
`cmake --build --preset release`. Run `build/release/bin/sl_bench_check` to
verify the independent quality geometry. The benchmark and its check executable
use only public Silk headers and remain separate from CTest.

```sh
# Full matrix with 120 warm-up / 600 measured steps:
build/release/bin/sl_bench
# Full matrix, five independent runs, schema/replay/physical-quality checks:
python3 bench/report.py run build/release/bin/sl_bench --output build/reports --quality
# Short valid-output/replay check, without settled-quality acceptance:
python3 bench/report.py run build/release/bin/sl_bench --output build/smoke --repeat 2 --warmup 2 --steps 8
python3 bench/report.py validate build/reports/run-1.json --quality
python3 bench/report.py compare build/reports/run-1.json build/reports/run-2.json
```

Windows multi-config executables are under `build/windows/bin/Debug/` and end
in `.exe`. No Python package is required. `--scene` accepts `all` (the default) or one fixture below.
`--warmup` accepts 0–10000 and `--steps` 1–10000. Output is always JSON.
Unknown/duplicate options, missing values, signs, trailing characters, overflowing
counts and extra arguments fail. Report repetitions are bounded to 1–20 (default
5), each child process has a 600-second timeout, and input reports are limited
to 1 MiB. The runner preserves raw stdout/stderr when validation fails.

## Committed fixtures (version 1)

Every fixture uses dt = binary32(1/60), four substeps, zero drag and sleeping
disabled. Solver/speed settings resolve to the public defaults and appear in
JSON. All new fixtures use seed `0x59B3AC01` as their identity; construction is
analytic and uses no random draws. The original rain xorshift seed is unchanged.
Box dimensions below are half extents. All new bodies have friction 0.6 and
restitution 0; static mass is zero, dynamic mass is 1 unless stated otherwise.

| Fixture | Construction | Body/contact/joint capacity |
|---|---|---|
| pyramid | Original 20-row unit-box pyramid, spacing 1.01, ground (12, 0.5); friction 0.6 | 211 / 844 / 0 |
| rain | Original 2,000 radius-0.15 circles, mass 0.1, 40 columns, spacing 0.38, original jitter/walls; friction 0.4, restitution 0.1, seed 0xC001D00D | 2003 / 16384 / 0 |
| piles | 16 ten-box columns, x = 3×(column−7.5), y = 0.5+row, shared ground (26, 0.5) | 161 / 2048 / 0 |
| chains | Eight vertical chains at x = 4×chain; static anchor y=14 plus twelve (0.25, 0.4) boxes at unit spacing; alternating distance/revolute links | 104 / 1024 / 96 |
| churn | 128 radius-0.25 circles in 16 columns at spacing 0.6; zero gravity | 128 / 2048 / 0 |
| table | Ground (8, 0.5); two (0.4, 1) legs at (±2, 1); mass-2 top (3, 0.2) at (0, 2.2); mass-5 load (0.5, 0.5) at (0, 2.9) | 5 / 64 / 0 |
| inverted | Six unit boxes at y=0.5+row, masses 0.25, 0.5, 1, 2, 4, 8; ground (8, 0.5) | 7 / 128 / 0 |

Chain local anchors are (0, −0.4)/(0, 0.6), distance target 0.005, collision
between connected neighbors disabled. Churn destroys/recreates four slots
`(4×step+j)%128` every warm-up/measured step, restores their grid location, then
moves x by ±0.15 according to step parity. LIFO reuse preserves ascending slot
order while generations advance; unaffected circles retain motion. These
teleports deliberately create overlaps. This is an active churn fixture, not
a settled physical equilibrium.

## Schema 1 and metric meanings

The machine output is one JSON object, validated strictly by `bench/report.py`:

- `schema_version`: integer 1.
- `metadata`: strings for compiler/version, build mode, compiler/warning flags,
  host OS/version/processor and configured source revision. Reconfigure after
  source changes. Source archives can supply `-DSL_BENCH_REVISION=<revision>`;
  otherwise Git supplies the revision and a tracked-change `-dirty` suffix.
- `results`: 1–7 unique scene records. `settings` contains fixture version,
  material values, seed, warm-up/measured counts, dt, substeps, capacities,
  gravity, drags, speed/solver defaults and the disabled sleep flag.
- `timing_ms`: average, median, nearest-rank p95, maximum complete-step time,
  and average mutation time (zero except churn). Samples are allocated before
  stepping; warm-up, setup, quality measurement, stats, sorting and output are
  excluded. Unavailable/backward/non-finite clock samples fail the run.
- `semantic_digest`: 16-digit word-wise binary32 hash including public body
  identities/transforms/velocities/mass/inertia/inverses/torque/material/type,
  active shapes and proxy bounds; ordered contact handles/materials/features/
  anchors/impulses; and joint handles/descriptors/reported impulses. This is
  a public snapshot digest, not the complete private replay comparison or a
  rollback format. It excludes addresses, padding and inactive shape fields.
- `counts`, `step`, `cumulative_work`: copied diagnostics with the definitions
  in [stats.md](stats.md). `drops` includes warm-up and measured steps and must
  be zero. Byte categories in `memory_bytes` sum to `arena`; `world` is separate.
- `quality`: geometry penetration and cached penetration maxima, joint error,
  translation/rotation drift, residual linear/angular speed, average static
  contact support force and total dynamic weight. `window_steps` is min(60,
  measured steps). Lengths are world units, rotation radians, speed units/s
  and radians/s, and force kg×units/s².

Penetration uses independent projection/SAT geometry at current transforms for
persistent contact pairs, including circle/polygon closest-vertex axes. It does
not claim to find pairs missing from the contact system; collision correctness
remains covered by engine oracles. Cached separation is reported distinctly.
Penetration and joint error maxima span all measured steps. Drift is relative
to the transform immediately before the last-window first step; speeds and
support forces use that final window. Joint error is anchor distance for
revolute joints or absolute distance-target error for distance joints.

Static contact support uses the final cached normal impulse divided by substep
dt and its vertical normal component. It excludes joint reactions, so chains
have no static-contact support ratio gate. The supported-weight reference is
total dynamic mass times downward gravity. Table/pyramid/rain/piles/inverted
require mean force/weight in [0.95, 1.05]. The table therefore checks load
transfer through its top and legs, not just whether bodies look stationary.

`quality_limits.json` pins full-profile limits from the unoptimized awake
measurements, before later experiments. The measured penetration maxima were
0.00614 pyramid, 0.05388 rain, 0.00641 piles, 0 chains, 0.16702 churn,
0.00125 table and 0.03250 inverted. Bounds retain those distinct workload
characteristics: 0.01/0.07/0.01/0.01/0.2/0.005/0.04 respectively. Settled drift
uses one linear slop (0.005); angular drift and residual limits are explicit in
the file. Chain joint error was 0.00608 and is bounded by two slops (0.01).
Churn's looser displacement/speed limits reflect intentional continuous
teleporting. These are host-baseline regression bounds, not new engine accuracy
promises or replacements for existing analytic tolerances. Smoke profiles do
not run these settling gates.

## Comparison and profiling

Same-binary repeated runs must match every result field except timing; metadata
is excluded. Never require equal hashes across compilers or platforms.
`compare --cross-build` requires equal settings and full-profile quality gates,
then reports timing, work, memory, digest equality and quality deltas. A digest
change is visible rather than silently accepted as proof of equivalent physics.
Review intentional solver changes with the existing physical tests as well.

Use five or more runs on the same otherwise-idle host/compiler/settings. Report
the median and range of per-run average step times, not only the best sample.
Quality evaluation outside the clock still changes cache residency: compare
candidate and baseline using the same harness. Shared CI has no speed threshold.
Use an external profiler (Instruments Time Profiler on macOS, `perf record` on
Linux) around a release invocation; inspect step, pair maintenance and solver
stacks. Keep profiling code/timers out of the library. Record the exact machine,
compiler flags, revision, scene and invocation with any optimization decision.

The recorded development host is an Apple M4 Pro (arm64), Darwin 25.6.0,
AppleClang 21.0.0.21000101, Release `-O3 -DNDEBUG`. The
[stats overhead record](../bench/reports/stats-overhead.json) preserves the
matched parent/instrumented measurement and its exact harness revisions.
It is historical measurement evidence, not an input to the current validator.

The [five-run summary](../bench/reports/matrix/summary.json) links by filename
to sibling raw reports. Each report records its measured source revision.
All seven fixtures pass full-profile quality gates and match deterministic
fields across the five executions.
