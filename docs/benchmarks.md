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
in `.exe`. No Python package is required. `--scene` accepts `all` (the default)
or one fixture below.
`--warmup` accepts 0–10000 and `--steps` 1–10000. Output is always JSON.
The runner preserves raw stdout/stderr when validation fails. Use
`python3 bench/report.py --help` for validation and comparison subcommands.

## Choose a fixture

Every fixture stays awake and uses dt = binary32(1/60), four substeps and zero
drag. [Fixture builders](../bench/main.c) define geometry, seeds, materials and
capacities; each report's `settings` records the resolved workload.

| Scene | Workload |
|---|---|
| `pyramid` | 20-row box pyramid settling on static ground |
| `rain` | Seeded 2,000-circle rain inside static walls |
| `piles` | 16 disconnected ten-box columns on shared ground |
| `chains` | Eight anchored chains with alternating distance/revolute joints |
| `churn` | Continuous bounded destruction, slot reuse and proxy movement |
| `table` | Load transfer through a tabletop and two legs |
| `inverted` | Six-box stack with mass increasing toward the top |

Churn deliberately creates overlaps by teleporting bodies; its quality limits
account for ongoing motion rather than expecting settled equilibrium.

## Schema 3 and metric meanings

[report.py](../bench/report.py) validates schema 3; a
[committed report](../bench/reports/matrix/run-1.json) shows every field.
Reconfigure after source changes to refresh build/revision metadata. Source
archives can supply `-DSL_BENCH_REVISION=<revision>`; Git builds record their
revision and a tracked-change `-dirty` suffix.

- `timing_ms` measures complete steps (average/median/nearest-rank p95/maximum)
  and churn mutations separately. Setup, warm-up, quality evaluation, diagnostics,
  sorting and reporting are outside the timing regions. Invalid or backward
  clock samples fail the run.
- `semantic_digest` hashes live public body/contact/joint state in stable order.
  See [the digest implementation](../bench/quality.c) for the exact fields.
  It is a public snapshot check, not the complete private replay comparison.
- Counts, work and memory follow the contracts in [world.h](../include/silk/world.h).
  The `world_state` category is private metadata within the allocation; `world`
  is the caller-owned shell. Compare their combined total when layouts change.
  `drops` covers warm-up and measured steps and must be zero.
- Quality lengths are world units, rotations radians, speeds units/s or radians/s,
  and forces kg×units/s². The final measurement window is min(60, measured steps).

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

[quality_limits.json](../bench/quality_limits.json) pins full-profile limits from the unoptimized awake
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

## Recorded evidence

The [five-run summary](../bench/reports/matrix/summary.json) and sibling raw
reports record the measured revision, toolchain, host, settings and timing spread.
Use those records for exact values; the full matrix passes quality gates and
matches deterministic fields across all five executions.

The [stats overhead record](../bench/reports/stats-overhead.json) preserves a
matched parent/instrumented measurement with its own harness revisions. It is
historical evidence, not an input to the current validator. Small timing deltas
within the run-to-run spread do not establish a speedup or slowdown.

## Check the tools locally

Run `python3 bench/test_report.py --executable build/release/bin/sl_bench` for
parser and live CLI checks, and `python3 tools/check_format.py` for tracked C
sources/headers. These developer-tool checks run separately from CTest.

The [CI workflow](../.github/workflows/ci.yml) runs two all-scene 2/8 smoke
profiles across supported compilers. Full settling/performance runs remain
explicit developer work; machine speed cannot fail ordinary PR checks.
Download raw reports and comparison artifacts from the workflow run within
14 days. Their metadata identifies the tested revision, including the combined
PR/base checkout. Failed validation retains raw output when available.

The [world-storage comparison](../bench/reports/world-storage.json) records the
schema-1 to schema-2 measurement, including preserved parent reports. Historical
reports are evidence only; the current validator accepts schema 3.

Constraint-island reports use schema 3. `island` memory is 44 bytes per body
capacity plus four bytes per contact/joint capacity; padding and fixed state
metadata are separate. `step.islands` and `island_bodies_max` describe the last
completed graph build. Graph work counts inspected body-slot entries, source
constraint rows and union/find parent reads, including repeated passes. All
constraints retain their original relative order within each component.
Compare whole-step timings with matched parent runs; their difference is total
overhead, not an isolated graph-build timer.

The [island comparison](../bench/reports/islands.json) records five matched
schema-2 parent and schema-3 island runs, including exact combined memory and
unchanged physical digests, quality and preexisting work. Raw parent inputs are
archived under `bench/reports/islands-parent/`; the current matrix records the
measured island implementation revision. Graph construction adds work even when
all bodies are awake; sleeping benefits are measured separately in the next layer.
