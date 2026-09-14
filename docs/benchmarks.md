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
`--sleep off|on` selects activation policy and defaults to off.
`--warmup` accepts 0–10000 and `--steps` 1–10000. Output is always JSON.
The runner preserves raw stdout/stderr when validation fails. Use
`python3 bench/report.py --help` for validation and comparison subcommands.

The [WASM harness](wasm-benchmarks.md) runs these same C fixtures in Node and
browsers, retaining this native schema inside its measurement envelope.

## Choose a fixture

Every fixture uses dt = binary32(1/60), four substeps and zero
drag. [Fixture builders](../bench/fixtures.c) define geometry, seeds, materials and
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

## Schema 4 and metric meanings

[report.py](../bench/report.py) validates schema 4; a
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

The `cached_support_force_mean` estimate uses the final cached normal impulse divided by substep
dt and its vertical normal component. Sleeping steps retain that cache; it is not a measurement of newly executed
solver work. The estimate excludes joint reactions, so chains
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

## 0.4.0 release verification

[Matched release evidence](../bench/reports/phase4-release/comparison.json)
compares merged main `feb220c` with clean release candidate `cba1ebf`. Five
alternating repetitions per revision and sleep mode cover all seven matrix
scenes and five solver-study scenes; queries have five matched batches.
Every non-timing result matches, including physical digests, quality, work,
activation and exact memory. All quality gates pass and drops remain zero.
The eleven simulation object files are byte-identical; only the version
implementation object differs. This wrap introduces no physics optimization.

On the recorded Apple M4 Pro/AppleClang 21 host, pyramid/rain/piles medians
vary by less than 1% between revisions in both modes. Tiny fixtures have larger
relative spread, including roughly 6–7% higher inverted-stack medians at about
0.003 ms per step. These are whole-step measurements, not evidence of a new
engine cost or a promised performance improvement. Raw runs preserve the full
ranges rather than selecting favorable samples.

[Historical comparisons](../bench/reports/phase4-release/historical-comparison.json)
also retain exact non-timing equality with the accepted workset measurements.
Their timing comes from different sessions and is contextual evidence only.
The original reports and rejected experiment records remain unchanged.
[Verification details](../bench/reports/phase4-release/verification.json) record
provenance, prerequisite merges and local checks; hosted CI is recorded on the
release PR. Schema 4 remains the only current matrix format.

To repeat the matched collection, archive each recorded revision into a separate
source directory and configure each with `-DCMAKE_BUILD_TYPE=Release`,
`-DSL_BUILD_BENCH=ON`, `-DSL_BUILD_TESTS=OFF`, and
`-DSL_BENCH_REVISION=<full recorded revision>`. Build both, then run:

```sh
python3 bench/reports/phase4-release/collect.py /path/to/parent/build/bin /path/to/candidate/build/bin --output build/release-evidence
```

Keep the same compiler/flags and otherwise-idle host. The collector alternates
revision order, preserves raw reports, applies the current validators and quality
limits, and rejects any non-timing difference. It is a release comparison tool;
intentional future physics changes require separate acceptance criteria.

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
profiles for each sleep mode across supported compilers. Full settling/performance runs remain
explicit developer work; machine speed cannot fail ordinary PR checks.
Download raw reports and comparison artifacts from the workflow run within
14 days. Their metadata identifies the tested revision, including the combined
PR/base checkout. Failed validation retains raw output when available.

The [world-storage comparison](../bench/reports/world-storage.json) records the
schema-1 to schema-2 measurement, including preserved parent reports. Historical
reports are evidence only; the current validator accepts schema 4.

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
archived under `bench/reports/islands-parent/`; `bench/reports/islands-awake/`
records the measured island implementation revision. Graph construction adds work even when
all bodies are awake; sleeping benefits are measured separately in the next layer.

Schema 4 adds resolved sleep policy, current awake/sleeping dynamic counts,
last-step executed/skipped island counts, wake work and body transition totals.
`memory_bytes.sleep` is 13 bytes per body capacity; fixed state and padding are
reported separately. The physical digest is unchanged; activation is compared
through the other report fields and complete engine replay tests.

```sh
python3 bench/report.py run build/release/bin/sl_bench --output build/sleep-off --sleep off --quality
python3 bench/report.py run build/release/bin/sl_bench --output build/sleep-on --sleep on --quality
```

Each command defaults to five runs. Compare timing spread and executed work,
while checking physical quality independently across activation policies.
`compare` deliberately requires identical settings; it must not equate an awake
workload with a sleeping one. Rain and inverted-mass components can remain awake
when constraint-error guards fail, even with low residual speeds.

The [activation comparison](../bench/reports/sleep.json) records five full runs
per mode, sleep-disabled equivalence to the island parent, timing spread,
combined memory and quality. The current matrix contains sleep-off reports;
`bench/reports/sleep-on/` contains sleep-on reports. The schema-3 island parent
is archived under `bench/reports/islands-awake/` with its measured revision.

On the recorded host, sleep-on median total-step time was about 76% lower for
piles and 60% lower for chains than sleep-off, while rain was about 7% higher.
Churn's sleep-disabled path was about 32% higher than the island parent, with
another 22% increase when enabled. Those moving-body costs remain visible in
the evidence; settled-scene savings do not imply a universal speed improvement.

## Workset investigation

[Decision records and raw measurements](../bench/reports/worksets/decisions.json)
start from merged main `0b22cb6` on an Apple M4 Pro with AppleClang 21,
`-O3 -DNDEBUG`. Profiling uses a separate `-g` build and macOS `sample`; its
stacks include setup, warm-up and quality evaluation. They identify candidates,
not isolated function timings or hardware-counter measurements.

The first adopted change skips joint stages when no joint constraints were
prepared. Previously, four substeps caused thirteen full island traversals for
joint warm start, solves and storage even in worlds without joints. One count
check per stage now avoids those metadata reads and sleep-predicate calls.
Nonempty constraint order and allocation size are unchanged. Five alternating
baseline/candidate runs confirmed about 10% lower churn total-step time in both
sleep modes, with every non-timing report field unchanged. Other fixture timing
spreads remain in the raw reports; this is not a universal 10% improvement.

`build/release/bin/sl_bench_query` measures queries separately from steps. It
builds 1,024 static circles of radius 1/32 on a 1/8-unit grid, permuting creation
order by multiplication by 561 modulo 1,024. Each fixture warms up and measures
65,536 queries with an eight-handle buffer. Point queries alternate exact centers
and gaps that overlap fat proxies; broad queries cover the entire grid and
truncate. Geometric counts and ordered handles are checked outside timing.
Repeat the executable five times against each measured library, alternating
execution order. Its small JSON array is experiment output, separate from the
versioned simulation report schema; compare digests and per-batch milliseconds.

Filtering exact query matches before sorting was rejected: confirming runs
improved the point fixture by 10.7% but slowed the broad fixture by 1.7%.
Compaction adds writes and changes geometry access order, despite reducing
sorting from candidate count to match count. The archived patch reproduces the
experiment against the baseline; it is not a runtime alternative.

Compiler investigation uses `-Rpass=loop-vectorize`,
`-Rpass-missed=loop-vectorize`, `-Rpass-analysis=loop-vectorize`, and
`-Rpass=slp-vectorizer`. Build with `-j 1` to keep diagnostics readable. Compare
the same baseline with `-fno-vectorize -fno-slp-vectorize`; both sleep modes pass
quality gates and retain physical digests. Initial median timing differences
of roughly -0.5% to +2.6% do not establish a useful whole-workload change.

The targeted SIMD experiment replaces the four interval comparisons in AABB
overlap with an ARM NEON compare/reduction, retaining all input validation.
The [experiment patch](../bench/reports/worksets/simd-aabb.patch) applies to
baseline `0b22cb6` in an isolated source copy. It is not compiled by any normal
target. Configure that copy with the same release settings and
`-DSL_BENCH_REVISION=0b22cb6-simd-aabb-experiment`, then use the regular report
runner for both sleep modes. The confirming reports record exact invocations.

Release tests and every non-timing matrix field match the baseline. Five
alternating runs show no repeatable whole-step improvement; separate query
medians improve approximately 2–3%, with larger point-query spread. Both
versions still load four-float AABBs and follow dependent tree nodes. The
archived wrapper assembly shows six lane moves plus vector comparison/reduction
after unchanged validity checks. This does not justify a platform-specific
production path. Retain portable scalar source and normal compiler options.

To reproduce the wrapper assembly, compile a function returning
`sl_aabb_overlaps(a, b)` with `cc -std=c17 -O3 -DNDEBUG -S`, selecting the
baseline or patched include directory. The archived step assembly also shows
compiler-generated two-lane arithmetic already present in portable source.
These observations are specific to the recorded compiler and host.

Pair-table and wake-scratch rewrites remain deferred. Churn records 75,276
cumulative pair probes and 860,526 graph constraint visits; those are different
operations, not comparable cycle costs. The empty-stage profile supplied a
smaller supported change without altering pair occupancy, wake metadata or
memory layout. Broader changes need their own measured advantage.

### Threading feasibility

Dynamic islands are the useful future task boundary. Their body slots and
prepared constraint ranges are disjoint, and velocity stores explicitly skip
static and kinematic endpoints. Shared boundaries can therefore be read by
multiple island solvers after the global kinematic update. Preserve source
constraint order within each island: parallel updates to connected constraints
would change the sequential solver and require a separate numerical design.

The current whole-world functions are not worker entry points. Tree queries
share a traversal stack and candidate buffer; wake propagation and union-find
reuse the same scratch; contact insertion/removal, free lists, pair tables and
diagnostic accumulation are mutable world state. Broad-phase parallelism would
need bounded per-worker scratch and a deterministic merge before modifying
contact order. Allocating that scratch and merging candidates could cost more
than the saved queries. Caller queries remain non-reentrant.

A future island experiment would require a caller-controlled worker contract,
initialization-sized tasks, disjoint range execution, synchronization around
global kinematic updates and step publication, and bounded local counters
combined after completion. A platform thread pool or scheduler is outside this
change. Do not add optional runtime allocation or depend on platform-specific
thread APIs in the core merely to test this hypothesis.

Current work sizes do not justify that machinery. The awake piles fixture takes
about 0.08 ms for the *whole* step across sixteen components, or at most roughly
5 microseconds per component if all work were distributable; chains take about
0.018 ms across eight components. Actual independent solver work is smaller.
Sleeping eliminates most of it while leaving contact/graph work active. Rain's
larger components reduce available island parallelism. These are workload
estimates, not measurements of scheduling overhead. Defer production threading
until a larger independent-island workload demonstrates enough work to amortize
dispatch, barriers, cache-line sharing and the remaining serial phases.

## Solver-order study

The [experiment protocol](../bench/reports/solver-order/protocol.json) fixes the
baseline, graph policy, stage mapping and scratch budget before prototyping.
The [primary method](https://doi.org/10.1145/3747867) and
[author project](https://ziyanxiong.github.io/2PSP/) describe withholding lower-body
responses during an upward pass and applying them during a downward pass.
This differs from reversing the constraint sequence. The study separates
ordering alone from a bounded two-pass adaptation; equal substeps do not mean
equal constraint work. No solver alternative is enabled by the normal build.

The public-only `sl_bench_study` executable adds five fixed adversarial fixtures
to the unchanged main matrix. Each uses 16 body slots, 128 contact slots, four
substeps, dt = binary32(1/60), 120 warm-up steps and 600 measured steps. Geometry,
materials and events are committed in [study.c](../bench/study.c); no random
seed or frame-clock input enters the worlds.

| Fixture | Additional check |
|---|---|
| Bridge | A loaded beam transfers weight through two separated static supports |
| Collapse | Remove the supporting platform at the first measured step; retain a lower catch floor |
| Zero gravity | Two unequal masses exchange momentum in a restitution-one collision |
| Cycle | Equal-height contacting bodies expose ambiguous gravity layering |
| Topple | Give the top box horizontal velocity at the first measured step |

```sh
python3 bench/study.py build/release/bin/sl_bench_study --output build/study-off --quality
python3 bench/study.py build/release/bin/sl_bench_study --output build/study-on --sleep on --quality
python3 bench/test_study.py --executable build/release/bin/sl_bench_study
```

The runner defaults to five repetitions, checks finite output and deterministic
fields, and records timing spread and quality violations. Omit `--quality` when
collecting a candidate known to fail acceptance so its failures remain in the
summary. The separate study format has one strict version and no legacy reader.

All-pairs current-transform geometry measures penetration independently of
contact discovery. The maximum spans measured steps; ten samples at 60-step
intervals show its evolution. Drift and residual speeds use the final 60 steps.
Drop, horizontal movement and rotation excursion cover the entire measured
interval relative to the pre-event transforms. Momentum error covers warm-up
and measured steps and applies only to the zero-gravity fixture, whose initial
total momentum is zero. Lengths are world units, speeds units/s, angles radians,
angular speeds radians/s, momentum kg×units/s and cached support force
kg×units/s². Support values retain the cached-impulse caveat above.

[Study quality limits](../bench/study_limits.json) are pinned before solver
experiments. The unchanged solver reaches roughly 0.204 units of transient
penetration on the falling-stack impact; its bound is 0.22. Bridge/cycle bounds
use one linear slop, while toppling retains the 0.04 stress-scene bound. Motion
minimums require the removed-support stack to fall and the disturbed stack to
topple; a frozen but apparently stable candidate cannot pass. Existing main
matrix thresholds remain unchanged.

The [measured decision](../bench/reports/solver-order/decision.json) retains Soft
Step. Ordering alone passed quality, but added memory beyond the pinned budgets
and increased most full-matrix timings (about 9% for rain on the recorded host).
The bounded two-pass candidate reduced settled bridge penetration, but toppling
penetration rose from 0.031 to 0.118 units. It also failed existing pyramid,
piles, table, inverted-stack and ten-box drift checks. A faster failed pyramid
run reflects a different, unstable trajectory and is not an accepted speedup.
Both sleep modes were measured five times; raw reports retain timing spread,
physical results and actual work. Tiny study scenes show substantial timing
variation, so their medians are not precise microbenchmark claims.

The [isolated patch](../bench/reports/solver-order/experiment.patch) contains
both experiments and their private graph/rollback tests. It replaces the earlier
ordering-only snapshot. It is evidence, not a supported runtime backend. Apply
it to the harness revision in a disposable source directory:

```sh
git fetch origin 894213f18f40790ad16e6a0b6a5d30f8806c6f0a
mkdir -p build/solver-experiment
git archive 894213f | tar -x -C build/solver-experiment
git apply --directory=build/solver-experiment bench/reports/solver-order/experiment.patch
cmake -S build/solver-experiment -B build/solver-ordered -DCMAKE_BUILD_TYPE=Release -DSL_BUILD_BENCH=ON -DCMAKE_C_FLAGS=-DSL_STUDY_TWO_PASS=0 -DSL_BENCH_REVISION=894213f-order-experiment
cmake -S build/solver-experiment -B build/solver-two-pass -DCMAKE_BUILD_TYPE=Release -DSL_BUILD_BENCH=ON -DCMAKE_C_FLAGS=-DSL_STUDY_TWO_PASS=1 -DSL_BENCH_REVISION=894213f-two-pass-experiment
cmake --build build/solver-ordered
cmake --build build/solver-two-pass
python3 bench/study.py build/solver-two-pass/bin/sl_bench_study --output build/two-pass-study
python3 bench/report.py run build/solver-two-pass/bin/sl_bench --output build/two-pass-matrix
```

Repeat the last two commands with the ordering executable and with `--sleep on`.
Do not enable `--quality` when collecting known failures; validate them against
the unchanged limits afterward. The normal source retains no experiment API,
scratch or selector. This bounded adaptation uses one sweep per layer per
direction; it does not reproduce the reference implementation's much larger
iteration budgets or fallback substeps.

Experiment counters are written to stderr, once per scene, outside timing.
Each `STUDY` line contains graph body visits, graph contact visits, actual biased
and relaxation manifold evaluations, ordered island builds, extra arena bytes,
six fallback counts (zero gravity, joints, kinematics, equal heights, unrooted,
cycle), and rollbacks. The interval includes warm-up. Fallback reasons can
overlap. Warm start and restitution are excluded from evaluation counts.
Baseline reports have no such counters. `decision.json` names every field and
records exact allocation deltas, including context and alignment.

The experimental test suite deliberately retains failing production memory
assertions, and two-pass also retains the failed ten-box drift assertion.
Independent seeded layer reconstruction, fallback policies and finite failure
rollback pass; debug, release and sanitizer logs are archived alongside the decision.
No guardrail is relaxed to make the experiment appear acceptable.
