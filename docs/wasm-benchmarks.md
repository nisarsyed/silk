# WebAssembly benchmark measurements

The native executable and browser driver share C fixture construction, seeded
mutations, reference transforms, and independent quality sampling from
[`bench/fixtures.c`](../bench/fixtures.c). There is no JavaScript copy of the
seven physical workloads. Native reports retain schema 4; the WASM harness
places the same result rows inside a schema 1 measurement envelope.

This is a developer tool, separate from the installable package and any demo
renderer. The [browser contract](browser-contract.md) fixes the workloads and
acceptance conditions. [Native benchmark documentation](benchmarks.md) explains
the physical metrics and their units.

Vector dot products explicitly use the standard C `fmaf` operation. This
retains the fused first-product/addition rounding of the reference native
build on scalar WASM, where implicit contraction is unavailable. A cancellation
regression pins that numerical contract. There is no fast-math, relaxed SIMD,
new collision mode, changed solver pass, or changed fixture/quality limit.
WASM can implement `fmaf` in software, so its cost belongs in the measured
baseline; native hardware FMA timing is not a WASM cost estimate.

## Build and smoke checks

Activate the pinned SDK as described in [the WASM build guide](wasm.md), then:

```sh
cmake --preset wasm-release
cmake --build --preset wasm-release
ctest --preset wasm-release
node bench/wasm/test_driver.mjs build/wasm-release/benchmark/driver.mjs
npm ci --prefix wasm --ignore-scripts --no-audit --no-fund
PLAYWRIGHT_SKIP_BROWSER_GC=1 node wasm/node_modules/@playwright/test/cli.js install chromium
node bench/wasm/cli.mjs --build build/wasm-release/benchmark --output build/wasm-smoke/node-off --smoke --repeat 2
node bench/wasm/cli.mjs --build build/wasm-release/benchmark --output build/wasm-smoke/browser-off --smoke --repeat 2 --browser
python3 bench/wasm/validate.py validate build/wasm-smoke/node-off/run-1.json
python3 bench/wasm/validate.py compare build/wasm-smoke/node-off/run-1.json build/wasm-smoke/node-off/run-2.json
python3 bench/wasm/test_report.py --report build/wasm-smoke/node-off/run-1.json
```

Repeat with `--sleep on` and separate output directories, and with the
`wasm-debug` preset. Linux browser setup can use `install --with-deps chromium`.
CI performs both modes, both sleep policies, and two repetitions per host.
It uploads raw smoke reports, replay comparisons, build provenance, and test
logs. A smoke profile is exactly two warm-up steps and eight measured steps;
it validates structure and replay, and cannot claim settled quality.

The CLI defaults to all seven scenes, sleep off, five repetitions, 120 warm-up
steps, 600 measured steps, and fixed 64 MiB linear memory. `--scene` accepts one
fixture name or `all`; `--warmup` accepts 0–10000, `--steps` 1–10000, and
`--repeat` 1–20. `--memory` accepts a 64 KiB aligned byte count from 2–512 MiB.
Too little memory produces a retained failure report. Duplicate options and
explicit step counts combined with `--smoke` are rejected.

`--browser` launches actual Playwright Chromium in headless mode and executes
the harness inside its page. It does not turn Node timings into browser
timings. These automated runs are not native Safari/Firefox, a physical mobile
device, sustained thermal evidence, or a display-presentation measurement.

## Full profiles and comparisons

Run timing studies without competing builds, tests, or other benchmarks:

```sh
node bench/wasm/cli.mjs --build build/wasm-release/benchmark --output build/wasm-full/node-off
node bench/wasm/cli.mjs --build build/wasm-release/benchmark --output build/wasm-full/browser-off --browser
python3 bench/wasm/validate.py validate build/wasm-full/browser-off/run-1.json --quality
python3 bench/wasm/validate.py compare build/wasm-full/browser-off/run-1.json build/wasm-full/browser-off/run-2.json
python3 bench/wasm/validate.py compare build/wasm-full/node-off/run-1.json build/wasm-full/browser-off/run-1.json
```

Repeat for sleep on. Validate every raw repetition with `--quality`, and
compare every repetition against the first. Five runs expose timing spread;
they do not establish performance on another device. The process/page is
reused, but each repetition creates an independent WASM module and worlds.

To compare native and WASM physical quality:

```sh
cmake --preset release -DSL_BUILD_BENCH=ON
cmake --build --preset release
build/release/bin/sl_bench > build/native-off.json
python3 bench/wasm/validate.py native build/native-off.json build/wasm-full/browser-off/run-1.json
python3 bench/test_report.py --executable build/release/bin/sl_bench
python3 bench/test_study.py --executable build/release/bin/sl_bench_study
```

`compare` requires identical source/binary hashes and exact deterministic
results by default. `compare --cross-build` and `native` check matched workload
settings and the existing physical quality limits independently on both
builds; they permit different semantic digests, contact histories, work counts,
and platform memory layouts. Neither mode accepts contact drops. Keep other
engine/stress checks passing too: a successful seven-scene comparison alone
does not prove that a solver change is acceptable.

Full quality requires the exact 120/600 settling profile and the final
60-step drift/speed window. The CLI's `execution_status: complete` means that
the bounded driver completed its requested work. It is not a quality verdict;
the explicit validator adds that verdict. Failed execution retains the stage,
iteration, error, and available partial C report before disposal.

## Timing and memory meaning

The harness uses the host's `performance.now()`. The step region wraps
`bench.step()` only, including JavaScript dispatch, validation/phase checks,
and the WASM call boundary. C calls `sl_world_step` without any clock access.
Module loading, scene/output-buffer setup, warm-up, reference preparation,
churn mutation, quality sampling, one final snapshot copy, and reporting have
separate regions. Non-churn mutation is an untimed phase transition. Warm-up
includes the same C phase sequence and permits runtime JIT warm-up; it does
not erase first-module loading cost from its separate field.

Raw per-step samples are retained. Summary timing uses a median and nearest
rank p95 matching the native report. A 1000-pair timer probe records observed
resolution/overhead; its cost is never subtracted from samples. Coarse clocks
can legitimately produce zero-duration samples, especially in smoke runs.
There is no renderer, GPU, frame scheduling, or presentation in this harness.

Each module has fixed linear memory, a 1 MiB stack, and no growth. Reports
separate reserved linear memory, allocator occupancy, requested world/context
storage, private adapter staging, JS output payload, and timing sample arrays.
The output arrays are separate JS-owned buffers; writing their complete
backing buffers cannot modify engine memory. Object/GC overhead is not included
in payload byte counts. `static_end` and `heap_base` are addresses, not byte
sizes: debug can place the stack before data, while release places it after
data. The validator checks either exact aligned layout.

Counters remain `bigint` in JavaScript and canonical decimal strings in JSON,
including values above 2^53. The validator rejects duplicate keys, nonfinite
numbers, numeric counter substitutions, malformed or overflowing uint64
strings, incomplete work, inconsistent memory budgets, and incorrect timing
summaries. Input JSON is bounded to 10 MiB.

## Calling the driver from another harness

Import `createBenchmarkModule` from the generated `benchmark/driver.mjs`.
Create a scene with `module.create(name, {sleep, warmup, steps})`, which returns
null on bounded allocation failure. Every iteration must call `prepare()`,
`mutate()`, `step()`, then `sample()`, each returning a boolean. Invalid phase
order rejects without advancing physics. Run exactly `warmup + steps`
iterations. Reference capture and deterministic mutations use the shared C
implementation; callers cannot substitute a different workload through this
interface.

`refreshSnapshot(diagnostics = false)` copies a reusable snapshot for later
boundary/render studies. Its validity/revision and geometry-cache rules match
the package API; any phase operation invalidates the prior snapshot.
`report()` returns a copied report, so it belongs outside timed stepping.
Dispose scenes and modules explicitly. Independent modules do not share
worlds or memory.

The generated `browser.html` exposes `window.runBenchmark(options)` for manual
browser harnesses. Serve the generated benchmark directory over HTTP with
JavaScript/WASM MIME types. Supply `device` and run-condition metadata when
using it manually. Start/end visibility is recorded and hidden endpoints fail
the run; this endpoint check does not prove continuous foreground visibility.

Every build regenerates `benchmark/build.json` with the actual revision/dirty
state, source digest, binary digest, compiler identity, SDK commit, complete
compile commands, and link command. Use the provided Makefile presets and a
Git checkout for this provenance path; source archives or other generators
are not silently assigned fabricated provenance. Rebuild after source edits.
Keep generated evidence under ignored `build/` directories or another local
output directory, not in the installable package or checked-in machine files.
