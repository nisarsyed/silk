# Renderer comparison tooling

This directory belongs to [#95](https://github.com/nisarsyed/silk/issues/95).
It currently contains three **base and diagnostic correctness prototypes** and
their shared geometry checks. No renderer is selected. This is developer tooling,
excluded from the independent package and from any production sandbox.
The frozen [browser contract](../../docs/browser-contract.md) remains binding.

## Build and verify

Use the pinned Emscripten 6.0.9 environment and build the benchmark module first:

```sh
cmake --preset wasm-release
cmake --build --preset wasm-release
emcmake cmake -S . -B build/wasm-render -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DSL_BUILD_TESTS=OFF -DSL_INSTALL=OFF -DSL_RENDER_STUDY=ON
cmake --build build/wasm-render
npm --prefix wasm ci --ignore-scripts --no-audit --no-fund
PLAYWRIGHT_SKIP_BROWSER_GC=1 node wasm/node_modules/@playwright/test/cli.js install chromium
python3 bench/render/build.py
python3 bench/render/test_metadata.py
node bench/render/test_provenance.mjs
node bench/render/test_study.mjs build/wasm-release/render-physics
node bench/render/test_geometry.mjs
node bench/render/test_overlay.mjs
node bench/render/test_pointer.mjs
node bench/render/test_frozen.mjs
node bench/render/test_canvas_frozen.mjs
node bench/render/test_recording.mjs
node bench/render/test_timing.mjs
node bench/render/test_gpu.mjs
node bench/render/test_gl_stats.mjs
node bench/render/test_windows.mjs
node bench/render/test_runner.mjs
node bench/render/test_hud.mjs
node bench/render/test_browser.mjs
```

To exercise the installed Chrome with its actual graphics context, use
`SL_RENDER_BROWSER=chrome SL_RENDER_HEADED=1 node bench/render/test_runner.mjs
build/render-study build/reports/render-runner-chrome`. This opens a temporary
window, keeps the browser's actual viewport/DPR, and remains correctness-only.

`SL_RENDER_STUDY` is off by default and requires Emscripten. Enabling it fetches
raylib at the same **6.0** pin as the native sandbox. This web target uses its
WebGL 2 configuration and original batching implementation. The pin's
Emscripten `EM_ASM` calls require GNU mode for the third-party raylib target;
Silk and the first-party comparison adapter retain C17 with extensions off and
the authoritative warnings. `CUSTOMIZE_BUILD` stays off because of the 6.0
configuration-parser behavior documented in the sandbox build. The private
module combines its heap-view exports with raylib's required `ccall` export;
it does not modify vendored source or change the sandbox pin/configuration.

`build.py` compiles the TypeScript and copies the separately built modules into
ignored `build/render-study`. Its `--benchmark`, `--raylib`, and `--output`
arguments select other build directories. `--physics` defaults to the sibling
`render-physics` directory beside the selected benchmark. The browser test takes the assembled
directory and report directory as positional arguments. It starts an ephemeral
loopback-only static server and fresh Playwright Chromium pages, preserving
JSON results (including failures) and PNGs under `build/reports/render-correctness`. This is browser
correctness coverage, not actual desktop/mobile performance certification.

## Equal input and candidate paths

`DrawScene` consumes the shared C benchmark's copied snapshot, retaining row
order and indexing geometry by body slot. It uses the contract's orthographic
rectangles and letterboxing. Circle rendering uses the same explicit 32 local
vertices, starting at positive x, in all three candidates. Polygon rendering
uses the snapshot's original convex vertices. Static bodies use RGB
`(113,146,165)`, moving bodies `(235,189,112)`, and the opaque background is
`(16,24,32)`. These portable palette choices are shared, not candidate knobs.

The nine frozen render-only tiers repeat rain's dynamic rows in 32-unit-wide
horizontal tiles after 120 steps, omit static bodies, and trim the last tile.
Their camera is the union of the translated rain rectangles. All source shapes
and transforms are prepared before drawing; a frozen tier cannot refresh its
transforms. The original seven native workloads and their limits are unchanged.

Frozen diagnostics retain each contact involving any displayed dynamic body,
including contacts against omitted static shapes or trimmed bodies, once per
tile. Proxy bounds and source island colors are copied in draw order. A
setup-only C helper evaluates the global central queries over the actual
displayed instances using public shape predicates, with the same results as
public world queries. It allocates `20*instances + 32` column bytes plus its
small owner record, copies the results into independent JS columns, then frees
the native temporary storage. No additional simulated world or per-frame query
allocation is introduced. The source remains unchanged at step 120.

`FrozenOverlay` builds the shared immutable command arrays once. It reserves
the existing bounded capacities for contacts which actually have points;
zero-point contacts have no glyphs. Every eligible point is retained. Canvas
caches contiguous-color body, line and marker paths during setup; WebGL and
raylib reuse their unchanged command payloads on repeated frames. The contact
rule follows the user clarification recorded in contract revision 2.

- Canvas caches local `Path2D` objects for moving scenes. Frozen scenes merge
  paths during setup, preserving contiguous geometry/color batches. Paths and
  matrices are never allocated per body during drawing.
- WebGL 2 uses cached triangulated meshes, fixed VAOs and instanced draws for
  contiguous geometry/color batches. It preserves source ordering and uses
  preallocated GPU buffers. A changed transform revision uploads 16 bytes per
  instance; a repeated revision uploads none. No readbacks, shader-status
  checks, or diagnostic GL polling occur in `draw()`.
- The raylib prototype consumes exactly those triangles and poses through one
  private call, emits them through raylib's `rlgl` batching, and uses the pin's
  normal `BeginDrawing`/`EndDrawing` path with no frame sleep. Its current
  standalone correctness entry point has a fixed 64 MiB budget and copies 16
  pose bytes per changed instance across the JS/module boundary. This copy is
  reported. The same binary now also contains the shared C study owner: the
  separate `createRaylibStudyModule` entry point owns physics and graphics in
  one fixed 64–512 MiB memory. It retains the prototype's 64 MiB graphics budget
  as its minimum; the independent physics/package owner still allows 2–512 MiB.
  Its renderer writes 16 pose bytes per body directly in C after a step,
  using stored transforms without trigonometry or snapshot packing. Repeated
  draws reuse those poses. Meshes cross JS once during setup; live pose copies
  across JS are zero. Diagnostic commands are also prepared directly in the
  same C memory, with no JS command arrays or bulk diagnostic boundary copy.
  Two scalar command counts cross the boundary for reporting. Repeated draws
  reuse both poses and diagnostics until the next simulation step.

Shared pose payload is 16 bytes/instance, source rows and tile offsets another
8 bytes/instance, plus cached unique meshes/batches and JS object overhead.
WebGL GPU payload is 16 bytes/instance plus unique triangulated meshes; driver,
VAO and framebuffer overhead is not included. Canvas internal memory/uploads
remain unavailable. The collection loop observes submitted WebGL buffer uploads
and draw calls for both GL candidates, including raylib's actual batch flushes.
Driver-internal traffic and memory remain unavailable. Optional nonblocking GPU
timer queries are integrated; actual-device performance evidence remains required.

## Shared physics setup and sustained driver

`scene.c` builds the default fixture through `bench/fixtures.c`. The one-copy
case uses that original construction directly. Scaling copies the original
body and joint descriptors through public C APIs, translates each copy by the
contract's 32-unit formula, and multiplies capacities by exactly 1/2/4/8/16.
The temporary source world, fixture record and handle map are released after
initialization. No JS scene definitions or engine limits change. Body/joint
insertion order and within-copy joint endpoints are preserved.

`driver.c` owns one such world and its existing bounded snapshot/query adapter.
Each call steps at the original binary32 timestep and four substeps, records
contact drops as uint64, and stops at the caller's declared bound. Its separate
21,600-step maximum covers six minutes at 60 Hz; the host still must discard
the warm-up world and limit measured intervals to the contract's 60/300 seconds.
Clocks and scheduling are outside C. The native benchmark retains its original
10,000-step bound and unchanged full-quality oracle. The driver supplies a
private borrowed world/adapter to support co-located raylib rendering. Its
separate generated `render-physics/driver.mjs` browser/worker/Node factory owns
world lifetimes, validates fixed budgets and step/tier inputs, returns copied
settings, and reuses the package's isolated snapshot columns. `step()` and
`refreshSnapshot()` have no per-frame object allocation. `report()` explicitly
allocates copied settings/statistics/counters outside the measured path.
The private JS ownership implementation is shared by standalone physics and
raylib. A world can attach one renderer; world/module disposal closes graphics
before freeing physics. A replacement world can reuse the canvas. Neither raw
WASM pointers nor writable C pose buffers escape that ownership closure.
`rebuild()` consumes a world and returns its exact initial configuration while
retaining raylib's warmed graphics resources. It frees the old physics arena
before allocating its replacement; two arenas never coexist. Failure leaves
the old world disposed and closes retained graphics. On success the new world
owns their eventual disposal; disposing the consumed owner again is harmless.

The study checks finite transforms, velocities, proxy bounds, live contact
floats and joint impulses after each executed C step, including release builds.
This bounded scan costs O(bodies + contacts + joints), allocates nothing, and
is included in the measured study step cost. Failure is latched, preserves the
executed-step/drop counters and rejects later steps. The original step-only
benchmark and core simulation are unchanged. Tests inject non-finite values
between calls, restore them, and verify that the failed-run latch remains set.

## Interaction driver

The private study owner exposes `pointer(action,x,y)` for a separate interaction
trial on a one-copy default fixture. Actions 0/1/2/3 are press/move/release/cancel;
coordinates are finite world metres within the existing body-position domain.
A press picks the nearest-center dynamic body containing the point, breaking
ties by ascending slot, and stores its local grab point. A valid miss preserves
the gesture without creating a body. Duplicate presses and unheld moves reject;
release/cancel are idempotent. Scalar `pointerState` getters expose held state and
the selected slot/generation without leaking native buffers or allocating rows.

This matches the native sandbox's 15 N/m tether. Input events bank no force;
each executed fixed step applies force and torque once at the original local
grab point. Release/cancel stop it immediately. No fixture capacity, solver
setting or workload limit changes, and queries reuse existing bounded storage.
Stale selection clears safely; rejected force application fails the driver.
Rebuild starts with no held gesture. C tests compare each stepped world against
an independent twin driven through public force-at-point calls, across all
three default fixtures and both sleep modes.

This is the physics primitive for the required separate interaction run.
Browser checks compare scripted interaction poses/diagnostics against the
co-located C renderer, including graphics-preserving rebuilds with cleared grabs.
These scripted calls are correctness checks. Browser event recording, real-input
provenance, coalescing, event-to-step/frame
correlation and latency qualification still need integration; the normal
collection loop does not attach pointer listeners yet.

## Browser collection loop

`runner.mjs` exports `runStudy` for an attached, uniquely identified canvas and
an optional diagnostic HUD element. It accepts the three candidates, the three
physics fixtures, all physics tiers, sleep policy and frozen desktop/mobile
layouts. Render-only collection accepts the nine `instances` tiers with rain,
one copy and sleep off. It prepares the source at step 120, warms immutable
draws, then rebuilds the same source before measurement. Recorded simulation
steps, debt and drops during measurement remain zero; cumulative source
counters retain their 120-step preparation baseline. `sustained` is restricted to mobile rain, one copy, sleep off and base
visuals. Normal collection uses 60 seconds; sustained collection uses 300.
`memoryBytes` defaults to the contract's 64 MiB and accepts an explicit aligned
budget within the module's existing bounds. Larger physics tiers may require
a larger budget, which is recorded; the loop never grows or retries a budget.
An explicitly named `correctnessSeconds` option (0.25–5 seconds) exercises the
loop in CI and labels its output `correctness-only`. Every current output sets
`acceptanceEligible: false` and lists the missing protocol evidence.

The loop records 240 idle rAF intervals and applies the frozen median/divisor
calibration and 59–61 Hz gate. It warms a disposable world for 120 steps and
120 callbacks (60 seconds for sustained runs), then creates the exact initial
world while retaining cached paths, shaders and GPU buffers within the fixed
memory budget; allocation failure remains a failed run. Scheduling starts with zero debt, uses the original binary32
timestep, caps catch-up at eight steps and records discarded time. The clock's
planned steps must match the authoritative C count. Raw frame storage is bounded
from duration and calibrated target cadence, with no growth or discarded prefix.
If the first rAF timestamp is stale, its deadline advances on the calibrated
grid to the target nearest callback entry. Simulation still starts at zero debt
at that entry. This prevents queued startup timestamps from producing a burst
of overdue submissions. A full frame buffer fails before executing another
simulation step or attempting GPU timing; all existing limits remain unchanged.

Base and diagnostic frames record CPU stage timings, submission times, work,
drops and debt; diagnostic HUD updates remain at 4 Hz. Unknown GPU metrics are
explicitly unavailable. Hidden documents, context loss, display/viewport changes,
cancellation and numerical/step failures preserve the completed record prefix
and final world report. Resources and listeners are released after collection.
Reports use one monotonic clock origin and distinguish actual device DPR from
the contract's render DPR. Serialize uint64s using the study module's
`jsonReplacer`; raw frame work counters are already exact decimal strings.

This is a collection primitive, not the complete study controller. It does not
yet implement real-input trials, optional 120 Hz measurements,
device-condition recording,
GC/memory profiling, repeated-run ordering or qualification. Browser tests
use short runs and injected interruption events; they are correctness evidence.
`test_runner.mjs` defaults to real clocks for local and installed-browser checks.
CI explicitly sets `SL_RENDER_TEST_CLOCK=synthetic`: native rAF still schedules
real rendering, while a test-only clock supplies deterministic 60 Hz timestamps.
This avoids making headless host cadence a CI performance gate. Every resulting
report declares `clock.testOverride`, and the auditor marks its timing budgets
inapplicable. Installed/headed browsers reject this test mode. The production
collector and calibration limits are unchanged. Separate injected duplicate
interval and unsupported-rate cases must still fail calibration and retain
their reports; synthetic scheduling cannot authorize device acceptance.

Diagnostic refresh adds proxy AABBs keyed by body slot and the frozen central
point/AABB/ray results. These extra C buffers cost `20*bodyCapacity + 32` bytes;
the JS copies cost the same outside linear memory. Per-slot flags distinguish
query membership and proxy validity, and the output retains exact total counts
and ray geometry. `diagnostics.valid` becomes false after stepping, disposal, or
a refresh without diagnostics. C column packing is checked against public
queries and a twin-world replay; JS writes cannot mutate physics. Contacts and
joints still use the existing snapshot buffers. Timed collection remains in
progress.

## Diagnostic drawing

The default and scaled physics scenes share one bounded command builder for
proxy boxes, contact points/normals, joints, sleep/island colors and central
queries. Joint anchors are transformed from the snapshot's body-local frame;
stale endpoint generations fail. Query membership highlights matching proxy
boxes. Display normals are normalized to a 12-buffer-pixel glyph without
changing the raw physics normals. Markers have a three-pixel radius and the
same 32-vertex mesh in every candidate. Lines use filled one-pixel quads in all
three paths: Canvas strokes produced a faint pixel outside the frozen edge
allowance, so the geometry was corrected with that allowance unchanged.

For capacities B/C/J, commands reserve `4B + 2C + J + 6` lines at 20 bytes each,
`2C + 2J + 2` markers at 12 bytes each, four bytes per rendered body color,
and a four-byte-per-slot row map. Storage is allocated once; overflow fails the
frame. WebGL uses fixed GPU storage and uploads active prefixes plus body
colors only on revision changes. The standalone raylib correctness path copies
the same payload, validates finite coordinates and color/count ranges before
drawing, and reports these diagnostic bytes separately from pose bytes. That
prototype copy is absent from the co-located path used for future timing.
Canvas reports API submission calls; its internal GPU draw count is unknown.
For the co-located path, `overlay.c` builds the identical ordered commands from
public C getters and the driver's central queries. It uses double camera and
anchor arithmetic before storing floats to match TS preparation, preserves the
same normal glyph length, validates finite output and GPU arithmetic, and
rejects insufficient worst-case capacities. Its transient arrays live in the
raylib renderer; preparation allocates nothing and cannot mutate physics.
`diagnosticPrepareBytes` reports C writes separately from JS boundary copies.

The shared HUD component uses 4 Hz deadlines and skips missed updates without
catch-up bursts. It displays actual body/constraint/work counters, the fixed
WASM budget, allocator usage, known world/output payloads and available GPU
metrics. Text wraps within the mobile width. Collection includes the HUD update
cost. Frozen render-only diagnostics use the separate immutable command builder
and label the displayed instance count separately from source-world counters.
Canvas caches preserve primitive/color order in paths bounded to 8,192 edges
(256 circles or 2,048 line quads), avoiding a single giant path during setup.

## Offline timing audit

`python3 bench/render/validate.py <completed-report.json>` independently checks
completed collection records, using Python arithmetic rather than the browser's
summary implementation. It checks raw dimensions, exact uint64 counters,
calibration, fixed-step accounting, unavailable GPU metrics, observed GL totals,
nearest-rank tails and every 10-second window. Corrupt, failed and partial records
are rejected; retain their originals for investigation. Timing recomputation
permits only floating-point arithmetic rounding, while budget comparisons use
the unchanged exact inclusive thresholds. The cross-API rAF/callback ordering
check uses the same 1e-9 ms arithmetic allowance: CI observed `6091.7` from rAF
and `6091.6999999999825` from `performance.now()` for the same instant. Ordering
among `performance.now()` samples stays exact, and raw samples are never changed.

The audit reports each continuity/CPU/cadence budget separately, including empty
windows. Diagnostic and synthetic-clock budgets are marked inapplicable; short correctness runs
remain identified. A structurally valid report may still fail numeric budgets.
Every audit remains `acceptanceEligible: false`: provenance authenticity, real
input, device conditions, startup/cache protocol, allocation/GC, presentation,
companion quality runs and the repeated cross-device decision still require
separate evidence. This is a raw timing audit, not complete study qualification.

`python3 bench/render/test_validate.py <runner-report-directory> <assembled-build>`
checks short browser reports for all 12 candidate/profile combinations,
rejects deliberately corrupted copies and failed-run prefixes, and pins exact
budget boundaries. A synthetic stalled timeline from the actual JS producer
checks boundary gaps, empty windows, frozen preparation baselines and overshoot.

## Frame recording

`refreshFrameCounters()` copies 25 uint32 statistics and 26 uint64 last-step/
cumulative work counters into reusable JS arrays (308 bytes per world). The
statistics include the last contact-drop count, current allocator usage and
completed steps. These counters are invalidated by stepping, snapshot refresh
or disposal. Writes to the JS copies cannot mutate physics. The object-based
`report()` remains an allocation outside timed frames.

`FrameRecorder` allocates all arrays once: 24 float64 clock/duration/renderer
metrics, 25 uint32 statistics, 26 uint64 work values, an availability mask and a
completion byte per frame, totaling 505 payload bytes/frame. The duration and
calibration must determine the actual capacity; its outer bound is 300 seconds
at 240 submissions/s plus two boundary frames. Full storage fails collection
without growth or discarding earlier frames. Unknown optional metrics have a
cleared availability bit and must not be interpreted as measured zero.

Append copies the record; finish captures the recording cost and complete CPU
end time separately from submission completion. An unfinished record remains
explicitly pending. Nonblocking GPU query results can fill their original
completed row later, with duplicate or invalid results rejected. Serialization and nearest-rank distributions allocate only
after collection, and uint64 values serialize as exact decimal strings.
Summary cadence uses the frozen missed-slot denominator. The collection loop
integrates scheduling, calibration and finite-state checks, but these primitives
and short browser tests are not acceptance evidence. Provenance, input
collection and report qualification remain required.

Every collected run also has fixed 10-second summaries (30 for the full
300-second sustained profile). Frames are assigned by callback entry; each
incoming submission gap belongs to its arriving frame's window, so a stall
crossing a boundary is counted exactly once. The final boundary/overshoot frame
remains in the last window with its actual time exposed. Summaries retain
nearest-rank CPU/GPU and submission-gap distributions, missing GPU sample counts,
missed slots, executed-step/drop deltas, final debt and peak observed allocator
usage. Contact-drop deltas remain exact decimal uint64 strings. Empty or
unfinished windows expose missing coverage rather than zero-valued timing.
This post-run analysis does not alter frame collection or evaluate acceptance;
full report validation and input/device evidence remain necessary.

## Optional GPU timing

`GpuTimer` uses WebGL 2's `EXT_disjoint_timer_query_webgl2` when available. It
preallocates `ceil(100 / targetPeriodMs) + 2` query objects and fixed row/status
arrays. The capacity derives from the contract's 100 ms gap bound plus two
boundary frames, with an outer limit of 26 at 240 Hz. A full query pool keeps
the frame and records its timing as unavailable; it never grows or overwrites
a pending result. Driver query storage is unavailable; typed storage is exactly
one status byte per frame plus 20 bytes per query, excluding JS object overhead.

Queries enclose `renderer.draw()`, including its GL command production and any
producer gaps. This measures elapsed command span, not GPU utilization or actual
display time. The raylib span also includes its direct C pose/diagnostic
preparation. CPU draw time includes query begin/end overhead. On a later rAF
callback, the collector reads only results whose availability is true, then
checks the disjoint flag before committing them. A disjoint event invalidates
all pending results. Null/invalid results, insufficient counter bits and a
counter's possible wrap are explicitly unavailable. Earlier validated samples
remain intact. Polling cost has its own `gpuPollMs` field and is included in
the complete CPU critical path; renderers perform no query-status polling.

After the last measured submission, the loop drains without drawing for at
most the configured query count of additional callbacks. Unresolved tail
samples are labeled `drain-timeout`; interruptions label pending queries
`aborted`. This does not extend the recorded measurement interval. No busy wait,
`glFinish`, readback or per-frame query allocation/deletion is used. Canvas and
browsers without the extension report unsupported timing. Every attempted frame
has an explicit sample status, alongside the recorder's availability mask.

Behavior follows the [WebGL 2 timer-query specification](https://registry.khronos.org/webgl/extensions/EXT_disjoint_timer_query_webgl2/)
and its [disjoint/counter semantics](https://registry.khronos.org/OpenGL/extensions/EXT/EXT_disjoint_timer_query.txt).
Deterministic fake-driver tests cover pool reuse/exhaustion, delayed and invalid
results, disjoint events, short-counter wrap and cleanup. Browser collection
tests exercise the real extension when exposed; headless GPU results remain
correctness-only and do not establish actual-device performance.

## Submitted GL calls

`GlStats` observes seven core WebGL methods on the candidate's own context:
buffer data/subdata and the array, indexed, instanced and range draw variants.
It is installed before warm-up, retains the original functions/descriptors,
forwards the original buffer argument arity and source identity, and restores
the context on disposal. A context cannot have two observers. Fixed wrappers
perform no GL status queries and allocate no argument arrays or typed-array
views per call. Both WebGL candidates incur the same observation work per API
call, included in measured draw and complete CPU time. The raylib pin and its
batching implementation are unchanged.

Collection records actual submitted draw calls and host buffer bytes requested
through `bufferData`/`bufferSubData`. Numeric buffer allocation has no host
payload; typed-array offsets/lengths use elements, DataView uses bytes, and zero
length means the remaining view, following the [WebGL 2 buffer specification](https://registry.khronos.org/webgl/specs/latest/2.0/).
Uniforms, textures, framebuffer writes, extension multi-draw and driver-internal
traffic are outside this metric. The opaque study candidates use core draws
and buffer uploads during drawing. Canvas internal GL work remains unavailable.
Call counting does not assert that the driver accepted a command; separate
correctness checks retain GL/console error checks.

Warm-up totals are reset before measured draws. Each recorded frame and the
diagnostic HUD use the observed counts, while the report retains their totals
and explicit scope. Invalid byte accounting fails collection. Correctness
captures compare the WebGL candidate's own counters with observed calls, and
retain the existing pixel oracle while observing both GL candidates. Unit
tests pin buffer overload forwarding, element/byte ranges, all draw variants,
counter scopes, and exact descriptor restoration.

The native and WASM C suite checks every physics tier with sleep off/on,
compares every copied descriptor, verifies within-copy joint remapping, and
checks deterministic replay. The one-copy world matches the original fixture
including private replay state. The largest 32,048-body/262,144-contact driver
must allocate and refresh its bounded snapshot. A 10,001-step run demonstrates
that this independent owner crosses the original benchmark limit, then rejects
the next call without changing state or its fixed storage.

## Build provenance

The CMake study targets generate `build.json` after building their dependencies.
They record full Git revision and dirty state, a SHA-256 inventory of current
first-party sources and the frozen contract, actual compiler/version and SDK
commit, build mode, selected compilation commands and the configured link
command. Module/glue/wrapper hashes identify the built inputs. Raylib records
the requested 6.0 tag and hashes of its actual source tree, including local
FetchContent overrides. These generated records stay in ignored build output.

Assembly rejects changed artifacts, stale source inventories, mixed build modes
or mismatched compilers/SDKs. Rebuild both CMake trees after editing study
sources, then rerun `build.py`. The resulting `provenance.json` adds the assembly
revision/dirty state, TypeScript version, contract revision/last commit/hash,
both module build records and hashes/sizes of delivered files. The companion
benchmark retains its separate build record. Hashes exclude the two generated
provenance envelopes themselves to avoid recursive checksums.

Every collected or failed run includes its own copy of this record, parsed
after collection. This is assembly-time provenance, not a fresh browser hash
of served resources, a hermetic-build attestation or proof that the contract
has merged. It does not collect a personal device inventory. Full startup,
device conditions and profiler evidence remain separate requirements. Tests
reject changed binaries, stale source hashes and incomplete inventories, and
cover untracked edits, deletions and ignored report output.

## Correctness scope

The geometry test checks C snapshot row order, changing transforms, all nine
render-only tiers, circle vertices, and both camera profiles. Browser checks
compare every pixel for the three default scenes before and after stepping,
including a further transform update and 16-copy physics tiers, plus the
smallest/largest frozen render-only tiers in base and diagnostic modes, and diagnostic default scenes with
sleep off/on, at 1280×720 and 720×1280 (38 profiles per build). They also check zero repeated pose uploads, exact
changed-pose byte counts, idempotent disposal, disposed-call rejection and
console errors. Pixel readbacks run only in this correctness harness.

The image oracle permits candidate AA differences only where a pixel center
lies within one drawing-buffer pixel of an actual polygon boundary. It derives
that band from shared geometry, rather than from a reference image's color
transitions: Canvas coverage can hide a subpixel gap between same-color bodies.
An analytical edge test pins the one-pixel band and rejects a changed pixel
outside it. All pixels outside the band must match exactly.
The diagnostic band includes actual quad and marker edges with the same
one-pixel radius. Since a one-pixel line fits entirely inside that allowance,
an independent opaque-primitive probe checks exact line/marker center colors
and adjacent background pixels for every candidate. Command tests check
counts, frames, colors, buffer reuse and overflow at one and sixteen copies.
Browser checks also exercise thirteen private raylib validation failures and
successful drawing after repairing the caller-owned buffers.
An additional 56 base/diagnostic profiles compare co-located raylib against Canvas at one
and sixteen physics copies, sleep off/on and both buffer sizes, plus the
smallest/largest frozen tiers in both modes and sizes. They check
initial and updated images, zero JS pose copies, reuse on repeated draws,
unchanged allocator usage, owner disposal and replacement. Poisoned JS snapshot
copies cannot affect direct drawing, and snapshots match the separately stepped
physics module before and after drawing. Poisoned JS diagnostic arrays also
cannot affect direct drawing. Native/WASM C tests pin pose bits, command counts,
query colors, normal length, joint anchors, output bounds and invalid-input
recovery for the same scene/tier/sleep matrix, plus a settled sleeping world.

Still required for #95: complete
sustained protocol reporting; matched render-only/end-to-end timing and input
collection; startup, memory and allocation
records; context-loss/lifecycle recovery; report validators; the complete
five-repeat desktop and physical Android protocol; and an evidence-backed
renderer decision. These prototypes and CI-style screenshots cannot satisfy
those acceptance gates or authorize a production renderer choice.

Implementation references: [WebGL best practices](https://developer.mozilla.org/en-US/docs/Web/API/WebGL_API/WebGL_best_practices),
[Emscripten WebGL optimization](https://emscripten.org/docs/optimizing/Optimizing-WebGL.html),
and the pinned [raylib batching implementation](https://github.com/raysan5/raylib/blob/6.0/src/rlgl.h).
