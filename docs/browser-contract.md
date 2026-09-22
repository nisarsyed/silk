# Phase 5 browser and performance contract

Contract revision 2, originating in [issue #89](https://github.com/nisarsyed/silk/issues/89).
The [Phase 5 tracker](https://github.com/nisarsyed/silk/issues/88) and
[milestone](https://github.com/nisarsyed/silk/milestone/3) own delivery toward
0.5.0. This document specifies acceptance; it does not claim that a WASM build,
renderer, package, or browser performance result already exists.

The workload and measurement rules below are specified before baseline
collection. Record the merged contract revision in every report. Changes to
workloads, visual settings, or acceptance limits require an explained contract
revision and fresh matched baselines; never adjust them silently after seeing
a candidate fail. Reference hardware, installed versions, access arrangements,
and collection status live in GitHub issues and run artifacts, not this
repository. Changing the test environment requires a new matched baseline,
not an edit to a checked-in device inventory.

Revision 2 clarifies frozen render-only diagnostics before their matched
baselines: retain a contact when any dynamic endpoint is displayed, even if
the other endpoint's shape is omitted, and draw shared contacts once per tile.
The instance query and repeated source colors/proxies are specified below.
Workloads, capacities, timing budgets, quality limits and Phase 5 scope are
unchanged; collect matched results for this clarified diagnostic profile.

## Delivery and support

Deliver the existing 2D rigid-body engine as an independent JavaScript package
with TypeScript declarations, an installable archive, and a GitHub Pages demo.
Native consumers retain the portable C static library and existing build flow.
The package has no renderer dependency and works in a browser main thread,
dedicated worker, and Node consumer. Node is a consumer/test environment, not a
substitute for browser performance evidence.

New simulation modules, CCD/bullets, sensors/events, advanced joints,
serialization, parallel/GPU solvers, WebGPU, npm registry publication, and
GitHub Releases are outside this phase. A browser worker still runs a single
authoritative, single-threaded simulation. Renderer selection remains open
until the matched Canvas 2D/WebGL 2/raylib study in
[#95](https://github.com/nisarsyed/silk/issues/95).

### Browser coverage and test environments

Required acceptance covers desktop Chrome, Firefox, and Safari, plus Chrome
on the available Android reference device. iOS remains a compatibility target;
physical iOS acceptance is deferred until hardware is available and does not
block Phase 5. Report it as unverified, not passed. Chromium/Firefox/WebKit CI
covers repeatable correctness and loading; WebKit CI and viewport emulation
do not certify Safari/iOS or Android hardware.

Keep reference choices and access notes in
[#89](https://github.com/nisarsyed/silk/issues/89), with actual validation status
in [#101](https://github.com/nisarsyed/silk/issues/101). Collect OS/browser
versions, display/refresh, scaling, and graphics-backend metadata when running
tests and retain them with GitHub run artifacts. Do not check personal device
inventories, installed software versions, or environment-specific reports into
the repository. Portable protocol documentation and links to evidence suffice.
Do not include serial numbers or device IDs even in run artifacts.

Use identical environment settings for candidate comparisons, and re-run the
final package/demo on the actual versions used for acceptance. Desktop uses
mains power and targets 60 Hz presentation with the normalization below.
Measure 120 Hz headroom only where available. Installing a missing test browser
and collecting its version are test setup, not prerequisites for completing
this documentation issue or starting the build work.

## Frozen workloads

The source of truth is the fixture construction and mutation sequence in
[bench/main.c](../bench/main.c), inspected at the 0.4.0 release tree
`796e966a7ce26258932f4e90abe2bc968c819621` (merged as
[`b88bbdd`](https://github.com/nisarsyed/silk/commit/b88bbdd745c6a1891eb51e854d042179313a7d0a)).
[#94](https://github.com/nisarsyed/silk/issues/94) extracts these fixtures for
reuse; it must not translate them into independently evolving JS scenes.

All fixtures use `dt = binary32(1/60)` seconds, four substeps, and zero linear
and angular drag. Gravity is `(0, -9.81)` except churn, which uses `(0, 0)`.
All other world tuning resolves the 0.4.0 zero-config defaults in
[world.h](../include/silk/world.h): linear speed cap 400; contact hertz 30,
damping ratio 10, push velocity cap 3; restitution threshold 1; joint hertz
60 and damping ratio 2; sleep speed 0.02, angular speed 0.01, and quiet time
0.5. Units and validation follow that header. Record resolved values, not only
the zero-filled input descriptor.

| Scene | Live bodies (dynamic + static) | Body capacity | Contact capacity | Joint capacity/live joints | Seed | Friction / restitution |
| --- | --- | --- | --- | --- | --- | --- |
| pyramid | 210 + 1 | 211 | 844 (resolved default) | 0 / 0 | `0` | 0.6 / 0 |
| rain | 2000 + 3 | 2003 | 16384 | 0 / 0 | `0xC001D00D` | 0.4 / 0.1 |
| piles | 160 + 1 | 161 | 2048 | 0 / 0 | `0x59B3AC01` | 0.6 / 0 |
| chains | 96 + 8 | 104 | 1024 | 96 / 96 | `0x59B3AC01` | 0.6 / 0 |
| churn | 128 + 0 | 128 | 2048 | 0 / 0 | `0x59B3AC01` | 0.6 / 0 |
| table | 4 + 1 | 5 | 64 | 0 / 0 | `0x59B3AC01` | 0.6 / 0 |
| inverted | 6 + 1 | 7 | 128 | 0 / 0 | `0x59B3AC01` | 0.6 / 0 |

Seeds for non-random fixtures are metadata; do not introduce randomness there.
Rain retains the existing xorshift32 operation order and all four draws per
body. Churn retains four destroy/recreate/move operations per fixed step.
Shapes, masses, initial transforms/velocities, insertion order, and joint
anchors follow the source exactly, including the alternating distance/revolute
chain joints with connected collisions suppressed.

Pyramid, rain, and chains are the default demo scenes. Initial loading/reset
uses those exact counts and capacities. Default sleep is **off**, matching the
native harness; repeat required acceptance with sleep explicitly **on**.
Sleeping cannot conceal the cost of the awake baseline. User spawning is a
separate interaction profile: after reset, choose body capacity 4096, contact
capacity 32768, and joint capacity 96 before building the selected scene.
Report that configuration distinctly; it is not the frozen default benchmark.
Stop spawning visibly at capacity. Scene resets retain their chosen capacities.

### Scaling and visual comparability

Physics scaling uses `1, 2, 4, 8, 16` isolated copies of each default scene
inside one world, multiplying its body/contact/joint capacities by the copy
count. Translate copy `i` by `(32 * (i - (copies - 1)/2), 0)` world units and
create copies in increasing order. Each rain copy restarts the same seed.
These tiers measure increasing independent workloads, not a denser single
pile. The largest rain tier is 32048 bodies and 262144 contact slots; never
raise engine limits to add a tier. Report the largest **tested passing** tier;
do not interpolate an unmeasured maximum. Run all finite tiers, recording
capacity, allocation, quality, and timing failures separately.

Render-only scaling uses `256, 512, 1024, 2048, 4096, 8192, 16384, 32768,
65536` instances. Repeat the default rain transform/shape snapshot in row
order, translated into nonoverlapping 32-unit-wide tiles; omit static shapes
and trim the final tile to the requested count. All candidates consume the
same copied snapshot after 120 fixed steps with sleep off. Rendering this
frozen state isolates submission/fill cost; it is not a physics throughput or
dynamic-upload claim. The end-to-end profiles below cover changing transforms.

For frozen diagnostics, repeat the source proxy bounds and sleeping/island
colors with each displayed dynamic row; island IDs/colors remain literal
copies of the source snapshot. Within each tile, retain each source contact
with at least one displayed dynamic endpoint, including contacts against
omitted static shapes or bodies trimmed from the final tile. Draw it once,
with its original point order and translated points/normals. Contacts with no
points produce no glyphs. Run the single central point/AABB/ray query over the
displayed instances using the union camera rectangle, all body types and full
instance output capacity. Equal-fraction ray hits choose the first instance
in draw order. The frozen source world is not stepped during rendering.

| Profile | Nominal CSS frame | Fixed drawing buffer | Nominal render DPR |
| --- | --- | --- | --- |
| Desktop | 1280 x 720 | 1280 x 720 | 1 |
| Mobile portrait | 360 x 640 | 720 x 1280 | 2 |

Fit the CSS frame into available space without changing its aspect ratio or
the fixed drawing buffer. Record actual CSS dimensions, device DPR, and the
resulting buffer/CSS ratio. Disable dynamic resolution in timed comparisons.
Landscape/native-DPR operation is tested for usability and reported separately.

Use an orthographic camera fitting these world rectangles: pyramid
`[-13, 13] x [-1, 22]`; rain `[-9, 9] x [-1, 25]`; chains
`[-1, 29] x [-1, 16]`. Add the union of translated rectangles for scaling;
letterbox rather than crop or stretch. Keep every body visible. Use an opaque
flat background, flat body fills, no textures, shadows, grid, or body outlines
in the base comparison. Draw circles as regular 32-sided polygons in every
candidate, with the first vertex at local positive x. Physics remains circular.
Candidate-specific edge antialiasing may differ within one drawing-buffer
pixel; body geometry, fill, ordering, and coverage must otherwise agree.

Run two separate visual profiles: base (all debug overlays off) and diagnostic
(contacts/normals, proxy AABBs, joints, sleeping/island colors, a fixed central
point/AABB/ray query, and work/memory counters on). For the diagnostic query,
use the camera rectangle center, a square with half-extent 1, and a horizontal
ray through the center across the rectangle; mask all body types and allocate
query output for every body. Render lines at one drawing-buffer pixel and
markers at radius three pixels. Update the text HUD at 4 Hz; retain every
frame's numeric data in preallocated report storage. Compare identical overlay
content, ordering, and refresh rates; do not omit costly diagnostics from one
candidate. Default 60 Hz performance is gated on base; diagnostic results are
mandatory reports, with correctness required but no separate speed gate.

## Measurement and acceptance protocol

### Runs and provenance

Keep the native full-quality profile unchanged: 120 warm-up steps, 600 measured
steps, five repetitions, seven scenes, sleep off/on, and the existing
[quality limits](../bench/quality_limits.json). Native schema 4 and physical
oracles remain intact. Browser reports add their own versioned envelope and
identify which profile was run; smoke output cannot satisfy full-quality gates.

For each browser/device/default scene/sleep mode/visual profile, initialize a
disposable scene and exercise 120 steps and 120 render callbacks before the
measured run to warm code, shaders, and buffers. Then rebuild the exact initial
state, start with zero scheduling debt, and measure **60 seconds** of foreground
execution. Include initial falling motion, actual changing-transform uploads,
and the settled period; do not carry the warm-up world's settled state into
the measurement. Run five repetitions. Also run all seven scenes as step-only
WASM benchmarks and measure bulk snapshot preparation separately.

For sustained mobile acceptance, run rain with sleep off and the base visual
profile for **300 seconds**, five times per candidate/device, after a **60-second**
foreground warm-up on a disposable rain scene. Rebuild before timing; do not
reset or cool the device during the measured interval. Report 10-second windows
as well as whole-run distributions. This exposes thermal/tail behavior even
when a scene settles; no inference about GPU temperature is made when sensors
are unavailable. Keep the five-minute result separate from native 600-step
quality metrics; run the unchanged full-quality profile too.

Browser sustained runs can exceed the native CLI's 10000-step limit. They use
a separate bounded browser driver; preserve the native CLI limit. Allocate
measurement storage before each run, with explicit bounds for the selected
duration, submission rate, and fixed-step count; a full buffer fails the run
instead of reallocating or silently losing samples. Full-quality native-style
profiles keep their original 120/600 counts.

Scaling profiles run five 60-second repetitions per tier after the same warm-up.
For renderer study repetitions rotate candidate order as `A/B/C`, `B/C/A`,
`C/A/B`, `A/C/B`, `C/B/A`, where A=Canvas 2D, B=WebGL 2, C=raylib. For a
two-build optimization alternate baseline/candidate order between repetitions.
Use the same order across workloads and record it. Desktop is plugged in;
mobile is unplugged, starts each candidate repetition at 50-90% battery with
low-power mode off, and cools idle for at least five minutes between candidates.
Use the same screen brightness (50%), no downloads/background apps, and a
recorded ambient temperature. Start from normal thermal state when the OS
reports it; record that the state is unknown otherwise. Preserve any failed
run; re-run a disrupted measurement without deleting the original record.

Every report records full source/contract revision, dirty state, toolchain and
flags, artifact checksums, resolved settings, scene/seed, capacities, sleep,
browser/OS/device/display, worker mode, graphics backend, resolution, timing
precision, warm-up/duration/order, battery/power/thermal conditions, quality,
contact drops, memory, work, bytes uploaded, draw counts, and available profiler
evidence. Do not run interactive developer tools during timed collection.

### Clocks and numerical gates

Use monotonic browser timing outside C. Measure local intervals in one clock
domain; correlate input and worker timestamps using recorded time origins,
never by subtracting unrelated raw clock values. Calibrate idle refresh from
240 requestAnimationFrame intervals before warming the workload. Let `V` be
their median and `T = V * max(1, round((1000/60)/V))` milliseconds be the target
presentation period. Accept calibration only when it supports 59-61 Hz target
presentation; otherwise fix/record the display prerequisite before the required
60 Hz test. Recalibrate after display/refresh changes. Render at target
deadlines even on a higher-refresh display; physics always uses the fixed dt.

The following are engineering acceptance budgets established before profiling,
not measurements of current performance. Apply gates to **every** valid base
run on every required device/browser, including both sleep modes and each
10-second window of the sustained mobile run. Quantiles use nearest rank.

| Metric | Required gate | Meaning |
| --- | --- | --- |
| Simulation continuity | Zero dropped accumulator time; zero contact drops; finite state; final scheduling debt less than one fixed step | A high render rate cannot hide slow-motion physics; executed steps plus remainder must account for elapsed scheduled time |
| Application CPU critical path | p95 <= `0.8*T`; p99 <= `T` | Leaves 20% of one refresh period for browser/compositor variability; time step+snapshot+upload/submission, excluding asynchronous GPU execution |
| Submitted-frame cadence | p95 gap <= `1.25*T`; p99 gap <= `2*T + 1 ms`; no gap > 100 ms | Allows bounded refresh jitter without accepting recurring stalls |
| Missed target slots | <= 1% | Sum `max(0, round(gap/T)-1)` divided by submitted intervals plus missing slots |
| Input to visible-state submission | p95 <= 50 ms; p99 <= 100 ms | Three/six 60 Hz frame budgets from input event timestamp to submission of the first frame containing its effect; not a physical input-to-photon measurement |

Measure a worker's complete critical path on its own thread; report main-thread
UI/event work separately rather than summing overlapping work. Capture CPU
submission completion timestamps consistently for every renderer. rAF callback
intervals and submission cadence are presentation proxies, not proof of actual
displayed GPU frames. Collect nonblocking GPU timing/presentation traces where
available, and check real-device visible motion for repeated stalls. An
observed presentation defect still fails acceptance even if CPU gates pass.
Unavailable GPU metrics are explicitly null/unavailable; never use glFinish,
blocking readbacks, or CPU submission time as fabricated GPU duration.

For each default scene, additionally collect at least 100 real pointer samples
across 20 drag gestures in a separate 60-second base run. Reuse the input-to-step
sequence across renderer candidates where automation supports trusted input;
otherwise preserve each manual sequence and report it as interaction evidence,
not an identical physics trajectory. Correlate each input with the first
submitted frame containing the applied change. Disclose event timestamp
precision; if it cannot support the gate, collect an independent latency trace
rather than marking the metric passed. Synthetic-event tests are correctness
smokes only. Pause/resume/context-loss tests are separate from timed runs;
an unexpected interruption invalidates the timed run and remains in the record.

The host accumulator caps catch-up at eight steps per rendered frame, matching
the existing stepper limit; it exposes any dropped time. Background/resume may
discard suspended time explicitly and restart measurement. Within uninterrupted
acceptance runs dropped time fails. Do not change dt, substeps, solver tuning,
sleep policy, resolution, or body count to hide a failure. High-refresh
interpolation never feeds state back into C.

Measure 120 Hz rendering only on a device/display that actually supports it,
using a target period derived from its calibrated cadence and the same relative
CPU/cadence budgets. Physics stays at 60 steps/s. Report pass/fail/headroom; it
is not a release gate and cannot be certified by running an unpaced loop on a
60 Hz display. Unavailable hardware is reported as not measured.

### Selection and optimization decisions

[#95](https://github.com/nisarsyed/silk/issues/95) compares all three renderer
candidates with the same geometry and profiles. Each must pass visual,
correctness, lifecycle, and support checks before speed can justify adoption.
Reject unsupported or failing candidates with evidence. If none pass required
60 Hz/device gates, record a blocker and investigate; do not quietly select a
failing candidate as the completed result.

Among qualifying candidates, prefer a candidate with no material regression on
any required device and an improvement in sustainable tier or frame-time tails.
A timing improvement must exceed both 5% and the larger relative run-to-run
range of the two candidates' per-run medians. The 5% floor is a decision policy
against marginal complexity, not a statistical confidence claim; retain every
raw distribution. Tier improvements must pass all five runs. If candidates
remain indistinguishable, select the smaller implementation by production
first-party nonblank source lines plus required runtime dependencies, excluding
vendored/generated code from line counts but reporting dependency/download size
separately. If criteria conflict across devices or simplicity cannot break a
tie, document the tradeoff for Nisar's decision before closing the study.

[#96](https://github.com/nisarsyed/silk/issues/96) applies the same evidence
discipline to main-thread versus co-located worker simulation/rendering.
[#99](https://github.com/nisarsyed/silk/issues/99) repeats matched comparisons
for compiler/LTO/SIMD, boundary, renderer, and memory changes. A supported
rejection completes an experiment, but does not waive release performance
gates. CI has no machine-speed threshold. Same-build repeats preserve exact
deterministic fields; cross-build comparisons use explicit physical limits,
not an equal-hash requirement or blind replacement of expected results.

## Public package coverage and ownership

The API inventory below is a binding requirement, not permission to redesign
the core. C exports keep the `sl_` prefix and use explicit scalar/fixed-width
marshalling. Declarations and runtime exports are checked in
[#93](https://github.com/nisarsyed/silk/issues/93). All existing world-config
fields, enum values, bounds, zero defaults, units, and validation remain
available; consumers may select a fixed dt/substep configuration other than
the demo's frozen performance profile.

| Existing public surface | Required package behavior |
| --- | --- |
| `sl_version`, public version/limit/enum constants | Expose matching engine/package version and documented limits/types |
| `sl_world_init`, `destroy`, `reset`, `memory_bytes`, `memory_breakdown_get` | Typed configuration, owning world lifetime, recoverable creation failure, exact sizing and copied diagnostics |
| World gravity/drag/substep/speed getters and `sl_world_get_stats` | Copied configuration and all current/last-step/cumulative statistics |
| Body create/destroy/is_valid/count/capacity/first/next/at | World-scoped generational handles; deterministic traversal and swap-remove behavior |
| All `sl_world_body_get_*` and existing body setters | Position, rotation/angle, velocity/spin, mass/inertia, shape, materials, force/torque, proxy AABB and island diagnostics; no invented setter for currently immutable configuration/type |
| Force, torque, and force-at-point operations | Existing finite/overflow/type validation and atomic accumulation |
| Body is_awake/wake and config sleep fields | Existing opt-in sleep semantics and wake propagation |
| All `sl_world_joint_*` operations | Distance/revolute descriptors, handles, traversal, destruction, count/capacity, validity, impulse reads; no limits/motors or nonexistent joint setters |
| World contact count/capacity/drop_count/at | Copied or adapter-buffer snapshots of ordered manifolds, feature IDs, and impulse data |
| World point/AABB/ray queries | Explicit type mask, ordered bounded output, total count/truncation, valid miss and invalid-input distinctions |
| Shape none/make_circle/make_box/make_polygon/is_valid | Validated shape values with no persistent C pointer ownership; up to eight polygon vertices |
| Shape mass_data/aabb/contains_point/ray_cast | Existing public geometry and hit/miss/unchanged-output semantics |
| `sl_world_step` | Synchronous fixed stepping after async initialization; preflight invalid dt before assertion-sensitive C calls |
| `sl_stepper_init`, `sl_world_advance` | Supported explicit bounded-advance convenience, with host-visible dropped-time accounting; do not use its return count alone to claim continuity |

Intentional omissions: arbitrary standalone math.h function bindings (consumers
can use ordinary JS math and returned vectors/transforms); raw C pointers,
allocator/internal arena access, struct padding/layout, private solver/tree
APIs, and exact C assertion behavior. Null/validity handle helpers are represented
by typed package handles/validity operations rather than writable C records.
Geometry and body rotation are still computed by C where they affect physics.

- The async module factory creates independent modules; each world owns its
  engine and adapter storage. No global Module singleton or borrowed stack
  pointers escape. Disposal is idempotent; calls on a disposed owner fail
  before entering C. Reset invalidates all prior body/joint handles.
- Keep index and generation as separate uint32 values with a private JS owner
  token. Never pack them into a number or accept an unrelated world's equal
  numeric pair. Getter/traversal calls preflight handles and row bounds;
  invalid usage raises a wrapper error instead of triggering C assertions.
- Validate input types without string coercion; range-check integers before
  narrowing; require both the supplied number and its float32 conversion to
  be finite. Then apply C validation, including derived inverse/softness terms.
  Setters/queries preserve false-without-partial-mutation behavior; failed
  creation returns a null result. Module loading rejects with an actionable
  error. Treat disposed owners and malformed API call shapes as programmer
  errors; do not invent a large status enum.
- Expose uint64 work counters as bigint, JSON-encoded as decimal strings;
  wasm32 byte counts fit exactly in numbers. Preserve query total counts,
  ordered prefixes, ray misses, and output-unchanged rejection semantics.
- Bulk frame state is SoA in adapter-owned staging buffers populated through
  public APIs. Borrowed views are logically read-only and never alias private
  authoritative state. Invalidate them on the next non-const world operation,
  refresh, reset, or disposal; offer an explicit retained-copy operation.
  Keep shape caches keyed by handle generation and refresh/invalidate on
  shape replacement, destroy/reuse, or reset. Do not retain a native shape or
  contact pointer beyond its narrower documented C lifetime.

## Memory, loading, and runtime boundaries

Use Emscripten wasm32 6.0.9 initially, with separate debug/release artifacts and
recorded compile/link flags. No fast-math, relaxed SIMD, shared-memory threading,
or nonportable core intrinsics. Compiler-generated SIMD is a measured study;
the scalar path remains supported. Native builds remain independent of the
WASM/JS toolchain.

The initial package uses fixed module memory (`ALLOW_MEMORY_GROWTH=0`) and
recoverable allocator failure (`ABORTING_MALLOC=0`). Provision the module
budget at factory initialization, then all world/adapter/query/snapshot storage
at world initialization. Default module budget is 64 MiB; consumers can request
a 64 KiB-page-aligned budget up to 512 MiB, subject to actual device allocation
success. The default is a bounded starting budget for the three small default
fixtures, not a promise to fit maximum-capacity worlds or every scaling tier.
512 MiB leaves substantial room above the existing 108 MiB maximum native
world budget for bounded snapshots and runtime overhead while avoiding a
multi-gigabyte mobile reservation. Exact wasm32 overhead and capacity checks
must be measured/pinned by [#90](https://github.com/nisarsyed/silk/issues/90)
and [#91](https://github.com/nisarsyed/silk/issues/91) before calling the package
usable. Fixed-memory compilation must retain the documented factory budget
override; reject budgets below static/stack requirements before world creation.

Publish required engine arena bytes, adapter/query/snapshot bytes, module
static/stack/allocator overhead, alignment, and total committed linear memory
separately. Do not imply that `sl_world_memory_bytes` includes the entire
module. Validate arithmetic before allocation; clean up failed partial setup.
If a budget cannot fit a requested configuration, fail recoverably before
altering an existing world. Choose a larger module explicitly for a scaling
tier, recording the same budget across candidates; no growth during a run.

Allocate no memory in C stepping, normal body/joint mutations, queries, or
snapshot refresh. Keep the steady-state JS/render path free of per-body object
creation and reuse typed arrays/GPU buffers. Convenience handle objects,
user-requested retained copies, and report serialization are explicit JS
allocations outside the timed steady-state path. GC costs during an end-to-end
run still belong in its result.

The archive contains the binary, ES-module glue/wrapper, declarations,
version metadata, README, and licenses. It supports independent browser,
worker, Node, and bundler consumers, async asset-location overrides, and clear
missing/corrupt-asset errors. Test a packed archive outside the repository.
GitHub Pages serves the tested main artifact under the project subpath; module,
worker, and WASM URLs must work there without isolation headers. Publishing the
demo does not authorize npm or GitHub Release publication.

The worker study keeps simulation and graphics together, with bounded ordered
input and coalesced pointer movement. It must preserve press/release/cancel,
enforce queue capacity, and expose overload. Never run two authoritative owners
or send full-world state between threads each frame. A tested main-thread path
handles unsupported worker startup. Mid-run failure may require an explicit
scene reset; no silent state reconstruction or fabricated seamless recovery.

## Acceptance record

Close #89 after the protocol and roadmap links are reviewed and merged. Its
GitHub record owns the reference-device choices and the deferred physical iOS
coverage; no checked-in inventory or advance browser installation is required.
The first baseline report records the merged contract revision. Renderer,
worker, and build results do not belong here as unearned claims.

Collect physical evidence and environment details during
[#101](https://github.com/nisarsyed/silk/issues/101). Actual desktop and Android
performance gates remain required; missing access to a required test environment
blocks that validation, not completion of this documentation work. Describe
unverified iOS coverage clearly in the release support statement.

## Sources and verification

Repository contracts: [world](../include/silk/world.h),
[step](../include/silk/step.h), [shape](../include/silk/shape.h),
[queries](../include/silk/query.h), [native benchmark guide](benchmarks.md), and
[physical limits](../bench/quality_limits.json). The source table, capacities,
seeds, and defaults above were checked against the 0.4.0 implementation.

Browser clock/callback behavior is documented by
[requestAnimationFrame](https://developer.mozilla.org/en-US/docs/Web/API/Window/requestAnimationFrame)
and [performance.now](https://developer.mozilla.org/en-US/docs/Web/API/Performance/now).
Use the [Emscripten settings reference](https://emscripten.org/docs/tools_reference/settings_reference.html)
for fixed memory and recoverable allocation settings, and verify those settings
against the pinned SDK when implementing. Numeric frame and input budgets are
Silk engineering policy, not thresholds claimed by these references.

This is documentation-only work: validate relative/GitHub links, source
consistency, and `git diff --check`. No C behavior, version, tolerance, or
benchmark result changes in #89.
