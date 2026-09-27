# Browser host runtime work

This directory starts [#96](https://github.com/nisarsyed/silk/issues/96).
It does not choose main-thread or worker placement, and it does not ship a
browser sandbox yet. The renderer study in `bench/render` remains the source of
matched rendering evidence.

`FixedClock` receives explicit monotonic milliseconds from the host. It plans
only binary32 `1/60`-second steps, executes at most eight per callback, exposes
dropped time and interpolation debt, and records intentionally suspended time
separately. Pause and resume never repay hidden-tab time. Manual steps are
accepted only while paused and are bounded to eight outstanding requests.
Reset clears scheduling state without silently changing the pause state. The
caller executes every planned C step or fails the owner; rendering cannot be
used to conceal missed simulation work.

`PointerQueue` stores at most 256 ordered pointer actions in preallocated typed
columns. Adjacent moves from the same pointer coalesce. Saturation may discard
a move or evict an older move to preserve a later down/up/cancel transition.
If the queue contains only transitions and cannot admit another, it enters an
explicit failed state until a new epoch resets it. Old-epoch messages cannot
affect the replacement world. The sole owner drains actions immediately before
a fixed step; an action failure poisons the epoch to avoid replay. Neither
primitive reads wall-clock time, creates a second world, or requires shared
memory.

Run `node browser/test_runtime.mjs` for deterministic cadence, pause, overflow,
ordering and cancellation checks. Run `node browser/test_wasm_runtime.mjs
build/wasm-release/render-physics` after the WASM build to compare actual C
world snapshots and work reports across 30/60/120/144 Hz callback schedules
with the same queued pointer sequence. These are host correctness tests, not
real-device placement or latency measurements. `node
bench/render/test_placement_browser.mjs build/render-study` also runs actual
dedicated-worker WASM simulation and OffscreenCanvas drawing against the
main-thread candidate for Canvas 2D and WebGL 2, pyramid and chains, sleep
off/on, in both desktop and mobile buffers (16 profiles per build). It checks
final C pose/work and composited pixels without transferring
a world snapshot per frame. The worker's final rAF lets a synchronous replay
reach the compositor before its pixel comparison. This does not select a
renderer or worker placement.

`runtime_owner.mjs` in the assembled study creates one authoritative C world
and matching Canvas or WebGL renderer per canvas. Main and worker contexts use
the same fixed clock and pointer queue. It supports pause, single-step, exact
source reset, desktop/mobile buffer changes, pointer cancellation and disposal.
A lost graphics context stops the owner; recovery creates a new owner and
canvas, never a silent continuation. `runtime_controller.mjs` claims the DOM
canvas before worker startup. If startup fails, it terminates the worker,
replaces the transferred canvas and starts a fresh main-thread world. Mid-run
failure requires an explicit `recoverToMain()` reset. It bounds outstanding
worker controls to 32 and terminates a worker that fails or exceeds that bound.
`pointer_adapter.mjs` binds primary DOM pointer actions to the owned canvas,
maps CSS coordinates through the shared camera, coalesces moves in a 256-entry
queue, and sends at most one bounded batch at a time. Pause/reset epochs discard
stale input and release capture; a saturated transition queue pauses with an
explicit error. The adapter rebinds when explicit recovery replaces the canvas.
Optional `traceInput` preallocates 8,192 host-only records. A trusted DOM event
timestamp travels with each bounded pointer message; the owner records its
application time and the first completed renderer submission containing that
step. `inputTrace()` copies the records only on request, with main and owner
time origins for clock correlation. Reports include the trace storage size;
a full trace fails the run explicitly.
`test_input_trace_browser.mjs` checks event, step and submission ordering in
both renderers and both execution paths. This is a correctness trace, not a
physical-device input-latency measurement or an input-to-photon claim.
Optional `traceFrames` preallocates 131,072 frame records (5,373,952 bytes).
For each submitted frame it records the owner's callback time, scheduler work,
simulation/snapshot work, renderer submission work and executed step count.
The owner exposes one bounded copy on request, and a full trace fails the run.
The main and worker trace share identical stages; the worker's stages use its
own clock and are not added to overlapping main-thread work. These times are
CPU submission proxies, not GPU completion or physical presentation times.
Before stepping the world, each owner measures 240 idle animation intervals
and schedules submissions at the nearest 60 Hz target deadline. It reports the
median interval, target period, divisor, skipped/duplicate callbacks, missed
target slots and timestamp discontinuities. A regressing browser timestamp
triggers idle recalibration and is reported so the affected timed run can be
rejected. Calibration time is recorded as suspended time by the fixed-step clock;
it cannot become catch-up physics work. Resume and drawing-buffer resize
recalibrate. A refresh rate that cannot produce a 59–61 Hz target remains
usable, but `presentation.targetSupported` is false and that run cannot qualify
for the required 60 Hz performance comparison. The synthetic 120 Hz browser
test verifies this pacing against the actual WASM owner and frame trace.
Linux Playwright WebKit exposes WebGL 2 on HTML canvas but may lack WebGL 2 in a
dedicated OffscreenCanvas worker. Browser checks probe the worker context and
require explicit main-thread WebGL fallback when it is unavailable; macOS
WebKit checks exercise the actual WebGL worker path.
Visibility changes pause/resume without repaying suspended time; a user pause
stays paused when the tab returns. `test_runtime_browser.mjs` and
`test_runtime_controller_browser.mjs` and `test_pointer_adapter_browser.mjs`
exercise both actual execution contexts,
unsupported or failed worker startup, context failure, reset, resize, pointer
cancellation and shutdown. Portrait resizing updates the OffscreenCanvas
drawing buffer to 720 × 1280 and the visible CSS frame to 360 × 640. Browsers
may retain or mirror the old HTML placeholder's intrinsic dimensions after
transfer; they do not define the worker drawing buffer. Separate
portrait pixel comparisons verify the result against the main-thread image.
The CSS frame fits either viewport dimension at a fixed aspect ratio while
the drawing buffer stays fixed; reports include actual CSS size, device DPR,
and both buffer/CSS ratios.
These are correctness runs with no physical-device speed claim.
The runtime, controller, pointer and trace smokes run in Chromium, Firefox and
WebKit. A Playwright WebKit pass is not physical Safari/iOS acceptance.

Still required by #96: a real input latency trace, measured worker scheduling and memory on the
reference devices, sustained comparisons, and a supported placement decision.
