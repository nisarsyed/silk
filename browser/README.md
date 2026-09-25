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
off/on. It checks final C pose/work and composited pixels without transferring
a world snapshot per frame. The worker's final rAF lets a synchronous replay
reach the compositor before its pixel comparison. This does not select a
renderer or worker placement. The lifecycle owner, fallback/recovery, controls,
measured worker scheduling, reference-device evidence and placement decision
remain required by #96.
