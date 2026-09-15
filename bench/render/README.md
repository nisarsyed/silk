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
  -DSL_BUILD_TESTS=OFF -DSL_INSTALL=OFF -DSL_RENDER_STUDY=ON
cmake --build build/wasm-render
npm --prefix wasm ci --ignore-scripts --no-audit --no-fund
PLAYWRIGHT_SKIP_BROWSER_GC=1 node wasm/node_modules/@playwright/test/cli.js install chromium
python3 bench/render/build.py
node bench/render/test_study.mjs build/wasm-release/render-physics
node bench/render/test_geometry.mjs
node bench/render/test_overlay.mjs
node bench/render/test_browser.mjs
```

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
  Its base renderer writes 16 pose bytes per body directly in C after a step,
  using stored transforms without trigonometry or snapshot packing. Repeated
  draws reuse those poses. Meshes cross JS once during setup; live pose copies
  across JS are zero. This path still needs direct C diagnostic preparation
  before the end-to-end comparison can be measured fairly.

Shared pose payload is 16 bytes/instance, source rows and tile offsets another
8 bytes/instance, plus cached unique meshes/batches and JS object overhead.
WebGL GPU payload is 16 bytes/instance plus unique triangulated meshes; driver,
VAO and framebuffer overhead is not included. Canvas internal memory/uploads
and raylib's internal GPU uploads/draw counts are currently unavailable, not
zero. Instrumentation and nonblocking GPU timing remain required before the
measurement study.

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
colors only on revision changes. The current separate raylib module copies
the same payload, validates finite coordinates and color/count ranges before
drawing, and reports these diagnostic bytes separately from pose bytes. That
prototype copy must also be removed by the co-located path before timing.
Canvas reports API submission calls; its internal GPU draw count is unknown.

The 4 Hz work/memory HUD and frozen render-only diagnostic replication remain
unfinished. The current overlay builder explicitly rejects frozen scenes.

The native and WASM C suite checks every physics tier with sleep off/on,
compares every copied descriptor, verifies within-copy joint remapping, and
checks deterministic replay. The one-copy world matches the original fixture
including private replay state. The largest 32,048-body/262,144-contact driver
must allocate and refresh its bounded snapshot. A 10,001-step run demonstrates
that this independent owner crosses the original benchmark limit, then rejects
the next call without changing state or its fixed storage.

## Correctness scope

The geometry test checks C snapshot row order, changing transforms, all nine
render-only tiers, circle vertices, and both camera profiles. Browser checks
compare every pixel for the three default scenes before and after stepping,
including a further transform update and 16-copy physics tiers, plus the
smallest/largest frozen render-only tiers, and diagnostic default scenes with
sleep off/on, at 1280×720 and 720×1280 (34 profiles per build). They also check zero repeated pose uploads, exact
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
An additional 24 base profiles compare co-located raylib against Canvas at one
and sixteen physics copies, sleep off/on and both buffer sizes. They check
initial and updated images, zero JS pose copies, reuse on repeated draws,
unchanged allocator usage, owner disposal and replacement. Poisoned JS snapshot
copies cannot affect direct drawing, and snapshots match the separately stepped
physics module before and after drawing. Native/WASM C tests pin pose bits and
output bounds for the same scene/tier/sleep matrix.

Still required for #95: the diagnostic HUD and render-only diagnostic coverage; browser integration
of the sustained measurement loop and co-located raylib driver; matched render-only/end-to-end timing and input
collection; GPU/upload/draw instrumentation; startup, memory and allocation
records; context-loss/lifecycle recovery; source/toolchain/artifact provenance; report validators; the complete
five-repeat desktop and physical Android protocol; and an evidence-backed
renderer decision. These prototypes and CI-style screenshots cannot satisfy
those acceptance gates or authorize a production renderer choice.

Implementation references: [WebGL best practices](https://developer.mozilla.org/en-US/docs/Web/API/WebGL_API/WebGL_best_practices),
[Emscripten WebGL optimization](https://emscripten.org/docs/optimizing/Optimizing-WebGL.html),
and the pinned [raylib batching implementation](https://github.com/raysan5/raylib/blob/6.0/src/rlgl.h).
