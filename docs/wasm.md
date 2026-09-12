# WebAssembly toolchain

The opt-in `wasm-debug` and `wasm-release` presets compile the portable core
and complete C test runner for scalar wasm32. They also build the
JavaScript adapter; they do not build a browser UI.
Native builds and installed C consumers need neither Emscripten nor Node.
The [browser contract](browser-contract.md) defines the remaining package and
browser acceptance work.

## Setup

Install the pinned SDK outside the source tree. This uses Emscripten's
[CMake toolchain](https://emscripten.org/docs/compiling/Building-Projects.html)
through a small wrapper that diagnoses missing or mismatched installations.

```sh
git clone https://github.com/emscripten-core/emsdk.git /path/to/emsdk
git -C /path/to/emsdk checkout 5eb0bde7585670252e8ba05e9d361627bffd08b5
/path/to/emsdk/emsdk install 6.0.9
/path/to/emsdk/emsdk activate 6.0.9
. /path/to/emsdk/emsdk_env.sh

cmake --preset wasm-debug
cmake --build --preset wasm-debug
ctest --preset wasm-debug
node wasm/test.mjs build/wasm-debug/package/index.mjs

cmake --preset wasm-release
cmake --build --preset wasm-release
ctest --preset wasm-release
node wasm/test.mjs build/wasm-release/package/index.mjs
```

The SDK tag is `6.0.9`, commit
`5eb0bde7585670252e8ba05e9d361627bffd08b5`. Its release mapping pins compiler
artifacts to `f04ea239d533260dd1db760dd2d668d5f9a88d6b`.
The activated SDK supplies Node through `EMSDK_NODE`; otherwise an executable
`node` or `nodejs` must be on PATH at configure time. Adapter builds also need
a Python 3 interpreter, which Emscripten itself requires. On Windows activate
with `emsdk_env.bat` and select an
installed Make-compatible generator (or override with `-G Ninja`). When
switching SDK or runtime, remove only the affected `build/wasm-*` directories
and configure again. A missing SDK, wrong version, or missing Node produces a
configure error with setup instructions.

CMake sets `CMAKE_CROSSCOMPILING_EMULATOR` to Node before creating executable
targets. CTest retains the single `all` registration and propagates the
runner's exit status. Artifacts are `build/wasm-<mode>/bin/sl_tests.js` and its
sibling `.wasm`; run the JavaScript file through Node to inspect individual
case output. Keep both files together.

## Baseline flags and memory

Both presets use C17, `C_EXTENSIONS OFF`, and the unmodified `silk_warnings`
target. SDK defaults supply `-g` in Debug and `-O3 -DNDEBUG` in Release. No
fast-math, SIMD, relaxed SIMD, pthreads, LTO, or graphics flags are added.
The Node test executable alone receives:

```text
-sENVIRONMENT=node -sALLOW_MEMORY_GROWTH=0 -sABORTING_MALLOC=0
-sINITIAL_MEMORY=268435456 -sSTACK_SIZE=1048576 -sEXIT_RUNTIME=1
Debug:   -sASSERTIONS=2 -sSTACK_OVERFLOW_CHECK=2
Release: -sASSERTIONS=0
```

The test module reserves 256 MiB with a 1 MiB stack, accommodating tests that
construct multiple worlds and the maximum-capacity allocation checks. Ordinary
allocator exhaustion returns null. This is a test budget, separate from the
adapter's configurable 64 MiB default. No engine step gains
an allocator or platform dependency.

`compile_commands.json` records every compilation command; the Make generator's
`tests/CMakeFiles/sl_tests.dir/link.txt` records the exact link command. Capture
these, `emcc --version`, `node --version`, source revision/dirty status, and the
SDK commit alongside run results. Do not check machine-specific reports into
the repository. Compiler/linker studies must compare against this scalar
baseline and follow the frozen physical limits.

## wasm32 audit

The core stores entity counts and handle words as fixed-width integers.
`size_t` and pointers are 32 bits on wasm32; no pointer is a JS handle. Arena
carving uses 8-byte boundaries with compile-time alignment gates. Every
bounded body/tree slice fits wasm32; contact/joint slice multiplication and
summation check `SIZE_MAX`, and world creation rejects invalid capacities
before allocation. Existing exact budget tests cover small and maximum worlds,
including joint storage. Struct layout differences must be derived from the
actual pointer-bearing shell sizes, with the payload budgets kept exact.
In particular, the test's original 56-byte `sl_tree` header becomes 44 bytes
on wasm32 and occupies 48 aligned bytes. The test derives this difference
from its three pointers and eight uint32 fields, preserving the exact payload
totals and the existing 97/108 MiB maximum arena limits.

The full suites exercise finite input validation, pool exhaustion,
destroy/reuse/reset, deterministic replay, queries, memory accounting, and
allocation-free simulation behavior. Invalid direct stepping is a programmer
error in Debug; Release additionally tests its no-mutation fallback. The
JS adapter preflights these inputs before calling C.

## JavaScript adapter

The presets emit `build/wasm-<mode>/package/index.mjs`, `views.mjs`, `silk.mjs`,
and `silk.wasm`. The wrapper covers the runtime surface in the
[browser coverage inventory](browser-contract.md#public-package-coverage-and-ownership):
world/body/shape operations, distance/revolute joints, point/AABB/closest-ray
queries, contacts, activation/island diagnostics, exact counters, memory
breakdowns, fixed stepping, and bounded advance. The inventory's intentional
omissions remain: standalone math helpers, private storage/allocators, and
nonexistent engine features. Declarations and independent archive consumers
are subsequent package work; these files are not yet the completed package.

```js
import { createSilk } from './build/wasm-release/package/index.mjs';

const silk = await createSilk({ memoryBytes: 64 * 1024 * 1024 });
const world = silk.createWorld({ bodyCapacity: 128, gravity: { x: 0, y: -9.81 } });
if (!world) throw new Error('Invalid configuration or insufficient memory');
const body = world.createBody({ mass: 1, shape: silk.circle(0.5) });
if (!body) throw new Error('Body creation failed');
world.step(Math.fround(1 / 60));
console.log(world.readBody(body).position);
silk.dispose();
```

Creation returns null on validly typed but physically invalid descriptors or
insufficient capacity/memory. Setters/step return false on physical rejection.
Malformed types (including float32 overflow) throw before mutation. Unknown
descriptor fields also throw, catching misspellings. Numbers are never parsed
from strings. Getters reject stale, forged, and cross-world handles. Reset
invalidates old handles; disposal is idempotent and disposes owned worlds.
Independent module instances have independent memories. `wasmUrl` overrides
asset location; loader failures include the original cause.

`world.configuration` contains resolved defaults, `world.memory` distinguishes
engine arena, adapter requests, and JS output payload bytes, and `silk.memory` reports module/allocator
accounting. Fixed budgets and the pinned SDK's import-limit adjustment are
specified in the [private marshalling protocol](../wasm/protocol.md).

Compile the portable adapter into native tests for sanitizer verification:

```sh
cmake --preset sanitize -DSL_BUILD_WASM_ADAPTER=ON
cmake --build --preset sanitize
ctest --preset sanitize
python3 wasm/test_provision_memory.py
```

The same option works with native debug/release and adds no JS or SDK
dependency. CI runs the native adapter suite and both WASM modes; JS tests
cover malformed input, exhaustion/recovery, isolated instances, shape queries,
exact replay, fixed-memory growth rejection, and reset/disposal lifetimes.

## Bulk output and queries

`world.refreshSnapshot(diagnostics = false)` prepares a bounded snapshot and
returns success. Read `world.snapshot` to reuse the same object and typed arrays
on every frame. Transform/state columns use packed body rows; geometry uses
body slots: `snapshot.geometry.radius[snapshot.bodies.index[row]]`. Only read
rows below the reported body/contact/joint counts. The default skips contact,
joint, and island diagnostics; pass `true` to include them. Geometry is prepared
only after creation, shape replacement, destruction/reuse, or reset.

```js
world.refreshSnapshot();
const view = world.snapshot;
const revision = view.revision;
for (let row = 0; row < view.bodyCount; ++row) {
  const slot = view.bodies.index[row];
  // Submit view.bodies.x/y[row], cos/sin[row], and geometry columns at slot.
}
const retained = view.copy(); // Explicit allocation; survives subsequent calls.
world.step(Math.fround(1 / 60));
console.assert(!view.isCurrent(revision));
```

Borrowed outputs are logically read-only. Their buffers are JS-owned copies;
even constructing a writable view over `.buffer` cannot access the WASM heap.
Do not transfer, detach, resize, or modify borrowed buffers. Raw typed arrays
cannot enforce invalidation: check `valid`/`isCurrent(capturedRevision)` before
use, or call `copy()` to retain data. Count/scalar accessors and copying throw
while invalid. Refresh reuses the same object, so retaining only the object
without its revision does not identify an earlier frame.

World mutations, reset, disposal, and snapshot refresh invalidate all borrowed
outputs. A physically rejected mutation may also invalidate them, but cannot
change simulation state. Malformed JS input throws before mutation. Read-only
getters and queries preserve the current snapshot. Successful point/AABB
queries replace their shared query result; ray queries replace a separate ray
result. Invalid queries return null (or throw for malformed types), preserving
prior query output and revision. These lifetimes are independent of native
contact-pointer lifetimes; copied contact records never alias those pointers.

`queryPoint(x, y, typeMask = 0, capacity = bodyCapacity)` and
`queryAabb(lowerX, lowerY, upperX, upperY, typeMask = 0, capacity = bodyCapacity)`
return reused arrays `indices`/`generations`, total `count`, `written`, and
`truncated`. Zero capacity is a count-only query; insufficient result capacity
returns the deterministic ascending-slot prefix and explicit truncation.
Capacity above world capacity rejects. Mask zero includes all body types;
`queryMasks` exposes the individual bits. `bodyAt(row)` creates an owning-world
handle for a written row; `copy()` explicitly allocates a retained handle list.
A query result never grants access to a different world or prior reset lifetime.

`queryRay(originX, originY, translationX, translationY, typeMask = 0)` returns
closest-hit scalar fields and `body()` for an explicit handle. A valid miss has
`hit === false`, zero geometry, and a null body; invalid input returns null.
Use a translation vector, not an endpoint. Scalar query arguments and reused
output arrays avoid allocations on the hot path.

`createJoint` accepts `kind: 'distance' | 'revolute'`, `bodyA`, `bodyB`, optional
local anchors and `collideConnected`, and distance `length`. Joint traversal,
validity, destruction, copied descriptors, and linear impulses follow the same
world/generation ownership rules as bodies. Destroying a body invalidates its
attached joints. `contactAt` and `islandStats` provide copied diagnostics;
`stats()` includes all current/step/cumulative fields. Work counters are
`bigint`; use `JSON.stringify(world.stats(), jsonReplacer)` to encode them as
exact decimal strings. Import `jsonReplacer` from `index.mjs`.

`createStepper(dt)` creates world-owned fixed-step state. `advance(stepper,
frameTime)` uses the native eight-step bound, with `steps`, `remainder`, and
`droppedTime` describing the last advance. Negative frame time is ignored;
nonfinite input rejects. Reset invalidates old steppers. This convenience can
drop time at its explicit bound; the browser scheduling study must account for
that loss rather than interpreting the returned step count as continuity.

Initialization reserves every query/snapshot column at configured capacities.
The [protocol](../wasm/protocol.md#bulk-layout-cost-and-lifetimes) records exact
payload sizes, traversal work, and copy bandwidth. Native replay tests compare
otherwise identical worlds with and without read-only queries/snapshots; JS
tests verify output identities, allocator occupancy, retained copies, cache
invalidation, public-getter agreement, and values beyond 2^53.
