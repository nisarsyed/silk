# Silk WebAssembly

A bounded, deterministic 2D physics engine. This ES-module package contains
Silk 0.4.0, scalar wasm32 code, an asynchronous loader, and TypeScript types.
It has no rendering or runtime npm dependencies. Install the locally built
archive with `npm install /absolute/path/nisarsyed-silk-0.4.0.tgz`; registry
publication is deferred. Node 22.12+ and modern browsers with WebAssembly,
ES modules, bigint, and BigUint64Array are required. Current package integration
tests do not establish the later physical-device performance/support gates.

```js
import { createSilk, jsonReplacer } from '@nisarsyed/silk';

const silk = await createSilk(); // Independent instance; fixed 64 MiB by default.
const world = silk.createWorld({ bodyCapacity: 128, gravity: { x: 0, y: -9.81 } });
if (!world) throw new Error('Configuration invalid or memory exhausted');
const body = world.createBody({ mass: 1, shape: silk.circle(0.5) });
if (!body) throw new Error('Body creation failed');
world.step(Math.fround(1 / 60));
console.log(world.readBody(body).position);
console.log(JSON.stringify(world.stats(), jsonReplacer));
silk.dispose();
```

## Loading and assets

Keep `index.mjs`, `views.mjs`, generated `silk.mjs`, and `silk.wasm` together.
Default loading resolves the binary relative to the generated glue's URL.
Serve browser modules over HTTP(S), with `.mjs` as JavaScript and `.wasm` as
`application/wasm`. `file://` browser loading is unsupported. A vanilla page
can import the installed `node_modules/@nisarsyed/silk/index.mjs` by URL, or
map `@nisarsyed/silk` to that entry with an import map. No global Module exists.

Use an absolute asset override when relocating or bundling:

```js
import { createSilk } from '@nisarsyed/silk';
import wasmUrl from '@nisarsyed/silk/silk.wasm?url'; // Vite asset URL.
const silk = await createSilk({ wasmUrl });
```

This explicitly exported binary lets a bundler copy/hash it under an arbitrary
base path. Configure a modern JS target (for example Vite `build.target:
'es2022'`). Other bundlers can emit a binary URL with their asset mechanisms.
See [Vite's asset handling](https://vite.dev/guide/assets.html). For plain
browser code use `new URL('./assets/silk.wasm', import.meta.url)`. Relative
override strings resolve in the host's fetch/file context; use an absolute URL
to avoid page-versus-module ambiguity. Cross-origin assets need server CORS.
In Node use a `file:` URL or absolute filesystem path; default package loading
needs no override. HTTP URLs are a browser loading contract, not a Node file
loader contract. Loader rejection mentions the asset URL/memory budget and
retains the underlying failure as `error.cause`.

A dedicated module worker can directly import the entry URL; document import
maps do not apply inside workers:

```js
// worker.mjs, located beside node_modules/
import { createSilk } from './node_modules/@nisarsyed/silk/index.mjs';
const silk = await createSilk();
const world = silk.createWorld({ bodyCapacity: 32 });
if (!world) throw new Error('Cannot initialize world');
world.step(Math.fround(1 / 60));
postMessage({ bodies: world.bodyCount });
silk.dispose();
```

Create it with `new Worker(new URL('./worker.mjs', import.meta.url), {
type: 'module' })`. Every module/world belongs to its creating thread. Send
application commands or retained numeric copies between threads, never handles
or borrowed output buffers. Shared memory/pthreads are not used; cross-origin
isolation is not required by this package.

## Configuration, validation, and ownership

`index.d.ts` lists every method and field. `WorldConfig.bodyCapacity` is required
and positive (up to 65,536). Zero/omitted contacts resolve to four per body;
joints default to zero; substeps default to four (maximum eight). Gravity and
drag default to zero; sleep is opt-in. Zero selects defaults for the speed,
contact/joint tuning, and sleep thresholds: 400, 30/10/3/1, 60/2, and
0.02/0.01/0.5 respectively. `world.configuration` exposes resolved immutable
values. Existing body type/configuration and joints have no invented setters.

Inputs require plain descriptor objects, exact known field names, actual
booleans, bounded integers, and numbers that remain finite after float32
rounding. Malformed calls throw; finite physical rejection returns false/null
without partial simulation mutation. Pool or allocation failure returns null.
Getter calls on stale/foreign handles and calls on disposed owners throw.
Disposal is idempotent. `world.reset()` invalidates all prior body/joint handles
and steppers. Each handle has private world/generation ownership: copying its
numeric index/generation cannot forge one. `isBodyValid`/`isJointValid` test it.
Shapes are immutable canonical values made by `none`, `circle`, `box`, or
`polygon` (3–8 convex vertices), reusable across worlds and modules.

Coordinates use a world frame, radians, seconds, and caller-selected consistent
length/mass units. Body positions are bounded by ±8192; shape extents by 1024.
Ray translations are vectors, not endpoints; origin and endpoint must fit
±9216. Ray origins on/inside a shape miss that shape; closest ties use the
lowest body slot. C computes geometry and rotations affecting physics. Scalar
WASM preserves same-build deterministic replay; cross-platform comparisons
need explicit numerical tolerances. No fast-math or relaxed SIMD is enabled.

`createBody` defaults to dynamic, zero mass/velocities/forces/materials, no
shape, and zero pose. Dynamic bodies require positive mass; a zero-mass dynamic
descriptor rejects. Static/kinematic bodies require exactly zero mass. `readBody`
copies pose, velocity, mass/inertia, force, material, shape, AABB, and activation.
Position/angle, velocity/spin, mass, shape, friction, restitution, force/torque,
force-at-point, and wake operations mirror the existing C contracts. Body rows
use deterministic swap-remove traversal (`bodyAt`, `firstBody`, `nextBody`).

`createJoint({ kind: 'distance', bodyA, bodyB, length })` or `kind: 'revolute'`
accepts optional local anchors and `collideConnected`. Read/count/traverse/test/
destroy with the matching methods. Body destruction destroys attached joints.
`contactAt`, `islandStats`, and `stats` return copied records. Contact point
feature IDs, impulses, persisted flags, and all work counters are retained.
Work counters are bigint; `jsonReplacer` converts them to decimal strings for
JSON without losing values above 2^53. Other counts/byte sizes are exact Numbers.

`step(dt)` performs one fixed step. `createStepper(dt)` and `advance(stepper,
frameTime)` offer the native eight-step bound; stepper `steps`, `remainder`,
and `droppedTime` describe the last advance. A cap can discard accumulated time.
Negative frame time is ignored. Keep a fixed dt and explicitly account for
lost time when measuring continuity; the package supplies no wall-clock loop.

## Queries and snapshots

Point and AABB queries take scalar coordinates, optional type mask, and optional
result capacity (defaults to body capacity). Mask zero includes all types;
`queryMasks` supplies dynamic/kinematic/static/all bits. A result reports total
`count`, `written`, and `truncated`, with an ascending-slot prefix in reused
index/generation arrays. Capacity zero counts only. Capacity above the world's
initial capacity or invalid geometry rejects. `bodyAt(row)` explicitly creates
a world-owned result handle; `copy()` allocates a retained list. Closest ray
queries use separate reusable scalar results (`hit`, fraction, point/normal
components, `body()`, `copy()`). Valid misses return zero geometry and no body;
invalid queries return null or throw without replacing the prior result.

`refreshSnapshot(diagnostics = false)` prepares output in one C call. Then
read `world.snapshot`: packed `bodies` columns below `bodyCount`, and `geometry`
columns keyed by `bodies.index[row]`. Geometry is cached across frames, refreshed
on shape changes, destruction/reuse, and reset. `diagnostics: true` also fills
contact/joint columns and island IDs. Base snapshots use 0xffffffff for island
IDs and zero diagnostic counts. Read only active rows/live geometry slots.

Borrowed arrays and objects are allocated once and reused. They are logically
read-only: **do not write, transfer, detach, or resize them**. They never alias
private WASM memory, even through `.buffer`. Store `revision` and check
`isCurrent(revision)` or `valid` before use. Raw arrays cannot enforce lifetime;
count/scalar accessors throw when invalid. World mutation attempts reaching C,
reset, disposal, and refresh invalidate all borrowed outputs. Successful queries
replace only their own result; read-only getters/queries preserve snapshots.
The same object can become valid again with newer data. `copy()` explicitly
allocates data that survives later operations. Copied handles still obey world
lifetime. Native contact-pointer lifetimes do not govern these copied buffers.

## Bounded memory and costs

`createSilk({ memoryBytes })` accepts a fixed 64 KiB-aligned budget from 2 to
512 MiB, default 64 MiB. No memory growth is possible. Allocate worlds and their
query/snapshot storage during setup. Creation can fail from a small budget,
fragmentation, or host reservation failure; partial setup cleans up. Larger
configured worlds need larger explicit budgets; 64 MiB is not a maximum-world
promise. Browser garbage collection releases the module reservation eventually.

For capacities B/C/J, C snapshot/query payload is exactly `144B + 136C + 60J`
bytes plus the adapter context. Separate reusable JS typed-array payload is
`132B + 136C + 60J`. `world.memory` separates all engine memory categories,
`adapterBytes`, and `outputBytes`; module memory reports committed linear bytes,
stack (1 MiB), static/heap bounds, allocator occupancy, requested bytes, and
live-world JS output payload. JS object overhead and caller-retained copies
are not included. Do not add engine `worldBytes` to adapter bytes: the adapter
already embeds the world shell. Sizing helpers return engine arena or adapter
requests before creation; neither is the whole module budget.

C snapshots traverse live bodies and optional contacts/joints through public
APIs, preparing 48 bytes/body, 76 bytes/changed geometry slot, 136 bytes/contact,
and 60 bytes/joint. JS copies capacity-sized columns: 48B each refresh, 76B when
geometry changed, and 136C/60J when diagnostic counts are nonzero. A successful
point/AABB query copies 8B bytes. These bounded copies avoid per-frame subarray
allocation and prevent public buffers from exposing the full private heap.
Normal snapshot/query paths allocate no arrays, handles, or C storage. Explicit
copies, individual getters, and result-handle conversion allocate.

Intentional C-surface omissions are standalone math helpers, raw pointers,
struct padding/layout, allocators/private solver/tree storage, exact assertion
behavior, and nonexistent engine features (joint motors/limits, etc.). Typed
handles replace writable C handle records. Public enum mappings are `bodyTypes`,
`shapeKinds`, `jointKinds`; `limits` and `queryMasks` describe binding limits.
`defaults` exposes resolved physical defaults; `constants` exposes the float32
math/contact constants used by the public headers. These are inspection values,
not writable tuning settings.
See LICENSE and licenses/ for Silk and compiled-runtime attribution.
