# Private adapter marshalling protocol

This ABI belongs to the wrapper and is not installed as a C consumer API.
The adapter uses public Silk headers only. Pointers are private wrapper state;
JavaScript never reads C structs, padding, or private SoA storage. Every context
owns 32 input floats, 16 input uint32 words, 64 output floats, 32 output uint32
words, and a canonical scratch shape. These arrays are reused synchronously.
Outputs are copied before the next call can overwrite them. Rejection leaves
simulation state unchanged; staging input is not simulation state.

| Operation | Input float32 order | Input uint32 order |
| --- | --- | --- |
| World sizing/init | gravity x/y, linear/angular drag, linear speed cap, contact hertz/damping/push cap, restitution threshold, joint hertz/damping, sleep linear/angular speed/time | body/contact/joint capacity, substeps, sleep enabled (0/1) |
| Body create | position x/y, velocity x/y, mass, angle, angular velocity, friction, restitution | body type |
| Shape polygon | vertex x/y pairs (3–8) | count is an explicit scalar argument |
| Load canonical shape | radius, eight vertex x/y pairs | kind, vertex count |
| Geometry operations | position x/y, angle; point x/y or ray origin x/y and translation x/y | none |

Body handles are returned in output words 0/1 (index/generation); no 64-bit
packing occurs. Body reads return floats 0–16: position x/y, rotation c/s,
velocity x/y, angle, angular velocity, mass/inverse mass, inertia/inverse inertia,
force x/y, torque, friction, restitution. Floats 17–20 contain the proxy AABB
lower/upper x/y, with word 4 indicating presence. Words 2/3 are type/awake.

Shape reads use output words 8/9 for kind/count and floats 32–48 for radius and
eight vertex pairs, clearing inactive elements. Body reads also provide this
shape output. Canonical loads validate shape fields without recentering already
constructed polygons. Shape mass output is area, centroid x/y, and inertia per
unit mass. AABB output is lower x/y then upper x/y. Ray output is fraction,
point x/y, normal x/y; misses leave output unchanged.

Native adapter callers own context pointers exactly as they own C allocations;
freeing a context ends its lifetime. World disposal on a retained context is
idempotent. The supported JS wrapper owns and hides all context pointers,
preflights handles before assert-based access, makes disposal idempotent, and
invalidates its private owner token on reset. A handle's exposed numeric fields
are inspection data; a WeakMap retains its actual originating lifetime.

## Fixed memory provisioning

`wasm/CMakeLists.txt` links a 2 MiB minimum with a 1 MiB stack, imported memory,
`ALLOW_MEMORY_GROWTH=0`, and `ABORTING_MALLOC=0`. The wrapper defaults to 64 MiB
and accepts 64 KiB-aligned budgets from 2 to 512 MiB. It creates a private
`WebAssembly.Memory` whose initial and maximum page counts both equal that
budget. Its memory cannot grow, including through an explicit `memory.grow`.

Emscripten 6.0.9 emits `--no-growable-memory`, which also sets the WASM import's
maximum to the link-time minimum. That import type rejects larger fixed
instances. The build runs `provision_memory.py` to change only that import's
maximum from 32 to 8192 pages. All code and other sections remain byte-for-byte
unchanged. The script rejects unexpected, shared, duplicate, malformed, or
already-adjusted imports. It does not enable allocator growth; each supplied
memory still has equal initial/maximum sizes. Tests cover the binary change,
2/64/256 MiB instances, failed allocations, and an explicit growth rejection.
Revisit this workaround when upgrading the pinned SDK.

`module.memory` distinguishes committed linear bytes, fixed stack bytes, static
end address (including stack-first layout), heap base, allocator occupied bytes
(including block overhead), and live requested bytes. The difference between
allocator occupied and requested bytes includes runtime allocations and
allocator overhead; it is not extra engine storage. `world.memory` reports
exact engine arena and adapter requests separately. Initialization may still
fail for fragmentation or host reservation failure. Failed initialization
cleans up; it never changes an existing world. Disposing worlds returns their
allocations to the module; releasing the WASM linear-memory reservation itself
is controlled by JavaScript garbage collection once references disappear.

## Bulk layout, cost, and lifetimes

The adapter allocates one checked `calloc` block at world initialization.
Let B/C/J be resolved body/contact/joint capacities (zero configured contacts
resolve to 4B). On supported native and wasm32 ABIs the allocation is exactly
`144B + 136C + 60J` bytes. `world.memory.adapterBytes` adds the actual
`sizeof(sl_wasm_context)`, including its embedded world shell, scratch shape,
configuration, scalar IO, 26 uint64 work words, accounting, and pointers.
`worldAdapterBytes(config)` returns the same request before creation. Native
and WASM tests pin the payload formula without assuming context pointer size.
The engine arena is separately reported; do not add `worldBytes` on top of
`adapterBytes`, since the latter already owns the embedded world shell.

| Storage | Layout and exact bytes |
| --- | --- |
| Body output | 7 float32 + 5 uint32 columns, 48B |
| Geometry output | 17 float32 + 2 uint32 columns, 76B |
| Private geometry generations | 1 uint32 per slot, 4B |
| Query scratch + output | Native two-word handles + index/generation columns, 16B |
| Contacts | 24 float32 + 10 uint32 columns, 136C |
| Joints | 7 float32 + 8 uint32 columns, 60J |

No padding occurs between these four-byte-aligned slices. Query handle
alignment must remain at most four bytes. uint64 work arrays are members of
the aligned context, never carved from these slices. Tests verify layout and
all native/WASM public output comparisons. `views.mjs` defines the explicit
column order shared by the private C serialization and JS wrapper; no private
engine offset or struct layout crosses the boundary.

Body rows follow public packed traversal and carry slot/generation words.
Geometry columns use slots, contain radius and up to eight local vertex pairs,
and clear inactive vertices when regenerated. A private generation per slot
avoids rebuilding unchanged geometry. Successful shape replacement clears
that slot's generation; destruction and reset clear their cache entries. New
or reused slots rebuild on the next refresh. Only live body slots are valid
geometry. Normal snapshots write 48 bytes per live body plus 76 bytes per
changed geometry slot; diagnostic snapshots additionally write 136 bytes per
active contact and 60 bytes per active joint. Their traversal is O(live bodies
+ optional active contacts/joints), with at most eight vertices per change.
Base snapshots use UINT32_MAX for the unavailable island ID.

Exposing a typed array over WASM staging would also expose the *entire* private
linear memory through its `.buffer`. Therefore C staging views remain private.
The wrapper allocates separate reusable JS typed arrays with exact payload
`132B + 136C + 60J` bytes, reported as `world.memory.outputBytes` and summed for
live worlds by `silk.memory.outputBytes`. This is outside the linear memory
budget and excludes implementation-dependent JS object/ArrayBuffer overhead.
The output payload consists of body, geometry, contact, joint, and two query
columns. It excludes private query handles and cache generations. A failed JS
output allocation releases the newly created C world/context and returns null.
Retained copies are caller-owned and are not counted in live-world accounting;
JS garbage collection controls their release, including borrowed arrays kept
after disposal.

Each refresh makes one C call, then copies 48B bytes of body columns; it copies
76B geometry bytes only when at least one slot changed, 136C contact bytes when
diagnostic contact count is nonzero, and 60J joint bytes when diagnostic joint
count is nonzero. These capacity-sized copies use preallocated arrays and
captured typed-array `set`, avoiding per-frame subarray allocation. This is a
bounded bandwidth cost, even for sparsely populated worlds. Query output
copies another 8B bytes per successful point/AABB query. Native queries retain
their public bounded traversal/order and prepare eight bytes per written
result. Ray output is scalar and requires no bulk copy. No snapshot/query
refresh allocates C memory, JS arrays, handles, or result objects; explicit
`copy()`, `bodyAt()`, `body()`, and individual copied getters do allocate.

The JS snapshot, point/AABB result, and ray result each have a validity flag
and safe-integer revision. Mutation attempts that reach C, reset, disposal, and
snapshot refresh invalidate all three. Successful queries only replace their
own result; invalid queries leave all prior output untouched. A snapshot
refresh increments its revision, as does a successful query for its result.
Revision exhaustion throws before overwriting output. A retained object may
become valid again with new data, so compare a captured revision to distinguish
frames. `copy()` retains independent arrays/scalars/handles. These rules do not
depend on whether a native contact pointer happened to survive an operation.

## Remaining scalar protocol

Joint creation takes words kind, A index/generation, B index/generation,
collide-connected (0/1), and floats local anchor A x/y, B x/y, distance length.
Joint reads use the joint bulk column order with stride one. Revolute length
is serialized as zero. Contact reads likewise use contact columns with stride
one and zero inactive manifold points. Handles retain both words separately.

Point/AABB queries take two/four floats and explicit mask/capacity arguments;
output words are total count, truncation flag, written count. Query arrays use
capacity-B index and generation columns. Ray inputs are origin x/y and
translation x/y; output words are hit, body index/generation, and floats are
fraction, point x/y, normal x/y. A valid miss clears its geometry. Failed
queries do not change scalar output or result arrays. Island output is ID,
dynamic body count, contact count, joint count.

Stats serialize the 13 `statNames` fields, nine `stepNames` fields, and contact
drop count to uint32 output; 13 last-step and 13 cumulative `workNames` counters
are copied as uint64 and read through private `BigUint64Array` views. They
become bigint, never Number. `jsonReplacer` serializes bigint as decimal text.
Native tests exercise 2^53+1 and UINT64_MAX and JS tests preserve their bits and
JSON output. Memory output contains all twelve `memoryNames` size categories,
bounded below 2^32 by engine capacities.

Bounded advance takes float32 dt/remainder/frame time, preflights the native
stepper contract, and returns step count and remainder. Dropped time is exposed
through a double-precision accounting getter. It never enters simulation state.
