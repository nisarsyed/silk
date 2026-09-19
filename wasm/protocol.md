# Private adapter marshalling protocol

This ABI belongs to the wrapper and is not installed as a C consumer API.
`adapter.c` uses public Silk headers only. Pointers are private wrapper state;
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
2/64/128 MiB instances, failed allocations, and an explicit growth rejection.
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
