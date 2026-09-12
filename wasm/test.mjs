import assert from 'node:assert/strict';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';
const entry = pathToFileURL(resolve(process.argv[2] ?? 'build/wasm-debug/package/index.mjs'));
const { createSilk, limits } = await import(entry);
const engine = await createSilk();
assert.equal(engine.version, '0.4.0');
assert.equal(engine.memory.linearMemoryBytes, 64*1024*1024);
assert.equal(limits.polygonVertices, 8);
const config = { bodyCapacity: 2, gravity: { x: 0, y: -9.81 } };
const world = engine.createWorld(config);
assert(world);
assert.equal(world.memory.arenaBytes, engine.worldMemoryBytes(config));
assert.equal(world.configuration.substepCount, 4);
const circle = engine.circle(1);
assert.equal(engine.none().kind, 'none');
assert(engine.isShapeValid(circle));
assert.equal(engine.isShapeValid({}), false);
assert.equal(engine.massData(circle).inertiaPerUnitMass, 0.5);
assert.deepEqual(engine.shapeAabb(circle, { position: { x: 3, y: 2 } }),
  { lower: { x: 2, y: 1 }, upper: { x: 4, y: 3 } });
assert(engine.shapeContainsPoint(circle, {}, { x: 0, y: 0 }));
assert.equal(engine.shapeRayCast(circle, {}, { origin: { x: 0, y: 0 }, translation: { x: 3, y: 0 } }), null);
assert.equal(engine.shapeRayCast(circle, {}, { origin: { x: -2, y: 0 }, translation: { x: 4, y: 0 } }).fraction, 0.25);
assert.equal(engine.circle(-1), null);
assert.throws(() => engine.polygon(new Array(4)), TypeError);
const triangle = engine.polygon([{ x: 0, y: 0 }, { x: 2, y: 0 }, { x: 0, y: 2 }]);
assert(triangle); assert.equal(triangle.vertices.length, 3);
const body = world.createBody({ mass: 1, shape: circle });
assert(body);
assert.equal(world.readBody(body).inertia, 0.5);
for (const value of [NaN, Infinity, -Infinity, 1e100, '1', null, true, {}, [], 1n]) {
  assert.throws(() => world.setMass(body, value), TypeError);
  assert.throws(() => world.step(value), TypeError);
  assert.throws(() => world.createBody({ mass: value }), TypeError);
  assert.throws(() => engine.createWorld({ bodyCapacity: 2, linearDrag: value }), TypeError);
}
for (const value of [0, -1, 1e-45]) assert.equal(world.step(value), false);
for (const value of [-1, 1.5, 2**32, NaN, Infinity, '2', null, true]) {
  assert.throws(() => engine.createWorld({ bodyCapacity: value }), TypeError);
}
assert.throws(() => world.createBody({ type: 'other', mass: 1 }), TypeError);
assert.throws(() => world.createBody({ mass: 1, shape: { kind: 'circle', radius: 1 } }), TypeError);
assert.throws(() => world.createBody({ mas: 1 }), TypeError);
assert.throws(() => engine.createWorld({ bodyCapacity: 2, sleepEnabled: 1 }), TypeError);
assert.equal(engine.createWorld({ bodyCapacity: 0 }), null);
assert.equal(engine.createWorld({ bodyCapacity: 1, contactHertz: -1 }), null);
const initial = world.readBody(body);
assert.equal(world.setMass(body, 0), false);
assert.equal(world.setPosition(body, { x: 9000, y: 0 }), false);
assert.equal(world.setRestitution(body, 2), false);
assert.deepEqual(world.readBody(body), initial);
assert(world.applyForceAtPoint(body, { x: 0, y: 6 }, { x: 1, y: 0 }));
assert.equal(world.readBody(body).torque, 6);
assert(world.step(1/60));
assert.equal(world.readBody(body).torque, 0);
assert(world.setShape(body, engine.box(1, 1)));
assert.equal(world.readBody(body).shape.vertices.length, 4);
assert(world.setShape(body, world.readBody(body).shape));
assert.equal(world.readBody(body).shape.vertices.length, 4);
const other = engine.createWorld(config);
const otherBody = other.createBody({ mass: 1 });
assert.throws(() => other.readBody(body), /another world/);
assert.throws(() => world.setMass(otherBody, 1), /another world/);
assert.throws(() => world.readBody({ ...body }), /stale/);
assert.equal(other.isBodyValid(body), false);
assert.equal(other.destroyBody(body), false);
const extra = world.createBody({ mass: 1 });
assert(extra);
assert.equal(world.createBody({ mass: 1 }), null);
assert.equal(world.bodyCount, 2);
assert.deepEqual(world.firstBody(), body);
assert.deepEqual(world.nextBody(body), extra);
assert.equal(world.nextBody(extra), null);
assert.equal(world.bodyAt(2), null);
assert(world.destroyBody(body));
assert.equal(world.destroyBody(body), false);
const replacement = world.createBody({ mass: 1 });
assert.equal(replacement.index, body.index);
assert.notEqual(replacement.generation, body.generation);
assert.throws(() => world.readBody(body), /stale/);
world.reset();
assert.equal(world.isBodyValid(replacement), false);
assert.equal(other.isBodyValid(otherBody), true);
assert(world.createBody({ mass: 1 }));
world.dispose(); world.dispose();
assert.throws(() => world.step(1/60), /disposed/);
assert.throws(() => world.bodyCount, /disposed/);
assert.throws(() => world.createBody({ mass: 1 }), /disposed/);
const twinEngine = await createSilk();
const twin = twinEngine.createWorld(config);
const twinBody = twin.createBody({ mass: 1 });
assert.throws(() => other.readBody(twinBody), /another world/);
const heapBeforeSteps = engine.memory.allocatorUsedBytes;
for (let i = 0; i < 300; ++i) { assert(other.step(1/60)); assert(twin.step(1/60)); }
assert.equal(engine.memory.allocatorUsedBytes, heapBeforeSteps);
const a = other.readBody(otherBody), b = twin.readBody(twinBody);
assert.deepEqual(a.position, b.position);
assert.deepEqual(a.velocity, b.velocity);
assert.equal(engine.memory.linearMemoryBytes, 64*1024*1024);
engine.dispose(); engine.dispose();
assert.throws(() => other.step(1/60), /disposed/);
assert.throws(() => engine.circle(1), /disposed/);
assert.throws(() => engine.createWorld(config), /disposed/);
assert(twin.step(1/60));
twinEngine.dispose();
const small = await createSilk({ memoryBytes: 2*1024*1024 });
assert.equal(small.memory.linearMemoryBytes, 2*1024*1024);
assert.equal(small.createWorld({ bodyCapacity: limits.bodies, jointCapacity: limits.joints }), null);
// Failed large allocations must not consume the module's remaining budget.
for (let i = 0; i < 200; ++i) {
  const w = small.createWorld({ bodyCapacity: 16 }); assert(w); w.dispose();
}
const full = [];
for (let i = 0; i < 256; ++i) {
  const w = small.createWorld({ bodyCapacity: 128 });
  if (!w) break;
  full.push(w);
}
assert(full.length > 0 && full.length < 256);
for (const w of full) w.dispose();
assert(small.createWorld({ bodyCapacity: 128 }));
small.dispose();
for (const memoryBytes of [0, 65536, 2097153, 536870913, null, '67108864']) {
  await assert.rejects(createSilk({ memoryBytes }));
}
// Input getters may call application code: disposal must be caught before C.
const reentrant = await createSilk();
const reentrantWorld = reentrant.createWorld(config);
assert.throws(() => reentrantWorld.createBody({ get mass() { reentrantWorld.dispose(); return 1; } }), /disposed/);
assert.throws(() => reentrant.createWorld({ get bodyCapacity() { reentrant.dispose(); return 1; } }), /disposed/);
const { default: rawFactory } = await import(new URL('./silk.mjs', entry));
const fixedMemory = new WebAssembly.Memory({ initial: 32, maximum: 32 });
const raw = await rawFactory({ wasmMemory: fixedMemory });
assert.throws(() => fixedMemory.grow(1), RangeError);
assert(raw._sl_wasm_heap_base() < 2*1024*1024);
const large = await createSilk({ memoryBytes: 128*1024*1024 });
assert.equal(large.memory.linearMemoryBytes, 128*1024*1024);
assert(large.createWorld({ bodyCapacity: limits.bodies, jointCapacity: limits.joints }));
assert(large.memory.allocatorUsedBytes >= large.memory.requestedBytes);
large.dispose();
console.log('WASM lifecycle, validation, shapes, bodies, memory exhaustion, isolation, and replay passed');
