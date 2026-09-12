import createModule from './silk.mjs';
import { makeViews, workCopy, statNames, stepNames, memoryNames } from './views.mjs';
export { jsonReplacer } from './views.mjs';

// Handles and shapes expose values for inspection but ownership/shape payloads
// come exclusively from these private maps, never from caller-writable fields.
const handles = new WeakMap();
const shapes = new WeakMap();
const steppers = new WeakMap();
const worldKey = Symbol('world');
const bodyTypes = Object.freeze({ dynamic: 0, kinematic: 1, static: 2 });
export const queryMasks = Object.freeze({ dynamic: 1, kinematic: 2, static: 4, all: 7 });
export const limits = Object.freeze({ bodies: 65536, contacts: 262144, joints: 65536,
  polygonVertices: 8, substeps: 8, position: 8192, shapeExtent: 1024 });

function record(value, name) {
  if (value === null || typeof value !== 'object' || Array.isArray(value) ||
      (Object.getPrototypeOf(value) !== Object.prototype && Object.getPrototypeOf(value) !== null)) {
    throw new TypeError(`${name} must be a plain object`);
  }
  return value;
}
function fields(value, names, name) {
  record(value, name);
  for (const key of Object.keys(value)) {
    if (!names.includes(key)) throw new TypeError(`Unknown ${name} field: ${key}`);
  }
}
function f32(value, name) {
  if (typeof value !== 'number' || !Number.isFinite(value) || !Number.isFinite(Math.fround(value))) {
    throw new TypeError(`${name} must be a finite float32 number`);
  }
  return Math.fround(value);
}
function uint(value, max, name) {
  if (typeof value !== 'number' || !Number.isInteger(value) || value < 0 || value > max) {
    throw new TypeError(`${name} must be an integer in [0, ${max}]`);
  }
  return value;
}
function boolean(value, name) {
  if (typeof value !== 'boolean') throw new TypeError(`${name} must be boolean`);
  return value;
}
function vec(value, name) {
  fields(value, ['x', 'y'], name);
  return [f32(value.x, `${name}.x`), f32(value.y, `${name}.y`)];
}
const defaultValue = (value, fallback) => value === undefined ? fallback : value;
const vector = (x, y) => ({ x, y });
function shapeFromOutput(io) {
  const kind = io.ou[8], count = io.ou[9];
  const vertices = [];
  for (let i = 0; i < count; ++i) vertices.push(Object.freeze(vector(io.of[33 + 2*i], io.of[34 + 2*i])));
  const value = Object.freeze({ kind: ['none', 'circle', 'polygon'][kind],
    radius: io.of[32], vertices: Object.freeze(vertices) });
  shapes.set(value, { kind, count, radius: io.of[32], vertices });
  return value;
}
function shapeData(value) {
  if (value === undefined || value === null) return null;
  const data = shapes.get(value);
  if (!data) throw new TypeError('shape must come from a Silk shape factory or body read');
  return data;
}
function stageShape(m, io, data) {
  if (!data) { m._sl_wasm_shape_none(io.ptr); return; }
  io.iu[0] = data.kind; io.iu[1] = data.count; io.f[0] = data.radius;
  for (let i = 0; i < data.count; ++i) {
    io.f[1 + 2*i] = data.vertices[i].x; io.f[2 + 2*i] = data.vertices[i].y;
  }
  if (!m._sl_wasm_shape_load(io.ptr)) throw new Error('Invalid canonical shape');
}
function context(m) {
  const ptr = m._sl_wasm_context_create();
  if (!ptr) return null;
  return { ptr,
    f: new Float32Array(m.HEAPF32.buffer, m._sl_wasm_input_f32(ptr), 32),
    iu: new Uint32Array(m.HEAPU32.buffer, m._sl_wasm_input_u32(ptr), 16),
    of: new Float32Array(m.HEAPF32.buffer, m._sl_wasm_output_f32(ptr), 64),
    ou: new Uint32Array(m.HEAPU32.buffer, m._sl_wasm_output_u32(ptr), 32) };
}
const configFloats = ['linearDrag', 'angularDrag', 'linearSpeedMax', 'contactHertz',
  'contactDampingRatio', 'contactPushVelocityMax', 'restitutionThreshold',
  'jointHertz', 'jointDampingRatio', 'sleepSpeedMax', 'sleepAngularSpeedMax', 'sleepTimeMin'];
const configDefaults = [0, 0, 400, 30, 10, 3, 1, 60, 2, 0.02, 0.01, 0.5];
function configParse(input) {
  fields(input, ['bodyCapacity', 'contactCapacity', 'jointCapacity', 'substepCount',
    'sleepEnabled', 'gravity', ...configFloats], 'configuration');
  const gravity = vec(defaultValue(input.gravity, vector(0, 0)), 'gravity');
  const words = [uint(input.bodyCapacity, limits.bodies, 'bodyCapacity'),
    uint(defaultValue(input.contactCapacity, 0), limits.contacts, 'contactCapacity'),
    uint(defaultValue(input.jointCapacity, 0), limits.joints, 'jointCapacity'),
    uint(defaultValue(input.substepCount, 0), limits.substeps, 'substepCount'),
    boolean(defaultValue(input.sleepEnabled, false), 'sleepEnabled') ? 1 : 0];
  const floats = [...gravity, ...configFloats.map(name => f32(defaultValue(input[name], 0), name))];
  const resolved = { bodyCapacity: words[0], contactCapacity: words[1] || 4*words[0],
    jointCapacity: words[2], substepCount: words[3] || 4, sleepEnabled: words[4] === 1,
    gravity: Object.freeze(vector(...gravity)) };
  configFloats.forEach((name, i) => { resolved[name] = floats[i+2] || Math.fround(configDefaults[i]); });
  return { words, floats, resolved: Object.freeze(resolved) };
}
function configStage(io, parsed) { io.f.set(parsed.floats); io.iu.set(parsed.words); }

class World {
  #m; #io; #owner = {}; #release; #config; #arenaBytes; #views;
  constructor(key, m, io, config, arenaBytes, release) {
    if (key !== worldKey) throw new TypeError('Use createWorld');
    this.#m = m; this.#io = io; this.#config = config; this.#arenaBytes = arenaBytes;
    this.#release = release;
    this.#views = makeViews(m, io, config, (index, generation) => {
      this.#live();
      if (!m._sl_wasm_body_valid(this.#io.ptr, index, generation)) throw new TypeError('Query body is stale');
      return this.#handle(index, generation);
    });
  }
  #live() { if (!this.#io) throw new Error('World is disposed'); return this.#io; }
  #handle(index, generation, kind = 'body') {
    const value = Object.freeze({ index, generation });
    handles.set(value, { owner: this.#owner, index, generation, kind });
    return value;
  }
  #outputHandle(kind = 'body') { return this.#handle(this.#io.ou[0], this.#io.ou[1], kind); }
  #body(value) {
    const io = this.#live();
    const h = handles.get(value);
    if (!h || h.kind !== 'body' || h.owner !== this.#owner || !this.#m._sl_wasm_body_valid(io.ptr, h.index, h.generation)) {
      throw new TypeError('Body handle is stale or belongs to another world');
    }
    return h;
  }
  #invalidate() {
    this.#views.state.valid = false;
    this.#views.queryState.valid = false;
    this.#views.rayState.valid = false;
  }
  #revision(state) {
    if (state.revision === Number.MAX_SAFE_INTEGER) throw new RangeError('View revision exhausted; create a new world');
    state.revision += 1;
  }
  get configuration() { this.#live(); return this.#config; }
  get bodyCount() { return this.#m._sl_wasm_body_count(this.#live().ptr); }
  get memory() {
    const io = this.#live(); this.#m._sl_wasm_memory_read(io.ptr);
    return Object.freeze({ ...Object.fromEntries(memoryNames.map((name, i) => [name, io.ou[i]])),
      adapterBytes: this.#m._sl_wasm_adapter_bytes(io.ptr), outputBytes: this.#views.outputBytes });
  }
  dispose() {
    if (!this.#io) return;
    this.#invalidate();
    this.#m._sl_wasm_context_destroy(this.#io.ptr);
    this.#io = null; this.#owner = {}; this.#release(this);
  }
  reset() { const io = this.#live(); this.#invalidate(); this.#m._sl_wasm_world_reset(io.ptr); this.#owner = {}; }
  step(dt) { const io = this.#live(), value = f32(dt, 'dt'); this.#invalidate(); return !!this.#m._sl_wasm_world_step(io.ptr, value); }
  isBodyValid(body) {
    const io = this.#live(), h = handles.get(body);
    return !!h && h.kind === 'body' && h.owner === this.#owner && !!this.#m._sl_wasm_body_valid(io.ptr, h.index, h.generation);
  }
  bodyAt(row) {
    const io = this.#live(); uint(row, limits.bodies, 'row');
    return this.#m._sl_wasm_body_at(io.ptr, row) ? this.#outputHandle() : null;
  }
  firstBody() { return this.bodyAt(0); }
  nextBody(body) {
    const h = this.#body(body);
    return this.#m._sl_wasm_body_next(this.#io.ptr, h.index, h.generation) ? this.#outputHandle() : null;
  }
  createBody(desc) {
    const io = this.#live();
    fields(desc, ['type', 'position', 'velocity', 'mass', 'angle', 'angularVelocity',
      'friction', 'restitution', 'shape'], 'body');
    const type = defaultValue(desc.type, 'dynamic');
    if (typeof type !== 'string' || !Object.hasOwn(bodyTypes, type)) throw new TypeError('Invalid body type');
    const values = [...vec(defaultValue(desc.position, vector(0, 0)), 'position'),
      ...vec(defaultValue(desc.velocity, vector(0, 0)), 'velocity'),
      ...['mass', 'angle', 'angularVelocity', 'friction', 'restitution'].map(name => f32(defaultValue(desc[name], 0), name))];
    const shape = shapeData(desc.shape);
    this.#live();
    stageShape(this.#m, io, shape);
    io.f.set(values); io.iu[0] = bodyTypes[type]; this.#invalidate();
    return this.#m._sl_wasm_body_create(io.ptr) ? this.#outputHandle() : null;
  }
  destroyBody(body) {
    const io = this.#live(), h = handles.get(body);
    if (!h || h.kind !== 'body' || h.owner !== this.#owner) return false;
    this.#invalidate();
    return !!this.#m._sl_wasm_body_destroy(io.ptr, h.index, h.generation);
  }
  readBody(body) {
    const h = this.#body(body), io = this.#io;
    this.#m._sl_wasm_body_read(io.ptr, h.index, h.generation);
    const f = io.of, u = io.ou;
    return { handle: body, type: ['dynamic', 'kinematic', 'static'][u[2]], awake: !!u[3],
      position: vector(f[0], f[1]), rotation: { c: f[2], s: f[3] }, velocity: vector(f[4], f[5]),
      angle: f[6], angularVelocity: f[7], mass: f[8], inverseMass: f[9], inertia: f[10], inverseInertia: f[11],
      force: vector(f[12], f[13]), torque: f[14], friction: f[15], restitution: f[16],
      proxyAabb: u[4] ? { lower: vector(f[17], f[18]), upper: vector(f[19], f[20]) } : null,
      shape: shapeFromOutput(io) };
  }
  #scalar(body, value, operation) {
    const h = this.#body(body), v = f32(value, 'value');
    this.#invalidate();
    return !!this.#m[operation](this.#io.ptr, h.index, h.generation, v);
  }
  #vector(body, value, operation) {
    const v = vec(value, 'value'), h = this.#body(body);
    this.#invalidate();
    return !!this.#m[operation](this.#io.ptr, h.index, h.generation, ...v);
  }
  setPosition(h, v) { return this.#vector(h, v, '_sl_wasm_body_set_position'); }
  setVelocity(h, v) { return this.#vector(h, v, '_sl_wasm_body_set_velocity'); }
  applyForce(h, v) { return this.#vector(h, v, '_sl_wasm_body_apply_force'); }
  setAngle(h, v) { return this.#scalar(h, v, '_sl_wasm_body_set_angle'); }
  setAngularVelocity(h, v) { return this.#scalar(h, v, '_sl_wasm_body_set_angular_velocity'); }
  setMass(h, v) { return this.#scalar(h, v, '_sl_wasm_body_set_mass'); }
  setFriction(h, v) { return this.#scalar(h, v, '_sl_wasm_body_set_friction'); }
  setRestitution(h, v) { return this.#scalar(h, v, '_sl_wasm_body_set_restitution'); }
  applyTorque(h, v) { return this.#scalar(h, v, '_sl_wasm_body_apply_torque'); }
  applyForceAtPoint(body, force, point) {
    const f = vec(force, 'force'), p = vec(point, 'point'), h = this.#body(body);
    this.#invalidate();
    return !!this.#m._sl_wasm_body_apply_force_at_point(this.#io.ptr, h.index, h.generation, ...f, ...p);
  }
  setShape(body, shape) {
    const h = this.#body(body), data = shapeData(shape);
    stageShape(this.#m, this.#io, data);
    this.#invalidate();
    return !!this.#m._sl_wasm_body_set_shape(this.#io.ptr, h.index, h.generation);
  }
  wakeBody(body) {
    const h = this.#body(body);
    this.#invalidate();
    return !!this.#m._sl_wasm_body_wake(this.#io.ptr, h.index, h.generation);
  }
  createStepper(dt) {
    const io = this.#live(), timestep = f32(dt, 'dt');
    if (!this.#m._sl_wasm_timestep_valid(io.ptr, timestep)) return null;
    const data = { owner: this.#owner, timestep, remainder: 0, droppedTime: 0, steps: 0 };
    const value = Object.freeze({ get timestep() { return data.timestep; }, get remainder() { return data.remainder; },
      get droppedTime() { return data.droppedTime; }, get steps() { return data.steps; } });
    steppers.set(value, data); return value;
  }
  advance(stepper, frameTime) {
    const io = this.#live(), time = f32(frameTime, 'frameTime'), data = steppers.get(stepper);
    if (!data || data.owner !== this.#owner) throw new TypeError('Stepper is stale or belongs to another world');
    this.#invalidate();
    if (!this.#m._sl_wasm_world_advance(io.ptr, data.timestep, data.remainder, time)) return false;
    data.steps = io.ou[0]; data.remainder = io.of[0]; data.droppedTime = this.#m._sl_wasm_dropped_time(io.ptr);
    return true;
  }
  get snapshot() { this.#live(); return this.#views.snapshot; }
  refreshSnapshot(diagnostics = false) {
    const io = this.#live(), flag = boolean(diagnostics, 'diagnostics');
    this.#revision(this.#views.state); this.#invalidate();
    if (!this.#m._sl_wasm_snapshot_refresh(io.ptr, flag ? 1 : 0)) return false;
    const state = this.#views.state;
    this.#views.refreshCopy(io.ou[3], io.ou[1], io.ou[2]);
    state.bodyCount = io.ou[0]; state.contactCount = io.ou[1]; state.jointCount = io.ou[2];
    state.geometryUpdates = io.ou[3]; state.valid = true;
    return true;
  }
  #queryDone(operation, mask, capacity) {
    const io = this.#live();
    uint(mask, 0xffffffff, 'typeMask'); uint(capacity, limits.bodies, 'capacity');
    const state = this.#views.queryState;
    if (state.revision === Number.MAX_SAFE_INTEGER) throw new RangeError('Query revision exhausted');
    if (!this.#m[operation](io.ptr, mask, capacity)) return null;
    this.#views.queryCopy();
    this.#revision(state); state.count = io.ou[0]; state.truncated = !!io.ou[1];
    state.written = io.ou[2]; state.valid = true;
    return this.#views.query;
  }
  // Scalar query inputs and reused result views keep the hot path allocation-free.
  queryPoint(x, y, typeMask = 0, capacity = this.#config.bodyCapacity) {
    const px = f32(x, 'x'), py = f32(y, 'y'), io = this.#live();
    io.f[0] = px; io.f[1] = py;
    return this.#queryDone('_sl_wasm_query_point', typeMask, capacity);
  }
  queryAabb(lowerX, lowerY, upperX, upperY, typeMask = 0, capacity = this.#config.bodyCapacity) {
    const lx = f32(lowerX, 'lowerX'), ly = f32(lowerY, 'lowerY');
    const ux = f32(upperX, 'upperX'), uy = f32(upperY, 'upperY'), io = this.#live();
    io.f[0] = lx; io.f[1] = ly; io.f[2] = ux; io.f[3] = uy;
    return this.#queryDone('_sl_wasm_query_aabb', typeMask, capacity);
  }
  queryRay(originX, originY, translationX, translationY, typeMask = 0) {
    const ox = f32(originX, 'originX'), oy = f32(originY, 'originY');
    const tx = f32(translationX, 'translationX'), ty = f32(translationY, 'translationY');
    uint(typeMask, 0xffffffff, 'typeMask'); const io = this.#live(), state = this.#views.rayState;
    if (state.revision === Number.MAX_SAFE_INTEGER) throw new RangeError('Ray revision exhausted');
    io.f[0] = ox; io.f[1] = oy; io.f[2] = tx; io.f[3] = ty;
    if (!this.#m._sl_wasm_query_ray(io.ptr, typeMask)) return null;
    this.#revision(state); state.hit = !!io.ou[0]; state.index = io.ou[1]; state.generation = io.ou[2];
    state.fraction = io.of[0]; state.pointX = io.of[1]; state.pointY = io.of[2];
    state.normalX = io.of[3]; state.normalY = io.of[4]; state.valid = true;
    return this.#views.ray;
  }
  islandStats(body) {
    const h = this.#body(body), io = this.#io;
    if (!this.#m._sl_wasm_island_read(io.ptr, h.index, h.generation)) return null;
    return { id: io.ou[0], dynamicBodyCount: io.ou[1], contactCount: io.ou[2], jointCount: io.ou[3] };
  }
  stats() {
    const io = this.#live(); this.#m._sl_wasm_stats_read(io.ptr);
    return { ...Object.fromEntries(statNames.map((name, i) => [name, io.ou[i]])),
      step: { ...Object.fromEntries(stepNames.map((name, i) => [name, io.ou[i+13]])), work: workCopy(this.#views.work) },
      cumulative: workCopy(this.#views.work, 13), contactDropCount: io.ou[22] };
  }
  get jointCount() { return this.#m._sl_wasm_joint_count(this.#live().ptr); }
  get contactCount() { return this.#m._sl_wasm_contact_count(this.#live().ptr); }
  #joint(value) {
    const io = this.#live(), h = handles.get(value);
    if (!h || h.kind !== 'joint' || h.owner !== this.#owner || !this.#m._sl_wasm_joint_valid(io.ptr, h.index, h.generation)) {
      throw new TypeError('Joint handle is stale or belongs to another world');
    }
    return h;
  }
  isJointValid(value) {
    const io = this.#live(), h = handles.get(value);
    return !!h && h.kind === 'joint' && h.owner === this.#owner && !!this.#m._sl_wasm_joint_valid(io.ptr, h.index, h.generation);
  }
  createJoint(desc) {
    fields(desc, ['kind', 'bodyA', 'bodyB', 'localAnchorA', 'localAnchorB', 'length', 'collideConnected'], 'joint');
    const kind = desc.kind;
    if (kind !== 'distance' && kind !== 'revolute') throw new TypeError('Invalid joint kind');
    const a = vec(defaultValue(desc.localAnchorA, vector(0, 0)), 'localAnchorA');
    const b = vec(defaultValue(desc.localAnchorB, vector(0, 0)), 'localAnchorB');
    const length = f32(defaultValue(desc.length, 0), 'length');
    const collide = boolean(defaultValue(desc.collideConnected, false), 'collideConnected');
    // Read both input references before checking either: getters may call application code.
    const bodyA = desc.bodyA, bodyB = desc.bodyB;
    const ha = this.#body(bodyA), hb = this.#body(bodyB), io = this.#io;
    io.f[0] = a[0]; io.f[1] = a[1]; io.f[2] = b[0]; io.f[3] = b[1]; io.f[4] = length;
    io.iu[0] = kind === 'distance' ? 0 : 1; io.iu[1] = ha.index; io.iu[2] = ha.generation;
    io.iu[3] = hb.index; io.iu[4] = hb.generation; io.iu[5] = collide ? 1 : 0;
    this.#invalidate();
    return this.#m._sl_wasm_joint_create(io.ptr) ? this.#outputHandle('joint') : null;
  }
  jointAt(row) {
    const io = this.#live(); uint(row, limits.joints, 'row');
    return this.#m._sl_wasm_joint_at(io.ptr, row) ? this.#outputHandle('joint') : null;
  }
  destroyJoint(value) {
    const io = this.#live(), h = handles.get(value);
    if (!h || h.kind !== 'joint' || h.owner !== this.#owner) return false;
    this.#invalidate(); return !!this.#m._sl_wasm_joint_destroy(io.ptr, h.index, h.generation);
  }
  readJoint(value) {
    const h = this.#joint(value), io = this.#io;
    this.#m._sl_wasm_joint_read(io.ptr, h.index, h.generation);
    return { handle: value, kind: io.ou[2] === 0 ? 'distance' : 'revolute',
      bodyA: this.#handle(io.ou[3], io.ou[4]), bodyB: this.#handle(io.ou[5], io.ou[6]),
      collideConnected: !!io.ou[7], localAnchorA: vector(io.of[0], io.of[1]), localAnchorB: vector(io.of[2], io.of[3]),
      length: io.of[4], linearImpulse: vector(io.of[5], io.of[6]) };
  }
  contactAt(row) {
    const io = this.#live(); uint(row, limits.contacts, 'row');
    if (!this.#m._sl_wasm_contact_read(io.ptr, row)) return null;
    const f = io.of, u = io.ou, points = [];
    for (let i = 0; i < u[5]; ++i) {
      const p = 4 + 10*i;
      points.push({ anchorA: vector(f[p], f[p+1]), anchorB: vector(f[p+2], f[p+3]), point: vector(f[p+4], f[p+5]),
        separation: f[p+6], normalImpulse: f[p+7], tangentImpulse: f[p+8], normalVelocity: f[p+9], id: u[6+2*i], persisted: !!u[7+2*i] });
    }
    return { bodyA: this.#handle(u[0], u[1]), bodyB: this.#handle(u[2], u[3]), touching: !!u[4],
      friction: f[0], restitution: f[1], normal: vector(f[2], f[3]), points };
  }

}

export async function createSilk(options = {}) {
  fields(options, ['memoryBytes', 'wasmUrl'], 'module options');
  const memoryBytes = uint(defaultValue(options.memoryBytes, 64*1024*1024), 512*1024*1024, 'memoryBytes');
  // Reserve at least 2 MiB: 1 MiB stack plus static/runtime data and allocator.
  if (memoryBytes < 2*1024*1024 || memoryBytes % 65536 !== 0) {
    throw new RangeError('memoryBytes must be page-aligned (64 KiB), between 2 and 512 MiB');
  }
  const wasmUrl = options.wasmUrl;
  if (wasmUrl !== undefined && typeof wasmUrl !== 'string' && !(wasmUrl instanceof URL)) {
    throw new TypeError('wasmUrl must be a string or URL');
  }
  let m;
  try {
    m = await createModule({ wasmMemory: new WebAssembly.Memory({
      initial: memoryBytes / 65536, maximum: memoryBytes / 65536 }),
      ...(wasmUrl === undefined ? {} : { locateFile: () => String(wasmUrl) }) });
  } catch (cause) { throw new Error('Silk initialization failed: verify the WASM asset URL and memory budget', { cause }); }
  let scratch = context(m);
  if (!scratch) throw new Error('Silk initialization failed: insufficient module memory');
  const worlds = new Set();
  const live = () => { if (!scratch) throw new Error('Module is disposed'); };
  const makeShape = operation => {
    live();
    if (!operation()) return null;
    m._sl_wasm_shape_read(scratch.ptr);
    return shapeFromOutput(scratch);
  };
  const poseParse = pose => {
    fields(pose, ['position', 'angle'], 'pose');
    return [...vec(defaultValue(pose.position, vector(0, 0)), 'position'), f32(defaultValue(pose.angle, 0), 'angle')];
  };
  const stageGeometry = (shape, values) => {
    const data = shapeData(shape); live(); stageShape(m, scratch, data); scratch.f.set(values);
  };
  const version = m._sl_version();
  return Object.freeze({
    version: `${version >>> 16}.${(version >>> 8) & 255}.${version & 255}`,
    get memory() {
      live();
      let requestedBytes = m._sl_wasm_context_bytes(), outputBytes = 0;
      for (const world of worlds) {
        const memory = world.memory;
        requestedBytes += memory.arenaBytes + memory.adapterBytes;
        outputBytes += memory.outputBytes;
      }
      return Object.freeze({ linearMemoryBytes: m.HEAPU32.byteLength, stackBytes: 1048576,
        staticEnd: m._sl_wasm_static_end(), heapBase: m._sl_wasm_heap_base(),
        allocatorUsedBytes: m._sl_wasm_allocator_used(), requestedBytes, outputBytes });
    },
    none: () => makeShape(() => { m._sl_wasm_shape_none(scratch.ptr); return true; }),
    circle: radius => { live(); const r = f32(radius, 'radius'); return makeShape(() => m._sl_wasm_shape_circle(scratch.ptr, r)); },
    box: (halfWidth, halfHeight) => {
      live(); const x = f32(halfWidth, 'halfWidth'), y = f32(halfHeight, 'halfHeight');
      return makeShape(() => m._sl_wasm_shape_box(scratch.ptr, x, y));
    },
    polygon: points => {
      live();
      if (!Array.isArray(points)) throw new TypeError('polygon needs 3–8 vertices');
      const count = uint(points.length, 8, 'vertex count'), values = [];
      if (count < 3) throw new TypeError('polygon needs 3–8 vertices');
      for (let i = 0; i < count; ++i) values.push(...vec(points[i], `points[${i}]`));
      live(); scratch.f.set(values);
      return makeShape(() => m._sl_wasm_shape_polygon(scratch.ptr, count));
    },
    isShapeValid: shape => { live(); return shapes.has(shape); },
    massData: shape => {
      stageGeometry(shape, []); m._sl_wasm_shape_mass(scratch.ptr);
      return { area: scratch.of[0], centroid: vector(scratch.of[1], scratch.of[2]), inertiaPerUnitMass: scratch.of[3] };
    },
    shapeAabb: (shape, pose = {}) => {
      stageGeometry(shape, poseParse(pose));
      return m._sl_wasm_shape_aabb(scratch.ptr) ? { lower: vector(scratch.of[0], scratch.of[1]),
        upper: vector(scratch.of[2], scratch.of[3]) } : null;
    },
    shapeContainsPoint: (shape, pose, point) => {
      stageGeometry(shape, [...poseParse(pose), ...vec(point, 'point')]);
      return !!m._sl_wasm_shape_contains(scratch.ptr);
    },
    shapeRayCast: (shape, pose, ray) => {
      fields(ray, ['origin', 'translation'], 'ray');
      stageGeometry(shape, [...poseParse(pose), ...vec(ray.origin, 'origin'), ...vec(ray.translation, 'translation')]);
      return m._sl_wasm_shape_ray(scratch.ptr) ? { fraction: scratch.of[0], point: vector(scratch.of[1], scratch.of[2]),
        normal: vector(scratch.of[3], scratch.of[4]) } : null;
    },
    worldMemoryBytes: config => {
      const parsed = configParse(config); live(); configStage(scratch, parsed);
      return m._sl_wasm_world_bytes(scratch.ptr);
    },
    worldAdapterBytes: config => {
      const parsed = configParse(config); live(); configStage(scratch, parsed);
      return m._sl_wasm_adapter_bytes(scratch.ptr);
    },
    createWorld: config => {
      const parsed = configParse(config); live();
      configStage(scratch, parsed);
      const bytes = m._sl_wasm_world_bytes(scratch.ptr);
      if (!bytes) return null;
      const io = context(m);
      if (!io) return null;
      configStage(io, parsed);
      if (!m._sl_wasm_world_init(io.ptr)) { m._sl_wasm_context_destroy(io.ptr); return null; }
      let world;
      try { world = new World(worldKey, m, io, parsed.resolved, bytes, value => worlds.delete(value)); }
      catch (error) {
        m._sl_wasm_context_destroy(io.ptr);
        if (error instanceof RangeError) return null;
        throw error;
      }
      worlds.add(world); return world;
    },
    dispose: () => {
      if (!scratch) return;
      for (const world of worlds) world.dispose();
      m._sl_wasm_context_destroy(scratch.ptr); scratch = null;
    }
  });
}
