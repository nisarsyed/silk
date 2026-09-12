// Private column protocol shared by the wrapper and boundary tests. These names
// describe serialized arrays, not C struct layouts. Views are allocated once.
export const bodyFloats = ['x', 'y', 'cos', 'sin', 'velocityX', 'velocityY', 'angularVelocity'];
export const bodyWords = ['index', 'generation', 'type', 'awake', 'island'];
export const geometryFloats = ['radius', ...Array.from({ length: 8 }, (_, i) => [`x${i}`, `y${i}`]).flat()];
export const geometryWords = ['kind', 'count'];
export const contactFloats = ['friction', 'restitution', 'normalX', 'normalY',
  ...[0, 1].flatMap(i => ['anchorAX', 'anchorAY', 'anchorBX', 'anchorBY', 'pointX', 'pointY',
    'separation', 'normalImpulse', 'tangentImpulse', 'normalVelocity'].map(name => `${name}${i}`))];
export const contactWords = ['bodyAIndex', 'bodyAGeneration', 'bodyBIndex', 'bodyBGeneration', 'touching', 'pointCount',
  'id0', 'persisted0', 'id1', 'persisted1'];
export const jointFloats = ['anchorAX', 'anchorAY', 'anchorBX', 'anchorBY', 'length', 'impulseX', 'impulseY'];
export const jointWords = ['index', 'generation', 'kind', 'bodyAIndex', 'bodyAGeneration', 'bodyBIndex', 'bodyBGeneration', 'collideConnected'];
export const workNames = ['treeNodeVisits', 'pairCandidates', 'pairProbes', 'wakeVisits', 'bodyWakes', 'bodySleeps',
  'graphBodyVisits', 'graphConstraintVisits', 'graphParentProbes', 'proxyCreates', 'proxyDestroys', 'proxyMoves', 'contactDrops'];
export const statNames = ['awakeDynamicCount', 'sleepingDynamicCount', 'bodyCount', 'bodyCapacity', 'contactCount',
  'contactCapacity', 'jointCount', 'jointCapacity', 'pairCount', 'pairCapacity', 'bodyCountHigh', 'contactCountHigh', 'jointCountHigh'];
export const stepNames = ['dynamicBodyCount', 'kinematicBodyCount', 'contactConstraintCount', 'jointConstraintCount',
  'islandExecutedCount', 'islandSkippedCount', 'islandCount', 'islandBodyCountMax', 'substepCount'];
export const memoryNames = ['sleepBytes', 'islandBytes', 'worldStateBytes', 'bodyBytes', 'broadphaseBytes', 'contactBytes',
  'pairBytes', 'contactSolverBytes', 'jointBytes', 'paddingBytes', 'arenaBytes', 'worldBytes'];
export function workCopy(values, offset = 0) {
  return Object.fromEntries(workNames.map((name, i) => [name, values[offset + i]]));
}
export function jsonReplacer(_key, value) { return typeof value === 'bigint' ? value.toString() : value; }
const copyTyped = Function.prototype.call.bind(Uint8Array.prototype.set);
function columns(Type, buffer, pointer, capacity, names, pairs) {
  return Object.fromEntries(names.map((name, i) => {
    const source = new Type(buffer, pointer + i * capacity * Type.BYTES_PER_ELEMENT, capacity);
    const target = new Type(capacity);
    pairs.push([source, target]);
    return [name, target];
  }));
}
function copyPairs(pairs) {
  for (let i = 0; i < pairs.length; ++i) copyTyped(pairs[i][1], pairs[i][0]);
}
function copyColumns(value, count) {
  return Object.fromEntries(Object.entries(value).map(([name, values]) => [name, values.slice(0, count)]));
}
export function makeViews(m, io, config, makeBody) {
  const buffer = m.HEAPU32.buffer, ptr = io.ptr;
  const b = config.bodyCapacity, c = config.contactCapacity, j = config.jointCapacity;
  const bodyPairs = [], geometryPairs = [], contactPairs = [], jointPairs = [];
  const group = (floatPointer, wordPointer, capacity, floats, words, pairs) => Object.freeze({
    ...columns(Float32Array, buffer, floatPointer, capacity, floats, pairs),
    ...columns(Uint32Array, buffer, wordPointer, capacity, words, pairs) });
  const bodies = group(m._sl_wasm_snapshot_f32(ptr), m._sl_wasm_snapshot_u32(ptr), b, bodyFloats, bodyWords, bodyPairs);
  const geometry = group(m._sl_wasm_geometry_f32(ptr), m._sl_wasm_geometry_u32(ptr), b, geometryFloats, geometryWords, geometryPairs);
  const contacts = group(m._sl_wasm_contacts_f32(ptr), m._sl_wasm_contacts_u32(ptr), c, contactFloats, contactWords, contactPairs);
  const joints = group(m._sl_wasm_joints_f32(ptr), m._sl_wasm_joints_u32(ptr), j, jointFloats, jointWords, jointPairs);
  const state = { valid: false, revision: 0, bodyCount: 0, contactCount: 0, jointCount: 0, geometryUpdates: 0 };
  const current = state => { if (!state.valid) throw new Error('Borrowed view is invalid; refresh it or retain a copy'); };
  const snapshot = Object.freeze({ bodies, geometry, contacts, joints,
    get valid() { return state.valid; }, get revision() { return state.revision; },
    get bodyCount() { current(state); return state.bodyCount; },
    get contactCount() { current(state); return state.contactCount; },
    get jointCount() { current(state); return state.jointCount; },
    get geometryUpdates() { current(state); return state.geometryUpdates; },
    isCurrent: revision => state.valid && revision === state.revision,
    copy: () => { current(state); return { bodyCount: state.bodyCount, contactCount: state.contactCount,
      jointCount: state.jointCount, revision: state.revision, bodies: copyColumns(bodies, state.bodyCount),
      geometry: copyColumns(geometry, b), contacts: copyColumns(contacts, state.contactCount), joints: copyColumns(joints, state.jointCount) }; }
  });
  const indices = new Uint32Array(b), generations = new Uint32Array(b);
  const queryIndices = new Uint32Array(buffer, m._sl_wasm_query_u32(ptr), b);
  const queryGenerations = new Uint32Array(buffer, m._sl_wasm_query_u32(ptr) + 4*b, b);
  const queryState = { valid: false, revision: 0, count: 0, written: 0, truncated: false };
  const query = Object.freeze({ indices, generations,
    get valid() { return queryState.valid; }, get revision() { return queryState.revision; },
    get count() { current(queryState); return queryState.count; },
    get written() { current(queryState); return queryState.written; },
    get truncated() { current(queryState); return queryState.truncated; },
    isCurrent: revision => queryState.valid && revision === queryState.revision,
    bodyAt: row => {
      current(queryState);
      if (!Number.isInteger(row) || row < 0 || row >= queryState.written) throw new RangeError('Query row out of range');
      return makeBody(indices[row], generations[row]);
    },
    copy: () => { current(queryState); return { count: queryState.count, truncated: queryState.truncated,
      bodies: Array.from({ length: queryState.written }, (_, row) => makeBody(indices[row], generations[row])) }; }
  });
  const rayState = { valid: false, revision: 0, hit: false, index: 0, generation: 0,
    fraction: 0, pointX: 0, pointY: 0, normalX: 0, normalY: 0 };
  const ray = { get valid() { return rayState.valid; }, get revision() { return rayState.revision; },
    isCurrent: revision => rayState.valid && revision === rayState.revision,
    body: () => { current(rayState); return rayState.hit ? makeBody(rayState.index, rayState.generation) : null; },
    copy: () => { current(rayState); return { hit: rayState.hit, body: ray.body(), fraction: rayState.fraction,
      point: { x: rayState.pointX, y: rayState.pointY }, normal: { x: rayState.normalX, y: rayState.normalY } }; }
  };
  for (const name of ['hit', 'fraction', 'pointX', 'pointY', 'normalX', 'normalY']) {
    Object.defineProperty(ray, name, { enumerable: true, get: () => { current(rayState); return rayState[name]; } });
  }
  return { snapshot, state, query, queryState, ray: Object.freeze(ray), rayState,
    outputBytes: 132*b + 136*c + 60*j,
    refreshCopy: (geometryUpdates, contactCount, jointCount) => {
      copyPairs(bodyPairs);
      if (geometryUpdates > 0) copyPairs(geometryPairs);
      if (contactCount > 0) copyPairs(contactPairs);
      if (jointCount > 0) copyPairs(jointPairs);
    },
    queryCopy: () => { copyTyped(indices, queryIndices); copyTyped(generations, queryGenerations); },
    work: new BigUint64Array(buffer, m._sl_wasm_work_data(ptr), 26) };
}
