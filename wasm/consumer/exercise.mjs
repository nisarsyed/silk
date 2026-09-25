import { exports as declared, worldKeys, silkKeys } from './types/api.js';
const check = (value, message) => { if (!value) throw new Error(message); };
const reject = (run, message) => {
  try { run(); } catch { return; }
  throw new Error(`Malformed input accepted: ${message}`);
};
const keysEqual = (actual, expected) => JSON.stringify(actual.sort()) === JSON.stringify(Object.keys(expected).sort());
export async function exercise(api, options = {}) {
  check(keysEqual(Object.keys(api), declared), 'Declaration/runtime export mismatch');
  const a = await api.createSilk(options), b = await api.createSilk(options);
  check(keysEqual(Object.keys(a), silkKeys), 'Silk declaration/runtime member mismatch');
  const w = a.createWorld({ bodyCapacity: 16, jointCapacity: 2, gravity: {x:0,y:-9.81} });
  check(w, 'World initialization');
  check(keysEqual(Object.getOwnPropertyNames(Object.getPrototypeOf(w)).filter(x => x !== 'constructor'), worldKeys),
    'World declaration/runtime member mismatch');
  const shape = a.circle(0.5), body = w.createBody({mass:1,shape});
  const support = w.createBody({type:'static',shape,position:{x:0,y:-0.75}});
  check(body && support, 'Body creation');
  const original=w.readBody(body);
  reject(()=>a.circle(NaN),'non-finite circle radius');
  reject(()=>w.createBody({mass:'1'}),'string mass');
  reject(()=>w.step(Infinity),'non-finite step');
  check(w.readBody(body).position.y===original.position.y,'Malformed input changed world state');
  const other = b.createWorld({bodyCapacity:16});
  for (const [name,value] of Object.entries(api.defaults)) {
    check(JSON.stringify(other.configuration[name]) === JSON.stringify(value), `Public default mismatch: ${name}`);
  }
  const foreign = other.createBody({mass:1});
  check(!w.isBodyValid(foreign), 'Module ownership isolation');
  const joint = w.createJoint({kind:'distance',bodyA:body,bodyB:support,length:0.75});
  check(joint && w.isJointValid(joint), 'Joint binding');
  check(w.step(Math.fround(1/60)) && w.refreshSnapshot(true), 'Step/snapshot');
  const snapshot = w.snapshot, revision = snapshot.revision, retained = snapshot.copy();
  check(snapshot.bodyCount === 2 && snapshot.jointCount === 1, 'Bulk counts');
  check(snapshot.bodies.y[0] === w.readBody(body).position.y, 'Bulk public getter agreement');
  const query = w.queryAabb(-2,-2,2,2,0,1);
  check(query.count === 2 && query.written === 1 && query.truncated, 'Query truncation');
  check(w.isBodyValid(query.bodyAt(0)), 'Returned query handle');
  check(snapshot.isCurrent(revision), 'Read-only query preserves snapshot');
  check(w.queryRay(10,10,1,0).hit === false, 'Valid ray miss');
  check(typeof w.stats().cumulative.treeNodeVisits === 'bigint', 'Exact counters');
  check(JSON.stringify(9007199254740993n, api.jsonReplacer) === '"9007199254740993"', 'Counter JSON');
  w.reset();
  check(!w.isBodyValid(body) && !w.isJointValid(joint) && !snapshot.valid, 'Reset ownership/lifetime');
  check(retained.bodyCount === 2, 'Retained output');
  w.dispose(); w.dispose(); a.dispose(); a.dispose();
  reject(()=>w.readBody(body),'disposed world access');
  check(other.step(1/60), 'Disposing one instance leaves another usable');
  b.dispose();
  return { version: a.version, bodyCount: retained.bodyCount, exports: Object.keys(api).sort() };
}
