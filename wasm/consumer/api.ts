import type * as api from '@nisarsyed/silk';
import type { World, WorldConfig, BodyHandle, JointHandle, Shape, Snapshot, QueryView } from '@nisarsyed/silk';
// Record<> makes an omitted/extra runtime declaration a compile-time failure.
export const exports = { createSilk: true, jsonReplacer: true, limits: true, bodyTypes: true,
  shapeKinds: true, jointKinds: true, queryMasks: true, constants: true, defaults: true } satisfies Record<keyof typeof api, true>;
export const worldKeys = {
  configuration: true, bodyCount: true, jointCount: true, contactCount: true, memory: true, snapshot: true,
  dispose: true, reset: true, step: true, isBodyValid: true, bodyAt: true, firstBody: true, nextBody: true,
  createBody: true, destroyBody: true, readBody: true, setPosition: true, setVelocity: true, applyForce: true,
  setAngle: true, setAngularVelocity: true, setMass: true, setFriction: true, setRestitution: true,
  applyTorque: true, applyForceAtPoint: true, setShape: true, wakeBody: true, createStepper: true, advance: true,
  refreshSnapshot: true, queryPoint: true, queryAabb: true, queryRay: true, islandStats: true, stats: true,
  isJointValid: true, createJoint: true, jointAt: true, destroyJoint: true, readJoint: true, contactAt: true
} satisfies Record<keyof World, true>;
export const silkKeys = {
  version: true, memory: true, none: true, circle: true, box: true, polygon: true, isShapeValid: true,
  massData: true, shapeAabb: true, shapeContainsPoint: true, shapeRayCast: true,
  worldMemoryBytes: true, worldAdapterBytes: true, createWorld: true, dispose: true
} satisfies Record<keyof api.Silk, true>;
// Compile full API use without constructing fake handles or running error cases.
export async function typedConsumer(config: WorldConfig, api: typeof import('@nisarsyed/silk')) {
  const silk = await api.createSilk({ memoryBytes: 64*1024*1024, wasmUrl: new URL('silk.wasm', import.meta.url) });
  const shape = silk.box(1, 1)!;
  silk.none(); silk.circle(1); silk.polygon([{x:0,y:0}, {x:1,y:0}, {x:0,y:1}]);
  silk.isShapeValid(shape); silk.massData(shape); silk.shapeAabb(shape);
  silk.shapeContainsPoint(shape, {}, {x:0,y:0});
  silk.shapeRayCast(shape, {}, {origin:{x:-2,y:0},translation:{x:4,y:0}});
  silk.worldMemoryBytes(config); silk.worldAdapterBytes(config);
  const w = silk.createWorld(config)!;
  const a: BodyHandle = w.createBody({mass:1,shape})!;
  const b = w.createBody({type:'static',shape})!;
  w.isBodyValid(a); w.bodyAt(0); w.firstBody(); w.nextBody(a); w.readBody(a);
  w.setPosition(a,{x:0,y:0}); w.setVelocity(a,{x:0,y:0}); w.applyForce(a,{x:0,y:0});
  w.setAngle(a,0); w.setAngularVelocity(a,0); w.setMass(a,1); w.setFriction(a,0.5);
  w.setRestitution(a,0); w.applyTorque(a,0); w.applyForceAtPoint(a,{x:0,y:0},{x:0,y:0});
  w.setShape(a,shape); w.wakeBody(a);
  const joint: JointHandle = w.createJoint({kind:'distance',bodyA:a,bodyB:b,length:2})!;
  w.jointAt(0); w.readJoint(joint); w.isJointValid(joint); w.contactAt(0);
  w.step(1/60); w.advance(w.createStepper(1/60)!,1/60); w.refreshSnapshot(true);
  const frame: Snapshot = w.snapshot; frame.copy(); frame.isCurrent(frame.revision);
  const query: QueryView = w.queryPoint(0,0,api.queryMasks.dynamic,1)!;
  query.copy(); query.bodyAt(0); w.queryAabb(-1,-1,1,1); w.queryRay(-2,0,4,0)?.copy();
  const counter: bigint = w.stats().cumulative.treeNodeVisits;
  JSON.stringify(counter,api.jsonReplacer); w.islandStats(a);
  w.destroyJoint(joint); w.destroyBody(a); w.reset(); w.dispose(); silk.dispose();
  // @ts-expect-error A numeric pair cannot forge a body handle.
  const forged: BodyHandle = {index:0,generation:1};
  // @ts-expect-error Body and joint handles have distinct brands.
  w.readJoint(a);
  // @ts-expect-error Unknown descriptors are rejected by runtime and declarations.
  w.createBody({density:1});
  // @ts-expect-error Body coordinates must be numbers.
  w.setPosition(a,{x:'0',y:0});
  // @ts-expect-error Queries need scalar coordinates.
  w.queryPoint({x:0,y:0});
  // @ts-expect-error Shapes must come from a canonical factory.
  const fakeShape: Shape = {kind:'circle',radius:1,vertices:[]};
  // @ts-expect-error uint64 counters cannot silently narrow to Number.
  const narrowed: number = counter;
  // @ts-expect-error Configuration is immutable.
  w.configuration.bodyCapacity = 1;
  // @ts-expect-error Snapshot arrays cannot be replaced.
  frame.bodies.x = new Float32Array(1);
  // @ts-expect-error Bounded steppers are opaque world-owned state.
  w.advance({timestep:1/60,remainder:0,droppedTime:0,steps:0},1/60);
  void forged; void fakeShape; void narrowed;
}
