/** All physical numbers are validated and rounded to finite float32 before C. */
export interface Vec2 { x: number; y: number }
export interface Rotation { c: number; s: number }
export interface Aabb { lower: Vec2; upper: Vec2 }
export interface Pose { position?: Vec2; angle?: number }
export interface Ray { origin: Vec2; translation: Vec2 }
export interface RayHit { fraction: number; point: Vec2; normal: Vec2 }
declare const bodyBrand: unique symbol, jointBrand: unique symbol, shapeBrand: unique symbol, stepperBrand: unique symbol;
/** Owned by one world/reset lifetime. Numeric fields cannot construct a handle. */
export interface BodyHandle { readonly index: number; readonly generation: number; readonly [bodyBrand]: true }
export interface JointHandle { readonly index: number; readonly generation: number; readonly [jointBrand]: true }
export type BodyType = 'dynamic' | 'kinematic' | 'static';
export type ShapeKind = 'none' | 'circle' | 'polygon';
export type JointKind = 'distance' | 'revolute';
/** Immutable canonical value, produced by a factory or body read. */
export interface Shape {
  readonly [shapeBrand]: true;
  readonly kind: ShapeKind;
  readonly radius: number;
  readonly vertices: readonly Readonly<Vec2>[];
}
export interface WorldConfig {
  bodyCapacity: number;
  contactCapacity?: number;
  jointCapacity?: number;
  substepCount?: number;
  gravity?: Vec2;
  linearDrag?: number;
  angularDrag?: number;
  linearSpeedMax?: number;
  contactHertz?: number;
  contactDampingRatio?: number;
  contactPushVelocityMax?: number;
  restitutionThreshold?: number;
  jointHertz?: number;
  jointDampingRatio?: number;
  sleepEnabled?: boolean;
  sleepSpeedMax?: number;
  sleepAngularSpeedMax?: number;
  sleepTimeMin?: number;
}
export type ResolvedWorldConfig = Readonly<Omit<Required<WorldConfig>, 'gravity'> & { gravity: Readonly<Vec2> }>;
export interface BodyDesc {
  type?: BodyType;
  position?: Vec2;
  velocity?: Vec2;
  mass?: number;
  angle?: number;
  angularVelocity?: number;
  friction?: number;
  restitution?: number;
  shape?: Shape | null;
}
export interface BodyState {
  handle: BodyHandle;
  type: BodyType;
  awake: boolean;
  position: Vec2;
  rotation: Rotation;
  velocity: Vec2;
  angle: number;
  angularVelocity: number;
  mass: number;
  inverseMass: number;
  inertia: number;
  inverseInertia: number;
  force: Vec2;
  torque: number;
  friction: number;
  restitution: number;
  proxyAabb: Aabb | null;
  shape: Shape;
}
export interface JointDesc {
  kind: JointKind;
  bodyA: BodyHandle;
  bodyB: BodyHandle;
  localAnchorA?: Vec2;
  localAnchorB?: Vec2;
  length?: number;
  collideConnected?: boolean;
}
export interface JointState extends Required<JointDesc> { handle: JointHandle; linearImpulse: Vec2 }
export interface ContactPoint {
  anchorA: Vec2; anchorB: Vec2; point: Vec2;
  separation: number; normalImpulse: number; tangentImpulse: number; normalVelocity: number;
  id: number; persisted: boolean;
}
export interface ContactState {
  bodyA: BodyHandle; bodyB: BodyHandle; touching: boolean;
  friction: number; restitution: number; normal: Vec2; points: ContactPoint[];
}
export interface IslandStats { id: number; dynamicBodyCount: number; contactCount: number; jointCount: number }
export interface WorkCounters {
  treeNodeVisits: bigint; pairCandidates: bigint; pairProbes: bigint; wakeVisits: bigint;
  bodyWakes: bigint; bodySleeps: bigint; graphBodyVisits: bigint; graphConstraintVisits: bigint;
  graphParentProbes: bigint; proxyCreates: bigint; proxyDestroys: bigint; proxyMoves: bigint; contactDrops: bigint;
}
export interface StepStats {
  dynamicBodyCount: number; kinematicBodyCount: number; contactConstraintCount: number;
  jointConstraintCount: number; islandExecutedCount: number; islandSkippedCount: number;
  islandCount: number; islandBodyCountMax: number; substepCount: number; work: WorkCounters;
}
export interface WorldStats {
  awakeDynamicCount: number; sleepingDynamicCount: number; bodyCount: number; bodyCapacity: number;
  contactCount: number; contactCapacity: number; jointCount: number; jointCapacity: number;
  pairCount: number; pairCapacity: number; bodyCountHigh: number; contactCountHigh: number;
  jointCountHigh: number; step: StepStats; cumulative: WorkCounters; contactDropCount: number;
}
export interface WorldMemory {
  readonly sleepBytes: number; readonly islandBytes: number; readonly worldStateBytes: number;
  readonly bodyBytes: number; readonly broadphaseBytes: number; readonly contactBytes: number;
  readonly pairBytes: number; readonly contactSolverBytes: number; readonly jointBytes: number;
  readonly paddingBytes: number; readonly arenaBytes: number; readonly worldBytes: number;
  readonly adapterBytes: number; readonly outputBytes: number;
}
export interface ModuleMemory {
  readonly linearMemoryBytes: number; readonly stackBytes: number; readonly staticEnd: number;
  readonly heapBase: number; readonly allocatorUsedBytes: number; readonly requestedBytes: number;
  /** Live-world JS typed-array payload outside linear memory; excludes retained copies and JS object overhead. */
  readonly outputBytes: number;
}
export interface BodyColumns {
  readonly x: Float32Array; readonly y: Float32Array; readonly cos: Float32Array; readonly sin: Float32Array;
  readonly velocityX: Float32Array; readonly velocityY: Float32Array; readonly angularVelocity: Float32Array;
  readonly index: Uint32Array; readonly generation: Uint32Array; readonly type: Uint32Array;
  readonly awake: Uint32Array; readonly island: Uint32Array;
}
export type VertexColumn = `${'x' | 'y'}${0 | 1 | 2 | 3 | 4 | 5 | 6 | 7}`;
export type GeometryColumns = Readonly<Record<VertexColumn | 'radius', Float32Array> & { kind: Uint32Array; count: Uint32Array }>;
export type ContactPointColumn = `${'anchorAX' | 'anchorAY' | 'anchorBX' | 'anchorBY' | 'pointX' | 'pointY' | 'separation' | 'normalImpulse' | 'tangentImpulse' | 'normalVelocity'}${0 | 1}`;
export type ContactColumns = Readonly<Record<ContactPointColumn | 'friction' | 'restitution' | 'normalX' | 'normalY', Float32Array> &
  Record<'bodyAIndex' | 'bodyAGeneration' | 'bodyBIndex' | 'bodyBGeneration' | 'touching' | 'pointCount' | 'id0' | 'persisted0' | 'id1' | 'persisted1', Uint32Array>>;
export type JointColumns = Readonly<Record<'anchorAX' | 'anchorAY' | 'anchorBX' | 'anchorBY' | 'length' | 'impulseX' | 'impulseY', Float32Array> &
  Record<'index' | 'generation' | 'kind' | 'bodyAIndex' | 'bodyAGeneration' | 'bodyBIndex' | 'bodyBGeneration' | 'collideConnected', Uint32Array>>;
export interface SnapshotCopy {
  bodyCount: number; contactCount: number; jointCount: number; revision: number;
  bodies: BodyColumns; geometry: GeometryColumns; contacts: ContactColumns; joints: JointColumns;
}
export interface BorrowedView {
  readonly valid: boolean;
  readonly revision: number;
  isCurrent(revision: number): boolean;
}
/** Reused arrays are logically read-only. Do not write, transfer, or detach them. */
export interface Snapshot extends BorrowedView, Readonly<SnapshotCopy> {
  readonly geometryUpdates: number;
  copy(): SnapshotCopy;
}
export interface QueryCopy { count: number; truncated: boolean; bodies: BodyHandle[] }
export interface QueryView extends BorrowedView {
  readonly indices: Uint32Array; readonly generations: Uint32Array;
  readonly count: number; readonly written: number; readonly truncated: boolean;
  bodyAt(row: number): BodyHandle;
  copy(): QueryCopy;
}
export interface RayCopy extends RayHit { hit: boolean; body: BodyHandle | null }
export interface RayView extends BorrowedView {
  readonly hit: boolean; readonly fraction: number;
  readonly pointX: number; readonly pointY: number; readonly normalX: number; readonly normalY: number;
  body(): BodyHandle | null;
  copy(): RayCopy;
}
export interface Stepper {
  readonly [stepperBrand]: true;
  readonly timestep: number; readonly remainder: number;
  /** Time discarded by the most recent advance, not a cumulative total. */
  readonly droppedTime: number; readonly steps: number;
}
export interface World {
  readonly configuration: ResolvedWorldConfig;
  readonly bodyCount: number; readonly jointCount: number; readonly contactCount: number;
  readonly memory: WorldMemory; readonly snapshot: Snapshot;
  dispose(): void;
  reset(): void;
  step(dt: number): boolean;
  isBodyValid(body: unknown): body is BodyHandle;
  bodyAt(row: number): BodyHandle | null;
  firstBody(): BodyHandle | null;
  nextBody(body: BodyHandle): BodyHandle | null;
  createBody(desc: BodyDesc): BodyHandle | null;
  destroyBody(body: BodyHandle): boolean;
  readBody(body: BodyHandle): BodyState;
  setPosition(body: BodyHandle, value: Vec2): boolean;
  setVelocity(body: BodyHandle, value: Vec2): boolean;
  applyForce(body: BodyHandle, value: Vec2): boolean;
  setAngle(body: BodyHandle, value: number): boolean;
  setAngularVelocity(body: BodyHandle, value: number): boolean;
  setMass(body: BodyHandle, value: number): boolean;
  setFriction(body: BodyHandle, value: number): boolean;
  setRestitution(body: BodyHandle, value: number): boolean;
  applyTorque(body: BodyHandle, value: number): boolean;
  applyForceAtPoint(body: BodyHandle, force: Vec2, point: Vec2): boolean;
  setShape(body: BodyHandle, shape: Shape | null): boolean;
  wakeBody(body: BodyHandle): boolean;
  createStepper(dt: number): Stepper | null;
  advance(stepper: Stepper, frameTime: number): boolean;
  refreshSnapshot(diagnostics?: boolean): boolean;
  queryPoint(x: number, y: number, typeMask?: number, capacity?: number): QueryView | null;
  queryAabb(lowerX: number, lowerY: number, upperX: number, upperY: number, typeMask?: number, capacity?: number): QueryView | null;
  queryRay(originX: number, originY: number, translationX: number, translationY: number, typeMask?: number): RayView | null;
  islandStats(body: BodyHandle): IslandStats | null;
  stats(): WorldStats;
  isJointValid(joint: unknown): joint is JointHandle;
  createJoint(desc: JointDesc): JointHandle | null;
  jointAt(row: number): JointHandle | null;
  destroyJoint(joint: JointHandle): boolean;
  readJoint(joint: JointHandle): JointState;
  contactAt(row: number): ContactState | null;
}
export interface ModuleOptions {
  /** Fixed, 64 KiB-aligned budget in [2, 512] MiB; default 64 MiB. */
  memoryBytes?: number;
  /** Absolute URL recommended. Default is silk.wasm next to the generated glue. */
  wasmUrl?: string | URL;
}
export interface Silk {
  readonly version: string;
  readonly memory: ModuleMemory;
  none(): Shape;
  circle(radius: number): Shape | null;
  box(halfWidth: number, halfHeight: number): Shape | null;
  polygon(points: readonly Vec2[]): Shape | null;
  isShapeValid(shape: unknown): shape is Shape;
  massData(shape: Shape | null): { area: number; centroid: Vec2; inertiaPerUnitMass: number };
  shapeAabb(shape: Shape | null, pose?: Pose): Aabb | null;
  shapeContainsPoint(shape: Shape | null, pose: Pose, point: Vec2): boolean;
  shapeRayCast(shape: Shape | null, pose: Pose, ray: Ray): RayHit | null;
  worldMemoryBytes(config: WorldConfig): number;
  worldAdapterBytes(config: WorldConfig): number;
  createWorld(config: WorldConfig): World | null;
  dispose(): void;
}
export function createSilk(options?: ModuleOptions): Promise<Silk>;
/** Exact decimal encoding for uint64 work counters. */
export function jsonReplacer(key: string, value: unknown): unknown;
export const limits: Readonly<{ bodies: 65536; contacts: 262144; joints: 65536; polygonVertices: 8; substeps: 8; steps: 8; position: 8192; shapeExtent: 1024; rayCoordinate: 9216; manifoldPoints: 2; rotationPerStep: number }>;
export const bodyTypes: Readonly<{ dynamic: 0; kinematic: 1; static: 2 }>;
export const shapeKinds: Readonly<{ none: 0; circle: 1; polygon: 2 }>;
export const jointKinds: Readonly<{ distance: 0; revolute: 1 }>;
export const queryMasks: Readonly<{ dynamic: 1; kinematic: 2; static: 4; all: 7 }>;

// Keep the branding symbols module-private rather than implicit ambient exports.
export {};

export const constants: Readonly<{ pi: number; epsilon: number; vec2LengthEpsilonSq: number; linearSlop: number; speculativeDistance: number; aabbMargin: number }>;
export const defaults: Omit<ResolvedWorldConfig, 'bodyCapacity' | 'contactCapacity' | 'jointCapacity'>;
