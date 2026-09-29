import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {FixedClock} from './clock.mjs';
import {PointerQueue,pointerAction} from './input_queue.mjs';

const build = path.resolve(process.argv[2]??'build/wasm-release/render-physics');
const {createStudyModule} = await import(pathToFileURL(path.join(build,'driver.mjs')));
const module = await createStudyModule();

function replay(scene,sleep,hz) {
  const world = module.create(scene,{sleep,steps:300});
  assert.ok(world);
  try {
    assert.ok(world.refreshSnapshot());
    const bodies = world.snapshot.bodies;
    const row = bodies.type.findIndex(value=>value===0);
    assert.ok(row>=0);
    const x = bodies.x[row]+.02,y = bodies.y[row];
    const actions = new Map([
      [1,[pointerAction.down,x,y]],
      [10,[pointerAction.move,x+.2,y+.5]],
      [30,[pointerAction.up,x+.2,y+.5]],
      [60,[pointerAction.down,x,y]],
      [80,[pointerAction.cancel,x,y]],
    ]);
    const clock = new FixedClock(),queue = new PointerQueue(8);
    let step = 0,sequence = 0;
    for (let frame = 0; frame <= 3*hz; ++frame) {
      const count = clock.tick(frame*1000/hz);
      for (let i = 0; i < count; ++i) {
        ++step;
        if (actions.has(step)) {
          const [kind,worldX,worldY] = actions.get(step);
          assert.equal(queue.enqueue(1,++sequence,kind,7,worldX,worldY),'queued');
        }
        queue.drain((kind,_id,worldX,worldY)=>assert.ok(world.pointer(kind,worldX,worldY)));
        assert.ok(world.step());
      }
    }
    assert.equal(clock.droppedSeconds,0);
    assert.equal(clock.totalSteps,world.steps);
    assert.equal(queue.count,0);
    assert.ok(world.refreshSnapshot(true));
    return {report:world.report(),snapshot:world.snapshot.copy()};
  } finally { world.dispose(); }
}

try {
  for (const scene of ['pyramid','chains']) for (const sleep of [false,true]) {
    const reference = replay(scene,sleep,30);
    for (const hz of [60,120,144])
      assert.deepEqual(replay(scene,sleep,hz),reference,`${scene}, sleep ${sleep}, ${hz} Hz`);
  }
  console.log('Browser host schedule: real WASM poses/work equal across 30/60/120/144 Hz and queued input PASS');
} finally { module.dispose(); }
