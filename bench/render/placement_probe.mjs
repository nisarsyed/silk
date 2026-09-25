import {createStudyModule} from './physics/driver.mjs';
import {DrawScene} from './geometry.js';
import {CanvasCandidate} from './canvas.js';
import {WebglCandidate} from './webgl.js';
import {FixedClock} from './host_clock.mjs';
import {PointerQueue,pointerAction} from './host_input_queue.mjs';

// Correctness probe only: the same source scene, drawing code and precommitted
// callback/input sequence execute on main or inside a dedicated worker. No
// whole-world frame data crosses the thread boundary.
export async function runPlacement(canvas,{candidate,scene,sleep,layout='desktop'}) {
  if (!['canvas','webgl'].includes(candidate) || !['pyramid','chains'].includes(scene) ||
      typeof sleep !== 'boolean' || !['desktop','mobile'].includes(layout))
    throw new TypeError('Invalid placement probe');
  canvas.width = layout==='desktop'?1280:720;
  canvas.height = layout==='desktop'?720:1280;
  const module = await createStudyModule();
  let world,renderer;
  try {
    world = module.create(scene,{sleep,steps:300});
    if (!world || !world.refreshSnapshot()) throw new Error('Placement source failed');
    const bodies=world.snapshot.bodies,row=bodies.type.findIndex(value=>value===0);
    if (row<0) throw new Error('No dynamic probe body');
    const x=bodies.x[row]+.02,y=bodies.y[row];
    const actions=new Map([[1,[pointerAction.down,x,y]],
      [10,[pointerAction.move,x+.2,y+.5]],
      [30,[pointerAction.up,x+.2,y+.5]],
      [60,[pointerAction.down,x,y]],
      [80,[pointerAction.cancel,x,y]]]);
    const sceneDraw=new DrawScene(world.snapshot,scene);
    renderer=candidate==='canvas'?new CanvasCandidate(canvas,sceneDraw):new WebglCandidate(canvas,sceneDraw);
    const clock=new FixedClock(),queue=new PointerQueue(8);
    let step=0,sequence=0;
    for (let frame=0;frame<=180;++frame) {
      const count=clock.tick(frame*1000/60);
      for (let i=0;i<count;++i) {
        ++step;
        if (actions.has(step)) {
          const [kind,worldX,worldY]=actions.get(step);
          if (queue.enqueue(1,++sequence,kind,7,worldX,worldY)!=='queued')
            throw new Error('Probe input queue rejected a defined action');
        }
        queue.drain((kind,_id,worldX,worldY)=>{
          if (!world.pointer(kind,worldX,worldY)) throw new Error('Probe pointer action failed');
        });
        if (!world.step()) throw new Error('Probe step failed');
      }
      if (!world.refreshSnapshot()) throw new Error('Probe snapshot failed');
      sceneDraw.copyTransforms(world.snapshot);
      renderer.draw();
    }
    if (clock.droppedSeconds!==0 || queue.count!==0 || world.steps!==clock.totalSteps)
      throw new Error('Probe schedule diverged');
    const snapshot=world.snapshot,columns=[snapshot.bodies.x,snapshot.bodies.y,
      snapshot.bodies.cos,snapshot.bodies.sin];
    const bits=columns.map(column=>new Uint32Array(column.buffer,column.byteOffset,column.length));
    let hash=2166136261;
    for(let row=0;row<snapshot.bodyCount;++row)for(const column of bits)
      hash=Math.imul(hash^column[row],16777619)>>>0;
    const report=world.report();
    return {candidate,scene,sleep,layout,worker:canvas instanceof OffscreenCanvas,
      crossOriginIsolated:globalThis.crossOriginIsolated,
      steps:world.steps,bodyCount:snapshot.bodyCount,poseHash:hash,
      contacts:report.stats.contactCount,drops:String(report.drops),
      work:Object.fromEntries(Object.entries(report.work).map(([name,value])=>[name,String(value)])),
      drawCalls:renderer.drawCalls??renderer.submissionCalls,
      linearMemoryBytes:module.memory.linearMemoryBytes};
  } finally { renderer?.dispose();world?.dispose();module.dispose(); }
}
