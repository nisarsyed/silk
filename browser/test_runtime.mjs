import assert from 'node:assert/strict';
import {FixedClock,fixedStepSeconds,catchUpStepsMax} from './clock.mjs';
import {PointerQueue,pointerAction,pointerQueueCapacity} from './input_queue.mjs';

assert.equal(fixedStepSeconds,Math.fround(1/60));
assert.equal(catchUpStepsMax,8);
assert.equal(pointerQueueCapacity,256);

// The same fixed-step input sequence has identical state at 30/60/120/144 Hz.
function replay(hz) {
  const clock = new FixedClock(),queue = new PointerQueue(4);
  const events = new Map([[1,[pointerAction.down,0]],
    [40,[pointerAction.move,2]],[100,[pointerAction.move,-1]],
    [120,[pointerAction.up,0]]]);
  let x = Math.fround(0),velocity = Math.fround(0),force = Math.fround(0),step = 0,sequence = 0;
  const apply = (action,_pointerId,worldX) => {
    if (action === pointerAction.down || action === pointerAction.up) force = Math.fround(0);
    else force = worldX;
  };
  for (let frame = 0; frame <= 10*hz; ++frame) {
    const count = clock.tick(frame*1000/hz);
    for (let i = 0; i < count; ++i) {
      ++step;
      if (events.has(step)) {
        const [action,worldX] = events.get(step);
        assert.equal(queue.enqueue(1,++sequence,action,7,worldX,0),'queued');
      }
      queue.drain(apply);
      velocity = Math.fround(velocity+Math.fround(force*fixedStepSeconds));
      x = Math.fround(x+Math.fround(velocity*fixedStepSeconds));
    }
  }
  assert.equal(clock.droppedSeconds,0);
  assert.equal(clock.totalSteps,step);
  assert.ok(clock.interpolation>=0&&clock.interpolation<1);
  return {step,x,velocity};
}
const reference = replay(30);
for (const hz of [60,120,144]) assert.deepEqual(replay(hz),reference);

const stalled = new FixedClock();
assert.equal(stalled.tick(0),0);
assert.equal(stalled.tick(1000),8);
assert.equal(stalled.totalSteps,8);
assert.ok(stalled.droppedSeconds>0);
assert.ok(Math.abs(stalled.totalSteps*fixedStepSeconds+stalled.debtSeconds+
  stalled.droppedSeconds-1)<1e-12);
const unchanged = [stalled.totalSteps,stalled.debtSeconds,stalled.droppedSeconds];
for (const time of [-1,999,NaN,Infinity]) assert.throws(()=>stalled.tick(time));
assert.deepEqual([stalled.totalSteps,stalled.debtSeconds,stalled.droppedSeconds],unchanged);

const paused = new FixedClock();
assert.equal(paused.tick(0),0);
assert.equal(paused.tick(1000/60),0);
assert.equal(paused.setPaused(true,20),true);
assert.equal(paused.requestSingleStep(),true);
assert.equal(paused.tick(2000),1);
assert.equal(paused.tick(4000),0);
assert.equal(paused.totalSteps,1);
assert.ok(paused.suspendedSeconds>3.9);
assert.equal(paused.setPaused(false,5000),true);
assert.equal(paused.tick(5000),0);
assert.equal(paused.droppedSeconds,0);
assert.equal(paused.setPaused(false,5001),false);
paused.setPaused(true,5010);
for (let i=0;i<catchUpStepsMax;++i) assert.equal(paused.requestSingleStep(),true);
assert.equal(paused.requestSingleStep(),false);
assert.equal(paused.tick(5011),8);
paused.reset(5012);
assert.equal(paused.totalSteps,0);
assert.equal(paused.debtSeconds,0);
assert.equal(paused.droppedSeconds,0);
assert.equal(paused.suspendedSeconds,0);
assert.equal(paused.paused,true);

const queue = new PointerQueue(4),seen = [];
assert.equal(queue.enqueue(1,1,pointerAction.down,3,0,0),'queued');
assert.equal(queue.enqueue(1,2,pointerAction.move,3,1,0),'queued');
assert.equal(queue.enqueue(1,3,pointerAction.move,3,2,0),'coalesced');
assert.equal(queue.enqueue(1,4,pointerAction.up,3,2,0),'queued');
assert.equal(queue.count,3);
assert.equal(queue.drain((kind,id,x,y,sequence)=>seen.push([kind,id,x,y,sequence])),3);
assert.deepEqual(seen,[[0,3,0,0,1],[1,3,2,0,3],[2,3,2,0,4]]);
assert.equal(queue.statistics.coalesced,1);

// When saturated, transitions displace old movement, never down/up/cancel.
assert.equal(queue.enqueue(1,5,pointerAction.down,3,0,0),'queued');
assert.equal(queue.enqueue(1,6,pointerAction.move,3,1,0),'queued');
assert.equal(queue.enqueue(1,7,pointerAction.up,3,0,0),'queued');
assert.equal(queue.enqueue(1,8,pointerAction.down,4,0,0),'queued');
assert.equal(queue.enqueue(1,9,pointerAction.move,4,2,0),'dropped-move');
assert.equal(queue.enqueue(1,10,pointerAction.cancel,4,0,0),'queued');
assert.equal(queue.statistics.evictedMoves,1);
assert.equal(queue.statistics.droppedMoves,1);
const kept = [];
queue.drain((kind,id,_x,_y,sequence)=>kept.push([kind,id,sequence]));
assert.deepEqual(kept,[[0,3,5],[2,3,7],[0,4,8],[3,4,10]]);

assert.equal(queue.enqueue(1,11,pointerAction.down,1,0,0),'queued');
assert.equal(queue.enqueue(1,12,pointerAction.up,1,0,0),'queued');
assert.equal(queue.enqueue(1,13,pointerAction.down,2,0,0),'queued');
assert.equal(queue.enqueue(1,14,pointerAction.up,2,0,0),'queued');
assert.equal(queue.enqueue(1,15,pointerAction.cancel,2,0,0),'overflow');
assert.equal(queue.overflowed,true);
assert.throws(()=>queue.drain(()=>{}));
assert.equal(queue.enqueue(1,16,pointerAction.down,3,0,0),'overflow');
queue.reset(2);
assert.equal(queue.enqueue(1,17,pointerAction.down,3,0,0),'stale');
assert.equal(queue.enqueue(2,1,pointerAction.down,3,0,0),'queued');
assert.throws(()=>queue.enqueue(2,1,pointerAction.move,3,1,0));
assert.throws(()=>queue.enqueue(2,2,pointerAction.move,3,Infinity,0));
assert.equal(queue.count,1);
assert.throws(()=>queue.drain(()=>{throw new Error('C action failed');}));
assert.equal(queue.overflowed,true);
queue.reset(3);
assert.equal(queue.count,0);
assert.equal(queue.enqueue(3,1,pointerAction.down,5,0,0),'queued');
assert.throws(()=>queue.drain(()=>queue.enqueue(3,2,pointerAction.up,5,0,0)));
assert.equal(queue.overflowed,true);

console.log('Browser fixed scheduler and ordered bounded pointer queue PASS');
