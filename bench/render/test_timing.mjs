import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {calibrate60,StudyClock}=await import(pathToFileURL(path.join(build,'timing.js')));
const dt=Math.fround(1/60);
const calibration=hz=>calibrate60(new Float64Array(240).fill(1000/hz));
for(const hz of [60,120,180,240]){
  const c=calibration(hz);assert.equal(c.supports60Hz,true);assert.equal(c.divisor,hz/60);
  assert.ok(Math.abs(c.effectiveHz-60)<1e-10);
  const clock=new StudyClock(c,dt);let frames=0;
  for(let i=0;i<=hz*10;++i)if(clock.tick(i*1000/hz,i*1000/hz+5))++frames;
  assert.equal(frames,601);assert.equal(clock.totalSteps,Math.floor(10/dt));assert.equal(clock.droppedSeconds,0);
  assert.ok(Math.abs(clock.totalSteps*dt+clock.debtSeconds-10)<1e-12);
}
for(const hz of [30,50,75,90,144,165]){const c=calibration(hz);assert.equal(c.supports60Hz,false);assert.throws(()=>new StudyClock(c,dt));}
assert.throws(()=>calibrate60(new Float64Array(239)));
for(const invalid of [0,-1,NaN,Infinity]){const intervals=new Float64Array(240).fill(16);intervals[100]=invalid;assert.throws(()=>calibrate60(intervals));}
const mixed=new Float64Array(240);for(let i=0;i<240;++i)mixed[i]=i<120?16:17;
assert.equal(calibrate60(mixed).refreshPeriodMs,16);assert.equal(mixed[120],17);
assert.throws(()=>new StudyClock(calibration(60),1/60));
const stalled=new StudyClock(calibration(60),dt);assert.ok(stalled.tick(0,0));assert.ok(stalled.tick(1000,1000));
assert.equal(stalled.steps,8);assert.ok(stalled.droppedSeconds>0);
assert.ok(Math.abs(stalled.totalSteps*dt+stalled.debtSeconds+stalled.droppedSeconds-1)<1e-12);
const saved=JSON.stringify(stalled);
for(const [raf,now]of [[999,1001],[1001,999],[NaN,1001],[1001,Infinity]])assert.throws(()=>stalled.tick(raf,now));
assert.equal(JSON.stringify(stalled),saved);
// Reproducible 120 Hz jitter: no wall-clock/random seeding and no lost time.
let seed=0x735f210;const jittered=new StudyClock(calibration(120),dt);let raf=0,now=0;
for(let i=0;i<12000;++i){
  seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;
  raf+=1000/120+((seed>>>0)/4294967296-.5)*.2;now=raf+2;
  if(jittered.tick(raf,now)){
    assert.ok(jittered.steps<=8);assert.equal(jittered.droppedSeconds,0);
    assert.ok(Math.abs(jittered.totalSteps*dt+jittered.debtSeconds-jittered.elapsedSeconds)<1e-9);
  }
}
assert.ok(jittered.frameCount>=5999&&jittered.frameCount<=6001);
// A delayed first callback must not replay overdue submission slots. Exercise
// queued timestamps followed by normal callbacks, including the short-run
// storage bound which exposed this startup bug in real browser collection.
for(const hz of [60,120,180,240])for(const lag of [0,5,8,10,41.7,42,100,999])for(const duration of [.25,1,5]){
  const c=calibration(hz),clock=new StudyClock(c,dt),capacity=Math.ceil(duration*1000/c.targetPeriodMs)+2;
  for(let i=0;i<2400;++i){
    const raf=i*1000/hz,now=Math.max(lag,raf+1);
    if(clock.tick(raf,now)){
      assert.ok(clock.frameCount<=capacity,`startup overflow: ${hz} Hz / ${lag} ms / ${duration} s`);
      assert.ok(Math.abs(clock.totalSteps*dt+clock.debtSeconds+clock.droppedSeconds-clock.elapsedSeconds)<1e-9);
      if(clock.elapsedSeconds>=duration)break;
    }
  }
  assert.ok(clock.elapsedSeconds>=duration);assert.equal(clock.droppedSeconds,0);
}
const delayed=new StudyClock(calibration(60),dt);
assert.ok(delayed.tick(0,42));assert.equal(delayed.totalSteps,0);assert.equal(delayed.debtSeconds,0);
assert.equal(delayed.tick(1000/60,43),false);assert.equal(delayed.tick(2000/60,44),false);
const frozen=new StudyClock(calibration(60),dt,true);
assert.ok(frozen.tick(0,0));assert.ok(frozen.tick(1000,1000));
assert.equal(frozen.elapsedSeconds,1);assert.equal(frozen.frameCount,2);
assert.equal(frozen.steps,0);assert.equal(frozen.totalSteps,0);assert.equal(frozen.debtSeconds,0);assert.equal(frozen.droppedSeconds,0);
assert.throws(()=>new StudyClock(calibration(60),dt,1));
console.log('Study clock: frozen calibration, high-refresh pacing, binary32 dt, 8-step cap, exposed drops, jitter, delayed startup and invalid clocks PASS');
