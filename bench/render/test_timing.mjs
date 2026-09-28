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
for(const hz of [60,120,240]){
  const coarse=new Float64Array(240);
  for(let i=0;i<240;++i)coarse[i]=Math.round((i+1)*1000/hz)-Math.round(i*1000/hz);
  const c=calibrate60(coarse);assert.equal(c.divisor,hz/60);assert.equal(c.supports60Hz,true);
  assert.ok(c.effectiveHz>=59&&c.effectiveHz<=61);
  coarse[10]+=Math.round(1000/hz);
  assert.equal(calibrate60(coarse).supports60Hz,true);
}
assert.equal(calibrate60(new Float64Array(240).fill(17)).supports60Hz,false);
assert.throws(()=>new StudyClock(calibration(60),1/60));
const phased=new StudyClock(calibration(60),dt,false,8);
assert.ok(phased.tick(1000/60,1000/60));assert.equal(phased.steps,0);
assert.ok(phased.debtSeconds>0&&phased.debtSeconds<dt);
assert.ok(phased.tick(2000/60,2000/60));assert.equal(phased.steps,1);
assert.ok(Math.abs(phased.totalSteps*dt+phased.debtSeconds-phased.elapsedSeconds)<1e-12);
assert.throws(()=>new StudyClock(calibration(60),dt,false,-1));
assert.throws(()=>new StudyClock(calibration(60),dt,false,NaN));
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
// Chrome can coarsen the observed 60/120/240 Hz median to 16.7/8.3/4.2 ms
// although native callbacks advance at 16.667/8.333/4.167 ms. An absolute
// lattice oscillated between neighboring callbacks, while phase-locking
// alone could outpace the fixed duration/T record capacity.
for(const [hz,median] of [[60,16.7],[120,8.3],[240,4.2]])for(const duration of [60,300]){
  const c=calibrate60(new Float64Array(240).fill(median)),clock=new StudyClock(c,dt);
  let seed=0x735f210,previous=null,missing=0;const gaps=[];
  for(let i=0;i<=hz*duration;++i){
    seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;
    const raf=Math.round((i*1000/hz+((seed>>>0)/4294967296-.5))*10)/10;
    if(clock.tick(raf,raf+1)){
      if(previous!==null){
        const gap=raf-previous;
        missing+=Math.max(0,Math.round(gap/c.targetPeriodMs)-1);
        gaps.push(gap);
      }
      previous=raf;
    }
  }
  gaps.sort((a,b)=>a-b);
  const expected=duration*1000/c.targetPeriodMs;
  assert.ok(clock.frameCount<=Math.ceil(expected)+2);
  assert.ok(clock.frameCount>.99*expected);
  assert.ok(missing/(gaps.length+missing)<=.01);
  assert.ok(gaps[Math.ceil(.95*gaps.length)-1]<=1.25*c.targetPeriodMs);
  assert.ok(gaps[Math.ceil(.99*gaps.length)-1]<=2*c.targetPeriodMs+1);
  assert.ok(gaps.at(-1)<=100);
  assert.equal(clock.droppedSeconds,0);
}
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
