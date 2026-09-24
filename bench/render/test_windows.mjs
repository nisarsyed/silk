import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {FrameRecorder,numberNames,requiredNumbers}=await import(pathToFileURL(path.join(build,'recording.js')));
const counters={valid:true,stats:new Uint32Array(25),work:new BigUint64Array(26),
  statNames:Array.from({length:25},(_,i)=>String(i)),workNames:Array.from({length:13},(_,i)=>String(i))};
const r=new FrameRecorder(6),origin=500;
// A 60 ms stall crossing 10 seconds belongs to the arriving frame's window.
// Include an exact 20s boundary and a final overshoot frame, without dropping it.
for(const [row,elapsed]of [0,9980,10040,19990,20000,20030].entries()){
  const t=origin+elapsed,v=new Float64Array(numberNames.length);v[0]=v[1]=v[2]=t;v[3]=t+2;v[4]=t+3;
  v[12]=.001*row;v[13]=row>=2?.125:0;counters.stats[24]=row*2;counters.stats[23]=100+row;
  counters.work[25]=row>=4?9007199254740993n:0n;
  r.append(v,requiredNumbers,counters);r.finish(t+4);if(row===1||row===3)r.resolveGpu(row,.5);
}
const windows=r.windows(10,20),whole=r.summary(10);
assert.equal(windows.length,2);assert.deepEqual(windows.map(w=>[w.firstRow,w.lastRow]),[[0,1],[2,5]]);
assert.deepEqual(windows.map(w=>w.completeFrames),[2,4]);assert.ok(windows.every(w=>w.coverageReached));
assert.equal(windows[1].observedLastSeconds,20.03);assert.equal(windows[1].submissionGapMs.max,9950);
assert.equal(windows.reduce((n,w)=>n+w.submittedIntervals,0),whole.submittedIntervals);
assert.equal(windows.reduce((n,w)=>n+w.missedTargetSlots,0),whole.missedTargetSlots);
assert.equal(windows.reduce((n,w)=>n+w.executedSteps,0),10);assert.equal(windows[0].droppedSeconds,0);
assert.equal(windows[1].droppedSeconds,.125);assert.equal(windows[1].contactDrops,'9007199254740993');
assert.equal(windows[1].allocatorBytesMax,105);assert.equal(windows[1].endDebtSeconds,.005);
assert.deepEqual(windows[0].gpuMs,{count:1,p50:.5,p95:.5,p99:.5,max:.5});assert.equal(windows[1].gpuUnavailableFrames,3);
assert.deepEqual(windows[0].cpuMs,{count:2,p50:4,p95:4,p99:4,max:4});
const unfinished=r.windows(10,300);assert.equal(unfinished.length,30);assert.equal(unfinished[2].coverageReached,false);
assert.equal(unfinished[29].completeFrames,0);assert.equal(unfinished[29].cpuMs,null);assert.equal(unfinished[29].missedTargetFraction,null);
for(const duration of [0,-1,NaN,301])assert.throws(()=>r.windows(10,duration));
assert.throws(()=>r.windows(0,20));assert.throws(()=>r.windows(1e-300,20));
const empty=new FrameRecorder(1);assert.equal(empty.windows(10,10)[0].coverageReached,false);
const values=new Float64Array(numberNames.length);empty.append(values,requiredNumbers,counters);
const pending=empty.windows(10,10)[0];assert.equal(pending.pendingFrame,true);assert.equal(pending.completeFrames,0);assert.equal(pending.cpuMs,null);
r.numbers[3*numberNames.length+13]=0;assert.throws(()=>r.windows(10,20),/backwards/);
// A tail concentrated in one window must remain visible even when the whole
// run's p95 is below it. CPU work stays shorter than the synthetic cadence.
const tails=new FrameRecorder(201);counters.stats.fill(0);counters.work.fill(0n);
for(let i=0;i<=200;++i){
  const t=i*100,v=new Float64Array(numberNames.length);v[0]=v[1]=v[2]=t;v[3]=t+1;v[4]=t+2;
  counters.stats[24]=i;tails.append(v,requiredNumbers,counters);tails.finish(t+(i>=100&&i<108?40:4));
}
assert.equal(tails.summary(100).cpuMs.p95,4);assert.equal(tails.windows(100,20)[1].cpuMs.p95,40);
const skipped=new FrameRecorder(2);
for(const t of [0,20000]){const v=new Float64Array(numberNames.length);v[0]=v[1]=v[2]=v[3]=v[4]=t;
  skipped.append(v,requiredNumbers,counters);skipped.finish(t);}
assert.equal(skipped.windows(100,30)[1].coverageReached,false);assert.equal(skipped.windows(100,30)[1].cpuMs,null);
// Frozen preparation happened before measurement. Its 120 steps and exact
// cumulative counters must not become measured work in the first window.
const frozen=new FrameRecorder(3),baseline=9007199254740993n;
counters.stats[24]=120;counters.work[25]=baseline;
for(const t of [0,10000,20000]){const v=new Float64Array(numberNames.length);v[0]=v[1]=v[2]=v[3]=v[4]=t;
  frozen.append(v,requiredNumbers,counters);frozen.finish(t);}
const frozenWindows=frozen.windows(1000/60,20,120,baseline);
assert.deepEqual(frozenWindows.map(w=>w.executedSteps),[0,0]);assert.deepEqual(frozenWindows.map(w=>w.contactDrops),['0','0']);
frozen.work[2*26+25]=baseline+2n;assert.equal(frozen.windows(1000/60,20,120,baseline)[1].contactDrops,'2');
assert.throws(()=>frozen.windows(1000/60,20,121,baseline),/backwards/);
for(const steps of [-1,1.5,NaN,0x100000000])assert.throws(()=>frozen.windows(1000/60,20,steps,baseline));
for(const drops of [-1n,1,0x10000000000000000n])assert.throws(()=>frozen.windows(1000/60,20,120,drops));
console.log('Ten-second windows: boundary stalls, overshoot, exact drops, missing GPU samples, partial coverage and unchanged nearest-rank cadence PASS');
