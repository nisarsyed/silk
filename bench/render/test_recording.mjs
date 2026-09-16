import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {FrameRecorder,numberNames,requiredNumbers,frameCapacityMax}=await import(pathToFileURL(path.join(build,'recording.js')));
const counters={valid:true,stats:new Uint32Array(25),work:new BigUint64Array(26),
  statNames:Array.from({length:25},(_,i)=>String(i)),workNames:Array.from({length:13},(_,i)=>String(i))};
for(const capacity of [0,-1,1.5,NaN,Infinity,frameCapacityMax+1])assert.throws(()=>new FrameRecorder(capacity));
const recorder=new FrameRecorder(4),values=new Float64Array(numberNames.length);
assert.equal(recorder.bytes,4*(23*8+25*4+26*8+4+1));
assert.equal(recorder.summary(1000/60).cpuMs,null);
assert.equal(recorder.summary(1000/60).missedTargetFraction,null);
assert.throws(()=>recorder.finish(0));
assert.throws(()=>recorder.append(values,0,counters));
values[15]=1;assert.throws(()=>recorder.append(values,requiredNumbers,counters));values[15]=0;
for(const invalid of [NaN,Infinity,-1]){values[5]=invalid;assert.throws(()=>recorder.append(values,requiredNumbers,counters));}values[5]=0;
values[15]=.5;assert.throws(()=>recorder.append(values,requiredNumbers|(1<<15),counters));values[15]=0;
assert.throws(()=>recorder.append(values,requiredNumbers,{...counters,valid:false}));assert.equal(recorder.count,0);
const buffers=[recorder.numbers,recorder.stats,recorder.work,recorder.known,recorder.finished];
// Four complete submissions at 2,12,42,52 ms: exactly two missing 10ms slots.
for(let i=0;i<4;++i){
  const callback=[0,10,40,50][i];values.fill(0);
  values[0]=callback;values[1]=callback;values[2]=callback;values[3]=callback+2;values[4]=callback+3;
  values[5]=1;values[13]=i===3?.125:0;
  counters.stats[24]=i+1;counters.work[0]=9007199254740993n+BigInt(i);counters.work[25]=BigInt(i);
  assert.equal(recorder.append(values,requiredNumbers,counters),i);
  assert.throws(()=>recorder.append(values,requiredNumbers,counters),/previous/);
  assert.throws(()=>recorder.finish(callback+2));
  assert.equal(recorder.report(counters).pendingFrame,true);
  recorder.finish(callback+4);assert.equal(recorder.numbers[i*numberNames.length+11],1);
}
for(const [i,buffer]of [recorder.numbers,recorder.stats,recorder.work,recorder.known,recorder.finished].entries())assert.equal(buffer,buffers[i]);
assert.throws(()=>recorder.append(values,requiredNumbers,counters),/exhausted/);
assert.equal(recorder.count,4);
const summary=recorder.summary(10);
assert.deepEqual(summary.cpuMs,{count:4,p50:4,p95:4,p99:4,max:4});
assert.deepEqual(summary.submissionGapMs,{count:3,p50:10,p95:30,p99:30,max:30});
assert.equal(summary.missedTargetSlots,2);assert.equal(summary.submittedIntervals,3);assert.equal(summary.missedTargetFraction,2/5);
assert.throws(()=>recorder.summary(0));assert.throws(()=>recorder.summary(1e-300));
const report=JSON.parse(JSON.stringify(recorder.report(counters)));
assert.equal(report.work[0],'9007199254740993');assert.equal(report.work[26],'9007199254740994');
assert.equal(report.numbers[3*numberNames.length+13],.125);assert.equal(report.known[0]&(1<<14),0);
assert.deepEqual(report.finished,[1,1,1,1]);assert.equal(report.pendingFrame,false);
recorder.resolveGpu(1,.75);assert.equal(recorder.numbers[numberNames.length+14],.75);
assert.equal(recorder.known[1]&(1<<14),1<<14);assert.equal(recorder.numbers[numberNames.length+4],14);
assert.throws(()=>recorder.resolveGpu(1,.8));assert.throws(()=>recorder.resolveGpu(4,1));assert.throws(()=>recorder.resolveGpu(2,NaN));
counters.stats.fill(0);counters.work.fill(0n);values.fill(0);
assert.equal(recorder.stats[24],1);assert.equal(recorder.work[0],9007199254740993n);
const pending=new FrameRecorder(1);pending.append(values,requiredNumbers,counters);
assert.throws(()=>pending.resolveGpu(0,1));
assert.equal(pending.summary(10).pendingFrame,true);assert.equal(pending.summary(10).completeFrames,0);
assert.deepEqual(pending.report(counters).finished,[0]);
assert.equal(pending.known[0]&((1<<4)|(1<<11)),0);
console.log('Frame recorder: fixed storage, exact uint64s, availability, partial records, overflow and nearest-rank cadence PASS');
const {createStudyModule}=await import(pathToFileURL(path.join(build,'physics/driver.mjs')));
const module=await createStudyModule();const world=module.create('pyramid',{steps:2});
try{
  const actual=new FrameRecorder(2);values.fill(0);
  assert.ok(world.refreshFrameCounters());actual.append(values,requiredNumbers,world.frameCounters);actual.finish(1);
  assert.ok(world.step());assert.equal(world.frameCounters.valid,false);
  assert.throws(()=>actual.append(values,requiredNumbers,world.frameCounters));assert.equal(actual.count,1);
  assert.ok(world.refreshFrameCounters());values[1]=10;values[3]=12;values[4]=13;
  actual.append(values,requiredNumbers,world.frameCounters);actual.finish(14);
  const data=actual.report(world.frameCounters),report=world.report();
  assert.equal(data.stats[25+2],report.stats.bodyCount);assert.equal(data.stats[25+24],1);
  assert.equal(data.work[26+25],String(world.drops));
  assert.equal(data.statNames[23],'allocatorUsedBytes');assert.equal(data.workNames[12],'contactDrops');
  console.log('Frame recorder: real study counters, invalidation and schema round trip PASS');
}finally{world.dispose();module.dispose();}
