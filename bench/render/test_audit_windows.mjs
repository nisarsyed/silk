// Synthetic stalled render-only timeline for the independent offline auditor.
// Uses the real JS producer; this is test data and never device evidence.
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const source=JSON.parse(await fs.readFile(process.argv[2],'utf8'));
const build=path.resolve(process.argv[3]??'build/render-study');
const {FrameRecorder,numberNames,requiredNumbers}=await import(pathToFileURL(path.join(build,'recording.js')));
const r=structuredClone(source),period=r.calibration.targetPeriodMs,duration=60;
// This separate numeric test vector is independent of the browser test clock.
delete r.clock.testOverride;r.testSynthetic=true;r.kind='study-collection';r.configuration.durationSeconds=duration;
const recorder=new FrameRecorder(Math.ceil(duration*1000/period)+2);
const counters={valid:true,stats:new Uint32Array(r.frames.stats.slice(0,25)),work:new BigUint64Array(r.frames.work.slice(0,26).map(BigInt)),statNames:r.frames.statNames,workNames:r.frames.workNames};
const tails=process.argv[4]==='tails',timeline=tails?Array.from({length:601},(_,i)=>100*i):[0,9980,10040,19990,20000,59990,60030];
for(const [i,t]of timeline.entries()){
  const n=new Float64Array(numberNames.length);n[0]=n[1]=n[2]=t;n[3]=t+2;n[4]=t+3;
  recorder.append(n,requiredNumbers|(1<<23),counters);recorder.finish(t+((tails?i>=100&&i<106:i===2)?40:4));
}
r.frames=recorder.report(counters);r.summary=recorder.summary(period);r.windows=recorder.windows(period,duration,120,BigInt(r.initial.drops));
r.measurementStart=0;r.measurementEnd=timeline.at(-1)+4;r.elapsedSeconds=timeline.at(-1)/1000;r.debtSeconds=r.droppedSeconds=0;
r.gpu.frameCapacity=recorder.capacity;r.gpu.typedBytes=recorder.capacity+20*r.gpu.queryCapacity;r.gpu.attempts=recorder.count;r.gpu.statuses=Array(recorder.count).fill(6);
r.gpu.counts=Array(9).fill(0);r.gpu.counts[6]=recorder.count;
console.log(JSON.stringify(r));
