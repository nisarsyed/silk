import assert from 'node:assert/strict';
import {PresentationClock,presentationCalibrationIntervals} from './presentation_clock.mjs';

const simulate=(interval,count)=>{
  const clock=new PresentationClock();let submitted=0;
  for(let i=0;i<=count;++i)if(clock.tick(i*interval))++submitted;
  return {clock,submitted};
};
const sixty=simulate(1000/60,360);
assert.equal(sixty.clock.calibrated,true);
assert.equal(sixty.clock.report.calibratedIntervals,presentationCalibrationIntervals);
assert.equal(sixty.clock.report.divisor,1);
assert.equal(sixty.submitted,120);
assert.equal(sixty.clock.report.skippedCallbacks,0);

const oneTwenty=simulate(1000/120,360);
assert.equal(oneTwenty.clock.report.divisor,2);
assert.ok(Math.abs(oneTwenty.clock.report.targetMs-1000/60)<1e-8);
assert.equal(oneTwenty.submitted,60);
assert.equal(oneTwenty.clock.report.skippedCallbacks,60);
assert.equal(oneTwenty.clock.report.missedTargetSlots,0);
oneTwenty.clock.tick(4000);
assert.ok(oneTwenty.clock.report.missedTargetSlots>0);
const adjacent=new PresentationClock();
for(let i=0;i<=presentationCalibrationIntervals+2;++i)adjacent.tick(i*8.3);
assert.equal(adjacent.report.missedTargetSlots,0);
adjacent.tick((presentationCalibrationIntervals+2)*8.3+25);
assert.equal(adjacent.report.missedTargetSlots,1);
oneTwenty.clock.reset();
assert.equal(oneTwenty.clock.calibrated,false);
assert.equal(oneTwenty.clock.report.submittedFrames,0);

assert.equal(simulate(1000/75,240).clock.report.targetSupported,false);
const invalid=new PresentationClock();
assert.throws(()=>invalid.tick(NaN),/Invalid/);
invalid.tick(1);
assert.equal(invalid.tick(1),false);
assert.equal(invalid.report.duplicateCallbacks,1);
assert.equal(invalid.tick(0),false);
assert.equal(invalid.report.timestampDiscontinuities,1);
assert.equal(invalid.report.calibratedIntervals,0);
for(let i=1;i<=presentationCalibrationIntervals;++i)
  assert.equal(invalid.tick(i*1000/60),false);
assert.equal(invalid.calibrated,true);
assert.equal(invalid.report.targetSupported,true);
assert.equal(invalid.tick((presentationCalibrationIntervals+1)*1000/60),true);
// A 0.1 ms median grid cannot track real 60/120/240 Hz indefinitely. Pacing
// must phase-lock without submitting beyond the calibrated target capacity.
for(const hz of [60,120,240])for(const duration of [60,300]){
  const clock=new PresentationClock();let seed=0x735f210,previous=null;const gaps=[];
  for(let i=0;i<=hz*duration+presentationCalibrationIntervals;++i){
    seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;
    const raf=Math.round((i*1000/hz+((seed>>>0)/4294967296-.5))*10)/10;
    if(clock.tick(raf)){
      if(previous!==null)gaps.push(raf-previous);
      previous=raf;
    }
  }
  const {targetMs,submittedFrames,missedTargetSlots}=clock.report;
  gaps.sort((a,b)=>a-b);
  const expected=duration*1000/targetMs;
  assert.ok(submittedFrames<=Math.ceil(expected)+2);
  assert.ok(submittedFrames>.99*expected);
  assert.ok(missedTargetSlots/(gaps.length+missedTargetSlots)<=.01);
  assert.ok(gaps[Math.ceil(.95*gaps.length)-1]<=1.25*targetMs);
  assert.ok(gaps[Math.ceil(.99*gaps.length)-1]<=2*targetMs+1);
  assert.ok(gaps.at(-1)<=100);
}
console.log('Presentation clock: bounded calibration, 60/120 Hz pacing, missed slots and unsupported refresh reporting PASS');
