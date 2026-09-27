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
console.log('Presentation clock: bounded calibration, 60/120 Hz pacing, missed slots and unsupported refresh reporting PASS');
