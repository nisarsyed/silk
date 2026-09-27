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
// A 0.1 ms median grid cannot track a real 120/240 Hz display indefinitely.
// Pacing must phase-lock to accepted callbacks without weakening the target.
for(const hz of [120,240]){
  const clock=new PresentationClock();let seed=0x735f210,previous=null,maximumGap=0;
  for(let i=0;i<=hz*60+presentationCalibrationIntervals;++i){
    seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;
    const raf=Math.round((i*1000/hz+((seed>>>0)/4294967296-.5))*10)/10;
    if(clock.tick(raf)){
      if(previous!==null)maximumGap=Math.max(maximumGap,raf-previous);
      previous=raf;
    }
  }
  assert.ok(clock.report.submittedFrames>=3590&&clock.report.submittedFrames<=3610);
  assert.equal(clock.report.missedTargetSlots,0);
  assert.ok(maximumGap<1.25*clock.report.targetMs);
}
console.log('Presentation clock: bounded calibration, 60/120 Hz pacing, missed slots and unsupported refresh reporting PASS');
