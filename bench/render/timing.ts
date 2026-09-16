export interface Calibration60 {
  readonly intervalCount:240;readonly refreshPeriodMs:number;readonly targetPeriodMs:number;
  readonly divisor:number;readonly effectiveHz:number;readonly supports60Hz:boolean;
}
/** Frozen mandatory 60 Hz calibration: exactly 240 idle rAF intervals,
 * nearest-rank median, integer refresh divisor, and unchanged 59–61 Hz gate.
 * Unsupported displays retain their measured result instead of being coerced.
 */
export function calibrate60(intervals:Float64Array):Calibration60 {
  if(intervals.length!==240)throw new RangeError('Calibration requires 240 intervals');
  for(const value of intervals)if(!Number.isFinite(value)||value<=0)throw new RangeError('Invalid refresh interval');
  const sorted=intervals.slice().sort(),refreshPeriodMs=sorted[119];
  const divisor=Math.max(1,Math.round((1000/60)/refreshPeriodMs)),targetPeriodMs=refreshPeriodMs*divisor;
  const effectiveHz=1000/targetPeriodMs;
  return Object.freeze({intervalCount:240,refreshPeriodMs,targetPeriodMs,divisor,effectiveHz,
    supports60Hz:effectiveHz>=59&&effectiveHz<=61});
}
/** Clock-only schedule; no browser globals or simulation mutation. One accepted
 * tick describes at most eight fixed steps. The host must execute that plan or
 * fail the run, and compare totalSteps with the authoritative world count.
 * rAF and entry times must share the recorded monotonic time origin.
 */
export class StudyClock {
  readonly refreshPeriodMs:number;readonly targetPeriodMs:number;
  private previousRaf=-1;private previousNow=-1;private renderedNow=0;private origin=0;private deadline=0;
  frameCount=0;steps=0;totalSteps=0;debtSeconds=0;droppedSeconds=0;elapsedSeconds=0;targetRafMs=0;
  constructor(calibration:Calibration60,readonly timestepSeconds:number){
    if(!calibration.supports60Hz||!Number.isFinite(calibration.refreshPeriodMs)||calibration.refreshPeriodMs<=0||
        !Number.isFinite(calibration.targetPeriodMs)||calibration.targetPeriodMs<calibration.refreshPeriodMs||
        1000/calibration.targetPeriodMs<59||1000/calibration.targetPeriodMs>61||
        timestepSeconds!==Math.fround(1/60))throw new RangeError('Unsupported study clock configuration');
    this.refreshPeriodMs=calibration.refreshPeriodMs;this.targetPeriodMs=calibration.targetPeriodMs;
  }
  tick(rafMs:number,nowMs:number):boolean {
    if(!Number.isFinite(rafMs)||!Number.isFinite(nowMs)||rafMs<0||nowMs<0||
        rafMs<this.previousRaf||nowMs<this.previousNow)throw new RangeError('Invalid or backwards clock');
    const first=this.previousRaf<0,deadline=first?rafMs:this.deadline;
    // Select the callback nearest each calibrated target. Half a native refresh
    // avoids alternating missed deadlines from sub-millisecond rAF jitter; at
    // high refresh it still selects only one callback per integer target slot.
    const horizon=rafMs+this.refreshPeriodMs*.5;
    if(horizon<deadline){this.previousRaf=rafMs;this.previousNow=nowMs;return false;}
    const skipped=Math.floor((horizon-deadline)/this.targetPeriodMs);
    const target=deadline+skipped*this.targetPeriodMs,next=target+this.targetPeriodMs;
    const origin=first?nowMs:this.origin,previous=first?nowMs:this.renderedNow;
    const accumulated=this.debtSeconds+(nowMs-previous)/1000;
    let possible=Math.floor(accumulated/this.timestepSeconds);
    if(!Number.isSafeInteger(skipped)||!Number.isFinite(next)||!Number.isSafeInteger(possible))throw new RangeError('Clock range exhausted');
    // Correct only a division rounding across an exact step boundary. No
    // epsilon changes dt or discards debt. The products remain observable.
    if(possible*this.timestepSeconds>accumulated)--possible;
    let debt=accumulated-possible*this.timestepSeconds;
    if(debt>=this.timestepSeconds){++possible;debt-=this.timestepSeconds;}
    const steps=Math.min(8,possible),dropped=this.droppedSeconds+(possible-steps)*this.timestepSeconds;
    if(!Number.isSafeInteger(this.totalSteps+steps)||!Number.isSafeInteger(this.frameCount+1)||
        !Number.isFinite(dropped)||debt<0||debt>=this.timestepSeconds)throw new RangeError('Invalid scheduling arithmetic');
    this.previousRaf=rafMs;this.previousNow=nowMs;this.renderedNow=nowMs;this.origin=origin;this.deadline=next;
    this.targetRafMs=target;this.steps=steps;this.totalSteps+=steps;this.debtSeconds=debt;this.droppedSeconds=dropped;
    this.elapsedSeconds=(nowMs-origin)/1000;++this.frameCount;return true;
  }
}
