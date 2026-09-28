export interface Calibration60 {
  readonly intervalCount:240;readonly refreshPeriodMs:number;readonly targetPeriodMs:number;
  readonly divisor:number;readonly effectiveHz:number;readonly supports60Hz:boolean;
}
/** Exactly 240 idle rAF intervals, integer refresh divisor, and 59–61 Hz gate.
 * Whole-millisecond rAF clocks quantize a 60 Hz interval to 16 or 17 ms.
 * The median of ten nonoverlapping 24-interval means recovers their cadence
 * without accepting a display whose actual period is a steady 17 ms.
 */
export function calibrate60(intervals:Float64Array):Calibration60 {
  if(intervals.length!==240)throw new RangeError('Calibration requires 240 intervals');
  for(const value of intervals)if(!Number.isFinite(value)||value<=0)throw new RangeError('Invalid refresh interval');
  let refreshPeriodMs:number;
  if(intervals.every(Number.isInteger)){
    const blocks=new Float64Array(10);
    for(let block=0;block<10;++block){let span=0;for(let i=0;i<24;++i)span+=intervals[block*24+i];blocks[block]=span/24;}
    blocks.sort();refreshPeriodMs=blocks[4];
  }else{
    const sorted=intervals.slice().sort();refreshPeriodMs=sorted[119];
  }
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
  private targetOriginRaf=0;
  frameCount=0;steps=0;totalSteps=0;debtSeconds=0;droppedSeconds=0;elapsedSeconds=0;targetRafMs=0;
  constructor(calibration:Calibration60,readonly timestepSeconds:number,readonly renderOnly=false,
      private readonly startNowMs?:number){
    if(!calibration.supports60Hz||!Number.isFinite(calibration.refreshPeriodMs)||calibration.refreshPeriodMs<=0||
        !Number.isFinite(calibration.targetPeriodMs)||calibration.targetPeriodMs<calibration.refreshPeriodMs||
        1000/calibration.targetPeriodMs<59||1000/calibration.targetPeriodMs>61||
        timestepSeconds!==Math.fround(1/60)||typeof renderOnly!=='boolean'||
        (startNowMs!==undefined&&(!Number.isFinite(startNowMs)||startNowMs<0)))throw new RangeError('Unsupported study clock configuration');
    this.refreshPeriodMs=calibration.refreshPeriodMs;this.targetPeriodMs=calibration.targetPeriodMs;
  }
  tick(rafMs:number,nowMs:number):boolean {
    if(!Number.isFinite(rafMs)||!Number.isFinite(nowMs)||rafMs<0||nowMs<0||
        rafMs<this.previousRaf||nowMs<this.previousNow||
        (this.previousRaf<0&&this.startNowMs!==undefined&&nowMs<this.startNowMs))throw new RangeError('Invalid or backwards clock');
    const first=this.previousRaf<0;
    // Phase-locking must not submit faster than the calibrated target on a
    // coarsened 16.7/4.2 ms median. The absolute slot ceiling also preserves
    // the preallocated duration/T frame capacity without increasing it.
    const quota=first?rafMs:this.targetOriginRaf+this.frameCount*this.targetPeriodMs;
    const deadline=first?rafMs:Math.max(this.deadline,quota);
    // Select the callback nearest each calibrated target. Half a native refresh
    // avoids alternating missed deadlines from sub-millisecond rAF jitter; at
    // high refresh it still selects only one callback per integer target slot.
    const horizon=rafMs+this.refreshPeriodMs*.5;
    if(horizon<deadline){this.previousRaf=rafMs;this.previousNow=nowMs;return false;}
    // A first callback can arrive several refreshes after its rAF timestamp
    // (for example, driver initialization finished late). Start on the target
    // nearest its actual entry, not on that stale timestamp's overdue slot.
    // Keep the original calibrated phase; no warm-up time enters simulation.
    const skipped=first?Math.max(0,Math.round((nowMs-rafMs)/this.targetPeriodMs)):
      Math.floor((horizon-deadline)/this.targetPeriodMs);
    const target=deadline+skipped*this.targetPeriodMs;
    // Re-anchor after an accepted native refresh. Chromium can round a
    // 120 Hz median to 8.3 ms while the display actually advances by about
    // 8.333 ms. Keeping one absolute 16.6 ms grid then oscillates between
    // adjacent callbacks, yielding avoidable 8/25 ms submission gaps. The
    // calibrated target period and every acceptance budget stay unchanged.
    const next=(first?target:rafMs)+this.targetPeriodMs;
    const origin=first?(this.startNowMs??nowMs):this.origin,previous=first?origin:this.renderedNow;
    // Frozen render-only runs keep presentation deadlines and elapsed time but
    // execute no simulation. Their source's 120 preparation steps are separate.
    const accumulated=this.renderOnly?0:this.debtSeconds+(nowMs-previous)/1000;
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
    if(first)this.targetOriginRaf=rafMs;
    this.targetRafMs=target;this.steps=steps;this.totalSteps+=steps;this.debtSeconds=debt;this.droppedSeconds=dropped;
    this.elapsedSeconds=(nowMs-origin)/1000;++this.frameCount;return true;
  }
}
