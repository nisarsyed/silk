// Browser presentation scheduling only. The C world keeps its fixed 60 Hz step.
// Calibration is bounded to 240 idle rAF intervals before timed collection.
export const presentationCalibrationIntervals=240;

export class PresentationClock{
  #lastMs=null;
  #count=0;
  #calibrated=false;
  #targetSupported=null;
  #medianMs=null;
  #targetMs=null;
  #divisor=null;
  #nextMs=null;
  #lastSubmittedMs=null;
  #phaseOriginMs=null;
  #phaseSubmitted=0;
  #submitted=0;
  #skipped=0;
  #missed=0;
  #duplicates=0;
  #discontinuities=0;
  #intervals=new Float64Array(presentationCalibrationIntervals);

  get calibrated(){return this.#calibrated;}
  get report(){return {calibrated:this.#calibrated,calibratedIntervals:this.#count,
    medianMs:this.#medianMs,targetMs:this.#targetMs,divisor:this.#divisor,
    targetSupported:this.#targetSupported,
    submittedFrames:this.#submitted,skippedCallbacks:this.#skipped,
    missedTargetSlots:this.#missed,duplicateCallbacks:this.#duplicates,
    timestampDiscontinuities:this.#discontinuities};}

  #restart(rafMs){
    this.#lastMs=rafMs;this.#count=0;this.#calibrated=false;
    this.#medianMs=null;this.#targetMs=null;this.#divisor=null;this.#nextMs=null;
    this.#lastSubmittedMs=null;
    this.#phaseOriginMs=null;this.#phaseSubmitted=0;
    this.#targetSupported=null;
  }

  reset(){
    this.#restart(null);
    this.#submitted=0;this.#skipped=0;this.#missed=0;
    this.#duplicates=0;this.#discontinuities=0;
  }

  tick(rafMs){
    if(!Number.isFinite(rafMs)||rafMs<0||rafMs>Number.MAX_SAFE_INTEGER)
      throw new RangeError('Invalid presentation timestamp');
    if(this.#lastMs!==null&&rafMs===this.#lastMs){
      ++this.#duplicates;return false;
    }
    if(this.#lastMs!==null&&rafMs<this.#lastMs){
      // A browser/display clock discontinuity invalidates the old cadence.
      // Recalibrate on idle callbacks; the owner suspends this time in its
      // fixed-step clock. Keep the counter visible to reject a timed run.
      ++this.#discontinuities;this.#restart(rafMs);return false;
    }
    const previous=this.#lastMs;this.#lastMs=rafMs;
    if(previous===null)return false;
    if(!this.#calibrated){
      this.#intervals[this.#count++]=rafMs-previous;
      if(this.#count===presentationCalibrationIntervals){
        this.#intervals.sort();
        const median=(this.#intervals[119]+this.#intervals[120])/2;
        const divisor=Math.max(1,Math.round((1000/60)/median));
        const target=divisor*median;
        // An unsupported display remains usable, but cannot qualify a 60 Hz
        // physical acceptance run until its refresh setting is changed.
        this.#targetSupported=target>=1000/61&&target<=1000/59;
        this.#medianMs=median;this.#divisor=divisor;this.#targetMs=target;
        this.#nextMs=rafMs+target;this.#calibrated=true;
      }
      return false;
    }
    // Submit on the callback nearest the target, at most a quarter of one
    // calibrated rAF interval early. Late callback gaps remain observable.
    if(rafMs+this.#medianMs/4<this.#nextMs){++this.#skipped;return false;}
    // Count gaps in accepted rAF callbacks, not just expired arithmetic
    // deadlines. The latter misses a real 25 ms gap if quantized timestamps
    // make two successive callbacks straddle one fixed deadline.
    if(this.#lastSubmittedMs!==null)
      this.#missed+=Math.max(0,Math.round((rafMs-this.#lastSubmittedMs)/this.#targetMs)-1);
    this.#lastSubmittedMs=rafMs;
    // Re-anchor after each accepted native refresh. A coarsened 8.3 ms median
    // cannot preserve the phase of an actual 8.333 ms display over long runs.
    // The absolute slot ceiling also prevents an upward-rounded median from
    // submitting faster than its calibrated target on 60/240 Hz displays.
    if(this.#phaseOriginMs===null)this.#phaseOriginMs=rafMs;
    ++this.#phaseSubmitted;
    this.#nextMs=Math.max(rafMs+this.#targetMs,
      this.#phaseOriginMs+this.#phaseSubmitted*this.#targetMs);
    ++this.#submitted;return true;
  }
}
