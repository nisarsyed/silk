// Optional host-only frame measurements. Times use the owner's monotonic
// performance clock in milliseconds; the copied report includes its origin.
// A full preallocated trace fails instead of dropping a slow frame.
export const frameTraceCapacity=131072;

export class FrameTrace{
  constructor(capacity=frameTraceCapacity){
    if(!Number.isInteger(capacity)||capacity<1||capacity>frameTraceCapacity)
      throw new RangeError('Invalid frame trace capacity');
    this.capacity=capacity;this.count=0;
    this.callbackMs=new Float64Array(capacity);
    this.scheduleMs=new Float64Array(capacity);
    this.stepMs=new Float64Array(capacity);
    this.drawMs=new Float64Array(capacity);
    this.submittedMs=new Float64Array(capacity);
    this.steps=new Uint8Array(capacity);
    this.storageBytes=this.callbackMs.byteLength+this.scheduleMs.byteLength+
      this.stepMs.byteLength+this.drawMs.byteLength+this.submittedMs.byteLength+
      this.steps.byteLength;
  }
  record(callbackMs,scheduleEndMs,stepEndMs,submittedMs,steps){
    if(this.count===this.capacity)throw new Error('Frame trace capacity exhausted');
    if(!Number.isFinite(callbackMs)||!Number.isFinite(scheduleEndMs)||
        !Number.isFinite(stepEndMs)||!Number.isFinite(submittedMs)||
        callbackMs<0||scheduleEndMs<callbackMs||stepEndMs<scheduleEndMs||
        submittedMs<stepEndMs||submittedMs>Number.MAX_SAFE_INTEGER||
        !Number.isInteger(steps)||steps<0||steps>8)
      throw new RangeError('Invalid frame trace sample');
    const at=this.count++;
    this.callbackMs[at]=callbackMs;
    this.scheduleMs[at]=scheduleEndMs-callbackMs;
    this.stepMs[at]=stepEndMs-scheduleEndMs;
    this.drawMs[at]=submittedMs-stepEndMs;
    this.submittedMs[at]=submittedMs;
    this.steps[at]=steps;
  }
  copy(timeOrigin){
    if(!Number.isFinite(timeOrigin)||timeOrigin<0||timeOrigin>Number.MAX_SAFE_INTEGER)
      throw new RangeError('Invalid time origin');
    const n=this.count;
    return {capacity:this.capacity,count:n,storageBytes:this.storageBytes,timeOrigin,
      callbackMs:this.callbackMs.slice(0,n),scheduleMs:this.scheduleMs.slice(0,n),
      stepMs:this.stepMs.slice(0,n),drawMs:this.drawMs.slice(0,n),
      submittedMs:this.submittedMs.slice(0,n),steps:this.steps.slice(0,n)};
  }
}
