// Optional host-only measurement. Times are absolute monotonic milliseconds:
// performance.timeOrigin + a context-local event/rAF clock. Record both time
// origins when comparing main and worker clocks. All slots are allocated before
// stepping; capacity exhaustion fails instead of discarding latency samples.
export const inputTraceCapacity=8192;

export class InputTrace{
  constructor(capacity=inputTraceCapacity){
    if(!Number.isInteger(capacity)||capacity<1||capacity>inputTraceCapacity)
      throw new RangeError('Invalid input trace capacity');
    this.capacity=capacity;this.count=0;
    this.epoch=new Float64Array(capacity);this.sequence=new Float64Array(capacity);
    this.action=new Uint8Array(capacity);
    this.eventMs=new Float64Array(capacity);this.appliedMs=new Float64Array(capacity);
    this.submittedMs=new Float64Array(capacity);
    this.storageBytes=this.epoch.byteLength+this.sequence.byteLength+
      this.action.byteLength+this.eventMs.byteLength+this.appliedMs.byteLength+
      this.submittedMs.byteLength;
  }
  record(epoch,sequence,action,eventMs,appliedMs){
    if(this.count===this.capacity)throw new Error('Input trace capacity exhausted');
    if(!Number.isSafeInteger(epoch)||epoch<1||!Number.isSafeInteger(sequence)||sequence<1||
        !Number.isInteger(action)||action<0||action>3||
        !Number.isFinite(eventMs)||eventMs<=0||eventMs>Number.MAX_SAFE_INTEGER||
        !Number.isFinite(appliedMs)||appliedMs>Number.MAX_SAFE_INTEGER||
        appliedMs<eventMs)throw new RangeError('Invalid input trace time or action');
    const at=this.count++;
    this.epoch[at]=epoch;this.sequence[at]=sequence;this.action[at]=action;
    this.eventMs[at]=eventMs;this.appliedMs[at]=appliedMs;
  }
  submit(first,submittedMs){
    if(!Number.isInteger(first)||first<0||first>this.count||
        !Number.isFinite(submittedMs)||submittedMs>Number.MAX_SAFE_INTEGER)
      throw new RangeError('Invalid input submission');
    for(let i=first;i<this.count;++i){
      if(submittedMs<this.appliedMs[i])throw new RangeError('Submission preceded input application');
      this.submittedMs[i]=submittedMs;
    }
  }
  copy(timeOrigin){
    if(!Number.isFinite(timeOrigin)||timeOrigin<0||timeOrigin>Number.MAX_SAFE_INTEGER)
      throw new RangeError('Invalid time origin');
    const n=this.count;
    return {capacity:this.capacity,count:n,storageBytes:this.storageBytes,timeOrigin,
      epoch:this.epoch.slice(0,n),sequence:this.sequence.slice(0,n),
      action:this.action.slice(0,n),eventMs:this.eventMs.slice(0,n),
      appliedMs:this.appliedMs.slice(0,n),submittedMs:this.submittedMs.slice(0,n)};
  }
}
