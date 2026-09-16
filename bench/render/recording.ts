/** Developer study record, independent of renderer and clock implementation.
 * All steady-state storage is allocated by the constructor. Unknown optional
 * metrics store zero with their availability bit cleared; zero is not evidence
 * of a measurement. Reporting/sorting allocate only after collection.
 */
export const numberNames=Object.freeze(['rafMs','callbackMs','targetMs','submissionMs','endMs',
  'stepMs','snapshotMs','prepareMs','drawMs','counterMs','hudMs','recordMs','debtSeconds','droppedSeconds',
  'gpuMs','uploadBytes','poseCopyBytes','posePrepareBytes','diagnosticCopyBytes','diagnosticPrepareBytes',
  'gpuBytes','drawCalls','submissionCalls']);
export const requiredNumbers=(1<<14)-1;
export const allNumbers=(1<<numberNames.length)-1;
// 300 seconds at up to 240 recorded submissions/s, plus two boundary frames.
// The actual runner allocates only its duration/calibration-derived capacity.
export const frameCapacityMax=300*240+2;
export interface FrameCounters {
  readonly valid:boolean;readonly stats:Uint32Array;readonly work:BigUint64Array;
  readonly statNames:readonly string[];readonly workNames:readonly string[];
}
function distribution(values:number[]){
  if(!values.length)return null;
  values.sort((a,b)=>a-b);
  const rank=(p:number)=>values[Math.ceil(p*values.length)-1];
  return {count:values.length,p50:rank(.5),p95:rank(.95),p99:rank(.99),max:values[values.length-1]};
}
export class FrameRecorder {
  readonly numbers:Float64Array;readonly stats:Uint32Array;readonly work:BigUint64Array;
  readonly known:Uint32Array;readonly finished:Uint8Array;
  readonly bytes:number;
  private rows=0;private pending=false;private recordBegin=0;
  constructor(readonly capacity:number){
    if(!Number.isInteger(capacity)||capacity<1||capacity>frameCapacityMax)throw new RangeError('Invalid frame capacity');
    this.numbers=new Float64Array(capacity*numberNames.length);
    this.stats=new Uint32Array(capacity*25);this.work=new BigUint64Array(capacity*26);
    this.known=new Uint32Array(capacity);this.finished=new Uint8Array(capacity);
    this.bytes=this.numbers.byteLength+this.stats.byteLength+this.work.byteLength+this.known.byteLength+this.finished.byteLength;
  }
  get count():number{return this.rows;}
  get hasPendingFrame():boolean{return this.pending;}
  append(values:Float64Array,known:number,counters:FrameCounters):number {
    if(this.pending)throw new Error('Finish the previous frame before appending');
    if(this.rows===this.capacity)throw new RangeError('Frame recording capacity exhausted');
    if(values.length!==numberNames.length||!Number.isInteger(known)||known<0||known>allNumbers||
        (known&requiredNumbers)!==requiredNumbers||!counters.valid||counters.stats.length!==25||counters.work.length!==26)
      throw new Error('Invalid frame data');
    for(let i=0;i<values.length;++i)if(!Number.isFinite(values[i])||values[i]<0||
        (i>=15&&!Number.isSafeInteger(values[i]))||(!(known&(1<<i))&&values[i]!==0))
      throw new Error('Invalid frame metric or availability');
    if(values[3]<values[1]||values[4]<values[3]||values[11]!==0)throw new Error('Invalid frame clock order');
    const row=this.rows;
    if(row>0&&(values[1]<this.numbers[(row-1)*numberNames.length+1]||
        values[3]<this.numbers[(row-1)*numberNames.length+3]||counters.stats[24]<this.stats[(row-1)*25+24]))
      throw new Error('Frame clocks or steps moved backwards');
    this.numbers.set(values,row*numberNames.length);this.stats.set(counters.stats,row*25);this.work.set(counters.work,row*26);
    this.recordBegin=values[4];this.numbers[row*numberNames.length+4]=0;
    this.known[row]=known&~((1<<4)|(1<<11));this.pending=true;++this.rows;return row;
  }
  // Call immediately after append with the same monotonic clock. Critical-path
  // duration includes record writes; submission completion remains distinct.
  finish(endMs:number):void {
    if(!this.pending)throw new Error('No pending frame');
    const at=(this.rows-1)*numberNames.length,begin=this.recordBegin;
    if(!Number.isFinite(endMs)||endMs<begin)throw new Error('Invalid recording completion time');
    this.numbers[at+4]=endMs;this.numbers[at+11]=endMs-begin;
    this.known[this.rows-1]|=(1<<4)|(1<<11);this.finished[this.rows-1]=1;this.pending=false;
  }
  // A nonblocking GPU query may resolve after later CPU frames. The query
  // owner supplies its original row; CPU clocks and work records stay intact.
  resolveGpu(row:number,elapsedMs:number):void {
    if(!Number.isInteger(row)||row<0||row>=this.rows||!this.finished[row]||
        !Number.isFinite(elapsedMs)||elapsedMs<0||(this.known[row]&(1<<14)))throw new Error('Invalid or duplicate GPU sample');
    this.numbers[row*numberNames.length+14]=elapsedMs;this.known[row]|=1<<14;
  }
  summary(targetPeriodMs:number){
    if(!Number.isFinite(targetPeriodMs)||targetPeriodMs<=0)throw new RangeError('Invalid target period');
    const cpu:number[]=[],gaps:number[]=[];let previous:number|undefined,missing=0;
    for(let row=0;row<this.rows;++row){
      if(!this.finished[row])continue;
      const at=row*numberNames.length,submission=this.numbers[at+3];
      cpu.push(this.numbers[at+4]-this.numbers[at+1]);
      if(previous!==undefined){
        const gap=submission-previous,slots=Math.max(0,Math.round(gap/targetPeriodMs)-1);
        if(!Number.isSafeInteger(slots)||!Number.isSafeInteger(missing+slots))throw new RangeError('Cadence slot count exceeds safe integer range');
        gaps.push(gap);missing+=slots;
      }
      previous=submission;
    }
    return {completeFrames:cpu.length,pendingFrame:this.pending,cpuMs:distribution(cpu),submissionGapMs:distribution(gaps),
      missedTargetSlots:missing,submittedIntervals:gaps.length,
      missedTargetFraction:gaps.length+missing?missing/(gaps.length+missing):null};
  }
  report(counters:FrameCounters){
    if(counters.statNames.length!==25||counters.workNames.length!==13)throw new Error('Invalid counter schema');
    // Explicit raw row-major schema. uint64 values remain decimal strings.
    return {schema:1,kind:'frame-records',capacity:this.capacity,count:this.rows,bytes:this.bytes,pendingFrame:this.pending,
      numberNames,statNames:[...counters.statNames],workNames:[...counters.workNames],workOrder:['lastStep','cumulative'],
      availability:'known bit i identifies numberNames[i]; cleared bits are unavailable, not measured zero',
      numbers:Array.from(this.numbers.subarray(0,this.rows*numberNames.length)),
      stats:Array.from(this.stats.subarray(0,this.rows*25)),work:Array.from(this.work.subarray(0,this.rows*26),String),
      known:Array.from(this.known.subarray(0,this.rows)),finished:Array.from(this.finished.subarray(0,this.rows))};
  }
}
