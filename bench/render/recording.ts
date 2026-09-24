/** Developer study record, independent of renderer and clock implementation.
 * All steady-state storage is allocated by the constructor. Unknown optional
 * metrics store zero with their availability bit cleared; zero is not evidence
 * of a measurement. Reporting/sorting allocate only after collection.
 */
export const numberNames=Object.freeze(['rafMs','callbackMs','targetMs','submissionMs','endMs',
  'stepMs','snapshotMs','prepareMs','drawMs','counterMs','hudMs','recordMs','debtSeconds','droppedSeconds',
  'gpuMs','uploadBytes','poseCopyBytes','posePrepareBytes','diagnosticCopyBytes','diagnosticPrepareBytes',
  'gpuBytes','drawCalls','submissionCalls','gpuPollMs']);
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
        (i>=15&&i<=22&&!Number.isSafeInteger(values[i]))||(!(known&(1<<i))&&values[i]!==0))
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
  /** Fixed ten-second windows, allocated only after collection. A frame and
   * its incoming submission gap belong to the window containing its callback
   * entry. Thus a boundary-crossing stall is retained exactly once. The final
   * boundary/overshoot frame stays in the final window. Empty/unfinished
   * windows never fabricate zero-valued distributions or complete coverage.
   */
  windows(targetPeriodMs:number,durationSeconds:number,initialSteps=0,initialContactDrops=0n){
    if(!Number.isFinite(targetPeriodMs)||targetPeriodMs<=0||!Number.isFinite(durationSeconds)||
      durationSeconds<=0||durationSeconds>300||!Number.isInteger(initialSteps)||initialSteps<0||initialSteps>0xffffffff||
      typeof initialContactDrops!=='bigint'||initialContactDrops<0n||initialContactDrops>0xffffffffffffffffn)
      throw new RangeError('Invalid window configuration');
    const count=Math.ceil(durationSeconds/10),origin=this.rows?this.numbers[1]:0;
    const windows=Array.from({length:count},()=>({cpu:[] as number[],gaps:[] as number[],gpu:[] as number[],
      missing:0,firstRow:null as number|null,lastRow:null as number|null,pending:false,
      steps:0,dropped:0,contacts:0n,endDebt:null as number|null,allocatorMax:null as number|null}));
    let previousSubmission:number|undefined,previousSteps=initialSteps,previousDropped=0,previousContacts=initialContactDrops,lastElapsed=-1;
    for(let row=0;row<this.rows;++row){
      const at=row*numberNames.length,elapsed=(this.numbers[at+1]-origin)/1000;
      const w=windows[Math.min(count-1,Math.floor(elapsed/10))];
      if(!w||!Number.isFinite(elapsed)||elapsed<lastElapsed)throw new Error('Invalid window clock');
      if(!this.finished[row]){w.pending=true;continue;}
      const submission=this.numbers[at+3],steps=this.stats[row*25+24],dropped=this.numbers[at+13],contacts=this.work[row*26+25];
      if(steps<previousSteps||dropped<previousDropped||contacts<previousContacts)throw new Error('Cumulative window counters moved backwards');
      if(w.firstRow===null)w.firstRow=row;w.lastRow=row;lastElapsed=elapsed;
      w.cpu.push(this.numbers[at+4]-this.numbers[at+1]);
      if(this.known[row]&(1<<14))w.gpu.push(this.numbers[at+14]);
      if(previousSubmission!==undefined){
        const gap=submission-previousSubmission,missing=Math.max(0,Math.round(gap/targetPeriodMs)-1);
        if(!Number.isSafeInteger(missing)||!Number.isSafeInteger(w.missing+missing))throw new RangeError('Window cadence exceeds safe integer range');
        w.gaps.push(gap);w.missing+=missing;
      }
      w.steps+=steps-previousSteps;w.dropped+=dropped-previousDropped;w.contacts+=contacts-previousContacts;
      w.endDebt=this.numbers[at+12];w.allocatorMax=Math.max(w.allocatorMax??0,this.stats[row*25+23]);
      previousSubmission=submission;previousSteps=steps;previousDropped=dropped;previousContacts=contacts;
    }
    return windows.map((w,index)=>({index,startSeconds:10*index,endSeconds:Math.min(durationSeconds,10*(index+1)),
      coverageReached:w.cpu.length>0&&!w.pending&&lastElapsed>=Math.min(durationSeconds,10*(index+1)),pendingFrame:w.pending,
      firstRow:w.firstRow,lastRow:w.lastRow,completeFrames:w.cpu.length,
      observedFirstSeconds:w.firstRow===null?null:(this.numbers[w.firstRow*numberNames.length+1]-origin)/1000,
      observedLastSeconds:w.lastRow===null?null:(this.numbers[w.lastRow*numberNames.length+1]-origin)/1000,
      cpuMs:distribution(w.cpu),gpuMs:distribution(w.gpu),gpuUnavailableFrames:w.cpu.length-w.gpu.length,
      submissionGapMs:distribution(w.gaps),missedTargetSlots:w.missing,submittedIntervals:w.gaps.length,
      missedTargetFraction:w.gaps.length+w.missing?w.missing/(w.gaps.length+w.missing):null,
      executedSteps:w.steps,droppedSeconds:w.dropped,contactDrops:String(w.contacts),endDebtSeconds:w.endDebt,
      allocatorBytesMax:w.allocatorMax}));
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
