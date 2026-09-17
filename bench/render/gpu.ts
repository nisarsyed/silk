import {FrameRecorder,frameCapacityMax} from './recording.js';

interface TimerExtension {TIME_ELAPSED_EXT:number;GPU_DISJOINT_EXT:number;QUERY_COUNTER_BITS_EXT:number}
export const gpuSampleNames=Object.freeze(['not-attempted','pending','resolved','pool-full','disjoint',
  'invalid-result','unsupported','aborted','drain-timeout']);
// At the recorder's maximum 240 Hz: 100 ms of pending work plus two boundary
// frames. A run uses ceil(100 / calibrated target period) + 2, never this
// maximum by default. Exhaustion preserves the frame with unavailable timing.
export const queryCapacityMax=26;

/** Optional elapsed-command timing, not GPU utilization or presentation proof.
 * Create queries once, return to the browser between issue and poll, and only
 * read results after availability. A disjoint check AFTER reading the ready
 * prefix validates those values; a disjoint event invalidates all pending
 * queries. Previously committed samples were validated by an earlier check.
 * No finish, readPixels, spin, query allocation or deletion occurs per frame.
 * See the Khronos EXT_disjoint_timer_query_webgl2 and EXT_disjoint_timer_query
 * specifications. Renderer code itself remains free of query-status polling.
 */
export class GpuTimer {
  readonly statuses:Uint8Array;
  readonly bytes:number;
  readonly supported:boolean;
  readonly unavailableReason:string|null;
  readonly counterBits:number|null;
  private readonly extension:TimerExtension|null;
  private readonly queries:WebGLQuery[]=[];
  private readonly rows:Uint32Array;
  private readonly began:Float64Array;
  private readonly results:Float64Array;
  private head=0;private pending=0;private active=false;private stopped=false;private disposed=false;private attempts=0;private lastNow=-1;
  private disjointEvents=0;private polls=0;private maximumPending=0;
  constructor(private readonly gl:WebGL2RenderingContext|null,readonly frameCapacity:number,readonly queryCapacity:number){
    if(!Number.isInteger(frameCapacity)||frameCapacity<1||frameCapacity>frameCapacityMax||
      !Number.isInteger(queryCapacity)||queryCapacity<1||queryCapacity>queryCapacityMax)throw new RangeError('Invalid GPU timing capacity');
    this.statuses=new Uint8Array(frameCapacity);this.rows=new Uint32Array(queryCapacity);
    this.began=new Float64Array(queryCapacity);this.results=new Float64Array(queryCapacity);
    this.bytes=this.statuses.byteLength+this.rows.byteLength+this.began.byteLength+this.results.byteLength;
    this.extension=gl?.getExtension('EXT_disjoint_timer_query_webgl2')??null;
    let reason:string|null=gl?'extension-unavailable':'no-webgl-context',bits:number|null=null;
    if(gl&&this.extension){
      // lib.dom models core getQuery's CURRENT_QUERY return; the extension
      // adds the numeric QUERY_COUNTER_BITS_EXT result documented by Khronos.
      const queried:unknown=gl.getQuery(this.extension.TIME_ELAPSED_EXT,this.extension.QUERY_COUNTER_BITS_EXT);
      bits=typeof queried==='number'&&Number.isFinite(queried)?queried:null;
      if(bits===null||!Number.isInteger(bits)||bits<30||bits>64)reason='counter-bits-unavailable';
      else if(gl.getQuery(this.extension.TIME_ELAPSED_EXT,gl.CURRENT_QUERY)!==null)reason='timer-query-already-active';
      else{
        reason=null;
        for(let i=0;i<queryCapacity;++i){const query=gl.createQuery();if(!query){reason='query-allocation-failed';break;}this.queries.push(query);}
        if(reason){for(const query of this.queries)gl.deleteQuery(query);this.queries.length=0;}
        else gl.getParameter(this.extension.GPU_DISJOINT_EXT); // Clear pre-collection state.
      }
    }
    this.supported=reason===null;this.unavailableReason=reason;this.counterBits=bits;
  }
  get pendingCount():number{return this.pending;}
  private time(nowMs:number):void {
    if(this.disposed||this.stopped||!Number.isFinite(nowMs)||nowMs<0||nowMs<this.lastNow)throw new Error('Invalid GPU timer lifetime or clock');
  }
  begin(row:number,nowMs:number):boolean {
    this.time(nowMs);
    if(this.active||!Number.isInteger(row)||row!==this.attempts||row>=this.frameCapacity)throw new Error('Invalid GPU sample row');
    this.lastNow=nowMs;++this.attempts;
    if(!this.supported){this.statuses[row]=6;return false;}
    if(this.pending===this.queryCapacity){this.statuses[row]=3;return false;}
    const at=(this.head+this.pending)%this.queryCapacity;
    this.rows[at]=row;this.began[at]=nowMs;this.statuses[row]=1;
    this.gl!.beginQuery(this.extension!.TIME_ELAPSED_EXT,this.queries[at]);this.active=true;++this.pending;
    this.maximumPending=Math.max(this.maximumPending,this.pending);return true;
  }
  end():void {
    if(this.disposed||!this.active)throw new Error('No active GPU timer query');
    this.gl!.endQuery(this.extension!.TIME_ELAPSED_EXT);this.active=false;
  }
  poll(recorder:FrameRecorder,nowMs:number):void {
    this.time(nowMs);if(this.active)throw new Error('Cannot poll an active timer query');this.lastNow=nowMs;
    if(!this.supported||this.pending===0)return;
    ++this.polls;const gl=this.gl!;let ready=0;
    for(;ready<this.pending;++ready){
      const at=(this.head+ready)%this.queryCapacity;
      if(!gl.getQueryParameter(this.queries[at],gl.QUERY_RESULT_AVAILABLE))break;
      const result:unknown=gl.getQueryParameter(this.queries[at],gl.QUERY_RESULT);
      this.results[at]=typeof result==='number'?result:NaN;
    }
    const disjoint:unknown=gl.getParameter(this.extension!.GPU_DISJOINT_EXT);
    if(disjoint!==false){
      if(disjoint===true)++this.disjointEvents;
      this.discard(disjoint===true?4:5);return;
    }
    const limitNs=2**Math.min(this.counterBits!,53)-1;
    for(let i=0;i<ready;++i){
      const at=this.head,row=this.rows[at],ns=this.results[at];
      if(row>=recorder.count||!recorder.finished[row])this.statuses[row]=7;
      // A delayed query could have wrapped a short counter, even if its result
      // looks small. Preserve it as unavailable instead of claiming a fast GPU.
      else if(!Number.isSafeInteger(ns)||ns<0||ns>=limitNs||(nowMs-this.began[at])*1e6>=limitNs)this.statuses[row]=5;
      else{recorder.resolveGpu(row,ns/1e6);this.statuses[row]=2;}
      this.head=(this.head+1)%this.queryCapacity;--this.pending;
    }
  }
  private discard(status:number):void {
    for(let i=0;i<this.pending;++i)this.statuses[this.rows[(this.head+i)%this.queryCapacity]]=status;
    this.head=(this.head+this.pending)%this.queryCapacity;this.pending=0;
  }
  // Call after a bounded asynchronous drain, or immediately on interruption.
  stop(interrupted:boolean):void {
    if(this.disposed||this.stopped)return;
    if(this.active)this.end();this.discard(interrupted?7:8);this.stopped=true;
  }
  report(){
    const counts=new Array<number>(gpuSampleNames.length).fill(0);
    for(let i=0;i<this.attempts;++i)++counts[this.statuses[i]];
    return {supported:this.supported,unavailableReason:this.unavailableReason,counterBits:this.counterBits,
      queryCapacity:this.queryCapacity,frameCapacity:this.frameCapacity,typedBytes:this.bytes,driverStorageBytes:null,attempts:this.attempts,
      pending:this.pending,maximumPending:this.maximumPending,polls:this.polls,disjointEvents:this.disjointEvents,
      statusNames:gpuSampleNames,statuses:Array.from(this.statuses.subarray(0,this.attempts)),counts,
      interval:'elapsed GL command span around renderer.draw; not GPU utilization or presentation time'};
  }
  dispose():void {
    if(!this.disposed){this.stop(true);for(const query of this.queries)this.gl!.deleteQuery(query);this.disposed=true;}
  }
}
