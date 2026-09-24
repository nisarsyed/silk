import {camera, type Rect} from './geometry.js';

const eventLimit=65536;
const noIndex=0xffffffff;
const kinds=['down','move','up','cancel','lost-capture'] as const;
const eventTypes=['pointerdown','pointermove','pointerup','pointercancel','lostpointercapture'] as const;
const states=['ignored','accepted','queued','submitted','superseded','miss',
  'released-before-step','cancelled-before-step','not-stepped','applied-no-frame','rejected'] as const;
const gcd=(a:number,b:number):number=>{while(b){const next=a%b;a=b;b=next;}return a;};
type PointerWorld={
  pointer(action:number,x:number,y:number):boolean;
  pointerState:{readonly held:boolean;readonly bodyIndex:number;readonly bodyGeneration:number};
};

/** One preallocated trace for a separate real-pointer trial. Browser events may
 * allocate internally; this owner allocates no rows in the timed input path.
 * A move is a latency sample only after a C step and the first following draw.
 */
export class StudyInput {
  private readonly kind=new Uint8Array(eventLimit);private readonly state=new Uint8Array(eventLimit);
  private readonly trusted=new Uint8Array(eventLimit);private readonly primary=new Uint8Array(eventLimit);
  private readonly coalesced=new Uint16Array(eventLimit);private readonly pointerId=new Int32Array(eventLimit);
  private readonly gesture=new Uint32Array(eventLimit);private readonly body=new Uint32Array(eventLimit);
  private readonly button=new Int16Array(eventLimit);private readonly buttons=new Uint16Array(eventLimit);
  private readonly rawMs=new Float64Array(eventLimit);private readonly eventMs=new Float64Array(eventLimit);
  private readonly receivedMs=new Float64Array(eventLimit);private readonly clientX=new Float64Array(eventLimit);
  private readonly clientY=new Float64Array(eventLimit);private readonly worldX=new Float64Array(eventLimit);
  private readonly worldY=new Float64Array(eventLimit);private readonly submittedMs=new Float64Array(eventLimit);
  private readonly appliedStep=new Uint32Array(eventLimit);private readonly submittedFrame=new Uint32Array(eventLimit);
  private readonly timeKind=new Uint8Array(eventLimit);
  private count=0;private gestures=0;private active=-1;private pending=-1;private applied=-1;
  private readonly view;private readonly rect:DOMRect;private readonly touchAction:string;
  private listening=false;
  constructor(private readonly canvas:HTMLCanvasElement,private readonly world:PointerWorld,
    rect:Rect,private readonly fail:(reason:string)=>void){
    this.view=camera(rect,canvas.width,canvas.height);
    this.rect=canvas.getBoundingClientRect();
    if(!(this.rect.width>0&&this.rect.height>0))throw new Error('Interaction canvas has no layout');
    this.touchAction=canvas.style.touchAction;
  }
  private readonly handle=(event:PointerEvent):void=>{
    try{this.accept(event);}catch(error){this.fail(`Pointer trace: ${error instanceof Error?error.message:error}`);}
  };
  start():void{
    if(this.listening)throw new Error('Interaction listener already active');
    this.listening=true;this.canvas.style.touchAction='none';
    for(const kind of eventTypes)
      this.canvas.addEventListener(kind,this.handle);
  }
  private clearPending(state:number):void{if(this.pending>=0){this.state[this.pending]=state;this.pending=-1;}}
  private accept(event:PointerEvent):void{
    const kind=eventTypes.indexOf(event.type as typeof eventTypes[number]);
    if(kind<0)return;
    if(this.count===eventLimit)throw new Error('Event recording capacity exhausted');
    const rect=this.canvas.getBoundingClientRect();
    if(rect.left!==this.rect.left||rect.top!==this.rect.top||rect.width!==this.rect.width||rect.height!==this.rect.height)
      throw new Error('Interaction canvas layout changed');
    const i=this.count++,received=performance.now(),raw=event.timeStamp;
    const epoch=raw>1e9,normalized=epoch?raw-performance.timeOrigin:raw;
    const px=(event.clientX-rect.left)*this.canvas.width/rect.width;
    const py=(event.clientY-rect.top)*this.canvas.height/rect.height;
    const x=(px-this.view.x)/this.view.scale,y=(this.view.y-py)/this.view.scale;
    this.kind[i]=kind;this.state[i]=0;this.trusted[i]=event.isTrusted?1:0;
    this.primary[i]=event.isPrimary?1:0;this.pointerId[i]=event.pointerId;
    this.button[i]=event.button;this.buttons[i]=event.buttons;
    const coalesced=kind===1&&typeof event.getCoalescedEvents==='function'?event.getCoalescedEvents().length:0;
    if(coalesced>65535)throw new Error('Coalesced event count exceeded');
    this.coalesced[i]=coalesced;this.rawMs[i]=raw;this.eventMs[i]=normalized;
    this.receivedMs[i]=received;this.clientX[i]=event.clientX;this.clientY[i]=event.clientY;
    this.worldX[i]=x;this.worldY[i]=y;this.timeKind[i]=epoch?2:1;
    this.gesture[i]=this.gestures;this.body[i]=noIndex;
    if(kind===0){
      if(this.active>=0||!event.isPrimary||event.button!==0)return;
      if(!Number.isFinite(x)||!Number.isFinite(y)||Math.abs(x)>8192||Math.abs(y)>8192){this.state[i]=10;return;}
      if(!this.world.pointer(0,x,y)){this.state[i]=10;return;}
      this.active=event.pointerId;this.gesture[i]=++this.gestures;
      this.body[i]=this.world.pointerState.bodyIndex;this.state[i]=1;
      this.canvas.setPointerCapture(event.pointerId);event.preventDefault();return;
    }
    if(event.pointerId!==this.active)return;
    this.gesture[i]=this.gestures;this.body[i]=this.world.pointerState.bodyIndex;
    if(kind===1){
      if(!Number.isFinite(x)||!Number.isFinite(y)||Math.abs(x)>8192||Math.abs(y)>8192){this.state[i]=10;return;}
      if(!this.world.pointer(1,x,y)){this.state[i]=10;return;}
      this.clearPending(4);
      this.body[i]=this.world.pointerState.bodyIndex;
      if(this.body[i]===noIndex){this.state[i]=5;return;}
      this.state[i]=2;this.pending=i;event.preventDefault();return;
    }
    if(!this.world.pointer(kind===2?2:3,x,y)){this.state[i]=10;return;}
    this.clearPending(kind===2?6:7);this.state[i]=1;this.active=-1;
    if(this.canvas.hasPointerCapture(event.pointerId))this.canvas.releasePointerCapture(event.pointerId);
    event.preventDefault();
  }
  stepped(step:number):void{
    if(this.pending<0)return;
    const i=this.pending;this.pending=-1;
    if(!this.world.pointerState.held||this.world.pointerState.bodyIndex===noIndex){this.state[i]=5;return;}
    this.appliedStep[i]=step;this.state[i]=9;this.applied=i;
  }
  submitted(frame:number,atMs:number):void{
    if(this.applied<0)return;
    const i=this.applied;this.applied=-1;
    this.submittedFrame[i]=frame;this.submittedMs[i]=atMs;this.state[i]=3;
  }
  stop():void{
    if(!this.listening)return;
    this.listening=false;
    for(const kind of eventTypes)
      this.canvas.removeEventListener(kind,this.handle);
    this.canvas.style.touchAction=this.touchAction;
    this.clearPending(8);
    if(this.active>=0){
      if(this.canvas.hasPointerCapture(this.active))this.canvas.releasePointerCapture(this.active);
      this.active=-1;
    }
  }
  report(){
    const events=[];let trustedSubmitted=0,validTimestampSamples=0;
    let previousMicros=-1,quantumMicros=0,quantumPairs=0;
    const trustedGestures=new Set<number>();
    for(let i=0;i<this.count;++i){
      const latency=this.state[i]===3&&this.eventMs[i]>0&&this.submittedMs[i]>=this.eventMs[i]
        ?this.submittedMs[i]-this.eventMs[i]:null;
      if(this.kind[i]===1&&this.trusted[i]&&this.state[i]===3){
        ++trustedSubmitted;trustedGestures.add(this.gesture[i]);
        if(latency!==null){
          ++validTimestampSamples;
          const micros=Math.round(this.eventMs[i]*1000);
          if(Number.isSafeInteger(micros)&&previousMicros>=0&&micros>previousMicros){
            quantumMicros=gcd(quantumMicros,micros-previousMicros);++quantumPairs;
          }
          if(Number.isSafeInteger(micros))previousMicros=micros;
        }
      }
      events.push({kind:kinds[this.kind[i]],status:states[this.state[i]],trusted:!!this.trusted[i],
        primary:!!this.primary[i],pointerId:this.pointerId[i],button:this.button[i],buttons:this.buttons[i],
        gesture:this.gesture[i],bodyIndex:this.body[i]===noIndex?null:this.body[i],
        coalescedCount:this.coalesced[i],eventTimeStamp:this.rawMs[i],
        eventTimeKind:this.timeKind[i]===2?'epoch-adjusted':'relative',eventMs:this.eventMs[i],
        receivedMs:this.receivedMs[i],clientX:this.clientX[i],clientY:this.clientY[i],
        worldX:this.worldX[i],worldY:this.worldY[i],appliedStep:this.appliedStep[i]||null,
        submittedFrame:this.submittedFrame[i]||null,submittedMs:this.submittedMs[i]||null,latencyMs:latency});
    }
    return {source:'canvas PointerEvent',capacity:eventLimit,typedBytes:95*eventLimit,
      count:this.count,gestures:this.gestures,
      trustedSubmittedMoves:trustedSubmitted,trustedGesturesWithSubmittedMoves:trustedGestures.size,
      validTimestampSamples,
      // This describes only the grid visible in this trace. Privacy rounding,
      // jitter or a coarser configured clock can make it an unsafe precision
      // claim; the latency gate therefore remains unqualified.
      observedTimestampQuantumMs:quantumPairs>=4&&quantumMicros>0?quantumMicros/1000:null,
      observedTimestampQuantumPairs:quantumPairs,
      timestampPrecisionMs:null,timestampPrecisionStatus:'unverified',latencyGateEvaluable:false,
      timeOrigin:performance.timeOrigin,events};
  }
}
