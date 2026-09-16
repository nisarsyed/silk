import type {FrameCounters} from './recording.js';
export interface HudMemory {linearBytes:number;arenaBytes:number;outputBytes:number}
export interface HudRenderer {gpuBytes:number|null;uploadBytes:number|null;drawCalls:number|null}
const mib=(bytes:number)=>(bytes/(1024*1024)).toFixed(2)+' MiB';
const available=(value:number|null)=>value===null?'unavailable':String(value);
/** Common diagnostic text at 4 Hz deadlines. Missed refreshes are skipped,
 * never replayed in a burst. Numeric frame recording is independent of this
 * display rate. String/DOM work is measured as part of the diagnostic frame. */
export class StudyHud {
  private next=0;private last=-1;private disposed=false;
  updates=0;
  constructor(private readonly element:HTMLElement,private readonly memory:HudMemory){
    if(!Object.values(memory).every(v=>Number.isFinite(v)&&v>=0))throw new Error('Invalid HUD memory');
    this.memory=Object.freeze({...memory});
    element.setAttribute('aria-live','off');element.style.whiteSpace='pre-wrap';element.style.overflowWrap='anywhere';
  }
  update(nowMs:number,counters:FrameCounters,renderer:HudRenderer):boolean {
    if(this.disposed)throw new Error('Disposed study HUD');
    if(!Number.isFinite(nowMs)||nowMs<0||nowMs<this.last||!counters.valid||counters.stats.length!==25||counters.work.length!==26)
      throw new Error('Invalid HUD time or counters');
    if(this.last<0)this.next=nowMs;
    this.last=nowMs;if(nowMs<this.next)return false;
    this.next+=250*(Math.floor((nowMs-this.next)/250)+1);
    const s=counters.stats,w=counters.work;
    this.element.textContent=
      `Bodies ${s[2]}/${s[3]} · awake ${s[0]} · sleeping ${s[1]}\n`+
      `Contacts ${s[4]}/${s[5]} · joints ${s[6]}/${s[7]}\n`+
      `Steps ${s[24]} · substeps ${s[21]} · islands ${s[19]}\n`+
      `Step work: tree ${w[0]} · pairs ${w[1]} · probes ${w[2]}\n`+
      `Total work: tree ${w[13]} · pairs ${w[14]} · probes ${w[15]}\n`+
      `Contact drops ${w[25]}\n`+
      `WASM ${mib(this.memory.linearBytes)} · allocated ${mib(s[23])}\n`+
      `World arena ${mib(this.memory.arenaBytes)} · JS output payload ${mib(this.memory.outputBytes)}\n`+
      `GPU payload ${renderer.gpuBytes===null?'unavailable':mib(renderer.gpuBytes)}\n`+
      `Upload bytes ${available(renderer.uploadBytes)} · GPU draws ${available(renderer.drawCalls)}`;
    ++this.updates;return true;
  }
  dispose():void{if(!this.disposed){this.element.textContent='';this.disposed=true;}}
}
