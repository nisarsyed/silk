import type {SnapshotCopy} from '../../wasm/index.js';
import {DrawScene} from './geometry.js';
import {Overlay,type DiagnosticColumns} from './overlay.js';

export interface FrozenDiagnostics extends DiagnosticColumns {readonly instances:number;readonly temporaryNativeBytes:number}

function contacts(scene:DrawScene,snapshot:SnapshotCopy){
  if(!scene.frozen||snapshot.bodyCount!==2003||snapshot.jointCount!==0||
      snapshot.contactCount>16384||snapshot.geometry.kind.length!==2003)throw new Error('Invalid frozen rain snapshot');
  const b=snapshot.bodies,c=snapshot.contacts,rows=new Uint32Array(2003),ordinals=new Uint32Array(2003);
  rows.fill(0xffffffff);ordinals.fill(0xffffffff);let moving=0;
  for(let row=0;row<snapshot.bodyCount;++row){
    const slot=b.index[row];if(slot>=rows.length||rows[slot]!==0xffffffff)throw new Error('Invalid frozen body slot');
    rows[slot]=row;
    if(b.type[row]!==2&&snapshot.geometry.kind[slot]!==0)ordinals[slot]=moving++;
  }
  if(moving!==2000)throw new Error('Invalid frozen source instance count');
  for(let row=0;row<snapshot.contactCount;++row){
    const a=rows[c.bodyAIndex[row]],z=rows[c.bodyBIndex[row]];
    if(a===undefined||z===undefined||a===0xffffffff||z===0xffffffff||c.pointCount[row]>2||
      b.generation[a]!==c.bodyAGeneration[row]||b.generation[z]!==c.bodyBGeneration[row])throw new Error('Invalid frozen contact endpoint');
  }
  const tiles=Math.ceil(scene.count/moving);let count=0;
  for(let tile=0;tile<tiles;++tile){
    const included=Math.min(moving,scene.count-tile*moving);
    for(let row=0;row<snapshot.contactCount;++row)if(c.pointCount[row]>0&&
      (ordinals[c.bodyAIndex[row]]<included||ordinals[c.bodyBIndex[row]]<included))++count;
  }
  return {ordinals,moving,tiles,count};
}

/** Setup-only immutable commands. A contact appears once per tile when any
 * dynamic endpoint is displayed, including contacts against omitted statics
 * or trimmed bodies. Zero-point contacts have no drawing commands. Source
 * island IDs/colors are repeated literally; only the one global camera query
 * uses the displayed instances. All three candidates consume these arrays.
 */
export class FrozenOverlay extends Overlay {
  readonly drawnContacts:number;
  constructor(scene:DrawScene,snapshot:SnapshotCopy,diagnostics:FrozenDiagnostics,width:number,height:number){
    const input=contacts(scene,snapshot),n=scene.count;
    if(!diagnostics?.valid||diagnostics.instances!==n||diagnostics.floats.length!==4*n+5||diagnostics.words.length!==n+3)
      throw new Error('Invalid frozen query columns');
    // Preserve the existing overlay capacity limits. Only contacts with points
    // require command storage; every eligible point is still emitted below.
    super(scene,n,Math.max(1,input.count),0,width,height);this.drawnContacts=input.count;
    const b=snapshot.bodies,c=snapshot.contacts,d=diagnostics.floats,w=diagnostics.words;
    for(let i=0;i<n;++i){
      const row=scene.rows[i];
      if(row>=snapshot.bodyCount||input.ordinals[b.index[row]]!==i%input.moving)throw new Error('Frozen source row order differs');
      this.bodyColors[i]=!b.awake[row]?2:b.island[row]===0xffffffff?1:3+b.island[row]%8;
      if(w[i]&8){
        const lx=d[i],ly=d[n+i],ux=d[2*n+i],uy=d[3*n+i],code=w[i]&7?15:13;
        this.line(lx,ly,ux,ly,code);this.line(ux,ly,ux,uy,code);this.line(ux,uy,lx,uy,code);this.line(lx,uy,lx,ly,code);
      }
    }
    for(let tile=0;tile<input.tiles;++tile){
      const included=Math.min(input.moving,n-tile*input.moving),offset=32*(tile-(input.tiles-1)/2);
      for(let row=0;row<snapshot.contactCount;++row){
        if(input.ordinals[c.bodyAIndex[row]]>=included&&input.ordinals[c.bodyBIndex[row]]>=included)continue;
        for(let point=0;point<c.pointCount[row];++point){
          const x=Math.fround((point===0?c.pointX0[row]:c.pointX1[row])+offset),y=point===0?c.pointY0[row]:c.pointY1[row];
          this.normal(x,y,c.normalX[row],c.normalY[row],12);this.marker(x,y,11);
        }
      }
    }
    this.queries(diagnostics);this.complete();
  }
  override refresh(_snapshot:SnapshotCopy,_diagnostics:DiagnosticColumns):never {
    throw new Error('Frozen diagnostics cannot change');
  }
}
