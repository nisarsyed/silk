import type {SnapshotCopy} from '../../wasm/index.js';
import {DrawScene,camera,circleSides} from './geometry.js';

export const palette=Object.freeze(['#7192a5','#ebbd70','#586776','#e6a36d','#87c5a4','#baabdf',
  '#7dbdd0','#d5bb73','#d992af','#a4c477','#9eaee0','#ffcc66','#ef7777','#527d99','#7fe4d0','#e6a4f1']);
export const paletteValues=new Float32Array(palette.flatMap(hex=>[1,3,5].map(at=>parseInt(hex.slice(at,at+2),16)/255)));
export const markerVertices=new Float32Array(Array.from({length:circleSides*2},(_,i)=>
  i%2?Math.sin(Math.floor(i/2)*2*Math.PI/circleSides):Math.cos(Math.floor(i/2)*2*Math.PI/circleSides)));
export const markerTriangles=new Float32Array((circleSides-2)*6);
for(let i=1;i<circleSides-1;++i) markerTriangles.set([markerVertices[0],markerVertices[1],
  markerVertices[2*i],markerVertices[2*i+1],markerVertices[2*i+2],markerVertices[2*i+3]],(i-1)*6);
export interface DiagnosticColumns {readonly valid:boolean;readonly floats:Float32Array;readonly words:Uint32Array}

/** Developer comparison commands in drawing-buffer pixels. Fixed payload:
 * 20 bytes/line, 12 bytes/marker, 4 bytes/body color. Capacities cover all
 * contact points, proxy edges, joints and fixed queries; overflow fails a frame.
 * Draw order is bodies, lines (proxies/normals/joints/queries), then markers.
 */
export class Overlay {
  readonly lines:Float32Array;
  readonly markers:Float32Array;
  readonly bodyColors:Float32Array;
  private readonly rows:Uint32Array;
  readonly view;
  lineCount=0;markerCount=0;revision=0;
  constructor(readonly scene:DrawScene,readonly bodyCapacity:number,readonly contactCapacity:number,
      readonly jointCapacity:number,width:number,height:number) {
    if(!Number.isInteger(bodyCapacity)||bodyCapacity<scene.count||bodyCapacity>65536||
        !Number.isInteger(contactCapacity)||contactCapacity<1||contactCapacity>262144||
        !Number.isInteger(jointCapacity)||jointCapacity<0||jointCapacity>65536)throw new RangeError('Invalid diagnostic capacity');
    this.lines=new Float32Array(5*(4*bodyCapacity+2*contactCapacity+jointCapacity+6));
    this.markers=new Float32Array(3*(2*contactCapacity+2*jointCapacity+2));
    this.rows=new Uint32Array(bodyCapacity);this.bodyColors=new Float32Array(scene.count);this.view=camera(scene.rect,width,height);
  }
  protected line(ax:number,ay:number,bx:number,by:number,color:number):void {
    const at=this.lineCount*5;
    if(at+5>this.lines.length)throw new Error('Diagnostic line capacity exceeded');
    const v=this.view,l=this.lines;
    l[at]=v.x+ax*v.scale;l[at+1]=v.y-ay*v.scale;l[at+2]=v.x+bx*v.scale;l[at+3]=v.y-by*v.scale;l[at+4]=color;
    const dx=Math.fround(l[at+2]-l[at]),dy=Math.fround(l[at+3]-l[at+1]);
    if(!Number.isFinite(Math.fround(Math.fround(dx*dx)+Math.fround(dy*dy))))throw new Error('Diagnostic line overflows GPU arithmetic');
    ++this.lineCount;
  }
  protected marker(x:number,y:number,color:number):void {
    const at=this.markerCount*3;
    if(at+3>this.markers.length)throw new Error('Diagnostic marker capacity exceeded');
    this.markers[at]=this.view.x+x*this.view.scale;this.markers[at+1]=this.view.y-y*this.view.scale;this.markers[at+2]=color;
    ++this.markerCount;
  }
  protected normal(x:number,y:number,nx:number,ny:number,color:number):void {
    // Normalize only the display direction. Raw C normal components remain
    // available in the snapshot/report; glyph length is always 12 pixels.
    const length=Math.hypot(nx,ny);
    if(!Number.isFinite(length)||length===0)throw new Error('Invalid diagnostic normal');
    const scale=12/(length*this.view.scale);
    this.line(x,y,x+nx*scale,y+ny*scale,color);
  }
  refresh(snapshot:SnapshotCopy,diagnostics:DiagnosticColumns):void {
    if(this.scene.frozen)throw new Error('Use FrozenOverlay for render-only diagnostics');
    if(!diagnostics.valid||diagnostics.floats.length!==4*this.bodyCapacity+5||diagnostics.words.length!==this.bodyCapacity+3||
        snapshot.bodyCount>this.bodyCapacity||snapshot.contactCount>this.contactCapacity||snapshot.jointCount>this.jointCapacity)
      throw new Error('Invalid diagnostic snapshot');
    this.lineCount=0;this.markerCount=0;
    const b=snapshot.bodies,c=snapshot.contacts,j=snapshot.joints,d=diagnostics.floats,w=diagnostics.words,n=this.bodyCapacity;
    this.rows.fill(0xffffffff);
    for(let row=0;row<snapshot.bodyCount;++row)this.rows[b.index[row]]=row;
    for(let i=0;i<this.scene.count;++i) {
      const row=this.scene.rows[i],slot=b.index[row];
      this.bodyColors[i]=b.type[row]===2?0:!b.awake[row]?2:b.island[row]===0xffffffff?1:3+b.island[row]%8;
      if(w[slot]&8) {
        const lx=d[slot],ly=d[n+slot],ux=d[2*n+slot],uy=d[3*n+slot],code=w[slot]&7?15:13;
        this.line(lx,ly,ux,ly,code);this.line(ux,ly,ux,uy,code);this.line(ux,uy,lx,uy,code);this.line(lx,uy,lx,ly,code);
      }
    }
    for(let row=0;row<snapshot.contactCount;++row) {
      if(c.pointCount[row]>2)throw new Error('Invalid contact point count');
      for(let p=0;p<c.pointCount[row];++p) {
        const x=p===0?c.pointX0[row]:c.pointX1[row],y=p===0?c.pointY0[row]:c.pointY1[row];
        // Unit normal displayed at a fixed 12-buffer-pixel length.
        this.normal(x,y,c.normalX[row],c.normalY[row],12);
        this.marker(x,y,11);
      }
    }
    for(let row=0;row<snapshot.jointCount;++row) {
      const a=this.rows[j.bodyAIndex[row]],z=this.rows[j.bodyBIndex[row]];
      if(a===0xffffffff||z===0xffffffff||b.generation[a]!==j.bodyAGeneration[row]||b.generation[z]!==j.bodyBGeneration[row])
        throw new Error('Joint endpoint missing from snapshot');
      const ax=b.x[a]+b.cos[a]*j.anchorAX[row]-b.sin[a]*j.anchorAY[row];
      const ay=b.y[a]+b.sin[a]*j.anchorAX[row]+b.cos[a]*j.anchorAY[row];
      const bx=b.x[z]+b.cos[z]*j.anchorBX[row]-b.sin[z]*j.anchorBY[row];
      const by=b.y[z]+b.sin[z]*j.anchorBX[row]+b.cos[z]*j.anchorBY[row];
      this.line(ax,ay,bx,by,14);this.marker(ax,ay,14);this.marker(bx,by,14);
    }
    this.queries(diagnostics);this.complete();
  }
  protected queries(diagnostics:DiagnosticColumns):void {
    const d=diagnostics.floats,w=diagnostics.words,n=this.bodyCapacity;
    const rect=this.scene.rect,x=(rect[0]+rect[2])/2,y=(rect[1]+rect[3])/2;
    this.line(x-1,y-1,x+1,y-1,15);this.line(x+1,y-1,x+1,y+1,15);
    this.line(x+1,y+1,x-1,y+1,15);this.line(x-1,y+1,x-1,y-1,15);
    this.line(rect[0],y,rect[2],y,15);this.marker(x,y,15);
    if(w[n+2]) {
      const at=4*n;
      this.marker(d[at+1],d[at+2],15);
      this.normal(d[at+1],d[at+2],d[at+3],d[at+4],15);
    }
  }
  protected complete():void {
    for(let i=0;i<this.lineCount*5;++i)if(!Number.isFinite(this.lines[i]))throw new Error('Non-finite diagnostic line');
    for(let i=0;i<this.markerCount*3;++i)if(!Number.isFinite(this.markers[i]))throw new Error('Non-finite diagnostic marker');
    ++this.revision;
  }
}
