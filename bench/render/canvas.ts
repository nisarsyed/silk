import {DrawScene, camera, colors, background} from './geometry.js';
import {Overlay,palette,markerVertices} from './overlay.js';

/** Cached local paths for moving scenes; merged paths for frozen render-only
 * runs. addPath/DOMMatrix allocations occur only during initialization.
 * Browser-internal Canvas allocations and GPU work are not observable here.
 */
export class CanvasCandidate {
  private readonly context: CanvasRenderingContext2D;
  private readonly paths: Path2D[];
  private readonly frozenPaths: Path2D[];
  private readonly view;
  readonly uploadBytes = null;
  readonly gpuBytes = null;
  readonly drawCalls=null; // Canvas does not expose its internal GPU draws.
  submissionCalls=0;
  private disposed = false;
  private readonly markerPath=new Path2D();
  constructor(readonly canvas: HTMLCanvasElement, readonly scene: DrawScene,readonly overlay?:Overlay) {
    const ctx=canvas.getContext('2d',{alpha:false});
    if (!ctx) throw new Error('Canvas 2D unavailable');
    this.markerPath.moveTo(3*markerVertices[0],3*markerVertices[1]);
    for(let i=2;i<markerVertices.length;i+=2)this.markerPath.lineTo(3*markerVertices[i],3*markerVertices[i+1]);
    this.markerPath.closePath();
    this.context=ctx; this.view=camera(scene.rect,canvas.width,canvas.height);
    this.paths=scene.meshes.map(mesh=>{
      const path=new Path2D(), v=mesh.vertices;
      path.moveTo(v[0],v[1]);
      for (let i=2; i<v.length; i+=2) path.lineTo(v[i],v[i+1]);
      path.closePath(); return path;
    });
    this.frozenPaths=scene.frozen ? scene.batches.map(batch=>{
      const path=new Path2D(), p=scene.poses;
      for (let i=batch.first; i<batch.first+batch.count; ++i) {
        const at=i*4;
        path.addPath(this.paths[batch.mesh],new DOMMatrix([p[at+2],p[at+3],-p[at+3],p[at+2],p[at],p[at+1]]));
      }
      return path;
    }) : [];

  }
  draw(): void {
    if (this.disposed) throw new Error('Disposed Canvas candidate');
    const c=this.context, s=this.scene, p=s.poses, v=this.view;
    c.setTransform(1,0,0,1,0,0); c.fillStyle=background;
    c.fillRect(0,0,this.canvas.width,this.canvas.height);this.submissionCalls=1;
    for (let k=0; k<s.batches.length; ++k) {
      const b=s.batches[k]; c.fillStyle=colors[b.color];
      if (s.frozen) {
        c.setTransform(v.scale,0,0,-v.scale,v.x,v.y); c.fill(this.frozenPaths[k]);++this.submissionCalls;
      } else {
        for (let i=b.first; i<b.first+b.count; ++i) {
          const at=i*4, cs=p[at+2]*v.scale, sn=p[at+3]*v.scale;
          c.setTransform(cs,-sn,-sn,-cs,v.x+p[at]*v.scale,v.y-p[at+1]*v.scale);
          if(this.overlay)c.fillStyle=palette[this.overlay.bodyColors[i]];
          c.fill(this.paths[b.mesh]);++this.submissionCalls;
        }
      }
    }
    if(this.overlay)this.drawOverlay();
  }
  private drawOverlay():void {
    const o=this.overlay!,c=this.context;
    c.setTransform(1,0,0,1,0,0);
    for(let i=0;i<o.lineCount;) {
      const code=o.lines[5*i+4];c.fillStyle=palette[code];c.beginPath();
      do {
        const at=i*5,ax=o.lines[at],ay=o.lines[at+1],bx=o.lines[at+2],by=o.lines[at+3];
        const dx=bx-ax,dy=by-ay,len=Math.hypot(dx,dy),nx=len?-dy*.5/len:0,ny=len?dx*.5/len:0;
        c.moveTo(ax-nx,ay-ny);c.lineTo(bx-nx,by-ny);c.lineTo(bx+nx,by+ny);c.lineTo(ax+nx,ay+ny);c.closePath();++i;
      }while(i<o.lineCount&&o.lines[5*i+4]===code);
      c.fill();++this.submissionCalls;
    }
    for(let i=0;i<o.markerCount;++i) {
      const at=3*i;c.fillStyle=palette[o.markers[at+2]];c.setTransform(1,0,0,1,o.markers[at],o.markers[at+1]);c.fill(this.markerPath);++this.submissionCalls;
    }
  }
  dispose(): void { this.disposed=true; this.paths.length=0; this.frozenPaths.length=0; }
}
