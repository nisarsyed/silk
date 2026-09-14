import {DrawScene, camera, colors, background} from './geometry.js';

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
  readonly drawCalls: number;
  private disposed = false;
  constructor(readonly canvas: HTMLCanvasElement, readonly scene: DrawScene) {
    const ctx=canvas.getContext('2d',{alpha:false});
    if (!ctx) throw new Error('Canvas 2D unavailable');
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
    this.drawCalls=scene.frozen?scene.batches.length:scene.count;
  }
  draw(): void {
    if (this.disposed) throw new Error('Disposed Canvas candidate');
    const c=this.context, s=this.scene, p=s.poses, v=this.view;
    c.setTransform(1,0,0,1,0,0); c.fillStyle=background;
    c.fillRect(0,0,this.canvas.width,this.canvas.height);
    for (let k=0; k<s.batches.length; ++k) {
      const b=s.batches[k]; c.fillStyle=colors[b.color];
      if (s.frozen) {
        c.setTransform(v.scale,0,0,-v.scale,v.x,v.y); c.fill(this.frozenPaths[k]);
      } else {
        for (let i=b.first; i<b.first+b.count; ++i) {
          const at=i*4, cs=p[at+2]*v.scale, sn=p[at+3]*v.scale;
          c.setTransform(cs,-sn,-sn,-cs,v.x+p[at]*v.scale,v.y-p[at+1]*v.scale);
          c.fill(this.paths[b.mesh]);
        }
      }
    }
  }
  dispose(): void { this.disposed=true; this.paths.length=0; this.frozenPaths.length=0; }
}
