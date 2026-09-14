import {DrawScene, camera} from './geometry.js';
interface RaylibModule {
  HEAPF32: Float32Array; HEAPU32: Uint32Array;
  _sl_render_init(instances:number,vertices:number,batches:number,width:number,height:number):number;
  _sl_render_vertices():number; _sl_render_poses():number; _sl_render_batches():number;
  _sl_render_draw(scale:number,x:number,y:number):number; _sl_render_dispose():void;
}
export async function createRaylibCandidate(canvas:HTMLCanvasElement, scene:DrawScene) {
  if (!canvas.id || document.getElementById(canvas.id)!==canvas) throw new Error('Raylib requires an attached canvas with a unique id');
  const url=new URL('./raylib.mjs',import.meta.url).href;
  const {default:factory}=await import(url);
  const m:RaylibModule=await factory({canvas});
  const vertexCount=scene.meshes.reduce((n,mesh)=>n+mesh.triangles.length/2,0);
  if (!m._sl_render_init(scene.count,vertexCount,scene.batches.length,canvas.width,canvas.height))
    throw new Error('Raylib renderer initialization failed');
  try {
    const vertices=new Float32Array(m.HEAPF32.buffer,m._sl_render_vertices(),2*vertexCount);
    const poses=new Float32Array(m.HEAPF32.buffer,m._sl_render_poses(),4*scene.count);
    const batches=new Uint32Array(m.HEAPU32.buffer,m._sl_render_batches(),5*scene.batches.length);
    const offsets:number[]=[]; let offset=0;
    for (const mesh of scene.meshes) { offsets.push(offset/2); vertices.set(mesh.triangles,offset); offset+=mesh.triangles.length; }
    scene.batches.forEach((b,i)=>batches.set([b.first,b.count,offsets[b.mesh],scene.meshes[b.mesh].triangles.length/2,b.color],i*5));
    const view=camera(scene.rect,canvas.width,canvas.height);
    let revision=0,disposed=false;
    return {
      // CPU-to-WASM pose copies are separate from raylib's internal GPU uploads.
      poseCopyBytes:0, uploadBytes:null, gpuBytes:null, drawCalls:null,
      linearMemoryBytes:m.HEAPF32.buffer.byteLength,
      draw() {
        if (disposed) throw new Error('Disposed raylib candidate');
        this.poseCopyBytes=0;
        if (revision!==scene.revision) { poses.set(scene.poses); revision=scene.revision; this.poseCopyBytes=poses.byteLength; }
        if (!m._sl_render_draw(view.scale,view.x,view.y)) throw new Error('Raylib draw failed');
      },
      dispose() { if (!disposed) { m._sl_render_dispose(); disposed=true; } }
    };
  } catch (error) { m._sl_render_dispose(); throw error; }
}
