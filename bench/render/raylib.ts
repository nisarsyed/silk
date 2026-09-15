import {DrawScene, camera} from './geometry.js';
import {Overlay,palette,markerTriangles} from './overlay.js';
import type {SnapshotCopy} from '../../wasm/index.js';
interface RaylibModule {
  HEAPF32: Float32Array; HEAPU32: Uint32Array;
  _sl_render_init(instances:number,vertices:number,batches:number,width:number,height:number):number;
  _sl_render_vertices():number; _sl_render_poses():number; _sl_render_batches():number;
  _sl_render_overlay_init(lines:number,markers:number):number;
  _sl_render_overlay_lines():number;_sl_render_overlay_markers():number;_sl_render_overlay_colors():number;
  _sl_render_overlay_palette():number;_sl_render_overlay_mesh():number;
  _sl_render_overlay_counts(lines:number,markers:number):number;
  _sl_render_study_poses(study:number,poses:number,count:number):number;
  _sl_render_draw(scale:number,x:number,y:number):number; _sl_render_dispose():void;
}
export async function createRaylibCandidate(canvas:HTMLCanvasElement, scene:DrawScene,overlay?:Overlay) {
  if (!canvas.id || document.getElementById(canvas.id)!==canvas) throw new Error('Raylib requires an attached canvas with a unique id');
  const url=new URL('./raylib.mjs',import.meta.url).href;
  const {default:factory}=await import(url);
  const m:RaylibModule=await factory({canvas,wasmMemory:new WebAssembly.Memory({initial:1024,maximum:1024})});
  return initializeCandidate(m,canvas,scene,overlay);
}
interface DirectSource {study:number;steps():number;valid():void}
function initializeCandidate(m:RaylibModule,canvas:HTMLCanvasElement,scene:DrawScene,overlay?:Overlay,source?:DirectSource) {
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
    let lines:Float32Array|undefined,markers:Float32Array|undefined,colors:Float32Array|undefined;
    if(overlay){
      if(!m._sl_render_overlay_init(overlay.lines.length/5,overlay.markers.length/3))throw new Error('Raylib diagnostic allocation failed');
      lines=new Float32Array(m.HEAPF32.buffer,m._sl_render_overlay_lines(),overlay.lines.length);
      markers=new Float32Array(m.HEAPF32.buffer,m._sl_render_overlay_markers(),overlay.markers.length);
      colors=new Float32Array(m.HEAPF32.buffer,m._sl_render_overlay_colors(),overlay.bodyColors.length);
      new Float32Array(m.HEAPF32.buffer,m._sl_render_overlay_mesh(),markerTriangles.length).set(markerTriangles);
      new Uint32Array(m.HEAPU32.buffer,m._sl_render_overlay_palette(),48).set(palette.flatMap(hex=>[1,3,5].map(at=>parseInt(hex.slice(at,at+2),16))));
    }
    let revision=0,overlayRevision=0,disposed=false,lastStep=-1;
    return {
      // CPU-to-WASM pose copies are separate from raylib's internal GPU uploads.
      poseCopyBytes:0,posePrepareBytes:0,diagnosticCopyBytes:0, uploadBytes:null, gpuBytes:null, drawCalls:null,
      linearMemoryBytes:m.HEAPF32.buffer.byteLength,
      draw() {
        if (disposed) throw new Error('Disposed raylib candidate');
        this.poseCopyBytes=0;this.posePrepareBytes=0;
        if(source){
          source.valid();const step=source.steps();
          if(lastStep!==step){
            if(!m._sl_render_study_poses(source.study,poses.byteOffset,scene.count))throw new Error('Direct C pose preparation failed');
            lastStep=step;this.posePrepareBytes=poses.byteLength;
          }
        }else if (revision!==scene.revision) { poses.set(scene.poses); revision=scene.revision; this.poseCopyBytes=poses.byteLength; }
        this.diagnosticCopyBytes=0;
        if(overlay&&lines&&markers&&colors&&overlayRevision!==overlay.revision){
          // Active prefixes only, without temporary typed-array views. The
          // co-located C study will remove this prototype boundary copy.
          for(let i=0;i<overlay.lineCount*5;++i)lines[i]=overlay.lines[i];
          for(let i=0;i<overlay.markerCount*3;++i)markers[i]=overlay.markers[i];
          colors.set(overlay.bodyColors);
          if(!m._sl_render_overlay_counts(overlay.lineCount,overlay.markerCount))throw new Error('Invalid raylib diagnostic commands');
          this.diagnosticCopyBytes=overlay.lineCount*20+overlay.markerCount*12+overlay.bodyColors.byteLength;
          overlayRevision=overlay.revision;
        }
        if (!m._sl_render_draw(view.scale,view.x,view.y)) throw new Error('Raylib draw failed');
      },
      dispose() { if (!disposed) { m._sl_render_dispose(); disposed=true; } }
    };
  } catch (error) { m._sl_render_dispose(); throw error; }
}

interface StudyWorld {
  configuration:{scene:'pyramid'|'rain'|'chains';copies:number};
  snapshot:SnapshotCopy;steps:number;refreshSnapshot(diagnostic?:boolean):boolean;
}
/** One fixed memory owns physics and raylib. Geometry crosses JS during setup;
 * live base poses stay in C. Diagnostic direct preparation remains separate
 * work; this entry point currently creates only the base visual profile. */
export async function createRaylibStudyModule(canvas:HTMLCanvasElement,{memoryBytes=64*1024*1024}={}) {
  if(!canvas.id||document.getElementById(canvas.id)!==canvas)throw new Error('Raylib requires an attached canvas with a unique id');
  if(!Number.isInteger(memoryBytes)||memoryBytes<64*1024*1024||memoryBytes>512*1024*1024||memoryBytes%65536)
    throw new RangeError('Invalid fixed raylib memory budget');
  const {default:factory}=await import(new URL('./raylib.mjs',import.meta.url).href);
  const {ownStudyModule}=await import(new URL('./physics/owner.mjs',import.meta.url).href);
  const m:RaylibModule=await factory({canvas,wasmMemory:new WebAssembly.Memory({initial:memoryBytes/65536,maximum:memoryBytes/65536})});
  return ownStudyModule(m,memoryBytes,(study:number,world:StudyWorld,valid:()=>void)=>{
    if(!world.refreshSnapshot())throw new Error('Raylib geometry snapshot failed');
    const scene=new DrawScene(world.snapshot,world.configuration.scene,undefined,world.configuration.copies);
    return initializeCandidate(m,canvas,scene,undefined,{study,valid,steps:()=>world.steps});
  });
}
