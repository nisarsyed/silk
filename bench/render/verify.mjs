import {createStudyModule} from './physics/driver.mjs';
import {DrawScene} from './geometry.js';
import {Overlay} from './overlay.js';
import {edgeMask,compare} from './pixels.js';
import {CanvasCandidate} from './canvas.js';
import {WebglCandidate} from './webgl.js';
import {createRaylibCandidate,createRaylibStudyModule} from './raylib.js';

// Readbacks are correctness-only, outside every measured run. A single callback
// draws and reads before the non-preserved WebGL buffer can be discarded.
function capture(canvas, candidate) {
  return new Promise((resolve,reject)=>requestAnimationFrame(()=>{
    try {
      candidate.draw();
      const gl=canvas.getContext('webgl2');
      if (!gl) resolve(canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data);
      else {
        const bytes=new Uint8Array(canvas.width*canvas.height*4), flipped=new Uint8Array(bytes.length);
        gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,bytes);
        if (gl.getError()!==gl.NO_ERROR) throw new Error('GL correctness readback failed');
        const stride=canvas.width*4;
        for (let y=0; y<canvas.height; ++y) flipped.set(bytes.subarray(y*stride,(y+1)*stride),(canvas.height-1-y)*stride);
        resolve(flipped);
      }
    } catch(error) { reject(error); }
  }));
}
window.verifyRenderers=async ({scene='rain',steps=120,copies=1,sleep=false,diagnostic=false,instances,width=1280,height=720,images=false}={})=>{
  const module=await createStudyModule({memoryBytes:256*1024*1024});
  const world=module.create(scene,{copies,sleep,steps:steps+1});
  if (!world) throw new Error('Fixture allocation failed');
  const canvases=[], candidates=[];
  try {
    for (let i=0; i<steps; ++i) {
      if (!world.step()) throw new Error('Fixture stepping failed');
    }
    if (!world.refreshSnapshot(diagnostic)) throw new Error('Snapshot failed');
    const drawScene=new DrawScene(world.snapshot,scene,instances,copies), pixels=[];
    const config=world.configuration;
    const overlay=diagnostic?new Overlay(drawScene,config.bodyCapacity,config.contactCapacity,config.jointCapacity,width,height):undefined;
    if(overlay)overlay.refresh(world.snapshot,world.diagnostics);
    const edges=edgeMask(drawScene,width,height,overlay);
    for (const name of ['canvas','webgl','raylib']) {
      const canvas=document.createElement('canvas');canvas.width=width;canvas.height=height;canvas.dataset.candidate=name;canvas.id=`silk-study-${name}`;
      canvas.style.width=`${width===720?360:width}px`;canvas.style.height=`${height===1280?640:height}px`;
      document.body.append(canvas);canvases.push(canvas);
      const candidate=name==='canvas'?new CanvasCandidate(canvas,drawScene,overlay):name==='webgl'?
        new WebglCandidate(canvas,drawScene,overlay):await createRaylibCandidate(canvas,drawScene,overlay);
      candidates.push(candidate);
      if(canvas.width!==width || canvas.height!==height) throw new Error('Renderer changed the frozen drawing-buffer size');
      pixels.push(await capture(canvas,candidate));
    }
    // A repeated immutable frame must not upload/copy transforms again.
    for (const candidate of candidates) candidate.draw();
    if(candidates[1].uploadBytes!==0 || candidates[2].poseCopyBytes!==0 || candidates[2].diagnosticCopyBytes!==0) throw new Error('Unchanged poses copied again');
    let updated;
    if (!drawScene.frozen) {
      if(!world.step() || !world.refreshSnapshot(diagnostic))
        throw new Error('Transform update failed');
      drawScene.copyTransforms(world.snapshot);
      if(overlay)overlay.refresh(world.snapshot,world.diagnostics);
      const next=[];
      for(let i=0;i<candidates.length;++i) next.push(await capture(canvases[i],candidates[i]));
      const diagnosticBytes=overlay?overlay.lineCount*20+overlay.markerCount*12+overlay.bodyColors.byteLength:0;
      if(candidates[1].uploadBytes!==16*drawScene.count+diagnosticBytes || candidates[2].poseCopyBytes!==16*drawScene.count || candidates[2].diagnosticCopyBytes!==diagnosticBytes)
        throw new Error('Changing transform payload differs from its fixed budget');
      const band=edgeMask(drawScene,width,height,overlay);
      updated={webgl:compare(next[0],next[1],width,height,band),raylib:compare(next[0],next[2],width,height,band)};
    }
    const previews=images?pixels.map(bytes=>{
      const canvas=document.createElement('canvas');canvas.width=width;canvas.height=height;
      canvas.getContext('2d').putImageData(new ImageData(new Uint8ClampedArray(bytes),width,height),0,0);
      return canvas.toDataURL('image/png');
    }):undefined;
    return {scene,steps,copies,sleep,diagnostic,lines:overlay?.lineCount??0,markers:overlay?.markerCount??0,instances:drawScene.count,width,height,previews,updated,
      webgl:compare(pixels[0],pixels[1],width,height,edges),raylib:compare(pixels[0],pixels[2],width,height,edges),
      frozen:drawScene.frozen,batches:drawScene.batches.length,meshes:drawScene.meshes.length};
  } finally {
    for (const candidate of candidates) {
      candidate.dispose();candidate.dispose();
      let rejected=false;try{candidate.draw();}catch(error){rejected=/Disposed/.test(String(error));}
      if(!rejected) throw new Error('Disposed renderer accepted drawing');
    }
    for (const canvas of canvases) canvas.remove();
    world.dispose(); module.dispose();
  }
};

// Isolated correctness probe: opaque line-center and marker-center pixels must
// be present. The AA edge-band comparison alone cannot detect omitted 1px lines.
window.verifyOverlayPrimitives=async()=>{
  const module=await createStudyModule();const world=module.create('pyramid',{steps:1});
  if(!world)throw new Error('Primitive probe fixture failed');
  const candidates=[],canvases=[],results=[];
  try{
    if(!world.refreshSnapshot(true))throw new Error('Primitive probe snapshot failed');
    const scene=new DrawScene(world.snapshot,'pyramid');
    for(let i=0;i<scene.count;++i){scene.poses[4*i]=1e6;scene.poses[4*i+1]=1e6;}
    const config=world.configuration,overlay=new Overlay(scene,config.bodyCapacity,config.contactCapacity,config.jointCapacity,128,128);
    overlay.lines.set([16.5,24.5,96.5,24.5,12]);overlay.markers.set([64.5,64.5,11]);
    overlay.lineCount=1;overlay.markerCount=1;overlay.revision=1;
    for(const name of ['canvas','webgl','raylib']){
      const canvas=document.createElement('canvas');canvas.width=128;canvas.height=128;canvas.id=`silk-primitive-${name}`;
      document.body.append(canvas);canvases.push(canvas);
      const candidate=name==='canvas'?new CanvasCandidate(canvas,scene,overlay):name==='webgl'?new WebglCandidate(canvas,scene,overlay):await createRaylibCandidate(canvas,scene,overlay);
      candidates.push(candidate);const bytes=await capture(canvas,candidate);
      const rgb=(x,y)=>Array.from(bytes.slice(4*(y*128+x),4*(y*128+x)+3));
      results.push({candidate:name,line:rgb(40,24),above:rgb(40,23),below:rgb(40,25),marker:rgb(64,64),outside:rgb(64,68)});
    }
    return results;
  }finally{for(const candidate of candidates)candidate.dispose();for(const canvas of canvases)canvas.remove();world.dispose();module.dispose();}
};

// Exercise the private C validation boundary independently of the TS command
// builder. Repair caller-owned buffers after rejection, then prove recovery.
window.verifyRaylibValidation=async()=>{
  const {default:factory}=await import('./raylib.mjs');
  const canvas=document.createElement('canvas');canvas.id='silk-raylib-validation';canvas.width=128;canvas.height=128;document.body.append(canvas);
  const m=await factory({canvas,wasmMemory:new WebAssembly.Memory({initial:1024,maximum:1024})});let rejected=0;
  const expect=(value,message)=>{if(!value)throw new Error(message);};
  try{
    expect(!m._sl_render_overlay_init(1,1),'Overlay accepted before renderer setup');++rejected;
    expect(m._sl_render_init(1,3,1,128,128),'Validation renderer setup failed');
    expect(m._sl_render_overlay_init(1,1),'Validation overlay setup failed');
    expect(!m._sl_render_overlay_init(1,1),'Duplicate overlay accepted');++rejected;
    const f32=(pointer,count)=>new Float32Array(m.HEAPF32.buffer,pointer,count);
    const lines=f32(m._sl_render_overlay_lines(),5),markers=f32(m._sl_render_overlay_markers(),3);
    const colors=f32(m._sl_render_overlay_colors(),1),mesh=f32(m._sl_render_overlay_mesh(),180);
    const palette=new Uint32Array(m.HEAPU32.buffer,m._sl_render_overlay_palette(),48);
    lines.set([1,1,4,1,12]);markers.set([8,8,11]);
    expect(m._sl_render_overlay_counts(1,1),'Valid counts rejected');
    for(const counts of [[2,1],[1,2]]){expect(!m._sl_render_overlay_counts(...counts),'Overflow accepted');++rejected;}
    for(const [buffer,index,value]of [[lines,0,NaN],[lines,2,3.4e38],[lines,4,16],
        [markers,0,Infinity],[markers,2,-1],[colors,0,1.5],[mesh,0,NaN],[mesh,0,2],[palette,0,256]]){
      const saved=buffer[index];buffer[index]=value;
      expect(!m._sl_render_overlay_counts(1,1),'Invalid command accepted');++rejected;
      buffer[index]=saved;expect(m._sl_render_overlay_counts(1,1),'Repaired command rejected');
    }
    expect(m._sl_render_draw(1,0,0),'Repaired renderer draw failed');
    return {rejected};
  }finally{m._sl_render_dispose();canvas.remove();}
};

window.verifyColocated=async({scene,copies=1,sleep=false,width=1280,height=720})=>{
  const canvases=[],candidates=[];let module,referenceModule,world,reference;
  const expect=(condition,message)=>{if(!condition)throw new Error(message);};
  const equalSnapshot=(a,b)=>{
    for(const name of ['bodyCount','contactCount','jointCount'])expect(a[name]===b[name],`Snapshot ${name} differs`);
    for(const name of ['bodies','geometry','contacts','joints'])for(const key of Object.keys(a[name])){
      const x=a[name][key],y=b[name][key];expect(x.length===y.length,'Snapshot capacity differs');
      for(let i=0;i<x.length;++i)expect(Object.is(x[i],y[i]),`Snapshot ${name}.${key}[${i}] differs`);
    }
  };
  try{
    for(const id of ['reference','direct']){
      const canvas=document.createElement('canvas');canvas.id=`silk-colocated-${id}`;canvas.width=width;canvas.height=height;
      canvas.style.width=`${width===720?360:width}px`;canvas.style.height=`${height===1280?640:height}px`;
      document.body.append(canvas);canvases.push(canvas);
    }
    module=await createRaylibStudyModule(canvases[1],{memoryBytes:256*1024*1024});
    referenceModule=await createStudyModule({memoryBytes:256*1024*1024});
    const steps=copies===1?120:3;
    world=module.create(scene,{copies,sleep,steps:steps+1});reference=referenceModule.create(scene,{copies,sleep,steps:steps+1});
    expect(world&&reference,'Co-located fixture allocation failed');
    for(let i=0;i<steps;++i)expect(world.step()&&reference.step(),'Co-located step failed');
    const candidate=world.createRenderer();candidates.push(candidate);
    expect(candidate.linearMemoryBytes===256*1024*1024&&module.memory.linearMemoryBytes===candidate.linearMemoryBytes,'Fixed memory budget differs');
    let rejected=false;try{world.createRenderer();}catch{rejected=true;}expect(rejected,'Duplicate renderer accepted');
    expect(reference.refreshSnapshot(true)&&world.refreshSnapshot(true),'Snapshot failed');
    equalSnapshot(world.snapshot,reference.snapshot);
    const draw=new DrawScene(reference.snapshot,scene,undefined,copies),canvasCandidate=new CanvasCandidate(canvases[0],draw);candidates.push(canvasCandidate);
    const results=[];
    let allocatorUsed;
    for(let frame=0;frame<2;++frame){
      if(frame){
        expect(world.step()&&reference.step()&&reference.refreshSnapshot(true),'Updated step failed');draw.copyTransforms(reference.snapshot);
      }
      // A retained JS snapshot is caller-writable. Poisoning it must not affect
      // direct C drawing or the authoritative simulation.
      world.snapshot.bodies.x.fill(1e6);world.snapshot.bodies.y.fill(1e6);
      const actual=await capture(canvases[1],candidate),expected=await capture(canvases[0],canvasCandidate);
      expect(candidate.poseCopyBytes===0&&candidate.posePrepareBytes===16*draw.count,'Direct pose payload incorrect');
      results.push(compare(expected,actual,width,height,edgeMask(draw,width,height)));
      expect(world.refreshSnapshot(true),'Post-draw snapshot failed');equalSnapshot(world.snapshot,reference.snapshot);
      candidate.draw();expect(candidate.poseCopyBytes===0&&candidate.posePrepareBytes===0,'Repeated direct poses prepared again');
      if(frame===0)allocatorUsed=module.memory.allocatorUsedBytes;
      else expect(module.memory.allocatorUsedBytes===allocatorUsed,'Steady-state C allocation changed');
    }
    world.dispose();rejected=false;try{candidate.draw();}catch{rejected=true;}expect(rejected,'Renderer outlived its world');
    // A disposed world releases the sole raylib window; a new owner can attach.
    const recovered=module.create('pyramid',{steps:1});expect(recovered,'Replacement world failed');
    const replacement=recovered.createRenderer();replacement.draw();module.dispose();
    rejected=false;try{replacement.draw();}catch{rejected=true;}expect(rejected,'Renderer outlived its module');
    return {scene,copies,sleep,width,height,results,poseCopyBytes:0,linearMemoryBytes:256*1024*1024};
  }finally{
    for(const candidate of candidates)candidate.dispose();world?.dispose();reference?.dispose();module?.dispose();referenceModule?.dispose();
    for(const canvas of canvases)canvas.remove();
  }
};
