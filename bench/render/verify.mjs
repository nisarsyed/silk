import {createBenchmarkModule} from './benchmark/driver.mjs';
import {DrawScene} from './geometry.js';
import {edgeMask,compare} from './pixels.js';
import {CanvasCandidate} from './canvas.js';
import {WebglCandidate} from './webgl.js';
import {createRaylibCandidate} from './raylib.js';

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
window.verifyRenderers=async ({scene='rain',steps=120,instances,width=1280,height=720,images=false}={})=>{
  const module=await createBenchmarkModule();
  const world=module.create(scene,{warmup:0,steps:steps+1});
  if (!world) throw new Error('Fixture allocation failed');
  const canvases=[], candidates=[];
  try {
    for (let i=0; i<steps; ++i) {
      if (!world.prepare() || !world.mutate() || !world.step() || !world.sample()) throw new Error('Fixture stepping failed');
    }
    if (!world.refreshSnapshot()) throw new Error('Snapshot failed');
    const drawScene=new DrawScene(world.snapshot,scene,instances), pixels=[];
    const initialPoses=drawScene.poses.slice();
    for (const name of ['canvas','webgl','raylib']) {
      const canvas=document.createElement('canvas');canvas.width=width;canvas.height=height;canvas.dataset.candidate=name;canvas.id=`silk-study-${name}`;
      canvas.style.width=`${width===720?360:width}px`;canvas.style.height=`${height===1280?640:height}px`;
      document.body.append(canvas);canvases.push(canvas);
      const candidate=name==='canvas'?new CanvasCandidate(canvas,drawScene):name==='webgl'?
        new WebglCandidate(canvas,drawScene):await createRaylibCandidate(canvas,drawScene);
      candidates.push(candidate);
      if(canvas.width!==width || canvas.height!==height) throw new Error('Renderer changed the frozen drawing-buffer size');
      pixels.push(await capture(canvas,candidate));
    }
    // A repeated immutable frame must not upload/copy transforms again.
    for (const candidate of candidates) candidate.draw();
    if(candidates[1].uploadBytes!==0 || candidates[2].poseCopyBytes!==0) throw new Error('Unchanged poses copied again');
    let updated;
    if (!drawScene.frozen) {
      if(!world.prepare() || !world.mutate() || !world.step() || !world.sample() || !world.refreshSnapshot())
        throw new Error('Transform update failed');
      drawScene.copyTransforms(world.snapshot);
      const next=[];
      for(let i=0;i<candidates.length;++i) next.push(await capture(canvases[i],candidates[i]));
      if(candidates[1].uploadBytes!==16*drawScene.count || candidates[2].poseCopyBytes!==16*drawScene.count)
        throw new Error('Changing transform payload differs from its fixed budget');
      const band=edgeMask(drawScene,width,height);
      updated={webgl:compare(next[0],next[1],width,height,band),raylib:compare(next[0],next[2],width,height,band)};
    }
    // Initial image comparison uses its original poses, not the updated state.
    if (!drawScene.frozen) drawScene.poses.set(initialPoses);
    const edges=edgeMask(drawScene,width,height);
    const previews=images?pixels.map(bytes=>{
      const canvas=document.createElement('canvas');canvas.width=width;canvas.height=height;
      canvas.getContext('2d').putImageData(new ImageData(new Uint8ClampedArray(bytes),width,height),0,0);
      return canvas.toDataURL('image/png');
    }):undefined;
    return {scene,steps,instances:drawScene.count,width,height,previews,updated,
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
