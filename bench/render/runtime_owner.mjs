import {createStudyModule} from './physics/driver.mjs';
import {DrawScene} from './geometry.js';
import {CanvasCandidate} from './canvas.js';
import {WebglCandidate} from './webgl.js';
import {FixedClock} from './host_clock.mjs';
import {PointerQueue} from './host_input_queue.mjs';

const active=new WeakSet();
const layouts=Object.freeze({desktop:[1280,720],mobile:[720,1280]});

// One study owner per canvas. Main and worker instantiate the same code; no
// state copy or second C world is used for rendering. This is a bounded host
// path for the placement experiment, not a production renderer decision.
export async function createRuntimeOwner(canvas,{candidate,scene='pyramid',sleep=false,
  layout='desktop',onStatus}={}) {
  if (!((typeof HTMLCanvasElement!=='undefined'&&canvas instanceof HTMLCanvasElement) ||
        (typeof OffscreenCanvas!=='undefined'&&canvas instanceof OffscreenCanvas)) ||
      !['canvas','webgl'].includes(candidate) || !['pyramid','rain','chains'].includes(scene) ||
      typeof sleep!=='boolean' || !Object.hasOwn(layouts,layout) ||
      (onStatus!==undefined&&typeof onStatus!=='function'))
    throw new TypeError('Invalid browser runtime profile');
  if (active.has(canvas)) throw new Error('Canvas already has an authoritative owner');
  active.add(canvas);
  let module,world,drawScene,renderer,frameId=0,disposed=false,failure=null;
  let state='initializing',sceneName=scene,sleepEnabled=sleep,layoutName=layout;
  const clock=new FixedClock(),queue=new PointerQueue();
  const notify=()=>onStatus?.({state,reason:failure,steps:world?.steps??null,epoch:queue.epoch});
  const live=()=>{if(disposed)throw new Error('Browser runtime is disposed');};
  const stopped=()=>{live();if(state==='failed')throw new Error(`Browser runtime failed: ${failure}`);};
  const clearWorld=()=>{
    const oldRenderer=renderer,oldWorld=world;
    renderer=undefined;world=undefined;drawScene=undefined;
    try{oldRenderer?.dispose();}finally{oldWorld?.dispose();}
  };
  const fail=error=>{
    if(disposed||state==='failed')return;
    failure=error instanceof Error?error.message:String(error);
    state='failed';cancelAnimationFrame(frameId);frameId=0;notify();
  };
  const graphicsLost=event=>{event.preventDefault();fail('Graphics context lost; explicit recovery required');};
  const openWorld=()=>{
    world=module.create(sceneName,{sleep:sleepEnabled,steps:21600});
    if(!world||!world.refreshSnapshot())throw new Error('Runtime world creation failed');
    drawScene=new DrawScene(world.snapshot,sceneName);
    renderer=candidate==='canvas'?new CanvasCandidate(canvas,drawScene):new WebglCandidate(canvas,drawScene);
    renderer.draw();
  };
  const drawFrame=()=>{
    frameId=0;
    if(disposed||state==='failed')return;
    try{
      const steps=clock.tick(performance.now());
      for(let i=0;i<steps;++i){
        queue.drain((action,_id,x,y)=>{
          if(!world.pointer(action,x,y))throw new Error('Queued pointer action failed');
        });
        if(!world.step())throw new Error('Runtime step failed');
      }
      if(steps){
        if(!world.refreshSnapshot())throw new Error('Runtime snapshot failed');
        drawScene.copyTransforms(world.snapshot);
      }
      renderer.draw();
      frameId=requestAnimationFrame(drawFrame);
    }catch(error){fail(error);}
  };
  try{
    const [width,height]=layouts[layoutName];canvas.width=width;canvas.height=height;
    module=await createStudyModule();
    openWorld();
    canvas.addEventListener('webglcontextlost',graphicsLost);
    canvas.addEventListener('contextlost',graphicsLost);
    clock.reset(performance.now());
    state='running';frameId=requestAnimationFrame(drawFrame);notify();
    return Object.freeze({
      get state(){return state;},get failure(){return failure;},get epoch(){return queue.epoch;},
      get canvas(){return canvas;},
      pointer(epoch,sequence,action,pointerId,x,y){
        stopped();const result=queue.enqueue(epoch,sequence,action,pointerId,x,y);
        if(result==='overflow')fail('Pointer queue capacity exhausted');
        return result;
      },
      pause(){stopped();if(clock.paused)return false;
        try{
          clock.setPaused(true,performance.now());queue.reset(queue.epoch+1);
          if(!world.pointer(3,0,0))throw new Error('Pointer cancellation failed');
          state='paused';notify();return true;
        }catch(error){fail(error);return false;}
      },
      resume(){stopped();if(!clock.paused)return false;
        clock.setPaused(false,performance.now());state='running';notify();return true;},
      singleStep(){stopped();return clock.requestSingleStep();},
      reset(nextScene=sceneName,nextSleep=sleepEnabled){
        stopped();if(!['pyramid','rain','chains'].includes(nextScene)||typeof nextSleep!=='boolean')
          throw new TypeError('Invalid runtime reset profile');
        try{
          clearWorld();sceneName=nextScene;sleepEnabled=nextSleep;
          queue.reset(queue.epoch+1);clock.reset(performance.now());openWorld();
          notify();return true;
        }catch(error){fail(error);return false;}
      },
      resize(nextLayout){
        stopped();if(!Object.hasOwn(layouts,nextLayout))throw new TypeError('Invalid runtime layout');
        if(nextLayout===layoutName)return false;
        try{
          renderer.dispose();renderer=undefined;
          const [width,height]=layouts[nextLayout];canvas.width=width;canvas.height=height;
          layoutName=nextLayout;
          renderer=candidate==='canvas'?new CanvasCandidate(canvas,drawScene):new WebglCandidate(canvas,drawScene);
          renderer.draw();notify();return true;
        }catch(error){fail(error);return false;}
      },
      report(){live();return {state,reason:failure,candidate,scene:sceneName,sleep:sleepEnabled,
        layout:layoutName,bufferWidth:canvas.width,bufferHeight:canvas.height,
        steps:world?.steps??null,epoch:queue.epoch,
        pointerHeld:world?.pointerState.held??null,clock:{totalSteps:clock.totalSteps,
          debtSeconds:clock.debtSeconds,droppedSeconds:clock.droppedSeconds,
          suspendedSeconds:clock.suspendedSeconds,interpolation:clock.interpolation},
        input:queue.statistics,memory:module?.memory??null};},
      dispose(){if(disposed)return;
        disposed=true;state='disposed';cancelAnimationFrame(frameId);frameId=0;
        canvas.removeEventListener('webglcontextlost',graphicsLost);
        canvas.removeEventListener('contextlost',graphicsLost);
        try{clearWorld();}finally{module?.dispose();active.delete(canvas);notify();}}
    });
  }catch(error){
    cancelAnimationFrame(frameId);
    try{clearWorld();}finally{module?.dispose();active.delete(canvas);}
    throw error;
  }
}
