import {createRuntimeOwner} from './runtime_owner.mjs';
import {validPointerBatch} from './host_input_queue.mjs';

const claimed=new WeakSet();
const pendingMax=32;
const cssFrames=Object.freeze({desktop:[1280,720],mobile:[360,640]});

// Main-thread coordinator. A worker receives the canvas only after ownership
// is claimed. Startup failure terminates the worker, replaces the transferred
// canvas, then constructs one fresh main-thread world from the initial scene.
export async function startBrowserRuntime(canvas,profile={},{preferWorker=true,workerUrl,onStatus}={}){
  if(!(canvas instanceof HTMLCanvasElement)||!canvas.isConnected||
      profile===null||typeof profile!=='object'||Array.isArray(profile)||
      typeof preferWorker!=='boolean'||(onStatus!==undefined&&typeof onStatus!=='function'))
    throw new TypeError('Invalid browser runtime canvas or options');
  if(claimed.has(canvas))throw new Error('Canvas already has a runtime controller');
  claimed.add(canvas);
  let current=canvas,worker=null,owner=null,mode='starting',fallbackReason=null,failureReason=null,disposed=false;
  let currentEpoch=1,currentLayout=profile.layout??'desktop',currentScene=profile.scene??'pyramid';
  let userPaused=false,visibilityPaused=false;
  let nextId=0;
  const pending=new Map();
  const assertLive=()=>{if(disposed)throw new Error('Runtime controller is disposed');};
  const stopWorker=reason=>{
    const active=worker;worker=null;
    active?.terminate();
    for(const entry of pending.values()){clearTimeout(entry.timer);entry.reject(new Error(reason));}
    pending.clear();
  };
  const failWorker=reason=>{
    if(!worker)return;
    failureReason=reason;
    stopWorker(reason);
    if(mode==='worker'){mode='failed';onStatus?.({state:'failed',reason});}
  };
  const status=value=>{
    if(Number.isSafeInteger(value?.epoch))currentEpoch=value.epoch;
    onStatus?.(value);
  };
  const received=event=>{
    const data=event.data;
    if(data?.kind==='status'){
      if(data.status?.state==='failed')failWorker(data.status.reason||'Worker runtime failed');
      else status(data.status);
      return;
    }
    const entry=pending.get(data?.id);
    if(!entry)return;
    clearTimeout(entry.timer);pending.delete(data.id);
    if(data.ok)entry.resolve(data);
    else entry.reject(new Error(data.error));
  };
  const request=(kind,fields={},transfer=[],timeoutMs=15000)=>{
    assertLive();
    if(!worker||pending.size===pendingMax||nextId===Number.MAX_SAFE_INTEGER)
      throw new Error('Worker control capacity or connection exhausted');
    const id=++nextId;
    return new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{
        pending.delete(id);failWorker(`Worker ${kind} timed out`);
        reject(new Error(`Worker ${kind} timed out`));
      },timeoutMs);
      pending.set(id,{resolve,reject,timer});
      try{worker.postMessage({id,kind,...fields},transfer);}
      catch(error){clearTimeout(timer);pending.delete(id);reject(error);}
    });
  };
  const replaceCanvas=()=>{
    const replacement=current.cloneNode(false);
    current.replaceWith(replacement);
    claimed.delete(current);claimed.add(replacement);current=replacement;
  };
  const setCssFrame=layout=>{
    const [width,height]=cssFrames[layout];
    // Keep the contract buffer fixed while fitting its CSS frame into either
    // viewport dimension. The explicit ratio also covers a transferred HTML
    // canvas whose intrinsic attributes cannot be changed after transfer.
    current.style.aspectRatio=`${width} / ${height}`;
    current.style.width=`min(${width}px, 100vw, ${100*width/height}vh)`;
    current.style.height='auto';
  };
  const startMain=async()=>{
    owner=await createRuntimeOwner(current,{...profile,onStatus:status});mode='main';
    currentEpoch=owner.epoch;currentLayout=profile.layout??'desktop';
    currentScene=profile.scene??'pyramid';
    setCssFrame(profile.layout??'desktop');
  };
  try{
    if(preferWorker&&typeof Worker!=='undefined'&&
        typeof canvas.transferControlToOffscreen==='function'){
      try{
        worker=new Worker(workerUrl??new URL('./runtime_worker.mjs',import.meta.url),{type:'module'});
        worker.addEventListener('message',received);
        worker.addEventListener('error',event=>failWorker(event.message||'Worker startup failed'));
        worker.addEventListener('messageerror',()=>failWorker('Worker message failed'));
        const offscreen=canvas.transferControlToOffscreen();
        const initial=await request('init',{canvas:offscreen,profile},[offscreen]);
        currentEpoch=initial.epoch;
        mode='worker';
        setCssFrame(profile.layout??'desktop');
      }catch(error){
        stopWorker('Worker startup failed');
        fallbackReason=`Worker startup failed: ${error instanceof Error?error.message:error}`;
        replaceCanvas();await startMain();
      }
    }else{
      if(preferWorker)fallbackReason='Worker or OffscreenCanvas unavailable';
      await startMain();
    }
    const control=async(kind,fields={})=>{
      assertLive();
      if(mode==='main'){
        if(kind==='pause')return owner.pause();
        if(kind==='resume')return owner.resume();
        if(kind==='single-step')return owner.singleStep();
        if(kind==='reset')return owner.reset(...fields.args);
        if(kind==='resize')return owner.resize(fields.layout);
        if(kind==='pointer')return owner.pointer(...fields.args);
        if(kind==='pointer-batch'){
          const counts={queued:0,coalesced:0,droppedMove:0,stale:0};
          for(const item of fields.events){const status=owner.pointer(...item);
            if(status==='queued')++counts.queued;
            else if(status==='coalesced')++counts.coalesced;
            else if(status==='dropped-move')++counts.droppedMove;
            else if(status==='stale')++counts.stale;
            else throw new Error(`Pointer batch failed: ${status}`);
          }
          return counts;
        }
        if(kind==='report')return owner.report();
      }
      if(mode!=='worker')throw new Error('Runtime requires explicit recovery');
      try{const packet=await request(kind,fields);
        if(Number.isSafeInteger(packet.epoch))currentEpoch=packet.epoch;
        return packet.result;
      }
      catch(error){failWorker(`Worker ${kind} failed: ${error}`);throw error;}
    };
    const visibility=()=>{
      if(disposed||mode==='failed')return;
      if(document.visibilityState!=='visible'){
        if(!userPaused&&!visibilityPaused){visibilityPaused=true;
          void control('pause').catch(error=>failWorker(`Visibility pause failed: ${error}`));}
      }else if(visibilityPaused){visibilityPaused=false;
        void control('resume').catch(error=>failWorker(`Visibility resume failed: ${error}`));}
    };
    document.addEventListener('visibilitychange',visibility);
    if(document.visibilityState!=='visible')visibility();
    return Object.freeze({
      get canvas(){return current;},get mode(){return mode;},
      get epoch(){return currentEpoch;},get layout(){return currentLayout;},
      get scene(){return currentScene;},
      get fallbackReason(){return fallbackReason;},get failureReason(){return failureReason;},
      async pause(){userPaused=true;visibilityPaused=false;return control('pause');},
      async resume(){userPaused=false;
        if(document.visibilityState!=='visible'){visibilityPaused=true;return false;}
        return control('resume');},
      singleStep(){return control('single-step');},
      async reset(scene,sleep){const changed=await control('reset',{args:[scene,sleep]});
        if(changed)currentScene=scene??currentScene;return changed;},
      async resize(layout){const changed=await control('resize',{layout});
        if(changed){currentLayout=layout;setCssFrame(layout);}return changed;},
      pointer(epoch,sequence,action,pointerId,x,y){
        return control('pointer',{args:[epoch,sequence,action,pointerId,x,y]});
      },
      pointerBatch(events){
        if(!validPointerBatch(events))
          throw new TypeError('Invalid pointer batch');
        return control('pointer-batch',{events});
      },
      async report(){const value=await control('report'),rect=current.getBoundingClientRect();
        return {...value,cssWidth:rect.width,cssHeight:rect.height,
          devicePixelRatio:window.devicePixelRatio,
          bufferCssRatioX:value.bufferWidth/rect.width,
          bufferCssRatioY:value.bufferHeight/rect.height};},
      async recoverToMain(){
        assertLive();
        if(mode==='main'&&owner.state!=='failed')throw new Error('Live main owner needs no recovery');
        stopWorker('Explicit main-thread recovery');owner?.dispose();owner=null;
        replaceCanvas();mode='recovering';
        try{
          await startMain();
          fallbackReason='Explicit recovery restarted the source scene on the main thread';
          failureReason=null;
          return owner.report();
        }catch(error){mode='failed';failureReason=`Main-thread recovery failed: ${error}`;throw error;}
      },
      async dispose(){
        if(disposed)return;
        document.removeEventListener('visibilitychange',visibility);
        if(mode==='worker'){
          try{await request('dispose',{},[],2000);}catch{/* Termination still releases the worker owner. */}
        }
        disposed=true;stopWorker('Runtime controller disposed');
        owner?.dispose();owner=null;mode='disposed';claimed.delete(current);
      }
    });
  }catch(error){
    stopWorker('Runtime startup failed');owner?.dispose();claimed.delete(current);
    throw error;
  }
}
