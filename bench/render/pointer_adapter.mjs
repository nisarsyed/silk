import {PointerQueue,pointerAction,pointerQueueCapacity} from './host_input_queue.mjs';
import {camera,sceneRects} from './geometry.js';

const buffers=Object.freeze({desktop:[1280,720],mobile:[720,1280]});

// DOM input is copied into a bounded local queue. One batch at a time crosses
// the worker boundary; the simulation owner applies it at a fixed step.
export function createPointerAdapter(controller,{onError}={}){
  if(!controller||!(controller.canvas instanceof HTMLCanvasElement)||
      !Number.isSafeInteger(controller.epoch)||controller.epoch<1||
      typeof controller.pointerBatch!=='function'||
      (onError!==undefined&&typeof onError!=='function'))
    throw new TypeError('Invalid pointer adapter controller');
  let canvas=controller.canvas,epoch=controller.epoch,scene=controller.scene;
  let queue=new PointerQueue(pointerQueueCapacity,epoch),sequence=0,active=null;
  let pending=false,frame=0,disposed=false,suspended=false,failure=null;
  let touchAction=canvas.style.touchAction;
  const clearCapture=()=>{
    const id=active;active=null;
    if(id!==null&&canvas.hasPointerCapture(id))canvas.releasePointerCapture(id);
  };
  const fail=error=>{
    if(disposed||failure)return;
    failure=error instanceof Error?error.message:String(error);
    clearCapture();
    void controller.pause().catch(()=>{});
    onError?.(failure);
  };
  const coordinates=event=>{
    const rect=canvas.getBoundingClientRect(),[width,height]=buffers[controller.layout]??[];
    if(!width||!rect.width||!rect.height||!Object.hasOwn(sceneRects,scene))
      throw new Error('Invalid pointer coordinate frame');
    const view=camera(sceneRects[scene],width,height);
    const px=Math.max(0,Math.min(width,(event.clientX-rect.left)*width/rect.width));
    const py=Math.max(0,Math.min(height,(event.clientY-rect.top)*height/rect.height));
    return [Math.fround((px-view.x)/view.scale),Math.fround((view.y-py)/view.scale)];
  };
  const sync=()=>{
    if(controller.mode==='disposed'){dispose();return false;}
    if(controller.mode==='failed'||controller.mode==='recovering'){
      if(!suspended){clearCapture();queue=new PointerQueue(pointerQueueCapacity,epoch);
        sequence=0;suspended=true;}
      return false;
    }
    suspended=false;
    if(canvas!==controller.canvas){
      clearCapture();detach();canvas=controller.canvas;touchAction=canvas.style.touchAction;attach();
      queue=new PointerQueue(pointerQueueCapacity,controller.epoch);sequence=0;
    }
    if(epoch!==controller.epoch||scene!==controller.scene){
      clearCapture();epoch=controller.epoch;scene=controller.scene;
      queue=new PointerQueue(pointerQueueCapacity,epoch);sequence=0;
    }
    return true;
  };
  const enqueue=(action,event)=>{
    if(failure||disposed)return;
    try{
      if(!sync()||failure)return;
      const [x,y]=coordinates(event);
      const result=queue.enqueue(epoch,++sequence,action,event.pointerId,x,y);
      if(result==='overflow')fail('DOM pointer queue capacity exhausted');
    }catch(error){fail(error);}
  };
  const down=event=>{
    if(active!==null||!event.isPrimary||event.button!==0)return;
    if(!sync()||failure)return;
    active=event.pointerId;
    try{canvas.setPointerCapture(active);}catch(error){active=null;fail(error);return;}
    enqueue(pointerAction.down,event);event.preventDefault();
  };
  const move=event=>{if(event.pointerId===active){enqueue(pointerAction.move,event);event.preventDefault();}};
  const end=(event,action)=>{
    if(event.pointerId!==active)return;
    enqueue(action,event);clearCapture();event.preventDefault();
  };
  const up=event=>end(event,pointerAction.up);
  const cancel=event=>end(event,pointerAction.cancel);
  const attach=()=>{
    canvas.style.touchAction='none';
    canvas.addEventListener('pointerdown',down);canvas.addEventListener('pointermove',move);
    canvas.addEventListener('pointerup',up);canvas.addEventListener('pointercancel',cancel);
    canvas.addEventListener('lostpointercapture',cancel);
  };
  const detach=()=>{
    canvas.removeEventListener('pointerdown',down);canvas.removeEventListener('pointermove',move);
    canvas.removeEventListener('pointerup',up);canvas.removeEventListener('pointercancel',cancel);
    canvas.removeEventListener('lostpointercapture',cancel);canvas.style.touchAction=touchAction;
  };
  const dispose=()=>{if(disposed)return;disposed=true;cancelAnimationFrame(frame);clearCapture();detach();};
  const pump=()=>{
    frame=0;if(disposed||failure)return;
    try{
      const ready=sync();
      if(ready&&!failure&&!pending&&queue.count){
        const batch=[];
        queue.drain((action,id,x,y,number)=>batch.push([epoch,number,action,id,x,y]));
        pending=true;
        Promise.resolve(controller.pointerBatch(batch)).then(result=>{
          if(result?.queued+result?.coalesced+result?.droppedMove+result?.stale!==batch.length)
            throw new Error('Incomplete pointer batch acknowledgement');
        }).catch(error=>{if(controller.mode!=='failed'&&controller.mode!=='recovering')fail(error);})
          .finally(()=>{pending=false;});
      }
    }catch(error){fail(error);}
    if(!failure)frame=requestAnimationFrame(pump);
  };
  attach();frame=requestAnimationFrame(pump);
  return Object.freeze({
    get failure(){return failure;},get pending(){return pending;},
    get statistics(){return queue.statistics;},
    dispose(){dispose();}
  });
}
