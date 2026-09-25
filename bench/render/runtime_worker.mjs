import {createRuntimeOwner} from './runtime_owner.mjs';
import {validPointerBatch} from './host_input_queue.mjs';

let owner=null,opening=false;
const reply=(id,ok,result)=>self.postMessage({id,ok,epoch:owner?.epoch??null,
  ...(ok?{result}:{error:String(result)})});

self.onmessage=async event=>{
  const {id,kind}=event.data;
  if(!Number.isSafeInteger(id)||id<1)return;
  try{
    let result;
    if(kind==='init'){
      if(owner||opening)throw new Error('Worker already has an authoritative owner');
      opening=true;
      try{
        owner=await createRuntimeOwner(event.data.canvas,{...event.data.profile,
          onStatus:status=>self.postMessage({kind:'status',status})});
      }finally{opening=false;}
      result=owner.report();
    }else{
      if(!owner)throw new Error('Worker has no authoritative owner');
      if(kind==='pointer')result=owner.pointer(...event.data.args);
      else if(kind==='pointer-batch'){
        const batch=event.data.events;
        if(!validPointerBatch(batch))
          throw new TypeError('Invalid pointer batch');
        const counts={queued:0,coalesced:0,droppedMove:0,stale:0};
        for(const item of batch){
          const status=owner.pointer(...item);
          if(status==='queued')++counts.queued;
          else if(status==='coalesced')++counts.coalesced;
          else if(status==='dropped-move')++counts.droppedMove;
          else if(status==='stale')++counts.stale;
          else throw new Error(`Pointer batch failed: ${status}`);
        }
        result=counts;
      }
      else if(kind==='pause')result=owner.pause();
      else if(kind==='resume')result=owner.resume();
      else if(kind==='single-step')result=owner.singleStep();
      else if(kind==='reset')result=owner.reset(...event.data.args);
      else if(kind==='resize')result=owner.resize(event.data.layout);
      else if(kind==='report')result=owner.report();
      else if(kind==='dispose'){
        owner.dispose();owner=null;result='disposed';
      }else throw new TypeError('Unknown worker control');
    }
    reply(id,true,result);
    if(kind==='dispose')self.close();
  }catch(error){reply(id,false,error instanceof Error?error.message:error);}
};
