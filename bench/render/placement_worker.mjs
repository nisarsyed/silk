import {runPlacement} from './placement_probe.mjs';

self.onmessage=async event=>{
  try {
    if (!(self instanceof DedicatedWorkerGlobalScope) ||
        !(event.data.canvas instanceof OffscreenCanvas)) throw new Error('Worker canvas unavailable');
    const result=await runPlacement(event.data.canvas,event.data.profile);
    // WebGL's default drawing buffer is presented on the worker's frame
    // boundary. A synchronous correctness replay needs one actual worker rAF.
    await new Promise(requestAnimationFrame);
    self.postMessage({ok:true,result});
  } catch (error) {
    self.postMessage({ok:false,error:error instanceof Error?error.stack:String(error)});
  }
};
