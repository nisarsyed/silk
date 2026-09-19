import { exercise } from './exercise.mjs';
self.onmessage = async ({data}) => {
  try {
    if (!(self instanceof DedicatedWorkerGlobalScope)) throw new Error('Not an actual dedicated worker');
    const api = await import(data.entry);
    await exercise(api);
    const result = await exercise(api, {wasmUrl:data.wasmUrl});
    self.postMessage({...result,worker:true});
  } catch (error) { self.postMessage({error:error.stack}); }
};
