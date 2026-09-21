import * as api from '@nisarsyed/silk';
import { exercise } from './exercise.mjs';
async function run() {
  const result = await exercise(api);
  await exercise(api, {wasmUrl:new URL('./relocated/silk.wasm', import.meta.url)});
  for (const path of ['missing.wasm', 'corrupt.wasm']) {
    try { await api.createSilk({wasmUrl:new URL(path, import.meta.url)}); }
    catch (error) { if (error.message.includes('asset URL') && error.cause) continue; throw error; }
    throw new Error('Invalid asset unexpectedly loaded');
  }
  const worker = new Worker(new URL('./worker.mjs', import.meta.url), {type:'module'});
  try {
    const received = await new Promise((resolve,reject) => {
      const timer = setTimeout(() => reject(new Error('Dedicated worker timed out')),30000);
      worker.onmessage = event => { clearTimeout(timer); event.data.error ? reject(new Error(event.data.error)) : resolve(event.data); };
      worker.onerror = event => { clearTimeout(timer); reject(new Error(event.message || `Worker loading error at ${event.filename ?? 'module dependency'}`)); };
      worker.postMessage({entry:new URL('./node_modules/@nisarsyed/silk/index.mjs',import.meta.url).href,
        wasmUrl:new URL('./relocated/silk.wasm',import.meta.url).href});
    });
    if (!received.worker || received.version !== result.version) throw new Error('Worker result mismatch');
  } finally { worker.terminate(); }
  document.querySelector('#result').textContent = JSON.stringify({...result,worker:true});
  document.documentElement.dataset.result = 'pass';
}
run().catch(error => { document.querySelector('#result').textContent = error.stack; document.documentElement.dataset.result = 'fail'; });
