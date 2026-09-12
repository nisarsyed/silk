import * as api from '@nisarsyed/silk';
import wasmUrl from '@nisarsyed/silk/silk.wasm?url';
import { exercise } from './exercise.mjs';
exercise(api,{wasmUrl}).then(result => {
  if (!new URL(wasmUrl, location.href).pathname.startsWith('/nested/demo/assets/')) throw new Error('Bundler asset base mismatch');
  document.querySelector('#result').textContent = JSON.stringify(result);
  document.documentElement.dataset.result = 'pass';
}).catch(error => { document.querySelector('#result').textContent = error.stack; document.documentElement.dataset.result = 'fail'; });
