import createModule from './study.mjs';
import {ownStudyModule} from './owner.mjs';
export {jsonReplacer,physicsTiers,stepLimit} from './owner.mjs';
function integer(value,low,high,name) {
  if(!Number.isInteger(value)||value<low||value>high) throw new RangeError(`Invalid ${name}`);
}
export async function createStudyModule({memoryBytes=64*1024*1024,wasmUrl}={}) {
  integer(memoryBytes,2*1024*1024,512*1024*1024,'memoryBytes');
  if(memoryBytes%65536) throw new RangeError('Memory budget must be 64 KiB aligned');
  if(wasmUrl!==undefined&&typeof wasmUrl!=='string'&&!(wasmUrl instanceof URL)) throw new TypeError('Invalid wasmUrl');
  let m;
  try {m=await createModule({wasmMemory:new WebAssembly.Memory({initial:memoryBytes/65536,maximum:memoryBytes/65536}),
    ...(wasmUrl===undefined?{}:{locateFile:()=>String(wasmUrl)})});}
  catch(cause){throw new Error('Study initialization failed; check asset URL and fixed memory budget',{cause});}
  return ownStudyModule(m,memoryBytes);
}
