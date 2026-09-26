// Probe the actual dedicated-worker context before requiring its renderer path.
// Linux WebKit can expose WebGL 2 on HTMLCanvasElement yet not in a worker.
export function workerWebgl2(page){
  return page.evaluate(()=>new Promise((resolve,reject)=>{
    const source='self.postMessage(typeof OffscreenCanvas!=="undefined"&&'+
      '!!new OffscreenCanvas(16,16).getContext("webgl2"));';
    const url=URL.createObjectURL(new Blob([source],{type:'text/javascript'}));
    const worker=new Worker(url);
    const finish=()=>{clearTimeout(timer);worker.terminate();URL.revokeObjectURL(url);};
    const timer=setTimeout(()=>{finish();reject(new Error('Worker WebGL probe timed out'));},10000);
    worker.onmessage=event=>{finish();resolve(event.data===true);};
    worker.onerror=event=>{finish();reject(new Error(event.message));};
  }));
}
