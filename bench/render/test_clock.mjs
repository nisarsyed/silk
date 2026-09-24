// Installed only by the correctness harness when explicitly requested. Native
// rAF still yields between callbacks and renders real images; its variable
// host timestamps are replaced by a declared synthetic 60 Hz clock. No timing
// from this mode is hardware evidence. Production collector code is untouched.
export function installSyntheticClock(){
  const request=window.requestAnimationFrame.bind(window),cancel=window.cancelAnimationFrame.bind(window);
  const callbacks=new Map();let pending=0,nextId=0,now=0,previousRaf=0;
  const control={periodMs:1000/60,repeatAfter:0};window.__silkTestClock=control;
  Object.defineProperty(performance,'now',{configurable:true,value:()=>now});
  const tick=()=>{
    pending=0;now+=control.periodMs;
    const repeat=control.repeatAfter>0&&--control.repeatAfter===0;
    const raf=repeat?previousRaf:now;previousRaf=raf;
    const ids=Array.from(callbacks.keys());
    for(const id of ids){const callback=callbacks.get(id);if(callback){callbacks.delete(id);callback(raf);}}
  };
  window.requestAnimationFrame=callback=>{
    const id=++nextId;callbacks.set(id,callback);if(!pending)pending=request(tick);return id;
  };
  window.cancelAnimationFrame=id=>{
    callbacks.delete(id);if(!callbacks.size&&pending){cancel(pending);pending=0;}
  };
}
