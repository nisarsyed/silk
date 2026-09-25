import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';

const build=path.resolve(process.argv[2]??'build/render-study');
const server=http.createServer(async(req,res)=>{
  try{
    const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname);
    const target=path.resolve(build,`.${name}`);
    if(!target.startsWith(build+path.sep)){res.writeHead(403).end();return;}
    res.setHeader('Content-Type',target.endsWith('.wasm')?'application/wasm':
      target.endsWith('.html')?'text/html':'text/javascript');
    res.end(await fs.readFile(target));
  }catch{res.writeHead(404).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser;
try{
  browser=await chromium.launch({headless:true});
  const page=await browser.newPage(),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  try{
    await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
    const result=await page.evaluate(async()=>{
      const {startBrowserRuntime}=await import('./runtime_controller.mjs');
      const wait=async test=>{
        const deadline=performance.now()+10000;
        while(!await test()){if(performance.now()>deadline)throw new Error('Controller did not progress');
          await new Promise(resolve=>setTimeout(resolve,20));}
      };
      const mainCanvas=document.querySelector('#main');
      const main=await startBrowserRuntime(mainCanvas,{candidate:'canvas'},{preferWorker:false});
      let duplicate=false,mainRecovery=false,mainPortrait=false;
      try{
        try{await startBrowserRuntime(mainCanvas,{candidate:'canvas'},{preferWorker:false});}
        catch{duplicate=true;}
        await wait(async()=>(await main.report()).steps>=3);
        await main.pause();const held=(await main.report()).steps;
        await new Promise(resolve=>setTimeout(resolve,100));
        if((await main.report()).steps!==held)throw new Error('Main controller did not pause');
        await main.singleStep();
        await wait(async()=>(await main.report()).steps===held+1);
        await main.resume();
        Object.defineProperty(document,'visibilityState',{value:'hidden',configurable:true});
        document.dispatchEvent(new Event('visibilitychange'));
        await wait(async()=>(await main.report()).state==='paused');
        const hiddenSteps=(await main.report()).steps;
        await new Promise(resolve=>setTimeout(resolve,100));
        if((await main.report()).steps!==hiddenSteps)throw new Error('Hidden main world advanced');
        delete document.visibilityState;
        document.dispatchEvent(new Event('visibilitychange'));
        await wait(async()=>(await main.report()).steps>hiddenSteps);
        for(let cycle=0;cycle<3;++cycle){
          Object.defineProperty(document,'visibilityState',{value:'hidden',configurable:true});
          document.dispatchEvent(new Event('visibilitychange'));
          await wait(async()=>(await main.report()).state==='paused');
          delete document.visibilityState;
          document.dispatchEvent(new Event('visibilitychange'));
          await wait(async()=>(await main.report()).state==='running');
        }
        await main.pause();
        Object.defineProperty(document,'visibilityState',{value:'hidden',configurable:true});
        document.dispatchEvent(new Event('visibilitychange'));
        delete document.visibilityState;
        document.dispatchEvent(new Event('visibilitychange'));
        if((await main.report()).state!=='paused')throw new Error('User pause was lost after visibility');
        await main.resize('mobile');
        const portrait=await main.report(),rect=main.canvas.getBoundingClientRect();
        mainPortrait=portrait.bufferWidth===720&&portrait.bufferHeight===1280&&
          rect.width===360&&rect.height===640;
        main.canvas.dispatchEvent(new Event('contextlost',{cancelable:true}));
        if((await main.report()).state!=='failed')throw new Error('Main context failure was ignored');
        await main.recoverToMain();
        mainRecovery=main.mode==='main'&&main.canvas!==mainCanvas&&!mainCanvas.isConnected;
        await wait(async()=>(await main.report()).steps>=3);
      }finally{await main.dispose();}

      const unsupportedCanvas=document.createElement('canvas');unsupportedCanvas.id='unsupported';
      Object.defineProperty(unsupportedCanvas,'transferControlToOffscreen',{value:undefined});
      document.body.append(unsupportedCanvas);
      const unsupported=await startBrowserRuntime(unsupportedCanvas,{candidate:'canvas'});
      const unsupportedFallback=unsupported.mode==='main'&&unsupported.canvas===unsupportedCanvas&&
        /unavailable/.test(unsupported.fallbackReason);
      await unsupported.dispose();

      const workerCanvas=document.querySelector('#worker');
      const worker=await startBrowserRuntime(workerCanvas,{candidate:'webgl'});
      let workerMode,workerSteps,workerPlaceholder,workerCss,workerBuffer;
      try{
        workerMode=worker.mode;
        await wait(async()=>(await worker.report()).steps>=3);
        workerSteps=(await worker.report()).steps;
        await worker.pause();const held=(await worker.report()).steps;
        await new Promise(resolve=>setTimeout(resolve,100));
        if((await worker.report()).steps!==held)throw new Error('Worker controller did not pause');
        await worker.singleStep();
        await wait(async()=>(await worker.report()).steps===held+1);
        await worker.reset('chains',true);
        const reset=await worker.report();
        if(reset.scene!=='chains'||reset.sleep!==true||reset.state!=='paused'||reset.steps!==0)
          throw new Error('Worker reset did not preserve pause');
        await worker.resize('mobile');
        workerPlaceholder=[worker.canvas.width,worker.canvas.height];
        workerCss=[worker.canvas.getBoundingClientRect().width,
          worker.canvas.getBoundingClientRect().height];
        const portrait=await worker.report();
        workerBuffer=[portrait.bufferWidth,portrait.bufferHeight];
        await worker.resume();
        await wait(async()=>(await worker.report()).steps>=3);
        Object.defineProperty(document,'visibilityState',{value:'hidden',configurable:true});
        document.dispatchEvent(new Event('visibilitychange'));
        await wait(async()=>(await worker.report()).state==='paused');
        const hiddenSteps=(await worker.report()).steps;
        await new Promise(resolve=>setTimeout(resolve,100));
        if((await worker.report()).steps!==hiddenSteps)throw new Error('Hidden worker advanced');
        delete document.visibilityState;
        document.dispatchEvent(new Event('visibilitychange'));
        await wait(async()=>(await worker.report()).steps>hiddenSteps);
      }finally{await worker.dispose();}

      const failedCanvas=document.createElement('canvas');failedCanvas.id='failed-start';
      document.body.append(failedCanvas);
      const fallback=await startBrowserRuntime(failedCanvas,{candidate:'canvas'},
        {workerUrl:'./missing-placement-worker.mjs'});
      let startupFallback;
      try{
        await wait(async()=>(await fallback.report()).steps>=3);
        startupFallback={mode:fallback.mode,replaced:fallback.canvas!==failedCanvas,
          oldDetached:!failedCanvas.isConnected,reason:fallback.fallbackReason};
      }finally{await fallback.dispose();}

      const recoveryCanvas=document.createElement('canvas');recoveryCanvas.id='explicit-recovery';
      document.body.append(recoveryCanvas);
      const recovering=await startBrowserRuntime(recoveryCanvas,{candidate:'canvas'});
      let midrun;
      try{
        await wait(async()=>(await recovering.report()).steps>=3);
        const before=await recovering.report();
        let rejected=false;
        try{await recovering.pointer(before.epoch,1,99,3,0,0);}catch{rejected=true;}
        if(!rejected||recovering.mode!=='failed')throw new Error('Invalid worker action did not fail placement');
        const restored=await recovering.recoverToMain();
        await wait(async()=>(await recovering.report()).steps>=3);
        midrun={rejected,mode:recovering.mode,replaced:recovering.canvas!==recoveryCanvas,
          oldDetached:!recoveryCanvas.isConnected,restoredScene:restored.scene,
          restoredSteps:restored.steps,reason:recovering.fallbackReason};
      }finally{await recovering.dispose();}
      const saturation=[];
      for(const preferWorker of [false,true]){
        const canvas=document.createElement('canvas');document.body.append(canvas);
        const runtime=await startBrowserRuntime(canvas,{candidate:'canvas'},{preferWorker});
        try{
          await runtime.pause();const epoch=runtime.epoch;
          const batch=Array.from({length:256},(_,index)=>
            [epoch,index+1,index%2?2:0,7,0,0]);
          const result=await runtime.pointerBatch(batch),full=await runtime.report();
          let rejected=false;
          try{await runtime.pointerBatch([[epoch,257,3,7,0,0]]);}catch{rejected=true;}
          const failed=runtime.mode==='main'?(await runtime.report()).reason:
            runtime.failureReason;
          await runtime.recoverToMain();
          saturation.push({mode:preferWorker?'worker':'main',result,full:full.input.count,
            rejected,failed,recovered:runtime.mode==='main'&&runtime.canvas!==canvas});
        }finally{await runtime.dispose();}
      }
      return {duplicate,mainRecovery,mainPortrait,unsupportedFallback,workerMode,workerSteps,
        workerPlaceholder,workerCss,workerBuffer,startupFallback,midrun,saturation};
    });
    assert.equal(result.duplicate,true);
    assert.equal(result.mainRecovery,true);
    assert.equal(result.mainPortrait,true);
    assert.equal(result.unsupportedFallback,true);
    assert.equal(result.workerMode,'worker');
    assert.ok(result.workerSteps>=3);
    assert.deepEqual(result.workerPlaceholder,[1280,720]);
    assert.deepEqual(result.workerCss,[360,640]);
    assert.deepEqual(result.workerBuffer,[720,1280]);
    assert.equal(result.startupFallback.mode,'main');
    assert.equal(result.startupFallback.replaced,true);
    assert.equal(result.startupFallback.oldDetached,true);
    assert.match(result.startupFallback.reason,/Worker startup failed/);
    assert.equal(result.midrun.mode,'main');
    assert.equal(result.midrun.replaced,true);
    assert.equal(result.midrun.oldDetached,true);
    assert.equal(result.midrun.restoredScene,'pyramid');
    assert.ok(result.midrun.restoredSteps<=1);
    assert.match(result.midrun.reason,/Explicit recovery restarted/);
    for(const entry of result.saturation){
      assert.equal(entry.result.queued,256);
      assert.equal(entry.full,256);
      assert.equal(entry.rejected,true);
      assert.match(entry.failed,/Pointer queue capacity exhausted/);
      assert.equal(entry.recovered,true);
    }
    assert.deepEqual(errors,[]);
    console.log('Browser controller main/worker ownership, startup fallback, explicit recovery and shutdown PASS');
  }finally{await page.close();}
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
