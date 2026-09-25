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
  for(const candidate of ['canvas','webgl'])for(const preferWorker of [false,true]){
    const page=await browser.newPage({viewport:{width:1400,height:900}}),errors=[];
    const waitHeld=async expected=>{
      const deadline=Date.now()+10000;
      for(;;){
        if(await page.evaluate(async expected=>
          (await window.__study.controller.report()).pointerHeld===expected,expected))return;
        if(Date.now()>deadline)throw new Error(`Pointer held state did not become ${expected}`);
        await new Promise(resolve=>setTimeout(resolve,20));
      }
    };
    page.on('pageerror',error=>errors.push(String(error)));
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
      const initial=await page.evaluate(async({candidate,preferWorker})=>{
        const {startBrowserRuntime}=await import('./runtime_controller.mjs');
        const {createPointerAdapter}=await import('./pointer_adapter.mjs');
        const canvas=document.createElement('canvas');canvas.id='input';
        document.body.prepend(canvas);
        const controller=await startBrowserRuntime(canvas,{candidate},{preferWorker});
        const epoch=controller.epoch;
        let rejected=false;
        try{controller.pointerBatch([[epoch,1,0,1,0,0],[epoch,2,99,1,0,0]]);}
        catch{rejected=true;}
        const adapter=createPointerAdapter(controller);
        window.__study={controller,adapter};
        return {mode:controller.mode,rejected,held:(await controller.report()).pointerHeld};
      },{candidate,preferWorker});
      assert.equal(initial.mode,preferWorker?'worker':'main');
      assert.equal(initial.rejected,true);
      assert.equal(initial.held,false);
      const box=await page.locator('#input').boundingBox();
      await page.mouse.move(box.x+box.width/2,box.y+box.height/2);
      await page.mouse.down();
      await waitHeld(true);
      await page.mouse.move(box.x+box.width/2+25,box.y+box.height/2+10,{steps:4});
      await page.mouse.up();
      await waitHeld(false);

      await page.mouse.down();
      await waitHeld(true);
      const paused=await page.evaluate(async()=>{
        const {controller}=window.__study,before=controller.epoch;
        await controller.pause();
        return {before,after:controller.epoch,held:(await controller.report()).pointerHeld};
      });
      assert.equal(paused.after,paused.before+1);
      assert.equal(paused.held,false);
      await page.mouse.up();
      await page.evaluate(async()=>{
        const {controller}=window.__study;
        await controller.resize('mobile');await controller.reset('chains',true);await controller.resume();
      });
      const mobile=await page.locator('#input').boundingBox();
      assert.deepEqual([mobile.width,mobile.height],[360,640]);
      await page.mouse.move(mobile.x+mobile.width/2,mobile.y+mobile.height/2);
      await page.mouse.down();
      await waitHeld(true);
      await page.mouse.up();
      await waitHeld(false);
      const recovered=await page.evaluate(async()=>{
        const {controller}=window.__study,original=controller.canvas;
        if(controller.mode==='worker'){
          try{await controller.pointer(controller.epoch,999,99,3,0,0);}catch{}
        }else original.dispatchEvent(new Event('contextlost',{cancelable:true}));
        await controller.recoverToMain();
        return controller.canvas!==original&&!original.isConnected;
      });
      assert.equal(recovered,true);
      const replacement=await page.locator('#input').boundingBox();
      await page.mouse.move(replacement.x+replacement.width/2,replacement.y+replacement.height/2);
      await page.mouse.down();
      await waitHeld(true);
      await page.mouse.up();
      await waitHeld(false);
      const final=await page.evaluate(async()=>{
        const {controller,adapter}=window.__study;
        const report=await controller.report(),statistics=adapter.statistics;
        adapter.dispose();await controller.dispose();
        return {report,statistics,failure:adapter.failure};
      });
      assert.equal(final.failure,null);
      assert.equal(final.report.scene,'pyramid');
      assert.equal(final.report.layout,'desktop');
      assert.ok(final.report.steps>=1,JSON.stringify(final.report));
      assert.ok(final.statistics.capacity===256);
      assert.deepEqual(errors,[]);
      console.log(`${candidate} ${initial.mode} trusted DOM input, bounded batching, epoch cancellation and mobile mapping PASS`);
    }finally{await page.close();}
  }
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
