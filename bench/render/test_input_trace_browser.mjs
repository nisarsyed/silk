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
    page.on('pageerror',error=>errors.push(String(error)));
    const waitHeld=async expected=>{
      const deadline=Date.now()+10000;
      for(;;){
        if(await page.evaluate(async expected=>
          (await window.__study.controller.report()).pointerHeld===expected,expected))return;
        if(Date.now()>deadline)throw new Error(`Pointer held state did not become ${expected}`);
        await new Promise(resolve=>setTimeout(resolve,20));
      }
    };
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
      const mode=await page.evaluate(async({candidate,preferWorker})=>{
        const {startBrowserRuntime}=await import('./runtime_controller.mjs');
        const {createPointerAdapter}=await import('./pointer_adapter.mjs');
        const canvas=document.createElement('canvas');canvas.id='trace';document.body.prepend(canvas);
        const controller=await startBrowserRuntime(canvas,{candidate,traceInput:true},{preferWorker});
        const adapter=createPointerAdapter(controller);
        window.__study={controller,adapter};return controller.mode;
      },{candidate,preferWorker});
      assert.equal(mode,preferWorker?'worker':'main');
      const box=await page.locator('#trace').boundingBox();
      await page.mouse.move(box.x+box.width/2,box.y+box.height/2);
      await page.mouse.down();await waitHeld(true);
      await page.mouse.move(box.x+box.width/2+20,box.y+box.height/2+5,{steps:3});
      await page.mouse.up();await waitHeld(false);
      const result=await page.evaluate(async()=>{
        const {controller,adapter}=window.__study;
        const report=await controller.report(),trace=await controller.inputTrace();
        const rows=Array.from({length:trace.count},(_,i)=>({
          epoch:trace.epoch[i],sequence:trace.sequence[i],action:trace.action[i],
          eventMs:trace.eventMs[i],appliedMs:trace.appliedMs[i],
          submittedMs:trace.submittedMs[i]}));
        adapter.dispose();await controller.dispose();
        return {reportCount:report.inputTraceCount,reportStorageBytes:report.inputTraceStorageBytes,
          storageBytes:trace.storageBytes,capacity:trace.capacity,count:trace.count,
          mainTimeOrigin:trace.mainTimeOrigin,ownerTimeOrigin:trace.timeOrigin,rows};
      });
      assert.equal(result.reportCount,result.count);
      assert.equal(result.reportStorageBytes,result.storageBytes);
      assert.equal(result.storageBytes,41*result.capacity);
      assert.ok(result.count>=2&&result.count<=result.capacity);
      assert.ok(Number.isFinite(result.mainTimeOrigin)&&Number.isFinite(result.ownerTimeOrigin));
      assert.equal(result.rows[0].action,0);
      assert.equal(result.rows.at(-1).action,2);
      for(let i=0;i<result.rows.length;++i){
        const row=result.rows[i];
        assert.ok(row.epoch>=1&&row.sequence>=1);
        assert.ok(row.eventMs>0&&row.eventMs<=row.appliedMs&&
          row.appliedMs<=row.submittedMs);
        if(i)assert.ok(row.sequence>result.rows[i-1].sequence);
      }
      assert.deepEqual(errors,[]);
      console.log(`${candidate} ${mode} trusted input-to-step-to-submission trace PASS`);
    }finally{await page.close();}
  }
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
