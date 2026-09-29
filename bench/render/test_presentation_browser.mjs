import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {chromium,firefox,webkit} from '../../wasm/node_modules/@playwright/test/index.mjs';

const build=path.resolve(process.argv[2]??'build/render-study');
const engineName=process.argv[3]??'chromium';
const engine={chromium,firefox,webkit}[engineName];
if(!engine)throw new Error('Unknown browser engine');
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
  browser=await engine.launch({headless:true});
  const page=await browser.newPage(),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  try{
    await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
    await page.evaluate(async()=>{
      // Synthetic rAF timestamps isolate pacing from the host display. Real
      // performance.now still drives C's fixed-step scheduler.
      let rafMs=0;
      window.requestAnimationFrame=callback=>setTimeout(()=>{
        rafMs+=1000/120;callback(rafMs);
      },0);
      window.cancelAnimationFrame=clearTimeout;
      const {createRuntimeOwner}=await import('./runtime_owner.mjs');
      window.__runtime=await createRuntimeOwner(document.querySelector('#main'),
        {candidate:'canvas',traceFrames:true});
    });
    const deadline=Date.now()+20000;
    for(;;){
      const report=await page.evaluate(()=>window.__runtime.report());
      if(report.state==='failed')throw new Error(`Runtime failed: ${report.reason}`);
      if(report.presentation.skippedCallbacks>=60)break;
      if(Date.now()>deadline)throw new Error(`Presentation did not pace: ${JSON.stringify(report)}`);
      await new Promise(resolve=>setTimeout(resolve,20));
    }
    const result=await page.evaluate(()=>{
      const owner=window.__runtime,report=owner.report(),trace=owner.frameTrace();
      owner.dispose();
      return {report,frameCount:trace.count,
        executedSteps:Array.from(trace.steps).reduce((sum,steps)=>sum+steps,0)};
    });
    const timing=result.report.presentation;
    assert.equal(result.report.state,'running',
      JSON.stringify(result.report,(_key,value)=>typeof value==='bigint'?String(value):value));
    assert.equal(timing.calibrated,true);
    assert.equal(timing.calibratedIntervals,240);
    assert.equal(timing.divisor,2);
    assert.equal(timing.targetSupported,true);
    assert.ok(Math.abs(timing.targetMs-1000/60)<1e-8);
    assert.ok(timing.skippedCallbacks>=60);
    assert.equal(result.frameCount,timing.submittedFrames);
    assert.equal(result.executedSteps,result.report.steps);
    assert.deepEqual(errors,[]);
    console.log(`${engineName} real WASM owner: calibrated 120-to-60 Hz presentation and complete step trace PASS`);
  }finally{await page.close();}
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
