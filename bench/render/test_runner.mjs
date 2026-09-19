import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';
const build=path.resolve(process.argv[2]??'build/render-study');
const output=path.resolve(process.argv[3]??'build/reports/render-runner');
const provenance=JSON.parse(await fs.readFile(path.join(build,'provenance.json'),'utf8'));
const channel=process.env.SL_RENDER_BROWSER,headed=process.env.SL_RENDER_HEADED==='1';
if(channel!==undefined&&channel!=='chrome')throw new Error('SL_RENDER_BROWSER must be chrome or unset');
if(process.env.SL_RENDER_HEADED!==undefined&&!['0','1'].includes(process.env.SL_RENDER_HEADED))throw new Error('SL_RENDER_HEADED must be 0 or 1');
await fs.mkdir(output,{recursive:true});
const server=http.createServer(async(req,res)=>{
  try{
    const target=path.resolve(build,'.'+decodeURIComponent(new URL(req.url,'http://localhost').pathname));
    if(!target.startsWith(build+path.sep)){res.writeHead(403).end();return;}
    res.setHeader('Content-Type',target.endsWith('.wasm')?'application/wasm':target.endsWith('.html')?'text/html':'text/javascript');
    res.end(await fs.readFile(target));
  }catch{res.writeHead(404).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser;const results=[];
try{
  browser=await chromium.launch({headless:!headed,...(channel?{channel}:{})});
  const page=await browser.newPage({viewport:headed?null:{width:1280,height:900}}),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('favicon'))errors.push(message.text());});
  await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
  for(const candidate of ['canvas','webgl','raylib'])for(const diagnostic of [false,true]){
    const result=await page.evaluate(async({candidate,diagnostic})=>{
      const {runStudy}=await import('./runner.mjs');
      const canvas=document.createElement('canvas'),hudElement=document.createElement('pre');
      canvas.id='runner-test';document.body.append(canvas,hudElement);const phases=[];
      try{return JSON.parse(JSON.stringify(await runStudy({canvas,hudElement,candidate,diagnostic,correctnessSeconds:.25,
        layout:diagnostic?'mobile':'desktop',onPhase:value=>phases.push(value)}),(_key,value)=>typeof value==='bigint'?String(value):value));}
      finally{canvas.remove();hudElement.remove();}
    },{candidate,diagnostic});
    await fs.writeFile(path.join(output,`${candidate}-${diagnostic?'diagnostic':'base'}.json`),JSON.stringify(result)+'\n');
    assert.equal(result.status,'collected',JSON.stringify(result.failure));assert.equal(result.kind,'correctness-only');
    assert.equal(result.acceptanceEligible,false);assert.equal(result.warmup.callbacks,120);assert.equal(result.warmup.world.steps,120);
    assert.equal(result.rendererRecreatedAfterWarmup,false);
    assert.deepEqual(result.provenance,provenance);
    assert.equal(result.initial.steps,0);assert.equal(result.initial.drops,'0');assert.equal(result.final.finiteState,true);
    assert.ok(result.elapsedSeconds>=.25);assert.ok(result.frames.count>=2);assert.equal(result.frames.pendingFrame,false);
    assert.equal(result.summary.completeFrames,result.frames.count);assert.equal(result.calibrationIntervals.length,240);
    assert.equal(result.windows.length,1);assert.equal(result.windows[0].coverageReached,true);
    assert.equal(result.windows[0].completeFrames,result.frames.count);assert.equal(result.windows[0].executedSteps,result.final.steps);
    assert.equal(result.windows[0].contactDrops,result.final.drops);assert.equal(result.windows[0].endDebtSeconds,result.debtSeconds);
    const f=result.frames,n=f.numberNames.length,stepIndex=f.statNames.indexOf('completedSteps');
    assert.equal(result.gpu.pending,0);assert.equal(result.gpu.attempts,f.count);
    assert.equal(result.gpu.counts.reduce((a,b)=>a+b,0),f.count);
    if(candidate==='canvas')assert.equal(result.glCalls,null);
    else{
      assert.equal(result.glCalls.valid,true);assert.equal(result.glCalls.sections,f.count);
      let uploads=0,draws=0;
      for(let row=0;row<f.count;++row){uploads+=f.numbers[row*n+15];draws+=f.numbers[row*n+21];
        assert.equal(f.known[row]&(1<<15),1<<15);assert.equal(f.known[row]&(1<<21),1<<21);assert.ok(f.numbers[row*n+21]>0);}
      assert.equal(result.glCalls.totalUploadBytes,uploads);assert.equal(result.glCalls.totalDrawCalls,draws);
      if(candidate==='raylib')assert.ok(uploads>0);
    }
    assert.equal(f.stats[stepIndex],0);assert.equal(f.stats[(f.count-1)*25+stepIndex],result.final.steps);
    for(let row=0;row<f.count;++row){
      assert.equal(f.finished[row],1);assert.ok(f.numbers[row*n+4]>=f.numbers[row*n+3]);
      assert.ok(f.numbers[row*n+12]>=0&&f.numbers[row*n+12]<Math.fround(1/60));
      assert.equal(f.known[row]&(1<<14),result.gpu.statuses[row]===2?1<<14:0);
      assert.equal(f.known[row]&(1<<23),1<<23);assert.ok(f.numbers[row*n+23]>=0);
      if(!result.gpu.supported)assert.equal(result.gpu.statuses[row],6);
    }
    assert.ok(Math.abs(result.final.steps*Math.fround(1/60)+result.debtSeconds+result.droppedSeconds-result.elapsedSeconds)<1e-9);
    if(candidate==='raylib')for(let row=0;row<f.count;++row){assert.equal(f.numbers[row*n+16],0);assert.equal(f.numbers[row*n+18],0);}
    results.push({candidate,diagnostic,status:result.status,frames:f.count,gpu:result.gpu});
  }
  // Interrupt only after measurement begins, so the failed record must retain
  // its prefix. These are injected correctness events, not device evidence.
  for(const reason of ['abort','context','hidden']){
    const result=await page.evaluate(async reason=>{
      const {runStudy}=await import('./runner.mjs');const canvas=document.createElement('canvas');
      canvas.id='runner-failure';document.body.append(canvas);const controller=new AbortController();
      try{
        const report=await runStudy({canvas,candidate:'webgl',correctnessSeconds:1,signal:controller.signal,
          onPhase:phase=>{if(phase==='measurement')requestAnimationFrame(()=>requestAnimationFrame(()=>{
            if(reason==='abort')controller.abort();
            else if(reason==='context')canvas.dispatchEvent(new Event('webglcontextlost',{cancelable:true}));
            else{Object.defineProperty(document,'visibilityState',{value:'hidden',configurable:true});document.dispatchEvent(new Event('visibilitychange'));}
          }));}});
        if(reason==='hidden')delete document.visibilityState;
        const replacement=await runStudy({canvas,candidate:'webgl',signal:AbortSignal.abort(),correctnessSeconds:.25});
        return JSON.parse(JSON.stringify({report,replacement},(_key,value)=>typeof value==='bigint'?String(value):value));
      }finally{if(reason==='hidden')delete document.visibilityState;canvas.remove();}
    },reason);
    await fs.writeFile(path.join(output,`interrupted-${reason}.json`),JSON.stringify(result)+'\n');
    assert.equal(result.report.status,'failed');assert.equal(result.report.failure.phase,'measurement');
    assert.deepEqual(result.report.provenance,provenance);assert.deepEqual(result.replacement.provenance,provenance);
    assert.match(result.report.failure.reason,reason==='abort'?/aborted/:reason==='context'?/context lost/:/hidden/);
    assert.ok(result.report.frames.count>0);assert.ok(result.report.final);
    assert.equal(result.replacement.status,'failed');assert.match(result.replacement.failure.reason,/aborted/);
    assert.equal(result.replacement.calibrationIntervals.length,0);assert.equal(result.replacement.setupMs,null);
  }
  assert.deepEqual(errors,[]);
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({kind:'correctness-only',browser:browser.version(),
    channel:channel??'bundled-chromium',headless:!headed,results},null,2)+'\n');
  console.log('Browser runner: all candidates/profiles, exact rebuild, bounded records, fixed-step accounting, partial failures and disposal PASS');
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
