import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';
import {installSyntheticClock} from './test_clock.mjs';

const build=path.resolve(process.argv[2]??'build/render-study');
const output=path.resolve(process.argv[3]??'build/reports/render-input');
const testClock=process.env.SL_RENDER_TEST_CLOCK??'synthetic';
if(!['real','synthetic'].includes(testClock))throw new Error('Invalid test clock');
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
  browser=await chromium.launch({headless:true});
  for(const candidate of ['canvas','webgl','raylib']){
    const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
    page.on('pageerror',error=>errors.push(String(error)));
    page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('favicon'))errors.push(message.text());});
    try{
      if(testClock==='synthetic')await page.addInitScript(installSyntheticClock);
      await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
      await page.evaluate(candidate=>{
        const canvas=document.createElement('canvas');canvas.id='input-test';document.body.append(canvas);
        window.__silkInputPhase='initialization';
        window.__silkInputRun=import('./runner.mjs').then(({runStudy})=>runStudy({canvas,candidate,
          scene:'pyramid',interaction:true,correctnessSeconds:3,
          onPhase:phase=>{window.__silkInputPhase=phase;}}));
      },candidate);
      await page.waitForFunction(()=>window.__silkInputPhase==='measurement');
      const p=await page.evaluate(()=>{
        const canvas=document.getElementById('input-test'),rect=canvas.getBoundingClientRect();
        // Pyramid's first dynamic box starts at (-9.595, 0.5), but the top
        // box at (0, 19.69) remains isolated at reset and is easy to grab.
        const scale=Math.min(canvas.width/26,canvas.height/23);
        const cameraX=canvas.width/2,cameraY=canvas.height/2+scale*10.5;
        return {x:rect.left+cameraX,y:rect.top+cameraY-scale*19.69};
      });
      await page.mouse.move(p.x,p.y);await page.mouse.down();
      for(let i=1;i<=8;++i){await page.mouse.move(p.x+i*2,p.y+i*2);await page.waitForTimeout(25);}
      await page.mouse.up();
      const report=await page.evaluate(()=>window.__silkInputRun.then(value=>JSON.parse(JSON.stringify(value,
        (_key,item)=>typeof item==='bigint'?String(item):item))));
      if(testClock==='synthetic')report.clock.testOverride='deterministic-rAF-60Hz';
      await fs.writeFile(path.join(output,`${candidate}.json`),JSON.stringify(report)+'\n');
      assert.equal(report.status,'collected',JSON.stringify(report.failure));
      assert.equal(report.configuration.interaction,true);
      assert.equal(report.initial.steps,0);
      assert.ok(report.input.count>=10);
      assert.equal(report.input.timestampPrecisionStatus,'unverified');
      assert.equal(report.input.latencyGateEvaluable,false);
      const down=report.input.events.find(event=>event.kind==='down'&&event.status==='accepted');
      assert.ok(down?.trusted&&down.bodyIndex!==null,'Trusted press must select the same dynamic box');
      assert.ok(report.input.events.some(event=>event.kind==='up'&&event.status==='accepted'));
      assert.ok(report.input.trustedSubmittedMoves>=1,'At least one real browser move must reach a submitted frame');
      if(testClock==='real')assert.equal(report.input.validTimestampSamples,report.input.trustedSubmittedMoves);
      for(const event of report.input.events.filter(event=>event.status==='submitted')){
        assert.equal(event.kind,'move');assert.ok(event.appliedStep>0);assert.ok(event.submittedFrame>0);
        assert.ok(event.submittedMs>=report.measurementStart);
      }
      assert.deepEqual(errors,[]);
      results.push({candidate,events:report.input.count,submitted:report.input.trustedSubmittedMoves});
    }finally{await page.close();}
  }
  // The operator page must use a fresh canvas for each candidate, preserve
  // entered conditions outside measured frames, and offer failed/successful
  // records as downloads rather than discarding a run on navigation.
  const page=await browser.newPage({viewport:{width:1280,height:900},acceptDownloads:true}),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  try{
    await page.addInitScript(installSyntheticClock);
    await page.goto(`http://127.0.0.1:${server.address().port}/collect.html?smoke=1`);
    await page.locator('#device').fill('reference desktop');
    await page.locator('#power').selectOption('plugged');
    for(const [candidate,layout] of [['canvas','desktop'],['webgl','desktop'],['canvas','mobile']]){
      if(layout==='mobile')await page.setViewportSize({width:360,height:800});
      await page.locator('#candidate').selectOption(candidate);
      await page.locator('#layout').selectOption(layout);
      await page.locator('#start').click();
      await page.waitForFunction(()=>!document.getElementById('download').disabled,undefined,{timeout:60000});
      if(candidate==='canvas')await page.screenshot({path:path.join(output,`collector-${layout}.png`),fullPage:true});
      const arriving=page.waitForEvent('download');await page.locator('#download').click();
      const transfer=await arriving;
      const report=JSON.parse(await fs.readFile(await transfer.path(),'utf8'));
      assert.equal(report.status,'collected',JSON.stringify(report.failure));
      assert.equal(report.kind,'correctness-only');
      assert.equal(report.configuration.candidate,candidate);
      assert.equal(report.configuration.layout,layout);
      assert.equal(report.configuration.interaction,true);
      assert.equal(report.collectionConditions.deviceLabel,'reference desktop');
      assert.equal(report.collectionConditions.power,'plugged');
      assert.equal(report.collectionConditions.source,'operator-entered');
      assert.equal(report.input.count,0);
      assert.equal(report.input.typedBytes,95*65536);
      assert.equal(await page.locator('#start').isEnabled(),true);
      results.push({candidate,layout,operatorPage:'downloaded'});
    }
    assert.deepEqual(errors,[]);
  }finally{await page.close();}
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({kind:'correctness-only',
    clock:testClock==='synthetic'?'deterministic-rAF-60Hz':'actual-headless-rAF',browser:browser.version(),results},null,2)+'\n');
  console.log('Real PointerEvent smoke: three candidates, fixed-step-to-submission trace, trusted browser events and operator downloads PASS');
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
