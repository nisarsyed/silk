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
    const rotations=[['canvas','webgl','raylib'],['webgl','raylib','canvas'],
      ['raylib','canvas','webgl'],['canvas','raylib','webgl'],['raylib','webgl','canvas']];
    const choices=[
      {profile:'interaction',candidate:'canvas',layout:'desktop',scene:'pyramid',copies:1,sleep:false,repeat:1},
      {profile:'interaction',candidate:'webgl',layout:'desktop',scene:'pyramid',copies:1,sleep:false,repeat:2},
      {profile:'interaction',candidate:'canvas',layout:'mobile',scene:'pyramid',copies:1,sleep:false,repeat:3},
      {profile:'base',candidate:'webgl',layout:'desktop',scene:'rain',copies:2,sleep:true,repeat:4,memoryMiB:128},
      {profile:'diagnostic',candidate:'raylib',layout:'desktop',scene:'chains',copies:1,sleep:false,repeat:5},
      {profile:'frozen',candidate:'canvas',layout:'desktop',scene:'rain',copies:1,sleep:false,repeat:1,instances:256},
      {profile:'frozen-diagnostic',candidate:'webgl',layout:'mobile',scene:'rain',copies:1,sleep:false,repeat:2,instances:256}
    ];
    for(const choice of choices){
      await page.setViewportSize({width:choice.layout==='mobile'?360:1280,height:choice.layout==='mobile'?800:900});
      await page.locator('#profile').selectOption(choice.profile);
      await page.locator('#candidate').selectOption(choice.candidate);
      await page.locator('#repeat').selectOption(String(choice.repeat));
      if(await page.locator('#scene').isEnabled())await page.locator('#scene').selectOption(choice.scene);
      await page.locator('#layout').selectOption(choice.layout);
      if(await page.locator('#copies').isEnabled())await page.locator('#copies').selectOption(String(choice.copies));
      if(choice.instances)await page.locator('#instances').selectOption(String(choice.instances));
      await page.locator('#memory').selectOption(String(choice.memoryMiB??64));
      if(choice.sleep)await page.locator('#sleep').check();else if(await page.locator('#sleep').isEnabled())await page.locator('#sleep').uncheck();
      await page.locator('#start').click();
      assert.equal(await page.locator('#profile').isDisabled(),true);
      if(choice.profile==='interaction'&&choice.layout==='mobile'){
        await page.waitForFunction(()=>document.getElementById('phase').textContent==='DRAG NOW',undefined,{timeout:60000});
        const visible=await page.evaluate(()=>{const rect=document.getElementById('collection-canvas').getBoundingClientRect();return {top:rect.top,bottom:rect.bottom,height:innerHeight};});
        assert.ok(visible.top>=0&&visible.bottom<=visible.height,'Mobile interaction canvas must fit during measurement');
        await page.screenshot({path:path.join(output,'collector-mobile-measuring.png')});
      }
      await page.waitForFunction(()=>!document.getElementById('download').disabled,undefined,{timeout:60000});
      if(choice.profile==='interaction'&&choice.candidate==='canvas')await page.screenshot({path:path.join(output,`collector-${choice.layout}.png`),fullPage:true});
      const arriving=page.waitForEvent('download');await page.locator('#download').click();
      const transfer=await arriving;
      const report=JSON.parse(await fs.readFile(await transfer.path(),'utf8'));
      assert.equal(report.status,'collected',JSON.stringify(report.failure));
      assert.equal(report.kind,'correctness-only');
      assert.equal(report.configuration.candidate,choice.candidate);
      assert.equal(report.configuration.layout,choice.layout);
      assert.equal(report.configuration.scene,choice.scene);
      assert.equal(report.configuration.copies,choice.copies);
      assert.equal(report.configuration.sleep,choice.sleep);
      assert.equal(report.configuration.diagnostic,['diagnostic','frozen-diagnostic'].includes(choice.profile));
      assert.equal(report.configuration.interaction,choice.profile==='interaction');
      assert.equal(report.configuration.instances,choice.instances??null);
      assert.equal(report.configuration.memoryBytes,(choice.memoryMiB??64)*1024*1024);
      assert.equal(report.collectionConditions.deviceLabel,'reference desktop');
      assert.equal(report.collectionConditions.power,'plugged');
      assert.equal(report.collectionConditions.source,'operator-entered');
      assert.equal(report.collectionConditions.profile,choice.profile);
      assert.equal(report.collectionConditions.repeat,choice.repeat);
      assert.deepEqual(report.collectionConditions.candidateOrder,rotations[choice.repeat-1]);
      assert.equal(report.collectionConditions.candidateOrder[report.collectionConditions.candidateSlot-1],choice.candidate);
      if(choice.instances)assert.equal(report.initial.steps,120);
      if(choice.profile==='interaction'){
        assert.equal(report.input.count,0);assert.equal(report.input.typedBytes,95*65536);
      }else assert.equal(report.input,null);
      assert.equal(await page.locator('#start').isEnabled(),true);
      assert.equal(await page.locator('#profile').isEnabled(),true);
      results.push({...choice,operatorPage:'downloaded'});
    }
    await page.locator('#profile').selectOption('sustained');
    assert.equal(await page.locator('#scene').inputValue(),'rain');
    assert.equal(await page.locator('#layout').inputValue(),'mobile');
    assert.equal(await page.locator('#copies').inputValue(),'1');
    assert.equal(await page.locator('#sleep').isDisabled(),true);
    assert.deepEqual(errors,[]);
  }finally{await page.close();}
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({kind:'correctness-only',
    clock:testClock==='synthetic'?'deterministic-rAF-60Hz':'actual-headless-rAF',browser:browser.version(),results},null,2)+'\n');
  console.log('Real PointerEvent smoke: three candidates, fixed-step-to-submission trace, trusted browser events and operator downloads PASS');
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
