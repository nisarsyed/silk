import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';
const build=path.resolve(process.argv[2]??'build/render-study');
const output=path.resolve(process.argv[3]??'build/reports/render-correctness');
await fs.mkdir(output,{recursive:true});
const server=http.createServer(async (req,res)=>{
  try {
    const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname);
    const target=path.resolve(build,`.${name}`);
    if (!target.startsWith(build+path.sep)) { res.writeHead(403).end(); return; }
    res.setHeader('Content-Type',target.endsWith('.wasm')?'application/wasm':target.endsWith('.html')?'text/html':'text/javascript');
    res.end(await fs.readFile(target));
  } catch { res.writeHead(404).end(); }
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser, harnessError=null, primitive=null, raylibValidation=null;
const results=[];
const colocated=[];
try {
  browser=await chromium.launch({headless:true});
  const probe=await browser.newPage();
  try{
    await probe.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
    await probe.waitForFunction(()=>typeof window.verifyOverlayPrimitives==='function');
    primitive=await probe.evaluate(()=>window.verifyOverlayPrimitives());
    raylibValidation=await probe.evaluate(()=>window.verifyRaylibValidation());
    assert.equal(raylibValidation.rejected,13);
    for(const result of primitive){
      assert.deepEqual(result.line,[239,119,119]);assert.deepEqual(result.marker,[255,204,102]);
      assert.deepEqual(result.above,[16,24,32]);assert.deepEqual(result.below,[16,24,32]);assert.deepEqual(result.outside,[16,24,32]);
    }
  }finally{await probe.close();}
  for(const [width,height]of [[1280,720],[720,1280]])for(const scene of ['pyramid','rain','chains'])for(const copies of [1,16])for(const sleep of [false,true]){
    const page=await browser.newPage({viewport:{width:width===720?360:width,height:height===1280?640:height},deviceScaleFactor:width===720?2:1}),errors=[];
    const entry={scene,copies,sleep,width,height,status:'running'};colocated.push(entry);
    page.on('pageerror',error=>errors.push(String(error)));
    page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('favicon'))errors.push(message.text());});
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
      await page.waitForFunction(()=>typeof window.verifyColocated==='function');
      Object.assign(entry,await page.evaluate(profile=>window.verifyColocated(profile),{scene,copies,sleep,width,height}));
      assert.deepEqual(errors,[]);for(const result of entry.results)assert.equal(result.interiorMismatch,0);
      entry.status='passed';console.log(JSON.stringify({colocated:entry}));
    }catch(error){entry.status='failed';entry.error=String(error);throw error;}
    finally{await page.close();}
  }
  for (const [width,height] of [[1280,720],[720,1280]]) {
    const profiles=[];
    for (const scene of ['pyramid','rain','chains']) for (const steps of [0,120]) profiles.push({scene,steps,width,height});
    for(const scene of ['pyramid','rain','chains']) for(const sleep of [false,true]) profiles.push({scene,steps:120,diagnostic:true,sleep,width,height});
    for(const scene of ['pyramid','rain','chains']) profiles.push({scene,steps:3,copies:16,width,height});
    for (const instances of [256,65536]) profiles.push({scene:'rain',steps:120,instances,width,height});
    for (const profile of profiles) {
      const page=await browser.newPage({viewport:{width:width===720?360:width,height:height===1280?640:height},deviceScaleFactor:width===720?2:1}), errors=[];
      const entry={profile,status:'running'};results.push(entry);
      page.on('pageerror',error=>errors.push(String(error)));
      page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('favicon')) errors.push(message.text());});
      try {
        await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
        await page.waitForFunction(()=>typeof window.verifyRenderers==='function');
        const result=await page.evaluate(profile=>window.verifyRenderers(profile),{...profile,images:true});
        for (let i=0;i<result.previews.length;++i) await fs.writeFile(path.join(output,`${profile.scene}-${profile.steps}-${profile.diagnostic?"diagnostic":"base"}-${profile.sleep?"sleep":"awake"}-${profile.copies??1}copies-${profile.instances??"default"}-${width}x${height}-${["canvas","webgl","raylib"][i]}.png`),Buffer.from(result.previews[i].split(",")[1],"base64"));
        delete result.previews;
        result.errors=errors; Object.assign(entry,result); console.log(JSON.stringify(result));
        assert.deepEqual(errors,[]);
        assert.equal(result.webgl.interiorMismatch,0,'WebGL pixels differ outside the allowed one-pixel edge');
        assert.equal(result.raylib.interiorMismatch,0,'raylib pixels differ outside the allowed one-pixel edge');
        if(result.updated) {assert.equal(result.updated.webgl.interiorMismatch,0);assert.equal(result.updated.raylib.interiorMismatch,0);}
        entry.status='passed';
      } catch(error) {entry.status='failed';entry.error=String(error);throw error;}
      finally { await page.close(); }
    }
  }
} catch(error) {harnessError=String(error);throw error;}
finally {
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({browser:browser?.version()??null,kind:'correctness-only',harnessError,primitive,raylibValidation,colocated,results},null,2)+'\n');
  await browser?.close(); await new Promise(resolve=>server.close(resolve));
}
