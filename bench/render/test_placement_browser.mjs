import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';

const build=path.resolve(process.argv[2]??'build/render-study');
const server=http.createServer(async(req,res)=>{
  try {
    const name=decodeURIComponent(new URL(req.url,'http://localhost').pathname);
    const target=path.resolve(build,`.${name}`);
    if (!target.startsWith(build+path.sep)) {res.writeHead(403).end();return;}
    res.setHeader('Content-Type',target.endsWith('.wasm')?'application/wasm':
      target.endsWith('.html')?'text/html':'text/javascript');
    res.end(await fs.readFile(target));
  } catch {res.writeHead(404).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser;
try {
  browser=await chromium.launch({headless:true});
  for(const layout of ['desktop','mobile'])for(const candidate of ['canvas','webgl'])
    for(const scene of ['pyramid','chains'])for(const sleep of [false,true]){
    const page=await browser.newPage({viewport:{width:680,height:760}}),errors=[];
    page.on('pageerror',error=>errors.push(String(error)));
    try {
      await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
      const pair=await page.evaluate(async profile=>{
        const {runPlacement}=await import('./placement_probe.mjs');
        if(profile.layout==='mobile')for(const canvas of document.querySelectorAll('canvas')){
          canvas.style.width='360px';canvas.style.height='640px';
        }
        const main=await runPlacement(document.querySelector('#main'),profile);
        const worker=new Worker('./placement_worker.mjs',{type:'module'});
        const canvas=document.querySelector('#worker').transferControlToOffscreen();
        try {
          const result=await new Promise((resolve,reject)=>{
            const timer=setTimeout(()=>reject(new Error('Placement worker timed out')),30000);
            worker.onmessage=event=>{clearTimeout(timer);event.data.ok?
              resolve(event.data.result):reject(new Error(event.data.error));};
            worker.onerror=event=>{clearTimeout(timer);reject(new Error(event.message));};
            worker.postMessage({canvas,profile},[canvas]);
          });
          // The worker's drawing commands can reach the compositor after its
          // reply. Keep it alive through screenshot capture.
          await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));
          window.__placementWorker=worker;
          return {main,result};
        } catch(error) {worker.terminate();throw error;}
      },{candidate,scene,sleep,layout});
      assert.equal(pair.main.worker,false);
      assert.equal(pair.result.worker,true);
      assert.equal(pair.main.crossOriginIsolated,false);
      assert.equal(pair.result.crossOriginIsolated,false);
      delete pair.main.worker;delete pair.result.worker;
      assert.deepEqual(pair.result,pair.main);
      const first=await page.locator('#main').screenshot();
      const second=await page.locator('#worker').screenshot();
      if (!second.equals(first)) {
        const output=path.resolve('build/reports/placement');
        await fs.mkdir(output,{recursive:true});
        await fs.writeFile(path.join(output,`${layout}-${candidate}-${scene}-${sleep?'sleep':'awake'}-main.png`),first);
        await fs.writeFile(path.join(output,`${layout}-${candidate}-${scene}-${sleep?'sleep':'awake'}-worker.png`),second);
        console.log(`${layout}/${candidate}/${scene}/${sleep?'sleep':'awake'} PNG sizes: ${first.length}, ${second.length}`);
      }
      assert.equal(second.equals(first),true,`${layout}/${candidate}/${scene}/${sleep?'sleep':'awake'} main/worker pixels differ`);
      await page.evaluate(()=>window.__placementWorker.terminate());
      assert.deepEqual(errors,[]);
      console.log(`${layout}/${candidate}/${scene}/${sleep?'sleep':'awake'} actual OffscreenCanvas worker pose, work and pixels PASS`);
    } finally {await page.close();}
  }
} finally {await browser?.close();await new Promise(resolve=>server.close(resolve));}
