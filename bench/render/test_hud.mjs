import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';
const build=path.resolve(process.argv[2]??'build/render-study');
const output=path.resolve(process.argv[3]??'build/reports/render-hud');
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
  for(const [width,height,dpr]of [[1280,720,1],[360,640,2]]){
    const page=await browser.newPage({viewport:{width,height},deviceScaleFactor:dpr}),errors=[];
    page.on('pageerror',error=>errors.push(String(error)));
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
      await page.waitForFunction(()=>typeof window.verifyStudyHud==='function');
      const result=await page.evaluate(()=>window.verifyStudyHud());
      assert.equal(result.updates,4);assert.deepEqual(errors,[]);
      assert.equal(await page.evaluate(()=>{const e=document.getElementById('study-hud');return e.scrollWidth<=e.clientWidth&&e.getBoundingClientRect().right<=innerWidth;}),true);
      await page.screenshot({path:path.join(output,`hud-${width}x${height}.png`)});
      results.push({width,height,dpr,...result});
    }finally{await page.close();}
  }
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({kind:'correctness-only',results},null,2)+'\n');
  console.log('HUD: 4 Hz deadlines, skipped late refreshes, actual counters, unknown GPU metrics, disposal and both layouts PASS');
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
