import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {chromium} from '../../wasm/node_modules/@playwright/test/index.mjs';
const build=path.resolve(process.argv[2]??'build/render-study');
const output=path.resolve(process.argv[3]??'build/reports/render-pointer');
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
let browser,failure=null;const results=[];
try{
  browser=await chromium.launch({headless:true});
  for(const [width,height]of [[1280,720],[720,1280]])for(const scene of ['pyramid','rain','chains'])for(const sleep of [false,true])for(const diagnostic of [false,true]){
    const page=await browser.newPage({viewport:{width:width===720?360:width,height:height===1280?640:height},deviceScaleFactor:width===720?2:1}),errors=[];
    page.on('pageerror',error=>errors.push(String(error)));
    page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('favicon'))errors.push(message.text());});
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/verify.html`);
      await page.waitForFunction(()=>typeof window.verifyColocated==='function');
      const result=await page.evaluate(profile=>window.verifyColocated(profile),{scene,sleep,diagnostic,width,height,interaction:true});
      results.push(result);assert.deepEqual(errors,[]);
      for(const image of result.results)assert.equal(image.interiorMismatch,0);
    }finally{await page.close();}
  }
  console.log('Pointer rendering: 24 profiles, exact cross-module state, retained graphics, zero bulk JS copies and unchanged pixel allowance PASS');
}catch(error){failure=String(error);throw error;}
finally{
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({kind:'correctness-only',input:'scripted driver commands, not real pointer evidence',browser:browser?.version(),failure,results},null,2)+'\n');
  await browser?.close();await new Promise(resolve=>server.close(resolve));
}
