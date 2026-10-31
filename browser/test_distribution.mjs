import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import fs from 'node:fs/promises';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import {chromium} from '../wasm/node_modules/@playwright/test/index.mjs';

const site=path.resolve(process.argv[2]??'build/browser-site');
const metadata=JSON.parse(await fs.readFile(path.join(site,'distribution.json'),'utf8'));
const digest=value=>createHash('sha256').update(value).digest('hex');
assert.equal(metadata.schema,1);
assert.match(metadata.source.revision,/^[a-f0-9]{40}$/);
assert.equal(metadata.package.version,metadata.version);
for(const [name,expected] of Object.entries(metadata.artifacts)){
  const file=path.resolve(site,name);
  assert.ok(file.startsWith(site+path.sep));
  const content=await fs.readFile(file);
  assert.deepEqual({bytes:content.length,sha256:digest(content)},expected,name);
}
const checksums=(await fs.readFile(path.join(site,'checksums.sha256'),'utf8')).trim().split('\n');
assert.equal(checksums.length,Object.keys(metadata.artifacts).length+1);
for(const line of checksums){
  const [,hash,name]=/^([a-f0-9]{64})  (.+)$/.exec(line)??[];
  assert.ok(name&&hash,`Bad checksum line: ${line}`);
  assert.equal(hash,digest(await fs.readFile(path.join(site,name))));
}
const consumer=await fs.mkdtemp(path.join(os.tmpdir(),'silk-distribution-'));
try{
  await fs.writeFile(path.join(consumer,'package.json'),'{"private":true,"type":"module"}\n');
  const archive=path.join(site,metadata.package.archive);
  const install=spawnSync('npm',['install','--ignore-scripts','--no-audit','--no-fund',archive],
    {cwd:consumer,encoding:'utf8',timeout:120000});
  assert.equal(install.status,0,install.stderr||install.error);
  await fs.writeFile(path.join(consumer,'check.mjs'),
    "import {createSilk} from '@nisarsyed/silk';\n"+
    "const silk=await createSilk();const world=silk.createWorld({bodyCapacity:8});\n"+
    "if(!world||!world.createBody({mass:1,shape:silk.circle(.2)}))throw new Error('Consumer failed');\n"+
    "world.step(Math.fround(1/60));silk.dispose();\n");
  const run=spawnSync(process.execPath,['check.mjs'],
    {cwd:consumer,encoding:'utf8',timeout:120000});
  assert.equal(run.status,0,run.stderr||run.error);
}finally{await fs.rm(consumer,{recursive:true,force:true});}

const requests=[];
const server=http.createServer(async(request,response)=>{
  try{
    const url=new URL(request.url,'http://localhost');
    if(!url.pathname.startsWith('/silk/')){response.writeHead(404).end();return;}
    const relative=decodeURIComponent(url.pathname.slice('/silk/'.length))||'index.html';
    const file=path.resolve(site,relative);
    if(!file.startsWith(site+path.sep)){response.writeHead(403).end();return;}
    const mime=file.endsWith('.wasm')?'application/wasm':file.endsWith('.html')?'text/html':
      file.endsWith('.css')?'text/css':file.endsWith('.json')?'application/json':
      file.endsWith('.sha256')?'text/plain':'text/javascript';
    requests.push(url.pathname);
    response.writeHead(200,{'Content-Type':mime,'Cache-Control':'no-store'});
    response.end(await fs.readFile(file));
  }catch{response.writeHead(404).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const origin=`http://127.0.0.1:${server.address().port}`;
let browser;
try{
  browser=await chromium.launch({headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:900}}),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  await page.goto(`${origin}/silk/`);
  await page.getByText('World running').waitFor();
  assert.match(await page.locator('#revision').innerText(),new RegExp(metadata.version));
  assert.match(await page.locator('#counts').innerText(),/^211 \/ \d+$/);
  const archiveLink=await page.locator('.download-button').getAttribute('href');
  const archiveUrl=new URL(archiveLink,page.url()).href;
  assert.equal(digest(Buffer.from(await (await fetch(archiveUrl)).arrayBuffer())),metadata.package.sha256);
  assert.ok(requests.includes('/silk/physics/study.wasm'));
  assert.ok(requests.includes('/silk/sandbox.css'));
  assert.deepEqual(errors,[]);

  const worker=await page.evaluate(async()=>{
    const {startBrowserRuntime}=await import('./runtime_controller.mjs');
    const canvas=document.createElement('canvas');document.body.append(canvas);
    const runtime=await startBrowserRuntime(canvas,{candidate:'canvas'},
      {preferWorker:true,workerUrl:new URL('./runtime_worker.mjs',location.href)});
    try{return {mode:runtime.mode,state:(await runtime.report()).state};}
    finally{await runtime.dispose();canvas.remove();}
  });
  assert.deepEqual(worker,{mode:'worker',state:'running'});
  assert.ok(requests.includes('/silk/runtime_worker.mjs'));
  const failedWorker=await page.evaluate(async()=>{
    const {startBrowserRuntime}=await import('./runtime_controller.mjs');
    const canvas=document.createElement('canvas');document.body.append(canvas);
    const runtime=await startBrowserRuntime(canvas,{candidate:'canvas'},
      {preferWorker:true,workerUrl:new URL('./missing-worker.mjs',location.href)});
    try{return {mode:runtime.mode,state:(await runtime.report()).state,
      reason:runtime.fallbackReason};}
    finally{await runtime.dispose();canvas.remove();}
  });
  assert.equal(failedWorker.mode,'main');
  assert.equal(failedWorker.state,'running');
  assert.match(failedWorker.reason,/Worker startup failed/);

  for(const body of [null,'corrupt wasm']){
    const broken=await browser.newPage();
    try{
      await broken.route('**/physics/study.wasm',route=>body===null?
        route.fulfill({status:404,body:'missing'}):
        route.fulfill({status:200,contentType:'application/wasm',body}));
      await broken.goto(`${origin}/silk/`);
      await broken.getByText('Engine unavailable').waitFor();
      assert.equal(await broken.getByRole('button',{name:'Pause'}).isDisabled(),true);
    }finally{await broken.close();}
  }
  const mobile=await browser.newPage({viewport:{width:402,height:874},hasTouch:true});
  try{
    await mobile.goto(`${origin}/silk/`);
    await mobile.getByText('World running').waitFor();
    assert.equal(await mobile.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
  }finally{await mobile.close();}
  console.log('Distribution: checksums, clean archive install, project path, worker, failure recovery and mobile load PASS');
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
