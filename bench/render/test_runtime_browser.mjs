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
  for(const candidate of ['canvas','webgl']){
    const page=await browser.newPage({viewport:{width:700,height:760}}),errors=[];
    page.on('pageerror',error=>errors.push(String(error)));
    try{
      await page.goto(`http://127.0.0.1:${server.address().port}/placement.html`);
      const main=await page.evaluate(async candidate=>{
        const {createRuntimeOwner}=await import('./runtime_owner.mjs');
        const canvas=document.querySelector('#main');
        const wait=async test=>{
          const deadline=performance.now()+10000;
          while(!await test()){if(performance.now()>deadline)throw new Error('Runtime did not progress');
            await new Promise(resolve=>setTimeout(resolve,20));}
        };
        const owner=await createRuntimeOwner(canvas,{candidate});
        try{
          let duplicate=false;
          try{await createRuntimeOwner(canvas,{candidate});}catch{duplicate=true;}
          await wait(()=>owner.report().steps>=3);
          const initial=owner.report(),epoch=owner.epoch;
          if(owner.pointer(epoch,1,0,3,0,0)!=='queued')throw new Error('Pointer down was not queued');
          await wait(()=>owner.report().pointerHeld);
          if(!owner.pause())throw new Error('Pause failed');
          if(owner.report().pointerHeld)throw new Error('Pause left pointer held');
          const held=owner.report().steps;
          await new Promise(resolve=>setTimeout(resolve,100));
          if(owner.report().steps!==held)throw new Error('Paused world advanced');
          if(owner.pointer(epoch,1,0,3,0,0)!=='stale')throw new Error('Old input epoch accepted');
          if(!owner.singleStep())throw new Error('Single step failed');
          await wait(()=>owner.report().steps===held+1);
          if(!owner.reset('chains',true))throw new Error('Reset failed');
          const reset=owner.report();
          if(!owner.resize('mobile'))throw new Error('Resize failed');
          const mobile=[canvas.width,canvas.height];
          if(!owner.resume())throw new Error('Resume failed');
          await wait(()=>owner.report().steps>=3);
          canvas.dispatchEvent(new Event('webglcontextlost',{cancelable:true}));
          return {duplicate,initial,reset,mobile,failed:owner.report().state,
            failure:owner.failure,final:owner.report()};
        }finally{owner.dispose();}
      },candidate);
      assert.equal(main.duplicate,true);
      assert.equal(main.initial.state,'running');
      assert.equal(main.reset.state,'paused');
      assert.equal(main.reset.scene,'chains');
      assert.equal(main.reset.sleep,true);
      assert.equal(main.reset.steps,0);
      assert.deepEqual(main.mobile,[720,1280]);
      assert.equal(main.failed,'failed');
      assert.match(main.failure,/Graphics context lost/);

      const worker=await page.evaluate(async candidate=>{
        const canvas=document.querySelector('#worker').transferControlToOffscreen();
        const worker=new Worker('./runtime_worker.mjs',{type:'module'});
        let id=0;
        const ask=(kind,fields={},transfer=[])=>new Promise((resolve,reject)=>{
          const number=++id,timer=setTimeout(()=>reject(new Error(`${kind} timed out`)),10000);
          const receive=event=>{if(event.data.id!==number)return;
            clearTimeout(timer);worker.removeEventListener('message',receive);
            event.data.ok?resolve(event.data.result):reject(new Error(event.data.error));};
          worker.addEventListener('message',receive);
          worker.postMessage({id:number,kind,...fields},transfer);
        });
        const wait=async test=>{
          const deadline=performance.now()+10000;
          while(!await test()){if(performance.now()>deadline)throw new Error('Worker did not progress');
            await new Promise(resolve=>setTimeout(resolve,20));}
        };
        try{
          const initial=await ask('init',{canvas,profile:{candidate}},[canvas]);
          let duplicate=false;
          try{await ask('init');}catch{duplicate=true;}
          await wait(async()=>((await ask('report')).steps>=3));
          const epoch=initial.epoch;
          if(await ask('pointer',{args:[epoch,1,0,3,0,0]})!=='queued')
            throw new Error('Worker pointer down was not queued');
          await wait(async()=>(await ask('report')).pointerHeld);
          await ask('pause');
          if((await ask('report')).pointerHeld)throw new Error('Pause left worker pointer held');
          const held=(await ask('report')).steps;
          await new Promise(resolve=>setTimeout(resolve,100));
          if((await ask('report')).steps!==held)throw new Error('Paused worker advanced');
          const stale=await ask('pointer',{args:[epoch,1,0,3,0,0]});
          await ask('single-step');
          await wait(async()=>((await ask('report')).steps===held+1));
          await ask('reset',{args:['chains',true]});
          const reset=await ask('report');
          await ask('resize',{layout:'mobile'});
          await ask('resume');
          await wait(async()=>((await ask('report')).steps>=3));
          const resized=await ask('report');
          const closed=await ask('dispose');
          return {duplicate,initial,stale,reset,resized,closed};
        }finally{worker.terminate();}
      },candidate);
      assert.equal(worker.duplicate,true);
      assert.equal(worker.initial.state,'running');
      assert.equal(worker.stale,'stale');
      assert.equal(worker.reset.state,'paused');
      assert.equal(worker.reset.scene,'chains');
      assert.equal(worker.reset.sleep,true);
      assert.equal(worker.reset.steps,0);
      assert.equal(worker.resized.layout,'mobile');
      assert.equal(worker.closed,'disposed');
      assert.deepEqual(errors,[]);
      console.log(`${candidate} main/worker owner pause, step, reset, resize, context failure and disposal PASS`);
    }finally{await page.close();}
  }
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
