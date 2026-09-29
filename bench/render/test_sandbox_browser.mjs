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
      target.endsWith('.html')?'text/html':target.endsWith('.css')?'text/css':'text/javascript');
    res.end(await fs.readFile(target));
  }catch{res.writeHead(404).end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser;
try{
  browser=await engine.launch({headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:900}}),errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  const url=`http://127.0.0.1:${server.address().port}/sandbox.html`;
  await page.goto(url);
  await page.getByText('World running').waitFor();
  await page.waitForFunction(()=>Number(document.querySelector('#steps').textContent)>2);
  assert.match(await page.locator('#counts').innerText(),/^211 \/ \d+$/);
  assert.equal(await page.locator('#world').evaluate(canvas=>canvas.width),1280);
  await page.getByRole('button',{name:'Circle'}).click();
  await page.locator('#world').click({position:{x:400,y:100}});
  await page.waitForFunction(()=>document.querySelector('#counts').textContent.startsWith('212 /'));
  await page.getByRole('button',{name:'Box'}).click();
  await page.locator('#world').click({position:{x:480,y:100}});
  await page.waitForFunction(()=>document.querySelector('#counts').textContent.startsWith('213 /'));
  await page.getByText('View details').click();
  for(const label of ['Contacts','Bounds','Joints','Queries','Islands & sleep']){
    const checkbox=page.getByLabel(label,{exact:true});
    await checkbox.check();assert.equal(await checkbox.isChecked(),true);
  }
  await page.getByRole('button',{name:'Pause'}).click();
  await page.getByText('World paused').waitFor();
  await page.waitForTimeout(300);
  const held=Number(await page.locator('#steps').innerText());
  await page.waitForTimeout(400);
  assert.equal(Number(await page.locator('#steps').innerText()),held);
  await page.getByRole('button',{name:'Step once'}).click();
  await page.waitForFunction(before=>Number(document.querySelector('#steps').textContent)===before+1,held);
  await page.getByRole('button',{name:'Chains'}).click();
  await page.waitForFunction(()=>document.querySelector('#counts').textContent.startsWith('104 /'));
  assert.equal(await page.getByRole('button',{name:'Step once'}).isEnabled(),true);
  await page.getByRole('button',{name:'Resume'}).click();
  await page.getByText('World running').waitFor();
  await page.getByRole('button',{name:'Rain'}).click();
  await page.waitForFunction(()=>document.querySelector('#counts').textContent.startsWith('2003 /'));
  await page.getByRole('button',{name:'Pyramid'}).click();
  await page.getByRole('button',{name:'Reset'}).click();
  await page.setViewportSize({width:402,height:874});
  await page.waitForFunction(()=>document.querySelector('#world').width===720);
  const frame=await page.locator('#world').boundingBox();
  assert.ok(frame&&frame.x>=0&&frame.x+frame.width<=402&&frame.width>300);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
  await page.locator('#world').dispatchEvent('contextlost',{cancelable:true});
  await page.getByText('World stopped').waitFor();
  assert.equal(await page.getByRole('button',{name:'Reset'}).isDisabled(),true);
  await page.getByRole('button',{name:'Restart world'}).click();
  await page.getByText('World running').waitFor();
  assert.equal(await page.getByRole('button',{name:'Reset'}).isEnabled(),true);
  assert.deepEqual(errors,[]);
  const touch=await browser.newPage({viewport:{width:402,height:874},hasTouch:true,deviceScaleFactor:3});
  const touchErrors=[];
  touch.on('pageerror',error=>touchErrors.push(String(error)));
  try{
    await touch.goto(url);
    await touch.getByText('World running').waitFor();
    await touch.getByRole('button',{name:'Circle'}).tap();
    await touch.locator('#world').scrollIntoViewIfNeeded();
    const portrait=await touch.locator('#world').boundingBox();
    assert.ok(portrait&&portrait.x>=0&&portrait.x+portrait.width<=402&&portrait.y>=-1);
    await touch.touchscreen.tap(portrait.x+portrait.width/2,portrait.y+portrait.height/3);
    await touch.waitForFunction(()=>document.querySelector('#counts').textContent.startsWith('212 /'));
    await touch.setViewportSize({width:874,height:402});
    await touch.waitForFunction(()=>document.querySelector('#world').width===1280);
    await touch.setViewportSize({width:402,height:874});
    await touch.waitForFunction(()=>document.querySelector('#world').width===720);
    assert.equal(await touch.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    assert.deepEqual(touchErrors,[]);
  }finally{await touch.close();}
  console.log(`${engineName} sandbox scenes, transport, overlays, resize, failure and recovery PASS`);
}finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
