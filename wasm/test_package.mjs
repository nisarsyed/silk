// Distribution verification. The consumer is installed outside the checkout;
// SDK paths are removed from its environment and only public package exports are used.
import assert from 'node:assert/strict';
import { mkdtemp, readFile, writeFile, cp, mkdir, stat } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { resolve, dirname, join, extname, sep, delimiter } from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';
import { createServer } from 'node:http';
import { chromium, firefox, webkit } from '@playwright/test';
const source = dirname(fileURLToPath(import.meta.url));
const packageDir = process.argv[2] ? resolve(process.argv[2]) : resolve(source, '../build/wasm-release/package');
const browserTypes = {chromium,firefox,webkit};
const browserNames = process.argv[3] ? process.argv[3].split(',') : ['chromium'];
if (!browserNames.length || new Set(browserNames).size !== browserNames.length ||
    browserNames.some(name => !Object.hasOwn(browserTypes,name)))
  throw new TypeError('Expected unique chromium,firefox,webkit browser names');
const reportDir = process.argv[4] ? resolve(process.argv[4]) : null;
const consumer = await mkdtemp(join(tmpdir(), 'silk-package-consumer-'));
console.log(`Independent consumer: ${consumer}`);
const environment = Object.fromEntries(Object.entries(process.env).filter(([key]) => !/^(EMSDK|EM_|EMCC|EMSCRIPTEN|CMAKE)/.test(key)));
// Keep ordinary Node/npm paths; remove compiler/SDK search paths from the consumer.
if (process.env.EMSDK) environment.PATH = (environment.PATH ?? '').split(delimiter)
  .filter(path => !path.startsWith(process.env.EMSDK)).join(delimiter);
const npm = process.platform === 'win32' ? 'npm.cmd' : 'npm';
function run(command, args, cwd = consumer) {
  const result = spawnSync(command, args, {cwd, env:environment, encoding:'utf8', timeout:120000});
  if (result.error) throw result.error;
  assert.equal(result.status,0,`${command} ${args.join(' ')}\n${result.stdout}\n${result.stderr}`);
  return result.stdout;
}
const packed = JSON.parse(run(npm,['pack','--json','--pack-destination',consumer],packageDir))[0];
const expected = ['package.json','index.mjs','views.mjs','silk.mjs','silk.wasm','index.d.ts','README.md','LICENSE',
  'licenses/README.md','licenses/emscripten.txt','licenses/musl.txt','licenses/compiler-rt.txt','licenses/fmaf.txt'].sort();
assert.deepEqual(packed.files.map(file=>file.path).sort(),expected,'Archive allowlist');
assert(packed.files.find(file=>file.path==='silk.wasm').size > 10000,'Archive contains compiled binary');
await writeFile(join(consumer,'package.json'),JSON.stringify({private:true,type:'module'}));
run(npm,['install','--ignore-scripts','--no-audit','--no-fund',join(consumer,packed.filename)]);
await cp(join(source,'consumer'),consumer,{recursive:true});
await writeFile(join(consumer,'tsconfig.json'),JSON.stringify({compilerOptions:{target:'ES2022',module:'NodeNext',
  moduleResolution:'NodeNext',strict:true,noEmitOnError:true,outDir:'types',lib:['ES2022','DOM']},include:['api.ts']}));
console.log(run(process.execPath,[join(source,'node_modules/typescript/bin/tsc'),'--project','tsconfig.json']));
console.log(run(process.execPath,['node.mjs']));
await mkdir(join(consumer,'relocated'));
await cp(join(consumer,'node_modules/@nisarsyed/silk/silk.wasm'),join(consumer,'relocated/silk.wasm'));
await writeFile(join(consumer,'index.html'),'<html lang="en"><meta charset="utf-8"><title>Bundled Silk consumer</title><body><output id="result">Loading…</output><script type="module" src="/bundle.mjs"></script></body></html>');
await writeFile(join(consumer,'vite.config.mjs'),"export default {base:'/nested/demo/',build:{target:'es2022',assetsInlineLimit:0}};\n");
console.log(run(process.execPath,[join(source,'node_modules/vite/bin/vite.js'),'build']));
const mime = {'.mjs':'text/javascript','.js':'text/javascript','.html':'text/html','.wasm':'application/wasm','.json':'application/json'};
const server = createServer(async (request,response) => {
  try {
    const url = new URL(request.url,'http://localhost'), path = decodeURIComponent(url.pathname);
    const bundled = path.startsWith('/nested/demo/');
    const relative = bundled ? path.slice('/nested/demo/'.length) || 'index.html' : path.slice(1);
    const base = bundled ? join(consumer,'dist') : consumer;
    const file = resolve(base,relative);
    if (!file.startsWith(base+sep) || !(await stat(file)).isFile()) { response.writeHead(404); response.end(); return; }
    response.writeHead(200,{'Content-Type':mime[extname(file)] ?? 'application/octet-stream','Cache-Control':'no-store'});
    response.end(await readFile(file));
  } catch { response.writeHead(404); response.end(); }
});
await new Promise(resolve => server.listen(0,'127.0.0.1',resolve));
const origin = `http://127.0.0.1:${server.address().port}`;
const summary={archive:packed.filename,browsers:[]};
if(reportDir)await mkdir(reportDir,{recursive:true});
try {
  for(const name of browserNames){
    let browser,context;
    const row={name,version:null,paths:[],pageErrors:[],consoleErrors:[],passed:false};
    try {
      browser=await browserTypes[name].launch();row.version=browser.version();
      context=await browser.newContext();
      if(reportDir)await context.tracing.start({screenshots:true,snapshots:true});
      const page=await context.newPage();
      page.on('console',message => {if(message.type()==='error')row.consoleErrors.push(message.text());});
      page.on('pageerror',error => row.pageErrors.push(error.message));
      for(const path of ['/browser.html','/nested/demo/']){
        await page.goto(origin+path);
        await page.waitForFunction(()=>document.documentElement.dataset.result,null,{timeout:45000});
        const result=await page.locator('#result').innerText();
        const passed=await page.locator('html').getAttribute('data-result')==='pass';
        row.paths.push({path,passed,result});
        assert.equal(passed,true,`${name} ${path}: ${result}\n${row.consoleErrors.join('\n')}`);
        assert.deepEqual(row.pageErrors,[],`${name} uncaught browser errors`);
        console.log(`${name} installed consumer ${path}: ${result}`);
      }
      row.passed=true;
    }catch(error){
      row.error=String(error?.stack??error);
      if(reportDir&&context){
        try{await context.tracing.stop({path:join(reportDir,`${name}-failure.zip`)});}
        catch(traceError){row.traceError=String(traceError);}
      }
      throw error;
    }finally{
      if(reportDir&&context&&!row.error)await context.tracing.stop();
      await context?.close();await browser?.close();
      summary.browsers.push(row);
      if(reportDir)await writeFile(join(reportDir,'package-results.json'),JSON.stringify(summary,null,2));
    }
  }
  console.log(`Archive ${packed.filename}: exact allowlist, types, Node, ${browserNames.join('/')} browser and actual worker, relocated assets, failures, and production bundle passed`);
}finally{await new Promise(resolve=>server.close(resolve));}
