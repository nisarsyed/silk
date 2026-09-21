#!/usr/bin/env node
import {parseArgs} from 'node:util';
import {mkdir, readFile, writeFile, stat} from 'node:fs/promises';
import {resolve, join, sep, extname} from 'node:path';
import {pathToFileURL} from 'node:url';
import {cpus, platform, release} from 'node:os';
import {createServer} from 'node:http';
const {values,tokens}=parseArgs({tokens:true,options:{build:{type:'string'},output:{type:'string'},repeat:{type:'string',default:'5'},
  sleep:{type:'string',default:'off'},scene:{type:'string',default:'all'},warmup:{type:'string',default:'120'},
  steps:{type:'string',default:'600'},smoke:{type:'boolean'},browser:{type:'boolean'},memory:{type:'string',default:'67108864'}}});
const supplied=new Set();
for(const token of tokens) {
  if(token.kind!=='option') continue;
  if(supplied.has(token.name)) throw new Error(`Duplicate option: --${token.name}`);
  supplied.add(token.name);
}
if(values.smoke&&(supplied.has('warmup')||supplied.has('steps'))) throw new Error('--smoke fixes warmup/steps at 2/8; omit explicit counts');
if(!values.build||!values.output) throw new Error('Usage: --build <benchmark directory> --output <report directory> [--smoke] [--browser] [--sleep off|on] [--repeat 1..20]');
function integer(text,low,high,name) {
  if(!/^(0|[1-9][0-9]*)$/.test(text)||Number(text)<low||Number(text)>high) throw new Error(`Invalid ${name}`);
  return Number(text);
}
const repeats=integer(values.repeat,1,20,'repeat'), buildPath=resolve(values.build), output=resolve(values.output);
const metadata=JSON.parse(await readFile(join(buildPath,'build.json'),'utf8'));
if(!['off','on'].includes(values.sleep)) throw new Error('Invalid sleep policy');
const options={warmup:values.smoke?2:integer(values.warmup,0,10000,'warmup'),steps:values.smoke?8:integer(values.steps,1,10000,'steps'),
  sleep:values.sleep==='on',memoryBytes:integer(values.memory,2097152,536870912,'memory'),
  ...(values.scene==='all'?{}:{fixtures:[values.scene]})};
const device=cpus()[0]?.model??'unspecified';
await mkdir(output,{recursive:true});
const {jsonReplacer}=await import(pathToFileURL(join(buildPath,'driver.mjs')));
let server,browser,page;
try {
  if(values.browser) {
    const {chromium}=await import('../../wasm/node_modules/@playwright/test/index.mjs');
    server=createServer(async (request,response)=>{
      try {
        const url=new URL(request.url,'http://localhost');
        const file=resolve(buildPath,decodeURIComponent(url.pathname).slice(1));
        if(!file.startsWith(buildPath+sep)||!(await stat(file)).isFile()) {response.writeHead(404);response.end();return;}
        const mime={'.mjs':'text/javascript','.html':'text/html','.json':'application/json','.wasm':'application/wasm'};
        response.writeHead(200,{'Content-Type':mime[extname(file)]??'application/octet-stream','Cache-Control':'no-store'});
        response.end(await readFile(file));
      } catch {response.writeHead(404);response.end();}
    });
    await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
    browser=await chromium.launch(); page=await browser.newPage();
    page.on('pageerror',error=>console.error(error));
    await page.goto(`http://127.0.0.1:${server.address().port}/browser.html`);
    await page.waitForFunction(()=>typeof window.runBenchmark==='function');
  }
  const {runMatrix}=await import(pathToFileURL(join(buildPath,'run.mjs')));
  for(let i=1;i<=repeats;++i) {
    const runtime={kind:'node',engine:`Node ${process.versions.node}; V8 ${process.versions.v8}`,device,
      os:`${platform()} ${release()}`,hardware_concurrency:cpus().length,automated:true,
      visibility_start:'not-applicable',visibility_end:'not-applicable',process_reused:true,module_per_repeat:true};
    let report;
    if(page) {
      // Return exact JSON from the page: protocol serialization must not narrow bigint.
      const text=await page.evaluate(async options=>JSON.stringify(await window.runBenchmark(options),
        (_key,value)=>typeof value==='bigint'?value.toString():value),{...options,device,automated:true});
      report=JSON.parse(text);
    } else report=await runMatrix({...options,build:metadata,runtime});
    const path=join(output,`run-${i}.json`);
    await writeFile(path,JSON.stringify(report,jsonReplacer,2)+'\n');
    console.log(`${path}: ${report.execution_status}, ${report.native.results.length} scenes; ${options.warmup}/${options.steps}`);
    if(report.execution_status!=='complete') process.exitCode=1;
  }
} finally {
  await browser?.close();
  if(server) await new Promise(resolve=>server.close(resolve));
}
