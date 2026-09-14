// Shared Node/browser measurement harness. Host timing never enters C state.
import {fixtureNames, timestep, createBenchmarkModule} from './driver.mjs';
export {jsonReplacer} from './driver.mjs';
const round = value => Number(value.toPrecision(9));
function elapsed(start, end) {
  const value=end-start;
  if (!Number.isFinite(value) || value < 0) throw new Error('Invalid/backward performance timer');
  return value;
}
function distribution(values) {
  const sorted=Array.from(values).sort((a,b)=>a-b), n=sorted.length;
  let total=0; for (const value of values) total+=value;
  return {average:round(total/n),median:round(n%2?sorted[n>>1]:(sorted[n/2-1]+sorted[n/2])/2),
    p95:round(sorted[Math.ceil(0.95*n)-1]),max:round(sorted[n-1])};
}
function clockProbe(now) {
  let minimum=Infinity, total=0;
  for (let i=0;i<1000;++i) { const start=now(); const value=elapsed(start,now()); if(value>0) minimum=Math.min(minimum,value); total+=value; }
  return {pairs:1000,minimum_positive_ms:Number.isFinite(minimum)?minimum:null,average_pair_ms:total/1000};
}
export async function runMatrix({build,runtime,sleep=false,warmup=120,steps=600,fixtures=fixtureNames,
  memoryBytes=64*1024*1024,wasmUrl,now=()=>performance.now()} = {}) {
  if (!build || !runtime) throw new TypeError('Build and runtime provenance are required');
  if (!Number.isInteger(warmup)||warmup<0||warmup>10000||!Number.isInteger(steps)||steps<1||steps>10000||typeof sleep!=='boolean') throw new RangeError('Invalid benchmark profile');
  if (!Array.isArray(fixtures)||!fixtures.length||fixtures.length>7||new Set(fixtures).size!==fixtures.length||fixtures.some(x=>!fixtureNames.includes(x))) throw new TypeError('Invalid fixture selection');
  const report={schema_version:1,kind:'silk-wasm-benchmark',build,runtime,
    profile:{warmup_steps:warmup,measured_steps:steps,sleep_enabled:sleep,fixtures:[...fixtures],
      settling_profile:warmup===120&&steps===600},
    timing_scope:'Host performance.now around driver.step, including wrapper/FFI and phase checks; setup, preparation, mutations, quality and reporting excluded. Timer overhead is measured separately, never subtracted.',
    clock:clockProbe(now),module:{memory_bytes:memoryBytes,load_ms:0},regions:[],failures:[],execution_status:'failed',
    native:{schema_version:4,metadata:{compiler:'Emscripten',compiler_version:build.sdk_version,build:build.mode,
      flags:build.link_flags,warnings:'Authoritative silk_warnings; see build.compile_commands',host:runtime.kind,
      host_version:runtime.engine,processor:runtime.device,revision:build.revision+(build.dirty?'-dirty':'')},results:[]}};
  let module;
  try {
    let start=now(); module=await createBenchmarkModule({memoryBytes,wasmUrl}); report.module.load_ms=elapsed(start,now());
    for (const fixture of fixtures) {
      let bench,stage='setup',index=0;
      const regions={scene:fixture,setup_ms:0,warmup_ms:0,prepare_ms:0,mutation_ms:0,step_ms:0,quality_ms:0,
        report_ms:0,snapshot_ms:0,step_samples_ms:[],mutation_samples_ms:[],memory:null,driver:null};
      report.regions.push(regions);
      try {
        start=now();
        const samples=new Float64Array(steps), mutations=new Float64Array(steps);
        bench=module.create(fixture,{sleep,warmup,steps});
        if (!bench) throw new Error('Fixture allocation failed within the fixed module budget');
        regions.setup_ms=elapsed(start,now());
        const check = (success,name) => { if (!success) { stage=name; throw new Error(`Driver ${name} rejected at iteration ${index}`); } };
        start=now();
        for (;index<warmup;++index) {
          check(bench.prepare(),'prepare'); check(bench.mutate(),'mutate'); check(bench.step(),'step'); check(bench.sample(),'sample');
        }
        regions.warmup_ms=elapsed(start,now());
        for (;index<warmup+steps;++index) {
          const row=index-warmup;
          stage='prepare'; start=now(); check(bench.prepare(),stage); regions.prepare_ms+=elapsed(start,now());
          stage='mutate';
          if (fixture==='churn') {
            start=now(); check(bench.mutate(),stage); mutations[row]=elapsed(start,now()); regions.mutation_ms+=mutations[row];
          } else check(bench.mutate(),stage);
          stage='step'; start=now(); check(bench.step(),stage); samples[row]=elapsed(start,now()); regions.step_ms+=samples[row];
          stage='sample'; start=now(); check(bench.sample(),stage); regions.quality_ms+=elapsed(start,now());
        }
        stage='snapshot'; start=now(); check(bench.refreshSnapshot(),stage); regions.snapshot_ms=elapsed(start,now());
        stage='report'; start=now();
        const result=bench.report(), driver=result.driver; delete result.driver;
        result.timing_ms={...distribution(samples),mutation_average:round(regions.mutation_ms/steps)};
        regions.step_samples_ms=Array.from(samples);
        regions.mutation_samples_ms=fixture==='churn'?Array.from(mutations):[];
        regions.driver={index:driver.index,phase:driver.phase,failed:driver.failed,complete:driver.complete};
        regions.memory={...module.memory,context_bytes:driver.context_bytes,adapter_bytes:driver.adapter_bytes,
          output_bytes:driver.output_bytes,context_base_bytes:driver.context_base_bytes,
          requested_bytes:result.memory_bytes.arena+driver.context_bytes+driver.adapter_bytes-driver.context_base_bytes,
          sample_bytes:samples.byteLength+mutations.byteLength};
        report.native.results.push(result);
        regions.report_ms=elapsed(start,now());
        if (!driver.complete||driver.failed||result.drops!==0n) throw new Error('Incomplete driver, invalid quality, or contact drops');
      } catch(error) {
        let state=null; try { state=bench?.report()??null; } catch { /* Preserve the original failure. */ }
        report.failures.push({scene:fixture,stage,index,message:String(error.message),state});
      } finally { bench?.dispose(); }
    }
    report.execution_status=report.failures.length?'failed':'complete';
  } catch(error) { report.failures.push({scene:null,stage:'module',index:0,message:String(error.message),state:null}); }
  finally { module?.dispose(); }
  return report;
}
