import createModule from './bench.mjs';
import {makeViews, workCopy, memoryNames, statNames, stepNames, jsonReplacer} from './views.mjs';
export {jsonReplacer};
export const fixtureNames = Object.freeze(['pyramid','rain','piles','chains','churn','table','inverted']);
export const timestep = Math.fround(1/60);
const snake = name => name.replace(/[A-Z]/g, c => `_${c.toLowerCase()}`);
const round = value => Number(value.toPrecision(9)); // Native report %.9g compatibility.
const qualityNames = ['penetration_max','cached_penetration_max','translation_drift_max','rotation_drift_max',
  'linear_speed_max','angular_speed_max','joint_error_max','cached_support_force_mean','supported_weight'];
const settingNames = ['linear_drag','angular_drag','linear_speed_max','contact_hertz','contact_damping_ratio',
  'contact_push_velocity_max','restitution_threshold','joint_hertz','joint_damping_ratio',
  'sleep_speed_max','sleep_angular_speed_max','sleep_time_min'];
function uint(value, low, high, name) {
  if (!Number.isInteger(value) || value < low || value > high) throw new RangeError(`Invalid ${name}`);
  return value;
}
function wordsToObject(names, values, offset = 0) { return Object.fromEntries(names.map((name,i)=>[name,values[offset+i]])); }
const work = values => Object.fromEntries(Object.entries(workCopy(values)).map(([name,value])=>[snake(name),value]));
export async function createBenchmarkModule({memoryBytes = 64*1024*1024, wasmUrl} = {}) {
  uint(memoryBytes,2*1024*1024,512*1024*1024,'memoryBytes');
  if (memoryBytes % 65536) throw new RangeError('Memory budget must be 64 KiB aligned');
  if (wasmUrl !== undefined && typeof wasmUrl !== 'string' && !(wasmUrl instanceof URL)) throw new TypeError('Invalid wasmUrl');
  let m;
  try { m = await createModule({wasmMemory:new WebAssembly.Memory({initial:memoryBytes/65536,maximum:memoryBytes/65536}),
    ...(wasmUrl === undefined ? {} : {locateFile:()=>String(wasmUrl)})}); }
  catch(cause) { throw new Error('Benchmark module initialization failed; check asset URL and fixed memory budget',{cause}); }
  const worlds = new Set(); let disposed = false;
  const live = () => { if (disposed) throw new Error('Benchmark module is disposed'); };
  return Object.freeze({
    get memory() { live(); return {linear_memory:memoryBytes,stack:1048576,static_end:m._sl_wasm_static_end(),
      heap_base:m._sl_wasm_heap_base(),allocator_used:m._sl_wasm_allocator_used()}; },
    create(fixture, {sleep = false, warmup = 120, steps = 600} = {}) {
      live(); const id = fixtureNames.indexOf(fixture);
      if (id < 0 || typeof sleep !== 'boolean') throw new TypeError('Invalid fixture or sleep policy');
      uint(warmup,0,10000,'warmup'); uint(steps,1,10000,'steps');
      let ptr = m._sl_wasm_bench_create(id,sleep?1:0,warmup,steps);
      if (!ptr) return null;
      try {
        const adapter = m._sl_wasm_bench_adapter(ptr), buffer = m.HEAPU32.buffer;
        const io = {ptr:adapter, of:new Float32Array(buffer,m._sl_wasm_output_f32(adapter),64),
          ou:new Uint32Array(buffer,m._sl_wasm_output_u32(adapter),32)};
        const status = new Uint32Array(buffer,m._sl_wasm_bench_words(ptr),13);
        const quality = new Float64Array(buffer,m._sl_wasm_bench_quality(ptr),25);
        const counters = new BigUint64Array(buffer,m._sl_wasm_bench_counters(ptr),2);
        m._sl_wasm_bench_report(ptr);
        const config = {bodyCapacity:status[8],contactCapacity:status[9],jointCapacity:status[10]};
        const views = makeViews(m,io,config,()=>{throw new Error('Benchmark queries do not issue mutable world handles');});
        const valid = () => { live(); if (!ptr) throw new Error('Benchmark is disposed'); };
        const invalidate = () => { views.state.valid = false; views.queryState.valid = false; views.rayState.valid = false; };
        const invoke = fn => { valid(); invalidate(); return !!m[fn](ptr); };
        const value = Object.freeze({
          fixture, warmup, steps,
          prepare:()=>invoke('_sl_wasm_bench_prepare'), mutate:()=>invoke('_sl_wasm_bench_mutate'),
          step:()=>invoke('_sl_wasm_bench_step'), sample:()=>invoke('_sl_wasm_bench_sample'),
          get snapshot() { valid(); return views.snapshot; },
          refreshSnapshot(diagnostics = false) {
            valid(); if (typeof diagnostics !== 'boolean') throw new TypeError('Invalid diagnostics flag');
            if (views.state.revision === Number.MAX_SAFE_INTEGER) throw new RangeError('Snapshot revision exhausted');
            invalidate();
            if (!m._sl_wasm_snapshot_refresh(adapter,diagnostics?1:0)) return false;
            views.refreshCopy(io.ou[3],io.ou[1],io.ou[2]);
            const state = views.state;
            state.bodyCount=io.ou[0]; state.contactCount=io.ou[1]; state.jointCount=io.ou[2];
            state.geometryUpdates=io.ou[3]; ++state.revision; state.valid=true;
            return true;
          },
          report() {
            valid(); m._sl_wasm_bench_report(ptr);
            m._sl_wasm_stats_read(adapter);
            const stats=wordsToObject(statNames,io.ou), last=wordsToObject(stepNames,io.ou,13);
            const lastWork=work(views.work), cumulative=work(views.work.subarray(13));
            m._sl_wasm_memory_read(adapter);
            const memory=Object.fromEntries(memoryNames.map((name,i)=>[snake(name.replace(/Bytes$/,'')),io.ou[i]]));
            const settings={fixture_version:1,seed:status[1],warmup_steps:warmup,measured_steps:steps,
              dt_seconds:round(timestep),substeps:status[11],body_capacity:config.bodyCapacity,
              contact_capacity:config.contactCapacity,joint_capacity:config.jointCapacity,sleep_enabled:sleep,
              gravity:[round(quality[9]),round(quality[10])],friction:round(quality[23]),restitution:round(quality[24]),
              ...Object.fromEntries(settingNames.map((name,i)=>[name,round(quality[11+i])]))};
            return {scene:fixture,settings,semantic_digest:counters[0].toString(16).padStart(16,'0'),drops:counters[1],
              counts:{awake_dynamics:stats.awakeDynamicCount,sleeping_dynamics:stats.sleepingDynamicCount,
                bodies:stats.bodyCount,contacts:stats.contactCount,joints:stats.jointCount,pairs:stats.pairCount,
                pair_capacity:stats.pairCapacity,body_high:stats.bodyCountHigh,contact_high:stats.contactCountHigh,joint_high:stats.jointCountHigh},
              step:{dynamic_bodies:last.dynamicBodyCount,kinematic_bodies:last.kinematicBodyCount,
                contact_constraints:last.contactConstraintCount,joint_constraints:last.jointConstraintCount,
                substeps:last.substepCount,islands:last.islandCount,island_bodies_max:last.islandBodyCountMax,
                islands_executed:last.islandExecutedCount,islands_skipped:last.islandSkippedCount,work:lastWork},
              cumulative_work:cumulative,memory_bytes:memory,
              quality:{window_steps:status[7],...Object.fromEntries(qualityNames.map((name,i)=>[name,round(quality[i])]))},
              driver:{index:status[4],phase:status[5],failed:!!status[6],complete:status[4]===warmup+steps,
                context_bytes:m._sl_wasm_bench_bytes(),adapter_bytes:m._sl_wasm_adapter_bytes(adapter),
                output_bytes:views.outputBytes,context_base_bytes:m._sl_wasm_context_bytes()}};
          },
          dispose() { if (ptr) { invalidate(); m._sl_wasm_bench_destroy(ptr); ptr=0; worlds.delete(value); } }
        });
        worlds.add(value); return value;
      } catch (error) { m._sl_wasm_bench_destroy(ptr); if (error instanceof RangeError) return null; throw error; }
    },
    dispose() { if (!disposed) { for (const world of worlds) world.dispose(); disposed=true; } }
  });
}
