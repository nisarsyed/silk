import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {resolve} from 'node:path';
const entry=pathToFileURL(resolve(process.argv[2]??'build/wasm-debug/benchmark/driver.mjs'));
const {createBenchmarkModule,fixtureNames,jsonReplacer}=await import(entry);
await assert.rejects(createBenchmarkModule({memoryBytes:1}),RangeError);
await assert.rejects(createBenchmarkModule({memoryBytes:2097153}),RangeError);
const module=await createBenchmarkModule();
assert.throws(()=>module.create('unknown'),TypeError);
assert.throws(()=>module.create('table',{sleep:1}),TypeError);
assert.throws(()=>module.create('table',{steps:0}),RangeError);
assert.throws(()=>module.create('table',{warmup:10001}),RangeError);
for (const fixture of fixtureNames) {
  for (const sleep of [false,true]) {
    const a=module.create(fixture,{sleep,warmup:0,steps:2});
    const b=module.create(fixture,{sleep,warmup:0,steps:2});
    assert(a&&b); const before=a.report();
    assert.equal(a.step(),false); assert.equal(a.sample(),false); assert.equal(a.mutate(),false);
    assert.deepEqual(a.report(),before);
    const allocated=module.memory.allocator_used;
    for(let i=0;i<2;++i) {
      for(const world of [a,b]) {
        assert(world.prepare()); assert.equal(world.prepare(),false);
        assert(world.mutate()); assert(world.step()); assert(world.sample());
      }
      // Only one twin extracts snapshots: observations must preserve replay.
      assert(a.refreshSnapshot(true));
      assert.deepEqual(a.report(),b.report());
    }
    const snapshot=a.snapshot, revision=snapshot.revision, retained=snapshot.copy();
    new Uint8Array(snapshot.bodies.x.buffer).fill(255);
    assert.deepEqual(a.report(),b.report());
    assert(a.refreshSnapshot()); assert.equal(snapshot.isCurrent(revision),false);
    assert.equal(snapshot.bodyCount,retained.bodyCount);
    assert.equal(module.memory.allocator_used,allocated);
    assert.equal(a.prepare(),false); assert(a.report().driver.complete);
    a.dispose(); a.dispose(); assert.equal(snapshot.valid,false);
    assert.throws(()=>a.report(),/disposed/); assert.throws(()=>snapshot.copy(),/invalid/);
    assert(b.report().driver.complete); b.dispose();
  }
}
assert.equal(JSON.stringify(18446744073709551615n,jsonReplacer),'"18446744073709551615"');
module.dispose();module.dispose();assert.throws(()=>module.create('table'),/disposed/);
const tiny=await createBenchmarkModule({memoryBytes:2097152});
for(let i=0;i<8;++i) assert.equal(tiny.create('rain'),null);
const recovered=tiny.create('table',{warmup:0,steps:1});assert(recovered);
assert(recovered.prepare()&&recovered.mutate()&&recovered.step()&&recovered.sample());
tiny.dispose();assert.throws(()=>recovered.step(),/disposed/);
console.log('Benchmark lifecycle, phase order, all fixture/sleep replay, bounded output, and allocation recovery passed');
