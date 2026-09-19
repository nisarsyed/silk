import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/wasm-release/render-physics');
const {createStudyModule,physicsTiers,stepLimit}=await import(pathToFileURL(path.join(build,'driver.mjs')));
const {createBenchmarkModule}=await import(pathToFileURL(path.join(build,'../benchmark/driver.mjs')));
const module=await createStudyModule({memoryBytes:256*1024*1024});
const benchmark=await createBenchmarkModule();
const counts={pyramid:[211,844,0],rain:[2003,16384,0],chains:[104,1024,96]};
const snake=name=>name.replace(/[A-Z]/g,c=>`_${c.toLowerCase()}`);
try {
  assert.throws(()=>module.create(new String('rain')));
  for(const copies of [0,3,17,NaN,'1'])assert.throws(()=>module.create('rain',{copies}));
  for(const steps of [0,-1,1.5,stepLimit+1,Infinity,'1'])assert.throws(()=>module.create('rain',{steps}));
  assert.throws(()=>module.create('piles'));
  assert.throws(()=>module.create('rain',{sleep:1}));
  for(const scene of ['pyramid','rain','chains'])for(const sleep of [false,true]) {
    for(const copies of physicsTiers) {
      const reference=benchmark.create(scene,{sleep,warmup:0,steps:3});assert.ok(reference);
      const world=module.create(scene,{copies,sleep,steps:3});assert.ok(world);
      try {
        const config=world.configuration, expected=reference.report().settings;
        assert.equal(config.bodyCapacity,counts[scene][0]*copies);
        assert.equal(config.contactCapacity,counts[scene][1]*copies);
        assert.equal(config.jointCapacity,counts[scene][2]*copies);
        assert.equal(config.seed,expected.seed);assert.equal(config.substeps,expected.substeps);
        assert.deepEqual([config.gravityX,config.gravityY].map(v=>Number(v.toPrecision(9))),expected.gravity);
        for(const key of ['linearDrag','angularDrag','linearSpeedMax','contactHertz','contactDampingRatio',
          'contactPushVelocityMax','restitutionThreshold','jointHertz','jointDampingRatio',
          'sleepSpeedMax','sleepAngularSpeedMax','sleepTimeMin','friction','restitution'])
          assert.equal(Number(config[key].toPrecision(9)),expected[snake(key)],key);
        assert.equal(config.timestep,Math.fround(1/60));
        assert.ok(world.refreshSnapshot(true));assert.ok(reference.refreshSnapshot(true));
        assert.equal(world.diagnostics.valid,true);
        assert.equal(world.diagnostics.floats.length,4*config.bodyCapacity+5);
        assert.equal(world.diagnostics.words.length,config.bodyCapacity+3);
        assert.equal(world.report().memory.diagnosticBytes,20*config.bodyCapacity+32);
        const camera={pyramid:[-13,-1,13,22],rain:[-9,-1,9,25],chains:[-1,-1,29,16]}[scene];
        assert.deepEqual([config.cameraLeft,config.cameraBottom,config.cameraRight,config.cameraTop],
          [camera[0]-16*(copies-1),camera[1],camera[2]+16*(copies-1),camera[3]]);
        const initial=reference.snapshot, actual=world.snapshot;
        const initialCopy=world.snapshot.copy(),initialReport=world.report();
        for(let copy=0;copy<copies;++copy)for(let row=0;row<initial.bodyCount;++row) {
          const at=copy*initial.bodyCount+row, offset=32*(copy-(copies-1)/2);
          assert.equal(actual.bodies.x[at],Math.fround(initial.bodies.x[row]+offset));
          assert.equal(actual.bodies.y[at],initial.bodies.y[row]);
        }
        const used=module.memory.allocatorUsedBytes;
        const counterStats=world.frameCounters.stats,counterWork=world.frameCounters.work;
        assert.equal(world.frameCounters.valid,false);
        for(let i=0;i<3;++i) {
          assert.ok(world.step());assert.equal(world.snapshot.valid,false);assert.equal(world.diagnostics.valid,false);
          assert.throws(()=>world.snapshot.bodyCount);
          assert.ok(world.refreshSnapshot(true));
          assert.ok(world.refreshFrameCounters());assert.equal(world.frameCounters.valid,true);
          assert.equal(world.frameCounters.stats,counterStats);assert.equal(world.frameCounters.work,counterWork);
          const report=world.report();
          assert.equal(report.finiteState,true);
          for(let k=0;k<13;++k)assert.equal(counterStats[k],report.stats[world.frameCounters.statNames[k]]);
          for(let k=0;k<9;++k)assert.equal(counterStats[13+k],report.last[world.frameCounters.statNames[13+k]]);
          for(let k=0;k<13;++k){assert.equal(counterWork[k],report.work[world.frameCounters.workNames[k]]);assert.equal(counterWork[13+k],report.cumulative[world.frameCounters.workNames[k]]);}
          assert.equal(counterStats[23],used);assert.equal(counterStats[24],i+1);assert.equal(counterWork[25],world.drops);
          counterStats.fill(0xffffffff);counterWork.fill(0xffffffffffffffffn);
          assert.ok(world.refreshFrameCounters());assert.equal(counterStats[24],i+1);assert.equal(counterWork[25],world.drops);
          assert.ok(reference.prepare()&&reference.mutate()&&reference.step()&&reference.sample()&&reference.refreshSnapshot(true));
          if(copies===1) {
            const a=world.snapshot.copy(), b=reference.snapshot.copy();
            assert.deepEqual(a,b);
          }
        }
        assert.equal(module.memory.allocatorUsedBytes,used);
        assert.equal(world.steps,3);assert.equal(world.drops,0n);assert.equal(world.step(),false);
        assert.equal(world.report().configuration,config);
        assert.equal(typeof world.report().work.treeNodeVisits,'bigint');
        assert.ok(world.refreshSnapshot());assert.equal(world.diagnostics.valid,false);const x=world.snapshot.bodies.x[0];
        world.snapshot.bodies.x[0]=999;assert.ok(world.refreshSnapshot());assert.equal(world.snapshot.bodies.x[0],x);
        assert.ok(world.refreshSnapshot(true));
        const bound=world.diagnostics.floats[0], flags=world.diagnostics.words[0];
        world.diagnostics.floats[0]=999;world.diagnostics.words[0]=0xffffffff;
        assert.ok(world.refreshSnapshot(true));
        assert.equal(world.diagnostics.floats[0],bound);assert.equal(world.diagnostics.words[0],flags);
        assert.equal(world.snapshot.bodies.x[0],x);
        const rebuilt=world.rebuild();assert.ok(rebuilt);
        try{
          assert.throws(()=>world.rebuild(),/disposed/);assert.equal(world.frameCounters.valid,false);
          assert.deepEqual(rebuilt.report(),initialReport);assert.ok(rebuilt.refreshSnapshot(true));
          assert.deepEqual(rebuilt.snapshot.copy(),initialCopy);
          assert.equal(module.memory.allocatorUsedBytes,used);assert.ok(rebuilt.step());
        }finally{rebuilt.dispose();}
      } finally {world.dispose();world.dispose();reference.dispose();}
      assert.throws(()=>world.step(),/disposed/);
    }
  }
  const long=module.create('pyramid',{sleep:true,steps:10001});assert.ok(long);
  for(let i=0;i<10001;++i)assert.ok(long.step());
  assert.equal(long.step(),false);assert.equal(long.steps,10001);assert.equal(long.drops,0n);long.dispose();
  const orphan=module.create('chains');assert.ok(orphan);module.dispose();
  assert.throws(()=>orphan.refreshSnapshot(),/disposed/);orphan.dispose();
}finally{module.dispose();benchmark.dispose();}
const small=await createStudyModule({memoryBytes:2*1024*1024});
try {
  assert.equal(small.create('rain',{copies:16}),null);
  const used=small.memory.allocatorUsedBytes;
  for(let i=0;i<8;++i)assert.equal(small.create('rain',{copies:16}),null);
  assert.equal(small.memory.allocatorUsedBytes,used);
  const recovered=small.create('pyramid',{steps:1});assert.ok(recovered);assert.ok(recovered.step());recovered.dispose();
}finally{small.dispose();}
console.log('Study module: all physics tiers, settings, snapshot replay, ownership, fixed allocations, >10000 steps and exhaustion PASS');
