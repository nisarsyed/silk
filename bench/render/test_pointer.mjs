import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {createStudyModule}=await import(pathToFileURL(path.join(build,'physics/driver.mjs')));
const module=await createStudyModule();
try{
  for(const scene of ['pyramid','rain','chains'])for(const sleep of [false,true]){
    let a=module.create(scene,{sleep,steps:12}),b=module.create(scene,{sleep,steps:12});assert.ok(a&&b);
    try{
      assert.ok(a.refreshSnapshot());assert.ok(b.refreshSnapshot());const snapshot=a.snapshot.copy(),row=snapshot.bodies.type.findIndex(value=>value===0);
      assert.ok(row>=0);const x=snapshot.bodies.x[row]+.02,y=snapshot.bodies.y[row];
      const pointer=a.pointerState;assert.equal(pointer,a.pointerState);assert.equal(pointer.held,false);
      assert.equal(pointer.bodyIndex,0xffffffff);assert.equal(pointer.bodyGeneration,0);
      for(const world of [a,b]){
        assert.ok(world.pointer(0,x,y));assert.equal(world.pointerState.held,true);
        assert.equal(world.pointerState.bodyIndex,snapshot.bodies.index[row]);
        assert.equal(world.pointerState.bodyGeneration,snapshot.bodies.generation[row]);
        assert.ok(!world.snapshot.valid);assert.equal(world.steps,0);
      }
      const before=a.report();
      for(const action of [-1,4,1.5,NaN,'0'])assert.throws(()=>a.pointer(action,x,y));
      for(const value of [NaN,Infinity,-Infinity,8193,-8193,'1',null,undefined]){
        assert.throws(()=>a.pointer(1,value,y));assert.throws(()=>a.pointer(1,x,value));
      }
      assert.equal(a.pointer(0,x,y),false);assert.deepEqual(a.report(),before);
      for(let step=0;step<12;++step){
        for(const world of [a,b]){
          assert.ok(world.pointer(step<8?1:step===8?2:3,x+.1*(step+1),y+.5));
          assert.ok(world.step());assert.ok(world.refreshSnapshot(true));
        }
        assert.deepEqual(a.snapshot.copy(),b.snapshot.copy());assert.deepEqual(a.report(),b.report());
      }
      assert.equal(a.pointer(0,x,y),false);assert.equal(a.pointerState.held,false);
      const old=a;a=a.rebuild();assert.ok(a);assert.throws(()=>pointer.held,/disposed/);old.dispose();
      assert.equal(a.steps,0);assert.equal(a.pointerState.held,false);assert.equal(a.pointerState.bodyIndex,0xffffffff);
      assert.ok(a.pointer(0,8192,8192));assert.equal(a.pointerState.held,true);assert.equal(a.pointerState.bodyIndex,0xffffffff);
      assert.ok(a.pointer(3,0,0));assert.ok(a.pointer(3,0,0));assert.equal(a.pointerState.held,false);
    }finally{a?.dispose();b?.dispose();}
  }
  const scaled=module.create('pyramid',{copies:2,steps:1});assert.ok(scaled);
  try{const before=scaled.report();assert.equal(scaled.pointer(0,0,0),false);assert.deepEqual(scaled.report(),before);}
  finally{scaled.dispose();}
  for(const scene of ['pyramid','rain','chains']){
    let user=module.create(scene,{user:true}),base=module.create(scene,{steps:3});
    assert.ok(user&&base);
    try{
      assert.equal(user.configuration.userMode,true);
      assert.deepEqual([user.configuration.bodyCapacity,user.configuration.contactCapacity,user.configuration.jointCapacity],
        [4096,32768,96]);
      assert.ok(user.refreshSnapshot());assert.ok(base.refreshSnapshot());
      assert.equal(user.snapshot.bodyCount,base.snapshot.bodyCount);
      for(let row=0;row<base.snapshot.bodyCount;++row){
        assert.equal(user.snapshot.bodies.x[row],base.snapshot.bodies.x[row]);
        assert.equal(user.snapshot.bodies.y[row],base.snapshot.bodies.y[row]);
      }
      assert.throws(()=>user.spawn(2,0,20));
      assert.throws(()=>user.spawn(0,NaN,20));
      assert.throws(()=>base.spawn(0,0,20),/Spawning requires/);
      const initial=user.snapshot.bodyCount,before=module.memory.allocatorUsedBytes;
      assert.equal(user.spawn(0,0,20),true);
      assert.equal(user.spawn(1,1,20),true);
      assert.ok(user.refreshSnapshot());assert.equal(user.snapshot.bodyCount,initial+2);
      assert.ok(user.step());assert.ok(user.refreshSnapshot());
      assert.equal(module.memory.allocatorUsedBytes,before);
      const old=user;user=user.rebuild();assert.ok(user);
      assert.equal(user.configuration.userMode,true);assert.ok(user.refreshSnapshot());
      assert.equal(user.snapshot.bodyCount,initial);old.dispose();
    }finally{user?.dispose();base?.dispose();}
  }
  const baseline=module.memory.allocatorUsedBytes;
  for(let i=0;i<12;++i){
    const world=module.create(['pyramid','rain','chains'][i%3],{user:true});assert.ok(world);
    try{assert.ok(world.spawn(i%2,0,20));assert.ok(world.step());}
    finally{world.dispose();}
    assert.equal(module.memory.allocatorUsedBytes,baseline);
  }
  console.log('Pointer owner: all default fixtures/sleep modes, exact replay, bounded coordinates, no step on input, misses, release/cancel and rebuild lifetime PASS');
}finally{module.dispose();}
