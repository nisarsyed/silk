import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import path from 'node:path';
const build=path.resolve(process.argv[2]??'build/render-study');
const {DrawScene,camera,renderTiers}=await import(pathToFileURL(path.join(build,'geometry.js')));
const {edgeMask,compare}=await import(pathToFileURL(path.join(build,'pixels.js')));
const {createBenchmarkModule}=await import(pathToFileURL(path.join(build,'benchmark/driver.mjs')));
const module=await createBenchmarkModule();
try {
  for (const name of ['pyramid','rain','chains']) {
    const world=module.create(name,{warmup:0,steps:120});assert.ok(world);
    try {
      assert.ok(world.refreshSnapshot());
      const scene=new DrawScene(world.snapshot,name);
      assert.equal(scene.batches.reduce((sum,b)=>sum+b.count,0),scene.count);
      let next=0;
      for (const b of scene.batches) { assert.equal(b.first,next); next+=b.count; }
      const before=scene.poses.slice(),storage=scene.poses;
      for (let i=0;i<120;++i) assert.ok(world.prepare()&&world.mutate()&&world.step()&&world.sample());
      assert.ok(world.refreshSnapshot());scene.copyTransforms(world.snapshot);
      assert.equal(scene.poses,storage);assert.notDeepEqual(scene.poses,before);
      const snapshot=world.snapshot;
      for (let i=0;i<scene.count;++i) {
        assert.equal(scene.poses[4*i],snapshot.bodies.x[scene.rows[i]]);
        assert.equal(scene.poses[4*i+1],snapshot.bodies.y[scene.rows[i]]);
      }
      if (name==='rain') {
        for (const count of renderTiers) {
          const frozen=new DrawScene(snapshot,name,count);assert.equal(frozen.count,count);
          assert.equal(frozen.meshes.length,1); assert.equal(frozen.meshes[0].vertices.length,64);
          assert.equal(frozen.meshes[0].vertices[0],snapshot.geometry.radius[snapshot.bodies.index[frozen.rows[0]]]);
          assert.equal(frozen.meshes[0].vertices[1],0);
          assert.equal(frozen.rows[0],3); assert.equal(frozen.rows[count-1],3+(count-1)%2000);
          const tiles=Math.ceil(count/2000);
          assert.equal(frozen.offsets[0],32*(0-(tiles-1)/2));
          assert.equal(frozen.offsets[count-1],16*(tiles-1));
          assert.throws(()=>frozen.copyTransforms(snapshot),/Frozen/);
        }
      }
    } finally { world.dispose(); }
  }
  for (const [width,height] of [[1280,720],[720,1280]]) {
    const view=camera([-9,-1,9,25],width,height);
    assert.ok(view.x-9*view.scale>=0);assert.ok(view.x+9*view.scale<=width);
    assert.ok(view.y-25*view.scale>=-1e-12);assert.ok(view.y+view.scale<=height+1e-12);
  }
  assert.throws(()=>camera([0,0,0,1],1280,720));
  // Analytical one-pixel band: a horizontal edge at y=10 lies 0.5 px from
  // rows 9/10 and 1.5 px from row 8. Same-color occlusion cannot erase it.
  const rectangle={rect:[0,0,20,20],batches:[{first:0,count:1,mesh:0}],
    meshes:[{vertices:new Float32Array([5,5,15,5,15,10,5,10])}],poses:new Float32Array([0,0,1,0])};
  const mask=edgeMask(rectangle,20,20);
  assert.equal(mask[9*20+10],1);assert.equal(mask[10*20+10],1);assert.equal(mask[8*20+10],0);
  const reference=new Uint8Array(20*20*4), candidate=reference.slice();
  candidate[4*(9*20+10)]=255;assert.equal(compare(reference,candidate,20,20,mask).interiorMismatch,0);
  candidate[4*(8*20+10)]=255;assert.equal(compare(reference,candidate,20,20,mask).interiorMismatch,1);
  console.log('Geometry: source order, transforms, all nine scaling tiers, tessellation and camera PASS');
} finally { module.dispose(); }
