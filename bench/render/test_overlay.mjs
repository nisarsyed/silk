import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {createStudyModule}=await import(pathToFileURL(path.join(build,'physics/driver.mjs')));
const {DrawScene}=await import(pathToFileURL(path.join(build,'geometry.js')));
const {Overlay,markerVertices,markerTriangles}=await import(pathToFileURL(path.join(build,'overlay.js')));
const module=await createStudyModule({memoryBytes:256*1024*1024});
try{
  for(const scene of ['pyramid','rain','chains'])for(const copies of [1,16]){
    const world=module.create(scene,{copies,steps:120});assert.ok(world);
    try{
      assert.ok(world.refreshSnapshot(true));const config=world.configuration;
      const draw=new DrawScene(world.snapshot,scene,undefined,copies);
      const overlay=new Overlay(draw,config.bodyCapacity,config.contactCapacity,config.jointCapacity,1280,720);
      assert.throws(()=>overlay.refresh(world.snapshot,{...world.diagnostics,valid:false}));
      const lines=overlay.lines,markers=overlay.markers,colors=overlay.bodyColors;
      overlay.refresh(world.snapshot,world.diagnostics);
      for(let i=0;i<draw.count;++i)assert.equal(colors[i],world.snapshot.bodies.type[draw.rows[i]]===2?0:1);
      if(scene==='chains'&&copies===1){
        // The first two centers are (0,14) and (0,13); local anchors
        // (0,-0.4)/(0,+0.6) meet near (0,13.6), far from the origin.
        const at=4*104*5,v=overlay.view;
        assert.ok(Math.abs(lines[at]-v.x)<1e-5);
        assert.ok(Math.abs(lines[at+1]-(v.y-13.6*v.scale))<1e-4);
        assert.ok(Math.abs(lines[at+3]-(v.y-13.6*v.scale))<1e-4);
        assert.equal(lines[at+4],14);
      }
      for(let i=0;i<120;++i)assert.ok(world.step());assert.ok(world.refreshSnapshot(true));draw.copyTransforms(world.snapshot);
      overlay.refresh(world.snapshot,world.diagnostics);
      const snapshot=world.snapshot,d=world.diagnostics,n=config.bodyCapacity;
      let proxies=0,contacts=0;
      for(let i=0;i<snapshot.bodyCount;++i){
        const slot=snapshot.bodies.index[i];if(d.words[slot]&8)++proxies;
      }
      for(let i=0;i<snapshot.contactCount;++i)contacts+=snapshot.contacts.pointCount[i];
      const hit=d.words[n+2];
      assert.equal(overlay.lineCount,4*proxies+contacts+snapshot.jointCount+5+hit);
      assert.equal(overlay.markerCount,contacts+2*snapshot.jointCount+1+hit);
      for(let i=4*proxies;i<4*proxies+contacts;++i){
        const at=5*i;assert.equal(lines[at+4],12);
        assert.ok(Math.abs(Math.hypot(lines[at+2]-lines[at],lines[at+3]-lines[at+1])-12)<0.001,`${scene} copies=${copies} normal=${i-4*proxies} length=${Math.hypot(lines[at+2]-lines[at],lines[at+3]-lines[at+1])}`);
      }
      let row=0;
      for(let i=0;i<snapshot.bodyCount;++i){
        const slot=snapshot.bodies.index[i];if(!(d.words[slot]&8))continue;
        assert.equal(lines[row*5+4],d.words[slot]&7?15:13);row+=4;
      }
      overlay.refresh(snapshot,d);assert.equal(overlay.lines,lines);assert.equal(overlay.markers,markers);assert.equal(overlay.bodyColors,colors);
      const copy=snapshot.copy();const dynamic=copy.bodies.type.findIndex(t=>t===0);copy.bodies.awake[dynamic]=0;
      overlay.refresh(copy,d);assert.equal(colors[dynamic],2);
      if(scene==='chains'){
        copy.joints.bodyAGeneration[0]++;assert.throws(()=>overlay.refresh(copy,d),/endpoint/);
      }
      // Directly exercise overflow rejection: no primitive is silently dropped.
      overlay.lineCount=lines.length/5;assert.throws(()=>overlay.line(0,0,1,1,13),/capacity/);
      overlay.markerCount=markers.length/3;assert.throws(()=>overlay.marker(0,0,11),/capacity/);
    }finally{world.dispose();}
  }
  assert.equal(markerVertices.length,64);assert.equal(markerTriangles.length,180);
  assert.equal(markerVertices[0],1);assert.equal(markerVertices[1],0);
  console.log('Overlay commands: bounded counts, 12px normals, query highlights, joint frames, colors and reused buffers PASS');
}finally{module.dispose();}
