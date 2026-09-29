import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {createStudyModule}=await import(pathToFileURL(path.join(build,'physics/driver.mjs')));
const {DrawScene,renderTiers}=await import(pathToFileURL(path.join(build,'geometry.js')));
const {FrozenOverlay}=await import(pathToFileURL(path.join(build,'frozen.js')));
const module=await createStudyModule(),world=module.create('rain',{steps:121});assert.ok(world);
try{
  assert.equal(world.frozenDiagnostics(256),null);
  for(const count of [0,128,255,257,65537,NaN,Infinity])assert.throws(()=>world.frozenDiagnostics(count));
  for(let i=0;i<120;++i)assert.ok(world.step());assert.ok(world.refreshSnapshot(true));
  const source=world.snapshot.copy(),before=world.report(),b=source.bodies,c=source.contacts;
  let retained;
  for(const count of renderTiers){
    const scene=new DrawScene(source,'rain',count),diagnostics=world.frozenDiagnostics(count);assert.ok(diagnostics);
    assert.equal(diagnostics.floats.length,4*count+5);assert.equal(diagnostics.words.length,count+3);
    assert.ok(diagnostics.temporaryNativeBytes>=20*count+32);
    const overlay=new FrozenOverlay(scene,source,diagnostics,1280,720);
    const slots=new Set(),tiles=Math.ceil(count/2000);let points=0,contacts=0,staticContacts=0,trimmedContacts=0;
    for(let tile=0;tile<tiles;++tile){
      slots.clear();
      for(let i=tile*2000;i<Math.min(count,(tile+1)*2000);++i)slots.add(b.index[scene.rows[i]]);
      for(let row=0;row<source.contactCount;++row){
        if(!c.pointCount[row]||(!slots.has(c.bodyAIndex[row])&&!slots.has(c.bodyBIndex[row])))continue;
        ++contacts;
        if(b.type[b.index.indexOf(c.bodyAIndex[row])]===2||b.type[b.index.indexOf(c.bodyBIndex[row])]===2)++staticContacts;
        else if(!slots.has(c.bodyAIndex[row])||!slots.has(c.bodyBIndex[row]))++trimmedContacts;
        const offset=32*(tile-(tiles-1)/2);
        for(let point=0;point<c.pointCount[row];++point){
          const x=Math.fround((point?c.pointX1[row]:c.pointX0[row])+offset),y=point?c.pointY1[row]:c.pointY0[row];
          assert.equal(overlay.markers[3*points],Math.fround(overlay.view.x+x*overlay.view.scale));
          assert.equal(overlay.markers[3*points+1],Math.fround(overlay.view.y-y*overlay.view.scale));
          assert.equal(overlay.markers[3*points+2],11);++points;
        }
      }
    }
    assert.ok(staticContacts>0);assert.ok(trimmedContacts>0);
    const hit=diagnostics.words[count+2];assert.equal(overlay.drawnContacts,contacts);
    assert.equal(overlay.lineCount,4*count+points+5+hit);assert.equal(overlay.markerCount,points+1+hit);
    for(let i=0;i<count;++i){
      const row=scene.rows[i];assert.equal(overlay.bodyColors[i],!b.awake[row]?2:b.island[row]===0xffffffff?1:3+b.island[row]%8);
      for(let edge=0;edge<4;++edge)assert.equal(overlay.lines[(4*i+edge)*5+4],diagnostics.words[i]&7?15:13);
    }
    assert.throws(()=>overlay.refresh(source,diagnostics),/cannot change/);
    assert.throws(()=>new FrozenOverlay(scene,source,{...diagnostics,instances:count+1},1280,720));
    assert.deepEqual(world.report(),before);retained=diagnostics;
  }
  const original=retained.floats[0];retained.floats[0]=12345;
  assert.equal(world.frozenDiagnostics(retained.instances).floats[0],original);
  const malformed=structuredClone(source);malformed.contacts.bodyAGeneration[0]++;
  assert.throws(()=>new FrozenOverlay(new DrawScene(source,'rain',256),malformed,world.frozenDiagnostics(256),1280,720),/endpoint/);
  assert.ok(world.step());assert.equal(world.frozenDiagnostics(256),null);
  world.dispose();assert.equal(retained.valid,true);assert.equal(retained.floats[0],12345);
  assert.throws(()=>world.frozenDiagnostics(256),/disposed/);
  console.log('Frozen diagnostics: all tiers, retained static/trimmed contacts, command order, copied query lifetime and constant source state PASS');
}finally{world.dispose();module.dispose();}
