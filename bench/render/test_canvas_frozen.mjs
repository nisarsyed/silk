import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {CanvasCandidate,frozenPathEdgeLimit}=await import(pathToFileURL(path.join(build,'canvas.js')));
const {palette}=await import(pathToFileURL(path.join(build,'overlay.js')));
// Count the geometry submitted through Canvas APIs, independently of its
// private cache layout. This catches a return to one giant concatenated path.
class CountPath {
  edges=0;primitives=0;
  moveTo(){}lineTo(){++this.edges;}closePath(){++this.edges;++this.primitives;}
  addPath(source){this.edges+=source.edges;this.primitives+=source.primitives;}
}
globalThis.Path2D=CountPath;globalThis.DOMMatrix=class{};
try{
  const count=1024,lines=4097,markers=1025,poses=new Float32Array(count*4);
  for(let i=0;i<count;++i)poses[4*i+2]=1;
  const vertices=new Float32Array(64);
  for(let i=0;i<32;++i){vertices[2*i]=Math.cos(i*2*Math.PI/32);vertices[2*i+1]=Math.sin(i*2*Math.PI/32);}
  const scene={frozen:true,poses,meshes:[{vertices}],batches:[{first:0,count,mesh:0,color:1}],rect:[-9,-1,9,25]};
  const overlay={bodyColors:new Float32Array(count),lines:new Float32Array(lines*5),markers:new Float32Array(markers*3),lineCount:lines,markerCount:markers};
  const expected=[];
  for(let i=0;i<count;++i){const color=i<900?3:4;overlay.bodyColors[i]=color;expected.push(palette[color]);}
  for(let i=0;i<lines;++i){const color=i<3000?13:15;overlay.lines.set([0,0,1,0,color],i*5);expected.push(palette[color]);}
  for(let i=0;i<markers;++i){const color=i<900?11:15;overlay.markers.set([0,0,color],i*3);expected.push(palette[color]);}
  const fills=[],context={fillStyle:null,setTransform(){},fillRect(){},fill(shape){fills.push({edges:shape.edges,count:shape.primitives,color:this.fillStyle});}};
  const renderer=new CanvasCandidate({width:1280,height:720,getContext:()=>context},scene,overlay);
  renderer.draw();assert.equal(frozenPathEdgeLimit,8192);assert.ok(fills.every(f=>f.edges<=frozenPathEdgeLimit));
  const actual=[];let edges=0;
  for(const fill of fills){edges+=fill.edges;for(let i=0;i<fill.count;++i)actual.push(fill.color);}
  assert.deepEqual(actual,expected);assert.equal(edges,32*(count+markers)+4*lines);
  assert.equal(renderer.submissionCalls,fills.length+1);
  const previous=structuredClone(fills);fills.length=0;renderer.draw();assert.deepEqual(fills,previous);
  renderer.dispose();assert.throws(()=>renderer.draw(),/Disposed/);
  console.log('Frozen Canvas: bounded cached paths, complete primitive/color order, repeated submission and disposal PASS');
}finally{delete globalThis.Path2D;delete globalThis.DOMMatrix;}
