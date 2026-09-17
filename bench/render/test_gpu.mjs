import assert from 'node:assert/strict';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const build=path.resolve(process.argv[2]??'build/render-study');
const {GpuTimer,queryCapacityMax}=await import(pathToFileURL(path.join(build,'gpu.js')));
const {FrameRecorder,numberNames,requiredNumbers}=await import(pathToFileURL(path.join(build,'recording.js')));
const ext={TIME_ELAPSED_EXT:1,GPU_DISJOINT_EXT:2,QUERY_COUNTER_BITS_EXT:3};
function fake({supported=true,bits=64,allocationLimit=Infinity,current=null}={}){
  const queries=[],events=[];
  return {queries,events,CURRENT_QUERY:4,QUERY_RESULT_AVAILABLE:5,QUERY_RESULT:6,disjoint:false,current,
    getExtension(name){assert.equal(name,'EXT_disjoint_timer_query_webgl2');return supported?ext:null;},
    getQuery(target,pname){assert.equal(target,1);return pname===3?bits:this.current;},
    createQuery(){events.push('create');if(queries.length===allocationLimit)return null;const q={ready:false,value:0,deleted:false};queries.push(q);return q;},
    deleteQuery(q){assert.equal(q.deleted,false);q.deleted=true;events.push('delete');},
    beginQuery(target,q){assert.equal(target,1);assert.equal(this.current,null);assert.equal(q.deleted,false);q.ready=false;this.current=q;events.push('begin');},
    endQuery(target){assert.equal(target,1);assert.ok(this.current);this.current=null;events.push('end');},
    getQueryParameter(q,pname){assert.equal(q.deleted,false);events.push(pname===5?'available':'result');
      if(pname===5)return q.ready;assert.equal(q.ready,true,'Blocking result read');return q.value;},
    getParameter(pname){assert.equal(pname,2);events.push('disjoint');const value=this.disjoint;this.disjoint=false;return value;}
  };
}
const counters={valid:true,stats:new Uint32Array(25),work:new BigUint64Array(26),
  statNames:Array.from({length:25},(_,i)=>String(i)),workNames:Array.from({length:13},(_,i)=>String(i))};
function frame(recorder,now){const v=new Float64Array(numberNames.length);v[0]=v[1]=v[2]=now;v[3]=now+1;v[4]=now+2;
  recorder.append(v,requiredNumbers,counters);recorder.finish(now+3);}
for(const invalid of [0,-1,.5,NaN,queryCapacityMax+1])assert.throws(()=>new GpuTimer(null,10,invalid));
for(const gl of [null,fake({supported:false}),fake({bits:0}),fake({bits:29}),fake({current:{}}),fake({allocationLimit:1})]){
  const timer=new GpuTimer(gl,4,2);assert.equal(timer.supported,false);assert.ok(timer.unavailableReason);
  assert.equal(timer.begin(0,0),false);assert.equal(timer.statuses[0],6);timer.poll(new FrameRecorder(4),1);
  timer.dispose();timer.dispose();assert.throws(()=>timer.begin(1,2));
  if(gl)assert.ok(gl.queries.every(q=>q.deleted));
}
const gl=fake(),timer=new GpuTimer(gl,8,2),recorder=new FrameRecorder(8);
assert.equal(timer.supported,true);assert.equal(timer.bytes,8+2*20);assert.equal(gl.queries.length,2);
const identity=timer.statuses;
assert.equal(timer.begin(0,0),true);assert.throws(()=>timer.begin(1,0));assert.throws(()=>timer.poll(recorder,0));timer.end();frame(recorder,0);
timer.poll(recorder,4);assert.equal(timer.pendingCount,1);assert.equal(recorder.known[0]&(1<<14),0);
assert.equal(timer.begin(1,10),true);timer.end();frame(recorder,10);
assert.equal(timer.begin(2,20),false);assert.equal(timer.statuses[2],3);frame(recorder,20);
gl.queries[0].ready=gl.queries[1].ready=true;gl.queries[0].value=4294967297;gl.queries[1].value=250000;
gl.events.length=0;timer.poll(recorder,30);
assert.deepEqual(gl.events,['available','result','available','result','disjoint']);
assert.equal(recorder.numbers[14],4294967297/1e6);assert.equal(recorder.numbers[numberNames.length+14],.25);
assert.equal(timer.pendingCount,0);assert.equal(timer.statuses,identity);assert.deepEqual(Array.from(timer.statuses.slice(0,3)),[2,2,3]);
// A later disjoint event invalidates both ready and not-yet-ready queries, but
// cannot invalidate samples already covered by the earlier disjoint check.
timer.begin(3,40);timer.end();frame(recorder,40);timer.begin(4,50);timer.end();frame(recorder,50);
gl.queries[0].ready=true;gl.queries[0].value=100;gl.disjoint=true;timer.poll(recorder,60);
assert.deepEqual(Array.from(timer.statuses.slice(0,5)),[2,2,3,4,4]);assert.equal(timer.pendingCount,0);
assert.equal(recorder.known[0]&(1<<14),1<<14);assert.equal(recorder.known[3]&(1<<14),0);
timer.begin(5,70);timer.end();frame(recorder,70);timer.stop(false);assert.equal(timer.statuses[5],8);
assert.throws(()=>timer.begin(6,80));assert.throws(()=>timer.poll(recorder,80));
const report=timer.report();assert.equal(report.disjointEvents,1);assert.equal(report.maximumPending,2);assert.equal(report.pending,0);
assert.equal(report.counts[2],2);assert.equal(report.counts[3],1);assert.equal(report.counts[4],2);assert.equal(report.counts[8],1);
timer.dispose();timer.dispose();assert.ok(gl.queries.every(q=>q.deleted));assert.equal(gl.events.filter(x=>x==='create').length,0);
// Invalid/overflowing values and possibly wrapped short counters are unknown.
for(const [bits,value,elapsed]of [[64,null,10],[64,NaN,10],[64,-1,10],[64,1.5,10],[64,2**53,10],[30,1,1100],[30,2**30-1,10]]){
  const g=fake({bits}),t=new GpuTimer(g,1,1),r=new FrameRecorder(1);t.begin(0,0);t.end();frame(r,0);
  g.queries[0].ready=true;g.queries[0].value=value;t.poll(r,elapsed);assert.equal(t.statuses[0],5);assert.equal(r.known[0]&(1<<14),0);t.dispose();
}
const interruptedGl=fake(),interrupted=new GpuTimer(interruptedGl,1,1);
interrupted.begin(0,0);interrupted.stop(true);assert.equal(interrupted.statuses[0],7);assert.equal(interruptedGl.current,null);interrupted.dispose();
const lostGl=fake(),lost=new GpuTimer(lostGl,1,1),lostRecorder=new FrameRecorder(1);
lost.begin(0,0);lost.end();frame(lostRecorder,0);lostGl.queries[0].ready=true;lostGl.disjoint=null;
lost.poll(lostRecorder,10);assert.equal(lost.statuses[0],5);assert.equal(lostRecorder.known[0]&(1<<14),0);lost.dispose();
console.log('GPU timers: bounded reuse, nonblocking availability, full-precision results, disjoint invalidation, overflow, missing metrics and disposal PASS');
