import {createStudyModule} from './physics/driver.mjs';
import {createRaylibStudyModule} from './raylib.js';
import {DrawScene} from './geometry.js';
import {Overlay} from './overlay.js';
import {CanvasCandidate} from './canvas.js';
import {WebglCandidate} from './webgl.js';
import {StudyClock,calibrate60} from './timing.js';
import {FrameRecorder,numberNames,requiredNumbers} from './recording.js';
import {StudyHud} from './hud.js';

// Developer collection only. Reports deliberately cannot certify acceptance:
// device conditions, source/artifact provenance, real input, GPU instrumentation
// and the complete repeated protocol must be supplied by the study controller.
// Only one run may own a canvas at a time, including asynchronous initialization.
const active=new WeakSet();
export async function runStudy({canvas,hudElement,candidate,scene='pyramid',copies=1,sleep=false,
  diagnostic=false,layout='desktop',sustained=false,memoryBytes=64*1024*1024,correctnessSeconds,signal,onPhase}={}) {
  if(!(canvas instanceof HTMLCanvasElement)||!canvas.isConnected||!canvas.id||document.getElementById(canvas.id)!==canvas||
      !['canvas','webgl','raylib'].includes(candidate)||!['pyramid','rain','chains'].includes(scene)||
      ![1,2,4,8,16].includes(copies)||typeof sleep!=='boolean'||typeof diagnostic!=='boolean'||
      !['desktop','mobile'].includes(layout)||typeof sustained!=='boolean'||
      !Number.isInteger(memoryBytes)||memoryBytes<(candidate==='raylib'?64:2)*1024*1024||
      memoryBytes>512*1024*1024||memoryBytes%65536!==0||
      (signal!==undefined&&!(signal instanceof AbortSignal))||(onPhase!==undefined&&typeof onPhase!=='function')||
      (diagnostic&&!(hudElement instanceof HTMLElement))||
      (sustained&&(scene!=='rain'||sleep||diagnostic||copies!==1||layout!=='mobile'))||
      (correctnessSeconds!==undefined&&(!Number.isFinite(correctnessSeconds)||correctnessSeconds<.25||correctnessSeconds>5)))
    throw new TypeError('Invalid study run configuration');
  if(active.has(canvas))throw new Error('Canvas already has an active study');
  active.add(canvas);
  const duration=correctnessSeconds??(sustained?300:60),width=layout==='mobile'?720:1280,height=layout==='mobile'?1280:720;
  const deviceDpr=devicePixelRatio,viewportWidth=innerWidth,viewportHeight=innerHeight;
  const intervals=new Float64Array(240),values=new Float64Array(numberNames.length);
  let module,world,renderer,drawScene,overlay,hud,recorder,clock,calibration,phase='initialization',failure=null;
  let frameId=0,rejectFrames,initial,final,warmup,setupMs=null,measurementStart=null,measurementEnd=null,calibrationCount=0;
  const began=performance.now();
  const setPhase=value=>{phase=value;onPhase?.(value);};
  const fail=reason=>{
    if(!failure)failure={phase,reason:String(reason),atMs:performance.now()};
    if(rejectFrames){const reject=rejectFrames;rejectFrames=undefined;cancelAnimationFrame(frameId);reject(new Error(failure.reason));}
  };
  const visibility=()=>{if(document.visibilityState!=='visible')fail('Document became hidden');};
  const contextLost=event=>{event.preventDefault();fail('Graphics context lost');};
  const aborted=()=>fail('Run aborted');
  const resized=()=>{if(innerWidth!==viewportWidth||innerHeight!==viewportHeight||devicePixelRatio!==deviceDpr)
    fail('Display or viewport changed; recalibration required');};
  document.addEventListener('visibilitychange',visibility);
  canvas.addEventListener('webglcontextlost',contextLost);
  canvas.addEventListener('contextlost',contextLost);
  window.addEventListener('resize',resized);
  signal?.addEventListener('abort',aborted);
  const check=()=>{if(failure)throw new Error(failure.reason);if(signal?.aborted)throw new Error('Run aborted');
    if(document.visibilityState!=='visible')throw new Error('Document is not foreground');
    if(devicePixelRatio!==deviceDpr)throw new Error('Display ratio changed; recalibration required');};
  // One promise/callback per phase, no per-frame promises or scheduling objects.
  const frames=visit=>new Promise((resolve,reject)=>{
    rejectFrames=reject;
    const next=raf=>{
      const now=performance.now();
      try{check();if(visit(raf,now)){rejectFrames=undefined;resolve();}
        else frameId=requestAnimationFrame(next);
      }catch(error){rejectFrames=undefined;reject(error);}
    };
    frameId=requestAnimationFrame(next);
  });
  const closeWorld=()=>{hud?.dispose();hud=undefined;renderer?.dispose();renderer=undefined;
    world?.dispose();world=undefined;drawScene=undefined;overlay=undefined;};
  const resetHud=()=>{
    if(diagnostic){hud?.dispose();const memory=world.report().memory;
      hud=new StudyHud(hudElement,{linearBytes:module.memory.linearMemoryBytes,arenaBytes:memory.arenaBytes,outputBytes:memory.outputBytes});}
  };
  const openWorld=()=>{
    world=module.create(scene,{copies,sleep,steps:21600});
    if(!world)throw new Error('Fixed study allocation failed');
    if(candidate==='raylib')renderer=world.createRenderer({diagnostic});
    else{
      if(!world.refreshSnapshot(diagnostic))throw new Error('Initial snapshot failed');
      drawScene=new DrawScene(world.snapshot,scene,undefined,copies);
      const c=world.configuration;
      overlay=diagnostic?new Overlay(drawScene,c.bodyCapacity,c.contactCapacity,c.jointCapacity,width,height):undefined;
      overlay?.refresh(world.snapshot,world.diagnostics);
      renderer=candidate==='canvas'?new CanvasCandidate(canvas,drawScene,overlay):new WebglCandidate(canvas,drawScene,overlay);
    }
    if(canvas.width!==width||canvas.height!==height)throw new Error('Renderer changed drawing-buffer size');
    resetHud();
  };
  const step=count=>{for(let i=0;i<count;++i)if(!world.step())throw new Error('Study step failed (numeric state or step bound)');};
  const prepare=()=>{if(drawScene){if(!world.refreshSnapshot(diagnostic))throw new Error('Snapshot failed');
    drawScene.copyTransforms(world.snapshot);overlay?.refresh(world.snapshot,world.diagnostics);}};
  try{
    check();canvas.width=width;canvas.height=height;
    canvas.style.width=`${layout==='mobile'?360:width}px`;canvas.style.height=`${layout==='mobile'?640:height}px`;
    module=candidate==='raylib'?await createRaylibStudyModule(canvas,{memoryBytes}):await createStudyModule({memoryBytes});
    check();setupMs=performance.now()-began;setPhase('calibration');
    let previous;
    await frames(raf=>{if(previous!==undefined)intervals[calibrationCount++]=raf-previous;previous=raf;return calibrationCount===240;});
    calibration=calibrate60(intervals);
    if(!calibration.supports60Hz)throw new Error('Display calibration does not support 59–61 Hz target');
    setPhase('warmup');openWorld();
    let callbacks=0;
    const warmClock=sustained?new StudyClock(calibration,world.configuration.timestep):null;
    const warmStart=performance.now();
    await frames((raf,now)=>{
      if(warmClock&&!warmClock.tick(raf,now))return false;
      step(warmClock?warmClock.steps:1);prepare();renderer.draw();
      if(!world.refreshFrameCounters())throw new Error('Warm-up counters failed');
      hud?.update(now,world.frameCounters,renderer);++callbacks;
      return warmClock?warmClock.elapsedSeconds>=60:callbacks===120;
    });
    warmup={callbacks,elapsedMs:performance.now()-warmStart,world:world.report(),
      droppedSeconds:warmClock?.droppedSeconds??0};
    // Retain warmed paths, shaders and buffers while replacing the exact C
    // initial state. The consumed world's physics arena is freed first.
    setPhase('rebuild');
    world=world.rebuild();
    if(!world)throw new Error('Fixed allocation failed while rebuilding initial state');
    prepare();resetHud();initial=world.report();
    clock=new StudyClock(calibration,world.configuration.timestep);
    recorder=new FrameRecorder(Math.ceil(duration*1000/calibration.targetPeriodMs)+2);
    setPhase('measurement');
    await frames((raf,now)=>{
      if(canvas.width!==width||canvas.height!==height)throw new Error('Drawing-buffer size changed');
      if(!clock.tick(raf,now))return false;
      if(measurementStart===null)measurementStart=now;
      values.fill(0);values[0]=raf;values[1]=now;values[2]=clock.targetRafMs;
      let t=performance.now();step(clock.steps);values[5]=performance.now()-t;
      if(world.steps!==clock.totalSteps)throw new Error('Executed steps differ from the fixed-step clock');
      if(drawScene&&clock.steps>0){
        t=performance.now();if(!world.refreshSnapshot(diagnostic))throw new Error('Snapshot failed');values[6]=performance.now()-t;
        t=performance.now();drawScene.copyTransforms(world.snapshot);overlay?.refresh(world.snapshot,world.diagnostics);values[7]=performance.now()-t;
      }
      t=performance.now();renderer.draw();values[3]=performance.now();values[8]=values[3]-t;
      t=performance.now();if(!world.refreshFrameCounters())throw new Error('Frame counters failed');values[9]=performance.now()-t;
      t=performance.now();hud?.update(now,world.frameCounters,renderer);values[10]=performance.now()-t;
      values[12]=clock.debtSeconds;values[13]=clock.droppedSeconds;
      let known=requiredNumbers;
      // The optional field names match renderer counters. Missing fields remain
      // unavailable; GPU duration is supplied only by a future asynchronous query.
      for(let i=15;i<numberNames.length;++i){const value=renderer[numberNames[i]];
        if(value!==undefined&&value!==null){values[i]=value;known|=1<<i;}}
      values[4]=performance.now();recorder.append(values,known,world.frameCounters);recorder.finish(performance.now());
      measurementEnd=recorder.numbers[(recorder.count-1)*numberNames.length+4];
      return clock.elapsedSeconds>=duration;
    });
    setPhase('complete');
  }catch(error){fail(error instanceof Error?error.message:error);}
  finally{
    cancelAnimationFrame(frameId);rejectFrames=undefined;
    document.removeEventListener('visibilitychange',visibility);
    canvas.removeEventListener('webglcontextlost',contextLost);canvas.removeEventListener('contextlost',contextLost);
    window.removeEventListener('resize',resized);
    signal?.removeEventListener('abort',aborted);
    try{if(world)final=world.report();}catch(error){fail(`Final report: ${error}`);}
  }
  // Build the potentially large JSON envelope only after timed collection stops.
  let report;
  try{
    report={schema:1,kind:correctnessSeconds===undefined?'study-collection':'correctness-only',acceptanceEligible:false,
      status:failure?'failed':'collected',failure,phase,
      configuration:{candidate,scene,copies,sleep,diagnostic,layout,sustained,memoryBytes,durationSeconds:duration,
        width,height,cssWidth:layout==='mobile'?360:width,cssHeight:layout==='mobile'?640:height,
        renderDpr:layout==='mobile'?2:1,deviceDpr,viewportWidth,viewportHeight,worker:false},
      clock:{timeOrigin:performance.timeOrigin,unit:'milliseconds',source:'performance.now and requestAnimationFrame'},
      browser:{userAgent:navigator.userAgent},setupMs,calibration,calibrationIntervals:Array.from(intervals.subarray(0,calibrationCount)),
      warmup,rendererRecreatedAfterWarmup:initial?false:null,initial,final,moduleMemory:module?.memory,measurementStart,measurementEnd,
      elapsedSeconds:clock?.elapsedSeconds??0,debtSeconds:clock?.debtSeconds??0,droppedSeconds:clock?.droppedSeconds??0,
      summary:recorder?.summary(calibration.targetPeriodMs),frames:recorder?.report(world.frameCounters),
      missingEvidence:['provenance','device conditions','real input','GPU instrumentation','memory profiling',
        'sustained windows','full-quality companion runs','five-repeat protocol']};
  }finally{try{closeWorld();module?.dispose();}finally{active.delete(canvas);}}
  return report;
}
