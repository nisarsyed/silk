import {makeViews,workCopy,memoryNames,statNames,stepNames,workNames,jsonReplacer} from './views.mjs';
export {jsonReplacer};
export const physicsTiers=Object.freeze([1,2,4,8,16]);
export const stepLimit=21600;
const fixtures=Object.freeze({pyramid:0,rain:1,chains:3});
const settingNames=['gravityX','gravityY','linearDrag','angularDrag','linearSpeedMax',
  'contactHertz','contactDampingRatio','contactPushVelocityMax','restitutionThreshold',
  'jointHertz','jointDampingRatio','sleepSpeedMax','sleepAngularSpeedMax','sleepTimeMin',
  'friction','restitution','timestep','cameraLeft','cameraBottom','cameraRight','cameraTop'];
const object=(names,values,offset=0)=>Object.fromEntries(names.map((name,i)=>[name,values[offset+i]]));
const frameStatNames=Object.freeze([...statNames,...stepNames,'lastContactDrops','allocatorUsedBytes','completedSteps']);
const frameWorkNames=Object.freeze([...workNames]);
function integer(value,low,high,name) {
  if(!Number.isInteger(value)||value<low||value>high) throw new RangeError(`Invalid ${name}`);
}
// Private shared owner for standalone physics and the co-located raylib module.
// The optional renderer callback receives borrowed pointers only in its closure;
// disposal closes its graphics resources before freeing the authoritative world.
export function ownStudyModule(m,memoryBytes,attachRenderer) {
  const owners=new Set(),rendererTargets=new WeakMap();let disposed=false;
  const live=()=>{if(disposed)throw new Error('Study module is disposed');};
  const moduleOwner=Object.freeze({
    get memory(){live();return {linearMemoryBytes:memoryBytes,stackBytes:1048576,staticEnd:m._sl_wasm_static_end(),
      heapBase:m._sl_wasm_heap_base(),allocatorUsedBytes:m._sl_wasm_allocator_used()};},
    create(scene,{copies=1,sleep=false,steps=stepLimit}={}) {
      live();
      if(typeof scene!=='string'||!Object.hasOwn(fixtures,scene)||typeof sleep!=='boolean') throw new TypeError('Invalid fixture or sleep policy');
      if(!physicsTiers.includes(copies))throw new RangeError('Invalid physics tier');
      integer(steps,1,stepLimit,'step bound');
      let ptr=m._sl_render_study_create(fixtures[scene],copies,sleep?1:0,steps);
      if(!ptr)return null;
      try {
        const adapter=m._sl_render_study_adapter(ptr),buffer=m.HEAPU32.buffer;
        const io={ptr:adapter,f:new Float32Array(buffer,m._sl_wasm_input_f32(adapter),32),
          of:new Float32Array(buffer,m._sl_wasm_output_f32(adapter),64),
          ou:new Uint32Array(buffer,m._sl_wasm_output_u32(adapter),32)};
        const status=new Uint32Array(buffer,m._sl_render_study_status(ptr),10);
        const pointerStatus=new Uint32Array(buffer,m._sl_render_study_pointer_status(ptr),3);
        const settings=new Float32Array(buffer,m._sl_render_study_settings(ptr),21);
        const configuration=Object.freeze({scene,copies,sleepEnabled:sleep,stepLimit:steps,
          bodyCapacity:status[5],contactCapacity:status[6],jointCapacity:status[7],substeps:status[8],seed:status[9],
          ...object(settingNames,settings)});
        const views=makeViews(m,io,configuration,()=>{throw new Error('Study queries return indices, not mutable handles');});
        const capacity=configuration.bodyCapacity;
        const nativeDiagnosticFloats=new Float32Array(buffer,m._sl_render_study_diagnostic_f32(ptr),4*capacity+5);
        const nativeDiagnosticWords=new Uint32Array(buffer,m._sl_render_study_diagnostic_u32(ptr),capacity+3);
        const diagnosticFloats=new Float32Array(4*capacity+5),diagnosticWords=new Uint32Array(capacity+3);
        const nativeStats=new Uint32Array(buffer,io.ou.byteOffset,23);
        const frameStats=new Uint32Array(25),frameWork=new BigUint64Array(26);
        let frameValid=false,frameRevision=0;
        const frameCounters=Object.freeze({stats:frameStats,work:frameWork,statNames:frameStatNames,workNames:frameWorkNames,
          get valid(){return frameValid;},get revision(){return frameRevision;}});
        let diagnosticsValid=false;
        const diagnostics=Object.freeze({floats:diagnosticFloats,words:diagnosticWords,
          get valid(){return diagnosticsValid;}});
        const valid=()=>{live();if(!ptr)throw new Error('Study world is disposed');};
        const invalidate=()=>{frameValid=false;diagnosticsValid=false;views.state.valid=false;views.queryState.valid=false;views.rayState.valid=false;};
        const revision=state=>{if(state.revision===Number.MAX_SAFE_INTEGER)throw new RangeError('View revision exhausted');++state.revision;};
        const pointerState=Object.freeze({
          get held(){valid();return !!pointerStatus[0];},
          get bodyIndex(){valid();return pointerStatus[1];},
          get bodyGeneration(){valid();return pointerStatus[2];}
        });
        let renderer;
        const value=Object.freeze({
          ...(attachRenderer?{createRenderer(options={}){valid();if(renderer)throw new Error('Renderer already attached');
            renderer=attachRenderer(ptr,value,valid,options);return renderer;}}:{}),
          // Consume this world and rebuild its exact initial configuration.
          // Retain warmed graphics within this synchronous ownership operation,
          // freeing physics before allocation so two arenas never coexist.
          // On failure the old world is disposed and retained graphics close.
          rebuild(){valid();const retained=renderer;renderer=undefined;value.dispose();let next;
            try{next=moduleOwner.create(scene,{copies,sleep,steps});
              if(!next){retained?.dispose();return null;}
              if(retained)rendererTargets.get(next)(retained,configuration);
              return next;
            }catch(error){next?.dispose();retained?.dispose();throw error;}},
          configuration,diagnostics,frameCounters,pointerState,
          pointer(action,x,y){
            valid();integer(action,0,3,'pointer action');
            if(!Number.isFinite(x)||!Number.isFinite(y)||Math.abs(x)>8192||Math.abs(y)>8192)
              throw new RangeError('Invalid pointer world coordinate');
            invalidate();return !!m._sl_render_study_pointer(ptr,action,Math.fround(x),Math.fround(y));
          },
          get snapshot(){valid();return views.snapshot;},
          get steps(){valid();return status[4];},
          get drops(){valid();return m._sl_render_study_drops(ptr);},
          step(){valid();invalidate();return !!m._sl_render_study_step(ptr);},
          // Setup-only, independent copied columns for displayed frozen
          // instances. Native temporary storage is released before returning;
          // the result remains usable after this source world is disposed.
          frozenDiagnostics(instances){
            valid();integer(instances,256,65536,'render tier');
            if((instances&(instances-1))!==0)throw new RangeError('Invalid render tier');
            const frozen=m._sl_render_frozen_create(ptr,instances);
            if(!frozen)return null;
            try{
              if(m._sl_render_frozen_count(frozen)!==instances)throw new Error('Frozen instance count mismatch');
              return Object.freeze({valid:true,instances,
                floats:new Float32Array(new Float32Array(buffer,m._sl_render_frozen_f32(frozen),4*instances+5)),
                words:new Uint32Array(new Uint32Array(buffer,m._sl_render_frozen_u32(frozen),instances+3)),
                temporaryNativeBytes:m._sl_render_frozen_bytes(frozen)});
            }finally{m._sl_render_frozen_destroy(frozen);}
          },
          // Fixed 308-byte copied output: 25 uint32 stats and 26 uint64 work
          // counters. No per-frame objects or temporary subarray views.
          refreshFrameCounters(){
            valid();frameValid=false;
            if(frameRevision===Number.MAX_SAFE_INTEGER)throw new RangeError('Counter revision exhausted');
            ++frameRevision;
            if(!m._sl_wasm_stats_read(adapter))return false;
            frameStats.set(nativeStats);frameStats[23]=m._sl_wasm_allocator_used();frameStats[24]=status[4];
            frameWork.set(views.work);frameValid=true;return true;
          },
          refreshSnapshot(diagnostics=false){
            valid();if(typeof diagnostics!=='boolean')throw new TypeError('Invalid diagnostics flag');
            invalidate();revision(views.state);
            if(!m._sl_wasm_snapshot_refresh(adapter,diagnostics?1:0))return false;
            views.refreshCopy(io.ou[3],io.ou[1],io.ou[2]);
            views.state.bodyCount=io.ou[0];views.state.contactCount=io.ou[1];views.state.jointCount=io.ou[2];
            views.state.geometryUpdates=io.ou[3];
            if(diagnostics){
              if(!m._sl_render_study_diagnostics(ptr))return false;
              diagnosticFloats.set(nativeDiagnosticFloats);diagnosticWords.set(nativeDiagnosticWords);diagnosticsValid=true;
            }
            views.state.valid=true;
            return true;
          },
          // Reporting intentionally allocates copied objects. Keep it outside
          // measured steady-state frames; the collector will use bounded arrays.
          report(){
            valid();m._sl_wasm_stats_read(adapter);
            const stats=object(statNames,io.ou),last=object(stepNames,io.ou,13);
            const work=workCopy(views.work),cumulative=workCopy(views.work,13);
            m._sl_wasm_memory_read(adapter);
            return {configuration,steps:status[4],drops:m._sl_render_study_drops(ptr),finiteState:!m._sl_render_study_failed(ptr),stats,last,work,cumulative,
              memory:{...object(memoryNames,io.ou),adapterBytes:m._sl_wasm_adapter_bytes(adapter),
                contextBytes:m._sl_render_study_bytes(),diagnosticBytes:m._sl_render_study_diagnostic_bytes(ptr),
                outputBytes:views.outputBytes+diagnosticFloats.byteLength+diagnosticWords.byteLength+frameStats.byteLength+frameWork.byteLength}};
          },
          dispose(){if(ptr){renderer?.dispose();invalidate();m._sl_render_study_destroy(ptr);ptr=0;owners.delete(value);}}
        });
        rendererTargets.set(value,(previous,configuration)=>{
          valid();if(renderer)throw new Error('Target already owns a renderer');
          if(JSON.stringify(configuration)!==JSON.stringify(value.configuration))throw new Error('Renderer transfer requires identical fixtures');
          renderer=attachRenderer(ptr,value,valid,{},previous);return renderer;
        });
        owners.add(value);return value;
      }catch(error){m._sl_render_study_destroy(ptr);if(error instanceof RangeError)return null;throw error;}
    },
    dispose(){if(!disposed){for(const owner of owners)owner.dispose();disposed=true;}}
  });
  return moduleOwner;
}
