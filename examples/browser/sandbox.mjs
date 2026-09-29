import {startBrowserRuntime} from './runtime_controller.mjs';
import {createPointerAdapter} from './pointer_adapter.mjs';
import {provenanceJson} from './provenance.mjs';
import {camera,sceneRects} from './geometry.js';

const byId=id=>document.getElementById(id);
const canvas=byId('world'),connection=byId('connection'),notice=byId('notice');
const sceneButtons=[...document.querySelectorAll('[data-scene]')];
const spawnButtons=[...document.querySelectorAll('[data-spawn]')];
const sceneNames=Object.freeze({pyramid:'PYRAMID / 01',rain:'RAIN / 02',chains:'CHAINS / 03'});
const layouts=Object.freeze({desktop:'1280 × 720',mobile:'720 × 1280'});
const provenance=JSON.parse(provenanceJson);
byId('revision').textContent=`BUILD ${provenance.source.revision.slice(0,7)}`;

let runtime=null,pointer=null,busy=true,failed=false,full=false,paused=false,scene='pyramid',sleep=false;
let place=null,placementCanvas=null;
const overlays={contacts:false,proxies:false,joints:false,queries:false,islands:false};
let resizePending=false,resizeRunning=false,ended=false,overloadShown=false,contactShown=false;
const layout=()=>innerWidth<780||innerHeight>innerWidth?'mobile':'desktop';
const showNotice=(title,detail,recover=false)=>{
  byId('notice-title').textContent=title;byId('notice-detail').textContent=detail;
  byId('recover').hidden=!recover;notice.hidden=false;
};
const setSpawnHint=value=>{
  byId('spawn-hint').textContent=value;
  byId('mobile-spawn-hint').textContent=value;
};
const clearNotice=()=>{notice.hidden=true;};
const setConnection=(label,state)=>{
  connection.className=`connection ${state}`;
  connection.lastChild.textContent=` ${label}`;
};
const setBusy=value=>{
  busy=value;
  const disabled=value||failed;
  for(const button of sceneButtons)button.disabled=disabled;
  byId('toggle').disabled=disabled;byId('step').disabled=disabled||!paused;
  byId('reset').disabled=disabled;byId('sleep').disabled=disabled;
  for(const button of spawnButtons)button.disabled=disabled||full;
  for(const input of document.querySelectorAll('[data-overlay]'))input.disabled=disabled;
};
const syncControls=()=>{
  for(const button of sceneButtons){const selected=button.dataset.scene===scene;
    button.classList.toggle('active',selected);button.setAttribute('aria-pressed',String(selected));}
  byId('scene-label').textContent=sceneNames[scene];
  byId('layout-label').textContent=layouts[runtime?.layout??layout()];
  byId('toggle').textContent=paused?'Resume':'Pause';
  byId('sleep').checked=sleep;
  for(const button of spawnButtons){const selected=Number(button.dataset.spawn)===place;
    button.setAttribute('aria-pressed',String(selected));}
  runtime?.canvas.classList.toggle('placing',place!==null);
  for(const input of document.querySelectorAll('[data-overlay]'))input.checked=overlays[input.dataset.overlay];
  setBusy(busy);
};
const operation=async action=>{
  if(busy||!runtime)return;
  setBusy(true);
  try{const result=await action();if(result!==false)clearNotice();syncControls();}
  catch(error){showNotice('Action failed',String(error),failed||runtime.mode==='failed');}
  finally{setBusy(false);syncControls();}
};
const status=value=>{
  if(value.state==='failed'){
    failed=true;
    setConnection('Needs restart','error');
    showNotice('World stopped',value.reason||'The browser could not continue the simulation.',true);
    setBusy(true);
  }else if(value.state==='running'){
    setConnection('World running','ready');
  }else if(value.state==='paused'){
    setConnection('World paused','ready');
  }
};
const requestLayout=()=>{
  if(!runtime||ended||failed)return;
  resizePending=true;
  if(resizeRunning)return;
  resizeRunning=true;
  void (async()=>{
    try{while(resizePending){resizePending=false;
      const wanted=layout();
      if(wanted!==runtime.layout){await runtime.resize(wanted);byId('layout-label').textContent=layouts[wanted];}
    }}catch(error){showNotice('Resize failed',String(error),failed||runtime.mode==='failed');}
    finally{resizeRunning=false;}
  })();
};

for(const button of sceneButtons)button.addEventListener('click',()=>operation(async()=>{
  const next=button.dataset.scene;
  if(next===scene)return;
  if(!await runtime.reset(next,sleep))throw new Error('The scene could not be loaded');
  scene=next;place=null;full=false;overloadShown=false;contactShown=false;
  setSpawnHint('Choose a shape, then tap the world.');
}));
for(const button of spawnButtons)button.addEventListener('click',()=>{
  if(busy||failed||full)return;
  const selected=Number(button.dataset.spawn);
  place=place===selected?null:selected;
  setSpawnHint(place===null?'Choose a shape, then tap the world.':
    `Tap the world to place a ${place===0?'circle':'box'}.`);
  syncControls();
});
const placeBody=event=>{
  if(place===null||!runtime||!event.isPrimary||event.button!==0)return;
  event.stopImmediatePropagation();event.preventDefault();
  if(busy||failed||full)return;
  const rect=runtime.canvas.getBoundingClientRect();
  const [width,height]=runtime.layout==='mobile'?[720,1280]:[1280,720];
  const view=camera(sceneRects[scene],width,height);
  const px=Math.max(0,Math.min(width,(event.clientX-rect.left)*width/rect.width));
  const py=Math.max(0,Math.min(height,(event.clientY-rect.top)*height/rect.height));
  const x=Math.fround((px-view.x)/view.scale),y=Math.fround((view.y-py)/view.scale);
  const kind=place;
  void operation(async()=>{
    const result=await runtime.spawn(kind,x,y);
    if(result==='full'){
      full=true;place=null;
      setSpawnHint('Body capacity reached. Reset to make room.');
      showNotice('World is full','The 4,096-body interaction limit has been reached. Reset to make room.');
      return false;
    }
    if(result!=='created')throw new Error('The body could not be created');
    place=null;
    setSpawnHint('Choose a shape, then tap the world.');
    await updateTelemetry();
  });
};
const attachPlacement=()=>{
  if(placementCanvas)placementCanvas.removeEventListener('pointerdown',placeBody,true);
  placementCanvas=runtime.canvas;
  placementCanvas.addEventListener('pointerdown',placeBody,true);
};
byId('toggle').addEventListener('click',()=>operation(async()=>{
  if(paused){await runtime.resume();paused=false;}
  else{await runtime.pause();paused=true;}
}));
byId('step').addEventListener('click',()=>operation(async()=>{
  if(!paused||!await runtime.singleStep())throw new Error('Pause before stepping');
}));
byId('reset').addEventListener('click',()=>operation(async()=>{
  if(!await runtime.reset(scene,sleep))throw new Error('The world could not be reset');
  place=null;full=false;overloadShown=false;contactShown=false;
  setSpawnHint('Choose a shape, then tap the world.');
}));
byId('sleep').addEventListener('change',event=>{
  const next=event.target.checked;
  void operation(async()=>{
    if(!await runtime.reset(scene,next))throw new Error('Sleep setting could not be applied');
    sleep=next;place=null;full=false;overloadShown=false;contactShown=false;
    setSpawnHint('Choose a shape, then tap the world.');
  });
});
for(const input of document.querySelectorAll('[data-overlay]'))input.addEventListener('change',()=>operation(async()=>{
  const next={...overlays,[input.dataset.overlay]:input.checked};
  if(!await runtime.setOverlays(next))throw new Error('The diagnostic layer could not be changed');
  Object.assign(overlays,next);
}));
byId('recover').addEventListener('click',async()=>{
  if(!runtime)return;
  setBusy(true);
  try{pointer?.dispose();
    const rebuild=failed||runtime.mode==='failed';
    if(rebuild)await runtime.recoverToMain();
    else{
      if(!await runtime.reset(scene,sleep))throw new Error('The world could not be restarted');
      if((await runtime.report()).state==='paused')await runtime.resume();
    }
    pointer=createPointerAdapter(runtime,{onError:error=>showNotice('Input stopped',error,true)});
    attachPlacement();
    scene=runtime.scene;if(rebuild)sleep=false;
    paused=false;failed=false;full=false;place=null;overloadShown=false;contactShown=false;
    setSpawnHint('Choose a shape, then tap the world.');
    clearNotice();syncControls();
  }catch(error){showNotice('Restart failed',String(error),true);}
  finally{setBusy(false);syncControls();}
});
window.addEventListener('resize',requestLayout);
window.addEventListener('orientationchange',requestLayout);
window.addEventListener('pagehide',()=>{ended=true;pointer?.dispose();
  placementCanvas?.removeEventListener('pointerdown',placeBody,true);
  void runtime?.dispose();},{once:true});

const updateTelemetry=async()=>{
  if(ended||!runtime||failed||runtime.mode==='failed')return;
  try{const report=await runtime.report();
    byId('steps').textContent=Number(report.steps??0).toLocaleString();
    byId('counts').textContent=`${report.worldStats.bodyCount} / ${report.worldStats.contactCount}`;
    byId('work').textContent=report.worldWork.pairProbes.toLocaleString();
    if(report.worldStats.bodyCount>=report.bodyCapacity){
      full=true;place=null;
      setSpawnHint('Body capacity reached. Reset to make room.');
      syncControls();
    }
    const dropped=report.clock.droppedSeconds;
    byId('dropped').textContent=`${dropped.toFixed(2)} s`;
    byId('memory').textContent=`${(report.memory.linearMemoryBytes/1048576).toFixed(0)} MiB`;
    if(report.contactDrops!=='0'&&!contactShown){contactShown=true;
      showNotice('Contact capacity reached','Some contacts could not be stored in this world. Reset or choose a lighter scene.');}
    if(dropped>.25&&!overloadShown){overloadShown=true;
      showNotice('Device overloaded','This scene is dropping simulation time on this device. Try a lighter scene.');}
  }catch(error){showNotice('Telemetry unavailable',String(error),runtime.mode==='failed');}
};

syncControls();
try{
  runtime=await startBrowserRuntime(canvas,{candidate:'canvas',scene,sleep,userMode:true,layout:layout()},
    {preferWorker:false,onStatus:status});
  pointer=createPointerAdapter(runtime,{onError:error=>showNotice('Input stopped',error,true)});
  attachPlacement();
  byId('layout-label').textContent=layouts[runtime.layout];
  clearNotice();setBusy(false);syncControls();
  await updateTelemetry();
  setInterval(()=>{void updateTelemetry();},250);
}catch(error){
  setConnection('Could not load','error');
  showNotice('Engine unavailable',String(error));
  setBusy(true);
}
