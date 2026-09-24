import {runStudy} from './runner.mjs';

const byId=id=>document.getElementById(id);
const form=byId('controls'),start=byId('start'),download=byId('download');
const phase=byId('phase'),detail=byId('detail'),stage=byId('stage'),hud=byId('hud');
const smoke=new URL(location.href).searchParams.get('smoke')==='1';
const rotations=[['canvas','webgl','raylib'],['webgl','raylib','canvas'],
  ['raylib','canvas','webgl'],['canvas','raylib','webgl'],['raylib','webgl','canvas']];
const controls=[...form.querySelectorAll('select,input'),...document.querySelectorAll('.notes select,.notes input')];
byId('layout').value=innerWidth<650?'mobile':'desktop';
let saved=true,report=null,locked=false;
const write=(name,message)=>{phase.textContent=name;detail.textContent=message;};
const optionalNumber=id=>{const field=byId(id);return field.value===''?null:Number(field.value);};
const selection=()=>{
  const profile=byId('profile').value;
  const frozen=profile==='frozen'||profile==='frozen-diagnostic';
  const sustained=profile==='sustained';
  const interaction=profile==='interaction';
  const diagnostic=profile==='diagnostic'||profile==='frozen-diagnostic';
  return {profile,frozen,sustained,interaction,diagnostic};
};
const sync=()=>{
  if(locked)return;
  const {profile,frozen,sustained}=selection();
  if(frozen||sustained){byId('scene').value='rain';byId('sleep').checked=false;}
  if(frozen||sustained||profile==='interaction')byId('copies').value='1';
  if(sustained)byId('layout').value='mobile';
  byId('scene').disabled=frozen||sustained;
  byId('sleep').disabled=frozen||sustained;
  byId('copies').disabled=frozen||sustained||profile==='interaction';
  byId('instances').disabled=!frozen;
  byId('layout').disabled=sustained;
  byId('mode').textContent=smoke?'SHORT CORRECTNESS':sustained?'300 SECOND RUN':'60 SECOND RUN';
  const order=rotations[Number(byId('repeat').value)-1];
  const slot=order.indexOf(byId('candidate').value)+1;
  write('READY',`Repeat ${byId('repeat').value}: ${order.join(' → ')}. Selected candidate is slot ${slot}. ${profile==='interaction'?'Drag 100 times across 20 gestures.':'Keep the tab foreground during measurement.'}`);
};
for(const id of ['profile','candidate','repeat'])byId(id).addEventListener('change',sync);
sync();
const setLocked=value=>{
  locked=value;
  if(value){for(const control of controls)control.disabled=true;}
  else{for(const control of controls)control.disabled=false;sync();}
};
const conditions=(profile,candidate)=>{
  const repeat=Number(byId('repeat').value),order=rotations[repeat-1];
  return {source:'operator-entered',recordedAtIso:new Date().toISOString(),
    profile,repeat,candidateOrder:order,candidateSlot:order.indexOf(candidate)+1,
    deviceLabel:byId('device').value.trim(),power:byId('power').value,
    batteryPercent:optionalNumber('battery'),brightnessPercent:optionalNumber('brightness'),
    ambientCelsius:optionalNumber('ambient'),thermalState:byId('thermal').value,
    lowPowerMode:byId('lowPower').checked,
    display:{screenWidth:screen.width,screenHeight:screen.height,deviceDpr:devicePixelRatio},
    note:'Reported conditions are operator entries; browser userAgent is in the raw study report.'};
};
form.addEventListener('submit',async event=>{
  event.preventDefault();
  if(!saved||!form.reportValidity())return;
  const choice=selection(),candidate=byId('candidate').value,scene=byId('scene').value,
    layout=byId('layout').value,sleep=byId('sleep').checked,copies=Number(byId('copies').value),
    instances=choice.frozen?Number(byId('instances').value):undefined,
    memoryBytes=Number(byId('memory').value)*1024*1024;
  const recordedConditions=conditions(choice.profile,candidate);
  saved=false;report=null;start.disabled=true;download.disabled=true;setLocked(true);
  document.body.classList.add('measuring');
  if(layout==='mobile')scrollTo(0,0);
  const canvas=document.createElement('canvas');canvas.id='collection-canvas';
  canvas.setAttribute('aria-label','Silk renderer study');stage.replaceChildren(canvas);
  hud.hidden=!choice.diagnostic;hud.textContent='';
  write('INITIALIZING','Keep this tab visible. Calibration and disposable warm-up precede measurement.');
  try{
    report=await runStudy({canvas,candidate,scene,layout,sleep,copies,instances,
      diagnostic:choice.diagnostic,sustained:choice.sustained,interaction:choice.interaction,
      memoryBytes,...(choice.diagnostic?{hudElement:hud}:{}),
      ...(smoke?{correctnessSeconds:1}:{}),
      onPhase:value=>{
        if(value==='measurement')write(choice.interaction?'DRAG NOW':'MEASURING',choice.interaction
          ?'Drag moving bodies: at least 100 applied moves across 20 gestures in this separate trial.'
          :'Keep the tab foreground and avoid interacting with the canvas.');
        else if(value==='calibration')write('CALIBRATING','Measuring 240 idle display intervals.');
        else if(value==='warmup')write('WARMING',choice.sustained?'Disposable rain scene: 60 seconds of foreground warm-up.':'Disposable scene: 120 steps and render callbacks.');
        else if(value==='rebuild')write('RESETTING','Restoring the exact initial source scene.');
        else if(value==='gpu-drain')write('FINISHING','Resolving nonblocking GPU timers.');
      }});
    report.collectionConditions=recordedConditions;
    document.body.classList.remove('measuring');
    const input=report.input;
    write(report.status==='collected'?'SAVE REPORT':'RUN FAILED',report.status==='collected'
      ?input?`${input.trustedSubmittedMoves} trusted submitted moves in ${input.trustedGesturesWithSubmittedMoves} gestures. Timestamp precision remains unverified.`
        :`${report.frames.count} complete frames. Save the raw report for independent audit.`
      :`${report.failure?.phase??'unknown'}: ${report.failure?.reason??'Unknown failure'}. Save the failed record.`);
    download.disabled=false;
  }catch(error){document.body.classList.remove('measuring');saved=true;start.disabled=false;setLocked(false);
    write('SETUP ERROR',String(error));}
});
download.addEventListener('click',()=>{
  if(!report)return;
  const data=JSON.stringify(report,(_key,value)=>typeof value==='bigint'?String(value):value);
  const blob=new Blob([data+'\n'],{type:'application/json'});
  const url=URL.createObjectURL(blob),link=document.createElement('a');
  const c=report.collectionConditions;
  link.href=url;link.download=`silk-r${c.repeat}-${c.profile}-${report.configuration.candidate}-${report.configuration.scene}-${Date.now()}.json`;
  document.body.append(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
  saved=true;start.disabled=false;setLocked(false);
  write('DOWNLOAD REQUESTED','Confirm the raw JSON is on your device before starting the next trial.');
});
