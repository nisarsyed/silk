import {runStudy} from './runner.mjs';

const byId=id=>document.getElementById(id);
const form=byId('controls'),start=byId('start'),download=byId('download');
const phase=byId('phase'),detail=byId('detail'),stage=byId('stage');
const smoke=new URL(location.href).searchParams.get('smoke')==='1';
byId('mode').textContent=smoke?'SHORT CORRECTNESS':'60 SECOND RUN';
byId('layout').value=innerWidth<650?'mobile':'desktop';
let saved=true,report=null;
const write=(name,message)=>{phase.textContent=name;detail.textContent=message;};
const optionalNumber=id=>{const field=byId(id);return field.value===''?null:Number(field.value);};
const conditions=()=>({source:'operator-entered',recordedAtIso:new Date().toISOString(),
  deviceLabel:byId('device').value.trim(),power:byId('power').value,
  batteryPercent:optionalNumber('battery'),brightnessPercent:optionalNumber('brightness'),
  ambientCelsius:optionalNumber('ambient'),thermalState:byId('thermal').value,
  lowPowerMode:byId('lowPower').checked,
  display:{screenWidth:screen.width,screenHeight:screen.height,deviceDpr:devicePixelRatio},
  note:'Reported conditions are operator entries; browser userAgent is in the raw study report.'});
form.addEventListener('submit',async event=>{
  event.preventDefault();
  if(!saved)return;
  saved=false;report=null;start.disabled=true;download.disabled=true;
  if(!form.reportValidity()){saved=true;start.disabled=false;return;}
  const candidate=byId('candidate').value,scene=byId('scene').value,
    layout=byId('layout').value,sleep=byId('sleep').checked;
  const recordedConditions=conditions();
  const canvas=document.createElement('canvas');canvas.id='collection-canvas';
  canvas.setAttribute('aria-label','Interactive Silk renderer study');
  stage.replaceChildren(canvas);
  write('INITIALIZING','Keep this tab visible. Calibration and disposable warm-up run before input recording.');
  try{
    report=await runStudy({canvas,candidate,scene,layout,sleep,interaction:true,
      ...(smoke?{correctnessSeconds:1}:{}),
      onPhase:value=>{
        if(value==='measurement')write('DRAG NOW','Drag moving bodies: at least 100 applied moves across 20 gestures in this separate trial.');
        else if(value==='calibration')write('CALIBRATING','Measuring 240 idle display intervals.');
        else if(value==='warmup')write('WARMING','Disposable scene: 120 steps and render callbacks.');
        else if(value==='rebuild')write('RESETTING','Restoring the exact initial falling scene.');
        else if(value==='gpu-drain')write('FINISHING','Resolving nonblocking GPU timers.');
      }});
    report.collectionConditions=recordedConditions;
    const input=report.input;
    write(report.status==='collected'?'SAVE REPORT':'RUN FAILED',report.status==='collected'
      ?`${input.trustedSubmittedMoves} trusted submitted moves in ${input.trustedGesturesWithSubmittedMoves} gestures. Timestamp precision remains unverified.`
      :`${report.failure?.phase??'unknown'}: ${report.failure?.reason??'Unknown failure'}. Save the failed record.`);
    download.disabled=false;
  }catch(error){saved=true;start.disabled=false;
    write('SETUP ERROR',String(error));}
});
download.addEventListener('click',()=>{
  if(!report)return;
  const data=JSON.stringify(report,(_key,value)=>typeof value==='bigint'?String(value):value);
  const blob=new Blob([data+'\n'],{type:'application/json'});
  const url=URL.createObjectURL(blob),link=document.createElement('a');
  link.href=url;link.download=`silk-input-${report.configuration.candidate}-${report.configuration.scene}-${Date.now()}.json`;
  document.body.append(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
  saved=true;start.disabled=false;
  write('DOWNLOAD REQUESTED','Confirm the raw JSON is on your device before starting the next trial.');
});
