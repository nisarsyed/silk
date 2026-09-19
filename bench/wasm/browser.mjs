import {runMatrix} from './run.mjs';
const build = await (await fetch('./build.json',{cache:'no-store'})).json();
window.runBenchmark = async options => {
  const runtime={kind:'browser',engine:navigator.userAgent,device:options.device??'unspecified',os:navigator.platform,
    hardware_concurrency:navigator.hardwareConcurrency,automated:options.automated??false,
    visibility_start:document.visibilityState,visibility_end:'pending',process_reused:true,module_per_repeat:true};
  const report=await runMatrix({...options,build,runtime});
  runtime.visibility_end=document.visibilityState;
  if(runtime.visibility_start!=='visible'||runtime.visibility_end!=='visible') {
    report.failures.push({scene:null,stage:'visibility',index:0,message:'Foreground visibility was not maintained',state:null});
    report.execution_status='failed';
  }
  document.querySelector('#status').textContent=report.execution_status;
  return report;
};
document.querySelector('#status').textContent='Ready';
