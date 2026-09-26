// A headless Linux runner has no GPU. Firefox otherwise blocks WebGL 2 even
// when its software implementation is available; force that test-only path.
// https://bugzilla.mozilla.org/show_bug.cgi?id=1970486
export function launchTestBrowser(engine,name){
  const options={headless:true};
  if(name==='firefox'&&process.platform==='linux')
    options.firefoxUserPrefs={'webgl.force-enabled':true};
  return engine.launch(options);
}
