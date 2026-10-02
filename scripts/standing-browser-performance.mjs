import assert from 'node:assert/strict';
import { closeSync, existsSync, mkdirSync, mkdtempSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { spawn, spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { tmpdir } from 'node:os';
import { sha256 } from './capture-physics-baseline.mjs';
import { inventory, json, verifyInventory } from './coordinated-standing-evidence.mjs';

// This script deliberately has no browser-library dependency. It starts an isolated
// installed Chromium browser and uses its real DevTools protocol over Node's WebSocket.
export function browserExecutable(environment=process.env) {
  const candidates=environment.STANDING_BROWSER_EXECUTABLE?[environment.STANDING_BROWSER_EXECUTABLE]:[
    'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
    'C:/Program Files/Microsoft/Edge/Application/msedge.exe',
    'C:/Program Files/Google/Chrome/Application/chrome.exe',
    '/usr/bin/chromium','/usr/bin/chromium-browser','/usr/bin/google-chrome'];
  return candidates.find(path=>existsSync(path))??null;
}

export function assessBrowserRun(run,manifest) {
  const requiredTicks=manifest.runtime.warmupSteps+Math.ceil(manifest.memory.soakS/manifest.physical.dtS);
  const complete=run.completedTicks>=requiredTicks&&run.observedWallMs>=manifest.memory.soakS*1000
    &&run.runtime?.fullUpdateMs?.count>=requiredTicks-manifest.runtime.warmupSteps
    &&run.runtime?.controllerMs?.count===run.runtime?.fullUpdateMs?.count
    &&Number.isFinite(run.runtime?.fullUpdateMs?.max)&&Number.isFinite(run.runtime?.controllerMs?.max)
    &&Number.isInteger(run.runtime?.updateDeadlineMisses)&&Number.isInteger(run.runtime?.controllerDeadlineMisses)
    &&run.runtime.updateDeadlineMisses>=0&&run.runtime.controllerDeadlineMisses>=0
    &&run.memory?.samples?.length>=2&&run.memory?.wasmMeasured===true
    &&Number.isFinite(run.memory?.maximumRss)&&Number.isFinite(run.memory?.growthAfterWarmup)
    &&run.actualRenderer===run.renderer&&run.runtime?.renderMs?.count>0&&run.canvasPresent===true&&run.flatFloor===true
    &&run.controllerId===manifest.controller.id&&run.browser?.product&&run.browser?.userAgent;
  const explicitFailure=run.firstFailure||run.runtime?.updateDeadlineMisses>manifest.runtime.allowedDeadlineMisses
    ||run.runtime?.controllerDeadlineMisses>manifest.runtime.allowedDeadlineMisses
    ||run.runtime?.fullUpdateMs?.max>manifest.runtime.fullUpdateBudgetMs
    ||run.runtime?.controllerMs?.max>manifest.runtime.controllerDeadlineMs
    ||run.memory?.maximumRss>manifest.memory.maximumProcessRssMiB*2**20
    ||run.memory?.growthAfterWarmup>manifest.memory.maximumPostWarmupGrowthMiB*2**20;
  return explicitFailure?'fail':complete?'pass':'incomplete';
}

export const browserHarness=String.raw`
const config=JSON.parse(document.getElementById('configuration').textContent);
const renderer=new URLSearchParams(location.search).get('renderer');
const m=config.manifest;
const probe=window.__standingProbe={state:'loading',renderer,ticks:0,observedWallMs:0,firstFailure:null};
const memories=new Set();
for(const method of ['instantiate','instantiateStreaming']) {
  const original=WebAssembly[method];
  if(!original)continue;
  WebAssembly[method]=async function(...args) {
    const result=await original.apply(this,args),instance=result.instance??result;
    for(const value of Object.values(instance.exports))if(value instanceof WebAssembly.Memory)memories.add(value);
    return result;
  };
}
try {
  const [{createEmbodiedCharacter},{coordinatedStandingOptions},{DemoRuntime},{checkGraphicsCapabilities}]=await Promise.all([
    import('/src/character/index.ts'),import('/src/character/standing-selection.ts'),
    import('/src/demo/DemoRuntime.ts'),import('/src/scene/capabilities.ts')]);
  const graphics=checkGraphicsCapabilities();
  if(!graphics.canvas2d||(renderer==='webgl'&&!graphics.webgl2))throw new Error('Required actual renderer unavailable: '+renderer);
  const character=await createEmbodiedCharacter(renderer,coordinatedStandingOptions(config.controllerId,config.selectedOffsetM));
  const runtime=new DemoRuntime({host:document.getElementById('host'),character,renderer,capabilities:{...graphics,
    browser:navigator.userAgent,devicePixelRatio,viewport:{width:innerWidth,height:innerHeight}}});
  runtime.stop();
  const candidate=character.coordinatedStanding;
  if(!candidate||!runtime.loop||runtime.current.room||runtime.current.playground)throw new Error('Flat-floor candidate runtime unavailable');
  const durations=[],controllerDurations=[],frames=[],wasmSamples=[];
  let controllerMs=0,controllerCalls=0,warmupAt=null,settled=null;
  const update=candidate.update.bind(candidate);
  candidate.update=(...args)=>{const start=performance.now();try{return update(...args);}finally{controllerMs+=performance.now()-start;controllerCalls++;}};
  const fixed=runtime.loop.fixedUpdate;
  const render=runtime.loop.render;
  const required=m.runtime.warmupSteps+Math.ceil(m.memory.soakS/m.physical.dtS);
  const distance=(a,b)=>Math.hypot(a.x-b.x,a.z-b.z);
  const norm=v=>Math.hypot(v.x,v.y,v.z);
  const fail=reason=>{probe.firstFailure??={tick:probe.ticks,reason};};
  const percentile=values=>{if(!values.length)return null;const sorted=[...values].sort((a,b)=>a-b);return {count:values.length,
    p50:sorted[Math.ceil(sorted.length*.5)-1],p95:sorted[Math.ceil(sorted.length*.95)-1],p99:sorted[Math.ceil(sorted.length*.99)-1],max:sorted.at(-1)};};
  const finish=()=>{
    runtime.stop();
    probe.state='done';probe.completedTicks=probe.ticks;probe.controllerId=runtime.current.diagnostics.coordinatedStanding?.id;
    probe.actualRenderer=runtime.view.mode;probe.canvasPresent=Boolean(runtime.host.querySelector('canvas'));
    probe.viewClass=runtime.view.constructor.name;probe.flatFloor=!runtime.current.room&&!runtime.current.playground;
    probe.runtime={fullUpdateMs:percentile(durations),controllerMs:percentile(controllerDurations),renderMs:percentile(frames),
      updateDeadlineMisses:durations.filter(n=>n>m.runtime.fullUpdateBudgetMs).length,
      controllerDeadlineMisses:controllerDurations.filter(n=>n>m.runtime.controllerDeadlineMs).length,
      loop:runtime.timing,qualification:'Real DemoRuntime FixedStepLoop callback, including shared characterFrame and application bookkeeping; render timing reported separately.'};
    probe.wasmSamples=wasmSamples;probe.wasmMeasured=wasmSamples.length>1&&wasmSamples.every(sample=>sample.bytes>0);
    probe.finalDiagnostics=runtime.current.diagnostics;
    window.__standingDispose=()=>runtime.dispose();
  };
  runtime.loop.fixedUpdate=dt=>{
    if(probe.state==='done')return;
    controllerMs=0;controllerCalls=0;
    const started=performance.now();
    try{fixed(dt);}catch(error){fail(String(error));finish();return;}
    const elapsed=performance.now()-started;
    probe.ticks++;
    const snapshot=runtime.current,d=snapshot.diagnostics;
    if(!d.finite||d.errors.length)fail('Nonfinite state or diagnostics error');
    if(probe.ticks===m.runtime.warmupSteps){warmupAt=performance.now();settled=snapshot;}
    if(probe.ticks>m.runtime.warmupSteps){
      durations.push(elapsed);controllerDurations.push(controllerMs);
      if(controllerCalls!==1)fail('Candidate did not execute exactly once in the fixed update');
      if(d.coordinatedStanding?.mode!=='investigating'||d.coordinatedStanding?.id!==config.controllerId)fail('Candidate handed off or identity changed during browser standing');
      if(elapsed>m.runtime.fullUpdateBudgetMs||controllerMs>m.runtime.controllerDeadlineMs)fail('Inherited runtime deadline missed');
      if(!m.official.states.includes(snapshot.state)||snapshot.support.planted.length!==m.official.plantedFeet)fail('Standing state/support failed');
      if(d.stepCount!==settled.diagnostics.stepCount)fail('Additional step during undisturbed browser soak');
      if(distance(snapshot.rootPosition,settled.rootPosition)>m.official.pelvisEndpointM)fail('Pelvis excursion exceeded inherited limit');
      for(const segment of snapshot.segments){
        if(norm(segment.linearVelocity)>m.official.allSegmentLinearMps||norm(segment.angularVelocity)>m.official.allSegmentAngularRadps)fail('Segment speed exceeded inherited limit');
        if(['leftFoot','rightFoot'].includes(segment.id)&&distance(segment.position,settled.segments.find(s=>s.id===segment.id).position)>m.official.eachFootEndpointM)fail('Foot excursion exceeded inherited limit');
      }
      probe.observedWallMs=performance.now()-warmupAt;
    }
    if(probe.ticks%m.memory.sampleEveryTicks===0)wasmSamples.push({tick:probe.ticks,bytes:[...memories].reduce((sum,memory)=>sum+memory.buffer.byteLength,0),jsHeapBytes:performance.memory?.usedJSHeapSize??null});
    if(probe.firstFailure||(probe.ticks>=required&&probe.observedWallMs>=m.memory.soakS*1000))finish();
  };
  runtime.loop.render=(...args)=>{const start=performance.now();render(...args);if(probe.ticks>m.runtime.warmupSteps)frames.push(performance.now()-start);};
  probe.state='running';runtime.loop.start();
}catch(error){probe.state='error';probe.error=error.stack??String(error);}
`;

const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function connectCdp(url) {
  const socket=new WebSocket(url),pending=new Map();let sequence=0;
  await new Promise((resolve,reject)=>{socket.addEventListener('open',resolve,{once:true});socket.addEventListener('error',reject,{once:true});});
  socket.addEventListener('message',event=>{
    const message=JSON.parse(String(event.data)),item=pending.get(message.id);if(!item)return;
    pending.delete(message.id);clearTimeout(item.timer);
    if(message.error)item.reject(new Error(JSON.stringify(message.error)));else item.resolve(message.result);
  });
  socket.addEventListener('close',()=>{for(const item of pending.values()){clearTimeout(item.timer);item.reject(new Error('Browser CDP closed'));}pending.clear();});
  return {close:()=>socket.close(),send:(method,params={},sessionId)=>new Promise((resolve,reject)=>{
    const id=++sequence,timer=setTimeout(()=>{pending.delete(id);reject(new Error('CDP timed out: '+method));},15000);
    pending.set(id,{resolve,reject,timer});socket.send(JSON.stringify({id,method,params,...(sessionId?{sessionId}:{})}));
  })};
}

export function measureProcessMemory(processIds) {
  assert.ok(processIds.length&&processIds.every(id=>Number.isInteger(id)&&id>0),'Invalid browser process inventory');
  if(process.platform!=='win32')return {status:'incomplete',reason:'Total browser process-tree RSS collector is currently implemented for Windows only'};
  const result=spawnSync('powershell.exe',['-NoProfile','-NonInteractive','-Command',
    `$ErrorActionPreference='Stop'; @(Get-Process -Id ${processIds.join(',')} | ForEach-Object { @{ pid=$_.Id; rss=[long]$_.WorkingSet64 } }) | ConvertTo-Json -Compress`],
    {encoding:'utf8',windowsHide:true,timeout:10000});
  if(result.status!==0)return {status:'incomplete',reason:result.stderr||result.error?.message||'RSS collector failed'};
  const parsed=JSON.parse(result.stdout),processes=Array.isArray(parsed)?parsed:[parsed];
  if(processes.length!==processIds.length||processes.some(item=>!Number.isFinite(item.rss)||item.rss<=0))return {status:'incomplete',reason:'Missing browser process RSS'};
  return {status:'pass',rss:processes.reduce((sum,item)=>sum+item.rss,0),processes};
}

export async function runBrowserPerformance(directory) {
  directory=resolve(directory);assert.ok(existsSync(directory),'Acceptance must create the fresh stage directory');
  assert.ok(!existsSync(join(directory,'report.json')),'Retain earlier browser attempts');
  const root=dirname(directory),manifest=JSON.parse(readFileSync(join(root,'experiment.json'))),spec=JSON.parse(readFileSync(join(directory,'scenario.json')));
  const run=JSON.parse(readFileSync(join(root,'run-manifest.json'))),started=performance.now();
  const report={status:'incomplete',controllerId:spec.controllerId,renderers:[],reason:null};
  let server,browser,cdp,logFd;
  const matchesFrozen=()=>Object.entries({...run.source,...run.dependencies,...run.manifestDependencies})
    .every(([path,hash])=>existsSync(path)&&sha256(readFileSync(path))===hash)
    &&sha256(readFileSync(join(root,'experiment.json')))===run.resolvedExperimentSha256;
  try {
    assert.ok(['h78-v1','h79-v1','h80-v1'].includes(spec.controllerId),'Only created contact-force candidates are authorized');
    assert.equal(manifest.controller.id,spec.controllerId);assert.deepEqual(spec.renderers,['canvas2d','webgl']);
    assert.ok(matchesFrozen(),'Frozen source, dependency, or manifest changed before browser stage');
    const executable=browserExecutable();
    if(!executable)throw new Error('No installed Chromium browser found; STANDING_BROWSER_EXECUTABLE may identify an existing installation');
    report.executable={path:executable,sha256:sha256(readFileSync(executable))};
    const {createServer}=await import('vite');
    const temporary=mkdtempSync(join(tmpdir(),'standing-h78-browser-'));report.temporaryRuntimeState=temporary;
    const configuration=JSON.stringify({...spec,manifest}).replaceAll('<','\\u003c');
    const html=`<!doctype html><html><head><title>Standing browser acceptance ${spec.controllerId}</title></head><body style="margin:0"><div id="host" style="width:960px;height:720px"></div><script id="configuration" type="application/json">${configuration}</script><script type="module" src="/__standing-browser-harness.mjs"></script></body></html>`;
    writeFileSync(join(directory,'harness.html'),html,{flag:'wx'});writeFileSync(join(directory,'harness.mjs'),browserHarness,{flag:'wx'});
    server=await createServer({configFile:false,root:process.cwd(),cacheDir:join(temporary,'vite-cache'),logLevel:'warn',
      server:{host:'127.0.0.1',port:0,strictPort:true},plugins:[{name:'standing-browser-evidence',configureServer(vite){
        vite.middlewares.use((req,res,next)=>{const path=req.url.split('?')[0];
          if(!['/__standing-browser.html','/__standing-browser-harness.mjs'].includes(path))return next();
          res.setHeader('Content-Type',path.endsWith('.html')?'text/html':'text/javascript');res.end(path.endsWith('.html')?html:browserHarness);
        });
      }}]});
    await server.listen();
    const port=server.httpServer.address().port,profile=join(temporary,'browser-profile');mkdirSync(profile);
    const args=['--headless=new','--remote-debugging-port=0',`--user-data-dir=${profile}`,'--no-first-run','--no-default-browser-check',
      '--disable-background-networking','--disable-background-timer-throttling','--disable-renderer-backgrounding','--window-size=960,720','about:blank'];
    report.command=[executable,...args];logFd=openSync(join(directory,'browser.log'),'wx');
    browser=spawn(executable,args,{stdio:['ignore',logFd,logFd],windowsHide:true});
    let launchError=null;browser.once('error',error=>{launchError=error;});
    const activePort=join(profile,'DevToolsActivePort'),launchAt=performance.now();
    while(!existsSync(activePort)){
      if(launchError)throw launchError;
      if(browser.exitCode!==null||performance.now()-launchAt>15000)throw new Error('Installed browser did not expose a debugging endpoint');
      await delay(100);
    }
    const [debugPort,path]=readFileSync(activePort,'utf8').trim().split(/\r?\n/);
    cdp=await connectCdp(`ws://127.0.0.1:${debugPort}${path}`);report.browser=await cdp.send('Browser.getVersion');
    for(const renderer of spec.renderers){
      const {targetId}=await cdp.send('Target.createTarget',{url:'about:blank'});
      const {sessionId}=await cdp.send('Target.attachToTarget',{targetId,flatten:true});
      await cdp.send('Page.enable',{},sessionId);
      await cdp.send('Page.navigate',{url:`http://127.0.0.1:${port}/__standing-browser.html?renderer=${renderer}`},sessionId);
      const samples=[];let data=null,lastSampleTick=-manifest.memory.sampleEveryTicks;
      while(true){
        if(performance.now()-started>(manifest.budget.childTimeoutS-5)*1000)throw new Error('Browser stage child wall budget reached before both renderers completed');
        const value=await cdp.send('Runtime.evaluate',{expression:'window.__standingProbe ? JSON.stringify(window.__standingProbe) : null',returnByValue:true},sessionId);
        if(value.exceptionDetails)throw new Error('Browser probe evaluation failed');
        data=value.result.value?JSON.parse(value.result.value):null;
        if(data?.state==='error')throw new Error(data.error);
        if(data?.ticks>=manifest.runtime.warmupSteps&&(data.ticks-lastSampleTick>=manifest.memory.sampleEveryTicks||data.state==='done')){
          const processes=await cdp.send('SystemInfo.getProcessInfo');
          const memory=measureProcessMemory([...new Set(processes.processInfo.map(item=>item.id))]);
          samples.push({tick:data.ticks,wallMs:performance.now()-started,...memory});lastSampleTick=data.ticks;
          json(join(directory,`${renderer}-memory-${samples.length}.json`),samples.at(-1));
          if(memory.status!=='pass')throw new Error(memory.reason);
        }
        if(data?.state==='done')break;
        await delay(500);
      }
      const rss=samples.map(sample=>sample.rss),entry={...data,renderer,browser:report.browser,
        memory:{samples,maximumRss:rss.length?Math.max(...rss):null,growthAfterWarmup:rss.length?Math.max(...rss)-rss[0]:null,
          wasmMeasured:data.wasmMeasured,wasmSamples:data.wasmSamples,
          qualification:'Sum of OS working sets for every browser-reported process in this isolated browser instance; shared pages may be counted more than once. JS heap and WASM are recorded separately and never added to RSS.'}};
      entry.status=assessBrowserRun(entry,manifest);report.renderers.push(entry);json(join(directory,`${renderer}-report.json`),entry);
      const screenshot=await cdp.send('Page.captureScreenshot',{format:'png'},sessionId);
      writeFileSync(join(directory,`${renderer}.png`),Buffer.from(screenshot.data,'base64'),{flag:'wx'});
      await cdp.send('Runtime.evaluate',{expression:'window.__standingDispose?.()'},sessionId);
      await cdp.send('Target.closeTarget',{targetId});
      if(entry.status!=='pass')break;
    }
    report.status=report.renderers.some(entry=>entry.status==='fail')?'fail':report.renderers.length===2&&report.renderers.every(entry=>entry.status==='pass')?'pass':'incomplete';
  }catch(error){report.reason=error.message;report.error=error.stack;if(report.renderers.some(entry=>entry.status==='fail'))report.status='fail';}
  finally{
    if(cdp){try{await cdp.send('Browser.close');}catch{}cdp.close();}
    if(browser&&browser.exitCode===null){await Promise.race([new Promise(resolve=>browser.once('exit',resolve)),delay(1500)]);if(browser.exitCode===null)browser.kill();}
    if(logFd!==undefined)closeSync(logFd);
    if(server)await server.close();
    report.sourceUnchanged=matchesFrozen();
    if(!report.sourceUnchanged)report.status='fail';
    report.wallMs=performance.now()-started;
    json(join(directory,'report.json'),report);
    const stable=inventory(directory);
    verifyInventory(directory,stable);json(join(directory,'artifacts.json'),stable);
    console.log(JSON.stringify({status:report.status,reason:report.reason,renderers:report.renderers.map(run=>({renderer:run.renderer,status:run.status}))}));
  }
  return report;
}

if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
  const report=await runBrowserPerformance(process.argv[2]);process.exitCode=report.status==='pass'?0:1;
}
