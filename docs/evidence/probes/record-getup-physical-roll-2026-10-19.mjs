// SOURCE ONLY. Execution and exact-schema gates must both pass an independent audit.
// First original BACK only. Never build, reset, resume, retry or infer actors here.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { createWriteStream, existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync, appendFileSync } from 'node:fs';
import { once } from 'node:events';
import { finished } from 'node:stream/promises';
import { createGzip } from 'node:zlib';
import { createRequire } from 'node:module';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createPhysicalRollAnalyzer } from './analyze-physical-roll-stream-2026-10-19.mjs';

const root = dirname(fileURLToPath(import.meta.url));
const contractPath = join(root,'physical-roll-contract-2026-10-19.json');
const C = JSON.parse(readFileSync(contractPath));
assert.equal(C.executionApproved,true,'SOURCE-ONLY: first physical evaluation is not authorized');
assert.equal(C.schemaPinned,true,'SOURCE-ONLY: independent exact-schema/source audit pending');
assert.equal(process.argv.length,4,'Usage: recorder FROZEN_ADAPTED_APP FRESH_OUTPUT');
const artifact = resolve(process.argv[2]), output = resolve(process.argv[3]);
assert.ok(!existsSync(output),'Fresh output required');
assert.ok(output.startsWith(resolve(process.cwd())+sep),'Workspace-child output required');
assert.ok(output !== artifact && !output.startsWith(artifact+sep) && !artifact.startsWith(output+sep));
const hash = b => createHash('sha256').update(b).digest('hex');
const json = (name,value) => writeFileSync(join(output,name),JSON.stringify(value,null,2)+'\n');
const canonical = v => Array.isArray(v) ? v.map(canonical) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map(k=>[k,canonical(v[k])])) : v;
const equal = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
function hashes(path,base = path) {
  return Object.fromEntries(readdirSync(path,{withFileTypes:true}).flatMap(entry => {
    const p = join(path,entry.name);
    return entry.isDirectory() ? Object.entries(hashes(p,base)) : [[relative(base,p),hash(readFileSync(p))]];
  }));
}
for (const value of Object.values(C.pendingFreeze)) assert.match(value ?? '',/^[a-f0-9]{64}$/,'Unfrozen input');
for (const entry of [C.predeclaredContract,C.checkpointHelper,C.nativeSourceFreeze,C.fixedPresentationManifest,C.baselinePreparation])
  assert.equal(hash(readFileSync(resolve(entry.path))),entry.sha256,'Frozen input changed: '+entry.path);
assert.equal(hash(readFileSync(fileURLToPath(import.meta.url))),C.pendingFreeze.recorderSourceSha256);
assert.equal(hash(readFileSync(join(root,'analyze-physical-roll-stream-2026-10-19.mjs'))),C.pendingFreeze.analyzerSourceSha256);
const manifestPath = artifact+'-fixture-adapter-manifest.json', manifest = JSON.parse(readFileSync(manifestPath));
assert.deepEqual(hashes(artifact),manifest.artifactSha256,'Every adapted app byte must match its frozen manifest');
assert.equal(manifest.approvedFrozenManifest.sha256,C.pendingFreeze.approvedAppManifestSha256);
assert.equal(hash(readFileSync(resolve(manifest.approvedFrozenManifest.path))),C.pendingFreeze.approvedAppManifestSha256);
assert.equal(manifest.physicalRollSource.sha256,C.pendingFreeze.adapterSourceSha256);
assert.deepEqual(manifest.physicalRollContract.value,C);
assert.equal(manifest.physicalRollContract.sha256,hash(readFileSync(contractPath)));
assert.deepEqual([...manifest.sourceBuildFeatures].sort(),[...C.features].sort());
const profileBytes = readFileSync(resolve(C.profilePath)), profile = JSON.parse(profileBytes);
assert.equal(hash(profileBytes),C.profileSha256); assert.equal(manifest.explicitProfile.sha256,C.profileSha256);
assert.deepEqual(manifest.explicitProfile.value,profile);
const presentation = JSON.parse(readFileSync(resolve(C.fixedPresentationManifest.path)));
for (const [path,sha] of Object.entries(presentation.artifactSha256))
  if (!C.fixedPresentationManifest.allowedChangedAssets.includes(path)) assert.equal(manifest.artifactSha256[path],sha,'Fixed presentation changed: '+path);
const drivers = Object.keys(manifest.artifactSha256).filter(p=>p.endsWith('/driver.js'));
assert.equal(drivers.length,1); assert.equal(manifest.artifactSha256[drivers[0]],hash(readFileSync(resolve('rust/crates/browser/driver.js'))));
assert.ok(readFileSync(resolve('rust/crates/browser/driver.js'),'utf8').includes("message.type === 'initialize' ? 10000 : getupTrial ? 90000 : 5000"),'Original90-second GetUp request watchdog required');

mkdirSync(output);
const report = { at:new Date().toISOString(),observationComplete:false,completeCasePassed:false,
  qualificationClaim:false,releaseAccepted:false,productionRecoveryEnabled:false,performanceClaim:false,
  artifact,contractSha256:hash(readFileSync(contractPath)),adapterManifestSha256:hash(readFileSync(manifestPath)),
  profileSha256:C.profileSha256,errors:[],traceRows:0,publicationCount:0,rawCadenceVideo:true,
  scope:'One fresh original BACK actual-WASM world, native0 carry once, natural80-native prep and original trigger480 with325-word purity. Exact old passive native0..480 prefix. Ordinary20-native publications stream losslessly to gzip level1 with one-packet ACK backpressure. Full current native predicates; original25-second deadline including roll prep; same-world30-second hold. No commands, reset, checkpoint resume, alternate route or performance claim.' };
const a = createPhysicalRollAnalyzer(C);
const trace = createGzip({level:1}), traceFile = createWriteStream(join(output,'native-trace.ndjson.gz'),{flags:'wx'});
trace.pipe(traceFile);
let streamFault = null; trace.on('error',e=>{streamFault=e;traceFile.destroy(e);}); traceFile.on('error',e=>{streamFault=e;trace.destroy(e);});
const traceLine = async row => {
  if (streamFault) throw streamFault;
  if (!trace.write(JSON.stringify(row)+'\n')) await once(trace,'drain');
  report.traceRows++;
};
const recordVideo = process.env.RUST_BROWSER_NO_VIDEO !== '1';
assert.equal(recordVideo,true,'First physical evaluation requires raw-cadence recording');
if (process.env.RUST_BROWSER_FFMPEG) {
  const require = createRequire(import.meta.url), core = dirname(require.resolve('playwright-core/package.json'));
  const descriptor = JSON.parse(readFileSync(join(core,'browsers.json'))).browsers.find(x=>x.name==='ffmpeg');
  const tools = join(output,'playwright-tools'), executable = join(tools,`ffmpeg-${descriptor.revision}`,'ffmpeg-linux');
  mkdirSync(dirname(executable),{recursive:true}); symlinkSync(resolve(process.env.RUST_BROWSER_FFMPEG),executable);
  process.env.PLAYWRIGHT_BROWSERS_PATH = tools;
}
let server,browser,context,page,video,terminal = null,sinkBusy = false,lastArchivedNative = null;
const saveRaw = (name,value) => { writeFileSync(join(output,name),value); return {path:name,sha256:hash(value),bytes:Buffer.byteLength(value)}; };
try {
  const { chromium,browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
  server = spawn(process.execPath,['scripts/serve-rust-preview.mjs',artifact,'0'],{stdio:['ignore','pipe','pipe']});
  let stderr = '';
  server.stderr.on('data',b=>{stderr+=String(b);});
  const origin = await new Promise((accept,reject)=>{
    let stdout = ''; const timer = setTimeout(()=>reject(new Error('Preview startup watchdog: '+stderr)),C.previewStartupWatchdogMs);
    server.once('error',e=>{clearTimeout(timer);reject(e);});
    server.once('exit',(code,signal)=>{clearTimeout(timer);reject(new Error(`Preview exit ${code}/${signal}: ${stderr}`));});
    server.stdout.on('data',b=>{stdout+=String(b);const url=stdout.match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];if(url){clearTimeout(timer);accept(url);}});
  });
  browser = await chromium.launch(browserLaunchOptions(true)); report.browserVersion = browser.version();
  context = await browser.newContext({viewport:{width:1280,height:900},recordVideo:{dir:output,size:{width:1280,height:900}}});
  await context.exposeBinding('__recordPhysicalRollPacket',async (_source,packet)=>{
    if (sinkBusy) throw new Error('More than one native publication is in flight to the trace sink');
    sinkBusy = true;
    try {
      // Durable raw native records precede every semantic assertion. The only
      // transient row collection is this one bounded publication (at most20).
      if (packet.native0State && lastArchivedNative === null) {
        await traceLine({receiptKind:'native0-read-only-state',...packet.native0State,snapshot:packet.native0Snapshot});lastArchivedNative=0;
        json('native0-controller.json',packet.native0Controller);
      }
      if (packet.fault) {
        // A native integrity failure may advance once before its observer can
        // emit a row. Preserve every available raw row and the exact original
        // fault packet. Never interpolate that missing integration or force a
        // checkpoint/export. A stale repeated prior packet stays in raw fault
        // evidence and is not duplicated in the chronological native stream.
        json('worker-error-raw.json',packet.fault);
        if(Array.isArray(packet.rows))for(const row of packet.rows){
          if(Number.isSafeInteger(row.nativeSubsteps)&&row.nativeSubsteps>(lastArchivedNative??-1)){await traceLine(row);lastArchivedNative=row.nativeSubsteps;}
        }
        if(packet.fault.fixtureTest?.rollTerminal){terminal=packet.fault.fixtureTest.rollTerminal;json('terminal-raw.json',terminal);}
        report.errors.push({source:'original-worker',error:packet.fault.message ?? 'Worker fault',nativeCoverage:'Explicitly incomplete or inconclusive; raw fault and all available native rows retained.'});
        return {ok:true};
      }
      if (Array.isArray(packet.rows)) for (const row of packet.rows) {await traceLine(row);lastArchivedNative=row.nativeSubsteps;}
      if (packet.summary.rollTerminal) { terminal=packet.summary.rollTerminal; json('terminal-raw.json',terminal); }
      appendFileSync(join(output,'publications.ndjson'),JSON.stringify(packet.summary)+'\n');
      if (packet.native0State) a.consumeNative0(packet.native0State,packet.native0Snapshot);
      a.consumeRows(packet.rows,packet.summary.rollTerminal); a.consumePublication(packet.summary);
      report.publicationCount++;
      return {ok:true};
    } catch (error) {
      json('sink-fault-packet.json',{error:String(error),stack:error.stack ?? null,packet});
      report.errors.push({source:'stream-sink-or-native-audit',error:String(error),stack:error.stack ?? null});
      return {ok:false,error:String(error)};
    } finally { sinkBusy=false; }
  });
  await context.addInitScript(({C,profile})=>{
    window.__lhGetUpTrial = true;
    let api;
    Object.defineProperty(window,'__lhRust',{configurable:true,get:()=>api,set(value){api=value;api.setManual(true);}});
    const canonical = v => Array.isArray(v)?v.map(canonical):v&&typeof v==='object'?Object.fromEntries(Object.keys(v).sort().map(k=>[k,canonical(v[k])])):v;
    const equal = (x,y)=>JSON.stringify(canonical(x))===JSON.stringify(canonical(y));
    const e = window.__physicalRollEvidence = {workerInstances:0,workerIdentities:[],initializations:[],requests:0,advances:0,
      activation:null,native0Controller:null,trigger:null,terminal:null,terminalSeen:false,lastNative:null,lastPublicationNative:null,
      traceRows:0,publications:0,pendingSink:false,queuedAdvanceCount:0,maximumQueuedAdvances:0,
      profile:null,buildIdentity:null,prefix:null,physicalInputs:[],rawWorkerFaults:[],rawObserverFault:null,
      error:null,stopReason:null,startedAtMs:null,endedAtMs:null,firstRollWallMs:null,sourceStartWallMs:null,firstPassWallMs:null,
      lastPublicationWallMs:null,lastAdvanceRequest:null,sourceForecastBearingRequest:null,
      stop(reason,error=null){if(error!==null)this.error??=String(error);this.stopReason??=reason;this.endedAtMs??=performance.now();api?.setManual(true);}};
    let pendingAdvance = null;
    const Native = window.Worker;
    window.Worker = class extends Native {
      constructor(...args){
        super(...args); e.workerInstances++;
        this.addEventListener('message',event=>{
          const m = event.data, f = m.fixtureTest;
          // Synchronous latch precedes the unchanged driver's onmessage pump.
          if (f?.rollTerminal) { e.terminalSeen=true;api?.setManual(true); }
          if (m.type === 'ready') e.workerIdentities.push(structuredClone(m.workerIdentity));
          if (m.type === 'error') {
            e.rawWorkerFaults.push(structuredClone(m));e.terminal=structuredClone(f?.rollTerminal??null);e.prefix=structuredClone(f?.rollPrefixEvidence??null);
            e.pendingSink=true;
            window.__recordPhysicalRollPacket({fault:structuredClone(m),rows:f?.rollNativeTrace??[],native0State:null}).then(ack=>{
              e.pendingSink=false;if(!ack.ok)e.stop('stream-sink-error',ack.error);
            }).catch(error=>{e.pendingSink=false;e.stop('stream-sink-error',String(error));});
            e.stop('worker-error',m.message);return;
          }
          if (!m.reply) return;
          const s=m.reply.snapshot,n=Math.round(s.simulation_time_s*800),wall=performance.now();
          const rows=f?.rollNativeTrace;
          // Pass raw rows to Node BEFORE local assertions. The next ordinary
          // advance is held until the gzip sink acknowledges this packet.
          if(e.pendingSink){e.rawObserverFault={error:'Overlapping native packets',message:structuredClone(m)};e.stop('instrumentation-error','Overlapping native packets');return;}
          const summary={native:n,completedTicks:m.reply.completed_ticks,stamp:structuredClone(s.stamp),
            counters:structuredClone(s.counters),motion:s.motion,wallMs:wall,rollTraceError:f?.rollTraceError,
            physicalRoll:structuredClone(f?.physicalRoll ?? null),fixtureSolverBudget:structuredClone(f?.fixtureSolverBudget),
            fixtureRootRotationPolicy:structuredClone(f?.fixtureRootRotationPolicy),rollPrefixEvidence:structuredClone(f?.rollPrefixEvidence ?? null),
            rollTerminal:structuredClone(f?.rollTerminal ?? null),advanceRequest:e.lastAdvanceRequest?.requestId===m.requestId
              ? {...e.lastAdvanceRequest,publicationWallMs:wall,requestToPublicationMs:wall-e.lastAdvanceRequest.postedWallMs}:null};
          if(f?.physicalRoll?.sourceStartNative!=null&&e.sourceStartWallMs===null){
            e.sourceForecastBearingRequest=structuredClone(summary.advanceRequest);
            summary.sourceForecastBearingRequest=structuredClone(summary.advanceRequest);
            summary.sourceForecastBearingIntervalSemantics='One ordinary advance request containing source forecast plus20 real native steps, observers, WASM/JS serialization and transport. Not isolated forecast compute.';
          }
          e.pendingSink=true;
          window.__recordPhysicalRollPacket({rows,native0State:e.lastNative===null?f?.rollNative0State:null,
            native0Snapshot:e.lastNative===null?structuredClone(s):null,
            native0Controller:e.lastNative===null?structuredClone(f?.rollNative0Controller??null):null,summary}).then(ack=>{
              e.pendingSink=false;
              if(!ack.ok){e.stop('stream-sink-error',ack.error);pendingAdvance=null;return;}
              const queued=pendingAdvance;pendingAdvance=null;e.queuedAdvanceCount=0;
              if(queued&&!e.terminalSeen&&!e.error)queued();
            }).catch(error=>{e.pendingSink=false;pendingAdvance=null;e.stop('stream-sink-error',String(error));});
          try{
            if(!f?.isolatedTestOnly||f.kind!=='back'||f.preparationPollingNative!==80||!equal(f.profile,profile))throw new Error('Original BACK80-native full profile missing');
            if(s.simulation_time_s!==n/800||s.stamp.generation!==1||!s.diagnostics.getup?.learned_policy_loaded)throw new Error('Native clock/generation/actor changed');
            if(!Array.isArray(rows)||rows.length>20||f.rollTraceError!==null)throw new Error('Native trace packet missing, oversized or faulted');
            const identity=f.rollBuildIdentity;
            if(identity?.experiment!=='native-motor-observer-v1'||identity.variant!=='source-normal-motors'
              ||identity.nativeTrace!=='read-only-post-native-observer-v1'||identity.maxTicksPerCall!==1
              ||identity.checkpointResumeSupported!==true||identity.checkpointExportSupported!==true)throw new Error('Wrong immutable native observer build');
            if(e.buildIdentity&&!equal(identity,e.buildIdentity))throw new Error('Native build identity changed');e.buildIdentity=structuredClone(identity);
            if(e.lastNative===null){if(n!==0||m.reply.completed_ticks!==0||rows.length!==0||!f.rollNative0State||!f.rollNative0Controller)throw new Error('Fresh read-only native0/controller required');e.native0Controller=structuredClone(f.rollNative0Controller);e.lastNative=0;}
            else if(n-e.lastNative!==rows.length||rows.length===0||(!f.rollTerminal&&(n-e.lastNative!==20||m.reply.completed_ticks!==1)))throw new Error('Original one-tick native publication changed');
            for(const row of rows){if(row.nativeSubsteps!==e.lastNative+1)throw new Error('Native trace gap/duplicate');e.lastNative=row.nativeSubsteps;e.traceRows++;}
            if(f.carryActivationEvidence){if(e.activation!==null)throw new Error('Carry activation repeated');e.activation=structuredClone(f.carryActivationEvidence);}
            if(f.triggerEvidence){if(e.trigger!==null||f.triggerEvidence.native!==480)throw new Error('Original natural trigger repeated or changed');e.trigger=structuredClone(f.triggerEvidence);}
            if(f.triggerNative!==null&&f.triggerNative!==480)throw new Error('Original impact epoch changed');
            e.profile=structuredClone(f.profile);e.prefix=structuredClone(f.rollPrefixEvidence);e.publications++;e.lastPublicationNative=n;e.lastPublicationWallMs=wall;
            if(f.physicalRoll){e.firstRollWallMs??=wall;if(f.physicalRoll.sourceStartNative!==null)e.sourceStartWallMs??=wall;if(f.physicalRoll.standingPassed)e.firstPassWallMs??=wall;}
            if(f.rollTerminal){e.terminal=structuredClone(f.rollTerminal);e.stop(f.rollTerminal.reason,f.rollTerminal.error);}
            if(m.reply.failure)e.stop('physical-or-resource-halt',JSON.stringify(m.reply.failure));
          }catch(error){e.rawObserverFault={error:String(error),stack:error.stack??null,message:structuredClone(m)};e.stop('instrumentation-error',String(error));}
        });
      }
      postMessage(message,...args){
        if(e.terminalSeen&&!['recycle','shutdown'].includes(message.type)){e.stop('post-terminal-request','No requests after worker terminal latch');throw new Error(e.error);}
        if(message.type==='initialize'){
          if(e.initializations.length!==0){e.stop('runtime-replacement','Exactly one fresh runtime permitted');throw new Error(e.error);}
          message={...message,testFixedGetUpFixture:'back',testFixedGetUpProfile:profile};e.initializations.push(structuredClone(message));
        }
        if(message.commands?.length||message.type==='commands'||message.type==='reset'){e.stop('physical-command-or-reset','Physical commands and resets forbidden');throw new Error(e.error);}
        const send=()=>{e.requests++;if(message.type==='advance')e.lastAdvanceRequest={requestId:message.requestId,ticks:message.ticks,postedWallMs:performance.now()};return super.postMessage(message,...args);};
        if(message.type==='advance'){
          if(message.ticks!==1){e.stop('nonordinary-advance','Only original ordinary one-tick requests permitted');throw new Error(e.error);}
          e.advances++;
          if(e.pendingSink){if(pendingAdvance){e.stop('backpressure-overflow','At most one pending ordinary advance');throw new Error(e.error);}
            e.queuedAdvanceCount=1;e.maximumQueuedAdvances=Math.max(e.maximumQueuedAdvances,1);pendingAdvance=send;return;}
        }
        return send();
      }
    };
    for(const type of ['keydown','keyup','pointerdown'])document.addEventListener(type,event=>{
      if(e.startedAtMs!==null&&e.endedAtMs===null){e.physicalInputs.push({type,code:event.code??null,isTrusted:event.isTrusted});e.stop('unexpected-physical-input','No physical input permitted');}
    },true);
  },{C,profile});
  page = await context.newPage(); video=page.video();
  page.on('pageerror',error=>report.errors.push({source:'page',error:String(error),stack:error.stack??null}));
  page.on('response',response=>{if(response.status()>=400)report.errors.push({source:'http',status:response.status(),url:response.url()});});
  await page.goto(origin+'?renderer=webgl2',{timeout:C.browserStartupWatchdogMs});
  await page.waitForFunction(()=>window.__physicalRollEvidence?.error||window.__lhRust?.diagnostics.ready&&window.__lhRust.diagnostics.graphics.frames>=3&&!window.__physicalRollEvidence.pendingSink,null,{timeout:C.browserStartupWatchdogMs});
  report.initial=await page.evaluate(()=>window.__lhRust.diagnostics);
  assert.equal(report.initial.tick,0);assert.equal(report.initial.time,0);assert.equal(report.initial.backend,'WebGL2');
  await page.locator('#camera-focus').click();
  const viewport=await page.locator('#viewport').boundingBox();assert.ok(viewport);
  await page.mouse.move(viewport.x+viewport.width/2,viewport.y+viewport.height/2);await page.mouse.wheel(0,400);
  await page.waitForTimeout(150);report.framing={ordinaryFocusBodyUi:true,prePhysicsWheelDeltaY:400};
  await page.screenshot({path:join(output,'initial-real-back-native0.png'),fullPage:true});
  await page.evaluate(()=>{const e=window.__physicalRollEvidence;if(e.error||e.pendingSink)throw new Error(e.error??'Native0 sink pending');e.startedAtMs=performance.now();window.__lhRust.drainTelemetry();window.__lhRust.setManual(false);});
  await page.waitForFunction(()=>{
    const e=window.__physicalRollEvidence,d=window.__lhRust?.diagnostics;
    if(d?.workerFault||d?.disposed||d?.halted||d?.failure||d?.error)e?.stop('original-worker-or-runtime-halt',d.error??'Original runtime halted');
    return e?.endedAtMs!==null;
  },null,{timeout:C.totalWallWatchdogMs});
  await page.waitForFunction(()=>!window.__physicalRollEvidence?.pendingSink,null,{timeout:C.readOnlyIdleWatchdogMs});
  report.final=await page.evaluate(()=>window.__lhRust?.diagnostics);
  await page.screenshot({path:join(output,'terminal-real-back.png'),fullPage:true});
} catch(error) {report.errors.push({source:'recorder',error:String(error),stack:error.stack??null});}
finally {
  // Fault receipts are archived before purity checks. Never request a new
  // controller, checkpoint or world to recover from an original native fault.
  if(page)try{
    await page.evaluate(()=>window.__physicalRollEvidence?.stop('recorder-finally'));
    await page.waitForFunction(()=>!window.__physicalRollEvidence?.pendingSink,null,{timeout:C.readOnlyIdleWatchdogMs});
    const e=await page.evaluate(()=>{
      const e=window.__physicalRollEvidence;if(!e)return null;const {stop,...data}=e;
      const clone=structuredClone(data),encode=bytes=>{if(!bytes)return null;let v='';for(let i=0;i<bytes.length;i+=0x8000)v+=String.fromCharCode(...bytes.subarray(i,i+0x8000));return btoa(v);};
      if(clone.activation){for(const k of ['beforeCheckpoint','afterCheckpoint']){clone.activation[k+'Base64']=encode(clone.activation[k]);delete clone.activation[k];}}
      return clone;
    });
    json('browser-evidence-raw.json',e);
    if(e){
      const activation=e.activation;
      if(activation){
        const {proveActivationPurity}=await import(pathToFileURL(resolve(C.checkpointHelper.path)).href);
        const before=Buffer.from(activation.beforeCheckpointBase64??'','base64'),after=Buffer.from(activation.afterCheckpointBase64??'','base64');
        saveRaw('activation-before-checkpoint.bin',before);saveRaw('activation-after-checkpoint.bin',after);
        activation.independentActivationProof=proveActivationPurity(before,after);json('activation-proof.json',activation);
        assert.equal(activation.activationCalls,1);assert.equal(activation.beforeReport.nativeSubsteps,0);assert.equal(activation.afterReport.nativeSubsteps,0);
      }
      if(e.native0Controller)json('native0-controller.json',e.native0Controller);
      if(e.trigger){json('trigger-evidence.json',e.trigger);json('trigger-controller.json',e.trigger.controllerReport);assert.equal(e.trigger.native,480);assert.equal(e.trigger.physicalPoseVelocityBitsUnchanged,true);assert.deepEqual(e.trigger.beforePoseWords,e.trigger.afterPoseWords);assert.equal(e.trigger.beforePoseWords.length,325);}
      if(e.terminal){terminal=e.terminal;json('terminal-raw.json',terminal);json('stop-controller.json',terminal.stopController);json('stop-fixture.json',terminal.stopFixture);}
      assert.equal(e.workerInstances,1);assert.equal(e.initializations.length,1);assert.equal(e.workerIdentities.length,1);
      assert.equal(e.error,null);assert.equal(e.rawObserverFault,null);assert.equal(e.rawWorkerFaults.length,0);assert.equal(e.physicalInputs.length,0);
      assert.ok(activation && e.trigger && e.terminalSeen && terminal,'Native0/trigger/terminal evidence required');
      report.wallObservations={startMs:e.startedAtMs,firstRollPublicationMs:e.firstRollWallMs,sourceStartPublicationMs:e.sourceStartWallMs,
        firstPassPublicationMs:e.firstPassWallMs,terminalPublicationMs:e.endedAtMs,
        sourceForecastBearingRequest:e.sourceForecastBearingRequest,
        sourceForecastBearingIntervalSemantics:'Request-to-publication includes source forecast plus20 real native steps, native observation, serialization and transport; no isolated compute attribution.',
        interpretation:'Publication wall timestamps only, including forecast stalls, streaming backpressure and browser overhead; no isolated performance attribution.'};
      report.analysis=a.finish(terminal);report.observationComplete=true;report.completeCasePassed=report.analysis.completeCasePassed;
    }
  }catch(error){report.errors.push({source:'evidence-or-final-audit',error:String(error),stack:error.stack??null});}
  report.partialAnalysis=a.snapshot();
  trace.end();try{await Promise.all([finished(trace),finished(traceFile)]);}catch(error){report.errors.push({source:'trace-finalization',error:String(error)});}
  try{await context?.close();}catch(error){report.errors.push({source:'context-close',error:String(error)});}
  if(video)try{await video.saveAs(join(output,'fixed-fixture-native-wall-time.webm'));}catch(error){report.errors.push({source:'video-save',error:String(error)});}
  try{await browser?.close();}catch(error){report.errors.push({source:'browser-close',error:String(error)});}
  server?.kill('SIGTERM');
  report.completeCasePassed=report.completeCasePassed&&report.errors.length===0;
  json('result.json',report);
}
console.log(JSON.stringify({observationComplete:report.observationComplete,completeCasePassed:report.completeCasePassed,result:join(output,'result.json')}));
if(!report.observationComplete||report.errors.length)process.exitCode=1;
