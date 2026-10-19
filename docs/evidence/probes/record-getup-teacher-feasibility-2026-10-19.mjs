// SOURCE-ONLY draft. This script does not build an app, infer an actor itself,
// resume checkpoints, apply references, or advance after the trigger480 export.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const { parseGetUpCheckpoint, proveActivationPurity } = await import(pathToFileURL(resolve('docs/evidence/probes/getup-checkpoint-envelope-2026-10-19.mjs')).href);

const contractPath = join(dirname(fileURLToPath(import.meta.url)), 'teacher-feasibility-contract-2026-10-19.json');
const C = JSON.parse(readFileSync(contractPath));
assert.equal(C.executionApproved, true, 'SOURCE-ONLY draft: execution gate is false');
assert.equal(C.schemaPinned, true, 'Final native schema, raw typed archives, source freeze and independent audit are pending');
assert.equal(process.argv.length, 4, 'Usage: recorder FROZEN_ADAPTED_APP FRESH_OUTPUT');
const artifact = resolve(process.argv[2]), output = resolve(process.argv[3]);
assert.ok(!existsSync(output), 'Fresh output directory required; no overwrite');
assert.ok(output.startsWith(resolve(process.cwd()) + sep), 'Workspace-child output required');
assert.ok(!output.startsWith(artifact + sep) && !artifact.startsWith(output + sep), 'Non-overlapping artifacts required');
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
assert.equal(hash(readFileSync(resolve(C.checkpointHelper.path))),C.checkpointHelper.sha256);
assert.equal(hash(readFileSync(resolve(C.nativeSourceFreeze.path))),C.nativeSourceFreeze.sha256);
for (const [name,value] of Object.entries(C.pendingFreeze)) assert.match(value ?? '', /^[a-f0-9]{64}$/, 'Pending final freeze: '+name);
assert.equal(hash(readFileSync(fileURLToPath(import.meta.url))),C.pendingFreeze.recorderSourceSha256);
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
  ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
const equal = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
const jsonHash = value => hash(JSON.stringify(canonical(value)));
function hashes(root, base = root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root,entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path,base)) : [[relative(base,path),hash(readFileSync(path))]];
  }));
}
const manifestPath = artifact + '-fixture-adapter-manifest.json';
const manifest = JSON.parse(readFileSync(manifestPath));
assert.equal(manifest.approvedFrozenManifest.sha256,C.pendingFreeze.approvedAppManifestSha256);
assert.equal(hash(readFileSync(resolve(manifest.approvedFrozenManifest.path))),C.pendingFreeze.approvedAppManifestSha256);
assert.equal(manifest.teacherFeasibilitySource.sha256,C.pendingFreeze.adapterSourceSha256);
assert.deepEqual(hashes(artifact), manifest.artifactSha256, 'Every app byte must match the frozen adapter manifest');
assert.deepEqual([...manifest.sourceBuildFeatures].sort(), [...C.features].sort(), 'Exactly the prior five Cargo CLI features; terminal-balance is implied by observer');
assert.deepEqual(manifest.teacherFeasibilityContract.value, C);
assert.equal(manifest.teacherFeasibilityContract.sha256, hash(readFileSync(contractPath)));
const profileBytes = readFileSync(resolve(C.profilePath)), profile = JSON.parse(profileBytes);
assert.equal(hash(profileBytes), C.profileSha256);
assert.equal(manifest.explicitProfile.sha256, C.profileSha256);
assert.deepEqual(manifest.explicitProfile.value, profile, 'Complete approved aaa415a1 profile required');
const presentationBytes=readFileSync(resolve(C.fixedPresentationManifest.path));
assert.equal(hash(presentationBytes),C.fixedPresentationManifest.sha256);
for(const [path,expected]of Object.entries(JSON.parse(presentationBytes).artifactSha256)){
  if(!C.fixedPresentationManifest.allowedChangedAssets.includes(path))assert.equal(manifest.artifactSha256[path],expected,'Fixed presentation asset changed: '+path);
}
const legacyBytes = readFileSync(resolve(C.legacyArchivePath)), legacy = JSON.parse(legacyBytes);
assert.equal(hash(legacyBytes), C.legacyArchiveSha256, 'Untouched canonical old51 archive required');
assert.equal(legacy.teacher_frames.length, C.legacyPrefixFrames);
assert.deepEqual(legacy.teacher_initial.stamp, { schema:4, generation:1, sequence:481, tick:24 });
assert.equal(legacy.teacher_initial.simulation_time_s, .6);
const driverPaths = Object.keys(manifest.artifactSha256).filter(path => path.endsWith('/driver.js'));
assert.equal(driverPaths.length, 1);
assert.equal(manifest.artifactSha256[driverPaths[0]], hash(readFileSync(resolve('rust/crates/browser/driver.js'))), 'Original driver bytes required');
mkdirSync(output);
const report = { at: new Date().toISOString(), passed:false, observationComplete:false,
  qualificationClaim:false, recoveryDemonstrated:false, performanceClaim:false,
  mode:'teacher-only-at-original-trigger480', artifact, contractSha256:hash(readFileSync(contractPath)),
  adapterManifestSha256:hash(readFileSync(manifestPath)), artifactSha256:manifest.artifactSha256,
  profileSha256:C.profileSha256, sourceFeatures:C.features, errors:[],
  scope:'Fresh actual-WASM BACK, native0 carry once, unchanged fixed renderer/driver/transport, original80-native preparation. Real integration ends at480. One inline immutable teacher export before ordinary advance(1), complete raw checkpoint byte equality, unchanged empty real controller/fixture reports. Forecast-only full archives/current-native diagnostics; no source following, canonical suffix, reference application, handoff, real recovery counters or performance claim.' };
let server, browser, context, page;
const saveJson = (name,value) => writeFileSync(join(output,name), JSON.stringify(value,null,2)+'\n');
const saveRaw = (name,bytes) => {
  writeFileSync(join(output,name),bytes);
  return { path:join(output,name), bytes:Buffer.byteLength(bytes), sha256:hash(bytes) };
};

function validateReceipt(receipt) {
  assert.deepEqual(Object.keys(receipt).sort(), [...C.receiptFields].sort(), 'Final exact receipt schema must be pinned');
  assert.equal(receipt.experiment,C.experiment);
  assert.ok(C.statuses.includes(receipt.status));
  assert.equal(receipt.realNativeSubsteps,C.terminalNative);
  assert.equal(receipt.realPhysicsIntegrations,0);
  assert.equal(receipt.totalDecisionCeiling,C.totalDecisionCeiling);
  assert.equal(receipt.nativePerDecision,20);
  assert.equal(Math.fround(receipt.teacherPower),Math.fround(C.teacherPower));
  assert.equal(Math.fround(receipt.teacherNativeDtS),Math.fround(1/800),'Forecast dt must retain the exact native f32 value');
  assert.equal(receipt.requiredConsecutiveFullStandingSamples,C.requiredConsecutiveFullStandingSamples);
  assert.equal(receipt.standingIntervalSemantics,'Continuous original prospective anchors/streak from forecast native1; endpoint acceptance only after the complete inherited height-cut prefix. No reset or re-anchoring at that cut.');
  assert.equal(receipt.referenceGenerated,false);
  assert.equal(receipt.canonicalAppended,false);
  assert.equal(receipt.realHandoffActivated,false);
  const integrityFailure=receipt.status==='integrity-failure';
  assert.equal(integrityFailure,receipt.failure!==null,'Native integrity status must retain its raw failure');
  assert.ok(Number.isSafeInteger(receipt.decisionsStarted) && receipt.decisionsStarted >= (integrityFailure?1:51) && receipt.decisionsStarted <= 250);
  assert.ok(Number.isSafeInteger(receipt.forecastNativeIntegrations) && receipt.forecastNativeIntegrations >= (integrityFailure?1:1020) && receipt.forecastNativeIntegrations <= 5000);
  assert.ok(Number.isSafeInteger(receipt.lastDecisionNativeCount) && receipt.lastDecisionNativeCount >= 1 && receipt.lastDecisionNativeCount <= 20);
  assert.equal(receipt.nativeAccountingValid,true);
  assert.equal(receipt.forecastNativeIntegrations,(receipt.decisionsStarted-1)*20+receipt.lastDecisionNativeCount);
  assert.ok(Number.isSafeInteger(receipt.unobservedNativeTail) && receipt.unobservedNativeTail >= 0);
  if(integrityFailure&&receipt.decisionsStarted<=51)assert.equal(receipt.originalHeightCutDecisions,null);
  else assert.equal(receipt.originalHeightCutDecisions,C.legacyPrefixFrames);
  assert.equal(typeof receipt.archiveContactSemantics,'string');
  // Authoritative legacy snapshots are direct typed-f32 JSON strings. Never
  // round promoted Value numbers, strip contacts/motion, or compare subsets.
  const fields = C.archiveFields;
  assert.equal(typeof receipt[fields.typedInitialJson],'string', 'Pending final typed seed JSON field must be present');
  assert.equal(typeof receipt[fields.typedFramesJson],'string', 'Pending final typed full frames JSON field must be present');
  const initial = JSON.parse(receipt[fields.typedInitialJson]);
  const frames = JSON.parse(receipt[fields.typedFramesJson]);
  assert.deepEqual(initial,legacy.teacher_initial,'Complete seed, including legacy contacts/motion/stamp, must exactly equal old archive');
  assert.ok(Array.isArray(frames) && frames.length >= (integrityFailure?1:51) && frames.length <= 250);
  // An integrity-failing decision archives its partial terminal frame too.
  // Preserve that full frame, but do not compare it to an old completed20-step
  // decision or credit it as a completed canonical-prefix frame.
  const prefixLength=Math.min(frames.length-(integrityFailure?1:0),51);
  assert.deepEqual(frames.slice(0,prefixLength),legacy.teacher_frames.slice(0,prefixLength),'All fields of every available original-prefix TeacherFrame must exactly equal old archive');
  assert.equal(receipt[fields.frameCount],frames.length);
  assert.equal(frames.length,receipt.decisionsStarted);
  assert.ok(Array.isArray(receipt.nativeTrace));
  assert.equal(receipt.nativeTrace.length+receipt.unobservedNativeTail,receipt.forecastNativeIntegrations,'Every forecast integration must be observed or explicitly uncredited');
  if(receipt.failure===null)assert.equal(receipt.unobservedNativeTail,0,'No successful outcome may contain an unobserved native tail');
  else assert.equal(receipt.unobservedNativeTail,1,'Integrity inspection stops immediately on its one unobserved failed native');
  let consecutive = 0, maximum = 0, previousAnchors = null;
  for (let index=0; index<receipt.nativeTrace.length; index++) {
    const row = receipt.nativeTrace[index], n = index+1, d = row.current;
    assert.deepEqual(Object.keys(row).sort(),[...C.nativeTraceFields].sort());
    assert.equal(row.forecastNativeSubstep,n);
    assert.equal(row.decisionIndex,Math.floor(index/20));
    assert.equal(row.withinDecisionNative,index%20+1);
    assert.equal(row.internalPgsIterations,32);
    assert.equal(row.action.length,21); assert.ok(row.action.every(Number.isFinite));
    assert.equal(d.physicsIntegrations,0); assert.equal(d.nativeSubsteps,n);
    assert.deepEqual(Object.keys(d.predicates).sort(),[...C.standingPredicateKeys].sort());
    assert.equal(d.loadsN.length,25); assert.ok(d.loadsN.every(Number.isFinite));
    assert.equal(d.supportPersistence.length,25); assert.ok(d.supportPersistence.every(value=>Number.isSafeInteger(value)&&value>=0));
    assert.equal(d.motion.length,25);
    assert.ok(d.motion.every(value => Number.isFinite(value.linearMps) && Number.isFinite(value.angularRadps)));
    assert.equal(d.contactMotion.physicsIntegrations,0);
    assert.equal(d.contactMotion.velocitySource,'current-generalized-motion');
    assert.equal(d.contactMotion.feet.length,2);
    assert.equal(d.footIds.length,2);
    assert.equal(new Set(d.footIds).size,2);assert.ok(d.footIds.every(value=>Number.isSafeInteger(value)&&value>=0&&value<25));
    assert.equal(d.feet.length,2);
    assert.ok(d.feet.every(foot => ['x','y','z'].every(key => Number.isFinite(foot[key]))));
    const passed = C.standingPredicateKeys.every(key => d.predicates[key] === true);
    assert.equal(row.standingPassed,passed);
    consecutive = passed ? consecutive+1 : 0; maximum = Math.max(maximum,consecutive);
    assert.equal(row.consecutiveFullStandingSamples,consecutive);
    if (passed) {
      assert.ok(Array.isArray(row.prospectiveAnchorsUsed) && row.prospectiveAnchorsUsed.length===2);
      assert.deepEqual(row.prospectiveAnchorsForNext,row.prospectiveAnchorsUsed);
      if (previousAnchors !== null) assert.deepEqual(row.prospectiveAnchorsUsed,previousAnchors,'Prospective anchors cannot slide within an interval');
      else assert.deepEqual(row.prospectiveAnchorsUsed,d.feet,'A fresh prospective interval anchors at its first qualifying current sample');
    } else assert.equal(row.prospectiveAnchorsForNext,null,'Failed sample must clear the prospective interval');
    previousAnchors = row.prospectiveAnchorsForNext;
  }
  assert.equal(receipt.maximumConsecutiveFullStandingSamples,maximum);
  if (receipt.endpoint !== null) {
    assert.deepEqual(Object.keys(receipt.endpoint).sort(),[...C.endpointFields].sort());
    assert.equal(receipt.status,'feasible-full-standing-endpoint');
    assert.equal(receipt.failure,null);
    assert.equal(receipt.unobservedNativeTail,0,'Never credit an endpoint across an unobserved native tail');
    assert.ok(receipt.endpoint.consecutiveFullStandingSamples >= 80);
    assert.equal(receipt.endpoint.forecastNativeSubstep,receipt.forecastNativeIntegrations);
    const last=receipt.nativeTrace.at(-1);
    assert.equal(receipt.endpoint.decisionIndex,last.decisionIndex);
    assert.equal(receipt.endpoint.withinDecisionNative,last.withinDecisionNative);
    assert.deepEqual(receipt.endpoint.current,last.current);
    assert.deepEqual(receipt.endpoint.anchors,last.prospectiveAnchorsForNext);
    assert.equal(receipt.endpoint.consecutiveFullStandingSamples,last.consecutiveFullStandingSamples);
  } else if (receipt.failure === null) {
    assert.equal(receipt.status,'no-full-standing-endpoint-within-ceiling');
    assert.equal(receipt.forecastNativeIntegrations,5000);
    assert.equal(receipt.decisionsStarted,250);
  } else assert.equal(receipt.status,'integrity-failure');
  return { initial, frames, prefixProof:{ legacyArchiveSha256:C.legacyArchiveSha256,
    fullSeedExactlyEqual:true, first51CompleteFramesExactlyEqual:prefixLength===51,
    exactLegacyPrefixFrameCount:prefixLength,
    full51ProofUnestablishedDueToEarlyIntegrityFailure:prefixLength<51,
    seedCanonicalJsonSha256:jsonHash(initial), first51CanonicalJsonSha256:jsonHash(frames.slice(0,51)),
    fieldsOmitted:[], floatTolerance:null, contactsAndMotionPreserved:true,
    archiveSource:'direct typed-f32 JSON strings; full original legacy objects',
    currentLoadSource:'nativeTrace.current only; legacy TeacherFrame contacts are stale seed-cache evidence' } };
}

try {
  const { chromium,browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
  server=spawn(process.execPath,['scripts/serve-rust-preview.mjs',artifact,'0'],{stdio:['ignore','pipe','pipe']});
  let serverStderr=''; server.stderr.on('data',bytes=>{serverStderr+=String(bytes);});
  const origin=await new Promise((resolveOrigin,reject)=>{
    let stdout=''; const timer=setTimeout(()=>reject(new Error('Preview startup watchdog: '+serverStderr)),C.previewStartupWatchdogMs);
    server.once('error',error=>{clearTimeout(timer);reject(error);});
    server.once('exit',(code,signal)=>{clearTimeout(timer);reject(new Error(`Preview exited ${code}/${signal}: ${serverStderr}`));});
    server.stdout.on('data',bytes=>{stdout+=String(bytes);const url=stdout.match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];if(url){clearTimeout(timer);resolveOrigin(url);}});
  });
  browser=await chromium.launch(browserLaunchOptions(true));
  report.browserVersion=browser.version();
  context=await browser.newContext({viewport:{width:1280,height:900}});
  await context.addInitScript(({C,profile})=>{
    window.__lhGetUpTrial=true;
    let api;
    Object.defineProperty(window,'__lhRust',{configurable:true,get:()=>api,set(value){api=value;api.setManual(true);}});
    const canonical=x=>Array.isArray(x)?x.map(canonical):x&&typeof x==='object'?Object.fromEntries(Object.keys(x).sort().map(k=>[k,canonical(x[k])])):x;
    const equal=(a,b)=>JSON.stringify(canonical(a))===JSON.stringify(canonical(b));
    const e=window.__teacherFeasibilityEvidence={workerInstances:0,workerIdentities:[],initializations:[],requests:[],
      publications:[],activation:null,trigger:null,terminal:null,terminalSnapshot:null,terminalSeen:false,
      lastPublicationNative:null,terminalDuplicates:0,startedAtMs:null,endedAtMs:null,error:null,
      rawWorkerFaults:[],rawObserverFault:null,physicalInputs:[],stopReason:null,
      stop(reason,error=null){if(error!==null)this.error??=String(error);this.stopReason??=reason;this.endedAtMs??=performance.now();api?.setManual(true);}};
    const Native=window.Worker;
    window.Worker=class extends Native {
      constructor(...args){super(...args);e.workerInstances++;
        // Registered before the unchanged driver sets its onmessage. Pausing
        // here must precede its reply pump, including malformed/error receipts.
        this.addEventListener('message',event=>{
          const m=event.data;
          if(m.fixtureTest?.teacherTerminalEvidence?.terminal){e.terminalSeen=true;api?.setManual(true);}
          try{
            if(m.type==='ready')e.workerIdentities.push(m.workerIdentity);
            if(m.type==='error'){e.rawWorkerFaults.push(structuredClone(m));e.stop('worker-error',m.message);return;}
            if(!m.reply)return;
            const f=m.fixtureTest,s=m.reply.snapshot,n=Math.round(s.simulation_time_s*800),t=f?.teacherTerminalEvidence;
            if(!f?.isolatedTestOnly||f.kind!=='back'||f.preparationPollingNative!==80||!equal(f.profile,profile))throw new Error('Original BACK80-native complete fixture profile missing');
            if(s.simulation_time_s!==n/800||s.stamp.generation!==1||n>480)throw new Error('Real native clock/generation changed or exceeded480');
            const last=e.lastPublicationNative;
            if(last===null){if(n!==0||m.reply.completed_ticks!==0)throw new Error('Only initial native0 zero-tick publication is allowed');}
            else if(n===last){
              if(n!==480||m.reply.completed_ticks!==0||!t?.terminal||e.terminalDuplicates!==0)throw new Error('Only one terminal zero-tick duplicate480 is permitted');
              e.terminalDuplicates++;
            }else if(n-last!==20||m.reply.completed_ticks!==1||t!==null)throw new Error('Preparation must retain original20-native publications');
            if(f.carryActivationEvidence){if(e.activation!==null)throw new Error('Carry activation repeated');e.activation=structuredClone(f.carryActivationEvidence);}
            if(f.triggerEvidence){if(e.trigger!==null||f.triggerEvidence.native!==480)throw new Error('Natural trigger must occur exactly once at480');e.trigger=structuredClone(f.triggerEvidence);}
            if(f.triggerNative!==null&&f.triggerNative!==480)throw new Error('Unexpected trigger epoch');
            e.publications.push({native:n,completedTicks:m.reply.completed_ticks,requestId:m.requestId,
              stamp:structuredClone(s.stamp),motion:s.motion,counters:structuredClone(s.counters)});
            e.lastPublicationNative=n;
            if(t){
              e.terminal=structuredClone(t);e.terminalSnapshot=structuredClone(s);
              if(n!==480||last!==480||e.trigger?.native!==480||t.exportCalls!==1||t.terminalLatchBeforeCall!==true)throw new Error('Terminal receipt/call boundary invalid');
              e.stop('teacher-export-terminal480');
            }
          }catch(error){e.rawObserverFault={error:String(error),stack:error?.stack??null,message:structuredClone(m)};e.stop('instrumentation-error',String(error));}
        });
      }
      postMessage(message,...args){
        if(e.terminalSeen&&!['recycle','shutdown','getupReport','getupCheckpoint'].includes(message.type)){
          e.stop('post-terminal-request','All advances/commands/resets prohibited after terminal receipt');throw new Error(e.error);
        }
        if(message.type==='initialize'){
          if(e.initializations.length!==0){e.stop('runtime-replacement','Only one fresh runtime allowed');throw new Error(e.error);}
          message={...message,testFixedGetUpFixture:'back',testFixedGetUpProfile:profile};e.initializations.push(structuredClone(message));
        }
        if(message.commands?.length||message.type==='commands'||message.type==='reset'){
          e.stop('physical-command-or-reset','Physical commands and resets are forbidden');throw new Error(e.error);
        }
        if(message.type==='advance'&&message.ticks!==1){e.stop('nonordinary-advance','Only original ordinary one-tick requests are allowed');throw new Error(e.error);}
        e.requests.push({type:message.type,requestId:message.requestId??null,ticks:message.ticks??null,afterTerminal:e.terminalSeen});
        return super.postMessage(message,...args);
      }
    };
    for(const type of ['keydown','keyup','pointerdown'])document.addEventListener(type,event=>{
      if(e.startedAtMs!==null&&e.endedAtMs===null){e.physicalInputs.push({type,code:event.code??null,isTrusted:event.isTrusted});e.stop('unexpected-physical-input','No physical input is permitted');}
    },true);
  },{C,profile});
  page=await context.newPage();
  page.on('pageerror',error=>report.errors.push({source:'page',error:String(error),stack:error.stack??null}));
  page.on('response',response=>{if(response.status()>=400)report.errors.push({source:'http',status:response.status(),url:response.url()});});
  await page.goto(origin+'?renderer=webgl2',{timeout:C.browserStartupWatchdogMs});
  await page.waitForFunction(()=>window.__teacherFeasibilityEvidence?.error||window.__lhRust?.diagnostics.ready&&window.__lhRust.diagnostics.graphics.frames>=3,null,{timeout:C.browserStartupWatchdogMs});
  report.initial=await page.evaluate(()=>window.__lhRust?.diagnostics);
  assert.equal(report.initial.tick,0);assert.equal(report.initial.time,0);assert.equal(report.initial.backend,'WebGL2');
  await page.screenshot({path:join(output,'initial-fresh-real-back-native0.png')});
  await page.evaluate(()=>{const e=window.__teacherFeasibilityEvidence;if(e.error)throw new Error(e.error);e.startedAtMs=performance.now();window.__lhRust.setManual(false);});
  await page.waitForFunction(()=>window.__teacherFeasibilityEvidence?.endedAtMs!==null||window.__lhRust?.diagnostics.workerFault,null,{timeout:C.terminalWallWatchdogMs});
  await page.waitForFunction(()=>!window.__lhRust?.diagnostics.outstanding||window.__lhRust?.diagnostics.workerFault,null,{timeout:C.readOnlyIdleWatchdogMs});
  report.final=await page.evaluate(()=>window.__lhRust?.diagnostics);
  await page.screenshot({path:join(output,'terminal-real-back-trigger480.png')});
} catch(error){report.errors.push({source:'recorder',error:String(error),stack:error.stack??null});}
finally{
  // Preserve the original terminal payload and raw errors even if validation
  // or the unchanged driver's request watchdog fails. Never resurrect workers.
  if(page){
    try{
      const evidence=await page.evaluate(()=>{
        const e=window.__teacherFeasibilityEvidence;if(!e)return null;
        const encode=bytes=>{if(!bytes)return null;let value='';for(let i=0;i<bytes.length;i+=0x8000)value+=String.fromCharCode(...bytes.subarray(i,i+0x8000));return btoa(value);};
        const { stop, ...data } = e;
        const clone=structuredClone(data);
        for(const key of ['activation','terminal']){
          const value=clone[key];if(!value)continue;
          value.beforeCheckpointBase64=encode(value.beforeCheckpoint);value.afterCheckpointBase64=encode(value.afterCheckpoint);
          delete value.beforeCheckpoint;delete value.afterCheckpoint;
        }
        return clone;
      });
      if(evidence){
        // This untouched payload, including checkpoint base64 and raw export
        // errors/JSON, is durable before ANY decoder or semantic assertion.
        saveJson('browser-evidence-raw.json',evidence);
        report.rawBrowserEvidence={path:join(output,'browser-evidence-raw.json'),sha256:hash(readFileSync(join(output,'browser-evidence-raw.json')))};
        const a=evidence.activation,t=evidence.terminal;
        if(a){
          const before=Buffer.from(a.beforeCheckpointBase64??'','base64'),after=Buffer.from(a.afterCheckpointBase64??'','base64');
          delete a.beforeCheckpointBase64;delete a.afterCheckpointBase64;
          a.beforeCheckpoint=saveRaw('activation-before-checkpoint.bin',before);
          a.afterCheckpoint=saveRaw('activation-after-checkpoint.bin',after);
          // Native0 activation proof stays separate from export-only equality.
          a.independentActivationProof=proveActivationPurity(before,after);
          saveJson('activation-proof.json',a);
        }
        if(t){
          const before=Buffer.from(t.beforeCheckpointBase64??'','base64'),after=Buffer.from(t.afterCheckpointBase64??'','base64');
          delete t.beforeCheckpointBase64;delete t.afterCheckpointBase64;
          t.beforeCheckpoint=saveRaw('teacher-export-before-checkpoint.bin',before);
          t.afterCheckpoint=saveRaw('teacher-export-after-checkpoint.bin',after);
          t.independentWholeCheckpointBytesEqual=before.equals(after)&&before.length>0;
          t.checkpointSchemaBefore=before.length?parseGetUpCheckpoint(before).schema:null;
          t.checkpointSchemaAfter=after.length?parseGetUpCheckpoint(after).schema:null;
          for(const [field,name]of [['beforeControllerJson','before-controller.json'],['afterControllerJson','after-controller.json'],['beforeFixtureJson','before-fixture.json'],['afterFixtureJson','after-fixture.json']]){
            if(typeof t[field]==='string')t[field+'Artifact']=saveRaw(name,t[field]);
          }
          if(typeof t.rawExportJson==='string'){
            const raw=t.rawExportJson;t.rawExportArtifact=saveRaw('teacher-feasibility-raw-export.json',raw);delete t.rawExportJson;
            // Archive everything before assertions, including a negative or
            // malformed receipt, not only a successful standing endpoint.
            const receipt=JSON.parse(raw),fields=C.archiveFields;
            if(typeof receipt[fields.typedInitialJson]==='string')saveRaw('teacher-initial-typed.json',receipt[fields.typedInitialJson]);
            if(typeof receipt[fields.typedFramesJson]==='string')saveRaw('teacher-frames-typed.json',receipt[fields.typedFramesJson]);
            if(receipt.nativeTrace)saveJson('teacher-native-trace.json',receipt.nativeTrace);
            const validated=validateReceipt(receipt);
            report.prefixProof=validated.prefixProof;
            report.teacherOutcome={status:receipt.status,forecastNativeIntegrations:receipt.forecastNativeIntegrations,
              decisionsStarted:receipt.decisionsStarted,originalHeightCutDecisions:receipt.originalHeightCutDecisions,
              maximumConsecutiveFullStandingSamples:receipt.maximumConsecutiveFullStandingSamples,
              endpoint:receipt.endpoint,failure:receipt.failure,forecastOnly:true};
          }
        }
        // Save evidence before the final invariant audit, even when it fails.
        saveJson('browser-evidence.json',evidence);
        report.browserEvidence={path:join(output,'browser-evidence.json'),sha256:hash(readFileSync(join(output,'browser-evidence.json')))};
        assert.equal(evidence.error,null);assert.equal(evidence.workerInstances,1);assert.equal(evidence.workerIdentities.length,1);
        assert.equal(evidence.initializations.length,1);assert.equal(evidence.physicalInputs.length,0);
        assert.equal(evidence.rawWorkerFaults.length,0);assert.equal(evidence.rawObserverFault,null);
        assert.equal(evidence.terminalDuplicates,1);assert.equal(evidence.lastPublicationNative,480);
        assert.equal(evidence.publications.length,26); // native0, twenty-four ticks, one duplicate480.
        assert.equal(evidence.trigger.native,480);assert.equal(evidence.trigger.physicalPoseVelocityBitsUnchanged,true);
        assert.equal(a.activationCalls,1);assert.equal(a.beforeReport.nativeSubsteps,0);assert.equal(a.afterReport.nativeSubsteps,0);
        assert.equal(t.exportCalls,1);assert.equal(t.terminalLatchBeforeCall,true);assert.equal(t.exportError,null);assert.equal(t.captureError,null);
        assert.deepEqual(t.invariantErrors,[]);assert.equal(t.independentWholeCheckpointBytesEqual,true);
        assert.equal(t.checkpointBytesExactlyEqual,true);assert.equal(t.controllerReportExactlyEqual,true);assert.equal(t.fixtureReportExactlyEqual,true);
        assert.equal(t.beforeControllerJson,t.afterControllerJson);assert.equal(t.beforeFixtureJson,t.afterFixtureJson);
        const controller=JSON.parse(t.beforeControllerJson),fixture=JSON.parse(t.beforeFixtureJson);
        assert.equal(controller.started_substep,null);assert.equal(controller.impact_substep,480);
        assert.deepEqual(controller.teacher_frames,[]);assert.deepEqual(controller.reference,[]);
        assert.equal(fixture.nativeSubsteps,480);assert.deepEqual(fixture.profile,profile);
        assert.ok(evidence.requests.filter(r=>r.afterTerminal).every(r=>['recycle','shutdown','getupReport','getupCheckpoint'].includes(r.type)));
        assert.equal(report.final?.time,.6);assert.equal(report.final?.tick,24);assert.equal(report.final?.workerFault,false);
        assert.deepEqual(report.final.snapshot.counters,evidence.publications.at(-2).counters,'Trigger/export may not credit real recovery counters');
        assert.equal(report.errors.length,0);assert.ok(report.prefixProof);
        report.observationComplete=true;report.passed=true;
      }
    }catch(error){report.errors.push({source:'evidence-or-invariant-audit',error:String(error),stack:error.stack??null});}
  }
  try{await context?.close();}catch(error){report.errors.push({source:'context-close',error:String(error)});}
  try{await browser?.close();}catch(error){report.errors.push({source:'browser-close',error:String(error)});}
  server?.kill('SIGTERM');
  report.passed=report.passed&&report.errors.length===0;
  saveJson('result.json',report);
}
console.log(JSON.stringify({passed:report.passed,observationComplete:report.observationComplete,outcome:report.teacherOutcome?.status??null,result:join(output,'result.json')}));
if(!report.passed)process.exitCode=1;
