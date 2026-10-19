// SOURCE-ONLY gated maker. Running it requires a later independently reviewed GO.
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {readFileSync,readdirSync,writeFileSync,createReadStream} from 'node:fs';
import {createInterface} from 'node:readline';
import {join,relative,resolve,dirname} from 'node:path';
import {createHash} from 'node:crypto';
import {fileURLToPath,pathToFileURL} from 'node:url';
const here=dirname(fileURLToPath(import.meta.url));
const contractPath=join(here,'physical-roll-contract-2026-10-19.json');
const C=JSON.parse(readFileSync(contractPath));
assert.equal(C.executionApproved,true,'SOURCE ONLY: first actual-WASM physical evaluation gate is false');
assert.equal(C.schemaPinned,true,'Independent final harness audit and owner GO are required');
assert.equal(process.argv.length,6,'Usage: maker INPUT FRESH_OUTPUT FROZEN_MANIFEST PROFILE');
const input=resolve(process.argv[2]),output=resolve(process.argv[3]),manifestPath=resolve(process.argv[4]),profilePath=resolve(process.argv[5]);
const hash=b=>createHash('sha256').update(b).digest('hex');
const canonical=v=>Array.isArray(v)?v.map(canonical):v&&typeof v==='object'?Object.fromEntries(Object.keys(v).sort().map(k=>[k,canonical(v[k])])):v;
const jsonHash=v=>hash(JSON.stringify(canonical(v)));
const pick=(v,keys)=>Object.fromEntries(keys.map(k=>{assert.ok(Object.hasOwn(v,k),'Missing baseline field '+k);return[k,v[k]];}));
for(const [name,value]of Object.entries(C.pendingFreeze))assert.match(value??'',/^[a-f0-9]{64}$/,'Unpinned source '+name);
assert.equal(hash(readFileSync(fileURLToPath(import.meta.url))),C.pendingFreeze.adapterSourceSha256);
const fragmentPath=join(here,'physical-roll-worker-harness-2026-10-19.js');
assert.equal(hash(readFileSync(fragmentPath)),C.pendingFreeze.workerHarnessSourceSha256);
for(const pin of [C.predeclaredContract,C.checkpointHelper,C.fixedMaker,C.nativeSourceFreeze,C.fixedPresentationManifest])assert.equal(hash(readFileSync(resolve(pin.path))),pin.sha256,pin.path);
assert.equal(hash(readFileSync(manifestPath)),C.pendingFreeze.approvedAppManifestSha256);
const frozen=JSON.parse(readFileSync(manifestPath));
assert.equal(frozen.sourceBase,C.sourceCommit);
assert.equal(frozen.sourceManifest.sha256,C.pendingFreeze.nativeSourceManifestSha256);
assert.equal(hash(readFileSync(resolve(frozen.sourceManifest.path))),C.pendingFreeze.nativeSourceManifestSha256);
const sourceManifest=JSON.parse(readFileSync(resolve(frozen.sourceManifest.path)));
assert.equal(sourceManifest.sourceBase,C.sourceCommit);
assert.equal(Object.keys(sourceManifest.sourceFiles).length,372,'Exact frozen372 source files');
for(const [path,expected]of Object.entries(sourceManifest.sourceFiles))assert.equal(hash(readFileSync(join(dirname(manifestPath),'source',path))),expected,'Frozen native source '+path);
assert.deepEqual([...frozen.extraFeatures].sort(),[...C.features].sort(),'Prior five CLI features plus only physical-roll');
assert.equal(hash(readFileSync(profilePath)),C.profileSha256);
const profile=JSON.parse(readFileSync(profilePath));
const presentation=JSON.parse(readFileSync(resolve(C.fixedPresentationManifest.path)));
for(const [path,expected]of Object.entries(presentation.artifactSha256))if(!C.fixedPresentationManifest.allowedChangedAssets.includes(path))assert.equal(hash(readFileSync(join(input,path))),expected,'Fixed renderer/driver/transport '+path);
assert.equal(hash(readFileSync(resolve(C.baselinePreparation.path))),C.baselinePreparation.sha256);
const prefix={native0Sha256:null,nativeSha256:{},native0BaselineStamp:null};
let count=0;
const lines=createInterface({input:createReadStream(resolve(C.baselinePreparation.path)),crlfDelay:Infinity});
for await(const line of lines){
  if(!line.trim())continue;
  const row=JSON.parse(line),n=row.nativeSubsteps;
  if(n===0){assert.equal(prefix.native0Sha256,null);prefix.native0Sha256=jsonHash(pick(row,C.baselinePreparation.native0Fields));prefix.native0BaselineStamp=row.snapshot.stamp;continue;}
  if(n>480)break;
  assert.equal(n,++count,'Continuous original passive native prefix');
  prefix.nativeSha256[n]=jsonHash(pick(row,C.baselinePreparation.nativeFields));
}
lines.close();assert.equal(count,480);assert.ok(prefix.native0Sha256);
const {parseGetUpCheckpoint,proveActivationPurity}=await import(pathToFileURL(resolve(C.checkpointHelper.path)).href);
const built=spawnSync(process.execPath,[C.fixedMaker.path,input,output,manifestPath],{encoding:'utf8'});
assert.equal(built.status,0,built.stderr||built.stdout);
let source=readFileSync(join(output,'sim-worker.js'),'utf8');
assert.equal(hash(source),'da24141dabdd72cba4e57c84b6d40cdd2e876f8ac1f9555c369db8ed33947df2');
function replaceOnce(a,b){assert.equal(source.split(a).length,2,'Unique frozen insertion: '+a.slice(0,90));source=source.replace(a,b);}
const fragment=readFileSync(fragmentPath,'utf8').replace('__CONTRACT_JSON__',JSON.stringify(C)).replace('__PROFILE_JSON__',JSON.stringify(profile)).replace('__PREFIX_REFERENCE_JSON__',JSON.stringify(prefix));
replaceOnce('let postPassAnchors = null;',`let postPassAnchors = null;\n${fragment}\n${parseGetUpCheckpoint.toString()}\n${proveActivationPurity.toString()}`);
replaceOnce('self.onmessage = event => {','self.onmessage = async event => {');
replaceOnce("if (!['back', 'belly', 'side', 'side-supported'].includes(message.testFixedGetUpFixture)","if (message.testFixedGetUpFixture !== 'back'");
replaceOnce('      fixedFixtureProfile = JSON.parse(getup_trial_profile());',`      const sourceBaseline=JSON.parse(getup_trial_profile());
      sourceBaseline.id=approvedFixtureProfile.id;
      sourceBaseline.solver_iterations=3;sourceBaseline.additional_body_iterations=1;
      sourceBaseline.getup_solver_budget='consistent-recovery-phase-v1';
      sourceBaseline.root_rotation_normalization='unit-free-root-v1';
      sourceBaseline.getup_repeated_recovery='completed-cycle-confirmed-fall-v1';
      if(!sameJson(sourceBaseline,approvedFixtureProfile)||!sameJson(message.testFixedGetUpProfile,approvedFixtureProfile)) throw new Error('Exact full aaa415a1 repeat profile required');
      fixedFixtureProfile=message.testFixedGetUpProfile;`);
replaceOnce('fixedFixtureProfile.solver_iterations !== 20 || fixedFixtureProfile.additional_body_iterations !== 5','fixedFixtureProfile.solver_iterations !== 3 || fixedFixtureProfile.additional_body_iterations !== 1');
replaceOnce('      const next = WorkerRuntime.getup_fixture(fixedFixture);','      const next = WorkerRuntime.getup_fixture_with_profile(fixedFixture,JSON.stringify(fixedFixtureProfile));');
replaceOnce('      runtime = next;',`      runtime=next;
      rollBuildIdentity=JSON.parse(runtime.terminal_tail_build_identity());
      if(rollBuildIdentity.experiment!=='native-motor-observer-v1'||rollBuildIdentity.variant!=='source-normal-motors'
        ||rollBuildIdentity.checkpointResumeSupported!==true||rollBuildIdentity.checkpointExportSupported!==true
        ||rollBuildIdentity.nativeTrace!=='read-only-post-native-observer-v1'||rollBuildIdentity.maxTicksPerCall!==1
        ||rollBuildIdentity.observerFeature!=='getup-terminal-tail-observer-experiment') throw new Error('Original native observer identity required');
      const readNative0=()=>{
        const o=JSON.parse(runtime.advance_terminal_tail_observed(0)),s=o.native0State;
        if(o.traceError!==null||o.nativeTrace.length!==0||!sameJson(o.buildIdentity,rollBuildIdentity)||!s
          ||s.nativeSubsteps!==0||s.physicsIntegrations!==0||s.physicalObservationError!==null||s.integratorReadError!==null
          ||s.poseWords?.length!==325||s.physicalObservationWords?.length!==67)throw new Error('Native0 immutable observer receipt invalid');
        s.current=JSON.parse(runtime.getup_standing_diagnostic('null'));return o;
      };
      const beforeObserved=readNative0(),beforeRefresh=beforeObserved.reply,beforeNative0State=beforeObserved.native0State;
      const beforeReport=JSON.parse(runtime.getup_fixture_report());
      const beforeCheckpoint=runtime.getup_checkpoint();
      const beforeVerification=JSON.parse(runtime.verify_getup_checkpoint(beforeCheckpoint));
      if(beforeReport.nativeSubsteps!==0||beforeReport.rootTranslationCompensation!==undefined||beforeReport.physicalRoll!==undefined
        ||!sameJson(beforeReport.profile,approvedFixtureProfile))throw new Error('Fresh inactive native0 fixture required');
      runtime.set_root_translation_compensation_enabled(true);
      // Exact binary purity bracket ends BEFORE another legitimate zero-tick receipt.
      const afterCheckpoint=runtime.getup_checkpoint();
      const afterVerification=JSON.parse(runtime.verify_getup_checkpoint(afterCheckpoint));
      const purity=proveActivationPurity(beforeCheckpoint,afterCheckpoint);
      const afterReport=JSON.parse(runtime.getup_fixture_report());
      const afterObserved=readNative0(),afterRefresh=afterObserved.reply,afterNative0State=afterObserved.native0State;
      if(!sameJson(beforeReport.solverBudget,afterReport.solverBudget)||!sameJson(beforeReport.rootRotationNormalization,afterReport.rootRotationNormalization)
        ||afterReport.rootTranslationCompensation?.algorithm!==purity.algorithm||!sameJson(afterReport.rootTranslationCompensation.activeRoots,[purity.root])
        ||!sameJson(beforeNative0State.poseWords,afterNative0State.poseWords)||!sameJson(beforeNative0State.physicalObservationWords,afterNative0State.physicalObservationWords)
        ||!sameJson(beforeNative0State.current,afterNative0State.current)||beforeRefresh.completed_ticks!==0||afterRefresh.completed_ticks!==0
        ||beforeRefresh.snapshot.simulation_time_s!==0||afterRefresh.snapshot.simulation_time_s!==0
        ||beforeRefresh.snapshot.diagnostics.getup.policy.forecast_substeps!==0||afterRefresh.snapshot.diagnostics.getup.policy.forecast_substeps!==0
        ||![beforeVerification,afterVerification].every(v=>v.passed===true&&v.physicsIntegrations===0&&v.nativeSubsteps===0&&v.forecastSubsteps===0&&v.rootIntegratorStateEqual===true))throw new Error('Carry native0 physical/profile/binary purity violation');
      const actual0hash=await digestJson(subset(afterNative0State,rollContract.baselinePreparation.native0Fields));
      if(actual0hash!==rollPrefixReference.native0Sha256)throw new Error('Exact baseline native0 physical/current/carry mismatch');
      rollPrefixEvidence.native0match=true;rollPrefixEvidence.native0ActualStamp=afterRefresh.snapshot.stamp;
      const bs=rollPrefixReference.native0BaselineStamp,as=afterRefresh.snapshot.stamp;
      if(bs.schema!==as.schema||bs.generation!==as.generation||bs.tick!==as.tick)throw new Error('Native0 stamp differs outside observation lifecycle sequence');
      rollPrefixEvidence.native0StampAccounting={sequenceDelta:as.sequence-bs.sequence,physicalTickEqual:true,generationEqual:true,
        explanation:'Only zero-tick publication observation sequence may differ; physical/current/carry comparison is exact and contains no stamp exclusion.'};
      fixedFixtureProfile=afterReport.profile;rollNative0State=afterNative0State;
      rollNative0Controller=JSON.parse(runtime.getup_report());
      carryActivationEvidence={activationCalls:1,purity,beforeReport,afterReport,beforeNative0State,afterNative0State,
        beforeVerification,afterVerification,beforeCheckpoint,afterCheckpoint,poseVelocityF32WordsBefore:poseWords(beforeRefresh.snapshot),
        poseVelocityF32WordsAfter:poseWords(afterRefresh.snapshot),nativePhysicsBefore:0,nativePhysicsAfter:0,forecastBefore:0,forecastAfter:0};`);
replaceOnce('  if (!fixture.state.ready) {',`  if(native>=480&&!fixture.state.ready)throw new Error('Original BACK not ready at480');
  if(fixture.state.ready&&native!==480)throw new Error('Original BACK ready at unexpected polling boundary');
  if(!fixture.state.ready) {`);
replaceOnce('  const before = poseWords(lastFixtureSnapshot);',`  if(!rollPrefixEvidence.native0match||rollPrefixEvidence.verifiedThroughNative!==480||rollPrefixEvidence.matchedNativeRows!==480)throw new Error('Passive prefix incomplete before roll481');
  rollPrefixEvidence.triggerReadyAt480=true;rollPrefixEvidence.completeBeforeRoll481=true;
  const before = poseWords(lastFixtureSnapshot);`);
replaceOnce('  lastFixtureSnapshot = refreshed.snapshot;','  lastFixtureSnapshot = refreshed.snapshot;\n  triggerEvidence.controllerReport=JSON.parse(runtime.getup_report());');
// Replace only the fixed maker's read-only40Hz standing observer prologue.
const pubStart=source.indexOf('function publish(requestId, reply, started, timing) {');
const slotStart=source.indexOf('  const slot = buffers.find',pubStart);
assert.ok(pubStart>=0&&slotStart>pubStart);
source=source.slice(0,pubStart)+`function publish(requestId, reply, started, timing) {
  lastFixtureSnapshot=reply.snapshot;
  const rollPublication=readRollPublication(reply);
`+source.slice(slotStart);
replaceOnce(`    fixtureTest: { isolatedTestOnly: true, kind: fixedFixture, profile: fixedFixtureProfile,
      preparationPollingNative: fixedFixture?.startsWith('side') ? 20 : 80,
      triggerNative, triggerEvidence, postPassAnchors, postPassStanding },`,'    fixtureTest:rollPublication,');
replaceOnce('  triggerEvidence = null;','  triggerEvidence=null;carryActivationEvidence=null;rollNativeTrace=[];rollNative0State=null;rollNative0Controller=null;rollTraceError=null;');
replaceOnce('    prepareFixtureTrigger(message);',`    if(rollTerminal)throw new Error('Physical roll terminal latch forbids further advances');
    prepareFixtureTrigger(message);`);
const originalAdvance=`    if (profiling) {
      const json = runtime.advance_profiled(ticks);
      const parseStarted = performance.now();
      const result = JSON.parse(json);
      const parseMs = performance.now() - parseStarted;
      publish(message.requestId, result.reply, started, { ticks: result.timings, clockValid: result.clockValid,
        runtimeMs: result.runtimeMs, runtimeNonTickMs: result.runtimeNonTickMs,
        nativeOutputMs: runtime.native_output_ms(), jsonParseMs: parseMs });
    } else publish(message.requestId, JSON.parse(runtime.advance(ticks)), started, null);`;
replaceOnce(originalAdvance,`    rollLastRawReply=null;
    const observed=JSON.parse(runtime.advance_terminal_tail_observed(ticks));
    rollLastRawReply=observed.reply;
    rollNativeTrace=observed.nativeTrace;rollTraceError=observed.traceError;
    if(observed.native0State) {
      observed.native0State.current=JSON.parse(runtime.getup_standing_diagnostic('null'));
      if(await digestJson(subset(observed.native0State,rollContract.baselinePreparation.native0Fields))!==rollPrefixReference.native0Sha256)throw new Error('Published native0 physical/current/carry mismatch');
      rollNative0State=observed.native0State;rollPrefixEvidence.native0ActualStamp=observed.reply.snapshot.stamp;
      rollPrefixEvidence.native0StampAccounting.sequenceDelta=observed.reply.snapshot.stamp.sequence-rollPrefixReference.native0BaselineStamp.sequence;
    }
    if(!Array.isArray(rollNativeTrace)||rollNativeTrace.length!==(ticks===1?20:0)||!sameJson(observed.buildIdentity,rollBuildIdentity)||rollTraceError!==null)
      throw new Error('Bounded native observer failed: '+JSON.stringify(rollTraceError));
    await verifyRollPrefix(rollNativeTrace);
    inspectRollNativeRows(rollNativeTrace);
    publish(message.requestId,observed.reply,started,null);`);
replaceOnce("    self.postMessage({ type: 'error', requestId: message?.requestId ?? null, workerIdentity, message: String(error) });",`    rollTraceError=rollTraceError??String(error);stopRoll('worker-error',error);
    self.postMessage({type:'error',requestId:message?.requestId??null,workerIdentity,message:String(error),
      rawObservedReply:rollLastRawReply,
      fixtureTest:{isolatedTestOnly:true,kind:fixedFixture,rollNativeTrace,rollTraceError,rollNative0State,rollBuildIdentity,
        carryActivationEvidence,triggerEvidence,rollPrefixEvidence:{...rollPrefixEvidence},rollTerminal}});`);
writeFileSync(join(output,'sim-worker.js'),source);
function hashes(root,base=root){return Object.fromEntries(readdirSync(root,{withFileTypes:true}).flatMap(e=>{const p=join(root,e.name);return e.isDirectory()?Object.entries(hashes(p,base)):[[relative(base,p),hash(readFileSync(p))]];}));}
const manifest=JSON.parse(readFileSync(output+'-fixture-adapter-manifest.json'));
manifest.baseFixtureAdapterSha256=manifest.adapterSha256;manifest.adapterSha256=hash(source);
manifest.physicalRollSource={path:relative(resolve('.'),fileURLToPath(import.meta.url)),sha256:hash(readFileSync(fileURLToPath(import.meta.url))),workerHarnessSha256:hash(readFileSync(fragmentPath))};
manifest.physicalRollContract={path:relative(resolve('.'),contractPath),sha256:hash(readFileSync(contractPath)),value:C};
manifest.explicitProfile={path:profilePath,sha256:hash(readFileSync(profilePath)),value:profile};
manifest.passivePrefixReference={...prefix,fields:C.baselinePreparation,referenceJsonSha256:jsonHash(prefix)};
manifest.scope='First fresh original BACK physical-roll actual-WASM test only. Original fixed presentation/driver/transport/scheduler, binary-pure native0 carry once, exact passive prefix0..480, original trigger+zero-tick325word purity, full bounded native observer, source physical roll route and original deadline/anchors/counter. All publication summaries retained, no physical commands or retries. No performance or feasibility claim from preparation.';
manifest.activeAlgorithm='f32-dq-f64-remainder-v1';manifest.unitRootAlgorithm='unit-free-root-v1';manifest.artifactSha256=hashes(output);
writeFileSync(output+'-fixture-adapter-manifest.json',JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared isolated physical-roll app '+output);
