// Single authorized geometry-baseline replay. Original WASM, no new candidate.
import assert from 'node:assert/strict';
import {assertExactArchiveRow} from './exact-archive-row-parity.mjs';
import {readFileSync,writeFileSync,mkdirSync,existsSync,createReadStream} from 'node:fs';
import {createGunzip} from 'node:zlib';
import {createInterface} from 'node:readline';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
const lane=resolve('outputs/browser-capability-20261019/recovery-fixture/physical-roll');
const output=lane+'/prestall-context/replay-native1940-archive-boundary';
assert.equal(existsSync(output),false,'One-shot fresh directory required');mkdirSync(output);
const hash=b=>createHash('sha256').update(b).digest('hex');
const wasmPath=lane+'/build/bindgen-roll/lh_worker_bg.wasm',gluePath=lane+'/build/bindgen-roll/lh_worker.js';
const wasm=readFileSync(wasmPath),glue=readFileSync(gluePath);
assert.equal(hash(wasm),'0ca955fe762b23b462efa64d1e43295df94226dbc097ec14cbb2dc88e0567ca5');
assert.equal(hash(glue),'4ed81dc0ca68e43443b4db96730d52a827cad00a3c070195b293ed276452b84c');
const profile=readFileSync('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json');
assert.equal(hash(profile),'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
const write=(name,v)=>writeFileSync(output+'/'+name,JSON.stringify(v,null,2)+'\n');
const input=createReadStream(lane+'/actual-back-roll/native-trace.ndjson.gz').pipe(createGunzip());
const lines=createInterface({input,crlfDelay:Infinity})[Symbol.asyncIterator]();
let runtime=null,native=0,rowsMatched=0;const actualLines=[];
try {
 const {initSync,WorkerRuntime}=await import(pathToFileURL(gluePath));initSync({module:wasm});
 runtime=WorkerRuntime.getup_fixture_with_profile('back',JSON.stringify(JSON.parse(profile)));
 const identity=JSON.parse(runtime.terminal_tail_build_identity());
 assert.equal(identity.variant,'source-normal-motors');
 const initial=JSON.parse(runtime.advance_terminal_tail_observed(0));assert.equal(initial.nativeTrace.length,0);
 const activationBefore=runtime.getup_checkpoint(); runtime.set_root_translation_compensation_enabled(true);
 const activationAfter=runtime.getup_checkpoint();
 assert.deepStrictEqual(Buffer.from(activationBefore),readFileSync(lane+'/actual-back-roll/activation-before-checkpoint.bin'));
 assert.deepStrictEqual(Buffer.from(activationAfter),readFileSync(lane+'/actual-back-roll/activation-after-checkpoint.bin'));
 const after=JSON.parse(runtime.advance_terminal_tail_observed(0));assert.equal(after.nativeTrace.length,0);
 const zero=JSON.parse(runtime.advance_terminal_tail_observed(0));assert.equal(zero.nativeTrace.length,0);
 zero.native0State.current=JSON.parse(runtime.getup_standing_diagnostic('null'));
 const row0={receiptKind:'native0-read-only-state',...zero.native0State,snapshot:zero.reply.snapshot};
 assertExactArchiveRow(row0,(await lines.next()).value,'Exact original native0');
 actualLines.push(JSON.stringify(row0));rowsMatched++;
 let reply=zero.reply,triggerNative=null;
 while(native<1940) {
  if(triggerNative===null && native>0 && native%80===0) {
   const f=JSON.parse(runtime.getup_fixture_report());assert.equal(f.nativeSubsteps,native);
   if(f.state.ready) {
    assert.equal(native,480);runtime.trigger_getup_fixture();
    const refreshed=JSON.parse(runtime.advance(0));assert.equal(refreshed.completed_ticks,0);
    assert.equal(refreshed.snapshot.simulation_time_s,reply.snapshot.simulation_time_s);
    const words=s=>Array.from(new Uint32Array(new Float32Array(s.segments.flatMap(p=>[p.position.x,p.position.y,p.position.z,p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w,p.linearVelocity.x,p.linearVelocity.y,p.linearVelocity.z,p.angularVelocity.x,p.angularVelocity.y,p.angularVelocity.z])).buffer));
    assert.deepStrictEqual(words(refreshed.snapshot),words(reply.snapshot));reply=refreshed;triggerNative=native;
   }
  }
  const observed=JSON.parse(runtime.advance_terminal_tail_observed(1));
  assert.equal(observed.traceError,null);assert.equal(observed.nativeTrace.length,20);assert.deepStrictEqual(observed.buildIdentity,identity);
  for(const row of observed.nativeTrace) {
   const expectedRawLine=(await lines.next()).value;assert.equal(row.nativeSubsteps,native+1);
   assertExactArchiveRow(row,expectedRawLine,'Exact retained native'+row.nativeSubsteps);
   actualLines.push(JSON.stringify(row));native=row.nativeSubsteps;rowsMatched++;
  }
  reply=observed.reply;
 }
 assert.equal(triggerNative,480);assert.equal(native,1940);assert.equal(rowsMatched,1941);
 const controllerBefore=runtime.getup_report(),fixtureBefore=runtime.getup_fixture_report();
 const checkpointA=runtime.getup_checkpoint(),checkpointB=runtime.getup_checkpoint();
 assert.deepStrictEqual(checkpointA,checkpointB,'Pure checkpoint repeated exports differ');
 assert.equal(runtime.getup_report(),controllerBefore);assert.equal(runtime.getup_fixture_report(),fixtureBefore);
 const fixture=JSON.parse(fixtureBefore);assert.equal(fixture.nativeSubsteps,1940);assert.equal(fixture.physicalRoll.sourceForecastsGenerated,0);
 writeFileSync(output+'/checkpoint-native1940.bin',checkpointA);
 writeFileSync(output+'/checkpoint-native1940-second.bin',checkpointB);
 writeFileSync(output+'/controller-native1940.json',controllerBefore+'\n');
 writeFileSync(output+'/fixture-native1940.json',fixtureBefore+'\n');
 writeFileSync(output+'/exact-native0-1940.ndjson',actualLines.join('\n')+'\n');
 write('latest-native-packet.json',JSON.parse(actualLines.at(-1)));
 write('result.json',{status:'complete exact original-WASM geometry-baseline replay',candidateEvaluation:false,sourceCommit:'11d5786d1c5eba7f47a7cb36a4ba34a34c824c04',executionHost:'Node original immutable scalar WebAssembly module; browser renderer not involved. Exact retained browser state parity gates every row.',wasmSha256:hash(wasm),glueSha256:hash(glue),profileSha256:hash(profile),activationCheckpointBytesExactlyEqualToOriginal:true,triggerNative,native,matchedRowsIncludingNative0:rowsMatched,fullRowsComparedUsing:'Exact original browser JSON.stringify archive bytes, plus independent deep-strict raw Uint32 pose/current words and raw carry JSON strings; JSON-number signed-zero normalization is historical archive semantics' ,checkpointSchema:2,checkpointBytes:checkpointA.length,checkpointSha256:hash(checkpointA),twoPureExportsByteEqual:true,controllerRawStringBeforeAfterEqual:true,fixtureRawStringBeforeAfterEqual:true,remainingOriginalPreparationNative:4480-native,remainingOriginalPreparationS:(4480-native)/800,forecastIntegrations:0,newPhysicalInputs:[],oneCorrectedBaselineObservation:true,priorBookkeepingAttempt:{executedNativeIntegrations:60,comparisonStoppedAtNative:45,lastVerifiedNative:44,cause:'Rust JSON diagnostic−0 versus historical browser JSON.stringify0; no roll/candidate entered',rootApprovedArchiveBoundaryRestart:true}});
 process.stdout.write(JSON.stringify(JSON.parse(readFileSync(output+'/result.json')),null,2)+'\n');
} catch(error) {
 write('error.json',{native,rowsMatched,name:error.name,message:error.message,stack:error.stack});
 process.stderr.write(String(error)+'\n');process.exitCode=1;
} finally {lines.return?.();input.destroy();runtime?.free();}
