// Close the corrected side experiment using archived files only; zero integration.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {initSync,WorkerRuntime} from '../rust/dist/app/lh_worker.js';
const sha=x=>createHash('sha256').update(x).digest('hex');
initSync({module:readFileSync('rust/dist/app/lh_worker_bg.wasm')});
const out='outputs/getup-side-completion-2026-10-16', path=`${out}/side-supported`;
const ledger=JSON.parse(readFileSync('docs/evidence/getup-side-completion-2026-10-16.json'));
const old=readFileSync(ledger.priorLedger);assert.equal(sha(old),ledger.priorLedgerSha256);
assert.ok(ledger.cumulativeBatchPhysicalWallSeconds<=180);assert.equal(ledger.status,'closed');
const result=JSON.parse(readFileSync(`${path}/result.json`));assert.equal(result.counters.total,3200);
assert.equal(result.counters.forecast,0);assert.equal(result.triggerNative,null);
const rows=readFileSync(`${path}/trace.ndjson`,'utf8').trim().split('\n').map(JSON.parse);
for(const r of rows) {
 assert.equal(r.diagnostics.getup.policy.forecast_substeps,0);
 assert.equal(r.diagnostics.getup.policy.policy_decisions,0);
 assert.ok(r.diagnostics.getup.motor_controls.every(v=>v===0));
 assert.equal(r.diagnostics.direct_pelvis_force_n,0);assert.equal(r.diagnostics.direct_pelvis_torque_nm,0);
}
const reloads=[];
for(const label of ['initial','1600','3200','final']) {
 const bytes=readFileSync(`${path}/checkpoint-${label}.bin`),rt=WorkerRuntime.from_getup_checkpoint(bytes);
 const before=rt.getup_checkpoint(),verification=JSON.parse(rt.verify_getup_checkpoint(bytes));
 const f=JSON.parse(rt.getup_fixture_report());assert.equal(f.nativeSubsteps,verification.nativeSubsteps);
 const diagnostic=JSON.parse(rt.getup_standing_diagnostic('null'));
 assert.deepEqual(rt.getup_checkpoint(),before);reloads.push({label,sha256:sha(bytes),verification,fixture:f.state,diagnostic});rt.free();
}
assert.deepEqual(JSON.parse(readFileSync(`${path}/reference-report.json`)).status,
 result.final.snapshot.diagnostics.getup.policy);
const f=result.fixture;
const audit={physicsIntegrations:0,priorLedgerUnchanged:true,thresholdsChanged:false,forecastAndStudentDecisions:0,
 passivePreparationNative:3200,physicalSeconds:4,actualPhysicalWallSeconds:ledger.actualPhysicalWallSeconds,
 cumulativeBatchPhysicalWallSeconds:ledger.cumulativeBatchPhysicalWallSeconds,unusedPhysicalWallSeconds:ledger.unusedBatchPhysicalWallSeconds,
 cumulativeNativeSteps:24640+3200,fixtureValid:false,controllerEvaluated:false,outcome:result.outcome,
 orientationFailure:{faceUp:f.torso_face_up,absoluteFaceLimit:0.35,sideUp:f.torso_side_up,sideMinimum:0.8,
 forwardRollDegrees:Math.atan2(Math.abs(f.torso_face_up),f.torso_side_up)*180/Math.PI},
 finalFixture:f,reloads};
writeFileSync(`${out}/postrun-audit.json`,JSON.stringify(audit,null,2)+'\n');console.log(JSON.stringify({...audit,reloads:reloads.map(r=>({label:r.label,passed:r.verification.passed,native:r.verification.nativeSubsteps}))},null,2));
