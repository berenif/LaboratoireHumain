// Read-only checkpoint diagnostics. There are no calls with a positive tick count.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {initSync,WorkerRuntime} from '../rust/dist/app/lh_worker.js';
initSync({module:readFileSync('rust/dist/app/lh_worker_bg.wasm')});
const root='outputs/getup-three-poses-2026-10-16';const cases=[];
for(const [kind,native] of [['back',9600],['belly',11200]]) {
 const previous=WorkerRuntime.from_getup_checkpoint(readFileSync(`${root}/${kind}/checkpoint-${native}.bin`));
 const anchors=JSON.parse(previous.getup_report()).stable_feet;assert.ok(anchors);previous.free();
 const bytes=readFileSync(`${root}/${kind}/checkpoint-final.bin`);
 const rt=WorkerRuntime.from_getup_checkpoint(bytes);const before=rt.getup_checkpoint();
 const diagnostic=JSON.parse(rt.getup_standing_diagnostic(JSON.stringify(anchors)));
 assert.deepEqual(rt.getup_checkpoint(),before);
 const rows=readFileSync(`${root}/${kind}/trace.ndjson`,'utf8').trim().split('\n').map(JSON.parse);
 const i=rows.findIndex(r=>r.diagnostics.getup.policy.prototype_standing_passed);
 const row=rows[i-1];const n=Math.round(row.simulation_time_s*800),stable=Math.round(row.diagnostics.getup.policy.stable_standing_s*800);
 const proofNative=n-stable+800;assert.ok(Math.round(rows[i].simulation_time_s*800)-n<800);
 cases.push({kind,checkpointSha256:createHash('sha256').update(bytes).digest('hex'),
  suppliedAnchorsFromNative:native,oneSecondProofNative:proofNative,oneSecondProofPhysicalS:proofNative/800,
  oneSecondProofAfterTriggerS:(proofNative-400)/800,
  proofTimingMethod:'last pre-flag native stable counter + remaining steps to800; sample gap <800 precludes another full chain',
  diagnostic,failedPredicates:Object.entries(diagnostic.predicates).filter(([,v])=>v===false).map(([k])=>k)});rt.free();
}
const result={physicsIntegrations:0,thresholdsChanged:false,checkpointBytesUnchanged:true,cases};
writeFileSync(process.argv[2],JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(cases.map(c=>({kind:c.kind,failed:c.failedPredicates,
 motionOutsideThreshold:c.diagnostic.motion.filter(x=>!x.linearPassed||!x.angularPassed),
 head:c.diagnostic.headM,torso:c.diagnostic.torsoUp,proof:c.oneSecondProofAfterTriggerS})),null,2));
