import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

const directory=process.argv[2],report=JSON.parse(readFileSync(join(directory,'report.json')));
const failures=[];
for(const run of report.runs.filter(r=>r.name.startsWith('reference-')&&r.firstFailure)) {
  const failurePath=join(run.directory,'first-failure.json'),tracePath=join(run.directory,'trace.ndjson');
  const failure=JSON.parse(readFileSync(failurePath)),rows=readFileSync(tracePath,'utf8').trim().split('\n').map(JSON.parse);
  const firstAllocation=rows.find(r=>r.controller?.allocation);
  const firstTransition=rows.find(r=>r.controller?.mode==='transition');
  const allocation=firstTransition?.controller?.allocation??firstAllocation?.controller?.allocation;
  const contacts=failure.snapshot?.diagnostics?.contactDiagnostics?.contacts??rows.at(-1)?.contacts??[];
  const measuredNormalN=contacts.filter(c=>/Foot|Forefoot/.test(c.segment)).reduce((sum,c)=>sum+(c.measuredForceN??c.forceN)*c.normalY,0);
  const firstNativeMismatch=rows.find(r=>r.controller?.nativeReadback?.equivalent===false);
  failures.push({name:run.name,firstFailure:run.firstFailure,artifactSha256:sha256(readFileSync(failurePath)),
    nativeSha256:failure.nativeSnapshotSha256,completeControllerState:!!failure.completeControllerPreStep,
    firstAllocation:firstAllocation?{tick:firstAllocation.tick,allocation:firstAllocation.controller.allocation}:null,
    firstTransition:firstTransition?{tick:firstTransition.tick,reason:firstTransition.controller.reason}:null,
    requestedForce:failure.controllerAfter?.telemetry?.requestedForce??null,allocation,
    native:{firstMismatchTick:firstNativeMismatch?.tick??null,readback:failure.controllerAfter?.telemetry?.nativeReadback??null},
    measuredNormalN,forecast:failure.controllerAfter?.telemetry?.forecastError??null,
    earliestDemonstratedStage:firstTransition?.controller?.reason==='timeout'?'controller-deadline'
      :allocation?.status==='infeasible'?'force-allocation-feasibility':firstNativeMismatch?'native-command-equivalence':'measured-standing-response',
    qualification:'The first observed boundary mismatch localizes the failure; force/moment residuals do not establish the complete physical cause. No future forecast has passed trajectory validation.'});
}
assert.ok(failures.length,'No complete reference failure to diagnose');
writeFileSync(join(directory,'contact-failure-analysis.json'),JSON.stringify({candidate:JSON.parse(readFileSync(join(directory,'experiment.json'))).controller.id,
  failures,automaticSuccessor:false},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify(failures.map(r=>({name:r.name,firstFailure:r.firstFailure,earliestDemonstratedStage:r.earliestDemonstratedStage}))));
