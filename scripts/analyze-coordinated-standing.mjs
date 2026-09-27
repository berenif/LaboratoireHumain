import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { createInterface } from 'node:readline';
import { createReadStream } from 'node:fs';
import { sha256 } from './capture-physics-baseline.mjs';
import { inventory, json, norm, verifyInventory } from './coordinated-standing-evidence.mjs';
import { angularVelocity, sub } from '../src/character/math.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';

const input=resolve(process.argv[2]), output=resolve(process.argv[3]);
assert.ok(!existsSync(output),'Analysis must not overwrite prior evidence');mkdirSync(output,{recursive:true});
const report=JSON.parse(readFileSync(join(input,'report.json'))), results=[];
const executable=resolve('evidence/rapier-calibration-f32-target/release/snapshot_motors.exe');
const rawParts=handle=>{const view=new DataView(new ArrayBuffer(8));view.setFloat64(0,handle,true);return[view.getUint32(0,true),view.getUint32(4,true)];};
for(const run of report.runs.filter(r=>r.name.startsWith('reference-')&&r.firstFailure)) {
  const directory=join(input,run.name),failure=JSON.parse(readFileSync(join(directory,'first-failure.json')));
  const rows=[]; // Only the last three streamed rows are retained.
  const lines=createInterface({input:createReadStream(join(directory,'trace.ndjson')),crlfDelay:Infinity});
  for await(const line of lines){rows.push(JSON.parse(line));if(rows.length>3)rows.shift();}
  const bodyRates=Object.entries(failure.preBodies).map(([id,before])=>{
    const after=failure.postBodies[id], poseAngular=angularVelocity(before.rotation,after.rotation,1/60);
    const displacementVelocity={x:(after.position.x-before.position.x)*60,y:(after.position.y-before.position.y)*60,z:(after.position.z-before.position.z)*60};
    return{id,preAngularRadps:norm(before.angularVelocity),postAngularRadps:norm(after.angularVelocity),
      poseAngularRadps:norm(poseAngular),poseVsPostAngularDifferenceRadps:norm(sub(poseAngular,after.angularVelocity)),
      postLinearMps:norm(after.velocity),poseLinearMps:norm(displacementVelocity)};
  }).sort((a,b)=>b.postAngularRadps-a.postAngularRadps);
  let native={status:'incomplete',reason:'Native inspector unavailable'};
  if(existsSync(executable)) {
    const nativeFile=join(output,`${run.name}-native-motors.json`);
    const result=spawnSync(executable,[join(directory,'first-failure.bin'),nativeFile],{encoding:'utf8',windowsHide:true,timeout:30000});
    assert.equal(result.status,0,result.stderr);
    const inspection=JSON.parse(readFileSync(nativeFile));
    assert.ok(inspection.impulseJointPayloadExact&&inspection.otherSectionsExact&&inspection.wakeUpMembersExact&&inspection.joinMembersExact);
    const axes=[];
    for(const[id,bodyHandle]of failure.handles) {
      const definition=SEGMENT_BY_ID.get(id);if(!definition.jointProfile)continue;
      const joint=inspection.joints.find(j=>JSON.stringify(j.child)===JSON.stringify(rawParts(bodyHandle)));
      assert.ok(joint,id);
      for(const axis of definition.jointProfile.axes) {
        const index={x:3,y:4,z:5}[axis.coordinate],motor=joint.motors[index];
        assert.equal(motor.maxForce,Math.fround(axis.maxMotorTorqueNm),id);assert.equal(motor.model,'ForceBased');
        assert.equal(joint.lockedAxes&(1<<index),0,id);assert.ok(joint.motorAxes&(1<<index),id);
        axes.push({id,coordinate:axis.coordinate,capNm:motor.maxForce,stiffness:motor.stiffness,damping:motor.damping,targetVelocity:motor.targetVelocity});
      }
    }
    native={status:'pass',axes:axes.length,executableSha256:sha256(readFileSync(executable)),file:nativeFile,
      peakSegmentAxes:axes.filter(a=>a.id===bodyRates[0].id),
      qualification:'Native configuration readback only. A pre-step snapshot stores earlier solver impulses alongside newly configured commands; do not interpret it as delivered torque for the failing step.'};
  }
  results.push({name:run.name,tick:failure.tick,firstFailure:failure.violations,reference:failure.referenceMeasurement,bodyRates,native,
    recent:rows.map(row=>({tick:row.tick,linear:row.linear,angular:row.angular,contacts:row.contacts.filter(c=>c.loadBearing)
      .map(c=>({segment:c.segment,forceN:c.forceN,normalY:c.normalY})),requestedHeadroom:row.controller.requestedHeadroom}))});
}
const source=JSON.parse(readFileSync(join(input,'run-manifest.json'))).source;
const candidateSourceMatches=Object.entries(source).filter(([p])=>p.startsWith('src/')||p==='package-lock.json')
  .every(([p,hash])=>sha256(readFileSync(p))===hash);
const result={input,inputReportSha256:sha256(readFileSync(join(input,'report.json'))),candidateSourceMatches,
  analysisSourceSha256:sha256(readFileSync(new URL(import.meta.url))),results,
  interpretation:'The central first failure is measured at the forefoot with requested actuator headroom and bilateral contact. The saved output alone cannot separate controller/contact/constraint effects. H42 independently reproduces representation sensitivity in locked assemblies, not a proven cause of this articulated failure.',
  nextDiagnostic:'On the retained pre-step state, instrument native angular constraint rows and compare their coordinates/Jacobians with quaternion-derived joint coordinates; repeat capped signed single-axis perturbations under a new frozen manifest. No controller tuning, native representation change or MPC is justified by these results alone.'};
json(join(output,'report.json'),result);const files=inventory(output);verifyInventory(output,files);json(join(output,'artifacts.json'),files);
console.log(JSON.stringify({candidateSourceMatches,results:results.map(r=>({name:r.name,tick:r.tick,peak:r.bodyRates[0],native:r.native}))}));
