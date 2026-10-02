import assert from 'node:assert/strict';
import { appendFileSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { coordinatedStandingOptions } from '../src/character/standing-selection.ts';
import { forecastSupport } from '../src/character/support-state.ts';
import { BALANCE_LIMITS, massState } from '../src/character/BalanceController.ts';
import { SEGMENT_BY_ID, TOTAL_MASS_KG } from '../src/core/humanoid.ts';
import { captureControllerState } from './standing-controller-state.mjs';
import { json, inventory } from './coordinated-standing-evidence.mjs';
import { sha256 } from './capture-physics-baseline.mjs';

const output=process.argv[2], parent=dirname(output), manifest=JSON.parse(readFileSync(join(parent,'experiment.json')));
const scenario=JSON.parse(readFileSync(join(output,'scenario.json'))), cfg=manifest.prediction;
assert.equal(scenario.feasibility.status,'pass','Physical feasibility must precede prediction validation');
const cases=[], started=performance.now(), weightN=TOTAL_MASS_KG*9.81;
const loads=contacts=>Object.fromEntries(['left','right'].map(side=>[side,contacts.filter(c=>SEGMENT_BY_ID.get(c.segment)?.side===side
  && ['hindfoot','forefoot'].includes(SEGMENT_BY_ID.get(c.segment)?.role)).reduce((sum,c)=>sum+(c.measuredForceN??c.forceN)*c.normalY,0)]));
let firstFailure=null;
try {
  for(const factor of cfg.heldOutCommandFactors)for(const axis of ['x','z']) {
    if(performance.now()-started>manifest.budget.childTimeoutS*1000)throw new Error('Prediction child timeout');
    const character=await createEmbodiedCharacter('canvas2d',coordinatedStandingOptions(scenario.controllerId,scenario.selectedOffsetM));
    const name=`${axis}-${factor}`, trace=join(output,`${name}.ndjson`);
    writeFileSync(trace,'',{flag:'wx'});
    try {
      for(let tick=0;tick<120;tick++)character.fixedUpdate(1/60,null);
      const before=captureControllerState(character), native=character.world.takeSnapshot();
      writeFileSync(join(output,`${name}.bin`),native,{flag:'wx'});json(join(output,`${name}-controller.json`),before);
      const state=structuredClone(character.balance.supportState), mass=massState(character.poses);
      const initialLoads=loads(character.lastContacts), allocation=character.coordinatedStanding.diagnostics().allocation;
      const forceMagnitudeN=Math.min(1.5*Math.abs(factor),TOTAL_MASS_KG*BALANCE_LIMITS.maxBalanceAccelerationMps2)*Math.sign(factor);
      const externalForce={x:axis==='x'?forceMagnitudeN:0,y:0,z:axis==='z'?forceMagnitudeN:0};
      const forecastInput={contacts:structuredClone(character.lastContacts),target:state.supportTarget,
        retainedSide:state.retainedSide,transferring:state.retainedSide!==null,dt:1/60,
        speedMps:BALANCE_LIMITS.supportTransferSpeedMps,weightN,position:mass.position,velocity:mass.velocity,
        externalForce,massKg:TOTAL_MASS_KG,allocatedForce:allocation?.allocatedForce??{x:0,y:0,z:0},
        allocationFeasible:allocation?.status==='allocated',horizonS:cfg.horizonS};
      const predicted=forecastSupport(state,forecastInput);
      assert.deepEqual(state,character.balance.supportState,'Forecast mutated production state');
      assert.equal(forecastSupport(state,{...forecastInput,allocationFeasible:false}).readinessTimeS,null);
      let measuredReadyTimeS=null, physicalFailure=null;
      for(let tick=1;tick<=30;tick++) {
        character.ragdollBodies.get('torso').applyImpulse({x:externalForce.x/60,y:0,z:externalForce.z/60},true);
        character.fixedUpdate(1/60,null);
        const current=character.getSnapshot('canvas2d');
        if(character.balance.supportState.measuredReadiness.ready)measuredReadyTimeS??=tick/60;
        if(character.coordinatedStanding.diagnostics().authority==='transition'||['falling','fallen','recovering'].includes(current.state))physicalFailure??={tick,state:current.state,controller:current.diagnostics.coordinatedStanding};
        appendFileSync(trace,JSON.stringify({tick,mass:massState(character.poses),loads:loads(character.lastContacts),
          supportState:character.balance.supportState,diagnostics:current.diagnostics.coordinatedStanding})+'\n');
      }
      const actual=massState(character.poses), actualLoads=loads(character.lastContacts);
      // Held-load prediction is zero change; this is tested against actual
      // production loads, never against requested or allocated force.
      const footLoadChangeErrorWeightFraction=Math.max(...['left','right'].map(side=>Math.abs(actualLoads[side]-initialLoads[side])/weightN));
      const horizontalComErrorM=Math.hypot(actual.position.x-predicted.position.x,actual.position.z-predicted.position.z);
      const readinessTimeErrorS=measuredReadyTimeS===null&&predicted.readinessTimeS===null?0
        :measuredReadyTimeS===null||predicted.readinessTimeS===null?null:Math.abs(measuredReadyTimeS-predicted.readinessTimeS);
      const pass=predicted.valid&&!physicalFailure&&footLoadChangeErrorWeightFraction<=cfg.maximumFootLoadChangeErrorWeightFraction
        &&horizontalComErrorM<=cfg.maximumHorizontalComErrorM&&readinessTimeErrorS!==null&&readinessTimeErrorS<=cfg.maximumReadinessTimeErrorS;
      const record={name,factor,axis,forceMagnitudeN,status:pass?'pass':'fail',nativeSha256:sha256(native),forecastInput,predicted,
        actual,actualLoads,initialLoads,measuredReadyTimeS,physicalFailure,
        errors:{footLoadChangeErrorWeightFraction,horizontalComErrorM,readinessTimeErrorS}};
      cases.push(record);json(join(output,`${name}.json`),record);
      if(!pass)firstFailure??=record;
    } finally {character.dispose();}
  }
  // Quiet forced trajectories alone cannot validate timing during a transfer.
  // This guard prevents a vacuous pass when neither path ever reaches readiness.
  const exercisedReadiness=cases.some(c=>c.measuredReadyTimeS!==null);
  json(join(output,'report.json'),{status:firstFailure?'fail':exercisedReadiness?'pass':'incomplete',cases,
    firstFailure,reason:exercisedReadiness?null:'No production readiness event exercised; transfer timing is unvalidated',wallMs:performance.now()-started});
  process.exitCode=firstFailure||!exercisedReadiness?1:0;
} catch(error) {json(join(output,'report.json'),{status:'incomplete',reason:error.message,cases,firstFailure});process.exitCode=1;}
json(join(output,'artifacts.json'),inventory(output));
