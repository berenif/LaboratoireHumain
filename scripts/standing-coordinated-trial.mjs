import assert from 'node:assert/strict';
import { appendFileSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';
import { captureControllerState } from './standing-controller-state.mjs';
import { captureBodies, collisionContext, distance, inventory, json, makeInvariantMonitor, norm, percentiles, replayFailure, verifyInventory } from './coordinated-standing-evidence.mjs';

const directory = process.argv[2], spec = JSON.parse(readFileSync(join(directory, 'scenario.json')));
const manifest = JSON.parse(readFileSync(join(dirname(directory), 'experiment.json')));
const wasmMemories = new Set(), instantiate = WebAssembly.instantiate;
WebAssembly.instantiate = async function (...args) {
  const result = await instantiate.apply(this, args), instance = result.instance ?? result;
  for (const value of Object.values(instance.exports)) if (value instanceof WebAssembly.Memory) wasmMemories.add(value);
  return result;
};
const { default: RAPIER } = await import('@dimforge/rapier3d-compat');
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { characterFrame } = await import('../src/character/character-frame.ts');
const { coordinatedStandingOptions } = await import('../src/character/standing-selection.ts');
const { COORDINATED_STANDING, STANDING_FEET } = await import('../src/character/CoordinatedStandingController.ts');
const { IMPLICIT_STANDING } = await import('../src/character/ImplicitStandingController.ts');
const { CONTACT_STANDING } = await import('../src/character/ContactStandingController.ts');
const { CONTACT_STANDING_H79 } = await import('../src/character/ContactStandingControllerH79.ts');
const { CONTACT_STANDING_H80 } = await import('../src/character/ContactStandingControllerH80.ts');
const { CONTACT_SOLVE } = await import('../src/character/coordinated-contact-solve.ts');
const { angularVelocity } = await import('../src/character/math.ts');
const controllerId=manifest.controller.id;
const expectedConfiguration=controllerId==='h80-v1'?CONTACT_STANDING_H80:controllerId==='h79-v1'?CONTACT_STANDING_H79:controllerId==='h78-v1'?CONTACT_STANDING:controllerId==='h77-v1'?IMPLICIT_STANDING:COORDINATED_STANDING;
assert.ok(['h74-v1','h77-v1','h78-v1','h79-v1','h80-v1'].includes(controllerId),'Unknown standing controller manifest');
for (const [key, value] of Object.entries(expectedConfiguration)) assert.equal(manifest.controller[key], value, `Versioned configuration differs: ${key}`);
if(['h78-v1','h79-v1','h80-v1'].includes(controllerId))assert.deepEqual(manifest.allocation.solver,CONTACT_SOLVE,'Frozen solver differs');
const handlesOf = c => [...c.ragdollBodies].map(([id,b]) => [id,b.handle]);
const c = await createEmbodiedCharacter('canvas2d', { heading: spec.heading, ...coordinatedStandingOptions(controllerId, spec.offset) });
const initial = c.getSnapshot('canvas2d'), handles = handlesOf(c), collision = collisionContext(c);
const monitor = await makeInvariantMonitor(c, RAPIER);
const controller = c.coordinatedStanding;
let reference = Object.fromEntries(controller.reference), referenceHash = sha256(JSON.stringify(reference));
let observedStanceRevision = 0;
const triggerReasons = new Set();
let controllerInput, controllerBefore, nativeBefore, nativeBytes, nativeAfter, commands, nativeMotorCallsPreStep,
  appliedNative = [], firstFailure = null, actualFailure = false, referenceMeasurement = null;
let completeControllerPreStep;
const originalControllerUpdate = controller.update.bind(controller);
controller.update = input => {
  controllerBefore = controller.serialize();
  controllerInput = { ...input, poses: [...input.poses], gravity: [...input.gravity] };
  if (spec.fault && input.tick === spec.faultTick) input = { ...input, fault: spec.fault };
  return originalControllerUpdate(input);
};
const nativeApi = c.world.impulseJoints.raw;
for (const method of ['jointConfigureMotorModel','jointSetMotorMaxForce','jointConfigureMotor']) {
  const original = nativeApi[method].bind(nativeApi);
  nativeApi[method] = (...args) => { appliedNative.push({ method, args }); return original(...args); };
}
const originalStep = c.world.step.bind(c.world);
c.world.step = (...args) => {
  nativeBytes = c.world.takeSnapshot();
  completeControllerPreStep = captureControllerState(c);
  nativeBefore = captureBodies(c.world, handles);
  commands = structuredClone(c.standingCommands);
  nativeMotorCallsPreStep = structuredClone(appliedNative);
  const result = originalStep(...args);
  nativeAfter = captureBodies(c.world, handles);
  return result;
};
const costs = [], controllerCosts = [], observedCosts = [], observedControllerCosts = [], memories = [];
const trackIds = ['pelvis', ...STANDING_FEET];
const ranges = Object.fromEntries(trackIds.map(id => [id, { maximumExcursionM: 0, minimumY: Infinity, maximumY: -Infinity, endpointM: 0 }]));
let observationStart, maximumLinear = 0, maximumAngular = 0, minHeadroom = Infinity, completedTicks = 0;
let returnTick = null, returnHoldTicks = 0;
const started = performance.now();
const sampleMemory = tick => {
  const usage = process.memoryUsage();
  const sample = { tick, ...usage, wasmLinearBytes: wasmMemories.size ? [...wasmMemories].reduce((sum,m) => sum+m.buffer.byteLength,0) : null };
  memories.push(sample); return sample;
};
const recordFailure = (tick, violations, snapshot, inputs) => {
  if (firstFailure) return;
  firstFailure = { tick, violations, nativeSnapshotSha256: sha256(nativeBytes), handles,
    capturePhase: 'After all motor configuration and external impulses; immediately before the live world.step. Snapshot carries the current native motor settings.',
    preBodies: nativeBefore, postBodies: nativeAfter, commands, nativeMotorCallsPreStep, allTickMotorCalls: appliedNative,
    controllerBefore, controllerInput, controllerAfter: controller.serialize(), completeControllerPreStep, contacts: controllerInput?.contacts,
    collision, inputs, scenario: spec, seed: spec.seed ?? 0,
    history: 'trace.ndjson reconstructs original startup, each external input and native command; no reset or state setter.',
    provenance: '../run-manifest.json', snapshot, referenceMeasurement,
    qualification: 'Native replay verifies this integrated step. Reference/local/standing acceptance is evaluated separately.' };
  writeFileSync(join(directory, 'first-failure.bin'), nativeBytes, { flag: 'wx' });
  json(join(directory, 'first-failure.json'), firstFailure);
};
try {
  json(join(directory, 'initial.json'), { initial, reference, referenceHash, controller: controller.serialize(), collision, handles });
  writeFileSync(join(directory, 'trace.ndjson'), '', { flag: 'wx' });
  sampleMemory(0);
  for (let tick = 1; tick <= spec.steps; tick++) {
    appliedNative = []; nativeBytes = null;
    const inputs = [];
    for (const event of spec.inputs ?? []) {
      if (event.impulseNs && tick === event.tick) inputs.push({ segment: event.segment, impulse: event.impulseNs });
      if (event.forceN && tick >= event.tick && tick < event.tick + event.durationTicks) inputs.push({ segment: event.segment, impulse: event.forceN.map(f => f / 60) });
    }
    for (const event of inputs) c.ragdollBodies.get(event.segment).applyImpulse({ x:event.impulse[0],y:event.impulse[1],z:event.impulse[2] }, true);
    const start = performance.now();
    const snapshot = characterFrame(c,1/60,null,'canvas2d'), fullMs = performance.now()-start;
    const telemetry = snapshot.diagnostics.coordinatedStanding;
    if (telemetry.triggerReason) triggerReasons.add(telemetry.triggerReason);
    completedTicks = tick;
    const violations = monitor.check(snapshot);
    if (!nativeBytes) throw new Error('No pre-step capture: fixedUpdate did not integrate');
    costs.push(fullMs); controllerCosts.push(telemetry.solveMs);
    if (tick > 120) { observedCosts.push(fullMs); observedControllerCosts.push(telemetry.solveMs); }
    const currentReference=Object.fromEntries(controller.reference), currentReferenceHash=sha256(JSON.stringify(currentReference));
    const rescueAllowed=controllerId==='h77-v1' && ['saved-regression','disturbance'].includes(spec.kind);
    if (currentReferenceHash !== referenceHash) {
      if(rescueAllowed && telemetry.authority==='standing' && telemetry.stanceRevision===observedStanceRevision+1) {
        reference=currentReference;referenceHash=currentReferenceHash;observedStanceRevision=telemetry.stanceRevision;
      }else violations.push('reference-changed-without-qualified-revision');
    }else if(telemetry.stanceRevision!==observedStanceRevision)violations.push('stance-revision-without-reference');
    if (telemetry.mode === 'transition' && !spec.fault) violations.push(`controller:${telemetry.reason}`);
    const state = Object.fromEntries(snapshot.segments.map(p => [p.id,p]));
    const linear = Math.max(...snapshot.segments.map(p => norm(p.linearVelocity)));
    const angular = Math.max(...snapshot.segments.map(p => norm(p.angularVelocity)));
    if (tick === 120) observationStart = state;
    if (tick >= 120 && !spec.fault) {
      const refErrors = { pelvisM: Math.hypot(...['x','y','z'].map(a => state.pelvis.position[a]-reference.pelvis.position[a])),
        feetM: Math.max(...STANDING_FEET.map(id => Math.hypot(...['x','y','z'].map(a => state[id].position[a]-reference[id].position[a])))),
        pelvisRad: norm(angularVelocity(state.pelvis.rotation,reference.pelvis.rotation,1)) };
      const referenceOK = refErrors.pelvisM <= manifest.reference.maximumPelvisErrorM
        && refErrors.feetM <= manifest.reference.maximumFootErrorM && refErrors.pelvisRad <= manifest.reference.maximumPelvisRotationErrorRad
        && linear <= manifest.reference.maximumLinearMps && angular <= manifest.reference.maximumAngularRadps
        && telemetry.requestedHeadroom >= manifest.reference.minimumRequestedMotorHeadroom;
      referenceMeasurement = { ...refErrors, linearMps: linear, angularRadps: angular, requestedHeadroom: telemetry.requestedHeadroom, referenceOK };
      if (spec.kind === 'reference' && !referenceOK) violations.push('reference-entry-or-hold');
      if (spec.kind === 'local') {
        const pulse = spec.inputs[0].tick;
        if (tick < pulse && !referenceOK) violations.push('local-precondition');
        if (tick >= pulse) {
          const region = manifest.local.neighborhood;
          if (refErrors.pelvisM > region.pelvisM || refErrors.feetM > region.feetM || refErrors.pelvisRad > region.pelvisRad
            || linear > region.linearMps || angular > region.angularRadps) violations.push('local-neighborhood');
          if (referenceOK) { returnTick ??= tick; returnHoldTicks++; } else { returnHoldTicks = 0; }
          if (tick >= pulse+manifest.local.returnDeadlineS*60 && returnTick === null) violations.push('local-return-deadline');
        }
      }
      if (tick > 120) {
        maximumLinear = Math.max(maximumLinear, linear); maximumAngular = Math.max(maximumAngular, angular);
        minHeadroom = Math.min(minHeadroom, telemetry.requestedHeadroom);
        for (const id of trackIds) {
          const r = ranges[id], p = state[id].position;
          r.endpointM = distance(p,observationStart[id].position);
          r.maximumExcursionM = Math.max(r.maximumExcursionM,r.endpointM);
          r.minimumY = Math.min(r.minimumY,p.y); r.maximumY = Math.max(r.maximumY,p.y);
        }
        if (!rescueAllowed && (linear > 0.1 || angular > 0.5)) violations.push('official-speed');
        if (spec.kind === 'disturbance') {
          const r = manifest.heldOut.recovery;
          const pelvisLimit=c.stepCount>0?r.maximumStepPelvisExcursionM:r.pelvisExcursionM;
          const footLimit=c.stepCount>0?r.maximumStepFootExcursionM:r.footExcursionM;
          if (linear > r.linearMps || angular > r.angularRadps || ranges.pelvis.maximumExcursionM > pelvisLimit
            || STANDING_FEET.some(id => ranges[id].maximumExcursionM > footLimit)) violations.push('disturbance-envelope');
          const lastEnd = Math.max(...spec.inputs.map(i => i.tick+i.durationTicks-1));
          if (tick >= lastEnd+r.returnDeadlineS*60 && (linear > 0.1 || angular > 0.5 || !referenceOK
            || snapshot.support.planted.length!==2 || (controllerId==='h77-v1' && telemetry.authority!=='standing'))) violations.push('disturbance-return');
        }
        if (!rescueAllowed && (!['upright','reacting'].includes(snapshot.state) || snapshot.support.planted.length !== 2 || c.stepCount !== 0))
          violations.push('standing-state-support-steps');
        if (rescueAllowed && c.stepCount>manifest.heldOut.recovery.steps) violations.push('rescue-step-limit');
      }
    }
    if (tick % manifest.memory.sampleEveryTicks === 0) {
      const memory = sampleMemory(tick);
      if (memory.rss > manifest.memory.maximumProcessRssMiB*2**20) violations.push('memory-ceiling');
    }
    appendFileSync(join(directory,'trace.ndjson'), JSON.stringify({ tick, inputs, commands, nativeMotorCalls:appliedNative,
      preStepSha256:sha256(nativeBytes), bodies:nativeAfter, controller:telemetry, referenceHash,
      contacts:c.lastContacts, state:snapshot.state, planted:snapshot.support.planted, stepCount:c.stepCount,
      linear, angular, fullMs, referenceMeasurement, violations })+'\n');
    if (spec.fault && tick === spec.faultTick) recordFailure(tick,[`injected:${spec.fault}`],snapshot,inputs);
    if (violations.length) { actualFailure = true; recordFailure(tick,violations,snapshot,inputs); break; }
    if ((performance.now()-started)/1000 > manifest.budget.childTimeoutS) { recordFailure(tick,['wall-budget'],snapshot,inputs); break; }
  }
  const final = c.getSnapshot('canvas2d');
  if (!firstFailure && spec.kind === 'local' && returnHoldTicks < manifest.local.holdAfterReturnS*60)
    recordFailure(completedTicks,['local-return-hold'],final,[]);
  if (!firstFailure && observationStart && spec.kind !== 'fault'
    && (ranges.pelvis.endpointM > 0.03 || STANDING_FEET.some(id => ranges[id].endpointM > 0.01)))
    if(!(controllerId==='h77-v1' && ['saved-regression','disturbance'].includes(spec.kind) && observedStanceRevision>0))
      recordFailure(completedTicks,['official-endpoint-drift'],final,[]);
  if (!firstFailure && controllerId==='h77-v1' && ['saved-regression','disturbance'].includes(spec.kind)) {
    const finalTelemetry=final.diagnostics.coordinatedStanding;
    const violations=[];
    if(!['upright','reacting'].includes(final.state) || final.support.planted.length!==2 || finalTelemetry.authority!=='standing')
      violations.push('rescue-did-not-return-to-standing');
    if(c.stepCount>0 && observedStanceRevision<1)violations.push('rescue-missing-qualified-stance-revision');
    if(spec.kind==='saved-regression' && c.stepCount<1)violations.push('saved-failure-did-not-trigger-rescue-step');
    if(spec.kind==='saved-regression' && ![...triggerReasons].some(reason=>reason==='capture'||reason==='airborne-foot'))
      violations.push('saved-failure-missing-disturbance-independent-trigger');
    if(violations.length)recordFailure(completedTicks,violations,final,[]);
  }
  if (spec.fault) {
    assert.equal(final.diagnostics.coordinatedStanding.mode,'transition');
    assert.equal(final.diagnostics.coordinatedStanding.reason,spec.fault);
    assert.equal(c.fixedSteps,spec.steps);
    assert.ok(final.segments.every(p => Object.values(p.position).every(Number.isFinite)));
  }
  sampleMemory(completedTicks);
  const replay = firstFailure ? await replayFailure(directory,RAPIER) : null;
  const report = { status:firstFailure&&(!spec.fault||actualFailure)?'fail':'pass', kind:spec.kind, scenario:spec, completedTicks,
    firstFailure:firstFailure?{tick:firstFailure.tick,violations:firstFailure.violations}:null,
    observationCompleted:completedTicks===spec.steps, state:final.state, planted:final.support.planted, steps:c.stepCount,
    referenceMeasurement, authority:final.diagnostics.coordinatedStanding.authority??null,
    stanceRevision:final.diagnostics.coordinatedStanding.stanceRevision??0, triggerReasons:[...triggerReasons],
    maximumLinearMps:maximumLinear, maximumAngularRadps:maximumAngular, minimumRequestedHeadroom:Number.isFinite(minHeadroom)?minHeadroom:null,
    excursions:Object.fromEntries(Object.entries(ranges).map(([id,r])=>[id,{...r,verticalRangeM:r.maximumY-r.minimumY}])),
    invariants:monitor.maxima, replay, returnTick, returnHoldTicks,
    runtime:{pid:process.pid,node:process.version,execArgv:process.execArgv,rapier:RAPIER.version(),wallMs:performance.now()-started,
      fullUpdateMs:percentiles(costs),controllerMs:percentiles(controllerCosts),afterSettlingUpdateMs:percentiles(observedCosts),
      afterSettlingControllerMs:percentiles(observedControllerCosts),
      updateDeadlineMisses:observedCosts.filter(v=>v>manifest.runtime.fullUpdateBudgetMs).length,
      controllerDeadlineMisses:observedControllerCosts.filter(v=>v>manifest.runtime.controllerDeadlineMs).length,
      qualification:'fixedUpdate plus application getSnapshot; native pre-step capture is instrumented. Not browser frame timing.'},
    memory:{samples:memories,maximumRss:Math.max(...memories.map(m=>m.rss)),
      growthAfterWarmup:memories.find(m=>m.tick===120)?memories.at(-1).rss-memories.find(m=>m.tick===120).rss:null,
      wasmMeasured:memories.every(m=>m.wasmLinearBytes!==null),qualification:'RSS includes process allocations; heap/external/arrayBuffers/WASM overlap and must not be summed.'},
    qualification:'Finite bounded investigation; a pass here never establishes permanent stability or validated delivered motor headroom.' };
  json(join(directory,'report.json'),report);
  const files=inventory(directory);verifyInventory(directory,files);json(join(directory,'artifacts.json'),files);
  console.log(JSON.stringify({scenario:spec.name,status:report.status,completedTicks,firstFailure:report.firstFailure,replay}));
  if(report.status!=='pass')process.exitCode=1;
} catch(error) {
  json(join(directory,'error.json'),{message:error.message,stack:error.stack,completedTicks,firstFailure:firstFailure?.tick??null});
  console.error(error);process.exitCode=2;
} finally { c.dispose(); WebAssembly.instantiate=instantiate; }
