import assert from 'node:assert/strict';
import { appendFileSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';
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
const { angularVelocity } = await import('../src/character/math.ts');
for (const [key, value] of Object.entries(COORDINATED_STANDING)) assert.equal(manifest.controller[key], value, `Frozen configuration differs: ${key}`);
const handlesOf = c => [...c.ragdollBodies].map(([id,b]) => [id,b.handle]);
const c = await createEmbodiedCharacter('canvas2d', { heading: spec.heading, ...coordinatedStandingOptions('h74-v1', spec.offset) });
const initial = c.getSnapshot('canvas2d'), handles = handlesOf(c), collision = collisionContext(c);
const monitor = await makeInvariantMonitor(c, RAPIER);
const controller = c.coordinatedStanding;
const reference = Object.fromEntries(controller.reference), referenceHash = sha256(JSON.stringify(reference));
let controllerInput, controllerBefore, nativeBefore, nativeBytes, nativeAfter, commands, nativeMotorCallsPreStep,
  appliedNative = [], firstFailure = null, actualFailure = false, referenceMeasurement = null;
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
    controllerBefore, controllerInput, controllerAfter: controller.serialize(), contacts: controllerInput?.contacts,
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
    completedTicks = tick;
    const violations = monitor.check(snapshot);
    if (!nativeBytes) throw new Error('No pre-step capture: fixedUpdate did not integrate');
    costs.push(fullMs); controllerCosts.push(telemetry.solveMs);
    if (tick > 120) { observedCosts.push(fullMs); observedControllerCosts.push(telemetry.solveMs); }
    if (sha256(JSON.stringify(Object.fromEntries(controller.reference))) !== referenceHash) violations.push('reference-changed');
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
        if (spec.kind !== 'disturbance' && (linear > 0.1 || angular > 0.5)) violations.push('official-speed');
        if (spec.kind === 'disturbance') {
          const r = manifest.heldOut.recovery;
          if (linear > r.linearMps || angular > r.angularRadps || ranges.pelvis.maximumExcursionM > r.pelvisExcursionM
            || STANDING_FEET.some(id => ranges[id].maximumExcursionM > r.footExcursionM)) violations.push('disturbance-envelope');
          const lastEnd = Math.max(...spec.inputs.map(i => i.tick+i.durationTicks-1));
          if (tick >= lastEnd+r.returnDeadlineS*60 && (linear > 0.1 || angular > 0.5)) violations.push('disturbance-return');
        }
        if (!['upright','reacting'].includes(snapshot.state) || snapshot.support.planted.length !== 2 || c.stepCount !== 0) violations.push('standing-state-support-steps');
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
    recordFailure(completedTicks,['official-endpoint-drift'],final,[]);
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
    referenceMeasurement,
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
