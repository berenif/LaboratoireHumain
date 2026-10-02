import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { solveContactForces, CONTACT_SOLVE } = await import('../src/character/coordinated-contact-solve.ts');
const { contactJointTorque } = await import('../src/character/contact-joint-torque.ts');
const { createSupportState, updateSupportState, forecastSupport } = await import('../src/character/support-state.ts');
const { ContactStandingController } = await import('../src/character/ContactStandingController.ts');
const { ContactStandingControllerH80 } = await import('../src/character/ContactStandingControllerH80.ts');
const { BalanceController } = await import('../src/character/BalanceController.ts');
const { encodeControllerData, decodeControllerData } = await import('../scripts/standing-controller-state.mjs');
const { restPoseMap, composeUprightPose } = await import('../src/character/pose.ts');
const { SEGMENTS, HUMAN_PROPORTIONS } = await import('../src/core/humanoid.ts');
const { quatInverse, quatMultiply, rotate, quatFromAxisAngle, add, cross, sub } = await import('../src/character/math.ts');
const { coordinatedStandingOptions } = await import('../src/character/standing-selection.ts');
const zero = { x: 0, y: 0, z: 0 };
const contact = (segment, x, forceN = 300) => ({ segment, point: { x, y: 0, z: 0 },
  forceN, normalY: 1, persistenceS: 0.2, loadBearing: true });
const contacts = [contact('leftFoot', -0.1), contact('rightFoot', 0.1)];
const fixture = overrides => ({ contacts, origin: { x: 0, y: 1, z: 0 }, requestedForce: { x: 0, y: 600, z: 0 },
  requestedMoment: zero, weightN: 600, momentLengthM: 1, maxHorizontalForceN: 200, previous: [], motors: [], clock: () => 0, ...overrides });
const supportInput = overrides => ({ contacts, target: { x: -0.1, y: 0, z: 0 }, retainedSide: 'left',
  transferring: true, dt: 1 / 60, speedMps: 1.5, weightN: 600, ...overrides });

test('H78 accounts force/moment and obeys normal, friction, total-force and combined motor constraints', () => {
  const result = solveContactForces(fixture({ requestedForce: { x: 500, y: 1500, z: 0 },
    motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 9, capNm: 10, coefficients: [0, 20, 0, 0, 0, 0] }] }));
  assert.equal(result.status, 'allocated');
  const sum = result.loads.reduce((v, c) => add(v, c.plannedForce), zero);
  const moment = result.loads.reduce((v, c) => add(v, cross(sub(c.point, { x: 0, y: 1, z: 0 }), c.plannedForce)), zero);
  assert.deepEqual(sum, result.allocatedForce); assert.deepEqual(moment, result.allocatedMoment);
  assert.ok(sum.y <= 810 + 0.006); assert.ok(Math.hypot(sum.x, sum.z) <= 200.006);
  assert.ok(9 + result.loads[0].plannedForce.y / 600 * 20 <= 10.0001);
  for (const load of result.loads) {
    assert.ok(load.plannedForce.y >= -0.006);
    assert.ok(Math.hypot(load.plannedForce.x, load.plannedForce.z) <= 1.2 * load.plannedForce.y + 0.006);
  }
});

test('allocation is contact-order and heading invariant', () => {
  const input = fixture({ requestedForce: { x: 40, y: 600, z: -12 } });
  const first = solveContactForces(input), reversed = solveContactForces({ ...input, contacts: [...contacts].reverse() });
  assert.deepEqual(first, reversed);
  const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 1.1);
  const turned = solveContactForces({ ...input, contacts: contacts.map(c => ({ ...c, point: rotate(yaw, c.point) })),
    origin: rotate(yaw, input.origin), requestedForce: rotate(yaw, input.requestedForce) });
  const expected = rotate(yaw, first.allocatedForce);
  assert.ok(Math.hypot(...['x', 'y', 'z'].map(k => turned.allocatedForce[k] - expected[k])) < 0.001);
});

test('allocation failure transitions remain explicit and bounded', () => {
  assert.equal(solveContactForces(fixture({ contacts: [] })).status, 'invalid-contact');
  assert.equal(solveContactForces(fixture({ weightN: NaN })).status, 'invalid-state');
  assert.equal(solveContactForces(fixture({ previous: [{ segment: 'leftFoot', force: { x: NaN, y: 1, z: 0 } }] })).status, 'invalid-state');
  assert.equal(solveContactForces(fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 11, capNm: 10, coefficients: Array(6).fill(0) }] })).status, 'infeasible');
  assert.equal(solveContactForces(fixture({ clock: () => 2, deadlineAt: 1 })).status, 'timeout');
  assert.equal(CONTACT_SOLVE.effortRegularization, 0.02);
});

test('shared mapping uses descendant force and the same negative virtual-work moment', () => {
  assert.deepEqual(contactJointTorque('leftAnkle', { x: 0, y: 1, z: 0 }, 'leftFoot', zero, { x: 10, y: 0, z: 0 }), { x: 0, y: 0, z: -10 });
  assert.deepEqual(contactJointTorque('rightAnkle', zero, 'leftFoot', zero, { x: 10, y: 600, z: 0 }), zero);
});

test('contact loss removes allocations immediately; reappearance earns no prescribed share', () => {
  let state = updateSupportState(createSupportState(zero), supportInput());
  state.previousAllocation = contacts.map(c => ({ segment: c.segment, force: { x: 0, y: 300, z: 0 } }));
  state = updateSupportState(state, supportInput({ contacts: [contacts[0]] }));
  assert.deepEqual(state.previousAllocation.map(a => a.segment), ['leftFoot']);
  const revision = state.contactRevision;
  state = updateSupportState(state, supportInput());
  assert.equal(state.contactRevision, revision + 1);
  assert.deepEqual(state.previousAllocation.map(a => a.segment), ['leftFoot']);
});

test('weak retained support, interrupted readiness and opposite-side reversal never inherit launch permission', () => {
  let state = createSupportState(zero);
  const ready = supportInput({ contacts: [contact('leftFoot', -0.1, 400), contact('rightFoot', 0.1, 100)] });
  for (let i = 0; i < 5; i++) state = updateSupportState(state, ready);
  assert.equal(state.measuredReadiness.ready, false);
  state = updateSupportState(state, ready); assert.equal(state.measuredReadiness.ready, true);
  const reversed = updateSupportState(state, { ...ready, retainedSide: 'right' });
  assert.equal(reversed.measuredReadiness.ageS, 0); assert.equal(reversed.measuredReadiness.ready, false);
  state = updateSupportState(state, supportInput()); assert.equal(state.measuredReadiness.ageS, 0);
  state = updateSupportState(state, ready); assert.equal(state.measuredReadiness.ready, false);
  const sleeping = updateSupportState(state, { ...ready, contacts: ready.contacts.map(c => ({ ...c, measuredForceN: 0, sleepingEquilibrium: true })) });
  assert.equal(sleeping.measuredReadiness.ready, false); assert.equal(sleeping.measuredReadiness.retainedN, 0);
});

test('forecast copies the evolving state and infeasible or weak requests cannot predict readiness', () => {
  const state = createSupportState(zero), before = JSON.stringify(state);
  const input = { ...supportInput(), position: { x: 0, y: 1, z: 0 }, velocity: zero,
    externalForce: zero, massKg: 600 / 9.81, allocatedForce: { x: 0, y: 600, z: 0 }, allocationFeasible: true, horizonS: 0.5 };
  const forecast = forecastSupport(state, input);
  assert.equal(forecast.readinessTimeS, null); assert.equal(JSON.stringify(state), before);
  assert.deepEqual(forecast, forecastSupport(JSON.parse(before), input));
  assert.equal(forecastSupport(state, { ...input, allocationFeasible: false }).readinessTimeS, null);
});

test('H78 serialization/replay preserves pending authority and all injected transitions', () => {
  const poses = restPoseMap();
  const nominal = SEGMENTS.filter(s => s.jointProfile).map(s => ({ id: s.id,
    targetLocalRotation: quatMultiply(quatInverse(poses.get(s.parent).rotation), poses.get(s.id).rotation),
    stiffness: 100, damping: 10, strengthScale: 1 }));
  const input = { poses, nominal, gravity: new Map(), contacts, dt: 1 / 60, tick: 1, quiet: true,
    allocation: { ...fixture(), centerOfMass: { x: 0, y: 1, z: 0 }, supportState: createSupportState(zero) } };
  for (const fault of ['timeout', 'infeasible', 'invalid-contact', 'invalid-state', 'invalid-model', 'backup-invalid', 'backup-region-exit', 'work-limit']) {
    const controller = new ContactStandingController(poses, 0, 0, () => 0);
    assert.equal(controller.update({ ...input, fault }), null); assert.equal(controller.diagnostics().reason, fault);
    const next = new ContactStandingController(poses, 0, 0, () => 0);
    next.restore(JSON.parse(JSON.stringify(controller.serialize())));
    assert.deepEqual(next.serialize(), controller.serialize()); assert.equal(next.update(input), null);
  }
  const first = new ContactStandingController(poses, 0, 0, () => 0);
  first.update({ ...input, stepActive: true, quiet: false });
  const copy = new ContactStandingController(poses, 0, 0, () => 0); copy.restore(JSON.parse(JSON.stringify(first.serialize())));
  assert.deepEqual(copy.update(input), first.update(input));
  assert.equal(coordinatedStandingOptions('h78-v1').standingCandidate.controllerId, 'h78-v1');
  assert.throws(() => coordinatedStandingOptions('h81-v1'));
});

test('full controller data round-trips Maps and sentinel timers', () => {
  const data = { timer: Infinity, other: -Infinity, map: new Map([['sole', { previous: NaN }]]), set: new Set(['leftFoot']) };
  assert.deepEqual(decodeControllerData(JSON.parse(JSON.stringify(encodeControllerData(data)))), data);
});

test('H80 reused contact columns preserve every H78 motor coefficient exactly', () => {
  const poses = restPoseMap(), captured = [];
  const nominal = SEGMENTS.filter(s => s.jointProfile).map(s => ({ id: s.id,
    targetLocalRotation: quatMultiply(quatInverse(poses.get(s.parent).rotation), poses.get(s.id).rotation),
    stiffness: 100, damping: 10, strengthScale: 1 }));
  for(const Class of [ContactStandingController,ContactStandingControllerH80]) {
    const controller = new Class(poses,0,0,()=>0);
    controller.allocate = input => { captured.push(input.motors); return solveContactForces({...input,clock:()=>0}); };
    controller.update({poses,nominal,gravity:new Map(),contacts,dt:1/60,tick:1,quiet:true,
      allocation:{...fixture(),centerOfMass:{x:0,y:1,z:0},supportState:createSupportState(zero)}});
  }
  assert.equal(captured[0].length,46); assert.deepEqual(captured[1],captured[0]);
});

test('H78 rejects unreachable landing, preserves committed swing ownership and keeps forecast admission gated', () => {
  const rest = restPoseMap();
  const poses = composeUprightPose({ rootTranslation: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 },
    reactionOffset: zero, simulationTime: 0, activeGrab: null, step: null,
    kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
    supportFeet: { leftFoot: rest.get('leftFoot').position, rightFoot: rest.get('rightFoot').position } }).poses;
  const root = poses.get('pelvis').position;
  const balance = new BalanceController(); balance.useCoordinatedSupport = true; balance.reset(poses);
  balance.beginStep('rightFoot', poses.get('rightFoot').position, { ...root, x: root.x + 5 },
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  assert.equal(balance.step, null, 'unreachable touchdown cannot become an owned step');
  balance.beginStep('rightFoot', poses.get('rightFoot').position, root,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  assert.ok(balance.step);
  balance.step.elapsed = 0;
  const destination = { ...balance.step.to };
  balance.update({ dt: 1/60, poses, rootPosition: root, contacts, activeGrab: null, appliedGrabForce: zero });
  assert.equal(balance.step.foot, 'rightFoot');
  assert.equal(balance.step.to.x, destination.x); assert.equal(balance.step.to.z, destination.z);
  assert.equal(balance.forecastValidated, false);
  const copy = new BalanceController(); copy.restore(balance.serialize());
  assert.deepEqual(copy.serialize(), balance.serialize());
});
