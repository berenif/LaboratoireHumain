import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { DynamicRecovery } = await import("../src/character/DynamicRecovery.ts");
const { restPoseMap } = await import("../src/character/pose.ts");
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { recoveryMassState } = await import("../src/character/recovery-support.ts");
const { add, length, sub, quatFromAxisAngle } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;

function rig(poses = restPoseMap()) {
  const impulses = new Map();
  const bodies = new Map([...poses].map(([id, p]) => [id, {
    translation: () => ({ ...p.position }), rotation: () => ({ ...p.rotation }), linvel: () => zero, angvel: () => zero,
    effectiveWorldInvInertia: () => ({ m11: 1, m12: 0, m13: 0, m22: 1, m23: 0, m33: 1 }),
    applyTorqueImpulse: impulse => impulses.set(id, add(impulses.get(id) ?? zero, impulse)),
    applyImpulse: () => {},
  }]));
  const recovery = new DynamicRecovery(); recovery.reset(0, { x: 0, y: 0, z: 1 });
  recovery.data.phase = "kneel"; recovery.data.route = "half-kneel"; recovery.data.leadingSide = "left"; recovery.data.rollSide = "left";
  recovery.rootGoal = { x: 0, y: 0.70, z: 0 }; recovery.rootRotation = { x: 0, y: 0, z: 0, w: 1 };
  recovery.captureEntry(bodies); recovery.stageTime = 0.06;
  for (const id of ["leftFoot", "rightFoot", "leftHand", "rightHand"]) recovery.placements.set(id, { ...poses.get(id).position });
  return { recovery, bodies, poses, impulses };
}
function plant(pose) { return { position: { ...pose.position }, rotation: { ...pose.rotation }, absentS: 0 }; }
function patch(segment, x, z) {
  const points = [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: x + sx * 0.05, y: 0, z: z + sz * 0.135 })));
  return { segment, normalY: 1, forceN: 350, persistenceS: 1, loadBearing: true, point: { x, y: 0, z }, points };
}
function bothFeetCoverMass(state) {
  const center = recoveryMassState(state.poses.values()).position;
  // A narrow stance makes each actual 10 cm sole patch cover the projected mass.
  for (const [id, side] of [["leftFoot", -1], ["rightFoot", 1]]) state.poses.get(id).position = { x: center.x + side * 0.02, y: 0.045, z: center.z };
  state.recovery.data.contacts = [patch("leftFoot", center.x - 0.02, center.z), patch("rightFoot", center.x + 0.02, center.z)];
  for (const id of ["leftFoot", "rightFoot"]) state.recovery.plants.set(id, plant(state.poses.get(id)));
}
function actuatedDifference(a, b, ids) {
  a.recovery.actuate(a.bodies, a.poses, true, dt); b.recovery.actuate(b.bodies, b.poses, true, dt);
  return Math.max(...ids.map(id => length(sub(a.impulses.get(id), b.impulses.get(id)))));
}

test("a deliberately released foot cannot support a second release before it replants", () => {
  const state = rig(); bothFeetCoverMass(state);
  assert.equal(state.recovery.release(["leftFoot"], state.poses), true, "right sole alone covers projected COM");
  assert.equal(state.recovery.release(["rightFoot"], state.poses), false, "already released left sole cannot authorize lifting the remaining foot");
  assert.ok(state.recovery.plants.has("rightFoot"), "the remaining support target stays captured");
});

test("support geometry stops using a released support even while its last contact impulse persists", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.release(["leftFoot"], state.poses);
  assert.deepEqual(state.recovery.supporting().map(c => c.segment), ["rightFoot"]);
});

test("a planned rolling foot placement changes actual bounded joint impulses", () => {
  const a = rig(), b = rig();
  for (const state of [a, b]) {
    state.recovery.data.phase = "roll"; state.recovery.data.route = "prone"; state.recovery.data.transferStage = "roll";
    state.recovery.rootGoal = { x: 0, y: 0.45, z: 0 };
  }
  a.recovery.placements.set("leftFoot", { x: -0.09, y: 0.05, z: 0.03 });
  b.recovery.placements.set("leftFoot", { x: -0.09, y: 0.05, z: 0.20 });
  assert.ok(actuatedDifference(a, b, ["leftThigh", "leftShin", "leftFoot"]) > 1e-4,
    "changing the reachable planned landing must reach the physical actuators");
});

for (const [lower, end, joints] of [["leftForearm", "leftHand", ["leftUpperArm", "leftForearm"]], ["leftShin", "leftFoot", ["leftThigh", "leftShin"]]]) {
  test(`captured ${lower} affects joint action without endpoint support`, () => {
    const a = rig(), b = rig();
    for (const state of [a, b]) {
      state.recovery.data.phase = "brace"; state.recovery.data.route = "prone"; state.recovery.data.transferStage = "tuck-knee";
      state.recovery.plants.set(lower, plant(state.poses.get(lower)));
    }
    b.recovery.plants.get(lower).rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, lower.endsWith("Forearm") ? -0.25 : 0.25);
    assert.ok(actuatedDifference(a, b, joints) > 1e-4, "an isolated intermediate support reaches the torque actuator");
  });
  test(`captured ${lower} affects joint action while ${end} also supports`, () => {
    const a = rig(), b = rig();
    for (const state of [a, b]) {
      state.recovery.data.phase = "brace"; state.recovery.data.route = "prone"; state.recovery.data.transferStage = "tuck-knee";
      state.recovery.plants.set(lower, plant(state.poses.get(lower)));
      state.recovery.plants.set(end, plant(state.poses.get(end)));
      state.recovery.data.contacts = [lower, end].map(id => patch(id, state.poses.get(id).position.x, state.poses.get(id).position.z));
    }
    // Same measured bodies and endpoint. Only a fixed intermediate support differs by 25 mm.
    b.recovery.plants.get(lower).position = add(b.recovery.plants.get(lower).position, { x: 0, y: 0, z: 0.025 });
    assert.ok(actuatedDifference(a, b, joints) > 1e-4,
      "the fixed intermediate support must affect the torques until deliberately released");
  });
}

test("a sole near its new target is not recaptured from the old contact without a replant", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.transferStage = "bring-trailing";
  assert.equal(state.recovery.release(["rightFoot"], state.poses), true);
  state.recovery.placements.set("rightFoot", add(state.poses.get("rightFoot").position, { x: 0, y: 0, z: 0.03 }));
  state.recovery.transfer(state.bodies, state.poses, dt);
  assert.ok(state.recovery.released.has("rightFoot"), "old persistent contact is not new support establishment");
});


function observeLoads(state, loads) {
  const colliders = new Map([...state.poses.keys()].map(segment => [segment, { segment }]));
  const floor = { isEnabled: () => true };
  const world = { contactPair: (_floor, collider, callback) => {
    const force = loads.get(collider.segment) ?? 0;
    if (!force) return;
    const pose = state.poses.get(collider.segment), points = patch(collider.segment, pose.position.x, pose.position.z).points;
    callback({ normal: () => ({ x: 0, y: 1, z: 0 }), numSolverContacts: () => points.length,
      solverContactDist: () => 0, solverContactPoint: index => points[index], numContacts: () => 1, contactImpulse: () => force * dt }, false);
  } };
  state.recovery.observe(world, floor, colliders, state.bodies, dt);
}

for (const segment of ["rightFoot", "rightShin", "rightForearm"]) {
  test(segment + " keeps its captured point through sliding and reacquires after deliberate release and fresh load", () => {
    const state = rig(), loads = new Map([["leftFoot", 350], ["rightFoot", 350], [segment, 100]]);
    for (let i = 0; i < 3; i++) observeLoads(state, loads);
    const original = { ...state.recovery.plants.get(segment).position };
    state.poses.get(segment).position = add(state.poses.get(segment).position, { x: 0.025, y: 0, z: 0 });
    observeLoads(state, loads);
    assert.deepEqual(state.recovery.plants.get(segment).position, original, "ongoing load cannot drag its world target");
    bothFeetCoverMass(state);
    if (!segment.endsWith("Foot")) state.recovery.data.contacts.push(patch(segment, original.x, original.z));
    assert.equal(state.recovery.release([segment], state.poses), true);
    for (let i = 0; i < 4; i++) observeLoads(state, loads);
    assert.ok(!state.recovery.plants.has(segment), "unchanged old contact must not recreate the released target");
    loads.delete(segment); observeLoads(state, loads);
    state.poses.get(segment).position = add(state.poses.get(segment).position, { x: 0, y: 0, z: 0.03 });
    loads.set(segment, 100);
    for (let i = 0; i < 2; i++) observeLoads(state, loads);
    assert.ok(!state.recovery.plants.has(segment), "two new loaded frames are below the 0.05 s persistence requirement");
    observeLoads(state, loads);
    assert.deepEqual(state.recovery.plants.get(segment).position, state.poses.get(segment).position, "fresh persistent load captures the new physical support position");
  });
}


test("a trailing foot awaiting replant cannot authorize stand entry from its old load", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.phaseTimeS = 0.3; state.recovery.data.transferStage = "bring-trailing";
  assert.equal(state.recovery.release(["rightFoot"], state.poses), true);
  state.recovery.placements.set("rightFoot", add(state.poses.get("rightFoot").position, { x: 0, y: 0, z: 0.15 }));
  state.recovery.apply(state.bodies, dt);
  assert.ok(state.recovery.released.has("rightFoot"), "no observed unload or fresh landing occurred");
  assert.equal(state.recovery.diagnostics().phase, "kneel", "the previous foot impulse is not a freshly established trailing foot");
});


function crouchRig(fixture = RECOVERY_POSE_FIXTURES.find(f => f.pose === "crouch")) {
  const state = rig(recoveryFixturePoses(fixture));
  state.recovery.reset(fixture.heading, { x: 0, y: 0, z: 1 });
  state.recovery.data.phase = "settle";
  state.recovery.data.settledTimeS = 0.3;
  state.recovery.data.contacts = ["leftFoot", "rightFoot"].map(id => {
    const position = state.poses.get(id).position;
    // Actual rectangular sole patch rotated with the measured fixture heading.
    const contact = patch(id, 0, 0), c = Math.cos(fixture.heading), s = Math.sin(fixture.heading);
    contact.points = contact.points.map(p => ({ x: position.x + c * p.x + s * p.z, y: 0, z: position.z - s * p.x + c * p.z }));
    contact.point = { x: position.x, y: 0, z: position.z };
    return contact;
  });
  return state;
}

test("balanced planted crouches skip preparation with the same physical stand-entry conditions across mirrors and headings", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.pose === "crouch")) {
    const state = crouchRig(fixture);
    state.recovery.chooseRoute(state.bodies);
    assert.equal(state.recovery.data.route, "crouch", fixture.id);
    assert.equal(state.recovery.data.phase, "stand", fixture.id + " already has both soles and an upright torso above the entry height");
    assert.equal(state.recovery.data.transferStage, "extend");
  }
});

test("the crouch shortcut rejects weak sole orientation, insufficient entry height and balance outside the remaining soles", () => {
  const cases = [
    state => { state.poses.get("rightFoot").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.6); },
    state => { state.poses.get("torso").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.6); },
    state => { state.poses.get("pelvis").position.y = 0.54; },
    state => { state.recovery.data.contacts = state.recovery.data.contacts.slice(0, 1); },
    state => { for (const contact of state.recovery.data.contacts) { contact.point.z += 0.5; for (const p of contact.points) p.z += 0.5; } },
  ];
  for (const weaken of cases) {
    const state = crouchRig(); weaken(state); state.recovery.chooseRoute(state.bodies);
    assert.notEqual(state.recovery.data.phase, "stand", "a shortcut must not waive the physical support or pose gates");
  }
});

test("releasing an adjacent support preserves a limb's already observed unload", () => {
  const state=rig(); bothFeetCoverMass(state);
  assert.equal(state.recovery.release(["leftFoot"],state.poses),true);
  const loads=new Map([["rightFoot",350]]);
  for(let i=0;i<3;i++)observeLoads(state,loads);
  assert.ok(state.recovery.releasedUnloaded.has("leftFoot"));
  const center=recoveryMassState(state.poses.values()).position;
  state.recovery.data.contacts.push(patch("leftForearm",center.x,center.z));
  state.recovery.plants.set("leftForearm",plant(state.poses.get("leftForearm")));
  assert.equal(state.recovery.release(["leftFoot","leftForearm"],state.poses),true);
  assert.ok(state.recovery.releasedUnloaded.has("leftFoot"),"an adjacent forearm release cannot erase a measured foot unload");
  loads.set("leftFoot",350);
  for(let i=0;i<3;i++)observeLoads(state,loads);
  assert.ok(!state.recovery.released.has("leftFoot"));
  assert.ok(state.recovery.plants.has("leftFoot"));
});
test("residual upward assistance uses the actual bounded impulse and stops for weak or rolling support", () => {
  const state=rig();bothFeetCoverMass(state);
  state.recovery.rootGoal={...state.poses.get("pelvis").position,y:2};
  let actual=zero;
  state.bodies.get("pelvis").applyImpulse=impulse=>{actual=impulse;};
  state.recovery.assist(state.bodies,state.poses,dt);
  assert.ok(actual.y>0,"adequate loaded soles allow residual assistance");
  assert.ok(Math.abs(actual.y/dt-state.recovery.diagnostics().assistanceForce.y)<1e-9);
  assert.ok(actual.y/dt<=141.6564+1e-8,"the independent 72.2 kg body-weight ceiling remains 20 percent");
  state.recovery.data.phase="roll";
  state.recovery.assist(state.bodies,state.poses,dt);
  assert.deepEqual(actual,zero,"rolling has no upward or positional pelvis impulse");
  state.recovery.data.phase="kneel";
  state.recovery.data.contacts.forEach(c=>{c.forceN=10;});
  state.recovery.assist(state.bodies,state.poses,dt);
  assert.equal(actual.y,0,"weak contact load cannot authorize upward assistance");
});

test("observing contact loss does not erase the impulse that was actually integrated", () => {
  const state=rig();bothFeetCoverMass(state);
  state.recovery.rootGoal={...state.poses.get("pelvis").position,y:2};
  state.recovery.assist(state.bodies,state.poses,dt);
  const applied=state.recovery.diagnostics().assistanceForce;
  assert.ok(applied.y>0);
  observeLoads(state,new Map());
  assert.deepEqual(state.recovery.diagnostics().assistanceForce,applied,"post-step diagnostics retain the real previous-step impulse");
  state.recovery.apply(state.bodies,dt);
  assert.deepEqual(state.recovery.diagnostics().assistanceForce,zero,"the next unsupported step immediately stops assistance");
});