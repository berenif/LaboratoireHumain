import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { DynamicRecovery } = await import("../src/character/DynamicRecovery.ts");
const { restPoseMap } = await import("../src/character/pose.ts");
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { recoveryMassState } = await import("../src/character/recovery-support.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { add, angularVelocity, length, sub, quatFromAxisAngle, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;

function rig(poses = restPoseMap()) {
  const impulses = new Map();
  const bodies = new Map([...poses].map(([id, p]) => [id, {
    translation: () => ({ ...p.position }), rotation: () => ({ ...p.rotation }), linvel: () => zero, angvel: () => zero,
    worldCom: () => ({ ...p.position }), mass: () => SEGMENT_BY_ID.get(id).massKg,
    isSleeping: () => false,
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
function exactSurfacePatch(poses, segment, forceN = 350) {
  const definition = SEGMENT_BY_ID.get(segment), pose = poses.get(segment);
  const surface = definition.geometry.vertices.map(vertex => worldPoint(pose.position, pose.rotation, vertex));
  const minimumY = Math.min(...surface.map(point => point.y));
  const points = surface.filter(point => point.y <= minimumY + 0.003);
  return { segment, normalY: 1, forceN, persistenceS: 1, loadBearing: true, point: points[0], points };
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
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "newly-released", "right sole alone covers projected COM");
  assert.equal(state.recovery.release(["rightFoot"], state.poses), "blocked", "already released left sole cannot authorize lifting the remaining foot");
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
  assert.equal(state.recovery.release(["rightFoot"], state.poses), "newly-released");
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
    assert.equal(state.recovery.release([segment], state.poses), "newly-released");
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
  assert.equal(state.recovery.release(["rightFoot"], state.poses), "newly-released");
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
  state.recovery.data.contacts = ["left", "right"].flatMap(side =>
    [`${side}Foot`, `${side}Forefoot`].map(id => exactSurfacePatch(state.poses, id)));
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
    state => {
      const rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.6);
      state.poses.get("rightFoot").rotation = rotation;
      state.poses.get("rightForefoot").rotation = rotation;
    },
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
  assert.equal(state.recovery.release(["leftFoot"],state.poses),"newly-released");
  const loads=new Map([["rightFoot",350]]);
  for(let i=0;i<3;i++)observeLoads(state,loads);
  assert.ok(state.recovery.releasedUnloaded.has("leftFoot"));
  const center=recoveryMassState(state.poses.values()).position;
  state.recovery.data.contacts.push(patch("leftForearm",center.x,center.z));
  state.recovery.plants.set("leftForearm",plant(state.poses.get("leftForearm")));
  assert.equal(state.recovery.release(["leftFoot","leftForearm"],state.poses),"newly-released");
  assert.ok(state.recovery.releasedUnloaded.has("leftFoot"),"an adjacent forearm release cannot erase a measured foot unload");
  loads.set("leftFoot",350);
  for(let i=0;i<3;i++)observeLoads(state,loads);
  assert.ok(!state.recovery.released.has("leftFoot"));
  assert.ok(state.recovery.plants.has("leftFoot"));
});
test("recovery exposes zero direct pelvis assistance while joint motors remain active", () => {
  const state = rig(); bothFeetCoverMass(state);
  const linearImpulses = [];
  for (const [segment, body] of state.bodies) body.applyImpulse = impulse => linearImpulses.push({ segment, impulse });
  assert.equal(typeof state.recovery.assist, "undefined", "the deleted direct-assistance entrypoint stays retired");
  state.recovery.apply(state.bodies, dt);
  const diagnostics = state.recovery.diagnostics();
  assert.deepEqual(diagnostics.assistanceForce, zero);
  assert.deepEqual(diagnostics.assistanceTorque, zero);
  assert.equal(diagnostics.assistanceForceCapN, 0);
  assert.equal(diagnostics.assistanceTorqueCapNm, 0);
  assert.equal(linearImpulses.length, 0, "recovery never applies a direct body impulse");
  assert.ok([...state.impulses.values()].some(impulse => length(impulse) > 0),
    "bounded joint torque impulses still actuate the recovery target");
});

test("settling measures passive physics without continuing protective posture motors", () => {
  const state = rig();
  state.recovery.data.phase = "settle";
  state.recovery.data.settledTimeS = 0;
  state.recovery.apply(state.bodies, dt);
  assert.equal(state.impulses.size, 0,
    "the low-motion settle gate must not be kept open by recovery posture impulses");
  assert.equal(state.recovery.diagnostics().maxMotorTorqueNm, 0);
});

test("prone arm preparation raises shoulder clearance with bounded internal spinal torque", () => {
  const fixture = RECOVERY_POSE_FIXTURES.find(item => item.id === "landed-prone-left");
  const state = rig(recoveryFixturePoses(fixture));
  state.recovery.data.phase = "roll"; state.recovery.data.route = "prone";
  state.recovery.prepareArms(state.bodies, state.poses);
  const measuredTorso = state.poses.get("torso").rotation;
  const desiredTorso = state.recovery.torsoWorldTarget(state.poses);
  assert.ok(length(angularVelocity(measuredTorso, desiredTorso, 1)) > .3,
    "the clearance stage asks the bounded lumbar/ribcage chain to extend");
  let directImpulseCount = 0;
  for (const body of state.bodies.values()) body.applyImpulse = () => { directImpulseCount += 1; };
  state.recovery.actuate(state.bodies, state.poses, true, dt);
  assert.ok(length(state.impulses.get("lumbar") ?? zero) > 0 || length(state.impulses.get("torso") ?? zero) > 0);
  const total = [...state.impulses.values()].reduce(add, zero);
  assert.ok(length(total) < 1e-9, "clearance torques remain equal and opposite across the assembly");
  assert.equal(directImpulseCount, 0, "shoulder clearance cannot synthesize a root force");
  assert.equal(state.recovery.released.size, 0, "arms remain physical supports until clearance is measured");
});

test("a moving recovery arm requires unload and intended fresh support before recapture", () => {
  const state = rig(), ids = ["leftForearm", "leftForearmTwist", "leftHand"];
  state.recovery.data.phase = "roll"; state.recovery.data.transferStage = "arm-preparation";
  state.recovery.placingProneArms = true; state.recovery.movingArms.add("left");
  state.recovery.armTimes.set("left", .4);
  state.recovery.armBraces.set("left", { position: { ...state.poses.get("leftHand").position } });
  for (const id of ids) state.recovery.released.add(id);
  for (let i = 0; i < 4; i++) observeLoads(state, new Map([["leftHand", 100]]));
  assert.ok(state.recovery.movingArms.has("left"), "the old loaded hand cannot steal the moving-arm target");
  observeLoads(state, new Map());
  assert.ok(ids.every(id => state.recovery.releasedUnloaded.has(id)), "the entire arm support set unloaded");
  for (let i = 0; i < 7; i++) observeLoads(state, new Map([["leftHand", 100]]));
  assert.ok(!state.recovery.movingArms.has("left"));
  assert.ok(ids.every(id => !state.recovery.released.has(id)));
  assert.ok(state.recovery.plants.has("leftHand"), "fresh intended contact captures the physical brace");
});

test("push-brace keeps a captured root goal and cannot kick the unreleased legs toward placements", () => {
  const a = rig(), b = rig();
  for (const state of [a, b]) {
    state.recovery.data.phase = "roll"; state.recovery.data.route = "prone";
    state.recovery.data.transferStage = "push-brace"; state.recovery.placingProneArms = false;
    state.recovery.pushOrigin = { ...state.poses.get("pelvis").position };
    state.recovery.pushRootRotation = { ...state.poses.get("pelvis").rotation };
    state.recovery.stageTime = .2; state.recovery.captureEntry(state.bodies);
  }
  a.recovery.placements.set("leftFoot", add(a.poses.get("leftFoot").position, { x: 0, y: 0, z: -.2 }));
  b.recovery.placements.set("leftFoot", add(b.poses.get("leftFoot").position, { x: 0, y: 0, z: .2 }));
  a.recovery.actuate(a.bodies, a.poses, true, dt); b.recovery.actuate(b.bodies, b.poses, true, dt);
  for (const id of ["leftThigh", "leftShin", "leftAnkle", "leftFoot", "leftForefoot"])
    assert.ok(length(sub(a.impulses.get(id) ?? zero, b.impulses.get(id) ?? zero)) < 1e-9, id);

  const state = rig();
  state.recovery.data.phase = "roll"; state.recovery.data.transferStage = "push-brace";
  state.recovery.pushOrigin = { ...state.poses.get("pelvis").position };
  state.recovery.pushRootRotation = { ...state.poses.get("pelvis").rotation };
  state.recovery.stageTime = .2;
  state.recovery.transfer(state.bodies, state.poses, dt);
  const captured = { ...state.recovery.rootGoal };
  state.poses.get("pelvis").position = add(state.poses.get("pelvis").position, { x: .04, y: .05, z: -.03 });
  state.recovery.transfer(state.bodies, state.poses, dt);
  assert.deepEqual(state.recovery.rootGoal, captured, "the height goal cannot ratchet from the body's last rise");
});

test("contact observation and loss cannot synthesize pelvis assistance", () => {
  const state = rig(); bothFeetCoverMass(state);
  let directImpulseCount = 0;
  for (const body of state.bodies.values()) body.applyImpulse = () => { directImpulseCount += 1; };
  for (const loads of [new Map([["leftFoot", 350], ["rightFoot", 350]]), new Map()]) {
    observeLoads(state, loads);
    state.recovery.apply(state.bodies, dt);
    assert.deepEqual(state.recovery.diagnostics().assistanceForce, zero);
    assert.deepEqual(state.recovery.diagnostics().assistanceTorque, zero);
  }
  assert.equal(directImpulseCount, 0, "support transitions use only the joint motor chain");
});
test("repeated release rechecks remaining support and projected COM without resetting unload", () => {
  const state = rig(); bothFeetCoverMass(state);
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "newly-released");
  state.recovery.releasedUnloaded.add("leftFoot");
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "already-released");
  assert.ok(state.recovery.releasedUnloaded.has("leftFoot"));
  for (const pose of state.poses.values()) pose.linearVelocity = { x: 2, y: 0, z: 0 };
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "blocked", "future COM left the remaining sole");
  for (const pose of state.poses.values()) pose.linearVelocity = zero;
  state.recovery.data.contacts = [];
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "blocked", "an old authorization cannot substitute for current support");
});

test("an unloaded uncaptured limb still requires remaining support before fresh release", () => {
  const state = rig();
  assert.equal(state.recovery.release(["leftHand"], state.poses), "blocked");
  bothFeetCoverMass(state);
  assert.equal(state.recovery.release(["leftHand"], state.poses), "newly-released");
  assert.equal(state.recovery.release([], state.poses), "blocked");
});

test("toe-to-flat transfer advances when the trailing shin was already safely released", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.transferStage = "shift-weight";
  state.recovery.placements.set("rightFoot", { ...state.poses.get("rightFoot").position });
  const center = recoveryMassState(state.poses.values()).position;
  state.recovery.data.contacts.push(patch("rightShin", center.x, center.z));
  state.recovery.data.contacts.find(c => c.segment === "leftFoot").forceN = 500;
  state.recovery.plants.set("rightShin", plant(state.poses.get("rightShin")));
  state.poses.get("rightFoot").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.9);
  assert.equal(state.recovery.release(["rightShin"], state.poses), "newly-released");
  state.poses.get("rightFoot").rotation = { x: 0, y: 0, z: 0, w: 1 };
  state.recovery.transfer(state.bodies, state.poses, dt);
  assert.equal(state.recovery.data.transferStage, "extend");
});

test("terminal kneeling targets reconstruct upright world posture across mirrors, headings and rotated parents", async () => {
  const { recoveryStandingPosture } = await import("../src/character/DynamicRecovery.ts");
  const { quatInverse, quatMultiply, rotate } = await import("../src/character/math.ts");
  for (const heading of [0, Math.PI / 3]) for (const side of ["left", "right"]) {
    const state = rig(); bothFeetCoverMass(state);
    const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    state.recovery.heading = heading; state.recovery.data.leadingSide = side;
    state.recovery.data.transferStage = "extend"; state.recovery.stageTime = 2;
    state.recovery.rootEntry = yaw;
    state.poses.get("pelvis").position.y = .99;
    state.poses.get("pelvis").rotation = quatMultiply(yaw, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, .12));
    for (const id of ["leftFoot", "rightFoot"]) state.poses.get(id).rotation = yaw;
    state.recovery.transfer(state.bodies, state.poses, dt);
    const world = state.recovery.torsoWorldTarget(state.poses);
    const local = quatMultiply(quatInverse(state.poses.get("pelvis").rotation), world);
    assert.ok(Math.abs(local.x) > .01, "upright world intent must compensate the measured pelvis pitch");
    const reconstructed = quatMultiply(state.poses.get("pelvis").rotation, local);
    state.poses.get("torso").rotation = reconstructed;
    state.poses.get("pelvis").rotation = state.recovery.rootRotation;
    assert.ok(rotate(reconstructed, { x: 0, y: 1, z: 0 }).y > .999999);
    assert.equal(recoveryStandingPosture(state.poses, { linear: 0, angular: 0 }), true);
  }
});

test("standing posture retains height, both sole, pelvis, torso and speed requirements", async () => {
  const { recoveryStandingPosture, RECOVERY_LIMITS } = await import("../src/character/DynamicRecovery.ts");
  const still = { linear: 0, angular: 0 };
  assert.equal(recoveryStandingPosture(restPoseMap(), still), true);
  for (const id of ["pelvis", "torso", "leftFoot", "rightFoot"]) {
    const poses = restPoseMap(); poses.get(id).rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, .3);
    assert.equal(recoveryStandingPosture(poses, still), false, id);
  }
  const low = restPoseMap(); low.get("pelvis").position.y = .93;
  assert.equal(recoveryStandingPosture(low, still), false);
  assert.equal(recoveryStandingPosture(restPoseMap(), { linear: RECOVERY_LIMITS.stableLinearMps + 1e-6, angular: 0 }), false);
  assert.equal(recoveryStandingPosture(restPoseMap(), { linear: 0, angular: RECOVERY_LIMITS.stableAngularRadps + 1e-6 }), false);
});

test("support selector admits a trailing toe only during its explicit transfer stage", async () => {
  const { selectRecoveryContacts } = await import("../src/character/recovery-support.ts");
  const state = rig(); bothFeetCoverMass(state);
  state.poses.get("rightFoot").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 1);
  const select = (stage, excluded = new Set()) => selectRecoveryContacts(state.recovery.data.contacts, state.poses, "kneel", stage, "left", excluded).map(c => c.segment);
  assert.deepEqual(select("shift-weight"), ["leftFoot", "rightFoot"]);
  assert.deepEqual(select("bring-trailing"), ["leftFoot"]);
  assert.deepEqual(select("shift-weight", new Set(["rightFoot"])), ["leftFoot"]);
});

test("prospective brace support excludes torso load and proposed releases before testing balance", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.phase = "roll";
  const center = recoveryMassState(state.poses.values()).position;
  state.recovery.data.contacts.push(patch("torso", center.x, center.z));
  const torsoLoaded = state.recovery.prospectiveSupport("brace", "tuck-knee", state.poses);
  assert.equal(torsoLoaded.balanced, true);
  assert.equal(torsoLoaded.ready, false, "balance alone cannot discard a loaded torso");
  assert.deepEqual(torsoLoaded.excludedLoaded.map(c => c.segment), ["torso"]);
  state.recovery.data.contacts = state.recovery.data.contacts.filter(c => c.segment !== "torso");
  assert.equal(state.recovery.prospectiveSupport("brace", "tuck-knee", state.poses).ready, true);
  const releasingFeet = state.recovery.prospectiveSupport("brace", "tuck-knee", state.poses, ["leftFoot", "rightFoot"]);
  assert.equal(releasingFeet.balanced, false);
  assert.equal(releasingFeet.ready, false);
});

test("a tilted foot outside toe-transfer eligibility cannot authorize another release", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.transferStage = "bring-trailing";
  state.poses.get("rightFoot").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 1);
  assert.equal(state.recovery.release(["leftFoot"], state.poses), "blocked");
  assert.ok(state.recovery.plants.has("leftFoot"));
});

test("one measured arm brace can begin a persistent push without discarding torso load", () => {
  const state = rig(); bothFeetCoverMass(state);
  state.recovery.data.phase = "roll"; state.recovery.data.transferStage = "arm-preparation";
  state.recovery.data.phaseTimeS = .3; state.recovery.placingProneArms = true;
  state.recovery.armPlacementReady = side => side === "right";
  const center = recoveryMassState(state.poses.values()).position;
  state.recovery.data.contacts.push(patch("torso", center.x, center.z));
  state.recovery.apply(state.bodies, dt);
  assert.equal(state.recovery.data.transferStage, "push-brace");
  for (let i = 0; i < 20; i++) state.recovery.apply(state.bodies, dt);
  assert.equal(state.recovery.data.phase, "roll", "a loaded torso cannot disappear from the destination support model");
  state.recovery.data.contacts = state.recovery.data.contacts.filter(c => c.segment !== "torso");
  for (let i = 0; i < 6; i++) state.recovery.apply(state.bodies, dt);
  assert.equal(state.recovery.data.phase, "roll", "prospective support must persist");
  for (let i = 0; i < 7; i++) state.recovery.apply(state.bodies, dt);
  assert.equal(state.recovery.data.phase, "brace");
});

test("a moving foot owns its target until an unloaded, intended fresh plant qualifies", () => {
  const state = rig(); bothFeetCoverMass(state);
  assert.equal(state.recovery.beginFootMovement("right", state.poses, ["rightFoot"]), true);
  const movement = state.recovery.footMovements.get("rightFoot");
  observeLoads(state, new Map([["leftFoot", 500]]));
  assert.equal(movement.unloaded, true);
  movement.time = .5;
  state.poses.get("rightFoot").position = add(movement.target.position, { x: 0, y: 0, z: .3 });
  for (let i = 0; i < 15; i++) observeLoads(state, new Map([["leftFoot", 500], ["rightFoot", 150]]));
  assert.ok(state.recovery.released.has("rightFoot"));
  assert.ok(!state.recovery.plants.has("rightFoot"));
  assert.equal(movement.paused, true, "unexpected persistent contact pauses motion without stealing its target");
  observeLoads(state, new Map([["leftFoot", 500]]));
  state.poses.get("rightFoot").position = { ...movement.target.position };
  state.poses.get("rightFoot").rotation = { ...movement.target.rotation };
  for (let i = 0; i < 11; i++) observeLoads(state, new Map([["leftFoot", 500], ["rightFoot", 150]]));
  assert.ok(state.recovery.footMovements.has("rightFoot"), "0.05-second generic contact capture is insufficient for a moving foot");
  for (let i = 0; i < 3; i++) observeLoads(state, new Map([["leftFoot", 500], ["rightFoot", 150]]));
  assert.ok(!state.recovery.footMovements.has("rightFoot"));
  assert.ok(state.recovery.plants.has("rightFoot"));
});

test("a transient command-clearance pause resumes after current geometry is feasible", () => {
  const state = rig(); bothFeetCoverMass(state);
  assert.equal(state.recovery.beginFootMovement("right", state.poses, ["rightFoot"]), true);
  const movement = state.recovery.footMovements.get("rightFoot");
  movement.unloaded = true; movement.paused = true; movement.time = .1;
  state.recovery.data.contacts = state.recovery.data.contacts.filter(c => c.segment !== "rightFoot");
  state.recovery.data.transferStage = "bring-trailing";
  state.recovery.transfer(state.bodies, state.poses, dt);
  assert.equal(movement.paused, false);
  assert.ok(movement.time > .1, "current valid evidence restarts trajectory progress");
  assert.ok(state.recovery.released.has("rightFoot"), "resuming does not qualify a replant");
});
