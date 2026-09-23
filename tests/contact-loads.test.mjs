import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register(); after(unregister);
const { planContactLoads, minimumSupportTorqueLimit, supportChainTorqueLimit } = await import("../src/character/contact-loads.ts");
const { BalanceController } = await import("../src/character/BalanceController.ts");
const { DynamicRecovery } = await import("../src/character/DynamicRecovery.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const zero = { x: 0, y: 0, z: 0 };
const dt = 1 / 60;

function patch(segment, x, forceN = 350) {
  return { segment, point: { x, y: 0, z: 0 },
    points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: x + sx * 0.05, y: 0, z: sz * 0.1 }))),
    forceN, normalY: 1, persistenceS: 1, loadBearing: true };
}
function plan(contacts, velocity = zero, excluded = new Set()) {
  return planContactLoads(contacts, { x: 0, y: 1, z: 0 }, velocity,
    { x: 1000, y: TOTAL_MASS_KG * 9.81, z: 0 },
    { excluded, frictionCoefficient: 0.8, maxHorizontalForceN: 200,
      maxJointTorqueNm: minimumSupportTorqueLimit(contacts) });
}

test("unloaded, lost and deliberately released contacts receive zero planned load", () => {
  const left = patch("leftFoot", -0.12), right = patch("rightFoot", 0.12);
  const unloaded = { ...left, loadBearing: false };
  assert.equal(plan([unloaded, right]).loads.some(load => load.segment === "leftFoot"), false);
  assert.equal(plan([right]).loads.length, 1);
  assert.equal(plan([left, right], zero, new Set(["rightFoot"])).loads.some(load => load.segment === "rightFoot"), false);
  assert.deepEqual(plan([]).allocatedForce, zero);
});

test("contact allocation is bounded by friction, torque and actual measured support", () => {
  for (const sign of [-1, 1]) {
    const contacts = [patch("leftFoot", -0.12 * sign), patch("rightFoot", 0.12 * sign)];
    const result = plan(contacts, { x: 0.6 * sign, y: 0, z: 0 });
    assert.ok(result.loads.every(load => load.plannedForce.y >= 0));
    assert.ok(result.allocatedForce.y <= TOTAL_MASS_KG * 9.81 * 1.35 + 1e-6);
    assert.ok(Math.hypot(result.allocatedForce.x, result.allocatedForce.z) <= 200 + 1e-6);
    assert.ok(Math.hypot(result.allocatedForce.x, result.allocatedForce.z)
      <= 0.8 * result.allocatedForce.y + 1e-6);
    assert.ok(result.loads.every(load => load.measuredForceN === 350));
  }
});

test("weak qualified support keeps restoring intent after release without granting support in flight", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    const originalUpdate = character.balance.update.bind(character.balance);
    // Isolate wrench composition from planning: the balance task asks for
    // braking after release, before any step has completed.
    character.balance.update = input => {
      const output = originalUpdate(input);
      return { ...output, diagnostics: { ...output.diagnostics,
        balanceAcceleration: { x: 0.5, y: 0, z: 0 } } };
    };
    character.lastContacts = [patch("leftFoot", -0.09, 200), patch("rightFoot", 0.09, 200)];
    for (const pose of character.poses.values()) pose.linearVelocity = { x: 0, y: -0.2, z: 0 };
    character.fixedUpdate(dt, null);
    const weight = TOTAL_MASS_KG * 9.81;
    assert.equal(character.activeGrab, null);
    assert.equal(character.stepCount, 0);
    assert.ok(character.contactLoadPlan.requestedForce.x > 0,
      "braking must survive release even without a completed step");
    assert.ok(character.contactLoadPlan.requestedForce.y > weight,
      "falling height needs a restoring normal request above bodyweight");
    assert.ok(character.contactLoadPlan.allocatedForce.y <= 1.35 * weight + 1e-6);
    character.lastContacts = [];
    character.fixedUpdate(dt, null);
    assert.deepEqual(character.contactLoadPlan.loads, []);
    assert.deepEqual(character.contactLoadPlan.allocatedForce, zero);
    assert.deepEqual(character.getSnapshot("canvas2d").diagnostics.errors, []);
  } finally { character.dispose(); }
});

test("a loaded hindfoot can shift planned pressure to its measured heel points", () => {
  const foot = {
    ...patch("leftFoot", -0.1, 500),
    point: { x: -0.1, y: 0, z: 0.035 },
    points: [-0.04, 0.11].flatMap(z => [-0.14, -0.06].map(x => ({ x, y: 0, z }))),
  };
  const result = planContactLoads([foot], { x: -0.1, y: 1, z: 0 }, zero,
    { x: 0, y: 500, z: 100 },
    { frictionCoefficient: 0.8, maxHorizontalForceN: 200, maxJointTorqueNm: 200 });
  assert.equal(result.loads.length, 1, "one measured patch remains one support-chain owner");
  assert.equal(result.loads[0].segment, "leftFoot");
  assert.equal(result.loads[0].measuredForceN, 500);
  assert.ok(result.pressurePoint.z < -0.01,
    `forward recovery should use real heel pressure, got z=${result.pressurePoint.z}`);
  assert.ok(result.pressurePoint.z >= -0.04 - 1e-9);
  assert.ok(result.loads[0].point.z >= -0.04 - 1e-9);
  assert.ok(result.loads[0].point.z <= 0.11 + 1e-9);
  assert.ok(Math.abs(result.loads[0].point.z - result.pressurePoint.z) < 1e-9);
  assert.ok(Math.abs(result.allocatedForce.y - 500) < 1e-9);
  assert.ok(Math.abs(result.allocatedForce.z - 100) < 1e-9);
});

test("prone transfer retains measured load on each eligible distal patch", () => {
  const contacts = [patch("leftForefoot", -0.3, 20), patch("rightForefoot", -0.2, 25),
    patch("leftHand", 0.3, 55)];
  const result = planContactLoads(contacts, { x: 0.3, y: 1, z: 0 }, zero,
    { x: 0, y: TOTAL_MASS_KG * 9.81, z: 0 },
    { frictionCoefficient: 0.8, maxHorizontalForceN: 200,
      maxJointTorqueNm: minimumSupportTorqueLimit(contacts), minimumMeasuredShareFraction: 0.5 });
  const measuredTotal = contacts.reduce((sum, contact) => sum + contact.forceN, 0);
  for (const contact of contacts) {
    const planned = result.loads.find(load => load.segment === contact.segment)?.plannedForce.y;
    assert.ok(planned >= 0.5 * contact.forceN / measuredTotal * result.allocatedForce.y - 1e-6,
      `${contact.segment} retains half its measured share`);
  }
  assert.ok(Math.abs(result.allocatedForce.y - TOTAL_MASS_KG * 9.81) < 1e-6);
});

test("hindfoot and forefoot patches count each actuated support joint once", () => {
  const leftFoot = patch("leftFoot", -0.12);
  const leftForefoot = patch("leftForefoot", -0.12, 120);
  const rightFoot = patch("rightFoot", 0.12);
  const rightForefoot = patch("rightForefoot", 0.12, 120);
  assert.equal(minimumSupportTorqueLimit([leftFoot, leftForefoot]),
    supportChainTorqueLimit("leftForefoot"));
  assert.equal(minimumSupportTorqueLimit([leftFoot, leftForefoot, rightFoot, rightForefoot]),
    supportChainTorqueLimit("leftForefoot") + supportChainTorqueLimit("rightForefoot"));
  assert.equal(minimumSupportTorqueLimit([{ ...leftFoot, loadBearing: false }, rightFoot]),
    supportChainTorqueLimit("rightFoot"));
});

test("a trunk contact shares weight but an unsupported trunk receives none", () => {
  const foot = patch("leftFoot", -0.1), trunk = patch("torso", 0.1, 180);
  const supported = planContactLoads([foot, trunk], { x: 0, y: 1, z: 0 }, zero,
    { x: 0, y: TOTAL_MASS_KG * 9.81, z: 0 },
    { frictionCoefficient: 0.8, maxHorizontalForceN: 200,
      maxJointTorqueNm: minimumSupportTorqueLimit([foot, trunk]) });
  assert.ok(supported.loads.find(load => load.segment === "torso")?.plannedForce.y > 0);
  const lost = plan([foot, { ...trunk, loadBearing: false }]);
  assert.equal(lost.loads.some(load => load.segment === "torso"), false);
});

test("balance reports support only from loaded solver contacts", () => {
  const poses = restPoseMap(), controller = new BalanceController(); controller.reset(poses);
  const empty = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
    activeGrab: null, contacts: [] });
  assert.deepEqual(empty.diagnostics.supportingFeet, []);
  const left = patch("leftFoot", poses.get("leftFoot").position.x);
  let loaded;
  for (let i = 0; i < 4; i++) loaded = controller.update({ dt, poses,
    rootPosition: poses.get("pelvis").position, activeGrab: null, contacts: [left] });
  assert.deepEqual(loaded.diagnostics.supportingFeet, ["leftFoot"]);
});

test("an intentional step waits for the retained sole to accept body weight", () => {
  // Use the composed standing assembly, as runtime does. The raw construction
  // map is fully extended and cannot supply a validated floor landing. Keep
  // COM over the retained sole so this fixture isolates load persistence from
  // the separate rejection of an unreachable dynamic touchdown capture.
  const rest = restPoseMap();
  const poses = composeUprightPose({ rootTranslation: { ...rest.get("pelvis").position,
    x: rest.get("leftFoot").position.x, y: 0.96 },
    reactionOffset: zero, kneeFlexion: 0.12, simulationTime: 0, activeGrab: null,
    supportFeet: { leftFoot: rest.get("leftFoot").position, rightFoot: rest.get("rightFoot").position },
    step: null }).poses;
  const controller = new BalanceController(); controller.reset(poses);
  const pelvis = poses.get("pelvis").position;
  controller.beginStep("rightFoot", poses.get("rightFoot").position, pelvis,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero);
  const weight = TOTAL_MASS_KG * 9.81;
  const contactAtFoot = (foot, forceN) => {
    const position = poses.get(foot).position, contact = patch(foot, position.x, forceN);
    return { ...contact, point: { ...contact.point, z: position.z },
      points: contact.points.map(point => ({ ...point, z: point.z + position.z })) };
  };
  const stepWith = force => controller.update({ dt, poses, rootPosition: pelvis, activeGrab: null,
    contacts: [contactAtFoot("leftFoot", force), contactAtFoot("rightFoot", weight - force)] });
  let output;
  for (let i = 0; i < 40; i++) output = stepWith(weight * 0.4);
  assert.ok(output.step.elapsed < 0, "the swing sole remains planted while retained load is insufficient");
  for (let i = 0; i < 7; i++) output = stepWith(weight * 0.6);
  assert.ok(output.step.elapsed < 0, "52% retained load alone cannot lift a still-loaded moving sole");
  for (let i = 0; i < 5; i++) output = stepWith(weight * 0.8);
  assert.ok(output.step.elapsed < 0, "a brief qualified transfer must not release the swing sole");
  output = stepWith(weight * 0.4);
  assert.ok(output.step.elapsed < 0, "losing the retained patch resets transfer readiness");
  for (let i = 0; i < 7; i++) output = stepWith(weight * 0.8);
  assert.ok(output.step.elapsed >= 0, "measured transfer releases the swing target");
  assert.deepEqual(output.diagnostics.supportingFeet, ["leftFoot"]);
});

test("a sleeping island retains measured patches and drops them as soon as the floor contact disappears", () => {
  const poses = restPoseMap(), recovery = new DynamicRecovery();
  const bodies = new Map([...poses].map(([id, pose]) => [id, {
    translation: () => pose.position, rotation: () => pose.rotation,
    linvel: () => zero, angvel: () => zero, isSleeping: () => true,
  }]));
  const frame = { translation: () => zero, rotation: () => ({ x: 0, y: 0, z: 0, w: 1 }), contactSkin: () => 0 };
  const colliders = new Map(["leftFoot", "rightFoot"].map(segment => [segment, { segment, ...frame }]));
  const floor = { isEnabled: () => true, ...frame };
  let touching = true;
  const world = { contactPair: (_floor, collider, callback) => {
    if (!touching) return;
    const x = poses.get(collider.segment).position.x;
    callback({ normal: () => ({ x: 0, y: 1, z: 0 }), numSolverContacts: () => 1,
      solverContactDist: () => 0, solverContactPoint: () => ({ x, y: 0, z: 0 }),
      numContacts: () => 1, contactImpulse: () => 0, contactDist: () => 0,
      localContactPoint1: () => ({ x, y: 0, z: 0 }), localContactPoint2: () => ({ x, y: 0, z: 0 }) }, false);
  } };
  recovery.observe(world, floor, colliders, bodies, dt);
  const contacts = recovery.diagnostics().contacts;
  assert.equal(contacts.length, 2);
  assert.ok(contacts.every(contact => contact.loadBearing && contact.sleepingEquilibrium
    && contact.measuredForceN === 0 && contact.forceN > 0));
  touching = false;
  recovery.observe(world, floor, colliders, bodies, dt);
  assert.deepEqual(recovery.diagnostics().contacts, []);
});
