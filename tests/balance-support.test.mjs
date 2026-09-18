import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { BalanceController, BALANCE_LIMITS } = await import("../src/character/BalanceController.ts");
const { composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { quatFromAxisAngle, rotate, worldPoint } = await import("../src/character/math.ts");

const dt = 1 / 60;
const zero = { x: 0, y: 0, z: 0 };
const feet = ["leftFoot", "rightFoot"];

function fixture(heading = 0) {
  const rest = restPoseMap();
  const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  const rootPosition = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
  const supportFeet = Object.fromEntries(feet.map((foot) => [foot, rotate(yaw, {
    ...rest.get(foot).position,
    y: -SEGMENT_BY_ID.get(foot).geometry.localBounds.min.y,
  })]));
  const poses = composeUprightPose({
    rootTranslation: rootPosition, reactionOffset: zero, simulationTime: 0,
    activeGrab: null, step: null, heading, supportFeet,
  }).poses;
  const controller = new BalanceController();
  controller.reset(poses, heading);
  return { controller, input: { dt, poses, rootPosition, activeGrab: null, heading } };
}

function loadedContact(input, segment = "leftFoot", loadBearing = true) {
  const pose = input.poses.get(segment);
  const geometry = SEGMENT_BY_ID.get(segment).geometry;
  const patch = geometry.supportPatch ?? geometry.vertices.filter((vertex) =>
    Math.abs(vertex.y - geometry.localBounds.min.y) < 1e-6);
  const points = patch.map((point) => worldPoint(pose.position, pose.rotation, point));
  return {
    segment, point: points[0] ?? { ...pose.position }, points,
    normalY: 1, forceN: loadBearing ? 100 : 0, persistenceS: 0.1, loadBearing,
  };
}

test("pose-only callers retain the startup support fallback", () => {
  for (const heading of [0, 1.1, -1.7]) {
    const { controller, input } = fixture(heading);
    assert.deepEqual(controller.update(input).diagnostics.supportingFeet, feet);
  }
});

test("an empty measured contact set cannot manufacture stance from near-floor poses", () => {
  for (const heading of [0, 1.1, -1.7]) {
    const { controller, input } = fixture(heading);
    for (let tick = 0; tick < 6; tick += 1) {
      const output = controller.update({ ...input, contacts: [] });
      assert.deepEqual(output.diagnostics.supportingFeet, [], `heading ${heading}, tick ${tick}`);
      assert.ok(output.diagnostics.supportMarginM < 0);
    }
  }
});

test("unloaded contacts cannot be replaced by pose-only support", () => {
  const { controller, input } = fixture();
  const contacts = feet.map((foot) => loadedContact(input, foot, false));
  assert.deepEqual(controller.update({ ...input, contacts }).diagnostics.supportingFeet, []);
});

test("only the measured loaded side contributes stance, at rotated headings too", () => {
  for (const heading of [0, 1.1, -1.7]) {
    for (const foot of feet) {
      const { controller, input } = fixture(heading);
      const output = controller.update({ ...input, contacts: [loadedContact(input, foot)] });
      assert.deepEqual(output.diagnostics.supportingFeet, [foot]);
    }
  }
});

test("non-foot contact cannot authorize either pose-derived sole", () => {
  const { controller, input } = fixture();
  const contact = { ...loadedContact(input), segment: "leftHand" };
  assert.deepEqual(controller.update({ ...input, contacts: [contact] }).diagnostics.supportingFeet, []);
});

test("lost support clears immediately and recontact must rebuild stance persistence", () => {
  const { controller, input } = fixture();
  const contacts = feet.map((foot) => loadedContact(input, foot));
  assert.deepEqual(controller.update({ ...input, contacts }).diagnostics.supportingFeet, feet);
  assert.deepEqual(controller.update({ ...input, contacts: [] }).diagnostics.supportingFeet, []);
  for (let tick = 1; tick * dt < BALANCE_LIMITS.stancePersistenceS - 1e-12; tick += 1) {
    assert.deepEqual(controller.update({ ...input, contacts }).diagnostics.supportingFeet, [], `frame ${tick}`);
  }
  assert.deepEqual(controller.update({ ...input, contacts }).diagnostics.supportingFeet, feet);
});

test("a manipulated near-floor foot remains excluded even with measured load", () => {
  const { controller, input } = fixture();
  const start = input.poses.get("rightFoot").position;
  const contacts = feet.map((foot) => loadedContact(input, foot));
  const activeGrab = {
    region: "rightFoot", target: { ...start, x: start.x + 0.1 },
    startTarget: start, startSegmentPosition: start,
  };
  assert.deepEqual(controller.update({ ...input, contacts, activeGrab }).diagnostics.supportingFeet, ["leftFoot"]);
});
