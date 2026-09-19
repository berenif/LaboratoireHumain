import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register();
after(unregister);
const { BalanceController } = await import("../src/character/BalanceController.ts");
const { composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const { worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 };

function activeStep(velocity) {
  const rest = restPoseMap();
  const root = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
  const poses = composeUprightPose({ rootTranslation: root, reactionOffset: zero,
    simulationTime: 0, activeGrab: null, step: null, kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
    supportFeet: { leftFoot: rest.get("leftFoot").position, rightFoot: rest.get("rightFoot").position },
  }).poses;
  for (const pose of poses.values()) pose.linearVelocity = { ...velocity };
  const controller = new BalanceController();
  controller.reset(poses);
  controller.beginStep("rightFoot", poses.get("rightFoot").position, root,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  const contacts = ["leftFoot", "rightFoot"].map(segment => {
    const definition = SEGMENT_BY_ID.get(segment), pose = poses.get(segment);
    const points = definition.geometry.supportPatch.map(point => worldPoint(pose.position, pose.rotation, point));
    return { segment, point: points[0], points, normalY: 1, forceN: TOTAL_MASS_KG * 9.81 / 2,
      loadBearing: true, persistenceS: 0.2 };
  });
  return controller.update({ dt: 1 / 60, poses, rootPosition: root,
    contacts, activeGrab: null, appliedGrabForce: zero });
}

test("a committed step cannot hide measured momentum outside its landing footprint", () => {
  const result = activeStep({ x: 2.2, y: 0, z: 0 });
  assert.ok(result.step, "the landing intent is still committed");
  assert.ok(result.diagnostics.supportMarginM < -0.12);
  assert.equal(result.diagnostics.recoveryCapacityM, 0);
  assert.deepEqual(result.diagnostics.externalForce, zero, "no synthetic user force triggers failure");
  assert.equal(result.shouldFall, true);
});

test("a viable, loaded unloading step is not declared a fall just because it is active", () => {
  const result = activeStep(zero);
  assert.ok(result.step);
  assert.deepEqual(result.diagnostics.measuredSupportingFeet, ["leftFoot", "rightFoot"]);
  assert.equal(result.shouldFall, false);
  assert.equal(result.stepCount, 0, "an intention is not a completed physical step");
});
