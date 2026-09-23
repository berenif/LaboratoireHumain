import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);
const { SEGMENT_BY_ID, HUMAN_PROPORTIONS } = await import("../src/core/humanoid.ts");
const { solveRecoveryArmTarget } = await import("../src/character/recovery-support.ts");
const { jointRotationFromCoordinates, jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");
const { add, sub, rotate, quatFromAxisAngle, quatMultiply, quatInverse } = await import("../src/character/math.ts");
const UP = { x: 0, y: 1, z: 0 };
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const agreement = (a, b) => Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);

// Independent forward construction: the elbow, wrist and hand anchors form
// actual bones, not the solver's effective forearm-and-hand approximation.
function capture(side, heading, elbowFlex, wristFlex, wristDeviation) {
  const elbowProfile = SEGMENT_BY_ID.get(`${side}Forearm`).jointProfile;
  const handDefinition = SEGMENT_BY_ID.get(`${side}Hand`);
  const yaw = quatFromAxisAngle(UP, heading);
  const shoulder = add({ x: .7, y: .8, z: -.6 }, rotate(yaw, { x: side === "left" ? -.25 : .25, y: 0, z: .12 }));
  const upper = quatMultiply(yaw, quatFromAxisAngle({ x: 0, y: 0, z: 1 }, side === "left" ? .4 : -.4));
  const localElbow = jointRotationFromCoordinates({ x: elbowFlex, y: 0, z: 0 }, elbowProfile);
  const forearm = quatMultiply(upper, localElbow);
  const localWrist = jointRotationFromCoordinates({ x: wristFlex, y: 0, z: wristDeviation }, handDefinition.jointProfile);
  const handRotation = quatMultiply(forearm, localWrist);
  const elbow = sub(shoulder, rotate(upper, { x: 0, y: HUMAN_PROPORTIONS.arm.upperLengthM, z: 0 }));
  const wrist = sub(elbow, rotate(forearm, { x: 0, y: HUMAN_PROPORTIONS.arm.forearmLengthM, z: 0 }));
  const hand = sub(wrist, rotate(handRotation, handDefinition.jointAnchorChild));
  const solved = solveRecoveryArmTarget(shoulder, hand, sub(elbow, shoulder), localWrist, heading);
  return { shoulder, hand, handRotation, elbowProfile, localWrist, solved };
}

function verifyEndpoint(sample, label) {
  const { shoulder, hand, elbowProfile, localWrist, solved } = sample;
  assert.ok(distance(solved.handPosition, hand) < 1e-8, `${label}: captured hand endpoint must be exact`);
  assert.ok(solved.reachErrorM < 1e-8, `${label}: a legal captured arm is reachable`);
  assert.ok(Math.abs(distance(shoulder, solved.elbow) - HUMAN_PROPORTIONS.arm.upperLengthM) < 1e-10);
  assert.ok(Math.abs(distance(solved.elbow, solved.wrist) - HUMAN_PROPORTIONS.arm.forearmLengthM) < 1e-10);
  const coordinates = jointCoordinates(solved.upperRotation, solved.forearmRotation, elbowProfile);
  assert.ok(jointLimitErrorMagnitude(coordinates, elbowProfile) < 1e-8, `${label}: actual elbow stays within its profile`);
  const wrist = quatMultiply(quatInverse(solved.forearmTwistRotation), solved.handRotation);
  assert.ok(agreement(wrist, localWrist) > 1 - 1e-10, `${label}: local wrist orientation is preserved`);
}

test("captured arms near maximum elbow flexion remain reachable with either wrist-flex sign", () => {
  const maximum = SEGMENT_BY_ID.get("leftForearm").jointProfile.axes.find(axis => axis.coordinate === "x").maxRadians;
  for (const side of ["left", "right"]) for (const heading of [0, .73, Math.PI, -1.7]) {
    for (const elbow of [maximum - .03, maximum]) for (const wrist of [-.45, 0, .45]) {
      const sample = capture(side, heading, elbow, wrist, 0);
      verifyEndpoint(sample, `${side}/${heading}/${elbow}/${wrist}`);
      assert.ok(agreement(sample.solved.handRotation, sample.handRotation) > 1 - 1e-8,
        "returning to a captured plant must preserve its final orientation");
    }
  }
});

test("wrist deviation does not turn an effective-bone limit into a physical elbow limit", () => {
  for (const side of ["left", "right"]) for (const heading of [0, .73, Math.PI]) {
    for (const elbow of [0, .1, .5, 1.5, 2.5]) for (const wrist of [-.45, .45]) for (const deviation of [-.2, .2]) {
      verifyEndpoint(capture(side, heading, elbow, wrist, deviation), `${side}/${heading}/${elbow}/${wrist}/${deviation}`);
    }
  }
});


test("prone fixtures stay horizontal with outward rather than trunk-crossing upper arms", async () => {
  const { recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
  for (const side of ["left", "right"]) for (const heading of [0, .73, Math.PI]) {
    const poses = recoveryFixturePoses({ id: "flat-prone-regression", pose: "prone", side, heading });
    const face = rotate(poses.get("pelvis").rotation, { x: 0, y: 0, z: 1 });
    assert.ok(Math.abs(face.y + 1) < 1e-12, "the original fully face-down root is not tilted to ease the test");
    const bodyRight = rotate(quatFromAxisAngle(UP, heading), { x: 1, y: 0, z: 0 });
    for (const arm of ["left", "right"]) {
      const upper = poses.get(`${arm}UpperArm`);
      const axis = rotate(upper.rotation, { x: 0, y: -1, z: 0 });
      const outward = (arm === "left" ? -1 : 1) * (axis.x * bodyRight.x + axis.y * bodyRight.y + axis.z * bodyRight.z);
      assert.ok(outward > .1, `${side}/${heading}/${arm}: abduction stays outward in the shared joint frame`);
    }
  }
});
