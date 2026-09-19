import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { composeUprightPose, poseAnchor, restPoseMap, solveTwoBone } = await import("../src/character/pose.ts");
const { SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");
const { dot, length, quatFromAxisAngle, quatInverse, rotate, sub } = await import("../src/character/math.ts");

const UP = { x: 0, y: 1, z: 0 };
const FORWARD = { x: 0, y: 0, z: 1 };
const rest = restPoseMap();
const supportFeet = {
  leftFoot: rest.get("leftFoot").position,
  rightFoot: rest.get("rightFoot").position,
};
const input = (overrides = {}) => ({
  rootTranslation: { x: 0, y: 0.99, z: 0 },
  reactionOffset: { x: 0, y: 0, z: 0 },
  simulationTime: 0,
  activeGrab: null,
  supportFeet,
  step: null,
  ...overrides,
});
const localDelta = (parent, from, to) => rotate(quatInverse(parent.rotation), sub(to, from));
const anchor = (poses, parent, child) => poseAnchor(poses.get(parent), SEGMENT_BY_ID.get(child).jointAnchorParent);

test("mirrored relaxed elbows bend forward and knees follow the pelvis at varied headings and leans", () => {
  for (const heading of [0, 0.73, Math.PI / 2, 2.7]) {
    for (const reactionOffset of [{ x: 0, y: 0, z: 0 }, { x: 0.15, y: 0, z: -0.09 }]) {
      for (const torsoTwistRadians of [-0.12, 0, 0.12]) {
      const yaw = quatFromAxisAngle(UP, heading);
      const rotatedFeet = {
        leftFoot: rotate(yaw, supportFeet.leftFoot),
        rightFoot: rotate(yaw, supportFeet.rightFoot),
      };
      const poses = composeUprightPose(input({ heading, torsoTwistRadians, reactionOffset: rotate(yaw, reactionOffset), supportFeet: rotatedFeet })).poses;
      const torso = poses.get("torso"), pelvis = poses.get("pelvis");
      for (const side of ["left", "right"]) {
        const shoulder = anchor(poses, `${side}ShoulderGirdle`, `${side}UpperArm`);
        const elbow = anchor(poses, `${side}UpperArm`, `${side}Forearm`);
        const wrist = anchor(poses, `${side}ForearmTwist`, `${side}Hand`);
        const upper = localDelta(torso, shoulder, elbow);
        const forearm = localDelta(torso, elbow, wrist);
        assert.ok(upper.z < -0.035, `${side} elbow remains behind shoulder at ${heading}`);
        assert.ok(forearm.z > 0.02, `${side} relaxed forearm bends forward at ${heading}`);
        assert.ok((side === "left" ? -1 : 1) * upper.x > 0.002, `${side} elbow stays outside torso`);

        const hip = anchor(poses, "pelvis", `${side}Thigh`);
        const knee = anchor(poses, `${side}Thigh`, `${side}Shin`);
        const ankle = anchor(poses, `${side}Shin`, `${side}Ankle`);
        const thigh = localDelta(pelvis, hip, knee);
        const shin = localDelta(pelvis, knee, ankle);
        assert.ok(thigh.z > 0.015, `${side} knee bends toward pelvis front`);
        assert.ok(shin.z < -0.015, `${side} shin folds behind knee`);
        assert.ok(dot(rotate(poses.get(`${side}Foot`).rotation, FORWARD), rotate(quatFromAxisAngle(UP, heading), FORWARD)) > 0.99,
          `${side} toes face heading`);
      }
      for (const definition of SEGMENTS) {
        if (!definition.parent) continue;
        const coordinates = jointCoordinates(poses.get(definition.parent).rotation, poses.get(definition.id).rotation, definition.jointProfile);
        assert.ok(jointLimitErrorMagnitude(coordinates, definition.jointProfile) < 1e-8, `${definition.id} remains within its joint frame`);
      }
      }
    }
  }
});

test("forward hand and foot requests use positive anatomical shoulder and hip flexion", () => {
  for (const side of ["left", "right"]) for (const kind of ["Hand", "Foot"]) {
    const region = `${side}${kind}`;
    const start = rest.get(region).position;
    const poses = composeUprightPose(input({ activeGrab: {
      region,
      startTarget: start,
      startSegmentPosition: start,
      target: { ...start, z: start.z + 0.25 },
    } })).poses;
    const proximalId = `${side}${kind === "Hand" ? "UpperArm" : "Thigh"}`;
    const definition = SEGMENT_BY_ID.get(proximalId);
    const flexion = jointCoordinates(poses.get(definition.parent).rotation, poses.get(proximalId).rotation, definition.jointProfile).x;
    assert.ok(flexion > 0.1, `${proximalId} forward flexion is positive: ${flexion}`);
  }
});

test("near-straight bend choices vary continuously and unreachable limbs stay connected", () => {
  const start = { x: 0, y: 0, z: 0 };
  const preferred = { x: 0, y: 0, z: -1 };
  const left = solveTwoBone(start, { x: -0.0001, y: 0, z: -0.57 }, 0.31, 0.27, preferred, 2.5);
  const center = solveTwoBone(start, { x: 0, y: 0, z: -0.57 }, 0.31, 0.27, preferred, 2.5);
  const right = solveTwoBone(start, { x: 0.0001, y: 0, z: -0.57 }, 0.31, 0.27, preferred, 2.5);
  assert.ok(length(sub(left.middle, center.middle)) < 0.005);
  assert.ok(length(sub(right.middle, center.middle)) < 0.005);

  for (const region of ["leftHand", "rightHand", "leftFoot", "rightFoot"]) {
    const startTarget = rest.get(region).position;
    const poses = composeUprightPose(input({ activeGrab: {
      region, startTarget, startSegmentPosition: startTarget,
      target: { x: startTarget.x + 2, y: startTarget.y + 1, z: startTarget.z + 2 },
    } })).poses;
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const parentJoint = poseAnchor(poses.get(definition.parent), definition.jointAnchorParent);
      const childJoint = poseAnchor(poses.get(definition.id), definition.jointAnchorChild);
      assert.ok(length(sub(parentJoint, childJoint)) < 1e-8, `${definition.id} stays attached`);
    }
  }
});

test("a planted foot keeps its yaw through weight transfer until a step begins", () => {
  const planted = quatFromAxisAngle(UP, 0.2);
  const supportFootRotations = { leftFoot: planted };
  const from = supportFeet.leftFoot;
  const to = { ...from, x: from.x + 0.12 };
  const before = composeUprightPose(input({
    heading: 0.5, supportFootRotations,
    step: { foot: "leftFoot", from, to, elapsed: -0.1, duration: 0.4 },
  })).poses.get("leftFoot");
  const during = composeUprightPose(input({
    heading: 0.5, supportFootRotations,
    step: { foot: "leftFoot", from, to, elapsed: 0.2, duration: 0.4 },
  })).poses.get("leftFoot");
  const plantedForward = rotate(planted, FORWARD);
  assert.ok(dot(rotate(before.rotation, FORWARD), plantedForward) > 0.999);
  assert.ok(dot(rotate(during.rotation, FORWARD), rotate(quatFromAxisAngle(UP, 0.5), FORWARD)) > 0.99);
});
