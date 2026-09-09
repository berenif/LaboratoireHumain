import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);
const { recoveryMassState, recoverySupportHull, recoverySupportMargin, supportGeometry, canReleaseSupport, selectRecoveryRoute, usableRecoveryArmSupport, reachableArmBraceTarget, solveRecoveryArmTarget } = await import("../src/character/recovery-support.ts");
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { restPoseMap } = await import("../src/character/pose.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { quatFromAxisAngle, quatFromTo, quatInverse, quatMultiply, rotate, add, sub, normalize, scale, cross, dot, clamp } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, up = { x: 0, y: 1, z: 0 };

function yawPoses(heading = 0, pitch = -Math.PI / 2) {
  const poses = restPoseMap(), yaw = quatFromAxisAngle(up, heading);
  for (const pose of poses.values()) {
    pose.position = rotate(yaw, pose.position);
    pose.rotation = quatMultiply(yaw, pose.rotation);
  }
  poses.get("torso").rotation = quatMultiply(yaw, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, pitch));
  return poses;
}

function contact(segment, poses, forceN = 100, explicitPoints) {
  const pose = poses.get(segment), shape = SEGMENT_BY_ID.get(segment).shape;
  const points = explicitPoints ?? (shape.kind === "box"
    ? [-1, 1].flatMap(x => [-1, 1].map(z => add(pose.position, rotate(pose.rotation,
      { x: x * shape.halfExtents.x, y: -shape.halfExtents.y, z: z * shape.halfExtents.z }))))
    : [{ ...pose.position, y: 0 }]);
  return { segment, normalY: 1, forceN, persistenceS: 0.2, loadBearing: true, point: points[0], points };
}

test("recovery COM weights both positions and velocities by segment mass", () => {
  const poses = yawPoses();
  const pelvis = { ...poses.get("pelvis"), position: { x: 2, y: 1, z: 0 }, linearVelocity: { x: 3, y: 0, z: 0 } };
  const hand = { ...poses.get("leftHand"), position: { x: -4, y: 0, z: 0 }, linearVelocity: { x: -3, y: 0, z: 0 } };
  const result = recoveryMassState([pelvis, hand]);
  assert.equal(result.massKg, 12.6);
  assert.ok(Math.abs(result.position.x - (24 - 2.4) / 12.6) < 1e-12);
  assert.ok(Math.abs(result.velocity.x - (36 - 1.8) / 12.6) < 1e-12);
  assert.deepEqual(recoveryMassState([]), { position: zero, velocity: zero, massKg: 0 });
});

test("support hull preserves measured contact area and rejects degenerate support", () => {
  const points = [{ x: -0.2, y: 0, z: -0.1 }, { x: 0.2, y: 0, z: -0.1 },
    { x: 0.2, y: 0, z: 0.1 }, { x: -0.2, y: 0, z: 0.1 }];
  const polygon = recoverySupportHull([...points, points[0], zero]);
  assert.equal(polygon.length, 4);
  assert.equal(recoverySupportMargin(zero, polygon), 0.1);
  assert.ok(Math.abs(recoverySupportMargin({ x: 0.3, y: 5, z: 0.2 }, polygon) + Math.SQRT2 * 0.1) < 1e-12);
  assert.ok(recoverySupportMargin(zero, recoverySupportHull([zero, { x: 1, y: 0, z: 0 }])) < 0);
  const poses = yawPoses();
  const tiny = points.map(point => ({ ...point, x: point.x * 0.02, z: point.z * 0.02 }));
  const raw = contact("leftFoot", poses, 100, tiny);
  const mass = { position: zero, velocity: zero, massKg: 10 };
  const geometry = supportGeometry([raw], poses, mass);
  assert.ok(Math.abs(geometry.marginM - 0.002) < 1e-12, "solver patch must override the larger foot collider");
  assert.deepEqual(geometry.center, zero, "weight transfer targets the patch centre, not its last corner");
  assert.equal(supportGeometry([{ ...raw, loadBearing: false }], poses, mass).polygon.length, 0);
});

test("support release uses projected COM 0.15 seconds ahead and the remaining loaded patch", () => {
  for (const heading of [0, 0.73, Math.PI / 2]) {
    const poses = yawPoses(heading), yaw = quatFromAxisAngle(up, heading);
    const foot = contact("leftFoot", poses), hand = contact("rightHand", poses);
    const position = { ...poses.get("leftFoot").position, y: 0.8 };
    const mass = { position, velocity: zero, massKg: 70 };
    assert.equal(canReleaseSupport([foot, hand], poses, mass, ["rightHand"]), true);
    const moving = { ...mass, velocity: rotate(yaw, { x: 0.4, y: 0, z: 0 }) };
    assert.equal(canReleaseSupport([foot, hand], poses, moving, ["rightHand"]), false);
    assert.equal(canReleaseSupport([foot, hand], poses, mass, ["leftFoot", "rightHand"]), false);
    const projected = supportGeometry([foot], poses, moving).projectedCenterOfMass;
    const expected = add(position, rotate(yaw, { x: 0.4 * 0.15, y: -position.y, z: 0 }));
    assert.ok(Math.hypot(projected.x - expected.x, projected.z - expected.z) < 1e-12);
  }
});

test("landed supports override identical torso orientation in mirrored and rotated selections", () => {
  for (const heading of [0, -0.9, Math.PI / 2, 2.7]) for (const side of ["left", "right"]) {
    const opposite = side === "left" ? "right" : "left";
    const poses = yawPoses(heading), foot = contact(`${side}Foot`, poses);
    const balanced = selectRecoveryRoute({ poses, contacts: [foot, contact(`${opposite}Foot`, poses)] });
    assert.equal(balanced.route, "crouch", "balanced planted feet outrank supine torso");
    const half = selectRecoveryRoute({ poses, contacts: [foot, contact(`${opposite}Shin`, poses)] });
    assert.equal(half.route, "half-kneel");
    assert.equal(half.leadingSide, side);
    assert.equal(selectRecoveryRoute({ poses, contacts: [] }).route, "roll");
    const prone = yawPoses(heading, Math.PI / 2);
    assert.equal(selectRecoveryRoute({ poses: prone, contacts: [contact(`${side}Forearm`, prone)] }).route, "prone");
    assert.equal(selectRecoveryRoute({ poses: prone, contacts: [] }).route, "roll");
  }
});

test("leading leg uses an established foot before shorter travel and deterministic side ties", () => {
  const poses = yawPoses(), targets = {
    left: add(poses.get("leftFoot").position, { x: 0.4, y: 0, z: 0 }),
    right: poses.get("rightFoot").position,
  };
  assert.equal(selectRecoveryRoute({ poses, contacts: [], footTargets: targets }).leadingSide, "right");
  assert.equal(selectRecoveryRoute({ poses, contacts: [contact("leftFoot", poses)], footTargets: targets }).leadingSide, "left");
  assert.equal(selectRecoveryRoute({ poses, contacts: [] }).leadingSide, "left");
  const airborne = { ...contact("rightFoot", poses), loadBearing: false };
  assert.equal(selectRecoveryRoute({ poses, contacts: [airborne] }).leadingSide, "left");
});

test("roll direction uses loaded arm before lower arm, then deterministic ties", () => {
  for (const side of ["left", "right"]) {
    const poses = yawPoses(), opposite = side === "left" ? "right" : "left";
    poses.get(`${opposite}Hand`).position = { x: 0, y: 0.01, z: 0 };
    assert.equal(selectRecoveryRoute({ poses, contacts: [contact(`${side}Forearm`, poses)] }).rollSide, side);
    assert.equal(selectRecoveryRoute({ poses, contacts: [] }).rollSide, opposite);
  }
  const poses = yawPoses();
  assert.equal(selectRecoveryRoute({ poses, contacts: [] }).rollSide, "left");
  const contacts = [contact("rightHand", poses, 130), contact("leftForearm", poses, 80)];
  const first = selectRecoveryRoute({ poses, contacts });
  for (let count = 0; count < 10; count++) assert.deepEqual(selectRecoveryRoute({ poses, contacts }), first);
});

test("roll proximity compares the contacting shape surface rather than arm centers", () => {
  const poses = yawPoses();
  for (const side of ["left", "right"]) poses.get(`${side}Hand`).position = { x: 0, y: 0.7, z: 0 };
  poses.get("leftForearm").position = { x: -0.2, y: 0.12, z: 0 };
  poses.get("leftForearm").rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, Math.PI / 2);
  poses.get("rightForearm").position = { x: 0.2, y: 0.15, z: 0 };
  assert.equal(selectRecoveryRoute({ poses, contacts: [] }).rollSide, "right");
});

test("sprawled prone fixture contacts require placement before becoming usable arm braces", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(item => item.pose === "prone")) {
    const poses = recoveryFixturePoses(fixture);
    for (const side of ["left", "right"]) {
      const sprawled = contact(`${side}Hand`, poses);
      assert.equal(usableRecoveryArmSupport(side, poses, [sprawled]), false, fixture.id);
      const target = reachableArmBraceTarget(side, poses, fixture.heading);
      assert.ok(target?.floorReachable, `${fixture.id} ${side} has a reachable floor brace`);
      assert.ok(target.jointLimitErrorRad < 1e-8, "prone brace fits shoulder and wrist limits");
      poses.set(`${side}Hand`, { ...poses.get(`${side}Hand`), position: target.position, rotation: target.rotation });
      const placed = contact(`${side}Hand`, poses);
      assert.equal(usableRecoveryArmSupport(side, poses, [placed]), true, fixture.id);
      assert.equal(usableRecoveryArmSupport(side, poses, [{ ...placed, loadBearing: false }]), false);
    }
  }
});

test("arm brace targets obey two-bone reach and oriented hand floor height at every heading", () => {
  for (const pose of ["prone", "supine", "side"]) for (const side of ["left", "right"]) {
    let reference;
    for (const heading of [0, 0.73, Math.PI / 2, 2.7]) {
      const poses = recoveryFixturePoses({ id: "geometry", pose, side, heading });
      const target = reachableArmBraceTarget(side, poses, heading);
      assert.ok(target?.floorReachable, `${pose} ${side} ${heading}`);
      assert.ok(Math.abs(Math.hypot(target.elbow.x - target.shoulder.x, target.elbow.y - target.shoulder.y,
        target.elbow.z - target.shoulder.z) - 0.31) < 1e-10);
      assert.ok(Math.abs(Math.hypot(target.wrist.x - target.elbow.x, target.wrist.y - target.elbow.y,
        target.wrist.z - target.elbow.z) - 0.27) < 1e-10);
      assert.ok(target.reachErrorM < 1e-10);
      const half = SEGMENT_BY_ID.get(`${side}Hand`).shape.halfExtents;
      const corners = [-1, 1].flatMap(x => [-1, 1].flatMap(y => [-1, 1].map(z =>
        add(target.position, rotate(target.rotation, { x: x * half.x, y: y * half.y, z: z * half.z })))));
      assert.ok(Math.abs(Math.min(...corners.map(point => point.y)) - 0.002) < 1e-10);
      const wrist = add(target.position, rotate(target.rotation, SEGMENT_BY_ID.get(`${side}Hand`).jointAnchorChild));
      assert.ok(Math.hypot(wrist.x - target.wrist.x, wrist.y - target.wrist.y, wrist.z - target.wrist.z) < 1e-10);
      assert.deepEqual(reachableArmBraceTarget(side, poses, heading), target, "placement is deterministic");
      if (!reference) reference = target;
      else for (const key of ["position", "bend", "wrist", "elbow"]) {
        const expected = rotate(quatFromAxisAngle(up, heading), reference[key]);
        assert.ok(Math.hypot(expected.x - target[key].x, expected.y - target[key].y, expected.z - target[key].z) < 1e-9,
          `${pose} ${key} must rotate with the heading`);
      }
    }
  }
});

test("brace support rejects distant forearms and target reports an unreachable floor honestly", () => {
  const poses = recoveryFixturePoses({ id: "forearm", pose: "prone", side: "left", heading: 0 });
  const torso = poses.get("torso"), anchor = SEGMENT_BY_ID.get("leftUpperArm").jointAnchorParent;
  const shoulder = add(torso.position, rotate(torso.rotation, anchor));
  poses.get("leftForearm").position = { x: shoulder.x, y: 0.045, z: shoulder.z - 0.12 };
  assert.equal(usableRecoveryArmSupport("left", poses, [contact("leftForearm", poses)]), true);
  poses.get("leftForearm").position.z -= 0.5;
  assert.equal(usableRecoveryArmSupport("left", poses, [contact("leftForearm", poses)]), false);
  const unreachable = reachableArmBraceTarget("left", restPoseMap(), 0);
  assert.equal(unreachable.floorReachable, false);
  assert.ok(unreachable.reachErrorM > 0.1);
  assert.ok(unreachable.position.y > 0.1, "unreachable floor must not produce an overstretched floor target");
  assert.equal(reachableArmBraceTarget("left", new Map(), 0), null);
});

test("prone brace hand orientation fits the independently reconstructed shoulder and wrist frames", () => {
  const angles = q => ({
    x: Math.asin(clamp(2 * (q.w * q.x - q.y * q.z), -1, 1)),
    y: Math.atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.x * q.x + q.y * q.y)),
    z: Math.atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)),
  });
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(item => item.pose === "prone")) for (const testedSide of ["left", "right"]) for (const heightChange of [0, -0.17]) {
    const poses = recoveryFixturePoses(fixture);
    for (const pose of poses.values()) pose.position.y += heightChange;
    const target = reachableArmBraceTarget(testedSide, poses, fixture.heading);
    const axisA = normalize(sub(target.shoulder, target.elbow)), axisB = normalize(sub(target.elbow, target.wrist));
    const hinge = normalize(scale(cross(axisA, axisB), -1));
    const base = quatFromTo(up, axisA), baseX = rotate(base, { x: 1, y: 0, z: 0 });
    const twist = Math.atan2(dot(axisA, cross(baseX, hinge)), dot(baseX, hinge));
    const upper = quatMultiply(quatFromAxisAngle(axisA, twist), base);
    const lower = quatMultiply(upper, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -Math.acos(clamp(dot(axisA, axisB), -1, 1))));
    const handId = `${testedSide}Hand`, half = SEGMENT_BY_ID.get(handId).shape.halfExtents;
    const corners = [-1, 1].flatMap(x => [-1, 1].flatMap(y => [-1, 1].map(z =>
      add(target.position, rotate(target.rotation, { x: x * half.x, y: y * half.y, z: z * half.z })))));
    const lowest = Math.min(...corners.map(point => point.y));
    const patch = corners.filter(point => point.y < lowest + 0.003);
    poses.set(handId, { ...poses.get(handId), position: target.position, rotation: target.rotation });
    assert.equal(usableRecoveryArmSupport(testedSide, poses, [contact(handId, poses, 100, patch)]), true,
      `${fixture.id} height ${heightChange}: generated target contact must be recognized as usable`);
    const wrist = angles(quatMultiply(quatInverse(lower), target.rotation));
    const shoulder = angles(quatMultiply(quatInverse(poses.get("torso").rotation), upper));
    for (const axis of ["x", "y", "z"]) {
      assert.ok(Math.abs(wrist[axis]) <= SEGMENT_BY_ID.get(`${testedSide}Hand`).jointLimitRadians[axis] + 1e-7, `wrist ${axis}`);
      assert.ok(Math.abs(shoulder[axis]) <= SEGMENT_BY_ID.get(`${testedSide}UpperArm`).jointLimitRadians[axis] + 1e-7, `shoulder ${axis}`);
    }
  }
});

test("unplanted arm clearance follows local wrist flex and returns to the captured brace orientation", () => {
  for (const side of ["left", "right"]) for (const heading of [0, 1.1]) {
    const poses = recoveryFixturePoses({ id: "clearance", pose: "prone", side, heading });
    for (const pose of poses.values()) pose.position.y -= 0.17;
    const target = reachableArmBraceTarget(side, poses, heading), from = poses.get(`${side}Hand`);
    for (let frame = 0; frame <= 40; frame++) {
      const t = frame / 40, blend = t * t * (3 - 2 * t);
      const center = add(scale(from.position, 1 - blend), scale(target.position, blend));
      center.y += Math.sin(Math.PI * t) * 0.15;
      const solved = solveRecoveryArmTarget(target.shoulder, center, target.trajectoryBend, target.wristRotation, heading);
      const wrist = quatMultiply(quatInverse(solved.forearmRotation), solved.handRotation);
      const agreement = Math.abs(wrist.x * target.wristRotation.x + wrist.y * target.wristRotation.y
        + wrist.z * target.wristRotation.z + wrist.w * target.wristRotation.w);
      assert.ok(agreement > 1 - 1e-6, `${side} ${heading} frame ${frame}: clearance must preserve local wrist flex`);
      if (frame === 40) {
        assert.ok(solved.reachErrorM < 1e-8);
        assert.ok(Math.hypot(solved.handPosition.x - target.position.x, solved.handPosition.y - target.position.y,
          solved.handPosition.z - target.position.z) < 1e-8);
        const finalAgreement = Math.abs(solved.handRotation.x * target.rotation.x + solved.handRotation.y * target.rotation.y
          + solved.handRotation.z * target.rotation.z + solved.handRotation.w * target.rotation.w);
        assert.ok(finalAgreement > 1 - 1e-6, "final orientation agrees with the captured floor brace");
      }
    }
  }
});


