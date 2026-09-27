import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { refineBoundedReach } = await import('../src/character/bounded-reach.ts');
const { composeUprightPose, restPoseMap, poseAnchor } = await import('../src/character/pose.ts');
const { SEGMENTS } = await import('../src/core/humanoid.ts');
const { jointCoordinates, jointLimitErrorMagnitude } = await import('../src/character/joint-coordinates.ts');
const { add, sub, length, rotate, quatFromAxisAngle, quatMultiply } = await import('../src/character/math.ts');
const zero = { x: 0, y: 0, z: 0 }, rest = restPoseMap();
const base = { rootTranslation: { x: 0, y: 0.99, z: 0 }, reactionOffset: zero, simulationTime: 0,
  activeGrab: null, supportFeet: { leftFoot: rest.get('leftFoot').position, rightFoot: rest.get('rightFoot').position }, step: null };
const idle = composeUprightPose(base).poses;
const request = (side, target) => {
  const region = `${side}Hand`, start = idle.get(region).position;
  return { ...base, collisionPlanning: false, activeGrab: { region, target, startTarget: start, startSegmentPosition: start } };
};

test('a blocked coordinate leaves remaining joints to solve the bounded least-squares problem', () => {
  const initial = Object.freeze([0, 0]), bounds = Object.freeze([Object.freeze([0, 1]), Object.freeze([-2, 2])]);
  const result = refineBoundedReach(initial, bounds, { x: -1, y: 0, z: 0 }, q => ({ x: q[0] + q[1], y: 0.1 * q[1], z: 0 }));
  assert.equal(result[0], 0);
  assert.ok(Math.abs(result[1] + 1 / 1.01) < 0.0001);
  assert.deepEqual(initial, [0, 0]);
});

test('reachable hand targets preserve millimetre accuracy on both sides', () => {
  const measured = new Map([...idle].map(([id, pose]) => [id, Object.freeze({ ...pose,
    position: Object.freeze({ ...pose.position }), rotation: Object.freeze({ ...pose.rotation }) })]));
  for (const side of ['left', 'right']) for (const measuredPoses of [undefined, measured]) {
    const target = { x: side === 'left' ? -0.3 : 0.3, y: 1.15, z: 0.3 };
    const poses = composeUprightPose({ ...request(side, target), measuredPoses }).poses;
    assert.ok(length(sub(poses.get(`${side}Hand`).position, target)) < 0.001);
  }
});

test('cross-body and unreachable hand requests preserve every joint bound and anchor', () => {
  for (const side of ['left', 'right']) for (const target of [
    { x: side === 'left' ? 0.25 : -0.25, y: 1.38, z: 0.01 }, { x: 2, y: 3, z: -2 },
  ]) {
    const poses = composeUprightPose(request(side, target)).poses;
    for (const d of SEGMENTS.filter(d => d.parent)) {
      const parent = poses.get(d.parent), child = poses.get(d.id);
      const coordinates = jointCoordinates(parent.rotation, child.rotation, d.jointProfile);
      assert.ok(jointLimitErrorMagnitude(coordinates, d.jointProfile) < 1e-8, d.id);
      assert.ok(length(sub(poseAnchor(parent, d.jointAnchorParent), poseAnchor(child, d.jointAnchorChild))) < 1e-8, d.id);
    }
  }
});

test('bounded arm reach is covariant under world yaw and horizontal translation', () => {
  const offset = { x: 0.31, y: 0, z: -0.27 };
  for (const side of ['left', 'right']) for (const heading of [Math.PI / 3, -Math.PI / 4]) {
    const input = { ...request(side, { x: side === 'left' ? 0.16 : -0.16, y: 1.35, z: 0.12 }), measuredPoses: idle };
    const original = composeUprightPose(input).poses, yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const point = p => add(offset, rotate(yaw, p));
    const transformed = composeUprightPose({ ...input, heading, rootTranslation: point(input.rootTranslation),
      supportFeet: Object.fromEntries(Object.entries(input.supportFeet).map(([id, p]) => [id, point(p)])),
      measuredPoses: new Map([...idle].map(([id, pose]) => [id, { ...pose, position: point(pose.position),
        rotation: quatMultiply(yaw, pose.rotation) }])),
      activeGrab: { ...input.activeGrab, target: point(input.activeGrab.target), startTarget: point(input.activeGrab.startTarget),
        startSegmentPosition: point(input.activeGrab.startSegmentPosition) } }).poses;
    for (const suffix of ['ShoulderGirdle', 'UpperArm', 'Forearm', 'ForearmTwist', 'Hand']) {
      const id = `${side}${suffix}`;
      assert.ok(length(sub(transformed.get(id).position, point(original.get(id).position))) < 1e-6, id);
    }
  }
});
