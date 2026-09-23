import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { observeActiveContactManifold } = await import("../src/character/contact-observation.ts");
const { add, sub, scale, rotate, quatInverse, quatFromAxisAngle, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 };

function fixture({ points = [{ x: -0.04, y: 0, z: 0 }, { x: 0.04, y: 0, z: 0 }],
  impulses = [2, 3], active = [0, 1], flipped = false,
  surfacePose = { position: zero, rotation: identity },
  bodyPose = { position: { x: 0, y: 0.01, z: 0 }, rotation: identity },
  surfaceSkin = 0, bodySkin = 0.004 } = {}) {
  const normal = rotate(surfacePose.rotation, { x: 0, y: 1, z: 0 });
  const localPoint = (pose, point) => rotate(quatInverse(pose.rotation), sub(point, pose.position));
  const first = points.map(point => localPoint(surfacePose, point));
  const second = points.map(point => localPoint(bodyPose, add(point, scale(normal, 0.002))));
  const collider = (pose, skin) => ({ translation: () => pose.position, rotation: () => pose.rotation,
    contactSkin: () => skin });
  const surface = collider(surfacePose, surfaceSkin), body = collider(bodyPose, bodySkin);
  const midpoint = i => add(scale(add(worldPoint(surfacePose.position, surfacePose.rotation, first[i]),
    worldPoint(bodyPose.position, bodyPose.rotation, second[i])), 0.5), scale(normal, (bodySkin - surfaceSkin) * 0.5));
  const manifold = {
    normal: () => scale(normal, flipped ? -1 : 1),
    numContacts: () => points.length, numSolverContacts: () => active.length,
    localContactPoint1: i => flipped ? second[i] : first[i],
    localContactPoint2: i => flipped ? first[i] : second[i],
    contactDist: () => 0.002, contactImpulse: i => impulses[i],
    solverContactDist: () => 0.002 - bodySkin - surfaceSkin,
    solverContactPoint: i => midpoint(active[i]),
  };
  return { manifold, surface, body, flipped, midpoint, normal };
}
const observe = f => observeActiveContactManifold(f.manifold, f.surface, f.body, f.flipped);

test("inactive geometric contacts cannot donate stale impulses or pressure", () => {
  const f = fixture({ points: [{ x: -0.04, y: 0, z: 0 }, zero, { x: 0.04, y: 0, z: 0 }],
    impulses: [2, 100, 3], active: [2, 0] });
  const result = observe(f);
  assert.deepEqual(result.contacts.map(c => c.geometricIndex), [2, 0]);
  assert.equal(result.contacts.reduce((sum, c) => sum + c.impulseNs, 0), 5);
  const pressure = scale(result.contacts.reduce((sum, c) => add(sum, scale(c.point, c.impulseNs)), zero), 1 / 5);
  assert.ok(Math.abs(pressure.x - 0.008) < 1e-12);
  f.manifold.numSolverContacts = () => 0;
  assert.deepEqual(observe(f).contacts, [], "removed solver contacts cannot retain support");
});

test("coincident and near-coincident solver points retain distinct impulse owners", () => {
  for (const separation of [0, 1e-7]) {
    const f = fixture({ points: [zero, { x: separation, y: 0, z: 0 }], impulses: [2, 3], active: [1, 0] });
    const result = observe(f);
    assert.equal(new Set(result.contacts.map(c => c.geometricIndex)).size, 2);
    assert.equal(result.contacts.reduce((sum, c) => sum + c.impulseNs, 0), 5);
    if (separation) assert.deepEqual(result.contacts.map(c => c.geometricIndex), [1, 0]);
  }
});

test("flipped rotated contacts use both current collider transforms and unequal skins", () => {
  for (const flipped of [false, true]) {
    const surfacePose = { position: { x: 3, y: 0.4, z: -2 },
      rotation: quatFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.3) };
    const bodyPose = { position: { x: 3.02, y: 0.8, z: -1.8 },
      rotation: quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -0.17) };
    const points = [worldPoint(surfacePose.position, surfacePose.rotation, { x: -0.04, y: 0, z: 0 }),
      worldPoint(surfacePose.position, surfacePose.rotation, { x: 0.04, y: 0, z: 0 })];
    const f = fixture({ flipped, surfacePose, bodyPose, points, surfaceSkin: 0.007, bodySkin: 0.003 });
    // Current witnesses move with both bodies. Pre-step/surface-only matching fails.
    surfacePose.position = add(surfacePose.position, { x: 0.03, y: -0.006, z: 0.012 });
    bodyPose.position = add(bodyPose.position, { x: -0.004, y: 0.009, z: 0.003 });
    bodyPose.rotation = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -0.11);
    const result = observe(f);
    assert.deepEqual(result.normal, f.normal);
    assert.deepEqual(result.contacts.map(c => c.geometricIndex), [0, 1]);
    assert.deepEqual(result.contacts.map(c => c.point), [f.midpoint(0), f.midpoint(1)]);
  }
});

test("an unmatched solver point does not borrow an unrelated geometric impulse", () => {
  const f = fixture();
  f.manifold.solverContactPoint = () => ({ x: 1, y: 0, z: 0 });
  const result = observe(f);
  assert.ok(result.contacts.every(c => c.impulseNs === 0 && c.geometricIndex === null));
});
