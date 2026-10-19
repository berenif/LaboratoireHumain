import assert from 'node:assert/strict';
import test from 'node:test';
import { measurePhysicalCenterOfMass, rotateRawQuaternion } from '../scripts/rust-browser-center-of-mass.mjs';

const identity = { x: 0, y: 0, z: 0, w: 1 };
const body = (id, massKg, center = { x: 0, y: 0, z: 0 }) => ({ id, massKg, goldenMass: { centerOfMass: center } });
const pose = (id, x, rotation = identity) => ({ id, position: { x, y: 0, z: 0 }, rotation });

test('raw quaternion observer rotates local COM and does not use body origins as COM', () => {
  const s = Math.SQRT1_2, q = { x: 0, y: s, z: 0, w: s };
  const rotated = rotateRawQuaternion(q, { x: 1, y: 0, z: 0 });
  assert.ok(Math.abs(rotated.x) < 1e-14 && Math.abs(rotated.z + 1) < 1e-14);
  const result = measurePhysicalCenterOfMass({ segments: [body('a', 1, { x: 1, y: 0, z: 0 })] }, { segments: [pose('a', 3, q)] });
  assert.ok(Math.abs(result.position.z + 1) < 1e-6);
  assert.equal(result.position.x, 3);
});

test('non-unit quaternion preserves full product instead of unit-only shortcut or normalization', () => {
  assert.deepEqual(rotateRawQuaternion({ x: 0, y: 0.5, z: 0, w: 0.5 }, { x: 1, y: 0, z: 0 }), { x: 0, y: 0, z: -0.5 });
  assert.deepEqual(rotateRawQuaternion({ x: 0, y: 0, z: 0, w: 2 }, { x: 1, y: 2, z: 3 }), { x: 4, y: 8, z: 12 });
  const result = measurePhysicalCenterOfMass({ segments: [body('a', 1, { x: 1, y: 0, z: 0 })] }, { segments: [pose('a', 0, { x: 0, y: 0, z: 0, w: 2 })] });
  assert.equal(result.position.x, 4);
  assert.equal(result.maxQuaternionNormError, 1);
});

test('positive mass weighting excludes coordinate frames', () => {
  const result = measurePhysicalCenterOfMass({ segments: [body('a', 1), body('b', 3), { id: 'frame', massKg: 0 }] },
    { segments: [pose('a', 0), pose('b', 4), pose('frame', 999)] });
  assert.equal(result.position.x, 3);
  assert.equal(result.massKg, 4);
  assert.equal(result.contributors, 2);
});

test('unequal masses retain rotated local offsets in every centroid component', () => {
  const q = { x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2 };
  const result = measurePhysicalCenterOfMass({ segments: [
    body('a', 1, { x: 1, y: 0, z: 0 }), body('b', 3, { x: 0, y: 1, z: 0 }),
  ] }, { segments: [pose('a', 0, q), pose('b', 4)] });
  assert.equal(result.position.x, 3);
  assert.equal(result.position.y, 0.75);
  assert.ok(Math.abs(result.position.z + 0.25) < 1e-7);
});

test('model constants and exported poses are restored to f32 before independent accumulation', () => {
  const result = measurePhysicalCenterOfMass({ segments: [body('a', 0.1, { x: 0.2, y: 0, z: 0 })] }, { segments: [pose('a', 0.3)] });
  assert.equal(result.massKg, Math.fround(0.1));
  assert.equal(result.position.x, Math.fround(0.3) + Math.fround(0.2));
});

test('missing, duplicated and unknown poses cannot silently change the centroid', () => {
  const model = { segments: [body('a', 1)] };
  for (const segments of [[], [pose('a', 0), pose('a', 1)], [pose('other', 0)]]) {
    assert.throws(() => measurePhysicalCenterOfMass(model, { segments }));
  }
});

test('nonfinite data, negative or absent mass, and zero quaternions are rejected', () => {
  assert.throws(() => measurePhysicalCenterOfMass({ segments: [body('a', -1)] }, { segments: [pose('a', 0)] }));
  assert.throws(() => measurePhysicalCenterOfMass({ segments: [body('a', 0)] }, { segments: [pose('a', 0)] }));
  assert.throws(() => measurePhysicalCenterOfMass({ segments: [body('a', 1)] }, { segments: [pose('a', NaN)] }));
  assert.throws(() => measurePhysicalCenterOfMass({ segments: [body('a', 1)] }, { segments: [pose('a', 0, { x: 0, y: 0, z: 0, w: 0 })] }));
});
