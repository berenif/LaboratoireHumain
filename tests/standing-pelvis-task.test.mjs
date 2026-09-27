import assert from 'node:assert/strict';
import test from 'node:test';
import { pelvisPoseRates, pelvisBoundedResidual } from '../scripts/standing-pelvis-task.mjs';

test('pose rates preserve tiny rotations and quaternion sign/scale invariance', () => {
  const initial = { position: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } };
  const angle = 1e-7, dt = 1 / 60;
  const current = { position: { x: 0.001, y: -0.002, z: 0.003 },
    rotation: { x: 0, y: -2 * Math.sin(angle / 2), z: 0, w: -2 * Math.cos(angle / 2) } };
  const measured = pelvisPoseRates(initial, current, dt);
  measured.pelvisLinearResidual.forEach((v, i) => assert.ok(Math.abs(v - [0.06, -0.12, 0.18][i]) < 1e-15));
  assert.ok(Math.abs(measured.pelvisAngularResidual[1] - angle / dt) < 1e-15);
  assert.ok(Math.abs(measured.pelvisAngularResidual[0]) < 1e-15);
  assert.ok(Math.abs(measured.pelvisAngularResidual[2]) < 1e-15);
});

test('bounded pelvis task penalizes only the vector excess beyond the declared norm', () => {
  const inside = pelvisBoundedResidual({ pelvisLinearResidual: [0.003, 0.004, 0], pelvisAngularResidual: [0, 0, 0.025] });
  assert.ok(inside.every(v => Math.abs(v) === 0));
  const outside = pelvisBoundedResidual({ pelvisLinearResidual: [0.012, 0.016, 0], pelvisAngularResidual: [0, 0, -0.1] });
  const expected = [0.006, 0.008, 0, 0, 0, -0.05];
  outside.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-15));
});

test('pose rotation rates use world axes for a nonidentity initial orientation', () => {
  const s = Math.SQRT1_2, angle = 0.01, dt = 0.02;
  const initial = { position: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: s, z: 0, w: s } };
  const current = { position: { ...initial.position }, rotation: {
    x: s * Math.sin(angle / 2), y: s * Math.cos(angle / 2), z: s * Math.sin(angle / 2), w: s * Math.cos(angle / 2) } };
  const measured = pelvisPoseRates(initial, current, dt);
  assert.ok(Math.abs(measured.pelvisAngularResidual[0] - angle / dt) < 1e-12);
  assert.ok(Math.abs(measured.pelvisAngularResidual[1]) < 1e-12);
  assert.ok(Math.abs(measured.pelvisAngularResidual[2]) < 1e-12);
});
