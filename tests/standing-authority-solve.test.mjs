import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { solveDampedAngularStep } = await import('../scripts/standing-forefoot-authority.mjs');

test('the coupled angular step recovers a known six-axis correction', () => {
  const columns = Array.from({ length: 6 }, (_, i) => Array.from({ length: 12 }, (_, j) =>
    j === i ? 1 : j === 6 + i ? 2 : j === 6 + (i + 1) % 6 ? 0.3 : 0));
  const expected = [0.4, -0.5, 0.3, 0.2, -0.7, 0.1];
  const residual = Array.from({ length: 12 }, (_, j) => -columns.reduce((sum, col, i) => sum + col[j] * expected[i], 0));
  const frozen = JSON.stringify({ columns, residual });
  const actual = solveDampedAngularStep(columns, residual);
  assert.ok(actual.every((v, i) => Math.abs(v - expected[i]) < 1e-7));
  assert.equal(JSON.stringify({ columns, residual }), frozen);
});

test('redundant actuator columns produce a finite symmetric minimum-norm step', () => {
  const columns = [[1, 0], [1, 0], [0, 1], [0, 1], [0, 0], [0, 0]];
  const actual = solveDampedAngularStep(columns, [-2, 4]);
  assert.ok(actual.every(Number.isFinite));
  for (const [i, expected] of [1, 1, -2, -2, 0, 0].entries()) assert.ok(Math.abs(actual[i] - expected) < 1e-6);
});
