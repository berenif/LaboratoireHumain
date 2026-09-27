import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { solveTwoBone } = await import('../src/character/pose.ts');
const { add, sub, scale, cross, length, normalize, rotate, quatFromAxisAngle } = await import('../src/character/math.ts');
const start = { x: 0, y: 0, z: 0 }, end = { x: -0.1, y: -0.5, z: 0 };
const preferred = { x: 0.22, y: 0, z: -1 }, right = { x: 1, y: 0, z: 0 };
const lengths = [0.31, 0.27], bendLimit = 2.5;
const solve = clearance => solveTwoBone(start, end, ...lengths, preferred, bendLimit, right, clearance);
const close = (a, b, tolerance = 1e-10) => assert.ok(length(sub(a, b)) < tolerance, JSON.stringify({ a, b }));
const checkLengths = result => {
  assert.ok(Math.abs(length(sub(result.middle, start)) - lengths[0]) < 1e-12);
  assert.ok(Math.abs(length(sub(result.end, result.middle)) - lengths[1]) < 1e-12);
};

test('elbow clearance preserves the wrist and lengths with the nearest feasible point on the circle', () => {
  const before = solve(), clearance = { direction: right, minimumOffset: 0.025 }, after = solve(clearance);
  assert.ok(before.middle.x < clearance.minimumOffset);
  assert.ok(Math.abs(after.middle.x - clearance.minimumOffset) < 1e-12);
  assert.deepEqual(after.end, before.end);
  checkLengths(after);
  // Independently enumerate the circle to reject a more distant branch.
  const direction = normalize(end), distance = length(end);
  const along = (lengths[0] ** 2 - lengths[1] ** 2 + distance ** 2) / (2 * distance);
  const center = scale(direction, along), radius = Math.sqrt(lengths[0] ** 2 - along ** 2);
  const u = normalize(cross(direction, { x: 0, y: 0, z: 1 })), v = cross(direction, u);
  for (let i = 0; i < 1440; i++) {
    const angle = i * Math.PI / 720;
    const point = add(center, scale(add(scale(u, Math.cos(angle)), scale(v, Math.sin(angle))), radius));
    if (point.x >= clearance.minimumOffset) assert.ok(length(sub(point, before.middle)) >= length(sub(after.middle, before.middle)) - 1e-12);
  }
});

test('inactive clearance is identical and its activation is continuous', () => {
  const before = solve();
  assert.deepEqual(solve({ direction: right, minimumOffset: before.middle.x - 0.001 }), before);
  const after = solve({ direction: right, minimumOffset: before.middle.x + 1e-7 });
  close(before.middle, after.middle, 1e-6);
  checkLengths(after);
});

test('infeasible clearance maximizes the available offset without stretching', () => {
  const result = solve({ direction: right, minimumOffset: 10 });
  checkLengths(result);
  assert.deepEqual(result.end, solve().end);
  const direction = normalize(end), distance = length(end);
  const along = (lengths[0] ** 2 - lengths[1] ** 2 + distance ** 2) / (2 * distance);
  const radius = Math.sqrt(lengths[0] ** 2 - along ** 2);
  const maximumX = along * direction.x + radius * Math.sqrt(1 - direction.x ** 2);
  assert.ok(Math.abs(result.middle.x - maximumX) < 1e-12);
  assert.deepEqual(solve({ direction, minimumOffset: 10 }), solve());
});

test('clearance follows a rotated and translated anatomical frame', () => {
  const clearance = { direction: right, minimumOffset: 0.025 }, expected = solve(clearance);
  const rotation = quatFromAxisAngle(normalize({ x: 1, y: 2, z: -1 }), 1.3), offset = { x: 0.3, y: -0.7, z: 0.5 };
  const point = p => add(offset, rotate(rotation, p));
  const actual = solveTwoBone(point(start), point(end), ...lengths, rotate(rotation, preferred), bendLimit,
    rotate(rotation, right), { ...clearance, direction: rotate(rotation, right) });
  close(actual.middle, point(expected.middle));
  close(actual.end, point(expected.end));
});
