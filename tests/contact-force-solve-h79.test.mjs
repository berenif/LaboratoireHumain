import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { solveContactForcesH79 } = await import('../src/character/coordinated-contact-solve-h79.ts');
const { solveContactForces, CONTACT_SOLVE } = await import('../src/character/coordinated-contact-solve.ts');
const { add, cross, sub, rotate, quatFromAxisAngle } = await import('../src/character/math.ts');
const zero = { x: 0, y: 0, z: 0 }, axes = ['x', 'y', 'z'];
const contact = (segment, x) => ({ segment, point: { x, y: 0, z: 0 }, forceN: 300,
  measuredForceN: 300, normalY: 1, persistenceS: .2, loadBearing: true });
const contacts = [contact('leftFoot', -.1), contact('rightFoot', .1)];
const fixture = overrides => ({ contacts, origin: { x: 0, y: 1, z: 0 }, requestedForce: { x: 0, y: 600, z: 0 },
  requestedMoment: zero, weightN: 600, momentLengthM: 1, maxHorizontalForceN: 200,
  previous: [], motors: [], clock: () => 0, ...overrides });
const squared = vector => axes.reduce((sum, axis) => sum + vector[axis] ** 2, 0);
// Independent physical-unit objective evaluation, without constructing solver rows.
function objective(input, loads) {
  const force = loads.reduce((sum, load) => add(sum, load.plannedForce), zero);
  const moment = loads.reduce((sum, load) => add(sum, cross(sub(load.point, input.origin), load.plannedForce)), zero);
  let total = squared(sub(force, input.requestedForce)) / input.weightN ** 2
    + squared(sub(moment, input.requestedMoment)) / (input.weightN * input.momentLengthM) ** 2;
  const normalized = loads.flatMap(load => axes.map(axis => load.plannedForce[axis] / input.weightN));
  for (const row of input.motors) total += CONTACT_SOLVE.effortRegularization
    * ((row.baseNm + row.coefficients.reduce((sum, coefficient, i) => sum + coefficient * normalized[i], 0)) / row.capNm) ** 2;
  for (const load of loads) total += CONTACT_SOLVE.changeRegularization
    * squared(sub(load.plannedForce, input.previous.find(item => item.segment === load.segment)?.force ?? zero)) / input.weightN ** 2;
  return total;
}
function assertConstraints(input, result) {
  assert.equal(result.status, 'allocated');
  const cfg = CONTACT_SOLVE, toleranceN = cfg.constraintTolerance * input.weightN + 1e-9;
  assert.ok(result.constraintViolation <= cfg.constraintTolerance);
  assert.ok(result.allocatedForce.y <= cfg.maximumVerticalWeight * input.weightN + toleranceN);
  assert.ok(Math.hypot(result.allocatedForce.x, result.allocatedForce.z) <= input.maxHorizontalForceN + toleranceN);
  for (const load of result.loads) {
    assert.ok(load.plannedForce.y >= -toleranceN);
    assert.ok(Math.hypot(load.plannedForce.x, load.plannedForce.z) <= cfg.frictionCoefficient * load.plannedForce.y + toleranceN);
    assert.ok(axes.every(axis => Number.isFinite(load.plannedForce[axis])));
  }
  const forces = result.loads.flatMap(load => axes.map(axis => load.plannedForce[axis] / input.weightN));
  for (const row of input.motors) {
    const request = row.baseNm + row.coefficients.reduce((sum, coefficient, i) => sum + coefficient * forces[i], 0);
    assert.ok(Math.abs(request) <= row.capNm * (1 + cfg.constraintTolerance) + 1e-9);
  }
  assert.ok(Math.abs(objective(input, result.loads) - result.objective) < 1e-10);
}

test('H79 gives the exact unchanged symmetric least-squares optimum', () => {
  const input = fixture(), result = solveContactForcesH79(input);
  assertConstraints(input, result); assert.equal(result.method, 'analytic');
  assert.equal(result.converged, true); assert.equal(result.iterations, 1);
  for (const load of result.loads) {
    assert.ok(Math.abs(load.plannedForce.y - 600 / 2.02) < 1e-9);
    assert.ok(Math.hypot(load.plannedForce.x, load.plannedForce.z) < 1e-9);
  }
  assert.ok(result.objective <= solveContactForces(input).objective + 1e-12);
});

test('analytic objective and stationary solution include original motor and previous-load regularization', () => {
  for (let i = 0; i < 8; i++) {
    const input = fixture({ requestedForce: { x: 10 + i, y: 500 + i * 10, z: -5 }, requestedMoment: { x: .2, y: -.1, z: .3 },
      previous: contacts.map((c, j) => ({ segment: c.segment, force: { x: j + i, y: 240 + j * 20, z: -i } })),
      motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 1, capNm: 20, coefficients: [1, 2, 3, -2, 1, .5] },
        { id: 'rightAnkle', coordinate: 'z', baseNm: -1, capNm: 30, coefficients: [-1, 1, -.5, 2, -1, 3] }] });
    const before = structuredClone({ ...input, clock: undefined }), result = solveContactForcesH79(input), old = solveContactForces(input);
    assertConstraints(input, result); assert.equal(result.method, 'analytic');
    assert.ok(result.objective <= old.objective + 1e-12);
    assert.deepEqual({ ...input, clock: undefined }, before);
    for (const [index] of result.loads.entries()) for (const axis of axes) for (const sign of [-1, 1]) {
      const shifted = structuredClone(result.loads); shifted[index].plannedForce[axis] += sign * .01;
      assert.ok(objective(input, shifted) >= result.objective - 1e-14);
    }
  }
});

test('contact ordering and heading rotate the same unconstrained optimum and all diagnostics', () => {
  const input = fixture({ requestedForce: { x: 30, y: 600, z: -12 }, requestedMoment: { x: 2, y: 1, z: -3 },
    previous: contacts.map(c => ({ segment: c.segment, force: { x: 3, y: 280, z: -2 } })),
    motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 1, capNm: 30, coefficients: [1, 2, -1, 3, -2, 1] }] });
  const first = solveContactForcesH79(input), reversed = solveContactForcesH79({ ...input, contacts: [...contacts].reverse() });
  assert.deepEqual(first, reversed);
  const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 1.1);
  const turnedInput = { ...input, contacts: contacts.map(c => ({ ...c, point: rotate(yaw, c.point) })),
    origin: rotate(yaw, input.origin), requestedForce: rotate(yaw, input.requestedForce), requestedMoment: rotate(yaw, input.requestedMoment),
    previous: input.previous.map(p => ({ ...p, force: rotate(yaw, p.force) })),
    motors: input.motors.map(row => ({ ...row, coefficients: contacts.flatMap((_, i) => {
      const vector = rotate(yaw, { x: row.coefficients[i * 3], y: row.coefficients[i * 3 + 1], z: row.coefficients[i * 3 + 2] });
      return axes.map(axis => vector[axis]);
    }) })) };
  const turned = solveContactForcesH79(turnedInput); assertConstraints(turnedInput, turned);
  assert.equal(turned.method, 'analytic');
  assert.ok(Math.abs(turned.objective - first.objective) < 1e-12);
  for (const key of ['allocatedForce', 'allocatedMoment', 'forceResidual', 'momentResidual']) {
    const expected = rotate(yaw, first[key]); assert.ok(Math.sqrt(squared(sub(turned[key], expected))) < 1e-8);
  }
});

test('inadmissible analytic requests preserve the original constrained motor, normal, friction and total-force solve', () => {
  const cases = [fixture({ requestedForce: { x: 500, y: 1500, z: 0 },
    motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 9, capNm: 10, coefficients: [0, 20, 0, 0, 0, 0] }] }),
  fixture({ origin: zero, requestedForce: { x: 900, y: 300, z: 0 }, maxHorizontalForceN: 2000 }),
  fixture({ requestedForce: { x: 0, y: -600, z: 0 } }),
  fixture({ requestedForce: { x: 0, y: 1500, z: 0 } })];
  for (const input of cases) {
    const result = solveContactForcesH79(input), old = solveContactForces(input);
    assert.equal(result.method, 'h78-fallback');
    const { method, ...output } = result; assert.deepEqual(output, old);
    assertConstraints(input, result);
  }
});

test('malformed state, missing contacts, and impossible motor constants cannot enter the analytic path', () => {
  const cases = [fixture({ contacts: [] }), fixture({ weightN: NaN }), fixture({ origin: { ...zero, x: Infinity } }),
    fixture({ contacts: [contacts[0], contacts[0]] }), fixture({ contacts: [{ ...contacts[0], loadBearing: false, forceN: NaN }] }),
    fixture({ previous: [{ segment: 'leftFoot', force: { ...zero, y: NaN } }] }),
    fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 11, capNm: 10, coefficients: Array(6).fill(0) }] }),
    fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 0, capNm: 0, coefficients: Array(6).fill(0) }] }),
    fixture({ contacts: contacts.map(c => ({ ...c, sleepingEquilibrium: true, measuredForceN: 0 })) })];
  for (const input of cases) {
    const result = solveContactForcesH79(input); assert.equal(result.method, 'h78-fallback');
    assert.equal(result.status, solveContactForces(input).status); assert.notEqual(result.status, 'allocated');
  }
});

test('ill-conditioned normal equations defer to the unchanged H78 solve', () => {
  const input = fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 0, capNm: 1, coefficients: [1e9, 1e9, 0, 0, 0, 0] }] });
  const result = solveContactForcesH79(input), { method, ...output } = result;
  assert.equal(method, 'h78-fallback'); assert.deepEqual(output, solveContactForces(input));
});

test('analytic preparation and fallback share the original absolute eight-millisecond deadline', () => {
  assert.equal(solveContactForcesH79(fixture({ clock: () => 9, deadlineAt: 8 })).status, 'timeout');
  // Start and preprocessing consume six ms; fallback starts at eight and must
  // time out at its first later clock read, rather than receiving another eight.
  let time = -2;
  const result = solveContactForcesH79(fixture({ requestedForce: { x: 0, y: 1500, z: 0 }, clock: () => time += 2 }));
  assert.equal(result.method, 'h78-fallback'); assert.equal(result.status, 'timeout');
  assert.ok(result.solveMs >= 8);
  let calls = 0;
  const late = solveContactForcesH79(fixture({ clock: () => ++calls < 5 ? 0 : 9, deadlineAt: 8 }));
  assert.equal(late.status, 'timeout'); assert.ok(late.solveMs >= 9);
});
