import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { solveContactForcesH80, solveContactForcesH80Constrained } = await import('../src/character/coordinated-contact-solve-h80.ts');
const { solveContactForcesH79 } = await import('../src/character/coordinated-contact-solve-h79.ts');
const { solveContactForces, CONTACT_SOLVE } = await import('../src/character/coordinated-contact-solve.ts');
const { rotate, quatFromAxisAngle } = await import('../src/character/math.ts');
const axes = ['x', 'y', 'z'], zero = { x: 0, y: 0, z: 0 };
const contact = (segment, x, z = 0) => ({ segment, point: { x, y: 0, z }, forceN: 300,
  measuredForceN: 300, normalY: 1, persistenceS: .2, loadBearing: true });
const contacts = [contact('leftFoot', -.1), contact('rightFoot', .1)];
const fixture = overrides => ({ contacts, origin: { x: 0, y: 1, z: 0 }, requestedForce: { x: 0, y: 600, z: 0 },
  requestedMoment: zero, weightN: 600, momentLengthM: 1, maxHorizontalForceN: 200,
  previous: [], motors: [], clock: () => 0, ...overrides });
function near(a, b, tolerance = 1e-7) { assert.ok(Math.abs(a - b) <= tolerance, `${a} differs from ${b}`); }
function equivalent(a, b) {
  assert.equal(a.status, b.status); assert.equal(a.converged, b.converged); assert.equal(a.iterations, b.iterations);
  assert.equal(a.loads.length, b.loads.length);
  for (const key of ['allocatedForce', 'allocatedMoment', 'forceResidual', 'momentResidual']) for (const axis of axes) near(a[key][axis], b[key][axis]);
  for (const [i, load] of a.loads.entries()) {
    assert.equal(load.segment, b.loads[i].segment);
    for (const axis of axes) near(load.plannedForce[axis], b.loads[i].plannedForce[axis]);
  }
  for (const key of ['headroom', 'constraintViolation', 'objective']) near(a[key], b[key], 1e-10 * Math.max(1, Math.abs(b[key])));
}
function constraints(input, result) {
  if (result.status !== 'allocated') return;
  const tolerance = CONTACT_SOLVE.constraintTolerance, toleranceN = tolerance * input.weightN + 1e-9;
  assert.ok(result.constraintViolation <= tolerance);
  assert.ok(result.allocatedForce.y <= input.weightN * CONTACT_SOLVE.maximumVerticalWeight + toleranceN);
  assert.ok(Math.hypot(result.allocatedForce.x, result.allocatedForce.z) <= input.maxHorizontalForceN + toleranceN);
  for (const load of result.loads) {
    assert.ok(load.plannedForce.y >= -toleranceN);
    assert.ok(Math.hypot(load.plannedForce.x, load.plannedForce.z) <= CONTACT_SOLVE.frictionCoefficient * load.plannedForce.y + toleranceN);
  }
  const forces = result.loads.flatMap(load => axes.map(axis => load.plannedForce[axis] / input.weightN));
  for (const row of input.motors) {
    const request = row.baseNm + row.coefficients.reduce((sum, coefficient, i) => sum + coefficient * forces[i], 0);
    assert.ok(Math.abs(request) <= row.capNm * (1 + tolerance) + 1e-9);
  }
}

test('H80 preserves H78 numerical results across two/four soles, sparse motor rows and active constraints', () => {
  for (let sample = 0; sample < 24; sample++) {
    const soleSet = sample % 2 ? [...contacts, contact('leftForefoot', -.1, .15), contact('rightForefoot', .1, .15)] : contacts;
    const n = soleSet.length * 3;
    const input = fixture({ contacts: soleSet, requestedForce: { x: (sample % 5 - 2) * 130, y: 500 + (sample % 4) * 300, z: (sample % 3 - 1) * 50 },
      requestedMoment: { x: sample % 3, y: sample % 2, z: sample % 4 },
      previous: soleSet.map((c, i) => ({ segment: c.segment, force: { x: i * 2, y: 600 / soleSet.length, z: -i } })),
      motors: Array.from({ length: 48 }, (_, row) => ({ id: row % 2 ? 'leftAnkle' : 'rightAnkle', coordinate: axes[row % 3],
        baseNm: (row % 7 - 3) / 2, capNm: 10 + row % 3,
        coefficients: Array.from({ length: n }, (_, column) => row % 6 === 0 ? 0 : column % (row % 3 + 2) === 0 ? (row % 5 - 2) * 6 : 0) })) });
    const before = structuredClone({ ...input, clock: undefined }), old = solveContactForces(input), result = solveContactForcesH80Constrained(input);
    equivalent(result, old); constraints(input, result);
    assert.deepEqual({ ...input, clock: undefined }, before);
    assert.ok(result.iterations <= 64);
  }
  assert.equal(CONTACT_SOLVE.iterations, 64); assert.equal(CONTACT_SOLVE.projectionSweeps, 24); assert.equal(CONTACT_SOLVE.deadlineMs, 8);
});

test('H80 uses unchanged H79 certified analytic solution and optimized constrained fallback', () => {
  const simple = fixture(), analytic = solveContactForcesH80(simple);
  assert.deepEqual(analytic, solveContactForcesH79(simple)); assert.equal(analytic.method, 'analytic');
  const input = fixture({ requestedForce: { x: 500, y: 1500, z: 0 },
    motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 9, capNm: 10, coefficients: [0, 20, 0, 0, 0, 0] }] });
  const result = solveContactForcesH80(input), old = solveContactForcesH79(input);
  assert.equal(result.method, 'h80-constrained'); assert.equal(old.method, 'h78-fallback');
  equivalent(result, old); constraints(input, result);
});

test('H79 default fallback remains exactly unchanged when the optional H80 callback is omitted', () => {
  const input = fixture({ requestedForce: { x: 0, y: 1500, z: 0 } });
  const old = solveContactForcesH79(input), explicit = solveContactForcesH79(input, solveContactForces);
  assert.deepEqual(old, explicit);
  let passedDeadline;
  solveContactForcesH79({ ...input, deadlineAt: 3 }, supplied => { passedDeadline = supplied.deadlineAt; return solveContactForces(supplied); });
  assert.equal(passedDeadline, 3);
});

test('ordered sparse products preserve contact ordering and heading invariance', () => {
  const input = fixture({ origin: zero, requestedForce: { x: 900, y: 300, z: -30 }, maxHorizontalForceN: 2000,
    motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 0, capNm: 300, coefficients: [1, 2, 0, -1, 0, 2] }] });
  const first = solveContactForcesH80(input), reversed = solveContactForcesH80({ ...input, contacts: [...contacts].reverse() });
  assert.deepEqual(first, reversed);
  const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 1.1);
  const turnedInput = { ...input, contacts: contacts.map(c => ({ ...c, point: rotate(yaw, c.point) })),
    origin: rotate(yaw, input.origin), requestedForce: rotate(yaw, input.requestedForce),
    motors: input.motors.map(row => ({ ...row, coefficients: contacts.flatMap((_, i) => {
      const value = rotate(yaw, { x: row.coefficients[i * 3], y: row.coefficients[i * 3 + 1], z: row.coefficients[i * 3 + 2] });
      return axes.map(axis => value[axis]);
    }) })) };
  const turned = solveContactForcesH80(turnedInput); constraints(turnedInput, turned);
  // The inherited infinity-norm stopping test is coordinate dependent. Preserve
  // its exact H78 result at each heading and its existing 1e-3 N covariance bound.
  equivalent(turned, solveContactForcesH79(turnedInput));
  for (const key of ['allocatedForce', 'allocatedMoment', 'forceResidual', 'momentResidual']) {
    const expected = rotate(yaw, first[key]); for (const axis of axes) near(turned[key][axis], expected[axis], .001);
  }
  near(first.objective, turned.objective, 1e-10);
});

test('constant violated motors, malformed priors, sleeping/ineligible contacts and invalid parameters still fail explicitly', () => {
  const cases = [fixture({ contacts: [] }), fixture({ weightN: NaN }), fixture({ momentLengthM: 0 }), fixture({ maxHorizontalForceN: -1 }),
    fixture({ contacts: [{ ...contacts[0], loadBearing: false, measuredForceN: NaN }] }),
    fixture({ contacts: [contacts[0], contacts[0]] }),
    fixture({ contacts: contacts.map(c => ({ ...c, sleepingEquilibrium: true, measuredForceN: 0 })) }),
    fixture({ previous: [{ segment: 'leftFoot', force: { ...zero, y: Infinity } }] }),
    fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 11, capNm: 10, coefficients: Array(6).fill(0) }] }),
    fixture({ motors: [{ id: 'leftAnkle', coordinate: 'x', baseNm: 0, capNm: 0, coefficients: Array(6).fill(0) }] })];
  for (const input of cases) {
    const old = solveContactForces(input), raw = solveContactForcesH80Constrained(input), combined = solveContactForcesH80(input);
    assert.equal(raw.status, old.status); assert.equal(combined.status, old.status); assert.notEqual(combined.status, 'allocated');
  }
});

test('constrained and analytical work both use the original absolute deadline without a reset', () => {
  assert.equal(solveContactForcesH80Constrained(fixture({ clock: () => 9, deadlineAt: 8 })).status, 'timeout');
  const input = fixture({ requestedForce: { x: 0, y: 1500, z: 0 } });
  let clock = -2;
  const combined = solveContactForcesH80({ ...input, clock: () => clock += 2 });
  assert.equal(combined.status, 'timeout'); assert.equal(combined.method, 'h80-constrained'); assert.ok(combined.solveMs >= 8);
  let originalTime = 0, optimizedTime = 0;
  const old = solveContactForces({ ...input, clock: () => originalTime++ / 4, deadlineAt: 1 });
  const optimized = solveContactForcesH80Constrained({ ...input, clock: () => optimizedTime++ / 4, deadlineAt: 1 });
  assert.deepEqual(optimized, old);
});
