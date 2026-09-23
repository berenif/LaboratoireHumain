import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register(); after(unregister);
const { predictTransferTouchdown } = await import("../src/character/transfer-prediction.ts");
const { quatFromAxisAngle, rotate } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 };
const massKg = 72.2;

function patch(segment, x) {
  return { segment, point: { x, y: 0, z: 0 },
    points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: x + sx * 0.05, y: 0, z: sz * 0.10 }))),
    forceN: massKg * 9.81 / 2, normalY: 1, persistenceS: 1, loadBearing: true };
}
function fixture(overrides = {}) {
  return { centerOfMass: { x: 0.12, y: 1, z: 0 }, centerOfMassVelocity: zero,
    contacts: [patch("leftFoot", -0.12), patch("rightFoot", 0.12)], retainedSide: "right",
    desiredCenterOfMass: { x: 0.12, y: 0, z: 0 }, externalForce: zero, massKg, ...overrides };
}
function close(actual, expected, tolerance = 1e-9) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
}

function readyContacts() {
  return [patch("leftFoot", -0.12), patch("rightFoot", 0.12)]
    .map((contact, index) => ({ ...contact, forceN: massKg * 9.81 * (index ? 0.8 : 0.2) }));
}

test("an already captured transfer still requires readiness persistence before the swing forecast", () => {
  const input = fixture(), before = structuredClone(input);
  const result = predictTransferTouchdown(input);
  assert.equal(result.feasible, true);
  close(result.transferDurationS, 0.10);
  close(result.touchdownCapturePoint.x, 0.12);
  close(result.touchdownVelocity.x, 0);
  close(result.touchdownCenterOfMass.y, 1);
  assert.deepEqual(input, before, "prediction cannot mutate physical observations");
});

test("fully measured readiness starts the swing forecast without another transfer interval", () => {
  for (const readinessAgeS of [0.10, 0.17]) {
    const result = predictTransferTouchdown(fixture({ readinessAgeS, maxTransferDurationS: 0.01,
      contacts: readyContacts() }));
    assert.equal(result.feasible, true);
    close(result.transferDurationS, 0);
    close(result.touchdownCapturePoint.x, 0.12);
    close(result.touchdownVelocity.x, 0);
  }
});

test("partial measured readiness simulates exactly the remaining qualified interval", () => {
  for (const readinessAgeS of [0.04, 0.073]) {
    const result = predictTransferTouchdown(fixture({ readinessAgeS, contacts: readyContacts() }));
    assert.equal(result.feasible, true);
    close(result.transferDurationS, 0.10 - readinessAgeS);
    close(result.touchdownCapturePoint.x, 0.12);
    close(result.touchdownVelocity.x, 0);
  }
});

test("an initially unready candidate discards measured readiness history", () => {
  const input = fixture({ centerOfMass: { x: 0, y: 1, z: 0 },
    centerOfMassVelocity: { x: -0.10, y: 0, z: 0 } });
  const fresh = predictTransferTouchdown(input);
  assert.equal(fresh.feasible, true);
  assert.ok(fresh.transferDurationS > 0.10);
  for (const readinessAgeS of [0.073, 0.10]) {
    assert.deepEqual(predictTransferTouchdown({ ...input, readinessAgeS }), fresh);
    const expired = predictTransferTouchdown({ ...input, readinessAgeS, maxTransferDurationS: 0.10 });
    assert.equal(expired.feasible, false);
    assert.equal(expired.reason, "transfer-timeout");
  }
});

test("a new pressure forecast cannot erase measured transfer readiness", () => {
  const input = fixture({ contacts: readyContacts(),
    centerOfMass: { x: 0.04, y: 1, z: 0 }, centerOfMassVelocity: { x: 0.07, y: 0, z: 0 } });
  const full = predictTransferTouchdown({ ...input, readinessAgeS: 0.10, maxTransferDurationS: 0.01 });
  assert.equal(full.feasible, true, full.reason);
  close(full.transferDurationS, 0);
  const partial = predictTransferTouchdown({ ...input, readinessAgeS: 5 / 60 });
  assert.equal(partial.feasible, true, partial.reason);
  close(partial.transferDurationS, 1 / 60);
});

test("opposite momentum must be braked before transfer and cannot inherit an already-ready launch", () => {
  const result = predictTransferTouchdown(fixture({
    centerOfMass: { x: 0, y: 1, z: 0 }, centerOfMassVelocity: { x: -0.10, y: 0, z: 0 },
  }));
  assert.equal(result.feasible, true);
  assert.ok(result.transferDurationS > 0.10);
  assert.ok(result.touchdownCapturePoint.x > 0.07, "controlled transfer must replace straight velocity extrapolation");
  const escaping = predictTransferTouchdown(fixture({
    centerOfMass: { x: 0, y: 1, z: 0 }, centerOfMassVelocity: { x: -0.8, y: 0, z: 0 },
  }));
  assert.equal(escaping.feasible, false);
  assert.equal(escaping.reason, "force-budget");
  assert.ok(escaping.transferDurationS <= 1.2);
  const expired = predictTransferTouchdown(fixture({
    centerOfMass: { x: 0, y: 1, z: 0 }, centerOfMassVelocity: { x: -0.10, y: 0, z: 0 },
    maxTransferDurationS: 0.10,
  }));
  assert.equal(expired.feasible, false);
  assert.equal(expired.reason, "transfer-timeout");
  close(expired.transferDurationS, 0.10);
});

test("rotating the measured world rotates the touchdown prediction without changing transfer time", () => {
  const input = fixture({ centerOfMass: { x: -0.01, y: 1, z: 0.025 },
    centerOfMassVelocity: { x: -0.04, y: 0, z: 0.01 }, externalForce: { x: 2, y: 0, z: -1 } });
  // Unequal, skewed solver manifolds also exercise pressure-share ownership;
  // symmetric rectangles alone can hide a world-axis-dependent triangulation.
  input.contacts = input.contacts.map(contact => ({ ...contact,
    points: contact.points.map(point => ({ ...point,
      x: point.x + (point.z > 0 ? 0.01 : 0),
      z: point.z + (contact.segment === "leftFoot" ? 0.015 : -0.02)
        + (point.x > contact.point.x ? 0.012 : 0),
    })),
  }));
  const result = predictTransferTouchdown(input);
  assert.equal(result.feasible, true);
  for (const angle of [Math.PI / 3, Math.PI / 2, -Math.PI * 0.73]) {
    const quaternion = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, angle);
    const r = vector => rotate(quaternion, vector);
    const rotated = predictTransferTouchdown({ ...input,
      centerOfMass: r(input.centerOfMass), centerOfMassVelocity: r(input.centerOfMassVelocity),
      desiredCenterOfMass: r(input.desiredCenterOfMass), externalForce: r(input.externalForce),
      contacts: input.contacts.map(contact => ({ ...contact, point: r(contact.point), points: contact.points.map(r) })),
    });
    assert.equal(rotated.feasible, true);
    close(rotated.transferDurationS, result.transferDurationS);
    const expected = r(result.touchdownCapturePoint), expectedVelocity = r(result.touchdownVelocity);
    for (const axis of ["x", "y", "z"]) {
      close(rotated.touchdownCapturePoint[axis], expected[axis]);
      close(rotated.touchdownVelocity[axis], expectedVelocity[axis]);
    }
  }
});

test("a candidate without qualified retained sole contact cannot fabricate transfer support", () => {
  const input = fixture();
  input.contacts[1].persistenceS = 0.01;
  const result = predictTransferTouchdown(input);
  assert.equal(result.feasible, false);
  assert.equal(result.reason, "no-retained-support");
});

test("loaded point and line contacts forecast a dynamic catch without requiring static capture", () => {
  const point = { x: 0.12, y: 0, z: 0.03 };
  const variants = [[point], [{ x: 0.095, y: 0, z: 0.03 }, { x: 0.145, y: 0, z: 0.03 }]];
  const predictions = variants.map(points => predictTransferTouchdown(fixture({
    contacts: [{ ...patch("rightFoot", 0.12), point, points, forceN: massKg * 9.81 }],
    centerOfMass: { x: 0.12, y: 1, z: -0.02 }, centerOfMassVelocity: { x: 0, y: 0, z: -0.04 },
    desiredCenterOfMass: { x: 0.12, y: 0, z: 0.03 },
  })));
  for (const result of predictions) {
    assert.equal(result.feasible, true, "real point/line pressure must not be discarded for lacking area");
    close(result.transferDurationS, 0.10, 1e-9);
    assert.ok(result.touchdownCapturePoint.z < -0.02,
      "the caller must place the landing behind support to catch this outgoing capture");
    assert.ok(result.touchdownVelocity.z < 0);
    close(result.touchdownCapturePoint.x, 0.12);
  }
  close(predictions[0].touchdownCapturePoint.z, predictions[1].touchdownCapturePoint.z);
});
