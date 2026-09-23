import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register(); after(unregister);
const { predictLandingCapture, predictControlledLandingCapture } = await import("../src/character/landing-capture.ts");
const { quatFromAxisAngle, rotate } = await import("../src/character/math.ts");

function fixture(overrides = {}) {
  return { centerOfMass: { x: 0.2, y: 1, z: -0.1 },
    centerOfMassVelocity: { x: 0.3, y: 0, z: 0.2 },
    centerOfMassAcceleration: { x: 0, y: 0, z: 0 }, floorY: 0,
    stepDurationS: 0.48, transferRampDistanceM: 0.3, transferSpeedMps: 1.5,
    readinessAgeS: 0, remainingTransferWindowS: 1.2, ...overrides };
}
function close(actual, expected, tolerance = 1e-10) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
}
function closePoint(actual, expected) {
  for (const axis of ["x", "y", "z"]) close(actual[axis], expected[axis]);
}
function valid(input) {
  const result = predictLandingCapture(input);
  assert.equal(result.valid, true, JSON.stringify(result));
  return result;
}

test("one bounded horizon includes the commanded ramp and remaining measured persistence", () => {
  close(valid(fixture()).horizonS, 0.78);
  close(valid(fixture({ readinessAgeS: 0.04 })).horizonS, 0.74);
  close(valid(fixture({ readinessAgeS: 0.04, remainingTransferWindowS: 0.05 })).horizonS, 0.53);
  close(valid(fixture({ remainingTransferWindowS: 0 })).horizonS, 0.48);
  close(valid(fixture({ readinessDurationS: 0.2, readinessAgeS: 0.05 })).horizonS, 0.83);
});

test("qualified full readiness removes all future transfer time, including a stale ramp distance", () => {
  for (const readinessAgeS of [0.10, 0.25, Array(6).fill(1 / 60).reduce((a, b) => a + b, 0)]) {
    close(valid(fixture({ readinessAgeS, transferRampDistanceM: 2 })).horizonS, 0.48);
  }
});

test("capture advances both COM position and velocity under measured acceleration", () => {
  // Height gives omega=2; T=.5. Analytic future COM=(.3375,.05), v=(.25,.4).
  const result = valid(fixture({ centerOfMass: { x: 0.2, y: 9.81 / 4, z: -0.1 },
    centerOfMassVelocity: { x: 0.3, y: 9, z: 0.2 },
    centerOfMassAcceleration: { x: -0.1, y: 100, z: 0.4 },
    stepDurationS: 0.5, remainingTransferWindowS: 0 }));
  closePoint(result.acceleratedCapturePoint, { x: 0.4625, y: 0, z: 0.25 });
  closePoint(result.constantVelocityCapturePoint, { x: 0.5, y: 0, z: 0.1 });
});

test("braking forecast preserves a separate common continuation of measured momentum", () => {
  const base = fixture({ centerOfMass: { x: 0, y: 9.81, z: 0 },
    centerOfMassVelocity: { x: 1, y: 0, z: 0 }, stepDurationS: 0.5,
    readinessAgeS: 0.10 });
  const braking = valid({ ...base, centerOfMassAcceleration: { x: -2, y: 0, z: 0 } });
  close(braking.acceleratedCapturePoint.x, 0.25);
  close(braking.constantVelocityCapturePoint.x, 1.5);
  assert.deepEqual(braking.constantVelocityCapturePoint, valid(base).constantVelocityCapturePoint);
});

test("both forecasts rotate with measured motion without selecting a support side", () => {
  const input = fixture({ centerOfMassAcceleration: { x: -0.7, y: 0, z: 0.4 } });
  const base = valid(input);
  for (const heading of [Math.PI / 2, -0.71, Math.PI]) {
    const rotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const rotated = valid({ ...input, centerOfMass: rotate(rotation, input.centerOfMass),
      centerOfMassVelocity: rotate(rotation, input.centerOfMassVelocity),
      centerOfMassAcceleration: rotate(rotation, input.centerOfMassAcceleration) });
    close(rotated.horizonS, base.horizonS);
    closePoint(rotated.acceleratedCapturePoint, rotate(rotation, base.acceleratedCapturePoint));
    closePoint(rotated.constantVelocityCapturePoint, rotate(rotation, base.constantVelocityCapturePoint));
  }
});

test("translating the measured COM and floor vertically preserves horizontal forecasts", () => {
  const input = fixture(), expected = valid(input);
  assert.deepEqual(valid({ ...input, centerOfMass: { ...input.centerOfMass, y: 4 }, floorY: 3 }), expected);
});

test("frozen observations remain unchanged and output vectors do not alias observations", () => {
  const input = fixture(), before = structuredClone(input);
  Object.values(input).filter(value => value && typeof value === "object").forEach(Object.freeze);
  Object.freeze(input);
  const result = valid(input);
  result.acceleratedCapturePoint.x = 123;
  assert.deepEqual(input, before);
  assert.notEqual(result.constantVelocityCapturePoint.x, 123);
});

test("invalid timing, measurements and nonpositive COM height fail explicitly", () => {
  for (const override of [
    { floorY: NaN }, { stepDurationS: 0 }, { transferSpeedMps: 0 },
    { transferRampDistanceM: -1 }, { readinessAgeS: -1 }, { remainingTransferWindowS: -1 },
    { readinessDurationS: 0 }, { centerOfMassVelocity: { x: Infinity, y: 0, z: 0 } },
    { centerOfMassAcceleration: { x: 0, y: NaN, z: 0 } },
  ]) assert.deepEqual(predictLandingCapture(fixture(override)), { valid: false, reason: "invalid-input" });
  for (const floorY of [1, 2]) {
    assert.deepEqual(predictLandingCapture(fixture({ floorY })), { valid: false, reason: "invalid-height" });
  }
  assert.deepEqual(predictLandingCapture(fixture({ stepDurationS: 1e308,
    centerOfMassAcceleration: { x: 1, y: 0, z: 0 } })), { valid: false, reason: "non-finite-prediction" });
});

function controlledFixture(overrides = {}) {
  const massKg = 72.2;
  const contacts = ["leftFoot", "rightFoot"].map((segment, index) => {
    const x = index ? 0.12 : -0.12;
    return { segment, point: { x, y: 0, z: 0 },
      points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: x + sx * 0.05, y: 0, z: sz * 0.10 }))),
      forceN: massKg * 9.81 * (index ? 0.8 : 0.2), normalY: 1, persistenceS: 1, loadBearing: true };
  });
  return { ...fixture(), centerOfMass: { x: 0.12, y: 1, z: 0 },
    centerOfMassVelocity: { x: 0, y: 0, z: 0 },
    currentDesiredCenterOfMass: { x: 0.12, y: 0, z: 0 },
    committedDesiredCenterOfMass: { x: 0.12, y: 0, z: 0 },
    transferRampDistanceM: 0, readinessAgeS: 0.10, contacts, retainedSide: "right",
    externalForce: { x: 0, y: 0, z: 0 }, massKg, ...overrides };
}
function controlledValid(input) {
  const result = predictControlledLandingCapture(input);
  assert.equal(result.valid, true, JSON.stringify(result));
  return result;
}

test("controlled forecast shares the bounded ramp and measured-persistence horizon", () => {
  for (const readinessAgeS of [0, 0.04, 0.10]) {
    const input = controlledFixture({ readinessAgeS, transferRampDistanceM: 0.3 });
    const result = controlledValid(input);
    close(result.horizonS, valid(input).horizonS);
    closePoint(result.capturePoint, { x: 0.12, y: 0, z: 0 });
    closePoint(result.velocity, { x: 0, y: 0, z: 0 });
    assert.equal(result.pressureFeasible, true);
  }
});

test("controlled braking does not hold a transient measured acceleration through touchdown", () => {
  const input = controlledFixture({ centerOfMassVelocity: { x: 0.12, y: 0, z: 0 },
    centerOfMassAcceleration: { x: 0.6, y: 0, z: 0 } });
  const result = controlledValid(input), constant = valid(input);
  assert.ok(constant.acceleratedCapturePoint.x > 0.35, "constant acceleration would extrapolate beyond the sole");
  assert.ok(result.capturePoint.x < 0.17, "bounded sole pressure can brake this modest outward motion");
  assert.ok(result.velocity.x < 0.03, "the modeled controller must reduce outward velocity");
  assert.deepEqual(controlledValid({ ...input, centerOfMassAcceleration: { x: -0.6, y: 0, z: 0 } }), result,
    "the controlled forecast must not add a second measured-acceleration force");
});

test("controlled forecast cannot brake escaping rightward momentum with only a left sole", () => {
  const input = controlledFixture({ centerOfMass: { x: 0, y: 1, z: 0 },
    centerOfMassVelocity: { x: 1.2, y: 0, z: 0 }, retainedSide: "left",
    currentDesiredCenterOfMass: { x: -0.12, y: 0, z: 0 },
    committedDesiredCenterOfMass: { x: -0.12, y: 0, z: 0 } });
  input.contacts = [{ ...input.contacts[0], forceN: input.massKg * 9.81 }];
  const result = controlledValid(input);
  assert.ok(result.velocity.x >= 1.2, "left-only pressure cannot supply the requested leftward braking force");
  assert.ok(result.capturePoint.x > 0.43 + 0.05, "touchdown capture remains outside the full permitted step travel and sole");
  assert.equal(result.pressureFeasible, false);
  assert.ok(result.maxPressureForceResidualNm > 0, "future wrench residual remains visible without an invented force");
});

test("controlled forecast adds measured external force exactly once", () => {
  const dt = 1 / 60, input = controlledFixture({ stepDurationS: dt,
    externalForce: { x: 7.22, y: 100, z: -3.61 } });
  const result = controlledValid(input), omega = Math.sqrt(9.81);
  closePoint(result.velocity, { x: 0.1 * dt, y: 0, z: -0.05 * dt });
  closePoint(result.capturePoint, { x: 0.12 + 0.1 * dt * dt + 0.1 * dt / omega,
    y: 0, z: -0.05 * dt * dt - 0.05 * dt / omega });
});

test("controlled forecast rotates the committed command, real patches and capture together", () => {
  const input = controlledFixture({ centerOfMassVelocity: { x: 0.08, y: 0, z: -0.03 },
    currentDesiredCenterOfMass: { x: 0.08, y: 0, z: -0.01 },
    readinessAgeS: 0.04, transferRampDistanceM: 0.04,
    externalForce: { x: 2, y: 0, z: 1 } });
  const expected = controlledValid(input);
  for (const angle of [0.7, Math.PI / 2, -2.1]) {
    const q = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, angle), r = value => rotate(q, value);
    const actual = controlledValid({ ...input,
      centerOfMass: r(input.centerOfMass), centerOfMassVelocity: r(input.centerOfMassVelocity),
      currentDesiredCenterOfMass: r(input.currentDesiredCenterOfMass),
      committedDesiredCenterOfMass: r(input.committedDesiredCenterOfMass), externalForce: r(input.externalForce),
      contacts: input.contacts.map(contact => ({ ...contact, point: r(contact.point), points: contact.points.map(r) })) });
    close(actual.horizonS, expected.horizonS);
    closePoint(actual.capturePoint, r(expected.capturePoint));
    closePoint(actual.velocity, r(expected.velocity));
    assert.equal(actual.pressureFeasible, expected.pressureFeasible);
  }
});

test("controlled initial selection shares current support without inventing a future retained contact", () => {
  const input = controlledFixture({ centerOfMass: { x: 0, y: 1, z: 0 },
    currentDesiredCenterOfMass: { x: 0, y: 0, z: 0 },
    committedDesiredCenterOfMass: { x: 0, y: 0, z: 0 }, retainedSide: undefined });
  closePoint(controlledValid(input).capturePoint, { x: 0, y: 0, z: 0 });
  const leftOnly = [input.contacts[0]];
  assert.deepEqual(predictControlledLandingCapture({ ...input, contacts: leftOnly, retainedSide: "right" }),
    { valid: false, reason: "no-retained-support" });
  assert.deepEqual(predictControlledLandingCapture({ ...input,
    contacts: input.contacts.map(contact => ({ ...contact, loadBearing: false })) }),
    { valid: false, reason: "no-qualified-support" });
});

test("controlled swing conditions its budget on launch load without changing weak measured contacts", () => {
  for (const normalY of [1, 0.75]) {
    const input = controlledFixture({ readinessAgeS: 0,
      centerOfMassVelocity: { x: 0.12, y: 0, z: 0 } });
    const weight = input.massKg * 9.81;
    input.contacts = input.contacts.map((contact, index) => ({ ...contact, normalY,
      forceN: index ? 17 : weight - 17 }));
    const before = structuredClone(input);
    const weak = controlledValid(input);
    const atLaunch = controlledValid({ ...input,
      contacts: input.contacts.map((contact, index) => ({ ...contact,
        forceN: weight * (index ? 0.52 : 0.48) })) });
    closePoint(weak.capturePoint, atLaunch.capturePoint);
    closePoint(weak.velocity, atLaunch.velocity);
    assert.ok(weak.velocity.x < input.centerOfMassVelocity.x,
      "a conditional swing has the existing launch-load budget to brake");
    assert.deepEqual(input, before, "forecasting load acceptance cannot rewrite actual contact forces");
    assert.equal(input.contacts[1].forceN, 17);
    const noRetainedLoad = { ...input, contacts: input.contacts.map((contact, index) =>
      ({ ...contact, forceN: index ? 0 : weight })) };
    assert.deepEqual(predictControlledLandingCapture(noRetainedLoad),
      { valid: false, reason: "no-retained-support" }, "a launch assumption cannot create an unloaded contact");
  }
});

test("controlled forecast preserves frozen observations and rejects invalid actuation inputs", () => {
  const input = controlledFixture(), before = structuredClone(input);
  function freeze(value) {
    if (value && typeof value === "object") { Object.values(value).forEach(freeze); Object.freeze(value); }
    return value;
  }
  const result = controlledValid(freeze(input));
  result.capturePoint.x = 123;
  assert.deepEqual(input, before);
  assert.notEqual(result.centerOfMass.x, 123);
  for (const overrides of [{ massKg: 0 }, { maxBalanceAccelerationMps2: NaN }, { frictionCoefficient: -1 },
    { externalForce: { x: Infinity, y: 0, z: 0 } }, { committedDesiredCenterOfMass: { x: 0, y: 0, z: NaN } }]) {
    assert.deepEqual(predictControlledLandingCapture({ ...input, ...overrides }), { valid: false, reason: "invalid-input" });
  }
});
