import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { restPoseMap } = await import("../src/character/pose.ts");
const { SEGMENTS, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const { measureRecoveryPhysics, newRecoveryPhysicsMeasurements } = await import("../scripts/recovery-measurements.ts");
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;

function sensors() {
  const segments = [...restPoseMap().values()].map(p => ({ ...p, linearVelocity: zero, angularVelocity: zero }));
  const center = { ...zero };
  for (const p of segments) {
    const mass = SEGMENTS.find(d => d.id === p.id).massKg / TOTAL_MASS_KG;
    for (const axis of ["x", "y", "z"]) center[axis] += p.position[axis] * mass;
  }
  const snapshot = (time, released = []) => ({ simulationTime: time, segments, diagnostics: { authority: "ragdoll", recovery: {
    phase: "kneel", route: "none", retries: 0, transferStage: "none", leadingSide: "left", rollSide: "left",
    centerOfMass: center, projectedCenterOfMass: center, assistanceForce: zero, assistanceTorque: zero,
    plantedTargets: [], releasedSupports: released,
  } } });
  const loads = new Set(["leftFoot", "rightFoot"]);
  const colliders = new Map([...loads].map(segment => [segment, { segment }]));
  const internals = { floorCollider: { isEnabled: () => true }, ragdollColliders: colliders, world: {
    contactPair: (_floor, collider, callback) => {
      if (!loads.has(collider.segment)) return;
      const side = collider.segment === "leftFoot" ? -1 : 1;
      const points = [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: center.x + side * 0.02 + sx * 0.05, y: 0, z: center.z + sz * 0.135 })));
      callback({ normal: () => ({ x: 0, y: 1, z: 0 }), numSolverContacts: () => 4, solverContactDist: () => 0,
        solverContactPoint: i => points[i], numContacts: () => 1, contactImpulse: () => 350 * dt }, false);
    },
  } };
  const measured = newRecoveryPhysicsMeasurements(), violations = new Set();
  let previous = snapshot(0), time = 0;
  const step = (released = []) => {
    time += dt; const current = snapshot(time, released);
    measureRecoveryPhysics(measured, current, previous, internals, violations); previous = current;
  };
  return { measured, violations, loads, step };
}

test("independent contact measurement excludes an earlier deliberate release even while its old manifold remains loaded", () => {
  const state = sensors();
  for (let i = 0; i < 3; i++) state.step();
  state.step(["leftFoot"]);
  assert.equal(state.violations.size, 0, "right sole alone covers projected mass");
  state.step(["rightFoot"]);
  assert.ok(state.violations.has("Support intentionally released outside independently measured 0.15 s remaining support area"));
  assert.deepEqual(state.measured.report.releaseEvents.at(-1).remaining, [], "neither the old release nor the new release is valid remaining support");
});

test("independent release measurement restores a support only after actual unload and persistent replant", () => {
  const state = sensors();
  for (let i = 0; i < 3; i++) state.step();
  state.step(["leftFoot"]);
  state.loads.delete("leftFoot"); state.step();
  state.loads.add("leftFoot"); state.step(); state.step();
  assert.ok(state.measured.deliberatelyReleased.has("leftFoot"), "two loaded frames do not establish a new support");
  state.step();
  assert.ok(!state.measured.deliberatelyReleased.has("leftFoot"), "three persistent raw-load frames reestablish the sole");
  state.step(["rightFoot"]);
  assert.equal(state.violations.size, 0);
  assert.deepEqual(state.measured.report.releaseEvents.at(-1).remaining, ["leftFoot"]);
  assert.ok(state.measured.report.minimumReleaseMarginM > 0);
});
