import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register();
after(unregister);
const { restPoseMap } = await import("../src/character/pose.ts");
const { SEGMENTS, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const { measureRecoveryPhysics, newRecoveryPhysicsMeasurements } = await import("../scripts/recovery-measurements.ts");
const zero = { x: 0, y: 0, z: 0 };

function checkMassDiagnostic(useBodyOrigins) {
  const segments = [...restPoseMap().values()].map((pose, index) => ({
    ...pose,
    // Deliberately differ from both nominal body masses and transform origins.
    massKg: 1 + index / 10,
    centerOfMass: { x: pose.position.x + .02 * index, y: pose.position.y + .03, z: pose.position.z - .01 * index },
    linearVelocity: { x: .02 * index, y: -.1, z: .01 * index },
  }));
  const mass = segments.reduce((sum, segment) => sum + segment.massKg, 0);
  const position = { ...zero }, velocity = { ...zero }, oldPosition = { ...zero }, oldVelocity = { ...zero };
  for (const segment of segments) {
    const nominal = SEGMENTS.find(definition => definition.id === segment.id).massKg;
    for (const axis of ["x", "y", "z"]) {
      position[axis] += segment.massKg * segment.centerOfMass[axis] / mass;
      velocity[axis] += segment.massKg * segment.linearVelocity[axis] / mass;
      oldPosition[axis] += nominal * segment.position[axis] / TOTAL_MASS_KG;
      oldVelocity[axis] += nominal * segment.linearVelocity[axis] / TOTAL_MASS_KG;
    }
  }
  const centerOfMass = useBodyOrigins ? oldPosition : position;
  const selectedVelocity = useBodyOrigins ? oldVelocity : velocity;
  const projectedCenterOfMass = {
    x: centerOfMass.x + .15 * selectedVelocity.x,
    y: 0,
    z: centerOfMass.z + .15 * selectedVelocity.z,
  };
  const snapshot = simulationTime => ({
    simulationTime, state: "falling", segments,
    diagnostics: { physicsOwnership: "rapier-dynamic", recovery: {
      phase: "protect", route: "none", retries: 0, transferStage: "none", leadingSide: null, rollSide: null,
      centerOfMass, projectedCenterOfMass, assistanceForce: zero, assistanceTorque: zero,
      plantedTargets: [], releasedSupports: [],
    } },
  });
  const violations = new Set();
  measureRecoveryPhysics(newRecoveryPhysicsMeasurements(), snapshot(1 / 60), snapshot(0), {
    world: {}, floorCollider: { isEnabled: () => false }, ragdollColliders: new Map(),
  }, violations);
  return violations;
}

test("independent recovery observer accepts measured mass centres and measured-mass velocity projection", () => {
  assert.deepEqual([...checkMassDiagnostic(false)], []);
});

test("independent recovery observer still rejects origin-based COM and nominal-mass projection diagnostics", () => {
  const violations = checkMassDiagnostic(true);
  assert.ok(violations.has("Recovery COM is not the independent mass-weighted body center"));
  assert.ok(violations.has("Recovery projected COM does not use the independent 0.15 s mass-weighted velocity"));
});
