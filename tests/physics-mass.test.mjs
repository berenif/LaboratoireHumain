import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { measuredPhysicsMass } = await import("../scripts/physics-mass.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const zero = { x: 0, y: 0, z: 0 };

test("the QA mass oracle uses captured mass centers rather than mesh origins", () => {
  const sample = { segments: [
    { id: "pelvis", massKg: 4, position: { x: 90, y: 90, z: 90 }, centerOfMass: { x: 2, y: 3, z: 4 }, linearVelocity: { x: 1, y: 0, z: 0 } },
    { id: "leftHand", massKg: 1, position: { x: -90, y: -90, z: -90 }, centerOfMass: { x: -3, y: -2, z: -1 }, linearVelocity: zero },
  ] };
  assert.deepEqual(measuredPhysicsMass(sample), { position: { x: 1, y: 2, z: 3 }, velocity: { x: 0.8, y: 0, z: 0 } });
  assert.throws(() => measuredPhysicsMass({ segments: [] }), /empty/);
  assert.throws(() => measuredPhysicsMass({ segments: [{ ...sample.segments[0], massKg: NaN }] }), /Invalid/);
});

test("current-frame QA reads Rapier independently even if snapshot mass metadata is corrupt", async () => {
  const character = await createEmbodiedCharacter("canvas2d", { heading: 1.1 });
  try {
    for (let i = 0; i < 12; i++) character.fixedUpdate(1 / 60, null);
    const snapshot = character.getSnapshot("canvas2d");
    const expected = { position: { ...zero }, velocity: { ...zero } }; let total = 0;
    for (const body of character.ragdollBodies.values()) {
      const mass = body.mass(), center = body.worldCom(), velocity = body.linvel(); total += mass;
      for (const axis of ["x", "y", "z"]) {
        expected.position[axis] += mass * center[axis]; expected.velocity[axis] += mass * velocity[axis];
      }
    }
    for (const axis of ["x", "y", "z"]) { expected.position[axis] /= total; expected.velocity[axis] /= total; }
    const corrupted = { segments: snapshot.segments.map(pose => ({ ...pose, massKg: 1,
      position: { x: 99, y: 99, z: 99 }, centerOfMass: { x: 99, y: 99, z: 99 }, linearVelocity: { x: 99, y: 99, z: 99 } })) };
    assert.deepEqual(measuredPhysicsMass(corrupted, character.ragdollColliders), expected);
  } finally { character.dispose(); }
});
