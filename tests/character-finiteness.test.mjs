import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { createEmbodiedCharacter } = await import('../src/character/index.ts');

test('a non-finite native center of mass cannot pass the published-state guard', async () => {
  const character = await createEmbodiedCharacter('canvas2d');
  try {
    assert.equal(character.diagnostics().finite, true);
    const body = character.ragdollBodies.get('leftUpperArm');
    // Inject an invalid mass frame without moving or stepping the body. The
    // failed multibody calibration exposed this same finite-pose/NaN-COM gap.
    body.setAdditionalMassProperties(1, { x: NaN, y: 0, z: 0 },
      { x: 0.1, y: 0.1, z: 0.1 }, { x: 0, y: 0, z: 0, w: 1 }, true);
    body.recomputeMassPropertiesFromColliders();
    character.readPhysicsPoses();
    const snapshot = character.getSnapshot('canvas2d');
    const arm = snapshot.segments.find(pose => pose.id === 'leftUpperArm');
    for (const field of ['position', 'rotation', 'linearVelocity', 'angularVelocity']) {
      assert.ok(Object.values(arm[field]).every(Number.isFinite), `${field} is still finite`);
    }
    assert.ok(Object.values(arm.centerOfMass).some(value => !Number.isFinite(value)));
    assert.equal(snapshot.diagnostics.finite, false);
    assert.ok(snapshot.diagnostics.errors.includes('NONFINITE_CHARACTER_STATE'));
  } finally { character.dispose(); }
});

test('non-finite published mass is rejected independently of geometry and center of mass', async () => {
  const character = await createEmbodiedCharacter('canvas2d');
  try {
    const original = character.poses.get('pelvis');
    assert.ok(Object.values(original.centerOfMass).every(Number.isFinite));
    for (const massKg of [NaN, Infinity, -Infinity]) {
      character.poses.set('pelvis', { ...original, massKg });
      const diagnostics = character.diagnostics();
      assert.equal(diagnostics.finite, false);
      assert.ok(diagnostics.errors.includes('NONFINITE_CHARACTER_STATE'));
    }
    character.poses.set('pelvis', original);
    assert.equal(character.diagnostics().finite, true);
  } finally { character.dispose(); }
});
