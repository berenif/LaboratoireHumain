import assert from 'node:assert/strict';
import { writeFileSync } from 'node:fs';
import { SEGMENTS } from '../src/core/humanoid.ts';
import { flattenGeometryVertices, flattenGeometryIndices } from '../src/core/geometry.ts';
import { integrateConvexMass, inertiaTensor } from './convex-mass-properties.mjs';
import { sha256 } from './capture-physics-baseline.mjs';

const vector = v => ({ x: v.x, y: v.y, z: v.z });
const rotation = q => ({ ...vector(q), w: q.w });
const difference = (a, b) => Math.max(...a.flatMap((row, i) => row.map((v, j) => Math.abs(v - b[i][j]))));

export function installIntegratedInertia(character, path) {
  assert.equal(character.fixedSteps, 0, 'Mass properties are an initialization-only intervention');
  const records = [];
  for (const definition of SEGMENTS) {
    const body = character.ragdollBodies.get(definition.id), collider = character.ragdollColliders.get(definition.id);
    const capture = () => ({ position: vector(body.translation()), rotation: rotation(body.rotation()),
      velocity: vector(body.linvel()), angularVelocity: vector(body.angvel()), mass: body.mass(), com: vector(body.localCom()),
      principalInertia: vector(body.principalInertia()), principalFrame: rotation(body.principalInertiaLocalFrame()) });
    const before = capture();
    const integrated = integrateConvexMass([...flattenGeometryVertices(definition.geometry)],
      [...flattenGeometryIndices(definition.geometry)], before.mass);
    collider.setMassProperties(integrated.mass, integrated.centerOfMass, integrated.principalInertia, integrated.principalFrame);
    body.recomputeMassPropertiesFromColliders();
    const after = capture();
    for (const key of ['position', 'rotation', 'velocity', 'angularVelocity']) assert.deepEqual(after[key], before[key]);
    assert.ok(Math.abs(after.mass - before.mass) <= before.mass * 1e-6);
    assert.ok(Math.max(...['x', 'y', 'z'].map(axis => Math.abs(after.com[axis] - before.com[axis]))) < 1e-7);
    const beforeError = difference(inertiaTensor(before.principalInertia, before.principalFrame), integrated.inertia);
    const afterError = difference(inertiaTensor(after.principalInertia, after.principalFrame), integrated.inertia);
    assert.ok(afterError <= Math.max(...integrated.inertia.flat().map(Math.abs)) * 1e-6, `Inertia readback mismatch: ${definition.id}`);
    records.push({ id: definition.id, before, after, integrated, beforeError, afterError });
  }
  const originalStep = character.world.step.bind(character.world);
  character.world.step = (...args) => {
    const result = originalStep(...args);
    for (const record of records) {
      const body = character.ragdollBodies.get(record.id);
      assert.deepEqual(vector(body.principalInertia()), record.after.principalInertia);
      assert.deepEqual(rotation(body.principalInertiaLocalFrame()), record.after.principalFrame);
      assert.equal(body.mass(), record.after.mass);
    }
    return result;
  };
  const bytes = JSON.stringify({ generatedAt: new Date().toISOString(), records,
    qualification: 'Initialization-only geometric mass-property calculation on unchanged convex meshes and body masses. Body origins, rotations and velocities are unchanged; local COM roundoff is recorded. Solver, contacts and motor ceilings are untouched.' }, null, 2) + '\n';
  writeFileSync(path, bytes, { flag: 'wx' });
  return { path, sha256: sha256(bytes), bodies: records.length,
    maximumBeforeTensorError: Math.max(...records.map(r => r.beforeError)), maximumAfterTensorError: Math.max(...records.map(r => r.afterError)) };
}
