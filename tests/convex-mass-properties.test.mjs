import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register(); after(unregister);
const { diagonalize, inertiaTensor, integrateConvexMass } = await import('../src/core/geometry-mass.ts');

const near = (a, b, tolerance = 1e-11) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const triangles = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
test('unit right tetrahedron has the analytic COM and inertia', () => {
  const result = integrateConvexMass([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], triangles, 2);
  near(result.volume, 1 / 6);
  for (const v of Object.values(result.centerOfMass)) near(v, 1 / 4);
  result.inertia.forEach((row, i) => row.forEach((v, j) => near(v, i === j ? 3 / 20 : 1 / 40)));
});
test('integration is translation invariant and handles reversed winding', () => {
  const vertices = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
  const translated = vertices.map((v, i) => v + [3, -2, 5][i % 3]);
  const indices = triangles.flatMap((_, i) => i % 3 ? [] : [triangles[i], triangles[i + 2], triangles[i + 1]]);
  const result = integrateConvexMass(translated, indices, 2);
  Object.values(result.centerOfMass).forEach((v, i) => near(v, [3.25, -1.75, 5.25][i]));
  result.inertia.forEach((row, i) => row.forEach((v, j) => near(v, i === j ? 3 / 20 : 1 / 40)));
});
test('diagonalization preserves small rotated anisotropic and repeated inertias', () => {
  for (const size of [1e-9, 1e-3, 1, 1e6]) for (const values of [[3, 2, 1], [1, 1, 4], [2, 2, 2]]) {
    const n = Math.sqrt(30), q = { x: 1 / n, y: 2 / n, z: 3 / n, w: 4 / n };
    const matrix = inertiaTensor({ x: values[0] * size, y: values[1] * size, z: values[2] * size }, q);
    const result = diagonalize(matrix), actual = inertiaTensor(result.principalInertia, result.principalFrame);
    actual.forEach((row, i) => row.forEach((v, j) => near(v, matrix[i][j], size * 1e-11)));
  }
});

test('all production bodies retain geometric tensors through Rapier construction', async () => {
  const { createEmbodiedCharacter } = await import('../src/character/index.ts');
  const { SEGMENTS } = await import('../src/core/humanoid.ts');
  const { flattenGeometryVertices, flattenGeometryIndices } = await import('../src/core/geometry.ts');
  for (const heading of [0, Math.PI / 3]) {
    const character = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      for (const definition of SEGMENTS) {
        const expected = integrateConvexMass(Array.from(flattenGeometryVertices(definition.geometry)),
          Array.from(flattenGeometryIndices(definition.geometry)), definition.massKg);
        const body = character.ragdollBodies.get(definition.id);
        const measured = inertiaTensor(body.principalInertia(), body.principalInertiaLocalFrame());
        const tolerance = Math.max(...expected.inertia.flat().map(Math.abs)) * 1e-6;
        measured.forEach((row, i) => row.forEach((value, j) => near(value, expected.inertia[i][j], tolerance)));
        near(body.mass(), definition.massKg, definition.massKg * 1e-6);
        for (const axis of ['x', 'y', 'z']) near(body.localCom()[axis], expected.centerOfMass[axis], 1e-7);
      }
    } finally { character.dispose(); }
  }
});
