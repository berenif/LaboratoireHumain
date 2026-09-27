import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register(); after(unregister);
const { distributeSupportLoad } = await import("../src/character/support-loads.ts");
const ids = ["leftFoot", "leftForefoot", "rightFoot", "rightForefoot", "leftHand"];
const vertices = [[-2, -1], [1, -1], [2, 1], [0, 2], [-2, 1]];
const contacts = vertices.map(([x, z], index) => ({
  segment: ids[index], point: { x: x * 0.1, y: 0, z: z * 0.1 },
  forceN: 100, normalY: 1, persistenceS: 1, loadBearing: true,
}));
const distribute = (points, requested) => distributeSupportLoad(points, requested, { rotationInvariant: true });
const shares = loads => Object.fromEntries(loads.map(load => [load.segment, load.share]));

test("micrometre pressure motion across a support diagonal cannot jump contact loads", () => {
  const before = shares(distribute(contacts, { x: -1e-6, y: 0, z: 0 }));
  const after = shares(distribute(contacts, { x: 1e-6, y: 0, z: 0 }));
  for (const id of ids) assert.ok(Math.abs(before[id] - after[id]) < 1e-4, id);
});

test("continuous pressure allocation preserves force, moment, heading and contact owners", () => {
  for (const requested of [{ x: 0.02, y: 0, z: 0.01 }, contacts[0].point,
    { x: -0.05, y: 0, z: -0.1 }]) {
    const reference = shares(distribute(contacts, requested));
    for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
      const transform = p => ({ x: Math.cos(heading) * p.x + Math.sin(heading) * p.z + 2,
        y: 0.04, z: -Math.sin(heading) * p.x + Math.cos(heading) * p.z - 3 });
      const loads = distribute(contacts.map(c => ({ ...c, point: transform(c.point) })), transform(requested));
      assert.ok(loads.every(load => ids.includes(load.segment) && load.share >= 0));
      assert.ok(Math.abs(loads.reduce((sum, load) => sum + load.share, 0) - 1) < 1e-10);
      for (const coordinate of ["x", "y", "z"]) assert.ok(Math.abs(loads.reduce((sum, load) =>
        sum + load.point[coordinate] * load.share, 0) - transform(requested)[coordinate]) < 1e-9);
      const turned = shares(loads);
      for (const id of ids) assert.ok(Math.abs((turned[id] ?? 0) - (reference[id] ?? 0)) < 1e-8);
    }
  }
});
