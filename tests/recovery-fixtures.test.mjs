import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { SEGMENTS } = await import("../src/core/humanoid.ts");
const { length, sub, worldPoint, rotate, quatFromAxisAngle, quatInverse, quatMultiply } = await import("../src/character/math.ts");

test("landed recovery fixtures preserve anatomy, finite transforms and zero initial momentum", () => {
  assert.equal(RECOVERY_POSE_FIXTURES.length, 24);
  for (const fixture of RECOVERY_POSE_FIXTURES) {
    const poses = recoveryFixturePoses(fixture);
    assert.equal(poses.size, SEGMENTS.length);
    for (const d of SEGMENTS) {
      const p = poses.get(d.id);
      assert.ok([...Object.values(p.position), ...Object.values(p.rotation)].every(Number.isFinite));
      assert.equal(length(p.linearVelocity), 0);
      assert.equal(length(p.angularVelocity), 0);
      if (!d.parent) continue;
      const parent = poses.get(d.parent);
      const separation = length(sub(worldPoint(parent.position, parent.rotation, d.jointAnchorParent), worldPoint(p.position, p.rotation, d.jointAnchorChild)));
      assert.ok(separation < 1e-12, `${fixture.id}/${d.id} separated ${separation}`);
    }
  }
});

test("landed fixtures transform exactly with heading and mirror every asymmetric limb", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.heading === 0 && f.side === "left")) {
    const base = recoveryFixturePoses(fixture);
    const turned = recoveryFixturePoses({ ...fixture, heading: Math.PI / 3 });
    const mirrored = recoveryFixturePoses({ ...fixture, side: "right" });
    const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, Math.PI / 3);
    for (const [id, p] of base) {
      const opposite = id.startsWith("left") ? id.replace("left", "right") : id.startsWith("right") ? id.replace("right", "left") : id;
      assert.ok(length(sub(turned.get(id).position, rotate(yaw, p.position))) < 1e-12, `${fixture.id}/${id} heading`);
      assert.ok(length(sub(mirrored.get(opposite).position, { x: -p.position.x, y: p.position.y, z: p.position.z })) < 1e-12, `${fixture.id}/${id} mirror`);
    }
  }
});

test("supported crouch seeds two level soles underneath and half-kneel seeds a raised trailing foot", () => {
  for (const side of ["left", "right"]) {
    const crouch = recoveryFixturePoses({ id: "check", pose: "crouch", side, heading: 0 });
    for (const foot of ["leftFoot", "rightFoot"]) {
      assert.ok(Math.abs(crouch.get(foot).position.y - 0.046) < 1e-10);
      assert.ok(rotate(crouch.get(foot).rotation, { x: 0, y: 1, z: 0 }).y > 0.999);
    }
    const kneel = recoveryFixturePoses({ id: "check", pose: "half-kneel", side, heading: 0 });
    const trailing = side === "left" ? "right" : "left";
    assert.ok(kneel.get(`${side}Foot`).position.y < 0.06, "leading foot begins at floor");
    assert.ok(kneel.get(`${trailing}Foot`).position.y > kneel.get(`${side}Foot`).position.y + 0.07, "trailing foot is folded back");
    const thigh = kneel.get(`${trailing}Thigh`);
    const knee = worldPoint(thigh.position, thigh.rotation, { x: 0, y: -0.21, z: 0 });
    assert.ok(knee.y < 0.085, `trailing knee begins near floor: ${knee.y}`);
  }
});


test("landed fixture joint rotations stay within existing motor ranges", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES) {
    const poses = recoveryFixturePoses(fixture);
    for (const d of SEGMENTS) {
      if (!d.parent) continue;
      const q = quatMultiply(quatInverse(poses.get(d.parent).rotation), poses.get(d.id).rotation);
      const x = /Shin|Forearm/.test(d.id) ? 2 * Math.atan2(q.x, q.w) : Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.x - q.y * q.z))));
      const y = /Shin|Forearm/.test(d.id) ? 0 : Math.atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.x * q.x + q.y * q.y));
      const z = /Shin|Forearm/.test(d.id) ? 0 : Math.atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z));
      for (const [axis, angle] of Object.entries({ x, y, z })) assert.ok(Math.abs(angle) <= d.jointLimitRadians[axis] + 1e-8, fixture.id + "/" + d.id + "/" + axis + " motor range: " + angle);
      if (d.id.endsWith("Shin")) assert.ok(x >= 0, fixture.id + " knee bends forward");
      if (d.id.endsWith("Forearm")) assert.ok(x <= 0, fixture.id + " elbow keeps natural bend");
    }
  }
});


test("balanced half-kneeling starts with actual loaded foot and shin contacts around projected mass", async () => {
  const { createEmbodiedCharacter } = await import("../src/character/index.ts");
  const { recoveryMassState, supportGeometry } = await import("../src/character/recovery-support.ts");
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.pose === "half-kneel" && f.support !== "weak")) {
    const { seedRecoveryFixture } = await import("../scripts/recovery-fixtures.ts");
    const character = await createEmbodiedCharacter("canvas2d", { heading: fixture.heading });
    try {
      seedRecoveryFixture(character, fixture);
      for (let step = 0; step < 3; step++) character.fixedUpdate(1 / 60, null);
      const snapshot = character.getSnapshot("canvas2d"), contacts = [];
      for (const [segment, collider] of character.ragdollColliders) {
        let impulse = 0;
        const points = [];
        character.world.contactPair(character.floorCollider, collider, (manifold, flipped) => {
          if (manifold.normal().y * (flipped ? -1 : 1) < 0.65) return;
          for (let index = 0; index < manifold.numSolverContacts(); index++) {
            const point = manifold.solverContactPoint(index);
            if (point && manifold.solverContactDist(index) <= 0.012) points.push({ ...point });
          }
          for (let index = 0; index < manifold.numContacts(); index++) impulse += Math.max(0, manifold.contactImpulse(index));
        });
        const forceN = impulse * 60;
        if (points.length && forceN >= 3) contacts.push({ segment, point: points[0], points, normalY: 1, forceN, loadBearing: true, persistenceS: 1 / 60 });
      }
      const trailing = fixture.side === "left" ? "right" : "left";
      assert.ok(contacts.some(c => c.segment === fixture.side + "Foot"), fixture.id + " leading sole carries actual solver load");
      assert.ok(contacts.some(c => c.segment === trailing + "Shin"), fixture.id + " trailing shin carries actual solver load");
      const geometry = supportGeometry(contacts, new Map(snapshot.segments.map(p => [p.id, p])), recoveryMassState(snapshot.segments));
      assert.ok(geometry.marginM >= 0, fixture.id + " projected COM starts inside actual loaded support: " + geometry.marginM);
    } finally { character.dispose(); }
  }
});
