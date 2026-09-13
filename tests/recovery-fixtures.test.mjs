import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { length, sub, worldPoint, rotate, quatFromAxisAngle } = await import("../src/character/math.ts");
const { jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");

const soleLowestPoint = (poses, side) => Math.min(...[`${side}Foot`, `${side}Forefoot`].flatMap((id) =>
  SEGMENT_BY_ID.get(id).geometry.vertices.map((vertex) => worldPoint(poses.get(id).position, poses.get(id).rotation, vertex).y)));

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
    for (const footSide of ["left", "right"]) {
      const soleParts = [`${footSide}Foot`, `${footSide}Forefoot`];
      const lowest = soleLowestPoint(crouch, footSide);
      assert.ok(Math.abs(lowest - 0.001) < 1e-10, `${footSide} articulated sole reaches the floor`);
      assert.ok(soleParts.every((id) => rotate(crouch.get(id).rotation, { x: 0, y: 1, z: 0 }).y > 0.98));
    }
    const kneel = recoveryFixturePoses({ id: "check", pose: "half-kneel", side, heading: 0 });
    const trailing = side === "left" ? "right" : "left";
    const leadingSoleY = soleLowestPoint(kneel, side);
    const trailingSoleY = soleLowestPoint(kneel, trailing);
    assert.ok(leadingSoleY < 0.004, `leading articulated sole begins at floor: ${leadingSoleY}`);
    assert.ok(trailingSoleY > leadingSoleY + 0.02, `trailing articulated sole is folded back: ${trailingSoleY}`);
    const trailingShin = `${trailing}Shin`;
    const shinLowest = Math.min(...SEGMENT_BY_ID.get(trailingShin).geometry.vertices.map(vertex =>
      worldPoint(kneel.get(trailingShin).position, kneel.get(trailingShin).rotation, vertex).y));
    assert.ok(shinLowest < 0.006, `trailing shin begins at its exact floor surface: ${shinLowest}`);
  }
});


test("landed fixture joint coordinates stay within shared asymmetric profiles", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES) {
    const poses = recoveryFixturePoses(fixture);
    for (const d of SEGMENTS) {
      if (!d.parent) continue;
      const coordinates = jointCoordinates(poses.get(d.parent).rotation, poses.get(d.id).rotation, d.jointProfile);
      assert.ok(jointLimitErrorMagnitude(coordinates, d.jointProfile) < 1e-8,
        fixture.id + "/" + d.id + " stays inside its profile: " + JSON.stringify(coordinates));
      const permitted = new Set(d.jointProfile.axes.map(({ coordinate }) => coordinate));
      for (const axis of ["x", "y", "z"]) {
        if (!permitted.has(axis)) assert.ok(Math.abs(coordinates[axis]) < 1e-8, `${fixture.id}/${d.id}/${axis} is locked`);
      }
      if (d.role === "shin") assert.ok(coordinates.x >= -1e-8, fixture.id + " knee bends forward");
      if (d.role === "forearm") assert.ok(coordinates.x >= -1e-8, fixture.id + " elbow flexion is positive");
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
      assert.ok(contacts.some(c => c.segment === fixture.side + "Foot" || c.segment === fixture.side + "Forefoot"), fixture.id + " leading sole carries actual solver load");
      assert.ok(contacts.some(c => c.segment === trailing + "Shin"), fixture.id + " trailing shin carries actual solver load");
      const geometry = supportGeometry(contacts, new Map(snapshot.segments.map(p => [p.id, p])), recoveryMassState(snapshot.segments));
      assert.ok(geometry.marginM >= 0, fixture.id + " projected COM starts inside actual loaded support: " + geometry.marginM);
    } finally { character.dispose(); }
  }
});
