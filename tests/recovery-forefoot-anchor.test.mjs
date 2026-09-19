import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register(); after(unregister);
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { solveRecoveryForefootAnchor } = await import("../src/character/recovery-forefoot-anchor.ts");
const { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } = await import("../scripts/recovery-fixtures.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");

const IDENTITY = { x: 0, y: 0, z: 0, w: 1 };

test("a planted prone forefoot remains reachable while the pelvis rises through legal leg joints", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    seedRecoveryFixture(character, RECOVERY_POSE_FIXTURES.find(fixture => fixture.id === "landed-prone-left"));
    for (let step = 0; step < 32; step += 1) character.fixedUpdate(1 / 60, null);
    const snapshot = character.getSnapshot("canvas2d");
    assert.ok(snapshot.rootPosition.y < .13, "the integrated fixture has a low resting pelvis");
    const measured = new Map(snapshot.segments.map(pose => [pose.id, pose]));
    const before = JSON.stringify(snapshot.segments);
    for (const side of ["left", "right"]) {
      const forefoot = measured.get(`${side}Forefoot`);
      const contact = snapshot.diagnostics.recovery.contacts.find(item => item.segment === `${side}Forefoot`);
      assert.ok(contact?.loadBearing && contact.normalY >= .65, `${side} forefoot has a real upward floor contact`);
      const patch = { position: forefoot.position, rotation: forefoot.rotation, contact: contact.point };
      for (const rise of [0, .03, .06, .09]) {
        const poses = new Map(measured);
        poses.set("pelvis", { ...measured.get("pelvis"), position: {
          ...measured.get("pelvis").position, y: measured.get("pelvis").position.y + rise,
        } });
        const target = solveRecoveryForefootAnchor(side, poses, patch);
        assert.ok(target, `${side} rear patch remains reachable after ${rise} m root rise`);
        assert.ok(target.patchErrorM < .01, `${side} material patch drift remains below 10 mm`);
        assert.ok(target.floorClearanceM >= -.003, `${side} leg geometry clears the floor`);
        assert.ok(target.bodyClearanceM >= .004, `${side} leg clears the trunk`);
        for (const [name, rotation] of Object.entries(target.jointRotations)) {
          const definition = SEGMENT_BY_ID.get(`${side}${name[0].toUpperCase()}${name.slice(1)}`);
          const coordinates = jointCoordinates(IDENTITY, rotation, definition.jointProfile);
          assert.ok(jointLimitErrorMagnitude(coordinates, definition.jointProfile) < 1e-8,
            `${side} ${name} remains inside its existing joint limits`);
        }
      }
      const unreachable = solveRecoveryForefootAnchor(side, measured, { ...patch,
        contact: { ...patch.contact, x: patch.contact.x + 2 },
      });
      assert.equal(unreachable, null, "an unreachable patch cannot become a claimed plant");
    }
    assert.equal(JSON.stringify(snapshot.segments), before, "planning never changes the physical snapshot");
  } finally {
    character.dispose();
  }
});
