import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register();
after(unregister);
const { BalanceController, BALANCE_LIMITS, massState } = await import("../src/character/BalanceController.ts");
const { composeUprightPose, restPoseMap, poseAnchor } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { quatFromAxisAngle, quatMultiply, rotate, sub, length, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;
const footCenterHeight = SEGMENT_BY_ID.get("leftFoot").shape.halfExtents.y;

function poseInput(heading = 0) {
  const rest = restPoseMap(), yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  return { rootTranslation: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 }, reactionOffset: zero,
    simulationTime: 0, activeGrab: null, step: null, heading,
    supportFeet: Object.fromEntries(["leftFoot", "rightFoot"].map(foot => [foot, rotate(yaw, { ...rest.get(foot).position, y: footCenterHeight })])),
  };
}

function assertConnected(poses) {
  for (const definition of SEGMENTS) {
    if (!definition.parent) continue;
    const separation = length(sub(poseAnchor(poses.get(definition.parent), definition.jointAnchorParent), poseAnchor(poses.get(definition.id), definition.jointAnchorChild)));
    assert.ok(separation < 1e-8, `${definition.id}: joint separation ${separation}`);
  }
}

test("solved ankles stay connected for unreachable grabs and steps at rotated headings", () => {
  for (const heading of [0, Math.PI / 2, -0.7, 2.2]) {
    const base = poseInput(heading);
    const rest = composeUprightPose(base).poses;
    for (const side of ["left", "right"]) {
      const foot = `${side}Foot`;
      const start = rest.get(foot).position;
      const pose = composeUprightPose({ ...base, kneeFlexion: 0.65, reactionOffset: { x: 0.5, y: 0, z: -0.3 },
        activeGrab: { region: foot, startTarget: start, startSegmentPosition: start, target: { x: start.x + 4, y: start.y + 0.8, z: start.z - 2 } },
      }).poses;
      assertConnected(pose);
      const stepping = composeUprightPose({ ...base, step: { foot, from: start, to: { x: 3, y: footCenterHeight, z: -2 }, elapsed: 0.30, duration: 0.34 } }).poses;
      assertConnected(stepping);
    }
  }
});

test("heading rotates every composed segment and its joint geometry consistently", () => {
  const heading = 1.1, yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  const base = composeUprightPose({ ...poseInput(), kneeFlexion: 0.4 }).poses;
  const turned = composeUprightPose({ ...poseInput(heading), kneeFlexion: 0.4 }).poses;
  for (const [id, pose] of base) {
    assert.ok(length(sub(turned.get(id).position, rotate(yaw, pose.position))) < 1e-8, `${id} position`);
    const expected = quatMultiply(yaw, pose.rotation), actual = turned.get(id).rotation;
    assert.ok(Math.abs(expected.x * actual.x + expected.y * actual.y + expected.z * actual.z + expected.w * actual.w) > 1 - 1e-8, `${id} rotation`);
  }
});

test("balance mass estimate includes segment masses and rejects a manipulated near-floor foot", () => {
  const input = poseInput(), poses = composeUprightPose(input).poses;
  for (const pose of poses.values()) pose.linearVelocity = { x: 0.4, y: -0.2, z: 0.1 };
  const mass = massState(poses);
  assert.ok(length(sub(mass.velocity, { x: 0.4, y: -0.2, z: 0.1 })) < 1e-12);
  const controller = new BalanceController(); controller.reset(poses);
  const start = poses.get("rightFoot").position;
  const output = controller.update({ dt, poses, rootPosition: input.rootTranslation,
    activeGrab: { region: "rightFoot", target: { ...start, x: start.x + 0.1 }, startTarget: start, startSegmentPosition: start },
  });
  assert.deepEqual(output.diagnostics.supportingFeet, ["leftFoot"]);
  assert.ok(output.appliedGrabForceN <= BALANCE_LIMITS.maxPullForceN);
});

test("slow pulls stay connected and stepping remains available after release and reversal", async () => {
  for (const heading of [0, 1.1, -1.7]) {
    const character = await createEmbodiedCharacter("canvas2d", { heading });
    const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    try {
      const start = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand").position;
      character.fixedUpdate(dt, { kind: "begin", pointerId: 3, region: "rightHand", segment: "rightHand", localAnchor: zero, worldTarget: start, timestampMs: 0 });

      for (let tick = 1; tick <= 510; tick += 1) {
        let command = null;
        if (tick <= 90) {
          const delta = rotate(yaw, { x: 0.60 * tick / 90, y: 0, z: 0.08 * tick / 90 });
          command = { kind: "move", pointerId: 3, worldTarget: { x: start.x + delta.x, y: start.y, z: start.z + delta.z }, timestampMs: tick * dt * 1000 };
        } else if (tick <= 180) {
          const delta = rotate(yaw, { x: 0.6 - (tick - 90) * 0.9 / 90, y: 0, z: 0.08 });
          command = { kind: "move", pointerId: 3, worldTarget: { x: start.x + delta.x, y: start.y, z: start.z + delta.z }, timestampMs: tick * dt * 1000 };
        } else if (tick === 181) {
          command = { kind: "end", pointerId: 3, timestampMs: tick * dt * 1000 };
        }
        character.fixedUpdate(dt, command);
        const snapshot = character.getSnapshot("canvas2d");

        assert.equal(snapshot.diagnostics.authority, "character-motor", `heading ${heading}, tick ${tick}`);
        assert.ok(snapshot.diagnostics.maxJointSeparationM < 1e-8);
        assert.ok(snapshot.diagnostics.maxFloorPenetrationM <= 0.08);
        assert.ok(snapshot.diagnostics.finite);
        if (snapshot.support.swingFoot) assert.ok(!snapshot.support.planted.includes(snapshot.support.swingFoot));
      }
      const done = character.getSnapshot("canvas2d");
      assert.ok(done.diagnostics.stepCount >= 1);
      assert.equal(done.state, "upright");

    } finally { character.dispose(); }
  }
});

test("a planted reversal step does not create a transient unsupported fall", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  const localAnchor = { x: 0.025, y: 0.015, z: 0.01 };
  try {
    const hand = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand");
    const start = worldPoint(hand.position, hand.rotation, localAnchor);
    character.fixedUpdate(dt, { kind: "begin", pointerId: 41, region: "rightHand", segment: "rightHand", localAnchor, worldTarget: start, timestampMs: 0 });

    for (let tick = 1; tick <= 510; tick += 1) {
      let command = null;
      if (tick < 300) {
        const x = tick <= 90 ? 0.75 * tick / 90
          : tick <= 180 ? 0.75 - 1.5 * (tick - 90) / 90 : -0.75;
        command = { kind: "move", pointerId: 41, worldTarget: { x: start.x + x, y: start.y + 0.02 * Math.min(tick, 90) / 90, z: start.z }, timestampMs: tick * dt * 1000 };
      } else if (tick === 300) {
        command = { kind: "end", pointerId: 41, timestampMs: tick * dt * 1000 };
      }
      character.fixedUpdate(dt, command);
      const snapshot = character.getSnapshot("canvas2d");
      assert.equal(snapshot.diagnostics.authority, "character-motor", `tick ${tick}`);
    }

    const done = character.getSnapshot("canvas2d");
    assert.ok(done.diagnostics.stepCount >= 2);
    assert.equal(done.state, "upright");
  } finally { character.dispose(); }
});
