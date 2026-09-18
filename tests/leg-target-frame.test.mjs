import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { ankleFromHindfoot, hindfootFromAnkle, legReachRadius, legTargetReach } = await import("../src/character/leg-target-frame.ts");
const { standingChainDiagnostics } = await import("../src/character/standing-chain-diagnostics.ts");
const { composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { BalanceController } = await import("../src/character/BalanceController.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { add, sub, length, rotate, quatFromAxisAngle, quatMultiply, quatInverse, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, up = { x: 0, y: 1, z: 0 }, dt = 1 / 60;
const identity = { x: 0, y: 0, z: 0, w: 1 };
const close = (a, b) => assert.ok(length(sub(a, b)) < 1e-9, `${JSON.stringify(a)} != ${JSON.stringify(b)}`);

function poseInput() {
  const rest = restPoseMap();
  return { rootTranslation: { x: 0, y: 0.99, z: 0 }, reactionOffset: zero, simulationTime: 0,
    activeGrab: null, supportFeet: { leftFoot: rest.get("leftFoot").position, rightFoot: rest.get("rightFoot").position },
    step: null, heading: 0, kneeFlexion: 0.12 };
}

test("hindfoot/ankle conversion round-trips real anchors under yaw, roll and translation", () => {
  for (const side of ["left", "right"]) for (const yaw of [0, 0.8, -1.7, Math.PI]) {
    const q = quatMultiply(quatFromAxisAngle(up, yaw), quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.23));
    const ankleQ = quatFromAxisAngle(up, yaw);
    const center = { x: 3.1, y: 0.13, z: -2.4 };
    close(hindfootFromAnkle(side, ankleFromHindfoot(side, center, q, ankleQ), q, ankleQ), center);
    close(ankleFromHindfoot(side, { x: 0, y: 0.045, z: 0.035 }, identity), { x: 0, y: 0.09, z: 0 });
  }
  assert.equal(legReachRadius("left"), legReachRadius("right"));
  assert.ok(Math.abs(legReachRadius("left") - 0.818) < 1e-12);
});

test("radial reach uses the current hip and does not mistake a foot centre for an ankle", () => {
  const hip = { x: 0, y: 0.89, z: 0 };
  const reachable = legTargetReach("right", hip, { x: 0, y: 0.045, z: 0.035 }, identity);
  assert.equal(reachable.radiallyReachable, true);
  assert.ok(Math.abs(reachable.distanceM - 0.8) < 1e-12);
  const distant = legTargetReach("right", hip, { x: 0.5, y: 0.045, z: 0.035 }, identity);
  assert.equal(distant.radiallyReachable, false);
  assert.ok(distant.excessM > 0.12);
});

test("hip rotation signs agree with forward flexion and mirrored outward abduction", () => {
  for (const side of ["left", "right"]) {
    const axes = SEGMENT_BY_ID.get(`${side}Thigh`).jointProfile.axes;
    const x = axes.find(axis => axis.coordinate === "x");
    const z = axes.find(axis => axis.coordinate === "z");
    const forwardFlexion = -Math.PI / 18;
    assert.ok(forwardFlexion > x.minRadians && forwardFlexion < x.maxRadians);
    assert.ok(rotate(quatFromAxisAngle({ x: 1, y: 0, z: 0 }, forwardFlexion), { x: 0, y: -1, z: 0 }).z > 0.17);
    const outward = (side === "left" ? -1 : 1) * Math.PI / 18;
    assert.ok(outward > z.minRadians && outward < z.maxRadians);
    const direction = rotate(quatFromAxisAngle({ x: 0, y: 0, z: 1 }, outward), { x: 0, y: -1, z: 0 });
    assert.ok(direction.x * (side === "left" ? -1 : 1) > 0.17);
  }
});

test("committed step is a copied world hindfoot target at flat sole height", () => {
  const poses = composeUprightPose(poseInput()).poses;
  const controller = new BalanceController(); controller.reset(poses);
  controller.beginStep("rightFoot", poses.get("rightFoot").position, poses.get("pelvis").position,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  const input = { dt, poses, rootPosition: poses.get("pelvis").position, activeGrab: null };
  const first = controller.update({ ...input, heading: 0 }).step;
  const saved = structuredClone(first);
  assert.equal(first.to.y, 0.045);
  assert.ok(Math.abs(first.requested.z - 0.035) < 1e-8);
  first.from.x = 999; first.to.x = 999; first.requested.x = 999;
  const next = controller.update({ ...input, heading: 1.4 }).step;
  assert.deepEqual(next.from, saved.from);
  assert.deepEqual(next.to, saved.to);
  assert.deepEqual(next.requested, saved.requested);
  assert.equal(next.heading, 0);
});

test("chain diagnostics reconstruct the exact local commands and reveal floating-root frame error", () => {
  const poses = composeUprightPose(poseInput()).poses;
  const commands = SEGMENTS.filter(d => d.parent).map(d => ({
    id: d.id, targetLocalRotation: quatMultiply(quatInverse(poses.get(d.parent).rotation), poses.get(d.id).rotation),
    stiffness: 1, damping: 1, strengthScale: 1,
  }));
  const same = standingChainDiagnostics(poses, poses, commands, null, 0, 0, dt, []);
  for (const leg of same.legs) for (const segment of leg.segments) assert.ok(segment.commandFrameErrorM < 1e-9);
  const yaw = quatFromAxisAngle(up, 1.1);
  const moved = new Map([...poses].map(([id, pose]) => [id, { ...pose, position: rotate(yaw, pose.position), rotation: quatMultiply(yaw, pose.rotation) }]));
  const changed = standingChainDiagnostics(moved, poses, commands, null, 0, 0, dt, []);
  assert.ok(changed.legs[1].segments.find(s => s.id === "rightFoot").commandFrameErrorM > 0.09);
  assert.equal(changed.motorSampleTimeS, 0);
  assert.equal(changed.physicalSampleTimeS, dt);
});

test("standing contact and chain snapshots are detached and cannot change Rapier motion", async () => {
  const observed = await createEmbodiedCharacter("canvas2d");
  const control = await createEmbodiedCharacter("canvas2d");
  try {
    const start = observed.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand").position;
    const begin = { kind: "begin", pointerId: 7, region: "rightHand", segment: "rightHand", localAnchor: zero, worldTarget: start, timestampMs: 0 };
    observed.fixedUpdate(dt, begin); control.fixedUpdate(dt, begin);
    let sawLoaded = false;
    for (let tick = 0; tick < 25; tick++) {
      const snapshot = observed.getSnapshot("canvas2d");
      const contacts = snapshot.diagnostics.contactDiagnostics.contacts;
      sawLoaded ||= contacts.some(c => c.loadBearing && c.forceN > 0 && (c.points?.length ?? 0) > 0);
      if (contacts.length) {
        contacts[0].point.x = 999;
        if (contacts[0].points?.length) contacts[0].points[0].x = 999;
        contacts[0].forceN = -999;
      }
      const chain = snapshot.diagnostics.standingChain;
      assert.ok(chain);
      chain.pelvis.position.x = 999;
      chain.legs[0].segments[0].desired.position.x = 999;
      const move = { kind: "move", pointerId: 7, worldTarget: { ...start, x: start.x + (tick + 1) * 0.002 }, timestampMs: (tick + 1) * dt * 1000 };
      observed.fixedUpdate(dt, move); control.fixedUpdate(dt, move);
    }
    assert.ok(sawLoaded, "standing contacts include measured loaded sole patches before recovery");
    assert.deepEqual(observed.getSnapshot("canvas2d").segments, control.getSnapshot("canvas2d").segments);
    observed.reset();
    assert.equal(observed.getSnapshot("canvas2d").diagnostics.standingChain, null);
  } finally { observed.dispose(); control.dispose(); }
});
