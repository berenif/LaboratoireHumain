import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { reachableFootTarget } = await import("../src/character/recovery-foot-targets.ts");
const { selectRecoveryRoute } = await import("../src/character/recovery-support.ts");
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { add, sub, length, rotate, quatMultiply, quatFromAxisAngle, worldPoint } = await import("../src/character/math.ts");
const { jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");
const near = (actual, expected, message, tolerance = 1e-8) => assert.ok(Math.abs(actual - expected) < tolerance, message + `: ${actual} vs ${expected}`);
const samePoint = (actual, expected, message) => near(length(sub(actual, expected)), 0, message);
const UP = { x: 0, y: 1, z: 0 };
const IDENTITY = { x: 0, y: 0, z: 0, w: 1 };

function verifyLegalGeometry(poses, side, target) {
  const pelvis = poses.get("pelvis"), td = SEGMENT_BY_ID.get(`${side}Thigh`), sd = SEGMENT_BY_ID.get(`${side}Shin`);
  const ad = SEGMENT_BY_ID.get(`${side}Ankle`), fd = SEGMENT_BY_ID.get(`${side}Foot`), ffd = SEGMENT_BY_ID.get(`${side}Forefoot`);
  const thigh = quatMultiply(pelvis.rotation, target.jointRotations.thigh);
  const shin = quatMultiply(thigh, target.jointRotations.shin);
  const ankleRotation = quatMultiply(shin, target.jointRotations.ankle);
  const foot = quatMultiply(ankleRotation, target.jointRotations.foot);
  const forefoot = quatMultiply(foot, target.jointRotations.forefoot);
  const hip = worldPoint(pelvis.position, pelvis.rotation, td.jointAnchorParent);
  const knee = add(hip, rotate(thigh, sub(sd.jointAnchorParent, td.jointAnchorChild)));
  const ankle = add(knee, rotate(shin, sub(ad.jointAnchorParent, sd.jointAnchorChild)));
  samePoint(target.hip, hip, "measured hip anchor"); samePoint(target.knee, knee, "legal hip rotation produces knee"); samePoint(target.ankle, ankle, "legal knee rotation produces ankle");
  samePoint(worldPoint(target.position, target.rotation, fd.jointAnchorChild), ankle, "true foot ankle offset stays connected");
  samePoint(rotate(target.rotation, { x: 1, y: 0, z: 0 }), rotate(foot, { x: 1, y: 0, z: 0 }), "foot orientation is relative to the solved shin");
  near(length(sub(knee, hip)), 0.42, "thigh length"); near(length(sub(ankle, knee)), 0.40, "shin length");
  for (const [definition, rotation] of [[td, target.jointRotations.thigh], [sd, target.jointRotations.shin],
    [ad, target.jointRotations.ankle], [fd, target.jointRotations.foot], [ffd, target.jointRotations.forefoot]]) {
    const coordinates = jointCoordinates(IDENTITY, rotation, definition.jointProfile);
    assert.ok(jointLimitErrorMagnitude(coordinates, definition.jointProfile) < 1e-8,
      `${definition.id} reconstructed target exceeds its shared joint profile: ${JSON.stringify(coordinates)}`);
  }
  const kneeAngle = jointCoordinates(IDENTITY, target.jointRotations.shin, sd.jointProfile).x;
  const kneeLimit = sd.jointProfile.axes.find(({ coordinate }) => coordinate === "x");
  assert.ok(kneeAngle >= kneeLimit.minRadians - 1e-9 && kneeAngle <= kneeLimit.maxRadians + 1e-9, "natural knee bend stays inside actual hinge range");
  near(target.travelM, length(sub(target.position, poses.get(`${side}Foot`).position)), "travel is measured from the actual foot centre");
  if (target.feasible) {
    near(target.position.y, -fd.geometry.localBounds.min.y, "feasible flat sole center uses the exact hindfoot surface");
    near(rotate(target.rotation, UP).y, 1, "feasible support is a flat sole", 1e-6);
    for (const vertex of fd.geometry.vertices) assert.ok(worldPoint(target.position, target.rotation, vertex).y >= -0.003, "exact hindfoot surface exceeds the floor tolerance");
    const forefootPosition = sub(
      worldPoint(target.position, target.rotation, ffd.jointAnchorParent),
      rotate(forefoot, ffd.jointAnchorChild),
    );
    for (const vertex of ffd.geometry.vertices) assert.ok(worldPoint(forefootPosition, forefoot, vertex).y >= -0.003, "exact forefoot surface exceeds the floor tolerance");
  }
}

test("foot placements preserve actual bone lengths, ankle offsets and legal joint rotations for every landed fixture", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES) {
    const poses = recoveryFixturePoses(fixture);
    for (const side of ["left", "right"]) {
      const target = reachableFootTarget(side, poses, fixture.heading);
      assert.ok(target); verifyLegalGeometry(poses, side, target);
      assert.ok(Number.isFinite(target.reachErrorM) && Number.isFinite(target.floorClearanceM));
    }
  }
});

test("reachable foot selection mirrors and rotates with the entire measured pose", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.side === "left" && f.heading === 0)) {
    const mirror = { ...fixture, side: "right" }, turned = { ...fixture, heading: Math.PI / 3 };
    const base = recoveryFixturePoses(fixture), reflected = recoveryFixturePoses(mirror), rotated = recoveryFixturePoses(turned), yaw = quatFromAxisAngle(UP, turned.heading);
    for (const side of ["left", "right"]) {
      const a = reachableFootTarget(side, base, 0), b = reachableFootTarget(side === "left" ? "right" : "left", reflected, 0), c = reachableFootTarget(side, rotated, turned.heading);
      for (const field of ["position", "hip", "knee", "ankle"]) {
        samePoint(b[field], { ...a[field], x: -a[field].x }, "mirrored " + field);
        samePoint(c[field], rotate(yaw, a[field]), "rotated " + field);
      }
      near(a.travelM, b.travelM, "mirrored travel"); near(a.travelM, c.travelM, "rotated travel");
      assert.equal(a.feasible, b.feasible); assert.equal(a.feasible, c.feasible);
    }
  }
});

test("crouch soles choose a nearby structurally legal floor projection", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.pose === "crouch")) {
    const poses = recoveryFixturePoses(fixture);
    for (const side of ["left", "right"]) {
      const actual = poses.get(`${side}Foot`).position, target = reachableFootTarget(side, poses, fixture.heading);
      assert.equal(target.feasible, true);
      const floorCenter = -SEGMENT_BY_ID.get(`${side}Foot`).geometry.localBounds.min.y;
      near(target.position.y, floorCenter, "reachable hindfoot rests on its exact floor surface", 1e-7);
      assert.ok(target.travelM < 0.1, `nearby crouch plant moved ${target.travelM} m`);
      near(target.travelM, length(sub(target.position, actual)), "travel uses the measured hindfoot centre");
    }
  }
});

test("an unreachable floor returns a legal clamped endpoint and cannot claim feasible support", () => {
  const fixture = RECOVERY_POSE_FIXTURES.find(f => f.pose === "crouch"), poses = recoveryFixturePoses(fixture);
  for (const pose of poses.values()) pose.position.y += 1.5;
  for (const side of ["left", "right"]) {
    const target = reachableFootTarget(side, poses, 0);
    assert.equal(target.feasible, false); assert.equal(target.floorReachable, false);
    assert.ok(target.position.y > 1 && target.reachErrorM > 1, "the impossible floor target must not survive solver clamping");
    assert.ok(length(sub(target.ankle, target.hip)) <= 0.818 + 1e-8);
    verifyLegalGeometry(poses, side, target);
  }
});

test("route selection prefers planted contact, then actual reachable travel across mirrored headings", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.pose === "half-kneel" && f.support !== "weak")) {
    const poses = recoveryFixturePoses(fixture), targets = Object.fromEntries(["left", "right"].map(side => [side, reachableFootTarget(side, poses, fixture.heading)]));
    const footTargets = { left: targets.left.position, right: targets.right.position };
    const nearest = selectRecoveryRoute({ poses, contacts: [], footTargets });
    const nearestSide = targets.right.travelM < targets.left.travelM - 1e-8 ? "right" : "left";
    assert.equal(nearest.leadingSide, nearestSide, "least measured legal travel chooses the leading side");
    const leadingFoot = `${fixture.side}Foot`, measured = poses.get(leadingFoot).position;
    const planted = { segment: leadingFoot, normalY: 1, forceN: 350, persistenceS: 1, loadBearing: true, point: { ...measured, y: 0 } };
    // Even a newly preferable alternative cannot displace established leading support.
    const alternative = fixture.side === "left" ? "right" : "left";
    footTargets[alternative] = { ...poses.get(`${alternative}Foot`).position };
    assert.equal(selectRecoveryRoute({ poses, contacts: [planted], footTargets }).leadingSide, fixture.side);
  }
});

test("foot plans distinguish floor plants, intermediate folds and blocked clearance", async () => {
  const { revalidateRecoveryFootTarget } = await import("../src/character/recovery-foot-targets.ts");
  const fixture = RECOVERY_POSE_FIXTURES.find(f => f.pose === "crouch");
  const poses = recoveryFixturePoses(fixture);
  const plan = reachableFootTarget("left", poses, fixture.heading);
  assert.equal(plan.kind, "floor-plant");
  assert.equal(revalidateRecoveryFootTarget("left", poses, plan), true);
  poses.get("pelvis").position.y += .04;
  assert.equal(revalidateRecoveryFootTarget("left", poses, plan), false, "a moving parent invalidates the captured floor solution");
  for (const pose of poses.values()) pose.position.y += 1;
  const folded = reachableFootTarget("left", poses, fixture.heading);
  assert.equal(folded.feasible, false);
  assert.equal(folded.kind, "intermediate-fold");
  assert.ok(folded.position.y > 0.8, "a legal intermediate cannot be rewritten to a floor command");
  const invalid = { ...folded, kind: "blocked" };
  assert.equal(revalidateRecoveryFootTarget("left", poses, invalid), false);
});
