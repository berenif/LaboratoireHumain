import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { reachableFootTarget } = await import("../src/character/recovery-foot-targets.ts");
const { selectRecoveryRoute } = await import("../src/character/recovery-support.ts");
const { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { SEGMENT_BY_ID, HUMAN_PROPORTIONS } = await import("../src/core/humanoid.ts");
const { add, sub, length, rotate, quatMultiply, quatFromAxisAngle, worldPoint } = await import("../src/character/math.ts");
const near = (actual, expected, message, tolerance = 1e-8) => assert.ok(Math.abs(actual - expected) < tolerance, message + `: ${actual} vs ${expected}`);
const samePoint = (actual, expected, message) => near(length(sub(actual, expected)), 0, message);
const UP = { x: 0, y: 1, z: 0 };

function verifyLegalGeometry(poses, side, target) {
  const pelvis = poses.get("pelvis"), td = SEGMENT_BY_ID.get(`${side}Thigh`), sd = SEGMENT_BY_ID.get(`${side}Shin`), fd = SEGMENT_BY_ID.get(`${side}Foot`);
  const thigh = quatMultiply(pelvis.rotation, target.jointRotations.thigh), shin = quatMultiply(thigh, target.jointRotations.shin), foot = quatMultiply(shin, target.jointRotations.foot);
  const hip = worldPoint(pelvis.position, pelvis.rotation, td.jointAnchorParent);
  const knee = add(hip, rotate(thigh, sub(sd.jointAnchorParent, td.jointAnchorChild)));
  const ankle = add(knee, rotate(shin, sub(fd.jointAnchorParent, sd.jointAnchorChild)));
  samePoint(target.hip, hip, "measured hip anchor"); samePoint(target.knee, knee, "legal hip rotation produces knee"); samePoint(target.ankle, ankle, "legal knee rotation produces ankle");
  samePoint(worldPoint(target.position, target.rotation, fd.jointAnchorChild), ankle, "true foot ankle offset stays connected");
  samePoint(rotate(target.rotation, { x: 1, y: 0, z: 0 }), rotate(foot, { x: 1, y: 0, z: 0 }), "foot orientation is relative to the solved shin");
  near(length(sub(knee, hip)), 0.42, "thigh length"); near(length(sub(ankle, knee)), 0.40, "shin length");
  const kneeAngle = 2 * Math.atan2(target.jointRotations.shin.x, target.jointRotations.shin.w);
  assert.ok(kneeAngle >= 0.025 - 1e-9 && kneeAngle <= sd.jointLimitRadians.x + 1e-9, "natural knee bend stays inside actual hinge range");
  for (const [q, limit] of [[target.jointRotations.thigh, td.jointLimitRadians], [target.jointRotations.foot, fd.jointLimitRadians]]) {
    const angles = { x: Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.x - q.y * q.z)))), y: Math.atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.x * q.x + q.y * q.y)), z: Math.atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)) };
    for (const axis of ["x", "y", "z"]) assert.ok(Math.abs(angles[axis]) <= limit[axis] + 1e-8, "reconstructed local joint angle exceeds " + axis);
  }
  near(target.travelM, length(sub(target.position, poses.get(`${side}Foot`).position)), "travel is measured from the actual foot centre");
  if (target.feasible) {
    near(target.position.y, 0.045, "feasible flat sole center uses its actual box half height");
    near(rotate(target.rotation, UP).y, 1, "feasible support is a flat sole");
    const half = HUMAN_PROPORTIONS.foot.halfExtentsM;
    for (const x of [-half.x, half.x]) for (const y of [-half.y, half.y]) for (const z of [-half.z, half.z]) assert.ok(worldPoint(target.position, target.rotation, { x, y, z }).y >= -1e-8, "actual foot box penetrates the floor");
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

test("already reachable crouch soles choose the exact nearest floor projection", () => {
  for (const fixture of RECOVERY_POSE_FIXTURES.filter(f => f.pose === "crouch")) {
    const poses = recoveryFixturePoses(fixture);
    for (const side of ["left", "right"]) {
      const actual = poses.get(`${side}Foot`).position, target = reachableFootTarget(side, poses, fixture.heading);
      assert.equal(target.feasible, true);
      samePoint(target.position, { ...actual, y: 0.045 }, "nearest reachable floor projection");
      near(target.travelM, Math.abs(actual.y - 0.045), "minimum possible centre movement to floor");
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
    assert.equal(nearest.leadingSide, fixture.side, "true nearest reachable placement chooses the leading side");
    const leadingFoot = `${fixture.side}Foot`, measured = poses.get(leadingFoot).position;
    const planted = { segment: leadingFoot, normalY: 1, forceN: 350, persistenceS: 1, loadBearing: true, point: { ...measured, y: 0 } };
    // Even a newly preferable alternative cannot displace established leading support.
    const alternative = fixture.side === "left" ? "right" : "left";
    footTargets[alternative] = { ...poses.get(`${alternative}Foot`).position };
    assert.equal(selectRecoveryRoute({ poses, contacts: [planted], footTargets }).leadingSide, fixture.side);
  }
});
