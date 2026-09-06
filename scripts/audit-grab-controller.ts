import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import RAPIER from "@dimforge/rapier3d-compat";
import { GRAB_CONTROL_LIMITS, GrabAnchorController, type GrabControlDiagnostics } from "../src/character/GrabAnchorController";
import { SEGMENT_BY_ID } from "../src/core/humanoid";
import type { DiagnosticsSnapshot, SegmentDefinition, Vec3 } from "../src/core/types";

const DT = 1 / 60;
// Numerical comparison tolerance only; physical acceptance caps are unchanged.
const ROUNDING_EPSILON = 1e-8;
const tracePath = "evidence/successor-trace.ndjson";
const traceText = readFileSync(tracePath, "utf8");
type TraceRow = { scenario: string; sequence: number; time: number; root: Vec3; diagnostics: DiagnosticsSnapshot };
const rows: TraceRow[] = traceText.trim().split("\n").map((line) => JSON.parse(line));
const magnitude = (v: Vec3): number => Math.hypot(v.x, v.y, v.z);
const difference = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const finiteTree = (value: unknown): boolean => {
  if (typeof value === "number") return Number.isFinite(value);
  if (value !== null && typeof value === "object") return Object.values(value).every(finiteTree);
  return true;
};
const digest = (path: string): string => createHash("sha256").update(readFileSync(path)).digest("hex");
const failures: Array<{ scope: string; detail: string }> = [];
function check(condition: boolean, scope: string, detail: string): void {
  if (!condition) failures.push({ scope, detail });
}
function checkCaps(d: GrabControlDiagnostics, scope: string): void {
  check(finiteTree(d), scope, "nonfinite controller telemetry");
  check(magnitude(d.force) <= GRAB_CONTROL_LIMITS.forceN + ROUNDING_EPSILON, scope, "force exceeds frozen cap");
  check(magnitude(d.torque) <= GRAB_CONTROL_LIMITS.torqueNm + ROUNDING_EPSILON, scope, "torque exceeds frozen cap");
  check(d.linearImpulseLimitNs <= GRAB_CONTROL_LIMITS.forceN * DT + ROUNDING_EPSILON,
    scope, "reported linear impulse cap exceeds frozen 60Hz limit");
  check(d.angularImpulseLimitNms <= GRAB_CONTROL_LIMITS.torqueNm * DT + ROUNDING_EPSILON,
    scope, "reported angular impulse cap exceeds frozen 60Hz limit");
  check(d.positiveWorkLimitJ <= GRAB_CONTROL_LIMITS.positivePowerW * DT + ROUNDING_EPSILON,
    scope, "reported positive work cap exceeds frozen 60Hz limit");
  check(magnitude(d.impulse) <= d.linearImpulseLimitNs + ROUNDING_EPSILON, scope, "linear impulse exceeds frozen cap");
  check(magnitude(d.angularImpulse) <= d.angularImpulseLimitNms + ROUNDING_EPSILON, scope, "angular impulse exceeds frozen cap");
  check(d.injectedWorkJ <= d.positiveWorkLimitJ + ROUNDING_EPSILON, scope, "positive work exceeds frozen cap");
  check(d.targetSpeedMps <= GRAB_CONTROL_LIMITS.targetSpeedMps + ROUNDING_EPSILON, scope, "target speed exceeds frozen cap");
  check(d.effectiveMassKg > 0, scope, "effective mass is not positive");
  check(d.storedUserForceN < 0.01 && d.storedUserTorqueNm < 0.01, scope, "persistent user force/torque exists");
}

let activeFrames = 0;
let accelerationComparisons = 0;
let releasedFrames = 0;
const maxima = {
  forceN: 0, linearImpulseNs: 0, torqueNm: 0, angularImpulseNms: 0,
  positiveWorkJ: 0, targetSpeedMps: 0, targetAccelerationMps2: 0,
  storedUserForceN: 0, storedUserTorqueNm: 0, releasedGrabContributionN: 0,
};
let minimumEffectiveMassKg = Infinity;
let maximumEffectiveMassKg = 0;
for (let i = 0; i < rows.length; i += 1) {
  const row = rows[i];
  const d = row.diagnostics;
  const g = d.grabControl;
  const scope = `${row.scenario}:${row.sequence}`;
  check(finiteTree(row), scope, "nonfinite recorded state");
  check(d.finite && d.errors.length === 0, scope, "application state error");
  maxima.storedUserForceN = Math.max(maxima.storedUserForceN, g.storedUserForceN);
  maxima.storedUserTorqueNm = Math.max(maxima.storedUserTorqueNm, g.storedUserTorqueNm);
  check(g.storedUserForceN < 0.01 && g.storedUserTorqueNm < 0.01, scope, "persistent user force/torque exists");
  if (!d.activeGrab) {
    releasedFrames += 1;
    maxima.releasedGrabContributionN = Math.max(maxima.releasedGrabContributionN, d.appliedGrabForceN);
    check(d.appliedGrabForceN < 0.01 && magnitude(g.impulse) === 0, scope, "grab survives release");
  }
  if (!g.active) continue;
  activeFrames += 1;
  checkCaps(g, scope);
  minimumEffectiveMassKg = Math.min(minimumEffectiveMassKg, g.effectiveMassKg);
  maximumEffectiveMassKg = Math.max(maximumEffectiveMassKg, g.effectiveMassKg);
  maxima.forceN = Math.max(maxima.forceN, magnitude(g.force));
  maxima.linearImpulseNs = Math.max(maxima.linearImpulseNs, magnitude(g.impulse));
  maxima.torqueNm = Math.max(maxima.torqueNm, magnitude(g.torque));
  maxima.angularImpulseNms = Math.max(maxima.angularImpulseNms, magnitude(g.angularImpulse));
  maxima.positiveWorkJ = Math.max(maxima.positiveWorkJ, g.injectedWorkJ);
  maxima.targetSpeedMps = Math.max(maxima.targetSpeedMps, g.targetSpeedMps);
  const prior = rows[i - 1];
  if (prior && prior.scenario === row.scenario && prior.sequence + 1 === row.sequence
      && prior.diagnostics.grabControl.active && prior.diagnostics.activePointerId === d.activePointerId) {
    const dt = row.time - prior.time;
    check(dt > 0, scope, "nonpositive consecutive trace timestep");
    const acceleration = magnitude(difference(g.targetVelocity, prior.diagnostics.grabControl.targetVelocity)) / dt;
    accelerationComparisons += 1;
    maxima.targetAccelerationMps2 = Math.max(maxima.targetAccelerationMps2, acceleration);
    check(acceleration <= GRAB_CONTROL_LIMITS.targetAccelerationMps2 + ROUNDING_EPSILON, scope, "target acceleration exceeds frozen cap");
  }
}

await RAPIER.init();
function collider(definition: SegmentDefinition): RAPIER.ColliderDesc {
  const shape = definition.shape;
  if (shape.kind === "box") return RAPIER.ColliderDesc.cuboid(shape.halfExtents.x, shape.halfExtents.y, shape.halfExtents.z);
  if (shape.kind === "sphere") return RAPIER.ColliderDesc.ball(shape.radius);
  return RAPIER.ColliderDesc.capsule(shape.halfHeight, shape.radius);
}

// Bounded API-level mass/inertia comparison: original colliders, identical
// off-center anchor, target error, timestep and controller, one impulse each.
const localAnchor = { x: 0.04, y: 0.02, z: 0.01 };
const targetError = { x: 0.1, y: 0.02, z: 0 };
const massProbes = (["rightHand", "pelvis"] as const).map((id) => {
  const definition = SEGMENT_BY_ID.get(id)!;
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  world.timestep = DT;
  const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
  world.createCollider(collider(definition).setMass(definition.massKg), body);
  body.recomputeMassPropertiesFromColliders();
  const target = add(localAnchor, targetError);
  const controller = new GrabAnchorController(target);
  const diagnostic = controller.apply(body, localAnchor, target, DT);
  checkCaps(diagnostic, `isolated-mass:${id}`);
  const actualMassKg = body.mass();
  const postImpulseLinearVelocity = { ...body.linvel() };
  const postImpulseAngularVelocity = { ...body.angvel() };
  const inertia = body.effectiveAngularInertia();
  const v = postImpulseLinearVelocity;
  const w = postImpulseAngularVelocity;
  const measuredKineticEnergyJ = 0.5 * actualMassKg * magnitude(v) ** 2 + 0.5 * (
    w.x * (inertia.m11 * w.x + inertia.m12 * w.y + inertia.m13 * w.z)
    + w.y * (inertia.m21 * w.x + inertia.m22 * w.y + inertia.m23 * w.z)
    + w.z * (inertia.m31 * w.x + inertia.m32 * w.y + inertia.m33 * w.z)
  );
  // Rapier velocity readback is Float32; compare physical energy at Float32
  // precision separately from exact JavaScript controller-cap comparisons.
  check(Math.abs(measuredKineticEnergyJ - diagnostic.injectedWorkJ) < 1e-6,
    `isolated-mass:${id}`, "reported work disagrees with actual Rapier kinetic energy change");
  check(measuredKineticEnergyJ <= GRAB_CONTROL_LIMITS.positivePowerW * DT + 1e-6,
    `isolated-mass:${id}`, "actual Rapier kinetic energy change exceeds work cap");
  world.step();
  check(finiteTree([body.translation(), body.rotation(), body.linvel(), body.angvel()]), `isolated-mass:${id}`, "nonfinite physical state");
  const result = {
    segment: id, definedMassKg: definition.massKg, actualMassKg,
    localAnchor, targetError, dt: DT,
    effectiveMassKg: diagnostic.effectiveMassKg,
    impulseNs: magnitude(diagnostic.impulse), forceN: magnitude(diagnostic.force),
    angularImpulseNms: magnitude(diagnostic.angularImpulse),
    positiveWorkJ: diagnostic.injectedWorkJ,
    measuredKineticEnergyJ,
    postImpulseLinearVelocity, postImpulseAngularVelocity,
    diagnostic,
  };
  world.free();
  return result;
});
check(massProbes[1].effectiveMassKg > massProbes[0].effectiveMassKg, "isolated-mass", "pelvis effective mass did not exceed hand effective mass");
check(massProbes[1].impulseNs > massProbes[0].impulseNs, "isolated-mass", "same target error incorrectly produced identical hand/pelvis impulse");

// Retained API-only proof of the old cause; this is not an unchanged failed
// humanoid replay and does not modify or retune the accepted controller.
const proofWorld = new RAPIER.World({ x: 0, y: 0, z: 0 });
proofWorld.timestep = DT;
const proofBody = proofWorld.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
proofWorld.createCollider(RAPIER.ColliderDesc.ball(0.1).setMass(1), proofBody);
const unitForce = { x: 1, y: 0, z: 0 };
const origin = { x: 0, y: 0, z: 0 };
proofBody.addForceAtPoint(unitForce, origin, true);
proofWorld.step();
const firstAddition = { userForceN: { ...proofBody.userForce() }, velocityMps: { ...proofBody.linvel() } };
proofBody.addForceAtPoint(unitForce, origin, true);
proofWorld.step();
const secondAddition = { userForceN: { ...proofBody.userForce() }, velocityMps: { ...proofBody.linvel() } };
proofWorld.step();
const withoutAnotherContribution = { userForceN: { ...proofBody.userForce() }, velocityMps: { ...proofBody.linvel() } };
check(firstAddition.userForceN.x === 1 && secondAddition.userForceN.x === 2
  && withoutAnotherContribution.userForceN.x === 2, "old-force-API-proof", "force persistence differs from diagnosis");
proofWorld.free();

const result = {
  schema: 1,
  generatedAt: new Date().toISOString(),
  engine: `Rapier ${RAPIER.version()}`,
  scope: "Retained corrected humanoid trace audit plus bounded isolated API probes; not browser-input evidence",
  trace: { path: tracePath, sha256: digest(tracePath), frames: rows.length, activeFrames, releasedFrames, accelerationComparisons },
  sourceSha256: {
    controller: digest("src/character/GrabAnchorController.ts"),
    character: digest("src/character/index.ts"),
    humanoid: digest("src/core/humanoid.ts"),
  },
  frozenLimits: GRAB_CONTROL_LIMITS,
  roundingTolerance: ROUNDING_EPSILON,
  traceMaxima: maxima,
  traceEffectiveMassRangeKg: [minimumEffectiveMassKg, maximumEffectiveMassKg],
  isolatedMassProbes: massProbes,
  oldPersistentForceCauseProof: { dt: DT, massKg: 1, firstAddition, secondAddition, withoutAnotherContribution },
  passed: failures.length === 0,
  failures,
};
writeFileSync("evidence/grab-controller-audit.json", `${JSON.stringify(result, null, 2)}\n`);
console.log(JSON.stringify(result, null, 2));
process.exitCode = failures.length ? 1 : 0;
