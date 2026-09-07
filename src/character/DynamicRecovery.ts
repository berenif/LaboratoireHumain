import type { Collider, RigidBody, World } from "@dimforge/rapier3d-compat";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } from "../core/humanoid";
import type { MotionState, Quat, RecoveryDiagnostics, RecoveryPhase, SegmentId, SupportingContact, Vec3 } from "../core/types";
import { add, angularVelocity, clamp, clampLength, length, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const KNEEL_PELVIS_HEIGHT_M = 0.72;
/** Frozen acceptance parameters. Persistence always requires consecutive qualifying frames. */
export const RECOVERY_LIMITS = Object.freeze({
  normalY: 0.65, contactDistanceM: 0.012, minimumLoadN: 3,
  loadPersistenceS: 0.05, landingPersistenceS: 0.10, settlePersistenceS: 0.30,
  settleLinearMps: 0.65, settleAngularRadps: 1.8,
  stablePersistenceS: 0.55, stableLinearMps: 0.22, stableAngularRadps: 0.65,
  stableUpDot: 0.97, phaseMinimumS: 0.20, stallS: 3.0, supportLossS: 0.20,
  assistanceForceN: 950, assistanceTorqueNm: 300,
});

const ELIGIBLE: Record<RecoveryPhase, ReadonlyArray<SegmentId>> = {
  none: [], protect: [], settle: [],
  roll: ["torso", "pelvis", "leftForearm", "rightForearm", "leftUpperArm", "rightUpperArm", "leftThigh", "rightThigh", "leftShin", "rightShin", "leftHand", "rightHand", "leftFoot", "rightFoot"],
  brace: ["leftHand", "rightHand", "leftForearm", "rightForearm", "leftShin", "rightShin", "leftFoot", "rightFoot"],
  kneel: ["leftShin", "rightShin", "leftFoot", "rightFoot", "leftHand", "rightHand"],
  stand: ["leftFoot", "rightFoot"],
};

function pitch(angle: number): Quat { return quatFromAxisAngle({ x: 1, y: 0, z: 0 }, angle); }
function rotation(x = 0, y = 0, z = 0): Quat {
  return quatMultiply(quatMultiply(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, y), pitch(x)), quatFromAxisAngle({ x: 0, y: 0, z: 1 }, z));
}

export function emptyRecoveryDiagnostics(): RecoveryDiagnostics {
  return { phase: "none", orientation: "forward", contacts: [], phaseTimeS: 0, settledTimeS: 0,
    stableTimeS: 0, stalledTimeS: 0, retries: 0, assistanceForce: ZERO, assistanceTorque: ZERO,
    assistanceForceCapN: RECOVERY_LIMITS.assistanceForceN, assistanceTorqueCapNm: RECOVERY_LIMITS.assistanceTorqueNm,
    maxMotorTorqueNm: 0, supporting: [] };
}

// Implicit PD using Rapier's world inertia tensor avoids explicit damping oscillations
// on light wrists/ankles and leaves the final torque subject to its physical cap.
function implicitTorque(child: RigidBody, parent: RigidBody | null, error: Vec3, velocity: Vec3, kp: number, kd: number, dt: number): Vec3 {
  const raw = child.effectiveWorldInvInertia();
  // Rapier 0.20 defaults alias a shared WASM scratch buffer; copy before the next query.
  const c = { m11: raw.m11, m12: raw.m12, m13: raw.m13, m22: raw.m22, m23: raw.m23, m33: raw.m33 };
  const p = parent?.effectiveWorldInvInertia();
  const k = kd * dt + kp * dt * dt;
  const a = 1 + k * (c.m11 + (p?.m11 ?? 0)), b = k * (c.m12 + (p?.m12 ?? 0)), d = k * (c.m13 + (p?.m13 ?? 0));
  const e = 1 + k * (c.m22 + (p?.m22 ?? 0)), f = k * (c.m23 + (p?.m23 ?? 0)), g = 1 + k * (c.m33 + (p?.m33 ?? 0));
  const A = e*g-f*f, B = d*f-b*g, C = b*f-d*e, E = a*g-d*d, F = b*d-a*f, G = a*e-b*b;
  const determinant = a*A + b*B + d*C;
  const v = sub(scale(error, kp), scale(velocity, kd));
  return { x: (A*v.x+B*v.y+C*v.z)/determinant, y: (B*v.x+E*v.y+F*v.z)/determinant, z: (C*v.x+F*v.y+G*v.z)/determinant };
}

/** Dynamic muscles: equal/opposite joint torque impulses; only the supported pelvis gets external aid. */
export class DynamicRecovery {
  private data = emptyRecoveryDiagnostics();
  private heading = 0;
  private contactAges = new Map<SegmentId, number>();
  private landingTime = 0;
  private noSupportTime = 0;
  private bestError = Infinity;
  private targetOrigin: Vec3 = ZERO;
  private rise = 0;
  private rollHeight = 0;
  private bestStableTime = 0;

  reset(heading: number, direction: Vec3): void {
    this.data = emptyRecoveryDiagnostics();
    this.data.phase = "protect";
    this.heading = heading;
    const local = rotate(quatInverse(quatFromAxisAngle(UP, heading)), direction);
    this.data.orientation = Math.abs(local.x) > Math.abs(local.z) ? (local.x < 0 ? "left" : "right") : (local.z < 0 ? "backward" : "forward");
    this.contactAges.clear(); this.landingTime = 0; this.noSupportTime = 0;
    this.bestError = Infinity; this.rise = 0; this.bestStableTime = 0;
  }

  diagnostics(): RecoveryDiagnostics {
    return { ...this.data, contacts: this.data.contacts.map(c => ({ ...c, point: { ...c.point } })),
      supporting: [...this.data.supporting], assistanceForce: { ...this.data.assistanceForce }, assistanceTorque: { ...this.data.assistanceTorque } };
  }

  /** Called immediately after every integration; contact geometry and solved impulse must both qualify. */
  observe(world: World, floor: Collider, colliders: Map<SegmentId, Collider>, bodies: Map<SegmentId, RigidBody>, dt: number): void {
    const contacts: SupportingContact[] = [];
    for (const [segment, collider] of colliders) {
      let normalY = 0, impulse = 0, point = ZERO, touching = false;
      world.contactPair(floor, collider, (manifold, flipped) => {
        const upward = manifold.normal().y * (flipped ? -1 : 1);
        for (let index = 0; index < manifold.numSolverContacts(); index++) {
          if (manifold.solverContactDist(index) <= RECOVERY_LIMITS.contactDistanceM && upward >= RECOVERY_LIMITS.normalY) {
            touching = true; normalY = Math.max(normalY, upward);
            point = manifold.solverContactPoint(index) ?? point;
          }
        }
        if (touching) for (let index = 0; index < manifold.numContacts(); index++) impulse += Math.max(0, manifold.contactImpulse(index));
      });
      // Sleeping is disabled on the active ragdoll: measured solver load remains available.
      const forceN = impulse / dt;
      const loaded = touching && forceN >= RECOVERY_LIMITS.minimumLoadN;
      const age = loaded ? (this.contactAges.get(segment) ?? 0) + dt : 0;
      this.contactAges.set(segment, age);
      if (touching) contacts.push({ segment, normalY, forceN, persistenceS: age, point,
        loadBearing: loaded && age + 1e-9 >= RECOVERY_LIMITS.loadPersistenceS });
    }
    this.data.contacts = contacts;
    const contact = contacts.some(c => c.forceN >= RECOVERY_LIMITS.minimumLoadN);
    const landedContact = contacts.some(c => !c.segment.endsWith("Foot") && c.forceN >= RECOVERY_LIMITS.minimumLoadN);
    this.landingTime = landedContact ? this.landingTime + dt : 0;
    const motion = this.motion(bodies);
    const settled = contact && motion.linear <= RECOVERY_LIMITS.settleLinearMps && motion.angular <= RECOVERY_LIMITS.settleAngularRadps;
    this.data.settledTimeS = settled ? this.data.settledTimeS + dt : 0;
    this.data.supporting = this.supporting().map(c => c.segment);
    // Diagnostic output must stop showing aid on the very frame that support disappears.
    if (this.data.supporting.length === 0) { this.data.assistanceForce = ZERO; this.data.assistanceTorque = ZERO; }
  }

  private supporting(): SupportingContact[] {
    return this.data.contacts.filter(c => c.loadBearing && ELIGIBLE[this.data.phase].includes(c.segment));
  }
  private motion(bodies: Map<SegmentId, RigidBody>): { linear: number; angular: number } {
    let linear = 0, angular = 0;
    for (const definition of SEGMENTS) {
      const body = bodies.get(definition.id)!;
      linear += definition.massKg * length(body.linvel()) ** 2;
      angular += definition.massKg * length(body.angvel()) ** 2;
    }
    return { linear: Math.sqrt(linear / TOTAL_MASS_KG), angular: Math.sqrt(angular / TOTAL_MASS_KG) };
  }

  private enter(phase: RecoveryPhase, bodies: Map<SegmentId, RigidBody>): void {
    this.data.phase = phase; this.data.phaseTimeS = 0; this.data.stalledTimeS = 0;
    this.data.stableTimeS = 0; this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0;
    if (phase === "roll") {
      const p = bodies.get("pelvis")!.translation();
      this.targetOrigin = { x: p.x, y: 0, z: p.z };
      this.rise = 0;
      this.rollHeight = p.y;
    }
  }

  /** Prepare bounded impulses, then caller integrates exactly once. Returns whether stable transfer is permitted. */
  apply(bodies: Map<SegmentId, RigidBody>, dt: number): { state: MotionState; recovered: boolean } {
    this.data.phaseTimeS += dt;
    this.data.assistanceForce = ZERO; this.data.assistanceTorque = ZERO; this.data.maxMotorTorqueNm = 0;
    const pelvis = bodies.get("pelvis")!, torso = bodies.get("torso")!;
    const motion = this.motion(bodies);
    const up = rotate(torso.rotation(), UP).y;
    const feet = this.data.contacts.filter(c => c.loadBearing && (c.segment === "leftFoot" || c.segment === "rightFoot"));
    const shins = this.data.contacts.filter(c => c.loadBearing && (c.segment === "leftShin" || c.segment === "rightShin"));
    const hands = this.data.contacts.filter(c => c.loadBearing && /Hand|Forearm/.test(c.segment));
    const ready = this.data.phaseTimeS >= RECOVERY_LIMITS.phaseMinimumS;
    if (this.data.phase === "protect" && this.landingTime >= RECOVERY_LIMITS.landingPersistenceS) this.enter("settle", bodies);
    else if (this.data.phase === "settle" && this.data.settledTimeS >= RECOVERY_LIMITS.settlePersistenceS) {
      // Orientation is selected from the landed torso, retaining heading as the reference frame.
      const forward = rotate(torso.rotation(), { x: 0, y: 0, z: 1 });
      const right = rotate(torso.rotation(), { x: 1, y: 0, z: 0 });
      this.data.orientation = Math.abs(right.y) > Math.abs(forward.y) ? (right.y > 0 ? "left" : "right") : (forward.y > 0 ? "backward" : "forward");
      this.enter("roll", bodies);
    }
    else if (this.data.phase === "roll" && ready && (hands.length > 0 || shins.length > 0 || feet.length > 0) && up > 0.25 && pelvis.translation().y > 0.28) this.enter("brace", bodies);
    else if (this.data.phase === "brace" && ready && (shins.length > 0 || feet.length > 0) && up > 0.65 && pelvis.translation().y > 0.38) this.enter("kneel", bodies);
    else if (this.data.phase === "kneel" && ready && feet.length === 2 && up > 0.88 && pelvis.translation().y > 0.55) this.enter("stand", bodies);

    const recovering = ["roll", "brace", "kneel", "stand"].includes(this.data.phase);
    const supports = this.supporting();
    this.data.supporting = supports.map(c => c.segment);
    if (recovering) {
      this.noSupportTime = supports.length ? 0 : this.noSupportTime + dt;
      const desiredHeight = this.data.phase === "roll" ? 0.43 : this.data.phase === "brace" ? 0.57 : this.data.phase === "kneel" ? KNEEL_PELVIS_HEIGHT_M : HUMAN_PROPORTIONS.pelvis.centerHeightM;
      const error = Math.abs(desiredHeight - pelvis.translation().y) + Math.max(0, 1 - up) * 0.4;
      const stabilityProgress = this.data.phase === "stand" && this.data.stableTimeS > this.bestStableTime + 1e-9;
      this.bestStableTime = Math.max(this.bestStableTime, this.data.stableTimeS);
      if (stabilityProgress || error < this.bestError - 0.015) { this.bestError = error; this.data.stalledTimeS = 0; }
      else this.data.stalledTimeS += dt;
      if (this.noSupportTime > RECOVERY_LIMITS.supportLossS || this.data.stalledTimeS > RECOVERY_LIMITS.stallS) {
        this.data.retries++; this.enter("settle", bodies); this.data.settledTimeS = 0;
      }
    }

    const phase = this.data.phase;
    // Decide transfer from the last integrated poses and velocities before injecting new momentum.
    const stable = phase === "stand" && feet.length === 2 && feet.every(c => rotate(bodies.get(c.segment)!.rotation(), UP).y > 0.97)
      && up >= RECOVERY_LIMITS.stableUpDot && rotate(pelvis.rotation(), UP).y >= RECOVERY_LIMITS.stableUpDot
      && pelvis.translation().y > 0.93 && motion.linear <= RECOVERY_LIMITS.stableLinearMps && motion.angular <= RECOVERY_LIMITS.stableAngularRadps;
    this.data.stableTimeS = stable ? this.data.stableTimeS + dt : 0;
    if (this.data.stableTimeS >= RECOVERY_LIMITS.stablePersistenceS) return { state: "recovering", recovered: true };
    const recoveryActive = ["roll", "brace", "kneel", "stand"].includes(phase);
    // A prone trunk can rise using a braced arm and a loaded leg even before it points upward.
    const bracedProne = hands.length > 0 && supports.some(c => /Thigh|Shin|Foot/.test(c.segment));
    if (phase === "roll" && supports.length && (up > 0.25 || bracedProne)) {
      this.rollHeight = Math.min(0.43, Math.max(this.rollHeight, pelvis.translation().y) + dt * 0.22);
    }
    if (phase === "stand" && supports.length) this.rise = Math.min(1, this.rise + dt * 0.6);
    const rollPitch = this.data.orientation === "backward" ? -0.50 : this.data.orientation === "forward" ? -0.30 : -0.45;
    const targetPelvisPitch = phase === "roll" ? rollPitch : phase === "brace" ? -0.10 : phase === "kneel" ? -0.05 : 0;
    const targetPelvisRotation = quatMultiply(quatFromAxisAngle(UP, this.heading), pitch(targetPelvisPitch));
    const targets = this.jointTargets(phase);
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const child = bodies.get(definition.id)!, parent = bodies.get(definition.parent)!;
      const angles = targets.get(definition.id) ?? ZERO;
      const limit = definition.jointLimitRadians!;
      const target = rotation(clamp(angles.x, -limit.x, limit.x), clamp(angles.y, -limit.y, limit.y), clamp(angles.z, -limit.z, limit.z));
      const relative = quatMultiply(quatInverse(parent.rotation()), child.rotation());
      const error = rotate(parent.rotation(), angularVelocity(relative, target, 1));
      const relativeVelocity = sub(child.angvel(), parent.angvel());
      const large = /Thigh|Shin|torso/.test(definition.id);
      const cap = definition.id === "torso" ? 110 : /Thigh/.test(definition.id) ? 110 : /Shin/.test(definition.id) ? 110 : /UpperArm/.test(definition.id) ? 30 : /Forearm/.test(definition.id) ? 20 : /Foot/.test(definition.id) ? 65 : /neck|head/.test(definition.id) ? 22 : 9;
      const kp = recoveryActive ? (definition.id === "torso" ? 16000 : large || /Foot/.test(definition.id) ? 3000 : /neck|head/.test(definition.id) ? 2000 : 100) : (large ? 35 : 12);
      const kd = recoveryActive ? (large ? 60 : /Foot/.test(definition.id) ? 20 : /neck|head/.test(definition.id) ? 10 : 5) : (large ? 6 : 1.6);
      const torque = clampLength(implicitTorque(child, parent, error, relativeVelocity, kp, kd, dt), cap);
      child.applyTorqueImpulse(scale(torque, dt), true);
      parent.applyTorqueImpulse(scale(torque, -dt), true);
      this.data.maxMotorTorqueNm = Math.max(this.data.maxMotorTorqueNm, length(torque));
    }
    const qualified = this.supporting();
    if (recoveryActive && qualified.length > 0) {
      const supportCenter = scale(qualified.reduce((sum, c) => add(sum, c.point), ZERO), 1 / qualified.length);
      const ankleCenter = feet.length ? scale(feet.reduce((sum, contact) => {
        const foot = bodies.get(contact.segment)!;
        const anchor = SEGMENT_BY_ID.get(contact.segment)?.jointAnchorChild;
        return anchor ? add(sum, worldPoint(foot.translation(), foot.rotation(), anchor)) : sum;
      }, ZERO), 1 / feet.length) : supportCenter;
      const y = phase === "roll" ? this.rollHeight : phase === "brace" ? 0.57 : phase === "kneel" ? KNEEL_PELVIS_HEIGHT_M
        : KNEEL_PELVIS_HEIGHT_M + (HUMAN_PROPORTIONS.pelvis.centerHeightM - KNEEL_PELVIS_HEIGHT_M) * this.rise;
      const center = phase === "stand" || phase === "kneel" ? ankleCenter : this.targetOrigin;
      const target = { x: center.x, y, z: center.z };
      const error = sub(target, pelvis.translation());
      const velocity = pelvis.linvel();
      const force: { x: number; y: number; z: number } = clampLength({ x: error.x * 650 - velocity.x * 220, y: TOTAL_MASS_KG * 9.81 * (phase === "stand" ? 0.88 - this.rise * 0.35 : 0.88) + error.y * 1800 - velocity.y * 350, z: error.z * 650 - velocity.z * 220 }, RECOVERY_LIMITS.assistanceForceN);
      // Roll first; upward lift must not unload the floor while the trunk is still inverted.
      if (phase === "roll" && up < 0.25) force.y = Math.min(force.y, TOTAL_MASS_KG * 9.81 * 0.55);
      const torque = clampLength(implicitTorque(pelvis, null, angularVelocity(pelvis.rotation(), targetPelvisRotation, 1), pelvis.angvel(), 12000, 160, dt), RECOVERY_LIMITS.assistanceTorqueNm);
      pelvis.applyImpulse(scale(force, dt), true);
      pelvis.applyTorqueImpulse(scale(torque, dt), true);
      this.data.assistanceForce = force; this.data.assistanceTorque = torque;
    }
    return { state: recoveryActive ? "recovering" : phase === "settle" ? "fallen" : "falling", recovered: false };
  }

  private jointTargets(phase: RecoveryPhase): Map<SegmentId, Vec3> {
    const targets = new Map<SegmentId, Vec3>();
    const recovery = ["roll", "brace", "kneel", "stand"].includes(phase);
    const backward = this.data.orientation === "backward";
    targets.set("torso", {
      x: phase === "roll" ? (backward ? 0.40 : 0.15)
        : phase === "stand" ? 0
          : recovery ? -0.10 : backward ? 0.30 : -0.18,
      y: 0,
      z: 0,
    });
    targets.set("head", { x: recovery ? 0 : 0.32, y: 0, z: 0 });
    for (const side of ["left", "right"] as const) {
      const sign = side === "left" ? -1 : 1;
      const near = this.data.orientation === side;
      const armX = phase === "roll" ? (backward ? 0.40 : -1.45) : phase === "brace" ? -0.80 : phase === "kneel" ? -0.4 : phase === "stand" ? -0.10 : backward ? -0.25 : -1.25;
      targets.set(`${side}UpperArm`, { x: armX, y: 0, z: recovery ? sign * 0.10 : near ? sign * 1.1 : sign * 0.22 });
      targets.set(`${side}Forearm`, { x: recovery ? -0.4 : -0.9, y: 0, z: 0 });
      const hip = phase === "roll" ? -1.10 : phase === "brace" ? -0.85 : phase === "kneel" ? -1.0 : phase === "stand" ? -1.0 + 0.96 * this.rise : -0.35;
      const knee = phase === "roll" ? 1.7 : phase === "brace" ? 1.8 : phase === "kneel" ? 1.85 : phase === "stand" ? 1.85 - 1.77 * this.rise : 0.70;
      targets.set(`${side}Thigh`, { x: hip, y: 0, z: sign * 0.06 });
      targets.set(`${side}Shin`, { x: knee, y: 0, z: 0 });
      targets.set(`${side}Foot`, { x: -(hip + knee + (phase === "roll" ? -0.30 : phase === "brace" ? -0.10 : phase === "kneel" ? -0.05 : 0)), y: 0, z: -sign * 0.06 });
    }
    return targets;
  }
}
