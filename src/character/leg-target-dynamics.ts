import type { RigidBody } from "@dimforge/rapier3d-compat";
import { SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";
import type { JointMotorCommand } from "./joint-motors";
import { add, cross, quatInverse, quatMultiply, quatNormalize, rotate, scale, sub, worldPoint } from "./math";

export type LegTargetBody = Pick<RigidBody, "mass" | "localCom" | "principalInertia" | "principalInertiaLocalFrame">;
type TargetCommand = Pick<JointMotorCommand, "id" | "targetLocalRotation">;
type PelvisPose = Pick<SegmentPose, "position" | "rotation">;
type Side = "left" | "right";
const SUFFIXES = ["Thigh", "Shin", "Ankle", "Foot", "Forefoot"] as const;
const zero = (): Vec3 => ({ x: 0, y: 0, z: 0 });
const finiteVector = (value: Vec3): boolean => [value.x, value.y, value.z].every(Number.isFinite);
const validRotation = (value: Quat): boolean => [value.x, value.y, value.z, value.w].every(Number.isFinite)
  && Math.hypot(value.x, value.y, value.z, value.w) > 1e-12;

/** Apply a body's world inertia tensor, including its local principal frame. */
export function worldInertiaProduct(principalInertia: Vec3, principalFrameWorld: Quat, value: Vec3): Vec3 {
  const frame = quatNormalize(principalFrameWorld);
  const local = rotate(quatInverse(frame), value);
  return rotate(frame, { x: principalInertia.x * local.x,
    y: principalInertia.y * local.y, z: principalInertia.z * local.z });
}

export interface TargetBodyMotion {
  centerOfMass: Vec3;
  massKg: number;
  linearAcceleration: Vec3;
  angularVelocity: Vec3;
  angularAcceleration: Vec3;
  principalInertia: Vec3;
  principalFrameWorld: Quat;
}

/** Child-side inertial moment only: gravity and contact forces belong elsewhere. */
export function targetInertialMoment(jointAnchor: Vec3, body: TargetBodyMotion): Vec3 {
  const inertia = (value: Vec3) => worldInertiaProduct(body.principalInertia, body.principalFrameWorld, value);
  return add(cross(sub(body.centerOfMass, jointAnchor), scale(body.linearAcceleration, body.massKg)),
    add(inertia(body.angularAcceleration), cross(body.angularVelocity, inertia(body.angularVelocity))));
}

interface TargetState extends PelvisPose {
  centerOfMass: Vec3;
  jointAnchor: Vec3;
  massKg: number;
  principalInertia: Vec3;
  principalFrameWorld: Quat;
}
interface IntervalMotion { linearVelocity: Vec3; angularVelocity: Vec3 }

/** Shortest world rotation increment, with no extra speed or torque clipping. */
function rotationVelocity(previous: Quat, current: Quat, dt: number): Vec3 {
  let delta = quatMultiply(current, quatInverse(previous));
  if (delta.w < 0) delta = { x: -delta.x, y: -delta.y, z: -delta.z, w: -delta.w };
  const sine = Math.hypot(delta.x, delta.y, delta.z);
  if (sine < 1e-12) return zero();
  const factor = 2 * Math.atan2(sine, delta.w) / (sine * dt);
  return { x: delta.x * factor, y: delta.y * factor, z: delta.z * factor };
}

/**
 * Reconstruct and differentiate only the final commanded active-leg chain.
 * World target acceleration includes measured pelvis/base motion. Backward
 * differences require three samples; phase changes never reuse stale motion.
 * This class reads mass properties and returns torque intent, never body forces.
 */
export class LegTargetDynamics {
  private side: Side | null = null;
  private previous: Map<SegmentId, TargetState> | null = null;
  private previousMotion: Map<SegmentId, IntervalMotion> | null = null;
  private previousDt = 0;

  reset(): void {
    this.side = null;
    this.previous = null;
    this.previousMotion = null;
    this.previousDt = 0;
  }

  sample(pelvisPose: PelvisPose, commands: readonly TargetCommand[],
    bodies: ReadonlyMap<SegmentId, LegTargetBody>, side: Side | null, dt: number): Map<SegmentId, Vec3> {
    if (side === null || !Number.isFinite(dt) || dt <= 0
      || !finiteVector(pelvisPose.position) || !validRotation(pelvisPose.rotation)) {
      this.reset();
      return new Map();
    }
    if (side !== this.side) { this.reset(); this.side = side; }
    const ids = SUFFIXES.map(suffix => `${side}${suffix}` as SegmentId);
    const byId = new Map(commands.map(command => [command.id, command]));
    const targets = new Map<SegmentId, TargetState>();
    let parent: PelvisPose = { position: { ...pelvisPose.position }, rotation: quatNormalize(pelvisPose.rotation) };
    for (const id of ids) {
      const definition = SEGMENT_BY_ID.get(id)!;
      const command = byId.get(id), body = bodies.get(id);
      if (!command || !body || !validRotation(command.targetLocalRotation)) {
        this.reset(); return new Map();
      }
      const massKg = body.mass(), localCom = body.localCom();
      const principalInertia = body.principalInertia(), localPrincipalFrame = body.principalInertiaLocalFrame();
      if (!Number.isFinite(massKg) || massKg <= 0 || !finiteVector(localCom)
        || !finiteVector(principalInertia) || Math.min(principalInertia.x, principalInertia.y, principalInertia.z) < 0
        || !validRotation(localPrincipalFrame)) {
        this.reset(); return new Map();
      }
      const rotation = quatMultiply(parent.rotation, command.targetLocalRotation);
      const jointAnchor = worldPoint(parent.position, parent.rotation, definition.jointAnchorParent!);
      const position = sub(jointAnchor, rotate(rotation, definition.jointAnchorChild!));
      const target: TargetState = { position, rotation, jointAnchor,
        centerOfMass: worldPoint(position, rotation, localCom), massKg,
        principalInertia: { ...principalInertia }, principalFrameWorld: quatMultiply(rotation, localPrincipalFrame) };
      targets.set(id, target);
      parent = target;
    }
    const result = new Map<SegmentId, Vec3>(ids.map(id => [id, zero()]));
    const motion = new Map<SegmentId, IntervalMotion>();
    if (this.previous) {
      for (const id of ids) {
        const target = targets.get(id)!, prior = this.previous.get(id)!;
        motion.set(id, {
          linearVelocity: scale(sub(target.centerOfMass, prior.centerOfMass), 1 / dt),
          angularVelocity: rotationVelocity(prior.rotation, target.rotation, dt),
        });
      }
      if (this.previousMotion) {
        // These velocities represent consecutive interval midpoints.
        const accelerationDt = (dt + this.previousDt) / 2;
        const bodyMotion = new Map<SegmentId, TargetBodyMotion>();
        for (const id of ids) {
          const current = motion.get(id)!, prior = this.previousMotion.get(id)!;
          bodyMotion.set(id, { ...targets.get(id)!, angularVelocity: current.angularVelocity,
            linearAcceleration: scale(sub(current.linearVelocity, prior.linearVelocity), 1 / accelerationDt),
            angularAcceleration: scale(sub(current.angularVelocity, prior.angularVelocity), 1 / accelerationDt) });
        }
        for (let joint = 0; joint < ids.length; joint++) {
          const anchor = targets.get(ids[joint])!.jointAnchor;
          let torque = zero();
          for (const descendant of ids.slice(joint)) torque = add(torque,
            targetInertialMoment(anchor, bodyMotion.get(descendant)!));
          if (!finiteVector(torque)) { this.reset(); return new Map(); }
          result.set(ids[joint], torque);
        }
      }
    }
    this.previous = targets;
    this.previousMotion = this.previous ? (motion.size ? motion : null) : null;
    this.previousDt = dt;
    return result;
  }
}
