import { MotorModel, type ImpulseJoint, type RigidBody, type World } from "@dimforge/rapier3d-compat";
import { SEGMENT_BY_ID } from "../core/humanoid";
import type { JointCoordinate, SegmentId, Vec3 } from "../core/types";
import { clampJointCoordinates, jointCoordinates, jointCoordinateTargetError, jointFrameAxesWorld } from "./joint-coordinates";
import { add, clamp, dot, scale, sub } from "./math";
import { assertRapierJointLimitCompatibility } from "./rapier-joint-adapter";
import type { JointMotorCommand, JointMotorResult } from "./joint-motors";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const IDENTITY = { ...ZERO, w: 1 };
const AXIS: Record<JointCoordinate, number> = { x: 3, y: 4, z: 5 };
interface RawMotorApi020 {
  jointConfigureMotorModel(handle: number, axis: number, model: number): void;
  jointSetMotorMaxForce(handle: number, axis: number, force: number): void;
  jointConfigureMotor(handle: number, axis: number, position: number, velocity: number, stiffness: number, damping: number): void;
}

/** Rapier solves these finite force-based motors together with its contact and
 * structural joint rows. No free-space inertia prediction, cached wrench,
 * positional correction, or accumulating integral error is applied by JS.
 */
export class NativeJointMotors {
  private readonly raw: RawMotorApi020;
  constructor(world: World) {
    assertRapierJointLimitCompatibility();
    const raw = world.impulseJoints.raw as unknown as Partial<RawMotorApi020>;
    for (const method of ["jointConfigureMotorModel", "jointSetMotorMaxForce", "jointConfigureMotor"] as const) {
      if (typeof raw[method] !== "function") throw new Error(`Missing pinned Rapier motor API: ${method}`);
    }
    this.raw = raw as RawMotorApi020;
  }

  apply(bodies: ReadonlyMap<SegmentId, RigidBody>, joints: ReadonlyMap<SegmentId, ImpulseJoint>, commands: readonly JointMotorCommand[]): Map<SegmentId, JointMotorResult> {
    const results = new Map<SegmentId, JointMotorResult>();
    for (const command of commands) {
      const definition = SEGMENT_BY_ID.get(command.id);
      const profile = definition?.jointProfile;
      if (!profile || !definition.parent) continue;
      const parent = bodies.get(definition.parent), child = bodies.get(command.id), joint = joints.get(command.id);
      if (!parent || !child || !joint) throw new Error(`Incomplete motor chain: ${command.id}`);
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
      const target = clampJointCoordinates(jointCoordinates(IDENTITY, command.targetLocalRotation, profile), profile);
      const error = jointCoordinateTargetError(coordinates, target, profile);
      // Native angular rows use the orthonormal parent joint-frame basis.
      const basis = jointFrameAxesWorld(parent.rotation(), profile);
      const strength = clamp(command.strengthScale, 0, 1);
      const kp = Math.max(0, command.stiffness) * strength;
      const kd = Math.max(0, command.damping) * strength;
      const relativeVelocity = sub(child.angvel(), parent.angvel());
      let torqueWorld = ZERO, saturationRatio = 0;
      for (const axis of profile.axes) {
        const coordinate = axis.coordinate, abiAxis = AXIS[coordinate];
        const cap = axis.maxMotorTorqueNm * strength;
        const feedforward = dot(command.feedforwardWorld ?? ZERO, basis[coordinate]) * strength;
        // A velocity bias encodes finite gravity/load compensation inside the
        // same force ceiling. No extra uncapped feedforward impulse is applied.
        const velocity = kd > 0 ? feedforward / kd : 0;
        this.raw.jointConfigureMotorModel(joint.handle, abiAxis, MotorModel.ForceBased);
        this.raw.jointSetMotorMaxForce(joint.handle, abiAxis, cap);
        this.raw.jointConfigureMotor(joint.handle, abiAxis, target[coordinate], velocity, kp, kd);
        const requested = kp * error[coordinate] + kd * (velocity - dot(relativeVelocity, basis[coordinate]));
        torqueWorld = add(torqueWorld, scale(basis[coordinate], clamp(requested, -cap, cap)));
        if (cap > 0) saturationRatio = Math.max(saturationRatio, Math.abs(requested) / cap);
      }
      // This is the bounded requested wrench, NOT the solver's delivered motor
      // impulse: the pinned JS API does not expose that impulse readback.
      results.set(command.id, { coordinates, targetCoordinates: target, coordinateError: error,
        torqueWorld, saturationRatio, torqueSource: "native-request" });
    }
    return results;
  }

  disable(joints: ReadonlyMap<SegmentId, ImpulseJoint>): void {
    for (const [id, joint] of joints) {
      for (const axis of SEGMENT_BY_ID.get(id)?.jointProfile?.axes ?? []) {
        this.raw.jointSetMotorMaxForce(joint.handle, AXIS[axis.coordinate], 0);
        this.raw.jointConfigureMotor(joint.handle, AXIS[axis.coordinate], 0, 0, 0, 0);
      }
    }
  }
}
