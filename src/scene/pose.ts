import { Q, V3 } from "../core/math";
import type { PoseSnapshot, Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";

export function clampInterpolationAlpha(alpha: number): number {
  return Number.isFinite(alpha) ? Math.max(0, Math.min(1, alpha)) : 1;
}

export function rotateVector(rotation: Quat, vector: Vec3): Vec3 {
  const q = Q.normalize(rotation);
  const tx = 2 * (q.y * vector.z - q.z * vector.y);
  const ty = 2 * (q.z * vector.x - q.x * vector.z);
  const tz = 2 * (q.x * vector.y - q.y * vector.x);
  return {
    x: vector.x + q.w * tx + (q.y * tz - q.z * ty),
    y: vector.y + q.w * ty + (q.z * tx - q.x * tz),
    z: vector.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

export function transformLocalPoint(pose: SegmentPose, localPoint: Vec3): Vec3 {
  return V3.add(pose.position, rotateVector(pose.rotation, localPoint));
}

/** Returns a detached display snapshot. It is never fed back to simulation. */
export function interpolatePoseSnapshot(
  previous: PoseSnapshot,
  current: PoseSnapshot,
  alpha: number,
): PoseSnapshot {
  const t = clampInterpolationAlpha(alpha);
  const previousById = new Map<SegmentId, SegmentPose>(
    previous.segments.map((segment) => [segment.id, segment]),
  );
  const segments = current.segments.map((segment): SegmentPose => {
    const before = previousById.get(segment.id) ?? segment;
    return {
      id: segment.id,
      position: V3.lerp(before.position, segment.position, t),
      rotation: Q.nlerp(before.rotation, segment.rotation, t),
      linearVelocity: V3.lerp(before.linearVelocity, segment.linearVelocity, t),
      angularVelocity: V3.lerp(before.angularVelocity, segment.angularVelocity, t),
    };
  });
  return {
    ...current,
    simulationTime: previous.simulationTime + (current.simulationTime - previous.simulationTime) * t,
    rootPosition: V3.lerp(previous.rootPosition, current.rootPosition, t),
    rootRotation: Q.nlerp(previous.rootRotation, current.rootRotation, t),
    segments,
    striker: current.striker && previous.striker ? {
      ...current.striker,
      position: V3.lerp(previous.striker.position, current.striker.position, t),
      rotation: Q.nlerp(previous.striker.rotation, current.striker.rotation, t),
    } : current.striker,
  };
}

type MutableVector = { x: number; y: number; z: number };
type MutableQuaternion = MutableVector & { w: number };
function lerpInto(out: MutableVector, a: Vec3, b: Vec3, t: number): void {
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.z = a.z + (b.z - a.z) * t;
}
function nlerpInto(out: MutableQuaternion, a: Quat, b: Quat, t: number): void {
  const sign = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0 ? -1 : 1;
  out.x = a.x + (b.x * sign - a.x) * t;
  out.y = a.y + (b.y * sign - a.y) * t;
  out.z = a.z + (b.z * sign - a.z) * t;
  out.w = a.w + (b.w * sign - a.w) * t;
  const length = Math.hypot(out.x, out.y, out.z, out.w) || 1;
  out.x /= length; out.y /= length; out.z /= length; out.w /= length;
}

/** Adapter-owned scratch state. Never publish this as a simulation snapshot. */
export class PresentationBuffer {
  readonly byId = new Map<SegmentId, SegmentPose>();
  private readonly before = new Map<SegmentId, SegmentPose>();
  private previous: PoseSnapshot | null = null;
  private readonly segments: SegmentPose[] = [];
  private readonly rootPosition = { x: 0, y: 0, z: 0 };
  private readonly rootRotation = { x: 0, y: 0, z: 0, w: 1 };
  private striker: NonNullable<PoseSnapshot["striker"]> | null = null;
  private display: PoseSnapshot | null = null;

  update(previous: PoseSnapshot, current: PoseSnapshot, alpha: number): PoseSnapshot {
    const t = clampInterpolationAlpha(alpha);
    if (this.previous !== previous) {
      this.before.clear();
      for (const pose of previous.segments) this.before.set(pose.id, pose);
      this.previous = previous;
    }
    this.segments.length = current.segments.length;
    for (let i = 0; i < current.segments.length; i++) {
      const pose = current.segments[i];
      let out = this.byId.get(pose.id);
      if (!out) {
        out = { id: pose.id, position: { ...pose.position }, rotation: { ...pose.rotation },
          linearVelocity: { ...pose.linearVelocity }, angularVelocity: { ...pose.angularVelocity } };
        this.byId.set(pose.id, out);
      }
      const before = this.before.get(pose.id) ?? pose;
      lerpInto(out.position, before.position, pose.position, t);
      nlerpInto(out.rotation, before.rotation, pose.rotation, t);
      lerpInto(out.linearVelocity, before.linearVelocity, pose.linearVelocity, t);
      lerpInto(out.angularVelocity, before.angularVelocity, pose.angularVelocity, t);
      this.segments[i] = out;
    }
    // Handle reduced/reordered snapshots without retaining stale joints.
    if (this.byId.size !== this.segments.length) {
      for (const id of this.byId.keys()) if (!current.segments.some(pose => pose.id === id)) this.byId.delete(id);
    }
    lerpInto(this.rootPosition, previous.rootPosition, current.rootPosition, t);
    nlerpInto(this.rootRotation, previous.rootRotation, current.rootRotation, t);
    if (current.striker) {
      this.striker ??= { ...current.striker, position: { ...current.striker.position }, rotation: { ...current.striker.rotation } };
      const { position, rotation } = this.striker;
      Object.assign(this.striker, current.striker, { position, rotation });
      lerpInto(position, previous.striker?.position ?? current.striker.position, current.striker.position, t);
      nlerpInto(rotation, previous.striker?.rotation ?? current.striker.rotation, current.striker.rotation, t);
    }
    this.display ??= { ...current };
    Object.assign(this.display, current, { segments: this.segments, rootPosition: this.rootPosition,
      rootRotation: this.rootRotation, striker: current.striker ? this.striker : current.striker,
      simulationTime: previous.simulationTime + (current.simulationTime - previous.simulationTime) * t });
    return this.display;
  }
}
