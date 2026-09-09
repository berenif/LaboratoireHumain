import { SEGMENT_BY_ID, SEGMENTS } from "../src/core/humanoid";
import type { CharacterController, Quat, SegmentId, Vec3 } from "../src/core/types";
import { add, quatFromAxisAngle, quatMultiply, rotate, sub, worldPoint } from "../src/character/math";
import { composeUprightPose, type MutablePose } from "../src/character/pose";

export interface RecoveryPoseFixture {
  id: string;
  pose: "prone" | "supine" | "side" | "half-kneel" | "crouch";
  side: "left" | "right";
  heading: number;
  support?: "balanced" | "weak";
}

/** Connected, zero-momentum landed poses. Every asymmetric pose has an exact mirror and yaw variant. */
export const RECOVERY_POSE_FIXTURES: readonly RecoveryPoseFixture[] = [
  ...(["prone", "supine", "side", "half-kneel", "crouch"] as const).flatMap(pose =>
    (["left", "right"] as const).flatMap(side => [0, Math.PI / 3].map(heading => ({
      id: `landed-${pose}-${side}${heading ? "-heading" : ""}`, pose, side, heading,
    })))),
  ...(["left", "right"] as const).flatMap(side => [0, Math.PI / 3].map(heading => ({
    id: `landed-half-kneel-weak-${side}${heading ? "-heading" : ""}`, pose: "half-kneel" as const, side, heading, support: "weak" as const,
  }))),
];

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const pitch = (angle: number): Quat => quatFromAxisAngle({ x: 1, y: 0, z: 0 }, angle);
const roll = (angle: number): Quat => quatFromAxisAngle({ x: 0, y: 0, z: 1 }, angle);

function setChild(poses: Map<SegmentId, MutablePose>, id: SegmentId, localRotation: Quat): void {
  const definition = SEGMENT_BY_ID.get(id)!;
  const parent = poses.get(definition.parent!)!;
  const rotation = quatMultiply(parent.rotation, localRotation);
  poses.set(id, { id, rotation,
    position: sub(worldPoint(parent.position, parent.rotation, definition.jointAnchorParent!), rotate(rotation, definition.jointAnchorChild!)),
    linearVelocity: ZERO, angularVelocity: ZERO });
}

function lowestPoint(poses: Map<SegmentId, MutablePose>): number {
  return Math.min(...SEGMENTS.map(definition => {
    const pose = poses.get(definition.id)!, shape = definition.shape;
    if (shape.kind === "sphere") return pose.position.y - shape.radius;
    if (shape.kind === "capsule") return pose.position.y - shape.radius - Math.abs(rotate(pose.rotation, UP).y) * shape.halfHeight;
    const x = rotate(pose.rotation, { x: 1, y: 0, z: 0 }), y = rotate(pose.rotation, UP), z = rotate(pose.rotation, { x: 0, y: 0, z: 1 });
    return pose.position.y - Math.abs(x.y) * shape.halfExtents.x - Math.abs(y.y) * shape.halfExtents.y - Math.abs(z.y) * shape.halfExtents.z;
  }));
}

export function recoveryFixturePoses(fixture: RecoveryPoseFixture): Map<SegmentId, MutablePose> {
  const leading = fixture.side;
  let poses: Map<SegmentId, MutablePose>;
  if (fixture.pose === "crouch") {
    poses = composeUprightPose({ rootTranslation: { x: 0, y: 0.825, z: 0 }, reactionOffset: ZERO,
      simulationTime: 0, activeGrab: null, step: null, heading: 0, kneeFlexion: 0,
      supportFeet: { leftFoot: { x: -0.11, y: 0.045, z: 0.10 }, rightFoot: { x: 0.11, y: 0.045, z: 0.10 } } }).poses;
    for (const side of ["left", "right"] as const) {
      setChild(poses, `${side}UpperArm`, quatMultiply(pitch(-0.10), roll(side === "left" ? -0.04 : 0.04)));
      setChild(poses, `${side}Forearm`, pitch(-0.20));
      setChild(poses, `${side}Hand`, pitch(0));
    }
  } else if (fixture.pose === "half-kneel") {
    poses = new Map();
    poses.set("pelvis", { id: "pelvis", position: { x: 0, y: 0.53, z: 0 }, rotation: pitch(-0.15), linearVelocity: ZERO, angularVelocity: ZERO });
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const lead = definition.id.startsWith(leading);
      let local = pitch(0);
      if (definition.id === "torso") local = fixture.support === "weak" ? pitch(0.15)
        : quatMultiply(pitch(0.45), roll(leading === "left" ? -0.30 : 0.30));
      if (definition.id.endsWith("UpperArm")) local = quatMultiply(pitch(-0.10), roll(definition.id.startsWith("left") ? -0.04 : 0.04));
      if (definition.id.endsWith("Forearm")) local = pitch(-0.20);
      if (definition.id.endsWith("Thigh")) local = pitch(lead ? -1.35 : 0.15);
      if (definition.id.endsWith("Shin")) local = pitch(2.10);
      if (definition.id.endsWith("Foot")) local = pitch(-0.60);
      setChild(poses, definition.id, local);
    }
  } else {
    poses = new Map();
    const rootRotation = fixture.pose === "prone" ? pitch(Math.PI / 2)
      : fixture.pose === "supine" ? pitch(-Math.PI / 2)
        : roll(fixture.side === "left" ? Math.PI / 2 : -Math.PI / 2);
    poses.set("pelvis", { id: "pelvis", position: ZERO, rotation: rootRotation, linearVelocity: ZERO, angularVelocity: ZERO });
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const near = definition.id.startsWith(fixture.side);
      let rotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
      if (definition.id.endsWith("UpperArm")) rotation = quatMultiply(pitch(fixture.pose === "supine" ? -0.30 : 0.12), roll((definition.id.startsWith("left") ? -1 : 1) * (near ? 0.18 : 0.45)));
      if (definition.id.endsWith("Forearm")) rotation = pitch(near ? -0.40 : -0.72);
      if (definition.id.endsWith("Hand")) rotation = pitch(0.28);
      if (definition.id.endsWith("Thigh")) rotation = pitch(near ? -0.18 : -0.35);
      if (definition.id.endsWith("Shin")) rotation = pitch(near ? 0.30 : 0.55);
      if (definition.id.endsWith("Foot")) rotation = pitch(-0.12);
      setChild(poses, definition.id, rotation);
    }
  }
  // Position the actual oriented collider surface on the floor, preserving every anatomical anchor.
  const lift = -lowestPoint(poses) + 0.001;
  const heading = quatFromAxisAngle(UP, fixture.heading);
  return new Map([...poses].map(([id, pose]) => [id, { ...pose,
    position: rotate(heading, add(pose.position, { x: 0, y: lift, z: 0 })), rotation: quatMultiply(heading, pose.rotation),
    linearVelocity: { ...ZERO }, angularVelocity: { ...ZERO } }]));
}

/** Test setup only: create the dynamic bodies from the measured fixture before any integration. */
export function seedRecoveryFixture(character: CharacterController, fixture: RecoveryPoseFixture): void {
  const target = character as unknown as { poses: Map<SegmentId, MutablePose>; previousPoses: Map<SegmentId, MutablePose>; activateRagdoll(direction: Vec3): void };
  target.poses = recoveryFixturePoses(fixture);
  target.previousPoses = new Map([...target.poses].map(([id, pose]) => [id, { ...pose }]));
  target.activateRagdoll(ZERO);
}


