import { SEGMENT_BY_ID, SEGMENTS } from "../src/core/humanoid";
import type { CharacterController, Quat, SegmentId, Vec3 } from "../src/core/types";
import { add, quatFromAxisAngle, quatMultiply, rotate, sub, worldPoint } from "../src/character/math";
import { clampJointCoordinates, jointRotationFromCoordinates } from "../src/character/joint-coordinates";
import type { MutablePose } from "../src/character/pose";

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

function setChild(poses: Map<SegmentId, MutablePose>, id: SegmentId, requested: Vec3 = ZERO): void {
  const definition = SEGMENT_BY_ID.get(id)!;
  const parent = poses.get(definition.parent!)!;
  if (!definition.jointProfile) throw new Error(`Fixture segment ${id} has no joint profile`);
  const coordinates = clampJointCoordinates(requested, definition.jointProfile);
  const rotation = quatMultiply(parent.rotation, jointRotationFromCoordinates(coordinates, definition.jointProfile));
  poses.set(id, { id, rotation,
    position: sub(worldPoint(parent.position, parent.rotation, definition.jointAnchorParent!), rotate(rotation, definition.jointAnchorChild!)),
    linearVelocity: ZERO, angularVelocity: ZERO });
}

function lowestPoint(poses: Map<SegmentId, MutablePose>): number {
  return Math.min(...SEGMENTS.map(definition => {
    const pose = poses.get(definition.id)!;
    return Math.min(...definition.geometry.vertices.map(vertex => worldPoint(pose.position, pose.rotation, vertex).y));
  }));
}

export function recoveryFixturePoses(fixture: RecoveryPoseFixture): Map<SegmentId, MutablePose> {
  const leading = fixture.side;
  let poses: Map<SegmentId, MutablePose>;
  if (fixture.pose === "crouch") {
    poses = new Map();
    poses.set("pelvis", { id: "pelvis", position: ZERO, rotation: { x: 0, y: 0, z: 0, w: 1 },
      linearVelocity: ZERO, angularVelocity: ZERO });
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      let coordinates = ZERO;
      if (definition.id.endsWith("UpperArm")) coordinates = {
        x: 0.10, y: 0, z: definition.id.startsWith("left") ? 0.04 : -0.04,
      };
      if (definition.role === "forearm") coordinates = { x: 0.20, y: 0, z: 0 };
      // A flat sole requires knee = hip + dorsiflexion in the shared
      // forward-flexion frames. Keep dorsiflexion below the anatomical 20°
      // stop; the former 1.10/-0.76 fixture demanded 43.5° at this ankle.
      if (definition.role === "thigh") coordinates = { x: 0.34, y: 0, z: 0 };
      if (definition.role === "shin") coordinates = { x: 0.66, y: 0, z: 0 };
      if (definition.role === "ankle") coordinates = { x: 0.32, y: 0, z: 0 };
      setChild(poses, definition.id, coordinates);
    }
  } else if (fixture.pose === "half-kneel") {
    poses = new Map();
    poses.set("pelvis", { id: "pelvis", position: ZERO, rotation: pitch(0), linearVelocity: ZERO, angularVelocity: ZERO });
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const lead = definition.id.startsWith(leading);
      let coordinates = ZERO;
      if (definition.id === "lumbar") coordinates = fixture.support === "weak"
        ? { x: 0.07, y: 0, z: 0 }
        : { x: 0.10, y: 0, z: leading === "left" ? -0.14 : 0.14 };
      if (definition.id === "torso") coordinates = fixture.support === "weak"
        ? { x: 0.08, y: 0, z: 0 }
        : { x: 0.10, y: 0, z: leading === "left" ? -0.16 : 0.16 };
      if (definition.id.endsWith("UpperArm")) coordinates = {
        x: 0.10, y: 0, z: definition.id.startsWith("left") ? 0.04 : -0.04,
      };
      if (definition.role === "forearm") coordinates = { x: 0.20, y: 0, z: 0 };
      if (definition.role === "thigh") coordinates = { x: lead ? Math.PI / 2 : 0, y: 0, z: 0 };
      if (definition.role === "shin") coordinates = { x: lead ? Math.PI / 2 : 1.85, y: 0, z: 0 };
      if (definition.role === "ankle") coordinates = { x: lead ? 0 : -0.65, y: 0, z: 0 };
      if (definition.role === "forefoot") coordinates = { x: lead ? 0 : -0.25, y: 0, z: 0 };
      setChild(poses, definition.id, coordinates);
    }
    // Match the leading sole to the actual convex trailing-shin surface. Hip
    // flexion and knee flexion cancel, so the leading shin and sole stay level.
    // This is fixture construction before integration, never a runtime body edit.
    const trailing = leading === "left" ? "right" : "left";
    const surfaceY = (id: SegmentId): number => Math.min(...SEGMENT_BY_ID.get(id)!.geometry.vertices.map(vertex => {
      const pose = poses.get(id)!;
      return worldPoint(pose.position, pose.rotation, vertex).y;
    }));
    const shinY = surfaceY(`${trailing}Shin`);
    let lower = 1.4, upper = 1.8;
    for (let iteration = 0; iteration < 48; iteration++) {
      const flexion = (lower + upper) / 2;
      setChild(poses, `${leading}Thigh`, { x: flexion, y: 0, z: 0 });
      setChild(poses, `${leading}Shin`, { x: flexion, y: 0, z: 0 });
      for (const suffix of ["Ankle", "Foot", "Forefoot"] as const) setChild(poses, `${leading}${suffix}`);
      const soleY = Math.min(surfaceY(`${leading}Foot`), surfaceY(`${leading}Forefoot`));
      if (soleY < shinY) lower = flexion;
      else upper = flexion;
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
      let coordinates = ZERO;
      // Shoulder/hip/ankle +X and shoulder +Z reversed in the shared
      // half-turn frames. Convert the legacy pose signs once; the prone root
      // remains horizontal, and the elbows keep positive anatomical flexion.
      if (definition.id.endsWith("UpperArm")) coordinates = {
        x: fixture.pose === "supine" ? 0.30 : -0.12,
        y: 0,
        z: (definition.id.startsWith("left") ? 1 : -1) * (near ? 0.18 : 0.34),
      };
      if (definition.role === "forearm") coordinates = { x: near ? 0.40 : 0.72, y: 0, z: 0 };
      if (definition.id.endsWith("Hand")) coordinates = { x: 0.28, y: 0, z: 0 };
      if (definition.id.endsWith("Thigh")) coordinates = { x: near ? 0.18 : 0.34, y: 0, z: 0 };
      if (definition.id.endsWith("Shin")) coordinates = { x: near ? 0.30 : 0.55, y: 0, z: 0 };
      if (definition.id.endsWith("Ankle")) coordinates = { x: 0.12, y: 0, z: 0 };
      setChild(poses, definition.id, coordinates);
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
  const target = character as CharacterController & {
    seedRecoveryFixture(poses: ReadonlyMap<SegmentId, MutablePose>, direction: Vec3): void;
  };
  if (typeof target.seedRecoveryFixture !== "function") {
    throw new Error("Character does not expose the recovery fixture initialization entrypoint");
  }
  target.seedRecoveryFixture(recoveryFixturePoses(fixture), ZERO);
}


