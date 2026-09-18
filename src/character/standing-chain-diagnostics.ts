import { SEGMENT_BY_ID } from "../core/humanoid";
import type { SegmentId, SegmentPose, StandingChainDiagnostics, SupportingContact } from "../core/types";
import type { JointMotorCommand } from "./joint-motors";
import type { StepMotion } from "./pose";
import { recoverySupportHull } from "./recovery-support";
import { LEG_TARGET_FRAME, legTargetReach } from "./leg-target-frame";
import { length, quatFromAxisAngle, quatMultiply, rotate, sub, worldPoint } from "./math";

const UP = { x: 0, y: 1, z: 0 };
const FORWARD = { x: 0, y: 0, z: 1 };
type Pose = Pick<SegmentPose, "id" | "position" | "rotation">;

function copyPose(pose: Pose | undefined) {
  return pose ? {
    id: pose.id,
    position: { ...pose.position },
    rotation: { ...pose.rotation },
    forward: rotate(pose.rotation, FORWARD),
  } : null;
}

export function copyStandingContacts(contacts: readonly SupportingContact[]): SupportingContact[] {
  return contacts.map(contact => ({
    ...contact,
    point: { ...contact.point },
    ...(contact.points ? { points: contact.points.map(point => ({ ...point })) } : {}),
  }));
}

/** Read-only FK reconstruction. None of these diagnostic poses is sent to Rapier. */
export function standingChainDiagnostics(
  physical: ReadonlyMap<SegmentId, Pose>,
  desired: ReadonlyMap<SegmentId, Pose>,
  commands: readonly JointMotorCommand[],
  step: StepMotion | null,
  heading: number,
  motorSampleTimeS: number,
  physicalSampleTimeS: number,
  contacts: readonly SupportingContact[],
  motorInputPelvis?: Pose | null,
): StandingChainDiagnostics {
  const pelvis = physical.get("pelvis")!;
  const commandRoot = motorInputPelvis ?? pelvis;
  const reconstructed = new Map<SegmentId, Pose>([["pelvis", commandRoot]]);
  const byId = new Map(commands.map(command => [command.id, command]));
  for (const side of ["left", "right"] as const) {
    for (const suffix of ["Thigh", "Shin", "Ankle", "Foot", "Forefoot"] as const) {
      const id = `${side}${suffix}` as SegmentId;
      const definition = SEGMENT_BY_ID.get(id)!;
      const parent = reconstructed.get(definition.parent!);
      const command = byId.get(id);
      if (!parent || !command) continue;
      const rotation = quatMultiply(parent.rotation, command.targetLocalRotation);
      const position = sub(worldPoint(parent.position, parent.rotation, definition.jointAnchorParent!),
        rotate(rotation, definition.jointAnchorChild!));
      reconstructed.set(id, { id, position, rotation });
    }
  }
  const headingRotation = quatFromAxisAngle(UP, step?.heading ?? heading);
  const stanceSide = step ? step.foot === "leftFoot" ? "right" : "left" : null;
  const stance = stanceSide ? physical.get(`${stanceSide}Ankle`) : null;
  const stanceAnkle = stance ? worldPoint(stance.position, stance.rotation,
    SEGMENT_BY_ID.get(stance.id)!.jointAnchorChild!) : null;
  const supportPoints = contacts.filter(contact => contact.loadBearing)
    .flatMap(contact => (contact.points?.length ? contact.points : [contact.point])
      .map(point => ({ ...point })));
  return {
    frame: LEG_TARGET_FRAME,
    motorSampleTimeS,
    physicalSampleTimeS,
    reconstruction: "local-commands-on-sampled-physical-pelvis" as const,
    pelvis: copyPose(pelvis),
    desiredPelvis: copyPose(desired.get("pelvis")),
    motorInputPelvis: copyPose(commandRoot),
    stanceAnkle,
    // Actual solver points, not a fabricated rectangular load-bearing footprint.
    supportPoints,
    supportPolygon: recoverySupportHull(supportPoints),
    step: step ? {
      foot: step.foot,
      from: { ...step.from },
      to: { ...step.to },
      requested: { ...(step.requested ?? step.to) },
      rebased: { ...step.to },
      heading: step.heading ?? heading,
      elapsedS: step.elapsed,
      durationS: step.duration,
    } : null,
    legs: (["left", "right"] as const).map(side => {
      const hip = worldPoint(pelvis.position, pelvis.rotation,
        SEGMENT_BY_ID.get(`${side}Thigh`)!.jointAnchorParent!);
      const targetFoot = desired.get(`${side}Foot`);
      const requested = step?.foot === `${side}Foot` ? step.to : targetFoot?.position;
      return {
        side,
        hip,
        landingReach: requested ? legTargetReach(side, hip, requested, headingRotation) : null,
        segments: (["Thigh", "Shin", "Ankle", "Foot", "Forefoot"] as const).map(suffix => {
          const id = `${side}${suffix}` as SegmentId;
          const actual = physical.get(id);
          const target = desired.get(id);
          const commanded = reconstructed.get(id);
          return {
            id,
            physical: copyPose(actual),
            desired: copyPose(target),
            commanded: copyPose(commanded),
            targetLocalRotation: byId.get(id) ? { ...byId.get(id)!.targetLocalRotation } : null,
            desiredErrorM: actual && target ? length(sub(actual.position, target.position)) : null,
            commandFrameErrorM: commanded && target ? length(sub(commanded.position, target.position)) : null,
          };
        }),
      };
    }),
    armForward: (["leftUpperArm", "leftForearm", "rightUpperArm", "rightForearm"] as const)
      .map(id => ({ id, physical: copyPose(physical.get(id)), desired: copyPose(desired.get(id)) })),
  };
}
