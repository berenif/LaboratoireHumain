import type { RegionId, Vec3 } from "../src/core/types";
import { HUMAN_PROPORTIONS } from "../src/core/humanoid";

/** Acceptance values are fixed before validation; scenarios run at exactly 60 Hz. */
export const ACCEPTANCE = Object.freeze({
  fixedDtS: 1 / 60,
  maxJointSeparationM: 0.08,
  maxFloorPenetrationM: 0.08,
  handoffTranslationM: 0.025,
  handoffRotationDegrees: 3,
  trajectoryPositionToleranceM: 1e-7,
  trajectoryRotationToleranceDegrees: 1e-5,
  trajectoryVelocityTolerance: 1e-7,
  inheritedLinearVelocityMps: 3,
  inheritedAngularVelocityRadps: 6,
  handoffVelocityTolerance: 1e-5,
  recoveryLimitS: 25,
  stableObservationS: 1,
  idleDurationS: 30,
  idleRootDriftM: 0.001,
  idleMaxLinearMps: 0.1,
  idleMaxAngularRadps: 0.5,
  maximumJointMotorTorqueNm: 110,
  repeatedCycles: 5,
});

/** Independent golden contract: changing controller thresholds cannot silently loosen scenario assertions. */
export const RECOVERY_ACCEPTANCE = Object.freeze({
  normalY: 0.65, contactDistanceM: 0.012, minimumLoadN: 3,
  loadPersistenceS: 0.05, landingPersistenceS: 0.10, settlePersistenceS: 0.30,
  settleLinearMps: 0.65, settleAngularRadps: 1.8,
  stablePersistenceS: 0.55, stableLinearMps: 0.22, stableAngularRadps: 0.65,
  stableUpDot: 0.97, phaseMinimumS: 0.20, stallS: 3.0, supportLossS: 0.20,
  assistanceForceN: 950, assistanceTorqueNm: 60,
});

export interface PullFixture {
  id: string;
  initial: { pose: "upright-rest"; position: Vec3; heading: number };
  region: RegionId;
  localAnchor: Vec3;
  /** Absolute fixed-step indices. Offsets are expressed in initial character coordinates. */
  targets: ReadonlyArray<{ frame: number; offset: Vec3 }>;
  releaseFrame: number;
  outcome: "recoverable" | "overpower";
  minimumSteps: number;
  observeUntilFrame: number;
  direction?: "forward" | "backward" | "left" | "right";
  releaseAtFirstSwing?: boolean;
  warmupFrames?: number;
  initialTargetWorldOffset?: Vec3;
  beginTimestampMs?: number;
  holdWithoutCommandsAfterLastTarget?: boolean;
}

const initial = (heading = 0): PullFixture["initial"] => ({
  pose: "upright-rest", position: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 }, heading,
});
const anchor = { x: 0.025, y: 0.015, z: 0.01 };
const slow = (id: string, region: RegionId, offset: Vec3, heading = 0): PullFixture => ({
  id, initial: initial(heading), region, localAnchor: anchor,
  targets: [{ frame: 0, offset: { x: 0, y: 0, z: 0 } }, { frame: 90, offset }],
  releaseFrame: 150, outcome: "recoverable", minimumSteps: 1, observeUntilFrame: 420,
});
const overload = (id: string, offset: Vec3, direction: PullFixture["direction"], heading = 0, duration = 5): PullFixture => ({
  id, initial: initial(heading), region: "rightHand", localAnchor: anchor,
  targets: [{ frame: 0, offset: { x: 0, y: 0, z: 0 } }, { frame: duration, offset }],
  releaseFrame: duration + 45, outcome: "overpower", minimumSteps: 0,
  observeUntilFrame: duration + 45 + ACCEPTANCE.recoveryLimitS * 60 + 60, direction,
});

/** No random seed, wall time, renderer state, or frame-rate-dependent interpolation enters these inputs. */
export const PULL_FIXTURES: ReadonlyArray<PullFixture> = [
  slow("slow-hand-forward", "rightHand", { x: 0, y: 0.03, z: 0.70 }),
  slow("slow-hand-backward", "leftHand", { x: 0, y: 0.03, z: -0.60 }),
  slow("slow-hand-left", "leftHand", { x: -0.60, y: 0.03, z: 0 }),
  slow("slow-hand-right-heading", "rightHand", { x: 0.60, y: 0.03, z: 0 }, Math.PI / 3),
  slow("slow-left-foot", "leftFoot", { x: -0.22, y: 0.03, z: 0.02 }),
  slow("slow-right-foot-heading", "rightFoot", { x: 0.22, y: 0.03, z: -0.02 }, -Math.PI / 4),
  { ...slow("direction-reversal", "rightHand", { x: 0.75, y: 0.02, z: 0 }),
    targets: [{ frame: 0, offset: { x: 0, y: 0, z: 0 } }, { frame: 90, offset: { x: 0.75, y: 0.02, z: 0 } },
      { frame: 180, offset: { x: -0.75, y: 0.02, z: 0 } }],
    releaseFrame: 300, observeUntilFrame: 510, minimumSteps: 2 },
  { ...slow("held-target", "leftHand", { x: -0.60, y: 0.02, z: 0.10 }), releaseFrame: 420, observeUntilFrame: 600 },
  { ...slow("release-during-corrective-step", "rightHand", { x: 0.60, y: 0.02, z: 0.08 }), releaseAtFirstSwing: true },
  overload("fast-forward", { x: 0, y: 0.12, z: 1.25 }, "forward"),
  overload("fast-backward", { x: 0, y: 0.12, z: -1.25 }, "backward"),
  overload("fast-left", { x: -1.25, y: 0.12, z: 0 }, "left"),
  overload("fast-right-heading", { x: 1.25, y: 0.12, z: 0 }, "right", Math.PI / 3),
  overload("sustained-forward", { x: 0, y: 0.10, z: 1.5 }, "forward", 0, 180),
  overload("sustained-backward-heading", { x: 0, y: 0.10, z: -1.5 }, "backward", -Math.PI / 4, 180),
];



/** Native-surface regression: literal target construction is retained from the independently failing probe. */
export const NATIVE_REGRESSION_FIXTURES: ReadonlyArray<PullFixture> = [[-0.8,5],[-0.8,15],[-0.4,5],[-0.4,15],[0,5],[0,15],[0.4,15]].map(([angle,duration]) => ({
  id: "native-surface-angle-" + angle + "-ramp-" + duration, initial: initial(), region: "rightHand" as const,
  localAnchor: {x:0,y:0,z:0.07}, initialTargetWorldOffset:{x:0,y:0,z:0.07}, beginTimestampMs:0, warmupFrames:60,
  targets:[{frame:0,offset:{x:0,y:0,z:0}},{frame:duration,offset:{x:1.3*Math.cos(angle),y:0.12,z:1.3*Math.sin(angle)}}],
  releaseFrame:1651,holdWithoutCommandsAfterLastTarget:true,outcome:"overpower" as const,minimumSteps:0,observeUntilFrame:1650,
}));
