import type { RegionId, SegmentDefinition, SegmentId } from "./types";

const identity = { x: 0, y: 0, z: 0, w: 1 } as const;
const noLimit = { x: Math.PI, y: Math.PI, z: Math.PI } as const;

/** Adult body dimensions in metres, shared by geometry, pose composition, and tests. */
export const HUMAN_PROPORTIONS = {
  totalHeightM: 1.84,
  pelvis: {
    centerHeightM: 0.99,
    halfExtentsM: { x: 0.17, y: 0.13, z: 0.11 },
    spineAnchorYM: 0.13,
    hipAnchorXM: 0.09,
    hipAnchorYM: -0.08,
  },
  torso: {
    halfExtentsM: { x: 0.225, y: 0.22, z: 0.11 },
    pelvisAnchorYM: -0.20,
    neckAnchorYM: 0.22,
    shoulderAnchorXM: 0.225,
    shoulderAnchorYM: 0.13,
  },
  neck: { radiusM: 0.05, halfHeightM: 0.015, anchorYM: 0.035 },
  head: { radiusM: 0.115 },
  arm: {
    upperLengthM: 0.31,
    upperRadiusM: 0.055,
    forearmLengthM: 0.27,
    forearmRadiusM: 0.045,
    handHalfExtentsM: { x: 0.045, y: 0.09, z: 0.025 },
    relaxedLateralOffsetM: 0.025,
    relaxedShorteningM: 0.02,
  },
  leg: {
    thighLengthM: 0.42,
    thighRadiusM: 0.075,
    shinLengthM: 0.40,
    shinRadiusM: 0.06,
  },
  foot: {
    halfExtentsM: { x: 0.05, y: 0.045, z: 0.135 },
    ankleOffsetZM: -0.085,
    stepClearanceBaseM: 0.055,
    stepClearancePerTravel: 0.15,
    minStepClearanceM: 0.06,
    maxStepClearanceM: 0.12,
  },
  stance: { neutralKneeFlexion: 0.12 },
} as const;

const P = HUMAN_PROPORTIONS;
const upperArmHalfLength = P.arm.upperLengthM / 2;
const forearmHalfLength = P.arm.forearmLengthM / 2;
const thighHalfLength = P.leg.thighLengthM / 2;
const shinHalfLength = P.leg.shinLengthM / 2;

export const SEGMENTS: ReadonlyArray<SegmentDefinition> = [
  {
    id: "pelvis", parent: null, region: "pelvis", massKg: 12,
    shape: { kind: "box", halfExtents: P.pelvis.halfExtentsM },
    localOffset: { x: 0, y: P.pelvis.centerHeightM, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: null, jointAnchorChild: null, jointLimitRadians: null, collisionGroup: 1,
  },
  {
    id: "torso", parent: "pelvis", region: "torso", massKg: 22,
    shape: { kind: "box", halfExtents: P.torso.halfExtentsM },
    localOffset: { x: 0, y: P.pelvis.spineAnchorYM - P.torso.pelvisAnchorYM, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: P.pelvis.spineAnchorYM, z: 0 }, jointAnchorChild: { x: 0, y: P.torso.pelvisAnchorYM, z: 0 },
    jointLimitRadians: { x: 0.45, y: 0.55, z: 0.4 }, collisionGroup: 1,
  },
  {
    id: "neck", parent: "torso", region: null, massKg: 1,
    shape: { kind: "capsule", radius: P.neck.radiusM, halfHeight: P.neck.halfHeightM },
    localOffset: { x: 0, y: P.torso.neckAnchorYM + P.neck.anchorYM, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: P.torso.neckAnchorYM, z: 0 }, jointAnchorChild: { x: 0, y: -P.neck.anchorYM, z: 0 },
    jointLimitRadians: { x: 0.4, y: 0.6, z: 0.35 }, collisionGroup: 1,
  },
  {
    id: "head", parent: "neck", region: "head", massKg: 5,
    shape: { kind: "sphere", radius: P.head.radiusM },
    localOffset: { x: 0, y: P.neck.anchorYM + P.head.radiusM, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: P.neck.anchorYM, z: 0 }, jointAnchorChild: { x: 0, y: -P.head.radiusM, z: 0 },
    jointLimitRadians: { x: 0.55, y: 0.8, z: 0.45 }, collisionGroup: 1,
  },
  ...(["left", "right"] as const).flatMap((side) => {
    const sign = side === "left" ? -1 : 1;
    const upperId = `${side}UpperArm` as SegmentId;
    const foreId = `${side}Forearm` as SegmentId;
    const handId = `${side}Hand` as RegionId;
    return [
      {
        id: upperId, parent: "torso" as SegmentId, region: null, massKg: 2.2,
        shape: { kind: "capsule" as const, radius: P.arm.upperRadiusM, halfHeight: upperArmHalfLength - P.arm.upperRadiusM },
        localOffset: { x: sign * P.torso.shoulderAnchorXM, y: P.torso.shoulderAnchorYM - upperArmHalfLength, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: sign * P.torso.shoulderAnchorXM, y: P.torso.shoulderAnchorYM, z: 0 },
        jointAnchorChild: { x: 0, y: upperArmHalfLength, z: 0 },
        jointLimitRadians: { x: 1.55, y: 1.25, z: 1.55 }, collisionGroup: 1,
      },
      {
        id: foreId, parent: upperId, region: null, massKg: 1.5,
        shape: { kind: "capsule" as const, radius: P.arm.forearmRadiusM, halfHeight: forearmHalfLength - P.arm.forearmRadiusM },
        localOffset: { x: 0, y: -upperArmHalfLength - forearmHalfLength, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -upperArmHalfLength, z: 0 }, jointAnchorChild: { x: 0, y: forearmHalfLength, z: 0 },
        jointLimitRadians: { x: 2.35, y: 0.25, z: 0.25 }, collisionGroup: 1,
      },
      {
        id: handId, parent: foreId, region: handId, massKg: 0.6,
        shape: { kind: "box" as const, halfExtents: P.arm.handHalfExtentsM },
        localOffset: { x: 0, y: -forearmHalfLength - P.arm.handHalfExtentsM.y, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -forearmHalfLength, z: 0 }, jointAnchorChild: { x: 0, y: P.arm.handHalfExtentsM.y, z: 0 },
        jointLimitRadians: { x: 0.6, y: 0.45, z: 0.45 }, collisionGroup: 1,
      },
    ];
  }),
  ...(["left", "right"] as const).flatMap((side) => {
    const sign = side === "left" ? -1 : 1;
    const thighId = `${side}Thigh` as SegmentId;
    const shinId = `${side}Shin` as SegmentId;
    const footId = `${side}Foot` as RegionId;
    return [
      {
        id: thighId, parent: "pelvis" as SegmentId, region: null, massKg: 6.5,
        shape: { kind: "capsule" as const, radius: P.leg.thighRadiusM, halfHeight: thighHalfLength - P.leg.thighRadiusM },
        localOffset: { x: sign * P.pelvis.hipAnchorXM, y: P.pelvis.hipAnchorYM - thighHalfLength, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: sign * P.pelvis.hipAnchorXM, y: P.pelvis.hipAnchorYM, z: 0 }, jointAnchorChild: { x: 0, y: thighHalfLength, z: 0 },
        jointLimitRadians: { x: 1.35, y: 0.65, z: 0.65 }, collisionGroup: 1,
      },
      {
        id: shinId, parent: thighId, region: null, massKg: 4.3,
        shape: { kind: "capsule" as const, radius: P.leg.shinRadiusM, halfHeight: shinHalfLength - P.leg.shinRadiusM },
        localOffset: { x: 0, y: -thighHalfLength - shinHalfLength, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -thighHalfLength, z: 0 }, jointAnchorChild: { x: 0, y: shinHalfLength, z: 0 },
        jointLimitRadians: { x: 2.25, y: 0.12, z: 0.12 }, collisionGroup: 1,
      },
      {
        id: footId, parent: shinId, region: footId, massKg: 1,
        shape: { kind: "box" as const, halfExtents: P.foot.halfExtentsM },
        localOffset: { x: 0, y: -shinHalfLength - P.foot.halfExtentsM.y, z: -P.foot.ankleOffsetZM }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -shinHalfLength, z: 0 }, jointAnchorChild: { x: 0, y: P.foot.halfExtentsM.y, z: P.foot.ankleOffsetZM },
        jointLimitRadians: { x: 0.65, y: 0.35, z: 0.35 }, collisionGroup: 1,
      },
    ];
  }),
] satisfies ReadonlyArray<SegmentDefinition>;

export const SEGMENT_BY_ID = new Map(SEGMENTS.map((segment) => [segment.id, segment]));
export const REGION_TO_SEGMENT = new Map(
  SEGMENTS.filter((segment) => segment.region).map((segment) => [segment.region!, segment.id]),
);

export const REGION_COLORS: Readonly<Record<RegionId, string>> = {
  head: "#ffd166",
  torso: "#62d5ff",
  pelvis: "#7c83ff",
  leftHand: "#ff7a90",
  rightHand: "#ff7a90",
  leftFoot: "#80e7a8",
  rightFoot: "#80e7a8",
};

export const PASSIVE_COLOR = "#b8c5d9";
export const TOTAL_MASS_KG = SEGMENTS.reduce((sum, segment) => sum + segment.massKg, 0);
export const UNLIMITED_JOINT = noLimit;
