import type { RegionId, SegmentDefinition, SegmentId } from "./types";

const identity = { x: 0, y: 0, z: 0, w: 1 } as const;
const noLimit = { x: Math.PI, y: Math.PI, z: Math.PI } as const;

export const SEGMENTS: ReadonlyArray<SegmentDefinition> = [
  {
    id: "pelvis", parent: null, region: "pelvis", massKg: 12,
    shape: { kind: "box", halfExtents: { x: 0.22, y: 0.16, z: 0.13 } },
    localOffset: { x: 0, y: 1.03, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: null, jointAnchorChild: null, jointLimitRadians: null, collisionGroup: 1,
  },
  {
    id: "torso", parent: "pelvis", region: "torso", massKg: 22,
    shape: { kind: "box", halfExtents: { x: 0.25, y: 0.31, z: 0.14 } },
    localOffset: { x: 0, y: 0.39, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: 0.16, z: 0 }, jointAnchorChild: { x: 0, y: -0.23, z: 0 },
    jointLimitRadians: { x: 0.45, y: 0.55, z: 0.4 }, collisionGroup: 1,
  },
  {
    id: "neck", parent: "torso", region: null, massKg: 1,
    shape: { kind: "capsule", radius: 0.07, halfHeight: 0.05 },
    localOffset: { x: 0, y: 0.38, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: 0.31, z: 0 }, jointAnchorChild: { x: 0, y: -0.07, z: 0 },
    jointLimitRadians: { x: 0.4, y: 0.6, z: 0.35 }, collisionGroup: 1,
  },
  {
    id: "head", parent: "neck", region: "head", massKg: 5,
    shape: { kind: "sphere", radius: 0.15 },
    localOffset: { x: 0, y: 0.22, z: 0 }, restLocalRotation: identity,
    jointAnchorParent: { x: 0, y: 0.07, z: 0 }, jointAnchorChild: { x: 0, y: -0.15, z: 0 },
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
        shape: { kind: "capsule" as const, radius: 0.085, halfHeight: 0.14 },
        localOffset: { x: sign * 0.34, y: 0.04, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: sign * 0.25, y: 0.18, z: 0 },
        jointAnchorChild: { x: 0, y: 0.14, z: 0 },
        jointLimitRadians: { x: 1.55, y: 1.25, z: 1.55 }, collisionGroup: 1,
      },
      {
        id: foreId, parent: upperId, region: null, massKg: 1.5,
        shape: { kind: "capsule" as const, radius: 0.07, halfHeight: 0.13 },
        localOffset: { x: 0, y: -0.31, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -0.14, z: 0 }, jointAnchorChild: { x: 0, y: 0.13, z: 0 },
        jointLimitRadians: { x: 2.35, y: 0.25, z: 0.25 }, collisionGroup: 1,
      },
      {
        id: handId, parent: foreId, region: handId, massKg: 0.6,
        shape: { kind: "box" as const, halfExtents: { x: 0.075, y: 0.105, z: 0.045 } },
        localOffset: { x: 0, y: -0.255, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -0.13, z: 0 }, jointAnchorChild: { x: 0, y: 0.105, z: 0 },
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
        shape: { kind: "capsule" as const, radius: 0.105, halfHeight: 0.19 },
        localOffset: { x: sign * 0.13, y: -0.35, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: sign * 0.13, y: -0.16, z: 0 }, jointAnchorChild: { x: 0, y: 0.19, z: 0 },
        jointLimitRadians: { x: 1.35, y: 0.65, z: 0.65 }, collisionGroup: 1,
      },
      {
        id: shinId, parent: thighId, region: null, massKg: 4.3,
        shape: { kind: "capsule" as const, radius: 0.085, halfHeight: 0.18 },
        localOffset: { x: 0, y: -0.43, z: 0 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -0.19, z: 0 }, jointAnchorChild: { x: 0, y: 0.18, z: 0 },
        jointLimitRadians: { x: 2.25, y: 0.12, z: 0.12 }, collisionGroup: 1,
      },
      {
        id: footId, parent: shinId, region: footId, massKg: 1,
        shape: { kind: "box" as const, halfExtents: { x: 0.105, y: 0.065, z: 0.17 } },
        localOffset: { x: 0, y: -0.245, z: 0.075 }, restLocalRotation: identity,
        jointAnchorParent: { x: 0, y: -0.18, z: 0 }, jointAnchorChild: { x: 0, y: 0.065, z: -0.075 },
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
