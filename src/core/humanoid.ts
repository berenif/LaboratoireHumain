import { createEllipsoidGeometry, createTaperedPrismGeometry } from "./geometry";
import {
  REGION_IDS,
  type JointAxisProfile,
  type JointCoordinate,
  type JointProfile,
  type Quat,
  type RegionId,
  type SegmentDefinition,
  type SegmentId,
  type Vec3,
} from "./types";

const identity = Object.freeze({ x: 0, y: 0, z: 0, w: 1 });
/** Body +X right, +Y up, +Z forward. Positive anatomical flexion at the
 * shoulder, elbow and hip rotates a hanging limb toward +Z. A shared 180-degree
 * Y basis in BOTH joint frames reverses X and Z without changing the rest pose.
 * Knees keep +X: knee flexion moves the distal leg backward, unlike the elbow.
 */
export const BODY_FRAME = Object.freeze({
  right: Object.freeze({ x: 1, y: 0, z: 0 }),
  up: Object.freeze({ x: 0, y: 1, z: 0 }),
  forward: Object.freeze({ x: 0, y: 0, z: 1 }),
});
const forwardFlexionFrame: Quat = Object.freeze({ x: 0, y: 1, z: 0, w: 0 });
const radians = (degrees: number): number => degrees * Math.PI / 180;

/** Adult body dimensions in metres, shared by anatomy, pose targets, and tests. */
export const HUMAN_PROPORTIONS = {
  totalHeightM: 1.84,
  pelvis: {
    centerHeightM: 0.99,
    halfExtentsM: { x: 0.17, y: 0.13, z: 0.11 },
    spineAnchorYM: 0.13,
    hipAnchorXM: 0.09,
    hipAnchorYM: -0.08,
  },
  lumbar: {
    halfHeightM: 0.06,
    radiusXM: 0.16,
    radiusZM: 0.10,
  },
  torso: {
    halfExtentsM: { x: 0.225, y: 0.15, z: 0.11 },
    pelvisAnchorYM: -0.15,
    lumbarAnchorYM: -0.15,
    neckAnchorYM: 0.15,
    shoulderInnerAnchorXM: 0.10,
    shoulderAnchorXM: 0.25,
    shoulderAnchorYM: 0.10,
  },
  shoulderGirdle: {
    // Inner socket at 0.10 m plus two half-lengths puts the humeral head
    // at 0.25 m, clear of the ribcage without disabling arm/trunk contact.
    halfLengthM: 0.075,
    radiusYM: 0.045,
    radiusZM: 0.055,
  },
  neck: { radiusM: 0.05, halfHeightM: 0.035, anchorYM: 0.035 },
  head: { radiusM: 0.115, radiusXM: 0.10, radiusZM: 0.105 },
  arm: {
    upperLengthM: 0.31,
    upperRadiusM: 0.055,
    forearmLengthM: 0.27,
    proximalForearmLengthM: 0.135,
    distalForearmLengthM: 0.135,
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
  ankle: { radiusXM: 0.065, halfHeightM: 0.035, radiusZM: 0.055 },
  foot: {
    halfExtentsM: { x: 0.05, y: 0.045, z: 0.135 },
    hindfootCenterZM: 0.035,
    hindfootBackZM: -0.075,
    hindfootFrontZM: 0.075,
    forefootCenterZM: 0.17,
    forefootHalfLengthM: 0.06,
    forefootJointZM: 0.11,
    ankleOffsetZM: -0.085,
    // A corrective shuffle only needs enough clearance to release the sole.
    // Keeping this proportional prevents short balance steps from turning into
    // high, slow kicks that cannot re-establish a loaded contact patch.
    stepClearanceBaseM: 0.02,
    stepClearancePerTravel: 0.08,
    minStepClearanceM: 0.025,
    maxStepClearanceM: 0.07,
  },
  stance: { neutralKneeFlexion: 0.12 },
} as const;

const P = HUMAN_PROPORTIONS;
const upperArmHalfLength = P.arm.upperLengthM / 2;
const proximalForearmHalfLength = P.arm.proximalForearmLengthM / 2;
const distalForearmHalfLength = P.arm.distalForearmLengthM / 2;
const thighHalfLength = P.leg.thighLengthM / 2;
const shinHalfLength = P.leg.shinLengthM / 2;

function axis(
  coordinate: JointCoordinate,
  minDegrees: number,
  maxDegrees: number,
  passiveStiffnessNmPerRad: number,
  dampingNmsPerRad: number,
  maxMotorTorqueNm: number,
): JointAxisProfile {
  return Object.freeze({
    coordinate,
    minRadians: radians(minDegrees),
    maxRadians: radians(maxDegrees),
    passiveStiffnessNmPerRad,
    dampingNmsPerRad,
    maxMotorTorqueNm,
  });
}

function joint(
  kind: JointProfile["kind"],
  parentAnchor: Vec3,
  childAnchor: Vec3,
  axes: readonly JointAxisProfile[],
  frameRotation: Quat = identity,
): JointProfile {
  const frozenParentAnchor = Object.freeze({ ...parentAnchor });
  const frozenChildAnchor = Object.freeze({ ...childAnchor });
  return Object.freeze({
    kind,
    parentFrame: Object.freeze({ anchor: frozenParentAnchor, rotation: frameRotation }),
    childFrame: Object.freeze({ anchor: frozenChildAnchor, rotation: frameRotation }),
    axes: Object.freeze([...axes]),
    limitSoftZoneFraction: 0.035,
  });
}

type SegmentInput = Omit<
  SegmentDefinition,
  "jointAnchorParent" | "jointAnchorChild" | "collisionExclusions" | "collisionGroup"
> & { collisionExclusions?: readonly SegmentId[] };

function segment(input: SegmentInput): SegmentDefinition {
  return {
    ...input,
    jointAnchorParent: input.jointProfile?.parentFrame.anchor ?? null,
    jointAnchorChild: input.jointProfile?.childFrame.anchor ?? null,
    collisionExclusions: input.collisionExclusions ?? [],
    collisionGroup: 1,
  };
}

const pelvisGeometry = createEllipsoidGeometry(P.pelvis.halfExtentsM, {
  radialSegments: 14, latitudeSegments: 8, bottomScale: 0.82, topScale: 1.04,
});
const lumbarGeometry = createEllipsoidGeometry(
  { x: P.lumbar.radiusXM, y: P.lumbar.halfHeightM, z: P.lumbar.radiusZM },
  { radialSegments: 14, latitudeSegments: 6, bottomScale: 1.04, topScale: 0.92 },
);
const torsoGeometry = createEllipsoidGeometry(P.torso.halfExtentsM, {
  radialSegments: 16, latitudeSegments: 9, bottomScale: 0.72, topScale: 1.04,
});
const neckGeometry = createEllipsoidGeometry(
  { x: P.neck.radiusM, y: P.neck.halfHeightM, z: P.neck.radiusM * 0.88 },
  { radialSegments: 10, latitudeSegments: 5, bottomScale: 0.92, topScale: 0.86 },
);
const headGeometry = createEllipsoidGeometry(
  { x: P.head.radiusXM, y: P.head.radiusM, z: P.head.radiusZM },
  { radialSegments: 16, latitudeSegments: 10, bottomScale: 0.82, topScale: 0.96 },
);
const shoulderGirdleGeometry = createEllipsoidGeometry(
  { x: P.shoulderGirdle.halfLengthM, y: P.shoulderGirdle.radiusYM, z: P.shoulderGirdle.radiusZM },
  { radialSegments: 10, latitudeSegments: 6, bottomScale: 0.95, topScale: 1.02 },
);
const upperArmGeometry = createEllipsoidGeometry(
  { x: P.arm.upperRadiusM, y: upperArmHalfLength, z: P.arm.upperRadiusM },
  { radialSegments: 10, latitudeSegments: 7, bottomScale: 0.78, topScale: 1.05 },
);
const proximalForearmGeometry = createEllipsoidGeometry(
  { x: P.arm.forearmRadiusM, y: proximalForearmHalfLength, z: P.arm.forearmRadiusM },
  { radialSegments: 10, latitudeSegments: 6, bottomScale: 0.88, topScale: 1.06 },
);
const distalForearmGeometry = createEllipsoidGeometry(
  { x: P.arm.forearmRadiusM * 0.88, y: distalForearmHalfLength, z: P.arm.forearmRadiusM * 0.88 },
  { radialSegments: 10, latitudeSegments: 6, bottomScale: 0.78, topScale: 1.02 },
);
const handGeometry = createEllipsoidGeometry(
  { x: P.arm.handHalfExtentsM.x, y: P.arm.handHalfExtentsM.y, z: P.arm.handHalfExtentsM.z },
  { radialSegments: 10, latitudeSegments: 7, bottomScale: 0.72, topScale: 0.98 },
);
const thighGeometry = createEllipsoidGeometry(
  { x: P.leg.thighRadiusM, y: thighHalfLength, z: P.leg.thighRadiusM },
  { radialSegments: 12, latitudeSegments: 8, bottomScale: 0.75, topScale: 1.05 },
);
const shinGeometry = createEllipsoidGeometry(
  { x: P.leg.shinRadiusM, y: shinHalfLength, z: P.leg.shinRadiusM },
  { radialSegments: 12, latitudeSegments: 8, bottomScale: 0.72, topScale: 1.05 },
);
const ankleGeometry = createEllipsoidGeometry(
  { x: P.ankle.radiusXM, y: P.ankle.halfHeightM, z: P.ankle.radiusZM },
  { radialSegments: 10, latitudeSegments: 5, bottomScale: 0.92, topScale: 0.98 },
);
const hindfootGeometry = createTaperedPrismGeometry(
  [
    { z: P.foot.hindfootBackZM, halfWidth: 0.044 },
    { z: 0, halfWidth: 0.053 },
    { z: P.foot.hindfootFrontZM, halfWidth: 0.052 },
  ],
  -P.foot.halfExtentsM.y,
  P.foot.halfExtentsM.y,
);
const forefootGeometry = createTaperedPrismGeometry(
  [
    { z: -P.foot.forefootHalfLengthM, halfWidth: 0.052 },
    { z: 0, halfWidth: 0.054 },
    { z: P.foot.forefootHalfLengthM, halfWidth: 0.045 },
  ],
  -P.foot.halfExtentsM.y,
  P.foot.halfExtentsM.y * 0.78,
);

const rawSegments: SegmentDefinition[] = [
  segment({
    id: "pelvis", parent: null, region: "pelvis", side: null, role: "pelvis", massKg: 12,
    geometry: pelvisGeometry,
    localOffset: { x: 0, y: P.pelvis.centerHeightM, z: 0 }, restLocalRotation: identity,
    jointProfile: null,
  }),
  segment({
    id: "lumbar", parent: "pelvis", region: "torso", side: null, role: "lumbar", massKg: 6,
    geometry: lumbarGeometry,
    localOffset: { x: 0, y: P.pelvis.spineAnchorYM + P.lumbar.halfHeightM, z: 0 }, restLocalRotation: identity,
    jointProfile: joint(
      "multi-axis",
      { x: 0, y: P.pelvis.spineAnchorYM, z: 0 },
      { x: 0, y: -P.lumbar.halfHeightM, z: 0 },
      [axis("x", -10, 18, 120, 10, 100), axis("y", -8, 8, 80, 8, 70), axis("z", -10, 10, 100, 9, 90)],
    ),
  }),
  segment({
    id: "torso", parent: "lumbar", region: "torso", side: null, role: "ribcage", massKg: 16,
    geometry: torsoGeometry,
    localOffset: { x: 0, y: P.lumbar.halfHeightM - P.torso.lumbarAnchorYM, z: 0 }, restLocalRotation: identity,
    jointProfile: joint(
      "multi-axis",
      { x: 0, y: P.lumbar.halfHeightM, z: 0 },
      { x: 0, y: P.torso.lumbarAnchorYM, z: 0 },
      [axis("x", -10, 15, 100, 9, 90), axis("y", -12, 12, 70, 7, 60), axis("z", -12, 12, 90, 8, 75)],
    ),
  }),
  segment({
    id: "neck", parent: "torso", region: null, side: null, role: "neck", massKg: 1,
    geometry: neckGeometry,
    localOffset: { x: 0, y: P.torso.neckAnchorYM + P.neck.anchorYM, z: 0 }, restLocalRotation: identity,
    jointProfile: joint(
      "multi-axis",
      { x: 0, y: P.torso.neckAnchorYM, z: 0 },
      { x: 0, y: -P.neck.anchorYM, z: 0 },
      [axis("x", -15, 20, 20, 2.5, 24), axis("y", -25, 25, 15, 1.8, 15), axis("z", -15, 15, 18, 2, 18)],
    ),
  }),
  segment({
    id: "head", parent: "neck", region: "head", side: null, role: "head", massKg: 5,
    geometry: headGeometry,
    localOffset: { x: 0, y: P.neck.anchorYM + P.head.radiusM, z: 0 }, restLocalRotation: identity,
    jointProfile: joint(
      "multi-axis",
      { x: 0, y: P.neck.anchorYM, z: 0 },
      { x: 0, y: -P.head.radiusM, z: 0 },
      [axis("x", -25, 25, 15, 1.8, 18), axis("y", -35, 35, 10, 1.2, 12), axis("z", -15, 15, 12, 1.5, 14)],
    ),
  }),
  ...(["left", "right"] as const).flatMap((side) => {
    const sign = side === "left" ? -1 : 1;
    const girdleId = `${side}ShoulderGirdle` as SegmentId;
    const upperId = `${side}UpperArm` as SegmentId;
    const forearmId = `${side}Forearm` as SegmentId;
    const twistId = `${side}ForearmTwist` as SegmentId;
    const handId = `${side}Hand` as SegmentId;
    const handRegion = handId as RegionId;
    const shoulderYaw: [number, number] = side === "left" ? [-55, 65] : [-65, 55];
    // Canonical forward-flexion frames reverse Z as well as X. These scalar
    // intervals represent the same physical limits as main's identity-frame
    // [-100, 20] / [-20, 100], not a second lateral sign correction.
    const shoulderLateral: [number, number] = side === "left" ? [-20, 100] : [-100, 20];
    const wristDeviation: [number, number] = side === "left" ? [-15, 30] : [-30, 15];
    return [
      segment({
        id: girdleId, parent: "torso", region: "torso", side, role: "shoulder-girdle", massKg: 0.4,
        geometry: shoulderGirdleGeometry,
        localOffset: {
          x: sign * (P.torso.shoulderInnerAnchorXM + P.shoulderGirdle.halfLengthM),
          y: P.torso.shoulderAnchorYM,
          z: 0,
        },
        restLocalRotation: identity,
        jointProfile: joint(
          "multi-axis",
          { x: sign * P.torso.shoulderInnerAnchorXM, y: P.torso.shoulderAnchorYM, z: 0 },
          { x: -sign * P.shoulderGirdle.halfLengthM, y: 0, z: 0 },
          [axis("x", -10, 30, 35, 3, 32),
            axis("y", side === "left" ? -15 : -20, side === "left" ? 20 : 15, 30, 2.5, 28),
            axis("z", side === "left" ? -20 : -8, side === "left" ? 8 : 20, 35, 3, 30)],
        ),
      }),
      segment({
        id: upperId, parent: girdleId, region: null, side, role: "upper-arm", massKg: 1.8,
        // Main's wider shoulder socket already removes the axillary overlap.
        // Preserve that shared geometry instead of additionally clipping the arm.
        geometry: upperArmGeometry,
        localOffset: { x: sign * P.shoulderGirdle.halfLengthM, y: -upperArmHalfLength, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "multi-axis",
          { x: sign * P.shoulderGirdle.halfLengthM, y: 0, z: 0 },
          { x: 0, y: upperArmHalfLength, z: 0 },
          [
            axis("x", -35, 120, 60, 5, 70),
            axis("y", shoulderYaw[0], shoulderYaw[1], 40, 4, 45),
            axis("z", shoulderLateral[0], shoulderLateral[1], 55, 5, 65),
          ],
          forwardFlexionFrame,
        ),
      }),
      segment({
        id: forearmId, parent: upperId, region: null, side, role: "forearm", massKg: 0.9,
        geometry: proximalForearmGeometry,
        localOffset: { x: 0, y: -upperArmHalfLength - proximalForearmHalfLength, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "hinge",
          { x: 0, y: -upperArmHalfLength, z: 0 },
          { x: 0, y: proximalForearmHalfLength, z: 0 },
          [axis("x", 0, 145, 80, 5, 55)],
          forwardFlexionFrame,
        ),
      }),
      segment({
        id: twistId, parent: forearmId, region: null, side, role: "forearm-twist", massKg: 0.6,
        geometry: distalForearmGeometry,
        localOffset: { x: 0, y: -proximalForearmHalfLength - distalForearmHalfLength, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "hinge",
          { x: 0, y: -proximalForearmHalfLength, z: 0 },
          { x: 0, y: distalForearmHalfLength, z: 0 },
          [axis("y", -80, 80, 20, 1.5, 15)],
        ),
      }),
      segment({
        id: handId, parent: twistId, region: handRegion, side, role: "hand", massKg: 0.6,
        geometry: handGeometry,
        localOffset: { x: 0, y: -distalForearmHalfLength - P.arm.handHalfExtentsM.y, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "multi-axis",
          { x: 0, y: -distalForearmHalfLength, z: 0 },
          { x: 0, y: P.arm.handHalfExtentsM.y, z: 0 },
          [axis("x", -45, 55, 25, 2, 18), axis("z", wristDeviation[0], wristDeviation[1], 20, 1.5, 14)],
        ),
      }),
    ];
  }),
  ...(["left", "right"] as const).flatMap((side) => {
    const sign = side === "left" ? -1 : 1;
    const thighId = `${side}Thigh` as SegmentId;
    const shinId = `${side}Shin` as SegmentId;
    const ankleId = `${side}Ankle` as SegmentId;
    const footId = `${side}Foot` as SegmentId;
    const forefootId = `${side}Forefoot` as SegmentId;
    const footRegion = footId as RegionId;
    const hipYaw: [number, number] = side === "left" ? [-30, 40] : [-40, 30];
    const hipLateral: [number, number] = side === "left" ? [-20, 40] : [-40, 20];
    const footTilt: [number, number] = side === "left" ? [-10, 25] : [-25, 10];
    return [
      segment({
        id: thighId, parent: "pelvis", region: null, side, role: "thigh", massKg: 6.5,
        geometry: thighGeometry,
        localOffset: { x: sign * P.pelvis.hipAnchorXM, y: P.pelvis.hipAnchorYM - thighHalfLength, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "multi-axis",
          { x: sign * P.pelvis.hipAnchorXM, y: P.pelvis.hipAnchorYM, z: 0 },
          { x: 0, y: thighHalfLength, z: 0 },
          [axis("x", -20, 110, 160, 12, 150), axis("y", hipYaw[0], hipYaw[1], 100, 9, 100), axis("z", hipLateral[0], hipLateral[1], 140, 10, 125)],
          forwardFlexionFrame,
        ),
      }),
      segment({
        id: shinId, parent: thighId, region: null, side, role: "shin", massKg: 4.3,
        geometry: shinGeometry,
        localOffset: { x: 0, y: -thighHalfLength - shinHalfLength, z: 0 },
        restLocalRotation: identity,
        jointProfile: joint(
          "hinge",
          { x: 0, y: -thighHalfLength, z: 0 },
          { x: 0, y: shinHalfLength, z: 0 },
          [axis("x", 0, 140, 180, 10, 165)],
        ),
        // The compact ankle/forefoot housings overlap the distal shin in the
        // neutral stance. They are intentional joint-neighbour overlaps, not
        // self-contact; leaving the forefoot active here kicks the entire leg
        // on the first solver step.
        collisionExclusions: [footId, forefootId],
      }),
      segment({
        id: ankleId, parent: shinId, region: footRegion, side, role: "ankle", massKg: 0.2,
        geometry: ankleGeometry,
        localOffset: { x: 0, y: -shinHalfLength, z: 0 }, restLocalRotation: identity,
        jointProfile: joint(
          "hinge",
          { x: 0, y: -shinHalfLength, z: 0 },
          { x: 0, y: 0, z: 0 },
          [axis("x", -45, 20, 120, 7, 110)],
          forwardFlexionFrame,
        ),
      }),
      segment({
        id: footId, parent: ankleId, region: footRegion, side, role: "hindfoot", massKg: 0.55,
        geometry: hindfootGeometry,
        localOffset: { x: 0, y: -P.foot.halfExtentsM.y, z: P.foot.hindfootCenterZM },
        restLocalRotation: identity,
        jointProfile: joint(
          "multi-axis",
          { x: 0, y: 0, z: 0 },
          { x: 0, y: P.foot.halfExtentsM.y, z: -P.foot.hindfootCenterZM },
          [axis("z", footTilt[0], footTilt[1], 60, 4, 50)],
        ),
        collisionExclusions: [shinId],
      }),
      segment({
        id: forefootId, parent: footId, region: footRegion, side, role: "forefoot", massKg: 0.25,
        geometry: forefootGeometry,
        localOffset: {
          x: 0,
          y: 0,
          z: P.foot.forefootCenterZM - P.foot.hindfootCenterZM,
        },
        restLocalRotation: identity,
        jointProfile: joint(
          "hinge",
          { x: 0, y: -0.015, z: P.foot.forefootJointZM - P.foot.hindfootCenterZM },
          { x: 0, y: -0.015, z: P.foot.forefootJointZM - P.foot.forefootCenterZM },
          [axis("x", -20, 45, 45, 3, 35)],
          forwardFlexionFrame,
        ),
      }),
    ];
  }),
];

// Every direct joint pair is excluded, plus the non-adjacent pairs whose
// simplified joint housings intentionally overlap around ankles. Upper-arm vs
// ribcage remains enabled: only the arm's direct shoulder-girdle joint is excluded.
const exclusionSets = new Map<SegmentId, Set<SegmentId>>(
  rawSegments.map(({ id, collisionExclusions }) => [id, new Set(collisionExclusions)]),
);
for (const definition of rawSegments) {
  if (!definition.parent) continue;
  exclusionSets.get(definition.id)!.add(definition.parent);
  exclusionSets.get(definition.parent)!.add(definition.id);
}

export const SEGMENTS: readonly SegmentDefinition[] = Object.freeze(rawSegments.map((definition) =>
  Object.freeze({
    ...definition,
    collisionExclusions: Object.freeze([...exclusionSets.get(definition.id)!]),
  }),
));

export const SEGMENT_BY_ID: ReadonlyMap<SegmentId, SegmentDefinition> = new Map(
  SEGMENTS.map((definition) => [definition.id, definition]),
);

export const SEGMENTS_BY_REGION: ReadonlyMap<RegionId, readonly SegmentId[]> = new Map(
  REGION_IDS.map((region) => [
    region,
    Object.freeze(SEGMENTS.filter((definition) => definition.region === region).map(({ id }) => id)),
  ]),
);

export const PRIMARY_SEGMENT_BY_REGION: ReadonlyMap<RegionId, SegmentId> = new Map([
  ["head", "head"],
  ["torso", "torso"],
  ["pelvis", "pelvis"],
  ["leftHand", "leftHand"],
  ["rightHand", "rightHand"],
  ["leftFoot", "leftFoot"],
  ["rightFoot", "rightFoot"],
]);

/** @deprecated Use PRIMARY_SEGMENT_BY_REGION or SEGMENTS_BY_REGION. */
export const REGION_TO_SEGMENT = PRIMARY_SEGMENT_BY_REGION;

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
export const TOTAL_MASS_KG = SEGMENTS.reduce((sum, definition) => sum + definition.massKg, 0);

/** @deprecated Joint profiles now carry asymmetric per-coordinate limits. */
export const UNLIMITED_JOINT = Object.freeze({ x: Math.PI, y: Math.PI, z: Math.PI });
