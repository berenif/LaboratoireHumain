import {
  version as rapierVersion,
  type ImpulseJoint,
  type World,
} from "@dimforge/rapier3d-compat";
import type {
  JointAxisProfile,
  JointCoordinate,
  JointProfile,
} from "../core/types";

export const EXPECTED_RAPIER_VERSION = "0.20.0";

// RawJointAxis exists in the package's internal declarations but is not part
// of the public runtime export. Keep the numeric ABI in this one adapter.
const ANGULAR_AXIS = {
  x: 3,
  y: 4,
  z: 5,
} as const satisfies Readonly<Record<JointCoordinate, 3 | 4 | 5>>;

interface RawImpulseJointLimitApi020 {
  jointSetLimits(handle: number, axis: number, min: number, max: number): void;
  jointLimitsEnabled(handle: number, axis: number): boolean;
  jointLimitsMin(handle: number, axis: number): number;
  jointLimitsMax(handle: number, axis: number): number;
}

type JointHandle = number | Pick<ImpulseJoint, "handle">;
type WorldWithImpulseJoints = Pick<World, "impulseJoints">;

export interface RapierJointLimitReadback {
  readonly coordinate: JointCoordinate;
  readonly enabled: boolean;
  readonly minRadians: number;
  readonly maxRadians: number;
}

const REQUIRED_METHODS = [
  "jointSetLimits",
  "jointLimitsEnabled",
  "jointLimitsMin",
  "jointLimitsMax",
] as const satisfies readonly (keyof RawImpulseJointLimitApi020)[];

function handleOf(joint: JointHandle): number {
  const handle = typeof joint === "number" ? joint : joint.handle;
  // Rapier 0.20 encodes generational arena handles in an f64 bit pattern;
  // valid nonzero generations may therefore appear as tiny subnormal numbers.
  if (!Number.isFinite(handle) || handle < 0) {
    throw new RangeError(`Invalid Rapier impulse-joint handle: ${handle}.`);
  }
  return handle;
}

function validateAxis(axis: JointAxisProfile): void {
  if (!(axis.coordinate in ANGULAR_AXIS)) {
    throw new RangeError(`Unsupported Rapier angular coordinate: ${String(axis.coordinate)}.`);
  }
  if (!Number.isFinite(axis.minRadians) || !Number.isFinite(axis.maxRadians)) {
    throw new RangeError(`Joint ${axis.coordinate} limits must be finite.`);
  }
  if (axis.minRadians > axis.maxRadians) {
    throw new RangeError(`Joint ${axis.coordinate} minimum exceeds its maximum.`);
  }
}

function assertReadback(
  expected: Pick<JointAxisProfile, "coordinate" | "minRadians" | "maxRadians">,
  actual: RapierJointLimitReadback,
): void {
  const tolerance = 2e-6 * Math.max(
    1,
    Math.abs(expected.minRadians),
    Math.abs(expected.maxRadians),
  );
  if (!actual.enabled
    || Math.abs(actual.minRadians - expected.minRadians) > tolerance
    || Math.abs(actual.maxRadians - expected.maxRadians) > tolerance) {
    throw new Error(
      `Rapier rejected ${expected.coordinate} limits `
      + `[${expected.minRadians}, ${expected.maxRadians}]; read back `
      + `${actual.enabled ? "enabled" : "disabled"} `
      + `[${actual.minRadians}, ${actual.maxRadians}].`,
    );
  }
}

export function assertRapierJointLimitCompatibility(): void {
  const installed = rapierVersion();
  if (installed !== EXPECTED_RAPIER_VERSION) {
    throw new Error(
      `Unsupported Rapier version ${installed}; joint-limit adapter requires `
      + `${EXPECTED_RAPIER_VERSION}. Revalidate the raw angular-limit ABI before upgrading.`,
    );
  }
}

export class RapierJointLimitAdapter {
  private readonly raw: RawImpulseJointLimitApi020;

  constructor(world: WorldWithImpulseJoints) {
    assertRapierJointLimitCompatibility();
    const candidate = world.impulseJoints.raw as unknown as Partial<RawImpulseJointLimitApi020>;
    for (const method of REQUIRED_METHODS) {
      if (typeof candidate?.[method] !== "function") {
        throw new Error(`Rapier ${EXPECTED_RAPIER_VERSION} raw joint API is missing ${method}().`);
      }
    }
    this.raw = candidate as RawImpulseJointLimitApi020;
  }

  read(joint: JointHandle, coordinate: JointCoordinate): RapierJointLimitReadback {
    const handle = handleOf(joint);
    const axis = ANGULAR_AXIS[coordinate];
    return {
      coordinate,
      enabled: this.raw.jointLimitsEnabled(handle, axis),
      minRadians: this.raw.jointLimitsMin(handle, axis),
      maxRadians: this.raw.jointLimitsMax(handle, axis),
    };
  }

  constrainAxis(
    joint: JointHandle,
    axisProfile: JointAxisProfile,
  ): RapierJointLimitReadback {
    validateAxis(axisProfile);
    const handle = handleOf(joint);
    this.raw.jointSetLimits(
      handle,
      ANGULAR_AXIS[axisProfile.coordinate],
      axisProfile.minRadians,
      axisProfile.maxRadians,
    );
    const result = this.read(handle, axisProfile.coordinate);
    assertReadback(axisProfile, result);
    return result;
  }

  constrain(
    joint: JointHandle,
    profile: JointProfile,
  ): readonly RapierJointLimitReadback[] {
    if (profile.kind === "hinge" && profile.axes.length !== 1) {
      throw new RangeError("A hinge joint profile must permit exactly one angular coordinate.");
    }
    if (profile.axes.length === 0 || profile.axes.length > 3) {
      throw new RangeError("A joint profile must permit between one and three angular coordinates.");
    }
    if (!Number.isFinite(profile.limitSoftZoneFraction)
      || profile.limitSoftZoneFraction < 0
      || profile.limitSoftZoneFraction > 1) {
      throw new RangeError("Joint limitSoftZoneFraction must be between zero and one.");
    }
    const seen = new Set<JointCoordinate>();
    for (const axis of profile.axes) {
      if (seen.has(axis.coordinate)) {
        throw new RangeError(`Joint profile repeats the ${axis.coordinate} coordinate.`);
      }
      seen.add(axis.coordinate);
      validateAxis(axis);
    }
    // Validate the complete profile before mutating the raw joint.
    return profile.axes.map((axis) => this.constrainAxis(joint, axis));
  }
}

export function createRapierJointLimitAdapter(
  world: WorldWithImpulseJoints,
): RapierJointLimitAdapter {
  return new RapierJointLimitAdapter(world);
}

/** Convenience helper for one-off construction paths. */
export function constrainRapierJoint(
  world: WorldWithImpulseJoints,
  joint: JointHandle,
  profile: JointProfile,
): readonly RapierJointLimitReadback[] {
  return createRapierJointLimitAdapter(world).constrain(joint, profile);
}
