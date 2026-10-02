import { SEGMENT_BY_ID } from "../core/humanoid";
import type { SegmentId, SupportingContact, Vec3 } from "../core/types";
import { add, clampLength, scale, sub } from "./math";

export interface SupportState {
  schema: 1;
  supportTarget: Vec3;
  contactRevision: number;
  contactKeys: SegmentId[];
  previousAllocation: { segment: SegmentId; force: Vec3 }[];
  retainedSide: "left" | "right" | null;
  transferAgeS: number;
  measuredReadiness: { ageS: number; retainedN: number; movingN: number; ready: boolean };
  stanceRevision: number;
  transitionReason: string | null;
}
export const createSupportState = (target: Vec3): SupportState => ({ schema: 1,
  supportTarget: { ...target }, contactRevision: 0, contactKeys: [], previousAllocation: [],
  retainedSide: null, transferAgeS: 0, measuredReadiness: { ageS: 0, retainedN: 0, movingN: 0, ready: false },
  stanceRevision: 0, transitionReason: null });

export function soleContacts(contacts: readonly SupportingContact[]): SupportingContact[] {
  return contacts.filter(c => c.loadBearing && c.normalY >= 0.65 && (c.measuredForceN ?? c.forceN) >= 3
    && !c.sleepingEquilibrium
    && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(c.segment)?.role ?? ""))
    .sort((a, b) => a.segment.localeCompare(b.segment));
}

/** Pure, serializable transition. Forecast callers receive their own copy.
 * Readiness is ALWAYS sourced from the supplied measurements, never allocation. */
export function updateSupportState(previous: SupportState, input: {
  contacts: readonly SupportingContact[]; target: Vec3; retainedSide: "left" | "right" | null;
  transferring: boolean; dt: number; speedMps: number; weightN: number; stanceRevision?: number;
}): SupportState {
  const state = structuredClone(previous), contacts = soleContacts(input.contacts);
  const keys = [...new Set(contacts.map(c => c.segment))];
  const changed = keys.join() !== state.contactKeys.join();
  if (changed) { state.contactRevision++; state.transitionReason = "contact-change"; }
  state.contactKeys = keys;
  state.previousAllocation = state.previousAllocation.filter(a => keys.includes(a.segment));
  const reversed = state.retainedSide !== input.retainedSide;
  const stanceChanged = input.stanceRevision !== undefined && input.stanceRevision !== state.stanceRevision;
  if (reversed || stanceChanged) {
    state.transferAgeS = 0; state.measuredReadiness.ageS = 0;
    state.transitionReason = reversed ? "retained-side-change" : "stance-change";
  }
  state.retainedSide = input.retainedSide;
  state.stanceRevision = input.stanceRevision ?? state.stanceRevision;
  const valid = [input.dt, input.speedMps, input.weightN, ...Object.values(input.target)].every(Number.isFinite)
    && input.dt >= 0 && input.dt <= 1 / 60 + 1e-9 && input.weightN > 0
    && contacts.every(c => Number.isFinite(c.forceN) && Object.values(c.point).every(Number.isFinite));
  if (!valid) {
    state.measuredReadiness = { ageS: 0, retainedN: 0, movingN: 0, ready: false };
    state.transitionReason = "invalid-state"; return state;
  }
  state.supportTarget = input.transferring
    ? add(state.supportTarget, clampLength(sub(input.target, state.supportTarget), input.speedMps * input.dt))
    : { ...input.target };
  state.transferAgeS = input.transferring ? state.transferAgeS + input.dt : 0;
  const retainedN = contacts.reduce((sum, c) => sum
    + (SEGMENT_BY_ID.get(c.segment)?.side === input.retainedSide && c.persistenceS >= 0.05 ? (c.measuredForceN ?? c.forceN) * c.normalY : 0), 0);
  const movingN = input.contacts.reduce((sum, c) => sum
    + (SEGMENT_BY_ID.get(c.segment)?.side !== input.retainedSide && c.normalY >= 0.65
      && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(c.segment)?.role ?? "")
      ? Math.max(0, c.measuredForceN ?? c.forceN) * c.normalY : 0), 0);
  const qualified = input.transferring && input.retainedSide !== null
    && retainedN >= 0.52 * input.weightN && movingN <= 0.25 * input.weightN;
  const ageS = qualified ? state.measuredReadiness.ageS + input.dt : 0;
  state.measuredReadiness = { ageS, retainedN, movingN, ready: ageS + 1e-9 >= 0.10 };
  if (!qualified && previous.measuredReadiness.ageS > 0) state.transitionReason = "readiness-interrupted";
  return state;
}

/** Conservative held-load forecast. No assumed future rise to 52%, contact
 * invention, or live timer mutation. Validation is required before admission. */
export function forecastSupport(previous: SupportState, input: Parameters<typeof updateSupportState>[1]
  & { position: Vec3; velocity: Vec3; externalForce: Vec3; massKg: number;
    allocatedForce: Vec3; allocationFeasible: boolean; horizonS: number }) {
  let state = structuredClone(previous), position = { ...input.position }, velocity = { ...input.velocity };
  let readinessTimeS: number | null = null;
  const count = Math.ceil(input.horizonS * 60);
  if (!input.allocationFeasible || !Number.isFinite(count) || count < 0 || count > 102
    || !Number.isFinite(input.massKg) || input.massKg <= 0
    || ![input.position, input.velocity, input.externalForce, input.allocatedForce].every(v => Object.values(v).every(Number.isFinite))) {
    return { valid: false, reason: "infeasible-request", state, position, velocity, readinessTimeS };
  }
  for (let tick = 0; tick < count; tick++) {
    const dt = Math.min(1 / 60, input.horizonS - tick / 60);
    state = updateSupportState(state, { ...input, dt });
    if (state.transitionReason === "invalid-state") return { valid: false, reason: "invalid-state", state, position, velocity, readinessTimeS: null };
    if (state.measuredReadiness.ready) readinessTimeS ??= (tick + 1) / 60;
    const acceleration = scale(add(input.allocatedForce, input.externalForce), 1 / input.massKg);
    velocity = add(velocity, scale({ ...acceleration, y: 0 }, dt));
    position = add(position, scale(velocity, dt));
  }
  return { valid: true, reason: readinessTimeS === null ? "measured-load-not-ready" : null,
    state, position, velocity, readinessTimeS };
}
