import type { CharacterInitialOptions, StandingControllerId } from "./EmbodiedCharacter";

/** Shared explicit selection for the application and acceptance harness. */
export function coordinatedStandingOptions(id: string, forwardOffsetM = 0): CharacterInitialOptions {
  if (!["legacy", "h74-v1", "h77-v1", "h78-v1", "h79-v1", "h80-v1"].includes(id)
    || ![0, -0.005, 0.005].includes(forwardOffsetM)) throw new Error("Unknown standing experiment configuration");
  return { standingCandidate: { controllerId: id as StandingControllerId, forwardOffsetM } };
}
