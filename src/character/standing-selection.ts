import type { CharacterInitialOptions } from "./EmbodiedCharacter";

/** Shared explicit selection for the application and acceptance harness. */
export function coordinatedStandingOptions(id: string, forwardOffsetM = 0): CharacterInitialOptions {
  if (id !== "h74-v1" || ![0, -0.005, 0.005].includes(forwardOffsetM)) throw new Error("Unknown standing experiment configuration");
  return { standingCandidate: { forwardOffsetM } };
}
