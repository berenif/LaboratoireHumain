import { readFile, writeFile } from "node:fs/promises";

const path = new URL("../src/character/BalanceController.ts", import.meta.url);
const before = `    const correctingStep = this.step !== null;
    const shouldFall = !correctingStep
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);`;
const after = `    const correctingStep = this.step !== null;
    // A measured touchdown deliberately enters a short double-support cooldown
    // before another step may start. Keep that bounded recovery opportunity
    // alive while at least one real support remains; torso lean, pelvis height,
    // and sustained support loss are still enforced by EmbodiedCharacter.
    const settlingAfterTouchdown = !correctingStep
      && this.stepCount > 0
      && this.cooldown > 0
      && supportingFeet.length > 0;
    const shouldFall = !correctingStep
      && !settlingAfterTouchdown
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);`;

const source = await readFile(path, "utf8");
if (source.includes(after)) {
  console.log("post-touchdown recovery guard already present");
} else if (!source.includes(before)) {
  throw new Error("expected BalanceController fall gate was not found");
} else {
  await writeFile(path, source.replace(before, after));
  console.log("applied post-touchdown recovery guard");
}
