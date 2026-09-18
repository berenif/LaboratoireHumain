import { readFile, writeFile } from "node:fs/promises";

async function replaceExact(path, before, after, label) {
  const source = await readFile(path, "utf8");
  if (source.includes(after)) {
    console.log(`${label} already present`);
    return;
  }
  if (!source.includes(before)) {
    throw new Error(`expected source for ${label} was not found`);
  }
  await writeFile(path, source.replace(before, after));
  console.log(`applied ${label}`);
}

const balancePath = new URL("../src/character/BalanceController.ts", import.meta.url);
const characterPath = new URL("../src/character/EmbodiedCharacter.ts", import.meta.url);

await replaceExact(
  balancePath,
  `      if (Math.abs(headingDelta) > 1e-8) {`,
  `      // Rebase only while the foot is travelling. Once the planned arc has
      // reached the floor, keep the landing target fixed in world space so a
      // loaded sole is not dragged by later torso yaw during touchdown proof.
      if (this.step.elapsed < this.step.duration && Math.abs(headingDelta) > 1e-8) {`,
  "post-arc landing target freeze",
);

await replaceExact(
  balancePath,
  `    const to = { ...add(from, travel), y: footCenterHeight(foot, floorY) - 0.04 };`,
  `    const to = { ...add(from, travel), y: footCenterHeight(foot, floorY) };`,
  "reachable landing center height",
);

await replaceExact(
  balancePath,
  `    this.step = { foot, from: { ...from }, to, elapsed: -0.50, duration };`,
  `    // Keep a brief load-transfer phase, but begin the swing before the
    // pelvis can outrun the bounded landing target under a sustained pull.
    this.step = { foot, from: { ...from }, to, elapsed: -0.18, duration };`,
  "bounded pre-swing load transfer",
);

await replaceExact(
  balancePath,
  `    const desiredAnkleCenter = this.step && stanceFoot
      // During swing, translate the COM over the retained stance anchor. The
      // new two-foot midpoint becomes valid only after measured touchdown.
      ? plannedAnkle(stanceFoot)
      : supportingFeet.length > 0`,
  `    const unloadingSwing = this.step !== null && this.step.elapsed < this.step.duration;
    const desiredAnkleCenter = unloadingSwing && stanceFoot
      // During the travelling arc, translate the COM over the retained stance
      // anchor. After the arc, measured loaded contacts own the support target
      // while touchdown persistence is being validated.
      ? plannedAnkle(stanceFoot)
      : supportingFeet.length > 0`,
  "measured touchdown COM handoff",
);

await replaceExact(
  balancePath,
  `      const activelySwinging = this.step?.foot === foot && this.step.elapsed >= 0;`,
  `      // Once the planned swing has finished, a measured loaded touchdown is
      // real support even while contact persistence is still being validated.
      const activelySwinging = this.step?.foot === foot
        && this.step.elapsed >= 0
        && this.step.elapsed < this.step.duration;`,
  "touchdown support eligibility",
);

await replaceExact(
  balancePath,
  `        supportCenter: { ...supportCenter }, supportingFeet: supportingFeet.filter(foot =>
          !(this.step?.foot === foot && this.step.elapsed >= 0)
        ), supportMarginM: supportMargin,`,
  `        supportCenter: { ...supportCenter }, supportingFeet: supportingFeet.filter(foot =>
          !(this.step?.foot === foot && this.step.elapsed >= 0
            && this.step.elapsed < this.step.duration)
        ), supportMarginM: supportMargin,`,
  "touchdown support diagnostics",
);

await replaceExact(
  balancePath,
  `    const correctingStep = this.step !== null;
    const shouldFall = !correctingStep
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);`,
  `    const correctingStep = this.step !== null;
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
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);`,
  "post-touchdown recovery guard",
);

await replaceExact(
  characterPath,
  `    const swingSide = this.step.foot === "leftFoot" ? "left" : "right";
    const supportBySide = new Map<"left" | "right", typeof this.lastContacts[number]>();`,
  `    const unloadedSwingSide = this.step.elapsed < this.step.duration
      ? (this.step.foot === "leftFoot" ? "left" : "right") : null;
    const supportBySide = new Map<"left" | "right", typeof this.lastContacts[number]>();`,
  "motor touchdown support phase",
);

await replaceExact(
  characterPath,
  `      if (!contact.loadBearing || !definition?.side || definition.side === swingSide
        || (definition.role !== "hindfoot" && definition.role !== "forefoot")) {`,
  `      if (!contact.loadBearing || !definition?.side || definition.side === unloadedSwingSide
        || (definition.role !== "hindfoot" && definition.role !== "forefoot")) {`,
  "motor touchdown contact acceptance",
);

await replaceExact(
  characterPath,
  `      const swingSide = this.step?.elapsed !== undefined && this.step.elapsed >= 0
        ? (this.step.foot === "leftFoot" ? "left" : "right") : null;`,
  `      const swingSide = this.step && this.step.elapsed >= 0
        && this.step.elapsed < this.step.duration
        ? (this.step.foot === "leftFoot" ? "left" : "right") : null;`,
  "gravity-compensation touchdown support",
);

await replaceExact(
  characterPath,
  `      if ((this.step?.foot === foot && this.step.elapsed >= 0) || this.activeGrab?.region === foot) return [];`,
  `      const activelySwinging = this.step?.foot === foot
        && this.step.elapsed >= 0
        && this.step.elapsed < this.step.duration;
      if (activelySwinging || this.activeGrab?.region === foot) return [];`,
  "support snapshot touchdown handoff",
);
