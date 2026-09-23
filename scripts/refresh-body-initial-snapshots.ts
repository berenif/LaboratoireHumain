/** Explicit fixture migration: preserve historical inputs, refresh only assembly snapshots. */
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { createEmbodiedCharacter } from "../src/character/EmbodiedCharacter";

const file = new URL("./fixtures/native-fixed-streams.json", import.meta.url);
const capture = JSON.parse(readFileSync(file, "utf8"));
if (capture.schema !== 2 || capture.fixtures.length !== 2) throw new Error("Unexpected native capture schema");
const historicalData = () => JSON.stringify(capture.fixtures.map((fixture: {
  updates: unknown; recordedTransfers: unknown;
}) => ({ updates: fixture.updates, recordedTransfers: fixture.recordedTransfers })));
const historicalBefore = historicalData();
for (const fixture of capture.fixtures) {
  const renderer = fixture.id.includes("canvas2d") ? "canvas2d" : "webgl";
  const character = await createEmbodiedCharacter(renderer);
  try { fixture.initialSnapshot = character.getSnapshot(renderer); }
  finally { character.dispose(); }
}
if (historicalData() !== historicalBefore) throw new Error("Historical input stream changed");
capture.migration.bodyAlignmentRefresh = {
  date: "2026-09-18",
  baselineCommit: "06c53a933c637c2425888558ae788f4f99d40f58",
  historicalInputSha256: createHash("sha256").update(historicalBefore).digest("hex"),
  scope: "Initial assembly snapshots only. Commands and recorded transfers are unchanged. Not a new browser capture or acceptance result.",
};
if (process.argv.includes("--write")) {
  writeFileSync(file, JSON.stringify(capture, null, 2) + "\n");
  console.log("Refreshed two initial assembly snapshots; historical inputs unchanged.");
} else {
  console.log("Dry run: use --write to refresh initial snapshots. Historical inputs unchanged.");
}
