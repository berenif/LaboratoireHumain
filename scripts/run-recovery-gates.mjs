import { spawnSync } from "node:child_process";
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";

// Distinct evidence per change; never overwrite the supplied baseline.
const prefix = process.argv[2];
if (!prefix || !/^evidence\/[a-zA-Z0-9-]+$/.test(prefix)) {
  throw new Error("Usage: node scripts/run-recovery-gates.mjs evidence/unique-prefix [scenario-regex]");
}
const pattern = process.argv[3] ?? "crouch";
if (existsSync(`${prefix}-gates.json`)) throw new Error("Evidence prefix already exists; choose a distinct prefix.");
mkdirSync("evidence", { recursive: true });
const source = spawnSync("git", ["ls-files", "--cached", "--others", "--exclude-standard", "src", "scripts", "tests", "package.json", "tsconfig.json", ".github/workflows"], { encoding: "utf8" });
if (source.status !== 0) throw new Error(source.stderr);
const files = [...new Set([...source.stdout.trim().split(/\r?\n/), "scripts/run-recovery-gates.mjs"])];
const report = { date: new Date().toISOString(), node: process.version, fixedHz: 60,
  rapier: JSON.parse(readFileSync("node_modules/@dimforge/rapier3d-compat/package.json", "utf8")).version,
  pattern, fingerprints: Object.fromEntries(files.map(file => [file, createHash("sha256").update(readFileSync(file)).digest("hex")])), results: [] };
for (const [name, args] of [
  ["typecheck", ["node_modules/typescript/bin/tsc", "--noEmit", "--pretty", "false"]],
  ["focused", ["--test", "tests/character-domain.test.mjs", "tests/recovery-*.test.mjs"]],
  ["physics", ["--import", "tsx", "scripts/run-physics-harness.ts"]],
]) {
  const start = Date.now();
  console.log(`Running ${name}...`);
  const result = spawnSync(process.execPath, args, { encoding: "utf8", maxBuffer: 64 * 1024 * 1024,
    env: { ...process.env, PHYSICS_SCENARIO_PATTERN: pattern === "all" ? "" : pattern, PHYSICS_OUTPUT_PREFIX: prefix } });
  writeFileSync(`${prefix}-${name}.log`, result.stdout + result.stderr);
  report.results.push({ name, command: [process.execPath, ...args], exitCode: result.status, elapsedMs: Date.now() - start, error: result.error?.message });
  writeFileSync(`${prefix}-gates.json`, JSON.stringify(report, null, 2) + "\n");
  console.log(`${name}: ${result.status === 0 ? "PASS" : "FAIL"}`);
}
process.exitCode = report.results.some(result => result.exitCode !== 0) ? 1 : 0;
