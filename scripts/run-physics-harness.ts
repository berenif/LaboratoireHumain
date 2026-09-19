import { spawn } from "node:child_process";
import { createHash } from "node:crypto";
import { createReadStream, createWriteStream, mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { once } from "node:events";
import { dirname, join, relative, resolve } from "node:path";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";

const activeChildren = new Set<ReturnType<typeof spawn>>();
const workerCount = Number(process.env.PHYSICS_WORKERS ?? "1");
if (!Number.isInteger(workerCount) || workerCount < 1) throw new Error("PHYSICS_WORKERS must be a positive integer.");

if (workerCount === 1 || process.env.PHYSICS_LIST_ONLY === "1") {
  await import("./physics-acceptance");
} else {
  try { await parallelAcceptance(workerCount); }
  finally { for (const child of activeChildren) child.kill(); }
}

interface ScenarioResult {
  name: string;
  passed: boolean;
  durationMs: number;
  metrics: Record<string, unknown>;
  failures: string[];
}
interface PhysicsReport {
  schema: number;
  generatedAt: string;
  selection: string;
  summary: { passed: boolean; passedScenarios: number; failedScenarios: number };
  results: ScenarioResult[];
  [key: string]: unknown;
}

function sourceFingerprint(): Record<string, string> {
  const paths: string[] = [];
  function visit(directory: string): void {
    for (const entry of readdirSync(directory, { withFileTypes: true })) {
      const path = join(directory, entry.name);
      if (entry.isDirectory()) visit(path);
      else if (entry.isFile()) paths.push(path);
    }
  }
  visit("src"); visit("scripts");
  paths.push("package.json", "package-lock.json");
  return Object.fromEntries(paths.sort().map(path => [
    relative(process.cwd(), resolve(path)).replaceAll("\\", "/"),
    createHash("sha256").update(readFileSync(path)).digest("hex"),
  ]));
}

async function runWorker(environment: Record<string, string>, logPath?: string): Promise<{ code: number | null; output: string }> {
  const script = fileURLToPath(new URL("./physics-acceptance.ts", import.meta.url));
  const child = spawn(process.execPath, ["--import", "tsx", script], {
    cwd: process.cwd(), env: { ...process.env, ...environment, PHYSICS_WORKERS: "1" },
    windowsHide: true, stdio: ["ignore", "pipe", "pipe"],
  });
  activeChildren.add(child);
  let output = "", errors = "";
  child.stdout.on("data", (chunk: Buffer) => {
    output += chunk.toString();
    if (logPath) process.stdout.write(chunk);
  });
  child.stderr.on("data", (chunk: Buffer) => {
    errors += chunk.toString();
    if (logPath) process.stderr.write(chunk);
  });
  const [code] = await once(child, "close").finally(() => activeChildren.delete(child)) as [number | null];
  if (logPath) writeFileSync(logPath, output + errors);
  if (code !== 0 && code !== 1) throw new Error(`Physics worker exited ${code}: ${errors}`);
  return { code, output };
}

function reportMetadata(report: PhysicsReport): string {
  const metadata = { ...report } as Partial<PhysicsReport>;
  delete metadata.generatedAt; delete metadata.selection; delete metadata.summary; delete metadata.results;
  return JSON.stringify(metadata);
}

async function parallelAcceptance(workers: number): Promise<void> {
  const selection = process.env.PHYSICS_SCENARIO_PATTERN;
  const outputPrefix = process.env.PHYSICS_OUTPUT_PREFIX ?? (selection ? "evidence/physics-selected" : "evidence/physics");
  const tracePath = process.env.PHYSICS_OUTPUT_PREFIX ? outputPrefix + "-trace.ndjson"
    : selection ? "evidence/physics-selected-trace.ndjson" : "evidence/successor-trace.ndjson";
  const runDirectory = `${outputPrefix}-workers/${Date.now()}-${process.pid}`;
  mkdirSync(runDirectory, { recursive: true });
  const before = sourceFingerprint();
  const execution = { workers, selection: selection ?? "all", runDirectory, scenarios: [] as string[], before, after: {} as Record<string, string>, sourceStable: false, validationErrors: [] as string[], completed: false };
  const provenancePath = outputPrefix + "-execution.json";
  writeFileSync(provenancePath, JSON.stringify(execution, null, 2) + "\n");
  const discovery = await runWorker({ PHYSICS_LIST_ONLY: "1" });
  if (discovery.code !== 0) throw new Error("Physics scenario discovery failed.");
  const lines = discovery.output.trim().split(/\r?\n/);
  const discovered = JSON.parse(lines[lines.length - 1]) as { scenarios: string[] };
  const names = discovered.scenarios;
  if (!Array.isArray(names) || !names.length || names.some(name => typeof name !== "string")
    || new Set(names).size !== names.length) throw new Error("Physics discovery must return unique scenario names and a nonempty selection.");
  execution.scenarios = names;
  writeFileSync(provenancePath, JSON.stringify(execution, null, 2) + "\n");
  console.log(`Running ${names.length} physics scenarios with ${Math.min(workers, names.length)} workers.`);
  const reports = new Array<PhysicsReport>(names.length);
  const prefixes = new Array<string>(names.length);
  let next = 0;
  await Promise.all(Array.from({ length: Math.min(workers, names.length) }, async () => {
    while (next < names.length) {
      const index = next++, name = names[index];
      const prefix = join(runDirectory, String(index).padStart(3, "0"));
      prefixes[index] = prefix;
      const pattern = "^" + name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&") + "$";
      const child = await runWorker({ PHYSICS_LIST_ONLY: "0", PHYSICS_SCENARIO_PATTERN: pattern, PHYSICS_OUTPUT_PREFIX: prefix }, prefix + "-console.txt");
      const report = JSON.parse(readFileSync(prefix + "-results.json", "utf8")) as PhysicsReport;
      if (report.results.length !== 1 || report.results[0].name !== name || report.selection !== pattern)
        throw new Error(`Worker scenario coverage differs from discovery: ${name}`);
      const passed = report.results[0].passed;
      if (report.summary.passed !== passed || report.summary.passedScenarios !== Number(passed)
        || report.summary.failedScenarios !== Number(!passed) || child.code !== (passed ? 0 : 1))
        throw new Error(`Worker outcome/report mismatch: ${name}`);
      reports[index] = report;
    }
  }));
  execution.after = sourceFingerprint();
  writeFileSync(provenancePath, JSON.stringify(execution, null, 2) + "\n");
  const first = reports[0];
  if (reports.some(report => reportMetadata(report) !== reportMetadata(first))) throw new Error("Physics worker acceptance metadata differs.");
  const results = reports.flatMap(report => report.results);
  if (results.length !== names.length || results.some((result, index) => result.name !== names[index]))
    throw new Error("Physics scenarios were missing, repeated, or reordered during aggregation.");
  const failed = results.filter(result => !result.passed).length;
  const report: PhysicsReport = {
    ...first, generatedAt: new Date().toISOString(), selection: selection ?? "all",
    summary: { passed: failed === 0, passedScenarios: results.length - failed, failedScenarios: failed }, results,
  };
  mkdirSync(dirname(tracePath), { recursive: true });
  const destination = createWriteStream(tracePath);
  for (const [index, prefix] of prefixes.entries()) {
    const source = createInterface({ input: createReadStream(prefix + "-trace.ndjson"), crlfDelay: Infinity });
    for await (const line of source) {
      if (!line.trim()) continue;
      if ((JSON.parse(line) as { scenario: string }).scenario !== names[index]) throw new Error("Worker trace contains an unexpected scenario.");
      if (!destination.write(line + "\n")) await once(destination, "drain");
    }
  }
  destination.end(); await once(destination, "finish");
  execution.after = sourceFingerprint();
  execution.sourceStable = JSON.stringify(before) === JSON.stringify(execution.after);
  if (!execution.sourceStable) {
    execution.validationErrors.push("Physics source changed during the parallel run; all scenario evidence is retained but validation is invalidated.");
    report.summary.passed = false;
  }
  writeFileSync(outputPrefix + "-results.json", JSON.stringify(report, null, 2) + "\n");
  execution.completed = true;
  writeFileSync(provenancePath, JSON.stringify(execution, null, 2) + "\n");
  console.log(JSON.stringify(report.summary));
  for (const error of execution.validationErrors) console.error(error);
  process.exitCode = execution.validationErrors.length ? 2 : failed ? 1 : 0;
}
