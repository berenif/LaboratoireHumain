import { spawn } from "node:child_process";
import { existsSync, mkdirSync } from "node:fs";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL("..", import.meta.url));
const runtime = resolve(root, ".sites-runtime");
const [tool, ...args] = process.argv.slice(2);
const entrypoints = {
  vite: "vite/bin/vite.js",
  vinext: "vinext/dist/cli.js",
  eslint: "eslint/bin/eslint.js",
  "drizzle-kit": "drizzle-kit/bin.cjs",
};

if (!Object.hasOwn(entrypoints, tool)) {
  throw new Error(`Unknown project tool: ${tool ?? "(missing)"}`);
}

const entrypoint = resolve(root, "node_modules", entrypoints[tool]);
if (!existsSync(entrypoint)) {
  throw new Error(`${tool} is unavailable. Run npm ci before using project tools.`);
}

function durationMs(name, fallback) {
  const duration = process.env[name] ?? fallback;
  const match = /^(\d+(?:\.\d+)?)(ms|s|m|h)?$/.exec(duration);
  if (!match || Number(match[1]) <= 0) {
    throw new Error(`${name} must be a positive duration, such as 180s or 3m.`);
  }
  const units = { ms: 1, s: 1000, m: 60_000, h: 3_600_000 };
  return Number(match[1]) * units[match[2] ?? "s"];
}

const isBuild = tool === "vinext" && args[0] === "build";
const buildTimeout = isBuild ? durationMs("SITES_BUILD_TIMEOUT", "3m") : 0;
const killAfter = isBuild ? durationMs("SITES_BUILD_KILL_AFTER", "10s") : 0;
const logPath = resolve(runtime, "wrangler", "logs");
mkdirSync(logPath, { recursive: true });

const child = spawn(process.execPath, [entrypoint, ...args], {
  cwd: root,
  stdio: "inherit",
  env: {
    ...process.env,
    WRANGLER_WRITE_LOGS: "false",
    WRANGLER_LOG_PATH: logPath,
    MINIFLARE_REGISTRY_PATH: resolve(runtime, "wrangler", "registry"),
  },
});

let timeout;
let forceKill;
let timedOut = false;

if (isBuild) {
  timeout = setTimeout(() => {
    timedOut = true;
    console.error(`Build exceeded ${buildTimeout / 1000}s; stopping it.`);
    child.kill("SIGTERM");
    forceKill = setTimeout(() => child.kill("SIGKILL"), killAfter);
  }, buildTimeout);
}

child.on("error", (error) => {
  clearTimeout(timeout);
  clearTimeout(forceKill);
  console.error(error.message);
  process.exitCode = 1;
});

child.on("exit", (code) => {
  clearTimeout(timeout);
  clearTimeout(forceKill);
  process.exitCode = timedOut ? 124 : (code ?? 1);
});

for (const signal of ["SIGINT", "SIGTERM"]) {
  process.on(signal, () => child.kill(signal));
}
