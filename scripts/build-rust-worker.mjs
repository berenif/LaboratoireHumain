import { spawnSync } from 'node:child_process';
import { copyFileSync, mkdirSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { browserEnvironment, checkBrowserTool, workspaceOutput } from './rust-browser-tools.mjs';
import { rustToolchain } from './rust-toolchain.mjs';

const root = process.cwd();
const output = workspaceOutput(process.argv[2] ?? 'rust/dist/worker');
const env = browserEnvironment();
const bindgen = checkBrowserTool('wasm-bindgen', undefined, env);
function run(executable, args) {
  const result = spawnSync(executable, args, { env, encoding: 'utf8', windowsHide: true, maxBuffer: 16 * 1024 * 1024 });
  if (result.status !== 0) throw new Error(`${executable}: ${result.error?.message ?? result.stderr ?? result.stdout}`);
  return result.stdout;
}
run('cargo', [`+${rustToolchain}`, 'build', '--offline', '--locked', '--release', '--manifest-path', 'rust/Cargo.toml', '--target', 'wasm32-unknown-unknown', '-p', 'lh-worker']);
mkdirSync(output, { recursive: true });
run(bindgen, [resolve('rust/target/wasm32-unknown-unknown/release/lh_worker.wasm'), '--target', 'web', '--out-dir', output, '--out-name', 'lh_worker']);
copyFileSync(resolve(root, 'rust/browser/sim-worker.js'), join(output, 'sim-worker.js'));
console.log(`Built worker at ${output}`);
