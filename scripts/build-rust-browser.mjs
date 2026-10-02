import { spawnSync } from 'node:child_process';
import { resolve } from 'node:path';
import { browserEnvironment, checkBrowserTool, publicPath, workspaceOutput } from './rust-browser-tools.mjs';

const root = process.cwd();
const output = workspaceOutput(process.argv[2] ?? 'rust/dist/app');
const mount = publicPath(process.argv[3]);
const env = browserEnvironment();
const trunk = checkBrowserTool('trunk', undefined, env);
checkBrowserTool('wasm-bindgen', undefined, env);
function run(executable, args, cwd = root) {
  const result = spawnSync(executable, args, { cwd, env, stdio: 'inherit', windowsHide: true });
  if (result.status !== 0) throw new Error(executable + ' failed: ' + (result.error?.message ?? result.status));
}
run(process.execPath, ['scripts/build-rust-worker.mjs']);
run(trunk, ['build', '--config', resolve('rust/Trunk.toml'), '--release', '--offline', 'true', '--locked', 'true', '--skip-version-check', '--dist', output,
  '--public-url', mount], resolve('rust'));
console.log('Built Rust browser application at ' + output);
