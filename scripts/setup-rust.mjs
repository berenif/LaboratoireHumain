import { spawnSync } from 'node:child_process';
import { browserTool, browserToolVersions, checkBrowserTool, installedToolRoot } from './rust-browser-tools.mjs';
import { rustEnvironment, rustToolchain } from './rust-toolchain.mjs';

const args = process.argv.slice(2);
if (args.some(arg => arg !== '--check')) throw new Error('Usage: node scripts/setup-rust.mjs [--check]');
const checkOnly = args.includes('--check');
const env = { ...rustEnvironment(), CARGO_NET_OFFLINE: 'false' };
function run(executable, args) {
  const result = spawnSync(executable, args, { env, stdio: 'inherit', windowsHide: true });
  if (result.status !== 0) throw new Error(`${executable} failed: ${result.error?.message ?? result.status}`);
}
if (!checkOnly) {
  run('rustup', ['toolchain', 'install', rustToolchain, '--profile', 'minimal', '--component', 'clippy,rustfmt', '--target', 'wasm32-unknown-unknown']);
  for (const manifest of ['rust/Cargo.toml', 'rust/vendor/rapier3d/Cargo.toml']) {
    run('cargo', [`+${rustToolchain}`, 'fetch', '--locked', '--manifest-path', manifest]);
  }
}
for (const name of Object.keys(browserToolVersions)) {
  try { checkBrowserTool(name); }
  catch (error) {
    if (checkOnly) throw error;
    if (process.env[name === 'trunk' ? 'TRUNK_BIN' : 'WASM_BINDGEN_BIN']) throw error;
    run('cargo', [`+${rustToolchain}`, 'install', name === 'trunk' ? name : 'wasm-bindgen-cli', '--version', browserToolVersions[name], '--locked', '--root', installedToolRoot]);
    checkBrowserTool(name);
  }
  console.log(`Ready: ${name} ${browserToolVersions[name]} (${browserTool(name)})`);
}
