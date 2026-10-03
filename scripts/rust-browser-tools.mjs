import { existsSync, readdirSync } from 'node:fs';
import { delimiter, dirname, isAbsolute, join, relative, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { rustEnvironment, rustToolchain } from './rust-toolchain.mjs';

export const browserToolVersions = Object.freeze({ trunk: '0.21.14', 'wasm-bindgen': '0.2.108' });
export const installedToolRoot = resolve('.sites-runtime/rust-browser-tools/installed');

export function workspaceOutput(value, root = process.cwd()) {
  const output = resolve(root, value);
  const local = relative(root, output);
  if (!local || local === '..' || local.startsWith(`..${process.platform === 'win32' ? '\\' : '/'}`) || isAbsolute(local)) {
    throw new Error('Build output must be a child directory of the workspace');
  }
  // A build must never overwrite source, tooling, or an ancestor of its inputs.
  const first = local.split(/[\\/]/)[0];
  if (!['rust', 'evidence', 'dist', 'build', 'outputs'].includes(first)
      || (first === 'rust' && !/^rust[\\/]dist[\\/]/.test(local))) {
    throw new Error('Build output must be below rust/dist, evidence, dist, build, or outputs');
  }
  return output;
}

export function publicPath(value = '/') {
  if (!/^\/(?:[A-Za-z0-9_-]+\/)*$/.test(value)) {
    throw new Error('Public URL must be an absolute directory path ending with /');
  }
  return value;
}

export function browserTool(name) {
  const override = process.env[name === 'trunk' ? 'TRUNK_BIN' : 'WASM_BINDGEN_BIN'];
  if (override) return override;
  const filename = name + (process.platform === 'win32' ? '.exe' : '');
  const installed = join(installedToolRoot, 'bin', filename);
  if (existsSync(installed)) return installed;
  const legacy = resolve(`.sites-runtime/rust-browser-tools/${name}-${browserToolVersions[name]}`);
  if (name === 'trunk' && existsSync(join(legacy, filename))) return join(legacy, filename);
  if (name === 'wasm-bindgen' && existsSync(legacy)) {
    const directory = readdirSync(legacy).find(entry => entry.startsWith(`${name}-${browserToolVersions[name]}-`));
    if (directory && existsSync(join(legacy, directory, filename))) return join(legacy, directory, filename);
  }
  return name;
}

export function browserEnvironment(bindgen = browserTool('wasm-bindgen'), baseEnvironment = rustEnvironment()) {
  const env = { ...baseEnvironment, NO_COLOR: 'true', CARGO_NET_OFFLINE: 'true', RUSTUP_TOOLCHAIN: rustToolchain };
  // Trunk locates the bindgen CLI on PATH, including when an explicit override is used.
  if (isAbsolute(bindgen) || /[\\/]/.test(bindgen)) {
    // Node passes only the first sorted spelling of Windows' case-insensitive
    // PATH. Keeping both Path and PATH can silently discard Cargo's directory.
    const pathKeys = Object.keys(env).filter(key => key.toLowerCase() === 'path');
    const inheritedPath = env.PATH ?? env[pathKeys[0]] ?? '';
    for (const key of pathKeys) delete env[key];
    env.PATH = dirname(resolve(bindgen)) + delimiter + inheritedPath;
  }
  return env;
}

export function checkBrowserTool(name, executable = browserTool(name), env = browserEnvironment()) {
  const result = spawnSync(executable, ['--version'], { env, encoding: 'utf8', windowsHide: true });
  const expected = `${name} ${browserToolVersions[name]}`;
  if (result.status !== 0 || result.stdout.trim() !== expected) {
    throw new Error(`Expected ${expected}; ${result.error?.message ?? result.stderr?.trim() ?? result.stdout?.trim()}. Run npm run setup:rust or set ${name === 'trunk' ? 'TRUNK_BIN' : 'WASM_BINDGEN_BIN'}.`);
  }
  return executable;
}
