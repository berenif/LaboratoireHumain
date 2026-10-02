import { existsSync } from 'node:fs';
import { resolve } from 'node:path';

export const rustToolchain = process.platform === 'win32'
  ? '1.89.0-x86_64-pc-windows-gnu'
  : '1.89.0';

// Use the browser builders' pinned compiler and cache locations for native
// checks too. The repository cache is optional on a fresh checkout.
export function rustEnvironment() {
  const env = { ...process.env, CARGO_TARGET_DIR: resolve('rust/target') };
  if (existsSync('evidence/rapier-calibration-cache')) {
    env.CARGO_HOME = resolve('evidence/rapier-calibration-cache');
  }
  return env;
}

export function rustCheckCommands(task) {
  const compiler = `+${rustToolchain}`;
  const workspace = ['--manifest-path', 'rust/Cargo.toml'];
  const locked = ['--offline', '--locked', '--release', ...workspace];
  switch (task) {
    case 'test':
      return [[compiler, 'test', ...locked, '--workspace']];
    case 'lint':
      return [
        [compiler, 'fmt', ...workspace, '--all', '--', '--check'],
        [compiler, 'clippy', ...locked, '--workspace', '--all-targets', '--', '-D', 'warnings'],
        [compiler, 'clippy', ...locked, '--target', 'wasm32-unknown-unknown',
          '--workspace', '--all-targets', '--', '-D', 'warnings'],
      ];
    default:
      throw new Error(`Unknown Rust check: ${task ?? '(missing)'}. Use test or lint.`);
  }
}
