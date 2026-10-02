import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { delimiter, dirname, resolve } from 'node:path';
import test from 'node:test';
import { browserEnvironment, browserToolVersions, publicPath, workspaceOutput } from '../scripts/rust-browser-tools.mjs';
import { rustCheckCommands, rustToolchain } from '../scripts/rust-toolchain.mjs';

test('Rust builders reject outputs that could replace source or escape the workspace', () => {
  for (const path of ['.', '..', '../external-build', 'rust', 'rust/dist', 'rust/crates/sim', 'scripts', '.git']) {
    assert.throws(() => workspaceOutput(path), /Build output/);
  }
  assert.equal(workspaceOutput('rust/dist/app'), resolve('rust/dist/app'));
  assert.equal(workspaceOutput('evidence/browser-root'), resolve('evidence/browser-root'));
});

test('root and Pages subpath builders and preview use the same mount contract', () => {
  for (const path of ['/', '/LaboratoireHumain/', '/preview/branch-1/']) assert.equal(publicPath(path), path);
  for (const path of ['', 'relative/', '/missing-slash', '/../../', 'https://example.com/']) {
    assert.throws(() => publicPath(path), /Public URL/);
  }
});

test('worker and Trunk share the pinned tools and explicit bindgen PATH', () => {
  const executable = resolve('build/custom-bindgen/wasm-bindgen');
  const env = browserEnvironment(executable);
  assert.equal(env.PATH.split(delimiter)[0], dirname(executable));
  assert.equal(env.RUSTUP_TOOLCHAIN, rustToolchain);
  assert.equal(env.CARGO_NET_OFFLINE, 'true');
  assert.equal(browserToolVersions['wasm-bindgen'], '0.2.108');
  const [, native, wasm] = rustCheckCommands('lint');
  for (const command of [native, wasm]) {
    assert.ok(command.includes('--workspace') && command.includes('--all-targets') && command.includes('--locked'));
  }
  assert.ok(wasm.includes('wasm32-unknown-unknown'));
});

test('verification rejects misspelled scope flags and reused evidence before running tools', () => {
  for (const args of [['--foundaton'], ['--foundation', 'rust'], ['one', 'two']]) {
    const result = spawnSync(process.execPath, ['scripts/run-rust-gates.mjs', ...args], { encoding: 'utf8', windowsHide: true });
    assert.notEqual(result.status, 0);
    assert.match(result.stderr, /Usage:|fresh workspace output/);
  }
});
