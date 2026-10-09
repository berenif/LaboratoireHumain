import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, statSync, unlinkSync, utimesSync, writeFileSync } from 'node:fs';
import { delimiter, dirname, join, resolve } from 'node:path';
import test from 'node:test';
import { browserEnvironment, browserToolVersions, publicPath, workspaceOutput } from '../scripts/rust-browser-tools.mjs';
import { prepareRustBuild, rustCheckCommands, rustToolchain } from '../scripts/rust-toolchain.mjs';

test('Rust freshness recovers invalid history, old timestamps and deleted inputs without changing bytes', () => {
  mkdirSync('evidence', { recursive: true });
  const root = mkdtempSync(resolve('evidence/rust-freshness-inputs-'));
  mkdirSync(join(root, 'rust/src'), { recursive: true });
  const first = join(root, 'rust/src/first.rs'), second = join(root, 'rust/src/second.rs');
  writeFileSync(first, 'first'); writeFileSync(second, 'second');
  prepareRustBuild(root);
  const before = statSync(first).mtimeMs;
  utimesSync(first, new Date(0), new Date(0));
  assert.deepEqual(prepareRustBuild(root).refreshedInputs, ['rust/src/first.rs']);
  assert.ok(statSync(first).mtimeMs > before);
  unlinkSync(second);
  const removed = prepareRustBuild(root);
  assert.deepEqual(removed.removedInputs, ['rust/src/second.rs']);
  assert.deepEqual(removed.refreshedInputs, ['rust/src/first.rs']);
  writeFileSync(join(root, 'rust/target/source-freshness.json'), '{bad history');
  assert.deepEqual(prepareRustBuild(root).refreshedInputs, ['rust/src/first.rs']);
  assert.equal(readFileSync(first, 'utf8'), 'first');
});

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

test('browser builds preserve inherited tool paths with Windows environment casing', () => {
  const executable = resolve('build/custom-bindgen/wasm-bindgen');
  const inherited = ['cargo-bin', 'node-bin'].join(delimiter);
  for (const key of ['Path', 'PATH', 'path']) {
    const original = { [key]: inherited, KEEP: 'value' };
    const env = browserEnvironment(executable, original);
    assert.deepEqual(Object.keys(env).filter(key => key.toLowerCase() === 'path'), ['PATH']);
    assert.equal(env.PATH, dirname(executable) + delimiter + inherited);
    assert.equal(env.KEEP, 'value');
    assert.equal(original[key], inherited);
  }
});
