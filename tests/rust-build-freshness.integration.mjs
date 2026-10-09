import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, statSync, utimesSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import test from 'node:test';
import { prepareRustBuild, rustEnvironment, rustToolchain } from '../scripts/rust-toolchain.mjs';

test('a timestamp-preserving restore rebuilds the actual executable, including vendored inputs', () => {
  const evidence = resolve('evidence');
  mkdirSync(evidence, { recursive: true });
  const root = mkdtempSync(join(evidence, 'rust-freshness-integration-'));
  const source = join(root, 'rust');
  mkdirSync(join(source, 'src'), { recursive: true });
  mkdirSync(join(source, 'vendor'), { recursive: true });
  writeFileSync(join(source, 'Cargo.toml'), '[package]\nname = "freshness-probe"\nversion = "0.1.0"\nedition = "2021"\n[workspace]\n');
  writeFileSync(join(source, 'Cargo.lock'), 'version = 4\n\n[[package]]\nname = "freshness-probe"\nversion = "0.1.0"\n');
  const main = join(source, 'src/main.rs');
  writeFileSync(main, 'fn main() { println!("{}", include_str!("../vendor/checkpoint.txt")); }\n');
  const input = join(source, 'vendor/checkpoint.txt');
  writeFileSync(input, 'checkpoint-a');
  const env = { ...rustEnvironment(), CARGO_TARGET_DIR: join(source, 'target') };
  const runs = [];
  function build(name) {
    const args = [`+${rustToolchain}`, 'build', '--offline', '--locked', '--release', '--manifest-path', join(source, 'Cargo.toml')];
    const result = spawnSync('cargo', args, { env, encoding: 'utf8', windowsHide: true, timeout: 60000 });
    writeFileSync(join(root, name + '.log'), (result.stdout ?? '') + (result.stderr ?? ''));
    assert.equal(result.status, 0, result.error?.message ?? result.stderr);
    const executable = join(source, 'target/release/freshness-probe' + (process.platform === 'win32' ? '.exe' : ''));
    const run = spawnSync(executable, [], { encoding: 'utf8', windowsHide: true, timeout: 10000 });
    assert.equal(run.status, 0, run.error?.message ?? run.stderr);
    const value = run.stdout.trim();
    runs.push({ name, command: ['cargo', ...args], value });
    return value;
  }
  const first = prepareRustBuild(root);
  assert.ok(first.refreshedInputs.includes('rust/src/main.rs'));
  assert.equal(build('a'), 'checkpoint-a');
  const originalTime = statSync(input).mtime;
  const mainTime = statSync(main).mtimeMs;
  writeFileSync(input, 'checkpoint-b');
  const second = prepareRustBuild(root);
  assert.deepEqual(second.refreshedInputs, ['rust/vendor/checkpoint.txt']);
  assert.equal(build('b'), 'checkpoint-b');
  // Reproduce the real failure before testing its repair.
  writeFileSync(input, 'checkpoint-a');
  utimesSync(input, originalTime, originalTime);
  assert.equal(build('restored-without-guard'), 'checkpoint-b');
  const restored = prepareRustBuild(root);
  assert.deepEqual(restored.refreshedInputs, ['rust/vendor/checkpoint.txt']);
  assert.equal(restored.inputFingerprint, first.inputFingerprint);
  assert.equal(build('restored-with-guard'), 'checkpoint-a');
  const finalTime = statSync(input).mtimeMs;
  assert.equal(statSync(main).mtimeMs, mainTime, 'Unchanged source must retain its timestamp');
  assert.deepEqual(prepareRustBuild(root).refreshedInputs, []);
  assert.equal(statSync(input).mtimeMs, finalTime, 'Repeated checks must preserve valid caches');
  writeFileSync(join(root, 'execution.json'), JSON.stringify({ passed: true, runs, first, second, restored }, null, 2) + '\n');
  assert.equal(readFileSync(input, 'utf8'), 'checkpoint-a');
});
