import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { copyFileSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const output = process.argv[2];
assert.ok(output, 'Supply a fresh output directory');
mkdirSync(output);
const sourceBefore = fingerprints(), runs = [];
const environment = {
  CARGO_HOME: resolve('evidence/rapier-calibration-cache'),
  RUSTUP_HOME: resolve('evidence/rapier-calibration-rustup'),
  CARGO_TARGET_DIR: resolve('evidence/rapier-calibration-f32-target'),
};
const toolchain = '+1.89.0-x86_64-pc-windows-gnu';
const common = ['--offline', '--manifest-path', 'scripts/rapier-calibration/Cargo.toml'];
try {
  for (const variant of ['normal', 'block']) {
    const features = variant === 'block' ? ['--features', 'block-solver'] : [];
    const command = ['cargo', toolchain, 'build', ...common, '--release', '--bin', 'rapier-calibration', ...features];
    const build = spawnSync(command[0], command.slice(1), { env: { ...process.env, ...environment }, encoding: 'utf8', windowsHide: true, timeout: 240000 });
    const run = { variant, command, exitCode: build.status, signal: build.signal, stdout: build.stdout, stderr: build.stderr };
    runs.push(run);
    assert.equal(build.status, 0, JSON.stringify(run));
    const executable = join(output, `${variant}.exe`);
    copyFileSync(join(environment.CARGO_TARGET_DIR, 'release/rapier-calibration.exe'), executable);
    run.executable = resolve(executable);
    run.executableSha256 = sha256(readFileSync(executable));
    const metadataCommand = ['cargo', toolchain, 'metadata', ...common, '--format-version', '1', ...features];
    const result = spawnSync(metadataCommand[0], metadataCommand.slice(1), { env: { ...process.env, ...environment }, encoding: 'utf8', windowsHide: true, timeout: 60000, maxBuffer: 16 * 1024 * 1024 });
    assert.equal(result.status, 0, result.stderr);
    const metadata = JSON.parse(result.stdout);
    run.metadataCommand = metadataCommand;
    run.resolvedFeatures = Object.fromEntries(metadata.resolve.nodes.map(node => [node.id, node.features]));
    console.log(JSON.stringify({ variant, executable, sha256: run.executableSha256 }));
  }
  const changes = Object.keys(runs[0].resolvedFeatures).filter(id =>
    JSON.stringify(runs[0].resolvedFeatures[id]) !== JSON.stringify(runs[1].resolvedFeatures[id]));
  assert.deepEqual(Object.keys(runs[0].resolvedFeatures), Object.keys(runs[1].resolvedFeatures));
  assert.equal(changes.length, 2);
  for (const id of changes) {
    assert.ok(id.includes('rapier-calibration') || id.endsWith('#rapier3d@0.35.0'), id);
    assert.deepEqual(runs[1].resolvedFeatures[id], [...runs[0].resolvedFeatures[id], 'block-solver'].sort());
  }
} finally {
  const sourceAfter = fingerprints();
  writeFileSync(join(output, 'manifest.json'), JSON.stringify({ generatedAt: new Date().toISOString(), sourceBefore, sourceAfter,
    environment, runs, qualification: 'Independent native f32 fixture comparison. The sole Cargo feature change is block-solver on the executable and Rapier; production dependency and settings remain unchanged.' }, null, 2) + '\n', { flag: 'wx' });
  assert.deepEqual(sourceBefore, sourceAfter);
}
