import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const [input, output] = process.argv.slice(2);
if (!input || !output || existsSync(output)) throw new Error('Usage: node scripts/run-native-impulse.mjs exported-report.json fresh-output-directory');
const unregister = register();
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { jointCoordinateKinematics, jointCoordinates } = await import('../src/character/joint-coordinates.ts');
const report = JSON.parse(readFileSync(input)), inputDirectory = dirname(input);
assert.ok(report.parameters.exportSnapshots, 'Paired native snapshots are required');
const executables = Object.fromEntries(['f32', 'f64'].map(precision => [precision,
  resolve(`evidence/rapier-calibration-${precision}-target/release/rapier-calibration.exe`)]));
const executableHashes = Object.fromEntries(Object.entries(executables).map(([precision, file]) => [precision, sha256(readFileSync(file))]));
const sourceBefore = fingerprints(), started = new Date().toISOString(), results = [], commands = [];
mkdirSync(output, { recursive: true });
const rate = (states, sourceId) => {
  const definition = SEGMENT_BY_ID.get(sourceId), parent = states[definition.parent], child = states[sourceId];
  const coordinates = jointCoordinates(parent.rotation, child.rotation, definition.jointProfile);
  return jointCoordinateKinematics(parent.angularVelocity, child.angularVelocity, parent.rotation, coordinates, definition.jointProfile).rates.x;
};
try {
  for (const [index, item] of report.results.entries()) {
    const evidenceBytes = readFileSync(join(inputDirectory, item.evidence.file));
    assert.equal(sha256(evidenceBytes), item.evidence.sha256);
    const evidence = JSON.parse(evidenceBytes), pair = {};
    for (const [precision, executable] of Object.entries(executables)) {
      pair[precision] = {};
      for (const side of ['control', 'driven']) {
        const snapshot = evidence.snapshots[side];
        const snapshotPath = join(inputDirectory, snapshot.snapshotFile), metadataPath = join(inputDirectory, snapshot.metadataFile);
        assert.equal(sha256(readFileSync(snapshotPath)), snapshot.snapshotSha256);
        assert.equal(sha256(readFileSync(metadataPath)), snapshot.metadataSha256);
        const metadata = JSON.parse(readFileSync(metadataPath));
        const nativePath = join(output, `${index}-${precision}-${side}.json`);
        const args = [snapshotPath, metadataPath, nativePath, 'normal'];
        const result = spawnSync(executable, args, { encoding: 'utf8', windowsHide: true, timeout: 60000, maxBuffer: 8 * 1024 * 1024 });
        commands.push({ command: [executable, ...args], exitCode: result.status, signal: result.signal, stdout: result.stdout, stderr: result.stderr });
        assert.equal(result.status, 0, JSON.stringify(commands.at(-1)));
        const native = JSON.parse(readFileSync(nativePath));
        for (const body of metadata.bodiesBeforeStep) {
          const initial = native.samples[0].measured[body.id];
          for (const key of ['position', 'rotation', 'angularVelocity']) assert.deepEqual(initial[key], body[key], `${precision} ${side} initial ${body.id} ${key}`);
          assert.deepEqual(initial.velocity, body.linearVelocity, `${precision} ${side} initial velocity`);
        }
        assert.equal(native.samples.at(-1).tick, 1);
        pair[precision][side] = native;
      }
    }
    const responses = Object.fromEntries(Object.entries(pair).map(([precision, sides]) => {
      const rates = Object.fromEntries(Object.entries(sides).map(([side, native]) => [side, rate(native.samples.at(-1).measured, item.sourceId)]));
      const actual = (rates.driven - rates.control) / item.impulseNmS;
      const relativeError = Math.abs(item.predicted - actual) / Math.abs(actual);
      return [precision, { rates, actual, relativeError, passed: relativeError < .1 }];
    }));
    const row = { index, heading: item.heading, sourceId: item.sourceId, impulseNmS: item.impulseNmS,
      predicted: item.predicted, wasm: { actual: item.actual, relativeError: item.relativeError }, responses };
    results.push(row); console.log(JSON.stringify(row));
  }
} finally {
  const sourceAfter = fingerprints();
  writeFileSync(join(output, 'report.json'), JSON.stringify({ started, finished: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version,
    input: { file: input, sha256: sha256(readFileSync(input)) }, executables, executableHashes,
    sourceBefore, sourceAfter, qualification: 'One-step precision comparison from serialized physical states and contact/joint caches, with fresh transient pipelines. Compare restored WASM histories and native f32 explicitly before interpretation. No production/fixture/tolerance change.',
    commands, results }, null, 2) + '\n', { flag: 'wx' });
  unregister();
  assert.deepEqual(sourceAfter, sourceBefore);
  for (const [precision, file] of Object.entries(executables)) assert.equal(sha256(readFileSync(file)), executableHashes[precision]);
}
