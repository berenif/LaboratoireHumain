import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const [snapshotPath, metadataPath, output] = process.argv.slice(2);
if (!snapshotPath || !metadataPath || !output || existsSync(output)) throw new Error('Usage: node scripts/run-rapier-calibration.mjs snapshot.bin metadata.json fresh-output-directory');
const executable = resolve(process.env.RAPIER_CALIBRATION_EXE ?? 'evidence/rapier-calibration-target/release/rapier-calibration.exe');
const bytes = readFileSync(snapshotPath), metadataBytes = readFileSync(metadataPath), metadata = JSON.parse(metadataBytes);
assert.equal(sha256(bytes), metadata.snapshotSha256, 'Snapshot changed');
const sourceBefore = fingerprints(), started = new Date().toISOString();
const executableSha256 = sha256(readFileSync(executable));
const allowedModes = ['normal', 'friction-in-bias', 'warmstart-joints', 'no-self-contact', 'rigid-fixed-contact', 'pgs-convergence', 'relax-convergence', 'no-ccd'];
const modes = process.env.RAPIER_CALIBRATION_MODES?.split(',') ?? allowedModes.filter(mode => mode !== 'no-ccd');
assert.ok(modes.length && modes.every(mode => allowedModes.includes(mode)) && new Set(modes).size === modes.length);
mkdirSync(output, { recursive: true });
writeFileSync(join(output, 'input.bin'), bytes, { flag: 'wx' });
writeFileSync(join(output, 'input.metadata.json'), metadataBytes, { flag: 'wx' });
const runs = [];
try {
  for (const mode of modes) {
    const args = [join(output, 'input.bin'), join(output, 'input.metadata.json'), join(output, `${mode}.json`), mode];
    const result = spawnSync(executable, args, { encoding: 'utf8', windowsHide: true, timeout: 60000, maxBuffer: 8 * 1024 * 1024 });
    const recorded = { mode, command: [executable, ...args], exitCode: result.status, signal: result.signal,
      stdout: result.stdout, stderr: result.stderr, error: result.error ? String(result.error) : null };
    runs.push(recorded);
    assert.equal(result.status, 0, JSON.stringify(recorded));
    const report = JSON.parse(readFileSync(args[2]));
    recorded.reportSha256 = sha256(readFileSync(args[2]));
    console.log(JSON.stringify({ mode, maxLinearAfter120: report.maxLinearAfter120, maxAngularAfter120: report.maxAngularAfter120 }));
  }
} finally {
  const sourceAfter = fingerprints();
  writeFileSync(join(output, 'manifest.json'), JSON.stringify({ started, finished: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version, sourceBefore, sourceAfter,
    executable, executableSha256, executableSha256After: sha256(readFileSync(executable)),
    inputs: { snapshotPath, snapshotSha256: sha256(bytes), metadataPath, metadataSha256: sha256(metadataBytes) },
    qualification: 'Isolated fixed-assembly solver diagnosis. Interventions deliberately vary native calibration parameters; no application setting, collider, actuator or acceptance threshold is changed.', runs }, null, 2) + '\n', { flag: 'wx' });
  assert.deepEqual(sourceAfter, sourceBefore, 'Source changed during calibration');
  assert.equal(sha256(readFileSync(executable)), executableSha256, 'Native executable changed during calibration');
}
