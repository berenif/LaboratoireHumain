import assert from 'node:assert/strict';
import { physicalReportAvailable, physicalReportEvidence } from './rust-physical-archive-tools.mjs';
import { existsSync, lstatSync, readFileSync, realpathSync } from 'node:fs';
import { dirname, isAbsolute, relative, resolve, sep } from 'node:path';
import { fingerprint, sha256 } from './rust-release-tools.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';

// Exact bytes used by the October 9 verified-a and verified-b strict-step runs.
export const recoveryProfileFile = 'rust/crates/sim/data/recovery-profile-v24.json';
export const recoveryProfileSha256 = '971661c88ae7258f157669d8e8a873161de40990274a9f2fd7d159f50006e00e';
export const strictStepIds = Object.freeze(['slow-hand-forward', 'slow-hand-backward', 'slow-hand-left',
  'slow-hand-right-heading', 'slow-left-foot', 'slow-right-foot-heading', 'direction-reversal',
  'held-target', 'release-during-corrective-step']);
export const recoveryBrowserStages = Object.freeze(['wasm-recovery-24', 'wasm-cycles-five',
  'live-root-webgpu', 'live-root-webgl2', 'live-subpath-webgpu', 'live-subpath-webgl2',
  'recordings-reviewed', 'asset-unchanged', 'browser-profile-applied', 'browser-worker-continuity']);
export const requiredRecoveryStages = Object.freeze(['profile-pinned', 'format', 'native-clippy',
  'wasm-clippy', 'tooling', 'native-tests', 'native-build', 'recovery-24', 'cycles-five',
  'strict-steps', 'native-repeat', ...recoveryBrowserStages, 'source-unchanged',
  'profile-unchanged', 'executable-unchanged']);

export function recoveryDecision(stages) {
  const names = stages.map(stage => stage.name);
  const missing = requiredRecoveryStages.filter(name => !names.includes(name));
  const pending = stages.filter(stage => stage.pending === true).map(stage => stage.name);
  const failed = stages.filter(stage => stage.passed !== true).map(stage => stage.name);
  const duplicate = names.filter((name, index) => names.indexOf(name) !== index);
  const unexpected = names.filter(name => !requiredRecoveryStages.includes(name));
  const complete = missing.length === 0 && pending.length === 0 && duplicate.length === 0 && unexpected.length === 0;
  return { complete, qualified: complete && failed.length === 0, releaseAccepted: false,
    missing, pending, failed, duplicate, unexpected };
}

export function freshRecoveryOutput(value, root = process.cwd()) {
  assert.equal(typeof value, 'string', 'Provide one fresh workspace output directory');
  assert.ok(value.length > 0, 'Provide one fresh workspace output directory');
  const output = workspaceOutput(value, root);
  assert.ok(!existsSync(output), 'Use a fresh workspace output directory');
  let ancestor = dirname(output);
  while (!existsSync(ancestor)) ancestor = dirname(ancestor);
  const local = relative(realpathSync(root), realpathSync(ancestor));
  assert.ok(!isAbsolute(local) && local !== '..' && !local.startsWith(`..${sep}`),
    'Output ancestor escapes the workspace');
  for (let current = ancestor; current !== dirname(current); current = dirname(current)) {
    assert.ok(!lstatSync(current).isSymbolicLink(), 'Refuse linked output ancestor');
    if (resolve(current) === resolve(root)) break;
  }
  return output;
}

export function recoveryProcessEnvironment(directory, inherited) {
  const env = { ...inherited };
  // Node keeps only the first sorted spelling of Windows environment keys.
  for (const key of Object.keys(env)) {
    if (['temp', 'tmp'].includes(key.toLowerCase())) delete env[key];
  }
  env.TEMP = resolve(directory);
  env.TMP = resolve(directory);
  return env;
}

export function pinnedRecoveryProfile(bytes) {
  assert.equal(sha256(bytes), recoveryProfileSha256, 'Recovery qualification profile SHA-256 mismatch');
  const profile = JSON.parse(bytes);
  assert.equal(profile.id, 'rust-physics-v24-com-restoration-release-fixtures');
  assert.equal(profile.corrective_steps_enabled, true, 'Pinned corrective stepping must remain enabled');
  assert.equal(profile.recovery_enabled, true, 'Pinned recovery must remain enabled');
  return profile;
}

export function verifyEmittedProfile(expected, actual) {
  assert.equal(fingerprint(actual), fingerprint(expected), 'Emitted simulation profile mismatch');
  return true;
}

export function physicalCoverage(name, report, fixtureCases = []) {
  if (name === 'cycles') {
    const cycles = report?.cycles;
    return { expectedCases: 5, actualCases: Array.isArray(cycles) ? cycles.length : 0,
      passedCases: Array.isArray(cycles) ? cycles.filter(row => row.passed === true).length : 0,
      passed: report?.passed === true && report.completedCycles === 5 && report.requiredCycles === 5
        && report.firstFailure === null && Array.isArray(cycles) && cycles.length === 5
        && cycles.every((row, index) => row.cycle === index + 1 && row.passed === true) };
  }
  assert.ok(['recovery', 'steps'].includes(name), 'Unknown recovery qualification diagnostic');
  const expectedIds = name === 'recovery' ? fixtureCases.map(row => row.id) : strictStepIds;
  const rows = Array.isArray(report) ? report : [];
  const expectedCases = name === 'recovery' ? 24 : 9;
  const ids = rows.map(row => row.id);
  const identity = expectedIds.length === expectedCases && new Set(expectedIds).size === expectedCases
    && ids.length === expectedCases && new Set(ids).size === expectedCases
    && expectedIds.every(id => ids.includes(id));
  const valid = row => row.passed === true && (name !== 'recovery' || (row.dynamicOwnership === true
    && row.failure === null && row.handoffStandingSubsteps >= 240
    && row.heading === Math.fround(fixtureCases.find(fixture => fixture.id === row.id)?.heading)));
  return { expectedCases, actualCases: rows.length, passedCases: rows.filter(valid).length,
    identityMatched: identity, passed: identity && rows.every(valid) };
}

// Compare only physical reports; timing and process logs are retained separately.
export function comparePhysicalReports(first, second) {
  const evidence = [first, second].map(file => physicalReportAvailable(file) ? physicalReportEvidence(file) : null);
  const hashes = evidence.map(value => value?.decodedSha256 ?? null);
  return { firstSha256: hashes[0], secondSha256: hashes[1], firstStorageEvidence: evidence[0], secondStorageEvidence: evidence[1],
    identical: hashes[0] !== null && hashes[0] === hashes[1] };
}
