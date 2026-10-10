import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, mkdtempSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import test from 'node:test';
import { gzipSync } from 'node:zlib';
import { archiveGeneratedReport, physicalReportEvidence, readPhysicalJson } from '../scripts/rust-physical-archive-tools.mjs';
import { recoveryBrowserLimits, recoveryContractSha256 } from '../scripts/rust-recovery-browser-limits.mjs';
import { assembleRecoveryCases, inspectRecoveryCase, recoveryCasePlan } from '../scripts/rust-recovery-case-tools.mjs';
import { fingerprint, sha256, sourceHashes } from '../scripts/rust-release-tools.mjs';
import { finalizeRecoveryQualification } from '../scripts/finalize-rust-recovery-qualification.mjs';
import { comparePhysicalReports, freshRecoveryOutput, physicalCoverage, pinnedRecoveryProfile,
  recoveryBrowserStages, recoveryDecision, recoveryProfileFile, recoveryProfileSha256, requiredRecoveryStages,
  recoveryProcessEnvironment, strictStepIds, verifyEmittedProfile } from '../scripts/rust-recovery-tools.mjs';

const pinnedBytes = readFileSync(recoveryProfileFile);
const pinned = pinnedRecoveryProfile(pinnedBytes);
const fixtureCases = JSON.parse(readFileSync('rust/crates/sim/data/recovery-v1.json', 'utf8')).cases;
const freshTestRoot = label => {
  mkdirSync('evidence', { recursive: true });
  return mkdtempSync(resolve(`evidence/rust-recovery-${label}-`));
};

test('Rust and browser subprocesses use writable qualification temp directories with Windows key casing', () => {
  const directory = resolve('evidence/recovery-process-temp/tmp');
  const inherited = { Temp: 'readonly-temp', tMp: 'readonly-tmp', KEEP: 'preserved' };
  const env = recoveryProcessEnvironment(directory, inherited);
  assert.equal(env.TEMP, directory);
  assert.equal(env.TMP, directory);
  assert.equal(env.KEEP, 'preserved');
  assert.equal('Temp' in env, false);
  assert.equal('tMp' in env, false);
  assert.deepEqual(inherited, { Temp: 'readonly-temp', tMp: 'readonly-tmp', KEEP: 'preserved' });
});

test('recovery qualification pins the exact proven v24 bytes and every physics field', () => {
  assert.equal(sha256(pinnedBytes), recoveryProfileSha256);
  assert.equal(pinned.id, 'rust-physics-v24-com-restoration-release-fixtures');
  assert.equal(pinned.corrective_steps_enabled, true);
  assert.equal(pinned.recovery_enabled, true);
  assert.equal(verifyEmittedProfile(pinned, JSON.parse(JSON.stringify(pinned))), true);
  for (const changed of [{ ...pinned, dt_s: 1 / 120 }, { ...pinned, motor_scale: 0.8 },
    { ...pinned, recovery_enabled: false }, { ...pinned, corrective_steps_enabled: false },
    { ...pinned, id: 'different' }, { ...pinned, extra_solver_field: 1 }]) {
    assert.throws(() => pinnedRecoveryProfile(Buffer.from(JSON.stringify(changed))), /SHA-256 mismatch/);
    assert.throws(() => verifyEmittedProfile(pinned, changed), /profile mismatch/);
  }
  // A semantically equal reserialization still lacks the exact proven byte identity.
  assert.equal(fingerprint(pinned), fingerprint(JSON.parse(JSON.stringify(pinned))));
  assert.throws(() => pinnedRecoveryProfile(Buffer.from(JSON.stringify(pinned))), /SHA-256 mismatch/);
});

test('a recovery decision fails closed on missing, pending, failed or duplicate stages and never accepts a release', () => {
  const stages = requiredRecoveryStages.map(name => ({ name, passed: true }));
  const accepted = recoveryDecision(stages);
  assert.equal(accepted.qualified, true);
  assert.equal(accepted.complete, true);
  assert.equal(accepted.releaseAccepted, false);
  for (const incomplete of [stages.slice(1), [...stages, stages[0]], [...stages, { name: 'unrecognized', passed: true }],
    stages.map((stage, index) => index === 0 ? { ...stage, pending: true } : stage),
    stages.map((stage, index) => index === 0 ? { ...stage, passed: false } : stage),
    stages.map((stage, index) => index === 0 ? { ...stage, passed: 'true' } : stage)]) {
    assert.equal(recoveryDecision(incomplete).qualified, false);
    assert.equal(recoveryDecision(incomplete).releaseAccepted, false);
  }
});

test('coverage requires all 24 unique fixtures, exact headings, physical ownership and completed standing handoff', () => {
  const report = fixtureCases.map(row => ({ id: row.id, heading: Math.fround(row.heading), passed: true,
    dynamicOwnership: true, failure: null, handoffStandingSubsteps: 240 }));
  assert.equal(physicalCoverage('recovery', report, fixtureCases).passed, true);
  for (const changed of [report.slice(1), [...report, report[0]],
    report.map((row, index) => index === 0 ? { ...row, id: report[1].id } : row),
    report.map((row, index) => index === 0 ? { ...row, heading: row.heading + 1 } : row),
    report.map((row, index) => index === 0 ? { ...row, dynamicOwnership: false } : row),
    report.map((row, index) => index === 0 ? { ...row, handoffStandingSubsteps: 239 } : row),
    report.map((row, index) => index === 0 ? { ...row, failure: { field: 'contact' } } : row)]) {
    assert.equal(physicalCoverage('recovery', changed, fixtureCases).passed, false);
  }
});

test('coverage requires five complete cycles and every strict-step case', () => {
  const cycles = { passed: true, requiredCycles: 5, completedCycles: 5, firstFailure: null,
    cycles: Array.from({ length: 5 }, (_, index) => ({ cycle: index + 1, passed: true })) };
  assert.equal(physicalCoverage('cycles', cycles).passed, true);
  for (const changed of [{ ...cycles, completedCycles: 4 }, { ...cycles, cycles: cycles.cycles.slice(1) },
    { ...cycles, firstFailure: {} }, { ...cycles, passed: false },
    { ...cycles, cycles: cycles.cycles.map(row => ({ ...row, cycle: 1 })) }]) {
    assert.equal(physicalCoverage('cycles', changed).passed, false);
  }
  const steps = strictStepIds.map(id => ({ id, passed: true }));
  assert.equal(physicalCoverage('steps', steps).passed, true);
  assert.equal(physicalCoverage('steps', steps.slice(1)).passed, false);
  assert.equal(physicalCoverage('steps', steps.map(row => ({ ...row, id: 'substitute' }))).passed, false);
  assert.equal(physicalCoverage('steps', steps.map(row => ({ ...row, passed: false }))).passed, false);
});

test('physical repeat comparison uses retained physical report bytes and fails when either result is missing', () => {
  const root = freshTestRoot('repeat');
  const first = join(root, 'one.json'), second = join(root, 'two.json');
  assert.equal(comparePhysicalReports(first, second).identical, false);
  writeFileSync(first, '{"pose":[1,2,3]}');
  writeFileSync(second, '{"pose":[1,2,3]}');
  assert.equal(comparePhysicalReports(first, second).identical, true);
  writeFileSync(second, '{"pose":[1,2,4]}');
  assert.equal(comparePhysicalReports(first, second).identical, false);
  assert.equal(readFileSync(first, 'utf8'), '{"pose":[1,2,3]}');
});

test('qualification requires an explicit fresh output and rejects profile substitutions before running Rust', () => {
  for (const args of [[], ['--profile', recoveryProfileFile], ['one', 'two'], ['rust'], ['.']]) {
    const result = spawnSync(process.execPath, ['scripts/run-rust-recovery-gates.mjs', ...args],
      { encoding: 'utf8', windowsHide: true });
    assert.notEqual(result.status, 0);
    assert.match(result.stderr, /Usage:|fresh workspace output|Build output/);
  }
  const root = freshTestRoot('output');
  assert.throws(() => freshRecoveryOutput(root), /fresh workspace output/);
  assert.throws(() => freshRecoveryOutput('..'), /Build output/);
  assert.equal(freshRecoveryOutput(join(root, 'fresh')), join(root, 'fresh'));
});

test('a pinned profile mismatch preserves an unqualified partial receipt and frozen inputs', () => {
  const root = freshTestRoot('mismatch');
  const input = join(root, recoveryProfileFile);
  mkdirSync(resolve(input, '..'), { recursive: true });
  const altered = Buffer.from(JSON.stringify({ ...pinned, motor_scale: 0.5 }));
  writeFileSync(input, altered);
  const result = spawnSync(process.execPath, [resolve('scripts/run-rust-recovery-gates.mjs'), 'evidence/failed'],
    { cwd: root, encoding: 'utf8', windowsHide: true, timeout: 30000 });
  assert.equal(result.status, 1, result.stderr);
  const receipt = JSON.parse(readFileSync(join(root, 'evidence/failed/execution.json'), 'utf8'));
  assert.equal(receipt.qualified, false);
  assert.equal(receipt.releaseAccepted, false);
  assert.equal(receipt.partial, true);
  assert.equal(receipt.observedProfileSha256, sha256(altered));
  assert.equal(receipt.processTempDirectory, join(root, 'evidence/failed/tmp'));
  assert.ok(existsSync(receipt.processTempDirectory));
  assert.equal(receipt.runs.length, 0);
  assert.equal(receipt.stages.find(stage => stage.name === 'profile-pinned').passed, false);
  assert.match(receipt.error, /SHA-256 mismatch/);
  assert.equal(readFileSync(join(root, 'evidence/failed/source', recoveryProfileFile), 'utf8'), altered.toString());
  assert.ok(existsSync(join(root, 'evidence/failed')));
  const retry = spawnSync(process.execPath, [resolve('scripts/run-rust-recovery-gates.mjs'), 'evidence/failed'],
    { cwd: root, encoding: 'utf8', windowsHide: true });
  assert.notEqual(retry.status, 0);
  assert.match(retry.stderr, /fresh workspace output/);
  assert.equal(readFileSync(join(root, 'evidence/failed/execution.json'), 'utf8'), JSON.stringify(receipt, null, 2) + '\n');
});

function visualReviewFixture() {
  const originalDirectory = process.cwd(), root = freshTestRoot('visual-finalization');
  const write = (file, bytes) => { mkdirSync(dirname(file), { recursive: true }); writeFileSync(file, bytes); };
  const json = (file, value) => write(file, JSON.stringify(value, null, 2) + '\n');
  write(join(root, recoveryProfileFile), pinnedBytes);
  json(join(root, 'rust/crates/sim/data/recovery-v1.json'), { cases: fixtureCases });
  process.chdir(root);
  const output = join(root, 'evidence/qualification'), browserDirectory = join(output, 'browser');
  const before = sourceHashes();
  for (const file of Object.keys(before)) write(join(output, 'source', file), readFileSync(file));
  const profileFile = join(output, 'qualification-profile.json'), executableFile = join(output, 'executable/lh-acceptance');
  write(profileFile, pinnedBytes); write(executableFile, 'frozen executable identity fixture');
  const stages = requiredRecoveryStages.map(name => name === 'recordings-reviewed'
    ? { name, passed: false, pending: true } : { name, passed: true });
  const cycles = { passed: true, requiredCycles: 5, completedCycles: 5, firstFailure: null,
    cycles: Array.from({ length: 5 }, (_, index) => ({ cycle: index + 1, passed: true })) };
  const recovery = fixtureCases.map(row => ({ id: row.id, heading: Math.fround(row.heading), passed: true,
    dynamicOwnership: true, failure: null, handoffStandingSubsteps: 240 }));
  const steps = strictStepIds.map(id => ({ id, passed: true }));
  const physicalReports = [];
  const recoveryCaseRuns = [];
  for (const [name, file, report] of [['recovery', 'recovery.json', recovery], ['cycles', 'cycles.json', cycles],
    ['steps', 'disturbances.json', steps]]) {
    for (const index of [1, 2]) {
      const destination = join(output, `${name}-${index}`);
      if (name === 'recovery') {
        mkdirSync(destination, { recursive: true });
        const retained = [];
        for (const fixture of recoveryCasePlan(fixtureCases)) {
          const directory = join(destination, 'fixtures', fixture.key);
          json(join(directory, file), [report[fixture.ordinal - 1]]);
          json(join(directory, 'profile.json'), pinned);
          archiveGeneratedReport(join(directory, file), output);
          const entry = { repeat: index, processName: `recovery-${index}-${fixture.runSuffix}`, processPassed: true,
            profileMatched: true, ...inspectRecoveryCase(directory, fixture) };
          retained.push(entry); recoveryCaseRuns.push(entry);
        }
        assembleRecoveryCases(destination, retained, output);
      } else {
        json(join(destination, file), report);
        json(join(destination, 'profile.json'), pinned);
        archiveGeneratedReport(join(destination, file), output);
      }
    }
    physicalReports.push({ name, ...comparePhysicalReports(join(output, `${name}-1`, file), join(output, `${name}-2`, file)) });
  }
  const browser = { schema: 1, sourceBefore: before, sourceAfter: before, sourceFingerprint: fingerprint(before),
    inputProfileSha256: recoveryProfileSha256, executionPassed: true, passed: false, releaseAccepted: false,
    stages: stages.filter(stage => recoveryBrowserStages.includes(stage.name)), assets: {}, assetDirectories: [],
    fixtures: [], liveRuns: [], contractSha256: recoveryContractSha256, limits: recoveryBrowserLimits };
  for (const label of ['root', 'subpath']) {
    const directory = join(browserDirectory, `${label}-artifact`), bytes = `${label} immutable asset`;
    write(join(directory, 'index.html'), bytes);
    const hashes = { 'index.html': sha256(bytes) };
    browser.assetDirectories.push({ directory, hashes }); browser.assets[`${label}/index.html`] = hashes['index.html'];
  }
  browser.assetsFingerprint = fingerprint(browser.assets);
  const diagnosticWorker = 'frozen-diagnostic-worker';
  browser.fixtureWorker = { type: 'ready', schema: 4, maximumAdvanceTicks: 4, maximumBuffers: 3, workerIdentity: diagnosticWorker };
  const wasmRecovery = recovery.map(value => ({ ...value, stableReturnTimeS: 20, recoveryLimitS: 25, stableObservationS: 1 }));
  browser.wasmRecoveryCases = [];
  for (const fixture of recoveryCasePlan(fixtureCases)) {
    const path = join(browserDirectory, 'wasm-recovery-cases', fixture.key + '.json');
    const row = wasmRecovery.find(value => value.id === fixture.id);
    json(path, { kind: 'recovery', fixtureId: fixture.id, fixtureHeading: Math.fround(fixture.heading),
      profile: pinned, cases: [row], releaseAccepted: false });
    const storageEvidence = archiveGeneratedReport(path, output);
    browser.wasmRecoveryCases.push({ fixtureId: fixture.id, heading: Math.fround(fixture.heading), ordinal: fixture.ordinal,
      requestId: fixture.ordinal, responseRequestId: fixture.ordinal, workerIdentity: diagnosticWorker,
      path, completed: true, identityMatched: true, passed: true, sha256: storageEvidence.decodedSha256,
      storageEvidence, releaseAccepted: false });
  }
  browser.wasmRecoveryExecution = { expectedCases: 24, completedAttempts: 24, completedCaseReports: 24,
    sourceFingerprint: browser.sourceFingerprint, inputProfileSha256: browser.inputProfileSha256,
    contractSha256: browser.contractSha256,
    partial: false, workerIdentity: diagnosticWorker, releaseAccepted: false };
  json(join(browserDirectory, 'wasm-recovery-case-execution.json'), { schema: 1,
    ...browser.wasmRecoveryExecution, cases: browser.wasmRecoveryCases });
  const recoveryPath = join(browserDirectory, 'wasm-recovery.json');
  write(recoveryPath, `{"kind":"recovery","profile":${JSON.stringify(pinned)},"cases":[\n${wasmRecovery.map(value => JSON.stringify(value)).join(',\n')}\n],"releaseAccepted":false}\n`);
  const recoveryStorage = archiveGeneratedReport(recoveryPath, output);
  browser.fixtures.push({ kind: 'recovery', path: recoveryPath, sha256: recoveryStorage.decodedSha256,
    storageEvidence: recoveryStorage, summary: { cases: 24, passedCases: 24, profileApplied: true, releaseAccepted: false } });
  const cyclesPath = join(browserDirectory, 'wasm-cycles.json');
  json(cyclesPath, { kind: 'cycles', cases: [cycles], profile: pinned, releaseAccepted: false });
  const cyclesStorage = archiveGeneratedReport(cyclesPath, output);
  browser.fixtures.push({ kind: 'cycles', path: cyclesPath, sha256: cyclesStorage.decodedSha256, storageEvidence: cyclesStorage,
    requestId: 25, workerIdentity: diagnosticWorker,
    summary: { cases: 1, passedCases: 1, profileApplied: true, releaseAccepted: false } });
  const recordings = [];
  for (const label of ['root', 'subpath']) for (const backend of ['webgpu', 'webgl2']) {
    const directory = join(browserDirectory, `live-${label}-${backend}`), recording = join(directory, 'recording.webm');
    write(recording, `recording identity fixture ${label} ${backend}`);
    const screenshots = [];
    for (const cycle of [1, 2, 3, 4, 5]) for (const phase of ['impact', 'fall', 'recovered', 'hand-drag', 'released-standing']) {
      const name = `cycle-${cycle}-${phase}`, path = join(directory, name + '.png');
      write(path, `screenshot identity fixture ${label} ${backend} ${name}`);
      screenshots.push({ name, path, sha256: sha256(readFileSync(path)) });
    }
    const trial = { label, backend, passed: true, cycles: cycles.cycles, profileApplied: true, continuityPassed: true,
      recording, recordingSha256: sha256(readFileSync(recording)), screenshots };
    browser.liveRuns.push(trial); json(join(directory, 'report.json'), trial);
    recordings.push({ path: recording, sha256: trial.recordingSha256, reviewed: true,
      observations: 'Fixture visual review', screenshots: screenshots.map(value => ({ path: value.path,
        sha256: sha256(readFileSync(value.path)), reviewed: true })) });
  }
  const execution = { schema: 1, scope: 'rust-recovery-qualification', qualified: false, releaseAccepted: false,
    browserProcessPassed: true, sourceBefore: before, sourceAfter: before, sourceFingerprint: fingerprint(before),
    profileFile, profileFingerprint: fingerprint(pinned), inputProfileSha256: recoveryProfileSha256,
    executableFile, executableSha256: sha256(readFileSync(executableFile)), physicalReports, recoveryCaseRuns, stages,
    assets: browser.assets, assetsFingerprint: browser.assetsFingerprint, checkpoints: [{ passed: true }],
    runs: ['format', 'native-clippy', 'wasm-clippy', 'tooling', 'native-tests', 'native-build',
      ...[1, 2].flatMap(index => recoveryCasePlan(fixtureCases).map(fixture => `recovery-${index}-${fixture.runSuffix}`)),
      'cycles-1', 'cycles-2', 'steps-1', 'steps-2', 'browser-qualification']
      .map(name => ({ name, exit: 0, command: name === 'native-tests' ? ['cargo', '--test-threads=1'] : ['fixture-command'],
        testCount: name === 'native-tests' ? 76 : null })) };
  const executionFile = join(output, 'execution.json'), browserFile = join(browserDirectory, 'qualification.json'),
    reviewFile = join(output, 'visual-review.json');
  const save = () => {
    json(executionFile, execution); json(browserFile, browser);
    json(reviewFile, { schema: 1, kind: 'rust-recovery-visual-review', actualVisualInspection: true,
      reviewer: 'Fixture reviewer', observations: 'Fixture review of all required phases',
      executionSha256: sha256(readFileSync(executionFile)), browserQualificationSha256: sha256(readFileSync(browserFile)), recordings });
  };
  save();
  return { originalDirectory, root, output, execution, browser, recordings, reviewFile, save };
}

test('visual finalization preserves original receipts and accepts only unchanged fully passed physical evidence', () => {
  const fixture = visualReviewFixture();
  try {
    const originalExecution = readFileSync(join(fixture.output, 'execution.json'));
    const result = finalizeRecoveryQualification(fixture.output, fixture.reviewFile);
    assert.equal(result.qualified, true);
    assert.equal(result.releaseAccepted, false);
    const final = JSON.parse(readFileSync(result.output));
    assert.equal(final.qualified, true); assert.equal(final.partial, false); assert.equal(final.releaseAccepted, false);
    assert.equal(final.stages.find(stage => stage.name === 'recordings-reviewed').passed, true);
    assert.ok(readFileSync(join(fixture.output, 'execution.json')).equals(originalExecution));
    assert.throws(() => finalizeRecoveryQualification(fixture.output, fixture.reviewFile), /existing final/);
  } finally { process.chdir(fixture.originalDirectory); }
});

test('visual review cannot waive failed physics, another pending stage, changed input or stale reviewed media', () => {
  for (const [mutation, expected] of [
    [fixture => { fixture.execution.stages.find(stage => stage.name === 'cycles-five').passed = false; fixture.save(); }, /Failed physics/],
    [fixture => { fixture.execution.stages.find(stage => stage.name === 'native-repeat').pending = true; fixture.save(); }, /Pending physics/],
    [fixture => { writeFileSync(fixture.execution.profileFile, '{}'); }, /SHA-256 mismatch/],
    [fixture => { writeFileSync(fixture.execution.executableFile, 'changed'); }, /executable changed/],
    [fixture => { writeFileSync(fixture.execution.recoveryCaseRuns[0].reportFile, '[]'); }, /case cannot be finalized|case report changed/],
    [fixture => { writeFileSync(join(fixture.output, 'source', recoveryProfileFile), '{}'); }, /snapshot changed/],
    [fixture => { writeFileSync(join(fixture.output, 'browser/root-artifact/index.html'), 'changed'); }, /assets changed/],
    [fixture => { fixture.recordings[0].reviewed = false; fixture.save(); }, /recording review/],
    [fixture => { fixture.browser.fixtures[0].sha256 = 'substitute'; fixture.save(); }, /WASM report substitution/],
    [fixture => { const entry = fixture.execution.recoveryCaseRuns[0]; writeFileSync(entry.reportStorageEvidence.archiveFile, 'corrupt'); }, /case cannot be finalized|container SHA/],
    [fixture => { writeFileSync(fixture.recordings[0].screenshots[0].path, 'changed'); }, /screenshot changed/],
    [fixture => { const review = JSON.parse(readFileSync(fixture.reviewFile)); review.executionSha256 = 'stale';
      writeFileSync(fixture.reviewFile, JSON.stringify(review)); }, /another execution/],
  ]) {
    const fixture = visualReviewFixture();
    try {
      mutation(fixture);
      assert.throws(() => finalizeRecoveryQualification(fixture.output, fixture.reviewFile), expected);
      assert.equal(existsSync(join(fixture.output, 'final-qualification.json')), false);
    } finally { process.chdir(fixture.originalDirectory); }
  }
});


function replaceWasmPhysicalReport(fixture, refreshDescriptor) {
  const descriptor = fixture.browser.fixtures.find(value => value.kind === 'recovery');
  const report = readPhysicalJson(descriptor.path, descriptor.storageEvidence);
  report.cases[0].passed = false;
  const bytes = Buffer.from(JSON.stringify(report)), container = gzipSync(bytes);
  const storage = descriptor.storageEvidence, metadata = JSON.parse(readFileSync(storage.metadataFile));
  Object.assign(metadata, { archiveSha256: sha256(container), archiveBytes: container.length,
    decodedSha256: sha256(bytes), decodedBytes: bytes.length });
  writeFileSync(storage.archiveFile, container); writeFileSync(storage.metadataFile, JSON.stringify(metadata));
  if (refreshDescriptor) {
    descriptor.storageEvidence = physicalReportEvidence(descriptor.path);
    descriptor.sha256 = descriptor.storageEvidence.decodedSha256;
    fixture.save();
  }
}

test('finalization rejects a substituted standalone WASM file even when its archive is internally valid', () => {
  const fixture = visualReviewFixture();
  try {
    replaceWasmPhysicalReport(fixture, false);
    assert.throws(() => finalizeRecoveryQualification(fixture.output, fixture.reviewFile), /WASM report substitution/);
    assert.equal(existsSync(join(fixture.output, 'final-qualification.json')), false);
  } finally { process.chdir(fixture.originalDirectory); }
});

test('finalization independently checks archived WASM physics despite refreshed hashes and a passing summary', () => {
  const fixture = visualReviewFixture();
  try {
    replaceWasmPhysicalReport(fixture, true);
    assert.equal(fixture.browser.fixtures[0].summary.passedCases, 24);
    assert.throws(() => finalizeRecoveryQualification(fixture.output, fixture.reviewFile), /WASM recovery physics failed/);
    assert.equal(existsSync(join(fixture.output, 'final-qualification.json')), false);
  } finally { process.chdir(fixture.originalDirectory); }
});

function refreshSelectedCheckpoint(fixture) {
  writeFileSync(join(fixture.output, 'browser', 'wasm-recovery-case-execution.json'), JSON.stringify({ schema: 1,
    ...fixture.browser.wasmRecoveryExecution, cases: fixture.browser.wasmRecoveryCases }));
  fixture.save();
}

test('finalization independently rejects missing, substituted and reordered selected WASM evidence', () => {
  const mutations = [
    [fixture => { fixture.browser.wasmRecoveryCases.pop(); }, /Missing selected WASM case coverage/],
    [fixture => { fixture.browser.wasmRecoveryExecution.partial = true; }, /false/],
    [fixture => { fixture.browser.wasmRecoveryCases[0].workerIdentity = 'replacement'; }, /worker identity changed/],
    [fixture => { fixture.browser.wasmRecoveryCases[0].responseRequestId = 2; }, /request boundary changed/],
    [fixture => { fixture.browser.wasmRecoveryCases.reverse(); }, /case order or coverage differs/],
    [fixture => { fixture.browser.wasmRecoveryCases[0].sha256 = 'substitute'; }, /case hash changed/],
    [fixture => { fixture.browser.wasmRecoveryCases[0].path = fixture.browser.wasmRecoveryCases[1].path; }, /report path substitution/],
    [fixture => { fixture.browser.wasmRecoveryExecution.sourceFingerprint = 'substitute'; }, /frozen input identity changed/],
    [fixture => { fixture.browser.fixtureWorker.workerIdentity = 'replacement'; }, /startup identity changed/],
    [fixture => { fixture.browser.fixtures.find(value => value.kind === 'cycles').workerIdentity = 'replacement'; }, /cycles diagnostic worker changed/],
    [fixture => { fixture.browser.fixtures.find(value => value.kind === 'cycles').requestId = 26; }, /cycles request boundary changed/],
    [fixture => { fixture.browser.contractSha256 = 'substitute'; }, /contract identity changed/],
    [fixture => { fixture.browser.limits = { ...fixture.browser.limits, recoveryLimitSubsteps: 6001 }; }, /contract limits changed/],
  ];
  for (const [mutation, expected] of mutations) {
    const fixture = visualReviewFixture();
    try {
      mutation(fixture); refreshSelectedCheckpoint(fixture);
      assert.throws(() => finalizeRecoveryQualification(fixture.output, fixture.reviewFile), expected);
      assert.equal(existsSync(join(fixture.output, 'final-qualification.json')), false);
    } finally { process.chdir(fixture.originalDirectory); }
  }
});
