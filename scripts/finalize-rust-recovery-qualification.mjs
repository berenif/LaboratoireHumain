// Finalize only the retained visual-review checkpoint; physics is never rerun or waived.
import assert from 'node:assert/strict';
import { physicalReportEvidence, readPhysicalJson } from './rust-physical-archive-tools.mjs';
import { verifyWasmRecoveryCases } from './rust-wasm-recovery-tools.mjs';
import { recoveryBrowserLimits, recoveryContractSha256 } from './rust-recovery-browser-limits.mjs';
import { createHash } from 'node:crypto';
import { inspectRecoveryCase, recoveryCasePlan } from './rust-recovery-case-tools.mjs';
import { existsSync, lstatSync, readFileSync, realpathSync } from 'node:fs';
import { isAbsolute, join, relative, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { workspaceOutput } from './rust-browser-tools.mjs';
import { fingerprint, persistReceipt, sha256, sourceHashes, treeHashes } from './rust-release-tools.mjs';
import { comparePhysicalReports, physicalCoverage, pinnedRecoveryProfile, recoveryBrowserStages,
  recoveryDecision, recoveryProfileFile, recoveryProfileSha256, requiredRecoveryStages,
  verifyEmittedProfile } from './rust-recovery-tools.mjs';

const readJson = readPhysicalJson;
const digest = file => sha256(readFileSync(file));
function containedFile(root, file) {
  const resolved = resolve(file), local = relative(realpathSync(root), realpathSync(resolved));
  assert.ok(local && !isAbsolute(local) && local !== '..' && !local.startsWith(`..${sep}`),
    `Reviewed evidence is outside qualification: ${file}`);
  assert.ok(!lstatSync(resolved).isSymbolicLink(), 'Refuse linked reviewed evidence');
  return resolved;
}
function completeStages(stages, expected) {
  assert.ok(Array.isArray(stages));
  assert.deepEqual([...stages.map(stage => stage.name)].sort(), [...expected].sort(), 'Stage coverage differs');
  for (const stage of stages) {
    if (stage.name === 'recordings-reviewed') {
      assert.equal(stage.passed, false, 'Visual review was already finalized');
      assert.equal(stage.pending, true, 'Only a pending visual review can be finalized');
    } else {
      assert.equal(stage.passed, true, `Failed physics or validation stage: ${stage.name}`);
      assert.notEqual(stage.pending, true, `Pending physics or validation stage: ${stage.name}`);
    }
  }
}
function appliedProfile(actual, expected) {
  if (typeof expected === 'number') return typeof actual === 'number' && Number.isFinite(actual)
    && Math.fround(actual) === Math.fround(expected);
  if (expected && typeof expected === 'object') return actual !== null && typeof actual === 'object'
    && fingerprint(Object.keys(actual).sort()) === fingerprint(Object.keys(expected).sort())
    && Object.entries(expected).every(([name, value]) => appliedProfile(actual[name], value));
  return actual === expected;
}

export function finalizeRecoveryQualification(directory, reviewFile) {
  const output = workspaceOutput(directory), browserDirectory = join(output, 'browser');
  assert.ok(!lstatSync(output).isSymbolicLink(), 'Refuse linked qualification directory');
  const finalFile = join(output, 'final-qualification.json');
  assert.ok(!existsSync(finalFile), 'Refuse an existing final qualification receipt');
  const executionFile = join(output, 'execution.json'), browserFile = join(browserDirectory, 'qualification.json');
  const execution = readJson(executionFile), browser = readJson(browserFile), review = readJson(reviewFile);
  const receiptDigests = { execution: digest(executionFile), browser: digest(browserFile), review: digest(reviewFile) };
  assert.equal(execution.schema, 1);
  assert.equal(execution.scope, 'rust-recovery-qualification');
  assert.equal(execution.qualified, false);
  assert.equal(execution.releaseAccepted, false);
  assert.equal(browser.releaseAccepted, false);
  assert.equal(browser.contractSha256, recoveryContractSha256, 'Browser contract identity changed');
  assert.deepEqual(browser.limits, recoveryBrowserLimits, 'Browser contract limits changed');
  assert.equal(execution.error, undefined, 'An execution error cannot be waived by visual review');
  assert.equal(browser.failure, undefined, 'A browser error cannot be waived by visual review');
  completeStages(execution.stages, requiredRecoveryStages);
  completeStages(browser.stages, recoveryBrowserStages);
  assert.equal(execution.browserProcessPassed, true, 'Browser automation did not complete successfully');
  assert.equal(browser.executionPassed, true, 'Browser physics did not complete successfully');
  const fixtureCases = readJson('rust/crates/sim/data/recovery-v1.json').cases;
  const casePlan = recoveryCasePlan(fixtureCases);
  const expectedRuns = ['format', 'native-clippy', 'wasm-clippy', 'tooling', 'native-tests', 'native-build',
    ...[1, 2].flatMap(index => casePlan.map(fixture => `recovery-${index}-${fixture.runSuffix}`)),
    'cycles-1', 'cycles-2', 'steps-1', 'steps-2', 'browser-qualification'];
  assert.deepEqual(execution.runs.map(run => run.name).sort(), expectedRuns.sort(), 'Process coverage differs');
  for (const run of execution.runs) assert.ok(run.exit === 0 && !run.signal && !run.error && !run.blocked,
    `Failed process cannot be finalized: ${run.name}`);
  const nativeTests = execution.runs.find(run => run.name === 'native-tests');
  assert.ok(nativeTests.testCount >= 76 && nativeTests.command.includes('--test-threads=1'),
    'Missing serial native workspace test coverage');
  assert.ok(execution.checkpoints?.length > 0 && execution.checkpoints.every(value => value.passed === true),
    'Frozen input checkpoint failed');

  assert.equal(review.schema, 1);
  assert.equal(review.kind, 'rust-recovery-visual-review');
  assert.equal(review.actualVisualInspection, true, 'Actual visual inspection is required');
  assert.ok(typeof review.reviewer === 'string' && review.reviewer.trim().length > 0, 'Missing visual reviewer');
  assert.ok(typeof review.observations === 'string' && review.observations.trim().length > 0, 'Missing visual observations');
  assert.equal(review.executionSha256, receiptDigests.execution, 'Review refers to another execution receipt');
  assert.equal(review.browserQualificationSha256, receiptDigests.browser, 'Review refers to another browser receipt');

  const currentSources = sourceHashes();
  assert.deepEqual(currentSources, execution.sourceBefore, 'Current qualification source changed');
  assert.deepEqual(execution.sourceAfter, execution.sourceBefore, 'Source changed during qualification');
  assert.deepEqual(browser.sourceBefore, execution.sourceBefore, 'Browser source differs from native source');
  assert.deepEqual(browser.sourceAfter, execution.sourceBefore, 'Source changed during browser qualification');
  assert.equal(fingerprint(currentSources), execution.sourceFingerprint, 'Source identity mismatch');
  assert.equal(browser.sourceFingerprint, execution.sourceFingerprint);
  assert.deepEqual(treeHashes(join(output, 'source'), true), execution.sourceBefore, 'Frozen source snapshot changed');
  containedFile(output, execution.profileFile);
  containedFile(output, execution.executableFile);
  const profile = pinnedRecoveryProfile(readFileSync(execution.profileFile));
  assert.equal(digest(recoveryProfileFile), recoveryProfileSha256, 'Repository pinned profile changed');
  assert.equal(execution.inputProfileSha256, recoveryProfileSha256);
  assert.equal(browser.inputProfileSha256, recoveryProfileSha256);
  assert.equal(fingerprint(profile), execution.profileFingerprint);
  assert.equal(digest(execution.executableFile), execution.executableSha256, 'Frozen executable changed');
  assert.deepEqual(execution.physicalReports.map(report => report.name).sort(), ['cycles', 'recovery', 'steps']);
  for (const [name, reportFile] of [['recovery', 'recovery.json'], ['cycles', 'cycles.json'], ['steps', 'disturbances.json']]) {
    const recorded = execution.physicalReports.find(report => report.name === name);
    const comparison = comparePhysicalReports(join(output, `${name}-1`, reportFile), join(output, `${name}-2`, reportFile));
    assert.equal(comparison.identical, true, `Native ${name} reports are not deterministic`);
    for (const storage of ['firstStorageEvidence', 'secondStorageEvidence']) {
      assert.equal(comparison[storage].storage, 'gzip', 'Physical aggregate must be archived');
      assert.deepEqual(comparison[storage], recorded[storage], 'Physical aggregate storage identity changed');
      containedFile(output, comparison[storage].archiveFile); containedFile(output, comparison[storage].metadataFile);
    }
    assert.equal(comparison.firstSha256, recorded.firstSha256, `Native ${name} evidence changed`);
    assert.equal(comparison.secondSha256, recorded.secondSha256, `Native ${name} repeat changed`);
    for (const index of [1, 2]) {
      const destination = join(output, `${name}-${index}`);
      assert.equal(physicalCoverage(name, readJson(join(destination, reportFile)), fixtureCases).passed, true,
        `Native ${name} physics failed`);
      verifyEmittedProfile(profile, readJson(join(destination, 'profile.json')));

      if (name === 'recovery') {
        assert.equal(execution.recoveryCaseRuns.length, 48, 'Missing retained recovery case processes');
        const cases = execution.recoveryCaseRuns.filter(value => value.repeat === index);
        assert.deepEqual(cases.map(value => value.processName).sort(),
          casePlan.map(fixture => `recovery-${index}-${fixture.runSuffix}`).sort(), 'Recovery case process coverage differs');
        const aggregateHash = createHash('sha256'); aggregateHash.update('[\n'); let count = 0;
        for (const fixture of casePlan) {
          const caseDirectory = join(destination, 'fixtures', fixture.key);
          const recordedCase = cases.find(value => value.processName === `recovery-${index}-${fixture.runSuffix}`);
          const actual = inspectRecoveryCase(caseDirectory, fixture);
          assert.equal(recordedCase.directory, caseDirectory);
          assert.ok(recordedCase.processPassed && recordedCase.profileMatched && actual.identityMatched,
            'Failed or incomplete recovery case cannot be finalized');
          assert.equal(actual.reportSha256, recordedCase.reportSha256, 'Retained recovery case report changed');
          assert.equal(actual.reportStorageEvidence.storage, 'gzip', 'Recovery case was not durably archived');
          assert.deepEqual(actual.reportStorageEvidence, recordedCase.reportStorageEvidence, 'Retained case archive identity changed');
          containedFile(output, actual.reportStorageEvidence.archiveFile);
          containedFile(output, actual.reportStorageEvidence.metadataFile); containedFile(output, actual.profileFile);
          assert.equal(actual.profileSha256, recordedCase.profileSha256, 'Retained recovery case profile changed');
          verifyEmittedProfile(profile, readJson(actual.profileFile));
          for (const row of readPhysicalJson(actual.reportFile, recordedCase.reportStorageEvidence)) {
            if (count++) aggregateHash.update(',\n');
            aggregateHash.update(JSON.stringify(row, null, 2));
          }
        }
        aggregateHash.update('\n]\n');
        assert.equal(aggregateHash.digest('hex'), physicalReportEvidence(join(destination, reportFile)).decodedSha256,
          'Recovery aggregate differs from the retained case reports');
      }

    }
  }

  const assets = {};
  assert.equal(browser.assetDirectories.length, 2, 'Missing browser artifacts');
  for (const label of ['root', 'subpath']) {
    const directory = join(browserDirectory, `${label}-artifact`);
    const entry = browser.assetDirectories.find(value => resolve(value.directory) === directory);
    assert.ok(entry, `Missing ${label} artifact identity`);
    containedFile(output, directory);
    assert.deepEqual(treeHashes(directory), entry.hashes, `${label} browser assets changed`);
    for (const [name, hash] of Object.entries(entry.hashes)) assets[`${label}/${name}`] = hash;
  }
  assert.deepEqual(assets, browser.assets, 'Browser assets are incomplete');
  assert.deepEqual(assets, execution.assets, 'Native receipt browser assets differ');
  assert.equal(fingerprint(assets), browser.assetsFingerprint);
  assert.equal(fingerprint(assets), execution.assetsFingerprint);
  assert.equal(browser.fixtureWorker?.type, 'ready', 'Missing diagnostic WASM worker startup');
  assert.equal(browser.fixtureWorker.schema, 4); assert.equal(browser.fixtureWorker.maximumAdvanceTicks, 4);
  assert.equal(browser.fixtureWorker.maximumBuffers, 3);
  assert.equal(browser.fixtureWorker.workerIdentity, browser.wasmRecoveryExecution?.workerIdentity, 'Diagnostic WASM startup identity changed');
  for (const kind of ['recovery', 'cycles']) {
    const fixture = readJson(join(browserDirectory, `wasm-${kind}.json`));
    const recorded = browser.fixtures.find(value => value.kind === kind);
    assert.ok(recorded, `Missing recorded WASM ${kind} report`);
    const fixtureFile = join(browserDirectory, `wasm-${kind}.json`);
    assert.ok(isAbsolute(recorded.path), 'WASM descriptor path must be absolute');
    assert.equal(resolve(recorded.path), fixtureFile, 'WASM report path substitution');
    const storage = physicalReportEvidence(fixtureFile);
    assert.equal(storage.storage, 'gzip', 'WASM report must be archived');
    assert.equal(storage.decodedSha256, recorded.sha256, 'WASM report substitution');
    assert.deepEqual(storage, recorded.storageEvidence, 'WASM archive identity changed');
    containedFile(output, storage.archiveFile); containedFile(output, storage.metadataFile);
    assert.equal(fixture.kind, kind);
    assert.equal(fixture.releaseAccepted, false);
    assert.ok(appliedProfile(fixture.profile, profile), 'WASM fixture applied an unpinned profile');
    if (kind === 'cycles') {
      assert.equal(fixture.cases.length, 1); assert.equal(recorded.requestId, fixtureCases.length + 1, 'WASM cycles request boundary changed');
      assert.equal(recorded.workerIdentity, browser.fixtureWorker.workerIdentity, 'WASM cycles diagnostic worker changed');
    }
    assert.equal(physicalCoverage(kind === 'cycles' ? 'cycles' : 'recovery',
      kind === 'cycles' ? fixture.cases[0] : fixture.cases, fixtureCases).passed, true, `WASM ${kind} physics failed`);
    if (kind === 'recovery') verifyWasmRecoveryCases(browserDirectory, browser.wasmRecoveryCases,
      browser.wasmRecoveryExecution, profile, fixtureCases, storage, { sourceFingerprint: browser.sourceFingerprint,
        inputProfileSha256: browser.inputProfileSha256, contractSha256: browser.contractSha256 });
  }

  assert.equal(browser.liveRuns.length, 4, 'Missing live browser runs');
  assert.ok(Array.isArray(review.recordings) && review.recordings.length === 4, 'Review must cover all four browser recordings');
  const reviewedPaths = new Set();
  for (const label of ['root', 'subpath']) for (const backend of ['webgpu', 'webgl2']) {
    const trial = browser.liveRuns.find(value => value.label === label && value.backend === backend);
    assert.ok(trial && trial.passed === true && trial.cycles.length === 5 && trial.cycles.every(cycle => cycle.passed === true),
      `Live ${label}/${backend} physics failed`);
    assert.deepEqual(readJson(join(browserDirectory, `live-${label}-${backend}`, 'report.json')), trial,
      'Retained live browser report changed');
    assert.equal(trial.profileApplied, true);
    assert.equal(trial.continuityPassed, true);
    const recording = containedFile(output, trial.recording);
    assert.equal(digest(recording), trial.recordingSha256, 'Recorded video changed');
    const inspected = review.recordings.find(value => resolve(value.path) === recording);
    assert.ok(inspected && inspected.reviewed === true, 'Missing actual recording review');
    assert.equal(inspected.sha256, trial.recordingSha256, 'Review refers to another recording');
    assert.ok(typeof inspected.observations === 'string' && inspected.observations.trim().length > 0,
      'Missing recording observations');
    assert.ok(!reviewedPaths.has(recording), 'Duplicate recording review'); reviewedPaths.add(recording);
    assert.ok(Array.isArray(inspected.screenshots) && inspected.screenshots.length === trial.screenshots.length,
      'Missing reviewed cycle screenshots');
    const screenshotPaths = new Set();
    for (const screenshot of trial.screenshots) {
      const file = containedFile(output, screenshot.path);
      const viewed = inspected.screenshots.find(value => resolve(value.path) === file);
      assert.ok(viewed && viewed.reviewed === true, 'Missing actual screenshot review');
      assert.equal(screenshot.sha256, digest(file), 'Captured screenshot changed');
      assert.equal(viewed.sha256, screenshot.sha256, 'Reviewed screenshot changed');
      assert.ok(!screenshotPaths.has(file), 'Duplicate screenshot evidence'); screenshotPaths.add(file);
    }
    for (const cycle of [1, 2, 3, 4, 5]) for (const phase of ['impact', 'fall', 'recovered', 'hand-drag', 'released-standing']) {
      assert.ok(trial.screenshots.some(value => value.name === `cycle-${cycle}-${phase}`), 'Missing cycle phase visual evidence');
    }
  }
  const stages = execution.stages.map(stage => stage.name === 'recordings-reviewed'
    ? { name: stage.name, passed: true, pending: false, reviewer: review.reviewer, observations: review.observations,
      reviewReceipt: resolve(reviewFile), reviewReceiptSha256: digest(reviewFile) } : stage);
  const decision = recoveryDecision(stages);
  assert.equal(decision.qualified, true);
  assert.equal(digest(executionFile), receiptDigests.execution, 'Execution receipt changed during finalization');
  assert.equal(digest(browserFile), receiptDigests.browser, 'Browser receipt changed during finalization');
  assert.equal(digest(reviewFile), receiptDigests.review, 'Visual review changed during finalization');
  assert.equal(fingerprint(sourceHashes()), execution.sourceFingerprint, 'Source changed during finalization');
  assert.equal(digest(execution.profileFile), recoveryProfileSha256, 'Profile changed during finalization');
  assert.equal(digest(execution.executableFile), execution.executableSha256, 'Executable changed during finalization');
  for (const entry of browser.assetDirectories) assert.deepEqual(treeHashes(entry.directory), entry.hashes,
    'Browser assets changed during finalization');
  const final = { schema: 1, scope: execution.scope, finalized: new Date().toISOString(), ...decision, partial: false,
    stages, sourceFingerprint: execution.sourceFingerprint, sourceUnchanged: true, sourceSnapshotUnchanged: true,
    inputProfileSha256: recoveryProfileSha256, profileUnchanged: true, executableSha256: execution.executableSha256,
    executableUnchanged: true, assets, assetsFingerprint: fingerprint(assets), assetsUnchanged: true,
    execution: { path: executionFile, sha256: receiptDigests.execution },
    browserQualification: { path: browserFile, sha256: receiptDigests.browser },
    visualReview: { path: resolve(reviewFile), sha256: receiptDigests.review, reviewer: review.reviewer } };
  persistReceipt(finalFile, final);
  return { output: finalFile, qualified: true, releaseAccepted: false };
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    assert.equal(process.argv.length, 4, 'Usage: finalize-rust-recovery-qualification.mjs <evidence-directory> <visual-review.json>');
    console.log(JSON.stringify(finalizeRecoveryQualification(process.argv[2], process.argv[3])));
  } catch (error) { console.error(String(error)); process.exitCode = 1; }
}
