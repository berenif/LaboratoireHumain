// Recovery qualification is an opt-in evidence path and never accepts a release.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { copyFileSync, existsSync, lstatSync, mkdirSync, readFileSync, realpathSync, writeFileSync } from 'node:fs';
import { dirname, isAbsolute, join, relative, resolve, sep } from 'node:path';
import { assembleRecoveryCases, inspectRecoveryCase, recoveryCasePlan } from './rust-recovery-case-tools.mjs';
import { archiveGeneratedReport, readPhysicalJson } from './rust-physical-archive-tools.mjs';
import { prepareRustBuild, rustCheckCommands, rustEnvironment, rustToolchain } from './rust-toolchain.mjs';
import { fingerprint, nativeTestCount, persistReceipt, sha256, sourceHashes, treeHashes } from './rust-release-tools.mjs';
import { comparePhysicalReports, freshRecoveryOutput, physicalCoverage, pinnedRecoveryProfile,
  recoveryBrowserStages, recoveryDecision, recoveryProfileFile, recoveryProfileSha256,
  recoveryProcessEnvironment, requiredRecoveryStages, verifyEmittedProfile } from './rust-recovery-tools.mjs';

const args = process.argv.slice(2);
assert.ok(args.length === 1 && !args[0].startsWith('--'),
  'Usage: npm run verify:rust:recovery -- <fresh-output-directory>');
const out = freshRecoveryOutput(args[0]);
mkdirSync(out, { recursive: true });
const processTempDirectory = join(out, 'tmp');
mkdirSync(processTempDirectory);
const env = recoveryProcessEnvironment(processTempDirectory, rustEnvironment());
const stages = requiredRecoveryStages.map(name => ({ name, passed: false, pending: true, reason: 'Not executed' }));
const receipt = { schema: 1, scope: 'rust-recovery-qualification', started: new Date().toISOString(),
  command: [process.execPath, ...process.argv.slice(1)], node: process.version, partial: true,
  qualified: false, releaseAccepted: false, runs: [], stages, requiredRecoveryStages,
  inputProfileSha256: recoveryProfileSha256, processTempDirectory, checkpoints: [] };
const receiptPath = join(out, 'execution.json');
const save = () => persistReceipt(receiptPath, receipt);
const mark = (name, passed, details = {}) => {
  assert.ok(requiredRecoveryStages.includes(name), `Unexpected stage ${name}`);
  const index = stages.findIndex(stage => stage.name === name);
  stages[index] = { ...details, name, passed: passed === true, pending: details.pending === true };
  save();
};
function checkpoint(name) {
  const sourceFingerprint = fingerprint(sourceHashes());
  const profileSha256 = receipt.profileFile && existsSync(receipt.profileFile) ? sha256(readFileSync(receipt.profileFile)) : null;
  const executableSha256 = receipt.executableFile && existsSync(receipt.executableFile) ? sha256(readFileSync(receipt.executableFile)) : null;
  const passed = sourceFingerprint === receipt.sourceFingerprint
    && (!receipt.profileFile || profileSha256 === receipt.inputProfileSha256)
    && (!receipt.executableFile || executableSha256 === receipt.executableSha256);
  receipt.checkpoints.push({ name, sourceFingerprint, profileSha256, executableSha256, passed });
  save();
  return passed;
}
function run(name, executable, command, timeout = 300000, minimumTests = 0) {
  if (!checkpoint(`${name}-before`)) {
    receipt.runs.push({ name, command: [executable, ...command], exit: null, error: 'Frozen inputs changed', blocked: true });
    save();
    return false;
  }
  if (executable === 'cargo' && ['build', 'test', 'clippy'].includes(command[1])) {
    (receipt.buildFreshness ??= []).push({ name, ...prepareRustBuild() });
    save();
  }
  receipt.running = { name, command: [executable, ...command], started: new Date().toISOString() };
  save();
  const started = performance.now();
  const result = spawnSync(executable, command, { env, encoding: 'utf8', windowsHide: true,
    timeout, maxBuffer: 32 * 1024 * 1024 });
  // Keep stderr, timeout errors and partial output for every failed attempt.
  writeFileSync(join(out, `${name}.log`), (result.stdout ?? '') + (result.stderr ?? ''));
  const testCount = minimumTests ? nativeTestCount(result.stdout ?? '') : null;
  const record = { name, command: [executable, ...command], exit: result.status, signal: result.signal,
    error: result.error?.message ?? null, testCount, minimumTests, wallMs: performance.now() - started };
  receipt.runs.push(record);
  delete receipt.running;
  save();
  console.log(JSON.stringify(record));
  const inputsUnchanged = checkpoint(`${name}-after`);
  return result.status === 0 && !result.signal && !result.error
    && (!minimumTests || testCount >= minimumTests) && inputsUnchanged;
}

function runRecoveryCases(index, fixtureCases, profile) {
  const repeatName = `recovery-${index}`, destination = join(out, repeatName);
  mkdirSync(destination);
  const cases = [];
  for (const fixture of recoveryCasePlan(fixtureCases)) {
    const name = `${repeatName}-${fixture.runSuffix}`;
    const caseDirectory = join(destination, 'fixtures', fixture.key);
    // One native process at a time. Exit 1/timeout still permits the next case.
    const simulationPassed = run(name, receipt.executableFile, [caseDirectory,
      '--diagnose-recovery', '--fixture', fixture.id, '--profile', receipt.profileFile], 1800000);
    let archivePassed = false;
    const generatedReport = join(caseDirectory, 'recovery.json');
    if (existsSync(generatedReport)) {
      try { archiveGeneratedReport(generatedReport, out); archivePassed = true; }
      catch (error) { (receipt.archiveErrors ??= []).push({ name, error: String(error) }); }
    }
    const processPassed = simulationPassed && archivePassed;
    const inspected = inspectRecoveryCase(caseDirectory, fixture);
    let profileMatched = false;
    try { profileMatched = verifyEmittedProfile(profile, readReport(inspected.profileFile)); }
    catch (error) { (receipt.profileErrors ??= []).push({ name, error: String(error) }); }
    const entry = { repeat: index, processName: name, processPassed, profileMatched, ...inspected };
    cases.push(entry); (receipt.recoveryCaseRuns ??= []).push(entry);
    // Durable checkpoint before aggregation: verified archives retain every emitted case byte.
    persistReceipt(join(destination, 'case-execution.json'), { schema: 1, repeat: index,
      expectedCases: 24, completedAttempts: cases.length,
      completedCaseReports: cases.filter(value => value.jsonReadable).length,
      partial: cases.length < 24, sourceFingerprint: receipt.sourceFingerprint,
      inputProfileSha256: receipt.inputProfileSha256, executableSha256: receipt.executableSha256,
      cases, releaseAccepted: false });
    save();
  }
  let aggregate;
  try { aggregate = assembleRecoveryCases(destination, cases, out); }
  catch (error) {
    aggregate = { file: join(destination, 'recovery.json'), completeCaseEvidence: false, error: String(error) };
    (receipt.archiveErrors ??= []).push({ name: repeatName, error: String(error) });
  }
  (receipt.recoveryAggregates ??= []).push({ repeat: index, ...aggregate }); save();
  return aggregate.completeCaseEvidence && cases.every(value => value.processPassed && value.profileMatched);
}

function readReport(file) {
  try { return readPhysicalJson(file); }
  catch (error) { (receipt.reportErrors ??= []).push({ file, error: String(error) }); save(); return null; }
}

save();
try {
  receipt.sourceBefore = sourceHashes();
  receipt.sourceFingerprint = fingerprint(receipt.sourceBefore);
  save();
  for (const [file, hash] of Object.entries(receipt.sourceBefore)) {
    const destination = join(out, 'source', file);
    mkdirSync(dirname(destination), { recursive: true });
    const bytes = readFileSync(file);
    assert.equal(sha256(bytes), hash, `Source changed during freeze: ${file}`);
    writeFileSync(destination, bytes, { flag: 'wx' });
  }
  const profileBytes = readFileSync(recoveryProfileFile);
  receipt.observedProfileSha256 = sha256(profileBytes);
  save();
  const profile = pinnedRecoveryProfile(profileBytes);
  receipt.profileFile = join(out, 'qualification-profile.json');
  writeFileSync(receipt.profileFile, profileBytes, { flag: 'wx' });
  receipt.profileFingerprint = fingerprint(profile);
  receipt.pinnedProfileFile = recoveryProfileFile;
  mark('profile-pinned', true, { sha256: recoveryProfileSha256, profileId: profile.id,
    provenance: 'evidence/strict-stepping-2026-10-09/verified-a/input-profile.json',
    physicsFieldsChanged: [] });
  const fixtureCases = JSON.parse(readFileSync('rust/crates/sim/data/recovery-v1.json', 'utf8')).cases;
  const compiler = `+${rustToolchain}`;
  const version = spawnSync('rustc', [compiler, '--version', '--verbose'], { env, encoding: 'utf8', windowsHide: true });
  assert.equal(version.status, 0, version.error?.message ?? version.stderr);
  assert.match(version.stdout, /rustc 1\.89\.0/);
  receipt.toolchainVersion = version.stdout;
  receipt.nativeToolchain = compiler;
  const [format, nativeClippy, wasmClippy] = rustCheckCommands('lint');
  for (const [name, command] of [['format', format], ['native-clippy', nativeClippy], ['wasm-clippy', wasmClippy]]) {
    mark(name, run(name, 'cargo', command));
  }
  mark('tooling', run('tooling', process.execPath, ['--test', 'tests/rust-tooling.test.mjs',
    'tests/rust-release.test.mjs', 'tests/rust-recovery-qualification.test.mjs',
    'tests/rust-recovery-browser.test.mjs', 'tests/rust-wasm-recovery-tools.test.mjs', 'tests/rust-recovery-case-tools.test.mjs', 'tests/rust-physical-archive-tools.test.mjs', 'tests/rust-qualification-driver.test.mjs', 'tests/rust-build-freshness.integration.mjs']));
  mark('native-tests', run('native-tests', 'cargo', [...rustCheckCommands('test')[0], '--', '--test-threads=1'], 1800000, 76));
  const built = run('native-build', 'cargo', [compiler, 'build', '--offline', '--locked', '--release',
    '--manifest-path', 'rust/Cargo.toml', '-p', 'lh-acceptance']);
  mark('native-build', built);
  if (built) {
    const executable = resolve(`rust/target/release/lh-acceptance${process.platform === 'win32' ? '.exe' : ''}`);
    receipt.executableSha256 = sha256(readFileSync(executable));
    receipt.executableFile = join(out, 'executable', `lh-acceptance${process.platform === 'win32' ? '.exe' : ''}`);
    mkdirSync(dirname(receipt.executableFile), { recursive: true });
    copyFileSync(executable, receipt.executableFile);
    save();
    receipt.physicalReports = [];
    for (const check of [
      { name: 'recovery', stage: 'recovery-24', command: ['--diagnose-recovery'], report: 'recovery.json' },
      { name: 'cycles', stage: 'cycles-five', command: ['--diagnose-cycles'], report: 'cycles.json' },
      { name: 'steps', stage: 'strict-steps', command: ['--diagnose-disturbances', '--require-steps'], report: 'disturbances.json' },
    ]) {
      const attempts = [];
      // A failed physical gate still runs twice with the same frozen executable.
      for (const index of [1, 2]) {
        const name = `${check.name}-${index}`, destination = join(out, name);
        let processPassed = check.name === 'recovery' ? runRecoveryCases(index, fixtureCases, profile)
          : run(name, receipt.executableFile,
            [destination, ...check.command, '--profile', receipt.profileFile], 1800000);
        if (check.name !== 'recovery' && existsSync(join(destination, check.report))) {
          try { archiveGeneratedReport(join(destination, check.report), out); }
          catch (error) { processPassed = false; (receipt.archiveErrors ??= []).push({ name, error: String(error) }); save(); }
        }
        const report = readReport(join(destination, check.report));
        const coverage = physicalCoverage(check.name, report, fixtureCases);
        let profileMatched = false;
        try { profileMatched = verifyEmittedProfile(profile, readReport(join(destination, 'profile.json'))); }
        catch (error) { (receipt.profileErrors ??= []).push({ name, error: String(error) }); }
        attempts.push({ name, processPassed, profileMatched, ...coverage });
        save();
      }
      let comparison;
      try { comparison = comparePhysicalReports(join(out, `${check.name}-1`, check.report),
        join(out, `${check.name}-2`, check.report)); }
      catch (error) { comparison = { identical: false, error: String(error) };
        (receipt.reportErrors ??= []).push({ name: check.name, error: String(error) }); save(); }
      receipt.physicalReports.push({ name: check.name, report: check.report, attempts, ...comparison });
      mark(check.stage, attempts.every(attempt => attempt.processPassed && attempt.profileMatched && attempt.passed),
        { attempts, deterministic: comparison.identical });
    }
    mark('native-repeat', receipt.physicalReports.every(report => report.identical
      && report.attempts.every(attempt => attempt.profileMatched)), { reports: receipt.physicalReports });
  } else {
    for (const name of ['recovery-24', 'cycles-five', 'strict-steps', 'native-repeat']) {
      mark(name, false, { reason: 'Native build failed; no frozen executable' });
    }
  }

  const browserDirectory = join(out, 'browser');
  const browserScript = 'scripts/qualify-rust-recovery-browser.mjs';
  if (existsSync(browserScript)) {
    receipt.browserProcessPassed = run('browser-qualification', process.execPath,
      // Transport ceiling only; integrated-substep acceptance remains unchanged.
      [browserScript, browserDirectory, receipt.profileFile, out], 30 * 60 * 60 * 1000);
    const browser = readReport(join(browserDirectory, 'qualification.json'));
    if (browser) {
      receipt.browserEvidence = browser;
      assert.equal(browser.sourceFingerprint, receipt.sourceFingerprint, 'Browser source mismatch');
      assert.equal(browser.inputProfileSha256, receipt.inputProfileSha256, 'Browser profile mismatch');
      assert.equal(browser.releaseAccepted, false, 'Recovery evidence cannot accept a release');
      assert.ok(Array.isArray(browser.stages) && browser.stages.every(stage => recoveryBrowserStages.includes(stage.name)),
        'Unexpected browser qualification stage');
      assert.equal(fingerprint(browser.assets), browser.assetsFingerprint, 'Browser asset manifest mismatch');
      assert.ok(Array.isArray(browser.assetDirectories) && browser.assetDirectories.length > 0, 'Missing browser asset directories');
      for (const item of browser.assetDirectories) {
        const directory = resolve(item.directory), local = relative(browserDirectory, directory);
        assert.ok(local && !isAbsolute(local) && local !== '..' && !local.startsWith(`..${sep}`), 'Browser asset directory escapes evidence');
        assert.ok(!lstatSync(browserDirectory).isSymbolicLink() && !lstatSync(directory).isSymbolicLink(), 'Refuse linked browser assets');
        const actualLocal = relative(realpathSync(out), realpathSync(directory));
        assert.ok(actualLocal && !isAbsolute(actualLocal) && actualLocal !== '..' && !actualLocal.startsWith(`..${sep}`),
          'Browser asset directory resolves outside evidence');
        assert.deepEqual(treeHashes(directory), item.hashes, 'Browser assets changed after qualification');
      }
      receipt.assets = browser.assets;
      receipt.assetsFingerprint = browser.assetsFingerprint;
      receipt.assetDirectories = browser.assetDirectories;
      for (const name of recoveryBrowserStages) {
        const matching = browser.stages?.filter(stage => stage.name === name) ?? [];
        if (matching.length === 1) mark(name, matching[0].passed, { ...matching[0], name });
        else mark(name, false, { pending: true, reason: `Browser stage has ${matching.length} entries` });
      }
    }
  } else {
    receipt.browserBlockedBy = `Missing ${browserScript}`;
  }
} catch (error) {
  receipt.error = String(error);
  console.error(receipt.error);
  if (stages.find(stage => stage.name === 'profile-pinned').pending) mark('profile-pinned', false, { reason: receipt.error });
} finally {
  receipt.finished = new Date().toISOString();
  receipt.sourceAfter = sourceHashes();
  receipt.sourceUnchanged = receipt.sourceFingerprint !== undefined
    && fingerprint(receipt.sourceAfter) === receipt.sourceFingerprint;
  receipt.sourceSnapshotUnchanged = existsSync(join(out, 'source'))
    && fingerprint(treeHashes(join(out, 'source'), true)) === receipt.sourceFingerprint;
  mark('source-unchanged', receipt.sourceUnchanged && receipt.sourceSnapshotUnchanged
    && receipt.checkpoints.every(value => value.passed), { sourceSnapshotUnchanged: receipt.sourceSnapshotUnchanged,
    checkpointsPassed: receipt.checkpoints.every(value => value.passed) });
  const profileUnchanged = receipt.profileFile !== undefined && existsSync(receipt.profileFile)
    && sha256(readFileSync(receipt.profileFile)) === recoveryProfileSha256;
  mark('profile-unchanged', profileUnchanged && existsSync(recoveryProfileFile)
    && sha256(readFileSync(recoveryProfileFile)) === recoveryProfileSha256);
  mark('executable-unchanged', receipt.executableFile !== undefined && existsSync(receipt.executableFile)
    && sha256(readFileSync(receipt.executableFile)) === receipt.executableSha256);
  Object.assign(receipt, recoveryDecision(stages));
  receipt.partial = !receipt.complete;
  receipt.qualified = receipt.qualified && receipt.browserProcessPassed === true
    && receipt.error === undefined && receipt.checkpoints.every(value => value.passed);
  receipt.releaseAccepted = false;
  save();
}
console.log(JSON.stringify({ out, qualified: receipt.qualified, complete: receipt.complete,
  releaseAccepted: false, failed: receipt.failed, pending: receipt.pending, error: receipt.error }));
process.exitCode = receipt.qualified ? 0 : 1;
