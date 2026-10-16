// Recovery-only evidence on isolated artifacts. Public defaults and release
// admission are deliberately outside this qualification.
import assert from 'node:assert/strict';
import { archiveGeneratedReport } from './rust-physical-archive-tools.mjs';
import { collectWasmRecoveryCases } from './rust-wasm-recovery-tools.mjs';
import { recoveryBrowserLimits, recoveryContractSha256, recoveryContractFile } from './rust-recovery-browser-limits.mjs';
export { recoveryBrowserLimits, recoveryContractSha256 };
import { spawn } from 'node:child_process';
import { createWriteStream, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';
import { fingerprint, persistReceipt, sha256, sourceHashes, treeHashes } from './rust-release-tools.mjs';

export const pinnedRecoverySha256 = '971661c88ae7258f157669d8e8a873161de40990274a9f2fd7d159f50006e00e';
export const recoveryBrowserStages = ['wasm-recovery-24', 'wasm-cycles-five', 'live-root-webgpu', 'live-root-webgl2',
  'live-subpath-webgpu', 'live-subpath-webgl2', 'recordings-reviewed', 'asset-unchanged',
  'browser-profile-applied', 'browser-worker-continuity'];

// Rust serializes f32 values after rounding; compare the actual applied fields
// at that precision while retaining the exact input byte hash separately.
export function sameProfile(actual, expected) {
  if (typeof expected === 'number') return typeof actual === 'number' && Number.isFinite(actual) && Math.fround(actual) === Math.fround(expected);
  if (expected && typeof expected === 'object') return actual !== null && typeof actual === 'object'
    && Object.keys(actual).length === Object.keys(expected).length
    && Object.entries(expected).every(([key, value]) => sameProfile(actual[key], value));
  return actual === expected;
}
export function continuityIdentity(value) {
  const evidence = value.recoveryProgress;
  assert.ok(evidence && evidence.ownershipUnchanged === true, 'Missing or changed actual WASM body/joint ownership');
  assert.equal(evidence.releaseAccepted, false, 'Qualification observer must never admit a release');
  assert.ok(typeof value.workerIdentity === 'string' && value.workerIdentity.length, 'Missing live worker identity');
  assert.equal(value.workerInstances, 1, 'Worker was restarted');
  assert.equal(evidence.generation, value.snapshot.stamp.generation, 'Observer generation differs');
  assert.ok(Array.isArray(evidence.bodyHandles) && evidence.bodyHandles.length === 25, 'Missing canonical body handles');
  assert.ok(Array.isArray(evidence.jointHandles) && evidence.jointHandles.length === 25 && evidence.jointHandles[0] === null
    && evidence.jointHandles.slice(1).every(value => value !== null), 'Missing canonical root plus 24 joint slots');
  return { workerIdentity: value.workerIdentity, workerInstances: value.workerInstances, generation: evidence.generation,
    bodyHandles: evidence.bodyHandles, jointHandles: evidence.jointHandles, profile: evidence.profile };
}
export function assertLiveObservation(value, initial, profile) {
  assert.equal(value.failure, null, 'Physical integrity: ' + JSON.stringify(value.failure));
  assert.equal(value.workerFault, false, 'Live worker fault');
  assert.equal(value.halted, false, 'Live renderer or physics halted');
  assert.equal(value.mode, 'protocol');
  assert.ok(sameProfile(value.recoveryProgress?.profile, profile), 'Pinned profile was not applied by WASM');
  assert.equal(value.recoveryProgress.firstFailure, null, 'Actual qualification observer failed');
  assert.deepEqual(continuityIdentity(value), initial, 'Live worker, generation, bodies, joints or profile were replaced');
  assert.equal(value.recoveryContinuity?.violations.length, 0, 'A worker packet violated continuity');
  assert.equal(value.snapshot.segments.length, 25);
  for (const pose of value.snapshot.segments) for (const key of ['position', 'rotation', 'linearVelocity', 'angularVelocity']) {
    assert.ok(Object.values(pose[key]).every(Number.isFinite), 'Non-finite physical pose');
  }
}
export function liveCycleDecision(cycle, limits = recoveryBrowserLimits) {
  return cycle.actualImpact === true && Number.isFinite(cycle.impactAtMs) && cycle.minimumPelvisY < 0.55 && cycle.fallAtMs !== null
    && cycle.recoveredAtMs !== null && Number.isSafeInteger(cycle.impactSubstep) && Number.isSafeInteger(cycle.stableReturnSubstep)
    && cycle.stableReturnSubstep >= cycle.impactSubstep && cycle.stableReturnSubstep - cycle.impactSubstep <= limits.recoveryLimitSubsteps
    && Number.isSafeInteger(cycle.physicalDescentSubstep) && cycle.physicalDescentSubstep >= cycle.impactSubstep
    && cycle.physicalDescentSubstep <= cycle.stableReturnSubstep
    && cycle.stableSubsteps >= limits.stableSubsteps && cycle.freshPressAccepted === true
    && cycle.dragMetres === limits.dragMetres && cycle.dragActualMs >= limits.dragActualMs && cycle.dragHeldSubsteps >= limits.dragSubsteps
    && cycle.handReleaseAccepted === true && cycle.postReleaseStableSubsteps >= limits.stableSubsteps
    && Number.isSafeInteger(cycle.releaseSubstep) && Number.isSafeInteger(cycle.postReleaseStableSubstep)
    && cycle.releaseSubstep >= cycle.stableReturnSubstep && cycle.postReleaseStableSubstep - cycle.releaseSubstep >= limits.stableSubsteps
    && cycle.postReleaseStableSubstep - cycle.releaseSubstep <= limits.recoveryLimitSubsteps
    && cycle.postReleaseActualMs >= limits.stableActualMs && cycle.failure === undefined;
}

// Operational transport/build limits; no physical acceptance threshold changes.
const fixtureTransportTimeoutMs = 2 * 60 * 60 * 1000;
const liveTransportTimeoutMs = 30 * 60 * 1000;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
async function build(output, label, artifact, mount, report, save) {
  const log = createWriteStream(join(output, label + '-build.log'));
  const result = await new Promise(resolve => {
    const child = spawn(process.execPath, ['scripts/build-rust-browser.mjs', artifact, mount], { windowsHide: true });
    child.stdout.pipe(log, { end: false }); child.stderr.pipe(log, { end: false });
    const timer = setTimeout(() => child.kill(), 1800000);
    let error;
    child.on('error', value => { error = String(value); });
    child.on('close', (exit, signal) => { clearTimeout(timer); log.end(); resolve({ exit, signal, error }); });
  });
  report.builds.push({ label, artifact, mount, ...result }); save();
  assert.ok(result.exit === 0 && !result.signal && !result.error, `${label} build failed; inspect retained log`);
}
async function serve(artifact, mount) {
  const child = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0', mount], { windowsHide: true });
  const origin = await new Promise((resolve, reject) => {
    const timer = setTimeout(() => { child.kill(); reject(new Error('Preview startup timeout')); }, 15000);
    let output = '';
    child.stdout.on('data', chunk => {
      output += chunk;
      const match = output.match(/Rust preview: (http:\/\/127\.0\.0\.1:\d+\/[^\s]*)/);
      if (match) { clearTimeout(timer); resolve(match[1]); }
    });
    child.on('error', error => { clearTimeout(timer); reject(error); });
    child.on('exit', code => { clearTimeout(timer); reject(new Error(`Preview exited ${code}`)); });
  });
  return { child, origin };
}
async function loadedAssets(page, assets) {
  return page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const response = await fetch(new URL(name, document.baseURI));
    if (!response.ok) throw new Error('Cannot load frozen asset ' + name);
    const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
    return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(assets));
}
async function wasmFixtureSession(browser, origin) {
  const context = await browser.newContext(), page = await context.newPage();
  try {
    await page.goto(new URL('recovery-qualification.html', origin).href);
    const metadata = await page.evaluate(async () => {
      const worker = new Worker(new URL('sim-worker.js', document.baseURI), { type: 'module' });
      const session = window.__lhRecoveryFixtures = { worker, pending: null, requestId: 0, fault: null };
      return await new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('Fixture worker startup timeout')), 45000);
        worker.onmessage = ({ data }) => {
          if (data.type === 'ready') { clearTimeout(timer); resolve(data); }
          else session.pending?.resolve(data);
        };
        worker.onerror = event => {
          session.fault = event.message; clearTimeout(timer);
          session.pending?.reject(new Error(event.message)); reject(new Error(event.message));
        };
      });
    });
    assert.equal(metadata.schema, 4); assert.equal(metadata.maximumAdvanceTicks, 4);
    assert.ok(typeof metadata.workerIdentity === 'string' && metadata.workerIdentity.length > 0);
    return { metadata,
      request: message => page.evaluate(async ({ message, timeout }) => {
        const session = window.__lhRecoveryFixtures;
        if (session.pending || session.fault) throw new Error('Diagnostic worker request is not available');
        if (message.requestId !== session.requestId + 1) throw new Error('Diagnostic worker request boundary mismatch');
        session.requestId = message.requestId;
        return await new Promise((resolve, reject) => {
          const timer = setTimeout(() => {
            session.fault = 'WASM fixture transport timed out'; session.pending = null;
            reject(new Error(session.fault));
          }, timeout);
          session.pending = {
            resolve(data) { clearTimeout(timer); session.pending = null; resolve(data); },
            reject(error) { clearTimeout(timer); session.pending = null; session.fault = String(error); reject(error); },
          };
          session.worker.postMessage({ type: 'calibrateFoundation', ...message });
        });
      }, { message, timeout: fixtureTransportTimeoutMs }),
      close: async () => { await page.evaluate(() => window.__lhRecoveryFixtures?.worker.terminate()).catch(() => {}); await context.close(); },
    };
  } catch (error) { await context.close(); throw error; }
}


async function liveRun(browser, output, label, backend, origin, assets, profile, save) {
  const directory = join(output, `live-${label}-${backend}`);
  mkdirSync(directory, { recursive: true });
  const trial = { label, backend, origin, cycles: [], errors: [], screenshots: [], passed: false,
    profileApplied: false, continuityPassed: false, releaseAccepted: false };
  const limits = recoveryBrowserLimits;
  const checkpoint = () => { persistReceipt(join(directory, 'report.json'), trial); save(); };
  const context = await browser.newContext({ viewport: { width: 1280, height: 900 },
    recordVideo: { dir: join(directory, 'recordings'), size: { width: 1280, height: 900 } } });
  await context.addInitScript(value => { window.__lhQualification = { kind: 'recovery', profile: value }; }, profile);
  const page = await context.newPage(), video = page.video();
  page.on('pageerror', error => { trial.errors.push(String(error)); checkpoint(); });
  page.on('console', message => { if (message.type() === 'error') { trial.errors.push(message.text()); checkpoint(); } });
  let initial;
  const capture = async (name, value) => {
    const path = join(directory, name + '.png');
    await page.screenshot({ path, fullPage: true });
    trial.screenshots.push({ name, path, sha256: sha256(readFileSync(path)), browserAtMs: value.browserAtMs, tick: value.snapshot?.stamp.tick });
    checkpoint();
  };
  const observe = async () => {
    const value = await page.evaluate(() => ({ ...window.__lhRust.diagnostics, browserAtMs: performance.now() }));
    trial.latest = value;
    assertLiveObservation(value, initial, profile);
    return value;
  };
  const command = action => page.evaluate(value => window.__lhRust.qualificationCommand(value), action);
  try {
    const url = new URL(origin); url.searchParams.set('renderer', backend);
    await page.goto(url.href);
    await page.waitForFunction(() => window.__lhRust?.diagnostics.ready, {}, { timeout: 45000 });
    const first = await page.evaluate(() => ({ ...window.__lhRust.diagnostics, browserAtMs: performance.now() }));
    trial.initial = first; initial = continuityIdentity(first);
    assertLiveObservation(first, initial, profile);
    assert.equal(first.backend, backend === 'webgpu' ? 'WebGPU' : 'WebGL2', 'Requested renderer was not used');
    trial.profileApplied = true;
    trial.loadedAssets = await loadedAssets(page, assets);
    assert.deepEqual(trial.loadedAssets, assets, 'Loaded bytes differ from isolated artifact');
    // The driver stays automatic for the entire run. The 25 s physical bound
    // uses integrated substeps; performance.now() supplies real hold durations
    // and watchdogs while the same worker advances through browser frames.
    await page.waitForFunction(stable => window.__lhRust.diagnostics.recoveryProgress?.stableSubsteps >= stable
      && window.__lhRust.diagnostics.canStrike, limits.stableSubsteps, { timeout: 30000 });
    await capture('initial-standing', await observe());
    for (let index = 1; index <= limits.repeatedCycles; index++) {
      const began = await observe();
      const cycle = { cycle: index, beganAtMs: began.browserAtMs, beganTick: began.tick, actualImpact: false,
        minimumPelvisY: began.snapshot.segments[0].position.y, fallAtMs: null, recoveredAtMs: null,
        stableSubsteps: 0, trace: [], passed: false };
      trial.cycles.push(cycle); checkpoint();
      try {
        assert.equal(began.canStrike, true, 'Apparatus not ready for next cycle');
        cycle.strikeAcknowledgement = await command('Strike');
        assert.equal(cycle.strikeAcknowledgement.rejected, null);
        const counters = began.snapshot.counters;
        let previousTraceTick = -1;
        while (true) {
          const value = await observe(), snapshot = value.snapshot;
          cycle.minimumPelvisY = Math.min(cycle.minimumPelvisY, snapshot.segments[0].position.y);
          if (snapshot.stamp.tick >= previousTraceTick + 6) {
            cycle.trace.push({ browserAtMs: value.browserAtMs, tick: snapshot.stamp.tick, timeS: snapshot.simulation_time_s,
              pelvisY: snapshot.segments[0].position.y, motion: snapshot.motion, counters: snapshot.counters,
              striker: snapshot.striker, recovery: snapshot.diagnostics.recovery, progress: value.recoveryProgress });
            previousTraceTick = snapshot.stamp.tick; checkpoint();
          }
          if (!cycle.actualImpact && snapshot.counters.strikes === counters.strikes + 1) {
            assert.ok(snapshot.striker.impact_id > began.snapshot.striker.impact_id && snapshot.striker.last_impact_impulse_ns > 0.25,
              'Strike counter has no actual solver impact');
            cycle.actualImpact = true; cycle.impactAtMs = value.browserAtMs;
            await capture(`cycle-${index}-impact`, value);
          }
          if (cycle.fallAtMs === null && snapshot.counters.falls > counters.falls) {
            cycle.fallAtMs = value.browserAtMs; cycle.fallTick = snapshot.stamp.tick;
            await capture(`cycle-${index}-fall`, value);
          }
          if (cycle.fallAtMs !== null && snapshot.counters.recoveries === counters.recoveries + 1
            && value.recoveryProgress.stableSubsteps >= limits.stableSubsteps && snapshot.motion === 'Upright') {
            cycle.recoveredAtMs = value.browserAtMs; cycle.recoveryTick = snapshot.stamp.tick;
            cycle.stableSubsteps = value.recoveryProgress.stableSubsteps;
            cycle.impactSubstep = value.recoveryProgress.impactSubstep;
            cycle.stableReturnSubstep = value.recoveryProgress.stableReturnSubstep;
            cycle.physicalDescentSubstep = value.recoveryProgress.physicalDescent?.integratedSubsteps;
            cycle.recovered = snapshot; await capture(`cycle-${index}-recovered`, value); break;
          }
          const elapsed = value.browserAtMs - (cycle.impactAtMs ?? cycle.beganAtMs);
          assert.ok(elapsed <= liveTransportTimeoutMs, 'Live transport stalled before the bounded substep observer completed');
          await wait(25);
        }
        assert.ok(cycle.actualImpact && cycle.minimumPelvisY < 0.55, 'Cycle has no solver contact and physical descent');
        assert.ok(Number.isSafeInteger(cycle.impactSubstep) && Number.isSafeInteger(cycle.stableReturnSubstep)
          && cycle.stableReturnSubstep >= cycle.impactSubstep && cycle.stableReturnSubstep - cycle.impactSubstep <= limits.recoveryLimitSubsteps,
        'Actual integrated recovery clock from solver impact exceeded 25 seconds');
        assert.ok(Number.isSafeInteger(cycle.physicalDescentSubstep) && cycle.physicalDescentSubstep >= cycle.impactSubstep
          && cycle.physicalDescentSubstep <= cycle.stableReturnSubstep, 'Actual substep observer did not measure physical descent');
        const standing = await observe();
        const segment = standing.snapshot.segments.findIndex(pose => pose.id === 'rightHand');
        assert.ok(segment >= 0, 'Canonical right hand missing');
        const press = index;
        cycle.grabAcknowledgement = await page.evaluate(value => window.__lhRust.qualificationGrabBegin(value), { press, segment });
        const originTarget = cycle.grabAcknowledgement.preparedOrigin;
        assert.ok(originTarget && Object.values(originTarget).every(Number.isFinite), 'Missing command-boundary physical hand origin');
        assert.equal(cycle.grabAcknowledgement.preparedStamp.tick, cycle.grabAcknowledgement.applied_tick,
          'Hand origin was prepared at another command boundary');
        assert.equal(cycle.grabAcknowledgement.preparedStamp.generation, cycle.grabAcknowledgement.stamp.generation);
        cycle.freshPressAccepted = cycle.grabAcknowledgement.rejected === null;
        assert.ok(cycle.freshPressAccepted, 'Fresh hand press rejected after recovery');
        const dragBegan = await page.evaluate(() => performance.now());
        cycle.moveAcknowledgement = await command({ GrabMove: { press,
          target: { ...originTarget, x: originTarget.x + limits.dragMetres } } });
        assert.equal(cycle.moveAcknowledgement.rejected, null, 'Hand drag rejected');
        // The actual target is held for both 0.25 browser seconds and all 60
        // integrated substeps. Delayed rendering cannot manufacture this hold.
        while (true) {
          const value = await observe();
          if (value.browserAtMs - dragBegan >= limits.dragActualMs && value.recoveryProgress.interactionHeldSubsteps >= limits.dragSubsteps) {
            cycle.dragHeldSubsteps = value.recoveryProgress.interactionHeldSubsteps; break;
          }
          assert.ok(value.browserAtMs - dragBegan <= 15000, 'Gentle hand target hold watchdog expired');
          await wait(10);
        }
        cycle.dragActualMs = (await page.evaluate(() => performance.now())) - dragBegan; cycle.dragMetres = limits.dragMetres;
        assert.ok(cycle.dragActualMs >= limits.dragActualMs, 'Hand target was held for less than 0.25 actual seconds');
        await capture(`cycle-${index}-hand-drag`, await observe());
        cycle.releaseAcknowledgement = await command({ GrabEnd: { press } });
        cycle.handReleaseAccepted = cycle.releaseAcknowledgement.rejected === null;
        assert.ok(cycle.handReleaseAccepted, 'Fresh hand release rejected');
        const released = await observe(); cycle.releaseAtMs = released.browserAtMs; cycle.releaseTick = released.tick;
        assert.ok(Number.isSafeInteger(cycle.releaseAcknowledgement.applied_tick), 'Missing actual acknowledged hand-release tick');
        // Commands apply at the completed tick boundary. Use that exact Rust
        // acknowledgement clock rather than a later browser poll's timestamp.
        cycle.releaseSubstep = cycle.releaseAcknowledgement.applied_tick * limits.substepsPerTick;
        assert.ok(Number.isSafeInteger(cycle.releaseSubstep) && cycle.releaseSubstep <= released.recoveryProgress.substepsCompleted,
          'Invalid acknowledged hand-release integrated clock');
        assert.equal(released.recoveryProgress.interactionReleaseAccepted, true, 'Actual WASM rejected the measured 2 cm hand hold/release');
        while (true) {
          const value = await observe();
          const sinceRelease = value.recoveryProgress.substepsCompleted - cycle.releaseSubstep;
          assert.ok(Number.isSafeInteger(sinceRelease) && sinceRelease >= 0 && sinceRelease <= limits.recoveryLimitSubsteps,
            'Actual integrated clock after hand release exceeded 25 seconds');
          if (value.browserAtMs - released.browserAtMs >= limits.stableActualMs
            && value.snapshot.stamp.tick - released.snapshot.stamp.tick >= limits.stableTicks
            && value.snapshot.motion === 'Upright' && value.recoveryProgress.stableSubsteps >= limits.stableSubsteps) {
            cycle.postReleaseStableSubsteps = value.recoveryProgress.stableSubsteps;
            cycle.postReleaseStableSubstep = value.recoveryProgress.substepsCompleted;
            cycle.postReleaseActualMs = value.browserAtMs - released.browserAtMs;
            cycle.final = value.snapshot; await capture(`cycle-${index}-released-standing`, value); break;
          }
          assert.ok(value.browserAtMs - released.browserAtMs <= liveTransportTimeoutMs, 'Hand release transport stalled before bounded substep completion');
          await wait(25);
        }
        cycle.passed = liveCycleDecision(cycle); assert.ok(cycle.passed, 'Physical cycle did not meet all gates'); checkpoint();
      } catch (error) { cycle.failure = String(error); checkpoint(); throw error; }
    }
    trial.passed = trial.cycles.length === limits.repeatedCycles && trial.cycles.every(cycle => cycle.passed) && trial.errors.length === 0;
  } catch (error) {
    trial.failure = String(error);
    try {
      trial.latest = await page.evaluate(() => ({ ...window.__lhRust?.diagnostics, browserAtMs: performance.now() }));
      await capture('failure', trial.latest);
    } catch (captureError) { trial.captureFailure = String(captureError); }
  } finally {
    if (initial && trial.latest?.recoveryProgress) {
      try {
        assert.equal(trial.latest.workerFault, false, 'Worker was terminated after a transport fault');
        assert.deepEqual(continuityIdentity(trial.latest), initial);
        assert.equal(trial.latest.recoveryContinuity?.violations.length, 0);
        trial.continuityPassed = true;
      } catch (error) { trial.continuityFailure = String(error); }
    }
    trial.finished = new Date().toISOString(); checkpoint();
    try {
      await context.close();
      if (video) { trial.recording = await video.path(); trial.recordingSha256 = sha256(readFileSync(trial.recording)); }
    } catch (error) { trial.recordingFailure = String(error); trial.passed = false; }
    checkpoint();
  }
  return trial;
}

export async function qualifyRecoveryBrowser(args) {
  assert.equal(args.length, 3, 'Usage: qualify-rust-recovery-browser.mjs <fresh-output/browser> <pinned-profile> <native-evidence-root>');
  const output = workspaceOutput(args[0]), profileFile = resolve(args[1]), nativeRoot = resolve(args[2]);
  assert.ok(!existsSync(output), 'Use a fresh browser evidence directory');
  const profileBytes = readFileSync(profileFile), inputProfileSha256 = sha256(profileBytes);
  assert.equal(inputProfileSha256, pinnedRecoverySha256, 'Recovery qualification requires exact pinned v24 profile bytes');
  assert.equal(sha256(readFileSync(resolve('rust/crates/sim/data/recovery-profile-v24.json'))), inputProfileSha256, 'Repository recovery profile differs from pinned input');
  const profile = JSON.parse(profileBytes), executionFile = join(nativeRoot, 'execution.json');
  const execution = JSON.parse(readFileSync(executionFile));
  assert.equal(execution.inputProfileSha256, inputProfileSha256, 'Native evidence used another profile');
  assert.equal(sha256(readFileSync(execution.profileFile)), inputProfileSha256, 'Native frozen profile bytes changed');
  const sourceBefore = sourceHashes();
  assert.equal(sourceBefore[recoveryContractFile], recoveryContractSha256, 'Frozen browser contract bytes changed');
  assert.deepEqual(sourceBefore, execution.sourceBefore, 'Native and browser source inputs differ');
  assert.equal(fingerprint(sourceBefore), execution.sourceFingerprint, 'Native source identity mismatch');
  mkdirSync(output, { recursive: true });
  const report = { schema: 1, started: new Date().toISOString(), sourceBefore, sourceFingerprint: fingerprint(sourceBefore),
    inputProfileSha256, profileFile, profile, contractSha256: recoveryContractSha256, limits: recoveryBrowserLimits, nativeExecution: { path: executionFile, sha256: sha256(readFileSync(executionFile)) },
    stages: recoveryBrowserStages.map(name => ({ name, passed: false, reason: 'Not completed' })),
    builds: [], assetDirectories: [], assets: {}, liveRuns: [], fixtures: [], wasmRecoveryCases: [],
    passed: false, partial: true, releaseAccepted: false };
  const save = () => persistReceipt(join(output, 'qualification.json'), report);
  const stage = (name, passed, detail = {}) => { Object.assign(report.stages.find(value => value.name === name), { passed, ...detail }); save(); };
  save();
  let browser;
  try {
    browser = await chromium.launch(browserLaunchOptions(true)); report.browser = browser.version(); save();
    for (const [label, mount] of [['root', '/'], ['subpath', '/LaboratoireHumain/']]) {
      const artifact = join(output, label + '-artifact');
      let server;
      try {
        await build(output, label, artifact, mount, report, save);
        writeFileSync(join(artifact, 'recovery-qualification.html'), `<!doctype html><base href="${mount}"><title>Recovery WASM qualification</title>\n`);
        const hashes = treeHashes(artifact);
        report.assetDirectories.push({ directory: artifact, hashes });
        for (const [name, digest] of Object.entries(hashes)) report.assets[`${label}/${name}`] = digest;
        report.assetsFingerprint = fingerprint(report.assets); save();
        server = await serve(artifact, mount);
        if (label === 'root') {
          let session;
          try {
            session = await wasmFixtureSession(browser, server.origin);
            report.fixtureWorker = session.metadata; save();
            const fixtureCases = JSON.parse(readFileSync('rust/crates/sim/data/recovery-v1.json', 'utf8')).cases;
            const recovery = await collectWasmRecoveryCases({ output, profile, fixtureCases,
              identity: { sourceFingerprint: report.sourceFingerprint, inputProfileSha256: report.inputProfileSha256,
                contractSha256: report.contractSha256 },
              workerIdentity: session.metadata.workerIdentity, request: session.request,
              checkpoint: (cases, execution) => { report.wasmRecoveryCases = cases; report.wasmRecoveryExecution = execution; save(); },
            });
            report.fixtures.push(recovery); save();
            stage('wasm-recovery-24', recovery.passed, { reason: 'Serial selected actual WASM fixture results', expectedCases: 24,
              evidence: 'wasm-recovery.json', completedCaseReports: recovery.execution.completedCaseReports });
            if (recovery.execution.transportFailure) throw new Error(recovery.execution.transportFailure);
            try {
              const packet = await session.request({ kind: 'cycles', profile, requestId: 25 });
              assert.equal(packet.requestId, 25); assert.equal(packet.workerIdentity, session.metadata.workerIdentity);
              assert.equal(packet.type, 'foundation', 'Actual WASM cycles failed: ' + packet.message);
              const fixture = packet.report, fixtureFile = join(output, 'wasm-cycles.json');
              persistReceipt(fixtureFile, fixture);
              const storageEvidence = archiveGeneratedReport(fixtureFile, output);
              report.fixtures.push({ kind: 'cycles', path: fixtureFile, sha256: storageEvidence.decodedSha256, storageEvidence,
                requestId: packet.requestId, workerIdentity: packet.workerIdentity,
                summary: { cases: fixture.cases?.length ?? 0, passedCases: fixture.cases?.filter(value => value.passed === true).length ?? 0,
                  profileApplied: sameProfile(fixture.profile, profile), requiredCycles: fixture.cases?.[0]?.requiredCycles ?? null,
                  completedCycles: fixture.cases?.[0]?.completedCycles ?? null, releaseAccepted: false } }); save();
              assert.equal(fixture.kind, 'cycles'); assert.equal(fixture.releaseAccepted, false);
              assert.ok(sameProfile(fixture.profile, profile), 'Actual fixture profile differs');
              assert.equal(fixture.cases.length, 1);
              assert.equal(fixture.cases[0].requiredCycles, recoveryBrowserLimits.repeatedCycles);
              assert.equal(fixture.cases[0].completedCycles, recoveryBrowserLimits.repeatedCycles);
              assert.equal(fixture.cases[0].cycles.length, recoveryBrowserLimits.repeatedCycles);
              stage('wasm-cycles-five', fixture.cases.every(value => value.passed === true), { reason: 'Actual WASM fixture result',
                expectedCases: 1, evidence: 'wasm-cycles.json' });
            } catch (error) { stage('wasm-cycles-five', false, { failure: String(error) }); }
          } catch (error) {
            if (!report.fixtures.some(value => value.kind === 'recovery')) stage('wasm-recovery-24', false, { failure: String(error) });
            stage('wasm-cycles-five', false, { failure: String(error) });
          } finally { await session?.close(); }
        }
        for (const backend of ['webgpu', 'webgl2']) {
          const trial = await liveRun(browser, output, label, backend, server.origin, hashes, profile, save);
          report.liveRuns.push(trial); stage(`live-${label}-${backend}`, trial.passed, { reason: trial.failure ?? 'Five physical cycles completed',
            completedCycles: trial.cycles.filter(value => value.passed).length, evidence: `live-${label}-${backend}/report.json` });
        }
      } catch (error) {
        report.buildFailures ??= []; report.buildFailures.push({ label, failure: String(error) }); save();
      } finally { server?.child.kill(); }
    }
    stage('browser-profile-applied', report.liveRuns.length === 4 && report.liveRuns.every(value => value.profileApplied)
      && report.fixtures.length === 2 && report.fixtures.every(value => value.summary.profileApplied === true),
    { reason: 'Actual WASM applied profile and raw pinned input identity', inputProfileSha256 });
    stage('browser-worker-continuity', report.liveRuns.length === 4 && report.liveRuns.every(value => value.continuityPassed),
      { reason: 'Every live worker packet plus actual substep body/joint ownership observer; partial runs retained',
        completedFiveCycleRuns: report.liveRuns.filter(value => value.cycles.length === recoveryBrowserLimits.repeatedCycles && value.cycles.every(cycle => cycle.passed)).length });
    stage('recordings-reviewed', false, { reason: 'Actual visual inspection required before admission; videos and screenshots are retained',
      recordings: report.liveRuns.map(value => ({ path: value.recording, sha256: value.recordingSha256,
        physicalPassed: value.passed, screenshots: value.screenshots })), reviewRequired: true, pending: true });
  } catch (error) { report.failure = String(error); }
  finally {
    try {
      report.sourceAfter = sourceHashes(); report.sourceUnchanged = fingerprint(report.sourceAfter) === report.sourceFingerprint;
      report.profileUnchanged = sha256(readFileSync(profileFile)) === inputProfileSha256;
      report.assetsUnchanged = report.assetDirectories.length === 2 && report.assetDirectories.every(value => fingerprint(treeHashes(value.directory)) === fingerprint(value.hashes));
    } catch (error) {
      report.finalHashFailure = String(error); report.sourceUnchanged = false; report.profileUnchanged = false; report.assetsUnchanged = false;
    }
    stage('asset-unchanged', report.assetsUnchanged, { reason: 'Independent post-run complete artifact tree hashes' });
    report.finished = new Date().toISOString();
    report.passed = report.sourceUnchanged && report.profileUnchanged && report.stages.every(value => value.passed === true);
    report.executionPassed = report.sourceUnchanged && report.profileUnchanged && report.stages.every(value => value.passed === true
      || value.name === 'recordings-reviewed' && value.pending === true);
    report.releaseAccepted = false; save(); await browser?.close();
  }
  return { output, passed: report.passed, executionPassed: report.executionPassed, pendingReview: true,
    releaseAccepted: false, stages: report.stages.map(({ name, passed }) => ({ name, passed })) };
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try { const result = await qualifyRecoveryBrowser(process.argv.slice(2)); console.log(JSON.stringify(result)); process.exitCode = result.executionPassed ? 0 : 1; }
  catch (error) { console.error(String(error)); process.exitCode = 1; }
}
