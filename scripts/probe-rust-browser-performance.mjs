// Actual shipped worker/render path diagnostic. Admission also needs the full
// physics/input/snapshot-age/OS-memory/soak matrix; this script does not grant it.
import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { resolve, join, relative } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3] ?? 'http://127.0.0.1:5185/';
const backends = process.argv[4] && process.argv[4] !== 'all' ? [process.argv[4]] : ['webgl2', 'webgpu'];
assert.ok(backends.every(value => ['webgl2', 'webgpu'].includes(value)));
const profilePath = process.argv[5] ? resolve(process.argv[5]) : null;
const profile = profilePath ? JSON.parse(readFileSync(profilePath)) : null;
const artifact = process.argv[6] ? resolve(process.argv[6]) : null;
assert.ok(!profilePath || artifact, 'Profile diagnostics require the exact served artifact directory');
const contract = JSON.parse(readFileSync('docs/rust-rework-performance-v1.json'));
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), scope: 'Every worker core tick/controller phase and render interval in the real browser, protocol idle. Diagnostic, not release admission.',
  contract, cases: [], releaseAccepted: false, missing: ['Other physics/mode fixtures', 'Actual displayed snapshot age', 'Input-visible latency', 'OS process memory', 'Soak', 'Independent hardware admission'] };
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function files(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap(entry => {
    assert.ok(!entry.isSymbolicLink(), 'Refuse artifact symlink');
    const path = join(directory, entry.name);
    return entry.isDirectory() ? files(path) : [path];
  });
}
const assetHashes = () => artifact ? Object.fromEntries(files(artifact).map(path =>
  [relative(artifact, path).replaceAll('\\', '/'), hash(readFileSync(path))])) : null;
report.artifact = artifact;
report.assets = assetHashes();
report.profileOverride = profilePath ? { path: profilePath, sha256: hash(readFileSync(profilePath)), profile } : null;
const sourceFiles = ['rust/crates/sim/src/lib.rs', 'rust/crates/sim/src/runtime.rs', 'rust/crates/sim/src/profiling.rs',
  'rust/crates/sim/src/measurement.rs', 'rust/crates/worker/src/lib.rs', 'rust/browser/sim-worker.js',
  'rust/crates/browser/driver.js', 'rust/crates/browser/src/main.rs', 'rust/crates/renderer/src/gpu.rs',
  'rust/Cargo.toml', 'rust/Cargo.lock', 'docs/rust-rework-performance-v1.json', 'scripts/probe-rust-browser-performance.mjs',
  'rust/vendor/rapier3d/Cargo.toml', 'rust/vendor/rapier3d/src/dynamics/joint/generic_joint.rs',
  'rust/vendor/rapier3d/src/dynamics/solver/joint_constraint/mod.rs',
  'rust/vendor/rapier3d/src/dynamics/solver/joint_constraint/joint_constraint_builder.rs',
  'rust/vendor/rapier3d/src/dynamics/solver/joint_constraint/motor_joint_group.rs',
  'rust/vendor/rapier3d/src/dynamics/solver/staged_island_solver/joints.rs',
  'rust/vendor/rapier3d/src/dynamics/solver/staged_island_solver/solve.rs',
  'rust/crates/sim/Cargo.toml'];
report.sourceBefore = Object.fromEntries(sourceFiles.map(file => [file, hash(readFileSync(file))]));
function stats(values, deadline = null) {
  assert.ok(values.length && values.every(value => Number.isFinite(value) && value >= 0));
  const sorted = [...values].sort((a, b) => a - b);
  const percentile = p => sorted[Math.min(sorted.length - 1, Math.ceil(sorted.length * p) - 1)];
  return { count: values.length, mean: values.reduce((a, b) => a + b, 0) / values.length,
    p50: percentile(0.5), p95: percentile(0.95), p99: percentile(0.99), maximum: sorted.at(-1),
    misses: deadline === null ? null : values.filter(value => value > deadline).length, deadline };
}
const browser = await chromium.launch(browserLaunchOptions(true));
report.browser = browser.version();
try {
  const cdp = await browser.newBrowserCDPSession();
  try { report.systemGpu = (await cdp.send('SystemInfo.getInfo')).gpu; }
  catch (error) { report.systemGpuUnavailable = String(error); }
  for (const backend of backends) {
    const trial = { backend, errors: [] }; report.cases.push(trial);
    const context = await browser.newContext({ viewport: { width: 1280, height: 900 } });
    if (profile) await context.addInitScript(profile => {
      const post = Worker.prototype.postMessage;
      window.__lhAppliedProfiles = [];
      Worker.prototype.postMessage = function(message, ...args) {
        if (message?.type === 'initialize') {
          window.__lhAppliedProfiles.push(structuredClone(profile));
          message = { ...message, profile: structuredClone(profile) };
        }
        return Reflect.apply(post, this, [message, ...args]);
      };
    }, profile);
    const page = await context.newPage();
    page.on('pageerror', error => trial.errors.push(String(error)));
    page.on('console', message => { if (message.type() === 'error') trial.errors.push(message.text()); });
    const url = new URL(origin); url.searchParams.set('renderer', backend);
    await page.goto(url.href);
    await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.drainTelemetry, {}, { timeout: 45000 });
    if (report.assets) {
      trial.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
        const response = await fetch(new URL(name, document.baseURI));
        if (!response.ok) throw new Error('Cannot load ' + name);
        const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
        return [name, [...new Uint8Array(digest)].map(v => v.toString(16).padStart(2, '0')).join('')];
      }))), Object.keys(report.assets));
      assert.deepEqual(trial.loadedAssets, report.assets);
    }
    trial.samples = await page.evaluate(async ({ warmupS, measurementS }) => {
      await new Promise(resolve => setTimeout(resolve, warmupS * 1000));
      window.__lhRust.drainTelemetry();
      const initial = window.__lhRust.diagnostics, beganAtMs = performance.now();
      const packets = [], frames = [], memory = [];
      let droppedPackets = 0, droppedFrames = 0;
      const drain = () => {
        const data = window.__lhRust.drainTelemetry();
        packets.push(...data.packets); frames.push(...data.frames);
        droppedPackets = data.droppedPackets; droppedFrames = data.droppedFrames;
        memory.push({ atMs: performance.now(), mainWasmLinearBytes: data.mainWasmLinearBytes,
          jsHeap: performance.memory ? { used: performance.memory.usedJSHeapSize, total: performance.memory.totalJSHeapSize } : null });
      };
      while (performance.now() - beganAtMs < measurementS * 1000) {
        await new Promise(resolve => setTimeout(resolve, 200)); drain();
      }
      const endedAtMs = performance.now(); window.__lhRust.setManual(true); drain();
      return { beganAtMs, endedAtMs, timeOriginMs: performance.timeOrigin, initial,
        final: window.__lhRust.diagnostics, packets, frames, memory, droppedPackets, droppedFrames };
    }, contract.timing);
    await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding);
    trial.finalAfterInFlight = await page.evaluate(() => window.__lhRust.diagnostics);
    if (profile) {
      trial.appliedProfiles = await page.evaluate(() => window.__lhAppliedProfiles);
      assert.deepEqual(trial.appliedProfiles, [profile], 'Exactly one declared profile initialized the measured worker');
    }
    const data = trial.samples;
    assert.equal(data.final.backend.toLowerCase(), backend);
    assert.equal(data.droppedPackets, 0); assert.equal(data.droppedFrames, 0);
    assert.equal(data.final.failure, null); assert.deepEqual(trial.errors, []);
    const activePackets = data.packets.filter(packet => packet.receivedAtMs >= data.beganAtMs && packet.receivedAtMs <= data.endedAtMs && packet.completedTicks > 0);
    assert.ok(activePackets.length);
    assert.ok(activePackets.every(packet => packet.timing?.clockValid && packet.timing.ticks.filter(t => t.complete).length === packet.completedTicks));
    const ticks = activePackets.flatMap(packet => packet.timing.ticks.filter(t => t.complete));
    assert.ok(ticks.every(tick => tick.completedSubsteps === 4 && tick.controllerSubstepsMs.every(value => value !== null)));
    trial.metrics = {
      simulationWallRatio: (data.final.time - data.initial.time) / ((data.endedAtMs - data.beganAtMs) / 1000),
      coreTick: stats(ticks.map(tick => tick.elapsedMs), contract.timing.fullFourSubstepUpdateDeadlineMs),
      controllerFourSubsteps: stats(ticks.map(tick => tick.controllerMs), contract.timing.controllerDeadlineMs),
      controllerSubstep: stats(ticks.flatMap(tick => tick.controllerSubstepsMs), contract.timing.controllerDeadlineMs),
      phases: Object.fromEntries(['controllerMs', 'physicsMs', 'contactsMs', 'integrityMs', 'observerMs'].map(name => [name, stats(ticks.map(tick => tick[name]))])),
      workerBatch: stats(activePackets.map(packet => packet.workerUpdateMs)),
      // Shared parsing/output/packing cost is charged to the final tick in each
      // batch. postMessage copy/IPC is measured separately by round trip.
      chargedCompleteTick: stats(activePackets.flatMap(packet => {
        const complete = packet.timing.ticks.filter(tick => tick.complete);
        const shared = packet.workerUpdateMs - complete.reduce((sum, tick) => sum + tick.elapsedMs, 0);
        assert.ok(shared >= -1e-6);
        return complete.map((tick, index) => tick.elapsedMs + (index === complete.length - 1 ? Math.max(0, shared) : 0));
      }), contract.timing.fullFourSubstepUpdateDeadlineMs),
      workerRoundTrip: stats(activePackets.map(packet => packet.roundTripMs), contract.timing.workerRoundTripMaximumMs),
      frameIntervals: stats(data.frames.filter(frame => frame.atMs > data.beganAtMs && frame.intervalMs !== null).map(frame => frame.intervalMs), contract.timing.maximumFrameIntervalMs),
      frameCpu: stats(data.frames.map(frame => frame.cpuMs)),
      maxBacklogS: Math.max(...data.frames.map(frame => frame.backlogSeconds)),
      mainWasmMaximumBytes: Math.max(...data.memory.map(sample => sample.mainWasmLinearBytes)),
      workerWasmMaximumBytes: Math.max(...activePackets.map(packet => packet.wasmLinearBytes)),
    };
    const m = trial.metrics;
    trial.partialTimingChecksPass = m.chargedCompleteTick.misses === 0 && m.controllerFourSubsteps.misses === 0
      && m.simulationWallRatio >= contract.timing.minimumSimulationWallTimeRatio
      && m.workerRoundTrip.p99 <= contract.timing.workerRoundTripP99Ms && m.workerRoundTrip.misses === 0
      && m.frameIntervals.p95 <= contract.timing.frameIntervalP95Ms && m.frameIntervals.p99 <= contract.timing.frameIntervalP99Ms
      && m.frameIntervals.misses === 0;
    await page.screenshot({ path: join(output, backend + '-end.png'), fullPage: true });
    console.log(JSON.stringify({ backend, ratio: m.simulationWallRatio, tickMaximum: m.chargedCompleteTick.maximum,
      tickMisses: m.chargedCompleteTick.misses, phases: Object.fromEntries(Object.entries(m.phases).map(([name, metric]) => [name, metric.mean])),
      frameP95: m.frameIntervals.p95, partialTimingChecksPass: trial.partialTimingChecksPass }));
    await context.close();
  }
} catch (error) { report.failure = String(error); process.exitCode = 1; }
finally {
  report.finished = new Date().toISOString();
  report.sourceUnchanged = sourceFiles.every(file => hash(readFileSync(file)) === report.sourceBefore[file]);
  report.assetsUnchanged = JSON.stringify(assetHashes()) === JSON.stringify(report.assets);
  report.profileUnchanged = !profilePath || hash(readFileSync(profilePath)) === report.profileOverride.sha256;
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
if (!report.sourceUnchanged || !report.assetsUnchanged || !report.profileUnchanged) process.exitCode = 1;
