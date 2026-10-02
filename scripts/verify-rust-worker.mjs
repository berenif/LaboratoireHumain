import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const browser = await chromium.launch(browserLaunchOptions());
mkdirSync(output, { recursive: true });
const report = { scope: 'Actual dedicated-worker WASM lifecycle smoke; no rendering, full physics or performance admission', browser: browser.version(), passed: false, consoleErrors: [] };
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const sourceFiles = ['rust/Cargo.toml', 'rust/Cargo.lock', 'rust/crates/contracts/src/lib.rs', 'rust/crates/model/src/lib.rs',
  'rust/crates/sim/src/lib.rs', 'rust/crates/sim/src/runtime.rs', 'rust/crates/worker/src/lib.rs', 'rust/browser/sim-worker.js',
  'scripts/build-rust-worker.mjs', 'scripts/serve-rust-preview.mjs', 'scripts/verify-rust-worker.mjs'];
report.sourceBefore = Object.fromEntries(sourceFiles.map(file => [file, hash(readFileSync(file))]));
const artifact = resolve(process.argv[4] ?? 'rust/dist/worker');
report.localAssets = Object.fromEntries(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(file => [file, hash(readFileSync(resolve(artifact, file)))]));
try {
  const page = await browser.newPage();
  page.on('pageerror', error => report.consoleErrors.push(String(error)));
  await page.goto(process.argv[3] ?? 'http://127.0.0.1:5184');
  report.loadedAssets = await page.evaluate(async () => Object.fromEntries(await Promise.all(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(async file => {
    const bytes = await (await fetch(new URL(file, location.href))).arrayBuffer();
    const digest = await crypto.subtle.digest('SHA-256', bytes);
    return [file, [...new Uint8Array(digest)].map(v => v.toString(16).padStart(2, '0')).join('')];
  }))));
  assert.deepEqual(report.loadedAssets, report.localAssets, 'Browser must execute the captured artifacts');
  report.result = await page.evaluate(async () => {
    const worker = new Worker(new URL('sim-worker.js', location.href), { type: 'module' });
    let requestId = 0;
    const waiters = new Map();
    let ready;
    const boot = Promise.race([
      new Promise(resolve => { ready = resolve; }),
      new Promise((_, reject) => setTimeout(() => reject(new Error('Worker boot timeout')), 10000)),
    ]);
    const receipts = [];
    worker.onmessage = ({ data }) => {
      if (data.type === 'ready') { ready(data); return; }
      const waiter = waiters.get(data.requestId);
      waiters.delete(data.requestId);
      waiter?.(data);
    };
    const send = message => new Promise(resolve => {
      const id = ++requestId;
      const timer = setTimeout(() => { waiters.delete(id); resolve({ type: 'error', message: 'Worker reply timeout' }); }, 10000);
      waiters.set(id, data => { clearTimeout(timer); resolve(data); });
      worker.postMessage({ ...message, requestId: id });
    });
    const recycle = result => {
      if (result.poseBuffer) worker.postMessage({ type: 'recycle', bufferId: result.bufferId, poseBuffer: result.poseBuffer }, [result.poseBuffer]);
    };
    const check = (condition, detail) => { if (!condition) throw new Error(detail); receipts.push(detail); };
    const snapshot = async message => {
      const result = await send(message);
      if (result.type === 'error') throw new Error(result.message);
      check(result.poseBuffer.byteLength === 1300 && new Float32Array(result.poseBuffer).every(Number.isFinite), 'finite 25-body transferred pose buffer');
      recycle(result);
      return result.reply;
    };
    const metadata = await boot;
    check(metadata.schema === 4 && metadata.maximumBuffers === 3 && metadata.maximumAdvanceTicks === 4, 'bounded schema-4 worker boot');
    let reply = await snapshot({ type: 'initialize', mode: 'playground', heading: Math.PI / 3 });
    check(reply.snapshot.segments.length === 25, 'canonical assembly initializes in worker');
    reply = await snapshot({ type: 'advance', ticks: 4 });
    const at = reply.snapshot.stamp;
    let sequence = 0;
    const command = action => ({ stamp: { schema: 4, generation: at.generation, sequence: ++sequence, tick: at.tick }, action });
    const rejectedPacket = [command('Pause'), command('Reset')];
    rejectedPacket[1].stamp.generation = 0;
    const packetError = await send({ type: 'commands', commands: rejectedPacket });
    check(packetError.type === 'error', 'invalid packet rejected atomically');
    reply = await snapshot({ type: 'advance', ticks: 0 });
    check(!reply.paused && reply.acknowledgements.length === 0, 'rejected packet left no hidden queued intent');
    const hand = reply.snapshot.segments.findIndex(pose => pose.id === 'rightHand');
    const grab = press => ({ GrabBegin: { press, segment: hand, local_anchor: { x: 0, y: 0, z: 0 }, target: reply.snapshot.segments[hand].position } });
    reply = await snapshot({ type: 'commands', commands: [command(grab(41))] });
    check(reply.acknowledgements[0].rejected === null, 'physical grab begins through stamped WASM command');
    const future = command({ GrabMove: { press: 41, target: reply.snapshot.segments[hand].position } });
    future.stamp.tick += 2;
    await snapshot({ type: 'commands', commands: [future] });
    reply = await snapshot({ type: 'commands', commands: [command('Pause')] });
    check(reply.acknowledgements.some(ack => ack.stamp.sequence === future.stamp.sequence && ack.rejected === 'Cancelled'), 'pause cancels queued future pointer intent');
    const frozen = JSON.stringify(reply.snapshot.segments);
    const frozenTime = reply.snapshot.simulation_time_s;
    reply = await snapshot({ type: 'advance', ticks: 4 });
    check(reply.completed_ticks === 0 && reply.snapshot.simulation_time_s === frozenTime && JSON.stringify(reply.snapshot.segments) === frozen, 'pause preserves poses, velocities and simulation time');
    reply = await snapshot({ type: 'commands', commands: [command('Resume')] });
    reply = await snapshot({ type: 'commands', commands: [command(grab(41))] });
    check(reply.acknowledgements[0].rejected === 'Invalid', 'old physical press cannot revive a paused grab');
    reply = await snapshot({ type: 'commands', commands: [command(grab(42))] });
    check(reply.acknowledgements[0].rejected === null, 'fresh physical press accepted after resume');
    reply = await snapshot({ type: 'visible', visible: false });
    const hidden = JSON.stringify(reply.snapshot.segments);
    reply = await snapshot({ type: 'advance', ticks: 4 });
    check(reply.completed_ticks === 0 && JSON.stringify(reply.snapshot.segments) === hidden, 'hidden worker does not integrate');
    reply = await snapshot({ type: 'visible', visible: true });
    reply = await snapshot({ type: 'commands', commands: [command({ GrabMove: { press: 42, target: reply.snapshot.segments[hand].position } })] });
    check(reply.acknowledgements[0].rejected === 'Invalid', 'hiding cancelled the active grab');
    const invalid = await send({ type: 'advance', ticks: 5 });
    check(invalid.type === 'error', 'oversized advance rejected before integration');
    reply = await snapshot({ type: 'commands', commands: [command('Reset')] });
    check(reply.snapshot.stamp.generation === at.generation + 1 && reply.snapshot.stamp.tick === 0 && reply.snapshot.simulation_time_s === 0, 'reset establishes an isolated new generation');
    const stale = await send({ type: 'commands', commands: [command('Pause')] });
    check(stale.type === 'error' && stale.message.includes('Generation'), 'stale generation rejected by actual WASM runtime');
    reply = await snapshot({ type: 'commands', commands: [{ stamp: { schema: 4, generation: reply.snapshot.stamp.generation, sequence: 1, tick: 0 }, action: 'Strike' }] });
    check(reply.acknowledgements[0].rejected === 'Unsupported' && reply.snapshot.counters.strikes === 0, 'unported protocol strike cannot claim execution');
    const held = [];
    for (let i = 0; i < 3; i++) held.push(await send({ type: 'advance', ticks: 0 }));
    const saturated = await send({ type: 'advance', ticks: 1 });
    check(saturated.type === 'error' && saturated.message.includes('recycled'), 'three-buffer pool stops an unobservable advance');
    held.forEach(recycle);
    reply = await snapshot({ type: 'advance', ticks: 0 });
    check(reply.snapshot.stamp.tick === 0, 'buffer exhaustion did not advance simulation');
    const closed = await send({ type: 'shutdown' });
    check(closed.type === 'closed', 'worker tears down explicitly');
    worker.terminate();
    return { receipts, metadata, final: reply.snapshot };
  });
  assert.equal(report.consoleErrors.length, 0);
  report.passed = true;
} catch (error) {
  report.failure = String(error);
  throw error;
} finally {
  report.sourceUnchanged = sourceFiles.every(file => hash(readFileSync(file)) === report.sourceBefore[file]);
  report.passed &&= report.sourceUnchanged;
  writeFileSync(resolve(output, 'report.json'), JSON.stringify(report, null, 2));
  await browser.close();
}
console.log(JSON.stringify({ output, passed: report.passed, scope: report.scope }));
process.exitCode = report.passed ? 0 : 1;
