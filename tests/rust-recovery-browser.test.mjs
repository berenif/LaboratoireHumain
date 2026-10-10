import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';
import { assertLiveObservation, continuityIdentity, liveCycleDecision, recoveryBrowserStages, sameProfile } from '../scripts/qualify-rust-recovery-browser.mjs';

const frozenFixtures = JSON.parse(readFileSync('rust/crates/sim/data/recovery-v1.json', 'utf8')).cases;
const pinnedProfile = JSON.parse(readFileSync('rust/crates/sim/data/recovery-profile-v24.json', 'utf8'));

// Exercise the real JS worker message boundary with a deterministic WASM stub.
// Actual physical qualification uses the compiled WASM via the CLI hook.
async function workerHarness({ failRecoveryEnable = false } = {}) {
  const packets = [], constructed = [], calls = [], profile = { recovery_enabled: true, gain: 0.7 };
  let tick = 0;
  class WorkerRuntime {
    constructor(mode, heading, generation, floor, profileJson) {
      this.mode = mode; this.generation = Number(generation); this.profile = JSON.parse(profileJson ?? '{}');
      this.tick = 0; this.stableSubsteps = 0; constructed.push(this);
    }
    free() { calls.push('free'); }
    shutdown() { calls.push('shutdown'); }
    enable_quiet_measurement() { calls.push('quiet'); }
    enable_quiet_drift_trace() { calls.push('quiet-drift'); }
    enable_recovery_measurement() {
      calls.push('recovery');
      if (failRecoveryEnable) { failRecoveryEnable = false; throw new Error('Invalid'); }
    }
    quiet_progress() { return JSON.stringify({ completedTicks: this.tick }); }
    quiet_report() { return JSON.stringify({ passed: true }); }
    recovery_progress() { return JSON.stringify({ profile: this.profile, generation: this.generation,
      bodyHandles: Array.from({ length: 25 }, (_, index) => [index, 0]),
      jointHandles: [null, ...Array.from({ length: 24 }, (_, index) => [index, 0])],
      substepsCompleted: this.tick * 4, stableSubsteps: this.stableSubsteps,
      ownershipUnchanged: true, firstFailure: null, releaseAccepted: false }); }
    enqueue_batch(json) { calls.push({ commands: JSON.parse(json) }); }
    set_visible(value) { calls.push({ visible: value }); }
    native_output_ms() { return 0; }
    advance(count) {
      calls.push({ advance: count }); this.tick += count; this.stableSubsteps += count * 4;
      const xyz = { x: 0, y: 0, z: 0 };
      return JSON.stringify({ mode: this.mode, failure: null, paused: false, visible: true, completed_ticks: count,
        acknowledgements: [], snapshot: { stamp: { generation: this.generation, tick: this.tick },
          segments: Array.from({ length: 25 }, (_, index) => ({ id: `body${index}`, position: xyz,
            rotation: { ...xyz, w: 1 }, linearVelocity: xyz, angularVelocity: xyz })) } });
    }
    advance_profiled(count) { return JSON.stringify({ reply: JSON.parse(this.advance(count)), timings: [], clockValid: true,
      runtimeMs: 0, runtimeNonTickMs: 0 }); }
  }
  const scope = { ArrayBuffer, Float32Array, Number, JSON, Math, Error, WorkerRuntime,
    crypto: { randomUUID: () => 'actual-worker-instance-1' },
    performance: { timeOrigin: 1000, now: () => ++tick },
    init: async () => ({ memory: { buffer: new ArrayBuffer(65536) } }), wire_schema: () => 4,
    calibrate_foundation: (kind, profileJson) => {
      calls.push({ calibration: kind }); return JSON.stringify({ kind, profile: JSON.parse(profileJson), cases: [] });
    },
    calibrate_recovery_fixture: (id, profileJson) => {
      calls.push({ selectedFixture: id });
      const fixture = frozenFixtures.find(value => value.id === id);
      if (!fixture) throw new Error('Unknown recovery fixture');
      if (JSON.stringify(JSON.parse(profileJson)) !== JSON.stringify(pinnedProfile)) throw new Error('Unpinned profile');
      return JSON.stringify({ kind: 'recovery', fixtureId: id, fixtureHeading: Math.fround(fixture.heading),
        profile: pinnedProfile, cases: [{ id, heading: Math.fround(fixture.heading) }], releaseAccepted: false });
    },
    self: { postMessage(value) { packets.push(structuredClone(value)); }, close() { calls.push('close'); } } };
  const context = vm.createContext(scope);
  const source = readFileSync(new URL('../rust/browser/sim-worker.js', import.meta.url), 'utf8').replace(/^import .*;\r?\n/m, '');
  await vm.runInContext(`(async () => { ${source}\n })()`, context);
  let requestId = 0;
  const send = message => {
    const before = packets.length;
    scope.self.onmessage({ data: { requestId: ++requestId, ...message } });
    assert.equal(packets.length, before + 1, 'Request must produce one bounded reply');
    return packets.at(-1);
  };
  const recycle = packet => scope.self.onmessage({ data: { type: 'recycle', bufferId: packet.bufferId, poseBuffer: packet.poseBuffer } });
  return { profile, packets, calls, constructed, send, recycle, raw: message => scope.self.onmessage({ data: message }) };
}

test('recovery qualification initialization is explicit and distinct from quiet measurement', async () => {
  const worker = await workerHarness();
  const result = worker.send({ type: 'initialize', mode: 'protocol', heading: 0, generation: 7,
    profile: worker.profile, qualification: 'recovery', profiling: false });
  assert.equal(result.type, 'snapshot'); assert.equal(result.workerIdentity, 'actual-worker-instance-1');
  assert.equal(result.recoveryProgress.generation, 7);
  assert.deepEqual(result.recoveryProgress.profile, worker.profile);
  assert.equal(result.quietProgress, undefined);
  assert.deepEqual(worker.calls.filter(value => typeof value === 'string'), ['recovery']);
  assert.equal(result.poseBuffer.byteLength, 1300);
  worker.recycle(result);
  const next = worker.send({ type: 'advance', ticks: 4 });
  assert.equal(next.workerIdentity, result.workerIdentity);
  assert.deepEqual(next.recoveryProgress.bodyHandles, result.recoveryProgress.bodyHandles);
  assert.deepEqual(next.recoveryProgress.jointHandles, result.recoveryProgress.jointHandles);
  assert.equal(next.recoveryProgress.substepsCompleted, 16);
  assert.equal(worker.constructed.length, 1, 'Advance must keep the same runtime');
});

test('recovery floor, mode, profile and qualification rejection leave initialization atomic', async () => {
  const worker = await workerHarness();
  const common = { type: 'initialize', mode: 'protocol', heading: 0, profile: worker.profile, qualification: 'recovery' };
  for (const change of [{ floorEnabled: false }, { mode: 'playground' }, { profile: undefined },
    { profile: {} }, { profile: { recovery_enabled: false } }, { qualification: 'recoverry' }, { profile: [] }]) {
    assert.equal(worker.send({ ...common, ...change }).type, 'error');
    assert.equal(worker.constructed.length, 0);
  }
  const initialized = worker.send(common);
  assert.equal(initialized.type, 'snapshot'); assert.equal(worker.constructed.length, 1);
});

test('failing recovery observer frees the candidate and allows one clean initialization', async () => {
  const worker = await workerHarness({ failRecoveryEnable: true });
  const initial = { type: 'initialize', mode: 'protocol', heading: 0, profile: worker.profile, qualification: 'recovery' };
  assert.equal(worker.send(initial).type, 'error');
  assert.ok(worker.calls.includes('free'));
  const next = worker.send(initial);
  assert.equal(next.type, 'snapshot'); assert.ok(next.recoveryProgress);
  assert.equal(worker.send(initial).type, 'error');
  assert.equal(worker.constructed.length, 2, 'Second valid initialization must not replace live bodies');
});

test('ordinary and quiet initialization retain their existing wire behavior', async () => {
  for (const qualification of [undefined, 'quiet', 'quiet-drift']) {
    const worker = await workerHarness();
    const result = worker.send({ type: 'initialize', mode: 'playground', heading: 0, qualification });
    assert.equal(result.type, 'snapshot'); assert.equal(result.recoveryProgress, undefined);
    assert.equal(result.workerIdentity, undefined);
    assert.equal(Boolean(result.quietProgress), qualification !== undefined);
    assert.equal(worker.calls.includes('quiet'), qualification !== undefined);
    assert.equal(worker.calls.includes('quiet-drift'), qualification === 'quiet-drift');
  }
});

test('recovery measurement preserves buffer backpressure and fixture isolation', async () => {
  const worker = await workerHarness();
  const first = worker.send({ type: 'initialize', mode: 'protocol', heading: 0, profile: worker.profile,
    qualification: 'recovery', profiling: false });
  const second = worker.send({ type: 'advance', ticks: 0 });
  const third = worker.send({ type: 'advance', ticks: 0 });
  const before = worker.calls.filter(value => value.advance !== undefined).length;
  const exhausted = worker.send({ type: 'advance', ticks: 1 });
  assert.equal(exhausted.type, 'error'); assert.match(exhausted.message, /recycled pose buffer/);
  assert.equal(worker.calls.filter(value => value.advance !== undefined).length, before);
  for (const packet of [first, second, third]) worker.recycle(packet);
  const refused = worker.send({ type: 'calibrateFoundation', kind: 'recovery', profile: worker.profile });
  assert.equal(refused.type, 'error'); assert.match(refused.message, /uninitialized worker/);
  const after = worker.send({ type: 'advance', ticks: 0 });
  assert.equal(after.reply.snapshot.stamp.tick, 0); assert.equal(worker.constructed.length, 1);
});

test('continuity and profile assertions reject replaced physical ownership and unpinned fields', () => {
  const profile = { recovery_enabled: true, damping: 0.7 };
  assert.ok(sameProfile({ recovery_enabled: true, damping: Math.fround(0.7) }, profile));
  assert.equal(sameProfile({ ...profile, unpinned_force: 1 }, profile), false);
  assert.equal(sameProfile({ ...profile, damping: 0.8 }, profile), false);
  const value = { workerIdentity: 'worker-one', workerInstances: 1, mode: 'protocol', failure: null,
    workerFault: false, halted: false, snapshot: { stamp: { generation: 3 }, segments: [] },
    recoveryProgress: { profile, generation: 3, bodyHandles: Array.from({ length: 25 }, (_, index) => [index, 1]),
      jointHandles: [null, ...Array.from({ length: 24 }, (_, index) => [index, 1])], ownershipUnchanged: true, firstFailure: null, releaseAccepted: false },
    recoveryContinuity: { violations: [] } };
  const xyz = { x: 0, y: 0, z: 0 };
  value.snapshot.segments = Array.from({ length: 25 }, () => ({ position: xyz, rotation: { ...xyz, w: 1 }, linearVelocity: xyz, angularVelocity: xyz }));
  const identity = structuredClone(continuityIdentity(value));
  assert.doesNotThrow(() => assertLiveObservation(value, identity, profile));
  for (const mutate of [v => { v.workerInstances = 2; }, v => { v.workerIdentity = 'replacement'; },
    v => { v.recoveryProgress.bodyHandles[0] = [0, 2]; }, v => { v.recoveryProgress.jointHandles[1] = [0, 2]; },
    v => { v.recoveryProgress.ownershipUnchanged = false; }, v => { v.recoveryContinuity.violations.push({ tick: 1 }); }]) {
    const changed = structuredClone(value); mutate(changed);
    assert.throws(() => assertLiveObservation(changed, identity, profile));
  }
});

test('five-cycle admission requires actual impact, a fresh 2 cm hold and stable release', () => {
  const cycle = { actualImpact: true, impactAtMs: 1000, impactSubstep: 10, physicalDescentSubstep: 500, stableReturnSubstep: 6010, fallAtMs: 1200, recoveredAtMs: 99999,
    minimumPelvisY: 0.5, stableSubsteps: 240, freshPressAccepted: true, dragMetres: 0.02, dragHeldSubsteps: 60, dragActualMs: 250,
    handReleaseAccepted: true, releaseSubstep: 6100, postReleaseStableSubstep: 12100,
    postReleaseStableSubsteps: 240, postReleaseActualMs: 1000 };
  assert.equal(liveCycleDecision(cycle), true);
  for (const changed of [{ actualImpact: false }, { impactAtMs: undefined }, { impactSubstep: undefined },
    { stableReturnSubstep: 6011 }, { stableReturnSubstep: 9 },
    { physicalDescentSubstep: undefined }, { physicalDescentSubstep: 9 }, { physicalDescentSubstep: 6011 },
    { minimumPelvisY: 0.56 }, { stableSubsteps: 239 }, { freshPressAccepted: false }, { dragMetres: 0.019 },
    { dragActualMs: 249 }, { dragHeldSubsteps: 59 }, { dragHeldSubsteps: undefined }, { handReleaseAccepted: false }, { releaseSubstep: undefined },
    { handReleaseAccepted: undefined, releaseAccepted: true }, { releaseSubstep: 6009 },
    { postReleaseStableSubstep: undefined }, { postReleaseStableSubstep: 12101 }, { postReleaseStableSubstep: 6099 },
    { postReleaseStableSubstep: 6339 },
    { postReleaseStableSubsteps: 239 }, { failure: 'physical failure' }]) {
    assert.equal(liveCycleDecision({ ...cycle, ...changed }), false);
  }
  assert.equal(recoveryBrowserStages.length, 10); assert.equal(new Set(recoveryBrowserStages).size, 10);
});

test('selected recovery calls share one uninitialized worker and preserve the ordinary foundation API', async () => {
  const worker = await workerHarness();
  const identity = worker.packets[0].workerIdentity;
  assert.equal(identity, 'actual-worker-instance-1');
  for (const fixture of frozenFixtures) {
    const result = worker.send({ type: 'calibrateFoundation', kind: 'recovery', profile: pinnedProfile, fixtureId: fixture.id });
    assert.equal(result.type, 'foundation'); assert.equal(result.workerIdentity, identity);
    assert.equal(result.report.fixtureId, fixture.id);
    assert.equal(result.report.cases.length, 1); assert.equal(result.report.cases[0].id, fixture.id);
  }
  assert.equal(worker.constructed.length, 0, 'Selected diagnostics never initialize the live runtime');
  const ordinary = worker.send({ type: 'calibrateFoundation', kind: 'recovery', profile: worker.profile });
  assert.equal(ordinary.type, 'foundation'); assert.equal(ordinary.report.fixtureId, undefined);
  assert.equal(worker.calls.filter(value => value.selectedFixture).length, 24);
  assert.equal(worker.calls.filter(value => value.calibration === 'recovery').length, 1);
});

test('selected fixture request boundaries, bounds and trial isolation reject before unsafe calibration', async () => {
  const worker = await workerHarness(), fixtureId = frozenFixtures[0].id;
  const message = { type: 'calibrateFoundation', kind: 'recovery', profile: pinnedProfile, fixtureId };
  const first = worker.send(message);
  worker.raw({ ...message, requestId: first.requestId });
  assert.equal(worker.packets.at(-1).type, 'error'); assert.match(worker.packets.at(-1).message, /Stale/);
  assert.equal(worker.calls.filter(value => value.selectedFixture).length, 1);
  for (const change of [{ fixtureId: '' }, { fixtureId: 'x'.repeat(129) }, { fixtureId: '../case' },
    { fixtureId: 2 }, { fixtureId: null }, { kind: 'cycles' }]) {
    const result = worker.send({ ...message, ...change });
    assert.equal(result.type, 'error'); assert.match(result.message, /Invalid selected recovery/);
  }
  assert.equal(worker.calls.filter(value => value.selectedFixture).length, 1);
  const unknown = worker.send({ ...message, fixtureId: 'unknown-immutable-fixture' });
  assert.equal(unknown.type, 'error'); assert.match(unknown.message, /Unknown recovery/);
  const unpinned = worker.send({ ...message, profile: { ...pinnedProfile, solver_iterations: 1 } });
  assert.equal(unpinned.type, 'error'); assert.match(unpinned.message, /Unpinned/);
  const initialized = worker.send({ type: 'initialize', mode: 'protocol', heading: 0, profile: worker.profile,
    qualification: 'recovery', profiling: false });
  const before = worker.calls.filter(value => value.selectedFixture).length;
  assert.equal(worker.send(message).type, 'error'); assert.equal(worker.constructed.length, 1);
  assert.equal(worker.calls.filter(value => value.selectedFixture).length, before);
  assert.equal(initialized.recoveryProgress.substepsCompleted, 0);
});
