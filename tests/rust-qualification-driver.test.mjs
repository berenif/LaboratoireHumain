import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

// Runs the actual bridge against a deterministic browser/worker scheduler.
// It checks transport ordering; physical acceptance remains in real WASM.
async function driverHarness(qualified = true) {
  const messages = [], workers = [], timers = new Map();
  let clock = 0, timerId = 0, raf;
  const listenable = () => ({ addEventListener() {}, removeEventListener() {} });
  const canvas = () => ({ ...listenable(), setAttribute() {}, focus() {},
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }) });
  const host = { ...listenable(), replaceChildren() {} };
  const renderer = { diagnostics: () => JSON.stringify({ backend: 'WebGL2' }),
    select_region() {}, set_floor_enabled() {}, accept_snapshot() {}, resize() {}, draw: () => false,
    dispose() {}, free() {} };
  class Worker {
    constructor() { workers.push(this); }
    postMessage(message) { messages.push(structuredClone(message)); }
    terminate() { this.terminated = true; }
  }
  const profile = { recovery_enabled: true };
  const window = { ...listenable(), dispatchEvent() {}, lhBindings: { make_renderer: async () => renderer } };
  if (qualified) window.__lhQualification = { kind: 'recovery', profile };
  const context = vm.createContext({ window, Worker, ArrayBuffer, Float32Array, URL, structuredClone,
    location: { href: 'http://localhost/?renderer=webgl2' }, navigator: {}, devicePixelRatio: 1,
    document: { ...listenable(), hidden: false, baseURI: 'http://localhost/', getElementById: () => host, createElement: canvas },
    performance: { timeOrigin: 1000, now: () => clock }, CustomEvent: class { constructor(type, detail) { this.type = type; this.detail = detail; } },
    requestAnimationFrame: callback => { raf = callback; return 1; }, cancelAnimationFrame() {},
    setTimeout: (callback, ms) => { timers.set(++timerId, { callback, ms }); return timerId; },
    clearTimeout: id => timers.delete(id) });
  const source = readFileSync(new URL('../rust/crates/browser/driver.js', import.meta.url), 'utf8').replace(/^export /gm, '');
  vm.runInContext(source, context);
  await context.start_bridge();
  const worker = workers[0], bridge = window.__lhRust;
  const handIndex = 24;
  const snapshot = (request, tick, handX, acknowledgements = []) => {
    const zero = { x: 0, y: 0, z: 0 };
    const segments = Array.from({ length: 25 }, (_, index) => ({ id: index === handIndex ? 'rightHand' : `body${index}`,
      position: { x: index === handIndex ? handX : 0, y: 1, z: 0 }, rotation: { ...zero, w: 1 },
      linearVelocity: zero, angularVelocity: zero }));
    const poses = new Float32Array(25 * 13);
    const stamp = { schema: 4, generation: 1, sequence: request.requestId, tick };
    worker.onmessage({ data: { type: 'snapshot', requestId: request.requestId, bufferId: 0, poseBuffer: poses.buffer,
      transferBuffersAllocated: 3, workerAtMs: clock, workerTimeOriginMs: 1000, workerIdentity: 'same-physical-worker',
      reply: { mode: 'protocol', failure: null, paused: false, visible: true, completed_ticks: request.type === 'advance' ? 1 : 0,
        acknowledgements, snapshot: { stamp, simulation_time_s: tick / 60, segments, contacts: [],
          motion: 'Upright', counters: { strikes: 0, falls: 0, recoveries: 1, steps: 0 }, diagnostics: {},
          striker: { available: true, phase: 'idle', impact_id: 0, last_impact_impulse_ns: 0 } } },
      ...(qualified ? { recoveryProgress: { profile, generation: 1, bodyHandles: Array.from({ length: 25 }, (_, i) => [i, 0]),
        jointHandles: [null, ...Array.from({ length: 24 }, (_, i) => [i, 0])], ownershipUnchanged: true, releaseAccepted: false,
        firstFailure: null, substepsCompleted: tick * 4, stableSubsteps: 240 } } : {}) } });
  };
  worker.onmessage({ data: { type: 'ready', schema: 4, maximumBuffers: 3, maximumAdvanceTicks: 4 } });
  snapshot(messages.find(value => value.type === 'initialize'), 0, 0);
  const frame = ms => { clock = ms; raf(ms); };
  const latest = type => messages.findLast(value => value.type === type);
  const acknowledge = (request, tick, handX) => snapshot(request, tick, handX,
    request.commands.map(command => ({ stamp: command.stamp, applied_tick: tick, rejected: null })));
  return { bridge, worker, workers, messages, timers, snapshot, frame, latest, acknowledge, handIndex, close: () => bridge.dispose() };
}

test('qualification press waits for an outstanding advance and prepares the current physical anchor before the next tick', async () => {
  const h = await driverHarness();
  try {
    h.frame(20);
    const inFlight = h.latest('advance');
    assert.equal(inFlight.ticks, 1); assert.equal(h.bridge.diagnostics.outstanding, true);
    const oldOrigin = structuredClone(h.bridge.diagnostics.snapshot.segments[h.handIndex].position);
    const pressed = h.bridge.qualificationGrabBegin({ press: 1, segment: h.handIndex });
    assert.equal(h.bridge.diagnostics.queueDepth, 1);
    assert.equal(h.messages.filter(value => value.type === 'commands').length, 0, 'Cannot bypass the outstanding request');
    h.frame(40); // Accumulate another tick while the original advance is pending.
    h.snapshot(inFlight, 1, 0.001);
    const press = h.latest('commands'), command = press.commands[0];
    assert.equal(command.stamp.tick, 1); assert.equal(command.stamp.generation, 1);
    assert.equal(command.action.GrabBegin.target.x, 0.001);
    assert.ok(Math.abs(command.action.GrabBegin.target.x - oldOrigin.x) > 1e-5,
      'Regression fixture must invalidate a target prepared from the old snapshot');
    assert.deepEqual(command.action.GrabBegin.local_anchor, { x: 0, y: 0, z: 0 });
    assert.equal(h.messages.filter(value => value.type === 'advance').length, 1,
      'Pending press must be sent before the accumulated automatic tick');
    h.acknowledge(press, 1, 0.001);
    const nextTick = h.latest('advance'); assert.notEqual(nextTick.requestId, inFlight.requestId);
    h.snapshot(nextTick, 2, 0.004); // Advance again before the awaiting caller reads its result.
    const accepted = await pressed;
    assert.equal(accepted.rejected, null); assert.equal(accepted.applied_tick, 1);
    assert.equal(accepted.preparedStamp.tick, 1); assert.equal(accepted.preparedStamp.generation, 1);
    assert.equal(accepted.preparedOrigin.x, 0.001, 'Accepted origin remains tied to its command boundary');
    assert.equal(h.bridge.diagnostics.snapshot.segments[h.handIndex].position.x, 0.004);
    const target = { ...accepted.preparedOrigin, x: accepted.preparedOrigin.x + 0.02 };
    const moved = h.bridge.qualificationCommand({ GrabMove: { press: 1, target } });
    const move = h.latest('commands');
    assert.equal(move.commands[0].action.GrabMove.target.x, 0.021, 'Measure 2 cm from the accepted physical origin');
    assert.notEqual(move.commands[0].action.GrabMove.target.x, 0.024, 'Do not rebase the move to a later hand pose');
    h.acknowledge(move, 2, 0.004); assert.equal((await moved).rejected, null);
    assert.equal(h.workers.length, 1); assert.equal(h.bridge.diagnostics.generation, 1);
    assert.equal(h.messages.flatMap(value => value.commands ?? []).some(value => ['Pause', 'Reset', 'Initialize'].includes(value.action)), false);
  } finally { h.close(); }
});

test('raw qualification commands retain supplied targets and ordinary pages expose no press intent helper', async () => {
  const qualified = await driverHarness();
  try {
    const supplied = { GrabBegin: { press: 2, segment: qualified.handIndex, local_anchor: { x: 0, y: 0, z: 0 }, target: { x: 0.5, y: 1, z: 0 } } };
    const result = qualified.bridge.qualificationCommand(supplied);
    const request = qualified.latest('commands'); assert.deepEqual(request.commands[0].action, supplied);
    qualified.acknowledge(request, 0, 0); assert.equal((await result).preparedOrigin, undefined);
  } finally { qualified.close(); }
  const ordinary = await driverHarness(false);
  try {
    assert.equal(ordinary.bridge.qualificationGrabBegin, undefined);
    assert.equal(ordinary.bridge.qualificationCommand, undefined);
    assert.equal(ordinary.latest('initialize').qualification, undefined);
  } finally { ordinary.close(); }
});

test('qualification hand preparation rejects an invalid or nonhand segment without sending a grab', async () => {
  const h = await driverHarness();
  try {
    await assert.rejects(h.bridge.qualificationGrabBegin({ press: 0, segment: h.handIndex }), /Invalid qualification/);
    await assert.rejects(h.bridge.qualificationGrabBegin({ press: 1, segment: 25 }), /Invalid qualification/);
    await assert.rejects(h.bridge.qualificationGrabBegin({ press: 1, segment: 3 }), /physical hand snapshot/);
    assert.equal(h.messages.filter(value => value.type === 'commands').length, 0);
    assert.equal(h.workers.length, 1); assert.equal(h.bridge.diagnostics.workerFault, false);
  } finally { h.close(); }
});
