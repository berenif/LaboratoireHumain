import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

// These are bridge/selection tests using the actual driver source. Browser GPU,
// renderer and worker APIs are deterministic doubles, not GPU or physics evidence.
const driverSource = readFileSync(new URL('../rust/crates/browser/driver.js', import.meta.url), 'utf8')
  .replace(/^export /gm, '');

function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function mockGpu(completion = () => Promise.resolve()) {
  const calls = [];
  const gpu = {
    async requestAdapter() {
      calls.push('adapter');
      return {
        async requestDevice() {
          calls.push('device');
          return {
            queue: {
              submit() { calls.push('submit'); },
              onSubmittedWorkDone() { calls.push('work-done'); return completion(); },
            },
            createCommandEncoder() {
              return {
                beginRenderPass() { calls.push('render-pass'); return { end() {} }; },
                finish() { return {}; },
              };
            },
            destroy() { calls.push('destroy'); },
          };
        },
      };
    },
    getPreferredCanvasFormat() { return 'bgra8unorm'; },
  };
  const context = {
    configure() { calls.push('configure'); },
    getCurrentTexture() { return { createView: () => ({}) }; },
    unconfigure() { calls.push('unconfigure'); },
  };
  return { gpu, calls, context };
}

async function microtasksUntil(predicate, description) {
  for (let turn = 0; turn < 64; turn++) {
    if (predicate()) return;
    await Promise.resolve();
  }
  assert.ok(predicate(), description);
}

function driverHarness({ requested = 'auto', gpu = mockGpu(), diagnostics = {} } = {}) {
  const workers = [], messages = [], rendererCalls = [], timers = new Map(), canvases = [];
  const listenable = () => {
    const listeners = new Map();
    return {
      addEventListener(type, callback) {
        if (!listeners.has(type)) listeners.set(type, new Set());
        listeners.get(type).add(callback);
      },
      removeEventListener(type, callback) { listeners.get(type)?.delete(callback); },
      dispatchEvent(event) { for (const callback of listeners.get(event.type) ?? []) callback(event); },
    };
  };
  const host = { ...listenable(), children: [], replaceChildren(...children) { this.children = children; } };
  const createCanvas = () => {
    const canvas = {
      ...listenable(), setAttribute() {}, focus() {},
      getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }),
      getContext(kind) { assert.equal(kind, 'webgpu'); return gpu?.context; },
    };
    canvases.push(canvas);
    return canvas;
  };
  class Worker {
    constructor() { this.identity = `physical-worker-${workers.length + 1}`; workers.push(this); }
    postMessage(message) { messages.push(structuredClone(message)); }
    terminate() { this.terminated = true; }
  }
  const navigator = gpu ? { gpu: gpu.gpu } : {};
  const window = {
    ...listenable(),
    lhBindings: {
      async make_renderer(canvas, backend) {
        const record = { canvas, backend, snapshots: [], disposed: false, freed: false };
        rendererCalls.push(record);
        return {
          diagnostics: () => JSON.stringify({ backend: backend === 'webgpu' ? 'WebGPU' : 'WebGL2' }),
          select_region() {}, set_floor_enabled() {}, resize() {}, draw: () => false,
          accept_snapshot(snapshot) { record.snapshots.push(JSON.parse(snapshot)); },
          dispose() { record.disposeWasMounted = host.children.includes(canvas); record.disposed = true; },
          free() { record.freed = true; },
        };
      },
    },
  };
  let timerId = 0;
  const context = vm.createContext({
    window, navigator, Worker, ArrayBuffer, Float32Array, URL, structuredClone, devicePixelRatio: 1,
    location: { href: `http://localhost/?renderer=${requested}` },
    document: {
      ...listenable(), hidden: false, baseURI: 'http://localhost/',
      getElementById: () => host, createElement: createCanvas,
    },
    performance: { timeOrigin: 1000, now: () => 0 },
    CustomEvent: class { constructor(type, { detail }) { this.type = type; this.detail = detail; } },
    requestAnimationFrame: () => 1, cancelAnimationFrame() {},
    setTimeout(callback, ms) { timers.set(++timerId, { callback, ms }); return timerId; },
    clearTimeout(id) { timers.delete(id); },
  });
  vm.runInContext(driverSource, context);
  const started = context.start_bridge();
  const bridge = window.__lhRust, worker = workers[0];
  bridge.setManual(true);
  const latest = type => messages.findLast(message => message.type === type);
  let tick = 0;
  const snapshot = request => {
    const poses = new Float32Array(25 * 13);
    worker.onmessage({ data: {
      type: 'snapshot', requestId: request.requestId, bufferId: 0, poseBuffer: poses.buffer,
      transferBuffersAllocated: 3, workerAtMs: 0, workerTimeOriginMs: 1000, workerIdentity: worker.identity,
      reply: {
        mode: 'protocol', failure: null, paused: false, visible: true,
        completed_ticks: request.type === 'advance' ? request.ticks : 0, acknowledgements: [],
        snapshot: {
          stamp: { schema: 4, generation: 1, sequence: request.requestId, tick },
          simulation_time_s: tick / 40, segments: [], contacts: [], motion: 'Upright', diagnostics,
          counters: { strikes: 0, falls: 0, recoveries: 0, steps: 0 },
          striker: { available: true, phase: 'idle' },
        },
      },
    } });
  };
  worker.onmessage({ data: { type: 'ready', schema: 4, maximumBuffers: 3, maximumAdvanceTicks: 4 } });
  snapshot(latest('initialize'));
  return {
    bridge, worker, workers, messages, rendererCalls, gpu, navigator, canvases, host, timers, started,
    async advance(ticks) {
      const pending = bridge.advance(ticks);
      tick += ticks;
      snapshot(latest('advance'));
      await pending;
    },
    async select(backend, expectedBackend) {
      const changes = bridge.diagnostics.rendererChanges;
      bridge.action({ type: 'renderer', value: backend });
      await microtasksUntil(() => bridge.diagnostics.rendererChanges === changes + 1
        && bridge.diagnostics.backend === expectedBackend && bridge.diagnostics.ready,
      `Renderer switch to ${backend} must finish`);
    },
    continuity() {
      return { worker, identity: worker.identity, snapshot: bridge.diagnostics.snapshot, messageCount: messages.length,
        tick: bridge.diagnostics.tick, generation: bridge.diagnostics.generation, time: bridge.diagnostics.time };
    },
    assertContinuity(before) {
      assert.equal(workers.length, 1);
      assert.equal(workers[0], before.worker);
      assert.equal(worker.identity, before.identity);
      assert.notEqual(worker.terminated, true);
      assert.equal(bridge.diagnostics.workerInstances, 1);
      assert.equal(bridge.diagnostics.workerFault, false);
      assert.equal(bridge.diagnostics.snapshot, before.snapshot);
      assert.equal(bridge.diagnostics.tick, before.tick);
      assert.equal(bridge.diagnostics.generation, before.generation);
      assert.equal(bridge.diagnostics.time, before.time);
      assert.deepEqual(messages.slice(before.messageCount), [], 'Renderer changes must not send worker commands');
    },
    close: () => bridge.dispose(),
  };
}

const rejectingGpu = () => mockGpu(() => Promise.reject(new Error('GPU render queue rejected')));

test('Auto falls back to WebGL2 when the WebGPU render queue rejects', async t => {
  const h = driverHarness({ gpu: rejectingGpu() }); t.after(h.close);
  await h.started;
  assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgl2']);
  assert.equal(h.bridge.diagnostics.requestedRenderer, 'auto');
  assert.equal(h.bridge.diagnostics.backend, 'WebGL2');
  assert.equal(h.bridge.diagnostics.ready, true);
  assert.equal(h.bridge.diagnostics.halted, false);
  assert.equal(h.bridge.diagnostics.error, '');
  assert.match(h.bridge.diagnostics.rendererFallbackReason, /WebGPU render check failed:.*GPU render queue rejected/);
  assert.equal(h.navigator.gpu, h.gpu.gpu, 'Fallback must not disable the browser GPU API');
  assert.deepEqual(h.gpu.calls.slice(-4), ['submit', 'work-done', 'unconfigure', 'destroy']);
});

test('Explicit WebGPU rejection reports a graphics error without creating a renderer', async t => {
  const h = driverHarness({ requested: 'webgpu', gpu: rejectingGpu() }); t.after(h.close);
  await h.started;
  assert.deepEqual(h.rendererCalls, []);
  assert.equal(h.bridge.diagnostics.backend, '');
  assert.equal(h.bridge.diagnostics.requestedRenderer, 'webgpu');
  assert.equal(h.bridge.diagnostics.ready, false);
  assert.equal(h.bridge.diagnostics.halted, true);
  assert.equal(h.bridge.diagnostics.workerFault, false);
  assert.equal(h.bridge.diagnostics.rendererFallbackReason, '');
  assert.match(h.bridge.diagnostics.error, /Graphics unavailable:.*GPU render queue rejected.*Auto or WebGL2/);
  assert.equal(h.host.children.length, 0, 'A failed probe must not mount a renderer canvas');
});

for (const [requested, expectedBackend] of [['webgl2', 'WebGL2'], ['auto', 'WebGL2']]) {
  test(`Switching failed explicit WebGPU to ${requested} preserves the worker and its tick/generation`, async t => {
    const h = driverHarness({ requested: 'webgpu', gpu: rejectingGpu() }); t.after(h.close);
    await h.started;
    await h.advance(3);
    const before = h.continuity();
    await h.select(requested, expectedBackend);
    h.assertContinuity(before);
    assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgl2']);
    assert.equal(h.bridge.diagnostics.halted, false);
    assert.equal(h.bridge.diagnostics.error, '');
    assert.equal(h.bridge.diagnostics.requestedRenderer, requested);
    assert.equal(h.rendererCalls[0].snapshots.at(-1).stamp.tick, 3);
    assert.equal(h.rendererCalls[0].snapshots.at(-1).stamp.generation, 1);
    assert.equal(h.gpu.calls.filter(call => call === 'adapter').length, requested === 'auto' ? 2 : 1);
  });
}

test('Auto without a browser GPU API directly creates WebGL2', async t => {
  const h = driverHarness({ gpu: null }); t.after(h.close);
  await h.started;
  assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgl2']);
  assert.equal(h.bridge.diagnostics.ready, true);
  assert.equal(h.bridge.diagnostics.rendererFallbackReason, '');
  assert.equal(h.canvases.length, 1, 'Missing GPU API must not create a probe canvas');
});

test('Repeated Auto preserves a working WebGL2 fallback after a known GPU failure', async t => {
  const h = driverHarness({ gpu: rejectingGpu() }); t.after(h.close);
  await h.started; await h.advance(2);
  const before = h.continuity(), renderer = h.rendererCalls[0];
  const changes = h.bridge.diagnostics.rendererChanges;
  h.bridge.action({ type: 'renderer', value: 'auto' });
  assert.equal(h.bridge.diagnostics.ready, true);
  assert.equal(h.bridge.diagnostics.backend, 'WebGL2');
  assert.equal(h.bridge.diagnostics.rendererChanges, changes);
  assert.equal(h.rendererCalls.length, 1);
  assert.equal(renderer.disposed, false);
  assert.equal(renderer.freed, false);
  assert.equal(h.gpu.calls.filter(call => call === 'adapter').length, 1, 'Known failed GPU path must not replace a working Auto fallback');
  assert.match(h.bridge.diagnostics.rendererFallbackReason, /GPU render queue rejected/);
  h.assertContinuity(before);
});

test('Successful GPU submission completion allows Auto to create WebGPU', async t => {
  const completion = deferred();
  const h = driverHarness({ gpu: mockGpu(() => completion.promise) }); t.after(h.close);
  await microtasksUntil(() => h.gpu.calls.includes('work-done'), 'The probe must submit GPU work');
  assert.deepEqual(h.rendererCalls, [], 'Adapter/device creation and submission are insufficient before GPU completion');
  assert.equal(h.bridge.diagnostics.ready, false);
  assert.equal(h.host.children.length, 0, 'The probe canvas remains offscreen');
  assert.equal(h.canvases[0].width, 1);
  assert.equal(h.canvases[0].height, 1);
  completion.resolve();
  await h.started;
  assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgpu']);
  assert.equal(h.bridge.diagnostics.backend, 'WebGPU');
  assert.equal(h.bridge.diagnostics.ready, true);
  assert.equal(h.bridge.diagnostics.rendererFallbackReason, '');
  assert.equal(h.navigator.gpu, h.gpu.gpu);
  assert.notEqual(h.rendererCalls[0].canvas, h.canvases[0], 'Rendering uses a fresh canvas after the probe');
});

test('Auto falls back when the submitted GPU work reaches its deterministic deadline', async t => {
  const h = driverHarness({ gpu: mockGpu(() => new Promise(() => {})) }); t.after(h.close);
  await microtasksUntil(() => h.gpu.calls.includes('work-done'), 'The probe must await GPU completion');
  const deadline = [...h.timers.values()].find(timer => timer.ms === 5000);
  assert.ok(deadline, 'A submitted-work timeout must be armed');
  assert.deepEqual(h.rendererCalls, []);
  deadline.callback();
  await h.started;
  assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgl2']);
  assert.equal(h.bridge.diagnostics.ready, true);
  assert.match(h.bridge.diagnostics.rendererFallbackReason, /WebGPU render submission timed out/);
  assert.deepEqual(h.gpu.calls.slice(-2), ['unconfigure', 'destroy']);
  assert.equal(h.timers.size, 0, 'The completed fallback must clear its deadline');
});

test('A later successful probe can recover failed explicit WebGPU through Auto', async t => {
  let shouldReject = true;
  const gpu = mockGpu(() => shouldReject ? Promise.reject(new Error('temporary GPU queue failure')) : Promise.resolve());
  const h = driverHarness({ requested: 'webgpu', gpu }); t.after(h.close);
  await h.started;
  assert.equal(h.bridge.diagnostics.halted, true);
  await h.advance(4);
  const before = h.continuity();
  shouldReject = false;
  await h.select('auto', 'WebGPU');
  h.assertContinuity(before);
  assert.deepEqual(h.rendererCalls.map(call => call.backend), ['webgpu']);
  assert.equal(h.gpu.calls.filter(call => call === 'adapter').length, 2);
  assert.equal(h.bridge.diagnostics.rendererFallbackReason, '');
  assert.equal(h.bridge.diagnostics.error, '');
  assert.equal(h.navigator.gpu, h.gpu.gpu);
});

test('Retired-canvas context loss cannot overwrite a WebGPU probe error or a replacement renderer', async t => {
  const h = driverHarness({ requested: 'webgl2', gpu: rejectingGpu() }); t.after(h.close);
  await h.started;
  await h.advance(2);
  const before = h.continuity(), retired = h.rendererCalls[0];
  h.bridge.action({ type: 'renderer', value: 'webgpu' });
  assert.equal(retired.disposed, true);
  assert.equal(retired.freed, true);
  assert.equal(retired.disposeWasMounted, false, 'The old canvas must detach before its device is destroyed');
  assert.equal(h.host.children.length, 0);
  await microtasksUntil(() => /GPU render queue rejected/.test(h.bridge.diagnostics.error),
    'Explicit WebGPU must retain its probe failure');
  const probeError = h.bridge.diagnostics.error;
  let prevented = 0;
  const loseContext = canvas => h.host.dispatchEvent({
    type: 'webglcontextlost', target: canvas, preventDefault() { prevented++; },
  });
  loseContext(retired.canvas);
  assert.equal(h.bridge.diagnostics.error, probeError, 'Delayed loss from the retired device must preserve the useful error');
  await h.select('webgl2', 'WebGL2');
  loseContext(retired.canvas);
  assert.equal(h.bridge.diagnostics.ready, true, 'A retired device must not fault its replacement');
  assert.equal(h.bridge.diagnostics.error, '');
  h.assertContinuity(before);
  loseContext(h.rendererCalls[1].canvas);
  assert.equal(prevented, 3);
  assert.equal(h.bridge.diagnostics.halted, true, 'The active canvas still reports genuine context loss');
  assert.match(h.bridge.diagnostics.error, /Graphics connection lost/);
});

test('Controller status labels distinguish measured joint balance from source-only reference PD', async t => {
  for (const [controller, label] of [
    ['measured-contact-standing-balance', 'Joint balance'],
    ['source-standing-pd-zero-residual', 'Reference PD'],
  ]) {
    const h = driverHarness({ requested: 'webgl2', diagnostics: { getup: { controller, learned_policy_loaded: false } } });
    t.after(h.close); await h.started;
    assert.equal(h.bridge.diagnostics.recoveryStep, `GetUp body · ${label} · Learned recovery disabled`);
  }
});
