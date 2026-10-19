import assert from 'node:assert/strict';
import test from 'node:test';
import { verify_webgpu_render_queue } from '../rust/crates/browser/driver.js';

function probe(workDone = () => Promise.resolve()) {
  const calls = [];
  const device = {
    queue: { submit(commands) { calls.push(['submit', commands]); }, onSubmittedWorkDone: workDone },
    createCommandEncoder() {
      return {
        beginRenderPass(descriptor) { calls.push(['render', descriptor]); return { end() { calls.push(['end']); } }; },
        finish() { return 'probe-command'; },
      };
    },
    destroy() { calls.push(['destroy']); },
  };
  const context = {
    configure(config) { calls.push(['configure', config]); },
    getCurrentTexture() { return { createView() { return 'probe-view'; } }; },
    unconfigure() { calls.push(['unconfigure']); },
  };
  const canvas = { getContext(kind) { assert.equal(kind, 'webgpu'); return context; } };
  const gpu = {
    async requestAdapter(options) { calls.push(['adapter', options]); return { async requestDevice() { return device; } }; },
    getPreferredCanvasFormat() { return 'bgra8unorm'; },
  };
  return { gpu, canvas, context, device, calls, createCanvas: () => canvas };
}

test('WebGPU readiness requires a real one-pixel render submission and completed GPU work', async () => {
  const p = probe();
  await verify_webgpu_render_queue(p.gpu, p.createCanvas);
  assert.equal(p.canvas.width, 1); assert.equal(p.canvas.height, 1);
  assert.deepEqual(p.calls.find(x => x[0] === 'adapter')[1], { powerPreference: 'high-performance' });
  assert.equal(p.calls.find(x => x[0] === 'configure')[1].format, 'bgra8unorm');
  const attachment = p.calls.find(x => x[0] === 'render')[1].colorAttachments[0];
  assert.equal(attachment.loadOp, 'clear'); assert.equal(attachment.storeOp, 'store');
  assert.deepEqual(p.calls.find(x => x[0] === 'submit')[1], ['probe-command']);
  assert.deepEqual(p.calls.slice(-2).map(x => x[0]), ['unconfigure', 'destroy']);
});

test('WebGPU render rejection propagates a useful error and releases the probe resources', async () => {
  const p = probe(() => Promise.reject(new Error('GPU process unavailable')));
  await assert.rejects(verify_webgpu_render_queue(p.gpu, p.createCanvas), /WebGPU render check failed:.*GPU process unavailable.*Auto or WebGL2/);
  assert.deepEqual(p.calls.slice(-2).map(x => x[0]), ['unconfigure', 'destroy']);
});

test('WebGPU render hangs are bounded and clean up the probe', async () => {
  const p = probe(() => new Promise(() => {}));
  await assert.rejects(verify_webgpu_render_queue(p.gpu, p.createCanvas, 5), /render submission timed out/);
  assert.deepEqual(p.calls.slice(-2).map(x => x[0]), ['unconfigure', 'destroy']);
});

test('Missing WebGPU adapters and canvas contexts fail before committing to a renderer', async () => {
  await assert.rejects(verify_webgpu_render_queue(null, () => {}), /WebGPU is unavailable/);
  await assert.rejects(verify_webgpu_render_queue({ requestAdapter: async () => null }, () => {}), /No WebGPU adapter/);
  const p = probe();
  await assert.rejects(verify_webgpu_render_queue(p.gpu, () => ({ getContext: () => null })), /canvas context is unavailable/);
  assert.equal(p.calls.at(-1)[0], 'destroy');
});
