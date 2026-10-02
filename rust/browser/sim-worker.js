// This module owns the only simulation instance in the browser.
import init, { WorkerRuntime, wire_schema, calibrate_tangent_contact, calibrate_foundation } from './lh_worker.js';

const wasm = await init();
const poseBytes = 25 * 13 * Float32Array.BYTES_PER_ELEMENT;
const buffers = Array.from({ length: 3 }, (_, id) => ({ id, buffer: new ArrayBuffer(poseBytes), inFlight: false }));
let runtime = null;
let lastRequest = 0;
let profiling = true;
let quietQualification = false;

function publish(requestId, reply, started, timing) {
  const slot = buffers.find(slot => !slot.inFlight);
  if (!slot) throw new Error('No recycled pose buffer');
  const poses = new Float32Array(slot.buffer);
  let offset = 0;
  for (const pose of reply.snapshot.segments) {
    for (const key of ['position', 'rotation', 'linearVelocity', 'angularVelocity']) {
      const value = pose[key];
      poses[offset++] = value.x;
      poses[offset++] = value.y;
      poses[offset++] = value.z;
      if (key === 'rotation') poses[offset++] = value.w;
    }
  }
  slot.inFlight = true;
  self.postMessage({ type: 'snapshot', requestId, reply, bufferId: slot.id, poseBuffer: slot.buffer,
    workerUpdateMs: performance.now() - started, workerAtMs: performance.now(),
    workerTimeOriginMs: performance.timeOrigin,
    timing,
    wasmLinearBytes: wasm.memory?.buffer.byteLength ?? null,
    ...(quietQualification ? { quietProgress: JSON.parse(runtime.quiet_progress()) } : {}) }, [slot.buffer]);
}

self.onmessage = event => {
  const message = event.data;
  const started = performance.now();
  try {
    if (message.type === 'recycle') {
      const slot = buffers[message.bufferId];
      if (!slot?.inFlight || !(message.poseBuffer instanceof ArrayBuffer) || message.poseBuffer.byteLength !== poseBytes) {
        throw new Error('Invalid recycled pose buffer');
      }
      slot.buffer = message.poseBuffer;
      slot.inFlight = false;
      return;
    }
    if (!Number.isSafeInteger(message.requestId) || message.requestId <= lastRequest) throw new Error('Stale worker request');
    lastRequest = message.requestId;
    if (message.type === 'calibrateFoundation') {
      if (runtime) throw new Error('Calibration requires an uninitialized worker');
      if (!['primitive', 'canonical', 'joints', 'feet', 'chains'].includes(message.kind)
        || !message.profile || typeof message.profile !== 'object' || Array.isArray(message.profile)) throw new Error('Invalid foundation fixture');
      const profile = JSON.stringify(message.profile);
      if (profile.length > 65536) throw new Error('Foundation profile exceeds bound');
      self.postMessage({ type: 'foundation', requestId: message.requestId, report: JSON.parse(calibrate_foundation(message.kind, profile)) });
      return;
    }
    if (message.type === 'calibrateTangent') {
      if (runtime) throw new Error('Calibration requires an uninitialized worker');
      if (!['simplified', 'coulomb'].includes(message.model) || typeof message.heading !== 'number' || !Number.isFinite(message.heading)
        || typeof message.warmstart !== 'number' || !Number.isFinite(message.warmstart) || message.warmstart < 0 || message.warmstart > 1
        || !['static', 'sliding', 'changing', 'unloading', 'contact-loss', 'frictionless'].includes(message.scenario)
        || ![0.5, 4].includes(message.friction)
        || !['stress', 'bounded'].includes(message.excitation ?? 'stress')) throw new Error('Invalid tangent calibration fixture');
      self.postMessage({ type: 'calibration', requestId: message.requestId, report: JSON.parse(calibrate_tangent_contact(message.model, message.heading, message.warmstart, message.scenario, message.friction, message.excitation ?? 'stress')) });
      return;
    }
    if (message.type === 'shutdown') {
      runtime?.shutdown();
      runtime?.free();
      runtime = null;
      self.postMessage({ type: 'closed', requestId: message.requestId });
      self.close();
      return;
    }
    if (message.type === 'quietReport') {
      if (!runtime || !quietQualification) throw new Error('Quiet measurement is not enabled');
      self.postMessage({ type: 'qualification', requestId: message.requestId, report: JSON.parse(runtime.quiet_report()) });
      return;
    }
    if (!buffers.some(slot => !slot.inFlight)) throw new Error('No recycled pose buffer');
    if (message.type === 'initialize') {
      if (runtime) throw new Error('Already initialized; use a stamped Initialize command');
      if (message.generation !== undefined && (!Number.isSafeInteger(message.generation) || message.generation < 1)) throw new Error('Invalid initial generation');
      if (message.floorEnabled !== undefined && typeof message.floorEnabled !== 'boolean') throw new Error('Invalid initial floor');
      if (message.qualification !== undefined && !['quiet', 'quiet-drift'].includes(message.qualification)) throw new Error('Unknown qualification');
      if (message.profile !== undefined && (typeof message.profile !== 'object' || message.profile === null || Array.isArray(message.profile))) throw new Error('Invalid initial profile');
      const profile = message.profile === undefined ? undefined : JSON.stringify(message.profile);
      if (profile !== undefined && profile.length > 65536) throw new Error('Initial profile exceeds bound');
      const playground = message.playground === undefined ? undefined : JSON.stringify(message.playground);
      if (playground && playground.length > 256) throw new Error('Playground settings exceed bound');
      const next = new WorkerRuntime(message.mode, message.heading, String(message.generation ?? 1), message.floorEnabled, profile, playground);
      try {
        if (message.qualification !== undefined) next.enable_quiet_measurement();
        if (message.qualification === 'quiet-drift') next.enable_quiet_drift_trace();
      } catch (error) { next.free(); throw error; }
      runtime = next;
      profiling = message.profiling !== false;
      quietQualification = message.qualification !== undefined;
    } else {
      if (!runtime) throw new Error('Worker is not initialized');
      if (message.type === 'commands') {
        if (!Array.isArray(message.commands) || message.commands.length > 128) throw new Error('Command batch exceeds bound');
        runtime.enqueue_batch(JSON.stringify(message.commands));
      } else if (message.type === 'visible') {
        if (typeof message.visible !== 'boolean') throw new Error('Invalid visibility');
        runtime.set_visible(message.visible);
      } else if (message.type === 'advance') {
        if (!Number.isInteger(message.ticks) || message.ticks < 0 || message.ticks > 4) throw new Error('Advance exceeds bound');
      } else throw new Error('Unknown worker request');
    }
    const ticks = message.type === 'advance' ? message.ticks : 0;
    if (profiling) {
      const json = runtime.advance_profiled(ticks);
      const parseStarted = performance.now();
      const result = JSON.parse(json);
      const parseMs = performance.now() - parseStarted;
      publish(message.requestId, result.reply, started, { ticks: result.timings, clockValid: result.clockValid,
        runtimeMs: result.runtimeMs, runtimeNonTickMs: result.runtimeNonTickMs,
        nativeOutputMs: runtime.native_output_ms(), jsonParseMs: parseMs });
    } else publish(message.requestId, JSON.parse(runtime.advance(ticks)), started, null);
  } catch (error) {
    self.postMessage({ type: 'error', requestId: message?.requestId ?? null, message: String(error) });
  }
};
self.postMessage({ type: 'ready', schema: wire_schema(), maximumBuffers: 3, maximumAdvanceTicks: 4 });
