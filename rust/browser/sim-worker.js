// This module owns the only simulation instance in the browser.
import init, { WorkerRuntime, wire_schema, calibrate_tangent_contact, calibrate_foundation, calibrate_recovery_fixture, getup_trial_profile, getup_build_capabilities } from './lh_worker.js';

const wasm = await init();
const poseBytes = 25 * 13 * Float32Array.BYTES_PER_ELEMENT;
const buffers = Array.from({ length: 3 }, (_, id) => ({ id, buffer: new ArrayBuffer(poseBytes), inFlight: false }));
let runtime = null;
let lastRequest = 0;
let profiling = true;
let quietQualification = false;
let recoveryQualification = false;
let recoveryDemo = false, recoveryTriggered = false, readyAfterTick = null, lastRecoveries = 0, lastSnapshot = null;
const recoveryProfileHash = 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6';
function completeRecoveryEntry(reply) {
  if (!recoveryDemo) return reply;
  if (!recoveryTriggered && !reply.failure && reply.snapshot.stamp.tick === 20) {
    if (!JSON.parse(runtime.getup_fixture_report()).state.ready) throw new Error('Initial prone body did not settle');
    runtime.trigger_getup_fixture(); // Declared initial lifecycle only; no body/clock edit.
    const triggered = JSON.parse(runtime.advance(0));
    reply = { ...reply, snapshot: triggered.snapshot, failure: triggered.failure };
    recoveryTriggered = true;
  }
  if (reply.snapshot.counters.recoveries > lastRecoveries) {
    lastRecoveries = reply.snapshot.counters.recoveries;
    readyAfterTick = reply.snapshot.stamp.tick + 120;
  }
  lastSnapshot = reply.snapshot;
  return reply;
}
function recoveryCanStrike() {
  const p = lastSnapshot?.diagnostics.getup?.policy;
  return recoveryTriggered && lastSnapshot?.motion === 'Upright' && p?.prototype_standing_passed
    && p.stable_standing_s >= 1 && readyAfterTick !== null && lastSnapshot.stamp.tick >= readyAfterTick
    && lastSnapshot.striker?.available;
}
const workerIdentity = globalThis.crypto?.randomUUID?.() ?? `${performance.timeOrigin}:${Math.random()}`;

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
    transferBuffersAllocated: buffers.length,
    ...(recoveryDemo ? { recoveryDemo: { readyAfterTick, initialTriggered: recoveryTriggered }, workerIdentity } : {}),
    ...(quietQualification ? { quietProgress: JSON.parse(runtime.quiet_progress()) } : {}),
    ...(recoveryQualification ? { recoveryProgress: JSON.parse(runtime.recovery_progress()), workerIdentity } : {}) }, [slot.buffer]);
}

self.onmessage = async event => {
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
      if (!['primitive', 'canonical', 'joints', 'feet', 'chains', 'stress', 'controls', 'actuators', 'disturbances', 'steps', 'legacy', 'recovery', 'cycles', 'terrain'].includes(message.kind)
        || !message.profile || typeof message.profile !== 'object' || Array.isArray(message.profile)) throw new Error('Invalid foundation fixture');
      const profile = JSON.stringify(message.profile);
      if (profile.length > 65536) throw new Error('Foundation profile exceeds bound');
      if (message.fixtureId !== undefined && (message.kind !== 'recovery' || typeof message.fixtureId !== 'string'
        || !/^[a-z0-9-]{1,128}$/.test(message.fixtureId))) throw new Error('Invalid selected recovery fixture');
      const report = message.fixtureId === undefined ? calibrate_foundation(message.kind, profile)
        : calibrate_recovery_fixture(message.fixtureId, profile);
      self.postMessage({ type: 'foundation', requestId: message.requestId, workerIdentity, report: JSON.parse(report) });
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
    if (message.type === 'getupReport') {
      if (!runtime) throw new Error('GetUp trial is not initialized');
      self.postMessage({ type: 'getupReport', requestId: message.requestId, report: JSON.parse(runtime.getup_report()) });
      return;
    }
    if (message.type === 'getupCheckpoint') {
      if (!runtime) throw new Error('GetUp trial is not initialized');
      const checkpoint = runtime.getup_checkpoint();
      const verification = JSON.parse(runtime.verify_getup_checkpoint(checkpoint));
      self.postMessage({ type: 'getupCheckpoint', requestId: message.requestId,
        checkpoint, verification }, [checkpoint.buffer]);
      return;
    }
    if (!buffers.some(slot => !slot.inFlight)) throw new Error('No recycled pose buffer');
    if (message.type === 'initialize') {
      if (runtime) throw new Error('Already initialized; use a stamped Initialize command');
      if (message.generation !== undefined && (!Number.isSafeInteger(message.generation) || message.generation < 1)) throw new Error('Invalid initial generation');
      if (message.floorEnabled !== undefined && typeof message.floorEnabled !== 'boolean') throw new Error('Invalid initial floor');
      if (message.qualification !== undefined && !['quiet', 'quiet-drift', 'recovery'].includes(message.qualification)) throw new Error('Unknown qualification');
      if (message.profile !== undefined && (typeof message.profile !== 'object' || message.profile === null || Array.isArray(message.profile))) throw new Error('Invalid initial profile');
      if (message.getupTrial !== undefined && (message.getupTrial !== true || message.mode !== 'protocol'
        || message.floorEnabled === false || message.profile !== undefined || message.qualification !== undefined)) throw new Error('Invalid GetUp prototype preload');
      if (message.recoveryDemo !== undefined && (message.recoveryDemo !== true || message.mode !== 'protocol'
        || message.heading !== 0 || (message.generation ?? 1) !== 1 || message.floorEnabled !== true
        || message.profile !== undefined || message.qualification !== undefined || message.getupTrial !== undefined)) throw new Error('Invalid explicit recovery demo');
      let demoProfile;
      if (message.recoveryDemo) {
        const [profileResponse, capabilityResponse] = await Promise.all([fetch('./profile.json'), fetch('./capabilities.json')]);
        if (!profileResponse.ok || !capabilityResponse.ok) throw new Error('Recovery assets unavailable');
        demoProfile = await profileResponse.text();
        const hash = [...new Uint8Array(await crypto.subtle.digest('SHA-256', new TextEncoder().encode(demoProfile)))].map(x=>x.toString(16).padStart(2,'0')).join('');
        const expected = await capabilityResponse.json(), actual = JSON.parse(getup_build_capabilities());
        if (hash !== recoveryProfileHash || JSON.stringify(actual.effectiveSimFeatures) !== JSON.stringify(expected.effective_sim_features)
          || !actual.effectiveSimFeatures.includes('getup-repeat-balance-acquire-experiment')) throw new Error('Unreviewed recovery profile or worker');
      }
      const profile = demoProfile ?? (message.getupTrial ? getup_trial_profile() : message.profile === undefined ? undefined : JSON.stringify(message.profile));
      if (profile !== undefined && profile.length > 65536) throw new Error('Initial profile exceeds bound');
      if (message.qualification === 'recovery' && (message.mode !== 'protocol' || message.floorEnabled === false
        || !message.profile || message.profile.recovery_enabled !== true)) throw new Error('Recovery qualification requires an explicit recovery profile and protocol floor');
      const playground = message.playground === undefined ? undefined : JSON.stringify(message.playground);
      if (playground && playground.length > 256) throw new Error('Playground settings exceed bound');
      const next = message.recoveryDemo ? WorkerRuntime.getup_fixture_with_profile('belly', profile)
        : new WorkerRuntime(message.mode, message.heading, String(message.generation ?? 1), message.floorEnabled, profile, playground);
      try {
        if (message.recoveryDemo) next.set_root_translation_compensation_enabled(true);
        if (['quiet', 'quiet-drift'].includes(message.qualification)) next.enable_quiet_measurement();
        if (message.qualification === 'quiet-drift') next.enable_quiet_drift_trace();
        if (message.qualification === 'recovery') next.enable_recovery_measurement();
      } catch (error) { next.free(); throw error; }
      runtime = next; recoveryDemo = message.recoveryDemo === true;
      profiling = message.profiling !== false;
      quietQualification = ['quiet', 'quiet-drift'].includes(message.qualification);
      recoveryQualification = message.qualification === 'recovery';
    } else {
      if (!runtime) throw new Error('Worker is not initialized');
      if (message.type === 'commands') {
        if (!Array.isArray(message.commands) || message.commands.length > 128) throw new Error('Command batch exceeds bound');
        if (recoveryDemo && message.commands.some(c => c.action === 'Reset' || (typeof c.action === 'object' && ['Initialize','SelectStation','SetDifficulty'].some(k=>k in c.action)))) throw new Error('Recovery reset/mode changes require an explicit new page trial');
        if (recoveryDemo && message.commands.some(c => ['Strike','StrikeFromBehind'].includes(c.action))
          && (!recoveryCanStrike() || message.commands.some(c=>c.action==='Strike'))) throw new Error('Recovery rear striker is not ready');
        runtime.enqueue_batch(JSON.stringify(message.commands));
      } else if (message.type === 'visible') {
        if (typeof message.visible !== 'boolean') throw new Error('Invalid visibility');
        runtime.set_visible(message.visible);
      } else if (message.type === 'advance') {
        if (!Number.isInteger(message.ticks) || message.ticks < 0 || message.ticks > 4) throw new Error('Advance exceeds bound');
      } else throw new Error('Unknown worker request');
    }
    let ticks = message.type === 'advance' ? message.ticks : 0;
    if (recoveryDemo && !recoveryTriggered) ticks = Math.min(ticks, 20 - (lastSnapshot?.stamp.tick ?? 0));
    if (profiling) {
      const json = runtime.advance_profiled(ticks);
      const parseStarted = performance.now();
      const result = JSON.parse(json);
      const parseMs = performance.now() - parseStarted;
      publish(message.requestId, completeRecoveryEntry(result.reply), started, { ticks: result.timings, clockValid: result.clockValid,
        runtimeMs: result.runtimeMs, runtimeNonTickMs: result.runtimeNonTickMs,
        nativeOutputMs: runtime.native_output_ms(), jsonParseMs: parseMs });
    } else publish(message.requestId, completeRecoveryEntry(JSON.parse(runtime.advance(ticks))), started, null);
  } catch (error) {
    self.postMessage({ type: 'error', requestId: message?.requestId ?? null, workerIdentity, message: String(error) });
  }
};
self.postMessage({ type: 'ready', schema: wire_schema(), maximumBuffers: 3, maximumAdvanceTicks: 4, workerIdentity });
