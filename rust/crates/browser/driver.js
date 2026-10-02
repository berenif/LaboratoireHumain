// Transport and browser events only. Rust owns anatomy, picking and rendering;
// the dedicated WASM worker owns all dynamic bodies and their physical clock.
let active = null;
const xyz = values => ({ x: values[0], y: values[1], z: values[2] });
const changesTrial = action => action === 'Reset' || (typeof action === 'object'
  && ['Initialize', 'SelectStation', 'SetDifficulty'].some(key => key in action));
export function stop_bridge() { active?.dispose(); active = null; }
export async function start_bridge() {
  for (let attempt = 0; attempt < 1000 && !window.lhBindings; attempt++) {
    await new Promise(resolve => setTimeout(resolve, 10));
  }
  if (!window.lhBindings) throw new Error('Rust browser bindings did not initialize');
  stop_bridge();
  const host = document.getElementById('viewport');
  const bindings = window.lhBindings;
  let worker = null;
  let initialization = { mode: 'protocol', heading: 0, generation: 1, floorEnabled: true };
  const state = {
    disposed: false, renderer: null, canvas: null, changing: false, graphicsFault: false, graphicsError: '',
    backendRequested: new URL(location.href).searchParams.get('renderer') || 'auto',
    quality: 'auto', selected: '', heading: 0, floorEnabled: true, desiredFloorEnabled: true, desiredPaused: false,
    playground: { station: 'flat', difficulty: 'challenging' },
    visible: !document.hidden, visibleSent: true, last: null, pending: [], acknowledgements: new Map(),
    requestId: 0, sequence: 0, outstanding: null, press: 0, pointer: null,
    accumulator: 0, lastClock: performance.now(), activeWall: 0, frameWindow: performance.now(),
    frameCount: 0, fps: 0, error: '', failed: false, workerFault: false, watchdog: null, raf: 0, workerInstances: 0, workerMs: 0,
    manual: false, manualRequests: [], rendererChanges: 0, listeners: [], lastPick: null,
    touches: new Map(), gesture: null,
    packets: [], frames: [], droppedPackets: 0, droppedFrames: 0, previousFrameMs: null,
  };
  function listen(target, type, callback, options) {
    target.addEventListener(type, callback, options);
    state.listeners.push(() => target.removeEventListener(type, callback, options));
  }
  function status() {
    const snapshot = state.last?.reply.snapshot;
    const graphics = state.renderer ? JSON.parse(state.renderer.diagnostics()) : {};
    return {
      ready: Boolean(state.last && state.renderer && !state.changing && !state.graphicsFault && !state.failed),
      canReset: Boolean(state.last || state.workerFault), heading: state.heading,
      halted: state.failed || state.graphicsFault,
      paused: state.desiredPaused, mode: state.last?.reply.mode || 'protocol',
      station: state.playground.station, difficulty: state.playground.difficulty,
      backend: graphics.backend || '', requestedRenderer: state.backendRequested, quality: state.quality, selected: state.selected,
      tick: snapshot?.stamp.tick || 0, generation: snapshot?.stamp.generation || 0,
      time: snapshot?.simulation_time_s || 0, fps: state.fps,
      realtimeRatio: state.activeWall > 0 ? (snapshot?.simulation_time_s || 0) / state.activeWall : 0,
      steps: snapshot?.counters.steps || 0, contacts: snapshot?.contacts.filter(c => c.normal_load_n >= 3).length || 0,
      strikes: snapshot?.counters.strikes || 0, strikerPhase: snapshot?.striker?.phase || '',
      canStrike: Boolean(snapshot?.striker?.available && state.renderer && !state.changing && !state.graphicsFault && !state.desiredPaused && state.visible && !state.failed && !state.workerFault && !state.outstanding?.strike && !state.pending.some(entry => entry.action === 'Strike')),
      motion: snapshot?.motion || 'Starting', falls: snapshot?.counters.falls || 0,
      floorEnabled: state.floorEnabled, desiredFloorEnabled: state.desiredFloorEnabled, error: state.error || state.graphicsError, workerMs: state.workerMs,
    };
  }
  function notify() { window.dispatchEvent(new CustomEvent('lh-status', { detail: JSON.stringify(status()) })); }
  function retain(collection, value, dropped) {
    if (collection.length === 256) { collection.shift(); state[dropped]++; }
    collection.push(value);
  }
  function fail(error) { state.error = String(error); notify(); }
  function failGraphics(error) { state.graphicsFault = true; state.graphicsError = String(error); notify(); }
  function clearWatchdog() { clearTimeout(state.watchdog); state.watchdog = null; }
  function workerFailed(error) {
    if (state.disposed || state.workerFault) return;
    clearWatchdog();
    const failure = new Error(String(error));
    state.outstanding?.manualResolve?.reject(failure); state.outstanding = null;
    state.manualRequests.splice(0).forEach(request => request.reject(failure));
    state.pending = []; state.acknowledgements.clear(); cancelPointer(false);
    state.failed = true; state.workerFault = true; state.desiredPaused = true; state.accumulator = 0;
    worker?.terminate(); worker = null;
    fail('Physical worker stopped: ' + failure.message + '. Reset to start a fresh trial.');
  }
  function spawnWorker() {
    const current = new Worker(new URL('sim-worker.js', document.baseURI), { type: 'module', name: 'physical-body' });
    worker = current; state.workerInstances++;
    current.onmessage = event => { if (worker === current) onWorkerMessage(event); };
    current.onerror = event => {
      event.preventDefault();
      if (worker === current) workerFailed(event.message || 'Worker execution failed');
    };
    current.onmessageerror = () => { if (worker === current) workerFailed('Worker packet could not be decoded'); };
    state.watchdog = setTimeout(() => { if (worker === current) workerFailed('Worker startup timed out'); }, 30000);
  }
  function restartWorker() {
    initialization = { mode: state.last?.reply.mode || initialization.mode, heading: state.heading,
      generation: (state.last?.reply.snapshot.stamp.generation || initialization.generation) + 1,
      floorEnabled: state.floorEnabled, playground: state.playground };
    state.last = null; state.failed = false; state.workerFault = false; state.error = '';
    state.desiredPaused = false; state.visibleSent = true; state.accumulator = 0; state.activeWall = 0;
    state.lastClock = performance.now(); state.previousFrameMs = null;
    spawnWorker(); notify();
  }
  function send(message, manualResolve = null) {
    if (state.outstanding || state.disposed) throw new Error('Worker request already outstanding');
    message.requestId = ++state.requestId;
    state.outstanding = { id: message.requestId, type: message.type, sent: performance.now(), manualResolve,
      strike: message.commands?.some(command => command.action === 'Strike') || false,
      commandSequences: message.commands?.map(command => command.stamp.sequence) || [] };
    const current = worker;
    clearWatchdog();
    state.watchdog = setTimeout(() => {
      if (worker === current && state.outstanding?.id === message.requestId) workerFailed('Worker request timed out');
    }, message.type === 'initialize' ? 10000 : 5000);
    try { current.postMessage(message); } catch (error) { workerFailed(error); }
  }
  function cancelPointer(sendCancellation = true, clearTouches = true) {
    if (state.pointer?.kind === 'grab' && sendCancellation) queue('CancelGrab');
    state.pointer = null;
    if (clearTouches) { state.touches.clear(); state.gesture = null; }
    state.pending = state.pending.filter(entry => typeof entry.action !== 'object' || !('GrabMove' in entry.action || 'GrabBegin' in entry.action || 'GrabEnd' in entry.action));
  }
  function queue(action, effect = null) {
    if (state.disposed || state.workerFault || !state.last) return;
    if (changesTrial(action)) {
      cancelPointer(false); state.pending = []; state.manualRequests.splice(0).forEach(request => request.reject(new Error('Reset cancelled queued advance')));
    }
    const press = action?.GrabMove?.press;
    if (press !== undefined && state.pending.at(-1)?.action?.GrabMove?.press === press) {
      state.pending[state.pending.length - 1] = { action, effect }; return;
    }
    if (state.pending.length >= 128) { fail('Input queue is full'); return; }
    state.pending.push({ action, effect });
    pump();
  }
  function pump() {
    if (state.disposed || state.workerFault || state.outstanding || !state.last) return;
    if (state.visibleSent !== state.visible) {
      state.visibleSent = state.visible; send({ type: 'visible', visible: state.visible }); return;
    }
    if (state.pending.length) {
      const commands = [];
      while (state.pending.length && commands.length < 32) {
        const entry = state.pending.shift();
        const reset = changesTrial(entry.action);
        if (reset && commands.length) { state.pending.unshift(entry); break; }
        const sequence = ++state.sequence;
        commands.push({ stamp: { schema: 4, generation: state.last.reply.snapshot.stamp.generation, sequence, tick: state.last.reply.snapshot.stamp.tick }, action: entry.action });
        if (entry.effect) state.acknowledgements.set(sequence, entry.effect);
        if (reset) break;
      }
      send({ type: 'commands', commands }); return;
    }
    if (state.manualRequests.length) {
      const request = state.manualRequests.shift();
      send({ type: 'advance', ticks: request.ticks }, request); return;
    }
    if (state.manual || !state.visible || state.desiredPaused || state.failed || !state.renderer || state.changing || state.graphicsFault) return;
    // Publish each automatic tick so input and presentation can run between
    // catch-up ticks. The reply pump drains the remaining accumulated time.
    const ticks = Math.min(1, Math.floor(state.accumulator * 60 + 1e-8));
    if (ticks) { state.accumulator -= ticks / 60; send({ type: 'advance', ticks }); }
  }
  function acceptPresentation() {
    if (state.renderer && state.last) {
      state.renderer.accept_snapshot(JSON.stringify(state.last.reply.snapshot), state.last.poses,
        state.last.reply.paused || !state.visible || state.failed, performance.now());
    }
  }
  function onWorkerMessage(event) {
    const message = event.data;
    const pendingRequest = state.outstanding;
    if (message.type === 'closed') { clearWatchdog(); worker.terminate(); return; }
    if (state.disposed) return;
    try {
      if (message.type === 'ready') {
        clearWatchdog();
        if (message.schema !== 4 || message.maximumBuffers !== 3 || message.maximumAdvanceTicks !== 4) throw new Error('Worker contract mismatch');
        send({ type: 'initialize', ...initialization }); return;
      }
      if (message.requestId !== state.outstanding?.id) throw new Error('Unexpected worker reply');
      clearWatchdog();
      const outstanding = state.outstanding;
      state.outstanding = null;
      if (message.type === 'error') {
        outstanding.commandSequences.forEach(sequence => state.acknowledgements.delete(sequence));
        outstanding.manualResolve?.reject(new Error(message.message));
        if (outstanding.type === 'initialize') workerFailed(message.message);
        else { fail(message.message); pump(); }
        return;
      }
      if (message.type !== 'snapshot' || !(message.poseBuffer instanceof ArrayBuffer) || message.poseBuffer.byteLength !== 1300) throw new Error('Invalid worker packet');
      const oldGeneration = state.last?.reply.snapshot.stamp.generation;
      const receivedAtMs = performance.now();
      state.last = { reply: message.reply, poses: new Float32Array(message.poseBuffer).slice(), timing: message.timing };
      retain(state.packets, { requestId: message.requestId, type: outstanding.type, receivedAtMs,
        generation: message.reply.snapshot.stamp.generation, tick: message.reply.snapshot.stamp.tick,
        simulationTimeS: message.reply.snapshot.simulation_time_s, completedTicks: message.reply.completed_ticks,
        roundTripMs: receivedAtMs - outstanding.sent, workerUpdateMs: message.workerUpdateMs,
        workerTimeOriginMs: message.workerTimeOriginMs, workerAtMs: message.workerAtMs,
        wasmLinearBytes: message.wasmLinearBytes, timing: message.timing }, 'droppedPackets');
      state.workerMs = message.workerUpdateMs;
      if (message.reply.snapshot.environment) state.playground = message.reply.snapshot.environment.settings;
      worker.postMessage({ type: 'recycle', bufferId: message.bufferId, poseBuffer: message.poseBuffer }, [message.poseBuffer]);
      for (const acknowledgement of message.reply.acknowledgements) {
        const effect = state.acknowledgements.get(acknowledgement.stamp.sequence);
        state.acknowledgements.delete(acknowledgement.stamp.sequence);
        if (acknowledgement.rejected) fail('Command rejected: ' + acknowledgement.rejected);
        else effect?.();
      }
      if (oldGeneration !== undefined && oldGeneration !== message.reply.snapshot.stamp.generation) {
        state.accumulator = 0; state.activeWall = 0; state.lastClock = performance.now(); state.failed = false; state.error = '';
        state.desiredPaused = message.reply.paused;
        state.desiredFloorEnabled = state.floorEnabled;
      }
      state.failed = Boolean(message.reply.failure);
      if (['Falling', 'Fallen', 'Recovering'].includes(message.reply.snapshot.motion)) cancelPointer(false);
      if (state.failed) { state.desiredPaused = true; state.error = 'Physics halted: ' + message.reply.failure.check + '. Reset to retry.'; cancelPointer(false); }
      acceptPresentation();
      outstanding.manualResolve?.resolve(structuredClone(message.reply));
      notify(); pump();
    } catch (error) { pendingRequest?.manualResolve?.reject(error); workerFailed(error); }
  }
  function freshCanvas() {
    const canvas = document.createElement('canvas');
    canvas.id = 'simulation'; canvas.tabIndex = 0; canvas.setAttribute('aria-label', 'Interactive physical body');
    host.replaceChildren(canvas); state.canvas = canvas; return canvas;
  }
  async function changeRenderer(requested) {
    if (state.changing || state.disposed) return;
    state.changing = true; cancelPointer(); state.backendRequested = requested; notify();
    state.renderer?.dispose(); state.renderer?.free(); state.renderer = null;
    try {
      const first = requested === 'auto' ? (navigator.gpu ? 'webgpu' : 'webgl2') : requested;
      try { state.renderer = await bindings.make_renderer(freshCanvas(), first); }
      catch (error) {
        if (requested !== 'auto' || first !== 'webgpu') throw error;
        state.renderer = await bindings.make_renderer(freshCanvas(), 'webgl2');
      }
      if (state.disposed) { state.renderer.dispose(); state.renderer.free(); state.renderer = null; return; }
      state.graphicsFault = false; state.graphicsError = ''; state.rendererChanges++;
      state.renderer.select_region(state.selected); state.renderer.set_floor_enabled(state.floorEnabled); acceptPresentation();
    } catch (error) { failGraphics('Graphics unavailable: ' + error); }
    finally { state.changing = false; state.accumulator = 0; state.lastClock = performance.now(); notify(); }
  }
  function action(value) {
    if (['pause', 'reset', 'floor', 'mode', 'heading', 'station', 'difficulty', 'strike'].includes(value.type)) {
      if (value.type === 'reset' && state.workerFault) { restartWorker(); return; }
      if (!state.last || state.workerFault || (state.failed && value.type !== 'reset')) return;
    }
    switch (value.type) {
      case 'pause': cancelPointer(); state.desiredPaused = !state.desiredPaused; state.accumulator = 0; queue(state.desiredPaused ? 'Pause' : 'Resume'); break;
      case 'reset': queue('Reset'); break;
      case 'floor': state.desiredFloorEnabled = value.value; queue({ SetFloorEnabled: { enabled: value.value } }, () => { state.floorEnabled = value.value; state.renderer?.set_floor_enabled(value.value); }); break;
      case 'mode': queue({ Initialize: { mode: value.value, heading: state.heading } }); break;
      case 'heading': state.heading = Number(value.value); queue({ Initialize: { mode: state.last?.reply.mode || 'protocol', heading: state.heading } }); break;
      case 'station': queue({ SelectStation: { station: value.value } }); break;
      case 'difficulty': queue({ SetDifficulty: { difficulty: value.value } }); break;
      case 'strike': if (status().canStrike) queue('Strike'); break;
      case 'region': state.selected = value.value; state.renderer?.select_region(value.value); break;
      case 'renderer': void changeRenderer(value.value); break;
      case 'quality': state.quality = value.value; break;
      case 'camera': cancelPointer(); state.renderer?.reset_camera(); break;
      case 'focus': cancelPointer(); state.renderer?.focus_body(); break;
      case 'retry': void changeRenderer(state.backendRequested); break;
      default: throw new Error('Unknown UI action');
    }
    notify();
  }
  listen(window, 'lh-action', event => { try { action(JSON.parse(event.detail)); } catch (error) { fail(error); } });
  listen(window, 'lh-graphics-lost', () => { if (!state.changing && !state.disposed) { cancelPointer(); failGraphics('Graphics connection lost. Retry to rebuild the view.'); } });
  listen(document, 'visibilitychange', () => {
    state.visible = !document.hidden; state.accumulator = 0; state.lastClock = performance.now(); cancelPointer(false); pump();
  });
  listen(window, 'blur', () => cancelPointer());
  listen(host, 'webglcontextlost', event => {
    event.preventDefault();
    if (!state.changing && !state.disposed) { cancelPointer(); failGraphics('Graphics connection lost. Retry to rebuild the view.'); }
  }, true);
  listen(host, 'contextmenu', event => event.preventDefault());
  function touchGeometry() {
    const [a, b] = [...state.touches.values()];
    return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, distance: Math.max(1, Math.hypot(a.x - b.x, a.y - b.y)) };
  }
  listen(host, 'pointerdown', event => {
    if (!state.renderer || state.changing || !state.last || event.target !== state.canvas) return;
    if (event.pointerType === 'touch') {
      event.preventDefault(); state.canvas.setPointerCapture(event.pointerId);
      state.touches.set(event.pointerId, { x: event.clientX, y: event.clientY });
      if (state.touches.size >= 2) {
        cancelPointer(true, false); state.gesture = touchGeometry(); return;
      }
    }
    if (state.pointer) return;
    event.preventDefault(); state.canvas.focus(); state.canvas.setPointerCapture(event.pointerId);
    const rect = state.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left, y = event.clientY - rect.top;
    const canGrab = ['Upright', 'Reacting'].includes(state.last?.reply.snapshot.motion);
    const hit = event.button === 0 && canGrab && !state.desiredPaused && !state.failed ? JSON.parse(state.renderer.pick(x, y, rect.width, rect.height)) : null;
    state.lastPick = hit;
    if (hit) {
      state.pointer = { kind: 'grab', id: event.pointerId, press: ++state.press, point: hit.worldPoint, normal: hit.planeNormal };
      queue({ GrabBegin: { press: state.press, segment: hit.segment, local_anchor: xyz(hit.localAnchor), target: xyz(hit.worldPoint) } });
    } else state.pointer = { kind: 'orbit', id: event.pointerId, x: event.clientX, y: event.clientY };
  });
  listen(host, 'pointermove', event => {
    if (state.touches.has(event.pointerId)) {
      state.touches.set(event.pointerId, { x: event.clientX, y: event.clientY });
      if (state.gesture && state.touches.size >= 2 && state.renderer) {
        const next = touchGeometry(), previous = state.gesture;
        state.renderer.zoom(previous.distance / next.distance);
        state.renderer.orbit(-(next.x - previous.x) * 0.008, (next.y - previous.y) * 0.005);
        state.gesture = next; return;
      }
    }
    const pointer = state.pointer;
    if (!pointer || pointer.id !== event.pointerId || !state.renderer) return;
    if (pointer.kind === 'orbit') {
      state.renderer.orbit(-(event.clientX - pointer.x) * 0.008, (event.clientY - pointer.y) * 0.005);
      pointer.x = event.clientX; pointer.y = event.clientY;
    } else {
      const rect = state.canvas.getBoundingClientRect();
      const target = state.renderer.drag_target(event.clientX - rect.left, event.clientY - rect.top, rect.width, rect.height, pointer.point, pointer.normal);
      queue({ GrabMove: { press: pointer.press, target: xyz(target) } });
    }
  });
  listen(host, 'pointerup', event => {
    if (state.touches.delete(event.pointerId) && state.gesture) {
      cancelPointer(true, false); state.gesture = null; return;
    }
    if (state.pointer?.id !== event.pointerId) return;
    if (state.pointer.kind === 'grab') queue({ GrabEnd: { press: state.pointer.press } });
    state.pointer = null;
  });
  listen(host, 'pointercancel', () => cancelPointer());
  listen(host, 'lostpointercapture', event => {
    state.touches.delete(event.pointerId);
    if (state.touches.size < 2) state.gesture = null;
    if (state.pointer?.id === event.pointerId) cancelPointer();
  });
  listen(host, 'wheel', event => { event.preventDefault(); cancelPointer(); state.renderer?.zoom(Math.exp(event.deltaY * 0.001)); }, { passive: false });
  listen(window, 'keydown', event => {
    if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement || event.target instanceof HTMLButtonElement || event.altKey || event.ctrlKey || event.metaKey || event.repeat) return;
    if (event.code === 'Space') { event.preventDefault(); action({ type: 'pause' }); }
    else if (event.code === 'KeyR') action({ type: 'reset' });
    else if (event.code === 'KeyG') action({ type: 'floor', value: !state.floorEnabled });
    else if (event.code === 'KeyP') { event.preventDefault(); action({ type: 'strike' }); }
    else if (event.code === 'Escape') cancelPointer();
    else if (event.code === 'Home') action({ type: 'camera' });
    else if (event.code === 'KeyF') action({ type: 'focus' });
    else if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Equal', 'Minus'].includes(event.code)) {
      event.preventDefault(); cancelPointer();
      if (event.code === 'ArrowLeft') state.renderer?.orbit(-0.12, 0);
      else if (event.code === 'ArrowRight') state.renderer?.orbit(0.12, 0);
      else if (event.code === 'ArrowUp') state.renderer?.orbit(0, -0.08);
      else if (event.code === 'ArrowDown') state.renderer?.orbit(0, 0.08);
      else if (event.code === 'Equal') state.renderer?.zoom(0.9);
      else state.renderer?.zoom(1.1);
    }
  });
  function frame(now) {
    if (state.disposed) return;
    const frameStarted = performance.now();
    const elapsed = Math.max(0, (now - state.lastClock) / 1000); state.lastClock = now;
    if (state.last && state.renderer && !state.changing && !state.graphicsFault && !state.manual && !state.desiredPaused && state.visible && !state.failed) {
      state.accumulator += elapsed; state.activeWall += elapsed;
    }
    try {
      if (state.renderer && !state.changing && !state.graphicsFault && state.visible) {
        const rect = state.canvas.getBoundingClientRect();
        const ratio = state.quality === 'low' ? 1 : Math.min(devicePixelRatio, state.quality === 'high' ? 2 : 1.5);
        // Layout/visibility transitions can temporarily collapse the canvas.
        // Keep its last valid surface and retry at the next drawable frame.
        if (rect.width > 0 && rect.height > 0) {
          state.renderer.resize(rect.width, rect.height, ratio);
          if (state.renderer.draw(now)) {
            state.frameCount++;
            retain(state.frames, { atMs: frameStarted, cpuMs: performance.now() - frameStarted,
              intervalMs: state.previousFrameMs === null ? null : frameStarted - state.previousFrameMs,
              observationTick: state.last?.reply.snapshot.stamp.tick, generation: state.last?.reply.snapshot.stamp.generation,
              paused: state.desiredPaused, visible: state.visible, backlogSeconds: state.accumulator }, 'droppedFrames');
            state.previousFrameMs = frameStarted;
          }
        } else cancelPointer();
      }
    } catch (error) { cancelPointer(); failGraphics('Graphics frame failed: ' + error); }
    if (now - state.frameWindow >= 500) { state.fps = state.frameCount * 1000 / (now - state.frameWindow); state.frameCount = 0; state.frameWindow = now; notify(); }
    pump(); state.raf = requestAnimationFrame(frame);
  }
  function dispose() {
    if (state.disposed) return;
    state.disposed = true; clearWatchdog(); cancelAnimationFrame(state.raf); state.listeners.splice(0).forEach(remove => remove());
    state.pending = []; state.pointer = null; state.manualRequests.splice(0).forEach(request => request.reject(new Error('Disposed')));
    state.outstanding?.manualResolve?.reject(new Error('Disposed')); state.acknowledgements.clear();
    state.outstanding = null;
    const closing = worker;
    closing?.postMessage({ type: 'shutdown', requestId: ++state.requestId });
    setTimeout(() => closing?.terminate(), 2000);
    state.renderer?.dispose(); state.renderer?.free(); state.renderer = null; state.last = null;
  }
  active = { dispose }; listen(window, 'pagehide', dispose);
  window.__lhRust = {
    get diagnostics() { return { ...status(), snapshot: state.last?.reply.snapshot, graphics: state.renderer ? JSON.parse(state.renderer.diagnostics()) : null,
      disposed: state.disposed, confirmedPaused: state.last?.reply.paused, outstanding: Boolean(state.outstanding), queueDepth: state.pending.length, workerInstances: state.workerInstances,
      workerFault: state.workerFault, manualRequests: state.manualRequests.length,
      rendererChanges: state.rendererChanges, backlogSeconds: state.accumulator, lastPick: state.lastPick,
      activePointer: state.pointer?.kind || null, failure: state.last?.reply.failure }; },
    drainTelemetry() { return { packets: state.packets.splice(0), frames: state.frames.splice(0),
      droppedPackets: state.droppedPackets, droppedFrames: state.droppedFrames,
      mainWasmLinearBytes: bindings.linear_memory_bytes() }; },
    setManual(value) { state.manual = Boolean(value); state.accumulator = 0; state.lastClock = performance.now(); },
    advance(ticks) {
      if (!Number.isInteger(ticks) || ticks < 0 || ticks > 4 || !state.manual) return Promise.reject(new Error('Manual advance requires 0–4 ticks and manual mode'));
      if (state.disposed || state.workerFault || !state.last) return Promise.reject(new Error('Physical worker is unavailable'));
      if (state.manualRequests.length || state.outstanding?.manualResolve) return Promise.reject(new Error('Manual advance is already outstanding'));
      return new Promise((resolve, reject) => { state.manualRequests.push({ ticks, resolve, reject }); pump(); });
    },
    action, dispose,
    projectSegment(segment) { const rect = state.canvas.getBoundingClientRect(); const point = state.renderer.project_segment(segment, rect.width, rect.height); return { x: rect.left + point[0], y: rect.top + point[1] }; },
  };
  spawnWorker(); state.raf = requestAnimationFrame(frame); notify(); await changeRenderer(state.backendRequested);
}
