// Loopback-only development adapter. No WASM physics instance or prerecorded poses.
if (self.location.hostname !== '127.0.0.1') throw new Error('Source lab requires loopback');
const buffers = Array.from({ length: 3 }, (_, id) => ({ id, buffer: new ArrayBuffer(1300), inFlight: false }));
let busy = false;
self.onmessage = async ({ data: message }) => {
  if (message.type === 'recycle') {
    const slot = buffers[message.bufferId];
    if (!slot?.inFlight || !(message.poseBuffer instanceof ArrayBuffer) || message.poseBuffer.byteLength !== 1300) throw new Error('Invalid recycled source pose buffer');
    slot.buffer = message.poseBuffer; slot.inFlight = false; return;
  }
  if (busy) throw new Error('One source request at a time');
  busy = true;
  const started = performance.now();
  try {
    const response = await fetch('/__source_step/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(message), cache: 'no-store', credentials: 'omit' });
    const reply = await response.json();
    if (!response.ok) throw new Error(reply.error || 'Native source request failed');
    if (message.type === 'shutdown') { self.postMessage({ type: 'closed', requestId: message.requestId }); self.close(); return; }
    const slot = buffers.find(value => !value.inFlight);
    if (!slot) throw new Error('No recycled source pose buffer');
    const poses = new Float32Array(slot.buffer); let index = 0;
    for (const pose of reply.snapshot.segments) {
      for (const key of ['position', 'rotation', 'linearVelocity', 'angularVelocity']) {
        const value = pose[key]; poses[index++] = value.x; poses[index++] = value.y; poses[index++] = value.z;
        if (key === 'rotation') poses[index++] = value.w;
      }
    }
    if (index !== 325) throw new Error('Source/body renderer identity mismatch');
    slot.inFlight = true;
    self.postMessage({ type: 'snapshot', requestId: message.requestId, reply, bufferId: slot.id, poseBuffer: slot.buffer, workerUpdateMs: performance.now() - started, workerAtMs: performance.now(), workerTimeOriginMs: performance.timeOrigin, timing: null, wasmLinearBytes: null, transferBuffersAllocated: 3 }, [slot.buffer]);
  } catch (error) { self.postMessage({ type: 'error', requestId: message.requestId, message: String(error) }); }
  finally { busy = false; }
};
self.postMessage({ type: 'ready', schema: 4, maximumBuffers: 3, maximumAdvanceTicks: 4 });
