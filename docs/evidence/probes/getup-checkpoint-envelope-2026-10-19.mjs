// Fixed-int bincode tuple: (u32 schema,String model,Vec<u8> world,String auxiliary).
// Preserve raw bytes and runtime text. Never reserialize schema2 u64 sidecar
// fields through JavaScript Numbers; only activation's exact-zero state is read.
export function parseGetUpCheckpoint(bytes) {
  if (!(bytes instanceof Uint8Array) || bytes.length > 64 * 1024 * 1024) throw new Error('Bounded checkpoint bytes required');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let offset = 0;
  const take = length => {
    if (!Number.isSafeInteger(length) || length < 0 || length > bytes.length - offset) throw new Error('Truncated checkpoint field');
    const result = bytes.subarray(offset, offset + length); offset += length; return result;
  };
  const scalar = take(4); const schema = new DataView(scalar.buffer, scalar.byteOffset, 4).getUint32(0, true);
  const field = () => {
    if (offset + 8 > bytes.length) throw new Error('Truncated checkpoint length');
    const length = view.getBigUint64(offset, true); offset += 8;
    if (length > 64n * 1024n * 1024n) throw new Error('Oversized checkpoint field');
    return take(Number(length));
  };
  const decoder = new TextDecoder('utf-8', { fatal: true });
  const modelBytes = field(), worldBytes = field(), auxiliaryBytes = field();
  if (offset !== bytes.length) throw new Error('Trailing checkpoint bytes');
  const auxiliaryText = decoder.decode(auxiliaryBytes);
  if (schema === 1) return { schema, modelBytes, worldBytes, auxiliaryBytes, runtimeText: auxiliaryText, auxiliaryText, wrapper: null };
  if (schema !== 2) throw new Error('Unsupported checkpoint schema');
  const wrapper = JSON.parse(auxiliaryText);
  if (Object.keys(wrapper).sort().join(',') !== 'format,roots,runtime'
    || wrapper.format !== 'f32-dq-f64-remainder-v1' || typeof wrapper.runtime !== 'string'
    || !Array.isArray(wrapper.roots)) throw new Error('Invalid schema2 wrapper');
  return { schema, modelBytes, worldBytes, auxiliaryBytes, runtimeText: wrapper.runtime, auxiliaryText, wrapper };
}
export function proveActivationPurity(beforeBytes, afterBytes) {
  const before = parseGetUpCheckpoint(beforeBytes), after = parseGetUpCheckpoint(afterBytes);
  const equalBytes = (a,b) => a.length === b.length && a.every((value,i) => value === b[i]);
  if (before.schema !== 1 || after.schema !== 2 || !equalBytes(before.modelBytes, after.modelBytes)
    || !equalBytes(before.worldBytes, after.worldBytes) || before.runtimeText !== after.runtimeText) {
    throw new Error('Activation changed binary world or embedded original Runtime string');
  }
  const roots = after.wrapper.roots;
  if (roots.length !== 1 || Object.keys(roots[0]).sort().join(',') !== 'root,state' || !Array.isArray(roots[0].root) || roots[0].root.length !== 2
    || !roots[0].root.every(x => Number.isInteger(x) && x >= 0 && x <= 0xffffffff)
    || !roots[0].state || Object.keys(roots[0].state).sort().join(',') !== 'qbits,remainderbits'
    || !Array.isArray(roots[0].state.qbits) || roots[0].state.qbits.length !== 3
    || !roots[0].state.qbits.every(x => Number.isInteger(x) && x >= 0 && x <= 0xffffffff)
    || !Array.isArray(roots[0].state.remainderbits) || roots[0].state.remainderbits.length !== 3
    || !roots[0].state.remainderbits.every(x => x === 0)) throw new Error('Fresh exact-zero carry state required');
  const marker = ',"roots":';
  const rootsStart = after.auxiliaryText.lastIndexOf(marker);
  // Rust emits this final compact integer-only field; a text equality check
  // rejects fractional underflow/-0/exponents that JSON Number could coerce.
  if (rootsStart < 0 || after.auxiliaryText.slice(rootsStart + marker.length, -1) !== JSON.stringify(roots)) {
    throw new Error('Activation sidecar requires exact canonical integer tokens');
  }
  return { schemaBefore: 1, schemaAfter: 2, modelBytesEqual: true, worldBytesEqual: true,
    embeddedRuntimeStringExactlyEqual: true, root: roots[0].root, algorithm: after.wrapper.format,
    initialRemainderBits: [0,0,0], bytesBefore: beforeBytes.length, bytesAfter: afterBytes.length };
}
