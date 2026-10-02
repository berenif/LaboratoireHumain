/** Lossless data encoding for controller Maps, Sets and non-finite sentinel
 * timers. Native resources are saved separately in the Rapier snapshot. */
export function encodeControllerData(value) {
  if (typeof value === 'number' && !Number.isFinite(value)) return { $number: String(value) };
  if (value instanceof Map) return { $map: [...value].map(([k,v]) => [k,encodeControllerData(v)]) };
  if (value instanceof Set) return { $set: [...value].map(encodeControllerData) };
  if (Array.isArray(value)) return value.map(encodeControllerData);
  if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value)
    .filter(([,v]) => typeof v !== 'function').map(([k,v]) => [k,encodeControllerData(v)]));
  return value;
}
export function decodeControllerData(value) {
  if (value?.$number) return Number(value.$number);
  if (value?.$map) return new Map(value.$map.map(([k,v]) => [k,decodeControllerData(v)]));
  if (value?.$set) return new Set(value.$set.map(decodeControllerData));
  if (Array.isArray(value)) return value.map(decodeControllerData);
  if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([k,v]) => [k,decodeControllerData(v)]));
  return value;
}
export function captureControllerState(character) {
  const resources = new Set(['world','nativeMotors','ragdollBodies','ragdollColliders','ragdollJoints',
    'jointsByChild','physicsHooks','eventQueue','floorCollider','playground','striker']);
  const state = Object.fromEntries(Object.entries(character).filter(([key]) => !resources.has(key)));
  return { schema: 1, state: encodeControllerData(state),
    nativeResources: [...resources], candidate: character.coordinatedStanding?.serialize(),
    qualification: 'All enumerable controller state, including pending stance, balance readiness, target dynamics, grab/recovery state and Maps. Native resources and methods reconstructed by original startup plus the paired native snapshot.' };
}
