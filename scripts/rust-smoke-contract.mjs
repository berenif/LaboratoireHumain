import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const model = JSON.parse(readFileSync(new URL('../rust/crates/model/data/getup-v1.json', import.meta.url), 'utf8'));
const poseIds = model.segments.map(segment => segment.id);
// Coordinate frames occupy pose slots but have no mesh to render.
const renderedIds = model.segments.filter(segment => segment.geometry.indices.length > 0).map(segment => segment.id);
const contracts = readFileSync(new URL('../rust/crates/contracts/src/lib.rs', import.meta.url), 'utf8');
function rustU32(name) {
  const match = contracts.match(new RegExp(`^pub const ${name}: u32 = ([0-9]+);$`, 'm'));
  assert.ok(match, `Missing exported Rust ${name} contract`);
  return Number(match[1]);
}
export const smokeClockContract = Object.freeze({
  tickHz: rustU32('TICK_HZ'),
  substepsPerTick: rustU32('SUBSTEPS'),
  maximumSeconds: 5,
});
export const maximumSmokeTicks = smokeClockContract.maximumSeconds * smokeClockContract.tickHz;
const nativeHz = smokeClockContract.tickHz * smokeClockContract.substepsPerTick;

export function assertSmokeModelContract(diagnostics) {
  assert.equal(diagnostics.graphics.renderedSegments, renderedIds.length, 'Rendered canonical meshes');
  assert.equal(diagnostics.snapshot.segments.length, poseIds.length, 'Canonical pose slots');
  assert.deepEqual(diagnostics.snapshot.segments.map(pose => pose.id), poseIds, 'Canonical pose identities and order');
}

export function readSmokeClock(diagnostics) {
  const time = diagnostics.snapshot.simulation_time_s;
  // This clock comes from native substeps_completed, not transport sequence.
  const integrated = time * nativeHz;
  const nativeSubsteps = Math.round(integrated);
  assert.ok(Number.isSafeInteger(nativeSubsteps) && nativeSubsteps >= 0
    && Math.abs(integrated - nativeSubsteps) <= 1e-6, 'Integral native substep clock');
  assert.ok(Math.abs(diagnostics.snapshot.diagnostics.getup.physics_step_s - 1 / nativeHz) < 1e-9, 'Native physics step');
  assert.equal(time, nativeSubsteps / nativeHz, 'Native simulation time');
  assert.equal(diagnostics.time, time, 'Displayed simulation time');
  assert.ok(Number.isSafeInteger(diagnostics.tick) && diagnostics.tick >= 0, 'Integral smoke tick');
  assert.equal(nativeSubsteps, diagnostics.tick * smokeClockContract.substepsPerTick, 'Completed tick/substep clock');
  assert.ok(diagnostics.tick <= maximumSmokeTicks && time <= smokeClockContract.maximumSeconds, 'Five-second smoke budget');
  return { tick: diagnostics.tick, time, nativeSubsteps };
}

export function assertSmokeAdvanceFits(diagnostics, ticks) {
  const clock = readSmokeClock(diagnostics);
  assert.ok(Number.isInteger(ticks) && ticks > 0 && ticks <= 4, 'Bounded smoke advance');
  assert.ok(clock.tick + ticks <= maximumSmokeTicks, 'Advance would exceed five-second smoke budget');
}
