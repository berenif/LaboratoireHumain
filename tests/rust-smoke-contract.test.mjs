import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import { assertSmokeAdvanceFits, assertSmokeModelContract, maximumSmokeTicks, readSmokeClock, smokeClockContract } from '../scripts/rust-smoke-contract.mjs';

const model = JSON.parse(readFileSync(new URL('../rust/crates/model/data/getup-v1.json', import.meta.url), 'utf8'));
const diagnostics = () => ({
  graphics: { renderedSegments: model.segments.filter(segment => segment.geometry.indices.length > 0).length },
  snapshot: { segments: model.segments.map(segment => ({ id: segment.id })) },
});

test('GetUp renders all 16 source bodies and retains nine meshless coordinate frames in its 25 pose slots', () => {
  const bodies = model.segments.filter(segment => segment.reference.sourceBody !== null);
  const frames = model.segments.filter(segment => segment.reference.sourceBody === null);
  const rendered = model.segments.filter(segment => segment.geometry.indices.length > 0);
  assert.equal(bodies.length, 16);
  assert.equal(frames.length, 9);
  assert.equal(model.segments.length, 25);
  assert.equal(frames.length, model.referenceMetadata.coordinateFrames);
  assert.deepEqual(bodies.map(segment => segment.reference.sourceBody), model.referenceMetadata.sourceBodyOrder);
  assert.deepEqual(rendered.map(segment => segment.id), bodies.map(segment => segment.id));
  assert.ok(bodies.every(segment => segment.massKg > 0));
  assert.ok(frames.every(segment => segment.massKg === 0 && segment.geometry.vertices.length === 0));
  assertSmokeModelContract(diagnostics());
});

test('smoke rejects missing meshes and the obsolete expectation that all pose slots render', () => {
  for (const renderedSegments of [0, 15, 17, 25]) {
    const value = diagnostics();
    value.graphics.renderedSegments = renderedSegments;
    assert.throws(() => assertSmokeModelContract(value), /Rendered canonical meshes/);
  }
});

test('smoke rejects dropped source-body or coordinate-frame poses', () => {
  for (const id of ['left_hand', '@abdomen_z']) {
    const value = diagnostics();
    value.snapshot.segments = value.snapshot.segments.filter(pose => pose.id !== id);
    assert.throws(() => assertSmokeModelContract(value), /Canonical pose slots/);
  }
});

test('smoke rejects reordered, duplicated or substituted poses even with all 25 slots', () => {
  for (const change of [
    poses => { [poses[0], poses[1]] = [poses[1], poses[0]]; },
    poses => { poses[1] = { ...poses[0] }; },
    poses => { poses[1].id = 'obsolete-body'; },
  ]) {
    const value = diagnostics();
    change(value.snapshot.segments);
    assert.throws(() => assertSmokeModelContract(value), /Canonical pose identities and order/);
  }
});

const timedDiagnostics = tick => ({
  tick, time: tick / 40,
  snapshot: {
    stamp: { tick, sequence: 999 },
    simulation_time_s: tick / 40,
    diagnostics: { getup: { physics_step_s: Math.fround(1 / 800) } },
  },
});

test('smoke uses the exported 40 Hz, 20-substep clock with a five-second maximum', () => {
  assert.deepEqual(smokeClockContract, { tickHz: 40, substepsPerTick: 20, maximumSeconds: 5 });
  assert.equal(maximumSmokeTicks, 200);
  for (const tick of [0, 1, 120, 196, 200]) {
    assert.deepEqual(readSmokeClock(timedDiagnostics(tick)), { tick, time: tick / 40, nativeSubsteps: tick * 20 });
  }
});

test('smoke admits the last complete advance and rejects any request that would cross five seconds', () => {
  assertSmokeAdvanceFits(timedDiagnostics(196), 4);
  assertSmokeAdvanceFits(timedDiagnostics(199), 1);
  for (const tick of [197, 198, 199, 200]) {
    assert.throws(() => assertSmokeAdvanceFits(timedDiagnostics(tick), 4), /Advance would exceed five-second smoke budget/);
  }
  assert.throws(() => readSmokeClock(timedDiagnostics(201)), /Five-second smoke budget/);
  for (const ticks of [0, -1, 1.5, 5, NaN]) {
    assert.throws(() => assertSmokeAdvanceFits(timedDiagnostics(0), ticks), /Bounded smoke advance/);
  }
});

test('smoke rejects nonfinite, partial and inconsistent native clocks without using transport sequence', () => {
  for (const time of [NaN, Infinity, -1, 1 / 1600]) {
    const value = timedDiagnostics(0);
    value.time = value.snapshot.simulation_time_s = time;
    assert.throws(() => readSmokeClock(value), /Integral native substep clock/);
  }
  const partial = timedDiagnostics(1);
  partial.time = partial.snapshot.simulation_time_s = 19 / 800;
  assert.throws(() => readSmokeClock(partial), /Completed tick\/substep clock/);
  const wrongStep = timedDiagnostics(1);
  wrongStep.snapshot.diagnostics.getup.physics_step_s = 1 / 240;
  assert.throws(() => readSmokeClock(wrongStep), /Native physics step/);
  const wrongDisplay = timedDiagnostics(1);
  wrongDisplay.time = 0;
  assert.throws(() => readSmokeClock(wrongDisplay), /Displayed simulation time/);
});
