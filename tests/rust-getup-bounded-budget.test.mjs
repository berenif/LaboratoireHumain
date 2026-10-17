import assert from 'node:assert/strict';
import test from 'node:test';
import { checkCounters, nextTickFits } from '../scripts/validate-getup-bounded.mjs';

const snapshot = (real, forecast) => ({ stamp: { sequence: real + 1 },
  simulation_time_s: real / 800, diagnostics: { getup: {
    physics_step_s: 0.00125, policy: { forecast_substeps: forecast } } } });

test('incremental trial counts actual native integrations including its entire forecast', () => {
  assert.deepEqual(checkCounters(snapshot(0, 0)), { real: 0, forecast: 0, total: 0 });
  assert.deepEqual(checkCounters(snapshot(3200, 5000)), { real: 3200, forecast: 5000, total: 8200 });
  assert.deepEqual(checkCounters(snapshot(2897, 617)), { real: 2897, forecast: 617, total: 3514 });
});

test('incremental trial refuses excess, missing, invalid or decreasing counters without physics', () => {
  for (const row of [snapshot(3201, 0), snapshot(0, 5001), snapshot(1.5, 0), snapshot(0, NaN)]) {
    assert.throws(() => checkCounters(row));
  }
  assert.throws(() => checkCounters(snapshot(20, 40), { real: 21, forecast: 40 }));
  assert.throws(() => checkCounters(snapshot(20, 40), { real: 20, forecast: 41 }));
  assert.throws(() => checkCounters({ stamp: { sequence: 1 }, simulation_time_s: 0, diagnostics: {} }));
  const wrongClock = snapshot(20, 0); wrongClock.diagnostics.getup.physics_step_s = 0.005;
  assert.throws(() => checkCounters(wrongClock));
});

test('transport observation sequence is independent of the physical native count', () => {
  // Retained failure packet: observation 2, physical tick 1, 20 integrations.
  const retained = snapshot(20, 0); retained.stamp = { generation: 1, sequence: 2, tick: 1 };
  assert.deepEqual(checkCounters(retained), { real: 20, forecast: 0, total: 20 });
  retained.stamp.sequence = 999;
  assert.deepEqual(checkCounters(retained), { real: 20, forecast: 0, total: 20 });
});

test('retry admits the complete possible forecast before a tick and preserves the cumulative cap', () => {
  assert.equal(nextTickFits({ real: 0, forecast: 0, total: 0 }, 8180), true);
  assert.equal(nextTickFits({ real: 3160, forecast: 0, total: 3160 }, 8180), true);
  assert.equal(nextTickFits({ real: 3180, forecast: 0, total: 3180 }, 8180), false);
  assert.equal(nextTickFits({ real: 3180, forecast: 4980, total: 8160 }, 8180), true);
  assert.equal(nextTickFits({ real: 3180, forecast: 5000, total: 8180 }, 8180), false);
  assert.equal(nextTickFits({ real: 3200, forecast: 20, total: 3220 }, 8180), false);
});
