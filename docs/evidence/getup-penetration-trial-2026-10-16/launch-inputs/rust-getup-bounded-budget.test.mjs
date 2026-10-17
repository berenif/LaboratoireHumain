import assert from 'node:assert/strict';
import test from 'node:test';
import { checkCounters } from '../scripts/validate-getup-bounded.mjs';

const snapshot = (real, forecast) => ({ stamp: { sequence: real + 1 },
  simulation_time_s: real / 800, diagnostics: { getup: { policy: { forecast_substeps: forecast } } } });

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
  const wrongClock = snapshot(20, 0); wrongClock.simulation_time_s = 0.1;
  assert.throws(() => checkCounters(wrongClock));
});
