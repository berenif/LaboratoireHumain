import assert from 'node:assert/strict';
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints } from './capture-physics-baseline.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const output = process.argv[2];
assert.ok(output, 'Supply a fresh output directory');
mkdirSync(output);
const contract = { purpose: 'Isolate solver substep count without changing controller, geometry, caps, timestep or acceptance limits.',
  room: process.env.SOLVER_PROBE_ROOM === '1',
  counts: JSON.parse(process.env.SOLVER_PROBE_COUNTS ?? '[20,8,4]'), headings: [0], frames: Number(process.env.SOLVER_PROBE_FRAMES ?? 360), settleFrames: 120, dt: 1 / 60,
  bounds: { linear: .1, angular: .5, footDrift: .01 }, source: fingerprints() };
writeFileSync(join(output, 'contract.json'), JSON.stringify(contract, null, 2));
const results = [];
try {
  for (const heading of contract.headings) for (const count of contract.counts) {
    const c = await createEmbodiedCharacter('canvas2d', { heading, room: contract.room });
    const result = { heading, count, peakLinear: 0, peakAngular: 0, footDrift: 0, firstFailure: null, samples: [] };
    let anchor;
    try {
      c.world.numSolverIterations = count;
      for (let tick = 1; tick <= contract.frames; tick++) {
        c.fixedUpdate(contract.dt, null);
        const snapshot = c.getSnapshot('canvas2d');
        const linear = Math.max(...snapshot.segments.map(s => Math.hypot(...Object.values(s.linearVelocity))));
        const angular = Math.max(...snapshot.segments.map(s => Math.hypot(...Object.values(s.angularVelocity))));
        if (tick === contract.settleFrames) anchor = snapshot;
        if (tick > contract.settleFrames) {
          result.peakLinear = Math.max(result.peakLinear, linear);
          result.peakAngular = Math.max(result.peakAngular, angular);
          result.footDrift = Math.max(result.footDrift, ...snapshot.segments.filter(s => /Foot|Forefoot/.test(s.id)).map(s => {
            const origin = anchor.segments.find(a => a.id === s.id).position;
            return Math.hypot(s.position.x - origin.x, s.position.z - origin.z);
          }));
          if (!result.firstFailure && (linear > .1 || angular > .5 || result.footDrift > .01 || snapshot.support.planted.length !== 2 || snapshot.state !== 'upright')) {
            result.firstFailure = { tick, linear, angular, state: snapshot.state, planted: snapshot.support.planted };
          }
        }
        if (tick % 30 === 0) result.samples.push({ tick, linear, angular, state: snapshot.state, pelvis: snapshot.rootPosition });
        assert.ok(snapshot.diagnostics.finite, 'Non-finite simulation');
        assert.deepEqual(snapshot.diagnostics.errors, []);
      }
    } catch (error) { result.error = String(error); }
    finally { c.dispose(); }
    results.push(result);
    writeFileSync(join(output, 'results.json'), JSON.stringify(results, null, 2));
    console.log(JSON.stringify({ ...result, samples: undefined }));
  }
  assert.deepEqual(fingerprints(), contract.source, 'Source changed during probe');
} finally { unregister(); }
