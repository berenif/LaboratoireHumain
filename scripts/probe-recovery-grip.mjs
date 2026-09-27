import assert from 'node:assert/strict';
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints } from './capture-physics-baseline.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const output = process.argv[2];
assert.ok(output, 'Supply a fresh output directory');
mkdirSync(output);
const roles = ['hand', 'forearm', 'forearm-twist', 'shin', 'ankle'];
const contract = { frames: 600, dt: 1 / 60, roles, variants: [.45, 1.2], source: fingerprints(),
  purpose: 'Compare recovery-support friction after a real strike; unchanged soles, floor, actuator budgets and solver settings.' };
writeFileSync(join(output, 'contract.json'), JSON.stringify(contract, null, 2));
const results = [];
try {
  for (const friction of contract.variants) {
    const c = await createEmbodiedCharacter('canvas2d', { room: true });
    const result = { friction, floorTick: null, supportSlipDistance: 0, supportSamples: 0,
      torsoHorizontalTravel: 0, peakPelvisY: 0, peakLinear: 0, samples: [] };
    let previousTorso;
    try {
      for (const [id, collider] of c.ragdollColliders)
        if (roles.includes(SEGMENT_BY_ID.get(id).role)) collider.setFriction(friction);
      assert.equal(c.requestStrike(), true);
      for (let tick = 1; tick <= contract.frames; tick++) {
        c.fixedUpdate(contract.dt, null);
        assert.deepEqual(c.runtimeErrors, []);
        const d = c.recovery.diagnostics(), pelvis = c.ragdollBodies.get('pelvis').translation();
        if (result.floorTick === null && pelvis.y < .2
          && d.contacts.some(p => p.loadBearing && ['pelvis', 'torso'].includes(p.segment))) result.floorTick = tick;
        if (result.floorTick !== null) {
          result.peakPelvisY = Math.max(result.peakPelvisY, pelvis.y);
          result.peakLinear = Math.max(result.peakLinear, ...[...c.ragdollBodies.values()].map(b => {
            const v = b.linvel(); return Math.hypot(v.x, v.y, v.z);
          }));
          const torso = c.ragdollBodies.get('torso').translation();
          if (previousTorso) result.torsoHorizontalTravel += Math.hypot(torso.x - previousTorso.x, torso.z - previousTorso.z);
          previousTorso = { ...torso };
          const patches = d.contacts.filter(p => p.loadBearing && roles.includes(SEGMENT_BY_ID.get(p.segment).role));
          const load = patches.reduce((sum, p) => sum + p.forceN, 0);
          if (load > 0) {
            result.supportSamples++;
            result.supportSlipDistance += contract.dt * patches.reduce((sum, p) => {
              const v = c.ragdollBodies.get(p.segment).velocityAtPoint(p.point);
              return sum + Math.hypot(v.x, v.z) * p.forceN / load;
            }, 0);
          }
        }
        if (tick % 60 === 0) result.samples.push({ tick, phase: d.phase, stage: d.transferStage,
          pelvisY: pelvis.y, retries: d.retries, supporting: d.supporting });
      }
      result.recoveries = c.protocolRecoveries;
      result.finalPhase = c.recovery.diagnostics().phase;
    } catch (error) { result.error = String(error); }
    finally { c.dispose(); }
    results.push(result);
    writeFileSync(join(output, 'results.json'), JSON.stringify(results, null, 2));
    console.log(JSON.stringify({ ...result, samples: undefined }));
  }
  assert.deepEqual(fingerprints(), contract.source);
} finally { unregister(); }
