import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints } from './capture-physics-baseline.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const output = process.argv[2];
assert.ok(output, 'Supply a fresh output directory');
mkdirSync(output);
const contract = { modes: JSON.parse(process.env.RECOVERY_PROBE_MODES ?? '["baseline","no-drive","distributed-torque"]'), interveneAtTick: 182, frames: 300,
  dt: 1 / 60, source: fingerprints(),
  purpose: 'Matched real strike followed by the second floor recovery attempt. Compare pre-step impulses with the same torque distributed over native substeps. No-drive is diagnostic only.' };
writeFileSync(join(output, 'contract.json'), JSON.stringify(contract, null, 2));
const results = [];
const speed = v => Math.hypot(v.x, v.y, v.z);
try {
  for (const mode of contract.modes) {
    const c = await createEmbodiedCharacter('canvas2d', { room: true });
    const samples = [], prefix = createHash('sha256');
    const measure = () => ({ pelvisY: c.ragdollBodies.get('pelvis').translation().y,
      maxLinear: Math.max(...[...c.ragdollBodies.values()].map(b => speed(b.linvel()))),
      maxAngular: Math.max(...[...c.ragdollBodies.values()].map(b => speed(b.angvel()))) });
    let actuationBefore, actuationAfter;
    const originalActuate = c.recovery.actuate.bind(c.recovery);
    c.recovery.actuate = (...args) => {
      actuationBefore = measure();
      originalActuate(...args);
      actuationAfter = measure();
    };
    const originalExecute = c.recovery.executeMotorCommands.bind(c.recovery);
    c.recovery.executeMotorCommands = (bodies, commands, active, dt) => {
      if (mode === 'native' && c.fixedSteps >= contract.interveneAtTick) {
        c.recovery.nativeRollArmCommands = commands;
        c.recovery.lastMotorResults = new Map();
        return;
      }
      return originalExecute(bodies, commands, active, dt);
    };
    for (const body of c.ragdollBodies.values()) {
      const original = body.applyTorqueImpulse.bind(body);
      body.applyTorqueImpulse = (impulse, wake) => {
        if (c.fixedSteps < contract.interveneAtTick || mode === 'baseline' || mode === 'native') return original(impulse, wake);
        if (mode === 'distributed-torque') body.addTorque({ x: impulse.x / contract.dt, y: impulse.y / contract.dt, z: impulse.z / contract.dt }, wake);
      };
    }
    const originalStep = c.world.step.bind(c.world);
    c.world.step = (...args) => {
      if (mode === 'no-drive' && c.fixedSteps >= contract.interveneAtTick) c.nativeMotors.disable(c.jointsByChild);
      try { return originalStep(...args); }
      finally { if (mode === 'distributed-torque') for (const body of c.ragdollBodies.values()) body.resetTorques(false); }
    };
    let failure;
    try {
      assert.equal(c.requestStrike(), true);
      for (let tick = 1; tick <= contract.frames; tick++) {
        c.fixedUpdate(contract.dt, null);
        if (tick < contract.interveneAtTick) prefix.update(JSON.stringify([...c.poses.values()]));
        const d = c.recovery.diagnostics();
        samples.push({ tick, state: c.state, phase: d.phase, stage: d.transferStage, retries: d.retries,
          ...measure(), actuationBefore, actuationAfter });
        assert.deepEqual(c.runtimeErrors, []);
      }
    } catch (error) { failure = String(error); }
    finally { c.dispose(); }
    const post = samples.filter(s => s.tick >= contract.interveneAtTick);
    const result = { mode, prefixHash: prefix.digest('hex'), failure,
      peakPelvisY: Math.max(...post.map(s => s.pelvisY)), peakLinear: Math.max(...post.map(s => s.maxLinear)),
      peakAngular: Math.max(...post.map(s => s.maxAngular)),
      peakBeforeIntegrationAngular: Math.max(...post.map(s => s.actuationAfter?.maxAngular ?? 0)), final: samples.at(-1) };
    results.push(result);
    writeFileSync(join(output, `${mode}.json`), JSON.stringify({ result, samples }, null, 2));
    console.log(JSON.stringify(result));
  }
  assert.equal(new Set(results.map(r => r.prefixHash)).size, 1, 'Interventions changed the shared prefix');
  assert.deepEqual(fingerprints(), contract.source, 'Source changed during probe');
  writeFileSync(join(output, 'results.json'), JSON.stringify(results, null, 2));
} finally { unregister(); }
