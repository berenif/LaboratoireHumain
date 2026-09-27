import assert from 'node:assert/strict';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { spawnSync } from 'node:child_process';
import RAPIER from '@dimforge/rapier3d-compat';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { solveDampedAngularStep } from './standing-forefoot-authority.mjs';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { pelvisPoseRates, pelvisBoundedResidual } from './standing-pelvis-task.mjs';

const [output, rawDirectory, fullDirectory, method = 'dimensional', scopeSet = 'distal-knees', objective = 'feet'] = process.argv.slice(2);
assert.ok(output && rawDirectory && fullDirectory, 'Specify fresh output, H49 raw directory and H50 full-step directory');
assert.ok(['dimensional', 'dimensionless', 'guarded'].includes(method));
assert.ok(['distal-knees', 'knees-legs'].includes(scopeSet));
assert.ok(['feet', 'pelvis', 'pelvis-bounded'].includes(objective));
const usePelvis = objective !== 'feet';
if (usePelvis) assert.equal(method, 'guarded');
mkdirSync(output, { recursive: false });
const sourceBefore = fingerprints(), started = new Date().toISOString();
const executable = resolve('evidence/rapier-calibration-f32-target/release/snapshot_motors.exe');
const executableSha256 = sha256(readFileSync(executable));
const feet = ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const rawParts = handle => {
  const data = new DataView(new ArrayBuffer(8)); data.setFloat64(0, handle, true);
  return [data.getUint32(0, true), data.getUint32(4, true)];
};
const results = [];
for (const [index, heading] of [0, Math.PI / 3, -Math.PI / 4].entries()) {
  const input = join(rawDirectory, `${index}-distal-preview-350.bin`);
  const sourceFile = join(rawDirectory, `${index}-distal-preview-350.json`);
  const fullFile = join(fullDirectory, `${index}-result.json`);
  const saved = JSON.parse(readFileSync(sourceFile, 'utf8')), full = JSON.parse(readFileSync(fullFile, 'utf8'));
  const snapshot = readFileSync(input);
  assert.equal(sha256(snapshot), saved.snapshotSha256);
  assert.equal(full.snapshotSha256, saved.snapshotSha256);
  const nativeFile = join(output, `${index}-native-motors.json`);
  const invocation = spawnSync(executable, [input, nativeFile], { encoding: 'utf8', windowsHide: true, timeout: 60000 });
  writeFileSync(join(output, `${index}-native-invocation.json`), JSON.stringify({
    executable, executableSha256, input, snapshotSha256: saved.snapshotSha256,
    status: invocation.status, stdout: invocation.stdout, stderr: invocation.stderr,
  }, null, 2) + '\n');
  assert.equal(invocation.status, 0, invocation.stderr);
  const native = JSON.parse(readFileSync(nativeFile, 'utf8'));
  assert.equal(native.originalBytes, native.serializedBytes);
  assert.equal(native.otherSectionsExact, true, 'Non-joint snapshot fields changed in native inspection');
  assert.equal(native.impulseJointPayloadExact, true, 'Native joint payload differs from original bytes');
  assert.equal(native.wakeUpMembersExact, true, 'Wake-up membership changed');
  assert.equal(native.joinMembersExact, true, 'Island join membership changed');
  const character = await createEmbodiedCharacter('canvas2d', { heading });
  const handles = [...character.ragdollBodies].map(([id, body]) => [id, body.handle]);
  const capture = world => Object.fromEntries(handles.map(([id, handle]) => {
    const body = world.getRigidBody(handle);
    return [id, { position: { ...body.translation() }, rotation: { ...body.rotation() },
      velocity: { ...body.linvel() }, angularVelocity: { ...body.angvel() },
      centerOfMass: { ...body.worldCom() }, mass: body.mass() }];
  }));
  const nativeJoint = handle => native.joints.find(j => JSON.stringify(j.handle) === JSON.stringify(rawParts(handle)));
  try {
    for (const axis of saved.axes) {
      const motor = nativeJoint(axis.handle)?.motors[axis.abiAxis]; assert.ok(motor);
      for (const key of ['targetPosition', 'targetVelocity', 'stiffness', 'damping']) {
        assert.equal(motor[key], Math.fround(axis[key]), `${axis.id} ${key}`);
      }
      assert.equal(motor.maxForce, Math.fround(axis.unchangedTorqueCeilingNm));
      assert.equal(motor.model, 'ForceBased');
    }
    const knees = ['leftShin', 'rightShin'].map(id => {
      const handle = character.jointsByChild.get(id).handle, joint = nativeJoint(handle);
      const profile = SEGMENT_BY_ID.get(id).jointProfile;
      assert.equal(profile.axes.length, 1); assert.equal(profile.axes[0].coordinate, 'x');
      assert.equal(joint.motorAxes, 1 << 3); assert.equal(joint.lockedAxes & (1 << 3), 0);
      const motor = joint.motors[3]; assert.equal(motor.model, 'ForceBased'); assert.ok(motor.damping > 0);
      assert.ok(motor.maxForce > 0 && motor.maxForce <= Math.fround(profile.axes[0].maxMotorTorqueNm));
      return { id, handle, abiAxis: 3, targetPosition: motor.targetPosition,
        targetVelocity: motor.targetVelocity, stiffness: motor.stiffness, damping: motor.damping,
        unchangedTorqueCeilingNm: motor.maxForce, provenance: 'Exact native f32 parameters from this snapshot' };
    });
    const hips = scopeSet === 'knees-legs' ? ['leftThigh', 'rightThigh'].flatMap(id => {
      const handle = character.jointsByChild.get(id).handle, joint = nativeJoint(handle);
      const definition = SEGMENT_BY_ID.get(id), profile = definition.jointProfile;
      assert.deepEqual(joint.parent, rawParts(character.ragdollBodies.get(definition.parent).handle));
      assert.deepEqual(joint.child, rawParts(character.ragdollBodies.get(id).handle));
      return profile.axes.map(axis => {
        const abiAxis = { x: 3, y: 4, z: 5 }[axis.coordinate], motor = joint.motors[abiAxis];
        assert.ok(joint.motorAxes & (1 << abiAxis)); assert.equal(joint.lockedAxes & (1 << abiAxis), 0);
        assert.equal(motor.model, 'ForceBased'); assert.ok(motor.damping > 0);
        assert.ok(motor.maxForce > 0 && motor.maxForce <= Math.fround(axis.maxMotorTorqueNm));
        return { id, coordinate: axis.coordinate, handle, abiAxis, targetPosition: motor.targetPosition,
          targetVelocity: motor.targetVelocity, stiffness: motor.stiffness, damping: motor.damping,
          unchangedTorqueCeilingNm: motor.maxForce, provenance: 'Exact native f32 parameters from this snapshot' };
      });
    }) : [];
    for (const scope of scopeSet === 'knees-legs' ? ['knees', 'legs'] : ['distal', 'knees']) {
      const axes = scope === 'legs' ? [...saved.axes, ...knees, ...hips]
        : scope === 'knees' ? [...saved.axes, ...knees] : saved.axes;
      const seed = [...full.best.deltaTorqueNm, ...Array(axes.length - saved.axes.length).fill(0)], cases = [];
      const simulate = (name, delta) => {
        const world = RAPIER.World.restoreSnapshot(snapshot), queue = new RAPIER.EventQueue(true);
        try {
          assert.deepEqual(capture(world), saved.initial, 'Restored initial fields differ');
          axes.forEach((a, i) => {
            if (delta[i] !== 0) world.impulseJoints.raw.jointConfigureMotor(a.handle, a.abiAxis,
              a.targetPosition, a.targetVelocity + delta[i] / a.damping, a.stiffness, a.damping);
          });
          world.step(queue, character.physicsHooks); queue.clear();
          const bodies = capture(world);
          const horizontalResidual = feet.flatMap(id => ['x', 'z'].map(k =>
            (bodies[id].position[k] - saved.initial[id].position[k]) / world.timestep));
          const pelvis = usePelvis ? pelvisPoseRates(saved.initial.pelvis, bodies.pelvis, world.timestep) : null;
          const residual = pelvis ? [...horizontalResidual, ...(objective === 'pelvis-bounded' ? pelvisBoundedResidual(pelvis)
            : [...pelvis.pelvisLinearResidual, ...pelvis.pelvisAngularResidual])] : horizontalResidual;
          const finite = Object.values(bodies).every(b => Object.values(b).flatMap(v =>
            typeof v === 'number' ? [v] : Object.values(v)).every(Number.isFinite));
          const maxLinear = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.velocity))));
          const maxAngular = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.angularVelocity))));
          const maxFootAngular = Math.max(...feet.map(id => Math.hypot(...Object.values(bodies[id].angularVelocity))));
          const result = { name, deltaTorqueNm: [...delta], bodies, residual, finite,
            score: residual.reduce((s, v, i) => s + (pelvis ? (v / (i < 11 ? 0.1 : 0.5)) ** 2 : v * v), 0),
            ...(pelvis ?? {}),
            maxHorizontalRate: Math.max(...feet.map((_, i) => Math.hypot(...horizontalResidual.slice(i * 2, i * 2 + 2)))),
            maxLinear, maxAngular, maxFootAngular,
            feasible: finite && maxLinear <= 0.1 && maxAngular <= 0.5 && maxFootAngular <= 0.4 };
          cases.push(result); return result;
        } finally { queue.free(); world.free(); }
      };
      const baseline = simulate('baseline', axes.map(() => 0));
      assert.deepEqual(baseline.bodies, saved.baseline.bodies);
      const retained = simulate('h50-best', seed);
      assert.deepEqual(retained.bodies, full.best.bodies); assert.equal(retained.feasible, true);
      let best = retained;
      const reached = c => c.maxHorizontalRate <= 0.0002 && (!usePelvis
        || (c.maxPelvisLinearRate <= 0.01 && c.maxPelvisAngularRate <= 0.05));
      for (let iteration = 0; iteration < 8 && !reached(best); iteration++) {
        const probeStart = cases.length;
        const columns = axes.map((axis, i) => {
          const p = [...best.deltaTorqueNm], m = [...best.deltaTorqueNm];
          p[i] = Math.min(axis.unchangedTorqueCeilingNm, p[i] + 1);
          m[i] = Math.max(-axis.unchangedTorqueCeilingNm, m[i] - 1);
          const a = simulate(`${iteration}-${i}-plus`, p), b = simulate(`${iteration}-${i}-minus`, m);
          assert.ok(a.finite && b.finite);
          return a.residual.map((v, j) => (v - b.residual[j]) / (p[i] - m[i]));
        });
        let step = method !== 'dimensional'
          ? solveDampedAngularStep(columns.map((column, i) => column.map((v, j) => v * axes[i].unchangedTorqueCeilingNm
            / (usePelvis && j >= 11 ? 0.5 : 0.1))),
            best.residual.map((v, j) => v / (usePelvis && j >= 11 ? 0.5 : 0.1))).map((v, i) => v * axes[i].unchangedTorqueCeilingNm)
          : solveDampedAngularStep(columns, best.residual);
        if (method === 'guarded') {
          const scale = Math.max(1, ...step.map((v, i) => Math.abs(v) / axes[i].unchangedTorqueCeilingNm));
          step = step.map(v => v / scale);
        }
        const bestProbe = method === 'guarded' ? cases.slice(probeStart)
          .filter(c => c.feasible && c.score < best.score - 1e-14).sort((a, b) => a.score - b.score)[0] : null;
        let next;
        for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625, 0.03125]) {
          const delta = step.map((v, i) => Math.max(-axes[i].unchangedTorqueCeilingNm,
            Math.min(axes[i].unchangedTorqueCeilingNm, best.deltaTorqueNm[i] + fraction * v)));
          const candidate = simulate(`${iteration}-line-${fraction}`, delta);
          if (candidate.feasible && candidate.score < best.score - 1e-14) { next = candidate; break; }
        }
        if (bestProbe && (!next || bestProbe.score < next.score)) next = bestProbe;
        if (!next) break;
        best = next;
      }
      const run = { heading, scope, method, objective, tick: 350, sourceFile, fullFile, snapshotSha256: saved.snapshotSha256,
        axes, exactBaselineBodies: 25, exactRetainedBodies: 25, baseline, retained, best, cases };
      writeFileSync(join(output, `${index}-${scope}.json`), JSON.stringify(run, null, 2) + '\n');
      const summary = { heading, scope, method, objective, cases: cases.length, baselineRate: retained.maxHorizontalRate,
        bestRate: best.maxHorizontalRate, bestScore: best.score, delta: best.deltaTorqueNm,
        deadbandReached: reached(best), feasible: best.feasible,
        ...(usePelvis ? { maxPelvisLinearRate: best.maxPelvisLinearRate, maxPelvisAngularRate: best.maxPelvisAngularRate } : {}) };
      results.push(summary); console.log(JSON.stringify(summary));
    }
  } finally { character.dispose(); }
}
const sourceAfter = fingerprints(); assert.deepEqual(sourceBefore, sourceAfter);
assert.equal(sha256(readFileSync(executable)), executableSha256);
writeFileSync(join(output, 'report.json'), JSON.stringify({ started, finished: new Date().toISOString(),
  command: [process.execPath, ...process.execArgv, ...process.argv.slice(1)], nodeVersion: process.version,
  sourceBefore, sourceAfter, executable, executableSha256, method, scopeSet, objective, results,
  qualification: 'Copied one-step knee/distal authority only. No live actuation or production adoption.' }, null, 2) + '\n');
