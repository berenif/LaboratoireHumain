import assert from 'node:assert/strict';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import RAPIER from '@dimforge/rapier3d-compat';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { clampJointCoordinates, jointCoordinates, jointFrameAxesWorld } from '../src/character/joint-coordinates.ts';
import { clamp, dot } from '../src/character/math.ts';
import { sha256 } from './capture-physics-baseline.mjs';

const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 };
const peakTicks = new Map([[0, 530], [Math.PI / 3, 536], [-Math.PI / 4, 688]]);

export function solveDampedAngularStep(columns, residual) {
  const inner = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0), n = columns.length;
  const matrix = columns.map((a, i) => columns.map((b, j) => inner(a, b) + (i === j ? 1e-8 : 0)));
  const rhs = columns.map(column => -inner(column, residual));
  for (let k = 0; k < n; k++) {
    let pivot = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(matrix[i][k]) > Math.abs(matrix[pivot][k])) pivot = i;
    assert.ok(Number.isFinite(matrix[pivot][k]) && Math.abs(matrix[pivot][k]) > 1e-18);
    [matrix[k], matrix[pivot]] = [matrix[pivot], matrix[k]];
    [rhs[k], rhs[pivot]] = [rhs[pivot], rhs[k]];
    for (let i = k + 1; i < n; i++) {
      const factor = matrix[i][k] / matrix[k][k];
      for (let j = k; j < n; j++) matrix[i][j] -= factor * matrix[k][j];
      rhs[i] -= factor * rhs[k];
    }
  }
  const step = Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    step[i] = (rhs[i] - matrix[i].reduce((sum, v, j) => sum + (j > i ? v * step[j] : 0), 0)) / matrix[i][i];
  }
  assert.ok(step.every(Number.isFinite));
  return step;
}

/** Local copied-world experiment, not an online controller or transfer model. */
export function installForefootAuthority(character, output, index, heading, mode, trace, scope = 'forefoot') {
  assert.ok(['observe', 'pulse'].includes(mode));
  assert.ok(['forefoot', 'distal'].includes(scope));
  const ids = scope === 'forefoot' ? ['leftForefoot', 'rightForefoot'] : ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
  const controlIds = scope === 'forefoot' ? ids : ['leftAnkle', 'leftFoot', 'leftForefoot', 'rightAnkle', 'rightFoot', 'rightForefoot'];
  const tick = peakTicks.get(heading);
  assert.ok(tick, 'This diagnostic uses the three retained H22 peak states');
  const world = character.world, originalStep = world.step.bind(world);
  const handles = [...character.ragdollBodies].map(([id, body]) => [id, body.handle]);
  const report = { mode, scope, tick, samples: [], qualification: `Exact one-step copied-world ${scope} authority. A pulse changes ${controlIds.length} existing native velocity biases only; no predicted body state is transferred. This is not sustained standing acceptance.` };
  const capture = current => Object.fromEntries(handles.map(([id, handle]) => {
    const body = current.getRigidBody(handle);
    return [id, { position: { ...body.translation() }, rotation: { ...body.rotation() },
      velocity: { ...body.linvel() }, angularVelocity: { ...body.angvel() },
      centerOfMass: { ...body.worldCom() }, mass: body.mass() }];
  }));
  const metrics = bodies => {
    const finite = Object.values(bodies).every(body => Object.values(body).flatMap(v =>
      typeof v === 'number' ? [v] : Object.values(v)).every(Number.isFinite));
    const residual = ids.flatMap(id => Object.values(bodies[id].angularVelocity));
    const maxLinear = Math.max(...Object.values(bodies).map(b => Math.hypot(...Object.values(b.velocity))));
    const maxOtherAngular = Math.max(...Object.entries(bodies).filter(([id]) => !ids.includes(id))
      .map(([, b]) => Math.hypot(...Object.values(b.angularVelocity))));
    return { finite, residual, score: residual.reduce((sum, v) => sum + v * v, 0), maxLinear, maxOtherAngular,
      withinOtherSpeedBounds: finite && maxLinear <= 0.1 && maxOtherAngular <= 0.5 };
  };
  world.step = (...args) => {
    delete trace.authorityOverride;
    if (character.fixedSteps !== tick) return originalStep(...args);
    const snapshot = world.takeSnapshot(), snapshotSha256 = sha256(snapshot), initial = capture(world);
    const axes = controlIds.map(id => {
      const command = character.standingCommands.find(c => c.id === id), definition = SEGMENT_BY_ID.get(id);
      assert.equal(definition.jointProfile.axes.length, 1);
      const coordinate = definition.jointProfile.axes[0].coordinate;
      assert.equal(coordinate, definition.role === 'hindfoot' ? 'z' : 'x');
      const strength = clamp(command.strengthScale, 0, 1), damping = command.damping * strength;
      assert.ok(damping > 0);
      const parent = character.ragdollBodies.get(definition.parent);
      const basis = jointFrameAxesWorld(parent.rotation(), definition.jointProfile);
      const target = clampJointCoordinates(jointCoordinates(identity, command.targetLocalRotation, definition.jointProfile), definition.jointProfile);
      return { id, handle: character.jointsByChild.get(id).handle, abiAxis: coordinate === 'z' ? 5 : 3,
        stiffness: command.stiffness * strength, damping, targetPosition: target[coordinate],
        targetVelocity: dot(command.feedforwardWorld ?? zero, basis[coordinate]) * strength / damping,
        unchangedTorqueCeilingNm: definition.jointProfile.axes[0].maxMotorTorqueNm * strength };
    });
    const configure = (current, delta) => axes.forEach((axis, i) => {
      if (delta[i] !== 0) current.impulseJoints.raw.jointConfigureMotor(axis.handle, axis.abiAxis,
        axis.targetPosition, axis.targetVelocity + delta[i] / axis.damping, axis.stiffness, axis.damping);
    });
    const stem = `${index}-${scope}-authority-${tick}`;
    if (mode === 'pulse') {
      const directory = process.env.STANDING_AUTHORITY_SOURCE;
      assert.ok(directory, 'Pulse requires the prior read-only authority capture');
      const matches = readdirSync(directory).filter(name => name.endsWith(`-${scope}-authority-${tick}.json`));
      assert.equal(matches.length, 1);
      const sourcePath = join(directory, matches[0]), sourceBytes = readFileSync(sourcePath);
      const source = JSON.parse(sourceBytes);
      assert.equal(source.scope ?? 'forefoot', scope);
      assert.equal(source.heading, heading); assert.equal(source.snapshotSha256, snapshotSha256);
      assert.deepEqual(source.axes, axes); assert.deepEqual(source.initial, initial);
      const delta = source.best.deltaTorqueNm;
      assert.equal(delta.length, axes.length);
      assert.ok(delta.every((d, i) => Number.isFinite(d) && Math.abs(d) <= axes[i].unchangedTorqueCeilingNm));
      configure(world, delta);
      trace.authorityOverride = { sourcePath, deltaTorqueNm: delta, axes,
        qualification: 'These additional native velocity biases are applied after the published base motor requests; the base request telemetry excludes this override. Force ceilings are unchanged.' };
      const result = originalStep(...args), actual = capture(world);
      assert.deepEqual(actual, source.best.bodies, 'Live pulse differs from its exact copied-world prediction');
      const data = { heading, scope, tick, snapshotSha256, sourcePath, sourceSha256: sha256(sourceBytes), axes,
        deltaTorqueNm: delta, actual, metrics: metrics(actual), exactPredictedBodies: handles.length, qualification: report.qualification };
      const bytes = JSON.stringify(data, null, 2) + '\n';
      writeFileSync(join(output, `${stem}.json`), bytes, { flag: 'wx' });
      report.samples.push({ tick, file: `${stem}.json`, sha256: sha256(bytes), exactPredictedBodies: handles.length, deltaTorqueNm: delta });
      return result;
    }
    const cases = [];
    const simulate = (name, delta) => {
      const copy = RAPIER.World.restoreSnapshot(snapshot), queue = new RAPIER.EventQueue(true);
      try {
        assert.deepEqual(capture(copy), initial, 'Restored world changed physical initial fields');
        configure(copy, delta); copy.step(queue, character.physicsHooks); queue.clear();
        const bodies = capture(copy), measured = metrics(bodies);
        const result = { name, deltaTorqueNm: [...delta], bodies, ...measured };
        cases.push(result); return result;
      } finally { queue.free(); copy.free(); }
    };
    const baseline = simulate('baseline', axes.map(() => 0));
    assert.equal(baseline.finite, true);
    let best = baseline;
    for (let iteration = 0; iteration < (scope === 'forefoot' ? 8 : 16) && best.score > 1e-8; iteration++) {
      const columns = [];
      for (let axis = 0; axis < axes.length; axis++) {
        const plus = [...best.deltaTorqueNm], minus = [...best.deltaTorqueNm], cap = axes[axis].unchangedTorqueCeilingNm;
        plus[axis] = clamp(plus[axis] + 1, -cap, cap); minus[axis] = clamp(minus[axis] - 1, -cap, cap);
        const positive = simulate(`iteration-${iteration}-axis-${axis}-plus`, plus);
        const negative = simulate(`iteration-${iteration}-axis-${axis}-minus`, minus);
        assert.ok(positive.finite && negative.finite, 'Non-finite finite-difference probe');
        columns.push(positive.residual.map((v, i) => (v - negative.residual[i]) / (plus[axis] - minus[axis])));
      }
      let step;
      if (axes.length === 2) {
        // Retain the original two-axis arithmetic for exact H46 replay.
        const inner = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
        const a = inner(columns[0], columns[0]) + 1e-8, b = inner(columns[0], columns[1]);
        const d = inner(columns[1], columns[1]) + 1e-8;
        const e = -inner(columns[0], best.residual), f = -inner(columns[1], best.residual), determinant = a * d - b * b;
        assert.ok(determinant > 0 && Number.isFinite(determinant));
        step = [(d * e - b * f) / determinant, (a * f - b * e) / determinant];
      } else step = solveDampedAngularStep(columns, best.residual);
      const trust = Math.max(1, ...step.map(Math.abs));
      let next;
      for (const fraction of [1, 0.5, 0.25, 0.125]) {
        const delta = step.map((v, i) => clamp(best.deltaTorqueNm[i] + fraction * v / trust,
          -axes[i].unchangedTorqueCeilingNm, axes[i].unchangedTorqueCeilingNm));
        const candidate = simulate(`iteration-${iteration}-line-${fraction}`, delta);
        if (candidate.withinOtherSpeedBounds && candidate.score < best.score - 1e-12) { next = candidate; break; }
      }
      if (!next) break;
      best = next;
    }
    const result = originalStep(...args), actual = capture(world);
    assert.deepEqual(actual, baseline.bodies, 'Copied baseline differs from the 25 live bodies');
    writeFileSync(join(output, `${stem}.bin`), snapshot, { flag: 'wx' });
    const data = { heading, scope, tick, snapshotSha256, initial, axes, baseline, best, cases,
      exactLiveBaselineBodies: handles.length, qualification: report.qualification };
    const bytes = JSON.stringify(data, null, 2) + '\n';
    writeFileSync(join(output, `${stem}.json`), bytes, { flag: 'wx' });
    report.samples.push({ tick, file: `${stem}.json`, sha256: sha256(bytes), cases: cases.length,
      snapshot: `${stem}.bin`, snapshotSha256, exactLiveBaselineBodies: handles.length,
      baselineScore: baseline.score, bestScore: best.score, deltaTorqueNm: best.deltaTorqueNm });
    console.log(JSON.stringify({ authority: stem, cases: cases.length, baselineScore: baseline.score,
      bestScore: best.score, deltaTorqueNm: best.deltaTorqueNm }));
    return result;
  };
  return report;
}
