import assert from 'node:assert/strict';
import { writeFileSync } from 'node:fs';
import { join } from 'node:path';
import RAPIER from '@dimforge/rapier3d-compat';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { clampJointCoordinates, jointCoordinates, jointFrameAxesWorld } from '../src/character/joint-coordinates.ts';
import { clamp, dot } from '../src/character/math.ts';
import { sha256 } from './capture-physics-baseline.mjs';

const coordinates = { x: 3, y: 4, z: 5 };
const identity = { x: 0, y: 0, z: 0, w: 1 }, zero = { x: 0, y: 0, z: 0 };
const vector = value => ({ x: value.x, y: value.y, z: value.z });

export function installStandingResponseModel(character, output, index) {
  const world = character.world, originalStep = world.step.bind(world);
  const handles = [...character.ragdollBodies].map(([id, body]) => [id, body.handle]);
  const report = { sampleTicks: [121], samples: [], qualification: 'Read-only copied-world plant response. Native motor settings are held for 30 steps in each copy; only the first baseline step is matched to the live controller. Not a production transfer predictor or standing acceptance.' };
  function capture(current) {
    const bodies = Object.fromEntries(handles.map(([id, handle]) => {
      const body = current.getRigidBody(handle), q = body.rotation();
      const state = { position: vector(body.translation()), rotation: { ...vector(q), w: q.w },
        velocity: vector(body.linvel()), angularVelocity: vector(body.angvel()), com: vector(body.worldCom()), mass: body.mass() };
      assert.ok(Object.values(state).flatMap(value => typeof value === 'number' ? [value] : Object.values(value)).every(Number.isFinite));
      return [id, state];
    }));
    const mass = Object.values(bodies).reduce((sum, body) => sum + body.mass, 0);
    const com = Object.fromEntries(['x', 'y', 'z'].map(axis => [axis,
      Object.values(bodies).reduce((sum, body) => sum + body.mass * body.com[axis], 0) / mass]));
    return { bodies, centerOfMass: com };
  }
  world.step = (...args) => {
    if (!report.sampleTicks.includes(character.fixedSteps)) return originalStep(...args);
    const started = new Date().toISOString();
    const snapshot = world.takeSnapshot(), axes = [];
    for (const command of character.standingCommands) {
      const definition = SEGMENT_BY_ID.get(command.id);
      if (!['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) continue;
      const strength = clamp(command.strengthScale, 0, 1);
      const damping = command.damping * strength, stiffness = command.stiffness * strength;
      assert.ok(damping > 0);
      const target = clampJointCoordinates(jointCoordinates(identity, command.targetLocalRotation, definition.jointProfile), definition.jointProfile);
      const basis = jointFrameAxesWorld(character.ragdollBodies.get(definition.parent).rotation(), definition.jointProfile);
      for (const axis of definition.jointProfile.axes) axes.push({ id: command.id, coordinate: axis.coordinate,
        side: definition.side, handle: character.jointsByChild.get(command.id).handle, abiAxis: coordinates[axis.coordinate],
        stiffness, damping, targetPosition: target[axis.coordinate],
        targetVelocity: dot(command.feedforwardWorld ?? zero, basis[axis.coordinate]) * strength / damping,
        unchangedTorqueCeilingNm: axis.maxMotorTorqueNm * strength });
    }
    assert.equal(axes.length, 14);
    const initial = capture(world);
    function simulate(delta) {
      const copy = RAPIER.World.restoreSnapshot(snapshot), queue = new RAPIER.EventQueue(true);
      try {
        assert.deepEqual(capture(copy), initial, 'Snapshot restoration changed a body field');
        for (let i = 0; i < axes.length; i++) {
          if (delta[i] === 0) continue;
          const axis = axes[i];
          copy.impulseJoints.raw.jointConfigureMotor(axis.handle, axis.abiAxis, axis.targetPosition,
            axis.targetVelocity + delta[i] / axis.damping, axis.stiffness, axis.damping);
        }
        const samples = [];
        for (let tick = 1; tick <= 30; tick++) {
          copy.step(queue, character.physicsHooks);
          queue.clear();
          if ([1, 6, 30].includes(tick)) samples.push({ tick, ...capture(copy) });
        }
        return samples;
      } finally { queue.free(); copy.free(); }
    }
    const cases = [];
    const addCase = (name, kind, delta) => cases.push({ name, kind, deltaTorqueNm: delta, samples: simulate(delta) });
    addCase('baseline', 'baseline', axes.map(() => 0));
    const result = originalStep(...args);
    const actual = capture(world), first = cases[0].samples[0];
    assert.deepEqual(actual.bodies, first.bodies, 'Copied baseline does not exactly predict all 25 live post-step bodies');
    assert.deepEqual(actual.centerOfMass, first.centerOfMass);
    for (let axis = 0; axis < axes.length; axis++) {
      for (const magnitude of [-1, 1, -0.75, 0.75, -1.25, 1.25]) {
        addCase(`${axes[axis].id}:${axes[axis].coordinate}:${magnitude}`, Math.abs(magnitude) === 1 ? 'calibration' : 'held-out-axis',
          axes.map((_, i) => i === axis ? magnitude : 0));
      }
    }
    for (const magnitude of [-0.75, 0.75, -1.25, 1.25]) {
      addCase(`opposed-sides:${magnitude}`, 'held-out-combined', axes.map(axis => magnitude * (axis.side === 'left' ? 1 : -1)));
    }
    const stem = `${index}-response-model-${character.fixedSteps}`;
    writeFileSync(join(output, `${stem}.bin`), snapshot, { flag: 'wx' });
    const data = { started, finished: new Date().toISOString(), tick: character.fixedSteps, axes,
      snapshotSha256: sha256(snapshot), initial, exactLiveBaselineBodies: handles.length, cases,
      qualification: report.qualification };
    const bytes = JSON.stringify(data, null, 2) + '\n';
    writeFileSync(join(output, `${stem}.json`), bytes, { flag: 'wx' });
    report.samples.push({ tick: character.fixedSteps, file: `${stem}.json`, sha256: sha256(bytes),
      snapshot: `${stem}.bin`, snapshotSha256: sha256(snapshot), exactLiveBaselineBodies: handles.length, cases: cases.length });
    console.log(JSON.stringify({ responseModel: stem, cases: cases.length, exactLiveBaselineBodies: handles.length }));
    return result;
  };
  return report;
}
