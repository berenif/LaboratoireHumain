import { writeFileSync } from 'node:fs';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { articulatedCoordinateResponse } from '../src/character/articulated-inertia.ts';
import { jointCoordinates, jointCoordinateKinematics } from '../src/character/joint-coordinates.ts';
import { solveReducedCoordinateMotorTorques } from '../src/character/recovery-motors.ts';
import { add, cross, dot, scale, sub, worldPoint } from '../src/character/math.ts';

const dt = 1 / 60, zero = { x: 0, y: 0, z: 0 };
const mode = process.env.FEEDFORWARD_PROBE_MODE ?? 'all';
const frames = Number(process.env.FEEDFORWARD_PROBE_FRAMES ?? 180);
let vetoes = [];
const c = await createEmbodiedCharacter('canvas2d');
const original = c.motorCommands.bind(c);
function solve(matrix, rhs) {
  const a = matrix.map((row, index) => [...row, rhs[index]]);
  for (let k = 0; k < a.length; k++) {
    let pivot = k;
    for (let j = k + 1; j < a.length; j++) if (Math.abs(a[j][k]) > Math.abs(a[pivot][k])) pivot = j;
    [a[k], a[pivot]] = [a[pivot], a[k]];
    const divisor = a[k][k];
    for (let j = k; j <= a.length; j++) a[k][j] /= divisor;
    for (let i = 0; i < a.length; i++) if (i !== k) {
      const factor = a[i][k];
      for (let j = k; j <= a.length; j++) a[i][j] -= factor * a[k][j];
    }
  }
  return a.map(row => row.at(-1));
}
c.motorCommands = targets => {
  const commands = original(targets);
  if (mode === 'stop') {
    vetoes = [];
    for (const command of commands) {
      const d = SEGMENT_BY_ID.get(command.id), profile = d.jointProfile;
      const parent = c.ragdollBodies.get(d.parent), child = c.ragdollBodies.get(d.id);
      const q = jointCoordinates(parent.rotation(), child.rotation(), profile);
      const kinematics = jointCoordinateKinematics(parent.angvel(), child.angvel(), parent.rotation(), q, profile);
      const worldAxes = profile.axes.map(axis => kinematics.torqueAxesWorld[axis.coordinate]);
      const ff = solve(worldAxes.map(a => worldAxes.map(b => dot(a, b))), worldAxes.map(a => dot(a, command.feedforwardWorld)));
      profile.axes.forEach((axis, index) => {
        const coordinate = q[axis.coordinate], torque = ff[index];
        if ((coordinate <= axis.minRadians + .001 && torque < 0)
          || (coordinate >= axis.maxRadians - .001 && torque > 0)) {
          vetoes.push({ id: d.id, coordinate: axis.coordinate, position: coordinate, removedTorque: torque });
          ff[index] = 0;
        }
      });
      command.feedforwardWorld = worldAxes.reduce((torque, axis, index) => add(torque, scale(axis, ff[index])), zero);
    }
    return commands;
  }
  const inverse = new Map([...c.ragdollBodies].map(([id, body]) => {
    const v = body.effectiveWorldInvInertia();
    return [id, { m11: v.m11, m12: v.m12, m13: v.m13, m22: v.m22, m23: v.m23, m33: v.m33 }];
  }));
  const response = articulatedCoordinateResponse(c.ragdollBodies, c.motorSupportConstraints());
  const selected = new Map(commands.map(command => {
    if (mode === 'all') return [command.id, command.feedforwardWorld];
    const d = SEGMENT_BY_ID.get(command.id), body = c.ragdollBodies.get(d.id);
    const joint = worldPoint(body.translation(), body.rotation(), d.jointProfile.childFrame.anchor);
    let torque = zero;
    for (const load of c.contactLoadPlan.loads) {
      let id = load.segment;
      while (id && id !== d.id) id = SEGMENT_BY_ID.get(id).parent;
      if (id) torque = add(torque, cross(sub(load.point, joint), { x: -load.plannedForce.x, y: 0, z: -load.plannedForce.z }));
    }
    return [command.id, torque];
  }));
  const intents = commands.map(command => {
    const d = SEGMENT_BY_ID.get(command.id), profile = d.jointProfile;
    const parent = c.ragdollBodies.get(d.parent), child = c.ragdollBodies.get(d.id);
    const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
    const axes = jointCoordinateKinematics(zero, zero, parent.rotation(), coordinates, profile).torqueAxesWorld;
    const worldAxes = profile.axes.map(axis => axes[axis.coordinate]);
    const ff = solve(worldAxes.map(a => worldAxes.map(b => dot(a, b))), worldAxes.map(a => dot(a, selected.get(command.id))));
    const gain = (command.damping * dt + command.stiffness * dt * dt) * command.strengthScale;
    return { id: d.id, parent: d.parent, axes: profile.axes.map((axis, i) => ({
      coordinate: axis.coordinate, worldAxis: axes[axis.coordinate], kp: 0, kd: gain / dt,
      error: 0, velocity: -ff[i] * dt / gain, feedforward: 0, cap: axis.maxMotorTorqueNm,
    })) };
  });
  const filtered = solveReducedCoordinateMotorTorques(intents, inverse, dt, { response });
  for (const command of commands) command.feedforwardWorld = add(sub(command.feedforwardWorld, selected.get(command.id)), filtered.get(command.id).torqueWorld);
  return commands;
};
const initial = c.getSnapshot('canvas2d').segments.find(p => p.id === 'rightHand');
const anchor = { x: .025, y: .015, z: .01 }, start = worldPoint(initial.position, initial.rotation, anchor);
c.fixedUpdate(dt, { kind: 'begin', pointerId: 41, region: 'rightHand', segment: 'rightHand', localAnchor: anchor, worldTarget: start, timestampMs: 0 });
const rows = [];
for (let frame = 1; frame <= frames; frame++) {
  const command = frame === 150 ? { kind: 'end', pointerId: 41, timestampMs: frame * dt * 1000 }
    : frame < 150 ? { kind: 'move', pointerId: 41, worldTarget: add(start, { x: 0, y: .03 * Math.min(frame, 90) / 90, z: .70 * Math.min(frame, 90) / 90 }), timestampMs: frame * dt * 1000 } : null;
  c.fixedUpdate(dt, command);
  const s = c.getSnapshot('canvas2d');
  rows.push({ frame, state: s.state, pelvis: s.segments.find(p => p.id === 'pelvis'),
    step: structuredClone(c.step), support: s.support, vetoes,
    joints: s.diagnostics.jointDiagnostics.filter(j => /Shin|Thigh/.test(j.segment)) });
}
writeFileSync(`evidence/collision/feedforward-command-filtered-${mode}.json`, JSON.stringify({ approximateActiveGainFilter: true, rows }, null, 2));
console.log(JSON.stringify({ firstFall: rows.find(r => r.state === 'falling')?.frame, final: rows.at(-1).state,
  transitions: rows.filter((r, i) => !i || r.state !== rows[i - 1].state).map(r => ({ frame: r.frame, state: r.state, pelvis: r.pelvis.position })) }));
c.dispose();
