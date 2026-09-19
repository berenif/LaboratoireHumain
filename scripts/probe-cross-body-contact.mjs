import { mkdirSync, writeFileSync } from 'node:fs';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { jointRotationFromCoordinates } from '../src/character/joint-coordinates.ts';
import { applyPassiveJointResistance } from '../src/character/joint-motors.ts';
import { limbBodyClearance } from '../src/character/limb-collisions.ts';
import { reconstructRecoveryLimb, recoveryLimbIds } from '../src/character/recovery-joints.ts';
import { add, cross, dot, length, lerp, quatInverse, rotate, sub, worldPoint } from '../src/character/math.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';

const dt = 1 / 60;
const output = process.argv[2] ?? 'evidence/collision/cross-body-contact.json';
const all = process.argv.includes('--all');
const cases = all
  ? [0, Math.PI / 3].flatMap(heading => ['leftHand', 'rightHand', 'leftFoot', 'rightFoot'].flatMap(limb =>
    (limb.endsWith('Hand') ? ['chest', 'abdomen', 'opposite-shoulder'] : ['midline']).map(route => ({ limb, route, heading }))))
  : [{ limb: process.env.COLLISION_PROBE_LIMB ?? 'leftFoot', route: process.env.COLLISION_PROBE_ROUTE ?? 'midline',
    heading: Number(process.env.COLLISION_PROBE_HEADING ?? Math.PI / 3) }];
const results = [];
const velocityAt = (body, point) => add(body.linvel(), cross(body.angvel(), sub(point, body.worldCom())));
for (const test of cases) {
  const side = (process.env.COLLISION_PROBE_PAIR?.split('/')[1] ?? test.limb).startsWith('right') ? 'right' : 'left';
  const pair = process.env.COLLISION_PROBE_PAIR?.split('/') ?? ['torso', `${side}UpperArm`];
  const c = await createEmbodiedCharacter('canvas2d', { heading: test.heading });
  if (process.env.COLLISION_PROBE_ARTICULATED_SETTLE) {
    const actuate = c.recovery.actuate.bind(c.recovery);
    c.recovery.actuate = (bodies, poses, active, dt) => {
      if (c.recovery.data.phase !== 'settle') return actuate(bodies, poses, active, dt);
      c.recovery.lastPassiveTorques = applyPassiveJointResistance(bodies, dt, { supports: [] });
      for (const torque of c.recovery.lastPassiveTorques.values())
        c.recovery.data.maxMotorTorqueNm = Math.max(c.recovery.data.maxMotorTorqueNm, length(torque));
    };
  }
  if (process.env.COLLISION_PROBE_CCD_SUBSTEPS) c.world.integrationParameters.maxCcdSubsteps = Number(process.env.COLLISION_PROBE_CCD_SUBSTEPS);
  const frames = [];
  let worst = null;
  const record = (frame) => {
    const s = c.getSnapshot('canvas2d');
    const poses = new Map(s.segments.map(p => [p.id, p]));
    const torso = c.ragdollColliders.get(pair[0]), arm = c.ragdollColliders.get(pair[1]);
    const contact = torso.contactCollider(arm, .06);
    const torsoBody = c.ragdollBodies.get(pair[0]), armBody = c.ragdollBodies.get(pair[1]);
    let impulse = 0, normal = null;
    c.world.contactPair(torso, arm, (manifold, flipped) => {
      const n = manifold.normal();
      normal = flipped ? { x: -n.x, y: -n.y, z: -n.z } : n;
      for (let i = 0; i < manifold.numContacts(); i++) impulse += Math.max(0, manifold.contactImpulse(i));
    });
    const relativeVelocity = contact ? sub(velocityAt(armBody, contact.point2), velocityAt(torsoBody, contact.point1)) : null;
    // Settle explicitly has no pose motors. Other recovery phases now expose
    // the actual bounded motor commands through the same diagnostics.
    let plannedClearance = null;
    if (s.diagnostics.recovery.phase !== 'settle') {
      const rotations = new Map(recoveryLimbIds(side, true).map(id => {
        const joint = s.diagnostics.jointDiagnostics.find(j => j.segment === id);
        return [id, jointRotationFromCoordinates(joint.targetCoordinates, SEGMENT_BY_ID.get(id).jointProfile)];
      }));
      const planned = new Map(poses);
      for (const pose of reconstructRecoveryLimb(side, true, poses.get('torso'), rotations).poses) planned.set(pose.id, pose);
      plannedClearance = limbBodyClearance(planned, side, 'arm');
    }
    const row = { frame, simulationTime: s.simulationTime, state: s.state, phase: s.diagnostics.recovery.phase,
      selfPenetrationM: s.diagnostics.maxSelfPenetrationM, selfPenetrationPair: s.diagnostics.selfPenetrationPair,
      pair, clearanceM: contact?.distance ?? null, impulseNs: impulse, normal,
      relativeVelocityMps: relativeVelocity, relativeSpeedMps: relativeVelocity ? length(relativeVelocity) : null,
      normalRelativeSpeedMps: relativeVelocity && normal ? dot(relativeVelocity, normal) : null,
      localArmContactPoint: contact ? rotate(quatInverse(armBody.rotation()), sub(contact.point2, armBody.translation())) : null,
      measuredClearance: limbBodyClearance(poses, side, 'arm'), plannedClearance,
      plannedClearanceUnavailableReason: plannedClearance ? null : s.diagnostics.recovery.phase === 'settle'
        ? 'Settle applies passive resistance and structural limits only; no commanded posture.' : 'Recovery motor results are not exposed by the standing joint diagnostics.',
      upperArmContactSkinM: arm.contactSkin(), blocking: s.diagnostics.recovery.blockingPredicate,
      torso: poses.get('torso'), upperArm: poses.get(pair[1]),
      recovery: { landingTimeS: c.recovery.landingTime, phaseTimeS: s.diagnostics.recovery.phaseTimeS,
        settledTimeS: s.diagnostics.recovery.settledTimeS, contacts: s.diagnostics.recovery.contacts },
      jointDiagnostics: s.diagnostics.jointDiagnostics.filter(j => recoveryLimbIds(side, true).includes(j.segment)) };
    frames.push(row);
    if (!worst || row.selfPenetrationM > worst.selfPenetrationM) worst = row;
  };
  for (let frame = -60; frame < 0; frame++) { c.fixedUpdate(dt, null); record(frame); }
  const initial = c.getSnapshot('canvas2d');
  const start = initial.segments.find(p => p.id === test.limb).position;
  c.fixedUpdate(dt, { kind: 'begin', pointerId: 701, region: test.limb, segment: test.limb,
    localAnchor: { x: 0, y: 0, z: 0 }, worldTarget: start, timestampMs: initial.simulationTime * 1000 });
  record(0);
  const begun = c.getSnapshot('canvas2d');
  const reference = begun.segments.find(p => p.id === (test.limb.endsWith('Hand') ? 'torso' : 'pelvis'));
  const sign = test.limb.startsWith('left') ? 1 : -1;
  const local = test.route === 'chest' ? { x: sign * .18, y: 0, z: .14 }
    : test.route === 'abdomen' ? { x: sign * .16, y: -.24, z: .13 }
      : test.route === 'opposite-shoulder' ? { x: sign * .28, y: .12, z: .10 }
        : { x: sign * .16, y: -.85, z: .13 };
  const target = worldPoint(reference.position, reference.rotation, local);
  for (let frame = 1; frame <= 90; frame++) {
    c.fixedUpdate(dt, { kind: 'move', pointerId: 701, worldTarget: lerp(start, target, frame / 90), timestampMs: (60 + frame) * dt * 1000 });
    record(frame);
  }
  c.fixedUpdate(dt, { kind: 'end', pointerId: 701, timestampMs: 151 * dt * 1000 }); record(91);
  results.push({ ...test, worst, nearWorst: frames.filter(f => Math.abs(f.frame - worst.frame) <= 5), frames });
  console.log(JSON.stringify({ ...test, worst: { frame: worst.frame, state: worst.state, phase: worst.phase,
    depthM: worst.selfPenetrationM, pair: worst.selfPenetrationPair, impulseNs: worst.impulseNs,
    relativeSpeedMps: worst.relativeSpeedMps } }));
  c.dispose();
}
mkdirSync(new URL('../evidence/collision/', import.meta.url), { recursive: true });
writeFileSync(output, JSON.stringify({ dt, source: 'Exact cross-body acceptance trajectory, post-begin reference pose; explicit contact pair in each frame.', results }, null, 2));
