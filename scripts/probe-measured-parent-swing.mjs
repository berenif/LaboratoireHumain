import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { jointRotationFromCoordinates } from '../src/character/joint-coordinates.ts';
import { reconstructRecoveryLimb, recoveryLimbIds } from '../src/character/recovery-joints.ts';
import { solveRecoveryLegTarget } from '../src/character/recovery-foot-targets.ts';
import { add, length, quatInverse, quatMultiply, rotate, sub, worldPoint } from '../src/character/math.ts';

const dt = 1 / 60, count = Number(process.env.MEASURED_SWING_FRAMES ?? 240);
const headingMeasured = !!process.env.MEASURED_SWING_HEADING;
const solver = process.env.MEASURED_SWING_SOLVER ?? 'recovery';
const prefix = process.env.MEASURED_SWING_OUTPUT ?? `evidence/collision/measured-parent-swing-${solver}-${headingMeasured ? 'heading' : 'fixed-heading'}`;
let composeMeasuredLeg;
if (solver === 'standing') {
  // Diagnostic-only access to the existing compositor; no source-file edit and
  // no reimplementation of its anatomical conventions.
  const ts = await import('typescript');
  const sourceUrl = new URL('../src/character/pose.ts', import.meta.url);
  const source = readFileSync(sourceUrl, 'utf8') + '\nexport { composeLeg };\n';
  const compiled = ts.transpileModule(source, { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.ESNext } }).outputText;
  const absolute = compiled.replace(/from "(\.[^"]+)"/g, (_match, specifier) =>
    `from "${new URL(specifier + '.ts', sourceUrl).href}"`);
  ({ composeLeg: composeMeasuredLeg } = await import('data:text/javascript;base64,' + Buffer.from(absolute).toString('base64')));
}
const c = await createEmbodiedCharacter('canvas2d');
if (headingMeasured) {
  const standing = c.updateStanding.bind(c);
  c.updateStanding = dt => {
    const forward = rotate(c.poses.get('pelvis').rotation, { x: 0, y: 0, z: 1 });
    c.heading = Math.atan2(forward.x, forward.z);
    return standing(dt);
  };
}
const original = c.motorCommands.bind(c);
let correction = null;
c.motorCommands = targets => {
  const commands = original(targets);
  correction = null;
  if (c.step && c.step.elapsed >= 0) {
    const side = c.step.foot.startsWith('left') ? 'left' : 'right';
    const requested = targets.get(c.step.foot), parent = c.poses.get('pelvis');
    const beforeRotations = new Map(recoveryLimbIds(side, false).map(id => [id, commands.find(command => command.id === id).targetLocalRotation]));
    const before = reconstructRecoveryLimb(side, false, parent, beforeRotations).poses.find(p => p.id === c.step.foot);
    let solved;
    if (composeMeasuredLeg) {
      const composed = new Map(targets);
      composed.set('pelvis', parent);
      composeMeasuredLeg(side, parent, composed,
        c.activeGrab ? sub(c.activeGrab.target, c.activeGrab.startTarget) : { x: 0, y: 0, z: 0 },
        { rootTranslation: parent.position, reactionOffset: c.reactionOffset, simulationTime: c.simulationTime,
          activeGrab: c.activeGrab, supportFeet: c.supportFeet, supportFootRotations: c.supportFootRotations,
          measuredPoses: c.poses, step: c.step, heading: c.heading, kneeFlexion: c.kneeFlexion });
      const jointRotations = {};
      for (const id of recoveryLimbIds(side, false)) {
        const definition = SEGMENT_BY_ID.get(id), child = composed.get(id), prior = composed.get(definition.parent);
        const suffix = id.slice(side.length);
        jointRotations[suffix[0].toLowerCase() + suffix.slice(1)] = quatMultiply(quatInverse(prior.rotation), child.rotation);
      }
      solved = { position: composed.get(c.step.foot).position, jointRotations,
        reachErrorM: length(sub(composed.get(c.step.foot).position, requested.position)), jointLimitErrorRad: 0 };
    } else solved = solveRecoveryLegTarget(side, c.poses, requested.position, requested.rotation);
    if (solved) {
      for (const [suffix, rotation] of Object.entries(solved.jointRotations)) {
        const id = side + suffix[0].toUpperCase() + suffix.slice(1);
        commands.find(command => command.id === id).targetLocalRotation = rotation;
      }
      correction = { foot: c.step.foot, requested: requested.position, parent: structuredClone(parent),
        before: before.position, beforeErrorM: length(sub(before.position, requested.position)),
        after: solved.position, afterErrorM: solved.reachErrorM, rawLimitErrorRad: solved.jointLimitErrorRad };
    }
  }
  return commands;
};
const initial = c.getSnapshot('canvas2d').segments.find(p => p.id === 'rightHand');
const anchor = { x: .025, y: .015, z: .01 }, start = worldPoint(initial.position, initial.rotation, anchor);
c.fixedUpdate(dt, { kind: 'begin', pointerId: 41, region: 'rightHand', segment: 'rightHand', localAnchor: anchor, worldTarget: start, timestampMs: 0 });
const rows = [];
for (let frame = 1; frame <= count; frame++) {
  const command = frame === 150 ? { kind: 'end', pointerId: 41, timestampMs: frame * dt * 1000 }
    : frame < 150 ? { kind: 'move', pointerId: 41, worldTarget: add(start, { x: 0, y: .03 * Math.min(frame, 90) / 90, z: .70 * Math.min(frame, 90) / 90 }), timestampMs: frame * dt * 1000 } : null;
  c.fixedUpdate(dt, command);
  const s = c.getSnapshot('canvas2d');
  const plannedRotations = new Map(s.diagnostics.jointDiagnostics.map(j =>
    [j.segment, jointRotationFromCoordinates(j.targetCoordinates, SEGMENT_BY_ID.get(j.segment).jointProfile)]));
  rows.push({ frame, state: s.state, correction, step: structuredClone(c.step), contacts: structuredClone(c.lastContacts),
    support: s.support, pelvis: s.segments.find(p => p.id === 'pelvis'),
    feet: s.segments.filter(p => /^(left|right)Foot$/.test(p.id)),
    joints: s.diagnostics.jointDiagnostics.filter(j => /Thigh|Shin|Ankle|Foot/.test(j.segment)),
    plannedRotations: [...plannedRotations], depthM: s.diagnostics.maxSelfPenetrationM, pair: s.diagnostics.selfPenetrationPair });
}
const summary = { solver, headingMeasured, count, firstFall: rows.find(row => row.state === 'falling')?.frame, finalState: rows.at(-1).state,
  maxStepCount: c.stepCount, maxCollision: rows.reduce((worst, row) => row.depthM > worst.depthM ? { frame: row.frame, depthM: row.depthM, pair: row.pair } : worst, { depthM: 0 }),
  transitions: rows.filter((row, i) => !i || row.state !== rows[i - 1].state || row.step?.foot !== rows[i - 1].step?.foot)
    .map(row => ({ frame: row.frame, state: row.state, step: row.step, pelvis: row.pelvis.position })) };
mkdirSync('evidence/collision', { recursive: true });
writeFileSync(`${prefix}.json`, JSON.stringify(summary, null, 2));
writeFileSync(`${prefix}.ndjson`, rows.map(row => JSON.stringify(row)).join('\n') + '\n');
console.log(JSON.stringify(summary));
c.dispose();
