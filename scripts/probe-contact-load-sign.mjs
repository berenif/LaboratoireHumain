import { mkdirSync, writeFileSync } from 'node:fs';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { TOTAL_MASS_KG, SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { add, scale } from '../src/character/math.ts';

const dt = 1 / 60, results = [];
for (const forceZ of [0, 100, -100]) {
  const c = await createEmbodiedCharacter('canvas2d');
  for (let frame = 0; frame < 60; frame++) c.fixedUpdate(dt, null);
  const commands = c.motorCommands.bind(c);
  c.motorCommands = targets => {
    // Diagnostic-only fixed horizontal load on the existing measured supports.
    const total = c.contactLoadPlan.loads.reduce((sum, load) => sum + load.plannedForce.y, 0);
    for (const load of c.contactLoadPlan.loads) load.plannedForce = {
      ...load.plannedForce, z: total > 0 ? forceZ * load.plannedForce.y / total : 0,
    };
    return commands(targets);
  };
  for (const body of c.ragdollBodies.values()) body.wakeUp();
  const frames = [];
  for (let frame = 0; frame < 30; frame++) {
    c.fixedUpdate(dt, null);
    const snapshot = c.getSnapshot('canvas2d');
    const massVelocity = scale(snapshot.segments.reduce((sum, pose) =>
      add(sum, scale(pose.linearVelocity, SEGMENT_BY_ID.get(pose.id).massKg)), { x: 0, y: 0, z: 0 }), 1 / TOTAL_MASS_KG);
    frames.push({ frame, state: snapshot.state, massVelocity,
      pelvis: snapshot.segments.find(pose => pose.id === 'pelvis'),
      loads: structuredClone(c.contactLoadPlan.loads),
      joints: snapshot.diagnostics.jointDiagnostics.filter(joint => joint.segment.startsWith('left') &&
        ['leftThigh', 'leftShin', 'leftAnkle', 'leftFoot'].includes(joint.segment)) });
  }
  results.push({ forceZ, frames });
  console.log(JSON.stringify({ forceZ, first: frames[0].massVelocity, fifth: frames[4].massVelocity,
    thirtieth: frames[29].massVelocity, finalState: frames[29].state }));
  c.dispose();
}
mkdirSync(new URL('../evidence/collision/', import.meta.url), { recursive: true });
writeFileSync('evidence/collision/contact-load-sign.json', JSON.stringify({ dt, results }, null, 2));
