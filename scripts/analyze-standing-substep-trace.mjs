import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const directory = resolve(process.argv[2]);
const output = resolve(process.argv[3] ?? join(directory, 'report.json'));
assert.ok(!existsSync(output), `Refusing to overwrite ${output}`);

function rawIndex(handle) {
  const buffer = new ArrayBuffer(8);
  const view = new DataView(buffer);
  view.setFloat64(0, handle, true);
  return view.getUint32(0, true);
}

const number = value => {
  const result = Number(value);
  assert.ok(Number.isFinite(result), `Non-finite trace scalar: ${value}`);
  return result;
};

function parseEvent(line) {
  const field = line.split('\t');
  if (field[0] === 'body') return {
    type: 'body', phase: field[1], substep: number(field[2]), slot: number(field[3]),
    handle: number(field[4]),
    linear: field.slice(6, 9).map(number), angular: field.slice(9, 12).map(number),
  };
  if (field[0] === 'joint') return {
    type: 'joint', phase: field[1], substep: number(field[2]), row: number(field[3]),
    joint: number(field[4]), kind: field[5], parent: number(field[6]), child: number(field[8]),
    impulse: number(field[10]), bounds: field.slice(11, 13).map(number),
    rhs: number(field[13]), rhsWithoutBias: number(field[14]), inverseLhs: number(field[15]),
    cfmCoefficient: number(field[16]), cfmGain: number(field[17]),
  };
  assert.equal(field[0], 'contact');
  return {
    type: 'contact', phase: field[1], substep: number(field[2]), chunk: number(field[3]),
    point: number(field[4]), manifold: field[5], body1: number(field[8]), body2: number(field[10]),
    impulse: number(field[12]), totalImpulse: number(field[13]),
    tangentImpulse: field.slice(14, 16).map(number), tangentTotalImpulse: field.slice(16, 18).map(number),
    twistImpulse: number(field[18]), twistTotalImpulse: number(field[19]),
    rhs: number(field[20]), rhsWithoutBias: number(field[21]), inverseLhs: number(field[22]),
    normal: field.slice(23, 26).map(number),
  };
}

const norm = values => Math.hypot(...values);
function vectorDelta(a, b) { return norm(a.map((value, index) => value - b[index])); }

function bodyParity(actual, expected) {
  const expectedByHandle = new Map(expected.map(body => [body.handle.join(':'), body]));
  const maxima = { positionM: 0, linearMps: 0, angularRadps: 0, quaternionNorm: 0 };
  for (const body of actual) {
    const match = expectedByHandle.get(body.handle.join(':'));
    assert.ok(match, `Missing body ${body.handle.join(':')}`);
    maxima.positionM = Math.max(maxima.positionM, vectorDelta(body.translation, match.translation));
    maxima.linearMps = Math.max(maxima.linearMps, vectorDelta(body.linearVelocity, match.linearVelocity));
    maxima.angularRadps = Math.max(maxima.angularRadps, vectorDelta(body.angularVelocity, match.angularVelocity));
    maxima.quaternionNorm = Math.max(maxima.quaternionNorm, vectorDelta(body.rotation, match.rotation));
  }
  return maxima;
}

const references = [];
for (const index of [0, 1, 2]) {
  const trace = JSON.parse(readFileSync(join(directory, `reference-${index}.json`)));
  const postInput = JSON.parse(readFileSync(join(directory, `reference-${index}-post-input.json`)));
  const failure = JSON.parse(readFileSync(resolve(`evidence/standing-h75-v1/attempt-01/reference-${index}/first-failure.json`)));
  const segmentByHandle = new Map(failure.handles.map(([segment, handle]) => [rawIndex(handle), segment]));
  const events = trace.events.map(parseEvent);
  const joints = events.filter(event => event.type === 'joint' && event.phase === 'solved');
  const contacts = events.filter(event => event.type === 'contact');
  const solvedContacts = contacts.filter(event => event.phase === 'solved');
  const warmContacts = contacts.filter(event => event.phase === 'warmstarted');
  const bodies = events.filter(event => event.type === 'body');
  const substeps = [...new Set(joints.map(row => row.substep))].sort((a, b) => a - b);
  const lastSubstep = substeps.at(-1);
  const motors = joints.filter(row => row.kind.startsWith('Motor('));
  const utilization = motors.map(row => Math.abs(row.impulse)
    / Math.max(Math.abs(row.bounds[0]), Math.abs(row.bounds[1])));
  const finalContacts = solvedContacts.filter(contact => contact.substep === lastSubstep);
  const finalChunks = new Map();
  for (const contact of finalContacts) finalChunks.set(contact.chunk, contact);
  const contactSegments = [...new Set(solvedContacts.map(contact => {
    const handle = contact.body1 === 0xffffffff ? contact.body2 : contact.body1;
    return segmentByHandle.get(handle) ?? `body-${handle}`;
  }))];
  const firstWarm = warmContacts.filter(contact => contact.substep === 0);

  const velocityByKey = new Map(bodies.map(body => [`${body.phase}:${body.substep}:${body.handle}`, body]));
  const selectedBodyDeltas = {};
  for (const [handle, segment] of segmentByHandle) {
    if (!['pelvis', 'torso', 'leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'].includes(segment)) continue;
    selectedBodyDeltas[segment] = substeps.map(substep => {
      const pre = velocityByKey.get(`pre:${substep}:${handle}`);
      const solved = velocityByKey.get(`solved:${substep}:${handle}`);
      assert.ok(pre && solved, `Missing body trace ${segment} substep ${substep}`);
      return { substep, linearMps: vectorDelta(solved.linear, pre.linear),
        angularRadps: vectorDelta(solved.angular, pre.angular) };
    });
  }
  const rowsPerSubstep = substeps.map(substep => {
    const rows = joints.filter(row => row.substep === substep);
    return {
      substep,
      motor: rows.filter(row => row.kind.startsWith('Motor(')).length,
      limit: rows.filter(row => row.kind.startsWith('Limit(')).length,
      lock: rows.filter(row => row.kind.startsWith('Dof(')).length,
    };
  });
  assert.ok(rowsPerSubstep.every(count => count.motor === 46 && count.limit === 46 && count.lock === 98));
  references.push({
    reference: index,
    outputNeutral: trace.outputNeutral,
    nativeStateBytes: { control: trace.controlBytes, traced: trace.tracedBytes },
    nativeToPreservedWasmPost: bodyParity(trace.afterBodies, postInput.beforeBodies),
    substeps: substeps.length,
    substepDtS: trace.integrationParameters.dt / substeps.length,
    jointRows: {
      perSubstep: rowsPerSubstep[0], total: joints.length,
      motorAtBound: utilization.filter(value => value >= 0.999999).length,
      maximumMotorBoundUtilization: Math.max(...utilization),
      finite: joints.every(row => [row.impulse, ...row.bounds, row.rhs, row.rhsWithoutBias,
        row.inverseLhs, row.cfmCoefficient, row.cfmGain].every(Number.isFinite)),
    },
    contacts: {
      segments: contactSegments,
      rows: solvedContacts.length,
      firstSubstepWarmstartNormalImpulseNs: firstWarm.reduce((sum, contact) => sum + contact.impulse, 0),
      finalNormalImpulseNs: finalContacts.reduce((sum, contact) => sum + contact.totalImpulse, 0),
      finalFrictionImpulseNsByChunk: [...finalChunks.values()].map(contact => ({
        chunk: contact.chunk,
        tangent: contact.tangentTotalImpulse,
        magnitude: norm(contact.tangentTotalImpulse),
        twist: contact.twistTotalImpulse,
      })),
      finite: solvedContacts.every(contact => [contact.impulse, contact.totalImpulse,
        ...contact.tangentImpulse, ...contact.tangentTotalImpulse, contact.twistImpulse,
        contact.twistTotalImpulse, contact.rhs, contact.rhsWithoutBias, contact.inverseLhs,
        ...contact.normal].every(Number.isFinite)),
    },
    bodyVelocityDeltas: selectedBodyDeltas,
  });
}

assert.ok(references.every(reference => reference.outputNeutral));
const report = {
  schema: 1,
  id: 'standing-h77-native-substep-trace-03',
  references,
  conclusion: {
    requestedImpulse: 'Motor rows are finite, retain all 46 motor rows plus 46 limit and 98 lock rows per actual substep, and do not reach their impulse bounds in these three states.',
    contactImpulse: 'The solved trace contains real sole normal, tangent-friction, and twist impulses plus pre/post solver-body velocities for every actual substep.',
    branch: 'correct-joint-rows/contact-wrench',
    qualification: 'The trace is bit-identical to a control native step from the same snapshot. Native-to-preserved-WASM comparison is reported separately and is close but not bit-identical, so this is directional branch evidence, not a replacement standing acceptance result.',
  },
};
writeFileSync(output, `${JSON.stringify(report, null, 2)}\n`, { flag: 'wx' });
console.log(JSON.stringify({ output, references: references.map(reference => ({
  reference: reference.reference, outputNeutral: reference.outputNeutral,
  substeps: reference.substeps, maxMotorUtilization: reference.jointRows.maximumMotorBoundUtilization,
  contacts: reference.contacts.segments, wasmParity: reference.nativeToPreservedWasmPost,
})) }));
