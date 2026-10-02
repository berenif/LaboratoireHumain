import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const [h77Arg, h74Arg, outputArg] = process.argv.slice(2);
assert.ok(h77Arg && h74Arg && outputArg,
  'analyze-standing-live-substep.mjs h77-capture-directory h74-capture-directory output.json');
const output = resolve(outputArg);
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
const norm = values => Math.hypot(...values);
const vectorDelta = (a, b) => norm(a.map((value, index) => value - b[index]));

function parseEvent(line) {
  const field = line.split('\t');
  if (field[0] === 'body') return {
    type: 'body', phase: field[1], substep: number(field[2]), handle: number(field[4]),
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
    body1: number(field[8]), body2: number(field[10]), impulse: number(field[12]),
    totalImpulse: number(field[13]), tangentTotalImpulse: field.slice(16, 18).map(number),
    twistTotalImpulse: number(field[19]), rhs: number(field[20]),
    rhsWithoutBias: number(field[21]), inverseLhs: number(field[22]),
  };
}

function analyze(directoryArg) {
  const directory = resolve(directoryArg);
  const capture = JSON.parse(readFileSync(join(directory, 'capture.json')));
  const trace = JSON.parse(readFileSync(join(directory, 'native-trace.json')));
  const segmentByHandle = new Map(capture.handles.map(([segment, handle]) => [rawIndex(handle), segment]));
  const segmentByJoint = new Map((capture.jointHandles ?? [])
    .map(([segment, handle]) => [rawIndex(handle), segment]));
  const events = trace.events.map(parseEvent);
  const joints = events.filter(event => event.type === 'joint' && event.phase === 'solved');
  const contacts = events.filter(event => event.type === 'contact' && event.phase === 'solved');
  const bodies = events.filter(event => event.type === 'body');
  const substeps = [...new Set(joints.map(row => row.substep))].sort((a, b) => a - b);
  const rowsPerSubstep = substeps.map(substep => {
    const rows = joints.filter(row => row.substep === substep);
    return { substep,
      motor: rows.filter(row => row.kind.startsWith('Motor(')).length,
      limit: rows.filter(row => row.kind.startsWith('Limit(')).length,
      lock: rows.filter(row => row.kind.startsWith('Dof(')).length };
  });
  assert.ok(rowsPerSubstep.every(count => count.motor === 46 && count.limit === 46 && count.lock === 98));
  const motors = joints.filter(row => row.kind.startsWith('Motor('));
  const motorRows = new Map();
  for (const row of motors) {
    const key = `${row.joint}:${row.kind}:${row.row}`;
    const values = motorRows.get(key) ?? [];
    values.push(row);
    motorRows.set(key, values);
  }
  const response = [...motorRows.values()].map(values => ({
    joint: values[0].joint,
    childSegment: segmentByJoint.get(values[0].joint) ?? null,
    kind: values[0].kind,
    firstRhs: values[0].rhs,
    lastRhs: values.at(-1).rhs,
    range: Math.max(...values.map(value => value.rhs)) - Math.min(...values.map(value => value.rhs)),
  }));
  const utilization = motors.map(row => Math.abs(row.impulse)
    / Math.max(Math.abs(row.bounds[0]), Math.abs(row.bounds[1])));
  const velocityByKey = new Map(bodies.map(body => [`${body.phase}:${body.substep}:${body.handle}`, body]));
  const bodyVelocityDeltas = {};
  for (const [handle, segment] of segmentByHandle) {
    if (!['pelvis', 'torso', 'leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'].includes(segment)) continue;
    bodyVelocityDeltas[segment] = substeps.map(substep => {
      const pre = velocityByKey.get(`pre:${substep}:${handle}`);
      const solved = velocityByKey.get(`solved:${substep}:${handle}`);
      assert.ok(pre && solved, `Missing ${segment} body trace at substep ${substep}`);
      return { substep, linearMps: vectorDelta(solved.linear, pre.linear),
        angularRadps: vectorDelta(solved.angular, pre.angular) };
    });
  }
  const finalSubstep = substeps.at(-1);
  const finalContacts = contacts.filter(contact => contact.substep === finalSubstep);
  const finalChunks = new Map(finalContacts.map(contact => [contact.chunk, contact]));
  const contactSegments = [...new Set(contacts.map(contact => {
    const handle = contact.body1 === 0xffffffff ? contact.body2 : contact.body1;
    return segmentByHandle.get(handle) ?? `body-${handle}`;
  }))];
  const expectedPost = capture.postBodies;
  const nativePostBySegment = Object.fromEntries(trace.afterBodies.map(body => [
    segmentByHandle.get(body.handle[0]), body,
  ]));
  const nativeToLiveWasm = { positionM: 0, linearMps: 0, angularRadps: 0, quaternionNorm: 0 };
  for (const [segment, expected] of Object.entries(expectedPost)) {
    const actual = nativePostBySegment[segment];
    assert.ok(actual, `Missing native body ${segment}`);
    nativeToLiveWasm.positionM = Math.max(nativeToLiveWasm.positionM,
      vectorDelta(Object.values(expected.position), actual.translation));
    nativeToLiveWasm.linearMps = Math.max(nativeToLiveWasm.linearMps,
      vectorDelta(Object.values(expected.velocity), actual.linearVelocity));
    nativeToLiveWasm.angularRadps = Math.max(nativeToLiveWasm.angularRadps,
      vectorDelta(Object.values(expected.angularVelocity), actual.angularVelocity));
    nativeToLiveWasm.quaternionNorm = Math.max(nativeToLiveWasm.quaternionNorm,
      vectorDelta(Object.values(expected.rotation), actual.rotation));
  }
  const nativeAxes = capture.nativeMotorResults.flatMap(([, result]) => result.nativeMotorAxes ?? []);
  return {
    controllerId: capture.controllerId,
    reference: capture.reference,
    tick: capture.tick,
    outputNeutral: trace.outputNeutral,
    nativeStateBytes: { control: trace.controlBytes, traced: trace.tracedBytes },
    nativeToLiveWasm,
    substeps: substeps.length,
    substepDtS: trace.integrationParameters.dt / substeps.length,
    jointRows: {
      perSubstep: rowsPerSubstep[0],
      finite: joints.every(row => [row.impulse, ...row.bounds, row.rhs, row.rhsWithoutBias,
        row.inverseLhs, row.cfmCoefficient, row.cfmGain].every(Number.isFinite)),
      maximumMotorBoundUtilization: Math.max(...utilization),
      motorAtBound: utilization.filter(value => value >= 0.999999).length,
      rhsRespondingRows: response.filter(row => row.range > 1e-7).length,
      maximumRhsRange: Math.max(...response.map(row => row.range)),
      response,
    },
    configuredNativeAxes: {
      count: nativeAxes.length,
      nonzeroKp: nativeAxes.filter(axis => axis.stiffness > 0).length,
      maximumCurrentRequestErrorNm: Math.max(...nativeAxes.map(axis =>
        Math.abs(axis.currentStateRequestNm - axis.intendedRequestNm))),
    },
    contacts: {
      segments: contactSegments,
      rows: contacts.length,
      finalNormalImpulseNs: finalContacts.reduce((sum, contact) => sum + contact.totalImpulse, 0),
      finalFrictionImpulseNsByChunk: [...finalChunks.values()].map(contact => ({
        chunk: contact.chunk,
        tangent: contact.tangentTotalImpulse,
        magnitude: norm(contact.tangentTotalImpulse),
        twist: contact.twistTotalImpulse,
      })),
    },
    bodyVelocityDeltas,
  };
}

const h77 = analyze(h77Arg);
const h74 = analyze(h74Arg);
assert.equal(h77.controllerId, 'h77-v1');
assert.equal(h74.controllerId, 'h74-v1');
assert.ok(h77.outputNeutral && h74.outputNeutral);
assert.equal(h77.configuredNativeAxes.count, 46);
assert.equal(h77.configuredNativeAxes.nonzeroKp, 46);
assert.ok(h77.jointRows.rhsRespondingRows > 0, 'H77 motor RHS did not respond across native substeps');
const report = {
  schema: 1,
  id: 'standing-h77-live-native-substep',
  h77,
  h74,
  conclusion: {
    nativePdResponse: `${h77.jointRows.rhsRespondingRows} of 46 H77 motor rows changed RHS across the actual substeps with nonzero kp on all axes.`,
    requestedImpulse: h77.jointRows.motorAtBound === 0
      ? 'All H77 joint rows are finite and no motor row reached its impulse bound.'
      : `${h77.jointRows.motorAtBound} H77 motor rows reached their impulse bound.`,
    branch: h77.jointRows.motorAtBound === 0 ? 'correct-joint-rows/contact-wrench' : 'controller-projection-allocation',
    qualification: 'Output-neutral native diagnostic from the live H77 peak state. Native-to-WASM parity is quantified separately; this is not standing acceptance.',
  },
};
writeFileSync(output, `${JSON.stringify(report, null, 2)}\n`, { flag: 'wx' });
console.log(JSON.stringify({ output, h77: {
  substeps: h77.substeps,
  motorAtBound: h77.jointRows.motorAtBound,
  maximumMotorBoundUtilization: h77.jointRows.maximumMotorBoundUtilization,
  rhsRespondingRows: h77.jointRows.rhsRespondingRows,
  contacts: h77.contacts.segments,
  nativeToLiveWasm: h77.nativeToLiveWasm,
}, h74: {
  maximumMotorBoundUtilization: h74.jointRows.maximumMotorBoundUtilization,
  contacts: h74.contacts.segments,
}}));
