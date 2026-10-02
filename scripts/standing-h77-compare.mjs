import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { register } from 'tsx/esm/api';

const output = resolve(process.argv[2]);
assert.ok(!existsSync(output), `Use a fresh comparison directory: ${output}`);
mkdirSync(output, { recursive: true });
const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { ImplicitStandingController, H75_MOTOR_ROUNDING_TOLERANCE_NM } = await import('../src/character/ImplicitStandingController.ts');
const { STANDING_FEET } = await import('../src/character/CoordinatedStandingController.ts');
const { coordinatedStandingOptions } = await import('../src/character/standing-selection.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const math = await import('../src/character/math.ts');
const coordinatesModule = await import('../src/character/joint-coordinates.ts');

const length = vector => Math.hypot(vector.x, vector.y, vector.z);
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

function preStepComparison(index) {
  const failure = JSON.parse(readFileSync(resolve(`evidence/standing-h75-v1/attempt-01/reference-${index}/first-failure.json`)));
  const poses = new Map(failure.controllerInput.poses);
  const gravity = new Map(failure.controllerInput.gravity);
  const reference = new Map(failure.controllerBefore.reference);
  const scenario = failure.scenario;
  const h77 = new ImplicitStandingController(poses, scenario.heading, scenario.offset, () => 0);
  h77.reference = reference;
  const input = { ...failure.controllerInput, poses, gravity, quiet: true,
    grabActive: false, stepActive: false, touchdownQualified: false,
    supportMarginM: failure.snapshot.diagnostics.balance?.supportMarginM ?? 0 };
  const h77Commands = h77.update(input);
  assert.ok(h77Commands && h77Commands.length === 24, `H77 rejected preserved state ${index}`);
  const h74ById = new Map(failure.commands.map(command => [command.id, command]));
  let maximumRequestDifferenceNm = 0;
  let maximumH77RequestNm = 0;
  let maximumH77ResidualNm = 0;
  let axes = 0;
  const rows = [];
  for (const command of h77Commands) {
    const h74 = h74ById.get(command.id);
    const definition = SEGMENT_BY_ID.get(command.id);
    const parent = poses.get(definition.parent);
    const child = poses.get(command.id);
    const basis = coordinatesModule.jointFrameAxesWorld(parent.rotation, definition.jointProfile);
    const relativeVelocity = math.sub(child.angularVelocity, parent.angularVelocity);
    for (const axis of definition.jointProfile.axes) {
      const coordinate = axis.coordinate;
      const rate = math.dot(relativeVelocity, basis[coordinate]);
      const h74Request = math.dot(h74.feedforwardWorld, basis[coordinate]) - h74.damping * rate;
      const h77Request = command.standingRequestByAxis[coordinate];
      const residual = command.standingResidualByAxis[coordinate];
      const difference = Math.abs(h77Request - h74Request);
      maximumRequestDifferenceNm = Math.max(maximumRequestDifferenceNm, difference);
      maximumH77RequestNm = Math.max(maximumH77RequestNm, Math.abs(h77Request));
      maximumH77ResidualNm = Math.max(maximumH77ResidualNm, Math.abs(residual));
      axes++;
      rows.push({ segment: command.id, coordinate, h74Request, h77Request, difference,
        residual, capNm: axis.maxMotorTorqueNm * command.strengthScale,
        kp: command.stiffness, kd: command.damping });
    }
  }
  return { reference: index, axes, maximumRequestDifferenceNm,
    toleranceNm: H75_MOTOR_ROUNDING_TOLERANCE_NM,
    equivalent: maximumRequestDifferenceNm <= H75_MOTOR_ROUNDING_TOLERANCE_NM,
    maximumH77RequestNm, maximumH77ResidualNm,
    nonzeroNativeKp: rows.every(row => row.kp > 0),
    unchangedCeilings: rows.every(row => Math.abs(row.h77Request) <= row.capNm + 1e-12), rows };
}

async function referenceTrial(controllerId, scenario) {
  const character = await createEmbodiedCharacter('canvas2d', {
    heading: scenario.heading,
    ...coordinatedStandingOptions(controllerId, scenario.offset),
  });
  const reference = character.coordinatedStanding.reference;
  const result = { controllerId, reference: scenario.name, completedTicks: 0,
    firstTransition: null, maximumPelvisErrorM: 0, maximumFootErrorM: 0,
    maximumLinearMps: 0, maximumAngularRadps: 0, stepCount: 0, finalState: null,
    peakAngular: null, plantedAtEnd: [] };
  try {
    for (let tick = 1; tick <= scenario.steps; tick++) {
      character.fixedUpdate(1 / 60, null);
      const snapshot = character.getSnapshot('canvas2d');
      result.completedTicks = tick;
      result.stepCount = snapshot.diagnostics.stepCount;
      result.finalState = snapshot.state;
      const telemetry = snapshot.diagnostics.coordinatedStanding;
      if (!result.firstTransition && (telemetry?.authority === 'transition' || telemetry?.mode === 'transition')) {
        result.firstTransition = { tick, reason: telemetry.reason };
      }
      if (tick >= 120) {
        const byId = new Map(snapshot.segments.map(pose => [pose.id, pose]));
        result.maximumPelvisErrorM = Math.max(result.maximumPelvisErrorM,
          distance(byId.get('pelvis').position, reference.get('pelvis').position));
        result.maximumFootErrorM = Math.max(result.maximumFootErrorM,
          ...STANDING_FEET.map(id => distance(byId.get(id).position, reference.get(id).position)));
        const maximumLinear = Math.max(...snapshot.segments.map(pose => length(pose.linearVelocity)));
        const maximumAngular = Math.max(...snapshot.segments.map(pose => length(pose.angularVelocity)));
        result.maximumLinearMps = Math.max(result.maximumLinearMps, maximumLinear);
        if (maximumAngular > result.maximumAngularRadps) {
          result.maximumAngularRadps = maximumAngular;
          const segment = snapshot.segments.find(pose => length(pose.angularVelocity) === maximumAngular);
          result.peakAngular = { tick, segment: segment.id, angularRadps: maximumAngular,
            planted: snapshot.support.planted,
            contactPlan: snapshot.diagnostics.contactDiagnostics?.standingPlan ?? null,
            contactSegments: snapshot.diagnostics.contactDiagnostics?.contacts
              .filter(contact => contact.loadBearing).map(contact => ({ segment: contact.segment,
                forceN: contact.forceN, persistenceS: contact.persistenceS })) ?? [] };
        }
      }
      if (['falling', 'fallen', 'recovering'].includes(snapshot.state)) break;
    }
    result.plantedAtEnd = character.getSnapshot('canvas2d').support.planted;
  } finally {
    character.dispose();
  }
  return result;
}

const preStep = [0, 1, 2].map(preStepComparison);
const trials = [];
for (const index of [0, 1, 2]) {
  const scenario = JSON.parse(readFileSync(resolve(`evidence/standing-h75-v1/attempt-01/reference-${index}/scenario.json`)));
  trials.push(await referenceTrial('h74-v1', scenario));
  trials.push(await referenceTrial('h77-v1', scenario));
}
const comparisons = [0, 1, 2].map(index => {
  const h74 = trials.find(trial => trial.reference === `reference-${index}` && trial.controllerId === 'h74-v1');
  const h77 = trials.find(trial => trial.reference === `reference-${index}` && trial.controllerId === 'h77-v1');
  const metrics = ['maximumPelvisErrorM', 'maximumFootErrorM', 'maximumLinearMps', 'maximumAngularRadps'];
  return { reference: index, improvesEveryMetric: metrics.every(metric => h77[metric] <= h74[metric]),
    metrics: Object.fromEntries(metrics.map(metric => [metric, { h74: h74[metric], h77: h77[metric],
      improved: h77[metric] <= h74[metric] }])) };
});
const report = { schema: 1, id: 'standing-h77-h74-comparison', preStep, trials, comparisons,
  h77ImprovesEveryReference: comparisons.every(comparison => comparison.improvesEveryMetric),
  qualification: 'Application-level comparison using the preserved H75 controller inputs and original reference scenarios. It is not standing acceptance.' };
writeFileSync(join(output, 'report.json'), `${JSON.stringify(report, null, 2)}\n`, { flag: 'wx' });
console.log(JSON.stringify({ output, equivalent: preStep.map(item => item.equivalent),
  trials: trials.map(({ controllerId, reference, completedTicks, firstTransition,
    maximumLinearMps, maximumAngularRadps }) => ({ controllerId, reference, completedTicks,
    firstTransition, maximumLinearMps, maximumAngularRadps })),
  improves: comparisons.map(item => item.improvesEveryMetric) }));
unregister();
