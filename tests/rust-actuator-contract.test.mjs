import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { validateActuatorReports } from '../scripts/rust-actuator-contract.mjs';
const model = JSON.parse(readFileSync(new URL('../rust/crates/model/data/getup-v1.json', import.meta.url)));
const floatFrame = frame => Object.fromEntries(Object.entries(frame).map(([part, values]) =>
  [part, Object.fromEntries(Object.entries(values).map(([axis, value]) => [axis, Math.fround(value)]))]));
function fixture() {
  const rows = [];
  for (const segment of model.segments.filter(s => s.parent != null)) for (let headingIndex = 0; headingIndex < 3; headingIndex++) {
    for (const posture of ['neutral', 'near-min', 'near-max']) for (const sign of [-1, 1]) {
      const hinge = segment.reference?.hinge, active = segment.jointProfile.axes.length > 0;
      rows.push({identity:{caseId:`${segment.id}:${headingIndex}:${posture}:${sign}`, segmentId:segment.id,
        parentId:segment.parent, fixtureKind:active ? 'active-hinge' : 'fixed-attachment',
        hingeName:hinge?.name ?? null, qposIndex:hinge?.qposIndex ?? null, actuatorIndex:hinge?.actuatorIndex ?? null,
        canonicalMassKg:Math.fround(segment.massKg), masslessCoordinateFrame:segment.role === 'coordinate' && segment.massKg === 0,
        canonicalAxisCount:segment.jointProfile.axes.length, canonicalCoordinate:segment.jointProfile.axes[0]?.coordinate ?? null, headingIndex, posture, sign},
        segment:segment.id, posture, sign, topology:'synthetic impulse-joint pair',
        productionMultibodyTorqueReadbackCertified:false, loadedChainCertified:false, passed:true, postureApplicable:active,
        ...(!active ? {canonicalParentFrame:floatFrame(segment.jointProfile.parentFrame), canonicalChildFrame:floatFrame(segment.jointProfile.childFrame)} : {}),
        samples:Array.from({length:4}, (_, substep) => ({substep, passed:true,
          ...(active ? {axisPassed:true, numericalToleranceNmS:1e-6, readbackErrorNmS:0, bodyMomentumResidualNmS:0,
            oppositeResidualNmS:0, vectorImpulseNmS:0.01, vectorBoundNmS:0.02,
            parentImpulseNmS:[0.01,0,0], childImpulseNmS:[-0.01,0,0]} : {
            zeroMotorPassed:true, constraintPassed:true, challengePassed:true,
            appliedLinearChallengeImpulseNs:[25,25], appliedAngularChallengeImpulseNmS:[10,10], motorAxesBits:0, lockedAxesBits:63, axisCapsNm:[0,0,0],
            motorMaxForces:Array(6).fill(0), motorDeliveredImpulse:Array(6).fill(0), motorDeliveredAbsImpulse:Array(6).fill(0),
            parentImpulseNmS:[0,0,0], childImpulseNmS:[0,0,0], vectorImpulseNmS:0, vectorBoundNmS:0, motorWorkJ:0,
            externalParentForceN:[-20,0,0], externalChildForceN:[20,0,0], externalParentTorqueNm:[0,-8,0], externalChildTorqueNm:[0,8,0],
            constraintLinearReactionNs:1, constraintAngularReactionNmS:1, lastSolverLinearRowImpulseNs:0, lastSolverAngularRowImpulseNmS:0,
            anchorThresholdM:Math.fround(0.08), lockedAngularThresholdRad:Math.fround(0.06), anchorErrorM:0.001, lockedAngularErrorRad:0.001})}))});
    }
  }
  return rows;
}
test('all 432 unique model identities include 378 active, 54 fixed and 162 massless active cases', () => {
  const summary = validateActuatorReports(fixture(), model);
  assert.equal(summary.activeHingeCases, 378);
  assert.equal(summary.fixedAttachmentCases, 54);
  assert.equal(summary.masslessActiveCategories, 9);
  assert.equal(summary.samples, 1728);
});
for (const [name, mutate] of [
  ['duplicate identity', rows => { rows[1] = structuredClone(rows[0]); }],
  ['empty samples', rows => { rows[0].samples = []; }],
  ['missing massless active cases', rows => rows.splice(18,162)],
  ['fixed mislabeled as active', rows => { rows[0].identity.fixtureKind = 'active-hinge'; }],
  ['zero active delivery', rows => { rows.find(r => r.postureApplicable).samples[0].vectorImpulseNmS = 0; }],
  ['active readback failure hidden behind passed', rows => { rows.find(r => r.postureApplicable).samples[0].readbackErrorNmS = 1e-5; }],
  ['fixed motor leakage', rows => { rows[0].samples[0].motorDeliveredImpulse[0] = 1; }],
  ['fixed motor work', rows => { rows[0].samples[0].motorWorkJ = 1; }],
  ['missing differential challenge', rows => { rows[0].samples[0].externalChildForceN = [0,0,0]; }],
  ['ineffective first challenge', rows => { rows[0].samples[0].appliedLinearChallengeImpulseNs = [0,0]; }],
  ['vacuous constraint reaction', rows => { rows[0].samples[0].constraintLinearReactionNs = 0; }],
  ['nonfinite final row cache', rows => { rows[0].samples[0].lastSolverAngularRowImpulseNmS = NaN; }],
  ['unlocked attachment', rows => { rows[0].samples[0].lockedAxesBits = 0; }],
  ['wrong anatomical anchor', rows => { rows[0].canonicalParentFrame.anchor.y += 0.01; }],
  ['widened bounds', rows => { rows[0].samples[0].anchorThresholdM = 0.1; }],
  ['out-of-bound locked angle', rows => { rows[0].samples[0].lockedAngularErrorRad = 0.061; }],
  ['production multibody certification', rows => { rows[0].productionMultibodyTorqueReadbackCertified = true; }],
]) test(`release rejects ${name}`, () => { const rows = fixture(); mutate(rows); assert.throws(() => validateActuatorReports(rows, model)); });
test('release rejects empty diagnostics', () => assert.throws(() => validateActuatorReports([], model)));
