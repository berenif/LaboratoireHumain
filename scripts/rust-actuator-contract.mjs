// Independent release-side checks for all canonical actuator/attachment identities.
import assert from 'node:assert/strict';

const postures = ['neutral', 'near-min', 'near-max'];
const tolerance = 1e-6;
const finite = value => typeof value === 'number' && Number.isFinite(value);
const positive = value => finite(value) && value > 0;
const vector = values => Array.isArray(values) && values.length === 3 && values.every(finite);
const zeros = (values, count) => Array.isArray(values) && values.length === count && values.every(value => value === 0);
const magnitude = values => Math.hypot(...values);
const opposed = (a, b) => vector(a) && vector(b) && magnitude(a.map((v, k) => v + b[k])) <= tolerance;
const floatFrame = frame => Object.fromEntries(Object.entries(frame).map(([part, values]) =>
  [part, Object.fromEntries(Object.entries(values).map(([axis, value]) => [axis, Math.fround(value)]))]));

export function validateActuatorReports(cases, model) {
  const categories = model.segments.filter(segment => segment.parent != null);
  const active = categories.filter(segment => segment.jointProfile?.axes.length === 1 && segment.reference?.hinge);
  const fixed = categories.filter(segment => segment.jointProfile?.axes.length === 0 && !segment.reference?.hinge);
  const massless = active.filter(segment => segment.role === 'coordinate' && segment.massKg === 0);
  assert.equal(categories.length, 24);
  assert.equal(active.length, 21);
  assert.equal(fixed.length, 3);
  assert.equal(massless.length, 9);
  assert.deepEqual(fixed.map(s => s.id).sort(), ['head', 'left_hand', 'right_hand']);
  const expected = new Map();
  for (const segment of categories) for (let headingIndex = 0; headingIndex < 3; headingIndex++) {
    for (const posture of postures) for (const sign of [-1, 1]) {
      const caseId = `${segment.id}:${headingIndex}:${posture}:${sign}`;
      expected.set(caseId, { segment, identity:{caseId, segmentId:segment.id, parentId:segment.parent,
        fixtureKind:segment.jointProfile.axes.length ? 'active-hinge' : 'fixed-attachment',
        hingeName:segment.reference?.hinge?.name ?? null, qposIndex:segment.reference?.hinge?.qposIndex ?? null,
        actuatorIndex:segment.reference?.hinge?.actuatorIndex ?? null, canonicalMassKg:Math.fround(segment.massKg),
        masslessCoordinateFrame:segment.role === 'coordinate' && segment.massKg === 0,
        canonicalAxisCount:segment.jointProfile.axes.length, canonicalCoordinate:segment.jointProfile.axes[0]?.coordinate ?? null, headingIndex, posture, sign} });
    }
  }
  assert.ok(Array.isArray(cases));
  assert.equal(cases.length, 432);
  for (const result of cases) {
    const key = result.identity?.caseId;
    const wanted = expected.get(key);
    assert.ok(wanted, `Duplicate or unknown actuator identity ${key}`);
    expected.delete(key);
    assert.deepEqual(result.identity, wanted.identity, `Mismatched actuator identity ${key}`);
    assert.equal(result.segment, wanted.segment.id);
    assert.equal(result.posture, wanted.identity.posture);
    assert.equal(result.sign, wanted.identity.sign);
    assert.equal(result.topology, 'synthetic impulse-joint pair');
    assert.equal(result.productionMultibodyTorqueReadbackCertified, false);
    assert.equal(result.loadedChainCertified, false);
    assert.equal(result.passed, true, `Failed actuator case ${key}`);
    assert.ok(Array.isArray(result.samples));
    assert.equal(result.samples.length, 4, `Incomplete actuator samples ${key}`);
    const isActive = wanted.identity.fixtureKind === 'active-hinge';
    assert.equal(result.postureApplicable, isActive);
    if (!isActive) {
      assert.deepEqual(result.canonicalParentFrame, floatFrame(wanted.segment.jointProfile.parentFrame));
      assert.deepEqual(result.canonicalChildFrame, floatFrame(wanted.segment.jointProfile.childFrame));
    }
    result.samples.forEach((sample, substep) => {
      assert.equal(sample.substep, substep);
      assert.equal(sample.passed, true, `Failed actuator sample ${key}/${substep}`);
      assert.ok(vector(sample.parentImpulseNmS) && vector(sample.childImpulseNmS));
      if (isActive) {
        assert.equal(sample.axisPassed, true);
        assert.equal(sample.numericalToleranceNmS, tolerance);
        for (const field of ['readbackErrorNmS', 'bodyMomentumResidualNmS', 'oppositeResidualNmS']) {
          assert.ok(finite(sample[field]) && sample[field] >= 0 && sample[field] <= tolerance, `${key}: ${field}`);
        }
        assert.ok(positive(sample.vectorImpulseNmS) && positive(sample.vectorBoundNmS));
        assert.ok(sample.vectorImpulseNmS <= sample.vectorBoundNmS + tolerance);
        assert.ok(opposed(sample.parentImpulseNmS, sample.childImpulseNmS));
        assert.ok(Math.abs(Math.max(magnitude(sample.parentImpulseNmS), magnitude(sample.childImpulseNmS))
          - sample.vectorImpulseNmS) <= tolerance);
      } else {
        assert.equal(sample.zeroMotorPassed, true);
        assert.equal(sample.constraintPassed, true);
        assert.equal(sample.challengePassed, true);
        for (const field of ['appliedLinearChallengeImpulseNs', 'appliedAngularChallengeImpulseNmS']) {
          assert.ok(Array.isArray(sample[field]) && sample[field].length === 2 && sample[field].every(positive), `${key}: ineffective challenge`);
        }
        assert.equal(sample.motorAxesBits, 0);
        assert.equal(sample.lockedAxesBits, 63);
        for (const [field, count] of [['axisCapsNm',3], ['motorMaxForces',6], ['motorDeliveredImpulse',6],
          ['motorDeliveredAbsImpulse',6], ['parentImpulseNmS',3], ['childImpulseNmS',3]]) {
          assert.ok(zeros(sample[field], count), `${key}: nonzero or missing ${field}`);
        }
        assert.equal(sample.vectorImpulseNmS, 0);
        assert.equal(sample.vectorBoundNmS, 0);
        assert.ok(sample.motorWorkJ === 0);
        for (const [parent, child] of [['externalParentForceN','externalChildForceN'], ['externalParentTorqueNm','externalChildTorqueNm']]) {
          assert.ok(opposed(sample[parent], sample[child]) && positive(magnitude(sample[child])), `${key}: missing differential challenge`);
        }
        for (const field of ['constraintLinearReactionNs', 'constraintAngularReactionNmS']) assert.ok(positive(sample[field]), `${key}: ${field}`);
        for (const field of ['lastSolverLinearRowImpulseNs', 'lastSolverAngularRowImpulseNmS']) {
          assert.ok(finite(sample[field]) && sample[field] >= 0, `${key}: invalid final row cache`);
        }
        assert.equal(sample.anchorThresholdM, Math.fround(0.08));
        assert.equal(sample.lockedAngularThresholdRad, Math.fround(0.06));
        assert.ok(finite(sample.anchorErrorM) && sample.anchorErrorM >= 0 && sample.anchorErrorM <= Math.fround(0.08));
        assert.ok(finite(sample.lockedAngularErrorRad) && sample.lockedAngularErrorRad >= 0 && sample.lockedAngularErrorRad <= Math.fround(0.06));
      }
    });
  }
  assert.equal(expected.size, 0);
  return {categories:24, activeHingeCategories:21, fixedAttachmentCategories:3, masslessActiveCategories:9,
    cases:432, activeHingeCases:378, fixedAttachmentCases:54, samples:1728,
    productionMultibodyTorqueReadbackCertified:false};
}
