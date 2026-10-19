"""Focused pure tests. No producer/helper/native or optimizer imports."""
import copy
import math
import unittest

import numpy as np
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

import audit_source_posture_references as audit


def jet(position, velocity=None, acceleration=None):
    p = np.asarray(position, dtype=float)
    return dict(position=p.tolist(), velocity=(np.zeros_like(p) if velocity is None else np.asarray(velocity)).tolist(),
        acceleration=(np.zeros_like(p) if acceleration is None else np.asarray(acceleration)).tolist())


def orientation(body, angles=(0., 0., 0.)):
    return dict(body=body, rotation=Rotation.from_euler('ZYX', angles).as_matrix().reshape(-1).tolist(),
        velocity=[0.]*3, acceleration=[0.]*3)


def rest_segment(start, duration, p, q):
    p, q = np.asarray(p), np.asarray(q); delta = q-p
    return dict(start=start, duration=duration, initial=jet(p), final=jet(q),
        coefficients=np.array([p, np.zeros_like(p), np.zeros_like(p), 10*delta, -15*delta, 6*delta]).tolist())


def rehash_plan(plan):
    plan['referenceSHA256'] = audit.value_sha({k: plan[k] for k in audit.PLAN_FIELDS})


def model_fixture():
    frame = dict(anchor=dict(x=0., y=0., z=0.), rotation=dict(x=0., y=0., z=0., w=1.))
    def part(name, parent, index=None, anchor=(0., 0., 0.), mass=0.):
        pf = copy.deepcopy(frame); pf['anchor'] = dict(zip('xyz', anchor))
        return dict(id=name, parent=parent, massKg=mass,
            jointProfile=None if parent is None else dict(parentFrame=pf, childFrame=copy.deepcopy(frame),
                axes=[] if index is None else [dict(minRadians=-1., maxRadians=1.)]),
            reference=dict(hinge=None if index is None else dict(qposIndex=index)),
            goldenMass=dict(centerOfMass=dict(x=0., y=0., z=0.)))
    rows = [part('torso', None, mass=1.), part('@abdomen_z', 'torso', 0),
        part('lower_waist', '@abdomen_z', 1), part('pelvis', 'lower_waist', 2),
        part('left_foot', 'pelvis', anchor=(-.1, 0., 0.)),
        part('right_foot', 'pelvis', anchor=(.1, 0., 0.))]
    rows += [part('unused'+str(i), 'torso', i) for i in range(3, 21)]
    return dict(totalMassKg=1., segments=rows)


def fixture():
    start, duration = 4., audit.DURATION
    q = np.zeros(21); q[[7, 13]] = -.98
    target_q = q.copy(); target_q[0] = -.03; target_q[[7, 13]] = -.94
    initial = dict(com=jet([.003, -.001, .8]), lower_joints=jet(q[:15]),
        thorax=orientation(1), pelvis=orientation(4))
    target = dict(com=jet([0., 0., .8]), lower_joints=jet(target_q[:15]),
        thorax=orientation(1, (0., .03, 0.)), pelvis=orientation(4))
    feet = [dict(translation=jet([0., y, .8]), orientation=orientation(body))
        for y, body in ((-.1, 10), (.1, 7))]
    support = dict(source_step=3200, source_time=start, step_index=1, phase='transfer',
        loaded_points_xy=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
        stance_points_xyz=[[-1., -1., 0.], [1., -1., 0.], [1., 1., 0.], [-1., 1., 0.]],
        present_corners=[True]*4, surviving_corners=[0, 1, 2], require_triangle=True)
    # Keep initial/path well inside the retained triangle instead of its diagonal.
    support['stance_points_xyz'] = [[-2., -1., 0.], [2., -1., 0.], [0., 2., 0.], [-2., 2., 0.]]
    inputs = dict(sourceStep=3200, sourceTimeS=start, stepIndex=1, phase='transfer',
        initialCoupledReference=initial, activeCompleteCOM=copy.deepcopy(initial['com']),
        activeBaselineJointRad=q.tolist(), activeBodyReferences={'1': copy.deepcopy(initial['thorax'])},
        activeFootReferences=feet, support=support, supportSHA256=audit.value_sha(support))
    candidate = dict(name='pure-test', provenance_sha256=audit.value_sha(inputs), target=target)
    plan = dict(version='coupled-posture-pure-v1', candidate=candidate,
        com=rest_segment(start, duration, initial['com']['position'], target['com']['position']),
        lower_joints=rest_segment(start, duration, q[:15], target_q[:15]),
        thorax=dict(body=1, angles=rest_segment(start, duration, [0., 0., 0.], [0., .03, 0.])),
        pelvis=dict(body=4, angles=rest_segment(start, duration, [0., 0., 0.], [0., 0., 0.])),
        lumbar_anchor=q[:3].tolist(), joint_limits=[[-1., 1.]]*21,
        lumbar_change_limit=math.radians(5.), finiteTimePhysicalFeasibilityProven=False, staticSeedsArePathProof=False)
    rehash_plan(plan)
    inherited = dict(policy='explicit-inherited-reference-to-reserved-endpoint-v1',
        initialReserveRad=np.minimum(q[:15]+1., 1.-q[:15]).tolist(),
        wholePathReferenceReserveFloorRad=[.02]*15,
        endpointRequiredReserveRad=[.02]*15, genericWholePath06GuardUnchanged=True, nativePhysicalGatesUnchanged=True)
    inherited['endpointRequiredReserveRad'][7] = inherited['endpointRequiredReserveRad'][13] = .06
    # Floating representation is checked numerically, not as a textual digest.
    receipt = dict(schema=1, version='current-reference-single-family-v1', accepted=True,
        staticEndpointAdmitted=True, rejectionReasons=[], maximumIterations=100, physicsIntegrations=0,
        fullPhysicalPathProven=False, riskHistoryAdmissionInherited=False, originalAdmissionInheritedForPosture=False,
        automaticReplanningImplemented=False, requestedSourceStep=3200, requestedSourceTimeS=start,
        input=inputs, inputSHA256=audit.value_sha(inputs), candidate=candidate, candidateKey=audit.value_sha(candidate),
        frozenPlan=plan, inheritedTransition=inherited, policy=copy.deepcopy(audit.STATIC_POLICY),
        fullFutureMappedJointReferenceRad=target_q.tolist())
    for key in ('oneBoundedFamily', 'noFallback', 'noParameterScan', 'noNativeIntegration', 'noLiveStateInstall',
                'initialJetsComeFromPreviousEffectiveContext', 'changedEffectiveTasksRequireFreshRiskHistory',
                'genericWholePath06GuardUnchanged', 'actualInputAndHistoryUnchanged'):
        receipt[key] = True
    delta = np.array([.004, -.002, 0.])
    config = dict(com1=(np.array(target['com']['position'])-delta).tolist(), q0=q.tolist(),
        rootR=Rotation.from_euler('ZYX', [.4, .07, .03]).as_matrix().tolist(), phase_durations=dict(settle=2.))
    config['coupledPosture'] = dict(profile=audit.PROFILE, plan=plan,
        frozenCandidateInputSHA256=receipt['inputSHA256'], completedBaselineCorrectionM=delta.tolist(),
        inheritedTransition=inherited, originalAdmissionInheritedForPosture=False,
        pelvisWeight=0., originalRestReturn=None)
    causal = dict(arm=dict(nativeSourceTimeS=start), robustCorrection=dict(selected=True, deltaM=delta.tolist(), clock=[1., 0., 0.]))
    return receipt, config, causal, model_fixture()


def inputs_for(config, causal, phase, now):
    causal['arm']['nativeSourceTimeS'] = now
    u = min(1., max(0., (now-4.)/audit.DURATION))
    config['coupledPosture']['pelvisWeight'] = 4*(10*u**3-15*u**4+6*u**5)
    baseline_com = tuple(np.array(x) for x in ([.1, .2, .8], [.01, .02, 0.], [.03, .04, 0.]))
    baseline_joints = tuple(np.r_[np.zeros(15), np.arange(6)*(.01+i*.02)] for i in range(3))
    other = dict(orientation(99), kp=9., kd=3., weight=2.)
    baseline_orientations = [dict(orientation(1), kp=40., kd=12., weight=4.), other]
    return baseline_com, baseline_joints, baseline_orientations


def install_return(config, causal, model, start=7.):
    config['q0'] = np.r_[[.02, -.01, .03], np.zeros(18)].tolist()
    target = config['coupledPosture']['plan']['candidate']['target']
    thorax = dict(body=1, rotation=np.array(config['rootR']).reshape(-1).tolist(), velocity=[0.]*3, acceleration=[0.]*3)
    pelvis = audit.pelvis_reference(model, tuple(np.asarray(config['q0']) if i == 0 else np.zeros(21) for i in range(3)), thorax)
    arcs = []
    for name, goal in (('thorax', thorax), ('pelvis', pelvis)):
        initial = np.array(target[name]['rotation']).reshape(3, 3)
        axis = Rotation.from_matrix(np.array(goal['rotation']).reshape(3, 3)@initial.T).as_rotvec()
        arcs.append(dict(body=target[name]['body'], initialRotation=initial.tolist(), axis=axis.tolist()))
    config['coupledPosture']['originalRestReturn'] = dict(start=start, duration=2.,
        lower=rest_segment(start, 2., target['lower_joints']['position'], config['q0'][:15]), arcs=arcs)


class PurePostureReferenceTests(unittest.TestCase):
    def test_valid_static_binding_and_independent_endpoint_fk(self):
        receipt, _, _, model = fixture()
        result = audit.audit_posture_static_receipt(receipt, model_data=model)
        self.assertTrue(result['passed'], result)
        self.assertFalse(result['nativeFeasibilityIndependentlyReplayed'])
        self.assertEqual(result['physicsIntegrations'], 0)

    def test_bound_executed_xml_limits_override_narrower_source_json_only_when_exact(self):
        receipt, _, _, model = fixture()
        expanded = np.array(receipt['frozenPlan']['joint_limits'])
        expanded[[3, 4, 5, 9, 10, 11]] += [-math.radians(10.), math.radians(10.)]
        receipt['frozenPlan']['joint_limits'] = expanded.tolist(); rehash_plan(receipt['frozenPlan'])
        initial_q = np.array(receipt['input']['initialCoupledReference']['lower_joints']['position'])
        receipt['inheritedTransition']['initialReserveRad'] = np.minimum(initial_q-expanded[:15, 0], expanded[:15, 1]-initial_q).tolist()
        self.assertFalse(audit.audit_posture_static_receipt(receipt, model_data=model)['passed'])
        result = audit.audit_posture_static_receipt(receipt, model_data=model, joint_limits=expanded)
        self.assertTrue(result['passed'], result)
        expanded[3, 0] += .001
        self.assertFalse(audit.audit_posture_static_receipt(receipt, model_data=model, joint_limits=expanded)['passed'])

    def test_mutated_input_candidate_and_policy_fail_closed(self):
        for kind in ('input', 'candidate', 'solution', 'policy', 'digest', 'profile'):
            receipt, _, _, model = fixture()
            if kind == 'input': receipt['input']['activeCompleteCOM']['position'][0] += .01
            if kind == 'candidate':
                receipt['candidate']['target']['com']['position'][0] += .001
                receipt['frozenPlan']['com'] = rest_segment(4., audit.DURATION,
                    receipt['input']['initialCoupledReference']['com']['position'],
                    receipt['candidate']['target']['com']['position'])
                receipt['candidateKey'] = audit.value_sha(receipt['candidate'])
                rehash_plan(receipt['frozenPlan'])
            if kind == 'solution': receipt['fullFutureMappedJointReferenceRad'][1] = .02
            if kind == 'policy': receipt['policy']['minimumStaticAnkleReferenceRad'] = .02
            if kind == 'digest': receipt['candidateKey'] = '0'*64
            if kind == 'profile': receipt['profile'] = 'reviewed-coupled-posture-pilot-v2'
            with self.subTest(kind=kind):
                result = audit.audit_posture_static_receipt(receipt, model_data=model)
                self.assertFalse(result['passed'])
                if kind == 'candidate': self.assertFalse(result['checks']['independentEndpointFullJointFK'])

    def test_rehashed_changed_polynomial_and_cache_poison_fail(self):
        receipt, _, _, _ = fixture(); plan = receipt['frozenPlan']
        first = audit.validate_posture_plan(plan)
        first['com'][0, 0] = 999.
        self.assertLess(abs(audit.validate_posture_plan(plan)['com'][0, 0]), 1.)
        plan['lower_joints']['coefficients'][4][7] += .0001; rehash_plan(plan)
        with self.assertRaisesRegex(ValueError, 'polynomial'):
            audit.validate_posture_plan(plan)

    def test_inherited_02_to_06_is_separate_from_generic_06_floor(self):
        receipt, _, _, _ = fixture(); plan = receipt['frozenPlan']
        result = audit.validate_posture_plan(plan)
        self.assertAlmostEqual(result['inheritedTransition']['initialReserveRad'][7], .02)
        self.assertAlmostEqual(result['inheritedTransition']['endpointRequiredReserveRad'][7], .06)
        self.assertLess(result['inheritedTransition']['initialReserveRad'][7], .06)
        plan['candidate']['target']['lower_joints']['position'][7] = -.941
        plan['lower_joints'] = rest_segment(4., audit.DURATION,
            plan['lower_joints']['initial']['position'], plan['candidate']['target']['lower_joints']['position'])
        rehash_plan(plan)
        with self.assertRaisesRegex(ValueError, 'endpoint lacks'):
            audit.validate_posture_plan(plan)

    def test_transfer_complete_com_uses_old_delta_exactly_once_and_preserves_arms(self):
        _, config, causal, _ = fixture(); now = 4.+.5*audit.DURATION
        args = inputs_for(config, causal, 'transfer', now)
        result = audit.posture_reference_jets(config, causal, 4., 'transfer', *args)
        assert_allclose(result['com'][0], [.0015, -.0005, .8], atol=1e-14)
        assert_allclose(result['com1'], [-.004, .002, .8], atol=1e-14)
        assert_allclose(result['correctionClock'], [1., 0., 0.])
        for got, old in zip(result['joints'], args[1]): assert_allclose(got[15:], old[15:])
        self.assertEqual([row['body'] for row in result['orientations']], [99, 1, 4])
        self.assertAlmostEqual(result['orientations'][-1]['weight'], 2.)
        self.assertEqual(result['orientations'][0], args[2][1])
        config['com1'][0] += .004
        with self.assertRaisesRegex(ValueError, 'double counts'):
            audit.posture_reference_jets(config, causal, 4., 'transfer', *args)

    def test_later_phases_preserve_complete_baseline_com(self):
        for phase in ('lift', 'translate', 'land'):
            _, config, causal, _ = fixture(); args = inputs_for(config, causal, phase, 6.)
            result = audit.posture_reference_jets(config, causal, .1, phase, *args)
            self.assertIs(result['com'], args[0])
            assert_allclose(result['joints'][0][:15], config['coupledPosture']['plan']['candidate']['target']['lower_joints']['position'])

    def test_heading_world_angular_jets_match_finite_differences(self):
        _, config, causal, _ = fixture(); plan = config['coupledPosture']['plan']
        plan['thorax']['angles'] = rest_segment(4., audit.DURATION, [.7, .03, -.05], [.7, .08, .02])
        plan['candidate']['target']['thorax'] = orientation(1, [.7, .08, .02]); rehash_plan(plan)
        now, h = 4.6, 1e-5; rows = []
        for t in (now-h, now, now+h):
            args = inputs_for(config, causal, 'transfer', t)
            rows.append(audit.posture_reference_jets(config, causal, 4., 'transfer', *args)['orientations'][-2])
        Rm, R, Rp = [np.array(row['rotation']).reshape(3, 3) for row in rows]
        assert_allclose(Rotation.from_matrix(Rp@Rm.T).as_rotvec()/(2*h), rows[1]['velocity'], atol=1e-10)
        assert_allclose((np.array(rows[2]['velocity'])-rows[0]['velocity'])/(2*h), rows[1]['acceleration'], atol=1e-9)
        self.assertAlmostEqual(math.atan2(R[1, 0], R[0, 0]), .7)

    def test_fixed_world_so3_settle_return_targets_same_q0_and_preserves_com(self):
        _, config, causal, model = fixture(); install_return(config, causal, model)
        now, h = 7.7, 1e-5; rows = []
        for t in (now-h, now, now+h):
            args = inputs_for(config, causal, 'settle', t)
            result = audit.posture_reference_jets(config, causal, t-7., 'settle', *args)
            self.assertIs(result['com'], args[0]); rows.append(result['orientations'][-2:])
        self.assertTrue(audit.audit_posture_return(config, causal, now+h-7., 'settle', model)['passed'])
        for i in (0, 1):
            Rm, Rp = [np.array(rows[j][i]['rotation']).reshape(3, 3) for j in (0, 2)]
            assert_allclose(Rotation.from_matrix(Rp@Rm.T).as_rotvec()/(2*h), rows[1][i]['velocity'], atol=1e-10)
            assert_allclose((np.array(rows[2][i]['velocity'])-rows[0][i]['velocity'])/(2*h), rows[1][i]['acceleration'], atol=1e-9)
        args = inputs_for(config, causal, 'stand', 10.)
        final = audit.posture_reference_jets(config, causal, 0., 'stand', *args)
        assert_allclose(final['joints'][0][:15], config['q0'][:15], atol=1e-13)
        assert_allclose(np.array(final['orientations'][-2]['rotation']).reshape(3, 3), config['rootR'], atol=1e-13)
        config['coupledPosture']['originalRestReturn']['arcs'][1]['axis'][0] += .01
        self.assertFalse(audit.audit_posture_return(config, causal, 0., 'stand', model)['passed'])

    def test_retargeted_lower_return_cannot_keep_older_pelvis_goal(self):
        _, config, causal, model = fixture(); install_return(config, causal, model)
        config['q0'][0] += .01
        ret = config['coupledPosture']['originalRestReturn']
        ret['lower'] = rest_segment(ret['start'], ret['duration'], ret['lower']['initial']['position'], config['q0'][:15])
        inputs_for(config, causal, 'settle', 7.5)
        self.assertFalse(audit.audit_posture_return(config, causal, .5, 'settle', model)['passed'])

    def test_stand_q0_reset_does_not_retarget_frozen_settle_return(self):
        _, config, causal, model = fixture(); install_return(config, causal, model)
        frozen = np.array(config['q0'][:15])
        settle_args = inputs_for(config, causal, 'settle', 9.)
        endpoint = audit.posture_reference_jets(config, causal, 2., 'settle', *settle_args)
        config['q0'] = (np.asarray(config['q0'])+.005).tolist()
        stand_args = inputs_for(config, causal, 'stand', 9.1)
        held = audit.posture_reference_jets(config, causal, .1, 'stand', *stand_args)
        self.assertTrue(audit.audit_posture_return(config, causal, .1, 'stand', model)['passed'])
        assert_allclose(held['joints'][0][:15], frozen, atol=1e-13)
        for a, b in zip(endpoint['orientations'][-2:], held['orientations'][-2:]):
            assert_allclose(a['rotation'], b['rotation'], atol=1e-13)
            assert_allclose(b['velocity'], np.zeros(3), atol=1e-13)
            assert_allclose(b['acceleration'], np.zeros(3), atol=1e-13)

    def test_whole_support_capture_detects_between_endpoint_escape(self):
        receipt, _, _, _ = fixture(); support = receipt['input']['support']
        coefficients = np.zeros((6, 3)); coefficients[0] = [0., 0., .8]
        # All endpoint positions fit; a fast interior bump leaves both hull/capture.
        coefficients[2, 0] = 16.; coefficients[3, 0] = -32.; coefficients[4, 0] = 16.
        self.assertLess(audit._whole_support_margin(support, coefficients, 1.), 0.)
        support['present_corners'][0] = False
        with self.assertRaises(ValueError): audit._whole_support_margin(support, coefficients, 1.)

    def test_profile_missing_return_and_delta_mutations_are_rejected(self):
        for kind in ('profile', 'return', 'delta', 'weight'):
            _, config, causal, _ = fixture(); phase = 'settle' if kind == 'return' else 'transfer'
            args = inputs_for(config, causal, phase, 6. if kind == 'return' else 4.5)
            if kind == 'profile': config['coupledPosture']['profile'] = 'reviewed-coupled-posture-pilot-v2'
            if kind == 'delta': causal['robustCorrection']['deltaM'][0] += .001
            if kind == 'weight': config['coupledPosture']['pelvisWeight'] += .1
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                audit.posture_reference_jets(config, causal, 0., phase, *args)


if __name__ == '__main__':
    unittest.main()
