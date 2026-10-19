"""Pure same-foot endpoint contracts; no optimizer/native trial in unit tests."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import copy
import os
import importlib.util
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from coupled_posture_kinematics import ArmMap, ModelFK, MatrixJet, P
from coupled_posture_reference import Jet
from source_reference_context import Jet as ContextJet, Foot, Orientation, ReferenceContext
from wholebody_settle_reference import (_rest_fk, solve_samefoot_endpoint,
    SameFootSettlePlan, digest)

REPO = Path(os.environ.get('REVIEWED_WALK_REPO_ROOT') or Path(__file__).resolve().parents[2])
SHARED = Path('/workspace/shared/reviewed-walk')


@unittest.skipUnless((SHARED/'phase-leg-reference/qualified-lift-entry-input.json').exists(),
    'optional archived source-reference fixture absent')
class EndpointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = json.loads((REPO/'rust/crates/model/data/getup-v1.json').read_text())
        cls.model = ModelFK.from_model(model)
        cls.mapping = ArmMap.from_archived_table(json.loads(
            (SHARED/'coupled-posture-design/arm-map-table.json').read_text()))
        data = json.loads((SHARED/'phase-leg-reference/qualified-lift-entry-input.json').read_text())
        cls.q = np.array(data['referenceContext']['joints']['position'])[:15]
        cls.origin = np.array(data['controller']['leg_origin'])
        xml = Path('/workspace/shared/cadence-robust-8-ten/model-session/source-model/humanoid_high_freq.xml')
        cls.limits = np.deg2rad(np.array([list(map(float, j.attrib['range'].split()))
            for j in ET.parse(xml).getroot().find('worldbody').findall('.//joint')]))
        relative = _rest_fk(cls.model, np.r_[cls.q, np.zeros(6)], np.eye(4))
        theta = []
        for name in ('right', 'left'):
            d = P@(relative[name+'_shin'][:3, 3]-relative[name+'_thigh'][:3, 3])
            theta.append(math.atan2(d[0], -d[2]))
        arms = cls.mapping.evaluate(Jet.rest(-(np.array(theta)-cls.origin))).position
        cls.full = np.r_[cls.q, arms]
        poses = _rest_fk(cls.model, cls.full, np.eye(4))
        cls.feet = [poses['left_foot'], poses['right_foot']]
        cls.com = sum(p.mass*(P@(poses[p.name]@np.r_[p.com, 1.])[:3])
            for p in cls.model.parts)/cls.model.total_mass
        cls.pelvis = P@poses['pelvis'][:3, :3]@P.T
        cls.context = ReferenceContext.from_receipt(data['referenceContext'])

    def args(self):
        x, y = self.com[:2]
        return dict(stance_side=0, current_feet_model=self.feet,
            measured_com_target=self.com, initial_lower=self.q, lumbar_anchor=self.q[:3],
            thorax_preference_source=np.eye(3), pelvis_preference_source=self.pelvis,
            actual_hull_equations=[[1., 0., -x-1.], [-1., 0., x-1.],
                [0., 1., -y-1.], [0., -1., y-1.]],
            source_identity=dict(sourceStep=123, sourceTimeS=123*.00125,
                actualStateSHA256='a'*64, modelSHA256='b'*64), heading_rad=0.)

    def fake_optimizer(self, objective, initial, **kwargs):
        self.calls += 1
        self.assertEqual(kwargs['method'], 'SLSQP')
        self.assertEqual(kwargs['options']['maxiter'], 100)
        self.assertEqual(len(kwargs['constraints']), 2)
        self.assertLess(max(abs(kwargs['constraints'][0]['fun'](initial))), 1e-10)
        self.assertGreaterEqual(min(kwargs['constraints'][1]['fun'](initial)), -1e-10)
        kwargs['callback'](initial)
        return SimpleNamespace(x=initial, success=True, message='fixture exact', nit=0)

    def solve(self, **changes):
        args = self.args(); args.update(changes); self.calls = 0
        return solve_samefoot_endpoint(self.model, self.limits, self.mapping, self.origin,
            optimizer=self.fake_optimizer, **args)

    def test_exact_same_pose_accepts_geometry_only(self):
        result = self.solve()
        self.assertTrue(result.accepted)
        self.assertEqual(self.calls, 1)
        r = result.receipt()
        self.assertFalse(r['C2PathCertified'])
        self.assertFalse(r['motorContactCollisionCertified'])
        self.assertFalse(r['physicalQualificationClaim'])
        self.assertEqual(r['nativeWorlds'], 0)
        self.assertTrue(r['rootRealizedFromCurrentStance'])
        self.assertLess(max(abs(np.array(result.full_joints)-self.full)), 1e-10)

    def test_fast_position_fk_matches_existing_analytic_graph(self):
        from coupled_posture_reference import Jet
        slow = self.model.forward(Jet.rest(self.full))
        fast = _rest_fk(self.model, self.full, np.eye(4))
        self.assertLess(max(np.max(abs(slow[k].arrays()[0]-v)) for k,v in fast.items()), 1e-12)

    def test_input_arrays_not_mutated(self):
        q = self.q.copy(); feet = [x.copy() for x in self.feet]
        self.solve(initial_lower=q, current_feet_model=feet)
        np.testing.assert_array_equal(q, self.q)
        for x,y in zip(feet, self.feet): np.testing.assert_array_equal(x,y)

    def test_genuine_out_of_bounds_initializer_rejects_no_clamp(self):
        q = self.q.copy(); q[7] = self.limits[7, 0]
        with self.assertRaisesRegex(ValueError, 'no clamp'): self.solve(initial_lower=q)
        self.assertEqual(self.calls, 0)

    def test_outside_current_support_rejects_before_solver(self):
        target = self.com.copy(); target[0] += 2.
        with self.assertRaisesRegex(ValueError, '3mm'): self.solve(measured_com_target=target)
        self.assertEqual(self.calls, 0)

    def test_nonrigid_actual_foot_rejected(self):
        feet = [x.copy() for x in self.feet]; feet[0][0, 0] += .1
        with self.assertRaisesRegex(ValueError, 'not rigid'): self.solve(current_feet_model=feet)

    def test_soft_posture_error_does_not_invent_hard_anatomy(self):
        from scipy.spatial.transform import Rotation
        result = self.solve(pelvis_preference_source=Rotation.from_euler('y', .2).as_matrix())
        self.assertTrue(result.accepted)

    def test_receipt_is_detached(self):
        result = self.solve(); r = result.receipt(); r['accepted'] = False
        self.assertTrue(result.receipt()['accepted'])

    def path(self):
        from wholebody_cycle_reference import StanceRealizedIK
        spec = importlib.util.spec_from_file_location('quiet_test_existing_kin', REPO/'scripts/getup-step-reference.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        model = json.loads((REPO/'rust/crates/model/data/getup-v1.json').read_text())
        kin = module.Kinematics(model)
        realizer = StanceRealizedIK(kin, 0, MatrixJet.rest(self.feet[0]), self.mapping, self.origin)
        feet = tuple(Foot(ContextJet.make(P@T[:3,3], np.zeros(3), np.zeros(3)),
            Orientation.make(i, P@T[:3,:3]@P.T, np.zeros(3), np.zeros(3)))
            for i,T in enumerate(self.feet))
        config = json.loads(self.context.config_json); config['stance_side'] = 0
        config['finalCom'] = self.com.tolist()
        context = replace(self.context, phase='settle', elapsed=0.,
            source_step=123, source_time=123*.00125, config_json=json.dumps(config),
            com=ContextJet.make(self.com,np.zeros(3),np.zeros(3)),
            joints=ContextJet.make(self.full,np.zeros(21),np.zeros(21)),
            arms=ContextJet.make(self.full[15:],np.zeros(6),np.zeros(6)),
            orientations=(Orientation.make(1,np.eye(3),np.zeros(3),np.zeros(3)),
                Orientation.make(4,self.pelvis,np.zeros(3),np.zeros(3))), feet=feet)
        return SameFootSettlePlan(realizer,context,self.solve(),self.feet[1],self.limits,
            current_identity=self.args()['source_identity']),context

    def rebuild(self, plan, context, **changes):
        args = dict(realizer=plan.realizer, context=context, endpoint=plan.endpoint,
            current_other_foot_model=self.feet[1], source_limits=self.limits,
            current_identity=self.args()['source_identity'])
        args.update(changes)
        return SameFootSettlePlan(**args)

    def test_settle_unchanged_common_pose_exact_jets_and_arms(self):
        plan, context = self.path()
        for t in (0., .3, 1.52, 8.):
            sample = plan.sample(t, runtime_arms=context.arms)
            for a,b in zip(sample.lower_joints.arrays(),(self.q,np.zeros(15),np.zeros(15))):
                self.assertLess(max(abs(a-b)), 1e-8)
            self.assertLess(max(abs(np.array(sample.com.position)-self.com)),1e-10)
            self.assertLess(max(abs(np.array(sample.com.velocity))),1e-8)
            self.assertLess(max(abs(np.array(sample.com.acceleration))),1e-8)
            self.assertEqual(sample.geometric.full_joints.position[15:],context.arms.position)

    def test_settle_does_not_reset_nonzero_incoming_joint_jet(self):
        plan, context = self.path()
        velocity = np.zeros(21); velocity[4] = 1e-12
        context = replace(context, joints=ContextJet.make(self.full,velocity,np.zeros(21)))
        with self.assertRaisesRegex(ValueError, 'no zeroing'):
            self.rebuild(plan,context)

    def test_settle_original_duration_and_current_boundary_required(self):
        plan, context = self.path()
        with self.assertRaisesRegex(ValueError, '1.52'):
            self.rebuild(plan,context,duration=2.)
        with self.assertRaisesRegex(ValueError, 'boundary'):
            self.rebuild(plan,replace(context,elapsed=.00125))

    def test_first_command_lag_uses_remaining_original_clock_without_backdating(self):
        plan, context = self.path()
        boundary = dict(sourceStep=context.source_step, sourceTimeS=context.source_time)
        identity = dict(self.args()['source_identity'])
        identity['sourceStep'] += 2; identity['sourceTimeS'] += .0025
        current = replace(context, source_step=context.source_step+2,
            source_time=context.source_time+.0025, elapsed=.0025)
        endpoint = self.solve(source_identity=identity)
        later = SameFootSettlePlan(plan.realizer,current,endpoint,self.feet[1],self.limits,
            current_identity=identity,phase_boundary=boundary)
        self.assertEqual(later.original_duration,1.52)
        self.assertAlmostEqual(later.duration,1.5175)
        self.assertEqual(later.sample(.0025).diagnostics['boundedClockS'],0.)
        self.assertAlmostEqual(later.sample(1.52).diagnostics['boundedClockS'],1.5175)
        with self.assertRaisesRegex(ValueError,'before causal'):
            later.sample(0.)

    def test_first_command_lag_rejects_stale_or_wrong_boundary(self):
        plan, context = self.path()
        for lag in (4,5):
            identity=dict(self.args()['source_identity'])
            identity['sourceStep']+=lag;identity['sourceTimeS']+=lag*.00125
            current=replace(context,source_step=context.source_step+lag,
                source_time=context.source_time+lag*.00125,elapsed=lag*.00125)
            endpoint=self.solve(source_identity=identity)
            with self.subTest(lag=lag),self.assertRaisesRegex(ValueError,'boundary'):
                SameFootSettlePlan(plan.realizer,current,endpoint,self.feet[1],self.limits,
                    current_identity=identity,phase_boundary=dict(sourceStep=context.source_step,
                        sourceTimeS=context.source_time))

    def test_settle_never_accepts_failed_endpoint(self):
        plan, context = self.path()
        with self.assertRaisesRegex(ValueError, 'accepted current'):
            self.rebuild(plan,context,endpoint=replace(plan.endpoint,accepted=False))

    def test_settle_requires_verified_current_identity(self):
        plan, context = self.path()
        with self.assertRaisesRegex(ValueError, 'verified current'):
            self.rebuild(plan,context,current_identity={})
        for key in ('sourceStep','sourceTimeS','actualStateSHA256','modelSHA256'):
            identity = dict(self.args()['source_identity'])
            identity[key] = identity[key]+1 if isinstance(identity[key],(float,int)) else 'f'*64
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'source/model identity'):
                self.rebuild(plan,context,current_identity=identity)
        with self.assertRaisesRegex(ValueError, 'source/model identity'):
            self.rebuild(plan,replace(context,source_step=context.source_step+1))

    def test_settle_binds_both_actual_feet_and_stance(self):
        plan, context = self.path()
        other = self.feet[1].copy(); other[0,3] += .001
        with self.assertRaisesRegex(ValueError, 'stance/other foot'):
            self.rebuild(plan,context,current_other_foot_model=other)
        realizer = copy.copy(plan.realizer)
        stance = self.feet[0].copy(); stance[0,3] += .001
        realizer.stance_pose = MatrixJet.rest(stance)
        with self.assertRaisesRegex(ValueError, 'stance/other foot'):
            self.rebuild(plan,context,realizer=realizer)
        config = json.loads(context.config_json); config['stance_side'] = 1
        with self.assertRaisesRegex(ValueError, 'stance/other foot'):
            self.rebuild(plan,replace(context,config_json=json.dumps(config)))

    def test_settle_binds_model_all_limits_map_and_origin(self):
        plan, context = self.path()
        limits = self.limits.copy(); limits[20,1] += .01
        with self.assertRaisesRegex(ValueError, 'model/limits/map/origin'):
            self.rebuild(plan,context,source_limits=limits)
        for attribute, value in (
                ('model',replace(plan.realizer.model,total_mass=plan.realizer.model.total_mass+.01)),
                ('arm_map',replace(self.mapping,goal=tuple(np.array(self.mapping.goal)+.001))),
                ('leg_origin',tuple(self.origin+.001))):
            realizer = copy.copy(plan.realizer); setattr(realizer,attribute,value)
            with self.subTest(attribute=attribute), self.assertRaisesRegex(ValueError, 'model/limits/map/origin'):
                self.rebuild(plan,context,realizer=realizer)

    def test_settle_binds_initializer_and_lumbar_anchor(self):
        plan, context = self.path()
        q = self.full.copy(); q[4] += .001
        with self.assertRaisesRegex(ValueError, 'initializer/lumbar anchor'):
            self.rebuild(plan,replace(context,joints=ContextJet.make(q,np.zeros(21),np.zeros(21))))
        receipt = plan.endpoint.receipt(); receipt['input']['lumbarAnchor'][0] += .001
        receipt['inputSHA256'] = digest(receipt['input'])
        endpoint = replace(plan.endpoint,receipt_json=json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, 'initializer/lumbar anchor'):
            self.rebuild(plan,context,endpoint=endpoint)

    def test_settle_binds_measured_COM_and_retained_heading(self):
        from scipy.spatial.transform import Rotation
        plan, context = self.path()
        config = json.loads(context.config_json); config['finalCom'][0] += .001
        with self.assertRaisesRegex(ValueError, 'COM/retained heading'):
            self.rebuild(plan,replace(context,config_json=json.dumps(config)))
        orientations = (Orientation.make(1,Rotation.from_euler('z',.001).as_matrix(),
            np.zeros(3),np.zeros(3)),context.orientations[1])
        with self.assertRaisesRegex(ValueError, 'COM/retained heading'):
            self.rebuild(plan,replace(context,orientations=orientations))

    def test_settle_saved_artifact_needs_explicit_verified_binding(self):
        from wholebody_settle_reference import _kinematic_bindings
        plan, context = self.path()
        receipt = plan.endpoint.receipt(); bindings = receipt['input'].pop('kinematicBindings')
        receipt['inputSHA256'] = digest(receipt['input'])
        endpoint = replace(plan.endpoint,receipt_json=json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, 'saved-artifact binding'):
            self.rebuild(plan,context,endpoint=endpoint)
        evidence = dict(kind='independently-verified-saved-artifact',
            endpointInputSHA256=receipt['inputSHA256'],endpointReceiptSHA256=digest(receipt),
            verificationReceiptSHA256='c'*64,kinematicBindings=bindings)
        self.rebuild(plan,context,endpoint=endpoint,saved_artifact_binding=evidence)
        evidence['endpointReceiptSHA256'] = 'd'*64
        with self.assertRaisesRegex(ValueError, 'saved-artifact binding'):
            self.rebuild(plan,context,endpoint=endpoint,saved_artifact_binding=evidence)

    def test_settle_prepare_rejects_stale_actual_support_before_sampling(self):
        from coupled_posture_reference import CurrentSupport
        plan, context = self.path(); x,y = self.com[:2]
        support = CurrentSupport(context.source_step+1,context.source_time,context.step_index,
            context.phase,((x-1,y-1),(x+1,y-1),(x+1,y+1),(x-1,y+1)),
            ((x-1,y-1,0),(x+1,y-1,0),(x+1,y+1,0),(x-1,y+1,0)),
            (True,True,True,True),(),False)
        plan.sample = lambda *a,**k: self.fail('stale support must fail before sampling')
        with self.assertRaisesRegex(ValueError, 'stale or wrong-phase'):
            plan.prepare(support,samples=9)


class MathematicalEndpointTests(EndpointTests):
    """Portable synthetic geometry only, never an admitted source posture."""
    __unittest_skip__ = False

    @classmethod
    def setUpClass(cls):
        from wholebody_math_fixture import make_math_fixture
        fixture = make_math_fixture(REPO)
        cls.model, cls.mapping, cls.limits = fixture.model, fixture.arm_map, fixture.limits
        cls.q = np.array(fixture.lower.position); cls.origin = np.array(fixture.leg_origin)
        cls.full = np.r_[cls.q, np.zeros(6)]
        root = np.eye(4); root[1,3] = 1.4
        poses = _rest_fk(cls.model, cls.full, root)
        cls.feet = [poses['left_foot'], poses['right_foot']]
        cls.com = sum(p.mass*(P@(poses[p.name]@np.r_[p.com,1.])[:3])
            for p in cls.model.parts)/cls.model.total_mass
        cls.pelvis = P@poses['pelvis'][:3,:3]@P.T
        cls.context = fixture.context


if __name__ == '__main__': unittest.main()
