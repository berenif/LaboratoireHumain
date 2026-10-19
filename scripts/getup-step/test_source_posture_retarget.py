"""Portable posture-planner tests: no native World or new optimization case.

Pure export/import/seed tests always run. Archived-terminal mock tests are
optional: set COUPLED_POSTURE_FIXTURES to the reviewed fixture directory. They
skip cleanly when the independent local evidence/model/helper files are absent;
no raw fixture assets are part of this test or required in the repository.
"""
import json,math,types,unittest,xml.etree.ElementTree as ET,sys,dataclasses,os,importlib.util
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
SCRIPT_DIR=Path(__file__).resolve().parent
sys.path.insert(0,str(SCRIPT_DIR))
from coupled_posture_reference import Jet
from coupled_posture_kinematics import ModelFK,MatrixJet,ArmMap,P
from source_reference_context import ReferenceContext
import source_posture_retarget as planner

HERE=Path(os.environ.get('COUPLED_POSTURE_FIXTURES','/workspace/shared/reviewed-walk/coupled-posture-reference'))
ROOT=Path(__file__).resolve().parents[2]
RUNROOT=HERE.parent/'causal-selection-one-01'
MODEL=ROOT/'rust/crates/model/data/getup-v1.json'
XML=RUNROOT/'model-session/source-model/humanoid_high_freq.xml'
HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
BODY={'torso':1,'head':2,'lower_waist':3,'pelvis':4,'right_thigh':5,'right_shin':6,
    'right_foot':7,'left_thigh':8,'left_shin':9,'left_foot':10,'right_upper_arm':11,
    'right_lower_arm':12,'right_hand':13,'left_upper_arm':14,'left_lower_arm':15,'left_hand':16}


FIXTURES=(MODEL,XML,HELPER,HERE/'current-reference-witness.json',HERE/'current-seed-admission.json',
    HERE/'current-support-witness.json',HERE/'retargeted-candidate.json',
    HERE.parent/'coupled-posture-design/arm-map-table.json',
    HERE.parent/'coupled-posture-design/free-com-candidate-step-01.json')
HAS_FIXTURES=all(p.is_file() for p in FIXTURES)


@unittest.skipUnless(HAS_FIXTURES,'optional archived evidence unavailable; set COUPLED_POSTURE_FIXTURES')
class RuntimeRetargetExtractionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=ReferenceContext.from_receipt(json.loads((HERE/'current-reference-witness.json').read_text())['referenceContext'])
        cls.cfg=json.loads(cls.context.config_json)
        cls.old=json.loads((HERE/'current-seed-admission.json').read_text())
        cls.geometry=json.loads((HERE/'current-support-witness.json').read_text())
        cls.final=json.loads((HERE/'retargeted-candidate.json').read_text())
        cls.seed=json.loads((HERE.parent/'coupled-posture-design/free-com-candidate-step-01.json').read_text())
        cls.modeljson=json.loads(MODEL.read_text());cls.model=ModelFK.from_model(cls.modeljson)
        cls.arm=ArmMap.from_archived_table(json.loads((HERE.parent/'coupled-posture-design/arm-map-table.json').read_text()))
        nodes={n.attrib['name']:n for n in ET.parse(XML).getroot().iter('joint') if 'name' in n.attrib}
        cls.limits=np.zeros((21,2))
        for p in cls.modeljson['segments']:
            h=(p.get('reference') or {}).get('hinge')
            if h:cls.limits[h['qposIndex']]=np.deg2rad([float(x) for x in nodes[h['name']].attrib['range'].split()])

    def fixtures(self):
        owner=self
        class Mapping:
            ranges=np.array(owner.arm.ranges);neutral_pitch=np.array(owner.arm.neutral_pitch);goal=np.array(owner.arm.goal)
            curves=tuple(types.SimpleNamespace(x=np.array(x),c=np.array(c)) for x,c in zip(owner.arm.grids,owner.arm.coefficients))
            def evaluate(self,delta):
                p=np.array(owner.arm.evaluate(Jet.rest(delta),reserve=0.).position)
                return p,np.zeros(6),np.zeros(6)
        class FakeWorld:
            def __init__(self,actual=False):
                self.actual=actual;self.closed=False;self.installs=0;self.limits=owner.limits.copy()
                self.xml=XML;self.helper=HELPER;self.base=RUNROOT/'model-session';self.h=object()
                self.gears=np.array([owner.cfg['B'][6+j][i] for i,j in enumerate(planner.ORDER)])
                self.mass=owner.model.total_mass;self.ids={name:BODY[name] for name in ('torso','left_foot','right_foot')}
                self.l=types.SimpleNamespace(lh_dim=lambda h,k:3200 if self.actual else 0,
                    lh_time=lambda h:owner.context.source_time if self.actual else 0.,
                    lh_support_count=lambda h:len(owner.geometry['currentLoadedHullPointsXY']),
                    lh_id=lambda h,kind,name:BODY[name.decode()],lh_self_separation=lambda *args:None,
                    lh_step_audited=lambda *args:(_ for _ in ()).throw(AssertionError('native step forbidden')))
                self.qpos=np.r_[np.zeros(3),1.,np.zeros(3),owner.context.joints.position]
                self.qvel=np.zeros(27);self.commands=np.zeros(21)
                self.positions=np.zeros((17,3));self.rotations=np.tile(np.eye(3),(17,1,1));self.com=np.zeros((17,3))
                if actual:
                    for key,value in owner.old['actualBodyOrientationsSource'].items():self.rotations[int(key)]=value
                    self.com[1]=owner.context.com.position
                else:self._update()
                self._initial_q=self.qpos.copy()
            def _update(self):
                root=np.eye(4);root[:3,:3]=P.T@Rotation.from_quat(self.qpos[[4,5,6,3]]).as_matrix()@P;root[:3,3]=P.T@self.qpos[:3]
                poses=owner.model.forward(Jet.rest(self.qpos[7:]),MatrixJet.rest(root))
                for name,body in BODY.items():
                    T=poses[name].T;self.positions[body]=P@T[:3,3];self.rotations[body]=P@T[:3,:3]@P.T
                self.com[1]=sum(p.mass*(P@(poses[p.name].T@np.r_[p.com,1.])[:3]) for p in owner.model.parts)/self.mass
            def arr(self,index,shape):
                if index==0:return self.qpos.copy()
                if index==1:return self.qvel.copy()
                if index==2:return self.commands.copy()
                if index==3:return self.positions.copy()
                if index==5:return self.rotations.copy()
                if index==6:return self.com.copy()
                if index==21:
                    out=np.zeros(128);loaded=np.array(owner.geometry['currentLoadedHullPointsXY']).ravel();out[:len(loaded)]=loaded;return out
                raise AssertionError('unexpected read')
            def feet(self):
                out=[]
                for i,name in enumerate(('left_foot','right_foot')):
                    if self.actual and i==1:
                        points=np.array(owner.geometry['currentGeometricStanceCornersSource']);points[:,2]+=.027
                    else:
                        # Geometry is mocked; no LP is run on these points.
                        points=np.array([[-.05,-.05,.027],[.05,-.05,.027],[.05,.05,.027],[-.05,.05,.027]])
                        points+=self.positions[BODY[name]]
                    out.append(dict(body=BODY[name],points=points,radius=.027,
                        groundRays=[dict(distanceM=.2265) for _ in range(4)]))
                return out
            def read(self,name,shape,*args):
                if self.actual:raise AssertionError('actual native dynamics must not be queried')
                if name=='lh_self_separation':
                    a=np.zeros(shape);a[0]=1.;a[5]=owner.final['minimumCleanSelfGapM'];return a
                raise AssertionError('unexpected native or optimizer operation: '+name)
            def install_cold(self,entry):
                if self.actual:raise AssertionError('actual state must not be installed')
                self.installs+=1;self.qpos=np.array(entry['qpos']);self.qvel=np.array(entry['qvel']);self._update()
            def close(self):
                if self.actual:raise AssertionError('actual world must not be closed')
                self.closed=True
        actual=FakeWorld(True)
        controller=types.SimpleNamespace(w=actual,_mapping=Mapping(),leg_origin=np.array(self.cfg['leg_origin']),
            stance_side=1,swing_side=0,robust_support=dict(selected=True,survivingCorners=[1,2,3]),
            _robust_survivors=np.array([1,2,3]),_reviewed_step_index=1,_reference_epoch=0,
            previous_time=self.context.source_time,previous_leg_rate=np.zeros(2),_seek_time=self.context.source_time,
            B=np.array(self.cfg['B']),phase_duration=lambda phase:self.cfg['phase_durations'][phase])
        worlds=[]
        def factory(world):
            self.assertIs(world,actual);w=FakeWorld(False);worlds.append(w);return w
        def mock_lp(w,side,B,points):
            self.assertFalse(w.actual)
            return dict(success=True,status='mock of already audited LP',requiredMotorFraction=.9249,
                sourceContactPointsM=np.array(points).tolist(),commands=[0.]*21,forcesN=[[0.,0.,100.]]*len(points),
                maximumEqualityResidual=0.,maximumFrictionViolation=0.,minimumNormalN=0.)
        return controller,factory,worlds,mock_lp

    def test_mock_boundary_replays_existing_terminal_without_new_optimizer(self):
        c,factory,worlds,mock_lp=self.fixtures();before=c.w.qpos.copy();times=(c.previous_time,c._seek_time)
        calls=[]
        def mock_minimize(objective,start,method,bounds,constraints,callback,options):
            calls.append(True);self.assertEqual(method,'SLSQP');self.assertEqual(options['maxiter'],100)
            np.testing.assert_allclose(start,self.seed['qReference'][:15],atol=1e-14)
            terminal=np.array(self.final['qReference'][:15]);self.assertLess(max(abs(constraints[0]['fun'](terminal))),1e-10)
            self.assertGreater(min(constraints[1]['fun'](terminal)),-1e-8)
            callback(terminal)
            return types.SimpleNamespace(x=terminal,success=True,message='mocked existing terminal',nit=5,fun=objective(terminal))
        with patch.object(planner,'minimize',mock_minimize),patch.object(planner,'_strict_lp',mock_lp), \
             patch.object(planner,'linprog',side_effect=AssertionError('no new LP/native case allowed')):
            result=planner.retarget_current_reference(c,self.context,seed=self.seed['qReference'],model_path=MODEL,scratch_factory=factory)
        receipt=result.receipt();self.assertTrue(result.accepted,receipt.get('rejectionReasons'))
        self.assertEqual(len(calls),1);self.assertEqual(len(worlds),2)
        self.assertTrue(all(w.closed for w in worlds));self.assertTrue(all(w.installs>0 for w in worlds))
        self.assertEqual((c.previous_time,c._seek_time),times);np.testing.assert_array_equal(c.w.qpos,before)
        self.assertEqual(c.w.installs,0);self.assertFalse(c.w.closed)
        self.assertTrue(receipt['actualInputAndHistoryUnchanged'])
        self.assertTrue(receipt['comHandoff']['wholeCurrent3mmGatePassed'])
        self.assertFalse(receipt['comHandoff']['wholeCurrent4mmDesignInsetPassed'])
        np.testing.assert_allclose(result.candidate.target.com.position,self.final['comReferenceSource'],atol=1e-14)
        self.assertEqual(receipt['input']['sourceStep'],3200)
        self.assertEqual(receipt['input']['supportSHA256'],self.geometry['supportSHA256'])
        self.assertEqual(result.candidate.provenance_sha256,receipt['inputSHA256'])
        initial=planner.initial_coupled_reference(result)
        self.assertEqual(initial.com.position,self.context.com.position)
        self.assertEqual(planner.initial_coupled_reference(result.receipt_json),initial)
        tampered=result.receipt();tampered['input']['initialCoupledReference']['com']['position'][0]+=.001
        with self.assertRaisesRegex(ValueError,'digest'):
            planner.initial_coupled_reference(tampered)
        with self.assertRaises(Exception):result.accepted=False

    def test_no_support_or_missing_dependency_rejects_without_optimizer_or_scratch(self):
        c,factory,worlds,mock_lp=self.fixtures();c.robust_support['selected']=False
        with patch.object(planner,'minimize',side_effect=AssertionError('no optimizer expected')):
            r=planner.retarget_current_reference(c,self.context,model_path=MODEL,scratch_factory=factory)
        self.assertFalse(r.accepted);self.assertEqual(worlds,[])
        self.assertIn('causal support',r.receipt()['rejectionReasons'][0])
        c.robust_support['selected']=True
        with patch.object(planner,'minimize',side_effect=AssertionError('no optimizer expected')):
            r=planner.retarget_current_reference(c,self.context,model_path=HERE/'does-not-exist.json',scratch_factory=factory)
        self.assertFalse(r.accepted);self.assertIn('missing',r.receipt()['rejectionReasons'][0]);self.assertEqual(worlds,[])

    def test_actual_world_cannot_be_returned_as_scratch(self):
        c,factory,worlds,mock_lp=self.fixtures()
        with patch.object(planner,'minimize',side_effect=AssertionError('no optimizer expected')):
            r=planner.retarget_current_reference(c,self.context,model_path=MODEL,scratch_factory=lambda actual:actual)
        self.assertFalse(r.accepted);self.assertIn('actual world',r.receipt()['rejectionReasons'][0])
        self.assertFalse(c.w.closed);self.assertEqual(c.w.installs,0)

    def test_provisional_actual_triangle_is_explicit_and_never_inherits_history(self):
        c,factory,worlds,mock_lp=self.fixtures();c.robust_support['selected']=False
        def existing_terminal(objective,start,**kwargs):
            terminal=np.array(self.final['qReference'][:15])
            return types.SimpleNamespace(x=terminal,success=True,message='mock existing terminal',nit=5,fun=objective(terminal))
        with patch.object(planner,'minimize',existing_terminal),patch.object(planner,'_strict_lp',mock_lp):
            r=planner.retarget_current_reference(c,self.context,seed=self.seed['qReference'],
                provisional_survivors=(1,2,3),model_path=MODEL,scratch_factory=factory)
        d=r.receipt();self.assertTrue(r.accepted,d.get('rejectionReasons'))
        self.assertTrue(d['input']['provisionalSupportIsNotAdmittedHistory'])
        self.assertIn('PROVISIONAL',d['input']['supportStatus'])
        self.assertFalse(d['riskHistoryAdmissionInherited'])
        self.assertTrue(d['changedEffectiveTasksRequireFreshRiskHistory'])
        self.assertTrue(d['provisionalTriangleMustMatchFreshAdmittedTriangle'])
        self.assertTrue(d['laterLPMayOnlyCertifyUnchangedCandidate'])
        self.assertFalse(d['automaticReplanningImplemented'])
        self.assertTrue(d['measuredLiftRequiresPostureAndCorrectionComplete'])
        self.assertFalse(c.robust_support['selected'])

    def test_provisional_duplicate_nonactual_and_noninteger_corners_reject(self):
        for ids in ((1,1,3),(1,2,4),(1.,2,3)):
            c,factory,worlds,mock_lp=self.fixtures();c.robust_support['selected']=False
            with patch.object(planner,'minimize',side_effect=AssertionError('no optimizer expected')):
                r=planner.retarget_current_reference(c,self.context,provisional_survivors=ids,
                    model_path=MODEL,scratch_factory=factory)
            self.assertFalse(r.accepted);self.assertEqual(worlds,[])
        c,factory,worlds,mock_lp=self.fixtures();c.robust_support['selected']=False
        original_arr=c.w.arr
        missing_index=self.geometry['uniqueNearestNativeSupportIndices'][1]
        def missing_point(index,shape):
            out=original_arr(index,shape)
            if index==21:out[2*missing_index:2*missing_index+2]=[10.,10.]
            return out
        c.w.arr=missing_point
        with patch.object(planner,'minimize',side_effect=AssertionError('no optimizer expected')):
            r=planner.retarget_current_reference(c,self.context,provisional_survivors=(1,2,3),
                model_path=MODEL,scratch_factory=factory)
        self.assertFalse(r.accepted);self.assertEqual(worlds,[])
        self.assertIn('currently measured',r.receipt()['rejectionReasons'][0])

    def test_failed_single_family_is_rejected_without_fallback(self):
        c,factory,worlds,mock_lp=self.fixtures();calls=[]
        def failure(objective,start,**kwargs):
            calls.append(True);x=np.array(self.final['qReference'][:15])
            return types.SimpleNamespace(x=x,success=False,message='mock100iteration stop',nit=100,fun=objective(x))
        with patch.object(planner,'minimize',failure),patch.object(planner,'_strict_lp',mock_lp):
            r=planner.retarget_current_reference(c,self.context,seed=self.seed['qReference'],model_path=MODEL,scratch_factory=factory)
        self.assertFalse(r.accepted);self.assertIsNone(r.candidate);self.assertEqual(len(calls),1)
        self.assertIn('optimizerConverged',r.receipt()['rejectionReasons'])
        self.assertTrue(all(w.closed for w in worlds))

    def test_initial_full_reference_jets_are_transported_in_receipt(self):
        c,factory,worlds,mock_lp=self.fixtures()
        p,v,a=self.context.com.arrays();v=np.array([.001,0.,0.]);a=np.array([0.,.002,0.])
        com=type(self.context.com).make(p,v,a)
        q,dq,ddq=self.context.joints.arrays();dq[:3]=[.001,-.002,.003];ddq[:3]=[-.002,.001,.002]
        joints=type(self.context.joints).make(q,dq,ddq)
        row=self.context.orientations[0];R,_,_=row.arrays()
        orientation=type(row).make(1,R,[.001,.002,-.001],[.002,-.001,.003])
        effective=dataclasses.replace(self.context,com=com,joints=joints,orientations=(orientation,))
        def terminal(objective,start,**kwargs):
            x=np.array(self.final['qReference'][:15])
            return types.SimpleNamespace(x=x,success=True,message='mock existing terminal',nit=5,fun=objective(x))
        with patch.object(planner,'minimize',terminal),patch.object(planner,'_strict_lp',mock_lp):
            r=planner.retarget_current_reference(c,effective,seed=self.seed['qReference'],model_path=MODEL,scratch_factory=factory)
        initial=r.receipt()['input']['initialCoupledReference']
        np.testing.assert_array_equal(initial['com']['velocity'],v)
        np.testing.assert_array_equal(initial['com']['acceleration'],a)
        np.testing.assert_array_equal(initial['lower_joints']['velocity'],dq[:15])
        np.testing.assert_array_equal(initial['lower_joints']['acceleration'],ddq[:15])
        np.testing.assert_array_equal(initial['thorax']['velocity'],orientation.velocity)
        self.assertGreater(np.linalg.norm(initial['pelvis']['velocity']),0.)
        self.assertTrue(r.receipt()['initialJetsComeFromPreviousEffectiveContext'])


# This is the approved tested JOINT-ONLY initializer, never a root/COM pose.
TESTED_INITIALIZER=(
    0.18050181924398612, -0.2972829659936228, -0.19732176749503477,
    0.14638199997990162, -0.07062275347295917, 0.123107691197508,
    -0.4568222193594367, -0.2890658503988637, -0.15162675091264105,
    -0.19939141296204496, 0.11589658398035425, 0.017912581470861395,
    -0.4782739862277805, -0.22022151461437406, -0.2220632498590764)


class PortablePostureExports(unittest.TestCase):
    def sample_receipt(self):
        from coupled_posture_reference import CoupledSample,OrientationJet,digest
        sample=CoupledSample(Jet([.1,-.2,.8],[.01,-.02,.03],[.001,.002,-.001]),
            Jet(np.zeros(15),np.linspace(-.01,.01,15),np.linspace(.001,-.001,15)),
            OrientationJet(1,np.eye(3).ravel(),[.01,.02,-.01],[.001,-.002,.003]),
            OrientationJet(4,np.eye(3).ravel(),[-.01,.01,.02],[.003,.002,-.001]))
        inputs=dict(sourceStep=3200,sourceTimeS=4.,initialCoupledReference=dataclasses.asdict(sample))
        receipt=dict(input=inputs,inputSHA256=digest(inputs))
        return sample,planner.RetargetResult(False,None,json.dumps(receipt))

    def test_exact_runtime_seed_is_tested_joint_only_initializer(self):
        import source_reviewed_posture
        self.assertIsInstance(source_reviewed_posture.SEED,tuple)
        self.assertEqual(source_reviewed_posture.SEED,TESTED_INITIALIZER)
        self.assertEqual(len(source_reviewed_posture.SEED),15)
        optional=HERE/'tested-retarget-initializer.json'
        if optional.is_file():
            self.assertEqual(tuple(json.loads(optional.read_text())['qLower15Rad']),TESTED_INITIALIZER)

    def test_helper_reconstructs_immutable_initial_jets_and_rejects_digest_tampering(self):
        from dataclasses import FrozenInstanceError
        sample,result=self.sample_receipt()
        for receipt in (result,result.receipt_json,result.receipt()):
            self.assertEqual(planner.initial_coupled_reference(receipt),sample)
        changed=result.receipt();changed['input']['initialCoupledReference']['com']['velocity'][0]+=.001
        with self.assertRaisesRegex(ValueError,'digest'):
            planner.initial_coupled_reference(changed)
        with self.assertRaisesRegex(ValueError,'digest'):
            planner.initial_coupled_reference({})
        with self.assertRaises(FrozenInstanceError):
            planner.initial_coupled_reference(result).com.velocity=(0.,)*3
        self.assertEqual(planner.initial_coupled_reference(result).com.velocity,sample.com.velocity)

    def test_fresh_module_imports_do_not_construct_worlds_or_invoke_solvers(self):
        import source_step_probe,scipy.optimize
        with patch.object(source_step_probe.World,'__init__',side_effect=AssertionError('no native World')), \
             patch.object(scipy.optimize,'minimize',side_effect=AssertionError('no optimizer')), \
             patch.object(scipy.optimize,'linprog',side_effect=AssertionError('no static LP')):
            for filename in ('source_posture_retarget.py','source_reviewed_posture.py'):
                name='_portable_import_'+filename.removesuffix('.py')
                spec=importlib.util.spec_from_file_location(name,SCRIPT_DIR/filename)
                module=importlib.util.module_from_spec(spec);sys.modules[name]=module
                try:
                    spec.loader.exec_module(module)
                    if filename=='source_posture_retarget.py':
                        self.assertTrue(callable(module.retarget_current_reference))
                        self.assertTrue(callable(module.initial_coupled_reference))
                finally:sys.modules.pop(name,None)


if __name__=='__main__':unittest.main(verbosity=2)
