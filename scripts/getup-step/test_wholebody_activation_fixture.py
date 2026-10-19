"""Portable retained-input contract tests with a SYNTHETIC mathematical witness."""
import copy,hashlib,json,os,tempfile,unittest,zlib
from dataclasses import asdict,replace
from pathlib import Path
from unittest.mock import patch
import numpy as np
from wholebody_math_fixture import make_math_fixture
from wholebody_activation_fixture import load_retained_activation,sha,INPUT_KIND
from wholebody_cycle_reference import WholeBodyCycle
from coupled_posture_reference import digest

class ActivationFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(os.environ.get('REVIEWED_WALK_REPO_ROOT')or Path(__file__).resolve().parents[2]);cls.f=make_math_fixture(cls.root)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.out=Path(self.temp.name);f=self.f
        config=json.loads(f.context.config_json);config.update(stance_side=f.realizer.stance_side,swing_side=f.realizer.swing_side,
            heading=f.heading.tolist(),leg_origin=list(f.leg_origin))
        context=replace(f.context,phase='transfer',elapsed=4.,config_json=json.dumps(config))
        support=replace(f.support,phase='transfer')
        self.xml=self.out/'source.xml';self.xml.write_text('<explicit-mathematical-fixture/>')
        self.payload=self.out/'state.zlib';raw=b'SYNTHETIC mathematical native witness; never a restored state';self.payload.write_bytes(zlib.compress(raw))
        self.knots=np.tile(np.asarray(context.joints.position)[:15][f.realizer.independent_indices],(3,1)).tolist()
        self.seed=self.out/'source-proposal.json';self.seed.write_text(json.dumps(dict(independentKnotsRad=self.knots,mathematicalFixture=True)))
        old_binding=dict(schema=1,proposalSHA256=sha(self.seed),stanceSide=f.realizer.stance_side,swingSide=f.realizer.swing_side,
            originalSourceLimitsRad=f.limits.tolist(),legOriginRightLeftRad=list(f.leg_origin),
            modelSHA256=sha(self.root/'rust/crates/model/data/getup-v1.json'),sourceXMLSHA256=sha(self.xml),armMapSemanticSHA256=digest(asdict(f.arm_map)))
        self.annotation=self.out/'binding.json';self.annotation.write_text(json.dumps(old_binding))
        self.data=dict(referenceContext=context.receipt(),currentSupport=asdict(support),currentFeetModel=[x.tolist()for x in f.feet_model],
            actualPayloadSHA256=hashlib.sha256(raw).hexdigest(),actualPayloadBytes=len(raw),retainedPayload=self.payload.name,
            compressedPayloadSHA256=sha(self.payload),proposal=dict(binding=old_binding,independentKnotsRad=self.knots,
                proposalSHA256=sha(self.seed),offlineReceiptSHA256=digest(json.loads(self.seed.read_text())),flightOptimizerCallsAtRuntime=0))
        self.path=self.out/'actual-activation.json';self.write()

    def write(self):self.path.write_text(json.dumps(self.data))
    def load(self):
        return load_retained_activation(self.path,repo_root=self.root,arm_map=self.f.arm_map,source_xml_path=self.xml,
            kinematics=self.f.kin,source_proposal_path=self.seed,source_proposal_binding_path=self.annotation)

    def test_exact_context_payload_support_feet_and_explicit_zero_lift_elapsed(self):
        with patch.object(self.f.kin,'forward',side_effect=AssertionError('adapter must not evaluate FK')):
            a=self.load()
        self.assertEqual(a.context.receipt(),self.data['referenceContext']);self.assertEqual(a.support.key,digest(self.data['currentSupport']))
        self.assertEqual(a.binding['currentSupportSHA256'],a.support.key)
        self.assertEqual(a.binding['actualFeetModelSHA256'],digest(self.data['currentFeetModel']))
        self.assertEqual(a.binding['headingSource'],'referenceContext.config.heading');self.assertEqual(a.binding['entryLiftElapsedS'],0.)
        self.assertEqual(a.binding['referenceElapsedS'],4.);self.assertEqual(a.binding['inputKind'],INPUT_KIND)
        self.assertTrue(a.binding['payloadHashVerified']);self.assertFalse(a.binding['stateRestored']);self.assertFalse(a.binding['oldFirstLiftInputReused'])
        np.testing.assert_array_equal(a.realizer.stance_pose.T,self.data['currentFeetModel'][a.realizer.stance_side])

    def test_runtime_constructor_uses_full_original_lift_clock(self):
        a=self.load();p=WholeBodyCycle(a.realizer,a.context,a.limits,a.heading,self.knots)
        self.assertEqual(p.durations,(.8,1.2,1.));self.assertEqual(p.entry_elapsed,0.)
        self.assertIs(p.entry,a.context);self.assertIs(p.realizer.arm_map,self.f.arm_map)

    def test_measured_strong_arms_and_different_weak_tail_preserved(self):
        a=self.load();self.assertEqual(a.context.arms.velocity,tuple(self.data['referenceContext']['arms']['velocity']))
        self.assertNotEqual(a.context.arms.velocity,a.context.joints.velocity[15:])
        self.assertEqual(a.context.joints.velocity[15:],(0.,)*6)

    def test_stale_support_rejected(self):
        self.data['currentSupport']['source_step']+=1;self.write()
        with self.assertRaisesRegex(ValueError,'support'):self.load()

    def test_payload_hash_and_basename_rejected(self):
        self.payload.write_bytes(self.payload.read_bytes()+b'corrupt')
        with self.assertRaisesRegex(ValueError,'witness hash'):self.load()
        self.data['retainedPayload']='../state.zlib';self.write()
        with self.assertRaisesRegex(ValueError,'basename'):self.load()

    def test_wrong_map_xml_or_source_proposal_rejected(self):
        self.xml.write_text('changed')
        with self.assertRaisesRegex(ValueError,'XML hash'):self.load()
        self.xml.write_text('<explicit-mathematical-fixture/>');self.seed.write_text('changed')
        with self.assertRaisesRegex((ValueError,json.JSONDecodeError),''):self.load()

    def test_incomplete_transfer_cannot_be_relabelled(self):
        from source_reference_context import ReferenceContext
        context=ReferenceContext.from_receipt(self.data['referenceContext']);context=replace(context,elapsed=3.9)
        self.data['referenceContext']=context.receipt();self.write()
        with self.assertRaisesRegex(ValueError,'stationary TRANSFER'):self.load()

if __name__=='__main__':unittest.main(verbosity=2)
