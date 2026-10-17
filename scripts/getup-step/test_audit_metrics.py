import copy,json,unittest
import numpy as np
from audit_metrics import ROOT,support_margin,decode_native_frame,task_errors
from source_math import P
class AuditMetricTests(unittest.TestCase):
    def test_actual_native_frame_COM_and_contact_hull(self):
        x=json.loads((ROOT/'docs/fixtures/getup-step/native-audit-frame.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());r=decode_native_frame(x['frame'],x['sample'],model)
        np.testing.assert_allclose(P@r['comWorld'],x['expectedCOMSource'],atol=5e-7,rtol=0.)
        self.assertEqual(r['sample']['right_support_dimension'],2)
        self.assertTrue(np.isfinite(r['controls']).all())
        bad=copy.deepcopy(x['frame']);bad['nativeSubstep']+=1
        with self.assertRaises(ValueError):decode_native_frame(bad,x['sample'],model)
    def test_same_geometric_target_zero_errors_without_clearance_clamp(self):
        x=json.loads((ROOT/'docs/fixtures/getup-step/native-audit-frame.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());r=decode_native_frame(x['frame'],x['sample'],model);p=r['physical']
        target=dict(q=p[:21].tolist(),qvel=p[40:67].tolist(),com=(P@r['comWorld']).tolist(),com_ori=p[34:37].tolist(),comVelocity=(P@r['comVelocityWorld']).tolist(),leftFoot=r['transforms']['left_foot'].tolist(),clearance=r['wholeFootClearanceM'])
        errors=task_errors(r,target,[1.,0.,0.],[0.,0.,0.])
        for name,error in errors.items():np.testing.assert_allclose(error,0.,atol=1e-14,err_msg=name)
    def test_invalid_native_packet_is_not_silently_repaired(self):
        x=json.loads((ROOT/'docs/fixtures/getup-step/native-audit-frame.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
        for kind in ['floatid','nancontact','duplicatesnapshot']:
            a=copy.deepcopy(x['frame']);b=copy.deepcopy(x['sample'])
            if kind=='floatid':b['native_substep']=float(b['native_substep'])
            elif kind=='nancontact':a['contactMotion']['feet'][1]['contacts'][0]['normalImpulseNs']=float('nan')
            else:a['snapshot']['segments'].append(a['snapshot']['segments'][0])
            with self.assertRaises(ValueError):decode_native_frame(a,b,model)
    def test_point_segment_and_empty_never_fabricate_polygon(self):
        for p,dim in [([], -1),([[0,0]],0),([[0,0],[1,0]],1)]:
            margin,actual=support_margin(p,[.5,0]);self.assertIsNone(margin);self.assertEqual(actual,dim)
        inside,dim=support_margin([[0,0],[1,0],[0,1]],[.2,.2]);self.assertEqual(dim,2);self.assertAlmostEqual(inside,.2)
        outside,_=support_margin([[0,0],[1,0],[0,1]],[-.1,.2]);self.assertAlmostEqual(outside,-.1)
if __name__=='__main__':unittest.main()
