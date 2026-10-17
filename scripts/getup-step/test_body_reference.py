import json,unittest
from pathlib import Path
import numpy as np
from body_reference import BodyReference,QuinticBridge,left_jacobian,left_jacobian_dot,ROOT,proper_rotation
from scipy.spatial.transform import Rotation
BASE=ROOT/'docs/fixtures/getup-step'

class BodyReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref=BodyReference(json.loads((BASE/'forward-static-witness.json').read_text()),json.loads((BASE/'load-acquisition-static-witness.json').read_text()),json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()),dense_per_segment=17)
    def test_quintic_matches_full_boundary_jets(self):
        p0=np.array([1.,-.2]);v0=np.array([-.4,.5]);a0=np.array([.1,.2]);p1=np.array([2.,.3]);v1=np.array([.2,-.1]);a1=np.array([-.2,.3]);b=QuinticBridge(p0,v0,a0,p1,v1,a1,.5)
        for actual,expected in zip(b.sample(0),[p0,v0,a0]):np.testing.assert_allclose(actual,expected,atol=1e-13)
        for actual,expected in zip(b.sample(.5),[p1,v1,a1]):np.testing.assert_allclose(actual,expected,atol=1e-12)
    def test_left_SO3_chart_angular_velocity_not_raw_quat_derivative(self):
        phi=np.array([.2,-.3,.1]);v=np.array([.4,.2,-.3]);e=1e-6
        measured=Rotation.from_matrix(Rotation.from_rotvec(phi+e*v).as_matrix()@Rotation.from_rotvec(phi-e*v).as_matrix().T).as_rotvec()/(2*e)
        np.testing.assert_allclose(measured,left_jacobian(phi)@v,atol=1e-10)
    def test_initial_twist_exception_is_only_elbow20(self):
        r=self.ref;s=r.sample('prepare',0.);np.testing.assert_allclose(s['q'][:20],r.initial_q[:20],atol=0)
        np.testing.assert_allclose(s['qvel'][6:26],r.initial_velocity[6:26],atol=1e-13)
        self.assertEqual(s['qvel'][-1],0.);self.assertAlmostEqual(s['q'][20]-r.initial_q[20],1.1920928955078125e-7)
        np.testing.assert_allclose(s['qvel'][:3],__import__('source_math').P@r.initial_velocity[:3],atol=1e-13)
        rr=proper_rotation(r.initial_root[:3,:3]);np.testing.assert_allclose(s['qvel'][3:6],__import__('source_math').P@rr.T@r.initial_velocity[3:6],atol=1e-13)
    def test_bridge_to_regular_position_velocity_and_acceleration(self):
        r=self.ref;q0,v0,a0=r.joint_bridge.sample(.5);q1,_=r.regular_geometry('prepare',.5);v1,a1=r.joint_motion('prepare',.5)
        for a,b in [(q0,q1),(v0,v1),(a0,a1)]:np.testing.assert_allclose(a,b,atol=1e-11)
        for t in np.linspace(0,.5,101):
            q=r.geometry('prepare',float(t))[0];self.assertTrue(np.all(q>=r.limits[:,0]-1e-12));self.assertTrue(np.all(q<=r.limits[:,1]+1e-12))
    def test_forward_path_and_placed_stance_remain_forward(self):
        r=self.ref;a=r.sample('advance',0.);b=r.sample('advance',2.);delta=b['leftFoot'][:3,3]-a['leftFoot'][:3,3];heading=np.array(r.plan['entryTorsoForwardWorldUnit']);self.assertAlmostEqual(delta@heading,.06,places=6)
        for t in np.linspace(0,2,41):self.assertGreater(r.sample('advance',float(t))['clearance'],.0249)
        for phase in ['lower','acquire','settle']:
            end=r.sample(phase,r.DURATIONS[phase]);self.assertLess(np.linalg.norm(end['leftFoot'][:3,3]-b['leftFoot'][:3,3]-np.array([0.,-.025,0.])),1e-5)
if __name__=='__main__':unittest.main()
