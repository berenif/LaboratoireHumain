"""Pure coordinate/rigid-wrench identities; never constructs native worlds."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation

from coupled_posture_reference import Jet
from coupled_posture_kinematics import MatrixJet, pose_jet_from_source
from native_planned_jet_provider import (NativePlannedJetProvider,
    generalized_planned_jets, jacobian_at_fixed_origin, skew)


class PlannedNativeTests(unittest.TestCase):
    def test_root_angular_jets_use_native_local_frame(self):
        rotation = Rotation.from_euler('xyz', [.31, -.27, .44]).as_matrix()
        omega, alpha = np.array([.2, -.3, .1]), np.array([-.4, .1, .3])
        translation = Jet([.3, -.2, .8], [.03, .02, -.01], [.1, -.2, .3])
        root = pose_jet_from_source(translation, rotation, omega, alpha)
        full = Jet(np.arange(21) * .01, np.arange(21) * .02, np.arange(21) * -.03)
        q, v, a = generalized_planned_jets(SimpleNamespace(root_model=root, full_joints=full))
        np.testing.assert_allclose(q[:3], translation.position, atol=1e-15)
        np.testing.assert_allclose(Rotation.from_quat(q[3:7][[1, 2, 3, 0]]).as_matrix(), rotation, atol=1e-15)
        np.testing.assert_allclose(v[:3], translation.velocity, atol=1e-15)
        np.testing.assert_allclose(v[3:6], rotation.T @ omega, atol=1e-15)
        np.testing.assert_allclose(a[:3], translation.acceleration, atol=1e-15)
        np.testing.assert_allclose(a[3:6], rotation.T @ alpha, atol=1e-15)
        np.testing.assert_array_equal(q[7:], full.position)
        np.testing.assert_array_equal(v[6:], full.velocity)
        np.testing.assert_array_equal(a[6:], full.acceleration)

    def test_fixed_origin_preserves_all_point_wrench_work(self):
        rng = np.random.default_rng(2026)
        J = rng.normal(size=(6, 27))
        body, origin, point = rng.normal(size=(3, 3))
        force = rng.normal(size=3)
        shifted = jacobian_at_fixed_origin(J, body, origin)
        point_jacobian = J[:3] - skew(point - body) @ J[3:]
        wrench = np.r_[force, np.cross(point - origin, force)]
        np.testing.assert_allclose(shifted.T @ wrench, point_jacobian.T @ force, atol=4e-15)
        np.testing.assert_array_equal(J[3:], shifted[3:])

    def test_running_world_is_refused_before_any_write(self):
        world = SimpleNamespace(mass=40., h=1,
            l=SimpleNamespace(lh_dim=lambda *_: 3200), install_cold=Mock())
        with self.assertRaisesRegex(ValueError, 'counter-zero'):
            NativePlannedJetProvider(world, [(1, (0., 0., 0.)), (1, (.1, 0., 0.)),
                (1, (0., .1, 0.))], [0., 0., 0.], np.eye(4))
        world.install_cold.assert_not_called()

    def test_changed_rigid_stance_refused_before_any_write(self):
        world = SimpleNamespace(mass=40., h=1,
            l=SimpleNamespace(lh_dim=lambda *_: 0), install_cold=Mock())
        provider = NativePlannedJetProvider(world, [(1, (0., 0., 0.)),
            (1, (.1, 0., 0.)), (1, (0., .1, 0.))], [0., 0., 0.], np.eye(4))
        changed = np.eye(4); changed[0, 3] = .001
        sample = SimpleNamespace(geometric=SimpleNamespace(pose=lambda _: MatrixJet.rest(changed)))
        with self.assertRaisesRegex(ValueError, 'rigid stance'):
            provider(sample)
        world.install_cold.assert_not_called()

    def test_invalid_contacts_and_shapes_fail_closed(self):
        world = SimpleNamespace(mass=40., h=1, l=SimpleNamespace(lh_dim=lambda *_: 0))
        for contacts in ([(1, (0., 0., 0.))] * 3,
                         [(0, (0., 0., 0.)), (1, (.1, 0., 0.)), (1, (0., .1, 0.))]):
            with self.assertRaises(ValueError):
                NativePlannedJetProvider(world, contacts, [0., 0., 0.], np.eye(4))
        with self.assertRaises(ValueError):
            jacobian_at_fixed_origin(np.zeros((3, 27)), np.zeros(3), np.zeros(3))

    def test_nonfinite_or_incomplete_joint_jets_fail_closed(self):
        for bad in ((np.zeros(20), np.zeros(20), np.zeros(20)),
                    (np.zeros(21), np.full(21, np.nan), np.zeros(21))):
            geometry = SimpleNamespace(root_model=MatrixJet.rest(np.eye(4)),
                full_joints=SimpleNamespace(arrays=lambda: bad))
            with self.assertRaisesRegex(ValueError, 'finite full21'):
                generalized_planned_jets(geometry)


if __name__ == '__main__':
    unittest.main(verbosity=2)
