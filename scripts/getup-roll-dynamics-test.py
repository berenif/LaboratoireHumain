#!/usr/bin/env python3
"""Pure symbolic/numerical checks. No native execution, dynamics solve, or integration."""
import importlib.util
import json
from pathlib import Path
import unittest

import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("roll_dynamics", ROOT / "scripts/getup-roll-dynamics.py")
roll = importlib.util.module_from_spec(spec)
spec.loader.exec_module(roll)
NATIVE_ROW = ROOT / "outputs/getup-physical-roll-20261019/final-native-row.json"
NATIVE_INSPECTION = ROOT / "outputs/getup-roll-trajectory-20261019/native1940-inspection.json"


def numpy_frame(frame):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat([frame["rotation"][key] for key in "xyzw"]).as_matrix()
    transform[:3, 3] = roll.xyz(frame["anchor"])
    return transform


def numpy_fk(model, configuration):
    """Independent source-frame FK for pure geometric test configurations."""
    poses = {}
    for part in model["segments"]:
        if part["parent"] is None:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(configuration[3:7]).as_matrix()
            pose[:3, 3] = configuration[:3]
        else:
            profile = part["jointProfile"]
            motion = np.eye(4)
            hinge = part["reference"].get("hinge")
            if hinge:
                motion[:3, :3] = Rotation.from_rotvec([configuration[7+hinge["qposIndex"]], 0, 0]).as_matrix()
            pose = poses[part["parent"]] @ numpy_frame(profile["parentFrame"]) @ motion @ np.linalg.inv(numpy_frame(profile["childFrame"]))
        poses[part["id"]] = pose
    return poses


def geometric_perturbation(configuration, velocity, epsilon):
    """Derivative-test configuration only; no physical state is produced."""
    result = configuration.copy()
    result[:3] += epsilon*velocity[:3]
    result[3:7] = (Rotation.from_rotvec(epsilon*velocity[3:6])*Rotation.from_quat(configuration[3:7])).as_quat()
    result[7:] += epsilon*velocity[6:]
    return result


class RollDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads(roll.MODEL_PATH.read_text())
        cls.d = roll.build_dynamics(cls.model)
        rng = np.random.default_rng(117)
        cls.x = np.r_[[.3, .8, -.2], Rotation.from_rotvec([.7, -.4, .2]).as_quat(),
            cls.d.limits.mean(axis=1)+.15*rng.uniform(-1, 1, 21)]
        cls.v = rng.normal(0, .6, 27)

    def test_build_shapes_and_source_only_root_passives(self):
        self.assertEqual(self.d.poses.size_out(0), (4, 100))
        self.assertEqual(self.d.mass_matrix.size_out(0), (27, 27))
        self.assertEqual(self.d.bias.size_out(0), (27, 1))
        self.assertEqual(self.d.body_kinematics.size_out(1), (150, 27))
        self.assertEqual(self.d.configuration_rate.size_out(0), (28, 1))
        self.assertEqual(self.d.primitive_endpoints.size_out(0), (3, 38))
        self.assertEqual(self.d.primitive_endpoints.size_out(1), (38, 1))
        np.testing.assert_array_equal(self.d.damping[:6], np.zeros(6))
        np.testing.assert_array_equal(self.d.armature[:6], np.zeros(6))
        np.testing.assert_array_equal(np.asarray(self.d.passive(self.x, self.v)).ravel()[:6], np.zeros(6))
        self.assertEqual(len(self.d.massive_indices), 16)
        self.assertEqual(len(self.d.body_ids)-len(self.d.massive_indices), 9)
        self.assertAlmostEqual(self.d.mass, self.model["totalMassKg"], places=12)

    def test_exact_source_frames_and_oblique_axes_at_arbitrary_root_pose(self):
        actual = np.asarray(self.d.poses(self.x))
        expected = numpy_fk(self.model, self.x)
        for i, part in enumerate(self.model["segments"]):
            np.testing.assert_allclose(actual[:, 4*i:4*i+4], expected[part["id"]], atol=2e-15)
        zero = self.x.copy()
        zero[7:] = 0
        zero_poses = numpy_fk(self.model, zero)
        _, jacobians, _, _ = self.d.body_kinematics(zero, np.zeros(27))
        jacobians = np.asarray(jacobians)
        for i, part in enumerate(self.model["segments"]):
            hinge = part["reference"].get("hinge")
            if hinge:
                # The actual frames include small source-body tilts. Merely
                # rotating the source hinge.axis by the root is not exact.
                expected_axis = (zero_poses[part["parent"]] @ numpy_frame(part["jointProfile"]["parentFrame"]))[:3, 0]
                np.testing.assert_allclose(jacobians[6*i+3:6*i+6, 6+hinge["qposIndex"]],
                    expected_axis, atol=2e-15)
        root_rotation = Rotation.from_quat(zero[3:7]).as_matrix()
        abdomen = jacobians[6*self.d.body_index["@abdomen_z"]+3:6*self.d.body_index["@abdomen_z"]+6, 6]
        self.assertGreater(np.linalg.norm(abdomen-root_rotation @ np.array([0, 1, 0])), .0039)

    def test_quaternion_normalization_and_world_angular_qdot(self):
        x = self.x.copy()
        x[3:7] *= 2.3
        np.testing.assert_allclose(self.d.poses(x), self.d.poses(self.x), atol=1e-15)
        xdot, error = self.d.configuration_rate(self.x, self.v)
        xdot = np.asarray(xdot).ravel()
        self.assertAlmostEqual(float(error), 0, places=14)
        self.assertAlmostEqual(np.dot(self.x[3:7], xdot[3:7]), 0, places=14)
        epsilon = 1e-6
        plus = geometric_perturbation(self.x, self.v, epsilon)
        minus = geometric_perturbation(self.x, self.v, -epsilon)
        np.testing.assert_allclose(xdot, (plus-minus)/(2*epsilon), atol=2e-10)

    def test_mass_is_finite_symmetric_positive_and_root_origin_based(self):
        rng = np.random.default_rng(313)
        for _ in range(5):
            x = self.x.copy()
            x[3:7] = rng.normal(size=4)
            x[7:] = rng.uniform(self.d.limits[:, 0], self.d.limits[:, 1])
            mass, bias = self.d.dynamics(x, self.v)
            mass = np.asarray(mass)
            self.assertTrue(np.isfinite(mass).all() and np.isfinite(bias).all())
            np.testing.assert_allclose(mass, mass.T, atol=2e-14)
            self.assertGreater(np.linalg.eigvalsh(mass)[0], .0099)
            np.testing.assert_allclose(mass[:3, :3], self.d.mass*np.eye(3), atol=1e-13)
            com = np.asarray(self.d.com(x, self.v)[0]).ravel()
            np.testing.assert_allclose(mass[:3, 3:6], -self.d.mass*np.asarray(roll.skew(com-x[:3])), atol=2e-14)

    def test_source_passive_signs_and_gear_applied_once(self):
        expected = self.d.spring[6:]*self.x[7:]+self.d.damping[6:]*self.v[6:]
        np.testing.assert_allclose(np.asarray(self.d.passive(self.x, self.v)).ravel(), np.r_[np.zeros(6), expected])
        controls = np.arange(21, dtype=float)/21-.5
        torque = np.asarray(self.d.controls_to_torque(controls)).ravel()
        np.testing.assert_allclose(torque, self.d.gears*controls[self.d.qpos_to_actuator])
        applied = np.asarray(self.d.S @ ca.DM(torque)).ravel()
        np.testing.assert_array_equal(applied[:6], np.zeros(6))
        np.testing.assert_array_equal(applied[6:], torque)
        self.assertEqual(self.d.qpos_to_actuator[0], 1)
        self.assertEqual(self.d.qpos_to_actuator[7], 8)

    def test_com_spatial_jacobians_and_ad_jdot_match_geometry_derivatives(self):
        epsilon = 2e-6
        plus = geometric_perturbation(self.x, self.v, epsilon)
        minus = geometric_perturbation(self.x, self.v, -epsilon)
        centers, jacobians, twists, jdot_v = [np.asarray(value) for value in self.d.body_kinematics(self.x, self.v)]
        plus_centers, plus_j, _, _ = [np.asarray(value) for value in self.d.body_kinematics(plus, self.v)]
        minus_centers, minus_j, _, _ = [np.asarray(value) for value in self.d.body_kinematics(minus, self.v)]
        np.testing.assert_allclose(twists[:3], (plus_centers-minus_centers)/(2*epsilon), atol=5e-10)
        numerical_bias = ((plus_j-minus_j)/(2*epsilon) @ self.v).reshape(25, 6).T
        np.testing.assert_allclose(jdot_v, numerical_bias, atol=2e-9)
        np.testing.assert_allclose(twists, (jacobians @ self.v).reshape(25, 6).T, atol=1e-14)
        self.assertEqual(centers.shape, (3, 25))

    def test_material_point_and_geometry_selected_point_are_separate(self):
        body, local = "right_upper_arm", np.array([-.08, -.07, .05])
        point, jacobian, twist, bias = [np.asarray(value) for value in self.d.point_function(body)(self.x, self.v, local)]
        np.testing.assert_allclose(self.d.world_point_jacobian(body)(self.x, point), jacobian, atol=1e-14)
        epsilon = 2e-6
        plus = geometric_perturbation(self.x, self.v, epsilon)
        minus = geometric_perturbation(self.x, self.v, -epsilon)
        pplus, jplus, _, _ = self.d.point_function(body)(plus, self.v, local)
        pminus, jminus, _, _ = self.d.point_function(body)(minus, self.v, local)
        np.testing.assert_allclose(twist[:3], (pplus-pminus)/(2*epsilon), atol=5e-10)
        np.testing.assert_allclose(bias, (jplus-jminus)@self.v/(2*epsilon), atol=2e-9)
        self.assertEqual(self.d.world_point_jacobian(body).n_out(), 1)

    def test_root_origin_velocity_is_not_root_center_of_mass_velocity(self):
        v = np.zeros(27)
        v[3:6] = [.5, -.7, .9]
        centers, _, twists, _ = [np.asarray(value) for value in self.d.body_kinematics(self.x, v)]
        np.testing.assert_allclose(twists[:3, 0], np.cross(v[3:6], centers[:, 0]-self.x[:3]), atol=2e-16)
        self.assertGreater(np.linalg.norm(twists[:3, 0]), .02)

    def test_bias_energy_power_identity_and_gravity(self):
        epsilon = 2e-6
        plus = geometric_perturbation(self.x, self.v, epsilon)
        minus = geometric_perturbation(self.x, self.v, -epsilon)
        mass_dot = (np.asarray(self.d.mass_matrix(plus))-np.asarray(self.d.mass_matrix(minus)))/(2*epsilon)
        bias = np.asarray(self.d.bias(self.x, self.v)).ravel()
        gravity_and_spring = np.asarray(self.d.bias(self.x, np.zeros(27))).ravel()
        coriolis = bias-gravity_and_spring-self.d.damping*self.v
        self.assertAlmostEqual(self.v @ coriolis, .5*self.v @ mass_dot @ self.v, delta=2e-8)
        gravity = gravity_and_spring-np.asarray(self.d.passive(self.x, np.zeros(27))).ravel()
        com_plus = np.asarray(self.d.com(plus, self.v)[0]).ravel()
        com_minus = np.asarray(self.d.com(minus, self.v)[0]).ravel()
        potential_rate = -self.d.mass*np.asarray(self.d.gravity).ravel() @ ((com_plus-com_minus)/(2*epsilon))
        self.assertAlmostEqual(self.v @ gravity, potential_rate, delta=2e-8)

    def test_primitive_endpoints_floor_gaps_and_material_jacobians(self):
        floor = .017
        points, gaps = [np.asarray(value) for value in self.d.primitive_endpoints(self.x, floor)]
        poses = numpy_fk(self.model, self.x)
        self.assertEqual(len(self.d.primitive_metadata), 38)
        for i, metadata in enumerate(self.d.primitive_metadata):
            expected = (poses[metadata["body"]] @ np.r_[metadata["local"], 1])[:3]
            np.testing.assert_allclose(points[:, i], expected, atol=2e-15)
            self.assertAlmostEqual(gaps[i, 0], expected[1]-metadata["radius"]-floor, places=14)

    @unittest.skipUnless(NATIVE_ROW.exists(), "Retained actual-native row is optional and is never regenerated")
    def test_retained_actual_native_pose_com_and_current_motion_parity(self):
        row = json.loads(NATIVE_ROW.read_text())
        before = NATIVE_ROW.read_bytes()
        parity = roll.native_row_parity(self.d, row)
        self.assertLess(parity["posePositionMaxErrorM"], 2e-6)
        self.assertLess(parity["poseRotationMatrixMaxError"], 2e-6)
        self.assertLess(parity["comMaxErrorM"], 2e-6)
        self.assertLess(parity["comVelocityMaxErrorMps"], 2e-8)
        self.assertLess(parity["footComTwistMaxError"], 2e-8)
        self.assertEqual(parity["physicsIntegrations"], 0)
        self.assertEqual(before, NATIVE_ROW.read_bytes())
        # Cached pose velocity words are receipts, never the current motion.
        changed = dict(row)
        poses = roll.decode_words(row["poseWords"]).astype(np.float32).reshape(25, 13)
        poses[:, 7:] = 123
        changed["poseWords"] = poses.reshape(-1).view(np.uint32).astype(np.uint64).tolist()
        np.testing.assert_array_equal(roll.native_row_state(row)[1], roll.native_row_state(changed)[1])

    @unittest.skipUnless(NATIVE_INSPECTION.exists(), "Immutable native M/J inspection is optional and is never regenerated")
    def test_retained_native_mass_jacobians_armature_and_limits(self):
        before = NATIVE_INSPECTION.read_bytes()
        inspection = json.loads(before)
        parity = roll.native_inspection_parity(self.d, inspection)
        self.assertEqual(parity["nativeSubsteps"], 1940)
        self.assertEqual(parity["nativeReportedPhysicsIntegrations"], 0)
        self.assertTrue(parity["nativeReportedBytesUnchanged"])
        self.assertLess(parity["inverseMass"]["maxAbsoluteError"], 1e-4)
        self.assertLess(parity["inverseMass"]["relativeFrobeniusError"], 3e-6)
        self.assertLess(parity["allBodyComSpatialJacobians"]["maxAbsoluteError"], 3e-6)
        self.assertLess(parity["allBodyCurrentTwists"]["maxAbsoluteError"], 3e-9)
        self.assertLess(parity["comMaxErrorM"], 2e-6)
        self.assertLess(parity["comVelocityMaxErrorMps"], 1e-8)
        self.assertLess(parity["armatureMaxError"], 3e-10)
        self.assertGreater(parity["inverseMassErrorWithoutHingeArmature"], 200)
        self.assertLess(parity["sourceVsNativeLimitMaxErrorRad"], 1e-7)
        shoulder = parity["limits"][18]
        self.assertEqual(shoulder["joint"], "left_shoulder1")
        self.assertAlmostEqual(shoulder["measuredMinusNativeLowerRad"], -2.2649765014648438e-6, places=14)
        self.assertFalse(parity["biasCertificate"])
        self.assertEqual(before, NATIVE_INSPECTION.read_bytes())


if __name__ == "__main__":
    unittest.main()
