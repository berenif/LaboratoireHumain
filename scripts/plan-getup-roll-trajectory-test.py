#!/usr/bin/env python3
"""Build-only and pure algebra tests; never loads or invokes an NLP solver."""
import copy
import importlib.util
from pathlib import Path
import unittest

import casadi as ca
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("roll_planner", ROOT / "scripts/plan-getup-roll-trajectory.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
NATIVE_ROWS = ROOT / "outputs/browser-capability-20261019/recovery-fixture/physical-roll/prestall-context/replay-native1940-archive-boundary/exact-native0-1940.ndjson"


def build_test_contract():
    """Deterministic parent-selected formulation values, NOT an optimizer run."""
    slot = lambda name, body, primitive, end: dict(id=name, body=body, primitive=primitive, end=end)
    slots = [slot("pelvis_a", "pelvis", "butt", "a"), slot("pelvis_b", "pelvis", "butt", "b"),
             slot("arm_a", "right_upper_arm", "right_upper_arm", "a"), slot("rear_foot", "right_foot", "left_right_foot", "a")]
    for body, primitive in (("torso", "torso"), ("torso", "upper_waist"), ("right_shin", "right_shin"), ("left_shin", "left_shin")):
        slots.extend(slot(body+"_"+primitive+"_"+end, body, primitive, end) for end in ("a", "b"))
    slots.append(slot("head", "head", "head", "a"))
    base = [s["id"] for s in slots[:4]]
    prone = [s["id"] for s in slots if s["id"] not in ("arm_a", "rear_foot")]
    return dict(schema="getup-roll-collocation-v1", native_row=str(NATIVE_ROWS), native_inspection="outputs/getup-roll-trajectory-20261019/native1940-inspection-allcontacts.json", start_native=1940, end_native=4480,
        physics_step_s=.00125, mesh_s=[round(.05*i, 12) for i in range(64)]+[3.175], floor_y_m=0., floor_friction=1., self_friction=0., slots=slots,
        self_slot=dict(id="existing_hand_forearm", body_a="right_hand", primitive_a="right_hand", body_b="left_lower_arm", primitive_b="left_lower_arm"),
        phases=[dict(name="load_redistribution", start_s=0., end_s=.3, contacts={s:"stick" for s in base}, left_shoulder1_stop=True, self_contact=True),
                dict(name="right_flank_rotation", start_s=.3, end_s=1.5, contacts={s:"stick" for s in ("pelvis_a", "arm_a", "rear_foot")}, left_shoulder1_stop=True, self_contact=False),
                dict(name="acquire_prone", start_s=1.5, end_s=2.7, contacts={s["id"]:"coulomb" for s in slots}, left_shoulder1_stop=False, self_contact=False),
                dict(name="brake_prone", start_s=2.7, end_s=3.175, contacts={s:"coulomb" for s in prone}, left_shoulder1_stop=False, self_contact=False)],
        tolerances=dict(floor_penetration_m=.0010000000475, self_penetration_m=.0010000000475, initial_joint_limit_rad=3e-6, initial_quaternion_norm=1e-6,
                        constraint_acceptance=1e-5, contact_normal_velocity_mps=1e-4),
        bounds=dict(velocity_abs=[2.]*3+[8.]*3+[10.]*21, acceleration_abs=[10.]*3+[40.]*3+[80.]*21, normal_force_max_weight=2., stop_reaction_max_nm=100.),
        terminal_hold_s=.175, lateral_boundary_s=1.5, lateral_hold_s=.025,
        solver=dict(max_iter=600, max_wall_time_s=1200., tol=1e-5, constr_viol_tol=1e-5, checkpoint_every_iterations=10, dense_output_step_s=.0125),
        acceptance_native=dict(default=1e-6, dynamics_N_or_Nm=.01, position_integration_m=1e-6, orientation_integration=1e-6, hinge_position_integration_rad=1e-6, velocity_integration=1e-6,
            force_separation_Nm=1e-4, self_force_separation_Nm=1e-4, friction_cone_N2=1e-4, loaded_normal_velocity_Nmps=1e-4, self_loaded_normal_velocity_Nmps=1e-4,
            coulomb_maximum_dissipation_Nmps=1e-4, stop_separation_Nmrad=1e-5, loaded_stop_velocity_Nmradps=1e-5),
        objective=dict(torque_squared=1., acceleration_squared=.001, force_squared=.001, torque_rate_squared=.001))


class PureGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        points = [ca.SX.sym(name, 3) for name in ("a0", "a1", "b0", "b1")]
        pa, pb = planner.segment_closest_points(*points)
        cls.function = ca.Function("test_closest", points, [pa, pb, ca.sumsqr(pb-pa)])

    def test_capsules_spheres_parallel_crossing_and_clamping(self):
        cases = [(([0, 0, 0], [1, 0, 0], [.5, -1, 0], [.5, 1, 0]), 0.),
                 (([0, 0, 0], [0, 0, 0], [1, 0, 0], [1, 0, 0]), 1.),
                 (([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]), 1.),
                 (([0, 0, 0], [1, 0, 0], [2, -1, 0], [2, 1, 0]), 1.),
                 (([0, 0, 0], [1, 0, 0], [.5, .3, -1], [.5, .3, 1]), .09)]
        for points, expected in cases:
            self.assertAlmostEqual(float(self.function(*points)[2]), expected, places=12)

    def test_random_distance_is_bracketed_by_independent_dense_grid(self):
        # Pure bounded enumeration, never an optimization-library invocation.
        # Nearest grid parameters differ from the true closest pair by <=1/512
        # in each parameter, giving a rigorous Euclidean distance error bound.
        rng = np.random.default_rng(311)
        parameters = np.linspace(0., 1., 257)
        for _ in range(30):
            a0, a1, b0, b1 = rng.normal(size=(4, 3))
            u, v = a1-a0, b1-b0
            a = a0+parameters[:, None]*u
            b = b0+parameters[:, None]*v
            grid_distance = np.sqrt(np.min(np.sum((a[:, None, :]-b[None, :, :])**2, axis=2)))
            exact = np.sqrt(max(float(self.function(a0, a1, b0, b1)[2]), 0.))
            bound = (np.linalg.norm(u)+np.linalg.norm(v))/512
            self.assertLessEqual(exact, grid_distance+1e-12)
            self.assertGreaterEqual(exact, grid_distance-bound-1e-12)

    def test_quaternion_exponential_zero_derivative_and_finite_rotation(self):
        r = ca.SX.sym("rotation", 3)
        increment = planner.quaternion_increment(r)
        function = ca.Function("test_quaternion", [r], [increment, ca.jacobian(increment, r)])
        value, derivative = function([0., 0., 0.])
        np.testing.assert_allclose(np.asarray(value).ravel(), [0., 0., 0., 1.])
        self.assertTrue(np.isfinite(np.asarray(derivative)).all())
        np.testing.assert_allclose(np.asarray(function([np.pi, 0., 0.])[0]).ravel(), [1., 0., 0., 0.], atol=1e-15)


@unittest.skipUnless(NATIVE_ROWS.exists(), "Retained native rows required; never regenerated")
class BuildOnlyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = build_test_contract()
        # A guard makes accidentally introducing a solve to build fail loudly.
        original = ca.nlpsol
        def forbidden(*args, **kwargs): raise AssertionError("Optimizer construction forbidden in build-only tests")
        ca.nlpsol = forbidden
        try:
            cls.problem = planner.RollTrajectoryProblem(cls.contract)
        finally:
            ca.nlpsol = original

    def test_entire_horizon_mesh_and_constraints_exist(self):
        p = self.problem
        self.assertEqual(p.N, 65)
        self.assertEqual(p.mesh[-1], 3.175)
        self.assertEqual(p.solve_calls, 0)
        categories = {row["category"] for row in p.constraints.rows}
        for name in ("position_integration_m", "orientation_integration", "hinge_position_integration_rad", "velocity_integration", "dynamics_N_or_Nm", "lateral_pelvis_torso_right_side_up", "terminal_pelvis_torso_face_up", "self_force_separation_Nm", "released_existing_self_contact_gap_m"):
            self.assertIn(name, categories)
        for category in ("position_integration_m", "orientation_integration", "hinge_position_integration_rad", "velocity_integration"):
            self.assertEqual(sum(row["category"] == category for row in p.constraints.rows), 64)
        self.assertEqual(sum(row["category"] == "terminal_ground_normal_N" for row in p.constraints.rows), 5)
        self.assertGreater(len(p.collision_pairs), 100)

    def test_exact_seed_and_original_torque_bounds(self):
        p = self.problem
        data, lower, upper = (p.unpack(v) for v in (p.w0, p.lbw, p.ubw))
        np.testing.assert_array_equal(data["x"][:, 0], p.x0)
        np.testing.assert_array_equal(data["v"][:, 0], p.v0)
        np.testing.assert_array_equal(upper["tau"], np.tile(p.dynamics.gears[:, None], (1, p.N)))
        np.testing.assert_array_equal(lower["tau"], -upper["tau"])
        np.testing.assert_array_equal(lower["x"][7:, 1], p.joint_limits[:, 0])
        self.assertTrue(0 < p.initial_compatibility["jointLimitViolationRad"][18] < 3e-6)
        self.assertEqual(float(upper["self_force"][0, 6]), 0.)
        self.assertTrue(np.all(upper["self_force"][0, :6] > 0))
        self.assertEqual(float(upper["stop"][0, 30]), 0.)
        self.assertFalse(p.initial_compatibility["stateModified"])

    def test_initial_audit_finite_and_goal_not_faked(self):
        p = self.problem
        result = p.audit(p.w0)
        self.assertTrue(result["allFinite"])
        self.assertFalse(result["withinFrozenAcceptance"])
        self.assertGreater(result["categories"]["terminal_pelvis_torso_face_up"]["maxNativeUnitViolation"], 1.)
        self.assertLess(p.parity["posePositionMaxErrorM"], 1e-5)

    def test_evolving_self_geometry_is_native_parity_and_momentum_neutral(self):
        p = self.problem
        a, b, normal, gap = [np.asarray(value) for value in p.self_geometry(p.x0)]
        self.assertAlmostEqual(float(gap.ravel()[0]), -.0001764195, delta=2e-6)
        np.testing.assert_allclose(normal.ravel(), [.70990598, .69586575, -.10864575], atol=3e-5)
        slot = p.contract["self_slot"]
        ja = np.asarray(p.dynamics.world_point_jacobian(slot["body_a"])(p.x0, a))[:3]
        jb = np.asarray(p.dynamics.world_point_jacobian(slot["body_b"])(p.x0, b))[:3]
        wrench = (jb-ja).T@normal.ravel()
        np.testing.assert_allclose(wrench[:6], np.zeros(6), atol=1e-14)

    def test_quintic_reference_is_coherent_and_preserves_node_derivatives(self):
        p = self.problem
        rng = np.random.default_rng(37)
        x0, x1 = p.x0.copy(), p.x0.copy()
        x1[:3] += [.02, .01, -.03]
        increment = np.asarray(planner.quaternion_increment(ca.DM([.1, -.06, .04]))).ravel()
        x1[3:7] = np.asarray(planner.quaternion_product(ca.DM(increment), ca.DM(x0[3:7]))).ravel()
        x1[7:] += rng.uniform(-.03, .03, 21)
        v0, v1 = rng.normal(0, .05, (2, 27))
        a0, a1 = rng.normal(0, .1, (2, 27))
        for fraction, expected_x, expected_v, expected_a in ((0., x0, v0, a0), (1., x1, v1, a1)):
            x, v, a, _, consistency = [np.asarray(value).ravel() for value in p.dense_curve(fraction, .05, x0, v0, a0, x1, v1, a1)]
            expected_x = expected_x.copy(); expected_x[3:7] /= np.linalg.norm(expected_x[3:7])
            np.testing.assert_allclose(x, expected_x, atol=2e-13)
            np.testing.assert_allclose(v, expected_v, atol=2e-12)
            np.testing.assert_allclose(a, expected_a, atol=2e-10)
            self.assertLess(max(abs(consistency)), 1e-12)
        fraction, epsilon, h = .37, 1e-6, .05
        x, v, a, xdot, consistency = [np.asarray(value).ravel() for value in p.dense_curve(fraction, h, x0, v0, a0, x1, v1, a1)]
        plus = p.dense_curve(fraction+epsilon, h, x0, v0, a0, x1, v1, a1)
        minus = p.dense_curve(fraction-epsilon, h, x0, v0, a0, x1, v1, a1)
        np.testing.assert_allclose(xdot, (np.asarray(plus[0]).ravel()-np.asarray(minus[0]).ravel())/(2*epsilon*h), atol=2e-8)
        np.testing.assert_allclose(a, (np.asarray(plus[1]).ravel()-np.asarray(minus[1]).ravel())/(2*epsilon*h), atol=2e-6)
        self.assertLess(max(abs(consistency)), 1e-12)

    def test_full_dense_admission_rejects_unoptimized_initial_guess(self):
        p = self.problem
        rows, audit = p.dense_interpolation(p.unpack(p.w0))
        self.assertFalse(audit["withinFrozenAcceptance"])
        self.assertFalse(audit["continuousTimeCertified"])
        self.assertTrue(all(group["allFinite"] for group in audit["categories"].values()))
        self.assertIn("dynamics_N_or_Nm", audit["categories"])
        self.assertIn("force_separation_Nm", audit["categories"])
        self.assertIn("released_existing_self_contact_gap_m", audit["categories"])
        self.assertIn("joint_limit_rad", audit["categories"])
        self.assertGreater(audit["categories"]["terminal_pelvis_torso_face_up"]["maxNativeUnitViolation"], 1.)
        np.testing.assert_array_equal(rows[0]["x"], p.x0)
        np.testing.assert_array_equal(rows[0]["v"], p.v0)
        self.assertLess(audit["categories"]["dense_configuration_rate_consistency"]["maxNativeUnitViolation"], 1e-6)

    def test_mesh_and_budget_changes_do_not_autocorrect(self):
        broken = copy.deepcopy(self.contract)
        broken["mesh_s"][-1] = 3.2
        with self.assertRaisesRegex(ValueError, "mesh_s"):
            planner.validate_contract(broken, self.problem.dynamics)
        broken = copy.deepcopy(self.contract)
        broken["solver"]["max_wall_time_s"] = 1201
        with self.assertRaisesRegex(ValueError, "1200"):
            planner.validate_contract(broken, self.problem.dynamics)


if __name__ == "__main__":
    unittest.main(verbosity=2)
