"""Complete analytic planning samples using the existing plain-triple helper.

This is a thin, pure materializer around AnalyticCOMGeometry, not a reference
owner or a certificate. All positional Newton/implicit derivatives, map policy
and matrix arithmetic stay in existing implementations. The original strict
sampler remains the independent final/runtime authority.
"""
import numpy as np

from analytic_com_geometry import AnalyticCOMGeometry, _source_spatial
from coupled_posture_kinematics import FKResult, MatrixJet, P, frozen
from coupled_posture_reference import Jet, OrientationJet, angular_chart, angular_world
from wholebody_cycle_reference import CycleSample, add

VERSION = 'complete-analytic-native-planning-sample-v1'


class AnalyticNativePlanningSampler:
    """No stored pose, IK seed, native data, force witness or live state."""
    def __init__(self, realizer):
        self.geometry = AnalyticCOMGeometry(realizer)

    def sample(self, path, phase, elapsed, *, runtime_arms=None,
               landing_adjustment=None):
        fast = self.geometry.sample(path, phase, elapsed,
                                    runtime_arms=runtime_arms,
                                    landing_adjustment=landing_adjustment)
        # The existing helper already solved every lower/full/map jet. A cheap
        # repeat of ONLY its existing transform operators materializes the poses
        # it intentionally omits from its minimal COM return type.
        root_values = self.geometry._root(fast.lower_joints.arrays(), path.realizer)
        values = self.geometry._forward(fast.geometric.full_joints.arrays(), root_values)
        poses = {name: MatrixJet(*pose) for name, pose in values.items()}
        com = [np.zeros(3), np.zeros(3), np.zeros(3)]
        for name, _, _, _, _, mass, local_com in self.geometry._parts:
            for order, matrix in enumerate(values[name]):
                com[order] += mass * (P @ (matrix @ local_com)[:3]) / self.geometry.model.total_mass

        def orientation(name, body):
            _, omega, alpha = _source_spatial(values[name])
            rotation = P @ values[name][0][:3, :3] @ P.T
            return OrientationJet(body, frozen(rotation), frozen(omega), frozen(alpha))

        result = FKResult(fast.geometric.full_joints,
                          fast.geometric.future_thigh_right_left,
                          fast.geometric.future_arm_delta_right_left,
                          Jet(*com), orientation('torso', 1), orientation('pelvis', 4),
                          poses['torso'], tuple(poses.items()))
        k, t, total = path.clock(phase, elapsed)
        R, w, a = angular_world(add(angular_chart(*result.thorax.arrays()),
                                    path.body_slack.at(total)))
        thorax = OrientationJet(1, frozen(R), frozen(w), frozen(a))
        R, w, a = angular_world(add(angular_chart(*result.pelvis.arrays()),
                                    path.pelvis_slack.at(total)))
        pelvis = OrientationJet(4, frozen(R), frozen(w), frozen(a))
        feet = tuple(path.command_foot(side, k, t, total, landing_adjustment)
                     for side in (0, 1))
        return CycleSample(fast.lower_joints, fast.com, thorax, pelvis, feet,
                           result, fast.diagnostics)
