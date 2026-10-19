"""Pure analytic WholeBodyCycle COM geometry, with no admission authority.

Inputs are already validated by the original immutable path/model constructors.
This evaluator only removes repeated MatrixJet validation, repeated conversion
to immutable pose receipts, and FK branches that cannot affect the requested
quantity. Position Newton and its fixed seed/tolerance/cap are the ORIGINAL
StanceRealizedIK.position. Its spatial Jacobian is also the original operation.
The original strict WholeBodyCycle.sample remains the independent final gate.

No optimizer, integration, state installation, clipping, extrapolation, mutable
IK seed, finite-difference derivative, or model/geometry/cap change is present.
"""
from dataclasses import dataclass
import copy
import math
import numpy as np

from coupled_posture_reference import Jet
from coupled_posture_kinematics import P
from wholebody_cycle_reference import (
    FEET, add, as_jet, compose_pose, pose_chart, subtract,
)
from coupled_posture_kinematics import pose_jet_to_source

VERSION = 'plain-triple-analytic-com-geometry-v2-sample-fk-reuse'


class _SampleFKMemo:
    """Exact, bounded positional-FK reuse inside ONE sample only.

    Original Newton and original spatial Jacobian request identical complete
    q/root transforms successively. Full float64 byte keys retain signed zero
    and every coordinate. No value is rounded, no IK solution is retained, and
    the original kinematics performs every cache miss with unchanged arithmetic.
    The cache is discarded as soon as sample() returns or raises.
    """
    def __init__(self, kinematics, capacity=64):
        self.original = kinematics
        self.capacity = capacity
        self.cache = {}
        self.requests = self.hits = self.misses = self.evictions = 0

    def __getattr__(self, name):
        return getattr(self.original, name)

    def forward(self, q, root):
        key = (np.asarray(q, dtype=np.float64).tobytes(),
               np.asarray(root, dtype=np.float64).tobytes())
        self.requests += 1
        if key in self.cache:
            self.hits += 1
            return self.cache[key]
        self.misses += 1
        result = self.original.forward(q, root)
        self.cache[key] = result
        if len(self.cache) > self.capacity:
            self.cache.pop(next(iter(self.cache)))
            self.evictions += 1
        return result


def _sample_realizer(realizer):
    """Shallow local adapters; ORIGINAL position/Jacobian bodies are unchanged."""
    memo = _SampleFKMemo(realizer.kin)
    local = copy.copy(realizer)
    local.kin = memo
    local.fk = copy.copy(realizer.fk)
    local.fk.kin = memo
    return local


def _readonly(value):
    value = np.array(value, dtype=float, copy=True)
    value.setflags(write=False)
    return value


def _multiply(left, right):
    # Exact MatrixJet.__matmul__ operation and arithmetic order.
    a, b, c = left
    x, y, z = right
    return a @ x, b @ x + a @ y, c @ x + 2 * b @ y + a @ z


def _inverse(value):
    # Exact MatrixJet.inverse operation, including a general matrix inverse.
    a, b, c = value
    x = np.linalg.inv(a)
    return x, -x @ b @ x, 2 * x @ b @ x @ b @ x - x @ c @ x


def _hinge(angle, velocity, acceleration):
    c, s = math.cos(angle), math.sin(angle)
    R = np.eye(4)
    D = np.zeros((4, 4))
    H = np.zeros((4, 4))
    R[1:3, 1:3] = [[c, -s], [s, c]]
    D[1:3, 1:3] = [[-s, -c], [c, -s]]
    H[1:3, 1:3] = [[-c, s], [-s, -c]]
    return R, D * velocity, H * velocity ** 2 + D * acceleration


def _source_spatial(value):
    """Original pose_jet_to_source arithmetic without any pose/Jet receipt."""
    T, D, H = value
    R, dR, ddR = [P @ x[:3, :3] @ P.T for x in (T, D, H)]
    W = dR @ np.linalg.inv(R)
    A = ddR @ np.linalg.inv(R) - W @ W
    def vee(K):
        return np.array([K[2, 1] - K[1, 2], K[0, 2] - K[2, 0],
                         K[1, 0] - K[0, 1]]) / 2.
    return tuple(P @ x[:3, 3] for x in (T, D, H)), vee(W), vee(A)


@dataclass(frozen=True)
class _ArrayPose:
    """Read-only structural adapter accepted by original FKJets Jacobian."""
    values: tuple

    def arrays(self):
        return self.values


@dataclass(frozen=True)
class COMGeometry:
    full_joints: Jet
    future_thigh_right_left: Jet
    future_arm_delta_right_left: Jet


@dataclass(frozen=True)
class COMSample:
    lower_joints: Jet
    com: Jet
    geometric: COMGeometry
    diagnostics: dict


class AnalyticCOMGeometry:
    """Reuse immutable static transforms, never a previous IK solution.

    Construct once from the path realizer. Copies of that realizer may change
    only their caller-owned arm-map policy: sample always uses path.realizer's
    map, preserving both the original strict map and the explicit optimizer
    raw-domain proxy. It does not introduce or loosen either map policy.
    """
    def __init__(self, realizer):
        self.model = realizer.model
        self.stance_side = realizer.stance_side
        self.swing_side = realizer.swing_side
        self._rest = (_readonly(np.eye(4)), _readonly(np.zeros((4, 4))),
                      _readonly(np.zeros((4, 4))))
        self._parts = tuple((p.name, p.parent, p.qindex,
                             tuple(_readonly(x) for x in p.parent_frame.arrays()),
                             tuple(_readonly(x) for x in p.child_inverse.arrays()),
                             p.mass, _readonly(np.r_[p.com, 1.]))
                            for p in self.model.parts)
        self._by_name = {p[0]: p for p in self._parts}
        self._stance_parts = self._ancestors((FEET[realizer.stance_side],))
        self._swing_parts = self._ancestors((FEET[realizer.swing_side],))
        self._thigh_parts = self._ancestors(
            ('right_thigh', 'right_shin', 'left_thigh', 'left_shin'))

    def _ancestors(self, names):
        selected = set()
        for name in names:
            while name and name != 'torso':
                selected.add(name)
                name = self._by_name[name][1]
        return tuple(p for p in self._parts[1:] if p[0] in selected)

    def _forward(self, joints, root=None, parts=None):
        q, v, a = joints
        out = {'torso': self._rest if root is None else root}
        for name, parent, index, parent_frame, child_inverse, _, _ in (
                self._parts[1:] if parts is None else parts):
            hinge = self._rest if index is None else _hinge(q[index], v[index], a[index])
            # Preserve all three left-associated products, including zeros.
            out[name] = _multiply(_multiply(_multiply(out[parent], parent_frame),
                                           hinge), child_inverse)
        return out

    def _root(self, lower, realizer):
        full = tuple(np.r_[x, np.zeros(6)] for x in lower)
        relative = self._forward(full, parts=self._stance_parts)
        return _multiply(realizer.stance_pose.arrays(),
                         _inverse(relative[FEET[realizer.stance_side]]))

    def _thigh(self, lower):
        full = tuple(np.r_[x, np.zeros(6)] for x in lower)
        poses = self._forward(full, parts=self._thigh_parts)
        values = [[], [], []]
        for side in ('right', 'left'):
            thigh, shin = poses[side + '_thigh'], poses[side + '_shin']
            d, dd, ddd = [P @ (b[:3, 3] - a[:3, 3]) for a, b in zip(thigh, shin)]
            x, z = d[[0, 2]]
            dx, dz = dd[[0, 2]]
            ddx, ddz = ddd[[0, 2]]
            den = x * x + z * z
            if den < 1e-12:
                raise ValueError('future thigh pitch singular')
            numerator = -z * dx + x * dz
            values[0].append(math.atan2(x, -z))
            values[1].append(numerator / den)
            values[2].append((-z * ddx + x * ddz) / den -
                             numerator * 2 * (x * dx + z * dz) / den ** 2)
        return Jet(*values)

    def sample(self, path, phase, elapsed, *, runtime_arms=None,
               landing_adjustment=None):
        realizer = path.realizer
        if (realizer.model is not self.model or realizer.stance_side != self.stance_side
                or realizer.swing_side != self.swing_side):
            raise ValueError('analytic COM geometry requires its original model identity')
        k, t, total = path.clock(phase, elapsed)
        # Preserve the original foot command/chart/slack path byte for byte.
        command = path.command_foot(path.swing_side, k, t, total, landing_adjustment)
        tr = pose_jet_to_source(command)
        st, sr = path.foot_slacks[path.swing_side]
        target_pose = compose_pose(subtract(tr.translation, st.at(total)),
                                  subtract(pose_chart(command), sr.at(total)))
        ip, iv, ia = path.independent_segments[k].at(t).arrays()
        # The same positional FK is needed for the original Newton residual
        # and its original Jacobian. Share exact results within this call only.
        local_realizer = _sample_realizer(realizer)
        q, receipt = local_realizer.position(ip, target_pose, path.lower0.position)
        v = np.zeros(15)
        a = np.zeros(15)
        v[realizer.independent_indices] = iv
        a[realizer.independent_indices] = ia
        root = self._root((q, v, a), realizer)
        full_zero_arms = lambda: tuple(np.r_[x, np.zeros(6)] for x in (q, v, a))
        foot = FEET[realizer.swing_side]
        bias = self._forward(full_zero_arms(), root, self._swing_parts)[foot]
        J = local_realizer.fk.spatial_jacobian(np.r_[q, np.zeros(6)], _ArrayPose(root),
                                             foot, realizer.swing_indices)
        target, target_w, target_a = _source_spatial(target_pose.arrays())
        current, current_w, _ = _source_spatial(bias)
        rhs = np.r_[P.T @ (target[1] - current[1]), P.T @ (target_w - current_w)]
        v[realizer.swing_indices] = np.linalg.solve(J, rhs)
        # Root is independent of swing/arm joints. Reusing this identical triple
        # avoids a redundant stance-chain evaluation without changing a bit.
        bias2 = self._forward(full_zero_arms(), root, self._swing_parts)[foot]
        current2, _, current2_a = _source_spatial(bias2)
        rhs2 = np.r_[P.T @ (target[2] - current2[2]), P.T @ (target_a - current2_a)]
        a[realizer.swing_indices] = np.linalg.solve(J, rhs2)
        lower = Jet(q, v, a)
        theta = self._thigh((q, v, a))
        tp, tv, ta = theta.arrays()
        delta = Jet(-(tp - np.array(realizer.leg_origin)), -tv, -ta)
        mapped = realizer.arm_map.evaluate(delta)
        arms = (add(mapped, path.arm_slack.at(total)) if runtime_arms is None
                else as_jet(runtime_arms))
        full = Jet(*(np.r_[x, y] for x, y in zip(lower.arrays(), arms.arrays())))
        poses = self._forward(full.arrays(), root)
        com = [np.zeros(3), np.zeros(3), np.zeros(3)]
        for name, _, _, _, _, mass, local_com in self._parts:
            for order, M in enumerate(poses[name]):
                com[order] += mass * (P @ (M @ local_com)[:3]) / self.model.total_mass
        actual, actual_w, actual_a = _source_spatial(poses[foot])
        receipt.update(
            maximumVelocityResidual=float(max(abs(np.r_[actual[1] - target[1], actual_w - target_w]))),
            maximumAccelerationResidual=float(max(abs(np.r_[actual[2] - target[2], actual_a - target_a]))))
        diagnostics = dict(**receipt,
            armConditioning=('predicted future map plus initial bridge' if runtime_arms is None
                             else 'exact prepared measured delayed arms'),
            runtimeArmsReplaced=False, all21TasksExactlyCompatibleClaim=False,
            initialCompatibilitySlackActive=total < path.durations[0],
            phase=phase, phaseTimeS=t)
        return COMSample(lower, add(Jet(*com), path.com_slack.at(total)),
                         COMGeometry(full, theta, delta), diagnostics)
