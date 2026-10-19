"""Native M*qdd+b for a declared path, on a counter-zero scratch only.

The reference root is a derived kinematic value. It is installed only in the
caller-owned scratch, never in a running world. This module never integrates,
optimizes, invents future contacts, or admits a gait.
"""
import ctypes as C
import math

import numpy as np
from scipy.spatial.transform import Rotation

from coupled_posture_kinematics import MatrixJet, P, pose_jet_to_source


VERSION = 'counter-zero-native-planned-jets-v1'


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def generalized_planned_jets(geometry):
    """Original native free-joint angular coordinates are torso-local."""
    q, jv, ja = geometry.full_joints.arrays()
    root = geometry.root_model.T
    if (any(a.shape != (21,) for a in (q, jv, ja)) or root.shape != (4, 4)
            or not all(np.isfinite(a).all() for a in (q, jv, ja, root))):
        raise ValueError('finite full21 planned geometry required')
    source = pose_jet_to_source(geometry.root_model)
    rotation = P @ root[:3, :3] @ P.T
    quaternion = Rotation.from_matrix(rotation).as_quat()
    qpos = np.r_[P @ root[:3, 3], quaternion[[3, 0, 1, 2]], q]
    velocity = np.r_[source.translation.velocity,
                     rotation.T @ np.asarray(source.angular_velocity_world), jv]
    acceleration = np.r_[source.translation.acceleration,
                         rotation.T @ np.asarray(source.angular_acceleration_world), ja]
    if not all(np.isfinite(a).all() for a in (qpos, velocity, acceleration)):
        raise ValueError('nonfinite planned generalized jets')
    return qpos, velocity, acceleration


def jacobian_at_fixed_origin(body_jacobian, body_origin, fixed_origin):
    """Translate the WORLD spatial Jacobian without rotating its axes."""
    jacobian = np.asarray(body_jacobian, dtype=float)
    if jacobian.shape != (6, 27):
        raise ValueError('native6x27 spatial Jacobian required')
    offset = np.asarray(fixed_origin, dtype=float) - np.asarray(body_origin, dtype=float)
    result = jacobian.copy()
    result[:3] -= skew(offset) @ jacobian[3:]
    if not np.isfinite(result).all():
        raise ValueError('nonfinite fixed-origin Jacobian')
    return result


class NativePlannedJetProvider:
    """Read-only-to-actual-world native provider; no ownership of world lifetime."""
    def __init__(self, world, contacts, fixed_origin_source, expected_stance_pose_model):
        from source_step_probe import ptr
        from source_posture_retarget import _actuators

        self.w, self.ptr, self.actuators = world, ptr, _actuators
        self.contacts = tuple((int(side), tuple(np.asarray(point, dtype=float)))
                              for side, point in contacts)
        if (len(self.contacts) != 3 or len({side for side, _ in self.contacts}) != 1
                or any(side not in (0, 1) or len(point) != 3
                       or not np.isfinite(point).all() for side, point in self.contacts)
                or len(set(self.contacts)) != 3):
            raise ValueError('three unique actual contacts on the same stance foot required')
        self.stance_side = self.contacts[0][0]
        self.stance_name = ('left_foot', 'right_foot')[self.stance_side]
        self.origin = np.array(fixed_origin_source, dtype=float, copy=True)
        self.stance_pose = np.array(expected_stance_pose_model, dtype=float, copy=True)
        if (self.origin.shape != (3,) or self.stance_pose.shape != (4, 4)
                or not np.isfinite(self.origin).all() or not np.isfinite(self.stance_pose).all()):
            raise ValueError('finite fixed source origin and actual stance transform required')
        MatrixJet.rest(self.stance_pose)  # Exact original rigid-frame validation.
        self.origin.setflags(write=False)
        self.stance_pose.setflags(write=False)
        self.force_scale = float(world.mass * 9.81)
        if not math.isfinite(self.force_scale) or self.force_scale <= 0.:
            raise ValueError('positive original body weight required')
        self.calls = 0
        self._counter_zero()

    def _counter_zero(self):
        if self.w.l.lh_dim(self.w.h, 6) != 0:
            raise ValueError('planned-jet provider requires counter-zero scratch')

    def __call__(self, sample, *, final=False):
        w = self.w
        self._counter_zero()
        geometry = sample.geometric
        stance = geometry.pose(self.stance_name)
        stance_error = float(np.max(abs(stance.T - self.stance_pose)))
        stance_jet_error = max(float(np.max(abs(value)))
                               for value in (stance.Tdot, stance.Tddot))
        if max(stance_error, stance_jet_error) > 1e-8:
            raise ValueError('fixed actual rigid stance pose/jets changed')
        qpos, velocity, acceleration = generalized_planned_jets(geometry)
        w.install_cold(dict(qpos=qpos.tolist(), qvel=velocity.tolist()))
        self.calls += 1
        installed_velocity = np.asarray(w.arr(1, (27,)))
        if (not np.isfinite(installed_velocity).all()
                or np.max(abs(installed_velocity - velocity)) > 1e-12):
            raise ValueError('native installed planned velocity differs')
        mass = w.read('lh_mass_matrix', (27, 27))
        bias = w.read('lh_bias', (27,))
        B = self.actuators(w)
        positions, rotations = w.arr(3, (17, 3)), w.arr(5, (17, 3, 3))
        com = w.arr(6, (17, 3))[1].copy()
        if not all(np.isfinite(a).all() for a in (mass, bias, B, positions, rotations, com)):
            raise ValueError('nonfinite native inverse-dynamics data')
        body = w.ids[self.stance_name]
        J = w.read('lh_body_jac', (6, 27), body)
        fixed_jacobian = jacobian_at_fixed_origin(J, positions[body], self.origin)
        G = np.hstack([w.read('lh_point_jac', (3, 27), body,
                             self.ptr(np.ascontiguousarray(point, dtype=float))).T
                       for _, point in self.contacts])
        demand = mass @ acceleration + bias
        if not np.isfinite(G).all() or not np.isfinite(demand).all():
            raise ValueError('nonfinite native point forces or planned demand')
        errors, body_identifiers = [], set()
        for name, pose in geometry.poses_model:
            identifier = w.l.lh_id(w.h, 1, name.encode())
            if identifier < 1:
                if not name.startswith('@'):
                    raise ValueError('unmapped physical native body: ' + name)
                continue
            if identifier in body_identifiers:
                raise ValueError('duplicate physical native body parity row')
            body_identifiers.add(identifier)
            source = pose_jet_to_source(pose)
            spatial = w.read('lh_body_jac', (6, 27), identifier)
            row = dict(body=name,
                       positionErrorM=float(np.max(abs(positions[identifier] - P @ pose.T[:3, 3]))),
                       rotationMatrixError=float(np.max(abs(rotations[identifier] - P @ pose.T[:3, :3] @ P.T))),
                       velocityError=float(np.max(abs(spatial @ velocity - np.r_[source.translation.velocity, source.angular_velocity_world]))))
            if final:
                directional = w.read('lh_jdot', (6,), identifier)
                row['accelerationDiagnosticError'] = float(np.max(abs(
                    spatial @ acceleration + directional
                    - np.r_[source.translation.acceleration, source.angular_acceleration_world])))
                if not math.isfinite(row['accelerationDiagnosticError']):
                    raise ValueError('nonfinite native directional acceleration diagnostic')
            errors.append(row)
        if body_identifiers != set(range(1, 17)):
            raise ValueError('complete original16 physical-body parity required')
        com_jacobian = w.read('lh_com_jac', (3, 27))
        com_position_error = float(np.max(abs(com - np.asarray(geometry.com_source.position))))
        com_velocity_error = float(np.max(abs(com_jacobian @ velocity - np.asarray(geometry.com_source.velocity))))
        parity = [value for row in errors for key, value in row.items()
                  if key in ('positionErrorM', 'rotationMatrixError', 'velocityError')]
        parity.extend((com_position_error, com_velocity_error))
        if not np.isfinite(parity).all() or max(parity) > 1e-8:
            raise ValueError('native/source planned body or COM parity failed')
        w.l.lh_self_separation.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
        w.l.lh_self_separation.restype = None
        raw = w.read('lh_self_separation', (897,))
        count = int(raw[0])
        if not np.isfinite(raw).all() or not 1 <= count <= 64 or count != raw[0]:
            raise ValueError('original clean self-separation pairs absent')
        gap = float(min(raw[1:1 + 14 * count].reshape(-1, 14)[:, 4]))
        receipt = dict(version=VERSION, nativeCounter=0,
                       rootAngularConvention='native torso-local',
                       massTimesPlannedAccelerationPlusOriginalBias=True,
                       biasIncludesOriginalPassive=True,
                       fixedOriginSource=self.origin.tolist(),
                       actualContacts=[dict(footSide=s, pointSource=list(p)) for s, p in self.contacts],
                       maximumStancePoseError=stance_error, maximumStanceJetError=stance_jet_error,
                       perPhysicalBodyParity=errors, nativeCOMErrorM=com_position_error,
                       nativeCOMVelocityError=com_velocity_error, nativeCOMHeightM=float(com[2]),
                       minimumCleanSelfGapM=gap, generalizedVelocity=velocity.tolist(),
                       generalizedAcceleration=acceleration.tolist(), physicsIntegrations=0,
                       actualWorldStateWrites=0, nativeAdmission=False,
                       sourceParityUsesGeometricJetsNotCompatibilityCommands=True,
                       all21TasksExactlyCompatibleClaim=False,
                       initialCompatibilitySlackActive=bool(sample.diagnostics.get('initialCompatibilitySlackActive')),
                       commandGeometricCOMPositionMismatchM=float(np.max(abs(
                           np.asarray(sample.com.position) - np.asarray(geometry.com_source.position)))),
                       commandGeometricCOMVelocityMismatchMps=float(np.max(abs(
                           np.asarray(sample.com.velocity) - np.asarray(geometry.com_source.velocity)))),
                       commandGeometricCOMAccelerationMismatchMps2=float(np.max(abs(
                           np.asarray(sample.com.acceleration) - np.asarray(geometry.com_source.acceleration)))))
        if final:
            receipt['nativeCOMAccelerationDiagnosticError'] = float(np.max(abs(
                com_jacobian @ acceleration + w.read('lh_jdot', (3,), -1)
                - np.asarray(geometry.com_source.acceleration))))
            if not math.isfinite(receipt['nativeCOMAccelerationDiagnosticError']):
                raise ValueError('nonfinite native COM acceleration diagnostic')
            receipt['directionalJdotDifferenceStepS'] = 1e-5
            receipt['finiteDifferenceAccelerationParityIsDiagnosticOnly'] = True
        self._counter_zero()
        return dict(B=B, G=G, demand=demand, stanceJacobian=fixed_jacobian,
                    referenceOriginSource=self.origin.tolist(),
                    forceScaleN=self.force_scale, minimumCleanSelfGapM=gap,
                    nativeCOMHeightM=float(com[2]), native=receipt)
