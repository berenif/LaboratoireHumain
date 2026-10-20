#!/usr/bin/env python3
"""Pure symbolic, floating-base GetUp dynamics; never steps or edits physics.

Configuration x = [free-origin XYZ, quaternion XYZW, 21 hinge qpos] (28).
Velocity v = [free-origin world XYZ, world angular XYZ, 21 hinge qvel] (27).
Acceleration is the ordinary time derivative of these velocity coordinates.
All spatial vectors/Jacobians use [world linear; world angular] ordering.

Equation: M(x) a + bias(x,v) = S tau_Nm + sum(Jpoint_linear.T f_world) + stops.
tau_Nm is PHYSICAL hinge torque in QPOS order. The equation applies no gear.
No contact model, stop model, integrator, optimizer, or trial runner lives here.
"""
import argparse
import json
from pathlib import Path

import casadi as ca
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "rust/crates/model/data/getup-v1.json"


def xyz(value):
    return np.array([value[key] for key in "xyz"], dtype=float)


def skew(value):
    x, y, z = value[0], value[1], value[2]
    return ca.vertcat(ca.horzcat(0, -z, y), ca.horzcat(z, 0, -x), ca.horzcat(-y, x, 0))


def quaternion_rotation(quaternion):
    """XY ZW quaternion, normalized symbolically. The zero quaternion is invalid."""
    q = quaternion / ca.sqrt(ca.dot(quaternion, quaternion))
    x, y, z, w = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        ca.horzcat(2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        ca.horzcat(2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )


def frame_transform(frame):
    q = ca.DM([frame["rotation"][key] for key in "xyzw"])
    return ca.vertcat(
        ca.horzcat(quaternion_rotation(q), ca.DM(xyz(frame["anchor"]))),
        ca.DM([[0, 0, 0, 1]]),
    )


def inverse_frame(transform):
    rotation = transform[:3, :3]
    return ca.vertcat(ca.horzcat(rotation.T, -rotation.T @ transform[:3, 3]), ca.DM([[0, 0, 0, 1]]))


class RollDynamics:
    """Small reusable CasADi Function bundle for the current 27-DOF source body.

    poses(x): 4 x (4*25), with a homogeneous pose block in source segment order.
    body_kinematics(x,v): COM positions 3x25, COM spatial Jacobians (6*25)x27,
      COM twists 6x25, and material COM Jdot*v 6x25, including massless frames.
    com(x,v): whole-body COM, Jcom, velocity, and Jdot*v.
    dynamics(x,v): M27x27 and bias27. mass_matrix and bias are also separate.
    primitive_endpoints(x,floor_y): endpoints 3xN and vertical sphere/capsule
      endpoint clearance N. Endpoint order/radii are in primitive_metadata.
    point_function(body)(x,v,local_point): body-fixed material point position,
      spatial Jacobian, twist, and Jdot*v. local_point is constant in its body.
    world_point_jacobian(body)(x,world_point): instantaneous spatial Jacobian.
      This deliberately supplies no Jdot for a geometry-selected moving point.
    """

    def __init__(self, model=None, gravity=(0.0, -9.81, 0.0)):
        self.model = json.loads(MODEL_PATH.read_text()) if model is None else model
        self.parts = self.model["segments"]
        self.body_ids = tuple(part["id"] for part in self.parts)
        self.body_index = {name: index for index, name in enumerate(self.body_ids)}
        self.massive_indices = tuple(i for i, part in enumerate(self.parts) if part["massKg"] > 0)
        self.mass = sum(part["massKg"] for part in self.parts)
        if len(self.parts) != 25 or len(self.massive_indices) != 16:
            raise ValueError("Expected the current 25-segment, 16-massive-body GetUp model")
        self.x = x = ca.SX.sym("x", 28)
        self.v = v = ca.SX.sym("v", 27)
        self.S = ca.DM.zeros(27, 21)
        self.S[6:, :] = ca.DM.eye(21)
        self.gravity = ca.DM(gravity)
        self.armature = np.zeros(27)
        self.damping = np.zeros(27)
        self.spring = np.zeros(27)
        self.limits = np.zeros((21, 2))
        self.gears = np.zeros(21)
        self.qpos_to_actuator = np.zeros(21, dtype=int)
        self.joint_names = [None] * 21
        self._poses = {}
        self._ancestors = {}
        self._axes = {}
        self._pivots = {}
        self._point_functions = {}
        self._world_point_functions = {}

        # qdot = .5*[world_omega,0] * q. Normalization in FK is invariant under
        # the input quaternion's nonzero scale; qdot preserves that scale.
        quat = x[3:7]
        qdot = .5 * ca.vertcat(quat[3]*v[3:6] + ca.cross(v[3:6], quat[:3]), -ca.dot(v[3:6], quat[:3]))
        self.xdot = ca.vertcat(v[:3], qdot, v[6:])
        self.configuration_rate = ca.Function("roll_configuration_rate", [x, v],
            [self.xdot, ca.dot(quat, quat)-1], ["x", "v"], ["xdot", "unit_quaternion_error"])
        root = self.parts[0]
        if root["parent"] is not None:
            raise ValueError("The first source body must be the free root")
        self._poses[root["id"]] = ca.vertcat(ca.horzcat(quaternion_rotation(quat), x[:3]), ca.DM([[0, 0, 0, 1]]))
        self._ancestors[root["id"]] = []
        found_qpos = []
        for part in self.parts[1:]:
            name, parent = part["id"], part["parent"]
            if parent not in self._poses:
                raise ValueError("Source segments must be in topological order")
            profile = part["jointProfile"]
            parent_frame = frame_transform(profile["parentFrame"])
            child_inverse = inverse_frame(frame_transform(profile["childFrame"]))
            joint_world = self._poses[parent] @ parent_frame
            relative = ca.SX.eye(4)
            ancestors = self._ancestors[parent].copy()
            hinge = part["reference"].get("hinge")
            if hinge:
                index = hinge["qposIndex"]
                if profile["axes"][0]["coordinate"] != "x":
                    raise ValueError("Expected source hinge motion about joint-frame X")
                found_qpos.append(index)
                ancestors.append(index)
                self._axes[index] = joint_world[:3, 0]
                self._pivots[index] = joint_world[:3, 3]
                c, s = ca.cos(x[7+index]), ca.sin(x[7+index])
                relative[1:3, 1:3] = ca.vertcat(ca.horzcat(c, -s), ca.horzcat(s, c))
                axis = profile["axes"][0]
                self.armature[6+index] = hinge["armature"]
                self.damping[6+index] = axis["dampingNmsPerRad"]
                self.spring[6+index] = axis["passiveStiffnessNmPerRad"]
                self.limits[index] = axis["minRadians"], axis["maxRadians"]
                self.gears[index] = hinge["gear"]
                self.qpos_to_actuator[index] = hinge["actuatorIndex"]
                self.joint_names[index] = hinge["name"]
            self._poses[name] = joint_world @ relative @ child_inverse
            self._ancestors[name] = ancestors
        if sorted(found_qpos) != list(range(21)) or sorted(self.qpos_to_actuator) != list(range(21)):
            raise ValueError("Expected one-to-one 21 hinge qpos/actuator mappings")
        # Keep both representations explicit. The real engine loads axis limits
        # as f32; neither representation justifies editing a measured qpos.
        self.native_limits = self.limits.astype(np.float32).astype(float)

        mass_matrix = ca.SX.zeros(27, 27)
        bias = ca.SX.zeros(27)
        com = ca.SX.zeros(3)
        jcom = ca.SX.zeros(3, 27)
        com_bias = ca.SX.zeros(3)
        centers, body_jacobians, twists, body_biases = [], [], [], []
        for part in self.parts:
            pose = self._poses[part["id"]]
            center = pose[:3, 3] + pose[:3, :3] @ ca.DM(xyz(part["goldenMass"]["centerOfMass"]))
            jacobian = self._point_jacobian(part["id"], center)
            twist = jacobian @ v
            acceleration_bias = ca.jtimes(twist, x, self.xdot)
            centers.append(center)
            body_jacobians.append(jacobian)
            twists.append(twist)
            body_biases.append(acceleration_bias)
            mass = part["massKg"]
            if mass == 0:
                continue
            inertia = pose[:3, :3] @ ca.DM(part["goldenMass"]["inertia"]) @ pose[:3, :3].T
            jv, jw = jacobian[:3, :], jacobian[3:, :]
            mass_matrix += mass*jv.T @ jv + jw.T @ inertia @ jw
            bias += jv.T @ (mass*(acceleration_bias[:3]-self.gravity))
            bias += jw.T @ (inertia @ acceleration_bias[3:] + ca.cross(twist[3:], inertia @ twist[3:]))
            com += mass*center
            jcom += mass*jv
            com_bias += mass*acceleration_bias[:3]
        mass_matrix += ca.diag(ca.DM(self.armature))
        passive = ca.vertcat(ca.SX.zeros(6), ca.DM(self.spring[6:])*x[7:] + ca.DM(self.damping[6:])*v[6:])
        bias += passive
        self.poses = ca.Function("roll_poses", [x], [ca.horzcat(*[self._poses[name] for name in self.body_ids])], ["x"], ["poses"])
        self.body_kinematics = ca.Function("roll_body_kinematics", [x, v],
            [ca.horzcat(*centers), ca.vertcat(*body_jacobians), ca.horzcat(*twists), ca.horzcat(*body_biases)],
            ["x", "v"], ["centers", "spatial_jacobians", "twists", "jdot_v"])
        self.com = ca.Function("roll_com", [x, v], [com/self.mass, jcom/self.mass, jcom@v/self.mass, com_bias/self.mass],
            ["x", "v"], ["position", "jacobian", "velocity", "jdot_v"])
        self.mass_matrix = ca.Function("roll_mass_matrix", [x], [mass_matrix], ["x"], ["M"])
        self.bias = ca.Function("roll_bias", [x, v], [bias], ["x", "v"], ["bias"])
        self.passive = ca.Function("roll_passive", [x, v], [passive], ["x", "v"], ["passive"])
        self.dynamics = ca.Function("roll_dynamics", [x, v], [mass_matrix, bias], ["x", "v"], ["M", "bias"])
        controls = ca.SX.sym("controls_actuator_order", 21)
        self.controls_to_torque = ca.Function("roll_controls_to_physical_torque", [controls],
            [ca.vertcat(*[self.gears[i]*controls[int(self.qpos_to_actuator[i])] for i in range(21)])],
            ["controls_actuator_order"], ["physical_torque_qpos_order"])

        endpoints, gaps = [], []
        floor_y = ca.SX.sym("floor_y")
        self.primitive_metadata = []
        for part in self.parts:
            pose = self._poses[part["id"]]
            for primitive in part["reference"]["primitives"]:
                for end in ("a", "b"):
                    endpoint = pose[:3, 3] + pose[:3, :3] @ ca.DM(xyz(primitive[end]))
                    endpoints.append(endpoint)
                    gaps.append(endpoint[1]-primitive["radius"]-floor_y)
                    self.primitive_metadata.append(dict(body=part["id"], name=primitive["name"],
                        kind=primitive["kind"], end=end, local=xyz(primitive[end]).tolist(), radius=primitive["radius"]))
        self.primitive_endpoints = ca.Function("roll_primitive_endpoints", [x, floor_y],
            [ca.horzcat(*endpoints), ca.vertcat(*gaps)], ["x", "floor_y"], ["positions", "floor_gaps"])
        self.metadata = dict(configuration="root_origin_xyz,quaternion_xyzw,21_qpos", velocity="root_origin_world_xyz,world_angular_xyz,21_qvel",
            spatial_order="world_linear,world_angular", torque_order="physical_Nm_in_qpos_order", gear_application="controls_to_torque_only",
            body_ids=list(self.body_ids), massive_body_indices=list(self.massive_indices), joint_names=self.joint_names,
            qpos_to_actuator=self.qpos_to_actuator.tolist(), gear_Nm=self.gears.tolist(), armature=self.armature.tolist(),
            damping=self.damping.tolist(), passive_stiffness=self.spring.tolist(), quaternion="nonzero input normalized in FK; unit constraint available",
            source_limits_rad=self.limits.tolist(), native_float32_limits_rad=self.native_limits.tolist(),
            gravity_world=np.asarray(self.gravity).ravel().tolist(), physics_integrations=0)

    def _point_jacobian(self, body, world_point):
        if body not in self.body_index:
            raise KeyError(body)
        linear, angular = ca.SX.zeros(3, 27), ca.SX.zeros(3, 27)
        linear[:, :3] = ca.SX.eye(3)
        linear[:, 3:6] = -skew(world_point-self.x[:3])
        angular[:, 3:6] = ca.SX.eye(3)
        for index in self._ancestors[body]:
            linear[:, 6+index] = ca.cross(self._axes[index], world_point-self._pivots[index])
            angular[:, 6+index] = self._axes[index]
        return ca.vertcat(linear, angular)

    def point_function(self, body):
        """Material point: local coordinates are constant during AD Jdot*v."""
        if body not in self._point_functions:
            local = ca.SX.sym("local_point", 3)
            pose = self._poses[body]
            point = pose[:3, 3] + pose[:3, :3] @ local
            jacobian = self._point_jacobian(body, point)
            twist = jacobian @ self.v
            self._point_functions[body] = ca.Function("roll_material_point_"+str(self.body_index[body]),
                [self.x, self.v, local], [point, jacobian, twist, ca.jtimes(twist, self.x, self.xdot)],
                ["x", "v", "local_point"], ["position", "spatial_jacobian", "twist", "jdot_v"])
        return self._point_functions[body]

    def world_point_jacobian(self, body):
        """Instantaneous force/velocity Jacobian, separate from geometry motion."""
        if body not in self._world_point_functions:
            point = ca.SX.sym("world_point", 3)
            self._world_point_functions[body] = ca.Function("roll_world_point_jacobian_"+str(self.body_index[body]),
                [self.x, point], [self._point_jacobian(body, point)], ["x", "world_point"], ["spatial_jacobian"])
        return self._world_point_functions[body]


def build_dynamics(model=None, gravity=(0.0, -9.81, 0.0)):
    return RollDynamics(model, gravity)


def decode_words(words):
    return np.asarray(words, dtype=np.uint32).view(np.float32).astype(float)


def native_row_state(row):
    """Read retained actual state. Never use stale cached pose velocity words."""
    poses = decode_words(row["poseWords"]).reshape(-1, 13)
    observation = decode_words(row["physicalObservationWords"])
    if poses.shape != (25, 13) or observation.shape != (67,):
        raise ValueError("Expected complete actual native pose/observation words")
    x = np.r_[poses[0, :7], observation[:21]]
    rotation = np.asarray(quaternion_rotation(ca.DM(x[3:7])))
    v = np.r_[observation[[41, 42, 40]], rotation @ observation[[44, 45, 43]], observation[46:67]]
    return x, v, poses


def native_row_parity(dynamics, row):
    x, v, actual = native_row_state(row)
    poses = np.asarray(dynamics.poses(x))
    position_errors, rotation_errors = [], []
    for i in range(25):
        position_errors.append(np.max(np.abs(poses[:3, 4*i+3]-actual[i, :3])))
        rotation = np.asarray(quaternion_rotation(ca.DM(actual[i, 3:7])))
        rotation_errors.append(np.max(np.abs(poses[:3, 4*i:4*i+3]-rotation)))
    com, _, com_velocity, _ = dynamics.com(x, v)
    current = row["current"]
    result = dict(physicsIntegrations=0, nativeSubsteps=row["nativeSubsteps"],
        posePositionMaxErrorM=float(max(position_errors)), poseRotationMatrixMaxError=float(max(rotation_errors)),
        comMaxErrorM=float(np.max(np.abs(np.asarray(com).ravel()-xyz(current["com"])))),
        comVelocityMaxErrorMps=float(np.max(np.abs(np.asarray(com_velocity).ravel()-xyz(current["comVelocity"])))),
        retainedRootQuaternionNormError=float(np.linalg.norm(x[3:7])-1))
    foot_velocity_errors = []
    for foot in current["contactMotion"]["feet"]:
        local_center = xyz(dynamics.parts[dynamics.body_index[foot["body"]]]["goldenMass"]["centerOfMass"])
        _, _, twist, _ = dynamics.point_function(foot["body"])(x, v, local_center)
        expected = np.r_[xyz(foot["comVelocityMps"]), xyz(foot["angularVelocityRadps"])]
        foot_velocity_errors.append(np.max(np.abs(np.asarray(twist).ravel()-expected)))
    result["footComTwistMaxError"] = float(max(foot_velocity_errors, default=0))
    return result


def native_inspection_state(inspection):
    """Decode immutable configuration and authoritative raw native27 velocity."""
    observation = decode_words(inspection["physicalObservationWords"])
    root = inspection["bodyData"][0]
    if observation.shape != (67,) or len(inspection["bodyData"]) != 25:
        raise ValueError("Expected complete actual native body/observation data")
    x = np.r_[xyz(root["origin"]), [root["rotation"][key] for key in "xyzw"], observation[:21]]
    v = np.asarray(inspection["matrix"]["generalizedVelocity"], dtype=float)
    if v.shape != (27,) or not np.isfinite(v).all():
        raise ValueError("Expected the unchanged checkpoint's complete raw generalized velocity")
    return x, v


def comparison_errors(actual, reference, elementwise_relative_threshold=1e-3):
    """State relative-error definitions, excluding near-zero entry ratios."""
    actual, reference = np.asarray(actual), np.asarray(reference)
    difference = np.abs(actual-reference)
    selected = np.abs(reference) >= elementwise_relative_threshold
    return dict(maxAbsoluteError=float(difference.max()),
        maxErrorRelativeToReferenceMax=float(difference.max()/max(np.abs(reference).max(), 1e-30)),
        relativeFrobeniusError=float(np.linalg.norm(actual-reference)/max(np.linalg.norm(reference), 1e-30)),
        maxElementwiseRelativeErrorAboveThreshold=float(np.max(difference[selected]/np.abs(reference[selected]))) if selected.any() else None,
        elementwiseRelativeReferenceThreshold=elementwise_relative_threshold,
        maxAbsoluteErrorIndex=[int(i) for i in np.unravel_index(difference.argmax(), difference.shape)])


def native_inspection_parity(dynamics, inspection):
    """Pure read-only M/J comparison. Matrix inverses occur only in this check.

    The diagnostic must export the physical mass inverse (dt=0, hinge armature,
    no dt*damping/dt^2*spring). It supplies no native force-bias certificate.
    """
    matrix = inspection["matrix"]
    if matrix["ndofs"] != 27 or not matrix["includesHingeArmature"] or matrix["includesDtDampingOrSpring"]:
        raise ValueError("Expected physical native M inverse with hinge armature and no timestep passives")
    x, v = native_inspection_state(inspection)
    native_bodies = inspection["bodyData"]
    if [body["body"] for body in native_bodies] != list(dynamics.body_ids):
        raise ValueError("Native/source body ordering differs")
    centers, jacobians, twists, _ = [np.asarray(value) for value in dynamics.body_kinematics(x, v)]
    native_jacobians = np.vstack([body["detachedCurrentComJacobian"] for body in native_bodies])
    native_twists = np.column_stack([np.r_[xyz(body["currentComVelocity"]), xyz(body["currentAngularVelocity"])] for body in native_bodies])
    native_centers = np.column_stack([xyz(body["origin"])
        + np.asarray(quaternion_rotation(ca.DM([body["rotation"][key] for key in "xyzw"]))) @ xyz(body["localCom"])
        for body in native_bodies])
    masses = np.array([body["massKg"] for body in native_bodies])
    native_com = native_centers @ masses/masses.sum()
    native_com_velocity = native_twists[:3] @ masses/masses.sum()
    com, _, com_velocity, _ = [np.asarray(value).ravel() for value in dynamics.com(x, v)]
    model_mass = np.asarray(dynamics.mass_matrix(x))
    native_inverse = np.asarray(matrix["inverseMassRows"])
    model_inverse = np.linalg.inv(model_mass)  # Diagnostic only, never a dynamics definition.
    native_mass = np.linalg.inv(native_inverse)
    poses = np.asarray(dynamics.poses(x))
    pose_position_error = max(np.max(np.abs(poses[:3, 4*i+3]-xyz(body["origin"]))) for i, body in enumerate(native_bodies))
    pose_rotation_error = max(np.max(np.abs(poses[:3, 4*i:4*i+3]
        - np.asarray(quaternion_rotation(ca.DM([body["rotation"][key] for key in "xyzw"])))))
        for i, body in enumerate(native_bodies))
    limits = []
    for index, name in enumerate(dynamics.joint_names):
        q = x[7+index]
        source, native = dynamics.limits[index], dynamics.native_limits[index]
        limits.append(dict(qpos=index, joint=name, measuredRad=float(q), sourceLimitsRad=source.tolist(),
            nativeFloat32LimitsRad=native.tolist(), measuredMinusNativeLowerRad=float(q-native[0]),
            nativeUpperMinusMeasuredRad=float(native[1]-q)))
    return dict(physicsIntegrations=0, nativeSubsteps=inspection["nativeSubsteps"],
        nativeReportedPhysicsIntegrations=inspection["physicsIntegrations"], nativeReportedBytesUnchanged=inspection["realWorldBytesUnchanged"],
        inverseMass=comparison_errors(model_inverse, native_inverse), mass=comparison_errors(model_mass, native_mass),
        allBodyComSpatialJacobians=comparison_errors(jacobians, native_jacobians),
        allBodyCurrentTwists=comparison_errors(twists, native_twists),
        posePositionMaxErrorM=float(pose_position_error), poseRotationMatrixMaxError=float(pose_rotation_error),
        bodyComPositionMaxErrorM=float(np.max(np.abs(centers-native_centers))),
        comMaxErrorM=float(np.max(np.abs(com-native_com))), comVelocityMaxErrorMps=float(np.max(np.abs(com_velocity-native_com_velocity))),
        generalizedVelocityMaxError=float(np.max(np.abs(v-np.asarray(matrix["generalizedVelocity"])))),
        armatureMaxError=float(np.max(np.abs(dynamics.armature-np.asarray(matrix["armature"])))),
        dampingMaxError=float(np.max(np.abs(dynamics.damping-np.asarray(matrix["damping"])))),
        inverseMassErrorWithoutHingeArmature=float(np.max(np.abs(np.linalg.inv(model_mass-np.diag(dynamics.armature))-native_inverse))),
        nativeInverseMassSymmetryMaxError=float(np.max(np.abs(native_inverse-native_inverse.T))),
        sourceVsNativeLimitMaxErrorRad=float(np.max(np.abs(dynamics.limits-dynamics.native_limits))), limits=limits,
        biasCertificate=False, interpretation="Physical M and instantaneous COM J comparison only; no native force-bias certificate or trajectory feasibility claim")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-row", type=Path, help="Retained actual native JSON row; read-only parity check")
    parser.add_argument("--native-inspection", type=Path, help="Retained immutable native M/J checkpoint inspection; read-only parity check")
    args = parser.parse_args()
    dynamics = build_dynamics()
    report = dict(physicsIntegrations=0, bodies=len(dynamics.body_ids), massiveBodies=len(dynamics.massive_indices),
        configurationSize=28, velocitySize=27, rootDamping=dynamics.damping[:6].tolist(), rootArmature=dynamics.armature[:6].tolist())
    if args.native_row:
        row = json.loads(args.native_row.read_text())
        report["nativeParity"] = native_row_parity(dynamics, row)
        x, v, _ = native_row_state(row)
        mass, bias = dynamics.dynamics(x, v)
        mass, bias = np.asarray(mass), np.asarray(bias).ravel()
        report.update(massSymmetryMaxError=float(np.max(np.abs(mass-mass.T))), minimumMassEigenvalue=float(np.linalg.eigvalsh(mass)[0]),
            allFinite=bool(np.isfinite(mass).all() and np.isfinite(bias).all()))
    if args.native_inspection:
        report["nativeInspectionParity"] = native_inspection_parity(dynamics, json.loads(args.native_inspection.read_text()))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
