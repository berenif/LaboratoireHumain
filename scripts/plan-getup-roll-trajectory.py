#!/usr/bin/env python3
"""One fixed-sequence, finite-horizon floating-base direct-collocation NLP.

Pure planning only: this file never instantiates/steps a physics engine or applies
forces. The default is BUILD ONLY. --solve is an explicit, single Ipopt call;
there are no retries, mesh changes, sequence searches, or authored pose targets.
Every contact slot, phase interval, tolerance and computational cap is frozen in
an input JSON contract. Outputs remain NOMINAL until independent actual-WASM
execution. Forces in the output are optimization variables, never commands.
"""
import os
# Set before importing either numerical package, including in build-only mode.
for _key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "BLIS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[_key] = "1"

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import casadi as ca
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("getup_roll_dynamics", ROOT / "scripts/getup-roll-dynamics.py")
roll = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(roll)


def json_write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def finite_vector(value, size, label, positive=False):
    array = np.asarray(value, dtype=float)
    if array.size == 1:
        array = np.full(size, float(array.ravel()[0]))
    if array.shape != (size,) or not np.isfinite(array).all() or (positive and np.any(array <= 0)):
        raise ValueError(f"{label} must be {'positive ' if positive else ''}{size}-vector or scalar")
    return array


def quaternion_product(a, b):
    return ca.vertcat(a[3]*b[:3] + b[3]*a[:3] + ca.cross(a[:3], b[:3]), a[3]*b[3]-ca.dot(a[:3], b[:3]))


def quaternion_increment(rotation_vector):
    """World-frame exponential; analytic series keeps the zero derivative finite."""
    theta2 = ca.dot(rotation_vector, rotation_vector)
    safe_theta = ca.sqrt(ca.fmax(theta2, 1e-24))
    coefficient = ca.if_else(theta2 < 1e-12,
        .5-theta2/48+theta2*theta2/3840, ca.sin(.5*safe_theta)/safe_theta)
    real = ca.if_else(theta2 < 1e-12, 1-theta2/8+theta2*theta2/384, ca.cos(.5*safe_theta))
    return ca.vertcat(coefficient*rotation_vector, real)


def orientation_defect(q0, q1, world_rotation_vector):
    """Three independent Lie-group equations plus a separate hemisphere check."""
    predicted = quaternion_product(quaternion_increment(world_rotation_vector), q0/ca.norm_2(q0))
    relative = quaternion_product(ca.vertcat(-predicted[:3], predicted[3]), q1/ca.norm_2(q1))
    return 2*relative[:3], relative[3]


def segment_closest_points(a0, a1, b0, b1):
    """Exact capsule centerline distance, including spheres and parallel lines.

    The minimum of four endpoint/segment candidates and the valid interior
    line/line stationary point is the global closed-segment minimum. Branches
    are piecewise differentiable and need no iterative closest-point solver.
    """
    u, v, r = a1-a0, b1-b0, a0-b0
    aa, bb, cc = ca.dot(u, u), ca.dot(u, v), ca.dot(v, v)
    dd, ee = ca.dot(u, r), ca.dot(v, r)
    clamp = lambda z: ca.fmin(1, ca.fmax(0, z))
    on_segment = lambda p, origin, direction, norm2: origin+clamp(ca.dot(p-origin, direction)/ca.fmax(norm2, 1e-24))*direction
    candidates = [(a0, on_segment(a0, b0, v, cc)), (a1, on_segment(a1, b0, v, cc)),
                  (on_segment(b0, a0, u, aa), b0), (on_segment(b1, a0, u, aa), b1)]
    determinant = aa*cc-bb*bb
    denominator = ca.fmax(determinant, 1e-24)
    s, t = (bb*ee-cc*dd)/denominator, (aa*ee-bb*dd)/denominator
    valid = ca.logic_and(determinant > 1e-20, ca.logic_and(ca.logic_and(s >= 0, s <= 1), ca.logic_and(t >= 0, t <= 1)))
    first_a, first_b = candidates[0]
    point_a = ca.if_else(valid, a0+s*u, first_a)
    point_b = ca.if_else(valid, b0+t*v, first_b)
    distance = ca.sumsqr(point_b-point_a)
    for candidate_a, candidate_b in candidates:
        candidate_distance = ca.sumsqr(candidate_b-candidate_a)
        choose = candidate_distance < distance
        point_a = ca.if_else(choose, candidate_a, point_a)
        point_b = ca.if_else(choose, candidate_b, point_b)
        distance = ca.fmin(distance, candidate_distance)
    return point_a, point_b


def segment_distance_squared(a0, a1, b0, b1):
    point_a, point_b = segment_closest_points(a0, a1, b0, b1)
    return ca.sumsqr(point_b-point_a)


def load_native_row(path, native=1940):
    path = Path(path)
    if path.suffix.lower() in (".ndjson", ".jsonl"):
        matches = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    else:
        value = json.loads(path.read_text())
        matches = value if isinstance(value, list) else [value]
    matches = [row for row in matches if row.get("nativeSubsteps", row.get("native")) == native]
    if len(matches) != 1:
        raise ValueError(f"Need exactly one retained actual-native {native} row in {path}")
    row = matches[0]
    if "physicalObservationWords" not in row or "poseWords" not in row:
        raise ValueError("Need actual poseWords + physicalObservationWords; cached snapshots are not admissible")
    return row


def validate_contract(contract, dynamics):
    """Fail closed on a missing physical/compute decision; never invent a mesh."""
    required = ("schema", "native_row", "native_inspection", "start_native", "end_native", "physics_step_s", "mesh_s", "floor_y_m",
        "floor_friction", "self_friction", "slots", "self_slot", "phases", "tolerances", "bounds", "terminal_hold_s", "lateral_boundary_s", "lateral_hold_s", "solver", "objective", "acceptance_native")
    missing = [key for key in required if key not in contract]
    if missing:
        raise ValueError("Missing frozen contract fields: " + ", ".join(missing))
    if contract["schema"] != "getup-roll-collocation-v1":
        raise ValueError("Unsupported contract schema")
    if contract["start_native"] != 1940 or contract["end_native"] != 4480 or contract["physics_step_s"] != .00125:
        raise ValueError("This planner is bounded to retained native1940 through original native4480")
    if contract["floor_friction"] != 1 or contract["self_friction"] != 0:
        raise ValueError("Original friction is floor=1, human-human=0")
    mesh = np.asarray(contract["mesh_s"], dtype=float)
    horizon = (contract["end_native"]-contract["start_native"])*contract["physics_step_s"]
    if mesh.ndim != 1 or len(mesh) < 2 or not np.isfinite(mesh).all() or mesh[0] != 0 or np.any(np.diff(mesh) <= 0) or abs(mesh[-1]-horizon) > 1e-12:
        raise ValueError(f"mesh_s must increase strictly from 0 to exactly {horizon}; no automatic resampling")
    lateral = float(contract["lateral_boundary_s"])
    if not np.any(np.isclose(mesh, lateral, atol=1e-12, rtol=0)) or not 0 < lateral < horizon:
        raise ValueError("lateral_boundary_s must be an interior frozen mesh node")
    lateral_hold = float(contract["lateral_hold_s"])
    if lateral_hold < .025 or lateral_hold > lateral:
        raise ValueError("lateral_hold_s must cover at least20 native intervals ending at lateral_boundary_s")
    hold = float(contract["terminal_hold_s"])
    if hold < .15 or not np.any(np.isclose(mesh, horizon-hold, atol=1e-12, rtol=0)):
        raise ValueError("Final >=.15s hold must begin on an explicit mesh node")
    tolerance_keys = ("floor_penetration_m", "self_penetration_m", "initial_joint_limit_rad", "initial_quaternion_norm", "constraint_acceptance", "contact_normal_velocity_mps")
    for key in tolerance_keys:
        value = float(contract["tolerances"][key])
        if not np.isfinite(value) or value < 0:
            raise ValueError(f"Invalid fixed tolerance {key}")
    for key, size in (("velocity_abs", 27), ("acceleration_abs", 27)):
        finite_vector(contract["bounds"][key], size, key, positive=True)
    for key in ("normal_force_max_weight", "stop_reaction_max_nm"):
        if not np.isfinite(float(contract["bounds"][key])) or float(contract["bounds"][key]) <= 0:
            raise ValueError(f"Need finite positive {key}")
    solver = contract["solver"]
    for key in ("max_iter", "max_wall_time_s", "tol", "constr_viol_tol", "checkpoint_every_iterations", "dense_output_step_s"):
        if not np.isfinite(float(solver[key])) or float(solver[key]) <= 0:
            raise ValueError(f"Invalid solver cap {key}")
    if solver["max_wall_time_s"] > 1200:
        raise ValueError("Optimizer wall budget cannot exceed 1200 seconds")
    if int(solver["max_iter"]) != solver["max_iter"] or int(solver["checkpoint_every_iterations"]) != solver["checkpoint_every_iterations"]:
        raise ValueError("Iteration bounds must be integers")
    objective_keys = {"torque_squared", "acceleration_squared", "force_squared", "torque_rate_squared"}
    if set(contract["objective"]) != objective_keys or any(not np.isfinite(float(v)) or float(v) < 0 for v in contract["objective"].values()):
        raise ValueError("Only nonnegative effort/acceleration/force/torque-rate weights are allowed; no authored posture target")
    if not any(float(v) > 0 for v in contract["objective"].values()):
        raise ValueError("At least one regularization weight must be positive")
    acceptance = contract["acceptance_native"]
    if "default" not in acceptance or any(not np.isfinite(float(value)) or float(value) <= 0 for value in acceptance.values()):
        raise ValueError("acceptance_native requires positive finite native-unit thresholds, including default")
    endpoint_lookup = {(p["body"], p["name"], p["end"]): i for i, p in enumerate(dynamics.primitive_metadata)}
    names = set()
    for slot in contract["slots"]:
        if set(slot) != {"id", "body", "primitive", "end"} or slot["id"] in names:
            raise ValueError("Each unique slot specifies only id/body/primitive/end")
        names.add(slot["id"])
        if (slot["body"], slot["primitive"], slot["end"]) not in endpoint_lookup:
            raise ValueError(f"Unknown source capsule/sphere endpoint slot {slot}")
    if not names:
        raise ValueError("Need explicit fixed contact slots")
    self_slot = contract["self_slot"]
    if set(self_slot) != {"id", "body_a", "primitive_a", "body_b", "primitive_b"} or self_slot["body_a"] != "right_hand" or self_slot["body_b"] != "left_lower_arm":
        raise ValueError("Only the prescribed existing right_hand/left_lower_arm frictionless self-slot is admissible")
    for side in ("a", "b"):
        if (self_slot["body_"+side], self_slot["primitive_"+side], "a") not in endpoint_lookup:
            raise ValueError("Unknown prescribed self-contact primitive")
    previous = 0
    for phase in contract["phases"]:
        if set(phase) != {"name", "start_s", "end_s", "contacts", "left_shoulder1_stop", "self_contact"}:
            raise ValueError("Phase fields must be name/start_s/end_s/contacts/left_shoulder1_stop/self_contact")
        if abs(phase["start_s"]-previous) > 1e-12 or phase["end_s"] <= phase["start_s"]:
            raise ValueError("Phases must partition the full horizon in order")
        if not np.any(np.isclose(mesh, phase["start_s"], atol=1e-12, rtol=0)) or not np.any(np.isclose(mesh, phase["end_s"], atol=1e-12, rtol=0)):
            raise ValueError("Every fixed phase boundary must be a mesh node")
        if not set(phase["contacts"]).issubset(names) or any(mode not in ("coulomb", "stick", "off") for mode in phase["contacts"].values()):
            raise ValueError("Contact modes are fixed coulomb/stick/off entries keyed by slot id")
        if type(phase["self_contact"]) is not bool:
            raise ValueError("self_contact must be a fixed boolean")
        if type(phase["left_shoulder1_stop"]) is not bool:
            raise ValueError("left_shoulder1_stop must be a fixed boolean")
        previous = phase["end_s"]
    if abs(previous-horizon) > 1e-12:
        raise ValueError("Phases must cover the exact horizon")
    if not contract["phases"][0]["self_contact"] or any(phase["self_contact"] for phase in contract["phases"][1:]):
        raise ValueError("The single measured self-contact is allowed only in phase A, then must release")
    # Once released, the measured shoulder stop cannot be reacquired as a prop.
    stops = [phase["left_shoulder1_stop"] for phase in contract["phases"]]
    if any(stops[i] and not stops[i-1] for i in range(1, len(stops))):
        raise ValueError("Measured initial left_shoulder1 stop can only persist then release")
    return mesh


class Constraints:
    def __init__(self):
        self.expressions, self.lower, self.upper, self.rows = [], [], [], []

    def add(self, category, expression, lower=0., upper=0., scale=1., location=None):
        expression = ca.vec(expression)
        size = expression.numel()
        lo = np.broadcast_to(np.asarray(lower, dtype=float).reshape(-1), (size,))
        hi = np.broadcast_to(np.asarray(upper, dtype=float).reshape(-1), (size,))
        start = len(self.lower)
        self.expressions.append(expression/scale)
        self.lower.extend((lo/scale).tolist())
        self.upper.extend((hi/scale).tolist())
        self.rows.append(dict(category=category, start=start, end=start+size, scale=float(scale), location=location))

    def audit(self, normalized_values):
        values = np.asarray(normalized_values, dtype=float).ravel()
        lo, hi = np.asarray(self.lower), np.asarray(self.upper)
        violation = np.maximum(np.maximum(lo-values, values-hi), 0)
        groups = {}
        for row in self.rows:
            section = slice(row["start"], row["end"])
            residual = violation[section]
            finite = np.isfinite(residual).all()
            maximum = float(np.max(residual)) if finite else None
            group = groups.setdefault(row["category"], dict(maxNativeUnitViolation=0., maxNormalizedViolation=0., rows=0, allFinite=True, worstLocation=None))
            group["rows"] += row["end"]-row["start"]
            group["allFinite"] = group["allFinite"] and bool(finite)
            if not finite:
                group["maxNativeUnitViolation"] = group["maxNormalizedViolation"] = None
                group["worstLocation"] = row["location"]
            elif group["maxNormalizedViolation"] is not None and maximum >= group["maxNormalizedViolation"]:
                group["maxNormalizedViolation"] = maximum
                group["maxNativeUnitViolation"] = maximum*row["scale"]
                group["worstLocation"] = row["location"]
        return dict(allFinite=bool(np.isfinite(values).all()),
            maximumNormalizedViolation=float(np.max(violation)) if np.isfinite(violation).all() else None, categories=groups)


class RollTrajectoryProblem:
    """Build a sparse MX NLP without loading or invoking an optimizer."""
    def __init__(self, contract, contract_path=None):
        started = time.monotonic()
        self.contract = contract
        self.contract_path = None if contract_path is None else str(Path(contract_path).resolve())
        self.dynamics = d = roll.RollDynamics()
        self.mesh = validate_contract(contract, d)
        self.N = len(self.mesh)
        self.tol = contract["tolerances"]
        native_path = Path(contract["native_row"])
        if not native_path.is_absolute():
            native_path = ROOT/native_path
        self.native_path = native_path
        self.native_row = load_native_row(native_path, contract["start_native"])
        self.x0, reconstructed_velocity, _ = roll.native_row_state(self.native_row)
        self.inspection_path = Path(contract["native_inspection"])
        if not self.inspection_path.is_absolute():
            self.inspection_path = ROOT/self.inspection_path
        self.inspection = json.loads(self.inspection_path.read_text())
        if self.inspection.get("nativeSubsteps") != 1940 or self.inspection.get("physicsIntegrations") != 0 or self.inspection.get("realWorldBytesUnchanged") is not True:
            raise ValueError("Initial inspection must be read-only unchanged-world native1940")
        self.v0 = finite_vector(self.inspection["matrix"]["generalizedVelocity"], 27, "native raw generalizedVelocity")
        observation = roll.decode_words(self.native_row["physicalObservationWords"])
        if not np.array_equal(self.v0[6:].astype(np.float32).view(np.uint32), observation[46:67].astype(np.float32).view(np.uint32)):
            raise ValueError("Native raw hinge velocities do not match exact retained observation words")
        if not np.array_equal(self.v0[:3].astype(np.float32).view(np.uint32), observation[[41, 42, 40]].astype(np.float32).view(np.uint32)):
            raise ValueError("Native raw root-origin linear velocity does not match retained source coordinate mapping")
        inspection_words = np.asarray(self.inspection["physicalObservationWords"], dtype=np.uint32)
        retained_words = np.asarray(self.native_row["physicalObservationWords"], dtype=np.uint32)
        state_indices = np.r_[np.arange(21), np.arange(40, 67)]
        if not np.array_equal(inspection_words[state_indices], retained_words[state_indices]):
            raise ValueError("Inspection q/root-qvel/hinge-qvel words disagree with retained exact native row")
        self.raw_velocity_difference = float(np.max(np.abs(self.v0-reconstructed_velocity)))
        # Native model joint-profile limits are f32, exactly as in the source
        # runtime. Converting source JSON to f32 is not a range expansion.
        self.joint_limits = limits = np.asarray(d.limits, dtype=np.float32).astype(float)
        self.parity = roll.native_row_parity(d, self.native_row)
        initial_limit_violation = np.maximum(np.maximum(limits[:, 0]-self.x0[7:], self.x0[7:]-limits[:, 1]), 0)
        if max(initial_limit_violation) > self.tol["initial_joint_limit_rad"]:
            raise ValueError(f"Exact retained initial joint state exceeds frozen microscopic allowance: {max(initial_limit_violation)}")
        initial_quat_norm_error = abs(np.linalg.norm(self.x0[3:7])-1)
        if initial_quat_norm_error > self.tol["initial_quaternion_norm"]:
            raise ValueError("Exact retained initial quaternion exceeds fixed unit-norm allowance")
        # No initial state is clamped, normalized, translated, repaired, or reset.
        self.initial_compatibility = dict(jointLimitViolationRad=initial_limit_violation.tolist(), quaternionNormError=float(initial_quat_norm_error), stateModified=False,
            velocitySource="read-only native checkpoint inspector matrix.generalizedVelocity, exact raw 27 values",
            reconstructedObservationVelocityMaxDifference=self.raw_velocity_difference, rawVelocity=self.v0.tolist(), rawQposAndQvelWordsAgree=True)
        self.slots = contract["slots"]
        self.slot_endpoint_indices = [next(i for i, p in enumerate(d.primitive_metadata) if (p["body"], p["name"], p["end"]) == (s["body"], s["primitive"], s["end"])) for s in self.slots]
        self.K = len(self.slots)
        self.phase_indices = [self.phase_index(t) for t in self.mesh]
        self.constraints = c = Constraints()
        self._build_geometry()
        self._build_stage()
        self._build_dense_curve()
        X = self.X = ca.MX.sym("X", 28, self.N)
        V = self.V = ca.MX.sym("V", 27, self.N)
        A = self.A = ca.MX.sym("A", 27, self.N)
        T = self.T = ca.MX.sym("physical_torque_Nm", 21, self.N)
        F = self.F = ca.MX.sym("floor_forces_world_N", 3*self.K, self.N)
        Z = self.Z = ca.MX.sym("slip_speed_mps", self.K, self.N)
        L = self.L = ca.MX.sym("left_shoulder1_lower_stop_Nm", 1, self.N)
        H = self.H = ca.MX.sym("right_hand_left_forearm_normal_N", 1, self.N)
        self.blocks = [("x", X), ("v", V), ("a", A), ("tau", T), ("force", F), ("slip", Z), ("stop", L), ("self_force", H)]
        self.w = ca.vertcat(*[ca.vec(block) for _, block in self.blocks])
        self.lbw, self.ubw, self.w0 = self._variable_bounds_and_initial_guess()
        c.add("initial_configuration", X[:, 0]-self.x0, location="native1940")
        c.add("initial_velocity", V[:, 0]-self.v0, location="native1940")
        weight = d.mass*9.81
        objective = 0
        for k, t in enumerate(self.mesh):
            location = dict(node=k, timeS=float(t), phase=contract["phases"][self.phase_indices[k]]["name"])
            out = self.stage(X[:, k], V[:, k], A[:, k], T[:, k], F[:, k], L[:, k], H[:, k])
            c.add("dynamics_N_or_Nm", out[0], scale=weight, location=location)
            # Exact initial_configuration already fixes all four raw quaternion
            # components. Its norm compatibility was checked without a repair;
            # another norm equation here would be rank-dependent.
            if k:
                c.add("unit_quaternion", out[1], location=location)
            c.add("floor_clearance_m", out[2], -self.tol["floor_penetration_m"], np.inf, location=location)
            # Squared centerline distances enforce exact capsule/sphere exclusion.
            radii = np.asarray([pair["radii"] for pair in self.collision_pairs])
            c.add("self_capsule_distance_squared_m2", out[3], np.maximum(radii-self.tol["self_penetration_m"], 0)**2, np.inf, location=location)
            phase = contract["phases"][self.phase_indices[k]]
            for j, slot in enumerate(self.slots):
                mode = phase["contacts"].get(slot["id"], "off")
                if mode == "off":
                    continue  # Its force and auxiliary variables are exactly zero bounds.
                force, slip = F[3*j:3*j+3, k], Z[j, k]
                gap, material_velocity = out[4][j], out[5][3*j:3*j+3]
                where = dict(**location, slot=slot["id"])
                c.add("friction_cone_N2", ca.sumsqr(force[[0, 2]])-force[1]**2, -np.inf, 0., scale=weight**2, location=where)
                # No force in separation; existing overlap allowance is unchanged.
                c.add("force_separation_Nm", force[1]*ca.fmax(gap, 0), scale=weight, location=where)
                if mode == "stick":
                    c.add("rolling_material_point_velocity_mps", material_velocity, -self.tol["contact_normal_velocity_mps"], self.tol["contact_normal_velocity_mps"], location=where)
                else:
                    # Exact maximum-dissipation Coulomb law. At v_t=0, slip=0
                    # leaves any force inside the cone; no epsilon friction law.
                    tangent = material_velocity[[0, 2]]
                    c.add("slip_speed_definition_m2ps2", slip**2-ca.sumsqr(tangent), location=where)
                    c.add("coulomb_maximum_dissipation_Nmps", slip*force[[0, 2]]+force[1]*tangent, scale=weight, location=where)
                    # Normal complementarity is unilateral; a loaded contact
                    # cannot continue separating or penetrating at mesh nodes.
                    c.add("loaded_normal_velocity_Nmps", force[1]*material_velocity[1], -weight*self.tol["contact_normal_velocity_mps"], weight*self.tol["contact_normal_velocity_mps"], scale=weight, location=where)
            if phase["self_contact"]:
                c.add("self_force_separation_Nm", H[0, k]*ca.fmax(out[12], 0), scale=weight, location=location)
                c.add("self_loaded_normal_velocity_Nmps", H[0, k]*out[13], -weight*self.tol["contact_normal_velocity_mps"], weight*self.tol["contact_normal_velocity_mps"], scale=weight, location=location)
            else:
                c.add("released_existing_self_contact_gap_m", out[12], 0., np.inf, location=location)
            if phase["left_shoulder1_stop"]:
                joint = d.joint_names.index("left_shoulder1")
                c.add("stop_separation_Nmrad", L[0, k]*ca.fmax(X[7+joint, k]-self.joint_limits[joint, 0], 0), scale=max(1., d.gears[joint]), location=location)
                # No velocity clamp at the exact initial native sample: its
                # measured tiny incoming stop speed is retained verbatim.
                if k:
                    c.add("loaded_stop_velocity_Nmradps", L[0, k]*V[6+joint, k], location=location)
            if contract["lateral_boundary_s"]-contract["lateral_hold_s"]-1e-12 <= t <= contract["lateral_boundary_s"]+1e-12:
                c.add("lateral_pelvis_torso_face_up", out[6], -.65, .65, location=location)
                c.add("lateral_pelvis_torso_long_up", out[7], -.35, .35, location=location)
                c.add("lateral_pelvis_torso_right_side_up", out[11], .65, np.inf, location=location)
                c.add("lateral_head_height_m", out[10], -np.inf, .5, location=location)
                c.add("lateral_ground_normal_N", sum(F[3*j+1, k] for j in range(self.K)), .8*weight, np.inf, scale=weight, location=location)
                c.add("lateral_right_flank_normal_N", sum(F[3*j+1, k] for j, slot in enumerate(self.slots) if slot["body"] in ("torso", "pelvis", "right_upper_arm", "right_lower_arm", "right_hand")), 3., np.inf, location=location)
            if t >= self.mesh[-1]-contract["terminal_hold_s"]-1e-12:
                c.add("terminal_pelvis_torso_face_up", out[6], -np.inf, -.8, location=location)
                c.add("terminal_pelvis_torso_long_up", out[7], -.35, .35, location=location)
                c.add("terminal_all_body_com_linear_squared_m2ps2", out[8], 0., .22**2, location=location)
                c.add("terminal_all_body_angular_squared_rad2ps2", out[9], 0., .65**2, location=location)
                c.add("terminal_ground_normal_N", sum(F[3*j+1, k] for j in range(self.K)), .8*weight, np.inf, scale=weight, location=location)
                c.add("terminal_head_height_m", out[10], -np.inf, .5, location=location)
            # Trapezoidal quadrature of effort-only regularization.
            quadrature = (self.mesh[1]-self.mesh[0])/2 if k == 0 else ((self.mesh[-1]-self.mesh[-2])/2 if k == self.N-1 else (self.mesh[k+1]-self.mesh[k-1])/2)
            ow = contract["objective"]
            objective += quadrature*(ow["torque_squared"]*ca.sumsqr(T[:, k]/ca.DM(d.gears)) + ow["acceleration_squared"]*ca.sumsqr(A[:, k]/ca.DM(self.amax)) + ow["force_squared"]*(ca.sumsqr(F[:, k]/weight)+ca.sumsqr(H[:, k]/weight)))
            if k:
                h = self.mesh[k]-self.mesh[k-1]
                c.add("position_integration_m", X[:3, k]-X[:3, k-1]-.5*h*(V[:3, k]+V[:3, k-1]), location=location)
                qdefect, hemisphere = orientation_defect(X[3:7, k-1], X[3:7, k], .5*h*(V[3:6, k]+V[3:6, k-1]))
                c.add("orientation_integration", qdefect, location=location)
                c.add("quaternion_hemisphere", hemisphere, 0., np.inf, location=location)
                c.add("hinge_position_integration_rad", X[7:, k]-X[7:, k-1]-.5*h*(V[6:, k]+V[6:, k-1]), location=location)
                c.add("velocity_integration", V[:, k]-V[:, k-1]-.5*h*(A[:, k]+A[:, k-1]), location=location)
                objective += ow["torque_rate_squared"]*ca.sumsqr((T[:, k]-T[:, k-1])/ca.DM(d.gears))/h
        self.objective = objective
        self.g = ca.vertcat(*c.expressions)
        self.evaluate = ca.Function("roll_trajectory_audit", [self.w], [objective, self.g])
        self.build_seconds = time.monotonic()-started
        self.solve_calls = 0

    def phase_index(self, t):
        # Right-continuous fixed mode at a boundary; final mode at horizon.
        for index, phase in enumerate(self.contract["phases"]):
            if t < phase["end_s"]-1e-12 or index == len(self.contract["phases"])-1:
                return index
        raise ValueError("Time outside frozen phases")

    def _build_geometry(self):
        d = self.dynamics
        x = ca.SX.sym("geometry_x", 28)
        endpoints, floor_gaps = d.primitive_endpoints(x, self.contract["floor_y_m"])
        # primitive_metadata contains a,b consecutively, spheres included.
        primitives = []
        for i in range(0, len(d.primitive_metadata), 2):
            p = d.primitive_metadata[i]
            primitives.append(dict(body=p["body"], primitive=p["name"], radius=p["radius"], a=i, b=i+1))
        self.primitives = primitives
        pairs, distances = [], []
        for i, a in enumerate(primitives):
            part_a = d.parts[d.body_index[a["body"]]]
            for b in primitives[i+1:]:
                part_b = d.parts[d.body_index[b["body"]]]
                if a["body"] == b["body"] or b["body"] in part_a["collisionExclusions"] or a["body"] in part_b["collisionExclusions"]:
                    continue
                pairs.append(dict(a=[a["body"], a["primitive"]], b=[b["body"], b["primitive"]], radii=a["radius"]+b["radius"]))
                distances.append(segment_distance_squared(endpoints[:, a["a"]], endpoints[:, a["b"]], endpoints[:, b["a"]], endpoints[:, b["b"]]))
        self.collision_pairs = pairs
        self.geometry = ca.Function("roll_capsule_geometry", [x], [endpoints, floor_gaps, ca.vertcat(*distances)])
        self_slot = self.contract["self_slot"]
        selected = []
        for side in ("a", "b"):
            selected.append(next(p for p in primitives if (p["body"], p["primitive"]) == (self_slot["body_"+side], self_slot["primitive_"+side])))
        a, b = selected
        closest_a, closest_b = segment_closest_points(endpoints[:, a["a"]], endpoints[:, a["b"]], endpoints[:, b["a"]], endpoints[:, b["b"]])
        displacement = closest_b-closest_a
        length = ca.sqrt(ca.fmax(ca.sumsqr(displacement), 1e-24))
        normal = displacement/length
        point_a, point_b = closest_a+a["radius"]*normal, closest_b-b["radius"]*normal
        self.self_geometry = ca.Function("roll_prescribed_self_contact_geometry", [x], [point_a, point_b, normal, length-a["radius"]-b["radius"]])

    def _build_stage(self):
        d = self.dynamics
        x, v, a, tau, forces, stop, self_force = (ca.MX.sym(name, size) for name, size in (("x", 28), ("v", 27), ("a", 27), ("tau", 21), ("forces", 3*self.K), ("stop", 1), ("self_force", 1)))
        mass, bias = d.dynamics(x, v)
        endpoints, floor_gaps, self_distances = self.geometry(x)
        applied = d.S@tau
        gaps, velocities = [], []
        for j, slot in enumerate(self.slots):
            endpoint_index = self.slot_endpoint_indices[j]
            radius = d.primitive_metadata[endpoint_index]["radius"]
            # A changing geometry-selected support point, never pinned locally.
            point = endpoints[:, endpoint_index]-ca.DM([0., radius, 0.])
            jacobian = d.world_point_jacobian(slot["body"])(x, point)[:3, :]
            applied += jacobian.T@forces[3*j:3*j+3]
            gaps.append(floor_gaps[endpoint_index])
            velocities.append(jacobian@v)
        stop_joint = d.joint_names.index("left_shoulder1")
        stop_axis = ca.DM.zeros(27)
        stop_axis[6+stop_joint] = 1
        applied += stop_axis*stop[0]
        self_a, self_b, self_normal, self_gap = self.self_geometry(x)
        self_slot = self.contract["self_slot"]
        self_jacobian_a = d.world_point_jacobian(self_slot["body_a"])(x, self_a)[:3, :]
        self_jacobian_b = d.world_point_jacobian(self_slot["body_b"])(x, self_b)[:3, :]
        relative_jacobian = self_jacobian_b-self_jacobian_a
        applied += relative_jacobian.T@self_normal*self_force[0]
        self_normal_velocity = ca.dot(self_normal, relative_jacobian@v)
        poses = d.poses(x)
        face, long, side = [], [], []
        for body in ("pelvis", "torso"):
            index = d.body_index[body]
            face.append(poses[1, 4*index+2])
            long.append(poses[1, 4*index+1])
            side.append(poses[1, 4*index])
        _, _, twists, _ = d.body_kinematics(x, v)
        linear_squared = ca.vertcat(*[ca.sumsqr(twists[:3, i]) for i in range(len(d.body_ids))])
        angular_squared = ca.vertcat(*[ca.sumsqr(twists[3:, i]) for i in range(len(d.body_ids))])
        head = poses[1, 4*d.body_index["head"]+3]
        self.stage = ca.Function("roll_collocation_stage", [x, v, a, tau, forces, stop, self_force],
            [mass@a+bias-applied, ca.dot(x[3:7], x[3:7])-1, floor_gaps, self_distances,
             ca.vertcat(*gaps), ca.vertcat(*velocities), ca.vertcat(*face), ca.vertcat(*long), linear_squared, angular_squared, head, ca.vertcat(*side), self_gap, self_normal_velocity])

    def _variable_bounds_and_initial_guess(self):
        d, n, k = self.dynamics, self.N, self.K
        self.vmax = finite_vector(self.contract["bounds"]["velocity_abs"], 27, "velocity_abs", True)
        self.amax = finite_vector(self.contract["bounds"]["acceleration_abs"], 27, "acceleration_abs", True)
        lb, ub, guess = {}, {}, {}
        lb["x"], ub["x"] = np.full((28, n), -np.inf), np.full((28, n), np.inf)
        lb["x"][3:7], ub["x"][3:7] = -1.000001, 1.000001
        lb["x"][7:], ub["x"][7:] = self.joint_limits[:, 0, None], self.joint_limits[:, 1, None]
        # Preserve exact initial words despite the documented microscopic native
        # stop overshoot. Original source limits apply at every later node.
        lb["x"][7:, 0] = np.minimum(self.joint_limits[:, 0], self.x0[7:])
        ub["x"][7:, 0] = np.maximum(self.joint_limits[:, 1], self.x0[7:])
        guess["x"] = np.tile(self.x0[:, None], (1, n))
        for name, maximum, dimension in (("v", self.vmax, 27), ("a", self.amax, 27), ("tau", d.gears, 21)):
            lb[name], ub[name] = -np.tile(maximum[:, None], (1, n)), np.tile(maximum[:, None], (1, n))
            guess[name] = np.zeros((dimension, n))
        guess["v"][:] = self.v0[:, None]
        # Bias evaluation is not a solve: no optimization, integration or root
        # support assignment is done to initialize motor torques.
        bias = np.asarray(d.bias(self.x0, self.v0)).ravel()
        guess["tau"][:] = np.clip(bias[6:], -d.gears, d.gears)[:, None]
        weight = d.mass*9.81
        fmax = self.contract["bounds"]["normal_force_max_weight"]*weight
        lb["force"], ub["force"] = np.full((3*k, n), -fmax), np.full((3*k, n), fmax)
        guess["force"] = np.zeros((3*k, n))
        lb["slip"], ub["slip"], guess["slip"] = np.zeros((k, n)), np.full((k, n), np.inf), np.zeros((k, n))
        lb["stop"], ub["stop"], guess["stop"] = np.zeros((1, n)), np.full((1, n), self.contract["bounds"]["stop_reaction_max_nm"]), np.zeros((1, n))
        lb["self_force"], ub["self_force"], guess["self_force"] = np.zeros((1, n)), np.full((1, n), fmax), np.zeros((1, n))
        speed = np.asarray(self.stage(self.x0, self.v0, np.zeros(27), np.zeros(21), np.zeros(3*k), 0, 0)[5]).ravel()
        source_loads = self.native_row["current"].get("loadsN", [0.]*len(d.body_ids))
        for node, phase_index in enumerate(self.phase_indices):
            phase = self.contract["phases"][phase_index]
            active = [j for j, slot in enumerate(self.slots) if phase["contacts"].get(slot["id"], "off") != "off"]
            for j, slot in enumerate(self.slots):
                mode = phase["contacts"].get(slot["id"], "off")
                if mode == "off":
                    lb["force"][3*j:3*j+3, node] = ub["force"][3*j:3*j+3, node] = 0
                    ub["slip"][j, node] = 0
                else:
                    lb["force"][3*j+1, node] = 0
                    same_body = sum(self.slots[i]["body"] == slot["body"] for i in active)
                    guess["force"][3*j+1, node] = min(fmax, source_loads[d.body_index[slot["body"]]]/same_body)
                    if mode == "stick":
                        ub["slip"][j, node] = 0
                    else:
                        guess["slip"][j, node] = np.linalg.norm(speed[3*j:3*j+3][[0, 2]])
            if not phase["left_shoulder1_stop"]:
                ub["stop"][0, node] = 0
            if not phase["self_contact"]:
                ub["self_force"][0, node] = 0
        self.variable_bound_rows = []
        position = 0
        for name, block in self.blocks:
            self.variable_bound_rows.append(dict(category=name, start=position, end=position+block.numel()))
            position += block.numel()
        flatten = lambda source: np.concatenate([source[name].ravel(order="F") for name, _ in self.blocks])
        return flatten(lb), flatten(ub), flatten(guess)

    def unpack(self, vector):
        vector = np.asarray(vector, dtype=float).ravel()
        result, start = {}, 0
        for name, block in self.blocks:
            size = block.numel()
            result[name] = vector[start:start+size].reshape(block.shape, order="F")
            start += size
        return result

    def audit(self, vector):
        vector = np.asarray(vector, dtype=float).ravel()
        objective, values = self.evaluate(vector)
        result = self.constraints.audit(values)
        bound_violation = np.maximum(np.maximum(self.lbw-vector, vector-self.ubw), 0)
        result["variableBounds"] = {row["category"]: float(np.max(bound_violation[row["start"]:row["end"]])) if np.isfinite(bound_violation[row["start"]:row["end"]]).all() else None for row in self.variable_bound_rows}
        result["objective"] = float(objective) if np.isfinite(float(objective)) else None
        result["allFinite"] = result["allFinite"] and bool(np.isfinite(vector).all())
        thresholds = self.contract["acceptance_native"]
        result["nativeAcceptanceThresholds"] = thresholds
        result["nativeCategoryAcceptance"] = {name: bool(group["allFinite"] and group["maxNativeUnitViolation"] <= thresholds.get(name, thresholds["default"])) for name, group in result["categories"].items()}
        result["withinFrozenAcceptance"] = bool(result["allFinite"] and result["maximumNormalizedViolation"] <= self.tol["constraint_acceptance"] and all(result["nativeCategoryAcceptance"].values()) and np.max(bound_violation) <= thresholds.get("variable_bounds", thresholds["default"]))
        return result

    def metadata(self):
        return dict(schema="getup-roll-collocation-build-v1", mode="build-only", physicsIntegrations=0,
            solverInvocations=self.solve_calls, physicalExecutabilityEstablished=False, horizonS=float(self.mesh[-1]),
            nodes=self.N, intervals=self.N-1, variableCount=self.w.numel(), constraintCount=self.g.numel(),
            capsuleSpherePrimitiveCount=len(self.primitives), nonexcludedPrimitivePairCount=len(self.collision_pairs),
            collisionPairs=self.collision_pairs, floorEndpointCount=len(self.dynamics.primitive_metadata),
            contactSlots=self.slots, prescribedSelfSlot=self.contract["self_slot"], nodePhaseIndices=self.phase_indices, fixedTolerances=self.tol,
            nativeAcceptanceThresholds=self.contract["acceptance_native"], sourceJointLimitsRad=self.joint_limits.tolist(), physicalMotorTorqueCapsNm=self.dynamics.gears.tolist(),
            initialStateCompatibility=self.initial_compatibility, nativeParity=self.parity,
            configurationIntegration="trapezoidal root/hinges; Lie-group exponential of trapezoidal world angular velocity",
            velocityIntegration="trapezoidal acceleration", contactPoint="world endpoint center minus radius*worldY; instantaneous material-point force/velocity Jacobian",
            friction="exact cone + maximum-dissipation slip-speed polynomial equations, or fixed sticking mode",
            selfContact="all original nonexcluded capsule/sphere pairs nonpenetrating; one prescribed frictionless right-hand/left-forearm reaction in phase A; released gap>=0 afterward",
            contactForces="optimization variables only; never external force commands", forceSeparation="fn*max(geometric_gap,0)=0; fixed existing penetration allowance",
            quaternion="exact retained initial words; subsequent nodes unit; no initial normalization", objective="effort/acceleration/force/torque-rate only, no posture reference",
            sourceModelSha256=hashlib.sha256(roll.MODEL_PATH.read_bytes()).hexdigest(), nativeRowsSha256=hashlib.sha256(self.native_path.read_bytes()).hexdigest(), nativeInspectionSha256=hashlib.sha256(self.inspection_path.read_bytes()).hexdigest(),
            contractSha256=hashlib.sha256(json.dumps(self.contract, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            buildSeconds=self.build_seconds, casadiVersion=ca.__version__, threads=1)

    def trajectory(self, vector, status):
        data = self.unpack(vector)
        rows = []
        for k, t in enumerate(self.mesh):
            phase = self.contract["phases"][self.phase_indices[k]]
            rows.append(dict(timeS=float(t), nativeEquivalent=self.contract["start_native"]+float(t)/self.contract["physics_step_s"],
                x=data["x"][:, k].tolist(), v=data["v"][:, k].tolist(), a=data["a"][:, k].tolist(), tauPhysicalNm=data["tau"][:, k].tolist(),
                floorForcesWorldN=data["force"][:, k].reshape(self.K, 3).tolist(), slipSpeedMps=data["slip"][:, k].tolist(), leftShoulder1StopReactionNm=float(data["stop"][0, k]), prescribedSelfContactNormalN=float(data["self_force"][0, k]),
                phase=phase["name"], modes={s["id"]: phase["contacts"].get(s["id"], "off") for s in self.slots}))
        dense, dense_geometry = self.dense_interpolation(data)
        return dict(schema="getup-roll-nominal-trajectory-v1", nominalOnly=True, physicsIntegrations=0, physicalExecutabilityEstablished=False,
            optimizerStatus=status, slotOrder=self.slots, nodes=rows, denseInterpolation=dense, denseResidualAudit=dense_geometry, nominalTrajectoryAdmitted=bool(dense_geometry["withinFrozenAcceptance"] and self.audit(vector)["withinFrozenAcceptance"]),
            denseInterpretation="Coherent nominal interpolation with sampled full residual admission; continuous-time and actual-WASM execution remain unverified", contract=self.contract)

    def _build_dense_curve(self):
        """One coherent configuration quintic; v/a are its derivatives.

        Endpoint quaternion derivatives come from WORLD omega and alpha. A
        normalized quaternion polynomial preserves SO(3); it does not impose an
        independent SLERP angular velocity or independent velocity interpolant.
        """
        s, h = ca.SX.sym("fraction"), ca.SX.sym("duration")
        x0, v0, a0, x1, v1, a1 = (ca.SX.sym(name, size) for name, size in (("x0", 28), ("v0", 27), ("a0", 27), ("x1", 28), ("v1", 27), ("a1", 27)))
        h00, h10, h20 = 1-10*s**3+15*s**4-6*s**5, s-6*s**3+8*s**4-3*s**5, .5*s**2-1.5*s**3+1.5*s**4-.5*s**5
        h01, h11, h21 = 10*s**3-15*s**4+6*s**5, -4*s**3+7*s**4-3*s**5, .5*s**3-s**4+.5*s**5
        def quintic(y0, dy0, ddy0, y1, dy1, ddy1):
            return h00*y0+h10*h*dy0+h20*h*h*ddy0+h01*y1+h11*h*dy1+h21*h*h*ddy1
        q0, q1 = x0[3:7]/ca.norm_2(x0[3:7]), x1[3:7]/ca.norm_2(x1[3:7])
        q1 = ca.if_else(ca.dot(q0, q1) < 0, -q1, q1)
        def quaternion_derivatives(q, omega, alpha):
            omega_q, alpha_q = ca.vertcat(omega, 0), ca.vertcat(alpha, 0)
            first = .5*quaternion_product(omega_q, q)
            second = .5*quaternion_product(alpha_q, q)+.5*quaternion_product(omega_q, first)
            return first, second
        dq0, ddq0 = quaternion_derivatives(q0, v0[3:6], a0[3:6])
        dq1, ddq1 = quaternion_derivatives(q1, v1[3:6], a1[3:6])
        unnormalized = quintic(q0, dq0, ddq0, q1, dq1, ddq1)
        quat = unnormalized/ca.norm_2(unnormalized)
        ordinary0, ordinary1 = ca.vertcat(x0[:3], x0[7:]), ca.vertcat(x1[:3], x1[7:])
        ordinary = quintic(ordinary0, ca.vertcat(v0[:3], v0[6:]), ca.vertcat(a0[:3], a0[6:]), ordinary1, ca.vertcat(v1[:3], v1[6:]), ca.vertcat(a1[:3], a1[6:]))
        x = ca.vertcat(ordinary[:3], quat, ordinary[3:])
        xdot = ca.jacobian(x, s)/h
        omega = 2*quaternion_product(xdot[3:7], ca.vertcat(-quat[:3], quat[3]))[:3]
        v = ca.vertcat(xdot[:3], omega, xdot[7:])
        a = ca.jacobian(v, s)/h
        expected_rate = self.dynamics.configuration_rate(x, v)[0]
        consistency = xdot-expected_rate
        self.dense_curve = ca.Function("roll_coherent_quintic_configuration", [s, h, x0, v0, a0, x1, v1, a1], [x, v, a, xdot, consistency])

    def dense_interpolation(self, data):
        step = self.contract["solver"]["dense_output_step_s"]
        times = sorted(set(np.r_[np.arange(0., self.mesh[-1], step), self.mesh].tolist()))
        rows, minimum_floor, minimum_self = [], np.inf, np.inf
        maximum_unit_error, categories = 0., {}
        weight = self.dynamics.mass*9.81
        thresholds = self.contract["acceptance_native"]
        bound_categories = {"joint_limit_rad", "velocity_bound", "acceleration_bound", "torque_bound_Nm", "inactive_force_N", "normal_force_bound_N", "self_normal_force_bound_N", "inactive_self_force_N", "stop_reaction_bound_Nm", "inactive_stop_reaction_Nm"}
        def record(category, violation, t):
            values = np.asarray(violation, dtype=float).ravel()
            finite = bool(np.isfinite(values).all())
            maximum = float(max(np.max(values), 0.)) if finite and len(values) else (0. if finite else None)
            group = categories.setdefault(category, dict(maxNativeUnitViolation=0., allFinite=True, worstTimeS=None, threshold=thresholds.get(category, thresholds.get("variable_bounds", thresholds["default"]) if category in bound_categories else thresholds["default"])))
            group["allFinite"] = group["allFinite"] and finite
            if not finite or group["maxNativeUnitViolation"] is not None and maximum > group["maxNativeUnitViolation"]:
                group["maxNativeUnitViolation"], group["worstTimeS"] = maximum, float(t)
        for t in times:
            k = min(max(np.searchsorted(self.mesh, t, side="right")-1, 0), self.N-2)
            h, s = self.mesh[k+1]-self.mesh[k], (t-self.mesh[k])/(self.mesh[k+1]-self.mesh[k])
            x, v, a, xdot, consistency = [np.asarray(value).ravel() for value in self.dense_curve(s, h, data["x"][:, k], data["v"][:, k], data["a"][:, k], data["x"][:, k+1], data["v"][:, k+1], data["a"][:, k+1])]
            interpolate = lambda name: ((1-s)*data[name][:, k]+s*data[name][:, k+1])
            tau, forces, stop, self_force = interpolate("tau"), interpolate("force"), float(interpolate("stop")[0]), float(interpolate("self_force")[0])
            knot = np.flatnonzero(np.isclose(self.mesh, t, atol=1e-12, rtol=0))
            if len(knot):
                # Preserve exact node words, including native1940's tiny raw
                # quaternion norm deviation. The curve derivative is audited
                # against the exact node's configuration-rate map as well.
                i = int(knot[0]); x, v, a = data["x"][:, i], data["v"][:, i], data["a"][:, i]
                tau, forces, stop, self_force = data["tau"][:, i], data["force"][:, i], float(data["stop"][0, i]), float(data["self_force"][0, i])
                consistency = xdot-np.asarray(self.dynamics.configuration_rate(x, v)[0]).ravel()
            out = [np.asarray(value).ravel() for value in self.stage(x, v, a, tau, forces, stop, self_force)]
            floor_gap = float(np.min(out[2]))
            radii = np.asarray([pair["radii"] for pair in self.collision_pairs])
            pair_gaps = np.sqrt(np.maximum(out[3], 0))-radii
            self_gap = float(np.min(pair_gaps)) if len(pair_gaps) else None
            minimum_floor = min(minimum_floor, floor_gap)
            if self_gap is not None: minimum_self = min(minimum_self, self_gap)
            unit_error = abs(np.linalg.norm(x[3:7])-1)
            maximum_unit_error = max(maximum_unit_error, unit_error)
            phase = self.contract["phases"][self.phase_index(t)]
            record("dense_configuration_rate_consistency", np.abs(consistency), t)
            if t == 0:
                record("initial_configuration", np.abs(x-self.x0), t)
                record("initial_velocity", np.abs(v-self.v0), t)
            record("dynamics_N_or_Nm", np.abs(out[0]), t)
            record("floor_clearance_m", -self.tol["floor_penetration_m"]-out[2], t)
            record("self_capsule_distance_squared_m2", np.maximum(radii-self.tol["self_penetration_m"], 0)**2-out[3], t)
            limits = self.joint_limits.copy()
            if t == 0:
                limits[:, 0], limits[:, 1] = np.minimum(limits[:, 0], self.x0[7:]), np.maximum(limits[:, 1], self.x0[7:])
            record("joint_limit_rad", np.r_[limits[:, 0]-x[7:], x[7:]-limits[:, 1]], t)
            record("velocity_bound", np.abs(v)-self.vmax, t)
            record("acceleration_bound", np.abs(a)-self.amax, t)
            record("torque_bound_Nm", np.abs(tau)-self.dynamics.gears, t)
            record("unit_quaternion", 0. if t == 0 else abs(out[1][0]), t)
            slip_speeds = []
            for j, slot in enumerate(self.slots):
                force, velocity = forces[3*j:3*j+3], out[5][3*j:3*j+3]
                mode = phase["contacts"].get(slot["id"], "off")
                slip = np.linalg.norm(velocity[[0, 2]])
                slip_speeds.append(float(slip))
                if mode == "off":
                    record("inactive_force_N", np.abs(force), t)
                    continue
                record("normal_force_bound_N", [-force[1], force[1]-self.contract["bounds"]["normal_force_max_weight"]*weight], t)
                record("friction_cone_N2", np.sum(force[[0, 2]]**2)-force[1]**2, t)
                record("force_separation_Nm", force[1]*max(out[4][j], 0.), t)
                if mode == "stick":
                    record("rolling_material_point_velocity_mps", np.abs(velocity)-self.tol["contact_normal_velocity_mps"], t)
                else:
                    record("coulomb_maximum_dissipation_Nmps", np.abs(slip*force[[0, 2]]+force[1]*velocity[[0, 2]]), t)
                    record("loaded_normal_velocity_Nmps", abs(force[1]*velocity[1])-weight*self.tol["contact_normal_velocity_mps"], t)
            if phase["self_contact"]:
                record("self_normal_force_bound_N", [-self_force, self_force-self.contract["bounds"]["normal_force_max_weight"]*weight], t)
                record("self_force_separation_Nm", self_force*max(out[12][0], 0.), t)
                record("self_loaded_normal_velocity_Nmps", abs(self_force*out[13][0])-weight*self.tol["contact_normal_velocity_mps"], t)
            else:
                record("inactive_self_force_N", abs(self_force), t)
                record("released_existing_self_contact_gap_m", -out[12][0], t)
            if phase["left_shoulder1_stop"]:
                joint = self.dynamics.joint_names.index("left_shoulder1")
                record("stop_reaction_bound_Nm", [-stop, stop-self.contract["bounds"]["stop_reaction_max_nm"]], t)
                record("stop_separation_Nmrad", stop*max(x[7+joint]-self.joint_limits[joint, 0], 0.), t)
                if t > 0: record("loaded_stop_velocity_Nmradps", abs(stop*v[6+joint]), t)
            else:
                record("inactive_stop_reaction_Nm", abs(stop), t)
            if self.contract["lateral_boundary_s"]-self.contract["lateral_hold_s"]-1e-12 <= t <= self.contract["lateral_boundary_s"]+1e-12:
                record("lateral_pelvis_torso_face_up", np.abs(out[6])-.65, t)
                record("lateral_pelvis_torso_long_up", np.abs(out[7])-.35, t)
                record("lateral_pelvis_torso_right_side_up", .65-out[11], t)
                record("lateral_head_height_m", out[10]-.5, t)
                record("lateral_ground_normal_N", .8*weight-np.sum(forces[1::3]), t)
                record("lateral_right_flank_normal_N", 3.-sum(forces[3*j+1] for j, slot in enumerate(self.slots) if slot["body"] in ("torso", "pelvis", "right_upper_arm", "right_lower_arm", "right_hand")), t)
            if t >= self.mesh[-1]-self.contract["terminal_hold_s"]-1e-12:
                record("terminal_pelvis_torso_face_up", out[6]+.8, t)
                record("terminal_pelvis_torso_long_up", np.abs(out[7])-.35, t)
                record("terminal_all_body_com_linear_squared_m2ps2", out[8]-.22**2, t)
                record("terminal_all_body_angular_squared_rad2ps2", out[9]-.65**2, t)
                record("terminal_ground_normal_N", .8*weight-np.sum(forces[1::3]), t)
                record("terminal_head_height_m", out[10]-.5, t)
            rows.append(dict(timeS=float(t), x=x.tolist(), v=v.tolist(), a=a.tolist(), tauPhysicalNm=tau.tolist(),
                floorForcesWorldN=forces.reshape(self.K, 3).tolist(), slipSpeedMps=slip_speeds,
                leftShoulder1StopReactionNm=stop, prescribedSelfContactNormalN=self_force, phase=phase["name"],
                minimumFloorGapM=floor_gap, minimumSelfCapsuleGapM=self_gap))
        admitted = all(group["allFinite"] and group["maxNativeUnitViolation"] <= group["threshold"] for group in categories.values())
        return rows, dict(samples=len(rows), minimumFloorGapM=minimum_floor, minimumSelfCapsuleGapM=minimum_self if np.isfinite(minimum_self) else None,
            maximumQuaternionNormError=maximum_unit_error, fixedFloorAllowanceM=self.tol["floor_penetration_m"], fixedSelfAllowanceM=self.tol["self_penetration_m"],
            interpolation="coherent normalized-quaternion/configuration quintic Hermite; v and a derived by differentiation",
            categories=categories, withinFrozenAcceptance=bool(admitted), continuousTimeCertified=False)

    def solve_once(self, output_directory):
        if self.solve_calls:
            raise RuntimeError("One optimizer invocation only; retries forbidden")
        output_directory = Path(output_directory)
        output_directory.mkdir(parents=True, exist_ok=True)
        self.solve_calls += 1
        observer = IterationObserver(self, output_directory)
        settings = self.contract["solver"]
        options = {"expand": False, "print_time": True, "error_on_fail": False,
            "iteration_callback": observer, "iteration_callback_step": 1,
            "ipopt": {"max_iter": int(settings["max_iter"]), "max_wall_time": float(settings["max_wall_time_s"]),
                "tol": float(settings["tol"]), "constr_viol_tol": float(settings["constr_viol_tol"]),
                "acceptable_iter": 0, "hessian_approximation": "limited-memory", "linear_solver": "mumps",
                "print_level": 5, "print_frequency_iter": 1, "print_timing_statistics": "yes", "sb": "yes",
                "bound_relax_factor": 0., "honor_original_bounds": "yes", "nlp_scaling_method": "gradient-based"}}
        started = time.monotonic()
        status, stats, vector = "not_started", {}, self.w0
        try:
            solver = ca.nlpsol("fixed_roll_one_solve", "ipopt", dict(x=self.w, f=self.objective, g=self.g), options)
            result = solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.constraints.lower, ubg=self.constraints.upper)
            vector = np.asarray(result["x"]).ravel()
            stats = solver.stats()
            status = str(stats.get("return_status", "unknown"))
        except Exception as error:
            status = "exception: " + str(error)
            if observer.last_vector is not None:
                vector = observer.last_vector
        audit = self.audit(vector)
        report = self.metadata()
        report.update(mode="one-solve-finished", solverInvocations=1, optimizerStatus=status,
            optimizerSuccess=bool(stats.get("success", False)), iterations=stats.get("iter_count"),
            wallSeconds=time.monotonic()-started, optimizerWallBudgetS=settings["max_wall_time_s"], audit=audit)
        report["nominalTrajectoryAdmitted"] = False
        if np.isfinite(vector).all():
            trajectory = self.trajectory(vector, status)
            report["denseResidualAudit"] = trajectory["denseResidualAudit"]
            report["nominalTrajectoryAdmitted"] = bool(report["optimizerSuccess"] and trajectory["nominalTrajectoryAdmitted"])
            trajectory["nominalTrajectoryAdmitted"] = report["nominalTrajectoryAdmitted"]
            json_write(output_directory/"nominal-trajectory.json", trajectory)
        json_write(output_directory/"solve-report.json", report)
        np.savez_compressed(output_directory/"last-iterate.npz", primal=vector)
        print(json.dumps({"event": "solve_finished", "status": status, "iterations": report["iterations"], "wallSeconds": report["wallSeconds"], "withinFrozenAcceptance": audit["withinFrozenAcceptance"], "maxNormalizedViolation": audit["maximumNormalizedViolation"], "nominalTrajectoryAdmitted": report["nominalTrajectoryAdmitted"]}), flush=True)
        return report


class IterationObserver(ca.Callback):
    """Visible progress + bounded periodic last-iterate checkpoints, no retries."""
    def __init__(self, problem, directory):
        ca.Callback.__init__(self)
        self.problem, self.directory = problem, Path(directory)
        self.last_vector, self.iteration, self.started = None, 0, time.monotonic()
        self.construct("roll_iteration_observer", {"enable_fd": False})

    def get_n_in(self): return ca.nlpsol_n_out()
    def get_n_out(self): return 1
    def get_name_in(self, i): return ca.nlpsol_out(i)
    def get_name_out(self, i): return "stop"
    def get_sparsity_in(self, i):
        name = ca.nlpsol_out(i)
        if name in ("x", "lam_x"): return ca.Sparsity.dense(self.problem.w.numel(), 1)
        if name in ("g", "lam_g"): return ca.Sparsity.dense(self.problem.g.numel(), 1)
        if name == "f": return ca.Sparsity.scalar()
        return ca.Sparsity(0, 1)
    def get_sparsity_out(self, i): return ca.Sparsity.scalar()
    def eval(self, arguments):
        self.last_vector = np.asarray(arguments[0]).ravel().copy()
        interval = int(self.problem.contract["solver"]["checkpoint_every_iterations"])
        if self.iteration % interval == 0:
            audit = self.problem.constraints.audit(arguments[2])
            progress = dict(event="optimizer_checkpoint", iteration=self.iteration, wallSeconds=time.monotonic()-self.started,
                objective=float(arguments[1]), maxNormalizedViolation=audit["maximumNormalizedViolation"], categories=audit["categories"])
            json_write(self.directory/"progress.json", progress)
            np.savez_compressed(self.directory/"checkpoint-last-iterate.npz", primal=self.last_vector)
            with (self.directory/"progress.ndjson").open("a") as stream: stream.write(json.dumps(progress, allow_nan=False)+"\n")
            print(json.dumps({key: progress[key] for key in ("event", "iteration", "wallSeconds", "objective", "maxNormalizedViolation")}), flush=True)
        self.iteration += 1
        elapsed = time.monotonic()-self.started
        # The external watchdog includes process/build overhead. This callback
        # independently prevents an in-progress Ipopt call exceeding its cap.
        return [1 if elapsed >= self.problem.contract["solver"]["max_wall_time_s"] else 0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Frozen contact sequence, geometry slots, mesh and computational caps")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--solve", action="store_true", help="Explicitly authorize the ONE optimizer invocation; default builds/audits only")
    args = parser.parse_args()
    contract = json.loads(args.config.read_text())
    problem = RollTrajectoryProblem(contract, args.config)
    metadata = problem.metadata()
    metadata["initialGuessAudit"] = problem.audit(problem.w0)
    json_write(args.output_dir/"build-report.json", metadata)
    print(json.dumps({key: metadata[key] for key in ("mode", "solverInvocations", "physicsIntegrations", "nodes", "variableCount", "constraintCount", "buildSeconds")}), flush=True)
    if args.solve:
        report = problem.solve_once(args.output_dir)
        return 0 if report["nominalTrajectoryAdmitted"] else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
