"""Quantitative static-witness checker, NOT a dynamic or full-path qualifier.

LPs and collision distances must be independently evaluated by the existing
model/geometry audit. This module consumes their source- and point-bound result;
it never runs a solve, trusts a saved endpoint as current, or installs a state.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
from coupled_posture_reference import frozen, chart, wrap


@dataclass(frozen=True)
class StaticWitness:
    sample_sha256: str
    support_sha256: str
    source_step: int
    source_time: float
    model_sha256: str
    helper_sha256: str
    strict_four_point_lp_success: bool
    strict_four_point_motor_fraction: float
    strict_triangle_lp_success: bool
    strict_triangle_motor_fraction: float
    minimum_clean_self_gap_m: float
    future_joint_reference: tuple  # all21; future thigh mapped arms
    verified_map_margins_right_left: tuple
    future_arm_identity_error_rad: float
    future_arm_fk_error_rad: float
    stance_contact_pose_feasible: bool
    both_foot_position_error_m: float
    both_foot_orientation_error_rad: float
    fk_com_source: tuple
    fk_thorax_rotation: tuple
    fk_pelvis_rotation: tuple
    lp_max_equilibrium_residual: float
    lp_max_friction_violation: float

    def __post_init__(self):
        for key,n in (('future_joint_reference',21),('verified_map_margins_right_left',2),
                      ('fk_com_source',3),('fk_thorax_rotation',9),('fk_pelvis_rotation',9)):
            object.__setattr__(self,key,frozen(getattr(self,key),n))
        for key,value in asdict(self).items():
            if isinstance(value,float) and not math.isfinite(value):
                raise ValueError('nonfinite static witness')


def verify_static_point(sample, witness, support, *, joint_limits, lumbar_anchor,
                        model_sha256, helper_sha256, retained_headings,
                        heading_tolerance=1e-8,
                        pose_tolerance=1e-6, lp_residual_tolerance=1e-8):
    """Checks a current proposed posture, without promising a finite-time path.

    Orientation preference slack is reported, not rejected. Heading, feet,
    lumbar/interior/ankle/map reserves and strict LP/gap criteria remain hard.
    """
    if (sample.fingerprint() != witness.sample_sha256 or support.key != witness.support_sha256
            or witness.source_step != support.source_step or witness.source_time != support.source_time
            or witness.model_sha256 != model_sha256 or witness.helper_sha256 != helper_sha256
            or len(model_sha256) != 64 or len(helper_sha256) != 64):
        raise ValueError('static witness is stale or bound to another reference/model')
    limits = np.asarray(joint_limits,dtype=float); q = np.array(witness.future_joint_reference)
    if limits.shape != (21,2) or not np.isfinite(limits).all():
        raise ValueError('invalid source limits')
    if np.max(abs(q[:15]-sample.lower_joints.position)) > 1e-12:
        raise ValueError('static witness lower joints differ from prepared reference')
    margins = np.minimum(q-limits[:,0],limits[:,1]-q)
    if np.min(margins) < .02-1e-10 or np.min(margins[[7,13]]) < .06-1e-10:
        raise ValueError('static joint interior/ankle reserves fail')
    if np.max(abs(q[:3]-np.array(frozen(lumbar_anchor,3)))) > math.radians(5.)+1e-10:
        raise ValueError('static lumbar compensation exceeds anchored5degree bound')
    if (not witness.strict_four_point_lp_success or witness.strict_four_point_motor_fraction >= .925
            or witness.strict_four_point_motor_fraction < 0.
            or (support.require_triangle and (not witness.strict_triangle_lp_success
                or not 0. <= witness.strict_triangle_motor_fraction < .925))):
        raise ValueError('strict original bounded static LP admission fails')
    if witness.minimum_clean_self_gap_m <= .025:
        raise ValueError('clean self gap must be strictly greater than25mm')
    if (np.min(witness.verified_map_margins_right_left) < .02-1e-10
            or not 0. <= witness.future_arm_identity_error_rad < 1e-12
            or not 0. <= witness.future_arm_fk_error_rad < 1e-8):
        raise ValueError('future-thigh exact arm mapping or original map reserve fails')
    if (not witness.stance_contact_pose_feasible
            or not 0. <= witness.both_foot_position_error_m <= pose_tolerance
            or not 0. <= witness.both_foot_orientation_error_rad <= pose_tolerance
            or not 0. <= witness.lp_max_equilibrium_residual <= lp_residual_tolerance
            or not 0. <= witness.lp_max_friction_violation <= lp_residual_tolerance):
        raise ValueError('static feet, equilibrium or friction witness fails')
    support_info = support.validate_com(sample.com)
    # Check achieved FK COM as well, so a desired soft COM cannot hide a bad seed.
    from coupled_posture_reference import Jet
    fk_support_info = support.validate_com(Jet.rest(witness.fk_com_source))
    angles = []
    headings = frozen(retained_headings,2)
    for index,(ref,R) in enumerate(((sample.thorax,witness.fk_thorax_rotation),(sample.pelvis,witness.fk_pelvis_rotation))):
        desired = chart(np.array(ref.rotation).reshape(3,3)); actual = chart(np.array(R).reshape(3,3))
        if max(abs(wrap(desired[0]-headings[index])),abs(wrap(actual[0]-headings[index]))) > heading_tolerance:
            raise ValueError('FK posture changes retained heading')
        angles.append(dict(body=ref.body,forwardPitchSlackRad=float(desired[1]-actual[1]),
            rollSlackRad=wrap(desired[2]-actual[2])))
    return dict(staticPointPassed=True,sourceStep=support.source_step,
        sourceTimeS=support.source_time,minimumJointReserveRad=float(np.min(margins)),
        ankleReservesRightLeftRad=margins[[7,13]].tolist(),
        referenceSupport=support_info,fkSupport=fk_support_info,
        comPreferenceSlackM=(np.array(sample.com.position)-witness.fk_com_source).tolist(),
        orientationPreferenceSlacks=angles,finiteTimePhysicalFeasibilityProven=False,
        currentMeasuredDynamicAnkleAdmissionProven=False,fullPathProven=False)
