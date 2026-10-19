//! Active motor delivery and fixed-attachment contract probes. Synthetic
//! impulse-joint pairs do not certify production multibody torque readback.
use crate::{Profile, configure, rotation};
use rapier3d::prelude::*;
use serde_json::{Value, json};

pub const ACTIVE_CATEGORIES: usize = 21;
pub const FIXED_CATEGORIES: usize = 3;
pub const MASSLESS_COORDINATE_CATEGORIES: usize = 9;
pub const ACTIVE_CASES: usize = ACTIVE_CATEGORIES * 3 * 3 * 2;
pub const FIXED_CASES: usize = FIXED_CATEGORIES * 3 * 3 * 2;
pub const CASES: usize = ACTIVE_CASES + FIXED_CASES;
const SUBSTEPS: usize = 4;
const HEADINGS: [f32; 3] = [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0];
const POSTURES: [&str; 3] = ["neutral", "near-min", "near-max"];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FixtureKind {
    ActiveHinge,
    FixedAttachment,
}
impl FixtureKind {
    fn label(self) -> &'static str {
        match self {
            Self::ActiveHinge => "active-hinge",
            Self::FixedAttachment => "fixed-attachment",
        }
    }
}

/// Classify canonical joints by their actual coordinates and source hinge, never
/// by body mass: all nine massless coordinate frames are active hinge categories.
fn categories(model: &lh_model::Model) -> Result<Vec<(&lh_model::Segment, FixtureKind)>, String> {
    let mut categories = Vec::new();
    let metadata = &model.reference_metadata;
    let joint_order = metadata["jointOrder"]
        .as_array()
        .ok_or("Missing source joint order")?;
    let actuator_order = metadata["actuatorOrder"]
        .as_array()
        .ok_or("Missing source actuator order")?;
    let actuator_to_qpos = metadata["actuatorToQpos"]
        .as_array()
        .ok_or("Missing source actuator mapping")?;
    if [
        joint_order.len(),
        actuator_order.len(),
        actuator_to_qpos.len(),
    ] != [ACTIVE_CATEGORIES; 3]
    {
        return Err("Incomplete canonical source hinge mapping".into());
    }
    let mut qpos_indices = std::collections::BTreeSet::new();
    let mut actuator_indices = std::collections::BTreeSet::new();
    for segment in &model.segments {
        match (&segment.parent, &segment.joint_profile) {
            (None, None) => continue,
            (Some(_), Some(anatomy)) => {
                let hinge = segment.reference.as_ref().and_then(|r| r.hinge.as_ref());
                if let Some(hinge) = hinge {
                    if hinge.gear <= 0.0
                        || !hinge.gear.is_finite()
                        || joint_order.get(hinge.qpos_index).and_then(Value::as_str)
                            != Some(hinge.name.as_str())
                        || actuator_order
                            .get(hinge.actuator_index)
                            .and_then(Value::as_str)
                            != Some(hinge.name.as_str())
                        || actuator_to_qpos
                            .get(hinge.actuator_index)
                            .and_then(Value::as_u64)
                            != Some(hinge.qpos_index as u64)
                        || !qpos_indices.insert(hinge.qpos_index)
                        || !actuator_indices.insert(hinge.actuator_index)
                    {
                        return Err(format!(
                            "Invalid or duplicate source hinge mapping for {}",
                            segment.id
                        ));
                    }
                }
                let kind = if anatomy.axes.is_empty()
                    && hinge.is_none()
                    && segment.role != "coordinate"
                    && segment
                        .reference
                        .as_ref()
                        .and_then(|r| r.source_body.as_ref())
                        .is_some()
                {
                    FixtureKind::FixedAttachment
                } else if anatomy.axes.len() == 1
                    && hinge.is_some()
                    && anatomy.axes[0].max_motor_torque_nm > 0.0
                {
                    FixtureKind::ActiveHinge
                } else {
                    return Err(format!(
                        "Unclassified canonical actuator category {}",
                        segment.id
                    ));
                };
                categories.push((segment, kind));
            }
            _ => return Err(format!("Parent/joint mismatch for {}", segment.id)),
        }
    }
    let active = categories
        .iter()
        .filter(|(_, k)| *k == FixtureKind::ActiveHinge)
        .count();
    let fixed = categories.len() - active;
    let massless = categories
        .iter()
        .filter(|(s, k)| {
            s.role == "coordinate" && s.mass_kg == 0.0 && *k == FixtureKind::ActiveHinge
        })
        .count();
    if active != ACTIVE_CATEGORIES
        || fixed != FIXED_CATEGORIES
        || massless != MASSLESS_COORDINATE_CATEGORIES
    {
        return Err(format!(
            "Canonical fixture coverage changed: {active} active, {fixed} fixed, {massless} massless active"
        ));
    }
    Ok(categories)
}

fn identity(
    segment: &lh_model::Segment,
    kind: FixtureKind,
    heading_index: usize,
    posture: &str,
    sign: f32,
) -> Value {
    let hinge = segment.reference.as_ref().and_then(|r| r.hinge.as_ref());
    json!({"caseId":format!("{}:{heading_index}:{posture}:{sign}", segment.id),
        "segmentId":segment.id,"parentId":segment.parent,"fixtureKind":kind.label(),
        "hingeName":hinge.map(|h| &h.name),"qposIndex":hinge.map(|h| h.qpos_index),
        "actuatorIndex":hinge.map(|h| h.actuator_index),"canonicalMassKg":segment.mass_kg,
        "masslessCoordinateFrame":segment.role == "coordinate" && segment.mass_kg == 0.0,
        "canonicalAxisCount":segment.joint_profile.as_ref().map(|p| p.axes.len()),
        "canonicalCoordinate":segment.joint_profile.as_ref().and_then(|p| p.axes.first().map(|a| a.coordinate)),
        "headingIndex":heading_index,"posture":posture,"sign":sign})
}

/// Reject duplicate/missing identities and empty samples even when an outer
/// caller receives a superficially correct total case count.
pub fn validate_report_coverage(reports: &[Value]) -> Result<(), String> {
    let model = lh_model::Model::canonical()?;
    let mut expected = std::collections::BTreeMap::new();
    for (segment, kind) in categories(&model)? {
        for heading_index in 0..HEADINGS.len() {
            for posture in POSTURES {
                for sign in [-1.0, 1.0] {
                    let id = identity(segment, kind, heading_index, posture, sign);
                    expected.insert(id["caseId"].as_str().unwrap().to_owned(), id);
                }
            }
        }
    }
    if reports.len() != CASES {
        return Err("Incomplete anatomical actuator coverage".into());
    }
    for report in reports {
        let id = &report["identity"];
        let key = id["caseId"]
            .as_str()
            .ok_or("Missing actuator case identity")?;
        if expected.remove(key).as_ref() != Some(id) {
            return Err(format!(
                "Duplicate, unknown or mismatched actuator identity {key}"
            ));
        }
        let samples = report["samples"]
            .as_array()
            .ok_or("Missing actuator samples")?;
        if samples.len() != SUBSTEPS || samples.iter().enumerate().any(|(i, s)| s["substep"] != i) {
            return Err(format!("Incomplete actuator substeps for {key}"));
        }
    }
    if !expected.is_empty() {
        return Err("Missing anatomical actuator identities".into());
    }
    Ok(())
}

pub fn reports_passed(reports: &[Value]) -> bool {
    validate_report_coverage(reports).is_ok()
        && reports.iter().all(|r| {
            r["passed"] == true
                && r["samples"]
                    .as_array()
                    .is_some_and(|s| s.iter().all(|s| s["passed"] == true))
        })
}

pub fn trials(profile: &Profile) -> Result<Vec<Value>, String> {
    let model = lh_model::Model::canonical()?;
    let total: f32 = model
        .segments
        .iter()
        .filter_map(|s| s.joint_profile.as_ref())
        .flat_map(|p| &p.axes)
        .map(|a| a.max_motor_torque_nm)
        .sum();
    let scale = profile
        .motor_scale
        .min(profile.aggregate_motor_cap_nm / total);
    let mut reports = Vec::new();
    for (segment, kind) in categories(&model)? {
        let anatomy = segment
            .joint_profile
            .as_ref()
            .ok_or("Missing canonical joint")?;
        for (heading_index, heading) in HEADINGS.into_iter().enumerate() {
            for posture in POSTURES {
                let coordinates: [f32; 3] = std::array::from_fn(|k| {
                    anatomy
                        .axes
                        .iter()
                        .find(|a| a.coordinate == ['x', 'y', 'z'][k])
                        .map_or(0.0, |a| match posture {
                            "near-min" => a.min_radians * 0.8,
                            "near-max" => a.max_radians * 0.8,
                            _ => 0.0_f32.clamp(a.min_radians, a.max_radians),
                        })
                });
                let relative = Rotation::from_xyzw(
                    (coordinates[0] * 0.5).tan(),
                    (coordinates[1] * 0.5).tan(),
                    (coordinates[2] * 0.5).tan(),
                    1.0,
                )
                .normalize();
                for sign in [-1.0_f32, 1.0] {
                    let mut report = match kind {
                        FixtureKind::ActiveHinge => {
                            active_trial(profile, anatomy, heading, relative, sign, scale)?
                        }
                        FixtureKind::FixedAttachment => {
                            fixed_trial(profile, anatomy, heading, posture, sign, true)?
                        }
                    };
                    report["identity"] = identity(segment, kind, heading_index, posture, sign);
                    report["segment"] = json!(segment.id);
                    report["heading"] = json!(heading);
                    report["posture"] = json!(posture);
                    report["sign"] = json!(sign);
                    report["topology"] = json!("synthetic impulse-joint pair");
                    report["productionMultibodyTorqueReadbackCertified"] = json!(false);
                    report["loadedChainCertified"] = json!(false);
                    reports.push(report);
                }
            }
        }
    }
    validate_report_coverage(&reports)?;
    Ok(reports)
}

fn active_trial(
    profile: &Profile,
    anatomy: &lh_model::Joint,
    heading: f32,
    relative: Rotation,
    sign: f32,
    scale: f32,
) -> Result<Value, String> {
    let mut world = PhysicsWorld::new();
    configure(&mut world, profile);
    world.gravity = Vector::ZERO;
    let parent_rotation = Rotation::from_rotation_y(heading);
    let child_rotation = parent_rotation
        * rotation(anatomy.parent_frame.rotation)
        * relative
        * rotation(anatomy.child_frame.rotation).inverse();
    let make_body = |rotation: Rotation| {
        RigidBodyBuilder::dynamic()
            .can_sleep(false)
            .rotation(rotation.to_scaled_axis())
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 2.0, Vector::ONE))
    };
    let parent = world.insert_body(make_body(parent_rotation));
    let child = world.insert_body(make_body(child_rotation));
    // No translational locks or external torque: momentum changes
    // can be compared directly to the reported angular motor rows.
    let mut joint = GenericJointBuilder::new(JointAxesMask::empty())
        .local_frame1(Pose::from_parts(
            Vector::ZERO,
            rotation(anatomy.parent_frame.rotation),
        ))
        .local_frame2(Pose::from_parts(
            Vector::ZERO,
            rotation(anatomy.child_frame.rotation),
        ))
        .build();
    let mut caps = [0.0_f32; 3];
    joint.enforce_motor_vector_limit = true;
    for axis in &anatomy.axes {
        let k = match axis.coordinate {
            'x' => 0,
            'y' => 1,
            _ => 2,
        };
        caps[k] = axis.max_motor_torque_nm * scale;
        joint
            .set_motor_model(crate::axis(axis.coordinate)?, MotorModel::ForceBased)
            .set_motor_velocity(crate::axis(axis.coordinate)?, sign * 10.0, 100.0)
            .set_motor_max_force(crate::axis(axis.coordinate)?, caps[k]);
    }
    let handle = world.insert_impulse_joint(parent, child, joint);
    let mut samples = Vec::new();
    let mut passed = true;
    for substep in 0..4 {
        let before = [world.bodies[parent].angvel(), world.bodies[child].angvel()];
        world.step();
        let motors = &world
            .impulse_joints
            .get(handle)
            .ok_or("Lost actuator probe joint")?
            .data
            .motors[3..];
        let delivered: [Vector; 2] = [
            motors.iter().map(|m| m.delivered_angular_impulse_1).sum(),
            motors.iter().map(|m| m.delivered_angular_impulse_2).sum(),
        ];
        let actual = [
            world.bodies[parent].angvel() - before[0],
            world.bodies[child].angvel() - before[1],
        ];
        let readback_error = (actual[0] - delivered[0])
            .length()
            .max((actual[1] - delivered[1]).length());
        let momentum_error = (actual[0] + actual[1]).length();
        let opposite_error = (delivered[0] + delivered[1]).length();
        let bound = Vector::from_array(caps).length() * profile.dt_s;
        let vector_impulse = delivered[0].length().max(delivered[1].length());
        let axis_passed = motors
            .iter()
            .enumerate()
            .all(|(k, m)| m.delivered_abs_impulse <= caps[k] * profile.dt_s + 1e-6);
        let sample_passed = axis_passed
            && readback_error <= 1e-6
            && momentum_error <= 1e-6
            && opposite_error <= 1e-6
            && vector_impulse <= bound + 1e-6
            && vector_impulse > 0.0;
        passed &= sample_passed;
        samples.push(json!({"substep":substep,"passed":sample_passed,"axisPassed":axis_passed,
            "vectorImpulseNmS":vector_impulse,"vectorBoundNmS":bound,"parentImpulseNmS":delivered[0].to_array(),
            "childImpulseNmS":delivered[1].to_array(),"readbackErrorNmS":readback_error,
            "bodyMomentumResidualNmS":momentum_error,"oppositeResidualNmS":opposite_error,"numericalToleranceNmS":1e-6}));
    }
    Ok(
        json!({"passed":passed,"samples":samples,"postureApplicable":true,
        "scope":"Canonical active angular motor coordinates and unchanged axis/vector ceilings; positive delivered angular impulse and independent equal/opposite momentum on free isotropic pairs over four unchanged substeps"}),
    )
}

/// The attachment challenge is deliberately large enough that deleting the
/// locks violates the unchanged anatomical bounds within the same four steps.
/// These are externally applied impulses F*dt and torque*dt, never motor effort.
fn fixed_trial(
    profile: &Profile,
    anatomy: &lh_model::Joint,
    heading: f32,
    posture: &str,
    sign: f32,
    locked: bool,
) -> Result<Value, String> {
    let mut world = PhysicsWorld::new();
    configure(&mut world, profile);
    world.gravity = Vector::ZERO;
    let pr = Rotation::from_rotation_y(heading);
    let cr = pr
        * rotation(anatomy.parent_frame.rotation)
        * rotation(anatomy.child_frame.rotation).inverse();
    let child_position = pr * crate::vector(anatomy.parent_frame.anchor)
        - cr * crate::vector(anatomy.child_frame.anchor);
    let make_body = |position, rotation: Rotation| {
        RigidBodyBuilder::dynamic()
            .pose(Pose::from_parts(position, rotation))
            .can_sleep(false)
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 2.0, Vector::ONE))
    };
    let parent = world.insert_body(make_body(Vector::ZERO, pr));
    let child = world.insert_body(make_body(child_position, cr));
    // Builder-supplied additional mass is initialized lazily by the pipeline.
    // Resolve it before applying the very first impulse, without an extra step.
    for body in [parent, child] {
        world.bodies[body].recompute_mass_properties_from_colliders(&world.colliders);
    }
    let mut joint = crate::anatomical_joint(anatomy)?;
    if !locked {
        joint.locked_axes = JointAxesMask::empty();
    }
    // Disabled rows normally retain Rapier's unbounded default max_force.
    // Make the fixture's zero allocation explicit without enabling any motor.
    for motor in &mut joint.motors {
        motor.max_force = 0.0;
    }
    let handle = world.insert_impulse_joint(parent, child, joint);
    let challenge_scale = match posture {
        "near-min" => 1.1,
        "near-max" => 1.2,
        _ => 1.0,
    };
    let mut passed = true;
    let mut samples = Vec::new();
    for substep in 0..SUBSTEPS {
        let frame = *world.bodies[parent].rotation() * rotation(anatomy.parent_frame.rotation);
        let force =
            frame * Vector::new(2.0, -3.0, 4.0).normalize() * (20_000.0 * challenge_scale * sign);
        let torque =
            frame * Vector::new(3.0, 4.0, -2.0).normalize() * (8_000.0 * challenge_scale * sign);
        let before = [world.bodies[parent].angvel(), world.bodies[child].angvel()];
        let before_linear = [world.bodies[parent].linvel(), world.bodies[child].linvel()];
        world.bodies[parent].apply_impulse(-force * profile.dt_s, true);
        world.bodies[child].apply_impulse(force * profile.dt_s, true);
        world.bodies[parent].apply_torque_impulse(-torque * profile.dt_s, true);
        world.bodies[child].apply_torque_impulse(torque * profile.dt_s, true);
        let challenged = [world.bodies[parent].angvel(), world.bodies[child].angvel()];
        let challenged_linear = [world.bodies[parent].linvel(), world.bodies[child].linvel()];
        let applied_linear: [f32; 2] =
            std::array::from_fn(|i| (challenged_linear[i] - before_linear[i]).length() * 2.0);
        let applied_angular: [f32; 2] =
            std::array::from_fn(|i| (challenged[i] - before[i]).length());
        let challenge_passed = applied_linear
            .iter()
            .chain(&applied_angular)
            .all(|v| v.is_finite() && *v > 0.0);
        world.step();
        let joint = world
            .impulse_joints
            .get(handle)
            .ok_or("Lost fixed attachment probe joint")?;
        let after = [world.bodies[parent].angvel(), world.bodies[child].angvel()];
        let delivered: [Vector; 2] = [
            joint
                .data
                .motors
                .iter()
                .map(|m| m.delivered_angular_impulse_1)
                .sum(),
            joint
                .data
                .motors
                .iter()
                .map(|m| m.delivered_angular_impulse_2)
                .sum(),
        ];
        let motor_work = delivered[0].dot((before[0] + after[0]) * 0.5)
            + delivered[1].dot((before[1] + after[1]) * 0.5);
        let zero_motor = joint.data.motor_axes.is_empty()
            && joint.data.motors.iter().all(|m| {
                m.max_force == 0.0
                    && m.impulse == 0.0
                    && m.delivered_impulse == 0.0
                    && m.delivered_abs_impulse == 0.0
                    && m.delivered_angular_impulse_1 == Vector::ZERO
                    && m.delivered_angular_impulse_2 == Vector::ZERO
            })
            && motor_work == 0.0;
        let anchor_error = world.bodies[parent]
            .position()
            .transform_point(crate::vector(anatomy.parent_frame.anchor))
            .distance(
                world.bodies[child]
                    .position()
                    .transform_point(crate::vector(anatomy.child_frame.anchor)),
            );
        let coordinates = crate::measurement::coordinates(
            *world.bodies[parent].rotation(),
            *world.bodies[child].rotation(),
            anatomy,
        );
        let locked_error = Vector::from_array(coordinates).length();
        // Independent velocity change after the external challenge establishes
        // a real constraint reaction; it is not counted as delivered motor work.
        let angular_reaction = (after[0] - challenged[0])
            .length()
            .max((after[1] - challenged[1]).length());
        let linear_reaction = (world.bodies[parent].linvel() - challenged_linear[0])
            .length()
            .max((world.bodies[child].linvel() - challenged_linear[1]).length())
            * 2.0;
        // These cached row values can be overwritten by the final nonlinear
        // correction (including zero). They are not accumulated step reactions.
        // Nonvacuity is established by the independent body changes above.
        let linear_rows =
            Vector::new(joint.impulses[0], joint.impulses[1], joint.impulses[2]).length();
        let angular_rows =
            Vector::new(joint.impulses[3], joint.impulses[4], joint.impulses[5]).length();
        let constraint_passed = joint.data.locked_axes == JointAxesMask::LOCKED_FIXED_AXES
            && anchor_error.is_finite()
            && anchor_error <= crate::measurement::MAX_ANCHOR_M
            && locked_error.is_finite()
            && locked_error <= crate::measurement::MAX_LIMIT_RAD
            && linear_rows.is_finite()
            && angular_rows.is_finite()
            && linear_reaction.is_finite()
            && linear_reaction > 0.0
            && angular_reaction.is_finite()
            && angular_reaction > 0.0;
        let sample_passed = zero_motor
            && constraint_passed
            && challenge_passed
            && force.is_finite()
            && force.length() > 0.0
            && torque.is_finite()
            && torque.length() > 0.0;
        passed &= sample_passed;
        samples.push(json!({"substep":substep,"passed":sample_passed,"zeroMotorPassed":zero_motor,
            "constraintPassed":constraint_passed,"challengePassed":challenge_passed,
            "appliedLinearChallengeImpulseNs":applied_linear,"appliedAngularChallengeImpulseNmS":applied_angular,
            "motorAxesBits":joint.data.motor_axes.bits(),
            "lockedAxesBits":joint.data.locked_axes.bits(),"axisCapsNm":[0.0,0.0,0.0],
            "motorMaxForces":joint.data.motors.iter().map(|m|m.max_force).collect::<Vec<_>>(),
            "motorDeliveredImpulse":joint.data.motors.iter().map(|m|m.delivered_impulse).collect::<Vec<_>>(),
            "motorDeliveredAbsImpulse":joint.data.motors.iter().map(|m|m.delivered_abs_impulse).collect::<Vec<_>>(),
            "vectorImpulseNmS":delivered[0].length().max(delivered[1].length()),"vectorBoundNmS":0.0,
            "parentImpulseNmS":delivered[0].to_array(),"childImpulseNmS":delivered[1].to_array(),
            "motorWorkJ":motor_work,"externalParentForceN":(-force).to_array(),"externalChildForceN":force.to_array(),
            "externalParentTorqueNm":(-torque).to_array(),"externalChildTorqueNm":torque.to_array(),
            "constraintLinearReactionNs":linear_reaction,"constraintAngularReactionNmS":angular_reaction,
            "lastSolverLinearRowImpulseNs":linear_rows,"lastSolverAngularRowImpulseNmS":angular_rows,
            "anchorErrorM":anchor_error,"lockedAngularErrorRad":locked_error,
            "anchorThresholdM":crate::measurement::MAX_ANCHOR_M,"lockedAngularThresholdRad":crate::measurement::MAX_LIMIT_RAD}));
    }
    Ok(
        json!({"passed":passed,"samples":samples,"postureApplicable":false,
        "canonicalParentFrame":anatomy.parent_frame,"canonicalChildFrame":anatomy.child_frame,
        "scope":"Fixed anatomical attachment under nonzero differential external force and torque; actual locked frames/anchors, zero motor axes/caps/delivery/work, independently measured nonzero constraint reactions and unchanged anatomical bounds. Posture labels vary challenge magnitude because fixed attachments have no angular coordinates."}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn free_isotropic_spin_preserves_world_angular_velocity() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut world = PhysicsWorld::new();
            configure(&mut world, &Profile::default());
            world.gravity = Vector::ZERO;
            let angular_velocity = Vector::new(0.31, -0.57, 0.83);
            let body = world.insert_body(
                RigidBodyBuilder::dynamic()
                    .can_sleep(false)
                    .rotation(Vector::Y * heading)
                    .angvel(angular_velocity)
                    .additional_mass_properties(MassProperties::new(
                        Vector::ZERO,
                        2.0,
                        Vector::ONE,
                    )),
            );
            for substep in 0..4 {
                world.step();
                assert_eq!(
                    world.bodies[body].angvel(),
                    angular_velocity,
                    "spherical inertia has no gyroscopic torque: heading {heading}, substep {substep}"
                );
            }
        }
    }

    #[test]
    fn anatomical_motor_delivery_satisfies_all_joint_headings_postures_and_directions() {
        let reports = trials(&Profile::default()).expect("actuator observations");
        assert_eq!(reports.len(), CASES);
        assert!(
            reports
                .iter()
                .all(|r| r["samples"].as_array().is_some_and(|s| s.len() == 4))
        );
        assert_eq!(
            reports
                .iter()
                .filter(|r| r["identity"]["fixtureKind"] == "active-hinge")
                .count(),
            ACTIVE_CASES
        );
        assert_eq!(
            reports
                .iter()
                .filter(|r| r["identity"]["fixtureKind"] == "fixed-attachment")
                .count(),
            FIXED_CASES
        );
        assert_eq!(
            reports
                .iter()
                .filter(|r| r["identity"]["masslessCoordinateFrame"] == true)
                .count(),
            9 * 18
        );
        for report in &reports {
            assert_eq!(report["passed"], true, "fixture contract failed: {report}");
            for sample in report["samples"].as_array().unwrap() {
                if report["identity"]["fixtureKind"] == "active-hinge" {
                    assert!(sample["vectorImpulseNmS"].as_f64().unwrap() > 0.0);
                } else {
                    assert_eq!(sample["zeroMotorPassed"], true);
                    assert_eq!(sample["constraintPassed"], true);
                    assert_eq!(sample["motorWorkJ"], 0.0);
                    assert!(sample["constraintLinearReactionNs"].as_f64().unwrap() > 0.0);
                    assert!(sample["constraintAngularReactionNmS"].as_f64().unwrap() > 0.0);
                }
            }
        }
        assert!(reports_passed(&reports));
    }

    #[test]
    fn canonical_classification_preserves_every_hinge_and_massless_coordinate() {
        let model = lh_model::Model::canonical().unwrap();
        let categories = categories(&model).unwrap();
        let mut fixed: Vec<_> = categories
            .iter()
            .filter(|(_, k)| *k == FixtureKind::FixedAttachment)
            .map(|(s, _)| s.id.as_str())
            .collect();
        fixed.sort_unstable();
        assert_eq!(fixed, ["head", "left_hand", "right_hand"]);
        let qpos: std::collections::BTreeSet<_> = categories
            .iter()
            .filter_map(|(s, _)| s.reference.as_ref()?.hinge.as_ref().map(|h| h.qpos_index))
            .collect();
        let actuators: std::collections::BTreeSet<_> = categories
            .iter()
            .filter_map(|(s, _)| {
                s.reference
                    .as_ref()?
                    .hinge
                    .as_ref()
                    .map(|h| h.actuator_index)
            })
            .collect();
        assert_eq!(qpos, (0..21).collect());
        assert_eq!(actuators, (0..21).collect());
        assert!(
            categories
                .iter()
                .filter(|(s, _)| s.role == "coordinate")
                .all(|(s, k)| s.mass_kg == 0.0 && *k == FixtureKind::ActiveHinge)
        );
    }

    #[test]
    fn unlocked_fixed_controls_violate_the_unchanged_anatomical_bounds() {
        let model = lh_model::Model::canonical().unwrap();
        let mut controls = 0;
        for (segment, kind) in categories(&model).unwrap() {
            if kind != FixtureKind::FixedAttachment {
                continue;
            }
            for heading in HEADINGS {
                for posture in POSTURES {
                    for sign in [-1.0, 1.0] {
                        let report = fixed_trial(
                            &Profile::default(),
                            segment.joint_profile.as_ref().unwrap(),
                            heading,
                            posture,
                            sign,
                            false,
                        )
                        .unwrap();
                        assert_eq!(report["passed"], false);
                        let samples = report["samples"].as_array().unwrap();
                        assert!(
                            samples.iter().any(|s| s["anchorErrorM"].as_f64().unwrap()
                                > f64::from(crate::measurement::MAX_ANCHOR_M)),
                            "Unlocked anchor control failed to challenge {}: {report}",
                            segment.id
                        );
                        assert!(
                            samples
                                .iter()
                                .any(|s| s["lockedAngularErrorRad"].as_f64().unwrap()
                                    > f64::from(crate::measurement::MAX_LIMIT_RAD)),
                            "Unlocked angular control failed to challenge {}: {report}",
                            segment.id
                        );
                        assert!(samples.iter().all(|s| s["zeroMotorPassed"] == true));
                        controls += 1;
                    }
                }
            }
        }
        assert_eq!(controls, FIXED_CASES);
    }

    #[test]
    fn coverage_rejects_duplicate_mismatched_and_empty_diagnostics() {
        let mut reports = trials(&Profile::default()).unwrap();
        let first = reports[0].clone();
        let second = reports[1].clone();
        reports[1] = first.clone();
        assert!(validate_report_coverage(&reports).is_err());
        reports[1] = second;
        reports[0]["identity"]["fixtureKind"] = json!("active-hinge");
        assert!(validate_report_coverage(&reports).is_err());
        reports[0] = first.clone();
        reports[0]["samples"] = json!([]);
        assert!(validate_report_coverage(&reports).is_err());
        reports[0] = first;
        reports[0]["samples"][0]["passed"] = json!(false);
        assert!(!reports_passed(&reports));
        assert!(!reports_passed(&[]));
    }

    #[test]
    fn zero_allocation_cannot_pass_active_delivery() {
        let profile = Profile {
            motor_scale: 0.0,
            ..Profile::default()
        };
        let reports = trials(&profile).unwrap();
        assert!(!reports_passed(&reports));
        assert!(
            reports
                .iter()
                .filter(|r| r["identity"]["fixtureKind"] == "active-hinge")
                .all(|r| r["passed"] == false)
        );
        assert!(
            reports
                .iter()
                .filter(|r| r["identity"]["fixtureKind"] == "fixed-attachment")
                .all(|r| r["passed"] == true)
        );
    }
}
