//! Contact impulses checked independently against linear momentum balance.
use crate::{Profile, configure};
use rapier3d::prelude::*;
use serde::Serialize;
pub mod tangent;
#[cfg(feature = "experimental-solver-step-observation")]
pub mod contact_interval_probe;

#[derive(Debug, Serialize)]
pub struct LoadCalibration {
    pub measurement_schema: u16,
    pub warmstart: f32,
    pub scenario: String,
    pub samples: u32,
    pub max_impulse_error_n_s: f32,
    pub mean_load_n: f32,
    pub expected_rest_load_n: f32,
    pub passed: bool,
}
pub fn contact_load(warmstart: f32, scenario: &str) -> Result<LoadCalibration, String> {
    contact_load_with_profile(warmstart, scenario, &Profile::default())
}
pub fn contact_load_with_profile(
    warmstart: f32,
    scenario: &str,
    profile: &Profile,
) -> Result<LoadCalibration, String> {
    let mut w = PhysicsWorld::new();
    configure(&mut w, profile);
    w.integration_parameters.warmstart_coefficient = warmstart;
    let floor = w.insert_collider(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0)
            .translation(Vector::new(0.0, -0.5, 0.0))
            .friction(0.0),
        None,
    );
    let (body, collider) = w.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .can_sleep(false),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .mass(10.0)
            .friction(0.0)
            .restitution(0.0),
    );
    let dt = w.integration_parameters.dt;
    let mass = 10.0_f32;
    for _ in 0..480 {
        w.step();
    }
    let mut max_error = 0.0_f32;
    let mut load_sum = 0.0;
    let mut passed = true;
    for step in 0..480 {
        let force = match scenario {
            "rest" => 0.0,
            "changing" => {
                if step < 240 {
                    -30.0
                } else {
                    20.0
                }
            }
            "unloading" => 110.0,
            "contact-loss" => 0.0,
            _ => return Err("Unknown load fixture".into()),
        };
        if scenario == "contact-loss" && step == 0 {
            w.colliders[floor].set_enabled(false);
        }
        let before = w.bodies[body].linvel().y;
        w.bodies[body].reset_forces(false);
        w.bodies[body].add_force(Vector::Y * force, true);
        w.step();
        let after = w.bodies[body].linvel().y;
        let impulse = crate::measurement::floor_normal_impulse(&w, floor, collider)?;
        let momentum_expected = mass * (after - before) - (-mass * 9.81 + force) * dt;
        let error = (impulse - momentum_expected).abs();
        max_error = max_error.max(error);
        load_sum += impulse / dt;
        // Fixed absolute error, not an arbitrary correction to the signal.
        passed &= error <= 0.002 && impulse.is_finite();
        if scenario == "contact-loss" {
            passed &= impulse == 0.0;
        }
    }
    let mean_load = load_sum / 480.0;
    if scenario == "rest" {
        passed &= (mean_load - 98.1).abs() <= 0.981;
    }
    Ok(LoadCalibration {
        measurement_schema: 2,
        warmstart,
        scenario: scenario.into(),
        samples: 480,
        max_impulse_error_n_s: max_error,
        mean_load_n: mean_load,
        expected_rest_load_n: 98.1,
        passed,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn delivered_motor_impulse_matches_independent_opposite_body_momentum() {
        let mut world = PhysicsWorld::new();
        configure(&mut world, &Profile::default());
        world.gravity = Vector::ZERO;
        let build = || {
            RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .additional_mass_properties(MassProperties::new(Vector::ZERO, 2.0, Vector::ONE))
        };
        let parent = world.insert_body(build());
        let child = world.insert_body(build());
        let mut joint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES).build();
        joint
            .set_motor_model(JointAxis::AngX, MotorModel::ForceBased)
            .set_motor_velocity(JointAxis::AngX, 10.0, 100.0)
            .set_motor_max_force(JointAxis::AngX, 4.0);
        let handle = world.insert_impulse_joint(parent, child, joint);
        {
            let before = [world.bodies[parent].angvel(), world.bodies[child].angvel()];
            world.step();
            let motor = world.impulse_joints.get(handle).unwrap().data.motors[3];
            let change = [
                world.bodies[parent].angvel() - before[0],
                world.bodies[child].angvel() - before[1],
            ];
            assert!(motor.delivered_abs_impulse > 0.0);
            assert!(motor.delivered_abs_impulse <= 4.0 * world.integration_parameters.dt + 1e-6);
            assert!((change[0] + change[1]).length() <= 1e-6);
            assert!(
                (change[0].length() - motor.delivered_abs_impulse).abs() <= 1e-6,
                "actual={:?}, delivered={motor:?}",
                change
            );
            assert!((change[1].length() - motor.delivered_abs_impulse).abs() <= 1e-6);
            assert!((change[0] - motor.delivered_angular_impulse_1).length() <= 1e-6);
            assert!((change[1] - motor.delivered_angular_impulse_2).length() <= 1e-6);
        }
    }
    #[test]
    fn contact_load_matches_momentum_for_rest_change_unloading_and_loss() {
        for warm in [0.0, 0.5, 1.0] {
            for scenario in ["rest", "changing", "unloading", "contact-loss"] {
                let r = contact_load(warm, scenario).expect("fixture");
                assert!(r.passed, "{r:?}");
            }
        }
    }
    #[test]
    fn canonical_contacts_follow_momentum_while_cached_point_totals_do_not() {
        let reports = canonical_contact_load().expect("canonical contact fixtures");
        let feet = lh_model::Model::canonical().unwrap().segments.iter()
            .filter(|s| matches!(s.role.as_str(), "hindfoot" | "forefoot")).count();
        assert_eq!(feet, 2, "canonical anatomical feet");
        assert_eq!(reports.len(), feet * 3 * 3 * 4);
        assert!(reports.iter().all(|r| r["passed"] == true), "{reports:?}");
        // The test is discriminating: the old cached-point sum fails on the
        // same trajectories and prescribed loads, without changing any solver.
        assert!(reports.iter().any(|r| {
            r["rawCachedPairMaxErrorNs"]
                .as_f64()
                .is_some_and(|e| e > 0.002)
        }));
    }

    #[test]
    fn tilted_contact_requires_signed_vertical_friction_in_momentum_audit() {
        let mut profile = Profile::default();
        profile.id = "getup-800hz-4plus1-qualification-v1".into();
        profile.solver_iterations = 4;
        profile.additional_body_iterations = 1;
        let model = lh_model::Model::canonical().unwrap();
        let foot = model.segments.iter().find(|s| s.id == "left_foot").unwrap();
        let (mut w, body, floor, collider) = foot_fixture(foot, -std::f32::consts::PI / 4.0, true, &profile).unwrap();
        w.integration_parameters.warmstart_coefficient = 0.0;
        let ballast = (72.2 / 2.0 - foot.mass_kg) * 9.81;
        let mut first = None;
        let mut peak = 0.0;
        for step in 0..=398 {
            let before = w.bodies[body].linvel();
            w.bodies[body].reset_forces(false);
            w.bodies[body].add_force(-Vector::Y * ballast, true);
            step_foot_fixture(&mut w);
            audit_foot_load(&w, (body, floor, collider, ballast), (step, profile.tick_substeps), before, &mut first, &mut peak).unwrap();
            if step == 398 {
                let expected = w.bodies[body].mass() * (w.bodies[body].linvel() - before)
                    - (w.gravity * w.bodies[body].mass() - Vector::Y * ballast) * profile.dt_s;
                let normal = crate::measurement::floor_normal_impulse(&w, floor, collider).unwrap();
                let tangent = crate::measurement::floor_tangent_impulse(&w, floor, collider).unwrap();
                let total = crate::measurement::floor_contact_impulse(&w, floor, collider).unwrap();
                assert!((normal - expected.y).abs() > 0.01, "normal-only audit must be discriminated");
                assert!(tangent.y > 0.01, "world tangent has a vertical component on a tilted manifold");
                assert!((total - expected).length() < 0.002);
                let opposite = crate::measurement::floor_contact_impulse(&w, collider, floor).unwrap();
                assert_eq!(total, -opposite, "opposite collider ordering must reverse every impulse component");
            }
        }
        assert!(first.is_none(), "{first:?}");
        assert!(peak < 0.002);
        w.physics_pipeline.counters.ccd.num_substeps = 2;
        assert!(audit_foot_load(&w, (body, floor, collider, ballast), (399, profile.tick_substeps), w.bodies[body].linvel(), &mut first, &mut peak).unwrap_err().contains("single solver interval"));
    }

    #[test]
    fn explicit_4plus1_contacts_pass_the_same_full_vector_momentum_bound() {
        let mut profile = Profile::default();
        profile.id = "getup-800hz-4plus1-qualification-v1".into();
        profile.solver_iterations = 4;
        profile.additional_body_iterations = 1;
        let reports = canonical_contact_load_with_profile(&profile).unwrap();
        assert_eq!(reports.len(), 2 * 3 * 3 * 4);
        assert!(reports.iter().all(|r| r["passed"] == true), "{reports:?}");
        assert!(reports.iter().all(|r| r["maxMomentumImpulseErrorNs"].as_f64().unwrap() <= 0.002));
    }
}

#[derive(Debug, Serialize)]
pub struct JointCalibration {
    pub segment: String,
    pub heading: f32,
    pub direction: i32,
    pub combined: bool,
    pub completed_substeps: u32,
    pub max_limit_error_rad: f32,
    pub max_anchor_m: f32,
    pub first_failure: Option<super::measurement::Failure>,
    pub passed: bool,
}
pub fn isolated_joints() -> Result<Vec<JointCalibration>, String> {
    isolated_joints_with_profile(&Profile::default())
}
pub fn isolated_joints_with_profile(profile: &Profile) -> Result<Vec<JointCalibration>, String> {
    let model = lh_model::Model::canonical()?;
    let mut reports = Vec::new();
    for s in &model.segments {
        let Some(p) = &s.joint_profile else {
            continue;
        };
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            for direction in [-1, 1] {
                for combined in if p.axes.len() > 1 {
                    vec![false, true]
                } else {
                    vec![false]
                } {
                    let mut w = PhysicsWorld::new();
                    configure(&mut w, profile);
                    w.gravity = Vector::ZERO;
                    let pr = Rotation::from_rotation_y(heading);
                    let cr = pr
                        * super::rotation(p.parent_frame.rotation)
                        * super::rotation(p.child_frame.rotation).inverse();
                    let child_pos = pr * super::vector(p.parent_frame.anchor)
                        - cr * super::vector(p.child_frame.anchor);
                    let parent = w.insert_body(
                        RigidBodyBuilder::dynamic()
                            .pose(Pose::from_parts(Vector::ZERO, pr))
                            .additional_mass_properties(MassProperties::new(
                                Vector::ZERO,
                                8.0,
                                Vector::ONE,
                            ))
                            .can_sleep(false),
                    );
                    let child = w.insert_body(
                        RigidBodyBuilder::dynamic()
                            .pose(Pose::from_parts(child_pos, cr))
                            .additional_mass_properties(MassProperties::new(
                                Vector::ZERO,
                                5.0,
                                Vector::splat(0.6),
                            ))
                            .can_sleep(false),
                    );
                    w.insert_impulse_joint(parent, child, super::anatomical_joint(p)?);
                    let mut max_error = 0.0_f32;
                    let mut max_anchor = 0.0_f32;
                    let mut failure = None;
                    let mut completed = 0;
                    for substep in 0..960 {
                        let frame =
                            *w.bodies[parent].rotation() * super::rotation(p.parent_frame.rotation);
                        let mut torque = Vector::ZERO;
                        for (i, a) in p.axes.iter().enumerate() {
                            let v = match a.coordinate {
                                'x' => Vector::X,
                                'y' => Vector::Y,
                                _ => Vector::Z,
                            };
                            let sign = if combined && i % 2 != 0 {
                                -direction
                            } else {
                                direction
                            };
                            torque += frame * v * (sign as f32 * 220.0);
                        }
                        w.bodies[child]
                            .apply_torque_impulse(torque * w.integration_parameters.dt, true);
                        w.bodies[parent]
                            .apply_torque_impulse(-torque * w.integration_parameters.dt, true);
                        w.step();
                        completed = substep + 1;
                        let q = super::measurement::coordinates(
                            *w.bodies[parent].rotation(),
                            *w.bodies[child].rotation(),
                            p,
                        );
                        let mut error_sq = 0.0_f32;
                        for (k, c) in ['x', 'y', 'z'].iter().enumerate() {
                            let e = p
                                .axes
                                .iter()
                                .find(|a| a.coordinate == *c)
                                .map_or(q[k].abs(), |a| {
                                    (a.min_radians - q[k]).max(q[k] - a.max_radians).max(0.0)
                                });
                            error_sq += e * e;
                        }
                        let error = error_sq.sqrt();
                        max_error = max_error.max(error);
                        let anchor = w.bodies[parent]
                            .position()
                            .transform_point(super::vector(p.parent_frame.anchor))
                            .distance(
                                w.bodies[child]
                                    .position()
                                    .transform_point(super::vector(p.child_frame.anchor)),
                            );
                        max_anchor = max_anchor.max(anchor);
                        if error > super::measurement::MAX_LIMIT_RAD
                            || anchor > super::measurement::MAX_ANCHOR_M
                            || !error.is_finite()
                            || !anchor.is_finite()
                        {
                            let (check, measured, threshold) =
                                if !error.is_finite() || !anchor.is_finite() {
                                    ("finite", 0.0, 0.0)
                                } else if error > super::measurement::MAX_LIMIT_RAD {
                                    ("joint-limit", error, super::measurement::MAX_LIMIT_RAD)
                                } else {
                                    ("joint-anchor", anchor, super::measurement::MAX_ANCHOR_M)
                                };
                            failure = Some(super::measurement::Failure::new(
                                substep as u64 / 4,
                                substep % 4,
                                check,
                                s.id.clone(),
                                measured,
                                threshold,
                            ));
                            break;
                        }
                    }
                    reports.push(JointCalibration {
                        segment: s.id.clone(),
                        heading,
                        direction,
                        combined,
                        completed_substeps: completed,
                        max_limit_error_rad: max_error,
                        max_anchor_m: max_anchor,
                        passed: failure.is_none(),
                        first_failure: failure,
                    });
                }
            }
        }
    }
    Ok(reports)
}

#[derive(Debug, Serialize)]
pub struct FootCalibration {
    pub measurement_schema: u16,
    pub segment: String,
    pub heading: f32,
    pub completed_substeps: u32,
    pub max_drift_m: f32,
    pub max_penetration_m: f32,
    pub mean_load_n: f32,
    pub raw_cached_pair_mean_load_n: f32,
    pub ccd_enabled: bool,
    pub actual_mass_kg: f32,
    pub expected_load_n: f32,
    pub max_momentum_impulse_error_n_s: f32,
    pub first_load_failure: Option<serde_json::Value>,
    pub passed: bool,
}

fn foot_fixture(
    s: &lh_model::Segment,
    heading: f32,
    ccd: bool,
    profile: &Profile,
) -> Result<
    (
        PhysicsWorld,
        RigidBodyHandle,
        ColliderHandle,
        ColliderHandle,
    ),
    String,
> {
    let mut w = PhysicsWorld::new();
    configure(&mut w, profile);
    if !ccd {
        w.integration_parameters.max_ccd_substeps = 0;
    }
    let floor = w.insert_collider(
        ColliderBuilder::cuboid(12.0, 0.08, 10.5)
            .translation(Vector::new(0.0, -0.08, 0.0))
            .friction(4.0),
        None,
    );
    let m = lh_model::integrate_mass(&s.geometry, s.mass_kg)?;
    let points = s
        .geometry
        .vertices
        .chunks_exact(3)
        .map(|p| Vector::new(p[0], p[1], p[2]))
        .collect();
    let triangles: Vec<_> = s
        .geometry
        .indices
        .chunks_exact(3)
        .map(|i| [i[0], i[1], i[2]])
        .collect();
    let shape = ColliderBuilder::convex_mesh(points, &triangles)
        .ok_or("Foot mesh")?
        .mass_properties(MassProperties::with_principal_inertia_frame(
            super::vector(m.center),
            s.mass_kg,
            super::vector(m.principal),
            super::rotation(m.frame),
        ))
        .friction(4.0)
        .contact_skin(0.004);
    let minimum_y = s
        .geometry
        .vertices
        .chunks_exact(3)
        .map(|p| p[1])
        .fold(f32::INFINITY, f32::min);
    let (body, collider) = w.insert(
        RigidBodyBuilder::dynamic()
            .pose(Pose::from_parts(
                Vector::new(0.0, -minimum_y + 0.004, 0.0),
                Rotation::from_rotation_y(heading),
            ))
            .can_sleep(false)
            .ccd_enabled(ccd),
        shape,
    );
    Ok((w, body, floor, collider))
}

/// Independent momentum balance on actual foot surfaces, including loss of
/// contact and changing selection of reduced manifold points. No motor acts.
pub fn canonical_contact_load() -> Result<Vec<serde_json::Value>, String> {
    canonical_contact_load_with_profile(&Profile::default())
}
pub fn canonical_contact_load_with_profile(
    profile: &Profile,
) -> Result<Vec<serde_json::Value>, String> {
    let model = lh_model::Model::canonical()?;
    let mut reports = Vec::new();
    for s in model
        .segments
        .iter()
        .filter(|s| matches!(s.role.as_str(), "hindfoot" | "forefoot"))
    {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            for warm in [0.0, 0.5, 1.0] {
                for scenario in ["rest", "changing", "unloading", "contact-loss"] {
                    let (mut w, body, floor, collider) = foot_fixture(s, heading, true, profile)?;
                    w.integration_parameters.warmstart_coefficient = warm;
                    let ballast = (72.2 / 2.0 - s.mass_kg) * 9.81;
                    let mut first = None;
                    let mut peak = 0.0;
                    let mut peak_raw = 0.0_f32;
                    let mut load = 0.0;
                    let mut contact_loss_pass = true;
                    for step in 0..960 {
                        let force = if step < 480 {
                            -ballast
                        } else {
                            match scenario {
                                "changing" => {
                                    if step < 720 {
                                        -ballast - 30.0
                                    } else {
                                        -ballast + 20.0
                                    }
                                }
                                "unloading" => s.mass_kg * 9.81 + 20.0,
                                _ => -ballast,
                            }
                        };
                        if scenario == "contact-loss" && step == 480 {
                            w.colliders[floor].set_enabled(false);
                        }
                        let before = w.bodies[body].linvel();
                        w.bodies[body].reset_forces(false);
                        w.bodies[body].add_force(Vector::Y * force, true);
                        step_foot_fixture(&mut w);
                        audit_foot_load(
                            &w,
                            (body, floor, collider, -force),
                            (step, profile.tick_substeps),
                            before,
                            &mut first,
                            &mut peak,
                        )?;
                        let impulse =
                            crate::measurement::floor_normal_impulse(&w, floor, collider)?;
                        let b = &w.bodies[body];
                        let expected = b.mass() * (b.linvel() - before)
                            - (w.gravity * (b.mass() * b.gravity_scale()) + b.user_force()) * w.integration_parameters.dt;
                        let raw = w
                            .narrow_phase
                            .contact_pair(floor, collider)
                            .map_or(Vector::ZERO, |p| p.total_impulse() * if p.collider2 == collider { 1.0 } else { -1.0 });
                        // Hold the actual signed tangent report fixed so this
                        // diagnostic isolates cached normal-point contamination.
                        let raw_total = raw + crate::measurement::floor_tangent_impulse(&w, floor, collider)?;
                        peak_raw = peak_raw.max((raw_total - expected).length());
                        if step >= 480 {
                            load += impulse / w.integration_parameters.dt;
                            if scenario == "contact-loss" {
                                contact_loss_pass &= impulse == 0.0;
                            }
                        }
                        let b = &w.bodies[body];
                        if !b.linvel().is_finite()
                            || !b.angvel().is_finite()
                            || !b.translation().is_finite()
                        {
                            return Err(format!("Non-finite contact fixture: {} {scenario}", s.id));
                        }
                    }
                    let mean = load / 480.0;
                    let passed = first.is_none()
                        && contact_loss_pass
                        && (scenario != "rest" || (mean - 354.141).abs() <= 3.0);
                    reports.push(
                        serde_json::json!({"measurementSchema":3,"momentumAudit":"signed-world-normal-plus-tangent-3d-single-interval","segment":s.id,"heading":heading,"warmstart":warm,
                        "scenario":scenario,"samples":960,"meanObservedLoadN":mean,"passed":passed,
                        "maxMomentumImpulseErrorNs":peak,"rawCachedPairMaxErrorNs":peak_raw,
                        "firstFailure":first,"contactLossPass":contact_loss_pass}),
                    );
                }
            }
        }
    }
    Ok(reports)
}
pub fn individual_feet() -> Result<Vec<FootCalibration>, String> {
    individual_feet_with_ccd(true)
}
pub fn individual_feet_with_ccd(ccd: bool) -> Result<Vec<FootCalibration>, String> {
    individual_feet_with_profile(ccd, &Profile::default())
}
pub fn individual_feet_with_profile(
    ccd: bool,
    profile: &Profile,
) -> Result<Vec<FootCalibration>, String> {
    let model = lh_model::Model::canonical()?;
    let mut reports = Vec::new();
    for s in model
        .segments
        .iter()
        .filter(|s| matches!(s.role.as_str(), "hindfoot" | "forefoot"))
    {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let (mut w, body, floor, collider) = foot_fixture(s, heading, ccd, profile)?;
            // Explicit test ballast via a known external load; original segment
            // mass/inertia remain unchanged. Not part of production control.
            let ballast = (72.2 / 2.0 - s.mass_kg) * 9.81;
            let mut first_load_failure = None;
            let mut max_momentum_error = 0.0_f32;
            for substep in 0..480 {
                w.bodies[body].reset_forces(false);
                w.bodies[body].add_force(-Vector::Y * ballast, true);
                let before = w.bodies[body].linvel();
                step_foot_fixture(&mut w);
                audit_foot_load(
                    &w,
                    (body, floor, collider, ballast),
                    (substep, profile.tick_substeps),
                    before,
                    &mut first_load_failure,
                    &mut max_momentum_error,
                )?;
                let c = rapier3d::parry::query::contact(
                    w.colliders[floor].position(),
                    w.colliders[floor].shape(),
                    w.colliders[collider].position(),
                    w.colliders[collider].shape(),
                    0.0,
                )
                .map_err(|e| format!("{e:?}"))?;
                if !w.bodies[body].translation().is_finite() || c.is_some_and(|c| c.dist < -0.08) {
                    return Err(format!("Foot fixture fails during settling: {}", s.id));
                }
            }
            let reference = w.bodies[body].translation();
            let mut drift = 0.0_f32;
            let mut penetration = 0.0_f32;
            let mut load = 0.0_f32;
            let mut raw_load = 0.0_f32;
            let mut completed = 0;
            let mut passed = true;
            for substep in 0..2400 {
                w.bodies[body].reset_forces(false);
                w.bodies[body].add_force(-Vector::Y * ballast, true);
                let before = w.bodies[body].linvel();
                step_foot_fixture(&mut w);
                audit_foot_load(
                    &w,
                    (body, floor, collider, ballast),
                    (480 + substep, profile.tick_substeps),
                    before,
                    &mut first_load_failure,
                    &mut max_momentum_error,
                )?;
                completed = substep + 1;
                drift = drift.max(reference.distance(w.bodies[body].translation()));
                let contact = rapier3d::parry::query::contact(
                    w.colliders[floor].position(),
                    w.colliders[floor].shape(),
                    w.colliders[collider].position(),
                    w.colliders[collider].shape(),
                    0.0,
                )
                .map_err(|e| format!("{e:?}"))?;
                penetration = penetration.max(contact.map_or(0.0, |c| (-c.dist).max(0.0)));
                let impulse = crate::measurement::floor_normal_impulse(&w, floor, collider)?;
                load += impulse / w.integration_parameters.dt;
                raw_load += w
                    .narrow_phase
                    .contact_pair(floor, collider)
                    .map_or(0.0, |p| p.total_impulse().y.abs())
                    / w.integration_parameters.dt;
                if !w.bodies[body].translation().is_finite() || drift > 0.01 || penetration > 0.08 {
                    passed = false;
                    break;
                }
            }
            let mean_load = load / completed as f32;
            passed &= completed == 2400
                && (mean_load - 72.2 / 2.0 * 9.81).abs() < 3.0
                && first_load_failure.is_none();
            reports.push(FootCalibration {
                measurement_schema: 3,
                segment: s.id.clone(),
                heading,
                completed_substeps: completed,
                max_drift_m: drift,
                max_penetration_m: penetration,
                mean_load_n: mean_load,
                raw_cached_pair_mean_load_n: raw_load / completed as f32,
                ccd_enabled: ccd,
                actual_mass_kg: w.bodies[body].mass(),
                expected_load_n: 72.2 / 2.0 * 9.81,
                max_momentum_impulse_error_n_s: max_momentum_error,
                first_load_failure,
                passed,
            });
        }
    }
    Ok(reports)
}

// The engine counter is reporting-only and otherwise accumulates when timers
// are disabled. Reset just this count to establish the report's interval scope.
fn step_foot_fixture(w: &mut PhysicsWorld) {
    w.physics_pipeline.counters.ccd.num_substeps = 0;
    w.step();
}

fn audit_foot_load(
    w: &PhysicsWorld,
    fixture: (RigidBodyHandle, ColliderHandle, ColliderHandle, f32),
    clock: (u32, u32),
    before: Vector,
    first: &mut Option<serde_json::Value>,
    peak: &mut f32,
) -> Result<(), String> {
    let (body, floor, collider, ballast) = fixture;
    let (step, tick_substeps) = clock;
    let b = &w.bodies[body];
    let dt = w.integration_parameters.dt;
    if w.physics_pipeline.counters.ccd.num_substeps != 1 {
        return Err("Foot momentum audit requires a single solver interval; latest-interval reports are not whole-step totals".into());
    }
    let reported = crate::measurement::floor_contact_impulse(w, floor, collider)?;
    let expected = b.mass() * (b.linvel() - before)
        - (w.gravity * (b.mass() * b.gravity_scale()) + b.user_force()) * dt;
    let error = (reported - expected).length();
    *peak = peak.max(error);
    if first.is_none() && error > 0.002 {
        *first = Some(
            serde_json::json!({"measurementSchema":3,"tick":step/tick_substeps,"substep":step%tick_substeps,"outerStep":step,"check":"contact-momentum-balance","impulseErrorNs":error,"thresholdNs":0.002,
            "reportedImpulseNs":super::vec3(reported),"expectedImpulseNs":super::vec3(expected),"dtS":dt,"externalBallastN":ballast,"massKg":b.mass(),
            "linearVelocityBefore":super::vec3(before),"linearVelocityAfter":super::vec3(b.linvel()),"position":super::vec3(b.translation()),"rotation":super::quat(*b.rotation())}),
        );
    }
    Ok(())
}

pub fn loaded_leg_chains() -> Result<Vec<serde_json::Value>, String> {
    loaded_leg_chains_with_profile(&Profile::default())
}
pub fn loaded_leg_chains_with_profile(profile: &Profile) -> Result<Vec<serde_json::Value>, String> {
    let mut reports = Vec::new();
    for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
        let mut s = super::Simulation::new(heading, profile.clone(), true)?;
        let retained: Vec<_> = s
            .model
            .segments
            .iter()
            .enumerate()
            .filter(|(_, s)| {
                matches!(
                    s.role.as_str(),
                    "pelvis" | "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot"
                )
            })
            .map(|(i, _)| i)
            .collect();
        if retained.len() != 11 {
            return Err(format!("Loaded-chain topology: {}", retained.len()));
        }
        for i in (0..s.bodies.len()).rev().filter(|i| !retained.contains(i)) {
            s.world.remove_body(s.bodies[i]);
        }
        s.model.segments = retained
            .iter()
            .map(|i| s.model.segments[*i].clone())
            .collect();
        s.bodies = retained.iter().map(|i| s.bodies[*i]).collect();
        s.colliders = retained.iter().map(|i| s.colliders[*i]).collect();
        s.joints = retained.iter().map(|i| s.joints[*i]).collect();
        s.targets = retained.iter().map(|i| s.targets[*i]).collect();
        s.rest_targets = retained.iter().map(|i| s.rest_targets[*i]).collect();
        s.capture_rest_com();
        let ballast = (72.2 - s.total_mass()) * 9.81;
        let mut failure = None;
        for _ in 0..720 {
            s.world.bodies[s.bodies[0]].reset_forces(false);
            s.world.bodies[s.bodies[0]].add_force(-Vector::Y * ballast, true);
            if let Err(f) = s.advance_tick_observed(|s, substep| {
                if s.tick >= 120 {
                    crate::measurement::standing_support(s, substep)
                } else {
                    Ok(())
                }
            }) {
                failure = Some(f);
                break;
            }
        }
        reports.push(serde_json::json!({"heading":heading,"passed":failure.is_none()&&s.tick==720,"fixture":"dynamic pelvis and both five-body leg chains; omitted upper-body weight applied as declared external test load; unchanged individual and aggregate motor caps; bilateral loaded feet and inherited up-vector threshold after two seconds settling", "completedTicks":s.tick,"externalBallastN":ballast,"firstFailure":failure,"final":s.snapshot()}));
    }
    Ok(reports)
}
