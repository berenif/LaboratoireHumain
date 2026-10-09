//! Independent motor delivery probes: free isotropic bodies isolate angular
//! momentum from gravity, contacts, damping and anatomical anchor reactions.
use crate::{Profile, configure, rotation};
use rapier3d::prelude::*;
use serde_json::{Value, json};

pub const CASES: usize = 24 * 3 * 3 * 2;
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
    for segment in &model.segments {
        let Some(anatomy) = &segment.joint_profile else {
            continue;
        };
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            for posture in ["neutral", "near-min", "near-max"] {
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
                // Invert the frozen centered quaternion-coordinate definition.
                let relative = Rotation::from_xyzw(
                    (coordinates[0] * 0.5).tan(),
                    (coordinates[1] * 0.5).tan(),
                    (coordinates[2] * 0.5).tan(),
                    1.0,
                )
                .normalize();
                for sign in [-1.0_f32, 1.0] {
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
                            .additional_mass_properties(MassProperties::new(
                                Vector::ZERO,
                                2.0,
                                Vector::ONE,
                            ))
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
                    reports.push(json!({"segment":segment.id,"heading":heading,"posture":posture,"sign":sign,
                        "passed":passed,"samples":samples,"scope":"All canonical angular motor coordinates and allocated axis/vector ceilings; independent equal/opposite body momentum on free isotropic pairs over four unchanged substeps"}));
                }
            }
        }
    }
    if reports.len() != CASES {
        return Err("Incomplete anatomical actuator coverage".into());
    }
    Ok(reports)
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
        for report in &reports {
            assert_eq!(report["passed"], true, "motor delivery failed: {report}");
        }
    }
}
