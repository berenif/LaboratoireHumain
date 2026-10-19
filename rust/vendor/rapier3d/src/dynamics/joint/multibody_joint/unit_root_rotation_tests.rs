//! Unit-root projection correctness; these fixtures do not change solver policy.

use super::Multibody;
use crate::alloc_prelude::*;
use crate::prelude::*;

fn norm_squared(q: Rotation) -> f64 {
    q.to_array()
        .into_iter()
        .map(|v| {
            let v = f64::from(v);
            v * v
        })
        .sum()
}

fn scaled(q: Rotation, scale: f32) -> Rotation {
    let v = q.to_array().map(|c| c * scale);
    Rotation::from_xyzw(v[0], v[1], v[2], v[3])
}

fn bits(q: Rotation) -> [u32; 4] {
    q.to_array().map(f32::to_bits)
}

fn fixture(heading: f32) -> (Multibody, RigidBodySet, RigidBodyHandle) {
    let mut bodies = RigidBodySet::new();
    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.1, 1.4, -0.2))
            .rotation(Vector::Y * heading)
            .additional_mass_properties(MassProperties::new(
                Vector::new(0.0, -0.0463685152, -0.00386404293),
                8.3,
                Vector::new(0.12, 0.17, 0.23),
            )),
    );
    bodies[body].recompute_mass_properties_from_colliders(&ColliderSet::new());
    let mut mb = Multibody::with_root(body, false);
    mb.forward_kinematics(&bodies, true);
    bodies.index_mut_internal(body).changes = RigidBodyChanges::empty();
    (mb, bodies, body)
}

#[test]
fn pure_projection_preserves_orientation_and_sign_for_finite_scales() {
    for axis in [
        Vector::X,
        Vector::Y,
        Vector::new(0.2, -0.3, 0.7).normalize(),
    ] {
        for angle in [-2.3, -0.2, 0.8, 2.9] {
            let original = Rotation::from_axis_angle(axis, angle);
            for scale in [0.001, 0.7, 1000.0, -0.001, -1.3, -1000.0] {
                let input = scaled(original, scale);
                let unit = MultibodyJoint::project_unit_root_rotation(input).unwrap();
                assert!((norm_squared(unit) - 1.0).abs() <= 4.0 * f64::from(f32::EPSILON));
                for (before, after) in input.to_array().into_iter().zip(unit.to_array()) {
                    assert_eq!(before.is_sign_negative(), after.is_sign_negative());
                }
                for vector in [Vector::X, Vector::Y, Vector::new(0.3, -0.2, 0.8)] {
                    assert!((unit * vector - original * vector).length() < 8.0e-7);
                }
            }
        }
    }
}

#[test]
fn pure_projection_handles_subnormal_and_maximum_finite_components() {
    for input in [
        Rotation::from_xyzw(f32::from_bits(1), 0.0, -0.0, 0.0),
        Rotation::from_xyzw(0.0, -f32::from_bits(1), f32::from_bits(2), 0.0),
        Rotation::from_xyzw(f32::MAX, -f32::MAX, f32::MAX, -f32::MAX),
        Rotation::from_xyzw(f32::MAX, 1.0, f32::from_bits(1), -f32::MAX),
    ] {
        let unit = MultibodyJoint::project_unit_root_rotation(input).unwrap();
        assert!(unit.to_array().into_iter().all(f32::is_finite));
        assert!((norm_squared(unit) - 1.0).abs() <= 4.0 * f64::from(f32::EPSILON));
        for (before, after) in input.to_array().into_iter().zip(unit.to_array()) {
            assert_eq!(before.is_sign_negative(), after.is_sign_negative());
        }
    }
}

#[test]
fn pure_invalid_input_is_refused_without_replacing_candidate() {
    for input in [
        Rotation::from_xyzw(0.0, -0.0, 0.0, -0.0),
        Rotation::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
        Rotation::from_xyzw(0.0, f32::INFINITY, 0.0, 1.0),
        Rotation::from_xyzw(0.0, 0.0, f32::NEG_INFINITY, 1.0),
    ] {
        let before = bits(input);
        assert!(MultibodyJoint::project_unit_root_rotation(input).is_err());
        assert_eq!(before, bits(input));
    }
}

#[test]
fn enabling_is_policy_only_and_invalid_activation_is_atomic() {
    let (mut mb, _, _) = fixture(0.7);
    assert!(!mb.unit_root_rotation_enabled());
    mb.links[0].joint.joint_rot = scaled(mb.links[0].joint.joint_rot, 1.0002);
    let before_q = bits(mb.root().joint.joint_rot());
    let before_coords = mb.root().joint.coords();
    let before_pose = *mb.root().local_to_world();
    let before_vel = mb.velocities.clone();
    mb.set_unit_root_rotation_enabled(true).unwrap();
    assert!(mb.unit_root_rotation_enabled());
    mb.validate_unit_root_rotation().unwrap();
    assert_eq!(bits(mb.root().joint.joint_rot()), before_q);
    assert_eq!(mb.root().joint.coords(), before_coords);
    assert_eq!(*mb.root().local_to_world(), before_pose);
    assert_eq!(mb.velocities, before_vel);
    mb.set_unit_root_rotation_enabled(false).unwrap();
    assert_eq!(bits(mb.root().joint.joint_rot()), before_q);
    for invalid in [
        Rotation::from_xyzw(0.0, 0.0, 0.0, 0.0),
        Rotation::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
        Rotation::from_xyzw(f32::INFINITY, 0.0, 0.0, 1.0),
        scaled(Rotation::IDENTITY, 1.001),
        scaled(Rotation::IDENTITY, f32::from_bits(1)),
        scaled(Rotation::IDENTITY, f32::MAX),
    ] {
        mb.links[0].joint.joint_rot = invalid;
        let before = bits(invalid);
        assert!(mb.set_unit_root_rotation_enabled(true).is_err());
        assert!(!mb.unit_root_rotation_enabled());
        assert_eq!(bits(mb.root().joint.joint_rot()), before);
        assert_eq!(mb.velocities, before_vel);
    }
}

#[test]
fn long_coordinate_sequence_keeps_unit_norm_and_all_other_coordinates_legacy() {
    let (mut active, _, _) = fixture(0.7);
    let mut disabled = active.clone();
    let mut literal = active.root().joint;
    let velocities = [0.002, -0.003, 0.001, 1.3, -2.1, 0.7];
    active
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    disabled
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    active.set_unit_root_rotation_enabled(true).unwrap();
    let dt = 0.00005;
    for _ in 0..100_000 {
        active.integrate(dt);
        disabled.integrate(dt);
        literal.integrate(dt, &velocities);
        assert!(
            (norm_squared(active.root().joint.joint_rot()) - 1.0).abs()
                <= 4.0 * f64::from(f32::EPSILON)
        );
    }
    assert_eq!(active.root().joint.coords(), literal.coords());
    assert_eq!(disabled.root().joint.coords(), literal.coords());
    assert_eq!(
        bits(disabled.root().joint.joint_rot()),
        bits(literal.joint_rot())
    );
    assert_eq!(active.velocities, disabled.velocities);
    assert_eq!(active.velocities.as_slice(), &velocities);
}

#[test]
fn invalid_active_roots_remain_visible_and_never_fall_back_to_identity() {
    for invalid in [
        Rotation::from_xyzw(0.0, 0.0, 0.0, 0.0),
        Rotation::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
        Rotation::from_xyzw(0.0, f32::INFINITY, 0.0, 1.0),
        scaled(Rotation::IDENTITY, 2.0),
    ] {
        let (mut mb, _, _) = fixture(0.2);
        mb.set_unit_root_rotation_enabled(true).unwrap();
        mb.links[0].joint.joint_rot = invalid;
        let before = bits(invalid);
        assert!(mb.set_unit_root_rotation_enabled(true).is_err());
        assert!(mb.unit_root_rotation_enabled());
        assert_eq!(bits(mb.root().joint.joint_rot()), before);
        assert!(mb.validate_unit_root_rotation().is_err());
        mb.apply_displacements(&[0.0; 6]);
        assert!(mb.validate_unit_root_rotation().is_err());
        assert_ne!(bits(mb.root().joint.joint_rot()), bits(Rotation::IDENTITY));
    }
}

#[test]
fn nonroot_free_joint_rotation_remains_bit_exact_legacy() {
    let (mut mb, mut bodies, _) = fixture(0.4);
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    let mut child_joint = MultibodyJoint::free(Pose::IDENTITY);
    child_joint.joint_rot = scaled(
        Rotation::from_scaled_axis(Vector::new(0.2, 0.3, -0.4)),
        1.0002,
    );
    mb.add_link(Some(0), child_joint, child);
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let mut literal = child_joint;
    let mut velocities = [0.0; 12];
    velocities[3..6].copy_from_slice(&[1.3, -2.1, 0.7]);
    velocities[9..12].copy_from_slice(&[-0.4, 0.6, 0.8]);
    mb.generalized_velocity_mut().copy_from_slice(&velocities);
    for _ in 0..500 {
        mb.integrate(0.00005);
        literal.integrate(0.00005, &velocities[6..]);
    }
    assert_eq!(
        bits(mb.links[1].joint.joint_rot()),
        bits(literal.joint_rot())
    );
    assert_eq!(mb.links[1].joint.coords(), literal.coords());
    assert!(
        (norm_squared(mb.root().joint.joint_rot()) - 1.0).abs() <= 4.0 * f64::from(f32::EPSILON)
    );
}

#[test]
fn eligibility_root_mutation_and_topology_disable_policy() {
    for invalid in 0..4 {
        let (mut mb, _, _) = fixture(0.2);
        match invalid {
            0 => mb.root_is_dynamic = false,
            1 => mb.links[0].joint.kinematic = true,
            2 => mb.links[0].joint.data.local_frame2.translation.x = 0.1,
            _ => mb.links[0].joint.data.locked_axes = JointAxesMask::LIN_X,
        }
        assert!(mb.set_unit_root_rotation_enabled(true).is_err());
        assert!(!mb.unit_root_rotation_enabled());
    }
    let (mut mb, mut bodies, root) = fixture(0.2);
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let _ = mb.root_mut();
    assert!(!mb.unit_root_rotation_enabled());
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    mb.add_link(Some(0), MultibodyJoint::fixed(Pose::IDENTITY), child);
    assert!(!mb.unit_root_rotation_enabled());
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let _ = mb.link_mut(1);
    assert!(mb.unit_root_rotation_enabled());
    let _ = mb.links_mut();
    assert!(!mb.unit_root_rotation_enabled());
    mb.set_unit_root_rotation_enabled(true).unwrap();
    bodies[root].set_body_type(RigidBodyType::Fixed, false);
    mb.forward_kinematics(&bodies, true);
    assert!(!mb.unit_root_rotation_enabled());
}

#[test]
fn clone_preserves_policy_and_raw_serialization_omits_it() {
    let (mut mb, _, _) = fixture(0.4);
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let mut cloned = mb.clone();
    assert!(cloned.unit_root_rotation_enabled());
    cloned.set_unit_root_rotation_enabled(false).unwrap();
    assert!(mb.unit_root_rotation_enabled());
    #[cfg(feature = "serde-serialize")]
    {
        let bytes = bincode::serialize(&mb).unwrap();
        assert_eq!(bytes, bincode::serialize(&cloned).unwrap());
        let restored: Multibody = bincode::deserialize(&bytes).unwrap();
        assert!(!restored.unit_root_rotation_enabled());
        assert_eq!(
            bits(mb.root().joint.joint_rot()),
            bits(restored.root().joint.joint_rot())
        );
    }
}

#[test]
fn routine_rigid_body_fk_import_retains_authoritative_quaternion_bits() {
    let (mut mb, mut bodies, body) = fixture(0.0);
    mb.links[0].joint.joint_rot = Rotation::from_xyzw(-0.0, 0.0, -0.0, 1.0002);
    mb.set_unit_root_rotation_enabled(true).unwrap();
    let before = bits(mb.root().joint.joint_rot());
    mb.forward_kinematics(&bodies, false);
    mb.update_rigid_bodies(&mut bodies, true);
    bodies.index_mut_internal(body).changes = RigidBodyChanges::empty();
    for _ in 0..10 {
        mb.forward_kinematics(&bodies, true);
        assert_eq!(before, bits(mb.root().joint.joint_rot()));
    }
}

#[test]
fn fk_is_read_only_and_displaced_branch_matches_full_root_and_child() {
    for heading in [0.0, 1.0471976, -0.7853982] {
        let (mut mb, mut bodies, _) = fixture(heading);
        let child = bodies.insert(RigidBodyBuilder::dynamic().additional_mass(1.0));
        mb.add_link(
            Some(0),
            MultibodyJoint::fixed(Pose::from_translation(Vector::new(0.2, 0.5, -0.1))),
            child,
        );
        mb.forward_kinematics(&bodies, false);
        mb.set_unit_root_rotation_enabled(true).unwrap();
        let displacement = [1.0e-8, -2.0e-8, 3.0e-8, 0.001, -0.002, 0.003];
        let before = bits(mb.root().joint.joint_rot());
        let mut branch_jacobian = Jacobian::zeros(mb.ndofs());
        let root_pose = mb.forward_kinematics_single_link(&bodies, 0, Some(&displacement), None);
        let child_pose = mb.forward_kinematics_single_link(
            &bodies,
            1,
            Some(&displacement),
            Some(&mut branch_jacobian),
        );
        assert_eq!(before, bits(mb.root().joint.joint_rot()));
        let mut full = mb.clone();
        full.apply_displacements(&displacement);
        full.forward_kinematics(&bodies, false);
        assert_eq!(root_pose, *full.root().local_to_world());
        assert_eq!(child_pose, *full.link(1).unwrap().local_to_world());
        assert_eq!(&branch_jacobian, full.body_jacobian(1));
        // Ordinary FK consumes existing quaternion bits without projecting again.
        let after = bits(full.root().joint.joint_rot());
        for _ in 0..10 {
            full.forward_kinematics(&bodies, false);
        }
        assert_eq!(after, bits(full.root().joint.joint_rot()));
    }
}

#[test]
fn projected_root_com_derivatives_match_existing_jacobian() {
    for heading in [0.0, 1.0471976, -0.7853982] {
        let (mut mb, bodies, root) = fixture(heading);
        mb.set_unit_root_rotation_enabled(true).unwrap();
        for column in 0..6 {
            let epsilon = 0.001;
            let mut displacement = [0.0; 6];
            displacement[column] = epsilon;
            let plus = mb
                .forward_kinematics_single_link(&bodies, 0, Some(&displacement), None)
                .transform_point(bodies[root].local_center_of_mass());
            displacement[column] = -epsilon;
            let minus = mb
                .forward_kinematics_single_link(&bodies, 0, Some(&displacement), None)
                .transform_point(bodies[root].local_center_of_mass());
            let derivative = (plus - minus) / (2.0 * epsilon);
            let jacobian = mb.body_jacobian(0);
            let expected = Vector::new(
                jacobian[(0, column)],
                jacobian[(1, column)],
                jacobian[(2, column)],
            );
            assert!((derivative - expected).length() < 0.0002);
        }
    }
}

#[test]
fn drifted_quaternion_fixture_exposes_and_removes_norm_squared_geometry_scale() {
    // Match the diagnosed drift magnitude without claiming these synthetic
    // orientation bits are the archived root quaternion itself.
    let original = Rotation::from_scaled_axis(Vector::new(0.2, -0.7, 0.4));
    let drifted = scaled(original, (1.0005303101620744_f64).sqrt() as f32);
    let projected = MultibodyJoint::project_unit_root_rotation(drifted).unwrap();
    let offset = Vector::new(0.3, 0.8, -0.5);
    let raw = drifted * offset;
    let corrected = projected * offset;
    let factor = norm_squared(drifted) as f32;
    assert!((raw - corrected * factor).length() < 5.0e-7);
    assert!((corrected.length() - offset.length()).abs() < 3.0e-7);
    assert!((raw - corrected).length() > 0.0004);
}

#[test]
#[cfg(feature = "compensated-root-integration")]
fn translation_carry_and_angular_coordinates_are_unchanged_by_projection_policy() {
    let (mut active, bodies, _) = fixture(0.7);
    active
        .set_root_translation_compensation_enabled(true)
        .unwrap();
    active.apply_displacements(&[2.0_f32.powi(-29), 0.0, 0.0, 0.0, 0.0, 0.0]);
    let before_carry = active.root_translation_integrator_state();
    active.set_unit_root_rotation_enabled(true).unwrap();
    assert_eq!(before_carry, active.root_translation_integrator_state());
    let mut disabled = active.clone();
    disabled.set_unit_root_rotation_enabled(false).unwrap();
    assert_eq!(before_carry, disabled.root_translation_integrator_state());
    let velocities = [0.002, -0.003, 0.001, 1.3, -2.1, 0.7];
    active
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    disabled
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    for _ in 0..500 {
        active.integrate(0.00005);
        disabled.integrate(0.00005);
        assert_eq!(
            active.root_translation_integrator_state(),
            disabled.root_translation_integrator_state()
        );
        assert_eq!(active.root().joint.coords(), disabled.root().joint.coords());
    }
    let before = active.root_translation_integrator_state();
    let displacement = [1.0e-8, -2.0e-8, 3.0e-8, 0.001, -0.002, 0.003];
    let branch = active.forward_kinematics_single_link(&bodies, 0, Some(&displacement), None);
    assert_eq!(before, active.root_translation_integrator_state());
    let mut copied = active.clone();
    copied.apply_displacements(&displacement);
    copied.forward_kinematics(&bodies, false);
    assert_eq!(branch, *copied.root().local_to_world());
}
