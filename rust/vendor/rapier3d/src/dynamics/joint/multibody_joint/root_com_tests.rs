//! Independent pose derivatives for an offset-COM free root.
use super::Multibody;
use crate::alloc_prelude::*;
use crate::prelude::*;
use std::println;
use super::na;

fn fixture(heading: Real) -> (Multibody, RigidBodySet, RigidBodyHandle) {
    let mut bodies = RigidBodySet::new();
    let root = bodies.insert(RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.1, 1.4, -0.2))
        .rotation(Vector::Y * heading)
        .can_sleep(false)
        .additional_mass_properties(MassProperties::new(
            Vector::new(0.0, -0.0463685152, -0.00386404293),
            8.3, Vector::new(0.12, 0.17, 0.23))));
    bodies[root].recompute_mass_properties_from_colliders(&ColliderSet::new());
    let mut mb = Multibody::with_root(root, false);
    mb.forward_kinematics(&bodies, true);
    // The isolated conservation fixture has no joint damping, armature, or
    // springs. Rapier's otherwise inherited free-joint angular damping is 0.1.
    mb.damping.fill(0.0);
    mb.armature.fill(0.0);
    (mb, bodies, root)
}

#[test]
fn offset_root_com_jacobian_matches_independent_pose_derivatives() {
    let mut maximum_error = 0.0_f32;
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mb, bodies, root) = fixture(heading);
        let local_com = bodies[root].local_center_of_mass();
        for column in 0..6 {
            let epsilon = 0.001;
            let mut displacement = vec![0.0; 6];
            displacement[column] = epsilon;
            let plus = mb.forward_kinematics_single_link(&bodies, 0, Some(&displacement), None).transform_point(local_com);
            displacement[column] = -epsilon;
            let minus = mb.forward_kinematics_single_link(&bodies, 0, Some(&displacement), None).transform_point(local_com);
            let derivative = (plus - minus) / (2.0 * epsilon);
            let j = mb.body_jacobian(0);
            let actual = Vector::new(j[(0,column)], j[(1,column)], j[(2,column)]);
            let error = (derivative - actual).length();
            maximum_error = maximum_error.max(error);
            println!("offset-COM root: heading={heading}, column={column}, pose_derivative={derivative:?}, Jacobian={actual:?}, error={error}");
        }
    }
    assert!(maximum_error <= 0.0001, "offset-COM free-root Jacobian error {maximum_error} m/rad");
}

#[test]
fn offset_root_cached_velocity_is_at_body_com_not_free_joint_origin() {
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mut mb, mut bodies, root) = fixture(heading);
        let origin_velocity = Vector::new(0.3, -0.2, 0.1);
        let angular = Vector::new(0.7, -0.4, 0.2);
        for k in 0..3 { mb.velocities[k] = origin_velocity[k]; mb.velocities[3+k] = angular[k]; }
        mb.update_velocities(&mut bodies);
        let offset = bodies[root].rotation().normalize() * bodies[root].local_center_of_mass();
        let expected = origin_velocity + angular.cross(offset);
        println!("offset-COM root velocity: heading={heading}, expected={expected:?}, actual={:?}", bodies[root].linvel());
        assert!((bodies[root].linvel() - expected).length() <= 0.000001);
    }
}

#[test]
fn offset_root_displaced_branch_jacobian_matches_full_forward_kinematics() {
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mb, bodies, _) = fixture(heading);
        let displacement = [0.03, -0.02, 0.04, 0.2, -0.15, 0.1];
        let mut proposed = mb.clone();
        proposed.apply_displacements(&displacement);
        proposed.forward_kinematics(&bodies, false);
        let mut branch_j = mb.body_jacobian(0).clone();
        let branch_pose = mb.forward_kinematics_single_link(&bodies, 0, Some(&displacement), Some(&mut branch_j));
        assert_eq!(branch_pose, *proposed.link(0).unwrap().local_to_world());
        assert_eq!(&branch_j, proposed.body_jacobian(0));
    }
}

#[test]
fn offset_root_mass_energy_and_point_wrench_match_com_mechanics() {
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mut mb, mut bodies, root) = fixture(heading);
        let origin_velocity = Vector::new(0.3, -0.2, 0.1);
        let angular = Vector::new(0.7, -0.4, 0.2);
        for k in 0..3 { mb.velocities[k] = origin_velocity[k]; mb.velocities[3+k] = angular[k]; }
        mb.update_velocities(&mut bodies);
        mb.update_mass_matrix(0.0, &bodies);
        let rotation = *bodies[root].rotation();
        let r = rotation * bodies[root].local_center_of_mass();
        let com_velocity = origin_velocity + angular.cross(r);
        let local_angular = rotation.inverse() * angular;
        let inertia_angular = rotation * (Vector::new(0.12,0.17,0.23) * local_angular);
        let expected_energy_twice = bodies[root].mass() * com_velocity.length_squared() + angular.dot(inertia_angular);
        let matrix_energy_twice = mb.velocities.dot(&(&mb.augmented_mass * &mb.velocities));
        assert!((matrix_energy_twice-expected_energy_twice).abs() <= 0.000002);
        let point_from_origin = rotation * Vector::new(0.2,0.1,-0.05);
        let force = Vector::new(2.0,-3.0,4.0);
        let com_moment = (point_from_origin-r).cross(force);
        let origin_moment = point_from_origin.cross(force);
        let j = mb.body_jacobian(0);
        for k in 0..3 {
            let projected = (0..3).map(|i|j[(i,3+k)]*force[i]+j[(3+i,3+k)]*com_moment[i]).sum::<Real>();
            assert!((projected-origin_moment[k]).abs() <= 0.000001);
        }
    }
}

#[test]
fn offset_root_zero_force_bias_preserves_com_and_angular_momentum_derivatives() {
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mut mb, mut bodies, root) = fixture(heading);
        let angular = Vector::new(0.7,-0.4,0.2);
        for k in 0..3 { mb.velocities[3+k] = angular[k]; }
        mb.update_velocities(&mut bodies);
        mb.update_mass_matrix(0.0,&bodies);
        mb.update_acceleration(0.0,&bodies);
        let origin_acceleration = Vector::new(mb.accelerations[0],mb.accelerations[1],mb.accelerations[2]);
        let alpha = Vector::new(mb.accelerations[3],mb.accelerations[4],mb.accelerations[5]);
        let rotation = *bodies[root].rotation();
        let r = rotation * bodies[root].local_center_of_mass();
        let com_acceleration = origin_acceleration + alpha.cross(r) + angular.cross(angular.cross(r));
        assert!(com_acceleration.length() <= 0.000001, "{com_acceleration:?}");
        let inertia = |w:Vector| rotation*(Vector::new(0.12,0.17,0.23)*(rotation.inverse()*w));
        let angular_balance = inertia(alpha)+angular.cross(inertia(angular));
        println!("offset-COM root bias: heading={heading}, origin_acceleration={origin_acceleration:?}, alpha={alpha:?}, com_acceleration={com_acceleration:?}, angular_balance={angular_balance:?}");
        assert!(angular_balance.length() <= 0.000001, "{angular_balance:?}");
    }
}

#[test]
fn offset_root_finite_dt_augmented_dynamics_match_independent_bias_derivative() {
    for heading in [0.0, 1.0471975512, -0.7853981634] {
        let (mut mb, mut bodies, root) = fixture(heading);
        let angular = Vector::new(0.7,-0.4,0.2);
        for k in 0..3 { mb.velocities[k] = [0.3,-0.2,0.1][k]; mb.velocities[3+k] = angular[k]; }
        mb.update_velocities(&mut bodies);
        let q = *bodies[root].rotation();
        let rotation = na::UnitQuaternion::<f64>::new_normalize(na::Quaternion::new(q.w as f64,q.x as f64,q.y as f64,q.z as f64)).to_rotation_matrix();
        let local = bodies[root].local_center_of_mass();
        let r = rotation * na::Vector3::new(local.x as f64,local.y as f64,local.z as f64);
        let inertia = rotation.matrix()*na::Matrix3::from_diagonal(&na::Vector3::new(0.12_f32 as f64,0.17_f32 as f64,0.23_f32 as f64))*rotation.matrix().transpose();
        let mass = bodies[root].mass() as f64;
        let bias = |omega:na::Vector3<f64>| {
            let force = mass*omega.cross(&omega.cross(&r));
            let moment = r.cross(&force)+omega.cross(&(inertia*omega));
            na::SVector::<f64,6>::from_iterator(force.iter().chain(moment.iter()).copied())
        };
        let omega = na::Vector3::new(angular.x as f64,angular.y as f64,angular.z as f64);
        let mut derivative = na::SMatrix::<f64,6,6>::zeros();
        for k in 0..3 {
            let mut plus = omega; let mut minus = omega;
            plus[k] += 0.00001; minus[k] -= 0.00001;
            derivative.set_column(3+k,&((bias(plus)-bias(minus))/0.00002));
        }
        let dt = 1.0_f32/800.0;
        mb.update_mass_matrix(dt,&bodies);
        let mut expected = na::SMatrix::<f64,6,6>::zeros();
        let mut maximum_error = 0.0_f64;
        for row in 0..6 { for col in 0..6 {
            expected[(row,col)] = mb.augmented_mass[(row,col)] as f64 + dt as f64*derivative[(row,col)];
            maximum_error = maximum_error.max((mb.acc_augmented_mass[(row,col)] as f64-expected[(row,col)]).abs());
        }}
        println!("offset-COM finite-dt mass derivative: heading={heading}, maximum_error={maximum_error}");
        assert!(maximum_error<=0.000002);
        mb.update_acceleration(dt,&bodies);
        let acceleration = na::SVector::<f64,6>::from_iterator(mb.accelerations.iter().map(|x|*x as f64));
        let residual = expected*acceleration+bias(omega);
        assert!(residual.amax()<=0.000001,"implicit acceleration residual {residual:?}");
    }
}
