use rapier3d::prelude::*;
use serde_json::json;
use std::collections::BTreeSet;
use std::io::Write;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 4, "Usage: hybrid_constraints input.bin output.bin receipt.json");
    let input = std::fs::read(&args[1]).unwrap();
    let mut world: PhysicsWorld = bincode::deserialize(&input).unwrap();
    assert_eq!(world.multibody_joints.iter().count(), 0);
    let before_bodies = bincode::serialize(&world.bodies).unwrap();
    let before_colliders = bincode::serialize(&world.colliders).unwrap();
    let joints: Vec<_> = world.impulse_joints.iter()
        .map(|(handle, joint)| (handle, joint.body1(), joint.body2(), joint.data)).collect();
    assert_eq!(joints.len(), 24, "Only the 25-body anatomical tree is supported");
    let bodies: BTreeSet<_> = joints.iter().flat_map(|(_, a, b, _)|
        [a.into_raw_parts(), b.into_raw_parts()]).collect();
    assert_eq!(bodies.len(), 25);
    for (index, generation) in &bodies {
        let body = &world.bodies[RigidBodyHandle::from_raw_parts(*index, *generation)];
        assert!(body.is_dynamic() && body.linvel() == Vector::ZERO && body.angvel() == Vector::ZERO,
            "This initialization conversion requires a stationary dynamic tree");
    }
    let angular = JointAxesMask::ANG_X | JointAxesMask::ANG_Y | JointAxesMask::ANG_Z;
    let linear = JointAxesMask::LIN_X | JointAxesMask::LIN_Y | JointAxesMask::LIN_Z;
    let mut rows = vec![];
    let mut last_handle = None;
    for (handle, parent, child, data) in joints {
        assert_eq!(data.locked_axes & linear, linear);
        let spherical = SphericalJointBuilder::new()
            .local_anchor1(data.local_frame1.translation)
            .local_anchor2(data.local_frame2.translation)
            .contacts_enabled(data.contacts_enabled);
        let link_handle = world.multibody_joints.insert(parent, child, spherical, true).unwrap();
        let relative = (world.bodies[parent].rotation().inverse() * world.bodies[child].rotation()).normalize();
        let rotation_vector = relative.to_scaled_axis();
        let (multibody, link_id) = world.multibody_joints.get_mut(link_handle).unwrap();
        multibody.link_mut(link_id).unwrap().joint.apply_displacement(&rotation_vector.to_array());
        let joint = world.impulse_joints.get_mut(handle, true).unwrap();
        joint.data.locked_axes &= angular;
        let mut expected = data;
        expected.locked_axes &= angular;
        assert_eq!(bincode::serialize(&joint.data).unwrap(), bincode::serialize(&expected).unwrap(),
            "Only the three linear locked axes may move to the multibody representation");
        rows.push(json!({"impulseHandle": handle.into_raw_parts(), "multibodyHandle": link_handle.into_raw_parts(),
            "parent": parent.into_raw_parts(), "child": child.into_raw_parts(),
            "lockedAxesBefore": data.locked_axes.bits(), "lockedAxesAfter": joint.data.locked_axes.bits(),
            "limitAxes": data.limit_axes.bits(), "motorAxes": data.motor_axes.bits()}));
        last_handle = Some(link_handle);
    }
    let (multibody, _) = world.multibody_joints.get_mut(last_handle.unwrap()).unwrap();
    multibody.set_self_contacts_enabled(true);
    multibody.forward_kinematics(&world.bodies, true);
    assert_eq!(multibody.num_links(), 25);
    let mut max_position_error = 0.0_f32;
    let mut max_rotation_error = 0.0_f32;
    for link in multibody.links() {
        let body = &world.bodies[link.rigid_body_handle()];
        max_position_error = max_position_error.max((link.local_to_world().translation - body.translation()).length());
        let difference = link.local_to_world().rotation.inverse() * body.rotation();
        max_rotation_error = max_rotation_error.max(difference.normalize().to_scaled_axis().length());
    }
    assert!(max_position_error < 1.0e-5 && max_rotation_error < 1.0e-5,
        "Initial kinematics differ: position {max_position_error}, rotation {max_rotation_error}");
    assert_eq!(before_bodies, bincode::serialize(&world.bodies).unwrap());
    assert_eq!(before_colliders, bincode::serialize(&world.colliders).unwrap());
    let receipt = json!({"command": args, "bodiesUnchanged": true, "collidersUnchanged": true,
        "maxInitialKinematicPositionError": max_position_error,
        "maxInitialKinematicRotationError": max_rotation_error, "joints": rows,
        "qualification": "Initialization-only diagnostic. All angular locks, limits, frames and motor parameters remain on their original impulse joints. Only translational closure moves to spherical multibody links. No body pose or velocity is changed by conversion."});
    let mut binary = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[2]).unwrap();
    binary.write_all(&bincode::serialize(&world).unwrap()).unwrap();
    let mut output = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[3]).unwrap();
    writeln!(output, "{}", serde_json::to_string_pretty(&receipt).unwrap()).unwrap();
}
