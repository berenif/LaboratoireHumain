use rapier3d::prelude::*;
use serde_json::json;
use std::io::Write;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 4, "Usage: hinge_constraints input.bin output.bin receipt.json");
    let input = std::fs::read(&args[1]).unwrap();
    let mut world: PhysicsWorld = bincode::deserialize(&input).unwrap();
    assert_eq!(world.multibody_joints.iter().count(), 0);
    let before_bodies = bincode::serialize(&world.bodies).unwrap();
    let before_colliders = bincode::serialize(&world.colliders).unwrap();
    let joints: Vec<_> = world.impulse_joints.iter()
        .map(|(handle, joint)| (handle, joint.body1(), joint.body2(), joint.data)).collect();
    assert_eq!(joints.len(), 24);
    let linear = JointAxesMask::LIN_X | JointAxesMask::LIN_Y | JointAxesMask::LIN_Z;
    let angular = JointAxesMask::ANG_X | JointAxesMask::ANG_Y | JointAxesMask::ANG_Z;
    let mut links = vec![];
    let mut rows = vec![];
    for (handle, parent, child, data) in joints {
        assert_eq!(data.locked_axes & linear, linear);
        let free_angular = angular & !data.locked_axes;
        if free_angular.bits().count_ones() != 1 { continue; }
        for body in [parent, child] {
            let body = &world.bodies[body];
            assert!(body.is_dynamic() && body.linvel() == Vector::ZERO && body.angvel() == Vector::ZERO);
        }
        let mut structural = data;
        structural.limit_axes = JointAxesMask::empty();
        structural.motor_axes = JointAxesMask::empty();
        let link_handle = world.multibody_joints.insert(parent, child, structural, true).unwrap();
        let relative = (data.local_frame1.rotation.inverse() * world.bodies[parent].rotation().inverse()
            * world.bodies[child].rotation() * data.local_frame2.rotation).normalize();
        let axis = free_angular.bits().trailing_zeros() as usize - 3;
        let component = [relative.x, relative.y, relative.z][axis];
        let angle = 2.0 * component.atan2(relative.w);
        let angle = angle.sin().atan2(angle.cos());
        let (multibody, link_id) = world.multibody_joints.get_mut(link_handle).unwrap();
        multibody.link_mut(link_id).unwrap().joint.apply_displacement(&[angle]);
        let joint = world.impulse_joints.get_mut(handle, true).unwrap();
        joint.data.locked_axes = JointAxesMask::empty();
        let mut expected = data;
        expected.locked_axes = JointAxesMask::empty();
        assert_eq!(bincode::serialize(&joint.data).unwrap(), bincode::serialize(&expected).unwrap());
        rows.push(json!({"impulseHandle":handle.into_raw_parts(), "multibodyHandle":link_handle.into_raw_parts(),
            "parent":parent.into_raw_parts(), "child":child.into_raw_parts(), "angularAxis":axis,
            "initialAngle":angle, "lockedAxesBefore":data.locked_axes.bits(), "lockedAxesAfter":0,
            "limitAxes":data.limit_axes.bits(), "motorAxes":data.motor_axes.bits()}));
        links.push(link_handle);
    }
    assert!(!links.is_empty());
    let mut max_position_error = 0.0_f32;
    let mut max_rotation_error = 0.0_f32;
    for handle in links {
        let (multibody, _) = world.multibody_joints.get_mut(handle).unwrap();
        multibody.set_self_contacts_enabled(true);
        multibody.forward_kinematics(&world.bodies, true);
        for link in multibody.links() {
            let body = &world.bodies[link.rigid_body_handle()];
            max_position_error = max_position_error.max((link.local_to_world().translation - body.translation()).length());
            let difference = link.local_to_world().rotation.inverse() * body.rotation();
            max_rotation_error = max_rotation_error.max(difference.normalize().to_scaled_axis().length());
        }
    }
    assert!(max_position_error < 1e-5 && max_rotation_error < 1e-5,
        "Initial kinematics differ: position {max_position_error}, rotation {max_rotation_error}");
    assert_eq!(before_bodies, bincode::serialize(&world.bodies).unwrap());
    assert_eq!(before_colliders, bincode::serialize(&world.colliders).unwrap());
    let assemblies: Vec<_> = world.multibody_joints.multibodies().map(|mb| json!({
        "links":mb.num_links(), "degreesOfFreedom":mb.ndofs(), "generalizedDamping":mb.damping().as_slice()
    })).collect();
    let receipt = json!({"command":args, "bodiesUnchanged":true, "collidersUnchanged":true,
        "maxInitialKinematicPositionError":max_position_error, "maxInitialKinematicRotationError":max_rotation_error,
        "joints":rows, "assemblies":assemblies,
        "qualification":"Diagnostic only. Single-axis structural closure moves to reduced-coordinate links; original impulse limits and finite motors remain. Multi-axis joints are unchanged. Native multibody damping/integration differ from rigid-body damping and have not passed the frozen production gate."});
    let mut binary = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[2]).unwrap();
    binary.write_all(&bincode::serialize(&world).unwrap()).unwrap();
    let mut output = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[3]).unwrap();
    writeln!(output, "{}", serde_json::to_string_pretty(&receipt).unwrap()).unwrap();
}
