use rapier3d::prelude::*;
use serde_json::json;
use std::io::Write;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "Usage: snapshot_motors input.bin output.json");
    let input = std::fs::read(&args[1]).unwrap();
    let world: PhysicsWorld = bincode::deserialize(&input).unwrap();
    let serialized = bincode::serialize(&world).unwrap();
    let sections = [
        ("gravity", bincode::serialize(&world.gravity).unwrap()),
        ("integrationParameters", bincode::serialize(&world.integration_parameters).unwrap()),
        ("islands", bincode::serialize(&world.islands).unwrap()),
        ("broadPhase", bincode::serialize(&world.broad_phase).unwrap()),
        ("narrowPhase", bincode::serialize(&world.narrow_phase).unwrap()),
        ("bodies", bincode::serialize(&world.bodies).unwrap()),
        ("colliders", bincode::serialize(&world.colliders).unwrap()),
        ("impulseJoints", bincode::serialize(&world.impulse_joints).unwrap()),
        ("multibodyJoints", bincode::serialize(&world.multibody_joints).unwrap()),
    ];
    let concatenated: Vec<u8> = sections.iter().flat_map(|(_, bytes)| bytes.iter().copied()).collect();
    assert_eq!(concatenated, serialized, "Section order does not reconstruct native serialization");
    let mut offset = 0;
    let section_receipts: Vec<_> = sections.iter().map(|(name, bytes)| {
        let start = offset; offset += bytes.len();
        let original = input.get(start..offset);
        json!({"name": name, "offset": start, "bytes": bytes.len(),
            "roundtripExact": original == Some(bytes.as_slice()),
            "differentBytes": original.map(|slice| slice.iter().zip(bytes).filter(|(a, b)| a != b).count())})
    }).collect();
    // ImpulseJointSet ends with two serialized HashSets. Native and WASM
    // hashing can change their iteration order without changing membership.
    // Isolate that representation difference; require all preceding joint
    // bytes (including motor fields) and both unordered memberships to match.
    let joint_value = serde_json::to_value(&world.impulse_joints).unwrap();
    let wakes: Vec<RigidBodyHandle> = serde_json::from_value(joint_value["to_wake_up"].clone()).unwrap();
    let joins: Vec<(RigidBodyHandle, RigidBodyHandle)> = serde_json::from_value(joint_value["to_join"].clone()).unwrap();
    let wake_bytes = bincode::serialize(&wakes).unwrap();
    let join_bytes = bincode::serialize(&joins).unwrap();
    let joint_index = sections.iter().position(|(name, _)| *name == "impulseJoints").unwrap();
    let joint_start: usize = sections[..joint_index].iter().map(|(_, bytes)| bytes.len()).sum();
    let joint_end = joint_start + sections[joint_index].1.len();
    let join_start = joint_end - join_bytes.len();
    let wake_start = join_start - wake_bytes.len();
    assert_eq!(serialized[wake_start..join_start], wake_bytes);
    assert_eq!(serialized[join_start..joint_end], join_bytes);
    let original_wakes: Vec<RigidBodyHandle> = bincode::deserialize(&input[wake_start..join_start]).unwrap();
    let original_joins: Vec<(RigidBodyHandle, RigidBodyHandle)> = bincode::deserialize(&input[join_start..joint_end]).unwrap();
    let sorted_wakes = |items: &[RigidBodyHandle]| { let mut result: Vec<_> = items.iter().map(|h| h.into_raw_parts()).collect(); result.sort(); result };
    let sorted_joins = |items: &[(RigidBodyHandle, RigidBodyHandle)]| { let mut result: Vec<_> = items.iter().map(|(a, b)| (a.into_raw_parts(), b.into_raw_parts())).collect(); result.sort(); result };
    let joint_payload_exact = input[joint_start..wake_start] == serialized[joint_start..wake_start];
    let wake_members_exact = sorted_wakes(&original_wakes) == sorted_wakes(&wakes);
    let join_members_exact = sorted_joins(&original_joins) == sorted_joins(&joins);
    let other_sections_exact = section_receipts.iter().all(|r| r["name"] == "impulseJoints" || r["roundtripExact"] == true);
    let rows: Vec<_> = world.impulse_joints.iter().map(|(handle, joint)| {
        let motors: Vec<_> = joint.data.motors.iter().enumerate().map(|(axis, motor)| json!({
            "axis": axis, "targetPosition": motor.target_pos as f64,
            "targetVelocity": motor.target_vel as f64,
            "stiffness": motor.stiffness as f64, "damping": motor.damping as f64,
            "maxForce": motor.max_force as f64, "impulse": motor.impulse as f64,
            "model": motor.model
        })).collect();
        json!({"handle": handle.into_raw_parts(), "parent": joint.body1().into_raw_parts(),
            "child": joint.body2().into_raw_parts(), "motorAxes": joint.data.motor_axes.bits(),
            "lockedAxes": joint.data.locked_axes.bits(), "motors": motors})
    }).collect();
    let result = json!({"command": args, "snapshotRoundtripExact": serialized == input,
        "originalBytes": input.len(), "serializedBytes": serialized.len(), "sections": section_receipts,
        "otherSectionsExact": other_sections_exact,
        "impulseJointPayloadExact": joint_payload_exact, "impulseJointPayloadBytes": wake_start - joint_start,
        "wakeUpMembersExact": wake_members_exact, "joinMembersExact": join_members_exact,
        "originalWakeUpOrder": original_wakes.iter().map(|h| h.into_raw_parts()).collect::<Vec<_>>(),
        "nativeWakeUpOrder": wakes.iter().map(|h| h.into_raw_parts()).collect::<Vec<_>>(),
        "joints": rows, "qualification": "Read-only native snapshot motor inspection; no world stepping or output-world mutation."});
    let mut output = std::fs::OpenOptions::new().write(true).create_new(true).open(&args[2]).unwrap();
    writeln!(output, "{}", serde_json::to_string_pretty(&result).unwrap()).unwrap();
}
