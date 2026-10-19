//! Force-application contracts only. These fixtures never integrate physics,
//! run an actor, or qualify recovery. Native counters below are synthetic.
use super::*;

fn profile(enabled: bool) -> Profile {
    Profile { getup_policy_enabled: enabled, ..Profile::default() }
}

fn simulation(enabled: bool) -> Simulation {
    Simulation::new(0.0, profile(enabled), true).unwrap()
}

fn reset_torques(s: &mut Simulation) {
    for handle in &s.bodies {
        s.world.bodies[*handle].reset_torques(false);
    }
}

fn torque_words(s: &Simulation) -> Vec<[u32; 3]> {
    s.bodies.iter().map(|handle| {
        let torque = s.world.bodies[*handle].user_torque();
        [torque.x, torque.y, torque.z].map(f32::to_bits)
    }).collect()
}

fn motion_words(s: &Simulation) -> Vec<u32> {
    let mut words = Vec::new();
    for handle in &s.bodies {
        let rb = &s.world.bodies[*handle];
        let p = rb.translation();
        let q = rb.rotation();
        let v = rb.linvel();
        let w = rb.angvel();
        words.extend([p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            v.x, v.y, v.z, w.x, w.y, w.z].map(f32::to_bits));
    }
    let root = s.world.multibody_joints.rigid_body_link(s.bodies[0]).unwrap();
    let mb = s.world.multibody_joints.get_multibody(root.multibody).unwrap();
    for link in mb.links() {
        words.extend(link.joint.coords().iter().map(|v| v.to_bits()));
    }
    words.extend(mb.generalized_velocity().iter().map(|v| v.to_bits()));
    let (q, dq) = joint_state(s).unwrap();
    words.extend(q.map(f32::to_bits));
    words.extend(dq.map(f32::to_bits));
    words
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
fn torques_in_actuator_order(s: &Simulation, controls: &[f32; 21]) -> [f32; 21] {
    let mut torques = [0.0; 21];
    let mut seen = [false; 21];
    for part in &s.model.segments {
        if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            assert!(!seen[h.actuator_index]);
            seen[h.actuator_index] = true;
            torques[h.actuator_index] = h.gear * controls[h.actuator_index];
        }
    }
    assert!(seen.into_iter().all(|present| present));
    torques
}

/// Independent copy of the established force pair, without the production
/// receipt, retaining the normal equal/opposite application for every motor.
fn expected_force_application(s: &mut Simulation, controls: &[f32; 21]) {
    for (i, part) in s.model.segments.iter().enumerate() {
        if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let parent = s.model.segments.iter()
                .position(|p| Some(&p.id) == part.parent.as_ref()).unwrap();
            let jp = part.joint_profile.as_ref().unwrap();
            let axis = (*s.world.bodies[s.bodies[parent]].rotation()
                * rotation(jp.parent_frame.rotation) * Vector::X).normalize();
            let tau = h.gear * controls[h.actuator_index];
            s.world.bodies[s.bodies[i]].add_torque(axis * tau, true);
            s.world.bodies[s.bodies[parent]].add_torque(-axis * tau, true);
        }
    }
}

fn nonuniform_controls() -> [f32; 21] {
    std::array::from_fn(|i| (i as f32 - 10.0) / 16.0)
}

#[test]
fn motor_application_target_is_the_pinned_right_ankle_y_not_its_neighbor() {
    let s = simulation(false);
    let selected: Vec<_> = s.model.segments.iter().filter_map(|part| {
        part.reference.as_ref().and_then(|r| r.hinge.as_ref())
            .filter(|h| h.qpos_index == 7 || h.actuator_index == 8)
            .map(|h| (part.id.as_str(), h.name.as_str(), h.qpos_index, h.actuator_index, h.gear))
    }).collect();
    assert_eq!(selected, vec![("@right_ankle_y", "right_ankle_y", 7, 8, 20.0)]);
    assert_eq!(s.substeps_completed, 0);
}

#[test]
fn motor_application_force_application_preserves_all_coordinates_velocities_and_controller_state() {
    let controls = nonuniform_controls();
    for enabled in [false, true] {
        let mut s = simulation(enabled);
        let before_motion = motion_words(&s);
        let before_multibody = bincode::serialize(&s.world.multibody_joints).unwrap();
        let before_controller = serde_json::to_string(&s.getup_controller).unwrap();
        let before_counters = serde_json::to_string(&s.counters).unwrap();
        for native in [2179, 2180, 2199, 2200] {
            s.substeps_completed = native;
            reset_torques(&mut s);
            apply_controls(&mut s, &controls).unwrap();
            assert_eq!(motion_words(&s), before_motion);
            assert_eq!(bincode::serialize(&s.world.multibody_joints).unwrap(), before_multibody);
            assert_eq!(serde_json::to_string(&s.getup_controller).unwrap(), before_controller);
            assert_eq!(serde_json::to_string(&s.counters).unwrap(), before_counters);
            assert_eq!(s.substeps_completed, native);
            assert_eq!(s.tick, 0);
        }
    }
}

#[test]
fn motor_application_matches_the_independent_normal_body_torque_pair() {
    let controls = nonuniform_controls();
    for enabled in [false, true] {
        let mut actual = simulation(enabled);
        let mut expected = simulation(enabled);
        for native in [0, 2179, 2180, 2181, 2199, 2200, 2201, u64::MAX] {
            actual.substeps_completed = native;
            expected.substeps_completed = native;
            reset_torques(&mut actual);
            reset_torques(&mut expected);
            expected_force_application(&mut expected, &controls);
            apply_controls(&mut actual, &controls).unwrap();
            assert_eq!(torque_words(&actual), torque_words(&expected),
                "actual force-buffer words: enabled={enabled}, native={native}");
            let effort = actual.model.segments.iter().filter_map(|part| {
                part.reference.as_ref().and_then(|r| r.hinge.as_ref())
                    .map(|h| (h.gear * controls[h.actuator_index]).abs())
            }).sum::<f32>();
            assert_eq!(actual.diagnostics.requested_motor_effort_nm.to_bits(), effort.to_bits());
        }
    }
}

#[test]
fn motor_application_isolated_actuator_eight_keeps_the_source_gear_at_every_clock() {
    let mut s = simulation(true);
    let mut controls = [0.0; 21];
    controls[8] = -0.375;
    let child = s.model.segments.iter().position(|part| part.id == "@right_ankle_y").unwrap();
    let parent = s.model.segments.iter().position(|part|
        Some(&part.id) == s.model.segments[child].parent.as_ref()).unwrap();
    for native in [2179, 2180, 2199, 2200] {
        s.substeps_completed = native;
        reset_torques(&mut s);
        apply_controls(&mut s, &controls).unwrap();
        for (i, handle) in s.bodies.iter().enumerate() {
            let torque = s.world.bodies[*handle].user_torque();
            if i != child && i != parent {
                assert_eq!(torque.length(), 0.0, "native={native}, body={i}");
            } else {
                assert!((torque.length() - 7.5).abs() < 1e-6,
                    "native={native}, body={i}, torque={torque:?}");
            }
        }
        let child_torque = s.world.bodies[s.bodies[child]].user_torque();
        let parent_torque = s.world.bodies[s.bodies[parent]].user_torque();
        assert_eq!((child_torque + parent_torque).length(), 0.0);
        assert_eq!(s.diagnostics.requested_motor_effort_nm, 7.5);
        assert_eq!(s.substeps_completed, native);
    }
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
#[test]
fn motor_application_receipts_record_all_twenty_one_actual_applications_in_actuator_order() {
    let controls = nonuniform_controls();
    for enabled in [false, true] {
        let mut s = simulation(enabled);
        // Construction evaluates the native0 controls without integration.
        // A receipt may therefore already exist; each application below must
        // replace it with the exact current pre-command clock.
        if let Some(receipt) = &s.last_motor_application {
            assert_eq!(receipt.command_input_native_substep, 0);
        }
        let intended_torque = torques_in_actuator_order(&s, &controls);
        // Consecutive synthetic command boundaries prove the receipt is
        // replaced at every application, independently of the synthetic clock.
        for native in 2179..=2200 {
            s.substeps_completed = native;
            reset_torques(&mut s);
            apply_controls(&mut s, &controls).unwrap();
            let receipt = s.last_motor_application.as_ref().unwrap();
            assert_eq!(receipt.command_input_native_substep, native);
            assert_eq!(receipt.intended_controls.map(f32::to_bits), controls.map(f32::to_bits));
            assert_eq!(receipt.intended_torque_nm.map(f32::to_bits), intended_torque.map(f32::to_bits));
            assert!(!receipt.override_applied);
            assert!(receipt.overridden_actuator_indices.is_empty());
            for actuator in 0..21 {
                let applied_control = controls[actuator];
                let applied_torque = intended_torque[actuator];
                assert_eq!(receipt.applied_controls[actuator].to_bits(), applied_control.to_bits(),
                    "enabled={enabled}, native={native}, actuator={actuator}");
                assert_eq!(receipt.applied_torque_nm[actuator].to_bits(), applied_torque.to_bits(),
                    "enabled={enabled}, native={native}, actuator={actuator}");
            }
        }
    }
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
#[test]
fn motor_application_receipt_is_ephemeral_in_json_and_binary_simulation_serialization() {
    let mut s = simulation(true);
    s.substeps_completed = 2180;
    let mut controls = [-0.0; 21];
    controls[8] = 0.5;
    apply_controls(&mut s, &controls).unwrap();
    assert!(s.last_motor_application.is_some());
    let with_json = serde_json::to_string(&s).unwrap();
    let with_binary = bincode::serialize(&s).unwrap();
    let value: serde_json::Value = serde_json::from_str(&with_json).unwrap();
    assert!(value.get("last_motor_application").is_none());
    assert!(value.get("lastMotorApplication").is_none());
    let restored: Simulation = serde_json::from_str(&with_json).unwrap();
    assert!(restored.last_motor_application.is_none());
    s.last_motor_application = None;
    assert_eq!(serde_json::to_string(&s).unwrap(), with_json);
    assert_eq!(bincode::serialize(&s).unwrap(), with_binary);
    assert_eq!(s.substeps_completed, 2180);
    assert_eq!(s.tick, 0);
}

#[test]
fn motor_application_observer_preserves_normal_checkpoint_export_and_resume() {
    let runtime = crate::runtime::Runtime::new(lh_contracts::Mode::Protocol, 0.0, profile(true)).unwrap();
    let before = serde_json::to_string(&runtime).unwrap();
    let checkpoint = runtime.getup_checkpoint().unwrap();
    assert!(!checkpoint.is_empty());
    let restored = crate::runtime::Runtime::from_getup_checkpoint(&checkpoint).unwrap();
    assert_eq!(serde_json::to_string(&restored).unwrap(), before);
    assert_eq!(motion_words(restored.simulation()), motion_words(runtime.simulation()));
    assert_eq!(restored.simulation().substeps_completed, 0);
    assert_eq!(runtime.simulation().substeps_completed, 0);
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    {
        let identity = crate::runtime::Runtime::terminal_tail_build_identity();
        assert_eq!(identity["checkpointExportSupported"], true);
        assert_eq!(identity["checkpointResumeSupported"], true);
    }
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
#[test]
fn motor_application_foot_clearance_matches_source_capsule_endpoints_and_actual_floor_top() {
    let s = simulation(true);
    let world = bincode::serialize(&s.world).unwrap();
    let before_motion = motion_words(&s);
    let floor_top = s.world.colliders[s.floor].compute_aabb().maxs.y;
    let actual = foot_floor_clearance(&s).unwrap();
    for (side, name) in ["left_foot", "right_foot"].into_iter().enumerate() {
        let part = s.model.segments.iter().position(|part| part.id == name).unwrap();
        let primitives = &s.model.segments[part].reference.as_ref().unwrap().primitives;
        assert_eq!(primitives.len(), 2);
        assert!(primitives.iter().all(|primitive| primitive.kind == "capsule"));
        let body_pose = s.world.bodies[s.bodies[part]].position();
        let mut expected = f32::INFINITY;
        for capsule in primitives {
            for endpoint in [capsule.a, capsule.b] {
                let point = body_pose.transform_point(vector(endpoint));
                expected = expected.min(point.y - capsule.radius - floor_top);
            }
        }
        assert!((actual[side] - expected).abs() < 1e-7,
            "{name}: actual={}, direct capsule clearance={expected}", actual[side]);
    }
    assert_eq!(motion_words(&s), before_motion);
    assert_eq!(bincode::serialize(&s.world).unwrap(), world);
    assert_eq!(s.substeps_completed, 0);
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
#[test]
fn motor_application_foot_clearance_reads_changed_floor_shape_without_any_pose_or_velocity_write() {
    let mut s = simulation(true);
    let before_motion = motion_words(&s);
    let before = foot_floor_clearance(&s).unwrap();
    let old_floor_top = s.world.colliders[s.floor].compute_aabb().maxs.y;
    // Synthetic geometry fixture only: change floor thickness, keeping its
    // transform and every body coordinate untouched. No integration occurs.
    s.world.colliders[s.floor].set_shape(SharedShape::cuboid(12.0, 0.33, 10.5));
    let new_floor_top = s.world.colliders[s.floor].compute_aabb().maxs.y;
    let rise = new_floor_top - old_floor_top;
    assert!((rise - 0.25).abs() < 1e-7);
    let world = bincode::serialize(&s.world).unwrap();
    let after = foot_floor_clearance(&s).unwrap();
    for side in 0..2 {
        assert!((after[side] - (before[side] - rise)).abs() < 1e-7);
        assert!(after[side] < 0.0, "signed clearance reports intersection with the raised floor top");
    }
    assert_eq!(motion_words(&s), before_motion);
    assert_eq!(bincode::serialize(&s.world).unwrap(), world);
    assert_eq!(s.substeps_completed, 0);
}
