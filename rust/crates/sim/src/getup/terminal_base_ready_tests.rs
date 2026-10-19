//! Controller-contract fixtures, never physical recovery qualification. The
//! cached ordinary standing geometry is integrated once; all changed rates,
//! repeated samples, seeded counters, and input targets below are synthetic.
use super::*;
use crate::getup::{joint_state, physical_observation, slow_controls, supported_standing};
use std::sync::OnceLock;

fn world_words(s: &Simulation) -> Vec<u8> { bincode::serialize(&s.world).unwrap() }

fn settled_geometry() -> Simulation {
    static SETTLED: OnceLock<(String, Vec<u8>)> = OnceLock::new();
    let (auxiliary, world) = SETTLED.get_or_init(|| {
        let mut s = Simulation::new_protocol(0.0, Profile::default(), true).unwrap();
        for _ in 0..24 { s.advance_tick().unwrap(); }
        (serde_json::to_string(&s).unwrap(), world_words(&s))
    });
    let mut s: Simulation = serde_json::from_str(auxiliary).unwrap();
    s.world = bincode::deserialize(world).unwrap();
    let root = *s.world.multibody_joints.rigid_body_link(s.bodies[0]).unwrap();
    s.world.multibody_joints.get_multibody_mut(root.multibody).unwrap()
        .generalized_velocity_mut().fill(0.0);
    s
}

fn terminal_controller(s: &Simulation) -> Controller {
    let mut c = Controller::new(&s.model);
    assert!(c.terminal_base_ready_variant());
    c.started_substep = Some(s.substeps_completed - 20);
    c.impact_substep = c.started_substep;
    c.reference = vec![standing_target(&s.model); 2];
    c.status.reference_frames = 1;
    c.status.reference_index = 1;
    c.last_decision = Some(1);
    c.holding = true;
    c
}

fn joint_rate(s: &mut Simulation, qpos: usize, rate: f32) {
    let part = s.model.segments.iter().position(|p| p.reference.as_ref()
        .and_then(|r| r.hinge.as_ref()).is_some_and(|h| h.qpos_index == qpos)).unwrap();
    let id = *s.world.multibody_joints.rigid_body_link(s.bodies[part]).unwrap();
    let mb = s.world.multibody_joints.get_multibody_mut(id.multibody).unwrap();
    let coordinate = mb.link(id.id).unwrap().assembly_id();
    mb.generalized_velocity_mut()[coordinate] = rate;
}

#[test]
fn source_tree_motion_bounds_include_virtual_shoulders_and_head_and_leg_frames() {
    let s = settled_geometry();
    let contract = &recovery_contract::contract().recovery_acceptance;
    for (i, part) in s.model.segments.iter().enumerate() {
        let arm = terminal_base_ready_arm_body(&s.model, i);
        assert_eq!(arm, i >= 17, "source tree classification: {}", part.id);
        let mut motion = vec![(Vector::ZERO, Vector::ZERO); s.model.segments.len()];
        let linear = if arm { contract.settle_linear_mps } else { contract.stable_linear_mps };
        let angular = if arm { contract.settle_angular_radps } else { contract.stable_angular_radps };
        motion[i] = (Vector::X * linear, Vector::Y * angular);
        assert!(terminal_base_ready_motion_passes(&s.model, &motion), "{} inclusive", part.id);
        motion[i].0.x = linear + 0.001;
        assert!(!terminal_base_ready_motion_passes(&s.model, &motion), "{} linear", part.id);
        motion[i] = (Vector::ZERO, Vector::Y * (angular + 0.001));
        assert!(!terminal_base_ready_motion_passes(&s.model, &motion), "{} angular", part.id);
        motion[i].1.y = f32::NAN;
        assert!(!terminal_base_ready_motion_passes(&s.model, &motion), "{} finite", part.id);
    }
}

#[test]
fn independent_79_80_admission_preserves_source_prefix_and_all_body_acceptance() {
    let mut s = settled_geometry();
    joint_rate(&mut s, 17, 0.8); // Bounded moving elbow; original .65 angular fails.
    let physical = physical_observation(&s).unwrap();
    let loads = s.integrated_support_loads_n;
    let mut c = terminal_controller(&s);
    c.residual = [0.125; 21];
    assert!(c.terminal_base_ready_candidate(&s, &physical, &loads).is_some());
    let before = world_words(&s);
    for sample in 1..=79 {
        c.observe(&mut s, &loads);
        assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, sample);
        assert_eq!(c.stable_substeps, 0);
        assert!(c.stable_feet.is_none());
        assert!(!c.status.prototype_standing_passed);
        assert!(c.terminal_balance.is_none());
        let expected = slow_controls(&s.model, &c.reference_pose, &c.residual,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(), true).unwrap();
        assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits), expected.map(f32::to_bits));
        assert!(c.student_decisions.is_empty() && c.teacher_frames.is_empty());
        assert_eq!(c.status.policy_decisions, 0);
    }
    let anchors = serde_json::to_value(&c.terminal_base_ready.as_ref().unwrap().feet).unwrap();
    c.observe(&mut s, &loads);
    assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 80);
    assert_eq!(serde_json::to_value(&c.terminal_base_ready.as_ref().unwrap().feet).unwrap(), anchors);
    assert_eq!(c.stable_substeps, 0);
    assert!(!c.status.prototype_standing_passed);
    assert!(c.terminal_base_ready_variant_valid());
    assert_eq!(s.counters.recoveries, 0);
    assert_eq!(s.motion, Motion::Recovering);
    let controls = c.command(&s, &physical).unwrap();
    assert_eq!(c.controller_label(), "experimental-recovery-base-ready-joint-balance-hold");
    let balance = c.terminal_balance.as_ref().unwrap();
    let transition = balance.first_control_transition.as_ref().unwrap();
    assert_eq!(transition.preview.controls.map(f32::to_bits), controls.map(f32::to_bits));
    assert!(transition.preview.finite_active_unclipped());
    assert_eq!(transition.actuator_jumps.len(), 21);
    assert_eq!(transition.actuator_jumps.iter().filter(|j| j.role == "arm").count(), 6);
    let max_nm = transition.actuator_jumps.iter().map(|j| j.torque_jump_nm.abs()).fold(0.0, f32::max);
    assert_eq!(balance.first_actual_torque_jump_max_nm, Some(max_nm));
    assert_eq!(world_words(&s), before, "neither observation, preview nor binding writes the world");
}

#[test]
fn admission_failures_reset_only_its_counter_and_anchors() {
    let s = settled_geometry();
    let physical = physical_observation(&s).unwrap();
    let loads = s.integrated_support_loads_n;
    for failure in ["head", "torso", "nonfoot", "foot load", "persistence", "capture", "finite", "arm speed", "base speed", "floor", "drift", "terminal", "raw clipping"] {
        let mut local = settled_geometry();
        let mut c = terminal_controller(&local);
        c.observe_terminal_base_ready(&local, &physical, &loads);
        assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 1);
        c.stable_substeps = 31;
        c.stable_feet = Some([Vec3 { x: 9.0, y: 8.0, z: 7.0 }; 2]);
        let stable_feet = serde_json::to_value(c.stable_feet).unwrap();
        let mut p = physical;
        let mut l = loads;
        let foot = local.model.segments.iter().position(|p| p.id == "left_foot").unwrap();
        match failure {
            "head" => p[21] = 1.4,
            "torso" => p[36] = 0.96,
            "nonfoot" => l[0] = 3.0,
            "foot load" => l[foot] = 2.9,
            "persistence" => local.integrated_support_substeps[foot] = 39,
            "capture" => p[38] = 5.0,
            "finite" => p[61] = f32::NAN,
            "arm speed" => { joint_rate(&mut local, 17, 1.801); p = physical_observation(&local).unwrap(); },
            "base speed" => { joint_rate(&mut local, 0, 0.651); p = physical_observation(&local).unwrap(); },
            "floor" => local.world.colliders[local.floor].set_enabled(false),
            "drift" => c.terminal_base_ready.as_mut().unwrap().feet.as_mut().unwrap()[0].x += 0.011,
            "terminal" => c.status.reference_index = 0,
            "raw clipping" => c.reference_pose[17] += 2.0,
            _ => unreachable!(),
        }
        let before = world_words(&local);
        c.observe_terminal_base_ready(&local, &p, &l);
        let ready = c.terminal_base_ready.as_ref().unwrap();
        assert_eq!(ready.consecutive_substeps, 0, "{failure}");
        assert!(ready.feet.is_none(), "{failure}");
        assert!(c.terminal_balance.is_none(), "{failure}");
        assert_eq!(c.stable_substeps, 31, "{failure}");
        assert_eq!(serde_json::to_value(c.stable_feet).unwrap(), stable_feet, "{failure}");
        assert_eq!(world_words(&local), before, "{failure}");
    }
}

#[test]
fn exact_preview_detects_both_clamps_and_cannot_hide_initial_clipping() {
    let mut s = settled_geometry();
    let reference = supported_standing::recovered_reference(&s).unwrap();
    let (q, dq) = joint_state(&s).unwrap();
    let before = world_words(&s);
    let preview = supported_standing::terminal_control_preview(&s, &reference, &q, &q, &dq).unwrap();
    let production = supported_standing::controls_with_reference(&s, &reference, &q, &q, &dq).unwrap().0;
    assert_eq!(preview.controls.map(f32::to_bits), production.map(f32::to_bits));
    assert!(preview.finite_active_unclipped());
    let (actuator, feedforward) = preview.raw_compensated.iter().copied().enumerate()
        .find(|(_, f)| f.abs() > 0.01 && f.abs() < 1.0).unwrap();
    let hinge = s.model.segments.iter().filter_map(|p| p.reference.as_ref()?.hinge.as_ref())
        .find(|h| h.actuator_index == actuator).unwrap();
    let mut target = q;
    target[hinge.qpos_index] += feedforward.signum() * 0.999;
    let final_clip = supported_standing::terminal_control_preview(&s, &reference, &target, &q, &dq).unwrap();
    assert!(!final_clip.pd_saturated[actuator]);
    assert!(final_clip.compensated_saturated[actuator]);
    assert!(!final_clip.finite_active_unclipped());
    target[hinge.qpos_index] = q[hinge.qpos_index] - feedforward.signum() * 1.25;
    let hidden_clip = supported_standing::terminal_control_preview(&s, &reference, &target, &q, &dq).unwrap();
    assert!(hidden_clip.pd_saturated[actuator]);
    assert!(!hidden_clip.compensated_saturated[actuator]);
    assert!(hidden_clip.controls[actuator].abs() < 1.0);
    assert!(!hidden_clip.finite_active_unclipped());
    target[0] = f32::NAN;
    assert!(supported_standing::terminal_control_preview(&s, &reference, &target, &q, &dq).is_err());
    assert_eq!(world_words(&s), before);
    s.world.colliders[s.floor].set_enabled(false);
    let unsupported = supported_standing::terminal_control_preview(&s, &reference, &q, &q, &dq).unwrap();
    assert!(!unsupported.compensation_active);
    assert!(!unsupported.finite_active_unclipped());
}

#[test]
fn new_admission_cannot_change_799_800_acceptance_or_deadline() {
    let mut s = settled_geometry();
    let mut c = terminal_controller(&s);
    s.substeps_completed = 20_000;
    c.impact_substep = Some(0);
    c.stable_substeps = 798;
    let loads = s.integrated_support_loads_n;
    let before = world_words(&s);
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 799);
    assert!(!c.status.prototype_standing_passed);
    assert!(c.terminal_balance.is_none());
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 800);
    assert!(c.status.prototype_standing_passed);
    assert_eq!(s.counters.recoveries, 1);
    let mut late = terminal_controller(&s);
    late.impact_substep = Some(0);
    late.stable_substeps = 799;
    s.substeps_completed = 20_001;
    late.observe(&mut s, &loads);
    assert_eq!(late.stable_substeps, 800);
    assert!(!late.status.prototype_standing_passed);
    assert_eq!(late.command(&s, &physical_observation(&s).unwrap()).unwrap_err(), "25-second physical recovery deadline reached");
    assert_eq!(world_words(&s), before);
}
