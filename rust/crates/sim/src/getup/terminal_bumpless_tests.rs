//! Algebra/controller contract fixtures only. The cached ordinary upright
//! geometry is integrated once by the test harness; synthetic counters, rates,
//! command clocks and repeated observations never qualify physical recovery.
use super::*;
use crate::getup::{physical_observation, supported_standing};
use std::sync::OnceLock;

fn settled_geometry() -> Simulation {
    static SETTLED: OnceLock<(String, Vec<u8>)> = OnceLock::new();
    let (auxiliary, world) = SETTLED.get_or_init(|| {
        let mut s = Simulation::new_protocol(0.0, Profile::default(), true).unwrap();
        for _ in 0..24 { s.advance_tick().unwrap(); }
        (serde_json::to_string(&s).unwrap(), bincode::serialize(&s.world).unwrap())
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
    assert!(c.terminal_bumpless_variant());
    c.started_substep = Some(s.substeps_completed - 20);
    c.impact_substep = c.started_substep;
    c.reference = vec![standing_target(&s.model); 2];
    c.status.reference_frames = 1;
    c.status.reference_index = 1;
    c.last_decision = Some(1);
    c.holding = true;
    c
}

fn bound_controller(s: &Simulation) -> Controller {
    let mut c = terminal_controller(s);
    c.stable_substeps = 80;
    c.bind_terminal_balance(s);
    assert!(c.terminal_balance.is_some());
    assert!(c.terminal_base_ready_variant_valid());
    c
}

fn weight_derivatives(s: f64) -> (f64, f64) {
    (30.0 * s.powi(2) - 60.0 * s.powi(3) + 30.0 * s.powi(4),
     60.0 * s - 180.0 * s.powi(2) + 120.0 * s.powi(3))
}

#[test]
fn quintic_has_exact_endpoint_values_and_zero_first_second_derivatives() {
    for (s, expected) in [(0.0, 0.0), (1.0, 1.0)] {
        assert_eq!(terminal_bumpless_weight(s), expected);
        assert_eq!(weight_derivatives(s), (0.0, 0.0));
    }
    assert_eq!(terminal_bumpless_weight(0.5), 0.5);
    assert_eq!(terminal_bumpless_weight(-1.0), 0.0);
    assert_eq!(terminal_bumpless_weight(2.0), 1.0);
    let weights: Vec<_> = (0..=20).map(|k| terminal_bumpless_weight(k as f64 / 20.0)).collect();
    assert!(weights.windows(2).all(|w| w[0] <= w[1]));
}

#[test]
fn constant_linear_and_quadratic_live_references_match_endpoint_derivatives() {
    let source = -0.75;
    // b(s)=a+bs+cs², including nonconstant live target derivatives.
    for (a, b, c) in [(0.5, 0.0, 0.0), (0.125, 0.5, 0.0), (0.125, -0.25, 0.5)] {
        for s in [0.0, 1.0] {
            let live = a + b * s + c * s * s;
            let live_prime = b + 2.0 * c * s;
            let live_second = 2.0 * c;
            let w = terminal_bumpless_weight(s);
            let (wp, wpp) = weight_derivatives(s);
            let output = (1.0 - w) * source + w * live;
            let first = wp * (live - source) + w * live_prime;
            let second = wpp * (live - source) + 2.0 * wp * live_prime + w * live_second;
            if s == 0.0 {
                assert_eq!((output, first, second), (source, 0.0, 0.0));
            } else {
                assert_eq!((output, first, second), (live, live_prime, live_second));
            }
        }
    }
}

#[test]
fn all_twenty_intervals_preserve_endpoint_words_and_convex_bounds() {
    let endpoints = [-1.0_f32, -0.999, -0.25, -0.0, 0.0, f32::MIN_POSITIVE, 0.25, 0.999, 1.0];
    let mut pairs: Vec<_> = endpoints.iter().flat_map(|a| endpoints.iter().map(move |b| (*a, *b))).collect();
    // Deterministic finite vectors, no external random state or extra history.
    let mut seed = 0x97a3_4e21_u32;
    for _ in 0..256 {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let a = (f64::from(seed) / f64::from(u32::MAX) * 2.0 - 1.0) as f32;
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let b = (f64::from(seed) / f64::from(u32::MAX) * 2.0 - 1.0) as f32;
        pairs.push((a, b));
    }
    for pair in pairs.chunks(21) {
        let source = std::array::from_fn(|i| pair[i % pair.len()].0);
        let live = std::array::from_fn(|i| pair[i % pair.len()].1);
        for k in 0..=20 {
            let (actual, clamped) = terminal_bumpless_controls(&source, &live, k).unwrap();
            assert_eq!(clamped, 0);
            assert!(normalized_controls_valid(&actual));
            for i in 0..21 {
                assert!(actual[i] >= source[i].min(live[i]) && actual[i] <= source[i].max(live[i]));
            }
            if k == 0 { assert_eq!(actual.map(f32::to_bits), source.map(f32::to_bits)); }
            else if k == 20 { assert_eq!(actual.map(f32::to_bits), live.map(f32::to_bits)); }
            else {
                let w = terminal_bumpless_weight(k as f64 / 20.0);
                let expected: [f32; 21] = std::array::from_fn(|i|
                    ((1.0 - w) * f64::from(source[i]) + w * f64::from(live[i])) as f32);
                assert_eq!(actual.map(f32::to_bits), expected.map(f32::to_bits));
            }
        }
        assert_eq!(terminal_bumpless_controls(&source, &live, u64::MAX).unwrap().0.map(f32::to_bits), live.map(f32::to_bits));
    }
    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 1.0001, -1.0001] {
        let mut source = [0.0; 21];
        source[13] = invalid;
        assert!(terminal_bumpless_controls(&source, &[0.0; 21], 0).is_err());
        assert!(terminal_bumpless_controls(&[0.0; 21], &source, 20).is_err());
    }
}

#[test]
fn full_body80_fallback_survives_rejected_early_preview_with_persisted_provenance() {
    let mut s = settled_geometry();
    let physical = physical_observation(&s).unwrap();
    let loads = s.integrated_support_loads_n;
    let mut c = terminal_controller(&s);
    c.reference_pose[17] += 2.0; // Strict early preview rejects saturated PD.
    c.stable_substeps = 79;
    c.terminal_base_ready.as_mut().unwrap().consecutive_substeps = 79;
    c.terminal_base_ready.as_mut().unwrap().feet = Some([Vec3 { x: 0.0, y: 0.0, z: 0.0 }; 2]);
    assert!(c.terminal_base_ready_candidate(&s, &physical, &loads).is_none());
    let world = bincode::serialize(&s.world).unwrap();
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 80);
    assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 0);
    assert!(c.terminal_base_ready.as_ref().unwrap().feet.is_none());
    let balance = c.terminal_balance.as_ref().unwrap();
    let blend = balance.bumpless.as_ref().unwrap();
    assert!(matches!(blend.admission, TerminalAdmission::FullBody80));
    assert_eq!(blend.stable_substeps_at_binding, 80);
    assert!(c.terminal_base_ready_variant_valid());
    assert!(!c.status.prototype_standing_passed);
    assert_eq!(s.counters.recoveries, 0);
    let source = balance.last_slow_controls;
    assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits), source.map(f32::to_bits));
    // Future acceptance samples can reset; binding validity uses captured proof.
    c.stable_substeps = 0;
    c.stable_feet = None;
    assert!(c.terminal_base_ready_variant_valid());
    assert_eq!(bincode::serialize(&s.world).unwrap(), world);
}

#[test]
fn original_full_body_admission_retains_all_lifecycle_and_recovered_support_guards() {
    let s = settled_geometry();
    for guard in ["not started", "empty reference", "not final", "no decision", "decision before terminal", "not holding", "no support"] {
        let mut local = settled_geometry();
        let mut c = terminal_controller(&s);
        c.stable_substeps = 80;
        match guard {
            "not started" => c.started_substep = None,
            "empty reference" => c.reference.clear(),
            "not final" => c.status.reference_index = 0,
            "no decision" => c.last_decision = None,
            "decision before terminal" => c.last_decision = Some(0),
            "not holding" => c.holding = false,
            "no support" => local.world.colliders[local.floor].set_enabled(false),
            _ => unreachable!(),
        }
        c.bind_terminal_balance(&local);
        assert!(c.terminal_balance.is_none(), "{guard}");
    }
}

#[test]
fn restored_seventy_nine_to_eighty_early_admission_captures_source_and_actor_prefix_once() {
    let mut s = settled_geometry();
    // Synthetic bounded elbow rate excludes the unchanged whole-body gate.
    let part = s.model.segments.iter().position(|p| p.reference.as_ref()
        .and_then(|r| r.hinge.as_ref()).is_some_and(|h| h.qpos_index == 17)).unwrap();
    let link = *s.world.multibody_joints.rigid_body_link(s.bodies[part]).unwrap();
    let mb = s.world.multibody_joints.get_multibody_mut(link.multibody).unwrap();
    let coordinate = mb.link(link.id).unwrap().assembly_id();
    mb.generalized_velocity_mut()[coordinate] = 0.8;
    let physical = physical_observation(&s).unwrap();
    let loads = s.integrated_support_loads_n;
    let mut c = terminal_controller(&s);
    for _ in 0..79 { c.observe(&mut s, &loads); }
    assert_eq!(c.stable_substeps, 0);
    assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 79);
    assert!(c.terminal_balance.is_none());
    let serialized = serde_json::to_string(&c).unwrap();
    let mut restored: Controller = serde_json::from_str(&serialized).unwrap();
    c.observe(&mut s, &loads);
    restored.observe(&mut s, &loads);
    assert_eq!(serde_json::to_string(&c).unwrap(), serde_json::to_string(&restored).unwrap());
    assert_eq!(c.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 80);
    let balance = c.terminal_balance.as_ref().unwrap();
    assert!(matches!(balance.bumpless.as_ref().unwrap().admission, TerminalAdmission::EarlyBase80));
    assert_eq!(balance.bumpless.as_ref().unwrap().stable_substeps_at_binding, 0);
    assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits),
        restored.command(&s, &physical).unwrap().map(f32::to_bits));
    assert!(c.teacher_frames.is_empty() && c.student_decisions.is_empty());
    assert_eq!(c.status.policy_decisions, 0);
    assert_eq!(c.status.teacher_decisions, 0);
    assert_eq!(c.status.forecast_substeps, 0);
}

#[test]
fn ready_label_and_invalid_binding_refusal_are_explicit_and_do_not_run_actors() {
    let s = settled_geometry();
    let physical = physical_observation(&s).unwrap();
    let mut fresh = Controller::new(&s.model);
    fresh.command(&s, &physical).unwrap();
    assert_eq!(fresh.controller_label(), "experimental-recovery-bumpless-joint-balance-ready");
    assert!(fresh.terminal_balance.is_none());
    for invalid in [f32::NAN, f32::INFINITY, 1.001, -1.001] {
        let mut c = bound_controller(&s);
        c.terminal_balance.as_mut().unwrap().last_slow_controls[0] = invalid;
        assert!(!c.terminal_base_ready_variant_valid());
        assert!(c.command(&s, &physical).is_err());
        assert!(c.teacher_frames.is_empty() && c.student_decisions.is_empty());
        assert_eq!(c.status.policy_decisions, 0);
    }
    let mut future = bound_controller(&s);
    future.terminal_balance.as_mut().unwrap().activated_native_substep = s.substeps_completed + 1;
    let before = serde_json::to_string(&future).unwrap();
    assert_eq!(future.command(&s, &physical).unwrap_err(),
        "Terminal bumpless activation is ahead of the native clock");
    assert_eq!(serde_json::to_string(&future).unwrap(), before);
}

#[test]
fn blend_uses_only_native_clock_preserves_actor_prefix_and_separates_intended_jump() {
    let mut s = settled_geometry();
    s.diagnostics.getup.as_mut().unwrap().motor_controls = vec![0.125; 21];
    let mut c = bound_controller(&s);
    let epoch = s.substeps_completed;
    let physical = physical_observation(&s).unwrap();
    let world = bincode::serialize(&s.world).unwrap();
    let prefix = serde_json::json!({"teacher": c.teacher_frames, "student": c.student_decisions,
        "decision": c.last_decision, "teacherCount": c.status.teacher_decisions,
        "policyCount": c.status.policy_decisions, "forecast": c.status.forecast_substeps});
    for k in 0..=25 {
        s.substeps_completed = epoch + k;
        let live = supported_standing::controls_with_reference(&s,
            &c.terminal_balance.as_ref().unwrap().reference, &c.reference_pose,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap()).unwrap().0;
        let expected = terminal_bumpless_controls(&[0.125; 21], &live, k).unwrap().0;
        let actual = c.command(&s, &physical).unwrap();
        assert_eq!(actual.map(f32::to_bits), expected.map(f32::to_bits));
        let after_first = serde_json::to_value(&c).unwrap();
        for _ in 0..3 {
            assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits), actual.map(f32::to_bits));
            assert_eq!(serde_json::to_value(&c).unwrap(), after_first);
        }
        assert_eq!(c.status.phase.contains("blending"), k < 20);
        assert_eq!(c.controller_label(), "experimental-recovery-bumpless-joint-balance-hold");
        assert_eq!(serde_json::json!({"teacher": c.teacher_frames, "student": c.student_decisions,
            "decision": c.last_decision, "teacherCount": c.status.teacher_decisions,
            "policyCount": c.status.policy_decisions, "forecast": c.status.forecast_substeps}), prefix);
    }
    let balance = c.terminal_balance.as_ref().unwrap();
    assert_eq!(balance.first_balance_controls.unwrap().map(f32::to_bits), [0.125_f32; 21].map(f32::to_bits));
    assert_eq!(balance.first_actual_control_jump_max, Some(0.0));
    assert_eq!(balance.first_actual_torque_jump_max_nm, Some(0.0));
    assert!(balance.first_control_transition.as_ref().unwrap().actuator_jumps.iter()
        .all(|j| j.normalized_jump == 0.0 && j.torque_jump_nm == 0.0));
    let intended = balance.bumpless.as_ref().unwrap();
    assert!(intended.first_intended_control_jump_max.unwrap() > 0.0);
    assert!(intended.first_intended_torque_jump_max_nm.unwrap() > 0.0);
    assert_eq!(intended.first_intended_controls.unwrap().map(f32::to_bits),
        balance.first_control_transition.as_ref().unwrap().preview.controls.map(f32::to_bits));
    assert_eq!(intended.last_blend_clamped_controls, 0);
    assert_eq!(bincode::serialize(&s.world).unwrap(), world);
}

#[test]
fn support_loss_stays_bounded_and_rejoins_exact_live_fallback_at_twenty() {
    let mut s = settled_geometry();
    let mut c = bound_controller(&s);
    s.world.colliders[s.floor].set_enabled(false);
    let physical = physical_observation(&s).unwrap();
    let epoch = s.substeps_completed;
    for k in 0..=20 {
        s.substeps_completed = epoch + k;
        let live = supported_standing::controls_with_reference(&s,
            &c.terminal_balance.as_ref().unwrap().reference, &c.reference_pose,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap()).unwrap();
        assert!(!live.1);
        let actual = c.command(&s, &physical).unwrap();
        assert!(normalized_controls_valid(&actual));
        if k == 20 { assert_eq!(actual.map(f32::to_bits), live.0.map(f32::to_bits)); }
    }
}

#[test]
fn fallback_does_not_change_800_consecutive_samples_or_25_second_deadline_and_reset() {
    let mut s = settled_geometry();
    let mut c = terminal_controller(&s);
    s.substeps_completed = 20_000;
    c.impact_substep = Some(0);
    c.stable_substeps = 798;
    let loads = s.integrated_support_loads_n;
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 799);
    assert!(c.terminal_balance.is_some());
    assert!(!c.status.prototype_standing_passed);
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
    assert_eq!(late.command(&s, &physical_observation(&s).unwrap()).unwrap_err(),
        "25-second physical recovery deadline reached");
    let fresh = Controller::new(&s.model);
    assert!(fresh.terminal_bumpless_variant());
    assert!(fresh.terminal_balance.is_none());
    assert_eq!(fresh.terminal_base_ready.as_ref().unwrap().consecutive_substeps, 0);
    assert!(fresh.terminal_base_ready.as_ref().unwrap().feet.is_none());
    assert!(fresh.teacher_frames.is_empty() && fresh.student_decisions.is_empty());
    assert_eq!(fresh.status.policy_decisions, 0);
}

#[test]
fn retained_v1_and_legacy_commands_remain_exact_live_balance_in_v2_build() {
    let s = settled_geometry();
    let physical = physical_observation(&s).unwrap();
    for v1 in [false, true] {
        let mut c = terminal_controller(&s);
        if v1 {
            let ready = c.terminal_base_ready.as_mut().unwrap();
            ready.format = TerminalBaseReadyFormat::V1;
            ready.consecutive_substeps = 80;
            ready.feet = Some([Vec3 { x: 0.0, y: 0.0, z: 0.0 }; 2]);
        } else { c.terminal_base_ready = None; }
        c.stable_substeps = 80;
        c.bind_terminal_balance(&s);
        let balance = c.terminal_balance.as_ref().unwrap();
        assert!(balance.bumpless.is_none());
        let live = supported_standing::controls_with_reference(&s, &balance.reference, &c.reference_pose,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap()).unwrap().0;
        assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits), live.map(f32::to_bits));
        assert!(!serde_json::to_string(&c).unwrap().contains("bumpless"));
        if v1 {
            assert_eq!(c.terminal_balance.as_ref().unwrap().first_control_transition.as_ref().unwrap()
                .preview.controls.map(f32::to_bits), live.map(f32::to_bits));
        }
    }
}
