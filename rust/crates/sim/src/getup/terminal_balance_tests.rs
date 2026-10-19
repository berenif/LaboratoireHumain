//! Focused controller-contract tests, not physical recovery qualification.
//! Only `settled_geometry` integrates: it records ordinary upright standing.
//! Repeated observations, seeded counters, zeroed rates, and rigidly transformed
//! poses below are explicitly synthetic fixtures and never recovery evidence.
use super::*;
use crate::getup::{physical_observation, slow_controls};
#[cfg(feature = "getup-terminal-balance-experiment")]
use crate::getup::joint_state;
use crate::getup::supported_standing;
use bincode::Options;
use std::sync::OnceLock;

fn world_words(s: &Simulation) -> Vec<u8> {
    // Solved Rapier maps have compound keys and cannot be compared as JSON.
    bincode::serialize(&s.world).unwrap()
}

fn settled_geometry() -> Simulation {
    static SETTLED: OnceLock<(String, Vec<u8>)> = OnceLock::new();
    let (auxiliary, world) = SETTLED.get_or_init(|| {
        let mut s = Simulation::new_protocol(0.0, Profile::default(), true).unwrap();
        for _ in 0..24 {
            s.advance_tick().unwrap();
        }
        assert_eq!(s.substeps_completed, 480);
        let diagnostic = standing_diagnostic(&s, None).unwrap();
        for (name, passed) in diagnostic["predicates"].as_object().unwrap() {
            if name != "footDriftFromSuppliedAnchors" {
                assert_eq!(passed, true, "settled standing predicate: {name}");
            }
        }
        (serde_json::to_string(&s).unwrap(), world_words(&s))
    });
    let mut s: Simulation = serde_json::from_str(auxiliary).unwrap();
    s.world = bincode::deserialize(world).unwrap();
    s
}

/// Synthetic lifecycle state only: no fall, teacher, or recovery is claimed.
fn terminal_controller(s: &Simulation) -> Controller {
    let mut c = Controller::new(&s.model);
    c.terminal_base_ready = None; // This suite specifies the retained legacy variant.
    c.started_substep = Some(s.substeps_completed - 20);
    c.impact_substep = c.started_substep;
    c.reference = vec![standing_target(&s.model); 2];
    c.status.reference_frames = 1; // The appended terminal target is excluded.
    c.status.reference_index = 1;
    c.last_decision = Some(1);
    c.holding = true;
    c
}

fn control_words(controls: [f32; 21]) -> [u32; 21] {
    controls.map(f32::to_bits)
}

#[test]
fn preeligible_commands_keep_exact_source_control_words_in_every_build() {
    let mut s = Simulation::new_protocol(0.0, Profile::default(), true).unwrap();
    s.substeps_completed = 20; // Synthetic clock only; no source actor is run.
    let mut physical = physical_observation(&s).unwrap();
    // Nonzero input rates exercise the original following/holding PD constants.
    for (i, rate) in physical[46..67].iter_mut().enumerate() {
        *rate = (i as f32 - 10.0) * 0.03125;
    }
    let before = world_words(&s);
    for state in ["pre-recovery holding", "unfinished reference", "79 stable samples"] {
        #[cfg(feature = "getup-terminal-balance-experiment")]
        if state == "pre-recovery holding" {
            continue; // The experimental ready branch is tested separately.
        }
        let mut c = Controller::new(&s.model);
        c.residual = std::array::from_fn(|i| (i as f32 - 10.0) * 0.0625);
        if state != "pre-recovery holding" {
            c.started_substep = Some(if state == "79 stable samples" { 0 } else { 20 });
            c.reference = vec![standing_target(&s.model); 2];
            c.stable_substeps = if state == "79 stable samples" { 79 } else { 80 };
            c.holding = state == "79 stable samples";
            c.status.reference_index = u32::from(c.holding);
            c.last_decision = Some(u64::from(c.holding));
        }
        let expected = slow_controls(
            &s.model, &c.reference_pose, &c.residual,
            physical[..21].try_into().unwrap(),
            physical[46..67].try_into().unwrap(), c.holding,
        ).unwrap();
        assert_eq!(control_words(c.command(&s, &physical).unwrap()), control_words(expected), "{state}");
        assert!(c.terminal_balance.is_none(), "{state}");
        assert_eq!(c.controller_label(), "pretrained-WEAK-SLOW");
    }
    assert_eq!(world_words(&s), before);
}

#[cfg(feature = "getup-terminal-balance-experiment")]
#[test]
fn ready_branch_keeps_exact_existing_standing_controls_without_actor_or_state_writes() {
    let s = settled_geometry();
    let physical = physical_observation(&s).unwrap();
    let mut c = Controller::new(&s.model);
    c.terminal_base_ready = None;
    let expected = supported_standing::controls(
        &s, &standing_target(&s.model)[..21].try_into().unwrap(),
        physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(),
    ).unwrap();
    let before_world = world_words(&s);
    let before_simulation = serde_json::to_value(&s).unwrap();
    let mut expected_controller = serde_json::to_value(&c).unwrap();
    expected_controller["status"]["phase"] = serde_json::json!("experimental-joint-balance-awaiting-fall");
    for _ in 0..2 {
        assert_eq!(control_words(c.command(&s, &physical).unwrap()), control_words(expected));
        assert_eq!(c.status.phase, "experimental-joint-balance-awaiting-fall");
        assert_eq!(c.controller_label(), "experimental-recovery-joint-balance-ready");
        assert_eq!(serde_json::to_value(&c).unwrap(), expected_controller,
            "only the explicit phase may change; actor counters, archives, reference, and binding stay untouched");
        assert_eq!(serde_json::to_value(&s).unwrap(), before_simulation);
        assert_eq!(world_words(&s), before_world);
    }
    assert!(c.started_substep.is_none());
    assert!(c.last_decision.is_none());
    assert!(c.teacher_frames.is_empty() && c.student_decisions.is_empty());
    assert_eq!(c.status.forecast_substeps, 0);
    assert_eq!(c.status.teacher_decisions, 0);
    assert_eq!(c.status.policy_decisions, 0);
}

#[test]
fn synthetic_counter_fixture_preserves_the_799_800_sample_and_25_second_gates() {
    let mut s = settled_geometry();
    let mut c = terminal_controller(&s);
    // A real settled geometry, but a synthetic elapsed clock and counter.
    // These two observer calls do not represent 800 physical recovery samples.
    s.substeps_completed = 20_000;
    c.started_substep = Some(0);
    c.impact_substep = Some(0);
    c.stable_substeps = 798;
    let loads = s.integrated_support_loads_n;
    let before = world_words(&s);
    let recoveries = s.counters.recoveries;
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 799);
    assert!(!c.status.prototype_standing_passed);
    assert_eq!(s.counters.recoveries, recoveries);
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 800);
    assert_eq!(c.status.stable_standing_s, 1.0);
    assert_eq!(c.status.since_impact_s, Some(25.0));
    assert!(c.status.prototype_standing_passed);
    assert_eq!(s.counters.recoveries, recoveries + 1);
    assert_eq!(s.motion, Motion::Upright);
    c.observe(&mut s, &loads);
    assert_eq!(s.counters.recoveries, recoveries + 1, "accept only once");
    assert_eq!(world_words(&s), before, "observer never integrates or edits poses");

    let mut late = terminal_controller(&s);
    late.impact_substep = Some(0);
    late.stable_substeps = 799;
    s.substeps_completed = 20_001;
    late.observe(&mut s, &loads);
    assert_eq!(late.stable_substeps, 800);
    assert!(!late.status.prototype_standing_passed, "a late synthetic count cannot accept");
    assert_eq!(s.counters.recoveries, recoveries + 1);
}

#[cfg(feature = "getup-terminal-balance-experiment")]
fn bound_controller(s: &Simulation) -> Controller {
    let mut c = terminal_controller(s);
    c.stable_substeps = 80;
    c.bind_terminal_balance(s);
    assert!(c.terminal_balance.is_some());
    c
}

#[cfg(feature = "getup-terminal-balance-experiment")]
#[test]
fn binding_requires_started_complete_terminal_and_80_samples_and_is_read_only_once() {
    let mut s = settled_geometry();
    for guard in ["pre-recovery holding", "empty reference", "unfinished reference", "no SLOW decision", "SLOW decision before terminal", "not holding"] {
        let mut c = terminal_controller(&s);
        c.stable_substeps = 80;
        match guard {
            "pre-recovery holding" => c.started_substep = None,
            "empty reference" => c.reference.clear(),
            "unfinished reference" => {
                // reference_frames - 1 is the last interpolated frame, not the
                // appended terminal index, even if holding was stale/true.
                c.status.reference_index = c.status.reference_frames - 1;
            }
            "no SLOW decision" => c.last_decision = None,
            "SLOW decision before terminal" => c.last_decision = Some(0),
            "not holding" => c.holding = false,
            _ => unreachable!(),
        }
        c.bind_terminal_balance(&s);
        assert!(c.terminal_balance.is_none(), "{guard}");
    }

    let mut c = terminal_controller(&s);
    s.motion = Motion::Recovering;
    let before = world_words(&s);
    let heading = s.heading;
    let rest_com = s.rest_com;
    let counters = serde_json::to_value(&s.counters).unwrap();
    let loads = s.integrated_support_loads_n;
    // Synthetic observer-only sample sequence on one unchanged solved geometry.
    for sample in 1..=79 {
        c.observe(&mut s, &loads);
        assert_eq!(c.stable_substeps, sample);
        assert!(c.terminal_balance.is_none(), "sample {sample}");
    }
    let anchors = serde_json::to_value(c.stable_feet).unwrap();
    let accepted = c.status.prototype_standing_passed;
    c.observe(&mut s, &loads);
    assert_eq!(c.stable_substeps, 80);
    assert_eq!(serde_json::to_value(c.stable_feet).unwrap(), anchors);
    assert_eq!(c.status.prototype_standing_passed, accepted);
    assert!(!accepted, "80 samples are not the 800-sample acceptance gate");
    assert_eq!(serde_json::to_value(&s.counters).unwrap(), counters);
    assert_eq!(s.heading, heading);
    assert_eq!(s.rest_com, rest_com);
    assert_eq!(world_words(&s), before);
    let bound = serde_json::to_value(&c.terminal_balance).unwrap();
    let balance = c.terminal_balance.as_ref().unwrap();
    assert_eq!(balance.activated_native_substep, s.substeps_completed);
    assert_eq!(balance.reference.com, s.center_of_mass());
    assert_eq!(balance.last_slow_controls.as_slice(), s.diagnostics.getup.as_ref().unwrap().motor_controls);
    assert!(balance.first_balance_controls.is_none());
    // A different epoch and source controls must not recapture the binding.
    s.substeps_completed += 1;
    s.diagnostics.getup.as_mut().unwrap().motor_controls.fill(0.75);
    c.bind_terminal_balance(&s);
    assert_eq!(serde_json::to_value(&c.terminal_balance).unwrap(), bound);

    let decisions = c.status.policy_decisions;
    let archived = c.student_decisions.len();
    let last_decision = c.last_decision;
    let controls = c.command(&s, &physical_observation(&s).unwrap()).unwrap();
    let balance = c.terminal_balance.as_ref().unwrap();
    let jump = controls.iter().zip(balance.last_slow_controls)
        .map(|(a, b)| (*a - b).abs()).fold(0.0, f32::max);
    assert_eq!(balance.first_balance_controls, Some(controls));
    assert_eq!(balance.first_actual_control_jump_max, Some(jump));
    assert_eq!(balance.first_saturated_controls, Some(controls.iter().filter(|v| v.abs() >= 1.0).count()));
    assert_eq!(balance.first_compensation_active, Some(true));
    assert_eq!(c.status.policy_decisions, decisions);
    assert_eq!(c.student_decisions.len(), archived);
    assert_eq!(c.last_decision, last_decision);
    assert_eq!(world_words(&s), before);
}

#[cfg(feature = "getup-terminal-balance-experiment")]
#[test]
fn active_balance_cannot_bypass_the_existing_deadline() {
    let mut s = settled_geometry();
    let mut c = bound_controller(&s);
    c.impact_substep = Some(0);
    s.substeps_completed = 20_000; // Synthetic lifecycle boundary, no integration.
    let before = world_words(&s);
    let error = c.command(&s, &physical_observation(&s).unwrap()).unwrap_err();
    assert_eq!(error, "25-second physical recovery deadline reached");
    assert!(c.terminal_balance.as_ref().unwrap().first_balance_controls.is_none());
    assert_eq!(world_words(&s), before);
}

#[cfg(feature = "getup-terminal-balance-experiment")]
#[test]
fn bound_controls_are_covariant_inside_the_floor_in_algebraic_rigid_pose_fixtures() {
    let mut original = settled_geometry();
    let root = *original.world.multibody_joints.rigid_body_link(original.bodies[0]).unwrap();
    // Counterfactual zero generalized rates, never integrated or acceptance-tested.
    original.world.multibody_joints.get_multibody_mut(root.multibody).unwrap()
        .generalized_velocity_mut().fill(0.0);
    let mut base = bound_controller(&original);
    let reference = &mut base.terminal_balance.as_mut().unwrap().reference;
    reference.com -= reference.forward * 0.02; // Exercise a nonzero COM intention.
    let base_controller = serde_json::to_value(&base).unwrap();
    let original_world = world_words(&original);
    let expected = base.command(&original, &physical_observation(&original).unwrap()).unwrap();
    assert_eq!(base.terminal_balance.as_ref().unwrap().first_compensation_active, Some(true));
    assert_eq!(world_words(&original), original_world);

    for (translation, yaw) in [
        (Vector::new(0.35, 0.0, -0.25), 0.0),
        (Vector::ZERO, std::f32::consts::FRAC_PI_3),
        (Vector::new(-0.6, 0.0, 0.25), 124.0_f32.to_radians()),
    ] {
        let mut transformed = super::fork(&original).unwrap();
        transformed.integrated_support_loads_n = original.integrated_support_loads_n;
        transformed.integrated_support_substeps = original.integrated_support_substeps;
        transformed.substeps_completed = original.substeps_completed;
        let yaw = Rotation::from_rotation_y(yaw);
        for handle in &transformed.bodies {
            let body = &mut transformed.world.bodies[*handle];
            body.set_position(Pose::from_parts(
                yaw * body.translation() + translation, yaw * *body.rotation(),
            ), false);
        }
        transformed.world.bodies.propagate_modified_body_positions_to_colliders(&mut transformed.world.colliders);
        // Original scene heading/rest COM are deliberately untouched. Only the
        // captured controller intention follows the algebraic rigid transform.
        let mut c: Controller = serde_json::from_value(base_controller.clone()).unwrap();
        let reference = &mut c.terminal_balance.as_mut().unwrap().reference;
        reference.com = yaw * reference.com + translation;
        reference.forward = yaw * reference.forward;
        reference.lateral = yaw * reference.lateral;
        let floor = transformed.world.colliders[transformed.floor].compute_aabb();
        for name in ["left_foot", "right_foot"] {
            let i = transformed.model.segments.iter().position(|p| p.id == name).unwrap();
            let bounds = transformed.world.colliders[transformed.colliders[i]].compute_aabb();
            for axis in [0, 2] {
                assert!(bounds.mins[axis] > floor.mins[axis] && bounds.maxs[axis] < floor.maxs[axis]);
            }
        }
        assert_eq!(joint_state(&transformed).unwrap(), joint_state(&original).unwrap());
        let before = world_words(&transformed);
        let actual = c.command(&transformed, &physical_observation(&transformed).unwrap()).unwrap();
        assert_eq!(c.terminal_balance.as_ref().unwrap().first_compensation_active, Some(true));
        let error = actual.iter().zip(expected).map(|(a, b)| (*a - b).abs()).fold(0.0, f32::max);
        assert!(error < 0.00001, "rigid control covariance error {error}, translation {translation:?}");
        assert_eq!(world_words(&transformed), before, "algebraic command does not integrate");
    }
}

#[cfg(feature = "getup-terminal-balance-experiment")]
#[test]
fn active_support_loss_returns_exact_zero_residual_holding_pd() {
    let original = settled_geometry();
    let mut c = bound_controller(&original);
    c.residual.fill(0.75); // Old SLOW residual must not leak into the fallback.
    let controller = serde_json::to_value(c).unwrap();
    for loss in ["floor disabled", "left foot unloaded", "right foot unloaded", "nonfoot support", "torso inverted"] {
        let mut s = settled_geometry();
        match loss {
            "floor disabled" => s.world.colliders[s.floor].set_enabled(false),
            "left foot unloaded" | "right foot unloaded" => {
                let name = if loss == "left foot unloaded" { "left_foot" } else { "right_foot" };
                let i = s.model.segments.iter().position(|p| p.id == name).unwrap();
                s.integrated_support_loads_n[i] = 0.0;
            }
            "nonfoot support" => s.integrated_support_loads_n[0] = 3.0,
            "torso inverted" => s.world.bodies[s.bodies[0]].set_rotation(Rotation::from_rotation_x(std::f32::consts::PI), false),
            _ => unreachable!(),
        }
        let mut c: Controller = serde_json::from_value(controller.clone()).unwrap();
        let physical = physical_observation(&s).unwrap();
        let expected = slow_controls(&s.model, &c.reference_pose, &[0.0; 21],
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(), true).unwrap();
        let before = world_words(&s);
        assert_eq!(control_words(c.command(&s, &physical).unwrap()), control_words(expected), "{loss}");
        assert_eq!(c.terminal_balance.as_ref().unwrap().first_compensation_active, Some(false), "{loss}");
        assert_eq!(world_words(&s), before);
    }
}

#[test]
fn legacy_json_omits_none_and_active_checkpoints_roundtrip_or_explicitly_refuse() {
    let s = settled_geometry();
    let legacy = Controller::new(&s.model);
    let mut legacy = legacy;
    legacy.terminal_base_ready = None;
    let legacy_json = serde_json::to_value(&legacy).unwrap();
    assert!(legacy_json.get("terminal_balance").is_none());
    let decoded: Controller = serde_json::from_value(legacy_json.clone()).unwrap();
    assert!(decoded.terminal_balance.is_none());
    assert!(decoded.checkpoint_variant_supported());
    assert_eq!(serde_json::to_value(decoded).unwrap(), legacy_json);

    // Explicit ready state also requires the experimental build, even though
    // no terminal binding has been captured and that optional field is absent.
    let mut ready = Controller::new(&s.model);
    ready.terminal_base_ready = None;
    ready.status.phase = "experimental-joint-balance-awaiting-fall".into();
    let ready_json = serde_json::to_value(&ready).unwrap();
    assert!(ready_json.get("terminal_balance").is_none());
    let ready: Controller = serde_json::from_value(ready_json.clone()).unwrap();
    assert_eq!(ready.checkpoint_variant_supported(), cfg!(feature = "getup-terminal-balance-experiment"));
    #[cfg(not(feature = "getup-terminal-balance-experiment"))]
    {
        let mut ready = ready;
        assert_eq!(ready.command(&s, &physical_observation(&s).unwrap()).unwrap_err(),
            "Active terminal balance requires its experimental controller build");
        assert_eq!(serde_json::to_value(&ready).unwrap(), ready_json);
    }
    #[cfg(feature = "getup-terminal-balance-experiment")]
    assert_eq!(ready.controller_label(), "experimental-recovery-joint-balance-ready");

    let mut c = terminal_controller(&s);
    c.stable_substeps = 80;
    #[cfg(feature = "getup-terminal-balance-experiment")]
    {
        c.bind_terminal_balance(&s);
        c.command(&s, &physical_observation(&s).unwrap()).unwrap();
    }
    #[cfg(not(feature = "getup-terminal-balance-experiment"))]
    {
        // Synthetic encoded active-state fixture: this build must decode it
        // intact so compatibility checks can refuse it instead of losing it.
        c.terminal_balance = Some(TerminalBalance {
            activated_native_substep: s.substeps_completed,
            reference: supported_standing::StandingReference {
                com: s.center_of_mass(), forward: Vector::Z, lateral: Vector::X,
            },
            last_slow_controls: [0.125; 21], first_balance_controls: Some([0.25; 21]),
            first_actual_control_jump_max: Some(0.125), first_saturated_controls: Some(0),
            first_compensation_active: Some(true),
            first_actual_torque_jump_max_nm: None, first_control_transition: None,
        });
    }
    let active_json = serde_json::to_value(&c).unwrap();
    let decoded: Controller = serde_json::from_value(active_json.clone()).unwrap();
    assert_eq!(serde_json::to_value(&decoded).unwrap(), active_json);
    assert_eq!(decoded.checkpoint_variant_supported(), cfg!(feature = "getup-terminal-balance-experiment"));
    assert_eq!(decoded.controller_label(), "experimental-recovery-joint-balance-hold");

    // Assemble an otherwise valid protocol checkpoint with the real solved
    // world and the explicitly synthetic lifecycle state used above.
    let mut profile = s.profile.clone();
    profile.getup_policy_enabled = true;
    let runtime = crate::runtime::Runtime::new(lh_contracts::Mode::Protocol, 0.0, profile).unwrap();
    let mut auxiliary = serde_json::to_value(runtime).unwrap();
    auxiliary["simulation"] = serde_json::to_value(&s).unwrap();
    auxiliary["simulation"]["profile"]["getup_policy_enabled"] = serde_json::json!(true);
    auxiliary["simulation"]["getup_controller"] = active_json.clone();
    let world = bincode::DefaultOptions::new().with_fixint_encoding().serialize(&s.world).unwrap();
    let checkpoint = bincode::DefaultOptions::new().with_fixint_encoding()
        .serialize(&(1_u32, lh_model::CANONICAL_JSON, world.clone(), auxiliary.to_string())).unwrap();
    let restored = crate::runtime::Runtime::from_getup_checkpoint(&checkpoint);
    #[cfg(feature = "getup-terminal-balance-experiment")]
    {
        let restored = restored.unwrap();
        let restored_controller: Controller = serde_json::from_str(&restored.getup_report().unwrap()).unwrap();
        assert_eq!(serde_json::to_value(restored_controller).unwrap(), active_json);
        assert_eq!(world_words(restored.simulation()), world_words(&s));
        let second = crate::runtime::Runtime::from_getup_checkpoint(&restored.getup_checkpoint().unwrap()).unwrap();
        assert_eq!(second.getup_report().unwrap(), restored.getup_report().unwrap());
        assert_eq!(second.simulation().substeps_completed, s.substeps_completed);
    }
    #[cfg(not(feature = "getup-terminal-balance-experiment"))]
    assert_eq!(restored.err().unwrap(), "Checkpoint requires the experimental terminal-balance controller build");

    auxiliary["simulation"]["getup_controller"] = ready_json.clone();
    let checkpoint = bincode::DefaultOptions::new().with_fixint_encoding()
        .serialize(&(1_u32, lh_model::CANONICAL_JSON, world, auxiliary.to_string())).unwrap();
    let restored = crate::runtime::Runtime::from_getup_checkpoint(&checkpoint);
    #[cfg(feature = "getup-terminal-balance-experiment")]
    {
        let restored_controller: Controller = serde_json::from_str(&restored.unwrap().getup_report().unwrap()).unwrap();
        assert_eq!(serde_json::to_value(restored_controller).unwrap(), ready_json);
    }
    #[cfg(not(feature = "getup-terminal-balance-experiment"))]
    assert_eq!(restored.err().unwrap(), "Checkpoint requires the experimental terminal-balance controller build");
}
