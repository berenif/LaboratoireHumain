//! Repeated-cycle lifecycle contracts, never physical recovery qualification.
//! The shared constructor records 24 ticks of ordinary upright standing only.
//! All completion flags, elapsed clocks, counters, loads, and observations below
//! are explicitly synthetic; the contract checks themselves never integrate.
use super::*;

fn explicit_profile() -> Profile {
    Profile {
        getup_policy_enabled: true,
        getup_repeated_recovery: Some(GetUpRepeatedRecovery::CompletedCycleConfirmedFallV1),
        ..Profile::default()
    }
}

#[test]
fn repeated_profile_none_is_byte_omitted_and_future_semantics_are_refused() {
    let legacy = serde_json::to_value(Profile::default()).unwrap();
    assert!(legacy.get("getup_repeated_recovery").is_none());
    let decoded: Profile = serde_json::from_value(legacy.clone()).unwrap();
    assert_eq!(decoded.getup_repeated_recovery, None);
    assert_eq!(serde_json::to_value(decoded).unwrap(), legacy);

    let active = serde_json::to_value(explicit_profile()).unwrap();
    assert_eq!(active["getup_repeated_recovery"], "completed-cycle-confirmed-fall-v1");
    let decoded: Profile = serde_json::from_value(active.clone()).unwrap();
    assert_eq!(decoded.getup_repeated_recovery, explicit_profile().getup_repeated_recovery);
    let mut future = active;
    future["getup_repeated_recovery"] = "future-unqualified-cycle-v2".into();
    assert!(serde_json::from_value::<Profile>(future).is_err());
}

#[cfg(not(feature = "getup-repeat-cycle-experiment"))]
#[test]
fn feature_off_profile_and_factories_refuse_before_constructing_a_world() {
    let expected = "Repeated GetUp recovery requires getup-repeat-cycle-experiment";
    assert_eq!(explicit_profile().validate().unwrap_err(), expected);
    assert_eq!(Simulation::new(0.0, explicit_profile(), true).err().unwrap(), expected);
    assert_eq!(Simulation::new_protocol(0.0, explicit_profile(), true).err().unwrap(), expected);
    assert_eq!(crate::runtime::Runtime::new(lh_contracts::Mode::Protocol, 0.0,
        explicit_profile()).err().unwrap(), expected);
    assert_eq!(crate::runtime::Runtime::new_getup_fixture_with_profile("belly",
        explicit_profile()).err().unwrap(), expected);
}

#[cfg(feature = "getup-repeat-cycle-experiment")]
#[test]
fn repeated_profile_requires_policy_and_excludes_early_and_bumpless_transfer() {
    let expected = "Repeated GetUp recovery requires the source/terminal-balance controller without early or bumpless transfer";
    let mut disabled = explicit_profile();
    disabled.getup_policy_enabled = false;
    assert_eq!(disabled.validate().unwrap_err(), expected);
    #[cfg(feature = "getup-terminal-base-ready-experiment")]
    {
        assert_eq!(explicit_profile().validate().unwrap_err(), expected);
        assert_eq!(crate::runtime::Runtime::new_getup_fixture_with_profile("belly",
            explicit_profile()).err().unwrap(), expected);
    }
    #[cfg(not(feature = "getup-terminal-base-ready-experiment"))]
    explicit_profile().validate().unwrap();
}

#[cfg(all(feature = "getup-repeat-cycle-experiment",
    not(feature = "getup-terminal-base-ready-experiment")))]
mod active {
    use super::*;
    use super::super::terminal_balance_tests::{settled_geometry, terminal_controller, world_words};
    use crate::getup::physical_observation;
    use bincode::Options;

    fn scene() -> Simulation {
        let mut s = settled_geometry();
        // The settled world is ordinary upright geometry, not a recovery result.
        s.profile = explicit_profile();
        s.substeps_completed = 2_000;
        s.counters.falls = 3;
        s.counters.recoveries = 2;
        s
    }

    fn completed(s: &Simulation) -> Controller {
        let mut c = terminal_controller(s);
        c.started_substep = Some(100);
        c.impact_substep = Some(100);
        c.stable_substeps = 900;
        c.bind_terminal_balance(s);
        assert!(c.terminal_balance.is_some());
        c.status.prototype_standing_passed = true;
        c.status.phase = "experimental-joint-balance-holding".into();
        c
    }

    fn timers(s: &Simulation) -> serde_json::Value {
        serde_json::to_value(&s.motion_observation).unwrap()
    }

    fn stale_timers(s: &mut Simulation) {
        s.motion_observation = serde_json::from_value(serde_json::json!({
            "unsupported_s": 4.0, "settled_s": 3.0,
        })).unwrap();
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    fn seed_nonzero_carry(s: &mut Simulation) {
        let link = *s.world.multibody_joints.rigid_body_link(s.bodies[0]).unwrap();
        let multibody = s.world.multibody_joints.get_multibody_mut(link.multibody).unwrap();
        multibody.set_root_translation_compensation_enabled(true).unwrap();
        let mut state = multibody.root_translation_integrator_state().unwrap();
        let axis = state.qbits.iter().position(|bits| f32::from_bits(*bits).abs() >= 0.001).unwrap();
        let q = f32::from_bits(state.qbits[axis]).abs();
        let ulp = f64::from(f32::from_bits(q.to_bits() + 1)) - f64::from(q);
        state.remainderbits[axis] = (ulp * 0.25).to_bits();
        multibody.restore_root_translation_integrator_state(state).unwrap();
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    fn carries(s: &Simulation) -> Vec<([u32; 2], Option<rapier3d::dynamics::RootTranslationIntegratorState>)> {
        s.world.multibody_joints.multibodies().map(|m| {
            let (index, generation) = m.root().rigid_body_handle().into_raw_parts();
            ([index, generation], m.root_translation_integrator_state())
        }).collect()
    }

    #[test]
    fn completed_contacts_record_pending_epoch_without_world_writes_or_counting_a_fall() {
        for motion in [Motion::Upright, Motion::Reacting] {
            let mut s = scene();
            s.motion = motion;
            s.getup_controller = Some(completed(&s));
            stale_timers(&mut s);
            let before_world = world_words(&s);
            let mut expected = serde_json::to_value(&s).unwrap();
            expected["motion"] = serde_json::to_value(Motion::Reacting).unwrap();
            expected["getup_controller"]["impact_substep"] = s.substeps_completed.into();
            s.observe_impact();
            assert_eq!(serde_json::to_value(&s).unwrap(), expected);
            assert_eq!(world_words(&s), before_world);
            assert_eq!(s.counters.falls, 3);
            assert_eq!(s.counters.recoveries, 2);
            assert!(s.getup_controller.as_ref().unwrap().completed_recovery());
        }
    }

    #[test]
    fn confirmed_falling_and_fallen_rearm_controller_once_and_preserve_all_simulation_state() {
        for motion in [Motion::Falling, Motion::Fallen] {
            for old_epoch in [None, Some(90), Some(100), Some(1_500)] {
                let mut s = scene();
                s.motion = motion;
                stale_timers(&mut s);
                #[cfg(feature = "root-translation-carry-experiment")]
                seed_nonzero_carry(&mut s);
                let mut c = completed(&s);
                c.impact_substep = old_epoch;
                c.teacher_initial = Some(s.snapshot());
                c.teacher_native_dt_s = Some(s.profile.dt_s);
                c.teacher_frames.push(TeacherFrame {
                    time_s: 0.025, observation: vec![0.25; 67], action: [0.25; 21],
                    target: standing_target(&s.model), snapshot: s.snapshot(), before: s.snapshot().segments,
                });
                c.student_decisions.push(StudentDecision {
                    native_substep: 120, observation: vec![0.5; 67], residual: [0.5; 21],
                    controls: [0.5; 21], holding: false, before: s.snapshot().segments,
                });
                c.decision_observation = vec![0.25; 67];
                c.residual = [0.25; 21];
                c.reference_pose = [0.5; 21];
                c.holding = false;
                c.status.policy_decisions = 17;
                c.status.teacher_decisions = 5;
                c.status.forecast_substeps = 100;
                c.stable_feet = Some([Vec3 { x: 1.0, y: 2.0, z: 3.0 }; 2]);
                let before_world = world_words(&s);
                let before_simulation = serde_json::to_value(&s).unwrap();
                #[cfg(feature = "root-translation-carry-experiment")]
                let before_carries = carries(&s);
                let loads = s.integrated_support_loads_n;
                c.observe(&mut s, &loads);
                let mut expected = Controller::new(&s.model);
                expected.impact_substep = Some(old_epoch.filter(|epoch| *epoch > 100)
                    .unwrap_or(s.substeps_completed));
                assert_eq!(serde_json::to_value(&c).unwrap(), serde_json::to_value(&expected).unwrap(),
                    "motion={motion:?}, prior epoch={old_epoch:?}");
                assert!(!c.status.prototype_standing_passed);
                assert!(c.reference.is_empty() && c.teacher_frames.is_empty() && c.student_decisions.is_empty());
                assert!(c.terminal_balance.is_none() && c.stable_feet.is_none());
                assert_eq!(c.stable_substeps, 0);
                assert_eq!(serde_json::to_value(&s).unwrap(), before_simulation,
                    "including handles, counters, clocks, targets, and observer memory");
                assert_eq!(world_words(&s), before_world);
                #[cfg(feature = "root-translation-carry-experiment")]
                assert_eq!(carries(&s), before_carries, "Rapier serde skips the active root sidecar");
                // A second observer call while awaiting source entry cannot
                // move the selected epoch or reset this new controller again.
                c.observe(&mut s, &loads);
                assert_eq!(serde_json::to_value(&c).unwrap(), serde_json::to_value(expected).unwrap());
                assert_eq!(serde_json::to_value(&s).unwrap(), before_simulation);
                assert_eq!(world_words(&s), before_world);
            }
        }
    }

    #[test]
    fn resisted_pending_contact_requires_both_800_sample_and_800_native_age_bounds() {
        for (native_age, stable_before, clears) in [
            (799, 799, false), (800, 798, false), (800, 799, true),
            (801, 799, true), (799, 900, false), (800, 900, true),
        ] {
            let mut s = scene();
            s.motion = Motion::Reacting;
            s.substeps_completed = 1_000 + native_age;
            let mut c = completed(&s);
            c.impact_substep = Some(1_000);
            c.stable_substeps = stable_before;
            let binding = serde_json::to_value(&c.terminal_balance).unwrap();
            let before_world = world_words(&s);
            let before_counters = serde_json::to_value(&s.counters).unwrap();
            let loads = s.integrated_support_loads_n;
            c.observe(&mut s, &loads);
            assert_eq!(c.stable_substeps, stable_before + 1);
            assert_eq!(c.impact_substep, if clears { None } else { Some(1_000) },
                "native age={native_age}, stable before={stable_before}");
            assert_eq!(serde_json::to_value(&c.terminal_balance).unwrap(), binding,
                "resisting contact never releases or recaptures the standing binding");
            assert!(c.status.prototype_standing_passed);
            assert_eq!(serde_json::to_value(&s.counters).unwrap(), before_counters);
            assert_eq!(world_words(&s), before_world);
        }
    }

    #[test]
    fn synthetic_799_to_800_latch_clears_stale_timers_and_resumes_existing_fall_observer() {
        let mut s = scene();
        s.motion = Motion::Recovering;
        stale_timers(&mut s);
        let stale = timers(&s);
        let mut c = completed(&s);
        c.status.prototype_standing_passed = false;
        c.stable_substeps = 798;
        let loads = s.integrated_support_loads_n;
        let before_world = world_words(&s);
        c.observe(&mut s, &loads);
        assert_eq!(c.stable_substeps, 799);
        assert!(!c.status.prototype_standing_passed);
        assert_eq!(timers(&s), stale);
        assert_eq!(s.counters.recoveries, 2);
        c.observe(&mut s, &loads);
        assert_eq!(c.stable_substeps, 800);
        assert!(c.status.prototype_standing_passed);
        assert_eq!(s.counters.recoveries, 3);
        assert_eq!(timers(&s), serde_json::json!({ "unsupported_s": 0.0, "settled_s": 0.0 }));
        assert_eq!(s.motion, Motion::Upright);
        s.getup_controller = Some(c);
        // Synthetic zero-load observation: the inherited observer now runs,
        // but one native interval must not inherit the old unsupported timer.
        s.observe_motion(&[0.0; 25]);
        assert_eq!(timers(&s)["unsupported_s"], serde_json::json!(s.profile.dt_s));
        assert_eq!(s.motion, Motion::Upright);
        assert_eq!(s.counters.falls, 3);
        for _ in 0..120 {
            s.observe_motion(&[0.0; 25]);
        }
        assert_eq!(s.motion, Motion::Falling);
        assert_eq!(s.counters.falls, 4, "the original persistent support-loss gate counts once");
        assert_eq!(s.counters.recoveries, 3);
        assert_eq!(world_words(&s), before_world);
    }

    #[test]
    fn future_start_impact_and_activation_refuse_commands_and_checkpoint_before_integration() {
        let s = scene();
        let runtime = crate::runtime::Runtime::new(lh_contracts::Mode::Protocol, 0.0,
            explicit_profile()).unwrap();
        let auxiliary = serde_json::to_value(runtime).unwrap();
        let physical = physical_observation(&s).unwrap();
        for clock in ["source start", "impact", "terminal activation"] {
            let mut c = completed(&s);
            match clock {
                "source start" => c.started_substep = Some(s.substeps_completed + 1),
                "impact" => c.impact_substep = Some(s.substeps_completed + 1),
                "terminal activation" => c.terminal_balance.as_mut().unwrap().activated_native_substep = s.substeps_completed + 1,
                _ => unreachable!(),
            }
            assert!(!c.repeated_recovery_clock_valid(s.substeps_completed));
            let before_controller = serde_json::to_value(&c).unwrap();
            let before_simulation = serde_json::to_value(&s).unwrap();
            let before_world = world_words(&s);
            assert_eq!(c.command(&s, &physical).unwrap_err(),
                "Invalid repeated GetUp recovery epoch or native clock", "{clock}");
            assert_eq!(serde_json::to_value(&c).unwrap(), before_controller);
            assert_eq!(serde_json::to_value(&s).unwrap(), before_simulation);
            assert_eq!(world_words(&s), before_world);

            let mut checkpoint_auxiliary = auxiliary.clone();
            checkpoint_auxiliary["simulation"] = serde_json::to_value(&s).unwrap();
            checkpoint_auxiliary["simulation"]["getup_controller"] = before_controller;
            let world = bincode::DefaultOptions::new().with_fixint_encoding().serialize(&s.world).unwrap();
            let checkpoint = bincode::DefaultOptions::new().with_fixint_encoding().serialize(&(
                1_u32, lh_model::CANONICAL_JSON, world, checkpoint_auxiliary.to_string(),
            )).unwrap();
            assert_eq!(crate::runtime::Runtime::from_getup_checkpoint(&checkpoint).err().unwrap(),
                "Invalid checkpoint repeated GetUp recovery clock", "{clock}");
            assert_eq!(world_words(&s), before_world);
        }
        // Boundary equality is valid for all three clocks.
        let mut c = completed(&s);
        c.started_substep = Some(s.substeps_completed);
        c.impact_substep = Some(s.substeps_completed);
        assert!(c.repeated_recovery_clock_valid(s.substeps_completed));
        assert_eq!(s.substeps_completed, 2_000);
    }

    #[test]
    fn active_source_cannot_smuggle_a_later_impact_epoch_into_a_checkpoint_deadline() {
        let s = scene();
        let physical = physical_observation(&s).unwrap();
        for epoch in [None, Some(101), Some(s.substeps_completed)] {
            let mut c = completed(&s);
            c.status.prototype_standing_passed = false;
            c.impact_substep = epoch;
            assert!(!c.repeated_recovery_clock_valid(s.substeps_completed));
            let before = serde_json::to_value(&c).unwrap();
            assert_eq!(c.command(&s, &physical).unwrap_err(),
                "Invalid repeated GetUp recovery epoch or native clock");
            assert_eq!(serde_json::to_value(&c).unwrap(), before);
        }
        let mut c = completed(&s);
        c.status.prototype_standing_passed = false;
        c.impact_substep = Some(100);
        assert!(c.repeated_recovery_clock_valid(s.substeps_completed));
    }

    #[test]
    fn unstruck_source_entry_sets_epoch_and_mid_source_contacts_cannot_extend_deadline() {
        let mut s = scene();
        s.motion = Motion::Fallen;
        // Deliberately reject teacher preflight before any shadow integration.
        // This checks source-entry bookkeeping, not reference success.
        s.world.colliders[s.floor].set_enabled(false);
        let physical = physical_observation(&s).unwrap();
        let mut c = Controller::new(&s.model);
        let before_world = world_words(&s);
        assert_eq!(c.command(&s, &physical).unwrap_err(),
            "GetUp prototype requires a free body on the protocol floor");
        assert_eq!(c.started_substep, Some(2_000));
        assert_eq!(c.impact_substep, Some(2_000));
        assert_eq!(c.status.forecast_substeps, 0);
        assert!(!c.completed_cycle_impact(2_100));
        c.impact(2_100);
        c.impact(2_200);
        assert_eq!(c.impact_substep, Some(2_000));
        s.substeps_completed = 22_000;
        assert_eq!(c.command(&s, &physical).unwrap_err(),
            "25-second physical recovery deadline reached");
        assert_eq!(c.impact_substep, Some(2_000));
        assert_eq!(c.status.forecast_substeps, 0);
        assert_eq!(world_words(&s), before_world);
    }

    #[test]
    fn running_source_command_and_later_contact_keep_the_first_cycle_epoch() {
        let mut s = scene();
        s.motion = Motion::Recovering;
        s.substeps_completed = 120;
        let mut c = terminal_controller(&s);
        c.started_substep = Some(100);
        c.impact_substep = Some(100);
        c.last_decision = Some(1);
        // An explicitly synthetic already-sampled source command. Holding the
        // exact last index avoids invoking either actor or a teacher branch.
        assert!(c.terminal_balance.is_none());
        assert!(!c.status.prototype_standing_passed);
        let physical = physical_observation(&s).unwrap();
        let expected = crate::getup::slow_controls(&s.model, &c.reference_pose, &c.residual,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(),
            c.holding).unwrap();
        let before_world = world_words(&s);
        assert_eq!(c.command(&s, &physical).unwrap().map(f32::to_bits), expected.map(f32::to_bits));
        assert_eq!(c.status.policy_decisions, 0);
        assert_eq!(c.status.forecast_substeps, 0);
        assert!(!c.completed_cycle_impact(120));
        s.getup_controller = Some(c);
        s.observe_impact();
        let mut c = s.getup_controller.take().unwrap();
        assert_eq!(c.started_substep, Some(100));
        assert_eq!(c.impact_substep, Some(100));
        s.substeps_completed = 20_100;
        assert_eq!(c.command(&s, &physical).unwrap_err(),
            "25-second physical recovery deadline reached");
        assert_eq!(c.impact_substep, Some(100));
        assert_eq!(world_words(&s), before_world);
    }

    #[test]
    fn teacher_fork_disables_lifecycle_only_and_preserves_exact_world_without_integration() {
        let mut s = scene();
        s.profile.getup_solver_budget = Some(GetUpSolverBudget::ConsistentRecoveryPhaseV1);
        #[cfg(feature = "root-translation-carry-experiment")]
        seed_nonzero_carry(&mut s);
        let before_simulation = serde_json::to_value(&s).unwrap();
        let before_world = world_words(&s);
        let shadow = fork(&s).unwrap();
        assert!(!shadow.profile.getup_policy_enabled);
        assert!(shadow.profile.getup_repeated_recovery.is_none());
        assert!(shadow.getup_controller.is_none());
        let mut expected_profile = s.profile.clone();
        expected_profile.getup_policy_enabled = false;
        expected_profile.getup_repeated_recovery = None;
        assert_eq!(serde_json::to_value(&shadow.profile).unwrap(), serde_json::to_value(expected_profile).unwrap());
        assert_eq!(world_words(&shadow), before_world);
        assert_eq!(crate::getup::joint_state(&shadow).unwrap(), crate::getup::joint_state(&s).unwrap());
        #[cfg(feature = "root-translation-carry-experiment")]
        assert_eq!(carries(&shadow), carries(&s));
        assert_eq!(serde_json::to_value(&s).unwrap(), before_simulation);
        assert_eq!(world_words(&s), before_world);
    }

    #[test]
    fn profile_none_retains_legacy_impact_and_does_not_release_completed_controller() {
        let mut s = scene();
        s.profile.getup_repeated_recovery = None;
        s.motion = Motion::Upright;
        s.getup_controller = Some(completed(&s));
        let before_world = world_words(&s);
        s.observe_impact();
        assert_eq!(s.motion, Motion::Falling);
        assert_eq!(s.counters.falls, 4);
        let mut c = s.getup_controller.take().unwrap();
        assert_eq!(c.impact_substep, Some(100), "legacy impact keeps the first epoch");
        let binding = serde_json::to_value(&c.terminal_balance).unwrap();
        let loads = s.integrated_support_loads_n;
        c.observe(&mut s, &loads);
        assert_eq!(c.started_substep, Some(100));
        assert!(c.status.prototype_standing_passed);
        assert_eq!(serde_json::to_value(c.terminal_balance).unwrap(), binding);
        assert_eq!(s.counters.recoveries, 2);
        assert_eq!(world_words(&s), before_world);
    }
}
