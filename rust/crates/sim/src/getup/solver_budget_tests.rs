//! Numerical phase selection only; these tests perform no integration.
use super::*;

fn explicit() -> Profile {
    Profile {
        getup_policy_enabled: true,
        getup_solver_budget: Some(GetUpSolverBudget::ConsistentRecoveryPhaseV1),
        ..Profile::default()
    }
}

#[test]
fn solver_budget_profile_is_explicit_and_future_variants_are_rejected() {
    let legacy = serde_json::to_value(Profile::default()).unwrap();
    assert!(legacy.get("getup_solver_budget").is_none());
    let decoded: Profile = serde_json::from_value(legacy.clone()).unwrap();
    assert_eq!(decoded.getup_solver_budget, None);
    assert_eq!(serde_json::to_value(decoded).unwrap(), legacy);
    let mut future = serde_json::to_value(explicit()).unwrap();
    future["getup_solver_budget"] = "future-unqualified-budget".into();
    assert!(serde_json::from_value::<Profile>(future).is_err());
    let complete: Profile = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/profiles/getup-800hz-3plus1-consistent-recovery-budget-v1.json"))).unwrap();
    complete.validate().unwrap();
    assert_eq!(complete.getup_solver_budget, explicit().getup_solver_budget);
    assert_eq!((complete.solver_iterations, complete.additional_body_iterations), (3, 1));
    assert_eq!((complete.dt_s, complete.tick_substeps), (0.00125, 20));
}

#[test]
fn solver_budget_helper_retains_normal_minimum_and_recovery_maximum() {
    let mut p = explicit();
    assert_eq!(p.internal_pgs_for_phase(false), 20);
    assert_eq!(p.internal_pgs_for_phase(true), 32);
    p.internal_pgs_iterations = 40;
    assert_eq!(p.internal_pgs_for_phase(false), 40);
    assert_eq!(p.internal_pgs_for_phase(true), 40);
}

#[test]
fn solver_budget_real_phase_distinguishes_awaiting_source_and_terminal_handoff() {
    let mut s = Simulation::new(0.0, explicit(), true).unwrap();
    assert_eq!(s.selected_internal_pgs_iterations(), 20);
    s.motion = Motion::Recovering;
    assert_eq!(s.selected_internal_pgs_iterations(), 20);
    s.getup_controller.as_mut().unwrap().started_substep = Some(0);
    assert_eq!(s.selected_internal_pgs_iterations(), 32);
    s.profile.getup_solver_budget = None;
    assert_eq!(s.selected_internal_pgs_iterations(), 20);
    s.profile.getup_solver_budget = explicit().getup_solver_budget;
    let c = s.getup_controller.as_mut().unwrap();
    c.terminal_balance = Some(TerminalBalance {
        activated_native_substep: 0,
        reference: super::super::supported_standing::StandingReference {
            com: Vector::ZERO, forward: Vector::Z, lateral: Vector::X,
        },
        last_slow_controls: [0.0; 21], first_balance_controls: None,
        first_actual_control_jump_max: None, first_actual_torque_jump_max_nm: None,
        first_saturated_controls: None, first_compensation_active: None,
        first_control_transition: None, bumpless: None,
    });
    assert!(!c.source_recovery_active());
    assert_eq!(s.motion, Motion::Recovering);
    assert_eq!(s.selected_internal_pgs_iterations(), 20);
    s.getup_controller = None;
    assert_eq!(s.selected_internal_pgs_iterations(), 20);
    for motion in [Motion::Falling, Motion::Fallen] {
        s.motion = motion;
        assert_eq!(s.selected_internal_pgs_iterations(), 32);
    }
    assert_eq!(s.substeps_completed, 0);
}

#[test]
fn solver_budget_forecast_selects_explicit_recovery_after_exact_fork() {
    for selected in [None, explicit().getup_solver_budget] {
        let mut p = explicit(); p.getup_solver_budget = selected;
        let real = Simulation::new(0.0, p, true).unwrap();
        let before = serde_json::to_value(real.snapshot()).unwrap();
        let mut shadow = fork(&real).unwrap();
        assert_eq!(shadow.world.integration_parameters.num_internal_pgs_iterations, 20);
        assert_eq!(serde_json::to_value(&real.world).unwrap(), serde_json::to_value(&shadow.world).unwrap());
        assert!(!shadow.profile.getup_policy_enabled);
        assert!(shadow.getup_controller.is_none());
        select_forecast_solver_budget(&mut shadow);
        assert_eq!(shadow.world.integration_parameters.num_internal_pgs_iterations,
            if selected.is_some() { 32 } else { 20 });
        assert_eq!(shadow.substeps_completed, 0);
        assert_eq!(serde_json::to_value(real.snapshot()).unwrap(), before);
        assert_eq!(serde_json::to_value(shadow.snapshot().segments).unwrap(), before["segments"]);
        // Restoring only the selected scalar recovers the complete exact world.
        shadow.world.integration_parameters.num_internal_pgs_iterations = 20;
        assert_eq!(serde_json::to_value(&real.world).unwrap(), serde_json::to_value(&shadow.world).unwrap());
    }
}

#[cfg(feature = "unit-root-rotation-experiment")]
#[test]
fn unit_root_teacher_clone_preserves_policy_and_all_seed_bits_without_integration() {
    let mut p = explicit();
    p.root_rotation_normalization = Some(crate::RootRotationNormalization::UnitFreeRootV1);
    let real = Simulation::new(0.0, p, true).unwrap();
    let shadow = fork(&real).unwrap();
    assert!(!shadow.profile.getup_policy_enabled);
    assert!(shadow.getup_controller.is_none());
    assert_eq!(shadow.profile.root_rotation_normalization, real.profile.root_rotation_normalization);
    shadow.validate_unit_root_rotation_policy().unwrap();
    assert_eq!(serde_json::to_value(&real.world).unwrap(), serde_json::to_value(&shadow.world).unwrap());
    assert_eq!(serde_json::to_value(real.snapshot().segments).unwrap(), serde_json::to_value(shadow.snapshot().segments).unwrap());
    assert_eq!(real.substeps_completed, 0); assert_eq!(shadow.substeps_completed, 0);
}
