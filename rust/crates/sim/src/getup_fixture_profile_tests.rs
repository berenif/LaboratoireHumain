//! Construction-only coverage: no real or forecast integration is requested.
use super::*;
use serde_json::Value;

const KINDS: [&str; 4] = ["back", "belly", "side", "side-supported"];
const THREE_PLUS_ONE_PROFILE: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../../docs/evidence/profiles/getup-800hz-3plus1-recovery-qualification-v1.json"
));

fn enabled_default() -> Profile {
    Profile {
        getup_policy_enabled: true,
        ..Profile::default()
    }
}

fn three_plus_one() -> Profile {
    serde_json::from_str(THREE_PLUS_ONE_PROFILE).expect("complete 3+1 qualification profile")
}

// Always take the same f32 -> shortest decimal -> parsed JSON path on both
// sides. Comparing parsed source JSON with to_value(f32) instead promotes the
// latter to f64 and can report different numbers for identical typed profiles.
fn json<T: Serialize>(value: &T) -> Value {
    serde_json::from_str(&serde_json::to_string(value).unwrap()).unwrap()
}

fn world_bytes(runtime: &Runtime) -> Vec<u8> {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(64 * 1024 * 1024)
        .serialize(&runtime.simulation.world)
        .unwrap()
}

fn assert_unstepped(runtime: &Runtime, kind: &str) {
    let simulation = &runtime.simulation;
    assert_eq!(simulation.tick, 0);
    assert_eq!(simulation.substeps_completed, 0);
    assert_eq!(runtime.timeline.tick, 0);
    assert_eq!(runtime.timeline.depth(), 0);
    assert_eq!(runtime.observation_sequence, 0);
    assert!(simulation.first_failure.is_none());
    let fixture = simulation.getup_fixture.as_ref().unwrap();
    assert_eq!(fixture.kind, kind);
    assert!(fixture.preparing);
    assert!(!fixture.ready);
    assert_eq!(fixture.settled_substeps, 0);
    assert_eq!(fixture.trigger_substep, None);
    let controller = simulation.getup_controller.as_ref().unwrap();
    assert_eq!(controller.started_substep, None);
    assert_eq!(controller.status.forecast_substeps, 0);
    assert_eq!(controller.status.teacher_decisions, 0);
    assert_eq!(controller.status.reference_frames, 0);
    assert_eq!(controller.status.policy_decisions, 0);
    let snapshot = simulation.snapshot();
    assert_eq!(snapshot.simulation_time_s, 0.0);
    assert_eq!(snapshot.contact_time_s, 0.0);
    assert!(snapshot.contacts.is_empty());
    assert_eq!(snapshot.diagnostics.delivered_motor_impulse_nm_s, None);
    assert!(snapshot.diagnostics.delivered_motors.is_none());
    assert!(
        snapshot
            .diagnostics
            .getup
            .as_ref()
            .unwrap()
            .motor_controls
            .iter()
            .all(|v| *v == 0.0)
    );
    let report: Value = serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    assert_eq!(report["nativeSubsteps"], 0);
}

fn assert_unchanged_articulation(runtime: &Runtime) {
    let simulation = &runtime.simulation;
    assert_eq!(simulation.bodies.len(), 25);
    assert_eq!(simulation.model.segments.len(), 25);
    assert!(
        simulation
            .bodies
            .iter()
            .all(|h| simulation.world.bodies[*h].is_dynamic())
    );
    let root = simulation
        .world
        .multibody_joints
        .rigid_body_link(simulation.bodies[0])
        .unwrap();
    let articulation = simulation
        .world
        .multibody_joints
        .get_multibody(root.multibody)
        .unwrap();
    assert_eq!(articulation.ndofs(), 27);
    assert_eq!(articulation.links().count(), 25);
    assert_eq!(
        simulation
            .model
            .segments
            .iter()
            .filter(|part| {
                part.reference
                    .as_ref()
                    .and_then(|r| r.hinge.as_ref())
                    .is_some()
            })
            .count(),
        21
    );
    assert_eq!(simulation.world.impulse_joints.len(), 0);
}

#[test]
fn default_fixture_wrapper_matches_explicit_enabled_default_exactly_at_native_zero() {
    for kind in KINDS {
        let wrapped = Runtime::new_getup_fixture(kind).unwrap();
        let explicit = Runtime::new_getup_fixture_with_profile(kind, enabled_default()).unwrap();
        assert_unstepped(&wrapped, kind);
        assert_unstepped(&explicit, kind);
        assert_unchanged_articulation(&explicit);
        assert_eq!(
            json(&wrapped.simulation.profile),
            json(&explicit.simulation.profile),
            "{kind}"
        );
        assert_eq!(
            json(&wrapped.simulation.snapshot()),
            json(&explicit.simulation.snapshot()),
            "{kind}"
        );
        assert_eq!(
            crate::getup::joint_state(&wrapped.simulation).unwrap(),
            crate::getup::joint_state(&explicit.simulation).unwrap(),
            "{kind}"
        );
        assert_eq!(
            wrapped.getup_fixture_report().unwrap(),
            explicit.getup_fixture_report().unwrap(),
            "{kind}"
        );
        assert_eq!(
            wrapped.getup_report().unwrap(),
            explicit.getup_report().unwrap(),
            "{kind}"
        );
        assert_eq!(world_bytes(&wrapped), world_bytes(&explicit), "{kind}");
        assert_unstepped(&wrapped, kind);
        assert_unstepped(&explicit, kind);
    }
}

#[test]
fn complete_three_plus_one_profile_changes_only_id_and_solver_budgets() {
    let profile = three_plus_one();
    profile.validate().unwrap();
    assert!(profile.getup_policy_enabled);
    assert_eq!(profile.id, "getup-800hz-3plus1-recovery-qualification-v1");
    assert_eq!(profile.solver_iterations, 3);
    assert_eq!(profile.additional_body_iterations, 1);
    let expected = json(&enabled_default());
    let mut actual = json(&profile);
    let source: Value = serde_json::from_str(THREE_PLUS_ONE_PROFILE).unwrap();
    assert_eq!(
        source.as_object().unwrap().keys().collect::<Vec<_>>(),
        actual.as_object().unwrap().keys().collect::<Vec<_>>(),
        "the file must contain every serialized profile field"
    );
    let differences = actual
        .as_object()
        .unwrap()
        .iter()
        .filter_map(|(key, value)| (expected.get(key) != Some(value)).then_some(key.as_str()))
        .collect::<Vec<_>>();
    assert_eq!(
        differences,
        ["additional_body_iterations", "id", "solver_iterations"]
    );
    for key in ["id", "solver_iterations", "additional_body_iterations"] {
        actual[key] = expected[key].clone();
    }
    assert_eq!(
        actual, expected,
        "all timing, physical and controller criteria stay unchanged"
    );
}

#[test]
fn explicit_three_plus_one_is_applied_before_all_fixture_initial_poses_without_integration() {
    for kind in KINDS {
        let profile = three_plus_one();
        let runtime = Runtime::new_getup_fixture_with_profile(kind, profile.clone()).unwrap();
        let default = Runtime::new_getup_fixture(kind).unwrap();
        let simulation = &runtime.simulation;
        assert_eq!(json(&simulation.profile), json(&profile), "{kind}");
        assert_eq!(simulation.profile.dt_s, 1.0 / 800.0);
        assert_eq!(simulation.profile.tick_substeps, 20);
        assert_eq!(simulation.world.integration_parameters.dt, 1.0 / 800.0);
        assert_eq!(
            simulation
                .world
                .integration_parameters
                .num_solver_iterations,
            3
        );
        assert_eq!(
            simulation
                .world
                .integration_parameters
                .num_internal_pgs_iterations,
            20
        );
        assert_eq!(simulation.profile.fall_internal_pgs_iterations, 32);
        for handle in &simulation.bodies {
            assert_eq!(
                simulation.world.bodies[*handle].additional_solver_iterations(),
                1,
                "{kind}: {handle:?}"
            );
        }
        assert_unchanged_articulation(&runtime);
        assert_unstepped(&runtime, kind);
        assert_eq!(
            json(&simulation.snapshot()),
            json(&default.simulation.snapshot()),
            "{kind}: physical initial state"
        );
        assert_eq!(
            crate::getup::joint_state(simulation).unwrap(),
            crate::getup::joint_state(&default.simulation).unwrap(),
            "{kind}: initial generalized state"
        );
        assert_eq!(
            runtime.getup_fixture_report().unwrap(),
            default.getup_fixture_report().unwrap(),
            "{kind}"
        );
        assert_eq!(
            runtime.getup_report().unwrap(),
            default.getup_report().unwrap(),
            "{kind}"
        );
        assert_unstepped(&runtime, kind);
        assert_unstepped(&default, kind);
    }
}

#[test]
fn fixture_profile_constructor_rejects_disabled_policy_invalid_timing_and_budgets_and_unknown_kind()
{
    for kind in KINDS {
        assert_eq!(
            Runtime::new_getup_fixture_with_profile(kind, Profile::default())
                .err()
                .unwrap(),
            "GetUp fixture profile must explicitly enable the policy"
        );
        let enabled = enabled_default();
        let invalid = [
            Profile {
                dt_s: 0.0,
                ..enabled.clone()
            },
            Profile {
                dt_s: 1.0 / 400.0,
                ..enabled.clone()
            },
            Profile {
                dt_s: f32::NAN,
                ..enabled.clone()
            },
            Profile {
                tick_substeps: 0,
                ..enabled.clone()
            },
            Profile {
                tick_substeps: 10,
                ..enabled.clone()
            },
            Profile {
                solver_iterations: 0,
                ..enabled.clone()
            },
            Profile {
                internal_pgs_iterations: 0,
                ..enabled.clone()
            },
            Profile {
                fall_internal_pgs_iterations: 0,
                ..enabled.clone()
            },
            Profile {
                max_ccd_substeps: 0,
                ..enabled.clone()
            },
            Profile {
                aggregate_motor_cap_nm: 0.0,
                ..enabled.clone()
            },
        ];
        for profile in invalid {
            assert_eq!(
                Runtime::new_getup_fixture_with_profile(kind, profile)
                    .err()
                    .unwrap(),
                "Invalid physics profile",
                "{kind}"
            );
        }
    }
    for kind in ["", "unknown", "SIDE", "side_supported"] {
        assert_eq!(
            Runtime::new_getup_fixture_with_profile(kind, enabled_default())
                .err()
                .unwrap(),
            "Unknown fixed GetUp fixture"
        );
        assert_eq!(
            Runtime::new_getup_fixture(kind).err().unwrap(),
            "Unknown fixed GetUp fixture"
        );
    }
}
