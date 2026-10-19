//! Serialization and direct-command continuity only. All bindings, counters,
//! clocks, and references below are synthetic. These tests never integrate a
//! world and make no physical-recovery or terminal-admission claim.
use super::*;
use serde_json::{Value, json};

type CheckpointParts = (u32, String, Vec<u8>, String);
const V1: &str = "terminal-base-ready-v1";
const V2: &str = "terminal-base-ready-bumpless-v2";
const BINDING_SUBSTEP: u64 = 1_000;

fn parts(bytes: &[u8]) -> CheckpointParts {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(GETUP_CHECKPOINT_LIMIT)
        .reject_trailing_bytes()
        .deserialize(bytes)
        .unwrap()
}

fn packed(parts: &CheckpointParts) -> Vec<u8> {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(GETUP_CHECKPOINT_LIMIT)
        .serialize(parts)
        .unwrap()
}

fn world_bytes(runtime: &Runtime) -> Vec<u8> {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(GETUP_CHECKPOINT_LIMIT)
        .serialize(&runtime.simulation.world)
        .unwrap()
}

fn edit_controller(runtime: &mut Runtime, edit: impl FnOnce(&mut Value)) {
    let mut controller =
        serde_json::to_value(runtime.simulation.getup_controller.as_ref().unwrap()).unwrap();
    edit(&mut controller);
    runtime.simulation.getup_controller = Some(serde_json::from_value(controller).unwrap());
}

fn ready(format: &str, count: u64) -> Value {
    json!({
        "format": format,
        "consecutive_substeps": count,
        "feet": if count == 0 { Value::Null } else { json!([
            lh_model::Vec3 { x: 0.125, y: 0.0, z: -0.375 },
            lh_model::Vec3 { x: -0.125, y: 0.0, z: -0.375 },
        ]) },
    })
}

fn bumpless(admission: &str) -> Value {
    json!({
        "admission": admission,
        "stable_substeps_at_binding": if admission == "full-body80" { 80 } else { 0 },
        "first_intended_controls": null,
        "first_intended_control_jump_max": null,
        "first_intended_torque_jump_max_nm": null,
        "last_blend_clamped_controls": 0,
    })
}

fn synthetic_runtime(format: Option<&str>, admission: Option<&str>) -> Runtime {
    let mut profile = Profile::default();
    profile.getup_policy_enabled = true;
    let mut runtime = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
    runtime.simulation.substeps_completed = BINDING_SUBSTEP;
    let com = runtime.simulation.center_of_mass();
    edit_controller(&mut runtime, |controller| {
        if let Some(format) = format {
            let count = if admission == Some("early-base80") || format == V1 { 80 } else { 0 };
            controller["terminal_base_ready"] = ready(format, count);
        } else {
            controller.as_object_mut().unwrap().remove("terminal_base_ready");
        }
        if let Some(admission) = admission {
            controller["started_substep"] = (BINDING_SUBSTEP - 20).into();
            controller["impact_substep"] = (BINDING_SUBSTEP - 20).into();
            controller["stable_substeps"] =
                if admission == "full-body80" { 80.into() } else { 0.into() };
            controller["status"]["phase"] = "experimental-joint-balance-holding".into();
            controller["terminal_balance"] = json!({
                "activated_native_substep": BINDING_SUBSTEP,
                "reference": {
                    "com": com,
                    "forward": rapier3d::prelude::Vector::Z,
                    "lateral": rapier3d::prelude::Vector::X,
                },
                "last_slow_controls": std::array::from_fn::<_, 21, _>(|i| (i as f32 - 10.0) / 32.0),
                "first_balance_controls": null,
                "first_actual_control_jump_max": null,
                "first_saturated_controls": null,
                "first_compensation_active": null,
            });
            if format == Some(V2) {
                controller["terminal_balance"]["bumpless"] = bumpless(admission);
            }
        }
    });
    runtime
}

fn schema_one(runtime: &Runtime) -> CheckpointParts {
    (
        1,
        lh_model::CANONICAL_JSON.into(),
        world_bytes(runtime),
        serde_json::to_string(runtime).unwrap(),
    )
}

fn schema_three(runtime: &Runtime) -> Vec<u8> {
    let mut outer = schema_one(runtime);
    outer.0 = 3;
    outer.3 = json!({
        // The outer envelope deliberately retains the original strict marker.
        "format": V1,
        "inner_schema": 1,
        "auxiliary": outer.3,
    })
    .to_string();
    packed(&outer)
}

fn mutate_schema_three_runtime(bytes: &[u8], edit: impl FnOnce(&mut Value)) -> Vec<u8> {
    let mut outer = parts(bytes);
    let mut wrapper: Value = serde_json::from_str(&outer.3).unwrap();
    let mut runtime: Value = serde_json::from_str(wrapper["auxiliary"].as_str().unwrap()).unwrap();
    edit(&mut runtime);
    wrapper["auxiliary"] = runtime.to_string().into();
    outer.3 = wrapper.to_string();
    packed(&outer)
}

#[test]
fn strict_controller_format_rejects_future_aliases_in_every_build() {
    let runtime = synthetic_runtime(Some(V2), None);
    for format in ["terminal-base-ready-v2", "terminal-base-ready-bumpless-v3", "bumpless-v2"] {
        let mut encoded = schema_one(&runtime);
        let mut auxiliary: Value = serde_json::from_str(&encoded.3).unwrap();
        auxiliary["simulation"]["getup_controller"]["terminal_base_ready"]["format"] = format.into();
        assert!(serde_json::from_value::<Runtime>(auxiliary.clone()).is_err(), "{format}");
        encoded.3 = auxiliary.to_string();
        assert!(Runtime::from_getup_checkpoint(&packed(&encoded)).is_err(), "{format}");
    }
}

#[test]
fn schema_one_never_accepts_hidden_v2_state() {
    for admission in [None, Some("full-body80"), Some("early-base80")] {
        let runtime = synthetic_runtime(Some(V2), admission);
        assert!(
            Runtime::from_getup_checkpoint(&packed(&schema_one(&runtime))).is_err(),
            "schema 1 accepted hidden V2 admission {admission:?}"
        );
    }
}

#[cfg(not(feature = "getup-terminal-bumpless-experiment"))]
#[test]
fn feature_off_refuses_v2_exports_restores_and_direct_commands() {
    for admission in [None, Some("full-body80"), Some("early-base80")] {
        let mut runtime = synthetic_runtime(Some(V2), admission);
        assert!(runtime.getup_checkpoint().is_err(), "export {admission:?}");
        assert!(Runtime::from_getup_checkpoint(&schema_three(&runtime)).is_err(), "restore {admission:?}");
        let physical = crate::getup::physical_observation(&runtime.simulation).unwrap();
        let mut controller = runtime.simulation.getup_controller.take().unwrap();
        let before = serde_json::to_string(&controller).unwrap();
        assert!(controller.command(&runtime.simulation, &physical).is_err(), "command {admission:?}");
        assert_eq!(serde_json::to_string(&controller).unwrap(), before);
        #[cfg(feature = "getup-terminal-base-ready-experiment")]
        {
            runtime.simulation.getup_controller = Some(controller);
            assert_eq!(
                runtime.getup_checkpoint().err().unwrap(),
                "Checkpoint requires getup-terminal-bumpless-experiment"
            );
        }
    }
}

#[test]
fn legacy_bindings_omit_bumpless_and_refuse_v2_only_state() {
    for format in [None, Some(V1)] {
        let mut runtime = synthetic_runtime(format, Some("full-body80"));
        let original = serde_json::to_string(runtime.simulation.getup_controller.as_ref().unwrap()).unwrap();
        assert!(!original.contains("\"bumpless\""));
        let binding = original.split_once("\"terminal_balance\":").unwrap().1;
        let old_fields = [
            "activated_native_substep", "reference", "last_slow_controls",
            "first_balance_controls", "first_actual_control_jump_max",
            "first_saturated_controls", "first_compensation_active",
        ];
        let positions: Vec<_> = old_fields.iter().map(|name| binding.find(&format!("\"{name}\":")).unwrap()).collect();
        assert!(positions.windows(2).all(|pair| pair[0] < pair[1]));
        let mut present_null = schema_one(&runtime);
        let mut auxiliary: Value = serde_json::from_str(&present_null.3).unwrap();
        auxiliary["simulation"]["getup_controller"]["terminal_balance"]["bumpless"] = Value::Null;
        assert!(
            serde_json::from_value::<Runtime>(auxiliary.clone()).is_err(),
            "legacy binding accepted explicitly present bumpless:null, format={format:?}"
        );
        present_null.3 = auxiliary.to_string();
        assert!(Runtime::from_getup_checkpoint(&packed(&present_null)).is_err());
        if format.is_some() {
            present_null.0 = 3;
            present_null.3 = json!({
                "format": V1,
                "inner_schema": 1,
                "auxiliary": present_null.3,
            }).to_string();
            assert!(Runtime::from_getup_checkpoint(&packed(&present_null)).is_err());
        }
        edit_controller(&mut runtime, |controller| {
            controller["terminal_balance"]["bumpless"] = bumpless("full-body80");
        });
        assert!(runtime.getup_checkpoint().is_err(), "legacy export {format:?}");
        assert!(Runtime::from_getup_checkpoint(&packed(&schema_one(&runtime))).is_err(), "legacy restore {format:?}");
        if format.is_some() {
            assert!(Runtime::from_getup_checkpoint(&schema_three(&runtime)).is_err());
        }
    }
}

#[cfg(feature = "root-translation-carry-experiment")]
fn activate_nonzero_carry(runtime: &mut Runtime) {
    runtime.set_root_translation_compensation_enabled(true).unwrap();
    let root = runtime.simulation.bodies[0];
    let index = runtime.simulation.world.multibody_joints.rigid_body_link(root).unwrap().multibody;
    let multibody = runtime.simulation.world.multibody_joints.get_multibody_mut(index).unwrap();
    let mut state = multibody.root_translation_integrator_state().unwrap();
    let axis = state.qbits.iter().position(|bits| f32::from_bits(*bits).abs() >= 0.001).unwrap();
    let q = f32::from_bits(state.qbits[axis]).abs();
    let ulp = f64::from(f32::from_bits(q.to_bits() + 1)) - f64::from(q);
    state.remainderbits[axis] = (ulp * 0.25).to_bits();
    multibody.restore_root_translation_integrator_state(state).unwrap();
}

#[cfg(feature = "root-translation-carry-experiment")]
#[test]
fn schema_two_never_accepts_hidden_v2_state() {
    let mut legacy = synthetic_runtime(None, None);
    activate_nonzero_carry(&mut legacy);
    let base = parts(&legacy.getup_checkpoint().unwrap());
    assert_eq!(base.0, 2);
    for admission in [None, Some("full-body80"), Some("early-base80")] {
        let runtime = synthetic_runtime(Some(V2), admission);
        let mut hidden = base.clone();
        let mut sidecar: RootTranslationCheckpoint = serde_json::from_str(&hidden.3).unwrap();
        sidecar.runtime = serde_json::to_string(&runtime).unwrap();
        hidden.3 = serde_json::to_string(&sidecar).unwrap();
        assert!(
            Runtime::from_getup_checkpoint(&packed(&hidden)).is_err(),
            "schema 2 accepted hidden V2 admission {admission:?}"
        );
    }
}

#[cfg(feature = "getup-terminal-bumpless-experiment")]
mod enabled {
    use super::*;

    fn command(runtime: &mut Runtime) -> [u32; 21] {
        let physical = crate::getup::physical_observation(&runtime.simulation).unwrap();
        let mut controller = runtime.simulation.getup_controller.take().unwrap();
        let controls = controller.command(&runtime.simulation, &physical).unwrap();
        runtime.simulation.getup_controller = Some(controller);
        controls.map(f32::to_bits)
    }

    #[test]
    fn fresh_v2_uses_unchanged_strict_schema_three_envelope() {
        let runtime = Runtime::new_getup_fixture("belly").unwrap();
        let controller = serde_json::to_value(runtime.simulation.getup_controller.as_ref().unwrap()).unwrap();
        assert_eq!(controller["terminal_base_ready"], ready(V2, 0));
        assert!(controller.get("terminal_balance").is_none());
        let checkpoint = runtime.getup_checkpoint().unwrap();
        let outer = parts(&checkpoint);
        assert_eq!(outer.0, 3);
        let wrapper: TerminalBaseReadyCheckpoint = serde_json::from_str(&outer.3).unwrap();
        assert_eq!(wrapper.format, V1);
        assert_eq!(wrapper.inner_schema, 1);
        assert_eq!(wrapper.auxiliary, serde_json::to_string(&runtime).unwrap());
        let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        assert_eq!(serde_json::to_string(&restored).unwrap(), wrapper.auxiliary);
        assert_eq!(runtime.getup_report().unwrap(), restored.getup_report().unwrap());
        for marker in [V2, "terminal-base-ready-v2"] {
            let mut invalid = outer.clone();
            let mut wrapper: Value = serde_json::from_str(&invalid.3).unwrap();
            wrapper["format"] = marker.into();
            invalid.3 = wrapper.to_string();
            assert!(Runtime::from_getup_checkpoint(&packed(&invalid)).is_err());
        }
    }

    #[test]
    fn reset_action_clears_binding_source_and_actor_state_in_a_fresh_v2_trial() {
        let mut runtime = synthetic_runtime(Some(V2), Some("full-body80"));
        command(&mut runtime);
        let generation = runtime.simulation.generation;
        runtime.apply(&Command {
            stamp: Stamp::new(generation, 1, 0),
            action: Action::Reset,
        }).unwrap();
        assert_eq!(runtime.simulation.generation, generation + 1);
        assert_eq!(runtime.simulation.substeps_completed, 0);
        let controller = serde_json::to_value(runtime.simulation.getup_controller.as_ref().unwrap()).unwrap();
        assert_eq!(controller["terminal_base_ready"], ready(V2, 0));
        assert!(controller.get("terminal_balance").is_none());
        assert_eq!(controller["started_substep"], Value::Null);
        assert_eq!(controller["teacher_frames"], json!([]));
        assert_eq!(controller["student_decisions"], json!([]));
        assert_eq!(controller["status"]["policy_decisions"], 0);
        assert_eq!(controller["status"]["teacher_decisions"], 0);
        assert_eq!(controller["status"]["forecast_substeps"], 0);
    }

    #[test]
    fn malformed_v2_bindings_are_refused_on_import_and_export() {
        let runtime = synthetic_runtime(Some(V2), Some("full-body80"));
        let checkpoint = runtime.getup_checkpoint().unwrap();
        type Mutation = Box<dyn FnOnce(&mut Value)>;
        let mutations: Vec<Mutation> = vec![
            Box::new(|c| { c["terminal_balance"].as_object_mut().unwrap().remove("bumpless"); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"] = Value::Null; }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["admission"] = "full-body81".into(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"].as_object_mut().unwrap().remove("last_blend_clamped_controls"); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["last_blend_clamped_controls"] = 22.into(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["progress"] = 0.5.into(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["stable_substeps_at_binding"] = 79.into(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["admission"] = "early-base80".into(); }),
            Box::new(|c| {
                c["terminal_balance"]["bumpless"]["admission"] = "early-base80".into();
                c["terminal_balance"]["bumpless"]["stable_substeps_at_binding"] = 0.into();
                c["terminal_base_ready"] = ready(V2, 79);
            }),
            Box::new(|c| { c["terminal_balance"]["activated_native_substep"] = (BINDING_SUBSTEP + 1).into(); }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"][0] = 1.001.into(); }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"][20] = (-1.001).into(); }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"][0] = json!(1.0e39_f64); }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"][20] = json!(-1.0e39_f64); }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"][0] = Value::Null; }),
            Box::new(|c| { c["terminal_balance"]["last_slow_controls"].as_array_mut().unwrap().pop(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["first_intended_controls"] = json!(([1.01_f32; 21])); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["first_intended_control_jump_max"] = (-0.01).into(); }),
            Box::new(|c| { c["terminal_balance"]["bumpless"]["first_intended_torque_jump_max_nm"] = (-0.01).into(); }),
        ];
        for (index, mutation) in mutations.into_iter().enumerate() {
            let invalid = mutate_schema_three_runtime(&checkpoint, |auxiliary| {
                mutation(&mut auxiliary["simulation"]["getup_controller"]);
            });
            assert!(Runtime::from_getup_checkpoint(&invalid).is_err(), "V2 import mutation {index}");
            let outer = parts(&invalid);
            let wrapper: Value = serde_json::from_str(&outer.3).unwrap();
            // Malformed typed state may fail decoding itself. Any state that
            // decodes must still be refused by the live export boundary.
            if let Ok(invalid_runtime) = serde_json::from_str::<Runtime>(wrapper["auxiliary"].as_str().unwrap()) {
                assert!(invalid_runtime.getup_checkpoint().is_err(), "V2 export mutation {index}");
            }
        }
    }

    fn midblend_restore(carry: bool) {
        for admission in ["full-body80", "early-base80"] {
            let mut continuous = synthetic_runtime(Some(V2), Some(admission));
            #[cfg(feature = "root-translation-carry-experiment")]
            if carry { activate_nonzero_carry(&mut continuous); }
            #[cfg(not(feature = "root-translation-carry-experiment"))]
            assert!(!carry);
            let before_world = world_bytes(&continuous);
            let first = command(&mut continuous);
            let state = serde_json::to_value(continuous.simulation.getup_controller.as_ref().unwrap()).unwrap();
            assert!(state["terminal_balance"]["bumpless"]["first_intended_controls"].is_array());
            assert!(state["terminal_balance"]["first_balance_controls"].is_array());
            let activated = state["terminal_balance"]["activated_native_substep"].clone();
            let source = state["terminal_balance"]["last_slow_controls"].clone();
            continuous.simulation.substeps_completed += 7;
            let midblend = command(&mut continuous);
            assert_ne!(first, midblend, "synthetic source differs from terminal intention");
            assert_eq!(
                continuous.simulation.getup_controller.as_ref().unwrap().status.phase,
                "experimental-joint-balance-blending"
            );
            let checkpoint = continuous.getup_checkpoint().unwrap();
            let outer = parts(&checkpoint);
            assert_eq!(outer.0, 3);
            let envelope: TerminalBaseReadyCheckpoint = serde_json::from_str(&outer.3).unwrap();
            assert_eq!(envelope.format, V1);
            assert_eq!(envelope.inner_schema, if carry { 2 } else { 1 });
            let mut restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
            assert_eq!(serde_json::to_string(&restored).unwrap(), serde_json::to_string(&continuous).unwrap());
            assert_eq!(restored.getup_report().unwrap(), continuous.getup_report().unwrap());
            assert_eq!(parts(&restored.getup_checkpoint().unwrap()).3, outer.3);
            #[cfg(feature = "root-translation-carry-experiment")]
            let roots = continuous.root_translation_checkpoint_entries().unwrap();
            let restored_world = world_bytes(&restored);
            for elapsed in [7, 8, 9, 10, 19, 20, 21, 37, 80] {
                continuous.simulation.substeps_completed = BINDING_SUBSTEP + elapsed;
                restored.simulation.substeps_completed = BINDING_SUBSTEP + elapsed;
                assert_eq!(command(&mut continuous), command(&mut restored), "{admission}, carry={carry}, elapsed={elapsed}");
                assert_eq!(serde_json::to_string(&continuous).unwrap(), serde_json::to_string(&restored).unwrap());
                let state = serde_json::to_value(restored.simulation.getup_controller.as_ref().unwrap()).unwrap();
                assert_eq!(state["terminal_balance"]["activated_native_substep"], activated);
                assert_eq!(state["terminal_balance"]["last_slow_controls"], source);
                assert_eq!(world_bytes(&continuous), before_world);
                assert_eq!(world_bytes(&restored), restored_world);
                #[cfg(feature = "root-translation-carry-experiment")]
                {
                    assert_eq!(continuous.root_translation_checkpoint_entries().unwrap(), roots);
                    assert_eq!(restored.root_translation_checkpoint_entries().unwrap(), roots);
                }
            }
        }
    }

    #[test]
    fn schema_three_midblend_restore_preserves_every_subsequent_command_word() {
        midblend_restore(false);
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    #[test]
    fn schema_three_midblend_restore_preserves_active_root_carry_and_commands() {
        midblend_restore(true);
    }
}
