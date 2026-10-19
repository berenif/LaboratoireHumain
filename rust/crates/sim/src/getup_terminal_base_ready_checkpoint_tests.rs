//! Checkpoint semantics only. Seeded counters and bindings are synthetic state,
//! never evidence of a physical recovery or of terminal admission.
use super::*;
use serde_json::{Value, json};

type CheckpointParts = (u32, String, Vec<u8>, String);

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

fn legacy_runtime() -> Runtime {
    let mut runtime = Runtime::new_getup_fixture("belly").unwrap();
    edit_controller(&mut runtime, |controller| {
        controller
            .as_object_mut()
            .unwrap()
            .remove("terminal_base_ready");
    });
    runtime
}

fn variant(count: u64) -> Value {
    json!({
        "format": "terminal-base-ready-v1",
        "consecutive_substeps": count,
        "feet": if count == 0 { Value::Null } else { json!([
            lh_model::Vec3 { x: 0.125, y: 0.0, z: -0.375 },
            lh_model::Vec3 { x: -0.125, y: 0.0, z: -0.375 },
        ]) },
    })
}

fn seed_variant(runtime: &mut Runtime, count: u64, admitted: bool) {
    let substeps_completed = runtime.simulation.substeps_completed;
    edit_controller(runtime, |controller| {
        controller["terminal_base_ready"] = variant(count);
        if admitted {
            // A deliberately synthetic admitted binding exercises persistence
            // of the controller variant, without claiming its admission gates.
            controller["started_substep"] = 0.into();
            controller["impact_substep"] = 0.into();
            controller["status"]["phase"] = "experimental-joint-balance-holding".into();
            controller["terminal_balance"] = json!({
                "activated_native_substep": substeps_completed,
                "reference": {
                    "com": rapier3d::prelude::Vector::new(0.0, 0.8, 0.0),
                    "forward": rapier3d::prelude::Vector::Z,
                    "lateral": rapier3d::prelude::Vector::X,
                },
                "last_slow_controls": ([0.125_f32; 21]),
                "first_balance_controls": null,
                "first_actual_control_jump_max": null,
                "first_saturated_controls": null,
                "first_compensation_active": null,
            });
        }
    });
}

fn variant_runtime(count: u64, admitted: bool) -> Runtime {
    let mut runtime = legacy_runtime();
    seed_variant(&mut runtime, count, admitted);
    runtime
}

fn legacy_parts(runtime: &Runtime) -> CheckpointParts {
    (
        1,
        lh_model::CANONICAL_JSON.into(),
        world_bytes(runtime),
        serde_json::to_string(runtime).unwrap(),
    )
}

#[test]
fn legacy_schema_one_keeps_exact_typed_auxiliary_and_none_in_every_build() {
    let live = legacy_runtime();
    let expected = legacy_parts(&live);
    assert!(
        !live
            .simulation
            .getup_controller
            .as_ref()
            .unwrap()
            .terminal_base_ready_variant()
    );
    assert_eq!(live.getup_checkpoint().unwrap(), packed(&expected));
    let restored = Runtime::from_getup_checkpoint(&packed(&expected)).unwrap();
    let actual = parts(&restored.getup_checkpoint().unwrap());
    assert_eq!(actual.0, 1);
    assert_eq!(actual.1, expected.1);
    assert_eq!(actual.3, expected.3);
    assert!(
        !restored
            .simulation
            .getup_controller
            .as_ref()
            .unwrap()
            .terminal_base_ready_variant()
    );
    assert_eq!(restored.simulation.substeps_completed, 0);
}

#[test]
fn legacy_schema_one_never_accepts_hidden_new_controller_state() {
    for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
        let live = variant_runtime(count, admitted);
        let result = Runtime::from_getup_checkpoint(&packed(&legacy_parts(&live)));
        assert!(
            result.is_err(),
            "schema 1 accepted count {count}, admitted={admitted}"
        );
    }
}

#[cfg(not(feature = "getup-terminal-base-ready-experiment"))]
#[test]
fn feature_off_refuses_schema_three_before_decoding_and_every_new_variant_export() {
    let bytes = packed(&(
        3,
        lh_model::CANONICAL_JSON.into(),
        vec![],
        "invalid auxiliary".into(),
    ));
    assert_eq!(
        Runtime::from_getup_checkpoint(&bytes).err().unwrap(),
        "Checkpoint requires getup-terminal-base-ready-experiment"
    );
    for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
        let runtime = variant_runtime(count, admitted);
        assert_eq!(
            runtime.getup_checkpoint().err().unwrap(),
            "Checkpoint requires getup-terminal-base-ready-experiment"
        );
        assert_eq!(
            Runtime::from_getup_checkpoint(&packed(&legacy_parts(&runtime)))
                .err()
                .unwrap(),
            "Checkpoint requires getup-terminal-base-ready-experiment"
        );
    }
}

#[cfg(feature = "root-translation-carry-experiment")]
fn activate_nonzero_carry(runtime: &mut Runtime) {
    runtime
        .set_root_translation_compensation_enabled(true)
        .unwrap();
    let root = runtime.simulation.bodies[0];
    let index = runtime
        .simulation
        .world
        .multibody_joints
        .rigid_body_link(root)
        .unwrap()
        .multibody;
    let multibody = runtime
        .simulation
        .world
        .multibody_joints
        .get_multibody_mut(index)
        .unwrap();
    let mut state = multibody.root_translation_integrator_state().unwrap();
    let axis = state
        .qbits
        .iter()
        .position(|bits| f32::from_bits(*bits).abs() >= 0.001)
        .unwrap();
    let q = f32::from_bits(state.qbits[axis]).abs();
    let ulp = f64::from(f32::from_bits(q.to_bits() + 1)) - f64::from(q);
    state.remainderbits[axis] = (ulp * 0.25).to_bits();
    multibody
        .restore_root_translation_integrator_state(state)
        .unwrap();
}

#[cfg(feature = "root-translation-carry-experiment")]
#[test]
fn legacy_schema_two_keeps_exact_sidecar_and_rejects_hidden_new_variants() {
    let mut runtime = legacy_runtime();
    activate_nonzero_carry(&mut runtime);
    let expected = packed(&(
        2,
        lh_model::CANONICAL_JSON.into(),
        world_bytes(&runtime),
        serde_json::to_string(&RootTranslationCheckpoint {
            format: rapier3d::dynamics::RootTranslationIntegratorState::ALGORITHM.into(),
            runtime: serde_json::to_string(&runtime).unwrap(),
            roots: runtime.root_translation_checkpoint_entries().unwrap(),
        })
        .unwrap(),
    ));
    assert_eq!(runtime.getup_checkpoint().unwrap(), expected);
    let restored = Runtime::from_getup_checkpoint(&expected).unwrap();
    let actual = parts(&restored.getup_checkpoint().unwrap());
    let original = parts(&expected);
    assert_eq!(actual.0, 2);
    assert_eq!(actual.3, original.3);
    assert!(
        !restored
            .simulation
            .getup_controller
            .as_ref()
            .unwrap()
            .terminal_base_ready_variant()
    );
    assert_eq!(
        runtime.root_translation_checkpoint_entries().unwrap(),
        restored.root_translation_checkpoint_entries().unwrap()
    );
    for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
        seed_variant(&mut runtime, count, admitted);
        let mut hidden = parts(&expected);
        let mut wrapper: RootTranslationCheckpoint = serde_json::from_str(&hidden.3).unwrap();
        wrapper.runtime = serde_json::to_string(&runtime).unwrap();
        hidden.3 = serde_json::to_string(&wrapper).unwrap();
        assert!(
            Runtime::from_getup_checkpoint(&packed(&hidden)).is_err(),
            "schema 2 accepted count {count}, admitted={admitted}"
        );
    }
}

#[cfg(feature = "getup-terminal-base-ready-experiment")]
mod enabled {
    use super::*;

    fn wrapper(bytes: &[u8]) -> TerminalBaseReadyCheckpoint {
        let (schema, _, _, auxiliary) = parts(bytes);
        assert_eq!(schema, 3);
        serde_json::from_str(&auxiliary).unwrap()
    }

    fn mutate_wrapper(bytes: &[u8], edit: impl FnOnce(&mut Value)) -> Vec<u8> {
        let mut outer = parts(bytes);
        let mut value = serde_json::from_str(&outer.3).unwrap();
        edit(&mut value);
        outer.3 = serde_json::to_string(&value).unwrap();
        packed(&outer)
    }

    fn json_without_wall_cost<T: Serialize>(data: &T) -> Value {
        fn remove(value: &mut Value) {
            match value {
                Value::Object(object) => {
                    object.remove("controller_ms");
                    object.remove("update_ms");
                    for value in object.values_mut() {
                        remove(value);
                    }
                }
                Value::Array(array) => {
                    for value in array {
                        remove(value);
                    }
                }
                _ => {}
            }
        }
        let mut value = serde_json::from_str(&serde_json::to_string(data).unwrap()).unwrap();
        remove(&mut value);
        value
    }

    #[test]
    fn fresh_zero_variant_uses_schema_three_before_any_integration() {
        let runtime = Runtime::new_getup_fixture("belly").unwrap();
        let checkpoint = runtime.getup_checkpoint().unwrap();
        let wrapper = wrapper(&checkpoint);
        assert_eq!(wrapper.format, "terminal-base-ready-v1");
        assert_eq!(wrapper.inner_schema, 1);
        assert_eq!(wrapper.auxiliary, serde_json::to_string(&runtime).unwrap());
        let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        assert_eq!(restored.simulation.substeps_completed, 0);
        assert!(
            restored
                .simulation
                .getup_controller
                .as_ref()
                .unwrap()
                .terminal_base_ready_variant()
        );
        assert_eq!(
            restored.getup_report().unwrap(),
            runtime.getup_report().unwrap()
        );
        assert_eq!(
            serde_json::from_str::<Value>(&restored.getup_report().unwrap()).unwrap()["terminal_base_ready"],
            variant(0)
        );
    }

    #[test]
    fn counts_and_admitted_binding_preserve_typed_auxiliary_exactly() {
        for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
            let runtime = variant_runtime(count, admitted);
            let original = serde_json::to_string(&runtime).unwrap();
            let checkpoint = runtime.getup_checkpoint().unwrap();
            let outer = parts(&checkpoint);
            let envelope = wrapper(&checkpoint);
            assert_eq!(outer.2, world_bytes(&runtime));
            assert_eq!(envelope.format, "terminal-base-ready-v1");
            assert_eq!(envelope.inner_schema, 1);
            assert_eq!(envelope.auxiliary, original);
            let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
            assert_eq!(serde_json::to_string(&restored).unwrap(), original);
            assert_eq!(
                restored.getup_report().unwrap(),
                runtime.getup_report().unwrap()
            );
            assert_eq!(parts(&restored.getup_checkpoint().unwrap()).3, outer.3);
            assert!(runtime.verify_getup_checkpoint(&checkpoint).is_ok());
            assert_eq!(runtime.simulation.substeps_completed, 0);
        }
    }

    #[test]
    fn schema_three_requires_strict_marker_nonrecursive_inner_and_present_valid_variant() {
        let runtime = variant_runtime(79, false);
        let checkpoint = runtime.getup_checkpoint().unwrap();
        type Mutation = Box<dyn FnOnce(&mut Value)>;
        let mutations: Vec<Mutation> = vec![
            Box::new(|v| {
                v["format"] = "terminal-base-ready-v2".into();
            }),
            Box::new(|v| {
                v.as_object_mut().unwrap().remove("format");
            }),
            Box::new(|v| {
                v.as_object_mut().unwrap().remove("inner_schema");
            }),
            Box::new(|v| {
                v.as_object_mut().unwrap().remove("auxiliary");
            }),
            Box::new(|v| {
                v["inner_schema"] = 0.into();
            }),
            Box::new(|v| {
                v["inner_schema"] = 3.into();
            }),
            Box::new(|v| {
                v["inner_schema"] = 4.into();
            }),
            Box::new(|v| {
                v["inner_schema"] = "1".into();
            }),
            Box::new(|v| {
                v["auxiliary"] = json!({});
            }),
            Box::new(|v| {
                v["extra"] = true.into();
            }),
            Box::new(|v| {
                let mut auxiliary: Value =
                    serde_json::from_str(v["auxiliary"].as_str().unwrap()).unwrap();
                auxiliary["simulation"]["getup_controller"]
                    .as_object_mut()
                    .unwrap()
                    .remove("terminal_base_ready");
                v["auxiliary"] = auxiliary.to_string().into();
            }),
            Box::new(|v| {
                let mut auxiliary: Value =
                    serde_json::from_str(v["auxiliary"].as_str().unwrap()).unwrap();
                auxiliary["simulation"]["getup_controller"]["terminal_base_ready"] = Value::Null;
                v["auxiliary"] = auxiliary.to_string().into();
            }),
            Box::new(|v| {
                let mut auxiliary: Value =
                    serde_json::from_str(v["auxiliary"].as_str().unwrap()).unwrap();
                auxiliary["simulation"]["getup_controller"]["terminal_base_ready"]["consecutive_substeps"] =
                    81.into();
                v["auxiliary"] = auxiliary.to_string().into();
            }),
            Box::new(|v| {
                let mut auxiliary: Value =
                    serde_json::from_str(v["auxiliary"].as_str().unwrap()).unwrap();
                auxiliary["simulation"]["getup_controller"]["terminal_base_ready"]["feet"] =
                    Value::Null;
                v["auxiliary"] = auxiliary.to_string().into();
            }),
            Box::new(|v| {
                let nested = v.to_string();
                v["auxiliary"] = nested.into();
            }),
        ];
        for (index, mutation) in mutations.into_iter().enumerate() {
            assert!(
                Runtime::from_getup_checkpoint(&mutate_wrapper(&checkpoint, mutation)).is_err(),
                "schema 3 mutation {index} accepted"
            );
        }
        for (count, admitted) in [(81, false), (79, true)] {
            assert!(variant_runtime(count, admitted).getup_checkpoint().is_err());
        }
        let mut invalid = variant_runtime(0, false);
        edit_controller(&mut invalid, |controller| {
            controller["terminal_base_ready"]["feet"] = variant(79)["feet"].clone();
        });
        assert!(invalid.getup_checkpoint().is_err());
    }

    #[cfg(not(feature = "root-translation-carry-experiment"))]
    #[test]
    fn inner_schema_two_still_requires_the_root_carry_feature() {
        let checkpoint = variant_runtime(0, false).getup_checkpoint().unwrap();
        let corrupt = mutate_wrapper(&checkpoint, |v| {
            v["inner_schema"] = 2.into();
        });
        assert_eq!(
            Runtime::from_getup_checkpoint(&corrupt).err().unwrap(),
            "Compensated checkpoint requires root-translation-carry-experiment"
        );
    }

    #[cfg(feature = "root-translation-carry-experiment")]
    #[test]
    fn schema_three_nests_exact_original_schema_two_sidecar_for_every_counter_state() {
        for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
            let mut runtime = variant_runtime(count, admitted);
            activate_nonzero_carry(&mut runtime);
            let roots = runtime.root_translation_checkpoint_entries().unwrap();
            let expected = serde_json::to_string(&RootTranslationCheckpoint {
                format: rapier3d::dynamics::RootTranslationIntegratorState::ALGORITHM.into(),
                runtime: serde_json::to_string(&runtime).unwrap(),
                roots: roots.clone(),
            })
            .unwrap();
            let checkpoint = runtime.getup_checkpoint().unwrap();
            let envelope = wrapper(&checkpoint);
            assert_eq!(envelope.inner_schema, 2);
            assert_eq!(envelope.auxiliary, expected);
            let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
            assert_eq!(
                roots,
                restored.root_translation_checkpoint_entries().unwrap()
            );
            assert_eq!(
                parts(&restored.getup_checkpoint().unwrap()).3,
                parts(&checkpoint).3
            );
            assert_eq!(
                runtime.getup_report().unwrap(),
                restored.getup_report().unwrap()
            );
            assert!(runtime.verify_getup_checkpoint(&checkpoint).is_ok());
            let missing = mutate_wrapper(&checkpoint, |v| {
                let mut sidecar: RootTranslationCheckpoint =
                    serde_json::from_str(v["auxiliary"].as_str().unwrap()).unwrap();
                let mut inner: Runtime = serde_json::from_str(&sidecar.runtime).unwrap();
                edit_controller(&mut inner, |controller| {
                    controller
                        .as_object_mut()
                        .unwrap()
                        .remove("terminal_base_ready");
                });
                sidecar.runtime = serde_json::to_string(&inner).unwrap();
                v["auxiliary"] = serde_json::to_string(&sidecar).unwrap().into();
            });
            assert!(Runtime::from_getup_checkpoint(&missing).is_err());
        }
    }

    #[test]
    fn restored_controller_and_world_take_the_exact_next_ordinary_tick() {
        for (count, admitted) in [(0, false), (79, false), (80, false), (80, true)] {
            let mut profile = Profile::default();
            profile.getup_policy_enabled = true;
            let mut continuous = Runtime::new(Mode::Protocol, 0.0, profile).unwrap();
            let first = continuous.advance(1).unwrap();
            assert_eq!(first.completed_ticks, 1);
            assert!(first.failure.is_none());
            seed_variant(&mut continuous, count, admitted);
            #[cfg(feature = "root-translation-carry-experiment")]
            activate_nonzero_carry(&mut continuous);
            let checkpoint = continuous.getup_checkpoint().unwrap();
            let mut restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
            assert_eq!(
                continuous.getup_report().unwrap(),
                restored.getup_report().unwrap()
            );
            assert_eq!(checkpoint, restored.getup_checkpoint().unwrap());
            let expected = continuous.advance(1).unwrap();
            let actual = restored.advance(1).unwrap();
            assert_eq!(expected.completed_ticks, 1);
            assert!(expected.failure.is_none());
            assert_eq!(
                json_without_wall_cost(&expected),
                json_without_wall_cost(&actual)
            );
            assert_eq!(world_bytes(&continuous), world_bytes(&restored));
            assert_eq!(
                json_without_wall_cost(&continuous),
                json_without_wall_cost(&restored)
            );
            #[cfg(feature = "root-translation-carry-experiment")]
            assert_eq!(
                continuous.root_translation_checkpoint_entries().unwrap(),
                restored.root_translation_checkpoint_entries().unwrap()
            );
        }
    }
}
