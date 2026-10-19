//! Checkpoint scope: only the isolated Runtime schema-2 sidecar preserves
//! active root integration history. Raw Rapier world serde deliberately does not.
use super::*;
#[cfg(feature = "root-translation-carry-experiment")]
use serde_json::Value;

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

fn assert_exact_bytes(left: &[u8], right: &[u8]) {
    assert_eq!(left.len(), right.len());
    assert!(
        left == right,
        "first differing byte: {:?}",
        left.iter().zip(right).position(|(a, b)| a != b)
    );
}

// At native zero, Rapier's pending MultibodyJointSet.to_join HashSet may
// serialize in a different order after restore. This is the sole demonstrated
// construction-only exception: all bytes outside that exact set remain exact.
// The pinned world ends in multibody_joints, whose final fields are to_join
// (u64 length, pairs of two u32/u32 body handles) and topology_epoch (u32).
fn assert_same_construction_world(left: &Runtime, right: &Runtime) {
    assert_eq!(left.simulation.substeps_completed, 0);
    assert_eq!(right.simulation.substeps_completed, 0);
    let queue_pairs = |runtime: &Runtime| {
        let fields = serde_json::to_value(&runtime.simulation.world.multibody_joints).unwrap();
        let pairs: Vec<(
            rapier3d::prelude::RigidBodyHandle,
            rapier3d::prelude::RigidBodyHandle,
        )> = serde_json::from_value(fields["to_join"].clone()).unwrap();
        pairs
            .into_iter()
            .map(|pair| {
                bincode::DefaultOptions::new()
                    .with_fixint_encoding()
                    .serialize(&pair)
                    .unwrap()
                    .try_into()
                    .unwrap()
            })
            .collect::<Vec<[u8; 16]>>()
    };
    let left_pairs = queue_pairs(left);
    let right_pairs = queue_pairs(right);
    let count = left_pairs.len();
    assert_eq!(right_pairs.len(), count);
    let left = world_bytes(left);
    let right = world_bytes(right);
    assert_eq!(left.len(), right.len());
    let end = left.len().checked_sub(4).unwrap();
    let start = end.checked_sub(8 + 16 * count).unwrap();
    for bytes in [&left, &right] {
        assert_eq!(
            u64::from_le_bytes(bytes[start..start + 8].try_into().unwrap()),
            count as u64
        );
    }
    assert_exact_bytes(&left[..start + 8], &right[..start + 8]);
    assert_exact_bytes(&left[end..], &right[end..]);
    let sorted_pairs = |bytes: &[u8], named_field_pairs: &[[u8; 16]]| {
        let mut pairs: Vec<[u8; 16]> = bytes[start + 8..end]
            .chunks_exact(16)
            .map(|pair| pair.try_into().unwrap())
            .collect();
        assert_eq!(
            pairs, named_field_pairs,
            "binary slice must be exactly the named to_join field"
        );
        pairs.sort_unstable();
        pairs
    };
    assert_eq!(
        sorted_pairs(&left, &left_pairs),
        sorted_pairs(&right, &right_pairs)
    );
}

fn assert_same_checkpoint_envelope(left: &[u8], right: &[u8]) {
    let left = parts(left);
    let right = parts(right);
    assert_eq!((left.0, left.1), (right.0, right.1));
    assert_exact_bytes(left.3.as_bytes(), right.3.as_bytes());
}

fn fixture() -> Runtime {
    Runtime::new_getup_fixture("belly").unwrap()
}

#[test]
fn inactive_checkpoint_retains_the_original_schema_one_bytes() {
    let live = fixture();
    let auxiliary = serde_json::to_string(&live).unwrap();
    let expected = packed(&(
        1,
        lh_model::CANONICAL_JSON.into(),
        world_bytes(&live),
        auxiliary,
    ));
    assert_eq!(live.getup_checkpoint().unwrap(), expected);
    let restored = Runtime::from_getup_checkpoint(&expected).unwrap();
    assert_same_checkpoint_envelope(&restored.getup_checkpoint().unwrap(), &expected);
    assert_same_construction_world(&live, &restored);
    assert_eq!(live.simulation.substeps_completed, 0);
    assert_eq!(restored.simulation.substeps_completed, 0);
    #[cfg(feature = "root-translation-carry-experiment")]
    assert!(
        restored
            .root_translation_checkpoint_entries()
            .unwrap()
            .iter()
            .all(|entry| entry.state.is_none())
    );
}

#[test]
fn checkpoint_input_is_bounded_before_decoding() {
    let oversized = vec![0; GETUP_CHECKPOINT_LIMIT as usize + 1];
    assert_eq!(
        Runtime::from_getup_checkpoint(&oversized).err().unwrap(),
        "Checkpoint exceeds size limit"
    );
}

#[cfg(not(feature = "root-translation-carry-experiment"))]
#[test]
fn feature_off_explicitly_rejects_schema_two_before_loading_its_world() {
    let bytes = packed(&(
        2,
        lh_model::CANONICAL_JSON.into(),
        Vec::new(),
        String::new(),
    ));
    assert_eq!(
        Runtime::from_getup_checkpoint(&bytes).err().unwrap(),
        "Compensated checkpoint requires root-translation-carry-experiment"
    );
}

#[cfg(feature = "root-translation-carry-experiment")]
mod compensated {
    use super::*;
    use rapier3d::dynamics::RootTranslationIntegratorState;

    fn sidecar(runtime: &Runtime) -> Vec<RootTranslationCheckpointEntry> {
        runtime.root_translation_checkpoint_entries().unwrap()
    }

    fn wrapper(bytes: &[u8]) -> RootTranslationCheckpoint {
        let (schema, _, _, auxiliary) = parts(bytes);
        assert_eq!(schema, 2);
        serde_json::from_str(&auxiliary).unwrap()
    }

    fn mutate_wrapper(bytes: &[u8], change: impl FnOnce(&mut Value)) -> Vec<u8> {
        let mut parts = parts(bytes);
        let mut auxiliary: Value = serde_json::from_str(&parts.3).unwrap();
        change(&mut auxiliary);
        parts.3 = serde_json::to_string(&auxiliary).unwrap();
        packed(&parts)
    }

    fn with_nonzero_history() -> Runtime {
        let mut runtime = fixture();
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
        // A deterministic sub-ULP residue at a nonzero root coordinate. It is
        // too small to alter the stored f32 coordinate and must survive exactly.
        let axis = state
            .qbits
            .iter()
            .position(|bits| f32::from_bits(*bits).abs() >= 0.001)
            .unwrap();
        let q = f32::from_bits(state.qbits[axis]);
        let ulp = f64::from(f32::from_bits(q.abs().to_bits() + 1)) - f64::from(q.abs());
        state.remainderbits[axis] = (ulp * 0.25).to_bits();
        multibody
            .restore_root_translation_integrator_state(state)
            .unwrap();
        assert!(
            state
                .remainderbits
                .iter()
                .any(|bits| f64::from_bits(*bits) != 0.0)
        );
        runtime
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
        let mut value: Value = serde_json::from_str(&serde_json::to_string(data).unwrap()).unwrap();
        remove(&mut value);
        value
    }

    // Clone every serialized physical world component, including Multibody's
    // non-serialized carry. Only documented pipeline/CCD workspaces are fresh.
    fn clone_physical_runtime(runtime: &Runtime) -> Runtime {
        let mut clone: Runtime =
            serde_json::from_str(&serde_json::to_string(runtime).unwrap()).unwrap();
        let source = &runtime.simulation.world;
        clone.simulation.world = rapier3d::prelude::PhysicsWorld {
            gravity: source.gravity,
            integration_parameters: source.integration_parameters,
            islands: source.islands.clone(),
            broad_phase: source.broad_phase.clone(),
            narrow_phase: source.narrow_phase.clone(),
            bodies: source.bodies.clone(),
            colliders: source.colliders.clone(),
            impulse_joints: source.impulse_joints.clone(),
            multibody_joints: source.multibody_joints.clone(),
            ..rapier3d::prelude::PhysicsWorld::new()
        };
        clone
    }

    fn assert_same_physics(left: &Runtime, right: &Runtime) {
        assert_exact_bytes(&world_bytes(left), &world_bytes(right));
        assert_same_observations_and_integrator(left, right);
    }

    fn assert_same_observations_and_integrator(left: &Runtime, right: &Runtime) {
        assert_eq!(
            sidecar(left),
            sidecar(right),
            "exact integrator integer bits"
        );
        assert_eq!(
            json_without_wall_cost(&left.simulation.snapshot()),
            json_without_wall_cost(&right.simulation.snapshot())
        );
        assert_eq!(
            json_without_wall_cost(left),
            json_without_wall_cost(right),
            "complete auxiliary state except wall cost"
        );
    }

    #[test]
    fn activation_and_nonzero_history_checkpoint_are_physics_neutral_and_exact() {
        let mut runtime = fixture();
        let world = world_bytes(&runtime);
        let auxiliary = serde_json::to_string(&runtime).unwrap();
        runtime
            .set_root_translation_compensation_enabled(true)
            .unwrap();
        assert_eq!(world_bytes(&runtime), world);
        assert_eq!(serde_json::to_string(&runtime).unwrap(), auxiliary);
        assert_eq!(runtime.simulation.substeps_completed, 0);
        let first_state = sidecar(&runtime);
        runtime
            .set_root_translation_compensation_enabled(true)
            .unwrap();
        assert_eq!(sidecar(&runtime), first_state, "re-enable is idempotent");

        let runtime = with_nonzero_history();
        let world = world_bytes(&runtime);
        let auxiliary = serde_json::to_string(&runtime).unwrap();
        let checkpoint = runtime.getup_checkpoint().unwrap();
        let wrapper = wrapper(&checkpoint);
        assert_eq!(
            wrapper.runtime, auxiliary,
            "Runtime stays the original typed JSON string"
        );
        assert_eq!(wrapper.format, RootTranslationIntegratorState::ALGORITHM);
        assert_eq!(wrapper.roots, sidecar(&runtime));
        let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        assert_same_construction_world(&runtime, &restored);
        assert_same_observations_and_integrator(&runtime, &restored);
        assert_same_checkpoint_envelope(&restored.getup_checkpoint().unwrap(), &checkpoint);
        assert_eq!(
            world_bytes(&runtime),
            world,
            "export and verification do not mutate the source"
        );
        let verification: Value =
            serde_json::from_str(&runtime.verify_getup_checkpoint(&checkpoint).unwrap()).unwrap();
        assert_eq!(verification["rootIntegratorStateEqual"], true);
        assert_eq!(verification["physicsIntegrations"], 0);
        let report: Value = serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
        assert_eq!(
            report["rootTranslationCompensation"]["algorithm"],
            RootTranslationIntegratorState::ALGORITHM
        );
        assert_eq!(runtime.simulation.substeps_completed, 0);
    }

    #[test]
    fn fresh_construction_checkpoint_and_physical_clone_preserve_the_first_ordinary_tick() {
        let mut continuous = with_nonzero_history();
        let checkpoint = continuous.getup_checkpoint().unwrap();
        let mut restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        let mut cloned = clone_physical_runtime(&continuous);
        assert_same_construction_world(&continuous, &restored);
        assert_same_observations_and_integrator(&continuous, &restored);
        assert_same_physics(&continuous, &cloned);
        // Clone retains the pre-existing serde-skipped island_chain_events;
        // checkpoint restore reconstructs that workspace. Test its first real
        // step explicitly instead of assuming construction serde is complete.
        let expected = continuous.advance(1).unwrap();
        let resumed = restored.advance(1).unwrap();
        let clone_reply = cloned.advance(1).unwrap();
        assert_eq!(expected.completed_ticks, 1);
        assert!(expected.failure.is_none());
        assert_eq!(
            json_without_wall_cost(&expected),
            json_without_wall_cost(&resumed)
        );
        assert_eq!(
            json_without_wall_cost(&expected),
            json_without_wall_cost(&clone_reply)
        );
        assert_same_physics(&continuous, &restored);
        assert_same_physics(&continuous, &cloned);
    }

    #[test]
    fn clone_and_checkpoint_resume_the_same_next_ordinary_tick_with_integrated_history() {
        let mut continuous = with_nonzero_history();
        let first = continuous.advance(1).unwrap();
        assert_eq!(first.completed_ticks, 1);
        assert!(first.failure.is_none());
        assert!(
            sidecar(&continuous)
                .iter()
                .filter_map(|entry| entry.state)
                .any(|state| state
                    .remainderbits
                    .iter()
                    .any(|bits| f64::from_bits(*bits) != 0.0))
        );
        let checkpoint = continuous.getup_checkpoint().unwrap();
        let mut restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        let mut cloned = clone_physical_runtime(&continuous);
        assert_same_physics(&continuous, &restored);
        assert_same_physics(&continuous, &cloned);
        let expected = continuous.advance(1).unwrap();
        let resumed = restored.advance(1).unwrap();
        let clone_reply = cloned.advance(1).unwrap();
        assert_eq!(expected.completed_ticks, 1);
        assert!(expected.failure.is_none());
        assert_eq!(
            json_without_wall_cost(&expected),
            json_without_wall_cost(&resumed)
        );
        assert_eq!(
            json_without_wall_cost(&expected),
            json_without_wall_cost(&clone_reply)
        );
        assert_same_physics(&continuous, &restored);
        assert_same_physics(&continuous, &cloned);
    }

    #[test]
    fn raw_world_serde_and_legacy_schema_one_are_explicit_inactive_boundaries() {
        let active = with_nonzero_history();
        let legacy = packed(&(
            1,
            lh_model::CANONICAL_JSON.into(),
            world_bytes(&active),
            serde_json::to_string(&active).unwrap(),
        ));
        let restored = Runtime::from_getup_checkpoint(&legacy).unwrap();
        assert!(sidecar(&restored).iter().all(|entry| entry.state.is_none()));
        assert_eq!(parts(&restored.getup_checkpoint().unwrap()).0, 1);
        assert_same_construction_world(&restored, &active);
        assert!(
            active.verify_getup_checkpoint(&legacy).is_err(),
            "active history loss is detected"
        );
        let mut disabled = active;
        let before = world_bytes(&disabled);
        disabled
            .set_root_translation_compensation_enabled(false)
            .unwrap();
        assert!(sidecar(&disabled).iter().all(|entry| entry.state.is_none()));
        assert_eq!(world_bytes(&disabled), before);
        assert_eq!(parts(&disabled.getup_checkpoint().unwrap()).0, 1);
    }

    #[test]
    fn malformed_missing_duplicate_wrong_generation_and_invalid_bit_states_are_rejected() {
        let runtime = with_nonzero_history();
        let checkpoint = runtime.getup_checkpoint().unwrap();
        type Mutation = Box<dyn FnOnce(&mut Value)>;
        let mutations: Vec<Mutation> = vec![
            Box::new(|v| {
                v["format"] = "unknown".into();
            }),
            Box::new(|v| {
                v.as_object_mut().unwrap().remove("roots");
            }),
            Box::new(|v| {
                v["roots"] = serde_json::json!([]);
            }),
            Box::new(|v| {
                let first = v["roots"][0].clone();
                v["roots"].as_array_mut().unwrap().push(first);
            }),
            Box::new(|v| {
                v["roots"][0]["root"][1] = 99.into();
            }),
            Box::new(|v| {
                v["roots"][0]["root"][0] = u32::MAX.into();
            }),
            Box::new(|v| {
                v["roots"][0].as_object_mut().unwrap().remove("state");
            }),
            Box::new(|v| {
                v["roots"][0]["state"] = Value::Null;
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["qbits"][0] = 1.into();
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["qbits"][1] = f32::NAN.to_bits().into();
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["remainderbits"][0] = f64::NAN.to_bits().into();
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["remainderbits"][0] = f64::INFINITY.to_bits().into();
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["remainderbits"][1] = 1.0_f64.to_bits().into();
            }),
            Box::new(|v| {
                v["roots"][0]["state"]["extra"] = true.into();
            }),
            Box::new(|v| {
                v["extra"] = true.into();
            }),
            Box::new(|v| {
                v["runtime"] = serde_json::json!({});
            }),
        ];
        for (index, mutation) in mutations.into_iter().enumerate() {
            let corrupt = mutate_wrapper(&checkpoint, mutation);
            assert!(
                Runtime::from_getup_checkpoint(&corrupt).is_err(),
                "mutation {index} was accepted"
            );
        }
        let mut trailing = checkpoint.clone();
        trailing.push(0);
        assert!(Runtime::from_getup_checkpoint(&trailing).is_err());
        assert!(Runtime::from_getup_checkpoint(&checkpoint[..checkpoint.len() / 2]).is_err());
        let mut wrong_schema = parts(&checkpoint);
        wrong_schema.0 = 3;
        assert!(Runtime::from_getup_checkpoint(&packed(&wrong_schema)).is_err());
        wrong_schema.0 = 1;
        assert!(
            Runtime::from_getup_checkpoint(&packed(&wrong_schema)).is_err(),
            "relabeling an active schema-2 wrapper must not discard its sidecar"
        );
        let mut wrong_model = parts(&checkpoint);
        wrong_model.1.push(' ');
        assert!(Runtime::from_getup_checkpoint(&packed(&wrong_model)).is_err());
    }

    #[test]
    fn sidecar_tracks_every_root_deterministically_including_inactive_roots() {
        use rapier3d::prelude::*;
        let mut runtime = with_nonzero_history();
        let a = runtime
            .simulation
            .world
            .bodies
            .insert(RigidBodyBuilder::dynamic());
        let b = runtime
            .simulation
            .world
            .bodies
            .insert(RigidBodyBuilder::dynamic());
        runtime
            .simulation
            .world
            .multibody_joints
            .insert(a, b, FixedJointBuilder::new(), false)
            .unwrap();
        let entries = sidecar(&runtime);
        assert_eq!(entries.len(), 2);
        assert!(entries[0].root < entries[1].root);
        assert!(entries[0].state.is_some());
        assert!(entries[1].state.is_none());
        let checkpoint = runtime.getup_checkpoint().unwrap();
        let restored = Runtime::from_getup_checkpoint(&checkpoint).unwrap();
        assert_same_construction_world(&runtime, &restored);
        assert_same_observations_and_integrator(&runtime, &restored);
        let missing = mutate_wrapper(&checkpoint, |v| {
            v["roots"].as_array_mut().unwrap().pop();
        });
        assert!(Runtime::from_getup_checkpoint(&missing).is_err());
        let reversed = mutate_wrapper(&checkpoint, |v| {
            v["roots"].as_array_mut().unwrap().reverse();
        });
        assert!(Runtime::from_getup_checkpoint(&reversed).is_err());
    }
}
