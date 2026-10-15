//! Arithmetic-only adapter tests. No scene, world, FK, installed motor or
//! native integration is constructed. A36 supplies archived command/support
//! fields; synthetic support sequences exercise only the observation policy.

use super::super::{
    load_transfer::AfterHeels,
    support_schedule::SupportSchedule,
    support_state::{CurrentContact, SegmentRole},
    whole_body_tasks::{AXES, HardTask, Task},
};
use super::*;
use crate::recovery::trajectory_program::Program;

struct Inputs {
    model: Model,
    observation: SupportObservation,
    entry: ObservedCommandEntry,
    source: CompiledProgram,
    timing: Timing,
}

fn inputs() -> Inputs {
    // Derived serde deserialization only. Model::canonical()/validate() would
    // reconstruct mass properties and are deliberately absent from these tests.
    let model: Model = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../model/data/canonical-v1.json"
    )))
    .unwrap();
    let archive: Value = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json"
    ))).unwrap();
    let observation =
        serde_json::from_value(archive["archivedNativeRow"]["support"].clone()).unwrap();
    let entry: ObservedCommandEntry =
        serde_json::from_value(archive["commandEntry"].clone()).unwrap();
    assert_eq!(entry.native_substep, 36);
    let source: Program = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json"
    ))).unwrap();
    let source = source.compile(&model).unwrap();
    Inputs {
        model,
        observation,
        entry,
        source,
        timing: Timing {
            heels_acquired: 168,
            recentered_hold_started: 300,
            recentered_hold_completed: 432,
            standing_reference: 1680,
            hold_completed: 1920,
        },
    }
}

fn adapter(input: &Inputs) -> Adapter {
    let tracker = SupportSchedule::bootstrap_observed(&input.observation, &[19, 24])
        .unwrap()
        .tracker()
        .clone();
    Adapter::new(
        &input.model,
        &input.observation,
        tracker,
        input.entry.clone(),
        input.source.clone(),
        input.timing,
        TransferIntent::from_archived_heel_reference(AfterHeels::ContinueBilateral, 516),
        0,
    )
    .unwrap()
}

/// Synthetic contact geometry only; the archived q/dq supply the compiler
/// contract, not a claim that this support arrangement was physically reached.
fn surface_continuity_inputs() -> Inputs {
    let mut input = inputs();
    // These synthetic observations do not inherit the archived step-36 CCD
    // impulses; carrying them forward would correctly fail the clock guard.
    input.observation.ccd_impulses.clear();
    for (segment, x) in [(19, -0.15), (24, 0.15)] {
        input.observation.bodies[segment].position_m = [x, 0.045, 0.0];
        input.observation.bodies[segment].rotation_xyzw = [0.0, 0.0, 0.0, 1.0];
        let support = &mut input.observation.segments[segment];
        support.persistent_load_n = 160.0;
        support.loaded_substeps = 36;
        support.contacts = [
            (0.0, 0.0, 100.0),
            (-0.04, -0.03, 20.0),
            (0.04, -0.03, 20.0),
            (0.0, 0.04, 20.0),
        ]
        .into_iter()
        .map(|(dx, z, load)| CurrentContact {
            local_point_m: [dx, -0.045, z],
            world_point_m: [x + dx, 0.0, z],
            upward_normal: 1.0,
            current_upward_load_n: load,
        })
        .collect();
    }
    input
}

#[test]
fn pure_equivalent_surface_keeps_original_anchor_and_still_requires_native_admission() {
    let input = surface_continuity_inputs();
    let mut adapter = adapter(&input);
    let initial = adapter.tracker.active[0].clone();
    assert!(initial.reference_surface.is_some());
    let mut next = input.observation.clone();
    next.native_substep += 1;
    next.time_s += 1.0 / 240.0;
    next.segments[19].contacts.remove(0);
    let proposal = adapter.observe(&next).unwrap();
    let retained = &adapter.tracker.active[0];
    assert!(!retained.current_contact_present);
    assert!(retained.has_current_support());
    assert!(retained.verify_current_support(&next, MATERIAL_MATCH_M).unwrap());
    assert_eq!(retained.local_point_m, initial.local_point_m);
    assert_eq!(retained.world_anchor_m, initial.world_anchor_m);
    assert_eq!(retained.reference_surface, initial.reference_surface);
    assert_eq!(retained.id, initial.id);
    assert!(proposal.material_obligations[0].has_current_support());
    assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
    assert!(adapter.require_native_command(17, 37, vec![1, 2, 3], &input.entry.command()).is_err());
}

#[test]
fn pure_equivalent_surface_cannot_hide_cumulative_slip_or_refresh_an_anchor() {
    let input = surface_continuity_inputs();
    let mut adapter = adapter(&input);
    let anchor = adapter.tracker.active[0].world_anchor_m;
    let mut next = input.observation.clone();
    next.segments[19].contacts.remove(0);
    // Each new observation moves only 1 mm; the old world anchor must still
    // expose the accumulated 11 mm instead of accepting pairwise small motion.
    for offset in 1..=11 {
        next.native_substep += 1;
        next.time_s = next.native_substep as f64 / 240.0;
        next.bodies[19].position_m[0] = -0.15 + offset as f64 * 0.001;
        for contact in &mut next.segments[19].contacts {
            contact.world_point_m[0] = next.bodies[19].position_m[0] + contact.local_point_m[0];
        }
        let before = adapter.evidence();
        let result = adapter.observe(&next);
        if offset < 10 {
            result.unwrap();
        } else if let Err(reason) = result {
            // The contract is stored as f32: its exact 0.01 representation may
            // reject the mathematical 10 mm boundary. Never enlarge it here.
            assert!(reason.contains("drift envelope"), "{reason}");
            assert_eq!(adapter.evidence(), before);
            assert_eq!(adapter.tracker.active[0].world_anchor_m, anchor);
            return;
        }
        assert_eq!(adapter.tracker.active[0].world_anchor_m, anchor);
    }
    panic!("Cumulative drift above the original limit was accepted");
}

#[test]
fn pure_surface_reference_mutation_or_lost_current_load_is_refused_atomically() {
    let input = surface_continuity_inputs();
    let mut original = adapter(&input);
    let mut next = input.observation.clone();
    next.native_substep += 1;
    next.time_s += 1.0 / 240.0;
    next.segments[19].contacts.remove(0);
    original.observe(&next).unwrap();
    next.native_substep += 1;
    next.time_s += 1.0 / 240.0;
    for variant in 0..3 {
        let mut changed = original.clone();
        let mut observation = next.clone();
        match variant {
            0 => changed.tracker.active[0].world_anchor_m[0] += 0.001,
            1 => changed.tracker.active[0].reference_surface.as_mut().unwrap()
                .unit_normal_local = [1.0, 0.0, 0.0],
            _ => {
                observation.segments[19].contacts.clear();
                observation.ccd_impulses.push(super::super::support_state::CcdImpulse {
                    native_substep: observation.native_substep,
                    ccd_interval: 0,
                    segment: 19,
                    interval_s: 1.0 / 240.0,
                    normal_ns: [0.0, 100.0, 0.0],
                    tangent_ns: [0.0; 3],
                });
            }
        }
        let before = changed.evidence();
        assert!(changed.observe(&observation).is_err());
        assert_eq!(changed.evidence(), before);
    }
}

fn memory(input: &Inputs) -> runtime::Memory {
    runtime::initialize(
        &input.model,
        input.entry.previous_q,
        input.entry.native_substep,
        0,
    )
    .unwrap()
}

fn coordinate_bits(q: &Coordinates) -> Vec<u32> {
    q.iter().flatten().map(|v| v.to_bits()).collect()
}

fn memory_evidence(memory: &runtime::Memory) -> Value {
    json!({"previousBits": coordinate_bits(&memory.previous),
        "nextSubstep": memory.next_substep,
        "rateBits": memory.rates.map(f64::to_bits).to_vec(),
        "modeSignature": memory.mode_signature})
}

fn assert_command_bits(a: &Command, b: &Command) {
    assert_eq!(coordinate_bits(&a.positions), coordinate_bits(&b.positions));
    assert_eq!(
        coordinate_bits(&a.velocities),
        coordinate_bits(&b.velocities)
    );
}

/// A synthetic coefficient problem, not a reconstructed body Jacobian. Rates
/// point into each anatomical interval; every material guide is explicitly
/// zero and carries no physical-support meaning.
fn coefficient_problem(input: &Inputs, adapter: &Adapter) -> Problem {
    let layout = runtime::axes(&input.model).unwrap();
    let soft = layout
        .iter()
        .enumerate()
        .map(|(i, axis)| {
            let midpoint = 0.5 * f64::from(axis.minimum) + 0.5 * f64::from(axis.maximum);
            let previous = f64::from(input.entry.previous_q[axis.segment][axis.coordinate]);
            let mut row = [0.; AXES];
            row[i] = 1.;
            Task {
                row,
                target: (midpoint - previous).signum() * 0.125,
                weight: 1.,
            }
        })
        .collect();
    let hard = adapter
        .tracker()
        .active
        .iter()
        .flat_map(|patch| {
            (0..3).map(move |axis| HardTask {
                name: format!("material-patch-{}-world-{axis}", patch.id),
                row: [0.; AXES],
                lower: -0.005,
                upper: 0.005,
            })
        })
        .collect();
    Problem {
        soft,
        hard,
        lower: [-0.25; AXES],
        upper: [0.25; AXES],
        regularization: 0.05,
    }
}

#[test]
fn pure_archived_four_command_queue_keeps_exact_q_dq_and_consumes_once() {
    let input = inputs();
    let mut adapter = adapter(&input);
    let mut memory = memory(&input);
    adapter.observe(&input.observation).unwrap();
    let problem = coefficient_problem(&input, &adapter);
    adapter
        .compile_batch(&input.model, &memory, &problem, 36)
        .unwrap();
    let certified = adapter.queue.clone();
    assert_eq!(certified.len(), 4);
    assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
    assert!(
        certified
            .iter()
            .any(|command| coordinate_bits(&command.positions)
                != coordinate_bits(&input.entry.previous_q))
    );
    for (offset, expected) in certified.iter().enumerate() {
        let at = 36 + offset as u64;
        let command = adapter.proposed_command(&input.model, &memory, at).unwrap();
        assert_command_bits(&command, expected);
        for i in 0..25 {
            for k in 0..3 {
                let difference = command.positions[i][k] - memory.previous[i][k];
                assert!(difference.abs() <= lh_contracts::SUBSTEP_S);
                assert_eq!(
                    command.velocities[i][k].to_bits(),
                    (difference / lh_contracts::SUBSTEP_S).to_bits()
                );
            }
        }
        validate_command(&input.model, &command).unwrap();
        adapter.commit_command(&mut memory, &command, at).unwrap();
        assert_eq!(memory.next_substep, at + 1);
        assert_eq!(
            coordinate_bits(&memory.previous),
            coordinate_bits(&command.positions)
        );
        assert_eq!(
            coordinate_bits(&adapter.previous_dq),
            coordinate_bits(&command.velocities)
        );
        let prior_adapter = adapter.evidence();
        let prior_memory = memory_evidence(&memory);
        assert!(adapter.commit_command(&mut memory, &command, at).is_err());
        assert!(adapter.proposed_command(&input.model, &memory, at).is_err());
        assert_eq!(adapter.evidence(), prior_adapter);
        assert_eq!(memory_evidence(&memory), prior_memory);
    }
    assert!(adapter.proposed_command(&input.model, &memory, 40).is_err());
    let after = adapter.evidence();
    assert!(
        adapter
            .compile_batch(&input.model, &memory, &problem, 36)
            .is_err()
    );
    assert_eq!(adapter.evidence(), after);
}

#[test]
fn pure_foreign_queue_command_or_changed_command_history_is_refused_atomically() {
    let input = inputs();
    let mut adapter = adapter(&input);
    let mut memory = memory(&input);
    adapter.observe(&input.observation).unwrap();
    adapter
        .compile_batch(
            &input.model,
            &memory,
            &coefficient_problem(&input, &adapter),
            36,
        )
        .unwrap();
    let expected = adapter.proposed_command(&input.model, &memory, 36).unwrap();
    let axis = &runtime::axes(&input.model).unwrap()[0];
    for alter_velocity in [false, true] {
        let mut foreign = expected.clone();
        if alter_velocity {
            foreign.velocities[axis.segment][axis.coordinate] += 0.01;
        } else {
            foreign.positions[axis.segment][axis.coordinate] += 0.001;
        }
        let before_adapter = adapter.evidence();
        let before_memory = memory_evidence(&memory);
        assert!(adapter.commit_command(&mut memory, &foreign, 36).is_err());
        assert_eq!(adapter.evidence(), before_adapter);
        assert_eq!(memory_evidence(&memory), before_memory);
    }
    let before_adapter = adapter.evidence();
    let mut wrong_history = memory.clone();
    wrong_history.previous[axis.segment][axis.coordinate] += 0.001;
    let before_memory = memory_evidence(&wrong_history);
    assert!(
        adapter
            .proposed_command(&input.model, &wrong_history, 36)
            .is_err()
    );
    assert_eq!(memory_evidence(&wrong_history), before_memory);
    assert_eq!(adapter.evidence(), before_adapter);
    assert_command_bits(
        &adapter.proposed_command(&input.model, &memory, 36).unwrap(),
        &expected,
    );
}

#[test]
fn pure_duplicate_observation_is_idempotent_but_changed_or_skipped_clock_is_atomic() {
    let input = inputs();
    let mut adapter = adapter(&input);
    adapter.observe(&input.observation).unwrap();
    let before = adapter.evidence();
    adapter.observe(&input.observation).unwrap();
    assert_eq!(adapter.evidence(), before);
    let mut altered = input.observation.clone();
    altered.center_of_mass_m[0] += 0.001;
    let reason = adapter.observe(&altered).unwrap_err();
    assert!(reason.contains("already counted clock"), "{reason}");
    assert_eq!(adapter.evidence(), before);
    let mut skipped = input.observation.clone();
    skipped.native_substep += 2;
    skipped.time_s += 2. / 240.;
    assert!(adapter.observe(&skipped).is_err());
    assert_eq!(adapter.evidence(), before);
    let mut rewind = input.observation.clone();
    rewind.native_substep -= 1;
    rewind.time_s -= 1. / 240.;
    assert!(adapter.observe(&rewind).is_err());
    assert_eq!(adapter.evidence(), before);
}

#[test]
fn pure_unarmed_native_command_and_passive_handoff_are_both_refused_atomically() {
    let input = inputs();
    let mut adapter = adapter(&input);
    adapter.terminal_clock.observe(36, false).unwrap();
    let before = adapter.evidence();
    let command_refusal = adapter
        .require_native_command(17, 36, vec![1, 2, 3], &input.entry.command())
        .unwrap_err();
    assert!(
        command_refusal.contains("No budgeted native transfer window"),
        "{command_refusal}"
    );
    assert_eq!(adapter.evidence(), before);
    let passive_refusal = adapter
        .require_native_passive(17, 36, vec![1, 2, 3])
        .unwrap_err();
    assert!(
        passive_refusal.contains("No budgeted native transfer window"),
        "{passive_refusal}"
    );
    assert_eq!(adapter.evidence(), before);
    assert!(adapter.passive_since.is_none());
}

#[test]
fn pure_reference_clock_uses_source_433_only_after_the_last_bridge_command() {
    let input = inputs();
    let mut adapter = adapter(&input);
    assert_eq!(adapter.reference_substep(36), 432);
    // Synthetic clock evidence exercises reference indexing, never native
    // admission. These q/dq remain the A36 archived command entry.
    let start = 200;
    let entry = ObservedCommandEntry {
        native_substep: start,
        ..input.entry.clone()
    };
    let raccord = PreparedRaccord::prepare(
        &input.model,
        &entry,
        &input.source,
        input.timing,
        Some(LocalReadyReceipt {
            native_substep: start,
            first_stable_substep: start - 131,
        }),
        0,
    )
    .unwrap();
    let bridge_steps = u64::from(raccord.timing.bridge_substeps);
    let end = raccord.timing.end_native_substep_exclusive;
    adapter.raccord = Some(raccord);
    assert_eq!(adapter.reference_substep(start), 432);
    assert_eq!(adapter.reference_substep(start + bridge_steps - 1), 432);
    assert_eq!(adapter.reference_substep(start + bridge_steps), 433);
    assert_eq!(adapter.reference_substep(start + bridge_steps + 1), 434);
    assert_eq!(adapter.reference_substep(end - 1), 1920);
    let reference = adapter.raccord.as_ref().unwrap();
    let last_bridge = reference.command(start + bridge_steps - 1).unwrap();
    assert_eq!(
        coordinate_bits(&last_bridge.positions),
        coordinate_bits(&input.source.command(432).positions)
    );
    let first_source = reference.command(start + bridge_steps).unwrap();
    assert_eq!(
        coordinate_bits(&first_source.positions),
        coordinate_bits(&input.source.command(433).positions)
    );
    for i in 0..25 {
        for k in 0..3 {
            assert_eq!(
                first_source.velocities[i][k].to_bits(),
                ((first_source.positions[i][k] - last_bridge.positions[i][k])
                    / lh_contracts::SUBSTEP_S)
                    .to_bits()
            );
        }
    }
    assert!(reference.command(end).is_err());
    assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
}

#[test]
fn pure_persistent_nonfoot_after_latched_local_event_is_refused_without_erasing_history() {
    let input = inputs();
    let (initial, tracker, intent, history) = load_transfer::stable_fixture_history(36);
    let mut adapter = Adapter::new(
        &input.model,
        &initial,
        tracker,
        input.entry.clone(),
        input.source.clone(),
        input.timing,
        intent,
        0,
    )
    .unwrap();
    for observation in &history {
        adapter.observe(observation).unwrap();
    }
    assert_eq!(adapter.policy().continuation_event(), Some(178));
    assert_eq!(adapter.policy().first_stable_substep(), Some(47));
    assert_eq!(adapter.tracker().active.len(), 4);
    let before = adapter.evidence();
    let mut nonfoot = history.last().unwrap().clone();
    nonfoot.native_substep += 1;
    nonfoot.time_s += 1. / 240.;
    let segment = &mut nonfoot.segments[1];
    segment.role = SegmentRole::Hand;
    segment.loaded_substeps = 12;
    segment.persistent_load_n = 3.;
    segment.contacts.push(CurrentContact {
        local_point_m: [0.; 3],
        world_point_m: [0.; 3],
        upward_normal: 1.,
        current_upward_load_n: 3.,
    });
    let refusal = adapter.observe(&nonfoot).unwrap_err();
    assert!(refusal.contains("nonfoot"), "{refusal}");
    assert_eq!(adapter.evidence(), before);
    assert_eq!(adapter.policy().continuation_event(), Some(178));
    assert_eq!(adapter.last_observed, 178);
}

#[test]
fn pure_adapter_clone_does_not_share_mutable_command_or_observation_state() {
    let input = inputs();
    let original = adapter(&input);
    let before = original.evidence();
    let mut copy = original.clone();
    copy.observe(&input.observation).unwrap();
    let memory = memory(&input);
    copy.compile_batch(
        &input.model,
        &memory,
        &coefficient_problem(&input, &copy),
        36,
    )
    .unwrap();
    assert_eq!(original.evidence(), before);
    assert_ne!(copy.evidence(), before);
    assert_eq!(copy.evidence()["authorization"]["kind"], "unarmed");
}

#[test]
fn pure_terminal_deadline_accepts_success_at_25_seconds_and_binds_one_extra_second() {
    let origin = 36;
    let deadline = origin + 6000;
    let mut clock = TerminalClock::new(origin);
    assert!(clock.require_next_step(origin).is_err());
    clock.observe(deadline - 1, false).unwrap();
    clock.require_next_step(deadline - 1).unwrap();
    let before = json!(clock);
    assert!(clock.observe(deadline, false).is_err());
    assert_eq!(json!(clock), before);
    clock.observe(deadline, true).unwrap();
    assert_eq!(clock.first_success_at, Some(deadline));
    clock.require_next_step(deadline).unwrap();
    for offset in 1..240 {
        clock.observe(deadline + offset, true).unwrap();
        clock.require_next_step(deadline + offset).unwrap();
        assert_eq!(clock.first_success_at, Some(deadline));
    }
    clock.observe(deadline + 240, true).unwrap();
    assert!(clock.require_next_step(deadline + 240).is_err());
    let exhausted = json!(clock);
    assert!(clock.observe(deadline + 241, true).is_err());
    assert_eq!(json!(clock), exhausted);
}

#[test]
fn pure_terminal_success_cannot_be_late_rewound_lost_or_used_at_a_stale_clock() {
    let mut late = TerminalClock::new(100);
    let before = json!(late);
    assert!(late.observe(6101, true).is_err());
    assert_eq!(json!(late), before);
    assert!(late.first_success_at.is_none());
    let mut clock = TerminalClock::new(100);
    clock.observe(150, false).unwrap();
    clock.observe(151, true).unwrap();
    let successful = json!(clock);
    assert!(clock.observe(152, false).is_err());
    assert_eq!(json!(clock), successful);
    assert!(clock.observe(150, true).is_err());
    assert_eq!(json!(clock), successful);
    assert!(clock.require_next_step(150).is_err());
    assert!(clock.require_next_step(152).is_err());
    assert_eq!(json!(clock), successful);
    clock.observe(151, true).unwrap();
    assert_eq!(json!(clock), successful);
    clock.observe(391, true).unwrap();
    assert_eq!(clock.first_success_at, Some(151));
    clock.require_next_step(391).unwrap();
    assert!(!clock.completed(391).unwrap());
    clock.observe(392, true).unwrap();
    assert!(clock.completed(392).unwrap());
    assert!(clock.require_next_step(392).is_err());
    let exhausted = json!(clock);
    assert!(clock.observe(393, true).is_err());
    assert_eq!(json!(clock), exhausted);
}

#[test]
fn pure_terminal_clock_overflow_is_refused_before_any_state_change() {
    let mut bad_deadline = TerminalClock::new(u64::MAX - 5999);
    let before = json!(bad_deadline);
    assert!(bad_deadline.observe(u64::MAX - 5999, false).is_err());
    assert_eq!(json!(bad_deadline), before);
    let mut bad_tail = TerminalClock::new(u64::MAX - 6000);
    let before = json!(bad_tail);
    assert!(bad_tail.observe(u64::MAX - 100, true).is_err());
    assert_eq!(json!(bad_tail), before);
}

#[test]
fn pure_terminal_monitoring_rounds_up_to_the_first_complete_tick_only() {
    for first in 148..152 {
        let earliest = first + 240;
        let end = TerminalClock::monitoring_end(first).unwrap();
        assert_eq!(end % 4, 0);
        assert!((earliest..=earliest + 3).contains(&end));
        let mut clock = TerminalClock::new(0);
        clock.observe(first, true).unwrap();
        for at in first..end {
            clock.observe(at, true).unwrap();
            clock.require_next_step(at).unwrap();
            assert!(!clock.completed(at).unwrap());
        }
        clock.observe(end, true).unwrap();
        assert_eq!(clock.first_success_at, Some(first));
        assert!(clock.completed(end).unwrap());
        assert!(clock.require_next_step(end).is_err());
    }
}

fn synthetic_identity(tag: u8, at: u64) -> Vec<u8> {
    std::iter::once(tag).chain(at.to_le_bytes()).collect()
}

/// Test only the private payload validator and consumer. These bytes are
/// deliberately synthetic; no native receipt or real admission is produced.
fn synthetic_window(input: &Inputs, start: u64, steps: u32) -> NativeWindowData {
    let mut checkpoints: Vec<_> = (24..=steps)
        .step_by(24)
        .map(|offset| {
            let at = start + u64::from(offset);
            NativeCheckpoint {
                at,
                identity: synthetic_identity(2, at).into(),
            }
        })
        .collect();
    if steps % 24 != 0 {
        let at = start + u64::from(steps);
        checkpoints.push(NativeCheckpoint {
            at,
            identity: synthetic_identity(2, at).into(),
        });
    }
    NativeWindowData {
        records: (0..steps)
            .map(|offset| {
                let at = start + u64::from(offset);
                NativeRecord {
                    at,
                    identity: synthetic_identity(1, at).into(),
                    command: (offset < steps - 4).then(|| input.entry.command()),
                }
            })
            .collect(),
        horizon_steps: steps,
        checkpoints,
        entry_identity: synthetic_identity(2, start).into(),
    }
}

#[test]
fn pure_long_payload_requires_each_checkpoint_and_consumes_exactly_once_to_its_endpoint() {
    let input = inputs();
    for steps in [120, 124] {
        let mut adapter = adapter(&input);
        let start = 36;
        adapter
            .admit_window_data(
                synthetic_window(&input, start, steps),
                start,
                &synthetic_identity(2, start),
            )
            .unwrap();
        for offset in 0..steps {
            let at = start + u64::from(offset);
            let command = (offset < steps - 4).then(|| input.entry.command());
            adapter.terminal_clock.observe(at, false).unwrap();
            if offset > 0 && offset % 24 == 0 {
                let before = adapter.evidence();
                assert!(
                    adapter
                        .require_native_action(17, at, synthetic_identity(1, at), command.as_ref())
                        .is_err()
                );
                assert!(
                    adapter
                        .continue_validated_window(at, &synthetic_identity(3, at))
                        .is_err()
                );
                assert_eq!(adapter.evidence(), before);
                adapter
                    .continue_validated_window(at, &synthetic_identity(2, at))
                    .unwrap();
                assert_eq!(
                    adapter.evidence()["authorization"]["windowEnd"],
                    (offset + 24).min(steps)
                );
            }
            let before = adapter.evidence();
            assert!(
                adapter
                    .require_native_action(17, at, synthetic_identity(3, at), command.as_ref())
                    .is_err()
            );
            let changed_command = command
                .clone()
                .map(|mut c| {
                    c.velocities[0][0] = f32::from_bits(c.velocities[0][0].to_bits() ^ 1);
                    c
                })
                .or_else(|| Some(input.entry.command()));
            assert!(
                adapter
                    .require_native_action(
                        17,
                        at,
                        synthetic_identity(1, at),
                        changed_command.as_ref()
                    )
                    .is_err()
            );
            assert_eq!(adapter.evidence(), before);
            if let Some(command) = &command {
                adapter
                    .require_native_command(17, at, synthetic_identity(1, at), command)
                    .unwrap();
            } else {
                adapter
                    .require_native_passive(17, at, synthetic_identity(1, at))
                    .unwrap();
            }
            assert_eq!(adapter.evidence()["authorization"]["next"], offset + 1);
            let consumed = adapter.evidence();
            assert!(
                adapter
                    .require_native_action(17, at, synthetic_identity(1, at), command.as_ref())
                    .is_err()
            );
            assert_eq!(adapter.evidence(), consumed);
        }
        let end = start + u64::from(steps);
        adapter.terminal_clock.observe(end, false).unwrap();
        adapter
            .require_window_checkpoint(end, &synthetic_identity(2, end))
            .unwrap();
        let exhausted = adapter.evidence();
        assert!(
            adapter
                .require_window_checkpoint(end, &synthetic_identity(3, end))
                .is_err()
        );
        assert!(
            adapter
                .continue_validated_window(end, &synthetic_identity(2, end))
                .is_err()
        );
        assert!(
            adapter
                .require_native_passive(17, end, synthetic_identity(1, end))
                .is_err()
        );
        assert_eq!(adapter.evidence(), exhausted);
        assert_eq!(adapter.passive_since, Some(end - 4));
    }
}

#[test]
fn pure_malformed_long_payload_is_refused_without_arming_the_adapter() {
    let input = inputs();
    let mut adapter = adapter(&input);
    for case in 0..10 {
        let mut payload = synthetic_window(&input, 36, 124);
        match case {
            0 => payload.horizon_steps = 116,
            1 => payload.horizon_steps = 125,
            2 => payload.horizon_steps = 484,
            3 => {
                payload.records.pop();
            }
            4 => payload.records[61].at += 1,
            5 => {
                payload.checkpoints.pop();
            }
            6 => payload.checkpoints.swap(1, 2),
            7 => payload.checkpoints[1].at = payload.checkpoints[0].at,
            8 => payload.entry_identity = synthetic_identity(3, 36).into(),
            9 => payload.records[0].at -= 1,
            _ => unreachable!(),
        }
        let before = adapter.evidence();
        assert!(
            adapter
                .admit_window_data(payload, 36, &synthetic_identity(2, 36))
                .is_err(),
            "malformed payload case {case}"
        );
        assert_eq!(adapter.evidence(), before);
        assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
    }
}

#[test]
fn pure_long_payload_cannot_replace_a_partly_consumed_window() {
    let input = inputs();
    let mut adapter = adapter(&input);
    adapter
        .admit_window_data(
            synthetic_window(&input, 36, 120),
            36,
            &synthetic_identity(2, 36),
        )
        .unwrap();
    adapter.terminal_clock.observe(36, false).unwrap();
    adapter
        .require_native_command(17, 36, synthetic_identity(1, 36), &input.entry.command())
        .unwrap();
    let before = adapter.evidence();
    assert!(
        adapter
            .admit_window_data(
                synthetic_window(&input, 37, 120),
                37,
                &synthetic_identity(2, 37)
            )
            .is_err()
    );
    assert!(
        adapter
            .continue_validated_window(37, &synthetic_identity(2, 37))
            .is_err()
    );
    assert_eq!(adapter.evidence(), before);
}

#[test]
fn pure_archived_a36_coefficients_reproduce_all_four_original_command_bits() {
    let input = inputs();
    let mut adapter = adapter(&input);
    let entry: Value = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json"))).unwrap();
    let result: Value = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
        "/../../../docs/evidence/coordination-transfer-candidate-2026-10-13/archived-a36-command-result.json"))).unwrap();
    assert_eq!(
        result["status"],
        "pure-command-pass-native-admission-required"
    );
    let rates = |value: &Value| -> [f64; AXES] {
        assert_eq!(value.as_array().unwrap().len(), AXES);
        std::array::from_fn(|i| value[i].as_f64().unwrap())
    };
    let archived_memory = &entry["ownerBeforeConfigure36"];
    let mut memory = runtime::Memory {
        previous: serde_json::from_value(archived_memory["q"].clone()).unwrap(),
        next_substep: 36,
        rates: rates(&archived_memory["rates"]),
        mode_signature: 0,
    };
    let coefficients = &result["problem"];
    // Decode the original f64 coefficients verbatim. No body/Jacobian/FK or
    // task assembly is called: this is a replay of archived arithmetic only.
    let problem = Problem {
        lower: rates(&coefficients["lower"]),
        upper: rates(&coefficients["upper"]),
        regularization: coefficients["regularization"].as_f64().unwrap(),
        soft: coefficients["soft"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| Task {
                row: rates(&t["row"]),
                target: t["target"].as_f64().unwrap(),
                weight: t["weight"].as_f64().unwrap(),
            })
            .collect(),
        hard: coefficients["hard"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| HardTask {
                name: t["name"].as_str().unwrap().to_owned(),
                row: rates(&t["row"]),
                lower: t["lower"].as_f64().unwrap(),
                upper: t["upper"].as_f64().unwrap(),
            })
            .collect(),
    };
    adapter.observe(&input.observation).unwrap();
    let nominal = adapter.nominal(&input.model, &memory, 36).unwrap();
    let archived_nominal = Command {
        positions: serde_json::from_value(result["nominal"]["positions"].clone()).unwrap(),
        velocities: serde_json::from_value(result["nominal"]["velocities"].clone()).unwrap(),
    };
    assert_command_bits(&nominal, &archived_nominal);
    adapter
        .compile_batch(&input.model, &memory, &problem, 36)
        .unwrap();
    let archived_commands = result["commands"].as_array().unwrap();
    assert_eq!(archived_commands.len(), 4);
    for (offset, archived) in archived_commands.iter().enumerate() {
        let at = 36 + offset as u64;
        assert_eq!(archived["at"], at);
        let expected = Command {
            positions: serde_json::from_value(archived["positions"].clone()).unwrap(),
            velocities: serde_json::from_value(archived["velocities"].clone()).unwrap(),
        };
        let actual = adapter.proposed_command(&input.model, &memory, at).unwrap();
        assert_command_bits(&actual, &expected);
        adapter.commit_command(&mut memory, &actual, at).unwrap();
    }
    assert_eq!(memory.next_substep, 40);
    assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
}

fn synthetic_technical(input: &Inputs, start: u64) -> NativeTechnicalData {
    NativeTechnicalData {
        records: (0..4)
            .map(|offset| {
                let at = start + offset;
                NativeRecord {
                    at,
                    identity: synthetic_identity(1, at).into(),
                    command: (offset < 3).then(|| input.entry.command()),
                }
            })
            .collect(),
        entry_identity: synthetic_identity(2, start).into(),
        end_identity: synthetic_identity(2, start + 4).into(),
        instance: 17,
        scope: b"synthetic-candidate-scope-only".to_vec(),
    }
}

#[test]
fn pure_technical_admission_refuses_foreign_scope_instance_clock_or_payload_atomically() {
    let input = inputs();
    let mut adapter = adapter(&input);
    for case in 0..8 {
        let mut data = synthetic_technical(&input, 36);
        match case {
            0 => data.instance = 18,
            1 => data.scope.push(0),
            2 => data.entry_identity = synthetic_identity(3, 36).into(),
            3 => data.records[0].at = 35,
            4 => data.records[2].at = 39,
            5 => {
                data.records.pop();
            }
            6 => data.records.push(data.records[3].clone()),
            7 => data.records.swap(1, 2),
            _ => unreachable!(),
        }
        let before = adapter.evidence();
        assert!(
            adapter
                .admit_technical_data(
                    data,
                    17,
                    36,
                    &synthetic_identity(2, 36),
                    b"synthetic-candidate-scope-only"
                )
                .is_err(),
            "technical payload case {case}"
        );
        assert_eq!(adapter.evidence(), before);
        assert_eq!(adapter.evidence()["authorization"]["kind"], "unarmed");
    }
}

#[test]
fn pure_technical_four_steps_are_exact_bound_nonrenewable_and_require_the_endpoint() {
    let input = inputs();
    let mut adapter = adapter(&input);
    adapter
        .admit_technical_data(
            synthetic_technical(&input, 36),
            17,
            36,
            &synthetic_identity(2, 36),
            b"synthetic-candidate-scope-only",
        )
        .unwrap();
    assert_eq!(
        adapter.evidence()["authorization"]["kind"],
        "technical-four-only"
    );
    for at in 36..40 {
        adapter.terminal_clock.observe(at, false).unwrap();
        let command = (at < 39).then(|| input.entry.command());
        let before = adapter.evidence();
        assert!(
            adapter
                .require_native_action(18, at, synthetic_identity(1, at), command.as_ref())
                .is_err()
        );
        assert!(
            adapter
                .require_native_action(17, at + 1, synthetic_identity(1, at), command.as_ref())
                .is_err()
        );
        assert!(
            adapter
                .require_native_action(17, at, synthetic_identity(3, at), command.as_ref())
                .is_err()
        );
        let foreign = command
            .clone()
            .map(|mut c| {
                c.positions[0][0] = f32::from_bits(c.positions[0][0].to_bits() ^ 1);
                c
            })
            .or_else(|| Some(input.entry.command()));
        assert!(
            adapter
                .require_native_action(17, at, synthetic_identity(1, at), foreign.as_ref())
                .is_err()
        );
        assert!(
            adapter
                .require_window_checkpoint(40, &synthetic_identity(2, 40))
                .is_err()
        );
        assert!(
            adapter
                .admit_technical_data(
                    synthetic_technical(&input, at),
                    17,
                    at,
                    &synthetic_identity(2, at),
                    b"synthetic-candidate-scope-only"
                )
                .is_err()
        );
        assert_eq!(adapter.evidence(), before);
        if let Some(command) = &command {
            adapter
                .require_native_command(17, at, synthetic_identity(1, at), command)
                .unwrap();
        } else {
            adapter
                .require_native_passive(17, at, synthetic_identity(1, at))
                .unwrap();
        }
        assert_eq!(adapter.evidence()["authorization"]["next"], at - 35);
        let consumed = adapter.evidence();
        assert!(
            adapter
                .require_native_action(17, at, synthetic_identity(1, at), command.as_ref())
                .is_err()
        );
        assert_eq!(adapter.evidence(), consumed);
    }
    adapter.terminal_clock.observe(40, false).unwrap();
    adapter
        .require_window_checkpoint(40, &synthetic_identity(2, 40))
        .unwrap();
    let exhausted = adapter.evidence();
    assert!(
        adapter
            .require_window_checkpoint(40, &synthetic_identity(3, 40))
            .is_err()
    );
    assert!(
        adapter
            .require_window_checkpoint(41, &synthetic_identity(2, 40))
            .is_err()
    );
    assert!(
        adapter
            .require_native_passive(17, 40, synthetic_identity(1, 40))
            .is_err()
    );
    assert!(
        adapter
            .require_native_command(17, 40, synthetic_identity(1, 40), &input.entry.command())
            .is_err()
    );
    assert!(
        adapter
            .continue_validated_window(40, &synthetic_identity(2, 40))
            .is_err()
    );
    assert!(
        adapter
            .admit_technical_data(
                synthetic_technical(&input, 40),
                17,
                40,
                &synthetic_identity(2, 40),
                b"synthetic-candidate-scope-only"
            )
            .is_err()
    );
    assert!(
        adapter
            .admit_technical_data(
                synthetic_technical(&input, 36),
                17,
                36,
                &synthetic_identity(2, 36),
                b"synthetic-candidate-scope-only"
            )
            .is_err()
    );
    assert_eq!(adapter.evidence(), exhausted);
    assert_eq!(adapter.passive_since, Some(39));
}
