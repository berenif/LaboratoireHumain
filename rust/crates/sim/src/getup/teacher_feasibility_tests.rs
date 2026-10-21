//! Synthetic bookkeeping and read-only admission checks. These tests never run
//! an actor, integrate a world, or qualify a physical recovery.
use super::*;

fn feet_at(offset: f32) -> [Vec3; 2] {
    [
        Vec3 { x: offset - 0.1, y: 0.025, z: 0.2 },
        Vec3 { x: offset + 0.1, y: 0.025, z: 0.2 },
    ]
}

fn full_receipt(feet: [Vec3; 2], drift: Option<bool>) -> serde_json::Value {
    let mut predicates = serde_json::Map::new();
    for key in &STANDING_KEYS[..9] {
        predicates.insert((*key).into(), true.into());
    }
    predicates.insert(STANDING_KEYS[9].into(), serde_json::json!(drift));
    serde_json::json!({
        "predicates": predicates,
        "feet": feet,
        "footDriftM": drift.map(|passed| if passed { [0.0, 0.0] } else { [0.011, 0.0] }),
    })
}

fn primed_interval() -> StandingInterval {
    let mut interval = StandingInterval::default();
    interval.observe(full_receipt(feet_at(0.0), None)).unwrap();
    for _ in 0..2 {
        interval.observe(full_receipt(feet_at(0.0), Some(true))).unwrap();
    }
    interval
}

#[test]
fn teacher_feasibility_first_sample_starts_prospective_80_sample_interval() {
    assert_eq!(REQUIRED_NATIVE_SAMPLES, 80);
    assert_eq!(STANDING_KEYS.len(), 10);
    let feet = feet_at(0.0);
    let receipt = full_receipt(feet, None);
    assert_eq!(receipt["predicates"].as_object().unwrap().len(), 10);
    let mut interval = StandingInterval::default();
    let (current, passed, used) = interval.observe(receipt).unwrap();
    assert!(passed);
    assert_eq!(used, Some(feet));
    assert_eq!(interval.anchors, Some(feet));
    assert_eq!(interval.consecutive, 1);
    assert_eq!(interval.maximum, 1);
    assert_eq!(current["footDriftM"], serde_json::json!([0.0, 0.0]));
    assert_eq!(current["predicates"][STANDING_KEYS[9]], true);
    assert!(interval.consecutive < REQUIRED_NATIVE_SAMPLES);

    for expected in 2..=79 {
        let (_, passed, used) = interval.observe(full_receipt(feet, Some(true))).unwrap();
        assert!(passed);
        assert_eq!(used, Some(feet));
        assert_eq!(interval.anchors, Some(feet));
        assert_eq!(interval.consecutive, expected);
        assert!(interval.consecutive < REQUIRED_NATIVE_SAMPLES);
    }
    let (_, passed, used) = interval.observe(full_receipt(feet, Some(true))).unwrap();
    assert!(passed);
    assert_eq!(used, Some(feet));
    assert_eq!(interval.consecutive, 80);
    assert_eq!(interval.maximum, 80);
    assert!(interval.consecutive >= REQUIRED_NATIVE_SAMPLES);
}

#[test]
fn teacher_feasibility_each_of_ten_predicates_resets_without_reanchoring_failure() {
    for key in STANDING_KEYS {
        let mut interval = primed_interval();
        let prior = interval.anchors;
        let failing_feet = feet_at(1.0);
        let mut receipt = full_receipt(failing_feet, Some(true));
        receipt["predicates"][key] = false.into();
        if key == STANDING_KEYS[9] {
            receipt["footDriftM"] = serde_json::json!([0.011, 0.0]);
        }
        let (current, passed, used) = interval.observe(receipt).unwrap();
        assert!(!passed, "failed predicate: {key}");
        assert_eq!(current["predicates"][key], false, "{key}");
        assert_eq!(used, prior, "failing sample uses the prior anchors: {key}");
        assert_eq!(interval.anchors, None, "{key}");
        assert_eq!(interval.consecutive, 0, "{key}");
        assert_eq!(interval.maximum, 3, "{key}");

        let next_feet = feet_at(2.0);
        let (_, passed, used) = interval.observe(full_receipt(next_feet, None)).unwrap();
        assert!(passed, "{key}");
        assert_eq!(used, Some(next_feet), "{key}");
        assert_eq!(interval.anchors, Some(next_feet), "{key}");
        assert_eq!(interval.consecutive, 1, "{key}");
        assert_eq!(interval.maximum, 3, "{key}");
    }
}

#[test]
fn teacher_feasibility_base_failure_cannot_seed_first_sample_anchors() {
    for key in &STANDING_KEYS[..9] {
        let mut interval = StandingInterval::default();
        let mut receipt = full_receipt(feet_at(1.0), None);
        receipt["predicates"][*key] = false.into();
        let (_, passed, used) = interval.observe(receipt).unwrap();
        assert!(!passed, "{key}");
        assert_eq!(used, None, "{key}");
        assert_eq!(interval.anchors, None, "{key}");
        assert_eq!(interval.consecutive, 0, "{key}");
        assert_eq!(interval.maximum, 0, "{key}");
    }
}

fn assert_malformed_receipt_refused(receipt: serde_json::Value) {
    let mut interval = primed_interval();
    let anchors = interval.anchors;
    assert!(interval.observe(receipt).is_err());
    assert_eq!(interval.anchors, anchors);
    assert_eq!(interval.consecutive, 3);
    assert_eq!(interval.maximum, 3);
}

#[test]
fn teacher_feasibility_missing_null_and_unknown_predicates_refuse_receipt() {
    let valid = full_receipt(feet_at(0.0), Some(true));
    for key in STANDING_KEYS {
        let mut missing = valid.clone();
        missing["predicates"].as_object_mut().unwrap().remove(key);
        assert_malformed_receipt_refused(missing);

        let mut null = valid.clone();
        null["predicates"][key] = serde_json::Value::Null;
        assert_malformed_receipt_refused(null);

        for wrong in [serde_json::json!("true"), serde_json::json!(1), serde_json::json!([])] {
            let mut receipt = valid.clone();
            receipt["predicates"][key] = wrong;
            assert_malformed_receipt_refused(receipt);
        }
    }
    let mut extra = valid.clone();
    extra["predicates"]["unknownStandingPredicate"] = true.into();
    assert_malformed_receipt_refused(extra);
    let mut replacement = valid.clone();
    replacement["predicates"].as_object_mut().unwrap().remove(STANDING_KEYS[0]);
    replacement["predicates"]["unknownStandingPredicate"] = true.into();
    assert_eq!(replacement["predicates"].as_object().unwrap().len(), 10);
    assert_malformed_receipt_refused(replacement);
    let mut null_object = valid.clone();
    null_object["predicates"] = serde_json::Value::Null;
    assert_malformed_receipt_refused(null_object);
    let mut missing_object = valid;
    missing_object.as_object_mut().unwrap().remove("predicates");
    assert_malformed_receipt_refused(missing_object);
}

#[test]
fn teacher_feasibility_unanchored_null_is_specific_to_the_drift_predicate() {
    for key in &STANDING_KEYS[..9] {
        let mut receipt = full_receipt(feet_at(0.0), None);
        receipt["predicates"][*key] = serde_json::Value::Null;
        assert!(StandingInterval::default().observe(receipt).is_err(), "{key}");
    }
    let mut receipt = full_receipt(feet_at(0.0), None);
    receipt["predicates"][STANDING_KEYS[9]] = serde_json::json!("unknown");
    assert!(StandingInterval::default().observe(receipt).is_err());
}

#[test]
fn teacher_feasibility_supplied_native_drift_failure_is_never_weakened() {
    // Distance arithmetic and the exact 10 mm boundary belong to the existing
    // standing_diagnostic. This wrapper must honor its supplied false result.
    let mut interval = primed_interval();
    let prior = interval.anchors;
    let receipt = full_receipt(feet_at(0.011), Some(false));
    let (current, passed, used) = interval.observe(receipt).unwrap();
    assert!(!passed);
    assert_eq!(used, prior);
    assert_eq!(current["footDriftM"], serde_json::json!([0.011, 0.0]));
    assert_eq!(current["predicates"][STANDING_KEYS[9]], false);
    assert_eq!(interval.anchors, None);
    assert_eq!(interval.consecutive, 0);
}

#[test]
fn teacher_feasibility_support_clock_includes_three_newtons_and_rejects_invalid_loads() {
    let minimum = recovery_contract::contract().recovery_acceptance.minimum_load_n;
    assert_eq!(minimum, 3.0);
    assert_eq!(next_support_persistence(39, 3.0, minimum).unwrap(), 40);
    assert_eq!(next_support_persistence(39, 2.999, minimum).unwrap(), 0);
    assert_eq!(next_support_persistence(39, 0.0, minimum).unwrap(), 0);
    for load in [f32::NAN, -0.001, f32::NEG_INFINITY, f32::INFINITY] {
        assert!(next_support_persistence(39, load, minimum).is_err());
    }
    for invalid_minimum in [0.0, -3.0, f32::NAN, f32::INFINITY] {
        assert!(next_support_persistence(39, 3.0, invalid_minimum).is_err());
    }
    assert!(next_support_persistence(u64::MAX, 3.0, minimum).is_err());
}

#[test]
fn teacher_feasibility_refresh_overwrites_all_25_measurements_without_world_or_snapshot_change() {
    let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
    assert_eq!(s.substeps_completed, 0);
    assert_eq!(s.colliders.len(), 25);
    assert_eq!(s.world.narrow_phase.contact_pairs().count(), 0);
    s.integrated_support_loads_n = std::array::from_fn(|i| i as f32 + 10.0);
    s.integrated_support_substeps = std::array::from_fn(|i| i as u64 + 39);
    let before_world = bincode::serialize(&s.world).unwrap();
    let before_snapshot = serde_json::to_value(s.snapshot()).unwrap();
    refresh_current_support(&mut s).unwrap();
    assert_eq!(s.integrated_support_loads_n, [0.0; 25]);
    assert_eq!(s.integrated_support_substeps, [0; 25]);
    assert_eq!(s.substeps_completed, 0);
    assert_eq!(bincode::serialize(&s.world).unwrap(), before_world);
    assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before_snapshot);
}

#[test]
fn teacher_feasibility_original_height_cut_is_strict_and_keeps_one_extra_complete_frame() {
    let mut finish = false;
    let mut cut = None;
    for (index, head) in [1.3_f32, 1.4, 1.400001, 1.0, 2.0].into_iter().enumerate() {
        let decisions = index + 1;
        original_cut_after_frame(&mut finish, &mut cut, decisions, head);
        match decisions {
            1 | 2 => { assert!(!finish); assert_eq!(cut, None); }
            3 => { assert!(finish); assert_eq!(cut, None); }
            _ => assert_eq!(cut, Some(4)),
        }
    }
}

#[test]
fn teacher_feasibility_total_ceiling_is_250_twenty_native_frames_and_never_cuts_prefix() {
    assert_eq!(MAX_TEACHER_TICKS, 250);
    let native_per_policy = policy_substeps(1.0 / 800.0).unwrap();
    assert_eq!(native_per_policy, 20);
    assert_eq!(MAX_TEACHER_TICKS * native_per_policy as usize, 5000);
    for crossing in [1, 2, 17, 249, 250] {
        let mut finish = false;
        let mut cut = None;
        let mut complete_frames = Vec::new();
        for decisions in 1..=MAX_TEACHER_TICKS {
            // Synthetic archive words stand in for a whole completed frame.
            complete_frames.push(vec![decisions; native_per_policy as usize]);
            original_cut_after_frame(&mut finish, &mut cut, decisions,
                if decisions == crossing { 1.5 } else { 1.0 });
            if let Some(prefix) = cut {
                assert_eq!(prefix, crossing + 1);
                assert_eq!(complete_frames.len(), decisions);
                assert!(complete_frames[..prefix].iter().enumerate().all(|(i, frame)|
                    frame == &vec![i + 1; native_per_policy as usize]));
            }
        }
        assert_eq!(complete_frames.len(), 250);
        assert_eq!(complete_frames.iter().map(Vec::len).sum::<usize>(), 5000);
        assert_eq!(cut, if crossing < MAX_TEACHER_TICKS { Some(crossing + 1) } else { None });
    }
}

fn synthetic_untouched_controller() -> Controller {
    // Controller::new initializes the actors, so construct only the inert
    // bookkeeping needed by the freshness gate instead.
    Controller {
        physical_roll: None, adaptation: None, supported_transfer: None,
        status: Default::default(), started_substep: None, impact_substep: None,
        reference: Vec::new(), teacher_frames: Vec::new(), teacher_initial: None,
        teacher_native_dt_s: None, teacher_failure: None,
        decision_observation: Vec::new(), student_decisions: Vec::new(),
        residual: [0.0; 21], reference_pose: [0.0; 21], holding: true,
        last_decision: None, stable_substeps: 0, stable_feet: None,
        terminal_balance: None, terminal_base_ready: None,
    }
}

#[test]
fn teacher_feasibility_invalid_fresh_state_rejects_before_fork_or_actor_without_mutation() {
    let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
    s.getup_controller = Some(synthetic_untouched_controller());
    s.getup_fixture = Some(crate::getup::fixtures::State {
        kind: "back".into(), preparing: true, ready: false,
        settled_substeps: 0, trigger_substep: None,
        maximum_body_com_speed_mps: 0.0, maximum_angular_radps: 0.0,
        ground_load_n: 0.0, torso_long_axis_up: 0.0, torso_face_up: 1.0,
        torso_side_up: 0.0, head_height_m: 0.2,
    });
    // A fork would reject this floor. The earlier freshness error must win.
    s.world.colliders[s.floor].set_enabled(false);
    let before_simulation = bincode::serialize(&s).unwrap();
    let before_world = bincode::serialize(&s.world).unwrap();
    let before_snapshot = serde_json::to_value(s.snapshot()).unwrap();
    let error = probe(&s).unwrap_err();
    assert!(error.contains("fresh triggered BACK at native480"), "{error}");
    assert_eq!(s.substeps_completed, 0);
    assert_eq!(bincode::serialize(&s).unwrap(), before_simulation);
    assert_eq!(bincode::serialize(&s.world).unwrap(), before_world);
    assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before_snapshot);
}
