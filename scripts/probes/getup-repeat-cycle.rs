#![recursion_limit = "256"]
//! Actual apparatus continuation only. Link against the frozen repeat-cycle
//! library. This program never prepares a fixture, morphs a profile, resets a
//! controller, or writes an anatomical pose, velocity, or integrator history.
use lh_sim::{getup::controller::standing_diagnostic, measurement::Failure, runtime::Runtime};
use serde_json::{json, Value};
use std::{
    collections::VecDeque,
    fs::{self, File, OpenOptions},
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};

const HZ: u64 = 800;
const MAX_CYCLES: u64 = 5;
const NO_CONTACT_WINDOW: u64 = 5 * HZ;
const ORIGINAL_DEADLINE: u64 = 25 * HZ;
const DIAGNOSTIC_WINDOW: u64 = 30 * HZ;
const REQUIRED_HOLD: u64 = HZ;
const RESISTED_RESOLUTION_AGE: u64 = ORIGINAL_DEADLINE + 20;
const SLIP_LIMIT_M: f64 = 0.020;

fn save(out: &Path, name: &str, value: &Value) {
    let mut file = File::create(out.join(name)).unwrap();
    serde_json::to_writer_pretty(&mut file, value).unwrap();
    writeln!(file).unwrap();
    file.sync_all().unwrap();
}

// Physical payloads are f32. Restore their original words from either typed
// serde_json::to_value (expanded f64) or shortest-f32 textual JSON. Comparison
// is exact in all words, including signed zero; there is no numeric tolerance.
fn f32_word(value: &Value) -> u32 {
    let restored = value.as_f64().expect("numeric physical f32") as f32;
    assert!(restored.is_finite(), "finite physical f32");
    restored.to_bits()
}
fn anchor_words(value: &Value) -> [[u32; 3]; 2] {
    assert_eq!(value.as_array().expect("two physical anchors").len(), 2);
    std::array::from_fn(|i| ["x", "y", "z"].map(|k| f32_word(&value[i][k])))
}
fn segment_words(value: &Value) -> Vec<(String, [u32; 13])> {
    value.as_array().expect("physical pose array").iter().map(|pose| {
        let mut words = [0_u32; 13];
        let mut n = 0;
        for (field, components) in [("position", &['x','y','z'][..]),
            ("rotation", &['x','y','z','w'][..]),
            ("linearVelocity", &['x','y','z'][..]),
            ("angularVelocity", &['x','y','z'][..])]
        {
            for component in components {
                words[n] = f32_word(&pose[field][component.to_string()]); n += 1;
            }
        }
        assert_eq!(n, 13);
        (pose["id"].as_str().unwrap().to_owned(), words)
    }).collect()
}

// Preserve integer clocks and boolean/optional structure. Physical floating
// payloads in this retained V1 binding are f32 and compare as original words.
fn binding_words(value: &Value) -> Value {
    match value {
        Value::Number(number) if number.is_f64() => json!({"f32Word":f32_word(value)}),
        Value::Array(values) => Value::Array(values.iter().map(binding_words).collect()),
        Value::Object(values) => Value::Object(values.iter()
            .map(|(key, value)| (key.clone(), binding_words(value))).collect()),
        _ => value.clone(),
    }
}

fn typed_profile(value: &Value) -> Value {
    serde_json::to_value(serde_json::from_value::<lh_sim::Profile>(value.clone()).unwrap()).unwrap()
}

fn fixture(runtime: &Runtime) -> Value {
    serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap()
}

fn controller(runtime: &Runtime) -> Value {
    serde_json::from_str(&runtime.getup_report().unwrap()).unwrap()
}

fn native(runtime: &Runtime) -> u64 {
    fixture(runtime)["nativeSubsteps"].as_u64().unwrap()
}

fn save_checkpoint(runtime: &Runtime, out: &Path, label: &str) -> Value {
    let before_native = native(runtime);
    let before_controller = runtime.getup_report().unwrap();
    let result = match runtime.getup_checkpoint() {
        Ok(bytes) => {
            let name = format!("checkpoint-{label}.bin");
            let mut file = File::create(out.join(&name)).unwrap();
            file.write_all(&bytes).unwrap();
            file.sync_all().unwrap();
            match runtime.verify_getup_checkpoint(&bytes) {
                Ok(report) => {
                    let verification: Value = serde_json::from_str(&report).unwrap();
                    let unchanged = before_native == native(runtime)
                        && before_controller == runtime.getup_report().unwrap();
                    let passed = verification["passed"] == true
                        && verification["physicsIntegrations"] == 0 && unchanged;
                    json!({"saved":true,"file":name,"verifiedZeroIntegration":passed,
                        "nativeBefore":before_native,"nativeAfter":native(runtime),
                        "liveControllerUnchanged":unchanged,"verification":verification})
                }
                Err(error) => json!({"saved":true,"file":name,
                    "verifiedZeroIntegration":false,"verificationError":error}),
            }
        }
        Err(error) => json!({"saved":false,"verifiedZeroIntegration":false,
            "exportError":error,"native":before_native,
            "note":"The exact failing state was not replaced with an earlier checkpoint."}),
    };
    save(out, &format!("checkpoint-{label}-verification.json"), &result);
    result
}

#[derive(Default)]
struct ForecastAccounting {
    previous: u64,
    added: u64,
    resets: Vec<Value>,
    ambiguous_reset: bool,
}

impl ForecastAccounting {
    fn observe(&mut self, current: u64, at_native: u64, source: &str) {
        if current >= self.previous {
            self.added += current - self.previous;
        } else {
            // The production rearm is observed after its controller-only reset
            // and before the next source command, so its counter must be zero.
            // A nonzero downward jump is reported as ambiguous, never silently
            // turned into a negative final-counter subtraction or a pass.
            self.ambiguous_reset |= current != 0;
            self.resets.push(json!({"native":at_native,"previous":self.previous,
                "current":current,"source":source,"resetToZeroObserved":current==0}));
        }
        self.previous = current;
    }
}

fn stable_count(seconds: f64) -> u64 {
    (seconds * HZ as f64).round() as u64
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "repeated-actual-striker FRESH_RECOVERED_CHECKPOINT NEW_OUTPUT_DIRECTORY");
    let out = Path::new(&args[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new().write(true).create_new(true).open(out.join("started-once")).unwrap();
    let began = Instant::now();
    let input = fs::read(&args[1]).unwrap();
    let mut runtime = Runtime::from_getup_checkpoint(&input).unwrap();
    let input_verification: Value = serde_json::from_str(
        &runtime.verify_getup_checkpoint(&input).unwrap()).unwrap();
    assert_eq!(input_verification["physicsIntegrations"], 0);
    assert_eq!(input_verification["passed"], true);
    save(out, "input-verification.json", &input_verification);
    let expected_profile = typed_profile(&serde_json::from_str(include_str!(
        "../../docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json")).unwrap());
    let input_fixture = fixture(&runtime);
    assert_eq!(typed_profile(&input_fixture["profile"]), expected_profile,
        "Input must have the complete fresh explicit repeat-cycle profile; never convert a legacy checkpoint");
    let initial_controller = controller(&runtime);
    assert_eq!(initial_controller["status"]["prototype_standing_passed"], true);
    assert!(initial_controller["terminal_balance"].is_object());
    assert!(initial_controller["stable_feet"].is_array());
    assert!(initial_controller["stable_substeps"].as_u64().unwrap() >= 800 + 3 * HZ,
        "Input must retain the first 1s standing pass plus at least 3s uninterrupted physical hold");
    let initial_native = native(&runtime);
    let mut reply = runtime.advance(0).unwrap();
    assert_eq!(reply.completed_ticks, 0);
    assert_eq!(native(&runtime), initial_native);
    assert!(reply.failure.is_none());
    assert_eq!(json!(reply.snapshot.motion), "Upright");
    assert_eq!(reply.snapshot.counters.recoveries, 1,
        "The parent must supply the newly fresh first-recovery checkpoint");
    let initial_snapshot = reply.snapshot.clone();
    save(out, "initial-controller.json", &initial_controller);
    save(out, "initial.json", &json!({"reply":reply,"fixture":input_fixture,
        "inputProvenanceRequirement":"Newly fresh explicit-profile first recovery followed by 3 physical seconds of hold; supplied by parent correctness-gated run",
        "importAndInitialBoundaryIntegrations":0,"profileChangedAtImport":false}));
    let initial_checkpoint = save_checkpoint(&runtime, out, "initial");
    assert_eq!(initial_checkpoint["verifiedZeroIntegration"], true);

    let mut forecast = ForecastAccounting {
        previous: initial_controller["status"]["forecast_substeps"].as_u64().unwrap(),
        ..Default::default()
    };
    let mut observed_total = 0_u64;
    let mut cycles = Vec::<Value>::new();
    let mut final_outcome = "all-five-actual-strike-responses-valid".to_string();

    for cycle in 1..=MAX_CYCLES {
        let cycle_dir = out.join(format!("cycle-{cycle:02}"));
        fs::create_dir(&cycle_dir).unwrap();
        let cycle_began = Instant::now();
        let before_controller = controller(&runtime);
        let before_snapshot = reply.snapshot.clone();
        let before_native = native(&runtime);
        let before_forecast = forecast.added;
        let before_resets = forecast.resets.len();
        let old_start = before_controller["started_substep"].as_u64().unwrap();
        let old_forecast = before_controller["status"]["forecast_substeps"].as_u64().unwrap();
        let before_binding_words = binding_words(&before_controller["terminal_balance"]);
        let baseline_strikes = before_snapshot.counters.strikes;
        let baseline_falls = before_snapshot.counters.falls;
        let baseline_recoveries = before_snapshot.counters.recoveries;
        save(&cycle_dir, "controller-before.json", &before_controller);
        save(&cycle_dir, "snapshot-before.json", &json!(before_snapshot));
        let before_checkpoint = save_checkpoint(&runtime, &cycle_dir, "before");
        let mut outcome = "pending".to_string();
        let mut ack_record = json!({"accepted":false});
        let mut ledger = BufWriter::new(File::create(cycle_dir.join("events-800hz.ndjson")).unwrap());
        let mut trace = BufWriter::new(File::create(cycle_dir.join("trace-40hz.ndjson")).unwrap());
        let mut observed = 0_u64;
        let mut last_observed_native = before_native;
        let mut contact_native = None::<u64>;
        let mut contact_event = None::<Value>;
        let mut first_new_fall = None::<Value>;
        let mut first_new_source = None::<Value>;
        let mut first_pass_native = None::<u64>;
        let mut first_pass_record = None::<Value>;
        let mut original_anchors = None;
        let mut anchors_json = Value::Null;
        let mut stable_at_pass = 0_u64;
        let mut postpass_samples = 0_u64;
        let mut all_postpass_predicates = true;
        let mut counter_continuous = true;
        let mut getup_slip = [0.0_f64; 2];
        let mut postpass_slip = [0.0_f64; 2];
        let mut maximum_contact_impulse = 0.0_f64;
        let mut gate_failure = None::<Value>;
        // A resisted response can briefly lose the standing predicate without
        // becoming a physical fall. Track the actual contiguous standing
        // interval's anchors and retain every one of its last 800 samples.
        let mut resisted_anchors = Some(serde_json::from_value(
            before_controller["stable_feet"].clone()).unwrap());
        let mut resisted_anchor_json = before_controller["stable_feet"].clone();
        let mut previous_stable = before_controller["stable_substeps"].as_u64().unwrap();
        let mut resisted_lifecycle_unchanged = true;
        let mut last_800 = VecDeque::<Value>::new();
        let mut rolling_800_slip = [0.0_f64; 2];
        let mut maximum_rolling_800_slip = [0.0_f64; 2];
        let mut resisted_resolution = None::<Value>;

        if before_checkpoint["verifiedZeroIntegration"] != true {
            outcome = "before-checkpoint-verification-failed".into();
        } else if !before_snapshot.striker.as_ref().is_some_and(|s| s.available) {
            outcome = "apparatus-not-available-at-cycle-boundary".into();
        } else {
            // Use this exact current reply stamp, and acknowledged sequence+1.
            // The public command merely moves the existing 4m/s apparatus.
            let stamp = before_snapshot.stamp;
            let sequence = before_snapshot.acknowledged_sequence.checked_add(1).unwrap();
            let command = serde_json::from_value(json!({"stamp":{
                "schema":stamp.schema,"generation":stamp.generation,
                "sequence":sequence,"tick":stamp.tick},"action":"Strike"})).unwrap();
            match runtime.enqueue(command) {
                Err(rejection) => {
                    outcome = "strike-enqueue-rejected".into();
                    ack_record = json!({"accepted":false,"enqueueRejection":rejection});
                }
                Ok(()) => {
                    reply = runtime.advance(0).unwrap();
                    let unchanged_segments = segment_words(&json!(reply.snapshot.segments))
                        == segment_words(&json!(before_snapshot.segments));
                    let accepted = reply.completed_ticks == 0 && native(&runtime) == before_native
                        && reply.acknowledgements.len() == 1
                        && reply.acknowledgements[0].rejected.is_none()
                        && reply.acknowledgements[0].stamp.sequence == sequence
                        && reply.acknowledgements[0].applied_tick == Some(stamp.tick)
                        && reply.snapshot.acknowledged_sequence == sequence
                        && unchanged_segments;
                    ack_record = json!({"accepted":accepted,"reply":reply,
                        "nativeBefore":before_native,"nativeAfter":native(&runtime),
                        "anatomicalSegmentsExactlyUnchanged":unchanged_segments,
                        "commandStamp":stamp,"commandSequence":sequence});
                    if !accepted { outcome = "strike-boundary-contract-failed".into(); }
                }
            }
        }
        save(&cycle_dir, "strike-ack.json", &ack_record);

        while outcome == "pending" {
            let advance = runtime.advance_observed(1, |s, substep| {
                let snapshot = s.snapshot();
                let policy = snapshot.diagnostics.getup.as_ref().unwrap().policy.as_ref().unwrap();
                let diagnostic = standing_diagnostic(s, None).map_err(|error|
                    Failure::new(s.tick, substep, "repeat-diagnostic-observer", error, 1.0, 0.0))?;
                let at_native = diagnostic["nativeSubsteps"].as_u64().unwrap();
                observed += 1;
                observed_total += 1;
                last_observed_native = at_native;
                forecast.observe(policy.forecast_substeps as u64, at_native, "800Hz observer");
                let source_start = policy.started_at_s.map(|seconds| stable_count(seconds));
                let stable = stable_count(policy.stable_standing_s);
                let mut current_error = None::<String>;
                if at_native != before_native + observed {
                    current_error = Some("native-observer-coverage-discontinuity".into());
                }
                if snapshot.counters.strikes > baseline_strikes {
                    if let Some(striker) = snapshot.striker.as_ref() {
                        maximum_contact_impulse = maximum_contact_impulse.max(striker.last_impact_impulse_ns as f64);
                        if contact_native.is_none() {
                            contact_native = Some(at_native);
                            contact_event = Some(json!({"native":at_native,"cycleNative":observed,
                                "impulseNs":striker.last_impact_impulse_ns,"striker":striker,
                                "motion":snapshot.motion,"counters":snapshot.counters,"policy":policy}));
                            save(&cycle_dir, "actual-contact.json", contact_event.as_ref().unwrap());
                            if striker.last_impact_impulse_ns <= 0.0 {
                                current_error = Some("counter-increment-without-positive-real-contact-impulse".into());
                            }
                        }
                    }
                }
                if first_new_fall.is_none() && snapshot.counters.falls > baseline_falls {
                    first_new_fall = Some(json!({"native":at_native,"motion":snapshot.motion,
                        "counters":snapshot.counters,"policy":policy}));
                }
                if first_new_source.is_none() && source_start.is_some_and(|start| start > old_start) {
                    first_new_source = Some(json!({"native":at_native,"sourceStartNative":source_start,
                        "policy":policy,"newForecastIntegrationsSoFar":forecast.added-before_forecast}));
                }
                let mut slip_increment = [0.0_f64; 2];
                for foot in 0..2 {
                    let contact = &diagnostic["contactMotion"]["feet"][foot];
                    if contact["loaded"] == true {
                        if let Some(speed) = contact["impulseWeightedTangentSpeedMps"].as_f64() {
                            if !speed.is_finite() || speed < 0.0 {
                                current_error = Some("invalid-loaded-contact-tangent-speed".into());
                            } else {
                                slip_increment[foot] = speed / HZ as f64;
                                getup_slip[foot] += slip_increment[foot];
                            }
                        }
                    }
                }

                if stable == 0 {
                    resisted_anchors = None;
                    resisted_anchor_json = Value::Null;
                } else if stable == 1 || resisted_anchors.is_none() {
                    // Controller inserts these exact integrated foot origins
                    // on the first stable sample. Verify this tracked anchor
                    // against its actual private state at final resolution.
                    resisted_anchor_json = diagnostic["feet"].clone();
                    resisted_anchors = Some(serde_json::from_value(
                        resisted_anchor_json.clone()).unwrap());
                }
                let resisted_diagnostic = if let Some(anchors) = resisted_anchors {
                    standing_diagnostic(s, Some(anchors)).map_err(|error|
                        Failure::new(s.tick, substep, "resisted-standing-observer", error, 1.0, 0.0))?
                } else { Value::Null };
                let resisted_predicates_pass = resisted_diagnostic["predicates"].as_object()
                    .is_some_and(|predicates| predicates.values().all(|value| value == true));
                let unchanged_lifecycle = snapshot.counters.falls == baseline_falls
                    && snapshot.counters.recoveries == baseline_recoveries
                    && source_start == Some(old_start)
                    && policy.forecast_substeps as u64 == old_forecast
                    && forecast.added == before_forecast && forecast.resets.len() == before_resets
                    && policy.prototype_standing_passed && policy.refusal.is_none();
                resisted_lifecycle_unchanged &= unchanged_lifecycle;
                let resisted_counter_continuous = stable == previous_stable + 1
                    && unchanged_lifecycle
                    && snapshot.counters.strikes == baseline_strikes + 1;
                previous_stable = stable;
                let anchor_word_state = if resisted_anchor_json.is_array() {
                    json!(anchor_words(&resisted_anchor_json))
                } else { Value::Null };
                last_800.push_back(json!({"native":at_native,"stableSubsteps":stable,
                    "predicates":resisted_diagnostic["predicates"],
                    "allOriginalPredicatesPassed":resisted_predicates_pass,
                    "originalCounterContinuous":resisted_counter_continuous,
                    "sourceStartNative":source_start,"counters":snapshot.counters,
                    "anchorF32Words":anchor_word_state,"loadedTangentIntegralIncrementM":slip_increment}));
                for foot in 0..2 { rolling_800_slip[foot] += slip_increment[foot]; }
                if last_800.len() > HZ as usize {
                    let removed = last_800.pop_front().unwrap();
                    for foot in 0..2 {
                        rolling_800_slip[foot] -= removed["loadedTangentIntegralIncrementM"][foot].as_f64().unwrap();
                    }
                }
                if last_800.len() == HZ as usize {
                    for foot in 0..2 {
                        maximum_rolling_800_slip[foot] = maximum_rolling_800_slip[foot].max(rolling_800_slip[foot]);
                    }
                }

                if first_pass_native.is_none()
                    && snapshot.counters.recoveries > baseline_recoveries
                    && policy.prototype_standing_passed && stable >= HZ
                {
                    // This is the genuine new controller's original acceptance
                    // event, not an old latch or an independently seeded flag.
                    if contact_native.is_none() || first_new_fall.is_none()
                        || first_new_source.is_none()
                        || snapshot.counters.recoveries != baseline_recoveries + 1
                        || source_start.is_none_or(|start| start <= old_start)
                        || forecast.added == before_forecast
                    {
                        current_error = Some("new-recovery-provenance-contract-failed".into());
                    }
                    if contact_native.is_some_and(|contact| at_native - contact > ORIGINAL_DEADLINE) {
                        current_error = Some("new-pass-outside-original-25-second-contact-deadline".into());
                    }
                    let state = serde_json::to_value(s).unwrap();
                    let actual_controller = &state["getup_controller"];
                    anchors_json = actual_controller["stable_feet"].clone();
                    let anchors = serde_json::from_value(anchors_json.clone()).map_err(|error|
                        Failure::new(s.tick, substep, "repeat-pass-anchors", error.to_string(), 1.0, 0.0))?;
                    let original_diagnostic = standing_diagnostic(s, Some(anchors)).map_err(|error|
                        Failure::new(s.tick, substep, "repeat-pass-standing", error, 1.0, 0.0))?;
                    if actual_controller["impact_substep"].as_u64() != contact_native {
                        current_error = Some("new-controller-did-not-retain-real-contact-epoch".into());
                    }
                    if actual_controller["stable_substeps"].as_u64() != Some(stable) {
                        current_error = Some("original-controller-counter-differs-from-policy-observation".into());
                    }
                    original_anchors = Some(anchors);
                    first_pass_native = Some(at_native);
                    stable_at_pass = stable;
                    first_pass_record = Some(json!({"native":at_native,"snapshot":snapshot,
                        "controller":actual_controller,"originalStandingDiagnostic":original_diagnostic,
                        "wholeGetupLoadedSlipMReportedNotGated":getup_slip,
                        "postpassLoadedSlipIntegralStartsAtNative":at_native,
                        "postpassLoadedSlipM":[0.0,0.0]}));
                    save(&cycle_dir, "new-first-pass.json", first_pass_record.as_ref().unwrap());
                }

                let mut original_diagnostic = Value::Null;
                if let (Some(pass), Some(anchors)) = (first_pass_native, original_anchors) {
                    original_diagnostic = standing_diagnostic(s, Some(anchors)).map_err(|error|
                        Failure::new(s.tick, substep, "repeat-hold-standing", error, 1.0, 0.0))?;
                    let predicates_pass = original_diagnostic["predicates"].as_object().unwrap()
                        .values().all(|value| value == true);
                    all_postpass_predicates &= predicates_pass;
                    let continuous = stable == stable_at_pass + (at_native - pass)
                        && snapshot.counters.recoveries == baseline_recoveries + 1
                        && snapshot.counters.falls == baseline_falls + 1
                        && snapshot.counters.strikes == baseline_strikes + 1
                        && policy.prototype_standing_passed;
                    counter_continuous &= continuous;
                    if !predicates_pass { current_error = Some("postpass-original-standing-predicate-failed".into()); }
                    if !continuous { current_error = Some("postpass-original-counter-continuity-failed".into()); }
                    postpass_samples = at_native - pass;
                    if at_native > pass {
                        for foot in 0..2 {
                            let contact = &original_diagnostic["contactMotion"]["feet"][foot];
                            if contact["loaded"] == true {
                                if let Some(speed) = contact["impulseWeightedTangentSpeedMps"].as_f64() {
                                    postpass_slip[foot] += speed / HZ as f64;
                                }
                            }
                            if postpass_slip[foot] > SLIP_LIMIT_M {
                                current_error = Some(format!("postpass-foot-{foot}-loaded-tangent-integral-exceeds-20mm"));
                            }
                        }
                    }
                }
                let last_800_valid = last_800.len() == HZ as usize
                    && last_800.iter().all(|sample| sample["allOriginalPredicatesPassed"] == true
                        && sample["originalCounterContinuous"] == true);
                if resisted_resolution.is_none() && substep == 19
                    && contact_native.is_some_and(|contact| at_native - contact >= RESISTED_RESOLUTION_AGE)
                    && first_new_fall.is_none() && first_new_source.is_none()
                    && first_pass_native.is_none() && resisted_lifecycle_unchanged
                    && policy.prototype_standing_passed && stable >= HZ
                    && json!(snapshot.motion) == "Upright" && last_800_valid
                    && rolling_800_slip.iter().all(|slip| *slip <= SLIP_LIMIT_M)
                    && getup_slip.iter().all(|slip| *slip <= SLIP_LIMIT_M)
                {
                    // One read-only full-state capture at resolution, rather
                    // than serializing the retained source archive every tick.
                    let state = serde_json::to_value(s).unwrap();
                    let actual_controller = &state["getup_controller"];
                    let actual_anchor_json = actual_controller["stable_feet"].clone();
                    let binding_unchanged = binding_words(&actual_controller["terminal_balance"])
                        == before_binding_words;
                    let actual_anchor_words = anchor_words(&actual_anchor_json);
                    let tracked_anchors_match = actual_anchor_words == anchor_words(&resisted_anchor_json);
                    let same_anchors_through_last_800 = last_800.iter().all(|sample|
                        sample["anchorF32Words"] == json!(actual_anchor_words));
                    let actual_anchors = serde_json::from_value(actual_anchor_json.clone()).map_err(|error|
                        Failure::new(s.tick, substep, "resisted-resolution-anchors", error.to_string(), 1.0, 0.0))?;
                    let resolved_diagnostic = standing_diagnostic(s, Some(actual_anchors)).map_err(|error|
                        Failure::new(s.tick, substep, "resisted-resolution-standing", error, 1.0, 0.0))?;
                    let resolved_predicates_pass = resolved_diagnostic["predicates"].as_object().unwrap()
                        .values().all(|value| value == true);
                    let controller_valid = actual_controller["impact_substep"].is_null()
                        && actual_controller["started_substep"].as_u64() == Some(old_start)
                        && actual_controller["stable_substeps"].as_u64() == Some(stable)
                        && actual_controller["status"]["forecast_substeps"].as_u64() == Some(old_forecast)
                        && actual_controller["status"]["prototype_standing_passed"] == true
                        && actual_controller["status"]["refusal"].is_null()
                        && actual_controller["status"]["teacher_decisions"] == before_controller["status"]["teacher_decisions"]
                        && actual_controller["status"]["policy_decisions"] == before_controller["status"]["policy_decisions"];
                    let valid = controller_valid && binding_unchanged && tracked_anchors_match
                        && same_anchors_through_last_800 && resolved_predicates_pass;
                    let resolution = json!({"classification":"actual-contact-resisted-without-new-fall",
                        "valid":valid,"native":at_native,"actualContactNative":contact_native,
                        "nativeSinceContact":at_native-contact_native.unwrap(),
                        "resolutionMinimumAgeNative":RESISTED_RESOLUTION_AGE,
                        "noNewFallRecoverySourceOrForecastThroughoutResponse":resisted_lifecycle_unchanged,
                        "pendingImpactCleared":actual_controller["impact_substep"].is_null(),
                        "terminalBindingExactF32WordsUnchanged":binding_unchanged,
                        "actualCurrentControllerStableFeet":actual_anchor_json,
                        "trackedStandingIntervalAnchorsExactF32WordsMatch":tracked_anchors_match,
                        "sameActualAnchorsThroughoutLast800":same_anchors_through_last_800,
                        "originalStandingDiagnosticUsingActualCurrentAnchors":resolved_diagnostic,
                        "allLast800OriginalPredicatesAndCountersPassed":last_800_valid,
                        "last800LoadedContactTangentIntegralM":rolling_800_slip,
                        "last800SlipLimitM":SLIP_LIMIT_M,
                        "resolvedResistedCumulativeLoadedIntegralFromCycleStartM":getup_slip,
                        "resolvedResistedCumulativeLimitM":SLIP_LIMIT_M,
                        "cumulativeIntegralHasNoResetDuringThisResponse":true,
                        "maximumRolling800LoadedContactTangentIntegralMReportedNotGated":maximum_rolling_800_slip,
                        "last800NativeEvidence":last_800,"controller":actual_controller,
                        "snapshot":snapshot,"resolutionAtNormalControlBoundary":true});
                    save(&cycle_dir, "resisted-response-resolution.json", &resolution);
                    if valid {
                        resisted_resolution = Some(resolution);
                    } else {
                        current_error = Some("resisted-response-controller-or-anchor-resolution-contract-failed".into());
                    }
                }
                if contact_native.is_some_and(|contact| at_native - contact >= DIAGNOSTIC_WINDOW)
                    && postpass_samples < REQUIRED_HOLD && resisted_resolution.is_none()
                {
                    current_error = Some("bounded-30-second-after-contact-diagnostic-window-ended".into());
                }
                if forecast.ambiguous_reset {
                    current_error = Some("forecast-counter-nonzero-downward-reset-accounting-ambiguous".into());
                }
                let event = json!({"native":at_native,"cycleNative":observed,
                    "substep":substep,"tick":s.tick,"motion":snapshot.motion,
                    "counters":snapshot.counters,"striker":snapshot.striker,
                    "policy":policy,"sourceStartNative":source_start,
                    "forecastAddedThisCycle":forecast.added-before_forecast,
                    "forecastResetEventsThisCycle":forecast.resets.len()-before_resets,
                    "contactNative":contact_native,"firstPassNative":first_pass_native,
                    "wholeGetupLoadedSlipMReportedNotGated":getup_slip,
                    "postpassNativeSamples":postpass_samples,"postpassLoadedSlipM":postpass_slip,
                    "resistedOriginalStandingPredicates":resisted_diagnostic["predicates"],
                    "resistedLifecycleUnchanged":resisted_lifecycle_unchanged,
                    "resistedOriginalCounterContinuous":resisted_counter_continuous,
                    "rolling800LoadedContactTangentIntegralM":rolling_800_slip,
                    "rolling800OriginalPredicatesAndCountersPassed":last_800_valid,
                    "validResistedResponseResolved":resisted_resolution.is_some(),
                    "originalStandingPredicates":original_diagnostic["predicates"],
                    "originalCounterContinuous":counter_continuous,"diagnosticStop":current_error});
                writeln!(ledger, "{event}").unwrap();
                if substep == 19 {
                    writeln!(trace, "{}", json!({"native":at_native,"snapshot":snapshot,
                        "standing":if first_pass_native.is_some() { &original_diagnostic } else { &resisted_diagnostic },
                        "event":event})).unwrap();
                }
                if let Some(reason) = current_error {
                    gate_failure = Some(event);
                    return Err(Failure::new(s.tick, substep, "repeat-diagnostic-stop", reason, 1.0, 0.0));
                }
                Ok(())
            });
            match advance {
                Ok(next) => reply = next,
                Err(rejection) => {
                    outcome = format!("runtime-advance-rejected-{rejection:?}");
                    break;
                }
            }
            ledger.flush().unwrap();
            trace.flush().unwrap();
            if reply.failure.is_some() || reply.completed_ticks != 1 {
                // Only terminal/incomplete ticks need a full-report forecast
                // reconciliation: good ticks are completely observed at 800Hz.
                let now_controller = controller(&runtime);
                forecast.observe(now_controller["status"]["forecast_substeps"].as_u64().unwrap(),
                    native(&runtime), "terminal-or-incomplete-tick reconciliation");
                outcome = if gate_failure.is_some() { "diagnostic-gate-stop" } else { "physical-or-controller-failure" }.into();
            } else if postpass_samples >= REQUIRED_HOLD {
                outcome = "new-recovery-and-postpass-hold-passed".into();
            } else if resisted_resolution.is_some() {
                outcome = "actual-contact-resisted-without-new-fall".into();
            } else if contact_native.is_none() && last_observed_native - before_native >= NO_CONTACT_WINDOW {
                outcome = "no-real-impact-within-5-second-apparatus-window".into();
            }
        }

        ledger.flush().unwrap();
        trace.flush().unwrap();
        let after_controller = controller(&runtime);
        let after_native = native(&runtime);
        forecast.observe(after_controller["status"]["forecast_substeps"].as_u64().unwrap(),
            after_native, "cycle-final reconciliation");
        save(&cycle_dir, "controller-after.json", &after_controller);
        save(&cycle_dir, "reply-after.json", &json!(reply));
        let after_checkpoint = save_checkpoint(&runtime, &cycle_dir, "after");
        let profile_unchanged = typed_profile(&fixture(&runtime)["profile"]) == expected_profile;
        let original_anchors_unchanged = first_pass_native.is_some()
            && anchor_words(&after_controller["stable_feet"]) == anchor_words(&anchors_json);
        let actual_integrations = after_native - before_native;
        let coverage = actual_integrations == observed;
        let recovered_response = outcome == "new-recovery-and-postpass-hold-passed"
            && first_new_fall.is_some()
            && first_new_source.is_some() && first_pass_native.is_some()
            && postpass_samples >= REQUIRED_HOLD && all_postpass_predicates
            && counter_continuous && postpass_slip.iter().all(|slip| *slip <= SLIP_LIMIT_M)
            && original_anchors_unchanged;
        let resisted_response = outcome == "actual-contact-resisted-without-new-fall"
            && resisted_resolution.as_ref().is_some_and(|resolution| resolution["valid"] == true)
            && first_new_fall.is_none() && first_new_source.is_none() && first_pass_native.is_none()
            && reply.snapshot.counters.falls == baseline_falls
            && reply.snapshot.counters.recoveries == baseline_recoveries
            && reply.snapshot.counters.strikes == baseline_strikes + 1
            && forecast.added == before_forecast && forecast.resets.len() == before_resets
            && after_controller["started_substep"].as_u64() == Some(old_start)
            && after_controller["impact_substep"].is_null()
            && after_controller["stable_substeps"].as_u64().is_some_and(|count| count >= HZ)
            && after_controller["status"]["prototype_standing_passed"] == true
            && after_controller["status"]["refusal"].is_null()
            && after_controller["status"]["forecast_substeps"].as_u64() == Some(old_forecast)
            && binding_words(&after_controller["terminal_balance"]) == before_binding_words
            && getup_slip.iter().all(|slip| *slip <= SLIP_LIMIT_M);
        let classification = if recovered_response { "fell-and-genuinely-recovered" }
            else if resisted_response { "resisted-without-new-fall" }
            else { "unresolved-or-failed-response" };
        let passed = (recovered_response || resisted_response)
            && ack_record["accepted"] == true && contact_native.is_some()
            && maximum_contact_impulse > 0.0 && profile_unchanged && coverage
            && before_checkpoint["verifiedZeroIntegration"] == true
            && after_checkpoint["verifiedZeroIntegration"] == true
            && !forecast.ambiguous_reset;
        let result = json!({"strikeResponseIteration":cycle,"passed":passed,"outcome":outcome,
            "responseClassification":classification,
            "genuineNewFallAndRecoveryDemonstrated":recovered_response,
            "actualContactResistedWithoutNewFall":resisted_response,
            "resistedResponseResolution":resisted_resolution,
            "maximumRolling800LoadedContactTangentIntegralMReportedNotGated":maximum_rolling_800_slip,
            "finalRolling800LoadedContactTangentIntegralM":rolling_800_slip,
            "actualNativeIntegrations":actual_integrations,"observedNativeIntegrations":observed,
            "unobservedTerminalTailNative":after_native-last_observed_native,
            "forecastNativeIntegrations":forecast.added-before_forecast,
            "forecastResetEvents":&forecast.resets[before_resets..],
            "forecastAccounting":"Previous counter initialized from imported policy; sum positive observed increments and record every downward reset separately; never final-minus-initial across a reset",
            "actualContact":contact_event,"firstNewFall":first_new_fall,
            "firstNewSource":first_new_source,"newFirstPass":first_pass_record,
            "maximumActualContactImpulseNs":maximum_contact_impulse,
            "requiredPostpassNativeSamples":REQUIRED_HOLD,"actualPostpassNativeSamples":postpass_samples,
            "postpassBoundaryTailNative":postpass_samples.saturating_sub(REQUIRED_HOLD),
            "postpassTailPolicy":"All gates remain active through the 0-19 native tail to the next 40Hz command boundary; no artificial success-stop failure or partial-tick state edit",
            "allOriginalStandingPredicatesPassedDuringPostpassHold":postpass_samples>=REQUIRED_HOLD && all_postpass_predicates,
            "originalControllerCounterContinuousDuringPostpassHold":postpass_samples>=REQUIRED_HOLD && counter_continuous,
            "originalControllerAnchorsUnchangedAfterHold":original_anchors_unchanged,
            "physicalEqualityMethod":"Exact restored f32 words for all stored anchor and anatomical pose comparisons; signed zero retained; no tolerance",
            "postpassLoadedContactTangentIntegralM":postpass_slip,"postpassSlipLimitM":SLIP_LIMIT_M,
            "wholeGetupLoadedContactTangentIntegralMReportedNotGated":getup_slip,
            "profileUnchanged":profile_unchanged,"gateFailure":gate_failure,
            "beforeCounters":before_snapshot.counters,"afterCounters":reply.snapshot.counters,
            "beforeNative":before_native,"afterNative":after_native,"finalFailure":reply.failure,
            "checkpointBefore":before_checkpoint,"checkpointAfter":after_checkpoint,
            "wallSeconds":cycle_began.elapsed().as_secs_f64(),"qualification":false});
        save(&cycle_dir, "result.json", &result);
        println!("{}", json!({"strikeResponseIteration":cycle,"passed":passed,"outcome":outcome,
            "responseClassification":classification,
            "actualNativeIntegrations":actual_integrations,
            "forecastNativeIntegrations":forecast.added-before_forecast,
            "actualContactNative":contact_native,"newFirstPassNative":first_pass_native,
            "postpassNativeSamples":postpass_samples,"postpassSlipM":postpass_slip,
            "finalFailure":reply.failure}));
        cycles.push(result);
        if !passed {
            final_outcome = format!("stopped-at-strike-response-{cycle}-{outcome}");
            break;
        }
    }

    let final_native = native(&runtime);
    let final_controller = controller(&runtime);
    save(out, "controller-final.json", &final_controller);
    let final_checkpoint = save_checkpoint(&runtime, out, "final");
    let all_five = cycles.len() == MAX_CYCLES as usize
        && cycles.iter().all(|cycle| cycle["passed"] == true)
        && final_checkpoint["verifiedZeroIntegration"] == true;
    if final_checkpoint["verifiedZeroIntegration"] != true {
        final_outcome = format!("{final_outcome}-final-checkpoint-verification-failed");
    }
    let genuine_new_recoveries = cycles.iter().filter(|response|
        response["passed"] == true && response["genuineNewFallAndRecoveryDemonstrated"] == true).count();
    let resisted_contacts = cycles.iter().filter(|response|
        response["passed"] == true && response["actualContactResistedWithoutNewFall"] == true).count();
    let result = json!({"classification":"fresh-explicit-repeat-profile-existing-actual-striker-diagnostic",
        "outcome":final_outcome,"allFiveActualStrikeResponsesValid":all_five,
        "requestedStrikeResponses":MAX_CYCLES,"attemptedStrikeResponses":cycles.len(),
        "genuineNewFallsAndRecoveriesDemonstrated":genuine_new_recoveries,
        "actualContactsResistedWithoutNewFall":resisted_contacts,
        "actualNativeIntegrations":final_native-initial_native,
        "observedNativeIntegrations":observed_total,"forecastNativeIntegrations":forecast.added,
        "forecastResetEvents":forecast.resets,"forecastAccountingAmbiguous":forecast.ambiguous_reset,
        "initialNative":initial_native,"finalNative":final_native,
        "initialCounters":initial_snapshot.counters,"finalCounters":reply.snapshot.counters,
        "strikeResponses":cycles,"finalCheckpoint":final_checkpoint,"inputCheckpoint":args[1],
        "strikerSpeedMps":4.0,"noImpactWindowNative":NO_CONTACT_WINDOW,
        "originalControllerDeadlineFromContactNative":ORIGINAL_DEADLINE,
        "boundedDiagnosticWindowAfterContactNative":DIAGNOSTIC_WINDOW,
        "resistedResponseEarliestResolutionAfterContactNative":RESISTED_RESOLUTION_AGE,
        "resistedResponseRequiresLast800OriginalPredicatesCountersAndSlipGate":true,
        "resistedSuccessEndsAtNormalControlBoundaryWithoutInjectedFailure":true,
        "expectedPhysicalFiveFallRecoveryTestIsSeparateAndNotImpliedByFiveValidResponses":true,
        "noFixturePreparationResetProfileMorphOrAnatomicalPoseVelocityWrite":true,
        "wholeGetupSlipIsReportedAndNeverUsedAsAStopGate":true,
        "postpassSlipIntegralBeginsAtEachNewFirstPassAndGatesOnlyItsPhysicalHold":true,
        "wallSeconds":began.elapsed().as_secs_f64(),
        "performanceQualification":false,"qualification24Cases":false,
        "qualification":false});
    save(out, "result.json", &result);
    println!("{}", json!({"outcome":final_outcome,"allFiveActualStrikeResponsesValid":all_five,
        "attemptedStrikeResponses":cycles.len(),"actualNativeIntegrations":final_native-initial_native,
        "genuineNewFallsAndRecoveriesDemonstrated":genuine_new_recoveries,
        "actualContactsResistedWithoutNewFall":resisted_contacts,
        "observedNativeIntegrations":observed_total,"forecastNativeIntegrations":forecast.added,
        "finalFailure":reply.failure,"qualification":false}));
}
