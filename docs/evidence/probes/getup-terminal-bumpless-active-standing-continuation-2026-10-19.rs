#![recursion_limit = "256"]
//! One checkpoint continuation with native-substep standing and contact-motion
//! evidence. No fixture preparation or teacher reference is replayed.
use bincode::Options;
use lh_sim::{getup::controller::standing_diagnostic, runtime::Runtime};
use serde_json::{Value, json};
use std::{
    fs::{self, OpenOptions},
    io::Write,
    path::Path,
    time::Instant,
};

fn save(out: &Path, name: &str, value: &Value) {
    fs::write(out.join(name), serde_json::to_vec_pretty(value).unwrap()).unwrap();
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(
        args.len(),
        5,
        "getup_standing_hold CHECKPOINT OUTPUT SIMULATED_SECONDS PHYSICS_COMPUTE_SECONDS"
    );
    let duration: f64 = args[3].parse().unwrap();
    let budget: f64 = args[4].parse().unwrap();
    assert!(
        duration >= 0.0
            && duration <= 30.0
            && budget.is_finite()
            && (budget > 1.0 || duration == 0.0 && budget == 0.0)
    );
    let ticks = (duration * 40.0).round() as u64;
    assert!((ticks as f64 / 40.0 - duration).abs() < 1e-8);
    let out = Path::new(&args[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("started-once"))
        .unwrap();
    let input = fs::read(&args[1]).unwrap();
    let input_parts: (u32, String, Vec<u8>, String) = bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(64 * 1024 * 1024)
        .reject_trailing_bytes()
        .deserialize(&input)
        .unwrap();
    assert_eq!(
        input_parts.0, 3,
        "Fresh base-readiness continuation requires schema3 input"
    );
    let readiness_wrapper: Value = serde_json::from_str(&input_parts.3).unwrap();
    assert_eq!(readiness_wrapper["format"], "terminal-base-ready-v1");
    assert_eq!(readiness_wrapper["inner_schema"], 2);
    let mut runtime = Runtime::from_getup_checkpoint(&input).unwrap();
    let verification: Value =
        serde_json::from_str(&runtime.verify_getup_checkpoint(&input).unwrap()).unwrap();
    save(out, "checkpoint-input-verification.json", &verification);
    let active_fixture: Value =
        serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    assert_eq!(
        active_fixture["rootTranslationCompensation"]["algorithm"],
        "f32-dq-f64-remainder-v1"
    );
    assert!(
        !active_fixture["rootTranslationCompensation"]["activeRoots"]
            .as_array()
            .unwrap()
            .is_empty()
    );
    assert_eq!(active_fixture["profile"]["solver_iterations"], 3);
    assert_eq!(active_fixture["profile"]["additional_body_iterations"], 1);
    save(
        out,
        "explicit-integration-mode.json",
        &json!({"carryAlreadyActiveAtInput":true,
        "profileUnmodified":true,"newActivationIntegrations":0,"activationOrResetDuringContinuation":false,
        "actualFixtureReport":active_fixture}),
    );
    let report_before: Value = serde_json::from_str(&runtime.getup_report().unwrap()).unwrap();
    assert!(report_before["terminal_base_ready"].is_object());
    let original_stable_native = report_before["stable_substeps"].as_u64().unwrap();
    let original_anchors = serde_json::from_value(report_before["stable_feet"].clone()).unwrap();
    let mut original_counter_failures = 0_u64;
    let mut first_original_counter_failure = None;
    let mut reply = runtime.advance(0).unwrap();
    let initial = reply.snapshot.clone();
    let initial_diag: Value =
        serde_json::from_str(&runtime.getup_standing_diagnostic("null").unwrap()).unwrap();
    let anchors = serde_json::from_value(initial_diag["feet"].clone()).unwrap();
    save(out, "initial-diagnostic.json", &initial_diag);
    let mut trace = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("trace.ndjson"))
        .unwrap();
    let mut native_trace = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("native-diagnostics.ndjson"))
        .unwrap();
    let mut physics_seconds = 0.0;
    let mut native = 0_u64;
    let mut passed_native = 0_u64;
    let mut max_linear = 0.0_f64;
    let mut max_angular = 0.0_f64;
    let mut max_drift = [0.0_f64; 2];
    let mut min_load = [f64::INFINITY; 2];
    let mut max_loaded_contact_tangent = [0.0_f64; 2];
    let mut loaded_contact_speed_integral = [0.0_f64; 2];
    let mut unloaded_samples = [0_u64; 2];
    let mut failure_trace_count = 0_u64;
    let mut first_predicate_failure = None;
    let mut predicate_failures = std::collections::BTreeMap::<String, u64>::new();
    let mut outcome = "completed";
    for tick in 0..ticks {
        if physics_seconds >= budget - 0.5 {
            outcome = "incomplete-physics-budget";
            break;
        }
        let began = Instant::now();
        reply = runtime
            .advance_observed(1, |s, _| {
                let diag = standing_diagnostic(s, Some(anchors)).map_err(|e| {
                    lh_sim::measurement::Failure::new(
                        s.tick,
                        0,
                        "standing-hold-observer",
                        e,
                        1.0,
                        0.0,
                    )
                })?;
                native += 1;
                let current_stable_native = (s.diagnostics.getup.as_ref().unwrap().policy.as_ref().unwrap().stable_standing_s * 800.0).round() as u64;
                let expected_stable_native = original_stable_native + native;
                if current_stable_native != expected_stable_native {
                    original_counter_failures += 1;
                    if first_original_counter_failure.is_none() {
                        first_original_counter_failure = Some(json!({"observedNative":native,
                            "physicalSeconds":s.snapshot().simulation_time_s,"expectedStableNative":expected_stable_native,
                            "actualStableNative":current_stable_native,
                            "diagnosticUsingOriginalControllerAnchors":standing_diagnostic(s,Some(original_anchors)).unwrap()}));
                    }
                }
                let predicates = diag["predicates"].as_object().unwrap();
                let mut passed = true;
                for (name, value) in predicates {
                    if value != true {
                        passed = false;
                        *predicate_failures.entry(name.clone()).or_default() += 1;
                    }
                }
                if passed {
                    passed_native += 1;
                }
                for row in diag["motion"].as_array().unwrap() {
                    max_linear = max_linear.max(row["linearMps"].as_f64().unwrap());
                    max_angular = max_angular.max(row["angularRadps"].as_f64().unwrap());
                }
                for i in 0..2 {
                    max_drift[i] = max_drift[i].max(diag["footDriftM"][i].as_f64().unwrap());
                    let foot = &diag["contactMotion"]["feet"][i];
                    min_load[i] = min_load[i].min(foot["loadN"].as_f64().unwrap());
                    if foot["loaded"] == true {
                        if let Some(speed) = foot["impulseWeightedTangentSpeedMps"].as_f64() {
                            loaded_contact_speed_integral[i] += speed * 0.00125;
                        }
                        for contact in foot["contacts"].as_array().unwrap() {
                            max_loaded_contact_tangent[i] = max_loaded_contact_tangent[i]
                                .max(contact["tangentSpeedMps"].as_f64().unwrap());
                        }
                    } else {
                        unloaded_samples[i] += 1;
                    }
                }
                if !passed && first_predicate_failure.is_none() {first_predicate_failure=Some(json!({"observedNative":native,"physicalSeconds":s.snapshot().simulation_time_s,"diagnostic":diag}));}
                if native % 20 == 0 || !passed && failure_trace_count<12 {
                    writeln!(native_trace, "{diag}").unwrap();
                    if !passed {failure_trace_count+=1;}
                }
                for foot in 0..2 {if loaded_contact_speed_integral[foot]>0.02 {return Err(lh_sim::measurement::Failure::new(s.tick,0,"loaded-foot-slip-stop",format!("foot-{foot}"),loaded_contact_speed_integral[foot] as f32,0.02));}}
                Ok(())
            })
            .unwrap();
        physics_seconds += began.elapsed().as_secs_f64();
        writeln!(
            trace,
            "{}",
            json!({"snapshot":reply.snapshot,"continuationTick":tick+1})
        )
        .unwrap();
        trace.flush().unwrap();
        if reply.failure.is_some() || reply.completed_ticks != 1 {
            outcome = "physical-failure";
            break;
        }
    }
    let bytes = runtime.getup_checkpoint().unwrap();
    fs::write(out.join("checkpoint-final.bin"), &bytes).unwrap();
    let final_verification: Value =
        serde_json::from_str(&runtime.verify_getup_checkpoint(&bytes).unwrap()).unwrap();
    save(
        out,
        "checkpoint-final-verification.json",
        &final_verification,
    );
    let report_after: Value = serde_json::from_str(&runtime.getup_report().unwrap()).unwrap();
    save(out, "reference-report.json", &report_after);
    let initial_com = initial_diag["com"].clone();
    let final_diag: Value = serde_json::from_str(
        &runtime
            .getup_standing_diagnostic(&serde_json::to_string(&anchors).unwrap())
            .unwrap(),
    )
    .unwrap();
    save(out, "final-diagnostic.json", &final_diag);
    let final_fixture: Value =
        serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    let actual_native = final_fixture["nativeSubsteps"].as_u64().unwrap()
        - (initial.simulation_time_s * 800.0).round() as u64;
    let result = json!({"actualContinuationIntegrations":actual_native,"unobservedTerminalTailNative":actual_native-native,"firstStandingPredicateFailure":first_predicate_failure,"loadedSlipStopM":0.02,"loadedSlipIntegralStartsAt":"First native integration after restoring this newly measured passing recovery checkpoint; unchanged recovery-wide slip remains separately reported without a whole-getup stop","outcome":outcome,"inputCheckpoint":args[1],"requestedSimulatedSeconds":duration,
        "physicsWallSeconds":physics_seconds,"authorizedPhysicsComputeSeconds":budget,"remainingPhysicsComputeSeconds":budget-physics_seconds,
        "nativeSubsteps":native,"simulatedSeconds":native as f64/800.0,
        "allStandingPredicatesPassedEveryNativeSubstep":native>0 && native==passed_native,
        "standingPredicateFailureSubsteps":predicate_failures,"maximumFootOriginDriftM":max_drift,"minimumFootLoadsN":min_load,
        "maximumBodyLinearMps":max_linear,"maximumBodyAngularRadps":max_angular,
        "maximumLoadedContactTangentSpeedMps":max_loaded_contact_tangent,
        "loadedContactSpeedIntegralM":loaded_contact_speed_integral,"unloadedFootSubsteps":unloaded_samples,
        "initialCom":initial_com,"finalCom":final_diag["com"],"initial":initial,"final":reply,
        "originalControllerStableCounterAtInput":original_stable_native,
        "originalControllerStableCounterExpectedFinal":original_stable_native+native,
        "originalControllerStableCounterFinal":report_after["stable_substeps"],
        "originalControllerCounterContinuousEveryNativeSubstep":native>0 && original_counter_failures==0,
        "originalControllerCounterDiscontinuitySubsteps":original_counter_failures,
        "firstOriginalControllerCounterFailure":first_original_counter_failure,
        "originalStableAnchorsAtInput":report_before["stable_feet"],
        "originalStableAnchorsUnchanged":report_before["stable_feet"]==report_after["stable_feet"],
        "teacherForecastUnchanged":report_before["status"]["forecast_substeps"]==report_after["status"]["forecast_substeps"],
        "slowDecisionCountUnchanged":report_before["status"]["policy_decisions"]==report_after["status"]["policy_decisions"],
        "slowDecisionArchiveLengthUnchanged":report_before["student_decisions"].as_array().unwrap().len()==report_after["student_decisions"].as_array().unwrap().len(),
        "terminalBalanceBindingBefore":report_before["terminal_balance"],"terminalBalanceBindingAfter":report_after["terminal_balance"],"terminalBaseReadyBefore":report_before["terminal_base_ready"],"terminalBaseReadyAfter":report_after["terminal_base_ready"],
        "controllerVariant":"getup-terminal-bumpless-experiment",
        "noPoseOrVelocityWrites":true,"noExternalSupport":true,"qualification24Cases":false});
    save(out, "result.json", &result);
    println!("{result}");
}
