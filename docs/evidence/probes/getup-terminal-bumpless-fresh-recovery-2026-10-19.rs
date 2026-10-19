//! Opt-in terminal joint-balance transfer; frozen WEAK/SLOW prefix and native observation.
use bincode::Options;
use lh_sim::{Profile, getup, runtime::Runtime};
use serde_json::{Value, json};
use std::{
    fs::{self, File, OpenOptions},
    io::Write,
    path::Path,
    time::Instant,
};
fn save(out: &Path, name: &str, value: &Value) {
    let mut f = File::create(out.join(name)).unwrap();
    serde_json::to_writer_pretty(&mut f, value).unwrap();
    writeln!(f).unwrap();
    f.sync_all().unwrap();
}
fn fixture(runtime: &Runtime) -> Value {
    serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap()
}
fn checkpoint(runtime: &Runtime, out: &Path, label: &str) {
    let bytes = runtime.getup_checkpoint().unwrap();
    let mut f = File::create(out.join(format!("checkpoint-{label}.bin"))).unwrap();
    f.write_all(&bytes).unwrap();
    f.sync_all().unwrap();
    save(
        out,
        &format!("checkpoint-{label}.json"),
        &serde_json::from_str(&runtime.verify_getup_checkpoint(&bytes).unwrap()).unwrap(),
    );
}
fn main() {
    let began = Instant::now();
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "frozen-recovery-replay KIND OUTPUT");
    let kind = &args[1];
    assert!(["back", "belly", "side", "side-supported"].contains(&kind.as_str()));
    let out = Path::new(&args[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("started-once"))
        .unwrap();
    let profile = match std::env::var("LH_GETUP_FIXTURE_PROFILE") {
        Ok(path) => serde_json::from_str::<Profile>(&fs::read_to_string(path).unwrap()).unwrap(),
        Err(std::env::VarError::NotPresent) => {
            let mut profile = Profile::default();
            profile.getup_policy_enabled = true;
            profile
        }
        Err(error) => panic!("Invalid explicit GetUp fixture profile path: {error}"),
    };
    profile.validate().unwrap();
    assert!(
        profile.getup_policy_enabled,
        "Explicit recovery profile must enable the policy"
    );
    let mut runtime = Runtime::new_getup_fixture_with_profile(kind, profile.clone()).unwrap();
    let carry = std::env::var("LH_GETUP_ROOT_TRANSLATION_CARRY").unwrap_or_else(|_| "0".into());
    assert!(carry == "0" || carry == "1");
    let legacy = runtime.getup_checkpoint().unwrap();
    let parse = |b: &[u8]| -> (u32, String, Vec<u8>, String) {
        bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .with_limit(64 * 1024 * 1024)
            .reject_trailing_bytes()
            .deserialize(b)
            .unwrap()
    };
    assert_eq!(parse(&legacy).0, 3);
    let unwrap_readiness = |text: &str, expected_inner: u64| -> String {
        let wrapper: Value = serde_json::from_str(text).unwrap();
        assert_eq!(wrapper["format"], "terminal-base-ready-v1");
        assert_eq!(wrapper["inner_schema"].as_u64().unwrap(), expected_inner);
        wrapper["auxiliary"].as_str().unwrap().to_owned()
    };
    if carry == "1" {
        runtime
            .set_root_translation_compensation_enabled(true)
            .unwrap();
        let active = runtime.getup_checkpoint().unwrap();
        let a = parse(&active);
        let b = parse(&legacy);
        assert_eq!(a.0, 3);
        assert_eq!(a.1, b.1);
        assert_eq!(a.2, b.2);
        let nested_active = unwrap_readiness(&a.3, 2);
        let nested_legacy = unwrap_readiness(&b.3, 1);
        let wrapper: Value = serde_json::from_str(&nested_active).unwrap();
        assert_eq!(wrapper["runtime"].as_str().unwrap(), nested_legacy);
    }
    let actual_fixture: Value =
        serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    let actual_profile: Profile =
        serde_json::from_value(actual_fixture["profile"].clone()).unwrap();
    assert_eq!(
        serde_json::to_string(&actual_profile).unwrap(),
        serde_json::to_string(&profile).unwrap()
    );
    save(
        out,
        "explicit-carry-mode.json",
        &json!({"explicitlyEnabled":carry=="1", "actualFixtureReport":actual_fixture,
        "activationNativeIntegrations":0,"serializedBodyWorldAndRuntimeUnchangedAtActivation":true}),
    );
    let initial_controller: Value = serde_json::from_str(&runtime.getup_report().unwrap()).unwrap();
    assert_eq!(initial_controller["terminal_base_ready"]["format"], "terminal-base-ready-bumpless-v2");
    save(out, "initial-readiness-controller.json", &initial_controller);
    let mut reply = runtime.advance(0).unwrap();
    let mut trace = File::create(out.join("trace.ndjson")).unwrap();
    let mut trigger = None;
    let mut trigger_slip = None;
    let mut native_samples = 0_u64;
    let mut last_observed_native = 0_u64;
    let mut first_pass_native = None;
    let mut first_pass_snapshot = None;
    let mut slip = [0_f64; 2];
    let mut first_20mm = [None::<f64>; 2];
    let mut max_tangent = [0_f64; 2];
    let mut loaded_samples = [0_u64; 2];
    let mut minimum_head = f64::INFINITY;
    let mut maximum_head = f64::NEG_INFINITY;
    let mut maximum_stable = 0_f64;
    let mut max_effort = 0_f32;
    let model = lh_model::Model::canonical().unwrap();
    let mut gears = [0.0_f32; 21];
    let mut actuator_bodies: Vec<String> = vec![String::new(); 21];
    for part in &model.segments {
        if let Some(hinge) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            gears[hinge.actuator_index] = hinge.gear;
            actuator_bodies[hinge.actuator_index] = part.id.clone();
        }
    }
    assert_eq!(gears.iter().sum::<f32>(), 920.0);
    assert!(gears.iter().all(|g| *g > 0.0));
    let mut previous_controls: Option<[f32; 21]> = None;
    let mut previous_native = 0_u64;
    let mut previous_blending = false;
    let mut transition_rows: Vec<Value> = Vec::new();
    let mut first_blend_native = None;
    let mut first_blend_actual_jump = None;
    let mut transition_max_normalized_jump = 0.0_f32;
    let mut transition_max_torque_jump_nm = 0.0_f32;
    let mut frozen_actor_counts = None;
    let outcome;
    let mut diagnostics = File::create(out.join("diagnostics-40hz.ndjson")).unwrap();
    checkpoint(&runtime, out, "initial");
    loop {
        let f = fixture(&runtime);
        let real = f["nativeSubsteps"].as_u64().unwrap();
        let policy = reply
            .snapshot
            .diagnostics
            .getup
            .as_ref()
            .unwrap()
            .policy
            .as_ref()
            .unwrap();
        assert_eq!(
            real,
            (reply.snapshot.simulation_time_s * 800.).round() as u64
        );
        assert!(policy.forecast_substeps <= 5000);
        assert_eq!(reply.snapshot.diagnostics.direct_pelvis_force_n, 0.);
        assert_eq!(reply.snapshot.diagnostics.direct_pelvis_torque_nm, 0.);
        assert!(reply.snapshot.diagnostics.requested_motor_effort_nm <= 920.001);
        maximum_stable = maximum_stable.max(policy.stable_standing_s);
        max_effort = max_effort.max(reply.snapshot.diagnostics.requested_motor_effort_nm);
        writeln!(
            trace,
            "{}",
            json!({"snapshot":reply.snapshot,"fixture":f,"triggerNative":trigger,"controlPhase":policy.phase})
        )
        .unwrap();
        trace.flush().unwrap();
        let diagnostic: Value =
            serde_json::from_str(&runtime.getup_standing_diagnostic("null").unwrap()).unwrap();
        writeln!(
            diagnostics,
            "{}",
            json!({"physicalSeconds":reply.snapshot.simulation_time_s,"standing":diagnostic})
        )
        .unwrap();
        if reply.failure.is_some() {
            outcome = if trigger.is_none() {
                "invalid-fixture-integrity"
            } else if policy.phase == "reference-rejected" {
                "controller-reference-failure"
            } else {
                "controller-physical-integrity-failure"
            };
            break;
        }
        if policy.prototype_standing_passed {
            outcome = if reply
                .snapshot
                .diagnostics
                .getup
                .as_ref()
                .unwrap()
                .controller
                == "experimental-recovery-bumpless-joint-balance-hold"
            {
                "experimental-terminal-balance-one-second-standing-pass"
            } else {
                "frozen-one-second-standing-pass"
            };
            break;
        }
        if trigger.is_none() {
            assert_eq!(policy.forecast_substeps, 0);
            if f["state"]["ready"] == true {
                let before = serde_json::to_value(&reply.snapshot.segments).unwrap();
                runtime.trigger_getup_fixture().unwrap();
                reply = runtime.advance(0).unwrap();
                assert_eq!(
                    before,
                    serde_json::to_value(&reply.snapshot.segments).unwrap()
                );
                trigger = Some(real);
                trigger_slip = Some(slip);
                save(out, "fixture-at-trigger.json", &f);
                checkpoint(&runtime, out, "trigger");
                continue;
            }
            let maximum_preparation = if kind == "side-supported" { 3200 } else { 1600 };
            if real >= maximum_preparation {
                outcome = "invalid-fixture-not-admitted";
                break;
            }
        } else if real - trigger.unwrap() >= 20000 {
            outcome = "25-second-recovery-deadline";
            break;
        }
        let ticks = if trigger.is_none() && !kind.starts_with("side") {
            4
        } else {
            1
        };
        reply = runtime
            .advance_observed(ticks, |s, _| {
                let d = getup::controller::standing_diagnostic(s, None).unwrap();
                native_samples += 1;
                last_observed_native = d["nativeSubsteps"].as_u64().unwrap();
                let p = s
                    .diagnostics
                    .getup
                    .as_ref()
                    .unwrap()
                    .policy
                    .as_ref()
                    .unwrap();
                maximum_stable = maximum_stable.max(p.stable_standing_s);
                if p.prototype_standing_passed && first_pass_native.is_none() {
                    first_pass_native = Some(last_observed_native);
                    first_pass_snapshot = Some(s.snapshot());
                }
                let control: [f32; 21] = s.diagnostics.getup.as_ref().unwrap().motor_controls.as_slice().try_into().unwrap();
                let controller_id = &s.diagnostics.getup.as_ref().unwrap().controller;
                let active_bumpless = controller_id == "experimental-recovery-bumpless-joint-balance-hold";
                let blending = active_bumpless && p.phase.contains("blending");
                if active_bumpless {
                    let counts = (p.policy_decisions, p.teacher_decisions, p.forecast_substeps);
                    if let Some(expected) = frozen_actor_counts { assert_eq!(counts, expected); }
                    else { frozen_actor_counts = Some(counts); }
                }
                if blending || previous_blending {
                    let old = previous_controls.expect("Observed blend must follow an actual native command");
                    assert_eq!(last_observed_native, previous_native + 1);
                    let delta: [f32; 21] = std::array::from_fn(|i| control[i] - old[i]);
                    let maximum_normalized = delta.iter().map(|d| d.abs()).fold(0.0_f32, f32::max);
                    let maximum_nm = delta.iter().zip(gears).map(|(d, g)| d.abs() * g).fold(0.0_f32, f32::max);
                    transition_max_normalized_jump = transition_max_normalized_jump.max(maximum_normalized);
                    transition_max_torque_jump_nm = transition_max_torque_jump_nm.max(maximum_nm);
                    if first_blend_native.is_none() {
                        assert!(blending);
                        assert_eq!(control.map(f32::to_bits), old.map(f32::to_bits), "First new torque command must copy the preceding actual vector");
                        first_blend_native = Some(last_observed_native);
                        first_blend_actual_jump = Some(maximum_nm);
                    }
                    transition_rows.push(json!({"nativeSubstep":last_observed_native,"phase":p.phase,
                        "blending":blending,"actualNormalizedControls":control,"actualNormalizedIncrement":delta,
                        "maximumActualNormalizedIncrement":maximum_normalized,"maximumActualTorqueIncrementNm":maximum_nm}));
                }
                previous_blending = blending;
                previous_native = last_observed_native;
                previous_controls = Some(control);
                max_effort = max_effort.max(s.diagnostics.requested_motor_effort_nm);
                assert!(s.diagnostics.requested_motor_effort_nm <= 920.001);
                assert_eq!(s.diagnostics.direct_pelvis_force_n, 0.);
                assert_eq!(s.diagnostics.direct_pelvis_torque_nm, 0.);
                let head = d["headM"].as_f64().unwrap();
                minimum_head = minimum_head.min(head);
                maximum_head = maximum_head.max(head);
                for foot in 0..2 {
                    let contact = &d["contactMotion"]["feet"][foot];
                    if contact["loaded"] == true {
                        loaded_samples[foot] += 1;
                        let speed = contact["impulseWeightedTangentSpeedMps"].as_f64().unwrap();
                        max_tangent[foot] = max_tangent[foot].max(speed);
                        slip[foot] += speed * 0.00125;
                        if slip[foot] > 0.02 && first_20mm[foot].is_none() {
                            first_20mm[foot] = Some(s.snapshot().simulation_time_s);
                        }
                    }
                }
                Ok(())
            })
            .unwrap();
    }
    checkpoint(&runtime, out, "final");
    let f = fixture(&runtime);
    let real = f["nativeSubsteps"].as_u64().unwrap();
    let policy = reply
        .snapshot
        .diagnostics
        .getup
        .as_ref()
        .unwrap()
        .policy
        .as_ref()
        .unwrap();
    let report_text = runtime.getup_report().unwrap();
    let report: Value = serde_json::from_str(&report_text).unwrap();
    fs::write(out.join("reference-report.json"), &report_text).unwrap();
    let anchored_terminal = if report["stable_feet"].is_array() {
        Some(
            serde_json::from_str::<Value>(
                &runtime
                    .getup_standing_diagnostic(&report["stable_feet"].to_string())
                    .unwrap(),
            )
            .unwrap(),
        )
    } else {
        None
    };
    save(out, "native-torque-transition.json", &json!({"physicsIntegrationsAddedByObservation":0,
        "samplingHz":800,"actuatorBodies":actuator_bodies,"actuatorGearsNm":gears,
        "firstObservedBlendNative":first_blend_native,"firstActualTorqueJumpNm":first_blend_actual_jump,
        "maximumTransitionActualNormalizedIncrement":transition_max_normalized_jump,
        "maximumTransitionActualTorqueIncrementNm":transition_max_torque_jump_nm,
        "actorCountsFrozenThroughAndAfterTransition":frozen_actor_counts,"samples":transition_rows,
        "scope":"Scalar joint torque commands gear times normalized control, not a delivered reaction-torque readback or continuous world-vector torque claim. Includes first exact-copy sample, all sampled blend intervals and the first following endpoint command."}));
    let final_diagnostic: Value =
        serde_json::from_str(&runtime.getup_standing_diagnostic("null").unwrap()).unwrap();
    let result = json!({"kind":kind,"outcome":outcome,"fixtureAdmitted":trigger.is_some(),"triggerNative":trigger,"realNativeIntegrations":real,"forecastNativeIntegrations":policy.forecast_substeps,"totalActualIntegrations":real+policy.forecast_substeps as u64,"nativeObserverSamples":native_samples,"lastObservedNative":last_observed_native,"unobservedRealTailNativeSteps":real-last_observed_native,"firstLatchedStandingPassNative":first_pass_native,"firstLatchedStandingPassSnapshot":first_pass_snapshot,"physicalSeconds":reply.snapshot.simulation_time_s,"wallSecondsSinceHarnessEntryIncludingConstructionObservationAndSerialization":began.elapsed().as_secs_f64(),"profile":profile,"sameFrozenActors":true,"terminalBalance":report["terminal_balance"],"terminalBaseReady":report["terminal_base_ready"],"controllerVariant":"getup-terminal-bumpless-experiment","externalRootAssistance":false,"maximumRequestedMotorEffortNm":max_effort,"standingPassed":policy.prototype_standing_passed,"maximumContiguousStableSecondsObservedAt800Hz":maximum_stable,"finalContiguousStableSeconds":policy.stable_standing_s,"minimumActualHeadM":minimum_head,"maximumActualHeadM":maximum_head,"loadedSlip":{"interpretation":"Read-only cumulative integral of impulse-weighted solved foot-point tangent speed, only while that foot is loaded, starting at first real fixture integration and ending at lastObservedNative. A terminal integrity failure may leave an explicitly reported unobserved real tail. Forecast clone is separately counted and not included in this real-body integral. Recovery has no whole-motion20mm stop;20mm crossings are diagnostic only. The frozen controller retains its10mm foot-origin gate within a contiguous stable-standing interval.","m":slip,"atTriggerM":trigger_slip,"first20mmCrossingPhysicalSeconds":first_20mm,"loadedNativeSamples":loaded_samples,"maximumTangentSpeedMps":max_tangent},"fixtureAdmissionStateFrozenAfterTrigger":f,"finalStandingDiagnosticWithoutAnchors":final_diagnostic,"stableIntervalAnchoredTerminalDiagnostic":anchored_terminal,"diagnosticScope":"Unanchored packets do not independently establish the foot-drift predicate. The unchanged-runtime latched1s standing gate is reported separately from terminal state and from broader recovery qualification.","preparationPollingTicks":if kind.starts_with("side"){1}else{4},"final":reply,"qualification24Cases":false});
    save(out, "result.json", &result);
    println!(
        "{}",
        json!({"kind":kind,"outcome":outcome,"realNativeIntegrations":real,"forecastNativeIntegrations":policy.forecast_substeps,"standingPassed":policy.prototype_standing_passed,"loadedSlipM":slip})
    );
}
