//! Pinned combined-feature belly checkpoint, one ordinary tick per process.
//! Do not build or run before the parent's explicit CPU-clear signal.
use lh_sim::runtime::Runtime;
#[cfg(coordinate_integration_capture)]
use rapier3d::coordinate_integration_diagnostic::{begin_capture, take_capture, RootIntegrationSample};
use serde_json::{Value, json};
use std::{fs::{self, File, OpenOptions}, io::Write, path::Path, process::{Command, Stdio}};

fn save(path: &Path, value: &Value) {
    let mut file = File::create(path).unwrap();
    serde_json::to_writer_pretty(&mut file, value).unwrap();
    writeln!(file).unwrap();
}
fn sha256(bytes: &[u8]) -> String {
    // Hash the exact in-memory bytes that will be restored, not a second file read.
    let mut child = Command::new("sha256sum").stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    child.stdin.take().unwrap().write_all(bytes).unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success());
    String::from_utf8(output.stdout).unwrap().split_whitespace().next().unwrap().to_owned()
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "CHECKPOINT OUTPUT_DIRECTORY");
    let context: Value = serde_json::from_str(include_str!(env!("LH_COORDINATE_DIAGNOSTIC_CONTEXT"))).unwrap();
    let bytes = fs::read(&args[1]).unwrap();
    let input_sha256 = sha256(&bytes);
    assert_eq!(input_sha256, context["checkpoint"]["sha256"].as_str().unwrap(), "Input checkpoint SHA differs");
    let out = Path::new(&args[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new().write(true).create_new(true).open(out.join("started-once")).unwrap();
    let mut runtime = Runtime::from_getup_checkpoint(&bytes).unwrap();
    let initial = runtime.advance(0).unwrap();
    assert!(initial.failure.is_none());
    let initial_json = serde_json::to_value(&initial.snapshot).unwrap();
    let root = &initial_json["segments"][0];
    let exported_root_pose = json!({"id":root["id"],"position":root["position"],"rotation":root["rotation"]});
    assert_eq!(exported_root_pose, context["initialExportedRootPose"], "Restored exported root pose differs");
    assert_eq!(initial.snapshot.simulation_time_s, context["initialSimulationTimeS"].as_f64().unwrap());
    save(&out.join("run-context.json"), &json!({"inputCheckpoint":args[1],"inputSha256":input_sha256,
        "observerCaptureCompiled":cfg!(coordinate_integration_capture),"pinnedSourceAndLibraryContext":context,
        "initialExportedRootPose":exported_root_pose}));
    save(&out.join("initial-snapshot.json"), &initial_json);
    save(&out.join("initial-diagnostic.json"),
        &serde_json::from_str(&runtime.getup_standing_diagnostic("null").unwrap()).unwrap());
    save(&out.join("initial-controller.json"), &serde_json::from_str(&runtime.getup_report().unwrap()).unwrap());
    let initial_fixture: Value = serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    save(&out.join("initial-fixture.json"), &initial_fixture);

    #[cfg(coordinate_integration_capture)]
    begin_capture(500).unwrap();
    // The sole integration call in either mode: one normal 40Hz tick.
    let reply = runtime.advance(1).unwrap();
    #[cfg(coordinate_integration_capture)]
    let captured = take_capture();
    let final_json = serde_json::to_value(&reply.snapshot).unwrap();
    save(&out.join("final-reply.json"), &serde_json::to_value(&reply).unwrap());
    save(&out.join("final-snapshot.json"), &final_json);
    save(&out.join("final-diagnostic.json"),
        &serde_json::from_str(&runtime.getup_standing_diagnostic("null").unwrap()).unwrap());
    save(&out.join("final-controller.json"), &serde_json::from_str(&runtime.getup_report().unwrap()).unwrap());
    let final_fixture: Value = serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    save(&out.join("final-fixture.json"), &final_fixture);
    assert_eq!(final_fixture["nativeSubsteps"].as_u64().unwrap() - initial_fixture["nativeSubsteps"].as_u64().unwrap(), 20);
    // A new diagnostic output only. The pinned input is never overwritten.
    fs::write(out.join("final-checkpoint.bin"), runtime.getup_checkpoint().unwrap()).unwrap();
    assert!(reply.failure.is_none(), "Normal physics step failed");
    assert_eq!(reply.completed_ticks, 1);
    let simulated_seconds = reply.snapshot.simulation_time_s-initial.snapshot.simulation_time_s;
    assert!((simulated_seconds-0.025).abs() < 1e-10);
    #[cfg(coordinate_integration_capture)]
    match captured {
        Ok(rows) => analyze_rows(out, rows, &args[1], simulated_seconds),
        Err(error) => {
            save(&out.join("capture-error.json"), &json!({"error":format!("{error:?}")}));
            panic!("Capture invalid: {error:?}");
        }
    }
    #[cfg(not(coordinate_integration_capture))]
    save(&out.join("summary.json"), &json!({"inputCheckpoint":args[1],"ordinaryTicks":1,
        "observerCaptureCompiled":false,"nativeIntegrationsExpected":20,"simulatedSeconds":simulated_seconds}));
}

#[cfg(coordinate_integration_capture)]
fn analyze_rows(out: &Path, rows: Vec<RootIntegrationSample>, checkpoint: &str, simulated_seconds: f64) {
    assert_eq!(rows.len(), 500, "Expected 20 native steps x 25 internal substeps");
    let root_body = rows[0].root_body;
    let mut file = File::create(out.join("root-internal-substeps.ndjson")).unwrap();
    let mut biased_integral = [0.0_f64; 3];
    let mut relaxed_integral = [0.0_f64; 3];
    let mut actual_delta_sum = [0.0_f64; 3];
    let mut rounding_error = [0.0_f64; 3];
    let mut bias_minus_relax = [0.0_f64; 3];
    let mut lost_nonzero_increment_count = [0_usize; 3];
    let mut rounded_operation_mismatch_count = [0_usize; 3];
    for (ordinal, row) in rows.iter().enumerate() {
        assert_eq!(row.root_body, root_body, "Unexpected additional articulation");
        assert_eq!(row.ndofs, 27);
        assert_eq!(row.stabilization_iterations, 1);
        assert!((row.dt as f64 - 0.00005).abs() < 1e-10);
        if ordinal > 0 {
            for axis in 0..3 {
                assert_eq!(rows[ordinal-1].coords_after[axis].to_bits(), row.coords_before[axis].to_bits(),
                    "Unexpected root-coordinate change between captured integrations");
            }
        }
        let mut per_axis = Vec::new();
        for axis in 0..3 {
            let q0 = row.coords_before[axis] as f64;
            let q1 = row.coords_after[axis] as f64;
            let dt = row.dt as f64;
            let vb = row.biased_root_velocity[axis] as f64;
            let vr = row.relaxed_root_velocity[axis] as f64;
            // Diagnostic arithmetic only; none is passed back into the solver.
            let actual = q1 - q0;
            let exact_biased_increment = vb * dt;
            let rounded_prediction = row.coords_before[axis] + row.biased_root_velocity[axis] * row.dt;
            let lost = actual == 0.0 && exact_biased_increment != 0.0;
            biased_integral[axis] += exact_biased_increment;
            relaxed_integral[axis] += vr * dt;
            actual_delta_sum[axis] += actual;
            rounding_error[axis] += actual - exact_biased_increment;
            bias_minus_relax[axis] += (vb - vr) * dt;
            lost_nonzero_increment_count[axis] += usize::from(lost);
            rounded_operation_mismatch_count[axis] += usize::from(rounded_prediction.to_bits() != row.coords_after[axis].to_bits());
            per_axis.push(json!({"actualIncrementM":actual,
                "biasedIncrementF64M":exact_biased_increment,
                "integrationRoundingErrorM":actual-exact_biased_increment,
                "biasMinusRelaxIncrementM":(vb-vr)*dt,
                "lostNonzeroIncrement":lost}));
        }
        writeln!(file, "{}", json!({"ordinal":ordinal,"rootBody":row.root_body,
            "solverId":row.solver_id,"ndofs":row.ndofs,"dt":row.dt,
            "coordsBefore":row.coords_before,"biasedRootVelocity":row.biased_root_velocity,
            "coordsAfter":row.coords_after,"relaxedRootVelocity":row.relaxed_root_velocity,
            "stabilizationIterations":row.stabilization_iterations,"translationDiagnostic":per_axis})).unwrap();
    }
    file.flush().unwrap();
    let closure_error: [f64; 3] = core::array::from_fn(|i|
        actual_delta_sum[i] - relaxed_integral[i] - rounding_error[i] - bias_minus_relax[i]);
    save(&out.join("summary.json"), &json!({"inputCheckpoint":checkpoint,
        "ordinaryTicks":1,"nativeIntegrationsExpected":20,"internalRows":rows.len(),
        "simulatedSeconds":simulated_seconds,
        "rootBody":root_body,"translationAxisOrder":["x","y","z"],
        "biasedVelocityIntegralM":biased_integral,"relaxedVelocityIntegralM":relaxed_integral,
        "actualCoordinateDeltaM":actual_delta_sum,"integrationRoundingErrorM":rounding_error,
        "biasMinusRelaxIntegralM":bias_minus_relax,"decompositionClosureErrorM":closure_error,
        "lostNonzeroIncrementCount":lost_nonzero_increment_count,
        "roundedOperationMismatchCount":rounded_operation_mismatch_count,
        "physicsOrThresholdChanges":false,"inputCheckpointOverwritten":false,
        "interpretation":"Separates root-coordinate rounding from biased-versus-relaxed motion; it does not by itself attribute all foot motion."}));
}
