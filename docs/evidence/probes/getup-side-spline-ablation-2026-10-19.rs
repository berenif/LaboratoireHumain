//! One-entry diagnostic ablation. No controller, world, gain or gate edits.
//! Link only the preserved ordinary lh_sim rlib and its pinned dependencies.
use bincode::Options;
use lh_sim::{getup, runtime::Runtime};
use serde_json::{json, Value};
use std::{fs::{self, File, OpenOptions}, io::Write, path::Path, time::Instant};

const TRIGGER: u64 = 380;
const END: u64 = TRIGGER + 2400;
const DT: f64 = 0.00125;
const LEAF: &str = "/simulation/getup_controller/reference/72/14";
type Checkpoint = (u32, String, Vec<u8>, String);

fn decode(bytes: &[u8]) -> Checkpoint {
    bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024)
        .reject_trailing_bytes().deserialize(bytes).unwrap()
}
fn encode(parts: &Checkpoint) -> Vec<u8> {
    bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024)
        .serialize(parts).unwrap()
}
fn save(path: &Path, value: &Value) {
    let mut file = File::create(path).unwrap();
    serde_json::to_writer_pretty(&mut file, value).unwrap();
    writeln!(file).unwrap(); file.sync_all().unwrap();
}
fn json_text(text: &str) -> Value { serde_json::from_str(text).unwrap() }
fn typed_profile(value: Value) -> Value {
    let profile: lh_sim::Profile=serde_json::from_value(value).unwrap();
    serde_json::to_value(profile).unwrap()
}
fn fixture(runtime: &Runtime) -> Value { json_text(&runtime.getup_fixture_report().unwrap()) }
fn report(runtime: &Runtime) -> Value { json_text(&runtime.getup_report().unwrap()) }
fn native(runtime: &Runtime) -> u64 { fixture(runtime)["nativeSubsteps"].as_u64().unwrap() }
fn differences(a: &Value, b: &Value, path: &str, out: &mut Vec<String>) {
    match (a, b) {
        (Value::Object(x), Value::Object(y)) if x.keys().eq(y.keys()) => {
            for (key, value) in x { differences(value, &y[key], &format!("{path}/{}", key.replace('~', "~0").replace('/', "~1")), out); }
        }
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => {
            for (i, (x, y)) in x.iter().zip(y).enumerate() { differences(x, y, &format!("{path}/{i}"), out); }
        }
        _ if a != b => out.push(path.to_owned()),
        _ => (),
    }
}
fn array3(value: &Value) -> [f64; 3] {
    ["x", "y", "z"].map(|key| value[key].as_f64().unwrap())
}
fn rotate(q: &Value, v: [f64; 3]) -> [f64; 3] {
    let [x, y, z] = array3(q); let w = q["w"].as_f64().unwrap();
    let t = [2.0*(y*v[2]-z*v[1]), 2.0*(z*v[0]-x*v[2]), 2.0*(x*v[1]-y*v[0])];
    [v[0]+w*t[0]+y*t[2]-z*t[1], v[1]+w*t[1]+z*t[0]-x*t[2], v[2]+w*t[2]+x*t[1]-y*t[0]]
}
fn clearance(snapshot: &Value, model: &Value, id: &str) -> f64 {
    let body = snapshot["segments"].as_array().unwrap().iter().find(|s| s["id"] == id).unwrap();
    let part = model["segments"].as_array().unwrap().iter().find(|s| s["id"] == id).unwrap();
    let y = body["position"]["y"].as_f64().unwrap();
    part["reference"]["primitives"].as_array().unwrap().iter().flat_map(|p| {
        ["a", "b"].map(|end| y + rotate(&body["rotation"], array3(&p[end]))[1] - p["radius"].as_f64().unwrap())
    }).fold(f64::INFINITY, f64::min)
}

#[derive(Clone)]
struct Aggregate {
    samples: u64, last: u64, peak_head: Value, peak_up: Value, peak_stable: f64,
    support_counts: [u64; 6], min_clearance: [f64; 2], max_clearance: [f64; 2],
    unloaded: [u64; 2], loaded_slip: [f64; 2], first_20mm: [Option<f64>; 2],
    max_effort: f64, profile: Value,
}
impl Aggregate {
    fn new(profile: Value) -> Self {
        Self { samples:0, last:0, peak_head:Value::Null, peak_up:Value::Null, peak_stable:0.0,
            support_counts:[0;6], min_clearance:[f64::INFINITY;2], max_clearance:[f64::NEG_INFINITY;2],
            unloaded:[0;2], loaded_slip:[0.0;2], first_20mm:[None;2], max_effort:0.0, profile }
    }
    fn observe(&mut self, snapshot: &Value, diagnostic: &Value, gaps: [f64; 2]) {
        let n = diagnostic["nativeSubsteps"].as_u64().unwrap(); self.samples += 1; self.last = n;
        let marker = json!({"nativeSubstep":n,"physicalSeconds":n as f64*DT,
            "elapsedAfterTriggerSeconds":(n as i64-TRIGGER as i64) as f64*DT,
            "headM":diagnostic["headM"],"torsoUp":diagnostic["torsoUp"],"predicates":diagnostic["predicates"]});
        if self.peak_head.is_null() || diagnostic["headM"].as_f64().unwrap() > self.peak_head["headM"].as_f64().unwrap() { self.peak_head=marker.clone(); }
        if self.peak_up.is_null() || diagnostic["torsoUp"].as_f64().unwrap() > self.peak_up["torsoUp"].as_f64().unwrap() { self.peak_up=marker; }
        let keys=["bothFeetLoadedAndPersistent","footLoadFraction","onlyFeet","comInSupport","captureInSupport","torso"];
        for (i,key) in keys.into_iter().enumerate() { self.support_counts[i] += u64::from(diagnostic["predicates"][key] == true); }
        for i in 0..2 {
            self.min_clearance[i]=self.min_clearance[i].min(gaps[i]); self.max_clearance[i]=self.max_clearance[i].max(gaps[i]);
            let foot=&diagnostic["contactMotion"]["feet"][i];
            if foot["loaded"] == true {
                self.loaded_slip[i] += foot["impulseWeightedTangentSpeedMps"].as_f64().unwrap()*DT;
                if self.loaded_slip[i]>0.02 && self.first_20mm[i].is_none() { self.first_20mm[i]=Some(n as f64*DT); }
            } else { self.unloaded[i] += 1; }
        }
        self.peak_stable=self.peak_stable.max(snapshot["diagnostics"]["getup"]["policy"]["stable_standing_s"].as_f64().unwrap());
        self.max_effort=self.max_effort.max(snapshot["diagnostics"]["requested_motor_effort_nm"].as_f64().unwrap());
        assert_eq!(snapshot["diagnostics"]["direct_pelvis_force_n"],0.0);
        assert_eq!(snapshot["diagnostics"]["direct_pelvis_torque_nm"],0.0);
        assert!(self.max_effort<=920.001);
        assert_eq!(diagnostic["contactMotion"]["nativeBaseSolverIterations"],20);
        assert_eq!(diagnostic["contactMotion"]["humanMaximumAdditionalSolverIterations"],5);
    }
    fn value(&self) -> Value {
        json!({"everyNativeObserverSamples":self.samples,"lastObservedNative":self.last,
            "peakHead":self.peak_head,"peakTorsoUp":self.peak_up,"maximumContiguousStableSeconds":self.peak_stable,
            "supportPassedNativeSampleCounts":{"bothFeetLoadedAndPersistent":self.support_counts[0],"footLoadFraction":self.support_counts[1],"onlyFeet":self.support_counts[2],"comInSupport":self.support_counts[3],"captureInSupport":self.support_counts[4],"torso":self.support_counts[5]},
            "footOrder":["left_foot","right_foot"],"minimumGeometricFootClearanceM":self.min_clearance,
            "maximumGeometricFootClearanceM":self.max_clearance,"unloadedNativeSamples":self.unloaded,
            "loadedSlipM":self.loaded_slip,"first20mmCrossingPhysicalSeconds":self.first_20mm,
            "wholeGetup20mm":"Diagnostic only; never a stop or success gate. Existing stable-interval 10 mm criterion is untouched.",
            "clearanceMethod":"Minimum world Y of each foot sphere/capsule endpoint minus radius, computed read-only from frozen model and observed poses in f64.",
            "maximumRequestedMotorEffortNm":self.max_effort,"unchangedProfile":self.profile})
    }
}
fn observe(s: &lh_sim::Simulation, model: &Value, all: &mut Aggregate, recovery: &mut Aggregate, stream: &mut File) {
    // Normalize both checkpoint-string and live-to_value f32 representations
    // through the same frozen typed Profile before comparison.
    assert_eq!(typed_profile(serde_json::to_value(&s.profile).unwrap()),typed_profile(all.profile.clone()));
    let d=getup::controller::standing_diagnostic(s,None).unwrap();
    let snapshot=serde_json::to_value(s.snapshot()).unwrap();
    let gaps=["left_foot","right_foot"].map(|id|clearance(&snapshot,model,id));
    all.observe(&snapshot,&d,gaps);
    if d["nativeSubsteps"].as_u64().unwrap()>TRIGGER { recovery.observe(&snapshot,&d,gaps); }
    writeln!(stream,"{}",json!({"nativeSubstep":d["nativeSubsteps"],"headM":d["headM"],"torsoUp":d["torsoUp"],
        "predicates":d["predicates"],"footClearanceM":gaps,"feetLoadsN":[d["loadsN"][16],d["loadsN"][10]],
        "referenceIndex":snapshot["diagnostics"]["getup"]["policy"]["reference_index"],
        "policyDecisions":snapshot["diagnostics"]["getup"]["policy"]["policy_decisions"],
        "forecastSubsteps":snapshot["diagnostics"]["getup"]["policy"]["forecast_substeps"]})).unwrap();
}
fn tick_record(runtime: &Runtime, snapshot: &Value, trace: &mut File, diagnostic: &mut File) {
    let f=fixture(runtime); let trigger=if f["state"]["preparing"]==true {Value::Null} else {json!(TRIGGER)};
    writeln!(trace,"{}",json!({"snapshot":snapshot,"fixture":f,"triggerNative":trigger})).unwrap();
    writeln!(diagnostic,"{}",json!({"physicalSeconds":snapshot["simulation_time_s"],
        "standing":json_text(&runtime.getup_standing_diagnostic("null").unwrap())})).unwrap();
    trace.flush().unwrap(); diagnostic.flush().unwrap();
}
fn first_field_difference(a: &[Value], b: &[Value], field: &str) -> Value {
    for (i,(a,b)) in a.iter().zip(b).enumerate() {
        if a[field]!=b[field] { return json!({"decisionIndex":i,"nativeSubstep":a["native_substep"],"field":field}); }
    }
    Value::Null
}
fn main() {
    let began=Instant::now(); let args:Vec<_>=std::env::args().collect();
    assert_eq!(args.len(),2,"side-spline-ablation OUTPUT_DIRECTORY");
    let out=Path::new(&args[1]); fs::create_dir_all(out).unwrap();
    OpenOptions::new().write(true).create_new(true).open(out.join("started-once")).unwrap();
    let mut runtime=Runtime::new_getup_fixture("side").unwrap();
    let initial=runtime.getup_checkpoint().unwrap(); let initial_parts=decode(&initial);
    assert_eq!(initial_parts.0,1); let model=json_text(&initial_parts.1); let initial_aux=json_text(&initial_parts.3);
    let profile=initial_aux["simulation"]["profile"].clone();
    let mut all=Aggregate::new(profile.clone()); let mut recovery=Aggregate::new(profile.clone());
    let mut trace=File::create(out.join("common-trace.ndjson")).unwrap();
    let mut diagnostic=File::create(out.join("common-diagnostics-40hz.ndjson")).unwrap();
    let mut native_stream=File::create(out.join("common-native-observations.ndjson")).unwrap();
    let mut reply=runtime.advance(0).unwrap();
    tick_record(&runtime,&serde_json::to_value(&reply.snapshot).unwrap(),&mut trace,&mut diagnostic);
    for poll in 1..=19 {
        reply=runtime.advance_observed(1,|s,_|{observe(s,&model,&mut all,&mut recovery,&mut native_stream);Ok(())}).unwrap();
        assert!(reply.failure.is_none()); assert_eq!(native(&runtime),poll*20);
        let ready=fixture(&runtime)["state"]["ready"].as_bool().unwrap();
        assert_eq!(ready,poll==19,"Fresh side must first be ready at poll19/native380");
        tick_record(&runtime,&serde_json::to_value(&reply.snapshot).unwrap(),&mut trace,&mut diagnostic);
    }
    assert_eq!(native(&runtime),TRIGGER); save(&out.join("fixture-at-trigger.json"),&fixture(&runtime));
    let before=serde_json::to_value(&reply.snapshot.segments).unwrap(); runtime.trigger_getup_fixture().unwrap();
    reply=runtime.advance(0).unwrap(); assert_eq!(before,serde_json::to_value(&reply.snapshot.segments).unwrap());
    fs::write(out.join("checkpoint-trigger.bin"),runtime.getup_checkpoint().unwrap()).unwrap();
    reply=runtime.advance_observed(1,|s,_|{observe(s,&model,&mut all,&mut recovery,&mut native_stream);Ok(())}).unwrap();
    assert!(reply.failure.is_none()); assert_eq!(native(&runtime),400);
    tick_record(&runtime,&serde_json::to_value(&reply.snapshot).unwrap(),&mut trace,&mut diagnostic);
    trace.sync_all().unwrap(); diagnostic.sync_all().unwrap(); native_stream.sync_all().unwrap();
    let baseline=runtime.getup_checkpoint().unwrap(); let parts=decode(&baseline);
    assert_eq!(parts.0,initial_parts.0); assert_eq!(parts.1,initial_parts.1);
    let auxiliary=json_text(&parts.3); let mut changed=auxiliary.clone();
    let foot=model["segments"].as_array().unwrap().iter().find(|s|s["id"]=="left_foot").unwrap();
    assert_eq!(foot["reference"]["hinge"]["qposIndex"],14); assert_eq!(foot["reference"]["hinge"]["actuatorIndex"],13);
    let bound=foot["jointProfile"]["axes"][0]["minRadians"].as_f64().unwrap() as f32;
    assert_eq!(bound.to_bits(),(-0.8726646259971648_f32).to_bits());
    let old=changed.pointer(LEAF).unwrap().clone(); assert!(old.as_f64().unwrap()<(bound as f64)-0.23);
    *changed.pointer_mut(LEAF).unwrap()=serde_json::to_value(bound).unwrap();
    let mut leaves=Vec::new(); differences(&auxiliary,&changed,"",&mut leaves); assert_eq!(leaves,vec![LEAF.to_owned()]);
    let clamped=encode(&(parts.0,parts.1.clone(),parts.2.clone(),serde_json::to_string(&changed).unwrap()));
    let clamped_parts=decode(&clamped);
    assert_eq!(parts.0,clamped_parts.0); assert_eq!(parts.1,clamped_parts.1); assert_eq!(parts.2,clamped_parts.2);
    fs::write(out.join("checkpoint-baseline.bin"),&baseline).unwrap(); fs::write(out.join("checkpoint-clamped.bin"),&clamped).unwrap();
    let initial_report=report(&runtime);
    assert_eq!(initial_report["started_substep"],TRIGGER); assert_eq!(initial_report["status"]["forecast_substeps"],740);
    assert_eq!(initial_report["status"]["teacher_decisions"],37); assert_eq!(initial_report["status"]["policy_decisions"],1);
    save(&out.join("mutation-audit.json"),&json!({"checkpointTuple":"(u32,String,Vec<u8>,String)",
        "bincode":"1.3.3 DefaultOptions/fixed-int/64MiB/reject-trailing-bytes", "sameSchema":true,"sameModelStringBytes":true,
        "sameWorldBytes":true,"worldBytes":parts.2.len(),"exactlyOneAuxiliaryJsonLeafDiffers":leaves,
        "oldValue":old,"newValue":bound,"newValueF32Bits":bound.to_bits(),"nativeSubstepsAtMutation":400,
        "triggerNative":TRIGGER,"preparationPollingTicks":1,"profileUnchanged":true,
        "experimentScope":"One diagnostic reference-entry ablation; no source-bug or recovery-success claim."}));
    let mut reports=Vec::new();
    for (name,bytes) in [("baseline",&baseline),("clamped",&clamped)] {
        let arm=out.join(name); fs::create_dir(&arm).unwrap(); let mut arm_runtime=Runtime::from_getup_checkpoint(bytes).unwrap();
        save(&arm.join("checkpoint-restore-audit.json"),&json_text(&arm_runtime.verify_getup_checkpoint(bytes).unwrap()));
        let mut all_arm=all.clone(); let mut recovery_arm=recovery.clone();
        fs::copy(out.join("common-trace.ndjson"),arm.join("trace.ndjson")).unwrap();
        fs::copy(out.join("common-diagnostics-40hz.ndjson"),arm.join("diagnostics-40hz.ndjson")).unwrap();
        fs::copy(out.join("common-native-observations.ndjson"),arm.join("native-observations.ndjson")).unwrap();
        let mut trace=OpenOptions::new().append(true).open(arm.join("trace.ndjson")).unwrap();
        let mut diag=OpenOptions::new().append(true).open(arm.join("diagnostics-40hz.ndjson")).unwrap();
        let mut stream=OpenOptions::new().append(true).open(arm.join("native-observations.ndjson")).unwrap();
        let mut arm_reply=arm_runtime.advance(0).unwrap();
        while native(&arm_runtime)<END && arm_reply.failure.is_none() {
            arm_reply=arm_runtime.advance_observed(1,|s,_|{observe(s,&model,&mut all_arm,&mut recovery_arm,&mut stream);Ok(())}).unwrap();
            tick_record(&arm_runtime,&serde_json::to_value(&arm_reply.snapshot).unwrap(),&mut trace,&mut diag);
        }
        stream.sync_all().unwrap(); let final_native=native(&arm_runtime); let final_report=report(&arm_runtime);
        assert_eq!(final_report["status"]["forecast_substeps"],initial_report["status"]["forecast_substeps"]);
        assert_eq!(final_report["teacher_frames"],initial_report["teacher_frames"]);
        let final_bytes=arm_runtime.getup_checkpoint().unwrap(); fs::write(arm.join("checkpoint-final.bin"),&final_bytes).unwrap();
        save(&arm.join("reference-report.json"),&final_report);
        save(&arm.join("result.json"),&json!({"arm":name,"diagnosticOnly":true,"terminalReached":final_native==END,
            "triggerNative":TRIGGER,"endNative":END,"finalNative":final_native,"elapsedAfterTriggerSeconds":(final_native-TRIGGER) as f64*DT,
            "realIntegrationsInThisArmAfterCheckpoint":final_native-400,"commonPreparationRealIntegrations":TRIGGER,
            "commonRecoveryRealIntegrations":20,"forecastIntegrationsSharedOnce":740,"additionalForecastIntegrationsInThisArm":0,
            "wholeGetupAggregate":all_arm.value(),"recoveryAggregate":recovery_arm.value(),"unobservedRealTailNativeSteps":final_native-all_arm.last,
            "frozenActorCounts":final_report["status"],"newPolicyDecisionsInThisArm":final_report["status"]["policy_decisions"].as_u64().unwrap()-1,
            "finalStandingDiagnostic":json_text(&arm_runtime.getup_standing_diagnostic("null").unwrap()),"final":arm_reply,
            "acceptance":"Existing controller gates unchanged. Whole-getup20mm is diagnostic only. No qualification assertion."}));
        reports.push(final_report);
    }
    let a=reports[0]["student_decisions"].as_array().unwrap(); let b=reports[1]["student_decisions"].as_array().unwrap();
    assert_eq!(a.len(),b.len()); assert!(a.len()>73); assert_eq!(&a[..67],&b[..67]);
    for i in 0..=67 { assert_eq!(a[i]["before"],b[i]["before"]); }
    let mut first_input_leaves=Vec::new(); differences(&a[67]["observation"],&b[67]["observation"],"",&mut first_input_leaves);
    assert_eq!(first_input_leaves,vec!["/106".to_owned()]);
    save(&out.join("comparison.json"),&json!({"diagnosticOnly":true,"sameDecisionCounts":a.len(),
        "allDecisionsBefore67ExactlyEqual":true,"allBeforePosesThroughDecision67ExactlyEqual":true,
        "firstPossibleDecision":67,"firstPossibleNativeSubstep":TRIGGER+67*20,"firstPossibleElapsedSeconds":1.675,
        "firstInputChangedLeavesAt67":first_input_leaves,
        "firstObservationDifference":first_field_difference(a,b,"observation"),"firstResidualDifference":first_field_difference(a,b,"residual"),
        "firstControlDifference":first_field_difference(a,b,"controls"),"firstBeforePoseDifference":first_field_difference(a,b,"before"),
        "distinctActualRealIntegrationsAcrossBothArms":400+2*(END-400),"sharedActualForecastIntegrations":740,
        "maximumPlannedActualIntegrationsIncludingForecast":400+2*(END-400)+740,
        "wallSecondsIncludingPreparationForecastBothArmsObservationSerialization":began.elapsed().as_secs_f64(),
        "interpretation":"A controlled diagnostic of one spline entry. Residuals may offset raw targets; an out-of-range raw reference alone proves neither infeasible effective target nor a source bug."}));
    println!("{}",json!({"output":out,"armsFinished":true,"elapsedAfterTriggerSeconds":3.0,"diagnosticOnly":true}));
}
