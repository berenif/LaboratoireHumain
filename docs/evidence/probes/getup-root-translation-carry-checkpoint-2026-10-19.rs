//! Explicit, zero-integration activation and exact next-tick checkpoint test.
use bincode::Options;
use lh_sim::runtime::Runtime;
use serde_json::{Value, json};
use std::{
    fs::{self, OpenOptions},
    path::Path,
};
fn save(out: &Path, name: &str, value: &Value) {
    fs::write(out.join(name), serde_json::to_vec_pretty(value).unwrap()).unwrap();
}
fn parts(bytes: &[u8]) -> (u32, String, Vec<u8>, String) {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(64 * 1024 * 1024)
        .reject_trailing_bytes()
        .deserialize(bytes)
        .unwrap()
}
fn physical(mut v: Value) -> Value {
    fn prune(v: &mut Value) {
        match v {
            Value::Object(m) => {
                m.remove("controller_ms");
                m.remove("update_ms");
                for x in m.values_mut() {
                    prune(x);
                }
            }
            Value::Array(a) => {
                for x in a {
                    prune(x);
                }
            }
            _ => {}
        }
    }
    prune(&mut v);
    v
}
fn main() {
    let a: Vec<_> = std::env::args().collect();
    assert_eq!(a.len(), 3, "CHECKPOINT OUT");
    let out = Path::new(&a[2]);
    fs::create_dir_all(out).unwrap();
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("started-once"))
        .unwrap();
    let input = fs::read(&a[1]).unwrap();
    let mut live = Runtime::from_getup_checkpoint(&input).unwrap();
    let initial = live.advance(0).unwrap();
    let fi: Value = serde_json::from_str(&live.getup_fixture_report().unwrap()).unwrap();
    assert_eq!(fi["profile"]["solver_iterations"], 20);
    assert_eq!(fi["profile"]["additional_body_iterations"], 5);
    assert_eq!(fi["profile"]["tick_substeps"], 20);
    let native = fi["nativeSubsteps"].as_u64().unwrap();
    let initial_controller = live.getup_report().unwrap();
    let initial_policy: Value = serde_json::from_str(&initial_controller).unwrap();
    let initial_forecast = initial_policy["status"]["forecast_substeps"]
        .as_u64()
        .unwrap();
    let before = live.getup_checkpoint().unwrap();
    assert_eq!(parts(&before).0, 1);
    live.set_root_translation_compensation_enabled(true)
        .unwrap();
    let after = live.getup_checkpoint().unwrap();
    let pbefore = parts(&before);
    let pafter = parts(&after);
    assert_eq!(pafter.0, 2);
    assert_eq!(pbefore.1, pafter.1);
    assert_eq!(pbefore.2, pafter.2, "Activation changed serialized world");
    let wrapper: Value = serde_json::from_str(&pafter.3).unwrap();
    assert_eq!(
        wrapper["runtime"].as_str().unwrap(),
        pbefore.3,
        "Activation changed Runtime/controller state outside the integrator sidecar"
    );
    assert_eq!(initial_controller, live.getup_report().unwrap());
    let f: Value = serde_json::from_str(&live.getup_fixture_report().unwrap()).unwrap();
    assert_eq!(f["nativeSubsteps"], native);
    fs::write(out.join("active-checkpoint-before-tick.bin"), &after).unwrap();
    save(
        out,
        "active-verification.json",
        &serde_json::from_str(&live.verify_getup_checkpoint(&after).unwrap()).unwrap(),
    );
    save(out, "initial-fixture.json", &fi);
    save(out, "enabled-fixture.json", &f);
    save(
        out,
        "initial-snapshot.json",
        &serde_json::to_value(&initial.snapshot).unwrap(),
    );
    save(
        out,
        "initial-controller.json",
        &serde_json::from_str(&initial_controller).unwrap(),
    );
    let mut restored = Runtime::from_getup_checkpoint(&after).unwrap();
    assert_eq!(
        parts(&restored.getup_checkpoint().unwrap()),
        pafter,
        "Active zero-integration checkpoint re-export differs"
    );
    let original_reply = live.advance(1).unwrap();
    save(
        out,
        "active-live-tick-receipt.json",
        &json!({
        "reply":original_reply,"fixture":serde_json::from_str::<Value>(&live.getup_fixture_report().unwrap()).unwrap(),
        "controller":serde_json::from_str::<Value>(&live.getup_report().unwrap()).unwrap()}),
    );
    let original_cp = live.getup_checkpoint().unwrap();
    fs::write(out.join("active-live-tick-receipt.bin"), &original_cp).unwrap();
    let restored_reply = restored.advance(1).unwrap();
    save(
        out,
        "active-restored-tick-receipt.json",
        &json!({
        "reply":restored_reply,"fixture":serde_json::from_str::<Value>(&restored.getup_fixture_report().unwrap()).unwrap(),
        "controller":serde_json::from_str::<Value>(&restored.getup_report().unwrap()).unwrap()}),
    );
    let restored_cp = restored.getup_checkpoint().unwrap();
    fs::write(out.join("active-restored-tick-receipt.bin"), &restored_cp).unwrap();
    assert_eq!(
        physical(serde_json::to_value(&original_reply).unwrap()),
        physical(serde_json::to_value(&restored_reply).unwrap()),
        "Next-tick physical reply differs"
    );
    assert_eq!(
        live.getup_report().unwrap(),
        restored.getup_report().unwrap(),
        "Next-tick controller differs"
    );
    let op = parts(&original_cp);
    let rp = parts(&restored_cp);
    assert_eq!(op.0, rp.0);
    assert_eq!(op.1, rp.1);
    assert_eq!(op.2, rp.2, "Next-tick binary world differs");
    let mut original_wrapper: Value = serde_json::from_str(&op.3).unwrap();
    let mut restored_wrapper: Value = serde_json::from_str(&rp.3).unwrap();
    for wrapper in [&mut original_wrapper, &mut restored_wrapper] {
        let runtime_value: Value =
            serde_json::from_str(wrapper["runtime"].as_str().unwrap()).unwrap();
        wrapper["runtime"] = physical(runtime_value);
    }
    assert_eq!(
        original_wrapper, restored_wrapper,
        "Next-tick wrapper/runtime state differs"
    );
    assert!(original_reply.failure.is_none());
    assert_eq!(original_reply.completed_ticks, 1);
    fs::write(out.join("active-checkpoint-after-tick.bin"), &original_cp).unwrap();
    save(
        out,
        "active-final-reply.json",
        &serde_json::to_value(&original_reply).unwrap(),
    );
    save(
        out,
        "active-final-controller.json",
        &serde_json::from_str(&live.getup_report().unwrap()).unwrap(),
    );
    let mut disabled = Runtime::from_getup_checkpoint(&input).unwrap();
    disabled.advance(0).unwrap();
    let disabled_reply = disabled.advance(1).unwrap();
    save(
        out,
        "disabled-tick-receipt.json",
        &json!({
        "reply":disabled_reply,"fixture":serde_json::from_str::<Value>(&disabled.getup_fixture_report().unwrap()).unwrap(),
        "controller":serde_json::from_str::<Value>(&disabled.getup_report().unwrap()).unwrap()}),
    );
    assert!(disabled_reply.failure.is_none());
    assert_eq!(disabled_reply.completed_ticks, 1);
    save(
        out,
        "disabled-final-reply.json",
        &serde_json::to_value(&disabled_reply).unwrap(),
    );
    fs::write(
        out.join("disabled-checkpoint-after-tick.bin"),
        disabled.getup_checkpoint().unwrap(),
    )
    .unwrap();
    for branch in [&live, &restored, &disabled] {
        let fixture: Value = serde_json::from_str(&branch.getup_fixture_report().unwrap()).unwrap();
        assert_eq!(fixture["nativeSubsteps"].as_u64().unwrap(), native + 20);
        let policy: Value = serde_json::from_str(&branch.getup_report().unwrap()).unwrap();
        assert_eq!(
            policy["status"]["forecast_substeps"].as_u64().unwrap(),
            initial_forecast
        );
    }
    let final_native: Value = serde_json::from_str(&live.getup_fixture_report().unwrap()).unwrap();
    assert_eq!(
        final_native["nativeSubsteps"].as_u64().unwrap(),
        native + 20
    );
    save(
        out,
        "result.json",
        &json!({"inputCheckpoint":a[1],"zeroIntegrationActivationPassed":true,"legacySchema":1,"activeSchema":2,"activeCheckpointRoundTripPassed":true,"nextTickPhysicalRepliesExactExcludingOnlyWallCost":true,"nextTickBinaryWorldExact":true,"nextTickControllerExact":true,"ordinaryTicksPerBranch":1,"branches":3,"actualRealNativeIntegrations":60,"forecastNativeIntegrations":0,"activeInitialNative":native,"activeFinalNative":native+20,"noBodyPoseVelocityOrForceWrite":true,"thirtySecondQualification":false}),
    );
}
