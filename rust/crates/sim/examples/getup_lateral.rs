//! One budgeted lateral fixture trial. No parameter search or recovery assistance.
use lh_sim::runtime::Runtime;
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
fn checkpoint(runtime: &Runtime, out: &Path, name: &str) {
    let bytes = runtime.getup_checkpoint().unwrap();
    fs::write(out.join(format!("checkpoint-{name}.bin")), &bytes).unwrap();
    let verification: Value =
        serde_json::from_str(&runtime.verify_getup_checkpoint(&bytes).unwrap()).unwrap();
    save(out, &format!("checkpoint-{name}.json"), &verification);
    fs::write(
        out.join("reference-report.json"),
        runtime.getup_report().unwrap(),
    )
    .unwrap();
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(
        args.len(),
        3,
        "getup_lateral OUTPUT REMAINING_PHYSICS_SECONDS"
    );
    let out = Path::new(&args[1]);
    fs::create_dir_all(out).unwrap();
    let budget: f64 = args[2].parse().unwrap();
    assert!(budget > 0.0 && budget <= 37.15260638300242);
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("started-once"))
        .unwrap();
    let began = Instant::now();
    let mut runtime = Runtime::new_getup_fixture("side-supported").unwrap();
    let mut physics_s = began.elapsed().as_secs_f64();
    let mut reply = runtime.advance(0).unwrap();
    let mut trace = OpenOptions::new()
        .append(true)
        .create_new(true)
        .open(out.join("trace.ndjson"))
        .unwrap();
    let mut trigger = None;
    let mut next_checkpoint = 1600;
    let outcome;
    checkpoint(&runtime, out, "initial");
    loop {
        let fixture: Value =
            serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
        let row = serde_json::to_value(&reply.snapshot).unwrap();
        let real = fixture["nativeSubsteps"].as_u64().unwrap();
        writeln!(
            trace,
            "{}",
            json!({"snapshot":row,"fixture":fixture,"triggerNative":trigger})
        )
        .unwrap();
        trace.flush().unwrap();
        if reply.failure.is_some() {
            outcome = "physical-or-reference-failure";
            break;
        }
        let policy = &row["diagnostics"]["getup"]["policy"];
        if policy["prototype_standing_passed"] == true {
            outcome = "one-second-standing-passed";
            break;
        }
        if physics_s >= budget - 1.0 {
            outcome = "incomplete-physical-budget";
            break;
        }
        if trigger.is_none() {
            if fixture["state"]["ready"] == true {
                let before = row["segments"].clone();
                runtime.trigger_getup_fixture().unwrap();
                reply = runtime.advance(0).unwrap();
                assert_eq!(
                    before,
                    serde_json::to_value(&reply.snapshot).unwrap()["segments"]
                );
                trigger = Some(real);
                save(out, "fixture-at-trigger.json", &fixture);
                checkpoint(&runtime, out, "trigger");
            } else if real >= 1600 {
                outcome = "invalid-lateral-fixture";
                break;
            }
        } else if real - trigger.unwrap() >= 20000 {
            outcome = "recovery-deadline";
            break;
        }
        if real >= next_checkpoint {
            checkpoint(&runtime, out, &real.to_string());
            next_checkpoint += 1600;
        }
        // One transport tick bounds admission latency and regular checkpoint tails.
        let began = Instant::now();
        reply = runtime.advance(1).unwrap();
        physics_s += began.elapsed().as_secs_f64();
    }
    checkpoint(&runtime, out, "final");
    let fixture: Value = serde_json::from_str(&runtime.getup_fixture_report().unwrap()).unwrap();
    let result = json!({"outcome":outcome,"physicsWallSeconds":physics_s,
        "authorizedBudgetSeconds":budget,"remainingBudgetSeconds":budget-physics_s,
        "triggerNative":trigger,"finalFixture":fixture,"final":reply,
        "sameBodyAndWeights":true,"externalSupportsDuringRecovery":false,
        "qualification24Cases":false});
    save(out, "result.json", &result);
    println!("{result}");
}
