//! Explicit bounded execution of the existing chain, not a recovery qualifier.
use super::*;

pub(super) const HORIZON_S: f32 = 25.0;
const WALL_BUDGET_S: f64 = 150.0;
thread_local! {static RUN:RefCell<Option<Instant>>=const{RefCell::new(None)};}
pub(super) fn enabled() -> bool {
    RUN.with(|r| r.borrow().is_some())
}
pub(super) fn contact_tracking_enabled() -> bool {
    enabled() && std::env::var("LH_CHAIN_CONTACT_TRACKING").is_ok_and(|x| x == "1")
}
pub(super) fn wall_guard(s: &Simulation, substep: u32) -> Result<(), measurement::Failure> {
    RUN.with(|r| {
        if let Some(started) = r.borrow().as_ref()
            && started.elapsed().as_secs_f64() >= WALL_BUDGET_S
        {
            return Err(measurement::Failure::new(
                s.tick,
                substep,
                "prone-chain-wall-budget",
                "declared same-world chain budget".into(),
                started.elapsed().as_secs_f64() as f32,
                WALL_BUDGET_S as f32,
            ));
        }
        Ok(())
    })
}
#[test]
#[ignore = "explicit full C-arm-support-trunk chain; repeat a real success from fresh fall"]
fn execute_prone_chain_one() {
    let out = PathBuf::from(std::env::var("LH_CHAIN_OUT").expect("out required"));
    assert!(!out.exists(), "Refuse overwriting chain evidence");
    std::fs::create_dir_all(&out).unwrap();
    let plan = Plan::load(&std::env::var("LH_DESCENT_PLAN").expect("C plan required")).unwrap();
    assert!(plan.stop_at_base && plan.phase_gate.is_some());
    let fixture = serde_json::from_str::<FixtureSet>(FIXTURES)
        .unwrap()
        .cases
        .into_iter()
        .find(|x| x.id == "landed-crouch-left")
        .unwrap();
    let started = Instant::now();
    RUN.with(|r| *r.borrow_mut() = Some(started));
    let case = run_with_policy(
        &fixture,
        &crate::recovery_contract::pinned_profile(),
        &plan,
        ArmMode::Prepared,
        fixture.heading,
        started,
        ObservationPolicy::ProneRecovery,
    )
    .unwrap();
    std::fs::write(out.join("case.json"), serde_json::to_vec(&case).unwrap()).unwrap();
    std::fs::write(out.join("summary.json"),serde_json::to_vec_pretty(&json!({
        "scope":"Complete physical C plus original arm/torso references, bounded joint gravity feedforward; no standing qualification",
        "caseCount":1,"physicalSeconds":case["physicalSeconds"],
        "physicalSteps":case["physicalSteps"],"elapsedNativeWallS":started.elapsed().as_secs_f64(),
        "horizonS":HORIZON_S,"wallBudgetS":WALL_BUDGET_S,"nativeSkinAwareLandTracking":contact_tracking_enabled(),
        "failure":case["failure"],"braceObserved":case["proneMilestonePassed"],
        "finalObservation":case["proneFinalObservation"],"freshRepeatRequired":true,
        "recoveryQualified":false,"productionRecoveryEnabled":false
    })).unwrap()).unwrap();
    RUN.with(|r| *r.borrow_mut() = None);
}
