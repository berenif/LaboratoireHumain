//! One fresh software acquisition and one left Step. No retries or world reset.
use lh_sim::{Profile, getup, integration_budget as budget, measurement::Failure, runtime::Runtime};
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::Write};

fn trial()->Result<Value,String> {
    let mut profile:Profile=serde_json::from_str(include_str!("../../../../docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json")).map_err(|e|e.to_string())?;
    profile.getup_native_locomotion=Some(getup::locomotion::Profile::NativeJointOwnerV1);
    profile.validate()?;
    let mut runtime=Runtime::new_getup_fixture_with_profile("back",profile)?;
    runtime.set_root_translation_compensation_enabled(true)?;
    let mut triggered=false;
    let mut stepping=false;
    let mut first_stop=None;
    loop {
        if let Err(reason)=budget::available() {first_stop=Some(reason);break;}
        let obs:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
        if !triggered {
            let fixture:Value=serde_json::from_str(&runtime.getup_fixture_report()?).map_err(|e|e.to_string())?;
            if fixture["state"]["ready"]==true {runtime.trigger_getup_fixture()?;triggered=true;}
            else if budget::snapshot()?.real>=1600 {first_stop=Some("fixture-not-admitted".into());break;}
        } else if !stepping && obs["earnedBalanceReady"]==true {
            // Admission performs algebra only. A refusal is the end of this trial.
            runtime.begin_getup_native_step(0)?;
            budget::begin_step()?;
            stepping=true;
        }
        let reply=runtime.advance_observed(1,|s,substep| {
            if stepping {
                let obs=getup::locomotion::observation(s).map_err(|e|Failure::new(s.tick,substep,"trial-observation",e,0.,0.))?;
                let phase=obs["ownership"]["step"]["phase"].as_str().unwrap_or("missing");
                let reason=if phase=="Complete" {Some("step-complete")}
                    else if phase=="Cancelled" {Some("step-cancelled")}
                    else if obs["ownership"]["last_application"]["owner"]!="step" {Some("step-ownership-lost")}
                    else if matches!(s.motion,lh_contracts::Motion::Falling|lh_contracts::Motion::Fallen|lh_contracts::Motion::Recovering|lh_contracts::Motion::Halted) {Some("step-fall-or-halt")} else {None};
                if let Some(reason)=reason {first_stop=Some(reason.into());return Err(Failure::new(s.tick,substep,"bounded-trial-stop",reason.into(),0.,0.));}
            }
            Ok(())
        }).map_err(|e|format!("runtime rejected: {e:?}"))?;
        if let Some(failure)=reply.failure {if first_stop.is_none(){first_stop=Some(format!("{}: {}",failure.check,failure.offending));}break;}
        // Zero progress without an explicit failure must not busy-loop forever.
        if reply.completed_ticks==0 {first_stop=Some("runtime-no-progress".into());break;}
    }
    Ok(json!({"firstStop":first_stop,"fixtureTriggered":triggered,"stepStarted":stepping,
        "finalObservation":runtime.getup_native_observation().ok().and_then(|s|serde_json::from_str::<Value>(&s).ok())}))
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if args.len()!=3 || args[1]!="--run-numeric-trial" {return Err("usage: getup_native_step_bounded --run-numeric-trial NEW_RESULT.json".into());}
    // Reserve the small result path before doing work. Never overwrite evidence.
    let mut file=OpenOptions::new().write(true).create_new(true).open(&args[2])?;
    file.write_all(b"{\"status\":\"started; final outcome not yet recorded\"}\n")?;file.sync_all()?;
    budget::install()?;
    let outcome=trial();
    let ledger=budget::snapshot()?;
    let result=json!({"format":"bounded-native-step-trial-v1","historical":7080,
        "ledger":ledger,"historicalPlusReservedIntegrations":7080+ledger.real+ledger.forecast,
        "remainingHistoricalBudget":8920-ledger.real-ledger.forecast,
        "result":outcome.as_ref().ok(),"error":outcome.as_ref().err(),
        "motionQualified":false});
    let bytes=serde_json::to_vec_pretty(&result)?;
    if bytes.len()>1024*1024 {return Err("result exceeds 1 MiB cap".into());}
    use std::io::{Seek,SeekFrom};file.seek(SeekFrom::Start(0))?;file.write_all(&bytes)?;file.set_len(bytes.len() as u64)?;file.sync_all()?;
    println!("{}",serde_json::to_string(&result)?);Ok(())
}
