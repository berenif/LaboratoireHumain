//! One fresh software acquisition and one left Step. No retries or world reset.
use lh_sim::{Profile, getup, integration_budget as budget, measurement::Failure, runtime::Runtime};
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::Write,path::Path};

fn save_checkpoint(runtime:&Runtime,path:&Path)->Result<Value,String> {
    let bytes=runtime.getup_checkpoint()?;
    // Two 7 MiB checkpoints + 1 MiB result stay below 16 MiB total.
    if bytes.len()>7*1024*1024 {return Err("checkpoint exceeds 7 MiB reserve".into());}
    let mut file=OpenOptions::new().write(true).create_new(true).open(path).map_err(|e|format!("checkpoint {}: {e}",path.display()))?;
    file.write_all(&bytes).and_then(|_|file.sync_all()).map_err(|e|format!("checkpoint write {}: {e}",path.display()))?;
    let verification=runtime.verify_getup_checkpoint(&bytes);
    Ok(json!({"path":path,"bytes":bytes.len(),"restored":verification.is_ok(),
        "verification":verification.as_ref().ok().and_then(|s|serde_json::from_str::<Value>(s).ok()),
        "restoreError":verification.as_ref().err(),"scope":"exact serialized state; next-step continuity not tested"}))
}
fn trial(result_path:&Path)->Result<Value,String> {
    let mut profile:Profile=serde_json::from_str(include_str!("../../../../docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json")).map_err(|e|e.to_string())?;
    profile.getup_native_locomotion=Some(getup::locomotion::Profile::NativeJointOwnerV1);
    profile.validate()?;
    let mut runtime=Runtime::new_getup_fixture_with_profile("back",profile)?;
    runtime.set_root_translation_compensation_enabled(true)?;
    let mut triggered=false;
    let mut stepping=false;
    let mut first_stop=None;
    let mut entry_checkpoint=None;
    let run=(||->Result<(),String> {
    loop {
        if let Err(reason)=budget::available() {first_stop=Some(reason);break;}
        let obs:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
        if !triggered {
            let fixture:Value=serde_json::from_str(&runtime.getup_fixture_report()?).map_err(|e|e.to_string())?;
            if fixture["state"]["ready"]==true {runtime.trigger_getup_fixture()?;triggered=true;}
            else if budget::snapshot()?.real>=1600 {first_stop=Some("fixture-not-admitted".into());break;}
        } else if !stepping && obs["earnedBalanceReady"]==true {
            entry_checkpoint=Some(save_checkpoint(&runtime,&result_path.with_extension("entry.bin"))?);
            if entry_checkpoint.as_ref().unwrap()["restored"]!=true {return Err("entry checkpoint restore verification failed; Step not admitted".into());}
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
    Ok(())})();
    if let Err(reason)=run {if first_stop.is_none() {first_stop=Some(reason);}}
    let stopped=save_checkpoint(&runtime,&result_path.with_extension("stop.bin"));
    Ok(json!({"entryCheckpoint":entry_checkpoint,"stopCheckpoint":stopped.as_ref().ok(),"stopCheckpointError":stopped.as_ref().err(),
        "stepCommands":budget::step_command_diagnostics(),"firstStop":first_stop,"fixtureTriggered":triggered,"stepStarted":stepping,
        "finalObservation":runtime.getup_native_observation().ok().and_then(|s|serde_json::from_str::<Value>(&s).ok())}))
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if args.len()!=3 || args[1]!="--run-numeric-trial" {return Err("usage: getup_native_step_bounded --run-numeric-trial NEW_RESULT.json".into());}
    // Reserve the small result path before doing work. Never overwrite evidence.
    let mut file=OpenOptions::new().write(true).create_new(true).open(&args[2])?;
    file.write_all(b"{\"status\":\"started; final outcome not yet recorded\"}\n")?;file.sync_all()?;
    budget::install()?;
    let outcome=trial(Path::new(&args[2]));
    let ledger=budget::snapshot()?;
    let result=json!({"format":"bounded-native-step-trial-v1","historical":18337,
        "ledger":ledger,"historicalPlusReservedIntegrations":18337+ledger.real+ledger.forecast,
        "prospectiveCumulativeLimit":21401,"authorizedTrialReserve":3064,
        "remainingAuthorizedBudget":3064-ledger.real-ledger.forecast,
        "result":outcome.as_ref().ok(),"error":outcome.as_ref().err(),
        "motionQualified":false});
    let bytes=serde_json::to_vec_pretty(&result)?;
    if bytes.len()>1024*1024 {return Err("result exceeds 1 MiB cap".into());}
    use std::io::{Seek,SeekFrom};file.seek(SeekFrom::Start(0))?;file.write_all(&bytes)?;file.set_len(bytes.len() as u64)?;file.sync_all()?;
    println!("{}",serde_json::to_string(&result)?);Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn native_checkpoint_pure_save_restore_and_no_overwrite() {
        let mut profile:Profile=serde_json::from_str(include_str!("../../../../docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json")).unwrap();
        profile.getup_native_locomotion=Some(getup::locomotion::Profile::NativeJointOwnerV1);
        let mut runtime=Runtime::new_getup_fixture_with_profile("back",profile).unwrap();
        runtime.set_root_translation_compensation_enabled(true).unwrap();
        let path=std::env::temp_dir().join(format!("lh-checkpoint-pure-{}-{}.bin",std::process::id(),
            std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
        let receipt=save_checkpoint(&runtime,&path).unwrap();
        assert_eq!(receipt["restored"],true);assert_eq!(receipt["verification"]["nativeSubsteps"],0);
        let bytes=std::fs::read(&path).unwrap();
        assert!(save_checkpoint(&runtime,&path).is_err());assert_eq!(std::fs::read(&path).unwrap(),bytes);
        assert!(runtime.verify_getup_checkpoint(&bytes).is_ok());
        std::fs::remove_file(&path).unwrap(); // only this test's new temporary file
    }
}
