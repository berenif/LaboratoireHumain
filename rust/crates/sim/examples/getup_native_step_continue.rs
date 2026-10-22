//! Continue only the verified earned native4540 checkpoint. Never acquire/reset.
use lh_sim::{getup,integration_budget as budget,measurement::Failure,runtime::Runtime};
use serde_json::{Value,json};
use std::{fs::OpenOptions,io::{Write,Seek,SeekFrom},path::Path};
#[path="support/checkpoint.rs"]
mod checkpoint;

fn run(input:&Path,out:&Path,preflight_only:bool)->Result<Value,String> {
    let bytes=std::fs::read(input).map_err(|e|e.to_string())?;
    let mut runtime=Runtime::from_getup_checkpoint(&bytes)?;
    let restored:Value=serde_json::from_str(&runtime.verify_getup_checkpoint(&bytes)?).map_err(|e|e.to_string())?;
    let before:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
    if restored["passed"]!=true || before["nativeCompleted"]!=4540 || before["earnedBalanceReady"]!=true {
        return Err("Continuation requires verified earned native4540 entry".into());
    }
    // This performs the complete reference/posture, passive/dynamic RNEA,
    // cone allocation and unchanged motor-cap admission, without integration.
    let admission=runtime.begin_getup_native_step(0);
    let preflight=json!({"restoration":restored,"nativeBefore":before["nativeCompleted"],
        "commandAdmitted":admission.is_ok(),"error":admission.as_ref().err(),"commands":budget::step_command_diagnostics(),
        "ledger":budget::snapshot()?});
    if preflight_only || admission.is_err() {
        return Ok(json!({"preflight":preflight,"integrated":false,"firstStop":admission.err()}));
    }
    // All later integration, including forecasts, uses the same global reserve.
    budget::begin_step()?;
    let mut first_stop=None;let mut separated_seen=false;let mut landed_seen=false;
    let mut samples=0u64;let mut extrema=json!({});
    let outcome=(||->Result<(),String> {
        loop {
            if let Err(e)=budget::available(){first_stop=Some(e);break;}
            let reply=runtime.advance_observed(1,|s,substep| {
                let o=getup::locomotion::observation(s).map_err(|e|Failure::new(s.tick,substep,"continuation-observation",e,0.,0.))?;
                samples+=1;
                let phase=o["ownership"]["step"]["phase"].as_str().unwrap_or("missing");
                separated_seen|=o["ownership"]["step"]["separated"]==true;
                landed_seen|=o["ownership"]["step"]["landed_samples"].as_u64().unwrap_or(0)>0;
                extrema=json!({"native":o["nativeCompleted"],"phase":phase,"feet":o["floorFootGeometry"],"separated":o["ownership"]["step"]["separated"],"landedSamples":o["ownership"]["step"]["landed_samples"]});
                let reason=if phase=="Complete" {Some("step-complete")} else if phase=="Cancelled" {Some("step-cancelled")}
                    else if o["ownership"]["last_application"]["owner"]!="step" {Some("step-ownership-lost")}
                    else if matches!(s.motion,lh_contracts::Motion::Falling|lh_contracts::Motion::Fallen|lh_contracts::Motion::Recovering|lh_contracts::Motion::Halted) {Some("step-fall-or-halt")} else {None};
                if let Some(reason)=reason {first_stop=Some(reason.into());return Err(Failure::new(s.tick,substep,"continuation-stop",reason.into(),0.,0.));}
                Ok(())
            }).map_err(|e|format!("runtime rejected: {e:?}"))?;
            if let Some(f)=reply.failure {if first_stop.is_none(){first_stop=Some(format!("{}: {}",f.check,f.offending));}break;}
            if reply.completed_ticks==0 {first_stop=Some("runtime-no-progress".into());break;}
        }
        Ok(())
    })();
    if let Err(e)=outcome {if first_stop.is_none(){first_stop=Some(e);}}
    let stop=checkpoint::save_checkpoint(&runtime,&out.with_extension("stop.bin"));
    Ok(json!({"preflight":preflight,"integrated":budget::snapshot()?.real>0,"firstStop":first_stop,
        "observedSamples":samples,"separatedSeen":separated_seen,"landingSeen":landed_seen,"lastMeasuredSupport":extrema,
        "stopCheckpoint":stop.as_ref().ok(),"stopCheckpointError":stop.as_ref().err(),
        "commands":budget::step_command_diagnostics(),
        "finalObservation":runtime.getup_native_observation().ok().and_then(|s|serde_json::from_str::<Value>(&s).ok())}))
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if args.len()!=4 || !matches!(args[1].as_str(),"--preflight-checkpoint"|"--continue-from-checkpoint") {
        return Err("usage: getup_native_step_continue --preflight-checkpoint|--continue-from-checkpoint ENTRY.bin NEW_RESULT.json".into());
    }
    let mut file=OpenOptions::new().write(true).create_new(true).open(&args[3])?;
    file.write_all(b"{\"status\":\"started; final outcome pending\"}\n")?;file.sync_all()?;
    budget::install_continuation()?;
    let result=run(Path::new(&args[2]),Path::new(&args[3]),args[1]=="--preflight-checkpoint");
    let ledger=budget::snapshot()?;
    let receipt=json!({"format":"native-step-checkpoint-continuation-v1","historical":18337,"trialLimit":2400,
        "cumulativeLimit":21401,"totalConsumed":18337+ledger.real+ledger.forecast,
        "remainingGlobalBudget":3064-ledger.real-ledger.forecast,"ledger":ledger,
        "inputCheckpoint":args[2],"result":result.as_ref().ok(),"error":result.as_ref().err(),"motionQualified":false});
    let bytes=serde_json::to_vec_pretty(&receipt)?;
    if bytes.len()>1024*1024 {return Err("result exceeds 1 MiB".into());}
    file.seek(SeekFrom::Start(0))?;file.write_all(&bytes)?;file.set_len(bytes.len() as u64)?;file.sync_all()?;
    println!("{}",serde_json::to_string(&receipt)?);Ok(())
}
