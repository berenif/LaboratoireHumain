//! One short resume of the verified native5339 software refusal. Never acquire/reset.
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
    if restored["passed"]!=true || before["nativeCompleted"]!=5339 || before["failure"]["offending"]!="Step reference exceeds pilot speed/acceleration envelope" {
        return Err("Resume requires verified saved native5339 refusal".into());
    }
    // This performs the complete reference/posture, passive/dynamic RNEA,
    // cone allocation and unchanged motor-cap admission, without integration.
    let admission=runtime.resume_getup_native_step_5339();
    let after_admission:Value=serde_json::from_str(&runtime.getup_native_observation()?).map_err(|e|e.to_string())?;
    let started=before["ownership"]["step"]["started_native"].as_u64().ok_or("Missing original Step clock")?;
    let native=before["nativeCompleted"].as_u64().ok_or("Missing native clock")?;
    let elapsed=native.checked_sub(started).ok_or("Invalid original Step clock")?;
    let pilot_remaining=2400u64.checked_sub(elapsed).ok_or("Original pilot expired")?;
    let duration=after_admission["ownership"]["step"]["reference"]["duration"].as_f64().ok_or("Missing reference duration")?;
    let reference_elapsed=after_admission["ownership"]["step"]["reference"]["elapsed"].as_f64().ok_or("Missing reference elapsed")?;
    let remaining_seconds=pilot_remaining.min(2200) as f64*0.00125;
    if duration-reference_elapsed>remaining_seconds {return Err(format!("Reference needs {} s, original pilot/reserve has {remaining_seconds} s; no compression/reset/extension",duration-reference_elapsed));}
    let preflight=json!({"restoration":restored,"nativeBefore":before["nativeCompleted"],
        "originalStartedNative":started,"originalPilotRemainingNative":pilot_remaining,"referenceRemainingSeconds":duration-reference_elapsed,"availableSeconds":remaining_seconds,
        "commandAdmitted":admission.is_ok(),"acknowledgement":admission.as_ref().ok().and_then(|s|serde_json::from_str::<Value>(s).ok()),"error":admission.as_ref().err(),"commands":budget::step_command_diagnostics(),
        "ledger":budget::snapshot()?});
    if preflight_only || admission.is_err() {
        return Ok(json!({"preflight":preflight,"integrated":false,"firstStop":admission.err()}));
    }
    // All later integration, including forecasts, uses the same global reserve.
    budget::begin_step()?;
    let mut first_stop=None;let mut separated_seen=false;let mut landed_seen=false;
    let mut samples=0u64;let mut extrema=json!({});let mut brake_exit=None;
    let foot_index=before["bodies"].as_array().ok_or("missing bodies")?.iter().position(|b|b["id"]=="left_foot").ok_or("missing left foot")?;
    let initial_foot:[f64;3]=serde_json::from_value(before["bodies"][foot_index]["position"].clone()).map_err(|e|e.to_string())?;
    let forward:[f64;3]=serde_json::from_value(before["ownership"]["step"]["forward"].clone()).map_err(|e|e.to_string())?;
    let mut max_retreat=0f64;let mut max_lift=0f64;let mut max_clearance=f64::NEG_INFINITY;let mut min_free_load=f64::INFINITY;
    let initial_com:[f64;3]=serde_json::from_value(before["com"].clone()).map_err(|e|e.to_string())?;
    let mut trace=vec![];let mut separation=None;let mut landing=None;
    let outcome=(||->Result<(),String> {
        loop {
            if let Err(e)=budget::available(){first_stop=Some(e);break;}
            let reply=runtime.advance_observed(1,|s,substep| {
                let o=getup::locomotion::observation(s).map_err(|e|Failure::new(s.tick,substep,"continuation-observation",e,0.,0.))?;
                samples+=1;
                let phase=o["ownership"]["step"]["phase"].as_str().unwrap_or("missing");
                separated_seen|=o["ownership"]["step"]["separated"]==true;
                landed_seen|=o["ownership"]["step"]["landed_samples"].as_u64().unwrap_or(0)>0;
                let foot:[f64;3]=serde_json::from_value(o["bodies"][foot_index]["position"].clone()).map_err(|e|Failure::new(s.tick,substep,"foot-observation",e.to_string(),0.,0.))?;
                let retreat=-(0..3).map(|i|(foot[i]-initial_foot[i])*forward[i]).sum::<f64>();
                max_retreat=max_retreat.max(retreat);max_lift=max_lift.max(foot[1]-initial_foot[1]);
                if brake_exit.is_none() && o["ownership"]["step"]["reference"]["braking"]==false {
                    brake_exit=Some(json!({"native":o["nativeCompleted"],"retreatM":retreat,"liftM":foot[1]-initial_foot[1],"feet":o["floorFootGeometry"]}));
                }
                let clearance=o["floorFootGeometry"]["feet"][0]["clearance"].as_f64().ok_or_else(||Failure::new(s.tick,substep,"sole-observation","missing capsule clearance".into(),0.,0.))?;
                let free_load=o["floorFootGeometry"]["feet"][0]["load"].as_f64().ok_or_else(||Failure::new(s.tick,substep,"load-observation","missing load".into(),0.,0.))?;
                max_clearance=max_clearance.max(clearance);min_free_load=min_free_load.min(free_load);
                let measured=json!({"native":o["nativeCompleted"],"com":o["com"],"freeLoadN":free_load,"retainedLoadN":o["floorFootGeometry"]["feet"][1]["load"],"wholeSoleClearanceM":clearance,"retreatM":retreat,"phase":phase});
                if samples==1 || samples%40==0 {trace.push(measured.clone());}
                if separated_seen && separation.is_none() {separation=Some(measured.clone());}
                if landed_seen && landing.is_none() {landing=Some(measured);}
                extrema=json!({"retreatM":retreat,"liftM":foot[1]-initial_foot[1],"braking":o["ownership"]["step"]["reference"]["braking"],"native":o["nativeCompleted"],"phase":phase,"feet":o["floorFootGeometry"],"separated":o["ownership"]["step"]["separated"],"landedSamples":o["ownership"]["step"]["landed_samples"]});
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
    let stop=checkpoint::save_compressed_checkpoint(&runtime,&out.with_extension("stop.bin.gz"));
    Ok(json!({"preflight":preflight,"integrated":budget::snapshot()?.real>0,"firstStop":first_stop,
        "initialCom":initial_com,"measuredTrace":trace,"firstSeparation":separation,"firstLanding":landing,"maximumWholeSoleClearanceM":max_clearance,"minimumFreeLoadN":min_free_load,
        "observedSamples":samples,"brakeExit":brake_exit,"maximumRetreatM":max_retreat,"maximumLiftM":max_lift,"separatedSeen":separated_seen,"landingSeen":landed_seen,"lastMeasuredSupport":extrema,
        "stopCheckpoint":stop.as_ref().ok(),"stopCheckpointError":stop.as_ref().err(),
        "commands":budget::step_command_diagnostics(),
        "finalObservation":runtime.getup_native_observation().ok().and_then(|s|serde_json::from_str::<Value>(&s).ok())}))
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if args.len()!=4 || !matches!(args[1].as_str(),"--preflight-stop"|"--resume-stop") {
        return Err("usage: getup_native_step_continue --preflight-stop|--resume-stop ENTRY.bin NEW_RESULT.json".into());
    }
    let mut file=OpenOptions::new().write(true).create_new(true).open(&args[3])?;
    file.write_all(b"{\"status\":\"started; final outcome pending\"}\n")?;file.sync_all()?;
    budget::install_coordinated_resume()?;
    let result=run(Path::new(&args[2]),Path::new(&args[3]),args[1]=="--preflight-stop");
    let ledger=budget::snapshot()?;
    let receipt=json!({"format":"native-step-stop-resume-v1","historical":19137,"trialLimit":2200,
        "cumulativeLimit":21401,"totalConsumed":19137+ledger.real+ledger.forecast,
        "remainingGlobalBudget":2264-ledger.real-ledger.forecast,"ledger":ledger,
        "inputCheckpoint":args[2],"result":result.as_ref().ok(),"error":result.as_ref().err(),"motionQualified":false});
    let bytes=serde_json::to_vec(&receipt)?;
    if bytes.len()>200_000 {return Err("result exceeds reserved 200000 bytes".into());}
    file.seek(SeekFrom::Start(0))?;file.write_all(&bytes)?;file.set_len(bytes.len() as u64)?;file.sync_all()?;
    println!("{}",json!({"ledger":ledger,"totalConsumed":receipt["totalConsumed"],"error":receipt["error"],"firstStop":receipt["result"]["firstStop"],"output":args[3]}));Ok(())
}
